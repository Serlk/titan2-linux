// SPDX-License-Identifier: GPL-2.0
//
// fs199x.c -- Driver for the FourSemi FS199x Audio Amplifier
//
// Mono smart amplifier with a DSP, configured by FourSemi firmware (the
// fs-amp-lib format). The structure follows fs210x.c. The register
// protocol, the start-up order and the decoding of the speaker protection
// parameters stored in the firmware model are derived from the GPL module
// snd-soc-fs19xx.ko (FourSemi) shipped with Titan 2 V01.00.14.
//
// Copyright (C) 2016-2025 Shanghai FourSemi Semiconductor Co.,Ltd.
// Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "fs199x.h"
#include "fs-amp-lib.h"

#define FS199X_DEFAULT_FWM_NAME		"fs199x_fwm.bin"
#define FS199X_DAI_NAME			"fs199x-aif"
#define FS199X_DEVICE_ID		0x37
#define FS199X_DEVICE_ID_REV		0x05
#define FS199X_INIT_SCENE		0
#define FS199X_DEFAULT_SCENE		1
#define FS199X_START_DELAY_MS		5
#define FS199X_FAULT_CHECK_INTERVAL_MS	2000
/*
 * The firmware's DSP effect sets the loudness; the volume register can be
 * left at its maximum. Start a little lower: the dB scale of the register
 * is not documented.
 */
#define FS199X_DEFAULT_VOLUME		(FS199X_16H_VOL_MAX - 64)

/* CRAM layout expected for the model and effect tables */
#define FS199X_MODEL_SIZE		700
#define FS199X_MODEL_SIZE_A		340
#define FS199X_EFFECT_SIZE		1212
#define FS199X_EFFECT_SIZE_A		240

/*
 * Speaker protection constants, stored in the init scene model of the
 * firmware file, each XORed with FS199X_PROT_XOR (a file format constant).
 */
#define FS199X_PROT_C_OFFSET		0x1A0
#define FS199X_PROT_A_OFFSET		0x1B0
#define FS199X_PROT_B_OFFSET		0x1B4
#define FS199X_PROT_XOR			0x001801AD

#define FS199X_RATES			(SNDRV_PCM_RATE_8000 | \
					 SNDRV_PCM_RATE_16000 | \
					 SNDRV_PCM_RATE_32000 | \
					 SNDRV_PCM_RATE_44100 | \
					 SNDRV_PCM_RATE_48000 | \
					 SNDRV_PCM_RATE_88200 | \
					 SNDRV_PCM_RATE_96000)
#define FS199X_FORMATS			(SNDRV_PCM_FMTBIT_S16_LE | \
					 SNDRV_PCM_FMTBIT_S24_LE | \
					 SNDRV_PCM_FMTBIT_S32_LE)

static const char *const fs199x_supply_names[] = {
	"pvdd",
	"dvdd",
};

#define FS199X_NUM_SUPPLIES		ARRAY_SIZE(fs199x_supply_names)

struct fs199x_priv {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[FS199X_NUM_SUPPLIES];
	struct gpio_desc *gpio_sdz;
	struct delayed_work start_work;
	struct delayed_work fault_check_work;
	struct fs_amp_lib amp_lib;
	const struct fs_amp_scene *cur_scene;
	const char *fwm_name;
	/*
	 * @lock: serializes chip access between probe/init, the DAPM and DAI
	 * callbacks, the controls, PM and the delayed works.
	 */
	struct mutex lock;
	unsigned int bclk;
	unsigned int srate;
	unsigned int i2sf;	/* 0: keep the firmware format */
	u32 ref_re;		/* speaker Re from DT, in OTP Re units << 2 */
	int scene_id;
	int volume;
	bool is_inited;
	bool is_suspended;
	bool is_playing;
	bool is_faulted;
};

static const struct fs_pll_div fs199x_pll_div[] = {
	/*    bclk,   pll1,   pll2,   pll3 */
	{  1024000, 0x0264, 0x0B40, 0x000C },
	{  1411200, 0x0260, 0x0540, 0x0006 },
	{  1536000, 0x0260, 0x0540, 0x0006 },
	{  2048000, 0x0264, 0x05A0, 0x000C },
	{  2822400, 0x0260, 0x0540, 0x000C },
	{  3072000, 0x0264, 0x03C0, 0x000C },
	{  5644800, 0x0260, 0x0540, 0x0018 },
	{  6144000, 0x0264, 0x01E0, 0x000C },
	{ 12288000, 0x0260, 0x0540, 0x0030 },
};

static const struct fs_i2s_srate fs199x_srate[] = {
	{  8000, 0x1 },
	{ 16000, 0x3 },
	{ 32000, 0x7 },
	{ 44100, 0x8 },
	{ 48000, 0x9 },
	{ 88200, 0xA },
	{ 96000, 0xB },
};

static int fs199x_reg_write(struct fs199x_priv *fs199x, u8 reg, u16 val)
{
	int ret;

	ret = regmap_write(fs199x->regmap, reg, val);
	if (ret)
		dev_err(fs199x->dev, "Failed to write %02Xh: %d\n", reg, ret);

	return ret;
}

static int fs199x_reg_read(struct fs199x_priv *fs199x, u8 reg, u16 *pval)
{
	unsigned int val;
	int ret;

	ret = regmap_read(fs199x->regmap, reg, &val);
	if (ret) {
		dev_err(fs199x->dev, "Failed to read %02Xh: %d\n", reg, ret);
		return ret;
	}

	*pval = val;

	return 0;
}

static int fs199x_reg_update_bits(struct fs199x_priv *fs199x,
				  u8 reg, u16 mask, u16 val)
{
	int ret;

	ret = regmap_update_bits(fs199x->regmap, reg, mask, val);
	if (ret)
		dev_err(fs199x->dev, "Failed to update %02Xh: %d\n", reg, ret);

	return ret;
}

static int fs199x_reg_bulk_write(struct fs199x_priv *fs199x,
				 u8 reg, const void *val, u32 size)
{
	int ret;

	ret = regmap_bulk_write(fs199x->regmap, reg, val, size / 2);
	if (ret)
		dev_err(fs199x->dev, "Failed to bulk write %02Xh: %d\n",
			reg, ret);

	return ret;
}

/* wait until (reg & mask) == val: 5 ms, then up to 20 polls 2 ms apart */
static int fs199x_wait_bits(struct fs199x_priv *fs199x, u8 reg,
			    u16 mask, u16 val)
{
	u16 cur = 0;
	int i, ret;

	fsleep(5000);
	for (i = 0; i < 20; i++) {
		ret = fs199x_reg_read(fs199x, reg, &cur);
		if (ret)
			return ret;
		if ((cur & mask) == val)
			return 0;
		fsleep(2000);
	}

	dev_err(fs199x->dev, "Wait %02Xh timeout: %04X\n", reg, cur);

	return -ETIMEDOUT;
}

static int fs199x_set_cmd_pkg(struct fs199x_priv *fs199x,
			      const struct fs_cmd_pkg *pkg,
			      unsigned int *offset)
{
	if (pkg->cmd <= FS199X_REG_MAX) {
		*offset = sizeof(pkg->regv);
		return fs199x_reg_write(fs199x, pkg->regv.reg, pkg->regv.val);
	} else if (pkg->cmd == FS_CMD_UPDATE) {
		*offset = sizeof(pkg->regb);
		return fs199x_reg_update_bits(fs199x, pkg->regb.reg,
					      pkg->regb.mask, pkg->regb.val);
	} else if (pkg->cmd == FS_CMD_DELAY) {
		if (pkg->regv.val > FS_CMD_DELAY_MS_MAX)
			return -EOPNOTSUPP;
		fsleep(pkg->regv.val * 1000);
		*offset = sizeof(pkg->regv);
		return 0;
	}

	dev_err(fs199x->dev, "Invalid pkg cmd: %d\n", pkg->cmd);

	return -EOPNOTSUPP;
}

static int fs199x_reg_write_table(struct fs199x_priv *fs199x,
				  const struct fs_reg_table *reg)
{
	const struct fs_cmd_pkg *pkg;
	unsigned int index, offset;
	int ret;

	if (!reg || reg->size == 0)
		return -EFAULT;

	for (index = 0; index < reg->size; index += offset) {
		pkg = (const struct fs_cmd_pkg *)(reg->buf + index);
		ret = fs199x_set_cmd_pkg(fs199x, pkg, &offset);
		if (ret) {
			dev_err(fs199x->dev, "Failed to set cmd pkg: %02X-%d\n",
				pkg->cmd, ret);
			return ret;
		}
	}

	if (index != reg->size) {
		dev_err(fs199x->dev, "Invalid reg table size: %d-%d\n",
			index, reg->size);
		return -EFAULT;
	}

	return 0;
}

/* PWDN, then wait for the amplifier and the digital core to settle */
static int fs199x_dev_mute(struct fs199x_priv *fs199x)
{
	u16 val;
	int ret;

	ret = fs199x_reg_write(fs199x, FS199X_10H_PWRCTRL, FS199X_10H_PWDN);
	if (ret)
		return ret;

	ret = fs199x_wait_bits(fs199x, FS199X_4CH_AMPCTRL,
			       FS199X_4CH_AMP_OFF, FS199X_4CH_AMP_OFF);
	ret |= fs199x_wait_bits(fs199x, FS199X_06H_DIGSTAT,
				FS199X_06H_AMP_BUSY, 0);

	/* clear the latched events */
	fs199x_reg_read(fs199x, FS199X_ABH_INTSTAT, &val);

	return ret;
}

static int fs199x_access_cram(struct fs199x_priv *fs199x, bool enable)
{
	int ret;

	if (enable) {
		ret = fs199x_reg_write(fs199x, FS199X_11H_SYSCTRL,
				       FS199X_11H_STANDBY);
		ret |= fs199x_reg_write(fs199x, FS199X_10H_PWRCTRL,
					FS199X_10H_ON);
		ret |= fs199x_reg_write(fs199x, FS199X_0BH_ACCKEY,
					FS199X_0BH_ACCKEY_ON);
	} else {
		ret = fs199x_reg_write(fs199x, FS199X_0BH_ACCKEY,
				       FS199X_0BH_ACCKEY_OFF);
		ret |= fs199x_reg_write(fs199x, FS199X_10H_PWRCTRL,
					FS199X_10H_PWDN);
		ret |= fs199x_reg_write(fs199x, FS199X_11H_SYSCTRL,
					FS199X_11H_STANDBY);
	}
	if (ret)
		return ret;

	return fs199x_wait_bits(fs199x, FS199X_06H_DIGSTAT,
				FS199X_06H_BUSY_MASK, 0);
}

static int fs199x_write_model(struct fs199x_priv *fs199x,
			      const struct fs_file_table *model)
{
	int ret;

	if (model->size != FS199X_MODEL_SIZE) {
		dev_err(fs199x->dev, "Invalid model size: %d\n", model->size);
		return -EINVAL;
	}

	ret = fs199x_reg_write(fs199x, FS199X_66H_CRAM_ADDR_A,
			       FS199X_66H_MODEL);
	ret |= fs199x_reg_write(fs199x, FS199X_86H_CRAM_ADDR_B,
				FS199X_86H_MODEL);
	if (!ret)
		ret = fs199x_reg_bulk_write(fs199x, FS199X_62H_CRAM_DATA_A,
					    model->buf, FS199X_MODEL_SIZE_A);
	if (!ret)
		ret = fs199x_reg_bulk_write(fs199x, FS199X_82H_CRAM_DATA_B,
					    model->buf + FS199X_MODEL_SIZE_A,
					    FS199X_MODEL_SIZE -
					    FS199X_MODEL_SIZE_A);

	fs199x_reg_write(fs199x, FS199X_66H_CRAM_ADDR_A, FS199X_CRAM_CLEAR);
	fs199x_reg_write(fs199x, FS199X_86H_CRAM_ADDR_B, FS199X_CRAM_CLEAR);

	return ret;
}

static int fs199x_write_effect(struct fs199x_priv *fs199x,
			       const struct fs_file_table *effect)
{
	int ret;

	if (effect->size != FS199X_EFFECT_SIZE) {
		dev_err(fs199x->dev, "Invalid effect size: %d\n", effect->size);
		return -EINVAL;
	}

	ret = fs199x_reg_write(fs199x, FS199X_66H_CRAM_ADDR_A,
			       FS199X_66H_EFFECT_A);
	if (!ret)
		ret = fs199x_reg_bulk_write(fs199x, FS199X_62H_CRAM_DATA_A,
					    effect->buf, FS199X_EFFECT_SIZE_A);
	if (!ret)
		ret = fs199x_reg_write(fs199x, FS199X_66H_CRAM_ADDR_A,
				       FS199X_66H_EFFECT_B);
	if (!ret)
		ret = fs199x_reg_bulk_write(fs199x, FS199X_62H_CRAM_DATA_A,
					    effect->buf + FS199X_EFFECT_SIZE_A,
					    FS199X_EFFECT_SIZE -
					    FS199X_EFFECT_SIZE_A);

	fs199x_reg_write(fs199x, FS199X_66H_CRAM_ADDR_A, FS199X_CRAM_CLEAR);

	return ret;
}

static int fs199x_write_cram(struct fs199x_priv *fs199x,
			     const struct fs_amp_scene *scene)
{
	const struct fs_amp_scene *cur = fs199x->cur_scene;
	bool model = scene->model && (!cur || cur->model != scene->model);
	bool effect = scene->effect && (!cur || cur->effect != scene->effect);
	u16 val;
	int ret;

	if (!model && !effect)
		return 0;

	ret = fs199x_access_cram(fs199x, true);
	if (ret)
		goto exit;

	if (model) {
		ret = fs199x_write_model(fs199x, scene->model);
		if (ret)
			goto exit;
	}

	if (effect)
		ret = fs199x_write_effect(fs199x, scene->effect);

	fs199x_reg_read(fs199x, FS199X_ABH_INTSTAT, &val);
exit:
	ret |= fs199x_access_cram(fs199x, false);

	return ret;
}

/*
 * Speaker protection parameters from the speaker resistance: the OTP copy
 * if the speaker was calibrated, otherwise the DT value. Without either
 * the protection is left as the firmware set it.
 */
static int fs199x_set_spkr_prot(struct fs199x_priv *fs199x)
{
	const struct fs_amp_scene *init_scene = fs199x->amp_lib.scene;
	const struct fs_file_table *model = init_scene ? init_scene->model : NULL;
	u32 a, b, c, spkre = 0;
	u64 prot0, prot1;
	u16 count, otp_re;
	int ret;

	if (!model || model->size < FS199X_PROT_B_OFFSET + 4)
		return 0;

	ret = fs199x_reg_write(fs199x, FS199X_0BH_ACCKEY, FS199X_0BH_ACCKEY_ON);
	if (ret)
		return ret;

	ret = fs199x_reg_read(fs199x, FS199X_F0H_OTP_STAT, &count);
	ret |= fs199x_reg_read(fs199x, FS199X_F1H_SPKRE, &otp_re);
	if (ret)
		goto exit;

	if ((count & FS199X_F0H_RE_COUNT) && otp_re != 0xFFFF)
		spkre = (u32)otp_re << 2;
	else
		spkre = fs199x->ref_re;

	if (!spkre) {
		dev_warn_once(fs199x->dev, "No speaker Re, protection as in firmware\n");
		goto exit;
	}

	c = get_unaligned_le32(model->buf + FS199X_PROT_C_OFFSET) ^ FS199X_PROT_XOR;
	a = get_unaligned_le32(model->buf + FS199X_PROT_A_OFFSET) ^ FS199X_PROT_XOR;
	b = get_unaligned_le32(model->buf + FS199X_PROT_B_OFFSET) ^ FS199X_PROT_XOR;
	if (!b || !c || (a & BIT(31))) {
		dev_err(fs199x->dev, "Invalid protection constants\n");
		ret = -EINVAL;
		goto exit;
	}

	prot0 = div_u64(((u64)a + spkre) << 18, b);
	prot0 = min_t(u64, prot0, 0xFFFF);
	prot1 = div_u64(div_u64(BIT_ULL(47), spkre), c);
	prot1 = min_t(u64, prot1, 0xFFFFFFFF);

	ret = fs199x_reg_write(fs199x, FS199X_95H_PROT0, prot0);
	ret |= fs199x_reg_write(fs199x, FS199X_96H_PROT1_HI, prot1 >> 16);
	ret |= fs199x_reg_write(fs199x, FS199X_97H_PROT1_LO, prot1 & 0xFFFF);
	ret |= fs199x_reg_write(fs199x, FS199X_F1H_SPKRE,
				min_t(u32, spkre >> 2, 0xFFFF));

exit:
	ret |= fs199x_reg_write(fs199x, FS199X_0BH_ACCKEY,
				FS199X_0BH_ACCKEY_OFF);

	return ret;
}

static int fs199x_dev_play(struct fs199x_priv *fs199x)
{
	u16 val;
	int ret;

	if (!fs199x->is_inited)
		return -EFAULT;

	if (fs199x->is_playing)
		return 0;

	ret = fs199x_reg_write(fs199x, FS199X_11H_SYSCTRL, FS199X_11H_PLAY);
	if (ret)
		return ret;

	ret = fs199x_set_spkr_prot(fs199x);
	if (ret)
		goto err_stop;

	ret = fs199x_reg_update_bits(fs199x, FS199X_16H_VOLUME,
				     FS199X_16H_VOL_MASK,
				     fs199x->volume << FS199X_16H_VOL_SHIFT);
	if (ret)
		goto err_stop;

	ret = fs199x_reg_write(fs199x, FS199X_10H_PWRCTRL, FS199X_10H_ON);
	if (ret)
		goto err_stop;

	fs199x_reg_read(fs199x, FS199X_ABH_INTSTAT, &val);
	fs199x->is_playing = true;
	fs199x->is_faulted = false;

	return 0;

err_stop:
	fs199x_reg_write(fs199x, FS199X_10H_PWRCTRL, FS199X_10H_PWDN);
	fs199x_reg_write(fs199x, FS199X_11H_SYSCTRL, FS199X_11H_STANDBY);

	return ret;
}

static int fs199x_dev_stop(struct fs199x_priv *fs199x)
{
	int ret;

	if (!fs199x->is_inited || !fs199x->is_playing)
		return 0;

	ret = fs199x_dev_mute(fs199x);
	ret |= fs199x_reg_write(fs199x, FS199X_11H_SYSCTRL,
				FS199X_11H_STANDBY);
	fs199x->is_playing = false;

	return ret;
}

static int fs199x_set_scene(struct fs199x_priv *fs199x, int scene_id)
{
	const struct fs_amp_scene *scene;
	bool is_playing;
	int ret;

	if (!fs199x->amp_lib.scene || scene_id < 0 ||
	    scene_id >= fs199x->amp_lib.scene_count)
		return -EINVAL;

	if (fs199x->scene_id == scene_id)
		return 0;

	scene = fs199x->amp_lib.scene + scene_id;
	dev_info(fs199x->dev, "Switch scene.%d: %s\n", scene_id, scene->name);

	is_playing = fs199x->is_playing;
	if (is_playing)
		fs199x_dev_stop(fs199x);

	if (scene->reg && fs199x->cur_scene != scene) {
		ret = fs199x_reg_write_table(fs199x, scene->reg);
		if (ret) {
			dev_err(fs199x->dev, "Failed to set reg: %d\n", ret);
			return ret;
		}
	}

	ret = fs199x_write_cram(fs199x, scene);
	if (ret) {
		dev_err(fs199x->dev, "Failed to write cram: %d\n", ret);
		return ret;
	}

	fs199x->cur_scene = scene;
	fs199x->scene_id = scene_id;

	if (is_playing) {
		fsleep(5000);
		fs199x_dev_play(fs199x);
	}

	return 0;
}

static int fs199x_init_chip(struct fs199x_priv *fs199x)
{
	int scene_id;
	int ret;

	fs199x->is_inited = false;
	fs199x->is_playing = false;
	fs199x->cur_scene = NULL;

	ret = fs199x_reg_write(fs199x, FS199X_10H_PWRCTRL,
			       FS199X_10H_I2C_RESET);
	if (ret)
		return ret;
	fsleep(10000); /* >= 10ms */

	ret = fs199x_wait_bits(fs199x, FS199X_0EH_CHIPINI,
			       FS199X_0EH_INIT_DONE, FS199X_0EH_INIT_DONE);
	if (ret) {
		dev_err(fs199x->dev, "Failed to check CHIPINI\n");
		return ret;
	}

	scene_id = fs199x->scene_id;
	fs199x->scene_id = -1;

	/* registers and the speaker model from the init scene */
	ret = fs199x_set_scene(fs199x, FS199X_INIT_SCENE);
	if (ret)
		goto power_down;

	ret = fs199x_dev_mute(fs199x);
	ret |= fs199x_reg_write(fs199x, FS199X_11H_SYSCTRL,
				FS199X_11H_STANDBY);
	if (ret)
		goto power_down;

	/* then the effect scene: the stored one, else the default one */
	if (fs199x->amp_lib.scene_count > 1) {
		if (scene_id < FS199X_DEFAULT_SCENE)
			scene_id = FS199X_DEFAULT_SCENE;
		ret = fs199x_set_scene(fs199x, scene_id);
		if (ret)
			goto power_down;
	}

	fs199x->is_inited = true;

	return 0;

power_down:
	fs199x_reg_write(fs199x, FS199X_10H_PWRCTRL, FS199X_10H_PWDN);
	fs199x_reg_write(fs199x, FS199X_11H_SYSCTRL, FS199X_11H_STANDBY);

	return ret;
}

static int fs199x_set_hw_params(struct fs199x_priv *fs199x)
{
	const struct fs_pll_div *pll_div = NULL;
	u16 mask = FS199X_17H_I2SSR_MASK;
	u16 val = 0;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(fs199x_srate); i++) {
		if (fs199x_srate[i].srate == fs199x->srate) {
			val = fs199x_srate[i].i2ssr << FS199X_17H_I2SSR_SHIFT;
			break;
		}
	}
	if (i == ARRAY_SIZE(fs199x_srate)) {
		dev_err(fs199x->dev, "Invalid sample rate: %d\n", fs199x->srate);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(fs199x_pll_div); i++) {
		if (fs199x_pll_div[i].bclk == fs199x->bclk) {
			pll_div = fs199x_pll_div + i;
			break;
		}
	}
	if (!pll_div) {
		dev_err(fs199x->dev, "No PLL table for bclk: %d\n", fs199x->bclk);
		return -EINVAL;
	}

	if (fs199x->i2sf) {
		mask |= FS199X_17H_I2SF_MASK;
		val |= fs199x->i2sf;
	}

	ret = fs199x_reg_update_bits(fs199x, FS199X_17H_I2SCTRL, mask, val);
	ret |= fs199x_reg_write(fs199x, FS199X_A1H_PLLCTRL1, pll_div->pll1);
	ret |= fs199x_reg_write(fs199x, FS199X_A2H_PLLCTRL2, pll_div->pll2);
	ret |= fs199x_reg_write(fs199x, FS199X_A3H_PLLCTRL3, pll_div->pll3);

	return ret;
}

static int fs199x_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(dai->component);

	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC) {
		dev_err(fs199x->dev, "Only supports consumer mode\n");
		return -EINVAL;
	}

	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		fs199x->i2sf = FS199X_17H_I2SF_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		fs199x->i2sf = FS199X_17H_I2SF_LJ;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		fs199x->i2sf = FS199X_17H_I2SF_RJ;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		fs199x->i2sf = FS199X_17H_I2SF_DSP;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int fs199x_dai_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(dai->component);
	int ret;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	fs199x->srate = params_rate(params);
	fs199x->bclk = snd_soc_params_to_bclk(params);
	if (params_channels(params) == 1)
		fs199x->bclk *= 2; /* I2S bus has 2 channels */

	mutex_lock(&fs199x->lock);
	ret = fs199x_set_hw_params(fs199x);
	mutex_unlock(&fs199x->lock);

	return ret;
}

static int fs199x_dai_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(dai->component);

	if (stream != SNDRV_PCM_STREAM_PLAYBACK || !mute)
		return 0;

	cancel_delayed_work_sync(&fs199x->start_work);
	cancel_delayed_work_sync(&fs199x->fault_check_work);

	mutex_lock(&fs199x->lock);
	if (fs199x->is_inited && !fs199x->is_suspended)
		fs199x_dev_stop(fs199x);
	mutex_unlock(&fs199x->lock);

	return 0;
}

static int fs199x_dai_trigger(struct snd_pcm_substream *substream,
			      int cmd, struct snd_soc_dai *dai)
{
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(dai->component);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* start once BCLK has been running for a few ms */
		schedule_delayed_work(&fs199x->start_work,
				      msecs_to_jiffies(FS199X_START_DELAY_MS));
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		cancel_delayed_work(&fs199x->start_work);
		break;
	default:
		break;
	}

	return 0;
}

static void fs199x_start_work(struct work_struct *work)
{
	struct fs199x_priv *fs199x =
		container_of(work, struct fs199x_priv, start_work.work);
	int ret;

	mutex_lock(&fs199x->lock);

	if (fs199x->is_inited && !fs199x->is_suspended) {
		ret = fs199x_dev_play(fs199x);
		if (ret)
			dev_err(fs199x->dev, "Failed to start playing: %d\n", ret);
		else
			schedule_delayed_work(&fs199x->fault_check_work,
				msecs_to_jiffies(FS199X_FAULT_CHECK_INTERVAL_MS));
	}

	mutex_unlock(&fs199x->lock);
}

static void fs199x_fault_check_work(struct work_struct *work)
{
	struct fs199x_priv *fs199x =
		container_of(work, struct fs199x_priv, fault_check_work.work);
	u16 status = 0, anastat = 0, events = 0;
	int ret;

	mutex_lock(&fs199x->lock);

	if (!fs199x->is_inited || fs199x->is_suspended || !fs199x->is_playing) {
		mutex_unlock(&fs199x->lock);
		return;
	}

	ret = fs199x_reg_read(fs199x, FS199X_00H_STATUS, &status);
	ret |= fs199x_reg_read(fs199x, FS199X_05H_ANASTAT, &anastat);
	ret |= fs199x_reg_read(fs199x, FS199X_ABH_INTSTAT, &events);
	if (ret)
		goto out;

	if (status != FS199X_00H_NORMAL ||
	    (anastat & FS199X_05H_READY_MASK) != FS199X_05H_READY_MASK || events)
		dev_warn(fs199x->dev, "status %04X anastat %04X events %04X\n",
			 status, anastat, events);

	if (!(status & FS199X_00H_PLLS))
		dev_err(fs199x->dev, "PLL unlock\n");
	if (!(status & FS199X_00H_CLKS))
		dev_err(fs199x->dev, "CLK ratio unstable\n");
	if (events & FS199X_ABH_CLKLOST)
		dev_err(fs199x->dev, "Clock lost\n");
	if (events & (FS199X_ABH_UV | FS199X_ABH_BOP | FS199X_ABH_BOL |
		      FS199X_ABH_BOV))
		dev_warn(fs199x->dev, "Supply low / brownout\n");
	if (events & FS199X_ABH_OTW)
		dev_warn(fs199x->dev, "Over temperature warning\n");

	/* the chip protects itself; keep it muted until the next start */
	if ((events & FS199X_ABH_FATAL) ||
	    (status & (FS199X_00H_VDDPS | FS199X_00H_OCAMP | FS199X_00H_OT |
		       FS199X_00H_OV))) {
		dev_err(fs199x->dev, "Fault (status %04X events %04X), muting\n",
			status, events);
		fs199x_dev_stop(fs199x);
		fs199x->is_faulted = true;
		goto out;
	}

	schedule_delayed_work(&fs199x->fault_check_work,
			      msecs_to_jiffies(FS199X_FAULT_CHECK_INTERVAL_MS));
out:
	mutex_unlock(&fs199x->lock);
}

static int fs199x_volume_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = FS199X_16H_VOL_MAX;

	return 0;
}

static int fs199x_volume_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(cmpnt);

	ucontrol->value.integer.value[0] = fs199x->volume;

	return 0;
}

static int fs199x_volume_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(cmpnt);
	long val = ucontrol->value.integer.value[0];
	int ret = 0;

	if (val < 0 || val > FS199X_16H_VOL_MAX)
		return -EINVAL;

	mutex_lock(&fs199x->lock);
	if (val == fs199x->volume) {
		mutex_unlock(&fs199x->lock);
		return 0;
	}

	fs199x->volume = val;
	if (fs199x->is_playing)
		ret = fs199x_reg_update_bits(fs199x, FS199X_16H_VOLUME,
					     FS199X_16H_VOL_MASK,
					     val << FS199X_16H_VOL_SHIFT);
	mutex_unlock(&fs199x->lock);

	return ret ? ret : 1;
}

static int fs199x_effect_scene_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(cmpnt);
	const struct fs_amp_scene *scene;
	int count = fs199x->amp_lib.scene_count - 1; /* skip init scene */
	const char *name = "N/A";

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;

	if (count < 1) {
		uinfo->value.enumerated.items = 0;
		return 0;
	}

	uinfo->value.enumerated.items = count;
	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	scene = fs199x->amp_lib.scene + uinfo->value.enumerated.item + 1;
	if (scene->name)
		name = scene->name;

	strscpy(uinfo->value.enumerated.name, name);

	return 0;
}

static int fs199x_effect_scene_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(cmpnt);

	mutex_lock(&fs199x->lock);
	ucontrol->value.enumerated.item[0] =
		fs199x->scene_id > 0 ? fs199x->scene_id - 1 : 0;
	mutex_unlock(&fs199x->lock);

	return 0;
}

static int fs199x_effect_scene_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(cmpnt);
	int scene_id = ucontrol->value.enumerated.item[0] + 1;
	bool changed;
	int ret = 0;

	if (scene_id < 1 || scene_id >= fs199x->amp_lib.scene_count)
		return -ERANGE;

	mutex_lock(&fs199x->lock);
	changed = scene_id != fs199x->scene_id;
	if (fs199x->is_suspended || !fs199x->is_inited)
		fs199x->scene_id = scene_id; /* applied by the next init */
	else
		ret = fs199x_set_scene(fs199x, scene_id);
	mutex_unlock(&fs199x->lock);

	if (ret)
		return ret;

	return changed;
}

static int fs199x_playback_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kc, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(cmpnt);
	int ret = 0;

	if (event != SND_SOC_DAPM_POST_PMD)
		return 0;

	cancel_delayed_work_sync(&fs199x->start_work);
	cancel_delayed_work_sync(&fs199x->fault_check_work);

	mutex_lock(&fs199x->lock);
	if (!fs199x->is_suspended)
		ret = fs199x_dev_stop(fs199x);
	mutex_unlock(&fs199x->lock);

	return ret;
}

static const struct snd_soc_dai_ops fs199x_dai_ops = {
	.set_fmt	= fs199x_dai_set_fmt,
	.hw_params	= fs199x_dai_hw_params,
	.mute_stream	= fs199x_dai_mute,
	.trigger	= fs199x_dai_trigger,
};

static struct snd_soc_dai_driver fs199x_dai = {
	.name = FS199X_DAI_NAME,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = FS199X_RATES,
		.formats = FS199X_FORMATS,
	},
	.ops = &fs199x_dai_ops,
};

static const struct snd_kcontrol_new fs199x_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Speaker Volume",
		.info = fs199x_volume_info,
		.get = fs199x_volume_get,
		.put = fs199x_volume_put,
	},
};

static const struct snd_kcontrol_new fs199x_scene_control[] = {
	FS_SOC_ENUM_EXT("Effect Scene",
			fs199x_effect_scene_info,
			fs199x_effect_scene_get,
			fs199x_effect_scene_put),
};

static const struct snd_soc_dapm_widget fs199x_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN_E("AIF IN", "Playback", 0, SND_SOC_NOPM, 0, 0,
			      fs199x_playback_event, SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route fs199x_dapm_routes[] = {
	{ "SPK", NULL, "AIF IN" },
};

static int fs199x_probe(struct snd_soc_component *cmpnt)
{
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(cmpnt);
	int ret;

	fs199x->amp_lib.dev = fs199x->dev;
	fs199x->amp_lib.devid = FS199X_DEVICE_ID;

	/* without the firmware there is no speaker protection: no output */
	ret = fs_amp_load_firmware(&fs199x->amp_lib, fs199x->fwm_name);
	if (ret)
		return ret;

	if (!fs199x->amp_lib.scene || !fs199x->amp_lib.scene[0].model) {
		dev_err(fs199x->dev, "Firmware has no speaker model\n");
		return -EINVAL;
	}

	if (fs199x->amp_lib.scene_count > 1) {
		ret = snd_soc_add_component_controls(cmpnt, fs199x_scene_control,
						     ARRAY_SIZE(fs199x_scene_control));
		if (ret)
			return ret;
	}

	mutex_lock(&fs199x->lock);
	ret = fs199x_init_chip(fs199x);
	mutex_unlock(&fs199x->lock);

	return ret;
}

static void fs199x_remove(struct snd_soc_component *cmpnt)
{
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(cmpnt);

	cancel_delayed_work_sync(&fs199x->start_work);
	cancel_delayed_work_sync(&fs199x->fault_check_work);

	mutex_lock(&fs199x->lock);
	fs199x_dev_stop(fs199x);
	mutex_unlock(&fs199x->lock);
}

static int fs199x_suspend(struct snd_soc_component *cmpnt)
{
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(cmpnt);

	cancel_delayed_work_sync(&fs199x->start_work);
	cancel_delayed_work_sync(&fs199x->fault_check_work);

	mutex_lock(&fs199x->lock);
	fs199x_dev_stop(fs199x);
	fs199x->is_inited = false;
	fs199x->is_suspended = true;
	gpiod_set_value_cansleep(fs199x->gpio_sdz, 1); /* shut down */
	mutex_unlock(&fs199x->lock);

	return regulator_bulk_disable(FS199X_NUM_SUPPLIES, fs199x->supplies);
}

static int fs199x_resume(struct snd_soc_component *cmpnt)
{
	struct fs199x_priv *fs199x = snd_soc_component_get_drvdata(cmpnt);
	int ret;

	ret = regulator_bulk_enable(FS199X_NUM_SUPPLIES, fs199x->supplies);
	if (ret)
		return ret;

	mutex_lock(&fs199x->lock);
	fsleep(10000);
	gpiod_set_value_cansleep(fs199x->gpio_sdz, 0);
	fsleep(10000);
	fs199x->is_suspended = false;
	ret = fs199x_init_chip(fs199x);
	mutex_unlock(&fs199x->lock);

	return ret;
}

static const struct snd_soc_component_driver fs199x_soc_component_dev = {
	.probe			= fs199x_probe,
	.remove			= fs199x_remove,
	.suspend		= pm_ptr(fs199x_suspend),
	.resume			= pm_ptr(fs199x_resume),
	.controls		= fs199x_controls,
	.num_controls		= ARRAY_SIZE(fs199x_controls),
	.dapm_widgets		= fs199x_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(fs199x_dapm_widgets),
	.dapm_routes		= fs199x_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(fs199x_dapm_routes),
};

/* no register cache: the chip is reset and reprogrammed from firmware */
static const struct regmap_config fs199x_regmap = {
	.reg_bits		= 8,
	.val_bits		= 16,
	.max_register		= FS199X_REG_MAX,
	.val_format_endian	= REGMAP_ENDIAN_BIG,
	.cache_type		= REGCACHE_NONE,
};

static int fs199x_detect_device(struct fs199x_priv *fs199x)
{
	u16 devid, rev;
	int ret;

	ret = fs199x_reg_read(fs199x, FS199X_03H_DEVID, &devid);
	if (ret)
		return ret;

	switch (HI_U16(devid)) {
	case FS199X_DEVICE_ID:
		break;
	case FS199X_DEVICE_ID_REV:
		ret = fs199x_reg_write(fs199x, FS199X_0BH_ACCKEY,
				       FS199X_0BH_ACCKEY_ON);
		ret |= fs199x_reg_read(fs199x, FS199X_C3H_REVCHK, &rev);
		ret |= fs199x_reg_write(fs199x, FS199X_0BH_ACCKEY,
					FS199X_0BH_ACCKEY_OFF);
		if (ret)
			return ret;
		if ((rev & FS199X_C3H_REV_MASK) != FS199X_C3H_REV_OK) {
			dev_err(fs199x->dev, "Unknown revision: %04X\n", rev);
			return -ENODEV;
		}
		break;
	default:
		dev_err(fs199x->dev, "DEVID: 0x%04X dismatch\n", devid);
		return -ENODEV;
	}

	dev_info(fs199x->dev, "FS199x detected, DEVID 0x%04X\n", devid);

	return 0;
}

static int fs199x_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fs199x_priv *fs199x;
	int i, ret;

	fs199x = devm_kzalloc(dev, sizeof(*fs199x), GFP_KERNEL);
	if (!fs199x)
		return -ENOMEM;

	fs199x->dev = dev;
	i2c_set_clientdata(client, fs199x);

	fs199x->regmap = devm_regmap_init_i2c(client, &fs199x_regmap);
	if (IS_ERR(fs199x->regmap))
		return dev_err_probe(dev, PTR_ERR(fs199x->regmap),
				     "Failed to get regmap\n");

	if (device_property_read_string(dev, "firmware-name", &fs199x->fwm_name))
		fs199x->fwm_name = FS199X_DEFAULT_FWM_NAME;
	device_property_read_u32(dev, "foursemi,speaker-re", &fs199x->ref_re);

	/* SDZ: kept low (reset asserted) until the supplies are on */
	fs199x->gpio_sdz = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(fs199x->gpio_sdz))
		return dev_err_probe(dev, PTR_ERR(fs199x->gpio_sdz),
				     "Failed to get reset-gpios\n");

	for (i = 0; i < FS199X_NUM_SUPPLIES; i++)
		fs199x->supplies[i].supply = fs199x_supply_names[i];
	ret = devm_regulator_bulk_get(dev, FS199X_NUM_SUPPLIES, fs199x->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get supplies\n");

	ret = regulator_bulk_enable(FS199X_NUM_SUPPLIES, fs199x->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable supplies\n");

	fsleep(10000); /* >= 10ms */
	gpiod_set_value_cansleep(fs199x->gpio_sdz, 0);
	fsleep(10000); /* >= 10ms */

	ret = fs199x_detect_device(fs199x);
	if (ret)
		goto err_off;

	fs199x->scene_id = -1;
	fs199x->volume = FS199X_DEFAULT_VOLUME;
	mutex_init(&fs199x->lock);
	INIT_DELAYED_WORK(&fs199x->start_work, fs199x_start_work);
	INIT_DELAYED_WORK(&fs199x->fault_check_work, fs199x_fault_check_work);

	ret = devm_snd_soc_register_component(dev, &fs199x_soc_component_dev,
					      &fs199x_dai, 1);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to register component\n");
		goto err_off;
	}

	return 0;

err_off:
	gpiod_set_value_cansleep(fs199x->gpio_sdz, 1);
	regulator_bulk_disable(FS199X_NUM_SUPPLIES, fs199x->supplies);

	return ret;
}

static void fs199x_i2c_remove(struct i2c_client *client)
{
	struct fs199x_priv *fs199x = i2c_get_clientdata(client);

	gpiod_set_value_cansleep(fs199x->gpio_sdz, 1);
	regulator_bulk_disable(FS199X_NUM_SUPPLIES, fs199x->supplies);
}

static void fs199x_i2c_shutdown(struct i2c_client *client)
{
	struct fs199x_priv *fs199x = i2c_get_clientdata(client);

	/* power the output stage off before the rails go */
	gpiod_set_value_cansleep(fs199x->gpio_sdz, 1);
}

static const struct i2c_device_id fs199x_i2c_id[] = {
	{ .name = "fs199x" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fs199x_i2c_id);

static const struct of_device_id fs199x_of_match[] = {
	{ .compatible = "foursemi,fs199x" },
	{ }
};
MODULE_DEVICE_TABLE(of, fs199x_of_match);

static struct i2c_driver fs199x_i2c_driver = {
	.driver = {
		.name = "fs199x",
		.of_match_table = fs199x_of_match,
	},
	.id_table = fs199x_i2c_id,
	.probe = fs199x_i2c_probe,
	.remove = fs199x_i2c_remove,
	.shutdown = fs199x_i2c_shutdown,
};
module_i2c_driver(fs199x_i2c_driver);

MODULE_DESCRIPTION("FourSemi FS199x Audio Amplifier Driver");
MODULE_LICENSE("GPL");
