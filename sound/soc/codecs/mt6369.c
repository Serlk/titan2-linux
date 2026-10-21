// SPDX-License-Identifier: GPL-2.0
//
// mt6369.c  --  mt6369 ALSA SoC audio codec driver
//
// The register sequences come from the MediaTek MT6369 codec driver of
// the MT6878 BSP (kernel_device_modules-6.1); the structure follows the
// mainline mt6359 driver. Left out: voice wake-up, DMIC, the second
// uplink (UL2/ADDA6), the third DAC (AIF2), headphone DC trim and
// impedance detection.
//
// Copyright (c) 2018 MediaTek Inc.
// Author: Chih-hong Yang <chih-hong.yang@mediatek.com>
// Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>

#include <linux/delay.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "mt6369.h"

struct mt6369_priv {
	struct device *dev;
	struct regmap *regmap;
	/* gains as register indexes: 0 = +8 dB ... 0x12 = -10 dB */
	int ana_gain[AUDIO_ANALOG_VOLUME_TYPE_MAX];
	unsigned int mux_select[MUX_NUM];
	int dev_counter[DEVICE_NUM];
	int mtkaif_protocol;
};

static void mt6369_set_gpio_smt(struct mt6369_priv *priv)
{
	/* set gpio SMT mode */
	regmap_update_bits(priv->regmap, MT6369_SMT_CON2, 0xff, 0xfc);
	regmap_update_bits(priv->regmap, MT6369_SMT_CON3, 0xf, 0xf);
}

static void mt6369_set_gpio_driving(struct mt6369_priv *priv)
{
	/* 8:4mA(default), a:8mA, c:12mA, e:16mA */
	regmap_update_bits(priv->regmap, MT6369_DRV_CON3, 0xff, 0x88);
	regmap_update_bits(priv->regmap, MT6369_DRV_CON4, 0xff, 0x88);
	regmap_update_bits(priv->regmap, MT6369_DRV_CON5, 0xff, 0x88);
	regmap_update_bits(priv->regmap, MT6369_DRV_CON6, 0xff, 0x88);
}

void mt6369_set_mtkaif_protocol(struct snd_soc_component *cmpnt,
				int mtkaif_protocol)
{
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	priv->mtkaif_protocol = mtkaif_protocol;
}
EXPORT_SYMBOL_GPL(mt6369_set_mtkaif_protocol);

static void mt6369_set_playback_gpio(struct mt6369_priv *priv)
{
	unsigned int reg_value = 0;

	/* set gpio mosi mode, clk / data / sync */
	regmap_read(priv->regmap, MT6369_GPIO_MODE4, &reg_value);
	/* only reset the IO mode if necessary */
	if ((reg_value & 0x38) != 0x8) {
		regmap_write(priv->regmap, MT6369_GPIO_MODE4_CLR, 0x38);
		regmap_write(priv->regmap, MT6369_GPIO_MODE4_SET, 0x8);
	}

	regmap_write(priv->regmap, MT6369_GPIO_MODE5_CLR, 0x3f);
	regmap_write(priv->regmap, MT6369_GPIO_MODE5_SET, 0x9);
	regmap_write(priv->regmap, MT6369_GPIO_MODE6_CLR, 0x7);
	regmap_write(priv->regmap, MT6369_GPIO_MODE6_SET, 0x1);
}

static void mt6369_reset_playback_gpio(struct mt6369_priv *priv)
{
	/*
	 * set pad_aud_*_mosi to GPIO mode and dir input
	 * pad_aud_dat_mosi* is a boot strap pin; clk/sync are kept for
	 * mtkaif protocol 2
	 */
	regmap_write(priv->regmap, MT6369_GPIO_MODE5_CLR, 0x3f);
	regmap_update_bits(priv->regmap, MT6369_GPIO_DIR1, 0x3 << 2, 0x0);
}

static void mt6369_set_capture_gpio(struct mt6369_priv *priv)
{
	/* set gpio miso mode */
	regmap_write(priv->regmap, MT6369_GPIO_MODE7_CLR, 0x3f);
	regmap_write(priv->regmap, MT6369_GPIO_MODE7_SET, 0x09);
}

static void mt6369_reset_capture_gpio(struct mt6369_priv *priv)
{
	/*
	 * set pad_aud_*_miso to GPIO mode and dir input
	 * pad_aud_clk_miso carries 26 MHz even for playback only (leak),
	 * pad_aud_dat_miso* is a boot strap pin
	 */
	regmap_write(priv->regmap, MT6369_GPIO_MODE7_CLR, 0x3f);
	regmap_update_bits(priv->regmap, MT6369_GPIO_DIR1, 0x3 << 6, 0x0);
}

/* used only by the mtkaif calibration */
static void mt6369_set_clksq(struct mt6369_priv *priv, bool enable)
{
	/* Enable/disable CLKSQ 26MHz */
	regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON12,
			   RG_CLKSQ_EN_MASK_SFT,
			   (enable ? 1 : 0) << RG_CLKSQ_EN_SFT);
}

/* used only by the mtkaif calibration */
static void mt6369_set_aud_global_bias(struct mt6369_priv *priv, bool enable)
{
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON24,
			   RG_AUDGLB_PWRDN_VA28_MASK_SFT,
			   (enable ? 0 : 1) << RG_AUDGLB_PWRDN_VA28_SFT);
}

/* used only by the mtkaif calibration */
static void mt6369_set_topck(struct mt6369_priv *priv, bool enable)
{
	regmap_update_bits(priv->regmap, MT6369_AUD_TOP_CKPDN_CON0,
			   0x66, enable ? 0x0 : 0x66);
}

static void mt6369_set_decoder_clk(struct mt6369_priv *priv, bool enable)
{
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON24,
			   RG_RSTB_DECODER_VA28_MASK_SFT,
			   (enable ? 1 : 0) << RG_RSTB_DECODER_VA28_SFT);
}

/* other analog outputs that still need the DAC clock */
static bool mt6369_dl_other_users(struct mt6369_priv *priv, int device)
{
	int i;

	for (i = DEVICE_HP; i <= DEVICE_RCV; i++)
		if (i != device && priv->dev_counter[i] > 0)
			return true;

	return false;
}

static void mt6369_mtkaif_tx_enable(struct mt6369_priv *priv)
{
	switch (priv->mtkaif_protocol) {
	case MT6369_MTKAIF_PROTOCOL_2_CLK_P2:
		/* MTKAIF TX format setting */
		regmap_update_bits(priv->regmap, MT6369_AFE_ADDA_MTKAIF_CFG0,
				   0xff, 0x10);
		regmap_update_bits(priv->regmap, MT6369_AFE_ADDA_MTKAIF_CFG1,
				   0xff, 0x2);
		/* enable aud_pad TX fifos */
		regmap_update_bits(priv->regmap, MT6369_AFE_AUD_PAD_TOP,
				   0xff, 0x12);
		regmap_update_bits(priv->regmap, MT6369_AFE_AUD_PAD_TOP,
				   0xff, 0x13);
		break;
	case MT6369_MTKAIF_PROTOCOL_2:
		regmap_update_bits(priv->regmap, MT6369_AFE_ADDA_MTKAIF_CFG0,
				   0xff, 0x10);
		regmap_update_bits(priv->regmap, MT6369_AFE_ADDA_MTKAIF_CFG1,
				   0xff, 0x2);
		regmap_update_bits(priv->regmap, MT6369_AFE_AUD_PAD_TOP,
				   0xff, 0x11);
		break;
	case MT6369_MTKAIF_PROTOCOL_1:
	default:
		regmap_update_bits(priv->regmap, MT6369_AFE_ADDA_MTKAIF_CFG0,
				   0xff, 0x0);
		regmap_update_bits(priv->regmap, MT6369_AFE_ADDA_MTKAIF_CFG1,
				   0xff, 0x0);
		regmap_update_bits(priv->regmap, MT6369_AFE_AUD_PAD_TOP,
				   0xff, 0x11);
		break;
	}
}

static void mt6369_mtkaif_tx_disable(struct mt6369_priv *priv)
{
	/* disable aud_pad TX fifos */
	regmap_update_bits(priv->regmap, MT6369_AFE_AUD_PAD_TOP, 0xff, 0x10);
}

void mt6369_mtkaif_calibration_enable(struct snd_soc_component *cmpnt)
{
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	mt6369_set_playback_gpio(priv);
	mt6369_set_capture_gpio(priv);
	mt6369_mtkaif_tx_enable(priv);

	mt6369_set_aud_global_bias(priv, true);
	mt6369_set_clksq(priv, true);
	mt6369_set_topck(priv, true);

	/* set dat_miso_loopback on */
	regmap_update_bits(priv->regmap, MT6369_AUDIO_DIG_CFG0,
			   RG_AUD_PAD_TOP_DAT_MISO_LOOPBACK_MASK_SFT,
			   1 << RG_AUD_PAD_TOP_DAT_MISO_LOOPBACK_SFT);
	regmap_update_bits(priv->regmap, MT6369_AUDIO_DIG_CFG1,
			   RG_AUD_PAD_TOP_DAT_MISO2_LOOPBACK_MASK_SFT,
			   1 << RG_AUD_PAD_TOP_DAT_MISO2_LOOPBACK_SFT);
	regmap_update_bits(priv->regmap, MT6369_AUDIO_DIG_CFG2,
			   RG_AUD_PAD_TOP_DAT_MISO3_LOOPBACK_MASK_SFT,
			   1 << RG_AUD_PAD_TOP_DAT_MISO3_LOOPBACK_SFT);
}
EXPORT_SYMBOL_GPL(mt6369_mtkaif_calibration_enable);

void mt6369_mtkaif_calibration_disable(struct snd_soc_component *cmpnt)
{
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	/* set dat_miso_loopback off */
	regmap_update_bits(priv->regmap, MT6369_AUDIO_DIG_CFG0,
			   RG_AUD_PAD_TOP_DAT_MISO_LOOPBACK_MASK_SFT, 0);
	regmap_update_bits(priv->regmap, MT6369_AUDIO_DIG_CFG1,
			   RG_AUD_PAD_TOP_DAT_MISO2_LOOPBACK_MASK_SFT, 0);
	regmap_update_bits(priv->regmap, MT6369_AUDIO_DIG_CFG2,
			   RG_AUD_PAD_TOP_DAT_MISO3_LOOPBACK_MASK_SFT, 0);

	mt6369_set_topck(priv, false);
	mt6369_set_clksq(priv, false);
	mt6369_set_aud_global_bias(priv, false);

	mt6369_mtkaif_tx_disable(priv);
	mt6369_reset_playback_gpio(priv);
	mt6369_reset_capture_gpio(priv);
}
EXPORT_SYMBOL_GPL(mt6369_mtkaif_calibration_disable);

void mt6369_set_mtkaif_calibration_phase(struct snd_soc_component *cmpnt,
					 int phase_1, int phase_2, int phase_3)
{
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	regmap_update_bits(priv->regmap, MT6369_AUDIO_DIG_CFG0,
			   RG_AUD_PAD_TOP_PHASE_MODE_MASK_SFT,
			   phase_1 << RG_AUD_PAD_TOP_PHASE_MODE_SFT);
	regmap_update_bits(priv->regmap, MT6369_AUDIO_DIG_CFG1,
			   RG_AUD_PAD_TOP_PHASE_MODE2_MASK_SFT,
			   phase_2 << RG_AUD_PAD_TOP_PHASE_MODE2_SFT);
	regmap_update_bits(priv->regmap, MT6369_AUDIO_DIG_CFG2,
			   RG_AUD_PAD_TOP_PHASE_MODE3_MASK_SFT,
			   phase_3 << RG_AUD_PAD_TOP_PHASE_MODE3_SFT);
}
EXPORT_SYMBOL_GPL(mt6369_set_mtkaif_calibration_phase);

static void zcd_enable(struct mt6369_priv *priv, bool enable, int device)
{
	if (enable) {
		switch (device) {
		case DEVICE_RCV:
			regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON20,
					   RG_AUDZCDMUXSEL_VAUDP15_MASK_SFT,
					   0x2 << RG_AUDZCDMUXSEL_VAUDP15_SFT);
			break;
		case DEVICE_LO:
			regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON20,
					   RG_AUDZCDMUXSEL_VAUDP15_MASK_SFT,
					   0x0 << RG_AUDZCDMUXSEL_VAUDP15_SFT);
			break;
		case DEVICE_HP:
		default:
			regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON20,
					   RG_AUDZCDMUXSEL_VAUDP15_MASK_SFT,
					   0x1 << RG_AUDZCDMUXSEL_VAUDP15_SFT);
			break;
		}
		/* Enable ZCD, for minimize pop noise */
		/* timeout, 1 = 5ms, 0 = 30ms */
		regmap_update_bits(priv->regmap, MT6369_ZCD_CON0,
				   RG_AUDZCDTIMEOUTMODESEL_MASK_SFT,
				   0x0 << RG_AUDZCDTIMEOUTMODESEL_SFT);
		regmap_update_bits(priv->regmap, MT6369_ZCD_CON0,
				   RG_AUDZCDGAINSTEPSIZE_MASK_SFT,
				   0x0 << RG_AUDZCDGAINSTEPSIZE_SFT);
		regmap_update_bits(priv->regmap, MT6369_ZCD_CON0,
				   RG_AUDZCDGAINSTEPTIME_MASK_SFT,
				   0x5 << RG_AUDZCDGAINSTEPTIME_SFT);
		regmap_update_bits(priv->regmap, MT6369_ZCD_CON0,
				   RG_AUDZCDENABLE_MASK_SFT,
				   0x1 << RG_AUDZCDENABLE_SFT);
	} else {
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON20,
				   RG_AUDZCDMUXSEL_VAUDP15_MASK_SFT,
				   0x4 << RG_AUDZCDMUXSEL_VAUDP15_SFT);
		regmap_write(priv->regmap, MT6369_ZCD_CON0, 0x0);
	}
}

static void hp_main_output_ramp(struct mt6369_priv *priv, bool up)
{
	int i, stage;
	int target = 7;

	/* Enable/Reduce HPL/R main output stage step by step */
	for (i = 0; i <= target; i++) {
		stage = up ? i : target - i;
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON3,
				   RG_HPLOUTSTGCTRL_VAUDP15_MASK_SFT,
				   stage << RG_HPLOUTSTGCTRL_VAUDP15_SFT);
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON3,
				   RG_HPROUTSTGCTRL_VAUDP15_MASK_SFT,
				   stage << RG_HPROUTSTGCTRL_VAUDP15_SFT);
		usleep_range(100, 120);
	}
}

static void hp_aux_feedback_loop_gain_ramp(struct mt6369_priv *priv, bool up)
{
	int i, stage;
	int target = 0xf;

	/* Enable/Reduce HP aux feedback loop gain step by step */
	for (i = 0; i <= target; i++) {
		stage = up ? i : target - i;
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON17,
				   0xf << 4, stage << 4);
		usleep_range(100, 120);
	}
}

static void hp_pull_down(struct mt6369_priv *priv, bool enable)
{
	int i;

	if (enable) {
		for (i = 0x0; i <= 0x7; i++) {
			regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON8,
					   0x7, i);
			usleep_range(100, 150);
		}
	} else {
		for (i = 0x7; i >= 0x0; i--) {
			regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON8,
					   0x7, i);
			usleep_range(100, 150);
		}
	}
}

static bool is_valid_hp_pga_idx(int reg_idx)
{
	return (reg_idx >= DL_GAIN_8DB && reg_idx <= DL_GAIN_N_10DB) ||
	       reg_idx == DL_GAIN_N_40DB;
}

/* step one ZCD gain register from one index to another, 1 dB at a time */
static void mt6369_gain_ramp(struct mt6369_priv *priv, unsigned int reg,
			     int from, int to)
{
	int step;

	if (!is_valid_hp_pga_idx(from) || !is_valid_hp_pga_idx(to))
		return;

	/* -40 dB is next to -10 dB on the scale */
	if (from == DL_GAIN_N_40DB)
		from = DL_GAIN_N_10DB + 1;
	if (to == DL_GAIN_N_40DB)
		to = DL_GAIN_N_10DB + 1;

	step = to > from ? 1 : -1;
	while (from != to) {
		from += step;
		regmap_write(priv->regmap, reg,
			     from > DL_GAIN_N_10DB ? DL_GAIN_N_40DB : from);
		usleep_range(100, 150);
	}
}

static void headset_volume_ramp(struct mt6369_priv *priv, int from_l,
				int to_l, int from_r, int to_r)
{
	mt6369_gain_ramp(priv, MT6369_ZCD_CON3, from_l, to_l);
	mt6369_gain_ramp(priv, MT6369_ZCD_CON4, from_r, to_r);
}

/*
 * Volume controls. The value seen by userspace is 0 (-10 dB) ... 0x12
 * (+8 dB); the gain registers count the other way (0 = +8 dB). The gain is
 * kept in ana_gain[] and reaches the hardware only while the output is on:
 * the enable sequences start from -10 dB and ramp to it.
 */
static const struct {
	unsigned int reg;
	int gain[2];	/* ana_gain[] index of the left and right channel */
	int device;
} mt6369_dl_gains[] = {
	{ MT6369_ZCD_CON3,
	  { AUDIO_ANALOG_VOLUME_HPOUTL, AUDIO_ANALOG_VOLUME_HPOUTR }, DEVICE_HP },
	{ MT6369_ZCD_CON1,
	  { AUDIO_ANALOG_VOLUME_LINEOUTL, AUDIO_ANALOG_VOLUME_LINEOUTR }, DEVICE_LO },
	{ MT6369_ZCD_CON5,
	  { AUDIO_ANALOG_VOLUME_HSOUTL, AUDIO_ANALOG_VOLUME_HSOUTL }, DEVICE_RCV },
};

static int mt6369_dl_gain_index(struct soc_mixer_control *mc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mt6369_dl_gains); i++)
		if (mt6369_dl_gains[i].reg == mc->reg)
			return i;

	return -EINVAL;
}

static int mt6369_get_playback_volsw(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int i = mt6369_dl_gain_index(mc);

	if (i < 0)
		return i;

	ucontrol->value.integer.value[0] =
		DL_GAIN_N_10DB - priv->ana_gain[mt6369_dl_gains[i].gain[0]];
	if (snd_soc_volsw_is_stereo(mc))
		ucontrol->value.integer.value[1] =
			DL_GAIN_N_10DB - priv->ana_gain[mt6369_dl_gains[i].gain[1]];

	return 0;
}

static int mt6369_put_playback_volsw(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int i = mt6369_dl_gain_index(mc);
	int old[2], new[2];
	long val[2];

	if (i < 0)
		return i;

	val[0] = ucontrol->value.integer.value[0];
	val[1] = snd_soc_volsw_is_stereo(mc) ?
		 ucontrol->value.integer.value[1] : val[0];
	if (val[0] < 0 || val[0] > mc->max || val[1] < 0 || val[1] > mc->max)
		return -EINVAL;

	old[0] = priv->ana_gain[mt6369_dl_gains[i].gain[0]];
	old[1] = priv->ana_gain[mt6369_dl_gains[i].gain[1]];
	new[0] = DL_GAIN_N_10DB - val[0];
	new[1] = DL_GAIN_N_10DB - val[1];
	if (old[0] == new[0] && old[1] == new[1])
		return 0;

	priv->ana_gain[mt6369_dl_gains[i].gain[0]] = new[0];
	priv->ana_gain[mt6369_dl_gains[i].gain[1]] = new[1];

	if (priv->dev_counter[mt6369_dl_gains[i].device] <= 0)
		return 1;

	switch (mt6369_dl_gains[i].device) {
	case DEVICE_HP:
		headset_volume_ramp(priv, old[0], new[0], old[1], new[1]);
		break;
	case DEVICE_LO:
		/* the MediaTek downstream driver drives both line out gains from one value */
		regmap_write(priv->regmap, MT6369_ZCD_CON1, new[0]);
		regmap_write(priv->regmap, MT6369_ZCD_CON2, new[1]);
		break;
	case DEVICE_RCV:
		regmap_write(priv->regmap, MT6369_ZCD_CON5, new[0]);
		break;
	}

	return 1;
}

static const DECLARE_TLV_DB_SCALE(playback_tlv, -1000, 100, 0);
static const DECLARE_TLV_DB_SCALE(capture_tlv, 0, 600, 0);

static const struct snd_kcontrol_new mt6369_snd_controls[] = {
	/* dl pga gain */
	SOC_DOUBLE_EXT_TLV("Headset Volume",
			   MT6369_ZCD_CON3, 0, 8, DL_GAIN_N_10DB, 0,
			   mt6369_get_playback_volsw, mt6369_put_playback_volsw,
			   playback_tlv),
	SOC_SINGLE_EXT_TLV("Lineout Volume",
			   MT6369_ZCD_CON1, 0, DL_GAIN_N_10DB, 0,
			   mt6369_get_playback_volsw, mt6369_put_playback_volsw,
			   playback_tlv),
	SOC_SINGLE_EXT_TLV("Handset Volume",
			   MT6369_ZCD_CON5, 0, DL_GAIN_N_10DB, 0,
			   mt6369_get_playback_volsw, mt6369_put_playback_volsw,
			   playback_tlv),

	/* ul pga gain, 0 ... 24 dB */
	SOC_SINGLE_TLV("PGA1 Volume",
		       MT6369_AUDENC_ANA_CON1, RG_AUDPREAMPLGAIN_SFT, 4, 0,
		       capture_tlv),
	SOC_SINGLE_TLV("PGA2 Volume",
		       MT6369_AUDENC_ANA_CON3, RG_AUDPREAMPRGAIN_SFT, 4, 0,
		       capture_tlv),
};

/* LOL MUX */
static const char *const lo_in_mux_map[] = {
	"Open", "Playback",
};

static const unsigned int lo_in_mux_map_value[] = {
	LO_MUX_OPEN, LO_MUX_3RD_DAC,
};

static SOC_VALUE_ENUM_SINGLE_DECL(lo_in_mux_map_enum, SND_SOC_NOPM, 0,
				  LO_MUX_MASK, lo_in_mux_map,
				  lo_in_mux_map_value);

static const struct snd_kcontrol_new lo_in_mux_control =
	SOC_DAPM_ENUM("LO Select", lo_in_mux_map_enum);

/* HP MUX */
static const char *const hp_in_mux_map[] = {
	"Open", "Audio Playback",
};

static const unsigned int hp_in_mux_map_value[] = {
	HP_MUX_OPEN, HP_MUX_HP,
};

static SOC_VALUE_ENUM_SINGLE_DECL(hpl_in_mux_map_enum, SND_SOC_NOPM, 0,
				  HP_MUX_MASK, hp_in_mux_map,
				  hp_in_mux_map_value);

static const struct snd_kcontrol_new hpl_in_mux_control =
	SOC_DAPM_ENUM("HPL Select", hpl_in_mux_map_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(hpr_in_mux_map_enum, SND_SOC_NOPM, 0,
				  HP_MUX_MASK, hp_in_mux_map,
				  hp_in_mux_map_value);

static const struct snd_kcontrol_new hpr_in_mux_control =
	SOC_DAPM_ENUM("HPR Select", hpr_in_mux_map_enum);

/* RCV MUX */
static const char *const rcv_in_mux_map[] = {
	"Open", "Mute", "Voice Playback",
};

static const unsigned int rcv_in_mux_map_value[] = {
	RCV_MUX_OPEN, RCV_MUX_MUTE, RCV_MUX_VOICE_PLAYBACK,
};

static SOC_VALUE_ENUM_SINGLE_DECL(rcv_in_mux_map_enum, SND_SOC_NOPM, 0,
				  RCV_MUX_MASK, rcv_in_mux_map,
				  rcv_in_mux_map_value);

static const struct snd_kcontrol_new rcv_in_mux_control =
	SOC_DAPM_ENUM("RCV Select", rcv_in_mux_map_enum);

/* UL SRC MUX */
static const char *const ul_src_mux_map[] = {
	"AMIC", "DMIC",
};

static const unsigned int ul_src_mux_map_value[] = {
	UL_SRC_MUX_AMIC, UL_SRC_MUX_DMIC,
};

static SOC_VALUE_ENUM_SINGLE_DECL(ul_src_mux_map_enum, MT6369_AFE_UL_SRC_CON1,
				  UL_SDM_3_LEVEL_CTL_SFT,
				  UL_SDM_3_LEVEL_CTL_MASK, ul_src_mux_map,
				  ul_src_mux_map_value);

static const struct snd_kcontrol_new ul_src_mux_control =
	SOC_DAPM_ENUM("UL_SRC_MUX Select", ul_src_mux_map_enum);

/* MISO MUX */
static const char *const miso_mux_map[] = {
	"UL1_CH1", "UL1_CH2", "UL2_CH1", "UL2_CH2",
};

static const unsigned int miso_mux_map_value[] = {
	MISO_MUX_UL1_CH1, MISO_MUX_UL1_CH2, MISO_MUX_UL2_CH1, MISO_MUX_UL2_CH2,
};

static SOC_VALUE_ENUM_SINGLE_DECL(miso0_mux_map_enum, MT6369_AFE_MTKAIF_MUX_CFG0,
				  RG_ADDA_CH1_SEL_SFT, RG_ADDA_CH1_SEL_MASK,
				  miso_mux_map, miso_mux_map_value);

static const struct snd_kcontrol_new miso0_mux_control =
	SOC_DAPM_ENUM("MISO_MUX Select", miso0_mux_map_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(miso1_mux_map_enum, MT6369_AFE_MTKAIF_MUX_CFG0,
				  RG_ADDA_CH2_SEL_SFT, RG_ADDA_CH2_SEL_MASK,
				  miso_mux_map, miso_mux_map_value);

static const struct snd_kcontrol_new miso1_mux_control =
	SOC_DAPM_ENUM("MISO_MUX Select", miso1_mux_map_enum);

/* ADC L MUX */
static const char *const adc_left_mux_map[] = {
	"Idle", "AIN0", "Left Preamplifier", "Idle_1",
};

static const unsigned int adc_mux_map_value[] = {
	ADC_MUX_IDLE, ADC_MUX_AIN0, ADC_MUX_PREAMPLIFIER, ADC_MUX_IDLE1,
};

static SOC_VALUE_ENUM_SINGLE_DECL(adc_left_mux_map_enum, MT6369_AUDENC_ANA_CON1,
				  RG_AUDADCLINPUTSEL_SFT,
				  RG_AUDADCLINPUTSEL_MASK, adc_left_mux_map,
				  adc_mux_map_value);

static const struct snd_kcontrol_new adc_left_mux_control =
	SOC_DAPM_ENUM("ADC L Select", adc_left_mux_map_enum);

/* ADC R MUX */
static const char *const adc_right_mux_map[] = {
	"Idle", "AIN0", "Right Preamplifier", "Idle_1",
};

static SOC_VALUE_ENUM_SINGLE_DECL(adc_right_mux_map_enum, MT6369_AUDENC_ANA_CON3,
				  RG_AUDADCRINPUTSEL_SFT,
				  RG_AUDADCRINPUTSEL_MASK, adc_right_mux_map,
				  adc_mux_map_value);

static const struct snd_kcontrol_new adc_right_mux_control =
	SOC_DAPM_ENUM("ADC R Select", adc_right_mux_map_enum);

/* PGA L/R MUX */
static const char *const pga_mux_map[] = {
	"None", "AIN0", "AIN1", "AIN2",
};

static const unsigned int pga_mux_map_value[] = {
	PGA_MUX_NONE, PGA_MUX_AIN0, PGA_MUX_AIN1, PGA_MUX_AIN2,
};

static SOC_VALUE_ENUM_SINGLE_DECL(pga_left_mux_map_enum, MT6369_AUDENC_ANA_CON0,
				  RG_AUDPREAMPLINPUTSEL_SFT,
				  RG_AUDPREAMPLINPUTSEL_MASK, pga_mux_map,
				  pga_mux_map_value);

static const struct snd_kcontrol_new pga_left_mux_control =
	SOC_DAPM_ENUM("PGA L Select", pga_left_mux_map_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(pga_right_mux_map_enum, MT6369_AUDENC_ANA_CON2,
				  RG_AUDPREAMPRINPUTSEL_SFT,
				  RG_AUDPREAMPRINPUTSEL_MASK, pga_mux_map,
				  pga_mux_map_value);

static const struct snd_kcontrol_new pga_right_mux_control =
	SOC_DAPM_ENUM("PGA R Select", pga_right_mux_map_enum);

static void mt6369_dl_ldo_ncp_on(struct mt6369_priv *priv)
{
	/* Turn on DA_600K_NCP_VA18 */
	regmap_update_bits(priv->regmap, MT6369_AUDNCP_CLKDIV_CON1,
			   RG_DIVCKS_ON_MASK_SFT, 0x1 << RG_DIVCKS_ON_SFT);
	/* Set NCP clock as 604kHz // 26MHz/43 = 604KHz */
	regmap_write(priv->regmap, MT6369_AUDNCP_CLKDIV_CON2, 0x2c);
	/* Toggle RG_DIVCKS_CHG */
	regmap_update_bits(priv->regmap, MT6369_AUDNCP_CLKDIV_CON0,
			   RG_DIVCKS_CHG_MASK_SFT, 0x1 << RG_DIVCKS_CHG_SFT);
	/* Set NCP soft start mode as default mode: 100us */
	regmap_update_bits(priv->regmap, MT6369_AUDNCP_CLKDIV_CON4,
			   RG_DIVCKS_PWD_NCP_ST_SEL_MASK_SFT,
			   0x3 << RG_DIVCKS_PWD_NCP_ST_SEL_SFT);
	/* Enable NCP */
	regmap_update_bits(priv->regmap, MT6369_AUDNCP_CLKDIV_CON3,
			   RG_DIVCKS_PWD_NCP_MASK_SFT, 0x0 << RG_DIVCKS_PWD_NCP_SFT);
	usleep_range(250, 270);

	/* Enable cap-less LDOs (1.5V) */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON26,
			   RG_VA33REFGEN_EN_VA18_MASK_SFT,
			   0x1 << RG_VA33REFGEN_EN_VA18_SFT);
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON25, 0x55);
	/* Enable NV regulator (-1.2V) */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON27,
			   RG_NVREG_EN_VAUDP15_MASK_SFT,
			   0x1 << RG_NVREG_EN_VAUDP15_SFT);
	usleep_range(100, 120);
}

static void mt6369_set_dl_bias(struct mt6369_priv *priv)
{
	/* Set HP & ZCD bias current optimization */
	/* 01: ZCD: 4uA, HP/HS/LO: 5uA */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON23,
			   IBIAS_ZCD_MASK_SFT, IBIAS_ZCD_4UA << IBIAS_ZCD_SFT);
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON23,
			   IBIAS_HS_MASK_SFT, IBIAS_5UA << IBIAS_HS_SFT);
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON23,
			   IBIAS_HP_MASK_SFT, IBIAS_5UA << IBIAS_HP_SFT);
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON23,
			   IBIAS_LO_MASK_SFT, IBIAS_5UA << IBIAS_LO_SFT);
}

static void mtk_hp_enable(struct mt6369_priv *priv)
{
	/* Pull-down HPL/R to AVSS28_AUD */
	hp_pull_down(priv, true);
	/* release HP CMFB gate rstb */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON8,
			   0x1 << 6, 0x1 << 6);
	/* Reduce ESD resistance of AU_REFN */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON5,
			   RG_AUDREFN_DERES_EN_VAUDP15_MASK_SFT,
			   0x1 << RG_AUDREFN_DERES_EN_VAUDP15_SFT);
	/* Set HPL/HPR gain to -10dB */
	regmap_write(priv->regmap, MT6369_ZCD_CON3, DL_GAIN_N_10DB);
	regmap_write(priv->regmap, MT6369_ZCD_CON4, DL_GAIN_N_10DB);

	mt6369_dl_ldo_ncp_on(priv);

	/* Enable AUD_ZCD */
	zcd_enable(priv, true, DEVICE_HP);

	/* Disable headphone short-circuit protection */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON1, 0x30);

	/* Enable IBIST */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON22,
			   RG_AUDIBIASPWRDN_VAUDP15_MASK_SFT,
			   0x0 << RG_AUDIBIASPWRDN_VAUDP15_SFT);

	/* Set HP DR bias current optimization, 010: 6uA */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON21,
			   DRBIAS_HP_MASK_SFT, DRBIAS_6UA << DRBIAS_HP_SFT);
	mt6369_set_dl_bias(priv);

	/* Set HPP/N STB enhance circuits */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON4, 0x33);

	/* Enable HP aux output stage */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0xc);
	/* Enable HP aux feedback loop */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0x3c);
	/* Enable HP aux CMFB loop */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON17, 0x0c);
	/* Enable HP driver bias circuits */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON0, 0xc0);
	/* Enable HP driver core circuits */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON0, 0xf0);
	/* Short HP main output to HP aux output stage */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0xfc);
	/* Enable HP main CMFB loop */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON17, 0xe);
	/* Disable HP aux CMFB loop */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON17, 0x2);

	/* Select CMFB resistor bulk to AC mode */
	/* Selec HS/LO cap size (6.5pF default) */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON18, 0x0);
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON19, 0x0);

	/* Enable HP main output stage */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0xff);
	/* Enable HPR/L main output stage step by step */
	hp_main_output_ramp(priv, true);

	/* Reduce HP aux feedback loop gain */
	hp_aux_feedback_loop_gain_ramp(priv, true);
	/* Disable HP aux feedback loop */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0xcf);

	/* apply volume setting */
	headset_volume_ramp(priv,
			    DL_GAIN_N_10DB,
			    priv->ana_gain[AUDIO_ANALOG_VOLUME_HPOUTL],
			    DL_GAIN_N_10DB,
			    priv->ana_gain[AUDIO_ANALOG_VOLUME_HPOUTR]);

	/* Disable HP aux output stage */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0xc3);
	/* Unshort HP main output to HP aux output stage */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0x03);
	usleep_range(100, 120);

	/* Enable AUD_CLK */
	mt6369_set_decoder_clk(priv, true);
	/* Enable Audio DAC  */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON0, 0xf, 0xf);
	/* Enable low-noise mode of DAC */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON16, 0x1, 0x1);
	usleep_range(100, 120);

	/* Switch HPL MUX to audio DACL */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON1,
			   RG_AUDHPLMUXINPUTSEL_VAUDP15_MASK_SFT,
			   2 << RG_AUDHPLMUXINPUTSEL_VAUDP15_SFT);
	/* Switch HPR MUX to audio DACR */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON1,
			   RG_AUDHPRMUXINPUTSEL_VAUDP15_MASK_SFT,
			   2 << RG_AUDHPRMUXINPUTSEL_VAUDP15_SFT);

	/* Disable Pull-down HPL/R to AVSS28_AUD */
	hp_pull_down(priv, false);
}

static void mtk_hp_disable(struct mt6369_priv *priv)
{
	/* Pull-down HPL/R to AVSS28_AUD */
	hp_pull_down(priv, true);

	/* HPR/HPL mux to open */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON1, 0x0f, 0x0);

	/* Disable low-noise mode of DAC */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON16, 0x1, 0x0);

	/* Disable Audio DAC and AUD_CLK unless RCV/LO still use them */
	if (!mt6369_dl_other_users(priv, DEVICE_HP)) {
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON0, 0xf, 0x0);
		mt6369_set_decoder_clk(priv, false);
	}

	/* Short HP main output to HP aux output stage */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0xc3);
	/* Enable HP aux output stage */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0xcf);

	/* decrease HPL/R gain to normal gain step by step */
	headset_volume_ramp(priv,
			    priv->ana_gain[AUDIO_ANALOG_VOLUME_HPOUTL],
			    DL_GAIN_N_10DB,
			    priv->ana_gain[AUDIO_ANALOG_VOLUME_HPOUTR],
			    DL_GAIN_N_10DB);

	/* set HP aux feedback loop gain to max */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON17, 0xf2);

	/* Enable HP aux feedback loop */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0xff);

	/* Reduce HP aux feedback loop gain */
	hp_aux_feedback_loop_gain_ramp(priv, false);

	/* decrease HPR/L main output stage step by step */
	hp_main_output_ramp(priv, false);

	/* Disable HP main output stage */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0x3, 0x0);

	/* Enable HP aux CMFB loop */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON17, 0xe);
	/* Disable HP main CMFB loop */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON17, 0xc);

	/* Unshort HP main output to HP aux output stage */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0x3 << 6, 0x0);

	/* Disable HP driver core circuits */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON0, 0x3 << 4, 0x0);

	/* Disable HP driver bias circuits */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON0, 0x3 << 6, 0x0);

	/* Disable HP aux CMFB loop */
	regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON17, 0x0);

	/* Open HP aux feedback loop */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0x3 << 4, 0x0);

	/* Disable HP aux output stage */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON2, 0x3 << 2, 0x0);

	/* Disable AUD_ZCD */
	zcd_enable(priv, false, DEVICE_HP);

	/* Set HPL/HPR gain to mute */
	regmap_write(priv->regmap, MT6369_ZCD_CON3, DL_GAIN_N_40DB);
	regmap_write(priv->regmap, MT6369_ZCD_CON4, DL_GAIN_N_40DB);

	/* Increase ESD resistance of AU_REFN */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON5,
			   RG_AUDREFN_DERES_EN_VAUDP15_MASK_SFT,
			   0x0 << RG_AUDREFN_DERES_EN_VAUDP15_SFT);

	/* Set HP CMFB gate rstb */
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON8,
			   0x1 << 6, 0x0 << 6);
	/* disable Pull-down HPL/R to AVSS28_AUD */
	hp_pull_down(priv, false);
}

static int mt_hp_event(struct snd_soc_dapm_widget *w,
		       struct snd_kcontrol *kcontrol,
		       int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	unsigned int mux = snd_soc_dapm_kcontrol_get_value(w->kcontrols[0]);
	int device = DEVICE_HP;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* HPL and HPR share the sequence */
		if (++priv->dev_counter[device] > 1)
			break;

		priv->mux_select[MUX_HP_L] = mux;
		if (mux == HP_MUX_HP)
			mtk_hp_enable(priv);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (--priv->dev_counter[device] > 0)
			break;
		if (priv->dev_counter[device] < 0) {
			dev_warn(priv->dev, "%s(), dev_counter[DEV_HP] %d < 0\n",
				 __func__, priv->dev_counter[device]);
			priv->dev_counter[device] = 0;
			break;
		}

		if (priv->mux_select[MUX_HP_L] == HP_MUX_HP)
			mtk_hp_disable(priv);
		priv->mux_select[MUX_HP_L] = mux;
		break;
	default:
		break;
	}

	return 0;
}

static int mt_rcv_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol,
			int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Reduce ESD resistance of AU_REFN */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON5,
				   RG_AUDREFN_DERES_EN_VAUDP15_MASK_SFT,
				   0x1 << RG_AUDREFN_DERES_EN_VAUDP15_SFT);

		mt6369_dl_ldo_ncp_on(priv);

		/* Disable handset short-circuit protection */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON10,
				   RG_AUDHSSCDISABLE_VAUDP15_MASK_SFT,
				   0x1 << RG_AUDHSSCDISABLE_VAUDP15_SFT);

		/* Enable IBIST */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON22,
				   RG_AUDIBIASPWRDN_VAUDP15_MASK_SFT,
				   0x0 << RG_AUDIBIASPWRDN_VAUDP15_SFT);
		/* Set HS DR bias current optimization, 010: 6uA */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON21,
				   DRBIAS_HS_MASK_SFT, DRBIAS_6UA << DRBIAS_HS_SFT);
		mt6369_set_dl_bias(priv);

		/* Set HS STB enhance circuits */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON10, 0x90);

		/* Disable HP main CMFB loop */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON17, 0x0);
		/* Select CMFB resistor bulk to AC mode */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON18, 0x0);
		/* Selec HS/LO cap size (6.5pF default) */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON19, 0x0);

		/* Enable HS driver bias circuits */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON10, 0x92);
		/* Enable HS driver core circuits */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON10, 0x93);

		/* Set HS gain to normal gain */
		regmap_write(priv->regmap, MT6369_ZCD_CON5,
			     priv->ana_gain[AUDIO_ANALOG_VOLUME_HSOUTL]);

		/* Enable AUD_CLK */
		mt6369_set_decoder_clk(priv, true);

		/* Enable Audio DAC L */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON0,
				   RG_AUDDACLPWRUP_VAUDP15_MASK_SFT |
				   RG_AUD_DAC_PWL_UP_VA28_MASK_SFT, 0x9);
		/* Enable low-noise mode of DAC */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON16, 0x1);
		/* Switch HS MUX to audio DAC L */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON10,
				   RG_AUDHSMUXINPUTSEL_VAUDP15_MASK_SFT,
				   0x2 << RG_AUDHSMUXINPUTSEL_VAUDP15_SFT);
		/* Enable AUD_ZCD */
		zcd_enable(priv, true, DEVICE_RCV);
		priv->dev_counter[DEVICE_RCV] = 1;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		priv->dev_counter[DEVICE_RCV] = 0;
		/* HS mux to open */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON10,
				   RG_AUDHSMUXINPUTSEL_VAUDP15_MASK_SFT,
				   RCV_MUX_OPEN << RG_AUDHSMUXINPUTSEL_VAUDP15_SFT);

		/* Disable Audio DAC L and AUD_CLK unless HP/LO use them */
		if (!mt6369_dl_other_users(priv, DEVICE_RCV)) {
			regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON0,
					   RG_AUDDACLPWRUP_VAUDP15_MASK_SFT |
					   RG_AUD_DAC_PWL_UP_VA28_MASK_SFT, 0x0);
			mt6369_set_decoder_clk(priv, false);
		}

		/* decrease HS gain to minimum gain */
		regmap_write(priv->regmap, MT6369_ZCD_CON5, DL_GAIN_N_40DB);

		/* Disable HS driver core circuits */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON10,
				   RG_AUDHSPWRUP_VAUDP15_MASK_SFT, 0x0);

		/* Disable HS driver bias circuits */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON10,
				   RG_AUDHSPWRUP_IBIAS_VAUDP15_MASK_SFT, 0x0);
		/* Disable AUD_ZCD */
		zcd_enable(priv, false, DEVICE_RCV);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_lo_event(struct snd_soc_dapm_widget *w,
		       struct snd_kcontrol *kcontrol,
		       int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Reduce ESD resistance of AU_REFN */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON5,
				   RG_AUDREFN_DERES_EN_VAUDP15_MASK_SFT,
				   0x1 << RG_AUDREFN_DERES_EN_VAUDP15_SFT);

		mt6369_dl_ldo_ncp_on(priv);

		/* Disable LO short-circuit protection */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON12,
				   RG_AUDLOLSCDISABLE_VAUDP15_MASK_SFT,
				   0x1 << RG_AUDLOLSCDISABLE_VAUDP15_SFT);

		/* Set LO DR bias current optimization, 010: 6uA */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON22,
				   DRBIAS_LO_MASK_SFT, DRBIAS_6UA << DRBIAS_LO_SFT);
		/* Set LO & ZCD bias current optimization */
		/* 01: ZCD: 4uA, HP/HS/LO: 5uA */
		if (priv->dev_counter[DEVICE_HP] == 0)
			regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON23,
					   IBIAS_ZCD_MASK_SFT,
					   IBIAS_ZCD_4UA << IBIAS_ZCD_SFT);

		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON23,
				   IBIAS_LO_MASK_SFT, IBIAS_5UA << IBIAS_LO_SFT);

		/* Set LO STB enhance circuits */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON13, 0x1);

		/* Disable HP main CMFB loop */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON17, 0x0);
		/* Select CMFB resistor bulk to AC mode */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON18, 0x0);
		/* Selec HS/LO cap size (6.5pF default) */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON19, 0x0);

		/* Enable LO driver bias circuits */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON12, 0x12);
		/* Enable LO driver core circuits */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON12, 0x13);

		/* Set LO gain to normal gain */
		regmap_write(priv->regmap, MT6369_ZCD_CON1,
			     priv->ana_gain[AUDIO_ANALOG_VOLUME_LINEOUTL]);
		regmap_write(priv->regmap, MT6369_ZCD_CON2,
			     priv->ana_gain[AUDIO_ANALOG_VOLUME_LINEOUTR]);

		/* Enable AUD_CLK */
		mt6369_set_decoder_clk(priv, true);

		/* Enable Audio DAC R */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON0,
				   RG_AUDDACRPWRUP_VAUDP15_MASK_SFT |
				   RG_AUD_DAC_PWR_UP_VA28_MASK_SFT, 0x6);
		/* Enable low-noise mode of DAC */
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON16, 0x1);
		/* Switch LOL MUX to DACR */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON12,
				   RG_AUDLOLMUXINPUTSEL_VAUDP15_MASK_SFT,
				   0x1 << RG_AUDLOLMUXINPUTSEL_VAUDP15_SFT);
		/* Enable AUD_ZCD */
		zcd_enable(priv, true, DEVICE_LO);
		priv->dev_counter[DEVICE_LO] = 1;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		priv->dev_counter[DEVICE_LO] = 0;
		/* Switch LOL MUX to open */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON12,
				   RG_AUDLOLMUXINPUTSEL_VAUDP15_MASK_SFT,
				   LO_MUX_OPEN << RG_AUDLOLMUXINPUTSEL_VAUDP15_SFT);

		/* Disable Audio DAC R and AUD_CLK unless HP/RCV use them */
		if (!mt6369_dl_other_users(priv, DEVICE_LO)) {
			regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON0,
					   RG_AUDDACRPWRUP_VAUDP15_MASK_SFT |
					   RG_AUD_DAC_PWR_UP_VA28_MASK_SFT, 0x0);
			mt6369_set_decoder_clk(priv, false);
		}

		/* decrease LO gain to minimum gain */
		regmap_write(priv->regmap, MT6369_ZCD_CON1, DL_GAIN_N_40DB);
		regmap_write(priv->regmap, MT6369_ZCD_CON2, DL_GAIN_N_40DB);

		/* Disable LO driver core circuits */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON12,
				   RG_AUDLOLPWRUP_VAUDP15_MASK_SFT, 0x0);

		/* Disable LO driver bias circuits */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON12,
				   RG_AUDLOLPWRUP_IBIAS_VAUDP15_MASK_SFT, 0x0);
		/* Disable AUD_ZCD */
		zcd_enable(priv, false, DEVICE_LO);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_adc_clk_gen_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol,
				int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* Enable audio ADC CLKGEN */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON24,
				   RG_RSTB_ENCODER_VA28_MASK_SFT,
				   0x1 << RG_RSTB_ENCODER_VA28_SFT);
		/* ADC CLK from CLKGEN (13MHz) */
		regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON6,
				   RG_AUDADCCLKSOURCE_MASK_SFT,
				   0x0 << RG_AUDADCCLKSOURCE_SFT);
		regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON6,
				   RG_AUDADCCLKSEL_MASK_SFT,
				   0x0 << RG_AUDADCCLKSEL_SFT);

		/* Enable LCLDO_ENC 1P8V */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON26,
				   RG_LCLDO_ENC_EN_VA28_MASK_SFT,
				   0x1 << RG_LCLDO_ENC_EN_VA28_SFT);
		/* LCLDO_ENC remote sense */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON26,
				   RG_LCLDO_ENC_REMOTE_SENSE_VA28_MASK_SFT,
				   0x1 << RG_LCLDO_ENC_REMOTE_SENSE_VA28_SFT);
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON26,
				   RG_VA28REFGEN_EN_VA28_MASK_SFT,
				   0x1 << RG_VA28REFGEN_EN_VA28_SFT);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		/* LCLDO_ENC remote sense off */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON26,
				   RG_LCLDO_ENC_REMOTE_SENSE_VA28_MASK_SFT,
				   0x0 << RG_LCLDO_ENC_REMOTE_SENSE_VA28_SFT);
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON26,
				   RG_VA28REFGEN_EN_VA28_MASK_SFT,
				   0x0 << RG_VA28REFGEN_EN_VA28_SFT);
		/* disable LCLDO_ENC 1P8V */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON26,
				   RG_LCLDO_ENC_EN_VA28_MASK_SFT,
				   0x0 << RG_LCLDO_ENC_EN_VA28_SFT);
		regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON6,
				   RG_AUDADCCLKSOURCE_MASK_SFT,
				   0x0 << RG_AUDADCCLKSOURCE_SFT);
		regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON6,
				   RG_AUDADCCLKSEL_MASK_SFT,
				   0x0 << RG_AUDADCCLKSEL_SFT);
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON24,
				   RG_RSTB_ENCODER_VA28_MASK_SFT,
				   0x0 << RG_RSTB_ENCODER_VA28_SFT);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_dcc_clk_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol,
			    int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* DCC 50k CLK (from 26M) */
		regmap_update_bits(priv->regmap, MT6369_AFE_DCCLK_CFG1, 0xff, 0x3);
		regmap_update_bits(priv->regmap, MT6369_AFE_DCCLK_CFG0, 0xf7, 0x22);
		regmap_update_bits(priv->regmap, MT6369_AFE_DCCLK_CFG0, 0xf7, 0x20);
		regmap_update_bits(priv->regmap, MT6369_AFE_DCCLK_CFG0, 0xf7, 0x21);
		regmap_write(priv->regmap, MT6369_AFE_DCCLK_CFG2, 0x10);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(priv->regmap, MT6369_AFE_DCCLK_CFG1, 0xff, 0x3);
		regmap_update_bits(priv->regmap, MT6369_AFE_DCCLK_CFG0, 0xf7, 0x20);
		regmap_update_bits(priv->regmap, MT6369_AFE_DCCLK_CFG0, 0xf7, 0x22);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_mic_bias_0_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol,
			       int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	unsigned int mic_type = priv->mux_select[MUX_MIC_TYPE_0];

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		switch (mic_type) {
		case MIC_TYPE_MUX_DCC_ECM_DIFF:
			regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON18,
					   0xff, 0x77);
			break;
		case MIC_TYPE_MUX_DCC_ECM_SINGLE:
			regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON18,
					   0xff, 0x11);
			break;
		default:
			regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON18,
					   0xff, 0x00);
			break;
		}

		/* MISBIAS0 = 1P9V */
		regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON17,
				   RG_AUDMICBIAS0VREF_MASK_SFT,
				   MIC_BIAS_1P9 << RG_AUDMICBIAS0VREF_SFT);
		/* normal (not low) power */
		regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON17,
				   RG_AUDMICBIAS0LOWPEN_MASK_SFT, 0);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* Disable MICBIAS0, MISBIAS0 = 1P7V */
		regmap_write(priv->regmap, MT6369_AUDENC_ANA_CON17, 0x0);
		regmap_write(priv->regmap, MT6369_AUDENC_ANA_CON18, 0x0);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_mic_bias_1_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol,
			       int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	unsigned int mic_type = priv->mux_select[MUX_MIC_TYPE_1];

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* MISBIAS1 = 2P6V */
		if (mic_type == MIC_TYPE_MUX_DCC_ECM_SINGLE)
			regmap_write(priv->regmap, MT6369_AUDENC_ANA_CON20, 0x1);
		else
			regmap_write(priv->regmap, MT6369_AUDENC_ANA_CON20, 0x0);
		/* normal (not low) power */
		regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON19,
				   RG_AUDMICBIAS1LOWPEN_MASK_SFT, 0);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_mtkaif_tx_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol,
			      int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mt6369_mtkaif_tx_enable(priv);
		break;
	case SND_SOC_DAPM_POST_PMD:
		mt6369_mtkaif_tx_disable(priv);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_adc_event(struct snd_soc_dapm_widget *w, unsigned int rc_reg,
			unsigned int adc_reg, unsigned int adc_mask,
			unsigned int pga_reg, unsigned int precharge_mask)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	unsigned int rc_tune = 0;

	usleep_range(500, 520);

	/* adc reset mechanism: restart the ADC if the RC tune got stuck */
	regmap_read(priv->regmap, rc_reg, &rc_tune);
	rc_tune &= 0x1f;
	if (rc_tune == 0x0 || rc_tune == 0x1f) {
		dev_dbg(priv->dev, "%s(), rc_tune 0x%x, reset adc\n",
			w->name, rc_tune);
		regmap_update_bits(priv->regmap, adc_reg, adc_mask, 0);
		regmap_update_bits(priv->regmap, adc_reg, adc_mask, adc_mask);
	}
	usleep_range(500, 520);

	/* preamplifier DCC precharge off */
	regmap_update_bits(priv->regmap, pga_reg, precharge_mask, 0x0);

	return 0;
}

static int mt_adc_l_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol,
			  int event)
{
	if (event != SND_SOC_DAPM_POST_PMU)
		return 0;

	return mt_adc_event(w, MT6369_AUDENC_ANA_CON10, MT6369_AUDENC_ANA_CON1,
			    RG_AUDADCLPWRUP_MASK_SFT, MT6369_AUDENC_ANA_CON0,
			    RG_AUDPREAMPLDCPRECHARGE_MASK_SFT);
}

static int mt_adc_r_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol,
			  int event)
{
	if (event != SND_SOC_DAPM_POST_PMU)
		return 0;

	return mt_adc_event(w, MT6369_AUDENC_ANA_CON11, MT6369_AUDENC_ANA_CON3,
			    RG_AUDADCRPWRUP_MASK_SFT, MT6369_AUDENC_ANA_CON2,
			    RG_AUDPREAMPRDCPRECHARGE_MASK_SFT);
}

static int mt_pga_l_mux_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol,
			      int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	unsigned int mux = snd_soc_dapm_kcontrol_get_value(w->kcontrols[0]);

	priv->mux_select[MUX_PGA_L] = mux >> RG_AUDPREAMPLINPUTSEL_SFT;
	return 0;
}

static int mt_pga_r_mux_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol,
			      int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	unsigned int mux = snd_soc_dapm_kcontrol_get_value(w->kcontrols[0]);

	priv->mux_select[MUX_PGA_R] = mux >> RG_AUDPREAMPRINPUTSEL_SFT;
	return 0;
}

static int mt6369_pga_mic_type(struct mt6369_priv *priv, unsigned int mux)
{
	switch (mux) {
	case PGA_MUX_AIN0:
		return priv->mux_select[MUX_MIC_TYPE_0];
	case PGA_MUX_AIN1:
		return priv->mux_select[MUX_MIC_TYPE_1];
	case PGA_MUX_AIN2:
		return priv->mux_select[MUX_MIC_TYPE_2];
	default:
		dev_err(priv->dev, "%s(), invalid pga mux %d\n", __func__, mux);
		return -EINVAL;
	}
}

static int mt_pga_l_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol,
			  int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	int mic_type = mt6369_pga_mic_type(priv, priv->mux_select[MUX_PGA_L]);

	if (mic_type < 0)
		return mic_type;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (IS_DCC_BASE(mic_type)) {
			/* Audio L preamplifier DCC precharge */
			regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON0,
					   RG_AUDPREAMPLDCPRECHARGE_MASK_SFT,
					   0x1 << RG_AUDPREAMPLDCPRECHARGE_SFT);
		}
		break;
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(1000, 1050);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_pga_r_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol,
			  int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	int mic_type = mt6369_pga_mic_type(priv, priv->mux_select[MUX_PGA_R]);

	if (mic_type < 0)
		return mic_type;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (IS_DCC_BASE(mic_type)) {
			/* Audio R preamplifier DCC precharge */
			regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON2,
					   RG_AUDPREAMPRDCPRECHARGE_MASK_SFT,
					   0x1 << RG_AUDPREAMPRDCPRECHARGE_SFT);
		}
		break;
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(1000, 1050);
		break;
	default:
		break;
	}

	return 0;
}

/* It is based on hw's control sequenece to add some delay when PMU/PMD */
static int mt_delay_250_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol,
			      int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
	case SND_SOC_DAPM_PRE_PMD:
		usleep_range(250, 270);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_delay_100_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol,
			      int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
	case SND_SOC_DAPM_PRE_PMD:
		usleep_range(100, 120);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_hp_mute_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol,
			    int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Set HPR/HPL gain to -10dB */
		regmap_write(priv->regmap, MT6369_ZCD_CON3, DL_GAIN_N_10DB);
		regmap_write(priv->regmap, MT6369_ZCD_CON4, DL_GAIN_N_10DB);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* Set HPL/HPR gain to mute */
		regmap_write(priv->regmap, MT6369_ZCD_CON3, DL_GAIN_N_40DB);
		regmap_write(priv->regmap, MT6369_ZCD_CON4, DL_GAIN_N_40DB);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_esd_resist_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol,
			       int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Reduce ESD resistance of AU_REFN */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON5,
				   RG_AUDREFN_DERES_EN_VAUDP15_MASK_SFT,
				   0x1 << RG_AUDREFN_DERES_EN_VAUDP15_SFT);
		usleep_range(250, 270);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* Increase ESD resistance of AU_REFN */
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON5,
				   RG_AUDREFN_DERES_EN_VAUDP15_MASK_SFT, 0x0);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_sdm_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol,
			int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* sdm audio fifo clock power on */
		regmap_update_bits(priv->regmap, MT6369_AFUNC_AUD_CON4, 0xfd, 0x6);
		/* scrambler clock on enable */
		regmap_write(priv->regmap, MT6369_AFUNC_AUD_CON1, 0xcb);
		regmap_write(priv->regmap, MT6369_AFUNC_AUD_CON0, 0xa1);
		/* sdm power on */
		regmap_update_bits(priv->regmap, MT6369_AFUNC_AUD_CON4, 0xfd, 0x3);
		/* sdm fifo enable */
		regmap_update_bits(priv->regmap, MT6369_AFUNC_AUD_CON4, 0xfd, 0xb);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* DL scrambler disabling sequence */
		regmap_update_bits(priv->regmap, MT6369_AFUNC_AUD_CON4, 0xfd, 0x0);
		regmap_write(priv->regmap, MT6369_AFUNC_AUD_CON1, 0xcb);
		regmap_write(priv->regmap, MT6369_AFUNC_AUD_CON0, 0xa0);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_ncp_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol,
			int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Turn on DA_600K_NCP_VA18 */
		regmap_update_bits(priv->regmap, MT6369_AUDNCP_CLKDIV_CON1,
				   RG_DIVCKS_ON_MASK_SFT, 0x1 << RG_DIVCKS_ON_SFT);
		/* Set NCP clock as 604kHz // 26MHz/43 = 604KHz */
		regmap_write(priv->regmap, MT6369_AUDNCP_CLKDIV_CON2, 0x2c);
		/* Toggle RG_DIVCKS_CHG */
		regmap_update_bits(priv->regmap, MT6369_AUDNCP_CLKDIV_CON0,
				   RG_DIVCKS_CHG_MASK_SFT, 0x1 << RG_DIVCKS_CHG_SFT);
		/* Set NCP soft start mode as default mode: 100us */
		regmap_update_bits(priv->regmap, MT6369_AUDNCP_CLKDIV_CON4,
				   RG_DIVCKS_PWD_NCP_ST_SEL_MASK_SFT,
				   0x3 << RG_DIVCKS_PWD_NCP_ST_SEL_SFT);
		/* Enable NCP */
		regmap_update_bits(priv->regmap, MT6369_AUDNCP_CLKDIV_CON3,
				   RG_DIVCKS_PWD_NCP_MASK_SFT,
				   0x0 << RG_DIVCKS_PWD_NCP_SFT);
		usleep_range(250, 270);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* Disable NCP */
		regmap_update_bits(priv->regmap, MT6369_AUDNCP_CLKDIV_CON3,
				   RG_DIVCKS_PWD_NCP_MASK_SFT,
				   0x1 << RG_DIVCKS_PWD_NCP_SFT);
		/* Turn off DA_600K_NCP_VA18 */
		regmap_update_bits(priv->regmap, MT6369_AUDNCP_CLKDIV_CON1,
				   RG_DIVCKS_ON_MASK_SFT, 0x0 << RG_DIVCKS_ON_SFT);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_dl_gpio_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol,
			    int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mt6369_set_playback_gpio(priv);
		break;
	case SND_SOC_DAPM_POST_PMD:
		mt6369_reset_playback_gpio(priv);
		break;
	default:
		break;
	}

	return 0;
}

static int mt_ul_gpio_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol,
			    int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mt6369_set_capture_gpio(priv);
		break;
	case SND_SOC_DAPM_POST_PMD:
		mt6369_reset_capture_gpio(priv);
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget mt6369_dapm_widgets[] = {
	/* Global Supply */
	SND_SOC_DAPM_SUPPLY_S("AUDGLB", SUPPLY_SEQ_AUD_GLB,
			      MT6369_AUDDEC_ANA_CON24,
			      RG_AUDGLB_PWRDN_VA28_SFT, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("CLKSQ Audio", SUPPLY_SEQ_CLKSQ,
			      MT6369_AUDENC_ANA_CON12,
			      RG_CLKSQ_EN_SFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("AUDNCP_CK", SUPPLY_SEQ_TOP_CK,
			      MT6369_AUD_TOP_CKPDN_CON0,
			      RG_AUDNCP_CK_PDN_SFT, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("ZCD13M_CK", SUPPLY_SEQ_TOP_CK,
			      MT6369_AUD_TOP_CKPDN_CON0,
			      RG_ZCD13M_CK_PDN_SFT, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("AUD_CK", SUPPLY_SEQ_TOP_CK_LAST,
			      MT6369_AUD_TOP_CKPDN_CON0,
			      RG_AUD_CK_PDN_SFT, 1, mt_delay_250_event,
			      SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY_S("AUDIF_CK", SUPPLY_SEQ_TOP_CK,
			      MT6369_AUD_TOP_CKPDN_CON0,
			      RG_AUDIF_CK_PDN_SFT, 1, NULL, 0),

	/* Digital Clock */
	SND_SOC_DAPM_SUPPLY_S("AUDIO_TOP_AFE_CTL", SUPPLY_SEQ_AUD_TOP_LAST,
			      MT6369_AUDIO_TOP_CON0,
			      PDN_AFE_CTL_SFT, 1,
			      mt_delay_250_event,
			      SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY_S("AUDIO_TOP_DAC_CTL", SUPPLY_SEQ_AUD_TOP,
			      MT6369_AUDIO_TOP_CON0,
			      PDN_DAC_CTL_SFT, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("AUDIO_TOP_ADC_CTL", SUPPLY_SEQ_AUD_TOP,
			      MT6369_AUDIO_TOP_CON0,
			      PDN_ADC_CTL_SFT, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("AUDIO_TOP_ADDA6_ADC_CTL", SUPPLY_SEQ_AUD_TOP,
			      MT6369_AUDIO_TOP_CON0,
			      PDN_ADDA6_ADC_CTL_SFT, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("AUDIO_TOP_PWR_CLK", SUPPLY_SEQ_AUD_TOP,
			      MT6369_AUDIO_TOP_CON0,
			      PWR_CLK_DIS_CTL_SFT, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("AUDIO_TOP_PDN_RESERVED", SUPPLY_SEQ_AUD_TOP,
			      MT6369_AUDIO_TOP_CON0,
			      PDN_AFE_DL_PREDIST_CTL_SFT, 1, NULL, 0),

	SND_SOC_DAPM_SUPPLY_S("SDM", SUPPLY_SEQ_DL_SDM,
			      SND_SOC_NOPM, 0, 0,
			      mt_sdm_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("SDM_FIFO_CLK", SUPPLY_SEQ_DL_SDM_FIFO_CLK,
			      MT6369_AFUNC_AUD_CON4,
			      CCI_AFIFO_CLK_PWDB_SFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("NCP", SUPPLY_SEQ_DL_NCP,
			      SND_SOC_NOPM, 0, 0,
			      mt_ncp_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("DL Digital Clock", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DL Digital Clock CH_1_2", SND_SOC_NOPM,
			    0, 0, NULL, 0),

	/* AFE ON */
	SND_SOC_DAPM_SUPPLY_S("AFE_ON", SUPPLY_SEQ_AFE,
			      MT6369_AFE_UL_DL_CON0, AFE_ON_SFT, 0, NULL, 0),

	/* GPIO */
	SND_SOC_DAPM_SUPPLY_S("DL_GPIO", SUPPLY_SEQ_DL_GPIO,
			      SND_SOC_NOPM, 0, 0,
			      mt_dl_gpio_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("UL_GPIO", SUPPLY_SEQ_UL_GPIO,
			      SND_SOC_NOPM, 0, 0,
			      mt_ul_gpio_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* AIF Rx */
	SND_SOC_DAPM_AIF_IN("AIF_RX", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SUPPLY_S("AFE_DL_SRC", SUPPLY_SEQ_DL_SRC,
			      MT6369_AFE_DL_SRC2_CON0,
			      DL_2_SRC_ON_TMP_CTL_PRE_SFT, 0, NULL, 0),

	/* DL Supply */
	SND_SOC_DAPM_SUPPLY("DL Power Supply", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("ESD_RESIST", SUPPLY_SEQ_DL_ESD_RESIST,
			      SND_SOC_NOPM, 0, 0,
			      mt_esd_resist_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("LDO_VA33REFGEN", SUPPLY_SEQ_DL_LDO_VA33REFGEN,
			      MT6369_AUDDEC_ANA_CON26,
			      RG_VA33REFGEN_EN_VA18_SFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("LCLDO", SUPPLY_SEQ_DL_LCLDO,
			      MT6369_AUDDEC_ANA_CON25,
			      RG_LCLDO_EN_VA18_SFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("LCLDO_REMOTE", SUPPLY_SEQ_DL_LCLDO_REMOTE_SENSE,
			      MT6369_AUDDEC_ANA_CON25,
			      RG_LCLDO_REMOTE_SENSE_VA18_SFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("HCLDO", SUPPLY_SEQ_DL_HCLDO,
			      MT6369_AUDDEC_ANA_CON25,
			      RG_HCLDO_EN_VA18_SFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("HCLDO_REMOTE", SUPPLY_SEQ_DL_HCLDO_REMOTE_SENSE,
			      MT6369_AUDDEC_ANA_CON25,
			      RG_HCLDO_REMOTE_SENSE_VA18_SFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("NV_REGULATOR", SUPPLY_SEQ_DL_NV,
			      MT6369_AUDDEC_ANA_CON27,
			      RG_NVREG_EN_VAUDP15_SFT, 0,
			      mt_delay_100_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_SUPPLY_S("IBIST", SUPPLY_SEQ_DL_IBIST,
			      MT6369_AUDDEC_ANA_CON22,
			      RG_AUDIBIASPWRDN_VAUDP15_SFT, 1, NULL, 0),

	/* DAC */
	SND_SOC_DAPM_DAC("DACL", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DACR", NULL, SND_SOC_NOPM, 0, 0),

	/* Headphone */
	SND_SOC_DAPM_MUX_E("HPL Mux", SND_SOC_NOPM, 0, 0,
			   &hpl_in_mux_control,
			   mt_hp_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MUX_E("HPR Mux", SND_SOC_NOPM, 0, 0,
			   &hpr_in_mux_control,
			   mt_hp_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("HP_Supply", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("HP_MUTE", SUPPLY_SEQ_HP_MUTE,
			      SND_SOC_NOPM, 0, 0,
			      mt_hp_mute_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* Receiver */
	SND_SOC_DAPM_MUX_E("RCV Mux", SND_SOC_NOPM, 0, 0,
			   &rcv_in_mux_control,
			   mt_rcv_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	/* LOL */
	SND_SOC_DAPM_MUX_E("LOL Mux", SND_SOC_NOPM, 0, 0,
			   &lo_in_mux_control,
			   mt_lo_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	/* Outputs */
	SND_SOC_DAPM_OUTPUT("Receiver"),
	SND_SOC_DAPM_OUTPUT("Headphone L"),
	SND_SOC_DAPM_OUTPUT("Headphone R"),
	SND_SOC_DAPM_OUTPUT("LINEOUT L"),

	/* Uplinks */
	SND_SOC_DAPM_AIF_OUT("AIF1TX", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_SUPPLY_S("ADC_CLKGEN", SUPPLY_SEQ_ADC_CLKGEN,
			      SND_SOC_NOPM, 0, 0,
			      mt_adc_clk_gen_event,
			      SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY_S("DCC_CLK", SUPPLY_SEQ_DCC_CLK,
			      SND_SOC_NOPM, 0, 0,
			      mt_dcc_clk_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("AIFTX_Supply", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("MTKAIF_TX", SUPPLY_SEQ_UL_MTKAIF,
			      SND_SOC_NOPM, 0, 0,
			      mt_mtkaif_tx_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("UL_SRC", SUPPLY_SEQ_UL_SRC,
			      MT6369_AFE_UL_SRC_CON1,
			      UL_SRC_ON_TMP_CTL_SFT, 0, NULL, 0),

	SND_SOC_DAPM_MUX("MISO0_MUX", SND_SOC_NOPM, 0, 0, &miso0_mux_control),
	SND_SOC_DAPM_MUX("MISO1_MUX", SND_SOC_NOPM, 0, 0, &miso1_mux_control),
	SND_SOC_DAPM_MUX("UL_SRC_MUX", SND_SOC_NOPM, 0, 0, &ul_src_mux_control),

	SND_SOC_DAPM_MUX("ADC_L_Mux", SND_SOC_NOPM, 0, 0, &adc_left_mux_control),
	SND_SOC_DAPM_MUX("ADC_R_Mux", SND_SOC_NOPM, 0, 0, &adc_right_mux_control),

	SND_SOC_DAPM_ADC("ADC_L", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC_R", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_SUPPLY_S("ADC_L_EN", SUPPLY_SEQ_UL_ADC,
			      MT6369_AUDENC_ANA_CON1,
			      RG_AUDADCLPWRUP_SFT, 0,
			      mt_adc_l_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_SUPPLY_S("ADC_R_EN", SUPPLY_SEQ_UL_ADC,
			      MT6369_AUDENC_ANA_CON3,
			      RG_AUDADCRPWRUP_SFT, 0,
			      mt_adc_r_event, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_MUX_E("PGA_L_Mux", SND_SOC_NOPM, 0, 0,
			   &pga_left_mux_control,
			   mt_pga_l_mux_event, SND_SOC_DAPM_WILL_PMU),
	SND_SOC_DAPM_MUX_E("PGA_R_Mux", SND_SOC_NOPM, 0, 0,
			   &pga_right_mux_control,
			   mt_pga_r_mux_event, SND_SOC_DAPM_WILL_PMU),

	SND_SOC_DAPM_PGA("PGA_L", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("PGA_R", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY_S("PGA_L_EN", SUPPLY_SEQ_UL_PGA,
			      MT6369_AUDENC_ANA_CON0,
			      RG_AUDPREAMPLON_SFT, 0,
			      mt_pga_l_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_SUPPLY_S("PGA_R_EN", SUPPLY_SEQ_UL_PGA,
			      MT6369_AUDENC_ANA_CON2,
			      RG_AUDPREAMPRON_SFT, 0,
			      mt_pga_r_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),

	/* UL input */
	SND_SOC_DAPM_INPUT("AIN0"),
	SND_SOC_DAPM_INPUT("AIN1"),
	SND_SOC_DAPM_INPUT("AIN2"),

	/* mic bias */
	SND_SOC_DAPM_SUPPLY_S("MIC_BIAS_0", SUPPLY_SEQ_MIC_BIAS,
			      MT6369_AUDENC_ANA_CON17,
			      RG_AUDPWDBMICBIAS0_SFT, 0,
			      mt_mic_bias_0_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("MIC_BIAS_1", SUPPLY_SEQ_MIC_BIAS,
			      MT6369_AUDENC_ANA_CON19,
			      RG_AUDPWDBMICBIAS1_SFT, 0,
			      mt_mic_bias_1_event, SND_SOC_DAPM_PRE_PMU),
};

static int mt_dcc_clk_connect(struct snd_soc_dapm_widget *source,
			      struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(sink->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	return IS_DCC_BASE(priv->mux_select[MUX_MIC_TYPE_0]) ||
	       IS_DCC_BASE(priv->mux_select[MUX_MIC_TYPE_1]) ||
	       IS_DCC_BASE(priv->mux_select[MUX_MIC_TYPE_2]);
}

static const struct snd_soc_dapm_route mt6369_dapm_routes[] = {
	/* Capture */
	{"AIFTX_Supply", NULL, "AUDGLB"},
	{"AIFTX_Supply", NULL, "CLKSQ Audio"},
	{"AIFTX_Supply", NULL, "AUD_CK"},
	{"AIFTX_Supply", NULL, "AUDIF_CK"},
	{"AIFTX_Supply", NULL, "AUDIO_TOP_AFE_CTL"},
	{"AIFTX_Supply", NULL, "AUDIO_TOP_PWR_CLK"},
	{"AIFTX_Supply", NULL, "AUDIO_TOP_PDN_RESERVED"},
	/*
	 * *_ADC_CTL should enable only if UL_SRC in use,
	 * but dm ck may be needed even UL_SRC_x not in use
	 */
	{"AIFTX_Supply", NULL, "AUDIO_TOP_ADC_CTL"},
	{"AIFTX_Supply", NULL, "AUDIO_TOP_ADDA6_ADC_CTL"},
	{"AIFTX_Supply", NULL, "AFE_ON"},

	/* ul ch 12 */
	{"AIF1TX", NULL, "MISO0_MUX"},
	{"AIF1TX", NULL, "MISO1_MUX"},
	{"AIF1TX", NULL, "AIFTX_Supply"},
	{"AIF1TX", NULL, "MTKAIF_TX"},
	{"AIF1TX", NULL, "UL_GPIO"},

	{"MISO0_MUX", "UL1_CH1", "UL_SRC_MUX"},
	{"MISO0_MUX", "UL1_CH2", "UL_SRC_MUX"},
	{"MISO1_MUX", "UL1_CH1", "UL_SRC_MUX"},
	{"MISO1_MUX", "UL1_CH2", "UL_SRC_MUX"},

	{"UL_SRC_MUX", "AMIC", "ADC_L"},
	{"UL_SRC_MUX", "AMIC", "ADC_R"},
	{"UL_SRC_MUX", NULL, "UL_SRC"},

	/* adc */
	{"ADC_L", NULL, "ADC_L_Mux"},
	{"ADC_L", NULL, "ADC_CLKGEN"},
	{"ADC_L", NULL, "ADC_L_EN"},
	{"ADC_R", NULL, "ADC_R_Mux"},
	{"ADC_R", NULL, "ADC_CLKGEN"},
	{"ADC_R", NULL, "ADC_R_EN"},
	/*
	 * amic fifo ch1/2 clk from ADC_L,
	 * enable ADC_L even use ADC_R only
	 */
	{"ADC_R", NULL, "ADC_L_EN"},

	{"ADC_L_Mux", "Left Preamplifier", "PGA_L"},
	{"ADC_R_Mux", "Right Preamplifier", "PGA_R"},

	{"PGA_L", NULL, "PGA_L_Mux"},
	{"PGA_L", NULL, "PGA_L_EN"},
	{"PGA_R", NULL, "PGA_R_Mux"},
	{"PGA_R", NULL, "PGA_R_EN"},

	{"PGA_L", NULL, "DCC_CLK", mt_dcc_clk_connect},
	{"PGA_R", NULL, "DCC_CLK", mt_dcc_clk_connect},

	{"PGA_L_Mux", "AIN0", "AIN0"},
	{"PGA_L_Mux", "AIN1", "AIN1"},
	{"PGA_L_Mux", "AIN2", "AIN2"},

	{"PGA_R_Mux", "AIN0", "AIN0"},
	{"PGA_R_Mux", "AIN2", "AIN2"},

	{"AIN0", NULL, "MIC_BIAS_0"},
	{"AIN1", NULL, "MIC_BIAS_1"},
	{"AIN2", NULL, "MIC_BIAS_0"},

	/* DL Supply */
	{"DL Power Supply", NULL, "AUDGLB"},
	{"DL Power Supply", NULL, "CLKSQ Audio"},
	{"DL Power Supply", NULL, "AUDNCP_CK"},
	{"DL Power Supply", NULL, "ZCD13M_CK"},
	{"DL Power Supply", NULL, "AUD_CK"},
	{"DL Power Supply", NULL, "AUDIF_CK"},
	{"DL Power Supply", NULL, "ESD_RESIST"},
	{"DL Power Supply", NULL, "LDO_VA33REFGEN"},
	{"DL Power Supply", NULL, "LCLDO"},
	{"DL Power Supply", NULL, "LCLDO_REMOTE"},
	{"DL Power Supply", NULL, "HCLDO"},
	{"DL Power Supply", NULL, "HCLDO_REMOTE"},
	{"DL Power Supply", NULL, "NV_REGULATOR"},
	{"DL Power Supply", NULL, "IBIST"},

	/* DL Digital Supply */
	{"DL Digital Clock", NULL, "AUDIO_TOP_AFE_CTL"},
	{"DL Digital Clock", NULL, "AUDIO_TOP_DAC_CTL"},
	{"DL Digital Clock", NULL, "AUDIO_TOP_PWR_CLK"},
	{"DL Digital Clock", NULL, "AUDIO_TOP_PDN_RESERVED"},
	{"DL Digital Clock", NULL, "SDM_FIFO_CLK"},
	{"DL Digital Clock", NULL, "NCP"},
	{"DL Digital Clock", NULL, "AFE_ON"},
	{"DL Digital Clock", NULL, "AFE_DL_SRC"},

	{"DL Digital Clock CH_1_2", NULL, "DL Digital Clock"},
	{"DL Digital Clock CH_1_2", NULL, "SDM"},

	{"AIF_RX", NULL, "DL Digital Clock CH_1_2"},
	{"AIF_RX", NULL, "DL_GPIO"},

	/* DL Path */
	{"DACL", NULL, "AIF_RX"},
	{"DACL", NULL, "DL Power Supply"},
	{"DACR", NULL, "AIF_RX"},
	{"DACR", NULL, "DL Power Supply"},

	/* Lineout Path */
	{"LOL Mux", "Playback", "DACR"},
	{"LINEOUT L", NULL, "LOL Mux"},

	/* Headphone Path */
	{"HP_Supply", NULL, "HP_MUTE"},
	{"HPL Mux", NULL, "HP_Supply"},
	{"HPR Mux", NULL, "HP_Supply"},

	{"HPL Mux", "Audio Playback", "DACL"},
	{"HPR Mux", "Audio Playback", "DACR"},

	{"Headphone L", NULL, "HPL Mux"},
	{"Headphone R", NULL, "HPR Mux"},

	/* Receiver Path */
	{"RCV Mux", "Voice Playback", "DACL"},
	{"Receiver", NULL, "RCV Mux"},
};

#define MT6369_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver mt6369_dai_driver[] = {
	{
		.id = MT6369_AIF_1,
		.name = "mt6369-snd-codec-aif1",
		.playback = {
			.stream_name = "AIF1 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000 |
				 SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_192000,
			.formats = MT6369_FORMATS,
		},
		.capture = {
			.stream_name = "AIF1 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000 |
				 SNDRV_PCM_RATE_16000 |
				 SNDRV_PCM_RATE_32000 |
				 SNDRV_PCM_RATE_48000 |
				 SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_192000,
			.formats = MT6369_FORMATS,
		},
	},
};

/*
 * Put the parts that no DAPM widget owns into a known, powered-down state:
 * MTKAIF pads to GPIO input, output gains muted, gain control through the
 * ZCD, ZCD off. Nothing is switched on here.
 */
static void mt6369_codec_init_reg(struct mt6369_priv *priv)
{
	/* audio clk source from internal dcxo */
	regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON12,
			   RG_CLKSQ_IN_SEL_TEST_MASK_SFT, 0x0);

	/* set gpio */
	mt6369_set_gpio_smt(priv);
	mt6369_set_gpio_driving(priv);
	mt6369_reset_playback_gpio(priv);
	mt6369_reset_capture_gpio(priv);

	/* hp gain ctl: ZCD, not NLE */
	regmap_update_bits(priv->regmap, MT6369_AFE_DL_NLE_CFG,
			   NLE_LCH_HPGAIN_SEL_MASK_SFT,
			   HP_GAIN_CTL_ZCD << NLE_LCH_HPGAIN_SEL_SFT);
	regmap_update_bits(priv->regmap, MT6369_AFE_DL_NLE_CFG,
			   NLE_RCH_HPGAIN_SEL_MASK_SFT,
			   HP_GAIN_CTL_ZCD << NLE_RCH_HPGAIN_SEL_SFT);

	/* Disable AUD_ZCD */
	zcd_enable(priv, false, DEVICE_NUM);

	/* mute all analog outputs */
	regmap_write(priv->regmap, MT6369_ZCD_CON1, DL_GAIN_N_40DB);
	regmap_write(priv->regmap, MT6369_ZCD_CON2, DL_GAIN_N_40DB);
	regmap_write(priv->regmap, MT6369_ZCD_CON3, DL_GAIN_N_40DB);
	regmap_write(priv->regmap, MT6369_ZCD_CON4, DL_GAIN_N_40DB);
	regmap_write(priv->regmap, MT6369_ZCD_CON5, DL_GAIN_N_40DB);

	/* DCC microphones need the DCC enable bits of the preamplifiers */
	if (IS_DCC_BASE(priv->mux_select[MUX_MIC_TYPE_0])) {
		regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON0,
				   RG_AUDPREAMPLDCCEN_MASK_SFT,
				   0x1 << RG_AUDPREAMPLDCCEN_SFT);
		regmap_update_bits(priv->regmap, MT6369_AUDENC_ANA_CON2,
				   RG_AUDPREAMPRDCCEN_MASK_SFT,
				   0x1 << RG_AUDPREAMPRDCCEN_SFT);
	}
}

static int mt6369_codec_probe(struct snd_soc_component *cmpnt)
{
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	int i;

	snd_soc_component_init_regmap(cmpnt, priv->regmap);

	/* every output starts at its lowest gain */
	for (i = 0; i < AUDIO_ANALOG_VOLUME_TYPE_MAX; i++)
		priv->ana_gain[i] = DL_GAIN_N_10DB;

	mt6369_codec_init_reg(priv);

	return 0;
}

static void mt6369_codec_remove(struct snd_soc_component *cmpnt)
{
	snd_soc_component_exit_regmap(cmpnt);
}

static const struct snd_soc_component_driver mt6369_soc_component_driver = {
	.name = CODEC_MT6369_NAME,
	.probe = mt6369_codec_probe,
	.remove = mt6369_codec_remove,
	.controls = mt6369_snd_controls,
	.num_controls = ARRAY_SIZE(mt6369_snd_controls),
	.dapm_widgets = mt6369_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6369_dapm_widgets),
	.dapm_routes = mt6369_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(mt6369_dapm_routes),
	.endianness = 1,
};

static int mt6369_parse_dt(struct mt6369_priv *priv)
{
	static const char * const props[] = {
		"mediatek,mic-type-0", "mediatek,mic-type-1",
		"mediatek,mic-type-2",
	};
	struct device_node *np = priv->dev->of_node;
	u32 mic_type;
	int i;

	if (!np)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		mic_type = MIC_TYPE_MUX_IDLE;
		of_property_read_u32(np, props[i], &mic_type);
		if (mic_type > MIC_TYPE_MUX_DCC_ECM_SINGLE ||
		    mic_type == MIC_TYPE_MUX_DMIC) {
			dev_warn(priv->dev, "unsupported %s %u\n", props[i],
				 mic_type);
			mic_type = MIC_TYPE_MUX_IDLE;
		}
		priv->mux_select[MUX_MIC_TYPE_0 + i] = mic_type;
	}

	return 0;
}

static int mt6369_platform_driver_probe(struct platform_device *pdev)
{
	struct mt6369_priv *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	priv->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!priv->regmap)
		return dev_err_probe(&pdev->dev, -ENODEV, "no PMIC regmap\n");

	dev_set_drvdata(&pdev->dev, priv);

	ret = mt6369_parse_dt(priv);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to parse DT\n");

	return devm_snd_soc_register_component(&pdev->dev,
					       &mt6369_soc_component_driver,
					       mt6369_dai_driver,
					       ARRAY_SIZE(mt6369_dai_driver));
}

static const struct of_device_id mt6369_of_match[] = {
	{ .compatible = "mediatek,mt6369-sound" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6369_of_match);

static struct platform_driver mt6369_platform_driver = {
	.driver = {
		.name = "mt6369-sound",
		.of_match_table = mt6369_of_match,
	},
	.probe = mt6369_platform_driver_probe,
};
module_platform_driver(mt6369_platform_driver);

MODULE_DESCRIPTION("MT6369 ALSA SoC codec driver");
MODULE_AUTHOR("Chih-hong Yang <chih-hong.yang@mediatek.com>");
MODULE_LICENSE("GPL");
