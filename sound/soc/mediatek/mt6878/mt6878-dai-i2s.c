// SPDX-License-Identifier: GPL-2.0
/*
 *  MediaTek ALSA SoC Audio DAI I2S Control
 *
 *  The ETDM pair I2SIN4/I2SOUT4 of MT6878, the interface of the speaker
 *  amplifier: I2SIN4 is the clock master (its BCK/LRCK pins drive the
 *  bus), I2SOUT4 sends the data in cowork with it. The register handling
 *  follows the MT8196 driver; the MT6878 differences (no I2S4 pad-top
 *  clock enable, init point 3) are those of the MediaTek downstream MT6878 driver.
 *
 *  Copyright (c) 2025 MediaTek Inc.
 *  Author: Darren Ye <darren.ye@mediatek.com>
 *  Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 */

#include <linux/bitops.h>
#include <linux/regmap.h>

#include <sound/pcm_params.h>

#include "mt6878-afe-clk.h"
#include "mt6878-afe-common.h"
#include "mt6878-interconnection.h"

#define ETDM_22M_CLOCK_THRES 11289600

enum {
	ETDM_CLK_SOURCE_H26M,
	ETDM_CLK_SOURCE_APLL,
};

enum {
	ETDM_RELATCH_SEL_H26M,
	ETDM_RELATCH_SEL_APLL,
};

enum {
	ETDM_RATE_8K,
	ETDM_RATE_12K,
	ETDM_RATE_16K,
	ETDM_RATE_24K,
	ETDM_RATE_32K,
	ETDM_RATE_48K,
	ETDM_RATE_64K,
	ETDM_RATE_96K,
	ETDM_RATE_128K,
	ETDM_RATE_192K,
	ETDM_RATE_256K,
	ETDM_RATE_384K,
	ETDM_RATE_11025 = 16,
	ETDM_RATE_22050,
	ETDM_RATE_44100,
	ETDM_RATE_88200,
	ETDM_RATE_176400,
	ETDM_RATE_352800,
};

enum {
	ETDM_CONN_8K,
	ETDM_CONN_11K,
	ETDM_CONN_12K,
	ETDM_CONN_16K = 4,
	ETDM_CONN_22K,
	ETDM_CONN_24K,
	ETDM_CONN_32K = 8,
	ETDM_CONN_44K,
	ETDM_CONN_48K,
	ETDM_CONN_88K = 13,
	ETDM_CONN_96K,
	ETDM_CONN_176K = 17,
	ETDM_CONN_192K,
};

enum {
	ETDM_WLEN_8_BIT = 0x7,
	ETDM_WLEN_16_BIT = 0xf,
	ETDM_WLEN_32_BIT = 0x1f,
};

/* ETDM_4_7_COWORK_CON0 slave select values */
enum {
	ETDM_SLAVE_SEL_ETDMIN4_MASTER,
	ETDM_SLAVE_SEL_ETDMIN4_SLAVE,
	ETDM_SLAVE_SEL_ETDMIN5_MASTER,
	ETDM_SLAVE_SEL_ETDMIN5_SLAVE,
	ETDM_SLAVE_SEL_ETDMIN6_MASTER,
	ETDM_SLAVE_SEL_ETDMIN6_SLAVE,
	ETDM_SLAVE_SEL_ETDMIN7_MASTER,
	ETDM_SLAVE_SEL_ETDMIN7_SLAVE,
	ETDM_SLAVE_SEL_ETDMOUT4_MASTER,
};

enum {
	MTK_DAI_ETDM_FORMAT_I2S,
	MTK_DAI_ETDM_FORMAT_LJ,
	MTK_DAI_ETDM_FORMAT_RJ,
	MTK_DAI_ETDM_FORMAT_EIAJ,
	MTK_DAI_ETDM_FORMAT_DSPA,
	MTK_DAI_ETDM_FORMAT_DSPB,
};

struct mtk_afe_i2s_priv {
	u32 rate; /* for determine which apll to use */
	int share_i2s_id;
	u8 format;
};

static unsigned int get_etdm_wlen(snd_pcm_format_t format)
{
	return snd_pcm_format_physical_width(format) <= 16 ?
		ETDM_WLEN_16_BIT : ETDM_WLEN_32_BIT;
}

static unsigned int get_etdm_lrck_width(snd_pcm_format_t format)
{
	/* The valid data bit number should be large than 7 due to hardware limitation. */
	return snd_pcm_format_physical_width(format) - 1;
}

static unsigned int get_etdm_rate(unsigned int rate)
{
	switch (rate) {
	case 8000:
		return ETDM_RATE_8K;
	case 12000:
		return ETDM_RATE_12K;
	case 16000:
		return ETDM_RATE_16K;
	case 24000:
		return ETDM_RATE_24K;
	case 32000:
		return ETDM_RATE_32K;
	case 48000:
		return ETDM_RATE_48K;
	case 96000:
		return ETDM_RATE_96K;
	case 192000:
		return ETDM_RATE_192K;
	case 11025:
		return ETDM_RATE_11025;
	case 22050:
		return ETDM_RATE_22050;
	case 44100:
		return ETDM_RATE_44100;
	case 88200:
		return ETDM_RATE_88200;
	case 176400:
		return ETDM_RATE_176400;
	default:
		return 0;
	}
}

static unsigned int get_etdm_inconn_rate(unsigned int rate)
{
	switch (rate) {
	case 8000:
		return ETDM_CONN_8K;
	case 12000:
		return ETDM_CONN_12K;
	case 16000:
		return ETDM_CONN_16K;
	case 24000:
		return ETDM_CONN_24K;
	case 32000:
		return ETDM_CONN_32K;
	case 48000:
		return ETDM_CONN_48K;
	case 96000:
		return ETDM_CONN_96K;
	case 192000:
		return ETDM_CONN_192K;
	case 11025:
		return ETDM_CONN_11K;
	case 22050:
		return ETDM_CONN_22K;
	case 44100:
		return ETDM_CONN_44K;
	case 88200:
		return ETDM_CONN_88K;
	case 176400:
		return ETDM_CONN_176K;
	default:
		return 0;
	}
}

static int get_i2s_id_by_name(const char *name)
{
	if (strncmp(name, "I2SIN4", 6) == 0)
		return MT6878_DAI_I2S_IN4;
	else if (strncmp(name, "I2SOUT4", 7) == 0)
		return MT6878_DAI_I2S_OUT4;

	return -EINVAL;
}

static struct mtk_afe_i2s_priv *get_i2s_priv_by_name(struct mtk_base_afe *afe,
						     const char *name)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	int dai_id = get_i2s_id_by_name(name);

	if (dai_id < 0)
		return NULL;

	return afe_priv->dai_priv[dai_id];
}

static const struct snd_kcontrol_new mtk_i2sout4_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("DL0_CH1", AFE_CONN116_1, I_DL0_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL1_CH1", AFE_CONN116_1, I_DL1_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL2_CH1", AFE_CONN116_1, I_DL2_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL3_CH1", AFE_CONN116_1, I_DL3_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL4_CH1", AFE_CONN116_1, I_DL4_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL5_CH1", AFE_CONN116_1, I_DL5_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL6_CH1", AFE_CONN116_1, I_DL6_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL7_CH1", AFE_CONN116_1, I_DL7_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL8_CH1", AFE_CONN116_1, I_DL8_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL_24CH_CH1", AFE_CONN116_1, I_DL_24CH_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL24_CH1", AFE_CONN116_2, I_DL24_CH1, 1, 0),
};

static const struct snd_kcontrol_new mtk_i2sout4_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("DL0_CH2", AFE_CONN117_1, I_DL0_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL1_CH2", AFE_CONN117_1, I_DL1_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL2_CH2", AFE_CONN117_1, I_DL2_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL3_CH2", AFE_CONN117_1, I_DL3_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL4_CH2", AFE_CONN117_1, I_DL4_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL5_CH2", AFE_CONN117_1, I_DL5_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL6_CH2", AFE_CONN117_1, I_DL6_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL7_CH2", AFE_CONN117_1, I_DL7_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL8_CH2", AFE_CONN117_1, I_DL8_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL_24CH_CH2", AFE_CONN117_1, I_DL_24CH_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL24_CH2", AFE_CONN117_2, I_DL24_CH2, 1, 0),
};

enum {
	SUPPLY_SEQ_APLL,
	SUPPLY_SEQ_I2S_CG_EN,
	SUPPLY_SEQ_I2S_EN,
};

static int mtk_apll_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol,
			  int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	bool apll1 = strcmp(w->name, APLL1_W_NAME) == 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return apll1 ? mt6878_apll1_enable(afe) : mt6878_apll2_enable(afe);
	case SND_SOC_DAPM_POST_PMD:
		if (apll1)
			mt6878_apll1_disable(afe);
		else
			mt6878_apll2_disable(afe);
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget mtk_dai_i2s_widgets[] = {
	SND_SOC_DAPM_MIXER("I2SOUT4_CH1", SND_SOC_NOPM, 0, 0,
			   mtk_i2sout4_ch1_mix,
			   ARRAY_SIZE(mtk_i2sout4_ch1_mix)),
	SND_SOC_DAPM_MIXER("I2SOUT4_CH2", SND_SOC_NOPM, 0, 0,
			   mtk_i2sout4_ch2_mix,
			   ARRAY_SIZE(mtk_i2sout4_ch2_mix)),

	/* i2s en */
	SND_SOC_DAPM_SUPPLY_S("I2SIN4_EN", SUPPLY_SEQ_I2S_EN,
			      ETDM_IN4_CON0, REG_ETDM_IN_EN_SFT, 0,
			      NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("I2SOUT4_EN", SUPPLY_SEQ_I2S_EN,
			      ETDM_OUT4_CON0, OUT_REG_ETDM_OUT_EN_SFT, 0,
			      NULL, 0),

	/* cg */
	SND_SOC_DAPM_SUPPLY_S("I2SOUT4_CG", SUPPLY_SEQ_I2S_CG_EN,
			      AUDIO_TOP_CON2, PDN_ETDM_OUT4_SFT, 1,
			      NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("I2SIN4_CG", SUPPLY_SEQ_I2S_CG_EN,
			      AUDIO_TOP_CON2, PDN_ETDM_IN4_SFT, 1,
			      NULL, 0),

	/* apll */
	SND_SOC_DAPM_SUPPLY_S(APLL1_W_NAME, SUPPLY_SEQ_APLL,
			      SND_SOC_NOPM, 0, 0,
			      mtk_apll_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S(APLL2_W_NAME, SUPPLY_SEQ_APLL,
			      SND_SOC_NOPM, 0, 0,
			      mtk_apll_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static int mtk_afe_i2s_share_connect(struct snd_soc_dapm_widget *source,
				     struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(sink->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mtk_afe_i2s_priv *i2s_priv;

	i2s_priv = get_i2s_priv_by_name(afe, sink->name);
	if (!i2s_priv || i2s_priv->share_i2s_id < 0)
		return 0;

	return i2s_priv->share_i2s_id == get_i2s_id_by_name(source->name);
}

static int mtk_afe_i2s_apll_connect(struct snd_soc_dapm_widget *source,
				    struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(sink->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mtk_afe_i2s_priv *i2s_priv;

	i2s_priv = get_i2s_priv_by_name(afe, sink->name);
	if (!i2s_priv)
		return 0;

	/* choose APLL from i2s rate */
	return mt6878_get_apll_by_rate(afe, i2s_priv->rate) ==
	       mt6878_get_apll_by_name(afe, source->name);
}

static const struct snd_soc_dapm_route mtk_dai_i2s_routes[] = {
	/* i2sin4: also the clock master of i2sout4 */
	{"I2SIN4", NULL, "I2SIN4_EN"},
	{"I2SIN4", NULL, "I2SOUT4_EN", mtk_afe_i2s_share_connect},
	{"I2SIN4", NULL, APLL1_W_NAME, mtk_afe_i2s_apll_connect},
	{"I2SIN4", NULL, APLL2_W_NAME, mtk_afe_i2s_apll_connect},
	{"I2SIN4", NULL, "I2SIN4_CG"},

	/* i2sout4 */
	{"I2SOUT4_CH1", "DL0_CH1", "DL0"},
	{"I2SOUT4_CH2", "DL0_CH2", "DL0"},
	{"I2SOUT4_CH1", "DL1_CH1", "DL1"},
	{"I2SOUT4_CH2", "DL1_CH2", "DL1"},
	{"I2SOUT4_CH1", "DL2_CH1", "DL2"},
	{"I2SOUT4_CH2", "DL2_CH2", "DL2"},
	{"I2SOUT4_CH1", "DL3_CH1", "DL3"},
	{"I2SOUT4_CH2", "DL3_CH2", "DL3"},
	{"I2SOUT4_CH1", "DL4_CH1", "DL4"},
	{"I2SOUT4_CH2", "DL4_CH2", "DL4"},
	{"I2SOUT4_CH1", "DL5_CH1", "DL5"},
	{"I2SOUT4_CH2", "DL5_CH2", "DL5"},
	{"I2SOUT4_CH1", "DL6_CH1", "DL6"},
	{"I2SOUT4_CH2", "DL6_CH2", "DL6"},
	{"I2SOUT4_CH1", "DL7_CH1", "DL7"},
	{"I2SOUT4_CH2", "DL7_CH2", "DL7"},
	{"I2SOUT4_CH1", "DL8_CH1", "DL8"},
	{"I2SOUT4_CH2", "DL8_CH2", "DL8"},
	{"I2SOUT4_CH1", "DL_24CH_CH1", "DL_24CH"},
	{"I2SOUT4_CH2", "DL_24CH_CH2", "DL_24CH"},
	{"I2SOUT4_CH1", "DL24_CH1", "DL24"},
	{"I2SOUT4_CH2", "DL24_CH2", "DL24"},

	{"I2SOUT4", NULL, "I2SOUT4_CH1"},
	{"I2SOUT4", NULL, "I2SOUT4_CH2"},

	{"I2SOUT4", NULL, "I2SIN4_EN", mtk_afe_i2s_share_connect},
	{"I2SOUT4", NULL, "I2SOUT4_EN"},
	{"I2SOUT4", NULL, APLL1_W_NAME, mtk_afe_i2s_apll_connect},
	{"I2SOUT4", NULL, APLL2_W_NAME, mtk_afe_i2s_apll_connect},
	{"I2SOUT4", NULL, "I2SOUT4_CG"},
	{"I2SOUT4", NULL, "I2SIN4_CG"},
};

static void mtk_dai_etdm_in4_config(struct mtk_base_afe *afe,
				    struct mtk_afe_i2s_priv *i2s_priv,
				    unsigned int rate, snd_pcm_format_t format,
				    unsigned int channels)
{
	struct regmap *map = afe->regmap;

	regmap_update_bits(map, ETDM_IN4_CON1, REG_INITIAL_COUNT_MASK_SFT,
			   0x5 << REG_INITIAL_COUNT_SFT);
	/* 3: pad top 5: no pad top */
	regmap_update_bits(map, ETDM_IN4_CON1, REG_INITIAL_POINT_MASK_SFT,
			   0x3 << REG_INITIAL_POINT_SFT);
	regmap_update_bits(map, ETDM_IN4_CON1, REG_LRCK_RESET_MASK_SFT,
			   0x1 << REG_LRCK_RESET_SFT);
	regmap_update_bits(map, ETDM_IN4_CON2, REG_CLOCK_SOURCE_SEL_MASK_SFT,
			   ETDM_CLK_SOURCE_APLL << REG_CLOCK_SOURCE_SEL_SFT);
	/* 0: manual 1: auto */
	regmap_update_bits(map, ETDM_IN4_CON2, REG_CK_EN_SEL_AUTO_MASK_SFT,
			   0x1 << REG_CK_EN_SEL_AUTO_SFT);
	regmap_update_bits(map, ETDM_IN4_CON3, REG_FS_TIMING_SEL_MASK_SFT,
			   get_etdm_rate(rate) << REG_FS_TIMING_SEL_SFT);
	regmap_update_bits(map, ETDM_IN4_CON4, REG_RELATCH_1X_EN_SEL_MASK_SFT,
			   get_etdm_inconn_rate(rate) << REG_RELATCH_1X_EN_SEL_SFT);
	regmap_update_bits(map, ETDM_IN4_CON8, REG_ETDM_USE_AFIFO_MASK_SFT,
			   0x0 << REG_ETDM_USE_AFIFO_SFT);
	regmap_update_bits(map, ETDM_IN4_CON8, REG_AFIFO_MODE_MASK_SFT,
			   0x0 << REG_AFIFO_MODE_SFT);
	regmap_update_bits(map, ETDM_IN4_CON9, REG_ALMOST_END_CH_COUNT_MASK_SFT,
			   0x0 << REG_ALMOST_END_CH_COUNT_SFT);
	regmap_update_bits(map, ETDM_IN4_CON9, REG_ALMOST_END_BIT_COUNT_MASK_SFT,
			   0x0 << REG_ALMOST_END_BIT_COUNT_SFT);
	regmap_update_bits(map, ETDM_IN4_CON9, REG_OUT2LATCH_TIME_MASK_SFT,
			   0x6 << REG_OUT2LATCH_TIME_SFT);

	/* format: 0 i2s, 4 dsp_a */
	regmap_update_bits(map, ETDM_IN4_CON0, REG_FMT_MASK_SFT,
			   i2s_priv->format << REG_FMT_SFT);
	regmap_update_bits(map, ETDM_IN4_CON0, REG_CH_NUM_MASK_SFT,
			   (channels - 1) << REG_CH_NUM_SFT);
	regmap_update_bits(map, ETDM_IN4_CON0, REG_SYNC_MODE_MASK_SFT, 0);

	/* APLL */
	regmap_update_bits(map, ETDM_IN4_CON0,
			   REG_RELATCH_1X_EN_DOMAIN_SEL_MASK_SFT,
			   ETDM_RELATCH_SEL_APLL << REG_RELATCH_1X_EN_DOMAIN_SEL_SFT);
	regmap_update_bits(map, ETDM_IN4_CON0, REG_BIT_LENGTH_MASK_SFT,
			   get_etdm_lrck_width(format) << REG_BIT_LENGTH_SFT);
	regmap_update_bits(map, ETDM_IN4_CON0, REG_WORD_LENGTH_MASK_SFT,
			   get_etdm_wlen(format) << REG_WORD_LENGTH_SFT);

	/* ---etdm cowork --- */
	regmap_update_bits(map, ETDM_4_7_COWORK_CON0, ETDM_IN4_SLAVE_SEL_MASK_SFT,
			   ETDM_SLAVE_SEL_ETDMOUT4_MASTER << ETDM_IN4_SLAVE_SEL_SFT);
}

static void mtk_dai_etdm_out4_config(struct mtk_base_afe *afe,
				     struct mtk_afe_i2s_priv *i2s_priv,
				     unsigned int rate, snd_pcm_format_t format,
				     unsigned int channels)
{
	struct regmap *map = afe->regmap;
	unsigned int wlen = get_etdm_wlen(format);

	regmap_update_bits(map, ETDM_OUT4_CON1, OUT_REG_INITIAL_COUNT_MASK_SFT,
			   0x5 << OUT_REG_INITIAL_COUNT_SFT);
	regmap_update_bits(map, ETDM_OUT4_CON1, OUT_REG_INITIAL_POINT_MASK_SFT,
			   0x6 << OUT_REG_INITIAL_POINT_SFT);

	/* clock speed > 22M need to set relatch time to avoid duplicate point */
	if (rate * channels * (wlen + 1) >= ETDM_22M_CLOCK_THRES)
		regmap_update_bits(map, ETDM_OUT4_CON2, OUT_REG_IN2LATCH_TIME_MASK_SFT,
				   (wlen - 2) << OUT_REG_IN2LATCH_TIME_SFT);
	else
		regmap_update_bits(map, ETDM_OUT4_CON2, OUT_REG_IN2LATCH_TIME_MASK_SFT,
				   0x6 << OUT_REG_IN2LATCH_TIME_SFT);

	regmap_update_bits(map, ETDM_OUT4_CON1, OUT_REG_LRCK_RESET_MASK_SFT,
			   0x1 << OUT_REG_LRCK_RESET_SFT);
	regmap_update_bits(map, ETDM_OUT4_CON4, OUT_REG_FS_TIMING_SEL_MASK_SFT,
			   get_etdm_rate(rate) << OUT_REG_FS_TIMING_SEL_SFT);
	regmap_update_bits(map, ETDM_OUT4_CON4, OUT_REG_CLOCK_SOURCE_SEL_MASK_SFT,
			   ETDM_CLK_SOURCE_APLL << OUT_REG_CLOCK_SOURCE_SEL_SFT);
	regmap_update_bits(map, ETDM_OUT4_CON4, OUT_REG_RELATCH_EN_SEL_MASK_SFT,
			   get_etdm_inconn_rate(rate) << OUT_REG_RELATCH_EN_SEL_SFT);

	/* format: 0 i2s, 4 dsp_a */
	regmap_update_bits(map, ETDM_OUT4_CON0, OUT_REG_FMT_MASK_SFT,
			   i2s_priv->format << OUT_REG_FMT_SFT);
	regmap_update_bits(map, ETDM_OUT4_CON0, REG_CH_NUM_MASK_SFT,
			   (channels - 1) << REG_CH_NUM_SFT);
	regmap_update_bits(map, ETDM_OUT4_CON0, REG_SYNC_MODE_MASK_SFT, 0);

	/* APLL */
	regmap_update_bits(map, ETDM_OUT4_CON0, OUT_REG_RELATCH_DOMAIN_SEL_MASK_SFT,
			   ETDM_RELATCH_SEL_APLL << OUT_REG_RELATCH_DOMAIN_SEL_SFT);
	regmap_update_bits(map, ETDM_OUT4_CON0, OUT_REG_BIT_LENGTH_MASK_SFT,
			   get_etdm_lrck_width(format) << OUT_REG_BIT_LENGTH_SFT);
	regmap_update_bits(map, ETDM_OUT4_CON0, OUT_REG_WORD_LENGTH_MASK_SFT,
			   wlen << OUT_REG_WORD_LENGTH_SFT);

	/* ---etdm cowork --- */
	regmap_update_bits(map, ETDM_4_7_COWORK_CON0, ETDM_OUT4_SLAVE_SEL_MASK_SFT,
			   ETDM_SLAVE_SEL_ETDMIN4_MASTER << ETDM_OUT4_SLAVE_SEL_SFT);
}

/* i2s dai ops */
static int mtk_dai_i2s_config(struct mtk_base_afe *afe,
			      struct snd_pcm_hw_params *params,
			      int i2s_id)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	struct mtk_afe_i2s_priv *i2s_priv = afe_priv->dai_priv[i2s_id];
	unsigned int rate = params_rate(params);
	snd_pcm_format_t format = params_format(params);
	unsigned int channels = params_channels(params);

	if (!i2s_priv)
		return -EINVAL;

	dev_dbg(afe->dev, "id: %d, rate: %d, pcm_fmt: %d, fmt: %d, ch: %d\n",
		i2s_id, rate, format, i2s_priv->format, channels);

	i2s_priv->rate = rate;

	switch (i2s_id) {
	case MT6878_DAI_I2S_IN4:
		mtk_dai_etdm_in4_config(afe, i2s_priv, rate, format, channels);
		break;
	case MT6878_DAI_I2S_OUT4:
		mtk_dai_etdm_out4_config(afe, i2s_priv, rate, format, channels);
		break;
	default:
		return -EINVAL;
	}

	/* the clock master of a cowork pair runs with the same setup */
	if (i2s_priv->share_i2s_id >= 0) {
		struct mtk_afe_i2s_priv *share_priv =
			afe_priv->dai_priv[i2s_priv->share_i2s_id];

		share_priv->format = i2s_priv->format;
		return mtk_dai_i2s_config(afe, params, i2s_priv->share_i2s_id);
	}

	return 0;
}

static int mtk_dai_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);

	return mtk_dai_i2s_config(afe, params, dai->id);
}

static int mtk_dai_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	struct mtk_afe_i2s_priv *i2s_priv = afe_priv->dai_priv[dai->id];

	if (!i2s_priv)
		return -EINVAL;

	/* the AFE provides BCK/LRCK */
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BP_FP)
		return -EINVAL;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		i2s_priv->format = MTK_DAI_ETDM_FORMAT_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		i2s_priv->format = MTK_DAI_ETDM_FORMAT_LJ;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		i2s_priv->format = MTK_DAI_ETDM_FORMAT_RJ;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		i2s_priv->format = MTK_DAI_ETDM_FORMAT_DSPA;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		i2s_priv->format = MTK_DAI_ETDM_FORMAT_DSPB;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops mtk_dai_i2s_ops = {
	.hw_params = mtk_dai_i2s_hw_params,
	.set_fmt = mtk_dai_i2s_set_fmt,
};

/* dai driver */
#define MTK_ETDM_RATES (SNDRV_PCM_RATE_8000_192000)
#define MTK_ETDM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			  SNDRV_PCM_FMTBIT_S24_LE |\
			  SNDRV_PCM_FMTBIT_S32_LE)

#define MT6878_I2S_DAI(_name, _id, max_ch, dir) \
{ \
	.name = #_name, \
	.id = _id, \
	.dir = { \
		.stream_name = #_name, \
		.channels_min = 1, \
		.channels_max = max_ch, \
		.rates = MTK_ETDM_RATES, \
		.formats = MTK_ETDM_FORMATS, \
	}, \
	.ops = &mtk_dai_i2s_ops, \
}

static struct snd_soc_dai_driver mtk_dai_i2s_driver[] = {
	MT6878_I2S_DAI(I2SIN4, MT6878_DAI_I2S_IN4, 2, capture),
	MT6878_I2S_DAI(I2SOUT4, MT6878_DAI_I2S_OUT4, 2, playback),
};

int mt6878_dai_i2s_register(struct mtk_base_afe *afe)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	struct mtk_afe_i2s_priv *in4, *out4;
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	in4 = devm_kzalloc(afe->dev, sizeof(*in4), GFP_KERNEL);
	out4 = devm_kzalloc(afe->dev, sizeof(*out4), GFP_KERNEL);
	if (!dai || !in4 || !out4)
		return -ENOMEM;

	/* I2SOUT4 always runs from the I2SIN4 clocks (pinmux of the board) */
	in4->share_i2s_id = -1;
	out4->share_i2s_id = MT6878_DAI_I2S_IN4;
	afe_priv->dai_priv[MT6878_DAI_I2S_IN4] = in4;
	afe_priv->dai_priv[MT6878_DAI_I2S_OUT4] = out4;

	dai->dai_drivers = mtk_dai_i2s_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mtk_dai_i2s_driver);
	dai->dapm_widgets = mtk_dai_i2s_widgets;
	dai->num_dapm_widgets = ARRAY_SIZE(mtk_dai_i2s_widgets);
	dai->dapm_routes = mtk_dai_i2s_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mtk_dai_i2s_routes);

	list_add(&dai->list, &afe->sub_dais);

	return 0;
}
