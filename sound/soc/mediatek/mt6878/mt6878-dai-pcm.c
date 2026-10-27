// SPDX-License-Identifier: GPL-2.0
/*
 *  MediaTek ALSA SoC Audio DAI PCM Control
 *
 *  PCM 1 is the in-SoC voice interface of the modem (MD1): during a call
 *  the modem's speech downlink arrives on PCM 1 capture and is routed by
 *  the interconnect to ADDA DL or I2SOUT4; the microphones (ADDA UL) and an
 *  echo reference go back on PCM 1 playback. No sample data passes the
 *  AP. The interface settings are those of the MediaTek downstream MT6878
 *  driver, the structure follows the mainline MT8189 driver.
 *
 *  Copyright (c) 2023 MediaTek Inc.
 *  Author: Tina Tsai <tina.tsai@mediatek.com>
 *  Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 */

#include <linux/regmap.h>

#include <sound/pcm_params.h>

#include "mt6878-afe-common.h"
#include "mt6878-interconnection.h"

enum {
	AUD_PCM_AFIFO_ASRC,
	/* the modem runs from the same crystal */
	AUD_PCM_AFIFO_AFIFO,
};

enum {
	AUD_PCM_WLEN_32_BCK_CYCLES,
	AUD_PCM_WLEN_64_BCK_CYCLES,
};

enum {
	AUD_PCM_FMT_I2S,
	AUD_PCM_FMT_EIAJ,
	AUD_PCM_FMT_PCM_MODE_A,
	AUD_PCM_FMT_PCM_MODE_B,
};

enum {
	AUD_PCM_1X_EN_DOMAIN_HOPPING_26M,
	AUD_PCM_1X_EN_DOMAIN_APLL,
};

/* PCM1_MODE */
enum {
	AUD_PCM_MODE_8K,
	AUD_PCM_MODE_16K,
	AUD_PCM_MODE_32K,
	AUD_PCM_MODE_48K,
};

static int mt6878_pcm_rate(unsigned int rate, unsigned int *mode,
			   unsigned int *x_mode)
{
	switch (rate) {
	case 8000:
		*mode = AUD_PCM_MODE_8K;
		*x_mode = MTK_AFE_IPM2P0_RATE_8K;
		break;
	case 16000:
		*mode = AUD_PCM_MODE_16K;
		*x_mode = MTK_AFE_IPM2P0_RATE_16K;
		break;
	case 32000:
		*mode = AUD_PCM_MODE_32K;
		*x_mode = MTK_AFE_IPM2P0_RATE_32K;
		break;
	case 48000:
		*mode = AUD_PCM_MODE_48K;
		*x_mode = MTK_AFE_IPM2P0_RATE_48K;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* dai component: AP -> modem channels */
static const struct snd_kcontrol_new mtk_pcm_1_playback_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN102_0,
				    I_ADDA_UL_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN102_0,
				    I_ADDA_UL_CH2, 1, 0),
};

static const struct snd_kcontrol_new mtk_pcm_1_playback_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN103_0,
				    I_ADDA_UL_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN103_0,
				    I_ADDA_UL_CH2, 1, 0),
};

/* channels 4/5: echo reference for the modem's echo canceller */
static const struct snd_kcontrol_new mtk_pcm_1_playback_ch4_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("DL0_CH1", AFE_CONN105_1,
				    I_DL0_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("I2SIN4_CH1", AFE_CONN105_4,
				    I_I2SIN4_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("I2SIN4_CH2", AFE_CONN105_4,
				    I_I2SIN4_CH2, 1, 0),
};

static const struct snd_kcontrol_new mtk_pcm_1_playback_ch5_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("DL0_CH2", AFE_CONN106_1,
				    I_DL0_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("I2SIN4_CH2", AFE_CONN106_4,
				    I_I2SIN4_CH2, 1, 0),
};

static const struct snd_soc_dapm_widget mtk_dai_pcm_widgets[] = {
	/* inter-connections */
	SND_SOC_DAPM_MIXER("PCM_1_PB_CH1", SND_SOC_NOPM, 0, 0,
			   mtk_pcm_1_playback_ch1_mix,
			   ARRAY_SIZE(mtk_pcm_1_playback_ch1_mix)),
	SND_SOC_DAPM_MIXER("PCM_1_PB_CH2", SND_SOC_NOPM, 0, 0,
			   mtk_pcm_1_playback_ch2_mix,
			   ARRAY_SIZE(mtk_pcm_1_playback_ch2_mix)),
	SND_SOC_DAPM_MIXER("PCM_1_PB_CH4", SND_SOC_NOPM, 0, 0,
			   mtk_pcm_1_playback_ch4_mix,
			   ARRAY_SIZE(mtk_pcm_1_playback_ch4_mix)),
	SND_SOC_DAPM_MIXER("PCM_1_PB_CH5", SND_SOC_NOPM, 0, 0,
			   mtk_pcm_1_playback_ch5_mix,
			   ARRAY_SIZE(mtk_pcm_1_playback_ch5_mix)),

	SND_SOC_DAPM_SUPPLY("PCM_1_CG", AUDIO_TOP_CON0, PDN_PCM1_SFT, 1,
			    NULL, 0),
	SND_SOC_DAPM_SUPPLY("PCM_1_EN", AFE_PCM1_INTF_CON0, PCM1_EN_SFT, 0,
			    NULL, 0),

	SND_SOC_DAPM_INPUT("MD1_TO_AFE"),
	SND_SOC_DAPM_OUTPUT("AFE_TO_MD1"),
};

static const struct snd_soc_dapm_route mtk_dai_pcm_routes[] = {
	{"PCM 1 Playback", NULL, "PCM_1_PB_CH1"},
	{"PCM 1 Playback", NULL, "PCM_1_PB_CH2"},
	{"PCM 1 Playback", NULL, "PCM_1_PB_CH4"},
	{"PCM 1 Playback", NULL, "PCM_1_PB_CH5"},

	{"PCM_1_EN", NULL, "PCM_1_CG"},
	{"PCM 1 Playback", NULL, "PCM_1_EN"},
	{"PCM 1 Capture", NULL, "PCM_1_EN"},

	{"AFE_TO_MD1", NULL, "PCM 1 Playback"},
	{"PCM 1 Capture", NULL, "MD1_TO_AFE"},

	{"PCM_1_PB_CH4", "DL0_CH1", "DL0"},
	{"PCM_1_PB_CH5", "DL0_CH2", "DL0"},
	{"PCM_1_PB_CH4", "I2SIN4_CH1", "I2SIN4"},
	{"PCM_1_PB_CH4", "I2SIN4_CH2", "I2SIN4"},
	{"PCM_1_PB_CH5", "I2SIN4_CH2", "I2SIN4"},
};

/* dai ops */
static int mtk_dai_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct snd_soc_dapm_widget *p_w =
		snd_soc_dai_get_widget(dai, SNDRV_PCM_STREAM_PLAYBACK);
	struct snd_soc_dapm_widget *c_w =
		snd_soc_dai_get_widget(dai, SNDRV_PCM_STREAM_CAPTURE);
	unsigned int rate = params_rate(params);
	unsigned int mode, x_mode, con0, con1;

	if (mt6878_pcm_rate(rate, &mode, &x_mode)) {
		dev_err(afe->dev, "PCM 1: rate %u not supported\n", rate);
		return -EINVAL;
	}

	dev_dbg(afe->dev, "PCM 1: stream %d, rate %u\n",
		substream->stream, rate);

	/* both directions share the interface: configure it once */
	if ((p_w && p_w->active) || (c_w && c_w->active))
		return 0;

	con0 = 0 << PCM1_TX_LCH_RPT_SFT;	/* no left channel repeat */
	con0 |= 0 << PCM1_VBT_16K_MODE_SFT;
	con0 |= 0 << PCM1_BT_MODE_SFT;		/* dual mic on TX */
	con0 |= AUD_PCM_AFIFO_AFIFO << PCM1_AFIFO_SFT;
	con0 |= AUD_PCM_WLEN_32_BCK_CYCLES << PCM1_WLEN_SFT;
	con0 |= mode << PCM1_MODE_SFT;
	con0 |= AUD_PCM_FMT_PCM_MODE_B << PCM1_FMT_SFT;

	con1 = AUD_PCM_1X_EN_DOMAIN_HOPPING_26M << PCM1_1X_EN_DOMAIN_SFT;
	con1 |= x_mode << PCM1_1X_EN_MODE_SFT;

	/* everything but the enable bit, which is a DAPM supply */
	regmap_update_bits(afe->regmap, AFE_PCM1_INTF_CON0,
			   ~(unsigned int)PCM1_EN_MASK_SFT, con0);
	regmap_update_bits(afe->regmap, AFE_PCM1_INTF_CON1,
			   PCM1_1X_EN_DOMAIN_MASK_SFT |
			   PCM1_1X_EN_MODE_MASK_SFT, con1);

	return 0;
}

static const struct snd_soc_dai_ops mtk_dai_pcm_ops = {
	.hw_params = mtk_dai_pcm_hw_params,
};

/* dai driver */
#define MTK_PCM_RATES (SNDRV_PCM_RATE_8000 |\
		       SNDRV_PCM_RATE_16000 |\
		       SNDRV_PCM_RATE_32000 |\
		       SNDRV_PCM_RATE_48000)

#define MTK_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			 SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver mtk_dai_pcm_driver[] = {
	{
		.name = "PCM 1",
		.id = MT6878_DAI_PCM_1,
		.playback = {
			.stream_name = "PCM 1 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "PCM 1 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mtk_dai_pcm_ops,
		.symmetric_rate = 1,
		.symmetric_sample_bits = 1,
	},
};

int mt6878_dai_pcm_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	dai->dai_drivers = mtk_dai_pcm_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mtk_dai_pcm_driver);
	dai->dapm_widgets = mtk_dai_pcm_widgets;
	dai->num_dapm_widgets = ARRAY_SIZE(mtk_dai_pcm_widgets);
	dai->dapm_routes = mtk_dai_pcm_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mtk_dai_pcm_routes);

	list_add(&dai->list, &afe->sub_dais);

	return 0;
}
