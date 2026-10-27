// SPDX-License-Identifier: GPL-2.0
/*
 *  MediaTek ALSA SoC Audio DAI Hostless Control
 *
 *  "Hostless Speech": a front end without a memory interface. Opening it
 *  (playback and capture) brings up the back ends of a voice call, PCM 1
 *  to and from the modem and ADDA or I2SOUT4 to the codec or the speaker
 *  amplifier, with the rate of the front end. The audio itself only flows
 *  through the interconnect switches set by the mixer controls. Same
 *  scheme as the mainline MT8183 and MT6797 drivers.
 *
 *  Copyright (c) 2023 MediaTek Inc.
 *  Author: Shu-wei Hsu <Shu-wei.Hsu@mediatek.com>
 *  Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 */

#include "mt6878-afe-common.h"

/* dai component */
static const struct snd_soc_dapm_route mtk_dai_hostless_routes[] = {
	/* modem downlink to the earpiece/headset or the loudspeaker */
	{"ADDA_DL_CH1", "PCM_1_CAP_CH1", "Hostless Speech DL"},
	{"ADDA_DL_CH2", "PCM_1_CAP_CH1", "Hostless Speech DL"},
	{"ADDA_DL_CH2", "PCM_1_CAP_CH2", "Hostless Speech DL"},
	{"I2SOUT4_CH1", "PCM_1_CAP_CH1", "Hostless Speech DL"},
	{"I2SOUT4_CH2", "PCM_1_CAP_CH1", "Hostless Speech DL"},

	/* microphones to the modem uplink */
	{"PCM_1_PB_CH1", "ADDA_UL_CH1", "Hostless Speech DL"},
	{"PCM_1_PB_CH1", "ADDA_UL_CH2", "Hostless Speech DL"},
	{"PCM_1_PB_CH2", "ADDA_UL_CH1", "Hostless Speech DL"},
	{"PCM_1_PB_CH2", "ADDA_UL_CH2", "Hostless Speech DL"},

	{"Hostless Speech UL", NULL, "PCM 1 Capture"},
	{"Hostless Speech UL", NULL, "ADDA Capture"},
};

/* dai ops */
static int mtk_dai_hostless_startup(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);

	return snd_soc_set_runtime_hwparams(substream, afe->mtk_afe_hardware);
}

static const struct snd_soc_dai_ops mtk_dai_hostless_ops = {
	.startup = mtk_dai_hostless_startup,
};

/* dai driver: the rates PCM 1 can run at */
#define MTK_HOSTLESS_RATES (SNDRV_PCM_RATE_8000 |\
			    SNDRV_PCM_RATE_16000 |\
			    SNDRV_PCM_RATE_32000 |\
			    SNDRV_PCM_RATE_48000)

#define MTK_HOSTLESS_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			      SNDRV_PCM_FMTBIT_S24_LE |\
			      SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver mtk_dai_hostless_driver[] = {
	{
		.name = "Hostless Speech DAI",
		.id = MT6878_DAI_HOSTLESS_SPEECH,
		.playback = {
			.stream_name = "Hostless Speech DL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_HOSTLESS_RATES,
			.formats = MTK_HOSTLESS_FORMATS,
		},
		.capture = {
			.stream_name = "Hostless Speech UL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_HOSTLESS_RATES,
			.formats = MTK_HOSTLESS_FORMATS,
		},
		.ops = &mtk_dai_hostless_ops,
	},
};

int mt6878_dai_hostless_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	dai->dai_drivers = mtk_dai_hostless_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mtk_dai_hostless_driver);
	dai->dapm_routes = mtk_dai_hostless_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mtk_dai_hostless_routes);

	list_add(&dai->list, &afe->sub_dais);

	return 0;
}
