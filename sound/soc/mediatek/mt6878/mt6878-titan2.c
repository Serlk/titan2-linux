// SPDX-License-Identifier: GPL-2.0
/*
 *  mt6878-titan2.c  --  MT6878 ALSA SoC machine driver for the Unihertz Titan 2
 *
 *  MT6369 PMIC codec on ADDA (receiver, headset over USB-C, microphones)
 *  and a FourSemi FS199x amplifier on I2SOUT4 (loudspeaker). The codec
 *  links and the audio routing come from the devicetree. The MTKAIF
 *  calibration is the one of the MediaTek downstream MT6878 machine driver, in the form
 *  of the mainline mt8195-mt6359 driver.
 *
 *  Copyright (c) 2023 MediaTek Inc.
 *  Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "mt6878-afe-common.h"

#include "../../codecs/mt6369.h"
#include "../common/mtk-afe-platform-driver.h"
#include "../common/mtk-soc-card.h"
#include "../common/mtk-soundcard-driver.h"

/* topckgen audio test registers used by the MTKAIF calibration */
#define CKSYS_AUD_TOP_CFG	0x032c
#define CKSYS_AUD_TOP_MON	0x0330

#define MT6878_MTKAIF_CALIB_PHASES	42	/* phase steps of the codec */

static const struct snd_soc_dapm_widget mt6878_titan2_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_SPK("Earpiece", NULL),
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
	SND_SOC_DAPM_MIC("Second Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
};

static const struct snd_kcontrol_new mt6878_titan2_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Earpiece"),
	SOC_DAPM_PIN_SWITCH("Headphones"),
	SOC_DAPM_PIN_SWITCH("Main Mic"),
	SOC_DAPM_PIN_SWITCH("Second Mic"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
};

static void mt6878_mtkaif_calibration(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt_afe =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct snd_soc_component *cmpnt_codec =
		snd_soc_rtd_to_codec(rtd, 0)->component;
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt_afe);
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	int prev_cycle[2] = { -1, -1 }, cycle[2];
	int test_done[2];
	unsigned int monitor = 0;
	bool ok = true;
	int counter, phase, i;

	afe_priv->mtkaif_calibration_ok = false;
	for (i = 0; i < MT6878_MTKAIF_MISO_NUM; i++) {
		afe_priv->mtkaif_chosen_phase[i] = -1;
		afe_priv->mtkaif_phase_cycle[i] = 0;
	}

	if (!afe_priv->topckgen) {
		dev_info(afe->dev, "no topckgen, MTKAIF not calibrated\n");
		return;
	}

	if (pm_runtime_resume_and_get(afe->dev))
		return;

	mt6369_mtkaif_calibration_enable(cmpnt_codec);

	/* set clock protocol 2 */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP_CFG0, 0xff, 0xb8);
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP_CFG0, 0xff, 0xb9);

	/* set test type to synchronizer pulse */
	regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0xffff, 0x4);
	afe_priv->mtkaif_calibration_num_phase = MT6878_MTKAIF_CALIB_PHASES;

	for (phase = 0; phase <= MT6878_MTKAIF_CALIB_PHASES && ok; phase++) {
		mt6369_set_mtkaif_calibration_phase(cmpnt_codec,
						    phase, phase, phase);

		regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0x1, 0x1);

		test_done[0] = 0;
		test_done[1] = 0;
		cycle[0] = -1;
		cycle[1] = -1;
		counter = 0;
		while (!test_done[0] || !test_done[1]) {
			regmap_read(afe_priv->topckgen, CKSYS_AUD_TOP_MON,
				    &monitor);

			test_done[0] = (monitor >> 28) & 0x1;
			test_done[1] = (monitor >> 29) & 0x1;
			if (test_done[0])
				cycle[0] = monitor & 0xf;
			if (test_done[1])
				cycle[1] = (monitor >> 4) & 0xf;

			/* handle if never test done */
			if (++counter > 10000) {
				dev_warn(afe->dev,
					 "MTKAIF test fail, cycle %d/%d, monitor 0x%x\n",
					 cycle[0], cycle[1], monitor);
				ok = false;
				break;
			}
		}

		if (phase == 0) {
			prev_cycle[0] = cycle[0];
			prev_cycle[1] = cycle[1];
		}

		for (i = 0; i < 2; i++) {
			if (cycle[i] != prev_cycle[i] &&
			    afe_priv->mtkaif_chosen_phase[i] < 0) {
				afe_priv->mtkaif_chosen_phase[i] = phase - 1;
				afe_priv->mtkaif_phase_cycle[i] = prev_cycle[i];
			}
		}

		regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0x1, 0x0);
	}

	mt6369_set_mtkaif_calibration_phase(cmpnt_codec,
		max(afe_priv->mtkaif_chosen_phase[0], 0),
		max(afe_priv->mtkaif_chosen_phase[1], 0),
		max(afe_priv->mtkaif_chosen_phase[2], 0));

	/* disable rx fifo */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP_CFG0, 0xff, 0xb8);

	mt6369_mtkaif_calibration_disable(cmpnt_codec);

	pm_runtime_put(afe->dev);

	afe_priv->mtkaif_calibration_ok = ok &&
		afe_priv->mtkaif_chosen_phase[0] >= 0 &&
		afe_priv->mtkaif_chosen_phase[1] >= 0;

	dev_info(afe->dev, "MTKAIF calibration %s, phase %d/%d, cycle %d/%d\n",
		 afe_priv->mtkaif_calibration_ok ? "ok" : "failed",
		 afe_priv->mtkaif_chosen_phase[0],
		 afe_priv->mtkaif_chosen_phase[1],
		 afe_priv->mtkaif_phase_cycle[0],
		 afe_priv->mtkaif_phase_cycle[1]);
}

static int mt6878_titan2_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt_afe =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct snd_soc_component *cmpnt_codec =
		snd_soc_rtd_to_codec(rtd, 0)->component;
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt_afe);
	struct mt6878_afe_private *afe_priv = afe->platform_priv;

	/* as the vendor platform: protocol 2, clock phase 2 */
	mt6369_set_mtkaif_protocol(cmpnt_codec, MT6369_MTKAIF_PROTOCOL_2_CLK_P2);
	afe_priv->mtkaif_protocol = MTKAIF_PROTOCOL_2_CLK_P2;

	mt6878_mtkaif_calibration(rtd);

	return 0;
}

/* the amplifier PLL takes 16..96 kHz at 2 x 32 bit slots */
static const unsigned int mt6878_titan2_spk_rates[] = {
	16000, 32000, 44100, 48000, 88200, 96000,
};

static const struct snd_pcm_hw_constraint_list mt6878_titan2_spk_rate_list = {
	.count = ARRAY_SIZE(mt6878_titan2_spk_rates),
	.list = mt6878_titan2_spk_rates,
};

static int mt6878_titan2_spk_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
					  SNDRV_PCM_HW_PARAM_RATE,
					  &mt6878_titan2_spk_rate_list);
}

static const struct snd_soc_ops mt6878_titan2_spk_ops = {
	.startup = mt6878_titan2_spk_startup,
};

/* the amplifier link always runs 2 x 32 bit slots */
static int mt6878_titan2_i2s_fixup(struct snd_soc_pcm_runtime *rtd,
				   struct snd_pcm_hw_params *params)
{
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	snd_mask_reset_range(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
			     0, (__force unsigned int)SNDRV_PCM_FORMAT_LAST);
	params_set_format(params, SNDRV_PCM_FORMAT_S32_LE);
	channels->min = 2;
	channels->max = 2;

	return 0;
}

/* FE */
SND_SOC_DAILINK_DEFS(playback0,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL0")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(playback1,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture0,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL0")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture1,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));
/* BE */
SND_SOC_DAILINK_DEFS(adda,
		     DAILINK_COMP_ARRAY(COMP_CPU("ADDA")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(i2sout4,
		     DAILINK_COMP_ARRAY(COMP_CPU("I2SOUT4")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(i2sin4,
		     DAILINK_COMP_ARRAY(COMP_CPU("I2SIN4")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link mt6878_titan2_dai_links[] = {
	/* Front End DAI links */
	{
		.name = "DL0_FE",
		.stream_name = "Codec Playback",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(playback0),
	},
	{
		.name = "DL1_FE",
		.stream_name = "Speaker Playback",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.playback_only = 1,
		.ops = &mt6878_titan2_spk_ops,
		SND_SOC_DAILINK_REG(playback1),
	},
	{
		.name = "UL0_FE",
		.stream_name = "Mic Capture",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.capture_only = 1,
		SND_SOC_DAILINK_REG(capture0),
	},
	{
		.name = "UL1_FE",
		.stream_name = "I2SIN4 Capture",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.capture_only = 1,
		SND_SOC_DAILINK_REG(capture1),
	},
	/* Back End DAI links */
	{
		.name = "ADDA_BE",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.init = mt6878_titan2_codec_init,
		SND_SOC_DAILINK_REG(adda),
	},
	{
		.name = "I2SOUT4_BE",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC,
		.no_pcm = 1,
		.playback_only = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6878_titan2_i2s_fixup,
		SND_SOC_DAILINK_REG(i2sout4),
	},
	{
		.name = "I2SIN4_BE",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC,
		.no_pcm = 1,
		.capture_only = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6878_titan2_i2s_fixup,
		SND_SOC_DAILINK_REG(i2sin4),
	},
};

static int mt6878_titan2_soc_card_probe(struct mtk_soc_card_data *soc_card_data,
					bool legacy)
{
	struct snd_soc_card *card = soc_card_data->card_data->card;

	/* the codec links and the routing must come from the devicetree */
	if (legacy)
		return dev_err_probe(card->dev, -EINVAL,
				     "audio-routing missing\n");

	return 0;
}

static struct snd_soc_card mt6878_titan2_soc_card = {
	.owner = THIS_MODULE,
	.dai_link = mt6878_titan2_dai_links,
	.num_links = ARRAY_SIZE(mt6878_titan2_dai_links),
	.dapm_widgets = mt6878_titan2_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6878_titan2_widgets),
	.controls = mt6878_titan2_controls,
	.num_controls = ARRAY_SIZE(mt6878_titan2_controls),
};

static const struct mtk_soundcard_pdata mt6878_titan2_card = {
	.card_name = "mt6878_titan2",
	.card_data = &(struct mtk_platform_card_data) {
		.card = &mt6878_titan2_soc_card,
	},
	.soc_probe = mt6878_titan2_soc_card_probe,
};

static const struct of_device_id mt6878_titan2_dt_match[] = {
	{ .compatible = "unihertz,titan2-sound", .data = &mt6878_titan2_card },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6878_titan2_dt_match);

static struct platform_driver mt6878_titan2_driver = {
	.driver = {
		.name = "mt6878-titan2",
		.of_match_table = mt6878_titan2_dt_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = mtk_soundcard_common_probe,
};
module_platform_driver(mt6878_titan2_driver);

MODULE_DESCRIPTION("MT6878 Unihertz Titan 2 ALSA SoC machine driver");
MODULE_LICENSE("GPL");
