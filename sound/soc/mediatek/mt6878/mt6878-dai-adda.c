// SPDX-License-Identifier: GPL-2.0
/*
 *  MediaTek ALSA SoC Audio DAI ADDA Control
 *
 *  ADDA is the AFE side of the MTKAIF link to the PMIC codec (MT6369):
 *  the DL SRC feeds the codec DACs, the UL SRC takes its ADCs. The
 *  sequences are those of the MediaTek downstream MT6878 driver; the structure follows
 *  the MT8196 driver. DMIC, the second uplink, sidetone and hi-res rates
 *  are left out.
 *
 *  Copyright (c) 2023 MediaTek Inc.
 *  Author: Shu-wei Hsu <Shu-wei.Hsu@mediatek.com>
 *  Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 */

#include <linux/delay.h>
#include <linux/regmap.h>

#include "mt6878-afe-clk.h"
#include "mt6878-afe-common.h"
#include "mt6878-interconnection.h"

/* -0.3 dB on the DL path, as the MediaTek downstream driver */
#define MT6878_ADDA_DL_GAIN_NORMAL	0xf74f
#define SDM_AUTO_RESET_THRESHOLD	0x190000

enum {
	UL_IIR_SW = 0,
	UL_IIR_5HZ,
	UL_IIR_10HZ,
	UL_IIR_25HZ,
	UL_IIR_50HZ,
	UL_IIR_75HZ,
};

enum {
	AUDIO_SDM_LEVEL_MUTE = 0,
	AUDIO_SDM_LEVEL_NORMAL = 0x1d,
};

enum {
	AUDIO_SDM_2ND = 0,
	AUDIO_SDM_3RD,
};

enum {
	DELAY_DATA_MISO1 = 0,
	DELAY_DATA_MISO2,
};

enum {
	MTK_AFE_ADDA_DL_RATE_8K = 0,
	MTK_AFE_ADDA_DL_RATE_11K = 1,
	MTK_AFE_ADDA_DL_RATE_12K = 2,
	MTK_AFE_ADDA_DL_RATE_16K = 4,
	MTK_AFE_ADDA_DL_RATE_22K = 5,
	MTK_AFE_ADDA_DL_RATE_24K = 6,
	MTK_AFE_ADDA_DL_RATE_32K = 8,
	MTK_AFE_ADDA_DL_RATE_44K = 9,
	MTK_AFE_ADDA_DL_RATE_48K = 10,
};

enum {
	MTK_AFE_ADDA_UL_RATE_8K = 0,
	MTK_AFE_ADDA_UL_RATE_16K = 1,
	MTK_AFE_ADDA_UL_RATE_32K = 2,
	MTK_AFE_ADDA_UL_RATE_48K = 3,
};

struct mtk_afe_adda_priv {
	int dl_rate;
	int ul_rate;
};

static unsigned int adda_dl_rate_transform(struct mtk_base_afe *afe,
					   unsigned int rate)
{
	switch (rate) {
	case 8000:
		return MTK_AFE_ADDA_DL_RATE_8K;
	case 11025:
		return MTK_AFE_ADDA_DL_RATE_11K;
	case 12000:
		return MTK_AFE_ADDA_DL_RATE_12K;
	case 16000:
		return MTK_AFE_ADDA_DL_RATE_16K;
	case 22050:
		return MTK_AFE_ADDA_DL_RATE_22K;
	case 24000:
		return MTK_AFE_ADDA_DL_RATE_24K;
	case 32000:
		return MTK_AFE_ADDA_DL_RATE_32K;
	case 44100:
		return MTK_AFE_ADDA_DL_RATE_44K;
	case 48000:
		return MTK_AFE_ADDA_DL_RATE_48K;
	default:
		dev_warn(afe->dev, "rate %d invalid, use 48kHz!!!\n", rate);
		return MTK_AFE_ADDA_DL_RATE_48K;
	}
}

static unsigned int adda_ul_rate_transform(struct mtk_base_afe *afe,
					   unsigned int rate)
{
	switch (rate) {
	case 8000:
		return MTK_AFE_ADDA_UL_RATE_8K;
	case 16000:
		return MTK_AFE_ADDA_UL_RATE_16K;
	case 32000:
		return MTK_AFE_ADDA_UL_RATE_32K;
	case 48000:
		return MTK_AFE_ADDA_UL_RATE_48K;
	default:
		dev_warn(afe->dev, "rate %d invalid, use 48kHz!!!\n", rate);
		return MTK_AFE_ADDA_UL_RATE_48K;
	}
}

/* dai component */
static const struct snd_kcontrol_new mtk_adda_dl_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("DL0_CH1", AFE_CONN014_1, I_DL0_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL1_CH1", AFE_CONN014_1, I_DL1_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL2_CH1", AFE_CONN014_1, I_DL2_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL3_CH1", AFE_CONN014_1, I_DL3_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL4_CH1", AFE_CONN014_1, I_DL4_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL5_CH1", AFE_CONN014_1, I_DL5_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL6_CH1", AFE_CONN014_1, I_DL6_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL7_CH1", AFE_CONN014_1, I_DL7_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL8_CH1", AFE_CONN014_1, I_DL8_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL_24CH_CH1", AFE_CONN014_1, I_DL_24CH_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL24_CH1", AFE_CONN014_2, I_DL24_CH1, 1, 0),
};

static const struct snd_kcontrol_new mtk_adda_dl_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("DL0_CH1", AFE_CONN015_1, I_DL0_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL0_CH2", AFE_CONN015_1, I_DL0_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL1_CH2", AFE_CONN015_1, I_DL1_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL2_CH2", AFE_CONN015_1, I_DL2_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL3_CH2", AFE_CONN015_1, I_DL3_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL4_CH2", AFE_CONN015_1, I_DL4_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL5_CH2", AFE_CONN015_1, I_DL5_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL6_CH2", AFE_CONN015_1, I_DL6_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL7_CH2", AFE_CONN015_1, I_DL7_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL8_CH2", AFE_CONN015_1, I_DL8_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL_24CH_CH2", AFE_CONN015_1, I_DL_24CH_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL24_CH2", AFE_CONN015_2, I_DL24_CH2, 1, 0),
};

enum {
	SUPPLY_SEQ_ADDA_AFE_ON,
	SUPPLY_SEQ_ADDA_CG,
	SUPPLY_SEQ_ADDA_DL_ON,
	SUPPLY_SEQ_ADDA_AUD_PAD_TOP,
	SUPPLY_SEQ_ADDA_MTKAIF_CFG,
	SUPPLY_SEQ_ADDA_UL_ON,
};

static int mtk_adda_sleep_on_pmd_event(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kcontrol,
				       int event)
{
	/* should delayed 1/fs(smallest is 8k) = 125us before afe off */
	if (event == SND_SOC_DAPM_POST_PMD)
		usleep_range(125, 135);

	return 0;
}

static int mtk_adda_pad_top_event(struct snd_soc_dapm_widget *w,
				  struct snd_kcontrol *kcontrol,
				  int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6878_afe_private *afe_priv = afe->platform_priv;

	if (event != SND_SOC_DAPM_PRE_PMU)
		return 0;

	if (afe_priv->mtkaif_protocol == MTKAIF_PROTOCOL_2_CLK_P2)
		regmap_write(afe->regmap, AFE_AUD_PAD_TOP_CFG0, 0xb8);
	else
		regmap_write(afe->regmap, AFE_AUD_PAD_TOP_CFG0, 0xb0);

	return 0;
}

static int mtk_adda_mtkaif_cfg_event(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol,
				     int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	int delay_data;
	int delay_cycle;

	if (event != SND_SOC_DAPM_PRE_PMU)
		return 0;

	switch (afe_priv->mtkaif_protocol) {
	case MTKAIF_PROTOCOL_2_CLK_P2:
		/* set protocol 2 */
		regmap_write(afe->regmap, AFE_MTKAIF0_CFG0, 0x00010000);
		regmap_write(afe->regmap, AFE_MTKAIF1_CFG0, 0x00010000);

		/* mtkaif_rxif_clkinv_adc inverse for calibration */
		regmap_update_bits(afe->regmap, AFE_MTKAIF0_CFG0,
				   RG_MTKAIF0_RXIF_CLKINV_MASK_SFT,
				   0x1 << RG_MTKAIF0_RXIF_CLKINV_SFT);
		regmap_update_bits(afe->regmap, AFE_MTKAIF1_CFG0,
				   RG_MTKAIF1_RXIF_CLKINV_ADC_MASK_SFT,
				   0x1 << RG_MTKAIF1_RXIF_CLKINV_ADC_SFT);

		/* align miso0/miso1 only after a successful calibration */
		if (!afe_priv->mtkaif_calibration_ok ||
		    afe_priv->mtkaif_chosen_phase[0] < 0 ||
		    afe_priv->mtkaif_chosen_phase[1] < 0) {
			dev_dbg(afe->dev, "mtkaif not calibrated, no phase delay\n");
			break;
		}

		/* set delay for ch12 to align phase of miso0 and miso1 */
		if (afe_priv->mtkaif_phase_cycle[0] >=
		    afe_priv->mtkaif_phase_cycle[1]) {
			delay_data = DELAY_DATA_MISO1;
			delay_cycle = afe_priv->mtkaif_phase_cycle[0] -
				      afe_priv->mtkaif_phase_cycle[1];
		} else {
			delay_data = DELAY_DATA_MISO2;
			delay_cycle = afe_priv->mtkaif_phase_cycle[1] -
				      afe_priv->mtkaif_phase_cycle[0];
		}

		regmap_update_bits(afe->regmap, AFE_MTKAIF0_RX_CFG2,
				   RG_MTKAIF0_RXIF_DELAY_DATA_MASK_SFT,
				   delay_data << RG_MTKAIF0_RXIF_DELAY_DATA_SFT);
		regmap_update_bits(afe->regmap, AFE_MTKAIF0_RX_CFG2,
				   RG_MTKAIF0_RXIF_DELAY_CYCLE_MASK_SFT,
				   delay_cycle << RG_MTKAIF0_RXIF_DELAY_CYCLE_SFT);
		break;
	case MTKAIF_PROTOCOL_2:
		regmap_write(afe->regmap, AFE_MTKAIF0_CFG0, 0x00010000);
		regmap_write(afe->regmap, AFE_MTKAIF1_CFG0, 0x00010000);
		break;
	default:
		regmap_write(afe->regmap, AFE_MTKAIF0_CFG0, 0x0);
		regmap_write(afe->regmap, AFE_MTKAIF1_CFG0, 0x0);
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget mtk_dai_adda_widgets[] = {
	/* inter-connections */
	SND_SOC_DAPM_MIXER("ADDA_DL_CH1", SND_SOC_NOPM, 0, 0,
			   mtk_adda_dl_ch1_mix,
			   ARRAY_SIZE(mtk_adda_dl_ch1_mix)),
	SND_SOC_DAPM_MIXER("ADDA_DL_CH2", SND_SOC_NOPM, 0, 0,
			   mtk_adda_dl_ch2_mix,
			   ARRAY_SIZE(mtk_adda_dl_ch2_mix)),

	SND_SOC_DAPM_SUPPLY_S("ADDA Enable", SUPPLY_SEQ_ADDA_AFE_ON,
			      AUDIO_ENGEN_CON0, AUDIO_F3P25M_EN_ON_SFT, 0,
			      NULL, 0),

	/* audiosys clock gates (power-down bits) */
	SND_SOC_DAPM_SUPPLY_S("ADDA_DL0_DAC_CG", SUPPLY_SEQ_ADDA_CG,
			      AUDIO_TOP_CON0, PDN_DL0_DAC_SFT, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("ADDA_DL0_PREDIS_CG", SUPPLY_SEQ_ADDA_CG,
			      AUDIO_TOP_CON0, PDN_DL0_PREDIS_SFT, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("ADDA_UL0_ADC_CG", SUPPLY_SEQ_ADDA_CG,
			      AUDIO_TOP_CON1, PDN_UL0_ADC_SFT, 1, NULL, 0),

	SND_SOC_DAPM_SUPPLY_S("ADDA Playback Enable", SUPPLY_SEQ_ADDA_DL_ON,
			      AFE_ADDA_DL_SRC_CON0,
			      AFE_DL_SRC_ON_TMP_CTL_PRE_SFT, 0,
			      mtk_adda_sleep_on_pmd_event,
			      SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("ADDA Capture Enable", SUPPLY_SEQ_ADDA_UL_ON,
			      AFE_ADDA_UL0_SRC_CON0,
			      UL_SRC_ON_TMP_CTL_SFT, 0,
			      mtk_adda_sleep_on_pmd_event,
			      SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY_S("AUD_PAD_TOP", SUPPLY_SEQ_ADDA_AUD_PAD_TOP,
			      AFE_AUD_PAD_TOP_CFG0, RG_RX_FIFO_ON_SFT, 0,
			      mtk_adda_pad_top_event,
			      SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_SUPPLY_S("ADDA_MTKAIF_CFG", SUPPLY_SEQ_ADDA_MTKAIF_CFG,
			      SND_SOC_NOPM, 0, 0,
			      mtk_adda_mtkaif_cfg_event,
			      SND_SOC_DAPM_PRE_PMU),
};

static const struct snd_soc_dapm_route mtk_dai_adda_routes[] = {
	/* playback */
	{"ADDA_DL_CH1", "DL0_CH1", "DL0"},
	{"ADDA_DL_CH2", "DL0_CH1", "DL0"},
	{"ADDA_DL_CH2", "DL0_CH2", "DL0"},

	{"ADDA_DL_CH1", "DL1_CH1", "DL1"},
	{"ADDA_DL_CH2", "DL1_CH2", "DL1"},

	{"ADDA_DL_CH1", "DL2_CH1", "DL2"},
	{"ADDA_DL_CH2", "DL2_CH2", "DL2"},

	{"ADDA_DL_CH1", "DL3_CH1", "DL3"},
	{"ADDA_DL_CH2", "DL3_CH2", "DL3"},

	{"ADDA_DL_CH1", "DL4_CH1", "DL4"},
	{"ADDA_DL_CH2", "DL4_CH2", "DL4"},

	{"ADDA_DL_CH1", "DL5_CH1", "DL5"},
	{"ADDA_DL_CH2", "DL5_CH2", "DL5"},

	{"ADDA_DL_CH1", "DL6_CH1", "DL6"},
	{"ADDA_DL_CH2", "DL6_CH2", "DL6"},

	{"ADDA_DL_CH1", "DL7_CH1", "DL7"},
	{"ADDA_DL_CH2", "DL7_CH2", "DL7"},

	{"ADDA_DL_CH1", "DL8_CH1", "DL8"},
	{"ADDA_DL_CH2", "DL8_CH2", "DL8"},

	{"ADDA_DL_CH1", "DL_24CH_CH1", "DL_24CH"},
	{"ADDA_DL_CH2", "DL_24CH_CH2", "DL_24CH"},

	{"ADDA_DL_CH1", "DL24_CH1", "DL24"},
	{"ADDA_DL_CH2", "DL24_CH2", "DL24"},

	{"ADDA Playback", NULL, "ADDA_DL_CH1"},
	{"ADDA Playback", NULL, "ADDA_DL_CH2"},

	{"ADDA Playback", NULL, "ADDA Enable"},
	{"ADDA Playback", NULL, "ADDA_DL0_DAC_CG"},
	{"ADDA Playback", NULL, "ADDA_DL0_PREDIS_CG"},
	{"ADDA Playback", NULL, "ADDA Playback Enable"},
	{"ADDA Playback", NULL, "AUD_PAD_TOP"},

	/* capture */
	{"ADDA Capture", NULL, "ADDA Enable"},
	{"ADDA Capture", NULL, "ADDA_UL0_ADC_CG"},
	{"ADDA Capture", NULL, "ADDA Capture Enable"},
	{"ADDA Capture", NULL, "AUD_PAD_TOP"},
	{"ADDA Capture", NULL, "ADDA_MTKAIF_CFG"},
};

/* dai ops */
static int mtk_dai_adda_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	struct mtk_afe_adda_priv *adda_priv = afe_priv->dai_priv[dai->id];
	unsigned int rate = params_rate(params);

	if (!adda_priv)
		return -EINVAL;

	dev_dbg(afe->dev, "id %d, stream %d, rate %d\n",
		dai->id, substream->stream, rate);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		unsigned int dl_src_con0;
		unsigned int dl_src_con1;

		adda_priv->dl_rate = rate;

		/* set sampling rate */
		dl_src_con0 = adda_dl_rate_transform(afe, rate) <<
			      AFE_DL_INPUT_MODE_CTL_SFT;

		/* set output mode, UP_SAMPLING_RATE_X8 */
		dl_src_con0 |= (0x3 << AFE_DL_OUTPUT_SEL_CTL_SFT);

		/* turn off mute function */
		dl_src_con0 |= (0x01 << AFE_DL_MUTE_CH2_OFF_CTL_PRE_SFT);
		dl_src_con0 |= (0x01 << AFE_DL_MUTE_CH1_OFF_CTL_PRE_SFT);

		/* set voice input data if input sample rate is 8k or 16k */
		if (rate == 8000 || rate == 16000)
			dl_src_con0 |= 0x01 << AFE_DL_VOICE_MODE_CTL_PRE_SFT;

		/* SA suggest apply -0.3db to audio/speech path */
		dl_src_con1 = MT6878_ADDA_DL_GAIN_NORMAL <<
			      AFE_DL_GAIN1_CTL_PRE_SFT;
		dl_src_con1 |= MT6878_ADDA_DL_GAIN_NORMAL <<
			       AFE_DL_GAIN2_CTL_PRE_SFT;

		/* turn on down-link gain */
		dl_src_con0 |= (0x01 << AFE_DL_GAIN_ON_CTL_PRE_SFT);

		/* clean predistortion */
		regmap_write(afe->regmap, AFE_ADDA_DL_PREDIS_CON0, 0);
		regmap_write(afe->regmap, AFE_ADDA_DL_PREDIS_CON1, 0);

		regmap_write(afe->regmap, AFE_ADDA_DL_SRC_CON0, dl_src_con0);
		regmap_write(afe->regmap, AFE_ADDA_DL_SRC_CON1, dl_src_con1);

		/* set sdm gain */
		regmap_update_bits(afe->regmap, AFE_ADDA_DL_SDM_DCCOMP_CON,
				   AFE_DL_ATTGAIN_CTL_MASK_SFT,
				   AUDIO_SDM_LEVEL_NORMAL << AFE_DL_ATTGAIN_CTL_SFT);

		/* 2nd sdm */
		regmap_update_bits(afe->regmap, AFE_ADDA_DL_SDM_DCCOMP_CON,
				   AFE_DL_USE_3RD_SDM_MASK_SFT,
				   AUDIO_SDM_2ND << AFE_DL_USE_3RD_SDM_SFT);

		/* sdm auto reset */
		regmap_write(afe->regmap, AFE_ADDA_DL_SDM_AUTO_RESET_CON,
			     SDM_AUTO_RESET_THRESHOLD);
		regmap_update_bits(afe->regmap, AFE_ADDA_DL_SDM_AUTO_RESET_CON,
				   AFE_DL_SDM_AUTO_RESET_TEST_ON_MASK_SFT,
				   0x1 << AFE_DL_SDM_AUTO_RESET_TEST_ON_SFT);
	} else {
		unsigned int ul_src_con0 = 0;

		adda_priv->ul_rate = rate;

		ul_src_con0 |= (adda_ul_rate_transform(afe, rate) << 17) &
			       (0x7 << 17);

		/* enable iir */
		ul_src_con0 |= (1 << UL_IIR_ON_TMP_CTL_SFT) &
			       UL_IIR_ON_TMP_CTL_MASK_SFT;
		ul_src_con0 |= (UL_IIR_SW << UL_IIRMODE_CTL_SFT) &
			       UL_IIRMODE_CTL_MASK_SFT;

		/* 35Hz @ 48k */
		regmap_write(afe->regmap, AFE_ADDA_UL0_IIR_COEF_02_01, 0x00000000);
		regmap_write(afe->regmap, AFE_ADDA_UL0_IIR_COEF_04_03, 0x00003FB8);
		regmap_write(afe->regmap, AFE_ADDA_UL0_IIR_COEF_06_05, 0x3FB80000);
		regmap_write(afe->regmap, AFE_ADDA_UL0_IIR_COEF_08_07, 0x3FB80000);
		regmap_write(afe->regmap, AFE_ADDA_UL0_IIR_COEF_10_09, 0x0000C048);

		regmap_write(afe->regmap, AFE_ADDA_UL0_SRC_CON0, ul_src_con0);

		/* mtkaif_rxif_data_mode = 0, amic */
		regmap_update_bits(afe->regmap, AFE_MTKAIF0_RX_CFG0, 0x1, 0x0);
	}

	return 0;
}

static const struct snd_soc_dai_ops mtk_dai_adda_ops = {
	.hw_params = mtk_dai_adda_hw_params,
};

/* dai driver */
#define MTK_ADDA_PLAYBACK_RATES (SNDRV_PCM_RATE_8000_48000)

#define MTK_ADDA_CAPTURE_RATES (SNDRV_PCM_RATE_8000 |\
				SNDRV_PCM_RATE_16000 |\
				SNDRV_PCM_RATE_32000 |\
				SNDRV_PCM_RATE_48000)

#define MTK_ADDA_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			  SNDRV_PCM_FMTBIT_S24_LE |\
			  SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver mtk_dai_adda_driver[] = {
	{
		.name = "ADDA",
		.id = MT6878_DAI_ADDA,
		.playback = {
			.stream_name = "ADDA Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_ADDA_PLAYBACK_RATES,
			.formats = MTK_ADDA_FORMATS,
		},
		.capture = {
			.stream_name = "ADDA Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_ADDA_CAPTURE_RATES,
			.formats = MTK_ADDA_FORMATS,
		},
		.ops = &mtk_dai_adda_ops,
	},
};

int mt6878_dai_adda_register(struct mtk_base_afe *afe)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	struct mtk_base_afe_dai *dai;
	int i;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	dai->dai_drivers = mtk_dai_adda_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mtk_dai_adda_driver);
	dai->dapm_widgets = mtk_dai_adda_widgets;
	dai->num_dapm_widgets = ARRAY_SIZE(mtk_dai_adda_widgets);
	dai->dapm_routes = mtk_dai_adda_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mtk_dai_adda_routes);

	afe_priv->dai_priv[MT6878_DAI_ADDA] =
		devm_kzalloc(afe->dev, sizeof(struct mtk_afe_adda_priv),
			     GFP_KERNEL);
	if (!afe_priv->dai_priv[MT6878_DAI_ADDA])
		return -ENOMEM;

	/* not calibrated until the machine driver did it */
	for (i = 0; i < MT6878_MTKAIF_MISO_NUM; i++) {
		afe_priv->mtkaif_chosen_phase[i] = -1;
		afe_priv->mtkaif_phase_cycle[i] = 0;
	}

	list_add(&dai->list, &afe->sub_dais);

	return 0;
}
