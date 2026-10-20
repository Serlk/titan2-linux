// SPDX-License-Identifier: GPL-2.0
/*
 *  Mediatek ALSA SoC AFE platform driver for 6878
 *
 *  The MT6878 AFE uses the same register layout as MT8196 (IPM 2.0), with
 *  fewer memory interfaces. This file follows the MT8196 driver; the memif,
 *  IRQ and volatile register tables come from the MediaTek downstream MT6878 driver.
 *
 *  Copyright (c) 2023 MediaTek Inc.
 *  Copyright (c) 2025 MediaTek Inc.
 *  Author: Shu-wei Hsu <Shu-wei.Hsu@mediatek.com>
 *  Author: Darren Ye <darren.ye@mediatek.com>
 */

#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <sound/pcm.h>
#include <sound/soc.h>

#include "mt6878-afe-clk.h"
#include "mt6878-afe-common.h"

#include "../common/mtk-afe-fe-dai.h"
#include "../common/mtk-afe-platform-driver.h"

static const struct snd_pcm_hardware mt6878_afe_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_NO_PERIOD_WAKEUP |
		 SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE |
		    SNDRV_PCM_FMTBIT_S24_LE |
		    SNDRV_PCM_FMTBIT_S32_LE),
	.period_bytes_min = 96,
	.period_bytes_max = 4 * 48 * 1024,
	.periods_min = 2,
	.periods_max = 256,
	.buffer_bytes_max = 256 * 1024,
	.fifo_size = 0,
};

static unsigned int mt6878_rate_transform(struct device *dev,
					  unsigned int rate)
{
	switch (rate) {
	case 8000:
		return MTK_AFE_IPM2P0_RATE_8K;
	case 11025:
		return MTK_AFE_IPM2P0_RATE_11K;
	case 12000:
		return MTK_AFE_IPM2P0_RATE_12K;
	case 16000:
		return MTK_AFE_IPM2P0_RATE_16K;
	case 22050:
		return MTK_AFE_IPM2P0_RATE_22K;
	case 24000:
		return MTK_AFE_IPM2P0_RATE_24K;
	case 32000:
		return MTK_AFE_IPM2P0_RATE_32K;
	case 44100:
		return MTK_AFE_IPM2P0_RATE_44K;
	case 48000:
		return MTK_AFE_IPM2P0_RATE_48K;
	case 88200:
		return MTK_AFE_IPM2P0_RATE_88K;
	case 96000:
		return MTK_AFE_IPM2P0_RATE_96K;
	case 176400:
		return MTK_AFE_IPM2P0_RATE_176K;
	case 192000:
		return MTK_AFE_IPM2P0_RATE_192K;
	/* not support 260K */
	case 352800:
		return MTK_AFE_IPM2P0_RATE_352K;
	case 384000:
		return MTK_AFE_IPM2P0_RATE_384K;
	default:
		dev_err(dev, "rate %u invalid, use %d\n",
			rate, MTK_AFE_IPM2P0_RATE_48K);
		return MTK_AFE_IPM2P0_RATE_48K;
	}
}

static int mt6878_fe_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int memif_num = snd_soc_rtd_to_cpu(rtd, 0)->id;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];
	int ret;

	memif->substream = substream;

	snd_pcm_hw_constraint_step(runtime, 0,
				   SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 16);

	if (memif_num == MT6878_MEMIF_VUL_CM0)
		snd_pcm_hw_constraint_step(runtime, 0,
					   SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 16);

	snd_soc_set_runtime_hwparams(substream, afe->mtk_afe_hardware);

	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		dev_warn(afe->dev, "snd_pcm_hw_constraint_integer failed\n");

	/* dynamic allocate irq to memif */
	if (memif->irq_usage < 0) {
		int irq_id = mtk_dynamic_irq_acquire(afe);

		if (irq_id != afe->irqs_size) {
			memif->irq_usage = irq_id;
		} else {
			dev_err(afe->dev, "no more asys irq\n");
			ret = -EBUSY;
		}
	}

	return ret;
}

static void mt6878_fe_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	int memif_num = snd_soc_rtd_to_cpu(rtd, 0)->id;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];

	memif->substream = NULL;

	if (!memif->const_irq) {
		mtk_dynamic_irq_release(afe, memif->irq_usage);
		memif->irq_usage = -1;
	}
}

static int mt6878_fe_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	int id = snd_soc_rtd_to_cpu(rtd, 0)->id;
	const struct mtk_base_memif_data *data = afe->memif[id].data;
	unsigned int channels = params_channels(params);

	/* multi-channel memifs carry the channel count in CON0 */
	if (data->ch_num_shift >= 0)
		regmap_update_bits(afe->regmap, data->ch_num_reg,
				   data->ch_num_maskbit << data->ch_num_shift,
				   channels << data->ch_num_shift);

	return mtk_afe_fe_hw_params(substream, params, dai);
}

static void mt6878_irq_clear(struct mtk_base_afe *afe,
			     const struct mtk_base_irq_data *irq_data)
{
	unsigned int tmp_reg;

	/*
	 * The clear bits toggle: writing the inverse of the current value
	 * clears the pending and miss flags.
	 */
	regmap_read(afe->regmap, irq_data->irq_clr_reg, &tmp_reg);
	regmap_update_bits(afe->regmap, irq_data->irq_clr_reg,
			   AFE_IRQ_CLR_CFG_MASK_SFT |
			   AFE_IRQ_MISS_FLAG_CLR_CFG_MASK_SFT,
			   tmp_reg ^ (AFE_IRQ_CLR_CFG_MASK_SFT |
				      AFE_IRQ_MISS_FLAG_CLR_CFG_MASK_SFT));
}

static int mt6878_fe_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_pcm_runtime *const runtime = substream->runtime;
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	int id = snd_soc_rtd_to_cpu(rtd, 0)->id;
	struct mtk_base_afe_memif *memif = &afe->memif[id];
	struct mtk_base_afe_irq *irqs = &afe->irqs[memif->irq_usage];
	const struct mtk_base_irq_data *irq_data = irqs->irq_data;
	unsigned int counter = runtime->period_size;
	unsigned int rate = runtime->rate;
	int fs;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		ret = mtk_memif_set_enable(afe, id);
		if (ret) {
			dev_err(afe->dev, "id %d, memif enable fail.\n", id);
			return ret;
		}

		/*
		 * For small latency record the UL memif needs to read some
		 * data before the irq is enabled. The trigger runs in atomic
		 * context, so it cannot sleep.
		 */
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE &&
		    (runtime->period_size * 1000) / rate <= 10)
			udelay(300);

		regmap_update_bits(afe->regmap, irq_data->irq_cnt_reg,
				   irq_data->irq_cnt_maskbit << irq_data->irq_cnt_shift,
				   counter << irq_data->irq_cnt_shift);

		fs = afe->irq_fs(substream, rate);
		if (fs < 0)
			return -EINVAL;

		if (irq_data->irq_fs_reg >= 0)
			regmap_update_bits(afe->regmap, irq_data->irq_fs_reg,
					   irq_data->irq_fs_maskbit << irq_data->irq_fs_shift,
					   fs << irq_data->irq_fs_shift);

		regmap_update_bits(afe->regmap, irq_data->irq_en_reg,
				   1 << irq_data->irq_en_shift,
				   1 << irq_data->irq_en_shift);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		ret = mtk_memif_set_disable(afe, id);
		if (ret)
			dev_warn(afe->dev, "id %d, memif disable fail\n", id);

		regmap_update_bits(afe->regmap, irq_data->irq_en_reg,
				   1 << irq_data->irq_en_shift, 0);

		mt6878_irq_clear(afe, irq_data);
		return ret;
	default:
		return -EINVAL;
	}
}

static int mt6878_memif_fs(struct snd_pcm_substream *substream,
			   unsigned int rate)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe;

	if (!component)
		return -EINVAL;

	afe = snd_soc_component_get_drvdata(component);

	return mt6878_rate_transform(afe->dev, rate);
}

static int mt6878_get_dai_fs(struct mtk_base_afe *afe,
			     int dai_id, unsigned int rate)
{
	return mt6878_rate_transform(afe->dev, rate);
}

static int mt6878_irq_fs(struct snd_pcm_substream *substream, unsigned int rate)
{
	return mt6878_memif_fs(substream, rate);
}

static int mt6878_get_memif_pbuf_size(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	if ((runtime->period_size * 1000) / runtime->rate > 10)
		return MT6878_MEMIF_PBUF_SIZE_256_BYTES;

	return MT6878_MEMIF_PBUF_SIZE_32_BYTES;
}

/* FE DAIs */
static const struct snd_soc_dai_ops mt6878_memif_dai_ops = {
	.startup	= mt6878_fe_startup,
	.shutdown	= mt6878_fe_shutdown,
	.hw_params	= mt6878_fe_hw_params,
	.hw_free	= mtk_afe_fe_hw_free,
	.prepare	= mtk_afe_fe_prepare,
	.trigger	= mt6878_fe_trigger,
};

#define MTK_PCM_RATES (SNDRV_PCM_RATE_8000_48000 |\
		       SNDRV_PCM_RATE_88200 |\
		       SNDRV_PCM_RATE_96000 |\
		       SNDRV_PCM_RATE_176400 |\
		       SNDRV_PCM_RATE_192000)

#define MTK_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			 SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S32_LE)

#define MT6878_FE_DAI(_name, _id, max_ch, dir) \
{ \
	.name = #_name, \
	.id = _id, \
	.dir = { \
		.stream_name = #_name, \
		.channels_min = 1, \
		.channels_max = max_ch, \
		.rates = MTK_PCM_RATES, \
		.formats = MTK_PCM_FORMATS, \
	}, \
	.ops = &mt6878_memif_dai_ops, \
}

static struct snd_soc_dai_driver mt6878_memif_dai_driver[] = {
	/* FE DAIs: memory intefaces to CPU */
	/* Playback */
	MT6878_FE_DAI(DL0, MT6878_MEMIF_DL0, 2, playback),
	MT6878_FE_DAI(DL1, MT6878_MEMIF_DL1, 2, playback),
	MT6878_FE_DAI(DL2, MT6878_MEMIF_DL2, 2, playback),
	MT6878_FE_DAI(DL3, MT6878_MEMIF_DL3, 2, playback),
	MT6878_FE_DAI(DL4, MT6878_MEMIF_DL4, 2, playback),
	MT6878_FE_DAI(DL5, MT6878_MEMIF_DL5, 2, playback),
	MT6878_FE_DAI(DL6, MT6878_MEMIF_DL6, 2, playback),
	MT6878_FE_DAI(DL7, MT6878_MEMIF_DL7, 2, playback),
	MT6878_FE_DAI(DL8, MT6878_MEMIF_DL8, 2, playback),
	MT6878_FE_DAI(DL23, MT6878_MEMIF_DL23, 2, playback),
	MT6878_FE_DAI(DL24, MT6878_MEMIF_DL24, 2, playback),
	MT6878_FE_DAI(DL25, MT6878_MEMIF_DL25, 2, playback),
	MT6878_FE_DAI(DL_24CH, MT6878_MEMIF_DL_24CH, 8, playback),
	/* Capture */
	MT6878_FE_DAI(UL0, MT6878_MEMIF_VUL0, 2, capture),
	MT6878_FE_DAI(UL1, MT6878_MEMIF_VUL1, 2, capture),
	MT6878_FE_DAI(UL2, MT6878_MEMIF_VUL2, 2, capture),
	MT6878_FE_DAI(UL3, MT6878_MEMIF_VUL3, 2, capture),
	MT6878_FE_DAI(UL4, MT6878_MEMIF_VUL4, 2, capture),
	MT6878_FE_DAI(UL5, MT6878_MEMIF_VUL5, 2, capture),
	MT6878_FE_DAI(UL6, MT6878_MEMIF_VUL6, 2, capture),
	MT6878_FE_DAI(UL7, MT6878_MEMIF_VUL7, 2, capture),
	MT6878_FE_DAI(UL8, MT6878_MEMIF_VUL8, 2, capture),
	MT6878_FE_DAI(UL9, MT6878_MEMIF_VUL9, 2, capture),
	MT6878_FE_DAI(UL10, MT6878_MEMIF_VUL10, 2, capture),
	MT6878_FE_DAI(UL24, MT6878_MEMIF_VUL24, 2, capture),
	MT6878_FE_DAI(UL25, MT6878_MEMIF_VUL25, 2, capture),
	MT6878_FE_DAI(UL_CM0, MT6878_MEMIF_VUL_CM0, 8, capture),
	MT6878_FE_DAI(UL_CM1, MT6878_MEMIF_VUL_CM1, 16, capture),
	MT6878_FE_DAI(UL_ETDM_IN1, MT6878_MEMIF_ETDM_IN1, 2, capture),
	MT6878_FE_DAI(UL_ETDM_IN2, MT6878_MEMIF_ETDM_IN2, 2, capture),
	MT6878_FE_DAI(UL_ETDM_IN4, MT6878_MEMIF_ETDM_IN4, 2, capture),
};

#define MT6878_DL_MEMIF(_id) \
	[MT6878_MEMIF_##_id] = { \
		.name = #_id, \
		.id = MT6878_MEMIF_##_id, \
		.reg_ofs_base = AFE_##_id##_BASE, \
		.reg_ofs_cur = AFE_##_id##_CUR, \
		.reg_ofs_end = AFE_##_id##_END, \
		.reg_ofs_base_msb = AFE_##_id##_BASE_MSB, \
		.reg_ofs_cur_msb = AFE_##_id##_CUR_MSB, \
		.reg_ofs_end_msb = AFE_##_id##_END_MSB, \
		.fs_reg = AFE_##_id##_CON0, \
		.fs_shift = _id##_SEL_FS_SFT, \
		.fs_maskbit = _id##_SEL_FS_MASK, \
		.mono_reg = AFE_##_id##_CON0, \
		.mono_shift = _id##_MONO_SFT, \
		.enable_reg = AFE_##_id##_CON0, \
		.enable_shift = _id##_ON_SFT, \
		.hd_reg = AFE_##_id##_CON0, \
		.hd_shift = _id##_HD_MODE_SFT, \
		.hd_align_reg = AFE_##_id##_CON0, \
		.hd_align_mshift = _id##_HALIGN_SFT, \
		.agent_disable_reg = -1, \
		.agent_disable_shift = -1, \
		.msb_reg = -1, \
		.msb_shift = -1, \
		.pbuf_reg = AFE_##_id##_CON0, \
		.pbuf_mask = _id##_PBUF_SIZE_MASK, \
		.pbuf_shift = _id##_PBUF_SIZE_SFT, \
		.minlen_reg = AFE_##_id##_CON0, \
		.minlen_mask = _id##_MINLEN_MASK, \
		.minlen_shift = _id##_MINLEN_SFT, \
		.ch_num_reg = -1, \
		.ch_num_shift = -1, \
	}

#define MT6878_MULTI_DL_MEMIF(_id) \
	[MT6878_MEMIF_##_id] = { \
		.name = #_id, \
		.id = MT6878_MEMIF_##_id, \
		.reg_ofs_base = AFE_##_id##_BASE, \
		.reg_ofs_cur = AFE_##_id##_CUR, \
		.reg_ofs_end = AFE_##_id##_END, \
		.reg_ofs_base_msb = AFE_##_id##_BASE_MSB, \
		.reg_ofs_cur_msb = AFE_##_id##_CUR_MSB, \
		.reg_ofs_end_msb = AFE_##_id##_END_MSB, \
		.fs_reg = AFE_##_id##_CON0, \
		.fs_shift = _id##_SEL_FS_SFT, \
		.fs_maskbit = _id##_SEL_FS_MASK, \
		.mono_reg = -1, \
		.mono_shift = -1, \
		.enable_reg = AFE_##_id##_CON0, \
		.enable_shift = _id##_ON_SFT, \
		.hd_reg = AFE_##_id##_CON0, \
		.hd_shift = _id##_HD_MODE_SFT, \
		.hd_align_reg = AFE_##_id##_CON0, \
		.hd_align_mshift = _id##_HALIGN_SFT, \
		.agent_disable_reg = -1, \
		.agent_disable_shift = -1, \
		.msb_reg = -1, \
		.msb_shift = -1, \
		.pbuf_reg = AFE_##_id##_CON0, \
		.pbuf_mask = _id##_PBUF_SIZE_MASK, \
		.pbuf_shift = _id##_PBUF_SIZE_SFT, \
		.minlen_reg = AFE_##_id##_CON0, \
		.minlen_mask = _id##_MINLEN_MASK, \
		.minlen_shift = _id##_MINLEN_SFT, \
		.ch_num_reg = AFE_##_id##_CON0, \
		.ch_num_maskbit = _id##_NUM_MASK, \
		.ch_num_shift = _id##_NUM_SFT, \
	}

#define MT6878_UL_MEMIF_FS(_id, _fs_reg, _fs_shift, _fs_maskbit, _mono_shift) \
	[MT6878_MEMIF_##_id] = { \
		.name = #_id, \
		.id = MT6878_MEMIF_##_id, \
		.reg_ofs_base = AFE_##_id##_BASE, \
		.reg_ofs_cur = AFE_##_id##_CUR, \
		.reg_ofs_end = AFE_##_id##_END, \
		.reg_ofs_base_msb = AFE_##_id##_BASE_MSB, \
		.reg_ofs_cur_msb = AFE_##_id##_CUR_MSB, \
		.reg_ofs_end_msb = AFE_##_id##_END_MSB, \
		.fs_reg = _fs_reg, \
		.fs_shift = _fs_shift, \
		.fs_maskbit = _fs_maskbit, \
		.mono_reg = AFE_##_id##_CON0, \
		.mono_shift = _mono_shift, \
		.enable_reg = AFE_##_id##_CON0, \
		.enable_shift = _id##_ON_SFT, \
		.hd_reg = AFE_##_id##_CON0, \
		.hd_shift = _id##_HD_MODE_SFT, \
		.hd_align_reg = AFE_##_id##_CON0, \
		.hd_align_mshift = _id##_HALIGN_SFT, \
		.agent_disable_reg = -1, \
		.agent_disable_shift = -1, \
		.msb_reg = -1, \
		.msb_shift = -1, \
		.ch_num_reg = -1, \
		.ch_num_shift = -1, \
	}

#define MT6878_UL_MEMIF(_id, _fs_shift, _fs_maskbit, _mono_shift) \
	MT6878_UL_MEMIF_FS(_id, AFE_##_id##_CON0, _fs_shift, _fs_maskbit, \
			   _mono_shift)

/* the ETDM capture memifs take their rate from the ETDM_INx_CON3 timing */
#define MT6878_ETDM_MEMIF(_id) \
	MT6878_UL_MEMIF_FS(_id, _id##_CON3, REG_FS_TIMING_SEL_SFT, \
			   REG_FS_TIMING_SEL_MASK, -1)

static const struct mtk_base_memif_data memif_data[MT6878_MEMIF_NUM] = {
	MT6878_DL_MEMIF(DL0),
	MT6878_DL_MEMIF(DL1),
	MT6878_DL_MEMIF(DL2),
	MT6878_DL_MEMIF(DL3),
	MT6878_DL_MEMIF(DL4),
	MT6878_DL_MEMIF(DL5),
	MT6878_DL_MEMIF(DL6),
	MT6878_DL_MEMIF(DL7),
	MT6878_DL_MEMIF(DL8),
	MT6878_DL_MEMIF(DL23),
	MT6878_DL_MEMIF(DL24),
	MT6878_DL_MEMIF(DL25),
	MT6878_MULTI_DL_MEMIF(DL_24CH),
	MT6878_UL_MEMIF(VUL0, VUL0_SEL_FS_SFT, VUL0_SEL_FS_MASK, VUL0_MONO_SFT),
	MT6878_UL_MEMIF(VUL1, VUL1_SEL_FS_SFT, VUL1_SEL_FS_MASK, VUL1_MONO_SFT),
	MT6878_UL_MEMIF(VUL2, VUL2_SEL_FS_SFT, VUL2_SEL_FS_MASK, VUL2_MONO_SFT),
	MT6878_UL_MEMIF(VUL3, VUL3_SEL_FS_SFT, VUL3_SEL_FS_MASK, VUL3_MONO_SFT),
	MT6878_UL_MEMIF(VUL4, VUL4_SEL_FS_SFT, VUL4_SEL_FS_MASK, VUL4_MONO_SFT),
	MT6878_UL_MEMIF(VUL5, VUL5_SEL_FS_SFT, VUL5_SEL_FS_MASK, VUL5_MONO_SFT),
	MT6878_UL_MEMIF(VUL6, VUL6_SEL_FS_SFT, VUL6_SEL_FS_MASK, VUL6_MONO_SFT),
	MT6878_UL_MEMIF(VUL7, VUL7_SEL_FS_SFT, VUL7_SEL_FS_MASK, VUL7_MONO_SFT),
	MT6878_UL_MEMIF(VUL8, VUL8_SEL_FS_SFT, VUL8_SEL_FS_MASK, VUL8_MONO_SFT),
	MT6878_UL_MEMIF(VUL9, VUL9_SEL_FS_SFT, VUL9_SEL_FS_MASK, VUL9_MONO_SFT),
	MT6878_UL_MEMIF(VUL10, VUL10_SEL_FS_SFT, VUL10_SEL_FS_MASK, VUL10_MONO_SFT),
	MT6878_UL_MEMIF(VUL24, VUL24_SEL_FS_SFT, VUL24_SEL_FS_MASK, VUL24_MONO_SFT),
	MT6878_UL_MEMIF(VUL25, VUL25_SEL_FS_SFT, VUL25_SEL_FS_MASK, VUL25_MONO_SFT),
	MT6878_UL_MEMIF(VUL_CM0, -1, -1, -1),
	MT6878_UL_MEMIF(VUL_CM1, -1, -1, -1),
	MT6878_ETDM_MEMIF(ETDM_IN1),
	MT6878_ETDM_MEMIF(ETDM_IN2),
	MT6878_ETDM_MEMIF(ETDM_IN4),
};

#define MT6878_AFE_IRQ(_id) \
	[MT6878_IRQ_##_id] = { \
		.id = MT6878_IRQ_##_id, \
		.irq_cnt_reg = AFE_IRQ##_id##_MCU_CFG1, \
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT, \
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK, \
		.irq_fs_reg = AFE_IRQ##_id##_MCU_CFG0, \
		.irq_fs_shift = AFE_IRQ##_id##_MCU_FS_SFT, \
		.irq_fs_maskbit = AFE_IRQ##_id##_MCU_FS_MASK, \
		.irq_en_reg = AFE_IRQ##_id##_MCU_CFG0, \
		.irq_en_shift = AFE_IRQ##_id##_MCU_ON_SFT, \
		.irq_clr_reg = AFE_IRQ##_id##_MCU_CFG1, \
		.irq_clr_shift = AFE_IRQ##_id##_CLR_CFG_SFT, \
	}

static const struct mtk_base_irq_data irq_data[MT6878_IRQ_NUM] = {
	MT6878_AFE_IRQ(0),
	MT6878_AFE_IRQ(1),
	MT6878_AFE_IRQ(2),
	MT6878_AFE_IRQ(3),
	MT6878_AFE_IRQ(4),
	MT6878_AFE_IRQ(5),
	MT6878_AFE_IRQ(6),
	MT6878_AFE_IRQ(7),
	MT6878_AFE_IRQ(8),
	MT6878_AFE_IRQ(9),
	MT6878_AFE_IRQ(10),
	MT6878_AFE_IRQ(11),
	MT6878_AFE_IRQ(12),
	MT6878_AFE_IRQ(13),
	MT6878_AFE_IRQ(14),
	MT6878_AFE_IRQ(15),
	MT6878_AFE_IRQ(16),
	MT6878_AFE_IRQ(17),
	MT6878_AFE_IRQ(18),
	MT6878_AFE_IRQ(19),
	MT6878_AFE_IRQ(20),
	MT6878_AFE_IRQ(21),
	MT6878_AFE_IRQ(22),
	MT6878_AFE_IRQ(23),
	MT6878_AFE_IRQ(24),
	MT6878_AFE_IRQ(25),
	MT6878_AFE_IRQ(26),
};

/*
 * Fixed memif to IRQ mapping of the MediaTek downstream driver. VUL_CM1 and the ETDM
 * memifs have no IRQ of their own and are not expected to be used from the
 * CPU yet.
 */
static const int memif_irq_usage[MT6878_MEMIF_NUM] = {
	[MT6878_MEMIF_DL0] = MT6878_IRQ_0,
	[MT6878_MEMIF_DL1] = MT6878_IRQ_1,
	[MT6878_MEMIF_DL2] = MT6878_IRQ_2,
	[MT6878_MEMIF_DL3] = MT6878_IRQ_3,
	[MT6878_MEMIF_DL4] = MT6878_IRQ_4,
	[MT6878_MEMIF_DL5] = MT6878_IRQ_5,
	[MT6878_MEMIF_DL6] = MT6878_IRQ_6,
	[MT6878_MEMIF_DL7] = MT6878_IRQ_7,
	[MT6878_MEMIF_DL8] = MT6878_IRQ_8,
	[MT6878_MEMIF_DL23] = MT6878_IRQ_9,
	[MT6878_MEMIF_DL24] = MT6878_IRQ_10,
	[MT6878_MEMIF_DL25] = MT6878_IRQ_11,
	[MT6878_MEMIF_DL_24CH] = MT6878_IRQ_12,
	[MT6878_MEMIF_VUL0] = MT6878_IRQ_13,
	[MT6878_MEMIF_VUL1] = MT6878_IRQ_14,
	[MT6878_MEMIF_VUL2] = MT6878_IRQ_15,
	[MT6878_MEMIF_VUL3] = MT6878_IRQ_16,
	[MT6878_MEMIF_VUL4] = MT6878_IRQ_17,
	[MT6878_MEMIF_VUL5] = MT6878_IRQ_18,
	[MT6878_MEMIF_VUL6] = MT6878_IRQ_19,
	[MT6878_MEMIF_VUL7] = MT6878_IRQ_20,
	[MT6878_MEMIF_VUL8] = MT6878_IRQ_21,
	[MT6878_MEMIF_VUL9] = MT6878_IRQ_22,
	[MT6878_MEMIF_VUL10] = MT6878_IRQ_23,
	[MT6878_MEMIF_VUL24] = MT6878_IRQ_24,
	[MT6878_MEMIF_VUL25] = MT6878_IRQ_25,
	[MT6878_MEMIF_VUL_CM0] = MT6878_IRQ_26,
	[MT6878_MEMIF_VUL_CM1] = MT6878_IRQ_0,
	[MT6878_MEMIF_ETDM_IN1] = MT6878_IRQ_0,
	[MT6878_MEMIF_ETDM_IN2] = MT6878_IRQ_0,
	[MT6878_MEMIF_ETDM_IN4] = MT6878_IRQ_0,
};

static bool mt6878_is_volatile_reg(struct device *dev, unsigned int reg)
{
	/* these auto-gen reg has read-only bit, so put it as volatile */
	/* volatile reg cannot be cached, so cannot be set when power off */
	switch (reg) {
	case AUDIO_TOP_CON0 ... AUDIO_TOP_CON4:
	case AFE_APLL1_TUNER_MON0:
	case AFE_APLL2_TUNER_MON0:
	case AFE_SPM_CONTROL_ACK:
	case AUDIO_TOP_IP_VERSION:
	case AUDIO_ENGEN_CON0_MON:
	case AFE_CONNSYS_I2S_IPM_VER_MON:
	case AFE_CONNSYS_I2S_MON:
	case AFE_PCM_INTF_MON:
	case AFE_PCM_TOP_IP_VERSION:
	case AFE_IRQ_MCU_STATUS:
	case AFE_CUSTOM_IRQ_MCU_STATUS:
	case AFE_IRQ_MCU_MON0 ... AFE_IRQ26_CNT_MON:
	case AFE_STF_MON:
	case AFE_STF_IP_VERSION:
	case AFE_CM0_MON:
	case AFE_CM0_IP_VERSION:
	case AFE_CM1_MON:
	case AFE_CM1_IP_VERSION:
	case AFE_ADDA_UL0_SRC_DEBUG_MON0 ... AFE_ADDA_UL0_SRC_MON1:
	case AFE_ADDA_UL0_IP_VERSION:
	case AFE_ADDA_UL1_SRC_DEBUG_MON0 ... AFE_ADDA_UL1_SRC_MON1:
	case AFE_ADDA_UL1_IP_VERSION:
	case AFE_MTKAIF_IPM_VER_MON:
	case AFE_MTKAIF_MON:
	case AFE_AUD_PAD_TOP_MON:
	case AFE_ADDA_MTKAIFV4_MON0:
	case AFE_ADDA_MTKAIFV4_MON1:
	case AFE_ADDA6_MTKAIFV4_MON0:
	case ETDM_IN1_MON:
	case ETDM_IN2_MON:
	case ETDM_IN4_MON:
	case ETDM_OUT1_MON:
	case ETDM_OUT2_MON:
	case ETDM_OUT4_MON:
	case AFE_CONN_MON0 ... AFE_CONN_MON5:
	case AFE_CBIP_SLV_DECODER_MON0 ... AFE_CBIP_SLV_MUX_MON1:
	case AFE_DL0_CUR_MSB ... AFE_DL0_CUR:
	case AFE_DL0_RCH_MON ... AFE_DL0_LCH_MON:
	case AFE_DL1_CUR_MSB ... AFE_DL1_CUR:
	case AFE_DL1_RCH_MON ... AFE_DL1_LCH_MON:
	case AFE_DL2_CUR_MSB ... AFE_DL2_CUR:
	case AFE_DL2_RCH_MON ... AFE_DL2_LCH_MON:
	case AFE_DL3_CUR_MSB ... AFE_DL3_CUR:
	case AFE_DL3_RCH_MON ... AFE_DL3_LCH_MON:
	case AFE_DL4_CUR_MSB ... AFE_DL4_CUR:
	case AFE_DL4_RCH_MON ... AFE_DL4_LCH_MON:
	case AFE_DL5_CUR_MSB ... AFE_DL5_CUR:
	case AFE_DL5_RCH_MON ... AFE_DL5_LCH_MON:
	case AFE_DL6_CUR_MSB ... AFE_DL6_CUR:
	case AFE_DL6_RCH_MON ... AFE_DL6_LCH_MON:
	case AFE_DL7_CUR_MSB ... AFE_DL7_CUR:
	case AFE_DL7_RCH_MON ... AFE_DL7_LCH_MON:
	case AFE_DL8_CUR_MSB ... AFE_DL8_CUR:
	case AFE_DL8_RCH_MON ... AFE_DL8_LCH_MON:
	case AFE_DL_24CH_CUR_MSB ... AFE_DL_24CH_CUR:
	case AFE_DL23_CUR_MSB ... AFE_DL23_CUR:
	case AFE_DL23_RCH_MON ... AFE_DL23_LCH_MON:
	case AFE_DL24_CUR_MSB ... AFE_DL24_CUR:
	case AFE_DL24_RCH_MON ... AFE_DL24_LCH_MON:
	case AFE_DL25_CUR_MSB ... AFE_DL25_CUR:
	case AFE_DL25_RCH_MON ... AFE_DL25_LCH_MON:
	case AFE_VUL0_CUR_MSB ... AFE_VUL0_CUR:
	case AFE_VUL1_CUR_MSB ... AFE_VUL1_CUR:
	case AFE_VUL2_CUR_MSB ... AFE_VUL2_CUR:
	case AFE_VUL3_CUR_MSB ... AFE_VUL3_CUR:
	case AFE_VUL4_CUR_MSB ... AFE_VUL4_CUR:
	case AFE_VUL5_CUR_MSB ... AFE_VUL5_CUR:
	case AFE_VUL6_CUR_MSB ... AFE_VUL6_CUR:
	case AFE_VUL7_CUR_MSB ... AFE_VUL7_CUR:
	case AFE_VUL8_CUR_MSB ... AFE_VUL8_CUR:
	case AFE_VUL9_CUR_MSB ... AFE_VUL9_CUR:
	case AFE_VUL10_CUR_MSB ... AFE_VUL10_CUR:
	case AFE_VUL24_CUR_MSB ... AFE_VUL24_CUR:
	case AFE_VUL25_CUR_MSB ... AFE_VUL25_CUR:
	case AFE_VUL_CM0_BASE_MSB ... AFE_VUL_CM0_CON0:
	case AFE_VUL_CM1_CUR_MSB ... AFE_VUL_CM1_CUR:
	case AFE_ETDM_IN1_CUR_MSB ... AFE_ETDM_IN1_CUR:
	case AFE_ETDM_IN2_CUR_MSB ... AFE_ETDM_IN2_CUR:
	case AFE_ETDM_IN4_CUR_MSB ... AFE_ETDM_IN4_CUR:
	case AFE_PROT_SIDEBAND0_MON ... AFE_PROT_SIDEBAND3_MON:
	case AFE_DOMAIN_SIDEBAND0_MON ... AFE_DOMAIN_SIDEBAND9_MON:
	case AFE_PCM0_INTF_CON1_MASK_MON:
	case AFE_PCM0_INTF_CON0_MASK_MON:
	case AFE_CONNSYS_I2S_CON_MASK_MON:
	case AFE_MTKAIF0_CFG0_MASK_MON:
	case AFE_MTKAIF1_CFG0_MASK_MON:
	case AFE_ADDA_UL0_SRC_CON0_MASK_MON:
	case AFE_ADDA_UL1_SRC_CON0_MASK_MON:
	case AFE_ASRC_NEW_CON0:
	case AFE_ASRC_NEW_CON6:
	case AFE_ASRC_NEW_CON8:
	case AFE_ASRC_NEW_CON9:
	case AFE_ASRC_NEW_CON12:
	case AFE_ASRC_NEW_IP_VERSION:
	case AFE_GASRC0_NEW_CON0:
	case AFE_GASRC0_NEW_CON6:
	case AFE_GASRC0_NEW_CON8 ... AFE_GASRC0_NEW_CON12:
	case AFE_GASRC0_NEW_IP_VERSION:
	case AFE_GASRC1_NEW_CON0:
	case AFE_GASRC1_NEW_CON6:
	case AFE_GASRC1_NEW_CON8:
	case AFE_GASRC1_NEW_CON9:
	case AFE_GASRC1_NEW_CON12:
	case AFE_GASRC1_NEW_IP_VERSION:
	case AFE_GASRC2_NEW_CON0:
	case AFE_GASRC2_NEW_CON6:
	case AFE_GASRC2_NEW_CON8:
	case AFE_GASRC2_NEW_CON9:
	case AFE_GASRC2_NEW_CON12:
	case AFE_GASRC2_NEW_IP_VERSION:
	case AFE_GAIN0_CUR_L:
	case AFE_GAIN0_CUR_R:
	case AFE_GAIN1_CUR_L:
	case AFE_GAIN1_CUR_R:
	case AFE_GAIN2_CUR_L:
	case AFE_GAIN2_CUR_R:
	case AFE_GAIN3_CUR_L:
	case AFE_GAIN3_CUR_R:
	/* also written by the audio DSP and SCP firmware */
	case AFE_IRQ_MCU_EN ... AFE_IRQ_MCU_SCP_EN:
	case AFE_DL5_CON0:
	case AFE_DL6_CON0:
	case AFE_DL23_CON0:
	case AFE_DL_24CH_CON0:
	case AFE_VUL1_CON0:
	case AFE_VUL3_CON0:
	case AFE_VUL4_CON0:
	case AFE_VUL5_CON0:
	case AFE_VUL9_CON0:
	case AFE_VUL25_CON0:
	case AFE_IRQ0_MCU_CFG0 ... AFE_IRQ26_MCU_CFG1:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config mt6878_afe_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.volatile_reg = mt6878_is_volatile_reg,

	.max_register = AFE_MAX_REGISTER,
	.num_reg_defaults_raw = AFE_MAX_REGISTER,

	.cache_type = REGCACHE_FLAT,
};

static irqreturn_t mt6878_afe_irq_handler(int irq_id, void *dev)
{
	struct mtk_base_afe *afe = dev;
	struct mtk_base_afe_irq *irq;
	unsigned int status, status_mcu, mcu_en;
	int ret, i;

	/* only care about the IRQs that are routed to the MCU */
	regmap_read(afe->regmap, AFE_IRQ_MCU_EN, &mcu_en);
	ret = regmap_read(afe->regmap, AFE_IRQ_MCU_STATUS, &status);
	status_mcu = status & mcu_en & AFE_IRQ_STATUS_BITS;
	if (ret || !status_mcu) {
		dev_err_ratelimited(afe->dev, "irq ret %d, status 0x%x, en 0x%x\n",
				    ret, status, mcu_en);
		return IRQ_NONE;
	}

	for (i = 0; i < MT6878_MEMIF_NUM; i++) {
		struct mtk_base_afe_memif *memif = &afe->memif[i];

		if (!memif->substream || memif->irq_usage < 0)
			continue;

		irq = &afe->irqs[memif->irq_usage];
		if (status_mcu & BIT(irq->irq_data->id))
			snd_pcm_period_elapsed(memif->substream);
	}

	for (i = 0; i < MT6878_IRQ_NUM; i++)
		if (status_mcu & BIT(i))
			mt6878_irq_clear(afe, &irq_data[i]);

	return IRQ_HANDLED;
}

/*
 * The AFE power domain, its SRAM and the DRAM path are arbitrated by the
 * trusted firmware (SPM sidebands). Same SiP call as on MT8188.
 */
static void mt6878_afe_smc(unsigned long op)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_AUDIO_CONTROL, op, 0, 0, 0, 0, 0, 0, &res);
}

static int mt6878_afe_runtime_suspend(struct device *dev)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dev);
	unsigned int value;
	int ret, i;

	if (!afe->regmap)
		goto skip_regmap;

	/* disable AFE */
	mt6878_afe_disable_main_clock(afe);

	ret = regmap_read_poll_timeout(afe->regmap, AUDIO_ENGEN_CON0_MON, value,
				       (value & AUDIO_ENGEN_MON_MASK_SFT) == 0,
				       20, 1 * 1000 * 1000);
	if (ret)
		dev_warn(afe->dev, "AFE engine still busy: %d\n", ret);

	/* make sure all irq status are cleared */
	for (i = 0; i < MT6878_IRQ_NUM; i++)
		mt6878_irq_clear(afe, &irq_data[i]);

	/* gate the AFE top clocks again */
	regmap_write(afe->regmap, AUDIO_TOP_CON4, AUDIO_TOP_CON4_ALL_PDN);

	regcache_cache_only(afe->regmap, true);
	regcache_mark_dirty(afe->regmap);

skip_regmap:
	mt6878_afe_smc(MTK_AUDIO_SMC_OP_SRAM_RELEASE);
	mt6878_afe_disable_reg_rw_clk(afe);
	return 0;
}

static int mt6878_afe_runtime_resume(struct device *dev)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dev);
	int ret;

	ret = mt6878_afe_enable_reg_rw_clk(afe);
	if (ret)
		return ret;

	mt6878_afe_smc(MTK_AUDIO_SMC_OP_DOMAIN_SIDEBANDS);
	mt6878_afe_smc(MTK_AUDIO_SMC_OP_SRAM_REQUEST);

	if (!afe->regmap)
		return 0;

	regcache_cache_only(afe->regmap, false);
	regcache_sync(afe->regmap);

	regmap_update_bits(afe->regmap, AFE_CBIP_CFG0, 0x1, 0x1);

	/* force cpu use 8_24 format when writing 32bit data */
	regmap_update_bits(afe->regmap, AFE_MEMIF_CON0,
			   CPU_HD_ALIGN_MASK_SFT, 0 << CPU_HD_ALIGN_SFT);

	/* enable AFE */
	return mt6878_afe_enable_main_clock(afe);
}

static int mt6878_afe_component_probe(struct snd_soc_component *component)
{
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	int ret;

	ret = pm_runtime_resume_and_get(afe->dev);
	if (ret)
		return dev_err_probe(afe->dev, ret, "failed to resume device\n");

	mtk_afe_add_sub_dai_control(component);
	pm_runtime_put_sync(afe->dev);

	return 0;
}

static int mt6878_afe_pcm_open(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	/* set the wait_for_avail to 2 sec */
	substream->wait_time = msecs_to_jiffies(2 * 1000);

	return 0;
}

static void mt6878_afe_pcm_free(struct snd_soc_component *component,
				struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static const struct snd_soc_component_driver mt6878_afe_component = {
	.name = AFE_PCM_NAME,
	.probe = mt6878_afe_component_probe,
	.pcm_new = mtk_afe_pcm_new,
	.pcm_free = mt6878_afe_pcm_free,
	.open = mt6878_afe_pcm_open,
	.pointer = mtk_afe_pcm_pointer,
};

static int mt6878_dai_memif_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	list_add(&dai->list, &afe->sub_dais);

	dai->dai_drivers = mt6878_memif_dai_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mt6878_memif_dai_driver);

	return 0;
}

typedef int (*dai_register_cb)(struct mtk_base_afe *);
static const dai_register_cb dai_register_cbs[] = {
	mt6878_dai_memif_register,
};

/* HOPPING and F26M on, APLL/H208M/tuner clocks gated until needed */
static const struct reg_sequence mt6878_cg_patch[] = {
	{ AUDIO_TOP_CON4, 0x361c },
};

static void mt6878_afe_release_reserved_mem(void *data)
{
	of_reserved_mem_device_release(data);
}

static int mt6878_afe_pcm_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6878_afe_private *afe_priv;
	struct mtk_base_afe *afe;
	int ret, i, irq_id;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(34));
	if (ret)
		return ret;

	ret = of_reserved_mem_device_init(dev);
	if (!ret) {
		ret = devm_add_action_or_reset(dev,
					       mt6878_afe_release_reserved_mem,
					       dev);
		if (ret)
			return ret;
	} else if (ret != -ENODEV) {
		return dev_err_probe(dev, ret, "failed to assign memory region\n");
	}

	afe = devm_kzalloc(dev, sizeof(*afe), GFP_KERNEL);
	if (!afe)
		return -ENOMEM;

	platform_set_drvdata(pdev, afe);

	afe_priv = devm_kzalloc(dev, sizeof(*afe_priv), GFP_KERNEL);
	if (!afe_priv)
		return -ENOMEM;

	afe->platform_priv = afe_priv;
	afe->dev = dev;

	afe->base_addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(afe->base_addr))
		return dev_err_probe(dev, PTR_ERR(afe->base_addr),
				     "AFE base_addr not found\n");

	ret = mt6878_init_clock(afe);
	if (ret)
		return ret;

	/* init memif: IPM2.0 has no bit banding */
	afe->memif_32bit_supported = 1;
	afe->memif_size = MT6878_MEMIF_NUM;
	afe->memif = devm_kcalloc(dev, afe->memif_size, sizeof(*afe->memif),
				  GFP_KERNEL);
	if (!afe->memif)
		return -ENOMEM;

	for (i = 0; i < afe->memif_size; i++) {
		afe->memif[i].data = &memif_data[i];
		afe->memif[i].irq_usage = memif_irq_usage[i];
		afe->memif[i].const_irq = 1;
	}

	mutex_init(&afe->irq_alloc_lock);

	/* init irq */
	afe->irqs_size = MT6878_IRQ_NUM;
	afe->irqs = devm_kcalloc(dev, afe->irqs_size, sizeof(*afe->irqs),
				 GFP_KERNEL);
	if (!afe->irqs)
		return -ENOMEM;

	for (i = 0; i < afe->irqs_size; i++)
		afe->irqs[i].irq_data = &irq_data[i];

	irq_id = platform_get_irq(pdev, 0);
	if (irq_id < 0)
		return irq_id;

	ret = devm_request_irq(dev, irq_id, mt6878_afe_irq_handler,
			       IRQF_TRIGGER_NONE, "Afe_ISR_Handle", afe);
	if (ret)
		return dev_err_probe(dev, ret, "could not request irq\n");

	mt6878_afe_smc(MTK_AUDIO_SMC_OP_INIT);

	/* init sub_dais */
	INIT_LIST_HEAD(&afe->sub_dais);

	for (i = 0; i < ARRAY_SIZE(dai_register_cbs); i++) {
		ret = dai_register_cbs[i](afe);
		if (ret)
			return dev_err_probe(dev, ret, "dai register %d failed\n", i);
	}

	ret = mtk_afe_combine_sub_dai(afe);
	if (ret)
		return dev_err_probe(dev, ret, "mtk_afe_combine_sub_dai failed\n");

	afe->mtk_afe_hardware = &mt6878_afe_hardware;
	afe->memif_fs = mt6878_memif_fs;
	afe->irq_fs = mt6878_irq_fs;
	afe->get_dai_fs = mt6878_get_dai_fs;
	afe->get_memif_pbuf_size = mt6878_get_memif_pbuf_size;

	afe->runtime_resume = mt6878_afe_runtime_resume;
	afe->runtime_suspend = mt6878_afe_runtime_suspend;

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	/*
	 * The AFE is part of a genpd. Registering it as a syscore device
	 * keeps genpd from powering it off behind our back on suspend.
	 */
	dev_pm_syscore_device(dev, true);

	/* enable clock for regcache get default value from hw */
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to resume device\n");

	afe->regmap = devm_regmap_init_mmio(dev, afe->base_addr,
					    &mt6878_afe_regmap_config);
	if (IS_ERR(afe->regmap)) {
		ret = PTR_ERR(afe->regmap);
		goto err_pm_put;
	}

	ret = regmap_register_patch(afe->regmap, mt6878_cg_patch,
				    ARRAY_SIZE(mt6878_cg_patch));
	if (ret < 0) {
		dev_err(dev, "failed to apply cg patch: %d\n", ret);
		goto err_pm_put;
	}

	/* route all AFE IRQs to the MCU */
	regmap_write(afe->regmap, AFE_IRQ_MCU_EN, 0xffffffff);

	pm_runtime_put_sync(dev);

	regcache_cache_only(afe->regmap, true);
	regcache_mark_dirty(afe->regmap);

	ret = devm_snd_soc_register_component(dev, &mt6878_afe_component,
					      afe->dai_drivers,
					      afe->num_dai_drivers);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register component\n");

	return 0;

err_pm_put:
	pm_runtime_put_sync(dev);
	return ret;
}

static void mt6878_afe_pcm_dev_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	if (!pm_runtime_status_suspended(dev))
		mt6878_afe_runtime_suspend(dev);
}

static const struct of_device_id mt6878_afe_pcm_dt_match[] = {
	{ .compatible = "mediatek,mt6878-afe", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6878_afe_pcm_dt_match);

static const struct dev_pm_ops mt6878_afe_pm_ops = {
	RUNTIME_PM_OPS(mt6878_afe_runtime_suspend,
		       mt6878_afe_runtime_resume, NULL)
};

static struct platform_driver mt6878_afe_pcm_driver = {
	.driver = {
		.name = "mt6878-afe",
		.of_match_table = mt6878_afe_pcm_dt_match,
		.pm = pm_ptr(&mt6878_afe_pm_ops),
	},
	.probe = mt6878_afe_pcm_dev_probe,
	.remove = mt6878_afe_pcm_dev_remove,
};
module_platform_driver(mt6878_afe_pcm_driver);

MODULE_DESCRIPTION("Mediatek ALSA SoC AFE platform driver for 6878");
MODULE_LICENSE("GPL");
