// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6375 ADC
 *
 * One-shot conversions of the charger and Type-C port voltages, currents
 * and the die temperature. The conversion sequence, result addresses and
 * scales follow the GPL MediaTek BSP driver (drivers/iio/adc/mt6375-adc.c,
 * Copyright (c) 2021 MediaTek Inc., ChiYuan Huang <cy_huang@richtek.com>).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>

#include <dt-bindings/iio/adc/mediatek,mt6375-adc.h>

#define MT6375_REG_ADC_MONRPTH	0x19c
#define MT6375_REG_ADC_CFG1	0x1a4
#define MT6375_REG_ADC_CFG3	0x1a6
#define MT6375_REG_ADC_CHEN	0x1a7
#define MT6375_REG_ADC_CH1RPTH	0x1aa

#define MT6375_ADCEN		BIT(7)
#define MT6375_ZCVEN		BIT(6)
#define MT6375_ONESHOT_MASK	GENMASK(7, 4)

#define MT6375_ADC_NUM_CHANNELS	16

/* The BSP waits two conversion times, then polls the one-shot field */
#define MT6375_CONV_TIME_US	(2200 * 2)
#define MT6375_POLL_US		100
#define MT6375_POLL_TIMEOUT_US	5000

struct mt6375_adc {
	struct device *dev;
	struct regmap *regmap;
	/* one conversion at a time: the one-shot field selects the channel */
	struct mutex lock;
};

static int mt6375_adc_read_channel(struct mt6375_adc *adc, int chan, int *val)
{
	unsigned int addr, regval;
	__be16 raw;
	int ret;

	mutex_lock(&adc->lock);

	ret = regmap_write(adc->regmap, MT6375_REG_ADC_CFG3,
			   FIELD_PREP(MT6375_ONESHOT_MASK, chan));
	if (ret)
		goto out;

	fsleep(MT6375_CONV_TIME_US);

	/*
	 * Like the BSP driver, read the result even if the one-shot field
	 * has not cleared yet; only a bus error is fatal.
	 */
	ret = regmap_read_poll_timeout(adc->regmap, MT6375_REG_ADC_CFG3, regval,
				       !(regval & MT6375_ONESHOT_MASK),
				       MT6375_POLL_US, MT6375_POLL_TIMEOUT_US);
	if (ret == -ETIMEDOUT)
		dev_dbg(adc->dev, "channel %d: conversion still busy\n", chan);
	else if (ret)
		goto out;

	if (chan == MT6375_ADC_VBATMON)
		addr = MT6375_REG_ADC_MONRPTH;
	else
		addr = MT6375_REG_ADC_CH1RPTH + (chan - 1) * 2;

	ret = regmap_raw_read(adc->regmap, addr, &raw, sizeof(raw));
	if (ret)
		goto out;

	*val = be16_to_cpu(raw);
	ret = IIO_VAL_INT;
out:
	mutex_unlock(&adc->lock);
	return ret;
}

/* Scales give mV, mA and milli-degC */
static int mt6375_adc_read_scale(int chan, int *val1, int *val2)
{
	switch (chan) {
	case MT6375_ADC_VBATMON:
	case MT6375_ADC_VSYS:
	case MT6375_ADC_VBAT:
	case MT6375_ADC_VREFTS:
	case MT6375_ADC_TS:
		*val1 = 5;		/* 1.25 mV */
		*val2 = 4;
		return IIO_VAL_FRACTIONAL;
	case MT6375_ADC_CHGVIN:
		*val1 = 25;		/* 6.25 mV */
		*val2 = 4;
		return IIO_VAL_FRACTIONAL;
	case MT6375_ADC_USBDP:
	case MT6375_ADC_USBDM:
	case MT6375_ADC_CC1:
	case MT6375_ADC_CC2:
	case MT6375_ADC_SBU1:
	case MT6375_ADC_SBU2:
	case MT6375_ADC_IBUS:
	case MT6375_ADC_IBAT:
		*val1 = 5;		/* 2.5 mV or 2.5 mA */
		*val2 = 2;
		return IIO_VAL_FRACTIONAL;
	case MT6375_ADC_PDVBUS:
		*val1 = 25;		/* 25 mV */
		return IIO_VAL_INT;
	case MT6375_ADC_TEMPJC:
		*val1 = 1000;		/* 1 degC, offset 56 */
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int mt6375_adc_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	struct mt6375_adc *adc = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return mt6375_adc_read_channel(adc, chan->channel, val);
	case IIO_CHAN_INFO_SCALE:
		return mt6375_adc_read_scale(chan->channel, val, val2);
	case IIO_CHAN_INFO_OFFSET:
		*val = -56;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const char * const mt6375_adc_labels[MT6375_ADC_NUM_CHANNELS] = {
	[MT6375_ADC_VBATMON] = "vbat_mon",
	[MT6375_ADC_CHGVIN] = "vbus",
	[MT6375_ADC_USBDP] = "usb_dp",
	[MT6375_ADC_VSYS] = "vsys",
	[MT6375_ADC_VBAT] = "vbat",
	[MT6375_ADC_IBUS] = "ibus",
	[MT6375_ADC_IBAT] = "ibat",
	[MT6375_ADC_USBDM] = "usb_dm",
	[MT6375_ADC_TEMPJC] = "temp_jc",
	[MT6375_ADC_VREFTS] = "vref_ts",
	[MT6375_ADC_TS] = "ts",
	[MT6375_ADC_PDVBUS] = "pd_vbus",
	[MT6375_ADC_CC1] = "cc1",
	[MT6375_ADC_CC2] = "cc2",
	[MT6375_ADC_SBU1] = "sbu1",
	[MT6375_ADC_SBU2] = "sbu2",
};

static int mt6375_adc_read_label(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, char *label)
{
	return sysfs_emit(label, "%s\n", mt6375_adc_labels[chan->channel]);
}

static const struct iio_info mt6375_adc_info = {
	.read_raw = mt6375_adc_read_raw,
	.read_label = mt6375_adc_read_label,
};

#define MT6375_ADC_CHAN(_idx, _type, _extra) {			\
	.type = _type,						\
	.channel = MT6375_ADC_##_idx,				\
	.scan_index = MT6375_ADC_##_idx,			\
	.datasheet_name = #_idx,				\
	.indexed = 1,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE) | (_extra),	\
}

static const struct iio_chan_spec mt6375_adc_channels[] = {
	MT6375_ADC_CHAN(VBATMON, IIO_VOLTAGE, 0),
	MT6375_ADC_CHAN(CHGVIN, IIO_VOLTAGE, 0),
	MT6375_ADC_CHAN(USBDP, IIO_VOLTAGE, 0),
	MT6375_ADC_CHAN(VSYS, IIO_VOLTAGE, 0),
	MT6375_ADC_CHAN(VBAT, IIO_VOLTAGE, 0),
	MT6375_ADC_CHAN(IBUS, IIO_CURRENT, 0),
	MT6375_ADC_CHAN(IBAT, IIO_CURRENT, 0),
	MT6375_ADC_CHAN(USBDM, IIO_VOLTAGE, 0),
	MT6375_ADC_CHAN(TEMPJC, IIO_TEMP, BIT(IIO_CHAN_INFO_OFFSET)),
	MT6375_ADC_CHAN(VREFTS, IIO_VOLTAGE, 0),
	MT6375_ADC_CHAN(TS, IIO_VOLTAGE, 0),
	MT6375_ADC_CHAN(PDVBUS, IIO_VOLTAGE, 0),
	MT6375_ADC_CHAN(CC1, IIO_VOLTAGE, 0),
	MT6375_ADC_CHAN(CC2, IIO_VOLTAGE, 0),
	MT6375_ADC_CHAN(SBU1, IIO_VOLTAGE, 0),
	MT6375_ADC_CHAN(SBU2, IIO_VOLTAGE, 0),
};

/* Same as the BSP: ADC on, zero-current sampling and background channels off */
static int mt6375_adc_reset(struct mt6375_adc *adc)
{
	__be16 chen = 0;
	unsigned int cfg1;
	int ret;

	ret = regmap_read(adc->regmap, MT6375_REG_ADC_CFG1, &cfg1);
	if (ret)
		return ret;

	dev_dbg(adc->dev, "boot: cfg1 0x%02x\n", cfg1);

	ret = regmap_update_bits(adc->regmap, MT6375_REG_ADC_CFG1,
				 MT6375_ADCEN | MT6375_ZCVEN, MT6375_ADCEN);
	if (ret)
		return ret;

	return regmap_raw_write(adc->regmap, MT6375_REG_ADC_CHEN, &chen,
				sizeof(chen));
}

static int mt6375_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct mt6375_adc *adc;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->dev = dev;
	adc->regmap = dev_get_regmap(dev->parent, NULL);
	if (!adc->regmap)
		return dev_err_probe(dev, -ENODEV, "no parent regmap\n");

	ret = devm_mutex_init(dev, &adc->lock);
	if (ret)
		return ret;

	ret = mt6375_adc_reset(adc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to reset ADC\n");

	indio_dev->name = "mt6375-adc";
	indio_dev->info = &mt6375_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = mt6375_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(mt6375_adc_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id mt6375_adc_of_match[] = {
	{ .compatible = "mediatek,mt6375-adc" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6375_adc_of_match);

static struct platform_driver mt6375_adc_driver = {
	.driver = {
		.name = "mt6375-adc",
		.of_match_table = mt6375_adc_of_match,
	},
	.probe = mt6375_adc_probe,
};
module_platform_driver(mt6375_adc_driver);

MODULE_DESCRIPTION("MediaTek MT6375 ADC driver");
MODULE_LICENSE("GPL");
