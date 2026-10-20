// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 *
 * MediaTek MT6685 clock buffer / RTC companion chip, SPMI core driver
 *
 * The MT6685 sits on the SPMI bus next to the PMICs and holds the 26 MHz
 * clock buffers (XO) and the RTC. This driver only provides the register
 * map (16 bit addresses, 8 bit registers) and creates the functions that
 * are described as child nodes in the device tree. It writes nothing.
 *
 * Based on the MediaTek downstream driver (drivers/mfd/mt6685-core.c,
 * Copyright (C) 2021 MediaTek Inc., author KY Liu).
 */

#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/spmi.h>

#define MT6685_HWCID_L		0x008
#define MT6685_HWCID_H		0x009

static const struct regmap_config mt6685_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x2000,
	.fast_io = true,
	.use_single_read = true,
	.use_single_write = true,
};

static int mt6685_probe(struct spmi_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct regmap *regmap;
	unsigned int lo, hi;
	int ret;

	regmap = devm_regmap_init_spmi_ext(sdev, &mt6685_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "failed to init regmap\n");

	ret = regmap_read(regmap, MT6685_HWCID_L, &lo);
	if (!ret)
		ret = regmap_read(regmap, MT6685_HWCID_H, &hi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read chip ID\n");

	dev_info(dev, "MT6685 chip ID %#04x%02x\n", hi, lo);

	return devm_of_platform_populate(dev);
}

static const struct of_device_id mt6685_of_match[] = {
	{ .compatible = "mediatek,mt6685" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6685_of_match);

static struct spmi_driver mt6685_driver = {
	.probe = mt6685_probe,
	.driver = {
		.name = "mt6685",
		.of_match_table = mt6685_of_match,
	},
};
module_spmi_driver(mt6685_driver);

MODULE_DESCRIPTION("MediaTek MT6685 SPMI core driver");
MODULE_LICENSE("GPL");
