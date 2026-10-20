// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek device information (efuse copy) handed over by the boot loader
 *
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Mac Lu <mac.lu@mediatek.com>
 *
 * On recent MediaTek smartphone SoCs the efuse block is not readable from
 * the normal world. The boot loader reads it and passes a copy in the
 * "atag,devinfo" property of /chosen: a little-endian word count followed
 * by that many little-endian 32-bit words. Expose the words as a read-only
 * NVMEM device so that the usual nvmem cells can describe calibration data
 * (thermal sensors, PHYs, ...).
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>

/* sanity limit on the word count found in the property */
#define MTK_DEVINFO_MAX_WORDS	1024

static int mtk_devinfo_read(void *context, unsigned int offset, void *val,
			    size_t bytes)
{
	const u8 *data = context;

	memcpy(val, data + offset, bytes);

	return 0;
}

static int mtk_devinfo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_config config = {};
	struct device_node *chosen;
	struct nvmem_device *nvmem;
	const __le32 *prop;
	u32 words;
	int len;
	void *data;

	chosen = of_find_node_by_path("/chosen");
	if (!chosen)
		return dev_err_probe(dev, -ENODEV, "no /chosen node\n");

	prop = of_get_property(chosen, "atag,devinfo", &len);
	if (!prop || len < sizeof(*prop)) {
		of_node_put(chosen);
		return dev_err_probe(dev, -ENODEV,
				     "no atag,devinfo from the boot loader\n");
	}

	words = le32_to_cpup(prop);
	if (!words || words > MTK_DEVINFO_MAX_WORDS ||
	    (words + 1) * sizeof(*prop) > len) {
		of_node_put(chosen);
		return dev_err_probe(dev, -EINVAL,
				     "bad atag,devinfo size %u (%d bytes)\n",
				     words, len);
	}

	data = devm_kmemdup(dev, prop + 1, words * sizeof(*prop), GFP_KERNEL);
	of_node_put(chosen);
	if (!data)
		return -ENOMEM;

	config.dev = dev;
	config.name = "mtk-devinfo";
	config.id = NVMEM_DEVID_NONE;
	config.owner = THIS_MODULE;
	config.read_only = true;
	config.root_only = true;
	config.add_legacy_fixed_of_cells = true;
	config.stride = 1;
	config.word_size = 1;
	config.size = words * sizeof(*prop);
	config.reg_read = mtk_devinfo_read;
	config.priv = data;

	nvmem = devm_nvmem_register(dev, &config);

	return PTR_ERR_OR_ZERO(nvmem);
}

static const struct of_device_id mtk_devinfo_of_match[] = {
	{ .compatible = "mediatek,mt6878-devinfo" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_devinfo_of_match);

static struct platform_driver mtk_devinfo_driver = {
	.probe = mtk_devinfo_probe,
	.driver = {
		.name = "mtk-devinfo",
		.of_match_table = mtk_devinfo_of_match,
	},
};
module_platform_driver(mtk_devinfo_driver);

MODULE_AUTHOR("Mac Lu <mac.lu@mediatek.com>");
MODULE_DESCRIPTION("MediaTek boot loader provided devinfo NVMEM driver");
MODULE_LICENSE("GPL");
