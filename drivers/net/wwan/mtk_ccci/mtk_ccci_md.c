// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: driver for the modem built into MediaTek smartphone SoCs
 * (MT6878: modem generation 6299, CCIF control path + DPMAIF v3 data path).
 *
 * Stage 1 only finds out what the bootloader set up and checks it:
 *  - the reserved memory LK carved out for the modem and the AP/modem
 *    shared memory,
 *  - the modem arguments LK left in its tag area,
 *  - the CCIF (AP side) register window.
 *
 * It does not power, clock, reset or start the modem, issues no TF-A
 * calls, requests no interrupt and writes no register or shared memory.
 * The modem power domain is deliberately absent from the device tree.
 *
 * Copyright (C) 2015 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>

#include "mtk_ccci.h"

/* CCIF, AP side */
#define CCIF_CON		0x00
#define CCIF_BUSY		0x04
#define CCIF_RCHNUM		0x10
#define CCIF_CHDATA		0x100
#define CCIF_SRAM_SIZE_DEFAULT	512

/*
 * Reading the CCIF registers needs the CCIF clocks, which stage 1 does
 * not take: they are on only because LK started the modem and the kernel
 * runs with clk_ignore_unused. Off by default.
 */
static bool ccif_regs;
module_param(ccif_regs, bool, 0444);
MODULE_PARM_DESC(ccif_regs, "expose read-only CCIF registers in debugfs");

static const char * const mtk_ccci_resv_compat[MTK_CCCI_RESV_NUM] = {
	[MTK_CCCI_RESV_TAGS] = "mediatek,ccci_tag_mem",
	[MTK_CCCI_RESV_NC_SMEM] = "mediatek,ap_md_nc_smem",
	[MTK_CCCI_RESV_C_SMEM] = "mediatek,ap_md_c_smem",
};

/*
 * LK creates the carveouts at boot ("/reserved-memory/mblock-N-<name>",
 * no-map, no phandle), so they can only be found by compatible.
 */
static int mtk_ccci_find_resv(struct mtk_ccci *ccci)
{
	struct device_node *parent;
	unsigned int i;
	int ret = 0;

	parent = of_find_node_by_path("/reserved-memory");
	if (!parent)
		return -ENODEV;

	for_each_available_child_of_node_scoped(parent, np) {
		struct resource r;

		if (of_address_to_resource(np, 0, &r))
			continue;

		if (of_device_is_compatible(np, "mediatek,md_mem_usage")) {
			if (ccci->md_mem_num < MTK_CCCI_MD_MEM_MAX)
				ccci->md_mem[ccci->md_mem_num++] = r;
			continue;
		}

		for (i = 0; i < MTK_CCCI_RESV_NUM; i++) {
			if (!of_device_is_compatible(np, mtk_ccci_resv_compat[i]))
				continue;
			if (!of_property_read_bool(np, "no-map")) {
				dev_err(ccci->dev, "%pOF is mapped\n", np);
				ret = -EINVAL;
			}
			ccci->resv[i] = r;
		}
	}
	of_node_put(parent);
	if (ret)
		return ret;

	for (i = 0; i < MTK_CCCI_RESV_NUM; i++) {
		if (!ccci->resv[i].end) {
			dev_info(ccci->dev, "no %s carveout\n",
				 mtk_ccci_resv_compat[i]);
			return -ENODEV;
		}
		dev_dbg(ccci->dev, "%s: %pR\n", mtk_ccci_resv_compat[i],
			&ccci->resv[i]);
	}
	if (!ccci->md_mem_num) {
		dev_info(ccci->dev, "no modem memory carveout\n");
		return -ENODEV;
	}

	return 0;
}

static int mtk_ccci_ccif_show(struct seq_file *s, void *unused)
{
	const struct mtk_ccci *ccci = s->private;
	u32 off;

	seq_printf(s, "con    %#010x\n", readl(ccci->ccif_ap + CCIF_CON));
	seq_printf(s, "busy   %#010x\n", readl(ccci->ccif_ap + CCIF_BUSY));
	seq_printf(s, "rchnum %#010x\n", readl(ccci->ccif_ap + CCIF_RCHNUM));

	for (off = 0; off < ccci->ccif_sram_size; off += 16)
		seq_printf(s, "sram+%03x %08x %08x %08x %08x\n", off,
			   readl(ccci->ccif_ap + CCIF_CHDATA + off),
			   readl(ccci->ccif_ap + CCIF_CHDATA + off + 4),
			   readl(ccci->ccif_ap + CCIF_CHDATA + off + 8),
			   readl(ccci->ccif_ap + CCIF_CHDATA + off + 12));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_ccci_ccif);

static int mtk_ccci_md_mem_show(struct seq_file *s, void *unused)
{
	const struct mtk_ccci *ccci = s->private;
	unsigned int i;

	for (i = 0; i < MTK_CCCI_RESV_NUM; i++)
		seq_printf(s, "%-24s %pa+%#llx\n", mtk_ccci_resv_compat[i],
			   &ccci->resv[i].start,
			   (u64)resource_size(&ccci->resv[i]));
	for (i = 0; i < ccci->md_mem_num; i++)
		seq_printf(s, "%-24s %pa+%#llx\n", "mediatek,md_mem_usage",
			   &ccci->md_mem[i].start,
			   (u64)resource_size(&ccci->md_mem[i]));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_ccci_md_mem);

static int mtk_ccci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_bulk_data *clks;
	struct mtk_ccci *ccci;
	int ret;

	ccci = devm_kzalloc(dev, sizeof(*ccci), GFP_KERNEL);
	if (!ccci)
		return -ENOMEM;
	ccci->dev = dev;
	platform_set_drvdata(pdev, ccci);

	ret = mtk_ccci_find_resv(ccci);
	if (ret)
		return dev_err_probe(dev, ret, "modem memory not reserved\n");

	ret = mtk_ccci_lk_parse(ccci);
	if (ret)
		return dev_err_probe(dev, ret, "no usable modem setup from LK\n");

	ccci->ccif_ap = devm_platform_ioremap_resource_byname(pdev, "ccif-ap");
	if (IS_ERR(ccci->ccif_ap))
		return PTR_ERR(ccci->ccif_ap);
	if (of_property_read_u32(dev->of_node, "mediatek,ccif-sram-size",
				 &ccci->ccif_sram_size))
		ccci->ccif_sram_size = CCIF_SRAM_SIZE_DEFAULT;
	ccci->ccif_sram_size = min(ccci->ccif_sram_size, 0x1000U - CCIF_CHDATA);

	/* only checks the description; enabling comes with the power-up */
	ret = devm_clk_bulk_get_all(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	ccci->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("md_mem", 0400, ccci->debugfs, ccci,
			    &mtk_ccci_md_mem_fops);
	mtk_ccci_lk_debugfs_init(ccci);
	if (ccif_regs)
		debugfs_create_file("ccif", 0400, ccci->debugfs, ccci,
				    &mtk_ccci_ccif_fops);

	dev_info(dev, "modem set up by LK, left alone (stage 1)\n");
	return 0;
}

static void mtk_ccci_remove(struct platform_device *pdev)
{
	struct mtk_ccci *ccci = platform_get_drvdata(pdev);

	debugfs_remove_recursive(ccci->debugfs);
}

static const struct of_device_id mtk_ccci_of_match[] = {
	{ .compatible = "mediatek,mt6878-ccci" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_ccci_of_match);

static struct platform_driver mtk_ccci_driver = {
	.probe = mtk_ccci_probe,
	.remove = mtk_ccci_remove,
	.driver = {
		.name = "mtk-ccci-md",
		.of_match_table = mtk_ccci_of_match,
	},
};
module_platform_driver(mtk_ccci_driver);

MODULE_DESCRIPTION("MediaTek CCCI driver for SoC-integrated modems");
MODULE_LICENSE("GPL");
