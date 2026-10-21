// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: driver for the modem built into MediaTek smartphone SoCs
 * (MT6878: modem generation 6299, CCIF control path + DPMAIF v3 data path).
 *
 * At probe the driver only finds out what the bootloader set up and
 * checks it:
 *  - the reserved memory LK carved out for the modem and the AP/modem
 *    shared memory,
 *  - the modem arguments LK left in its tag area,
 *  - the CCIF0 windows, clocks, supplies (got, never switched).
 *
 * Probing does not power, clock, reset or start the modem, issues no
 * TF-A calls, requests no interrupt and writes no register or shared
 * memory. With ccif_regs=1 it reads CCIF0 to tell what state LK left the
 * modem in. Restarting the modem the stock way is a debugfs command,
 * refused unless power_ctrl=1 and the DT describes everything it needs.
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

	ret = mtk_ccci_ccif_init(ccci);
	if (ret)
		return ret;

	ret = mtk_ccci_hif_init(ccci);
	if (ret)
		return ret;

	ret = mtk_ccci_fsm_init(ccci);
	if (ret)
		return ret;

	ret = mtk_ccci_port_init(ccci);
	if (ret)
		return ret;

	ret = mtk_ccci_cdev_init(ccci);
	if (ret)
		return ret;

	/* only with data=1; allocates, never starts */
	ret = mtk_ccci_dpmaif_init(ccci);
	if (ret)
		return ret;

	ccci->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("md_mem", 0400, ccci->debugfs, ccci,
			    &mtk_ccci_md_mem_fops);
	mtk_ccci_lk_debugfs_init(ccci);
	mtk_ccci_ccif_debugfs_init(ccci);
	mtk_ccci_fsm_debugfs_init(ccci);
	mtk_ccci_hif_debugfs_init(ccci);
	mtk_ccci_port_debugfs_init(ccci);

	/* only with ccif_regs=1: reads, logs what LK left */
	mtk_ccci_ccif_log_state(ccci);

	dev_info(dev, "modem set up by LK, left alone\n");
	return 0;
}

static void mtk_ccci_remove(struct platform_device *pdev)
{
	struct mtk_ccci *ccci = platform_get_drvdata(pdev);

	debugfs_remove_recursive(ccci->debugfs);
}

/*
 * genpd switches a domain off for system suspend and on again on resume
 * whatever its runtime state: that would stop the modem LK started, or
 * power the modem domain up after a stop. Until suspend is handled, a
 * board with the modem power domain does not suspend.
 */
static int mtk_ccci_prepare(struct device *dev)
{
	struct mtk_ccci *ccci = dev_get_drvdata(dev);

	if (!ccci->md_domain)
		return 0;
	dev_warn(dev, "system suspend with the modem power domain is not supported\n");
	return -EBUSY;
}

static const struct dev_pm_ops mtk_ccci_pm_ops = {
	.prepare = mtk_ccci_prepare,
};

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
		.pm = pm_sleep_ptr(&mtk_ccci_pm_ops),
		/* unbinding could power the modem off, see mtk_ccci_fsm_init() */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(mtk_ccci_driver);

MODULE_DESCRIPTION("MediaTek CCCI driver for SoC-integrated modems");
MODULE_LICENSE("GPL");
