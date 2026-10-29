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
 * TF-A calls, enables no interrupt (with data=1 the DPMAIF one is
 * requested, disabled) and writes no register or shared memory. With
 * ccif_regs=1 it reads CCIF0 to tell what state LK left the modem in.
 * Restarting the modem the downstream way is a debugfs command, refused unless
 * power_ctrl=1 and the DT describes everything it needs.
 *
 * System sleep (s2idle) works with the modem running, as on the downstream
 * kernel: the modem keeps its network registration and wakes the AP for
 * an incoming call, SMS or downlink data. The modem power domain is left
 * alone by genpd because the device is a wakeup device in an active-wakeup
 * domain; the CCIF and watchdog interrupts are armed as wakeup interrupts;
 * the FSM is locked for the duration. The per-block work is in
 * mtk_ccci_ccif.c and mtk_ccci_hif.c.
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
#include <linux/pm_wakeup.h>
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

	ret = mtk_ccci_ee_init(ccci);
	if (ret)
		return ret;

	/* only with data=1; allocates, set up only with the modem start */
	ret = mtk_ccci_dpmaif_init(ccci);
	if (ret)
		return ret;

	/*
	 * The modem wakes the AP from system sleep. Being a wakeup device
	 * also puts this device on the wakeup path in suspend, which keeps
	 * genpd from switching the modem power domain (GENPD_FLAG_ACTIVE_WAKEUP
	 * on the MD domain): off behind the running modem, or on again after
	 * a stop. Userspace can turn it off in power/wakeup; suspend is then
	 * refused while the domain is attached, see mtk_ccci_prepare().
	 */
	ret = devm_device_init_wakeup(dev);
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
	mtk_ccci_ee_debugfs_init(ccci);
	mtk_ccci_dpmaif_debugfs_init(ccci);

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
 * System sleep, modelled on the downstream kernel, which keeps the modem running
 * and never touches the MTCMOS in suspend:
 *
 *  prepare       the FSM is locked, no command switches the modem until
 *                complete; refused if the device may not wake the system
 *                (genpd would then switch the domain, see below)
 *  suspend       nothing; the PM core puts the device on the wakeup path
 *                because it may wake the system, so genpd's noirq callbacks
 *                leave the modem power domain as it is (active wakeup)
 *  suspend_late  the CCIF and watchdog interrupts become wakeup interrupts
 *  suspend_noirq the CCIF poll timer stops
 *  resume_noirq  TF-A is asked whether CCIF0 woke the AP; the RX worker
 *                takes what arrived
 *  resume_early  wakeup interrupts disarmed
 *
 * With the modem stopped or never started nothing but the lock happens:
 * the domain, off (or LK's, which genpd sees as off), stays as it is.
 */
static int mtk_ccci_prepare(struct device *dev)
{
	struct mtk_ccci *ccci = dev_get_drvdata(dev);

	if (!mutex_trylock(&ccci->fsm_lock)) {
		dev_warn(dev, "modem command in progress, not suspending\n");
		return -EBUSY;
	}
	if (ccci->md_domain && !device_may_wakeup(dev)) {
		dev_warn(dev, "wakeup disabled: genpd would switch the modem power domain, not suspending\n");
		mutex_unlock(&ccci->fsm_lock);
		return -EBUSY;
	}
	return 0;
}

static void mtk_ccci_complete(struct device *dev)
{
	struct mtk_ccci *ccci = dev_get_drvdata(dev);

	mutex_unlock(&ccci->fsm_lock);
}

static int mtk_ccci_suspend(struct device *dev)
{
	struct mtk_ccci *ccci = dev_get_drvdata(dev);

	dev_info(dev, "suspending, modem %s%s\n",
		 mtk_ccci_fsm_state_name(ccci->state),
		 ccci->md_domain_held ? " (power domain held)" : "");
	return 0;
}

static int mtk_ccci_suspend_late(struct device *dev)
{
	struct mtk_ccci *ccci = dev_get_drvdata(dev);

	mtk_ccci_irq_wake(ccci, true);
	return 0;
}

static int mtk_ccci_suspend_noirq(struct device *dev)
{
	struct mtk_ccci *ccci = dev_get_drvdata(dev);

	if (READ_ONCE(ccci->hif_on))
		ccci->sleep.suspends++;
	mtk_ccci_hif_suspend(ccci);
	return 0;
}

static int mtk_ccci_resume_noirq(struct device *dev)
{
	struct mtk_ccci *ccci = dev_get_drvdata(dev);

	mtk_ccci_ccif_resume_noirq(ccci);
	mtk_ccci_hif_resume(ccci);
	return 0;
}

static int mtk_ccci_resume_early(struct device *dev)
{
	struct mtk_ccci *ccci = dev_get_drvdata(dev);

	mtk_ccci_irq_wake(ccci, false);
	return 0;
}

static int mtk_ccci_resume(struct device *dev)
{
	struct mtk_ccci *ccci = dev_get_drvdata(dev);

	dev_info(dev, "resumed, modem %s%s\n",
		 mtk_ccci_fsm_state_name(ccci->state),
		 READ_ONCE(ccci->exception) ? ", EXCEPTION flagged" : "");
	return 0;
}

static const struct dev_pm_ops mtk_ccci_pm_ops = {
	.prepare = mtk_ccci_prepare,
	.complete = mtk_ccci_complete,
	.suspend = mtk_ccci_suspend,
	.resume = mtk_ccci_resume,
	.suspend_late = mtk_ccci_suspend_late,
	.resume_early = mtk_ccci_resume_early,
	.suspend_noirq = mtk_ccci_suspend_noirq,
	.resume_noirq = mtk_ccci_resume_noirq,
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
