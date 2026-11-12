// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * Author: Chunfeng Yun <chunfeng.yun@mediatek.com>
 */

#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/reset.h>

#include "mtu3.h"
#include "mtu3_dr.h"
#include "mtu3_debug.h"

/*
 * A device-only controller may come back from system sleep with the IP
 * (IPPC and MAC) at reset values: on MT6878 the gadget does not raise D+
 * again after a suspend through the SPM, while it does after a pm_test
 * one. 1 (the default): as the downstream driver does at every connect,
 * reset the IP and write the probe and start settings again at system
 * resume when no host is connected; 0: leave the IP as it is.
 */
static int reinit_on_resume = 1;
module_param(reinit_on_resume, int, 0644);
MODULE_PARM_DESC(reinit_on_resume, "Device mode: reset and set up the IP again at system resume (default 1)");

/*
 * The ip-sleep wakeup of the glue signals the SPM while the IP is not in
 * sleep: enabled with the IP awake (device role without D+, e.g. no gadget
 * bound) it wakes the system at once, as USB_POWERDWN_B with nothing slept
 * on MT6878. The downstream driver of that SoC enables it in host mode
 * only (in device mode its system suspend leaves the controller alone).
 * 0 (the default): enable the glue in the host role only; 1: in the device
 * role, too. Never while the IP is awake, nor unless the controller may
 * wake the system.
 */
static bool uwk_device_role;
module_param(uwk_device_role, bool, 0644);
MODULE_PARM_DESC(uwk_device_role, "Enable the wakeup glue in the device role, too (default 0)");

static const u32 ssusb_sleep_regs[] = {
	U3D_SSUSB_IP_PW_CTRL0, U3D_SSUSB_IP_PW_CTRL2,
	U3D_SSUSB_U2_CTRL_0P, U3D_SSUSB_IP_PW_STS1,
};

static void ssusb_sleep_regs_read(struct ssusb_mtk *ssusb, u32 *v)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ssusb_sleep_regs); i++)
		v[i] = mtu3_readl(ssusb->ippc_base, ssusb_sleep_regs[i]);
}

/* u2-port0 should be powered on and enabled; */
int ssusb_check_clocks(struct ssusb_mtk *ssusb, u32 ex_clks)
{
	void __iomem *ibase = ssusb->ippc_base;
	u32 value, check_val;
	int ret;

	check_val = ex_clks | SSUSB_SYS125_RST_B_STS | SSUSB_SYSPLL_STABLE |
			SSUSB_REF_RST_B_STS;

	ret = readl_poll_timeout(ibase + U3D_SSUSB_IP_PW_STS1, value,
			(check_val == (value & check_val)), 100, 20000);
	if (ret) {
		dev_err(ssusb->dev, "clks of sts1 are not stable!\n");
		return ret;
	}

	ret = readl_poll_timeout(ibase + U3D_SSUSB_IP_PW_STS2, value,
			(value & SSUSB_U2_MAC_SYS_RST_B_STS), 100, 10000);
	if (ret) {
		dev_err(ssusb->dev, "mac2 clock is not stable\n");
		return ret;
	}

	return 0;
}

static int wait_for_ip_sleep(struct ssusb_mtk *ssusb)
{
	bool sleep_check = true;
	u32 value;
	int ret;

	if (!ssusb->is_host)
		sleep_check = ssusb_gadget_ip_sleep_check(ssusb);

	if (!sleep_check)
		return 0;

	/* wait for ip enter sleep mode */
	ret = readl_poll_timeout(ssusb->ippc_base + U3D_SSUSB_IP_PW_STS1, value,
				 (value & SSUSB_IP_SLEEP_STS), 100, 100000);
	if (ret) {
		dev_err(ssusb->dev, "ip sleep failed!!!\n");
		ret = -EBUSY;
	} else {
		/* workaround: avoid wrong wakeup signal latch for some soc */
		usleep_range(100, 200);
	}

	return ret;
}

/* Whether the wakeup glue is to be enabled for this suspend; clocks on */
static bool ssusb_uwk_wanted(struct ssusb_mtk *ssusb, pm_message_t msg)
{
	const char *why = NULL;
	u32 sts = 0;

	if (!ssusb->uwk_en)
		return false;

	if (!device_may_wakeup(ssusb->dev)) {
		why = "may not wake the system";
	} else if (!ssusb->is_host && !uwk_device_role) {
		why = "device role";
	} else {
		sts = mtu3_readl(ssusb->ippc_base, U3D_SSUSB_IP_PW_STS1);
		if (!(sts & SSUSB_IP_SLEEP_STS))
			why = "IP not in sleep";
	}

	if (msg.event == PM_EVENT_SUSPEND)
		dev_info(ssusb->dev, "wakeup glue %s%s (pw_sts1 %#x)\n",
			 why ? "not enabled: " : "enabled", why ? why : "", sts);

	return !why;
}

static int ssusb_phy_init(struct ssusb_mtk *ssusb)
{
	int i;
	int ret;

	for (i = 0; i < ssusb->num_phys; i++) {
		ret = phy_init(ssusb->phys[i]);
		if (ret)
			goto exit_phy;
	}
	return 0;

exit_phy:
	for (; i > 0; i--)
		phy_exit(ssusb->phys[i - 1]);

	return ret;
}

static int ssusb_phy_exit(struct ssusb_mtk *ssusb)
{
	int i;

	for (i = 0; i < ssusb->num_phys; i++)
		phy_exit(ssusb->phys[i]);

	return 0;
}

static int ssusb_phy_power_on(struct ssusb_mtk *ssusb)
{
	int i;
	int ret;

	for (i = 0; i < ssusb->num_phys; i++) {
		ret = phy_power_on(ssusb->phys[i]);
		if (ret)
			goto power_off_phy;
	}
	return 0;

power_off_phy:
	for (; i > 0; i--)
		phy_power_off(ssusb->phys[i - 1]);

	return ret;
}

static void ssusb_phy_power_off(struct ssusb_mtk *ssusb)
{
	unsigned int i;

	for (i = 0; i < ssusb->num_phys; i++)
		phy_power_off(ssusb->phys[i]);
}

static int ssusb_rscs_init(struct ssusb_mtk *ssusb)
{
	int ret = 0;

	ret = regulator_enable(ssusb->vusb33);
	if (ret) {
		dev_err(ssusb->dev, "failed to enable vusb33\n");
		goto vusb33_err;
	}

	ret = clk_bulk_prepare_enable(BULK_CLKS_CNT, ssusb->clks);
	if (ret)
		goto clks_err;

	ret = ssusb_phy_init(ssusb);
	if (ret) {
		dev_err(ssusb->dev, "failed to init phy\n");
		goto phy_init_err;
	}

	ret = ssusb_phy_power_on(ssusb);
	if (ret) {
		dev_err(ssusb->dev, "failed to power on phy\n");
		goto phy_err;
	}

	return 0;

phy_err:
	ssusb_phy_exit(ssusb);
phy_init_err:
	clk_bulk_disable_unprepare(BULK_CLKS_CNT, ssusb->clks);
clks_err:
	regulator_disable(ssusb->vusb33);
vusb33_err:
	return ret;
}

static void ssusb_rscs_exit(struct ssusb_mtk *ssusb)
{
	clk_bulk_disable_unprepare(BULK_CLKS_CNT, ssusb->clks);
	regulator_disable(ssusb->vusb33);
	ssusb_phy_power_off(ssusb);
	ssusb_phy_exit(ssusb);
}

static void ssusb_ip_sw_reset(struct ssusb_mtk *ssusb)
{
	/* reset whole ip (xhci & u3d) */
	mtu3_setbits(ssusb->ippc_base, U3D_SSUSB_IP_PW_CTRL0, SSUSB_IP_SW_RST);
	udelay(1);
	mtu3_clrbits(ssusb->ippc_base, U3D_SSUSB_IP_PW_CTRL0, SSUSB_IP_SW_RST);

	/*
	 * device ip may be powered on in firmware/BROM stage before entering
	 * kernel stage;
	 * power down device ip, otherwise ip-sleep will fail when working as
	 * host only mode
	 */
	mtu3_setbits(ssusb->ippc_base, U3D_SSUSB_IP_PW_CTRL2, SSUSB_IP_DEV_PDN);
}

static void ssusb_u3_drd_check(struct ssusb_mtk *ssusb)
{
	struct otg_switch_mtk *otg_sx = &ssusb->otg_switch;
	u32 dev_u3p_num;
	u32 host_u3p_num;
	u32 value;

	/* u3 port0 is disabled */
	if (ssusb->u3p_dis_msk & BIT(0)) {
		otg_sx->is_u3_drd = false;
		goto out;
	}

	value = mtu3_readl(ssusb->ippc_base, U3D_SSUSB_IP_DEV_CAP);
	dev_u3p_num = SSUSB_IP_DEV_U3_PORT_NUM(value);

	value = mtu3_readl(ssusb->ippc_base, U3D_SSUSB_IP_XHCI_CAP);
	host_u3p_num = SSUSB_IP_XHCI_U3_PORT_NUM(value);

	otg_sx->is_u3_drd = !!(dev_u3p_num && host_u3p_num);

out:
	dev_info(ssusb->dev, "usb3-drd: %d\n", otg_sx->is_u3_drd);
}

static int get_ssusb_rscs(struct platform_device *pdev, struct ssusb_mtk *ssusb)
{
	struct device_node *node = pdev->dev.of_node;
	struct otg_switch_mtk *otg_sx = &ssusb->otg_switch;
	struct clk_bulk_data *clks = ssusb->clks;
	struct device *dev = &pdev->dev;
	int i;
	int ret;

	ssusb->vusb33 = devm_regulator_get(dev, "vusb33");
	if (IS_ERR(ssusb->vusb33)) {
		dev_err(dev, "failed to get vusb33\n");
		return PTR_ERR(ssusb->vusb33);
	}

	clks[0].id = "sys_ck";
	clks[1].id = "ref_ck";
	clks[2].id = "mcu_ck";
	clks[3].id = "dma_ck";
	clks[4].id = "xhci_ck";
	clks[5].id = "frmcnt_ck";
	ret = devm_clk_bulk_get_optional(dev, BULK_CLKS_CNT, clks);
	if (ret)
		return ret;

	ssusb->num_phys = of_count_phandle_with_args(node,
			"phys", "#phy-cells");
	if (ssusb->num_phys > 0) {
		ssusb->phys = devm_kcalloc(dev, ssusb->num_phys,
					sizeof(*ssusb->phys), GFP_KERNEL);
		if (!ssusb->phys)
			return -ENOMEM;
	} else {
		ssusb->num_phys = 0;
	}

	for (i = 0; i < ssusb->num_phys; i++) {
		ssusb->phys[i] = devm_of_phy_get_by_index(dev, node, i);
		if (IS_ERR(ssusb->phys[i])) {
			dev_err(dev, "failed to get phy-%d\n", i);
			return PTR_ERR(ssusb->phys[i]);
		}
	}

	ssusb->ippc_base = devm_platform_ioremap_resource_byname(pdev, "ippc");
	if (IS_ERR(ssusb->ippc_base))
		return PTR_ERR(ssusb->ippc_base);

	ssusb->wakeup_irq = platform_get_irq_byname_optional(pdev, "wakeup");
	if (ssusb->wakeup_irq == -EPROBE_DEFER)
		return ssusb->wakeup_irq;

	ssusb->dr_mode = usb_get_dr_mode(dev);
	if (ssusb->dr_mode == USB_DR_MODE_UNKNOWN)
		ssusb->dr_mode = USB_DR_MODE_OTG;

	of_property_read_u32(node, "mediatek,u3p-dis-msk", &ssusb->u3p_dis_msk);

	if (ssusb->dr_mode == USB_DR_MODE_PERIPHERAL)
		goto out;

	/* if host role is supported */
	ret = ssusb_wakeup_of_property_parse(ssusb, node);
	if (ret) {
		dev_err(dev, "failed to parse uwk property\n");
		return ret;
	}

	/* optional property, ignore the error if it does not exist */
	of_property_read_u32(node, "mediatek,u2p-dis-msk",
			     &ssusb->u2p_dis_msk);

	otg_sx->vbus = devm_regulator_get(dev, "vbus");
	if (IS_ERR(otg_sx->vbus)) {
		dev_err(dev, "failed to get vbus\n");
		return PTR_ERR(otg_sx->vbus);
	}

	if (ssusb->dr_mode == USB_DR_MODE_HOST)
		goto out;

	/* if dual-role mode is supported */
	otg_sx->manual_drd_enabled =
		of_property_read_bool(node, "enable-manual-drd");
	otg_sx->role_sw_used = of_property_read_bool(node, "usb-role-switch");

	/* can't disable port0 when use dual-role mode */
	ssusb->u2p_dis_msk &= ~0x1;

	if (otg_sx->role_sw_used || otg_sx->manual_drd_enabled)
		goto out;

	if (of_property_present(node, "extcon")) {
		otg_sx->edev = extcon_get_edev_by_phandle(ssusb->dev, 0);
		if (IS_ERR(otg_sx->edev)) {
			return dev_err_probe(dev, PTR_ERR(otg_sx->edev),
					     "couldn't get extcon device\n");
		}
	}

out:
	dev_info(dev, "dr_mode: %d, drd: %s\n", ssusb->dr_mode,
		 otg_sx->manual_drd_enabled ? "manual" : "auto");
	dev_info(dev, "u2p_dis_msk: %x, u3p_dis_msk: %x\n",
		 ssusb->u2p_dis_msk, ssusb->u3p_dis_msk);

	return 0;
}

static int mtu3_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct ssusb_mtk *ssusb;
	int ret = -ENOMEM;

	/* all elements are set to ZERO as default value */
	ssusb = devm_kzalloc(dev, sizeof(*ssusb), GFP_KERNEL);
	if (!ssusb)
		return -ENOMEM;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "No suitable DMA config available\n");
		return -ENOTSUPP;
	}

	platform_set_drvdata(pdev, ssusb);
	ssusb->dev = dev;

	ret = get_ssusb_rscs(pdev, ssusb);
	if (ret)
		return ret;

	ssusb_debugfs_create_root(ssusb);

	/* enable power domain */
	pm_runtime_set_active(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 4000);
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	device_init_wakeup(dev, true);

	ret = ssusb_rscs_init(ssusb);
	if (ret)
		goto comm_init_err;

	if (ssusb->wakeup_irq > 0) {
		ret = dev_pm_set_dedicated_wake_irq_reverse(dev, ssusb->wakeup_irq);
		if (ret) {
			dev_err(dev, "failed to set wakeup irq %d\n", ssusb->wakeup_irq);
			goto comm_exit;
		}
		dev_info(dev, "wakeup irq %d\n", ssusb->wakeup_irq);
	}

	ret = device_reset_optional(dev);
	if (ret) {
		dev_err_probe(dev, ret, "failed to reset controller\n");
		goto comm_exit;
	}

	ssusb_ip_sw_reset(ssusb);
	ssusb_u3_drd_check(ssusb);

	if (IS_ENABLED(CONFIG_USB_MTU3_HOST))
		ssusb->dr_mode = USB_DR_MODE_HOST;
	else if (IS_ENABLED(CONFIG_USB_MTU3_GADGET))
		ssusb->dr_mode = USB_DR_MODE_PERIPHERAL;

	/* default as host */
	ssusb->is_host = !(ssusb->dr_mode == USB_DR_MODE_PERIPHERAL);

	switch (ssusb->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		ret = ssusb_gadget_init(ssusb);
		if (ret) {
			dev_err(dev, "failed to initialize gadget\n");
			goto comm_exit;
		}
		break;
	case USB_DR_MODE_HOST:
		ret = ssusb_host_init(ssusb, node);
		if (ret) {
			dev_err(dev, "failed to initialize host\n");
			goto comm_exit;
		}
		break;
	case USB_DR_MODE_OTG:
		ret = ssusb_gadget_init(ssusb);
		if (ret) {
			dev_err(dev, "failed to initialize gadget\n");
			goto comm_exit;
		}

		ret = ssusb_host_init(ssusb, node);
		if (ret) {
			dev_err(dev, "failed to initialize host\n");
			goto gadget_exit;
		}

		ret = ssusb_otg_switch_init(ssusb);
		if (ret) {
			dev_err(dev, "failed to initialize switch\n");
			goto host_exit;
		}
		break;
	default:
		dev_err(dev, "unsupported mode: %d\n", ssusb->dr_mode);
		ret = -EINVAL;
		goto comm_exit;
	}

	device_enable_async_suspend(dev);
	pm_runtime_put_autosuspend(dev);
	pm_runtime_forbid(dev);

	return 0;

host_exit:
	ssusb_host_exit(ssusb);
gadget_exit:
	ssusb_gadget_exit(ssusb);
comm_exit:
	ssusb_rscs_exit(ssusb);
comm_init_err:
	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);
	ssusb_debugfs_remove_root(ssusb);

	return ret;
}

static void mtu3_remove(struct platform_device *pdev)
{
	struct ssusb_mtk *ssusb = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);

	switch (ssusb->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		ssusb_gadget_exit(ssusb);
		break;
	case USB_DR_MODE_HOST:
		ssusb_host_exit(ssusb);
		break;
	case USB_DR_MODE_OTG:
		ssusb_otg_switch_exit(ssusb);
		ssusb_gadget_exit(ssusb);
		ssusb_host_exit(ssusb);
		break;
	case USB_DR_MODE_UNKNOWN:
		/*
		 * This cannot happen because with dr_mode ==
		 * USB_DR_MODE_UNKNOWN, .probe() doesn't succeed and so
		 * .remove() wouldn't be called at all. However (little
		 * surprising) the compiler isn't smart enough to see that, so
		 * we explicitly have this case item to not make the compiler
		 * wail about an unhandled enumeration value.
		 */
		break;
	}

	ssusb_rscs_exit(ssusb);
	ssusb_debugfs_remove_root(ssusb);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
}

static int resume_ip_and_ports(struct ssusb_mtk *ssusb, pm_message_t msg)
{
	switch (ssusb->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		ssusb_gadget_resume(ssusb, msg);
		break;
	case USB_DR_MODE_HOST:
		ssusb_host_resume(ssusb, false);
		break;
	case USB_DR_MODE_OTG:
		ssusb_host_resume(ssusb, !ssusb->is_host);
		if (!ssusb->is_host)
			ssusb_gadget_resume(ssusb, msg);

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mtu3_suspend_common(struct device *dev, pm_message_t msg)
{
	struct ssusb_mtk *ssusb = dev_get_drvdata(dev);
	int ret = 0;

	dev_dbg(dev, "%s\n", __func__);

	switch (ssusb->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		ret = ssusb_gadget_suspend(ssusb, msg);
		if (ret)
			goto err;

		break;
	case USB_DR_MODE_HOST:
		ssusb_host_suspend(ssusb);
		break;
	case USB_DR_MODE_OTG:
		if (!ssusb->is_host) {
			ret = ssusb_gadget_suspend(ssusb, msg);
			if (ret)
				goto err;
		}
		ssusb_host_suspend(ssusb);
		break;
	default:
		return -EINVAL;
	}

	ret = wait_for_ip_sleep(ssusb);
	if (ret)
		goto sleep_err;

	if (msg.event == PM_EVENT_SUSPEND)
		ssusb_sleep_regs_read(ssusb, ssusb->sleep_ippc);
	ssusb->uwk_armed = ssusb_uwk_wanted(ssusb, msg);
	ssusb_phy_power_off(ssusb);
	clk_bulk_disable_unprepare(BULK_CLKS_CNT, ssusb->clks);
	if (ssusb->uwk_armed)
		ssusb_wakeup_set(ssusb, true);
	return 0;

sleep_err:
	resume_ip_and_ports(ssusb, msg);
err:
	return ret;
}

/*
 * System resume, device mode or the device role of a dual role controller:
 * log the IPPC power and port control if it changed during the sleep, then
 * set the device part up again as probe and mtu3_start() did (the
 * downstream driver resets the IP at every connect). A device-only
 * controller is reset first. A dual role one only if the IP came back
 * from a power down (held in reset, its reset value): the reset takes the
 * host part with it, so its ports are set up again, too, and the xHCI
 * driver finds a controller that lost power.
 */
static void ssusb_resume_reinit(struct ssusb_mtk *ssusb)
{
	u32 now[ARRAY_SIZE(ssusb_sleep_regs)];
	bool otg_device = ssusb->dr_mode == USB_DR_MODE_OTG && !ssusb->is_host;
	int ret;

	if (ssusb->dr_mode != USB_DR_MODE_PERIPHERAL && !otg_device)
		return;

	ssusb_sleep_regs_read(ssusb, now);
	if (memcmp(now, ssusb->sleep_ippc, sizeof(now)))
		dev_info(ssusb->dev, "IPPC changed in sleep: pw_ctrl0 %#x->%#x pw_ctrl2 %#x->%#x u2_ctrl %#x->%#x pw_sts1 %#x->%#x\n",
			 ssusb->sleep_ippc[0], now[0], ssusb->sleep_ippc[1],
			 now[1], ssusb->sleep_ippc[2], now[2],
			 ssusb->sleep_ippc[3], now[3]);

	if (!reinit_on_resume || (ssusb->u3d && ssusb->u3d->connected))
		return;

	if (!otg_device) {
		ssusb_ip_sw_reset(ssusb);
	} else if (now[0] & SSUSB_IP_SW_RST) {
		ssusb_ip_sw_reset(ssusb);
		ret = ssusb_host_restore(ssusb);
		if (ret)
			dev_err(ssusb->dev, "host part restore failed: %d\n", ret);
	}
	if (otg_device)
		ssusb_set_force_mode(ssusb, MTU3_DR_FORCE_DEVICE);
	ret = ssusb_gadget_restore(ssusb);
	if (ret)
		dev_err(ssusb->dev, "device IP restore failed: %d\n", ret);
	else
		dev_dbg(ssusb->dev, "device IP set up again after sleep\n");
}

static int mtu3_resume_common(struct device *dev, pm_message_t msg)
{
	struct ssusb_mtk *ssusb = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	/* also clear a glue left enabled by an older kernel or the firmware */
	ssusb_wakeup_set(ssusb, false);
	ssusb->uwk_armed = false;
	ret = clk_bulk_prepare_enable(BULK_CLKS_CNT, ssusb->clks);
	if (ret)
		goto clks_err;

	ret = ssusb_phy_power_on(ssusb);
	if (ret)
		goto phy_err;

	if (msg.event == PM_EVENT_SUSPEND)
		ssusb_resume_reinit(ssusb);

	return resume_ip_and_ports(ssusb, msg);

phy_err:
	clk_bulk_disable_unprepare(BULK_CLKS_CNT, ssusb->clks);
clks_err:
	return ret;
}

static int __maybe_unused mtu3_suspend(struct device *dev)
{
	return mtu3_suspend_common(dev, PMSG_SUSPEND);
}

static int __maybe_unused mtu3_resume(struct device *dev)
{
	return mtu3_resume_common(dev, PMSG_SUSPEND);
}

static int __maybe_unused mtu3_runtime_suspend(struct device *dev)
{
	if (!device_may_wakeup(dev))
		return 0;

	return mtu3_suspend_common(dev, PMSG_AUTO_SUSPEND);
}

static int __maybe_unused mtu3_runtime_resume(struct device *dev)
{
	if (!device_may_wakeup(dev))
		return 0;

	return mtu3_resume_common(dev, PMSG_AUTO_SUSPEND);
}

static const struct dev_pm_ops mtu3_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mtu3_suspend, mtu3_resume)
	SET_RUNTIME_PM_OPS(mtu3_runtime_suspend,
			   mtu3_runtime_resume, NULL)
};

#define DEV_PM_OPS (IS_ENABLED(CONFIG_PM) ? &mtu3_pm_ops : NULL)

static const struct of_device_id mtu3_of_match[] = {
	{.compatible = "mediatek,mt8173-mtu3",},
	{.compatible = "mediatek,mtu3",},
	{},
};
MODULE_DEVICE_TABLE(of, mtu3_of_match);

static struct platform_driver mtu3_driver = {
	.probe = mtu3_probe,
	.remove = mtu3_remove,
	.driver = {
		.name = MTU3_DRIVER_NAME,
		.pm = DEV_PM_OPS,
		.of_match_table = mtu3_of_match,
	},
};
module_platform_driver(mtu3_driver);

MODULE_AUTHOR("Chunfeng Yun <chunfeng.yun@mediatek.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek USB3 DRD Controller Driver");
