// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CONNINFRA: power and bus infrastructure of the connectivity
 * subsystem (CONNSYS) shared by Wi-Fi, Bluetooth, GNSS and FM.
 *
 * Copyright (c) 2026 serlk
 *
 * The power-on order, the register offsets and the expected IP version
 * follow MediaTek's GPL-2.0 conninfra driver (conn_drv/connv2, platform
 * mt6878), Copyright (c) 2023 MediaTek Inc.
 *
 * This is the first stage only: it switches the CONN_INFRA "on" part on
 * (PMIC rails, the CONN power domain with its bus protection, the D-die
 * to A-die interface pins) and checks the CONN_INFRA IP version. The
 * D-die/A-die configuration, the EMI remapping and the subsystem API for
 * the Wi-Fi/BT drivers come later.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

/* CONN_INFRA_CFG (0x1801_1000), offset from the CONN_INFRA base */
#define CONN_INFRA_CFG_IP_VERSION	0x11000

/* TOPCKGEN: keep the host clock request of CONN_VON_TOP out of "always on" */
#define TOPCKGEN_CONN_CK_CTRL		0x504
#define  CONN_VON_TOP_HOST_CK_REQ_EN	BIT(3)

#define CONNINFRA_IP_VER_POLL_US	1000
#define CONNINFRA_IP_VER_TIMEOUT_US	11000

enum mtk_conninfra_adie {
	MTK_ADIE_MT6637,
	MTK_ADIE_MT6631,
	MTK_ADIE_MT6631_MT6686,
};

struct mtk_conninfra_data {
	u32 ip_version;
	enum mtk_conninfra_adie adie;
	const char *adie_name;
};

static const char * const mtk_conninfra_supplies[] = {
	"vcn13",	/* D-die AFE/WBG, A-die digital */
	"vrfio18",	/* A-die I/O, crystal */
};

struct mtk_conninfra {
	struct device *dev;
	void __iomem *base;
	struct regmap *topckgen;
	struct regulator_bulk_data supplies[ARRAY_SIZE(mtk_conninfra_supplies)];
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_idle;
	const struct mtk_conninfra_data *data;
	struct mutex lock;	/* protects users */
	unsigned int users;
};

static void mtk_conninfra_select_pins(struct mtk_conninfra *ci, bool active)
{
	struct pinctrl_state *state = active ? ci->pins_active : ci->pins_idle;
	int ret;

	if (IS_ERR_OR_NULL(ci->pinctrl) || IS_ERR_OR_NULL(state))
		return;

	ret = pinctrl_select_state(ci->pinctrl, state);
	if (ret)
		dev_warn(ci->dev, "failed to select %s pins: %d\n",
			 active ? "active" : "idle", ret);
}

static int mtk_conninfra_check_ip_version(struct mtk_conninfra *ci)
{
	u32 ver;
	int ret;

	ret = readl_poll_timeout(ci->base + CONN_INFRA_CFG_IP_VERSION, ver,
				 ver == ci->data->ip_version,
				 CONNINFRA_IP_VER_POLL_US,
				 CONNINFRA_IP_VER_TIMEOUT_US);
	if (ret) {
		dev_err(ci->dev, "CONN_INFRA IP version 0x%08x, expected 0x%08x\n",
			ver, ci->data->ip_version);
		return -ENODEV;
	}

	dev_info(ci->dev, "CONN_INFRA IP version 0x%08x, A-die %s\n",
		 ver, ci->data->adie_name);
	return 0;
}

static int __mtk_conninfra_power_on(struct mtk_conninfra *ci)
{
	int ret;

	/* PMIC: rails of the D-die AFE, the crystal and the A-die */
	ret = regulator_bulk_enable(ARRAY_SIZE(ci->supplies), ci->supplies);
	if (ret) {
		dev_err(ci->dev, "failed to enable supplies: %d\n", ret);
		return ret;
	}

	if (ci->topckgen)
		regmap_clear_bits(ci->topckgen, TOPCKGEN_CONN_CK_CTRL,
				  CONN_VON_TOP_HOST_CK_REQ_EN);

	/* D-die <-> A-die interface: GPIO172..182 to their CONN functions */
	mtk_conninfra_select_pins(ci, true);

	/*
	 * CONN power domain: power, clock, isolation and reset in SPM, then
	 * the AP<->CONNSYS bus protection in INFRACFG_AO.
	 */
	ret = pm_runtime_resume_and_get(ci->dev);
	if (ret) {
		dev_err(ci->dev, "failed to power on the CONN domain: %d\n", ret);
		goto err_pins;
	}

	ret = mtk_conninfra_check_ip_version(ci);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	pm_runtime_put_sync(ci->dev);
err_pins:
	mtk_conninfra_select_pins(ci, false);
	regulator_bulk_disable(ARRAY_SIZE(ci->supplies), ci->supplies);
	return ret;
}

static void __mtk_conninfra_power_off(struct mtk_conninfra *ci)
{
	pm_runtime_put_sync(ci->dev);
	mtk_conninfra_select_pins(ci, false);
	regulator_bulk_disable(ARRAY_SIZE(ci->supplies), ci->supplies);
}

static int mtk_conninfra_power_on(struct mtk_conninfra *ci)
{
	int ret = 0;

	mutex_lock(&ci->lock);
	if (!ci->users) {
		ret = __mtk_conninfra_power_on(ci);
		if (ret)
			goto out;
	}
	ci->users++;
out:
	mutex_unlock(&ci->lock);
	return ret;
}

static void mtk_conninfra_power_off(struct mtk_conninfra *ci)
{
	mutex_lock(&ci->lock);
	if (!WARN_ON(!ci->users) && !--ci->users)
		__mtk_conninfra_power_off(ci);
	mutex_unlock(&ci->lock);
}

static int mtk_conninfra_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_conninfra *ci;
	struct resource *res;
	unsigned int i;
	int ret;

	ci = devm_kzalloc(dev, sizeof(*ci), GFP_KERNEL);
	if (!ci)
		return -ENOMEM;

	ci->dev = dev;
	ci->data = of_device_get_match_data(dev);
	if (!ci->data)
		return -EINVAL;

	ret = devm_mutex_init(dev, &ci->lock);
	if (ret)
		return ret;

	/*
	 * The CONN_INFRA window is shared with the Wi-Fi and Bluetooth
	 * blocks, which map parts of it as well: do not claim the region.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	ci->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!ci->base)
		return -ENOMEM;

	ci->topckgen = syscon_regmap_lookup_by_phandle_optional(dev->of_node,
								"mediatek,topckgen");
	if (IS_ERR(ci->topckgen))
		return dev_err_probe(dev, PTR_ERR(ci->topckgen),
				     "invalid mediatek,topckgen\n");

	for (i = 0; i < ARRAY_SIZE(ci->supplies); i++)
		ci->supplies[i].supply = mtk_conninfra_supplies[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ci->supplies),
				      ci->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	ci->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ci->pinctrl)) {
		ret = PTR_ERR(ci->pinctrl);
		if (ret == -EPROBE_DEFER)
			return ret;
		ci->pinctrl = NULL;
	} else {
		ci->pins_active = pinctrl_lookup_state(ci->pinctrl, "active");
		ci->pins_idle = pinctrl_lookup_state(ci->pinctrl, "idle");
	}

	platform_set_drvdata(pdev, ci);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	/* First stage: show that CONN_INFRA answers, then leave it off. */
	ret = mtk_conninfra_power_on(ci);
	if (ret)
		return ret;
	mtk_conninfra_power_off(ci);

	return 0;
}

static const struct mtk_conninfra_data mt6878_6637_data = {
	.ip_version = 0x02050401,
	.adie = MTK_ADIE_MT6637,
	.adie_name = "MT6637",
};

static const struct mtk_conninfra_data mt6878_6631_data = {
	.ip_version = 0x02050401,
	.adie = MTK_ADIE_MT6631,
	.adie_name = "MT6631",
};

static const struct mtk_conninfra_data mt6878_6631_6686_data = {
	.ip_version = 0x02050401,
	.adie = MTK_ADIE_MT6631_MT6686,
	.adie_name = "MT6631+MT6686",
};

/*
 * The stock LK rewrites the compatible of /soc/consys (or /consys) with
 * the A-die it detects through the hwid-node GPIOs; the three variants
 * below are the only values it writes on MT6878.
 */
static const struct of_device_id mtk_conninfra_of_match[] = {
	{ .compatible = "mediatek,mt6878-6637-consys", .data = &mt6878_6637_data },
	{ .compatible = "mediatek,mt6878-6631-consys", .data = &mt6878_6631_data },
	{ .compatible = "mediatek,mt6878-6631-6686-consys", .data = &mt6878_6631_6686_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_conninfra_of_match);

static struct platform_driver mtk_conninfra_driver = {
	.probe = mtk_conninfra_probe,
	.driver = {
		.name = "mtk-conninfra",
		.of_match_table = mtk_conninfra_of_match,
	},
};
module_platform_driver(mtk_conninfra_driver);

MODULE_DESCRIPTION("MediaTek CONNINFRA power driver");
MODULE_LICENSE("GPL");
