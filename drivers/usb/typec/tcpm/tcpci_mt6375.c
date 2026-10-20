// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6375 USB Type-C Port Controller Interface
 *
 * The port controller of the MT6375 SubPMIC is a TCPCI device behind the
 * MFD regmap (bank 0, I2C address 0x4e). Its vendor registers and the
 * initialisation values follow the GPL MediaTek BSP driver
 * (drivers/misc/mediatek/typec/tcpc/tcpc_mt6375.c, Copyright (c) 2020
 * MediaTek Inc.); the driver structure follows tcpci_mt6370
 * (Copyright (C) 2022 Richtek Technology Corp., ChiYuan Huang
 * <cy_huang@richtek.com>).
 *
 * The VBUS supply of the port is the charger of the same chip; this
 * driver never sources VBUS or VCONN: such requests are refused, and the
 * standard TCPCI source/sink VBUS commands, which the BSP never issues
 * either, are not sent. Water, foreign object and cable type detection of
 * the BSP are not supported; foreign object detection is switched off as
 * the BSP does when it is not used.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <linux/regmap.h>
#include <linux/usb/tcpci.h>
#include <linux/usb/tcpm.h>

#define MT6375_REG_PHYCTRL1	0x80
#define MT6375_REG_PHYCTRL2	0x81
#define MT6375_REG_PHYCTRL3	0x82
#define MT6375_REG_PHYCTRL7	0x86
#define MT6375_REG_VCONCTRL3	0x8c
#define MT6375_REG_SYSCTRL1	0x8f
#define MT6375_REG_SYSCTRL2	0x90
#define MT6375_REG_MTMASK1	0x91
#define MT6375_REG_MTINT1	0x98
#define MT6375_REG_MTST1	0x9f
#define MT6375_REG_PHYCTRL9	0xac
#define MT6375_REG_SYSCTRL3	0xb0
#define MT6375_REG_TCPCCTRL1	0xb1
#define MT6375_REG_TCPCCTRL2	0xb2
#define MT6375_REG_TCPCCTRL3	0xb3
#define MT6375_REG_LPWRCTRL3	0xbb
#define MT6375_REG_WATCHDOGCTRL	0xbe
#define MT6375_REG_HILOCTRL9	0xc8
#define MT6375_REG_SHIELDCTRL1	0xca
#define MT6375_REG_FODCTRL	0xcf

#define MT6375_NUM_VEND_INT	7

/* SYSCTRL2 */
#define MT6375_BMCIOOSC_EN	BIT(0)
#define MT6375_VBUSDET_EN	BIT(1)
/* SYSCTRL3 */
#define MT6375_SWRESET		BIT(0)
/* WATCHDOGCTRL */
#define MT6375_VBUSDISCHG_OPT	BIT(6)
/* SHIELDCTRL1 */
#define MT6375_OPEN40MS_EN	BIT(4)
/* FODCTRL */
#define MT6375_FOD_SNK_EN	BIT(6)
/* MTST1 */
#define MT6375_ST_VBUS80	BIT(1)
#define MT6375_ST_VBUSVALID	BIT(5)

#define MT6375_VENDOR_ID	0x29cf
#define MT6375_PRODUCT_ID	0x6375

/* DRP duty (dcSRC + 1) / 1024: 30 % */
#define MT6375_RP_DUTY		308

struct mt6375_tcpc {
	struct device *dev;
	struct tcpci *tcpci;
	struct tcpci_data tdata;
};

/* Write 1 to clear, the implemented bits of MTINT1..7 */
static const u8 mt6375_vend_int_all[MT6375_NUM_VEND_INT] = {
	0x3f, 0xfd, 0xf0, 0xe3, 0xff, 0xf8, 0x3f,
};

static const u8 mt6375_vend_mask_none[MT6375_NUM_VEND_INT];

static const struct reg_sequence mt6375_tcpc_init_seq[] = {
	/* Rd on both CC lines, no DRP: a sink until the port manager says */
	REG_SEQ0(TCPC_ROLE_CTRL, (TCPC_ROLE_CTRL_CC_RD << 2) | TCPC_ROLE_CTRL_CC_RD),
	/* tTCPCFilter 250 us */
	REG_SEQ0(MT6375_REG_TCPCCTRL1, 0x0a),
	/* DRP toggle cycle 51.2 ms + 6.4 ms * val */
	REG_SEQ0(MT6375_REG_TCPCCTRL2, 0x00),
	/* DRP duty, 16 bit little endian */
	REG_SEQ0(MT6375_REG_TCPCCTRL3, MT6375_RP_DUTY & 0xff),
	REG_SEQ0(MT6375_REG_TCPCCTRL3 + 1, MT6375_RP_DUTY >> 8),
	/* transition toggle count 7, oscillator config 1, no RX glitch filter */
	REG_SEQ0(MT6375_REG_PHYCTRL1, 0x74),
	/* PHY CDR threshold */
	REG_SEQ0(MT6375_REG_PHYCTRL2, 0x3a),
	/* transition window 43.29 us */
	REG_SEQ0(MT6375_REG_PHYCTRL3, 0x82),
	/* BMC decoder idle time 17.982 us */
	REG_SEQ0(MT6375_REG_PHYCTRL7, 0x36),
	/* retry period 26.208 us */
	REG_SEQ0(MT6375_REG_PHYCTRL9, 0x3c),
	/* VCONN current limit mode, OCP 100 mA, analog OVP */
	REG_SEQ0(MT6375_REG_VCONCTRL3, 0x11),
	/* HI/LO CC filter 250 us */
	REG_SEQ0(MT6375_REG_HILOCTRL9, 0xaa),
	/* low power LDO 2 V */
	REG_SEQ0(MT6375_REG_LPWRCTRL3, 0xd8),
};

static int mt6375_tcpc_init(struct tcpci *tcpci, struct tcpci_data *tdata)
{
	struct mt6375_tcpc *priv = container_of(tdata, struct mt6375_tcpc,
						tdata);
	struct regmap *rm = tdata->regmap;
	unsigned int val, st;
	__le16 mask;
	int ret;

	ret = regmap_write(rm, MT6375_REG_SYSCTRL3, MT6375_SWRESET);
	if (ret)
		return ret;

	/* the IC needs 1..2 ms after the reset command */
	usleep_range(1000, 2000);

	ret = regmap_read_poll_timeout(rm, TCPC_POWER_STATUS, val,
				       !(val & TCPC_POWER_STATUS_UNINIT),
				       1000, 50000);
	if (ret)
		return ret;

	/* No vendor events: water, FOD, OTP and the like are not handled */
	ret = regmap_raw_write(rm, MT6375_REG_MTMASK1, mt6375_vend_mask_none,
			       MT6375_NUM_VEND_INT);
	if (ret)
		return ret;

	ret = regmap_raw_write(rm, MT6375_REG_MTINT1, mt6375_vend_int_all,
			       MT6375_NUM_VEND_INT);
	if (ret)
		return ret;

	ret = regmap_multi_reg_write(rm, mt6375_tcpc_init_seq,
				     ARRAY_SIZE(mt6375_tcpc_init_seq));
	if (ret)
		return ret;

	/* CC open for 40 ms when the PMIC sees system undervoltage */
	ret = regmap_set_bits(rm, MT6375_REG_SHIELDCTRL1, MT6375_OPEN40MS_EN);
	if (ret)
		return ret;

	/* CC status alert when Looking4Connection changes */
	ret = regmap_set_bits(rm, TCPC_TCPC_CTRL, TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
	if (ret)
		return ret;

	/* auto discharge timer (BSP: about 12 mA less quiescent current) */
	ret = regmap_set_bits(rm, MT6375_REG_WATCHDOGCTRL, MT6375_VBUSDISCHG_OPT);
	if (ret)
		return ret;

	/* no bleed discharge (BSP: about 2 mA less quiescent current) */
	ret = regmap_clear_bits(rm, TCPC_POWER_CTRL,
				TCPC_POWER_CTRL_BLEED_DISCHARGE);
	if (ret)
		return ret;

	/* foreign object detection off, as the BSP does without it */
	ret = regmap_clear_bits(rm, MT6375_REG_FODCTRL, MT6375_FOD_SNK_EN);
	if (ret)
		return ret;

	/* shipping mode off, auto idle on, idle timeout 6.4 ms */
	ret = regmap_write(rm, MT6375_REG_SYSCTRL1, 0xb8);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	/* VBUS detection and the BMC oscillator on (BSP: low power mode off) */
	ret = regmap_write(rm, MT6375_REG_SYSCTRL2,
			   MT6375_VBUSDET_EN | MT6375_BMCIOOSC_EN);
	if (ret)
		return ret;

	/* The alerts the TCPCI core handles; the reset default is all */
	mask = cpu_to_le16(TCPC_ALERT_TX_SUCCESS | TCPC_ALERT_TX_FAILED |
			   TCPC_ALERT_TX_DISCARDED | TCPC_ALERT_RX_STATUS |
			   TCPC_ALERT_RX_HARD_RST | TCPC_ALERT_CC_STATUS |
			   TCPC_ALERT_POWER_STATUS);
	ret = regmap_raw_write(rm, TCPC_ALERT_MASK, &mask, sizeof(mask));
	if (ret)
		return ret;

	/*
	 * The BSP takes VBUS presence from the vendor status, the TCPCI
	 * core from POWER_STATUS; print both for comparison.
	 */
	if (!regmap_read(rm, TCPC_POWER_STATUS, &val) &&
	    !regmap_read(rm, MT6375_REG_MTST1, &st))
		dev_info(priv->dev,
			 "init: power status 0x%02x (vbus %d), vendor 0x%02x (vbus valid %d, vsafe0v %d)\n",
			 val, !!(val & TCPC_POWER_STATUS_VBUS_PRES), st,
			 !!(st & MT6375_ST_VBUSVALID), !!(st & MT6375_ST_VBUS80));

	return 0;
}

static int mt6375_tcpc_set_vconn(struct tcpci *tcpci, struct tcpci_data *tdata,
				 bool enable)
{
	struct mt6375_tcpc *priv = container_of(tdata, struct mt6375_tcpc,
						tdata);

	if (enable) {
		dev_warn(priv->dev, "refusing to source VCONN\n");
		return -EOPNOTSUPP;
	}

	return 0;
}

static int mt6375_tcpc_set_vbus(struct tcpci *tcpci, struct tcpci_data *tdata,
				bool source, bool sink)
{
	struct mt6375_tcpc *priv = container_of(tdata, struct mt6375_tcpc,
						tdata);

	if (source) {
		dev_warn(priv->dev, "refusing to source VBUS\n");
		return -EOPNOTSUPP;
	}

	/* The charger sinks on its own: nothing to switch, no TCPC command */
	return 1;
}

static irqreturn_t mt6375_tcpc_irq(int irq, void *data)
{
	struct mt6375_tcpc *priv = data;

	return tcpci_irq(priv->tcpci);
}

static int mt6375_tcpc_check_id(struct mt6375_tcpc *priv)
{
	struct regmap *rm = priv->tdata.regmap;
	__le16 vid, pid, did;
	int ret;

	ret = regmap_raw_read(rm, TCPC_VENDOR_ID, &vid, sizeof(vid));
	if (!ret)
		ret = regmap_raw_read(rm, TCPC_PRODUCT_ID, &pid, sizeof(pid));
	if (!ret)
		ret = regmap_raw_read(rm, TCPC_BCD_DEV, &did, sizeof(did));
	if (ret)
		return dev_err_probe(priv->dev, ret, "failed to read IDs\n");

	if (le16_to_cpu(vid) != MT6375_VENDOR_ID ||
	    le16_to_cpu(pid) != MT6375_PRODUCT_ID)
		return dev_err_probe(priv->dev, -ENODEV,
				     "unexpected vendor/product 0x%04x/0x%04x\n",
				     le16_to_cpu(vid), le16_to_cpu(pid));

	dev_info(priv->dev, "MT6375 TCPC, device 0x%04x\n", le16_to_cpu(did));
	return 0;
}

static void mt6375_tcpc_unregister_port(void *tcpci)
{
	tcpci_unregister_port(tcpci);
}

static int mt6375_tcpc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6375_tcpc *priv;
	int irq, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->tdata.regmap = dev_get_regmap(dev->parent, NULL);
	if (!priv->tdata.regmap)
		return dev_err_probe(dev, -ENODEV, "no parent regmap\n");

	ret = mt6375_tcpc_check_id(priv);
	if (ret)
		return ret;

	irq = platform_get_irq_byname(pdev, "pd_evt");
	if (irq < 0)
		return irq;

	priv->tdata.init = mt6375_tcpc_init;
	priv->tdata.set_vconn = mt6375_tcpc_set_vconn;
	priv->tdata.set_vbus = mt6375_tcpc_set_vbus;

	priv->tcpci = tcpci_register_port(dev, &priv->tdata);
	if (IS_ERR(priv->tcpci))
		return dev_err_probe(dev, PTR_ERR(priv->tcpci),
				     "failed to register TCPCI port\n");

	ret = devm_add_action_or_reset(dev, mt6375_tcpc_unregister_port,
				       priv->tcpci);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, irq, NULL, mt6375_tcpc_irq,
					IRQF_ONESHOT, dev_name(dev), priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;

	return devm_pm_set_wake_irq(dev, irq);
}

static const struct of_device_id mt6375_tcpc_of_match[] = {
	{ .compatible = "mediatek,mt6375-tcpc" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6375_tcpc_of_match);

static struct platform_driver mt6375_tcpc_driver = {
	.driver = {
		.name = "mt6375-tcpc",
		.of_match_table = mt6375_tcpc_of_match,
	},
	.probe = mt6375_tcpc_probe,
};
module_platform_driver(mt6375_tcpc_driver);

MODULE_DESCRIPTION("MediaTek MT6375 USB Type-C Port Controller Interface driver");
MODULE_LICENSE("GPL");
