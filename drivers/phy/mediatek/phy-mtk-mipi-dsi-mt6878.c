// SPDX-License-Identifier: GPL-2.0
/*
 * MIPI D-PHY transmitter of MT6878 (Dimensity 7300).
 *
 * This PHY generation has a register layout different from MT8183: the
 * lane and bandgap control moved to 0x04, the voltage selection to 0x08,
 * and the PLL is a sigma-delta PLL running at half the lane bit rate, with
 * the post divider as the only divider in use.
 *
 * Register offsets, values and the power up sequence follow the MediaTek
 * MediaTek downstream driver for this PHY (mtk_mipi_tx.c, "N4" generation),
 * Copyright (c) 2021 MediaTek Inc.
 *
 * The boot loader of MT6878 phones brings the display up and leaves the
 * PLL running at the rate of the panel. By default the PLL is taken over
 * as it is: it is neither reprogrammed nor switched off, and the rate it
 * runs at is reported to the DSI controller. The lane routing and the
 * output drive strength are never changed by this driver.
 *
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 */

#include <linux/math64.h>
#include <linux/module.h>

#include "phy-mtk-io.h"
#include "phy-mtk-mipi-dsi.h"

#define MIPITX_LANE_CON		0x0004
#define MIPITX_VOLTAGE_SEL	0x0008
#define RG_DSI_PRD_REF_SEL		GENMASK(5, 0)
#define MIPITX_PRESERVED	0x000c
#define MIPITX_PLL_PWR		0x0028
#define AD_DSI_PLL_SDM_PWR_ON		BIT(0)
#define AD_DSI_PLL_SDM_ISO_EN		BIT(1)
#define MIPITX_PLL_CON0		0x002c
#define MIPITX_PLL_CON1		0x0030
#define RG_DSI_PLL_EN			BIT(0)
#define RG_DSI_PLL_POSDIV		GENMASK(10, 8)
#define RG_DSI_PLL_FBSEL		BIT(13)
#define RG_DSI_PLL_DIV3_EN		BIT(28)
#define MIPITX_PLL_CON4		0x003c
#define MIPITX_SW_CTRL_CON4	0x0050
#define MIPITX_SW_ANA_CK_EN		BIT(0)

/* LANE_CON values of the vendor power up and down sequences */
#define LANE_CON_BG_CORE_ON	0x3fff0180
#define LANE_CON_BG_LPF_ON	0x3fff00c0
#define LANE_CON_BG_OFF		0x3fff0100
/* PRESERVED and PLL_CON4 values of the MediaTek downstream driver below 2.5 Gbps */
#define PRESERVED_BELOW_2G5	0xffff00f0
#define PLL_CON4_DEFAULT	0x00ff12e0

#define MT6878_RATE_MIN		125000000UL
#define MT6878_RATE_MAX		2500000000UL

/*
 * Off by default: the boot loader leaves the PLL running for the panel.
 * When set, the PLL is programmed for the rate the DSI controller asks for
 * whenever it differs from the running one, or the PLL is off.
 */
static bool mt6878_pll_reprogram;
module_param(mt6878_pll_reprogram, bool, 0444);
MODULE_PARM_DESC(mt6878_pll_reprogram,
		 "MT6878: program the DSI PLL instead of keeping the one set up by the boot loader");

static bool mtk_mipi_tx_pll_running(struct mtk_mipi_tx *mipi_tx)
{
	u32 pwr = readl(mipi_tx->regs + MIPITX_PLL_PWR);
	u32 con1 = readl(mipi_tx->regs + MIPITX_PLL_CON1);

	return (con1 & RG_DSI_PLL_EN) && (pwr & AD_DSI_PLL_SDM_PWR_ON) &&
	       !(pwr & AD_DSI_PLL_SDM_ISO_EN);
}

/*
 * Lane bit rate of the running PLL: the PLL output is twice the 26 MHz
 * reference times PCW (an 8.24 fixed point number), divided by 2^POSDIV.
 */
static unsigned long mtk_mipi_tx_pll_hw_rate(struct mtk_mipi_tx *mipi_tx)
{
	u32 pcw = readl(mipi_tx->regs + MIPITX_PLL_CON0);
	u32 con1 = readl(mipi_tx->regs + MIPITX_PLL_CON1);
	u64 rate;

	rate = mul_u32_u32(2 * 26000000, pcw) >> 24;
	rate >>= FIELD_GET(RG_DSI_PLL_POSDIV, con1);
	if (con1 & RG_DSI_PLL_DIV3_EN)
		rate = div_u64(rate, 3);
	if (con1 & RG_DSI_PLL_FBSEL)
		rate *= 2;

	return rate;
}

static int mtk_mipi_tx_pll_program(struct mtk_mipi_tx *mipi_tx)
{
	void __iomem *base = mipi_tx->regs;
	unsigned long rate = mipi_tx->data_rate;
	unsigned int txdiv, txdiv0;
	u64 pcw;

	if (rate >= 2000000000) {
		txdiv = 1;
		txdiv0 = 0;
	} else if (rate >= 1000000000) {
		txdiv = 2;
		txdiv0 = 1;
	} else if (rate >= 500000000) {
		txdiv = 4;
		txdiv0 = 2;
	} else if (rate >= 250000000) {
		txdiv = 8;
		txdiv0 = 3;
	} else if (rate >= MT6878_RATE_MIN) {
		txdiv = 16;
		txdiv0 = 4;
	} else {
		return -EINVAL;
	}

	dev_info(mipi_tx->dev, "programming the PLL for %lu bps\n", rate);

	/*
	 * Only the values the MediaTek downstream driver uses below 2.5 Gbps are known;
	 * the HS output voltage (HSTX_LDO_REF_SEL) is left as it is.
	 */
	mtk_phy_update_field(base + MIPITX_VOLTAGE_SEL, RG_DSI_PRD_REF_SEL, 0);
	writel(PRESERVED_BELOW_2G5, base + MIPITX_PRESERVED);
	writel(PLL_CON4_DEFAULT, base + MIPITX_PLL_CON4);

	/* bandgap core first, then its low pass filter */
	writel(LANE_CON_BG_CORE_ON, base + MIPITX_LANE_CON);
	usleep_range(500, 600);
	writel(LANE_CON_BG_LPF_ON, base + MIPITX_LANE_CON);

	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);
	usleep_range(30, 100);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);

	/* the PLL runs at half the bit rate, divided down by txdiv */
	pcw = div_u64((u64)(rate / 2) * txdiv << 24, 26000000);
	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_FBSEL);
	writel(pcw, base + MIPITX_PLL_CON0);
	mtk_phy_update_field(base + MIPITX_PLL_CON1, RG_DSI_PLL_POSDIV, txdiv0);
	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_DIV3_EN);
	mtk_phy_set_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);
	usleep_range(50, 100);

	return 0;
}

static void mtk_mipi_tx_pll_power_down(struct mtk_mipi_tx *mipi_tx)
{
	void __iomem *base = mipi_tx->regs;

	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);
	mtk_phy_clear_bits(base + MIPITX_SW_CTRL_CON4, MIPITX_SW_ANA_CK_EN);
	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);

	writel(LANE_CON_BG_CORE_ON, base + MIPITX_LANE_CON);
	writel(LANE_CON_BG_OFF, base + MIPITX_LANE_CON);
}

static int mtk_mipi_tx_pll_prepare(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	unsigned long hw_rate;

	if (mtk_mipi_tx_pll_running(mipi_tx)) {
		hw_rate = mtk_mipi_tx_pll_hw_rate(mipi_tx);

		if (!mt6878_pll_reprogram || hw_rate == mipi_tx->data_rate) {
			if (!mipi_tx->pll_from_boot)
				dev_info(mipi_tx->dev,
					 "keeping the PLL running at %lu bps (%u bps requested)\n",
					 hw_rate, mipi_tx->data_rate);
			mipi_tx->pll_from_boot = true;
			return 0;
		}

		mtk_mipi_tx_pll_power_down(mipi_tx);
	} else if (!mt6878_pll_reprogram) {
		dev_err(mipi_tx->dev,
			"PLL is off and reprogramming is disabled (mt6878_pll_reprogram)\n");
		return -ENODEV;
	}

	mipi_tx->pll_from_boot = false;

	return mtk_mipi_tx_pll_program(mipi_tx);
}

/* A PLL taken over from the boot loader keeps running for the next user */
static void mtk_mipi_tx_pll_unprepare(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);

	if (mipi_tx->pll_from_boot)
		return;

	mtk_mipi_tx_pll_power_down(mipi_tx);
}

static int mtk_mipi_tx_pll_determine_rate(struct clk_hw *hw,
					  struct clk_rate_request *req)
{
	req->rate = clamp_val(req->rate, MT6878_RATE_MIN, MT6878_RATE_MAX);

	return 0;
}

/*
 * The rate of a running PLL is read back from the hardware, so that the
 * DSI controller sets up its timings for the rate the lanes really run at.
 */
static unsigned long mtk_mipi_tx_pll_recalc_rate_mt6878(struct clk_hw *hw,
							unsigned long parent_rate)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);

	if (mtk_mipi_tx_pll_running(mipi_tx))
		return mtk_mipi_tx_pll_hw_rate(mipi_tx);

	return mipi_tx->data_rate;
}

static const struct clk_ops mtk_mipi_tx_pll_ops = {
	.prepare = mtk_mipi_tx_pll_prepare,
	.unprepare = mtk_mipi_tx_pll_unprepare,
	.determine_rate = mtk_mipi_tx_pll_determine_rate,
	.set_rate = mtk_mipi_tx_pll_set_rate,
	.recalc_rate = mtk_mipi_tx_pll_recalc_rate_mt6878,
};

/*
 * The lanes are under the control of the DSI controller, with the routing
 * and drive strength the boot loader set up: nothing to switch here.
 */
static void mtk_mipi_tx_power_on_signal(struct phy *phy)
{
}

static void mtk_mipi_tx_power_off_signal(struct phy *phy)
{
}

const struct mtk_mipitx_data mt6878_mipitx_data = {
	.mipi_tx_clk_ops = &mtk_mipi_tx_pll_ops,
	.mipi_tx_enable_signal = mtk_mipi_tx_power_on_signal,
	.mipi_tx_disable_signal = mtk_mipi_tx_power_off_signal,
};
