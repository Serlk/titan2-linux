// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 MediaTek Inc.
 *		      Chuan-wen Chen <chuan-wen.chen@mediatek.com>
 */

#include <dt-bindings/clock/mediatek,mt6878-clock.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-mux.h"

/* MUX SEL REG */
#define VLP_CLK_CFG_UPDATE		0x0004
#define VLP_CLK_CFG_0			0x0008
#define VLP_CLK_CFG_0_SET		0x000C
#define VLP_CLK_CFG_0_CLR		0x0010
#define VLP_CLK_CFG_1			0x0014
#define VLP_CLK_CFG_1_SET		0x0018
#define VLP_CLK_CFG_1_CLR		0x001C
#define VLP_CLK_CFG_2			0x0020
#define VLP_CLK_CFG_2_SET		0x0024
#define VLP_CLK_CFG_2_CLR		0x0028
#define VLP_CLK_CFG_3			0x002C
#define VLP_CLK_CFG_3_SET		0x0030
#define VLP_CLK_CFG_3_CLR		0x0034
#define VLP_CLK_CFG_4			0x0038
#define VLP_CLK_CFG_4_SET		0x003C
#define VLP_CLK_CFG_4_CLR		0x0040

/* MUX UPDATE SHIFT */
#define VLP_MUX_SCP_SHIFT			0
#define VLP_MUX_PWRAP_ULPOSC_SHIFT		1
#define VLP_MUX_SPMI_P_MST_SHIFT		2
#define VLP_MUX_SPMI_M_MST_SHIFT		3
#define VLP_MUX_DVFSRC_SHIFT			4
#define VLP_MUX_PWM_VLP_SHIFT			5
#define VLP_MUX_AXI_VLP_SHIFT			6
#define VLP_MUX_DBGAO_26M_SHIFT			7
#define VLP_MUX_SYSTIMER_26M_SHIFT		8
#define VLP_MUX_SSPM_SHIFT			9
#define VLP_MUX_SSPM_F26M_SHIFT			10
#define VLP_MUX_SRCK_SHIFT			11
#define VLP_MUX_SCP_SPI_SHIFT			12
#define VLP_MUX_SCP_IIC_SHIFT			13
#define VLP_MUX_SCP_SPI_HIGH_SPD_SHIFT		14
#define VLP_MUX_SCP_IIC_HIGH_SPD_SHIFT		15
#define VLP_MUX_SSPM_ULPOSC_SHIFT		16
#define VLP_MUX_APXGPT_26M_SHIFT		18

static const struct mtk_fixed_factor vlp_divs[] = {
	FACTOR(CLK_VLP_SCP, "vlp_scp_ck",
			"vlp_scp_sel", 1, 1),
	FACTOR(CLK_VLP_PWRAP_ULPOSC, "vlp_pwrap_ulposc_ck",
			"vlp_pwrap_ulposc_sel", 1, 1),
	FACTOR(CLK_VLP_SPMI_P_MST, "vlp_spmi_p_ck",
			"vlp_spmi_p_sel", 1, 1),
	FACTOR(CLK_VLP_SPMI_M_MST, "vlp_spmi_m_ck",
			"vlp_spmi_m_sel", 1, 1),
	FACTOR(CLK_VLP_DVFSRC, "vlp_dvfsrc_ck",
			"vlp_dvfsrc_sel", 1, 1),
	FACTOR(CLK_VLP_PWM_VLP, "vlp_pwm_vlp_ck",
			"vlp_pwm_vlp_sel", 1, 1),
	FACTOR(CLK_VLP_AXI_VLP, "vlp_axi_vlp_ck",
			"vlp_axi_vlp_sel", 1, 1),
	FACTOR(CLK_VLP_DBGAO_26M, "vlp_dbgao_26m_ck",
			"vlp_dbgao_26m_sel", 1, 1),
	FACTOR(CLK_VLP_SYSTIMER_26M, "vlp_systimer_26m_ck",
			"vlp_systimer_26m_sel", 1, 1),
	FACTOR(CLK_VLP_SSPM, "vlp_sspm_ck",
			"vlp_sspm_sel", 1, 1),
	FACTOR(CLK_VLP_SSPM_F26M, "vlp_sspm_f26m_ck",
			"vlp_sspm_f26m_sel", 1, 1),
	FACTOR(CLK_VLP_SRCK, "vlp_srck_ck",
			"vlp_srck_sel", 1, 1),
	FACTOR(CLK_VLP_SCP_SPI, "vlp_scp_spi_ck",
			"vlp_scp_spi_sel", 1, 1),
	FACTOR(CLK_VLP_SCP_IIC, "vlp_scp_iic_ck",
			"vlp_scp_iic_sel", 1, 1),
	FACTOR(CLK_VLP_SCP_SPI_HS, "vlp_scp_spi_hs_ck",
			"vlp_scp_spi_hs_sel", 1, 1),
	FACTOR(CLK_VLP_SCP_IIC_HS, "vlp_scp_iic_hs_ck",
			"vlp_scp_iic_hs_sel", 1, 1),
	FACTOR(CLK_VLP_SSPM_ULPOSC, "vlp_sspm_ulposc_ck",
			"vlp_sspm_ulposc_sel", 1, 1),
	FACTOR(CLK_VLP_APXGPT_26M, "vlp_apxgpt_26m_ck",
			"vlp_apxgpt_26m_sel", 1, 1),
	FACTOR(CLK_VLP_SPM, "vlp_spm_ck",
			"mainpll_d7_d4", 1, 1),
};

static const char * const vlp_scp_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d4",
	"univpll_d3",
	"mainpll_d3",
	"univpll_d6",
	"apll1_ck",
	"mainpll_d4",
	"mainpll_d7",
	"osc_d10"
};

static const char * const vlp_pwrap_ulposc_parents[] = {
	"tck_26m_mx9_ck",
	"osc_d10",
	"osc_d7",
	"osc_d8",
	"osc_d16",
	"mainpll_d7_d8"
};

static const char * const vlp_spmi_p_parents[] = {
	"tck_26m_mx9_ck",
	"f26m_d2",
	"osc_d8",
	"osc_d10",
	"osc_d16",
	"osc_d7",
	"clkrtc",
	"mainpll_d7_d8",
	"mainpll_d6_d8",
	"mainpll_d5_d8"
};

static const char * const vlp_spmi_m_parents[] = {
	"tck_26m_mx9_ck",
	"f26m_d2",
	"osc_d8",
	"osc_d10",
	"osc_d16",
	"osc_d7",
	"clkrtc",
	"mainpll_d7_d8",
	"mainpll_d6_d8",
	"mainpll_d5_d8"
};

static const char * const vlp_dvfsrc_parents[] = {
	"tck_26m_mx9_ck",
	"osc_d10"
};

static const char * const vlp_pwm_vlp_parents[] = {
	"tck_26m_mx9_ck",
	"osc_d4",
	"clkrtc",
	"osc_d10",
	"mainpll_d4_d8"
};

static const char * const vlp_axi_vlp_parents[] = {
	"tck_26m_mx9_ck",
	"osc_d10",
	"osc_d2",
	"mainpll_d7_d4",
	"mainpll_d7_d2"
};

static const char * const vlp_dbgao_26m_parents[] = {
	"tck_26m_mx9_ck",
	"osc_d10"
};

static const char * const vlp_systimer_26m_parents[] = {
	"tck_26m_mx9_ck",
	"osc_d10"
};

static const char * const vlp_sspm_parents[] = {
	"tck_26m_mx9_ck",
	"osc_d10",
	"mainpll_d5_d2",
	"osc_ck",
	"mainpll_d6"
};

static const char * const vlp_sspm_f26m_parents[] = {
	"tck_26m_mx9_ck",
	"osc_d10"
};

static const char * const vlp_srck_parents[] = {
	"tck_26m_mx9_ck",
	"osc_d10"
};

static const char * const vlp_scp_spi_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d5_d4",
	"mainpll_d7_d2",
	"osc_d10"
};

static const char * const vlp_scp_iic_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d5_d4",
	"mainpll_d7_d2",
	"osc_d10"
};

static const char * const vlp_scp_spi_hs_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d5_d4",
	"mainpll_d7_d2",
	"osc_d10"
};

static const char * const vlp_scp_iic_hs_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d5_d4",
	"mainpll_d7_d2",
	"osc_d10"
};

static const char * const vlp_sspm_ulposc_parents[] = {
	"osc_ck",
	"univpll_d5_d2"
};

static const char * const vlp_apxgpt_26m_parents[] = {
	"tck_26m_mx9_ck",
	"osc_d10"
};

static const struct mtk_mux vlp_muxes[] = {
	/* VLP_CLK_CFG_0 */
	/*
	 * The SCP runs firmware started by the bootloader whether or not Linux
	 * has a driver for it; never gate its clock.
	 */
	MUX_GATE_CLR_SET_UPD_FLAGS(CLK_VLP_SCP_SEL/* dts */, "vlp_scp_sel",
		vlp_scp_parents/* parent */, VLP_CLK_CFG_0, VLP_CLK_CFG_0_SET,
		VLP_CLK_CFG_0_CLR/* set parent */, 0/* lsb */, 4/* width */,
		7/* pdn */, VLP_CLK_CFG_UPDATE/* upd ofs */,
		VLP_MUX_SCP_SHIFT/* upd shift */,
		CLK_IS_CRITICAL | CLK_SET_RATE_PARENT),
	MUX_CLR_SET_UPD(CLK_VLP_PWRAP_ULPOSC_SEL/* dts */, "vlp_pwrap_ulposc_sel",
		vlp_pwrap_ulposc_parents/* parent */, VLP_CLK_CFG_0, VLP_CLK_CFG_0_SET,
		VLP_CLK_CFG_0_CLR/* set parent */, 8/* lsb */, 3/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_PWRAP_ULPOSC_SHIFT/* upd shift */),
	MUX_CLR_SET_UPD(CLK_VLP_SPMI_P_MST_SEL/* dts */, "vlp_spmi_p_sel",
		vlp_spmi_p_parents/* parent */, VLP_CLK_CFG_0, VLP_CLK_CFG_0_SET,
		VLP_CLK_CFG_0_CLR/* set parent */, 16/* lsb */, 4/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_SPMI_P_MST_SHIFT/* upd shift */),
	MUX_CLR_SET_UPD(CLK_VLP_SPMI_M_MST_SEL/* dts */, "vlp_spmi_m_sel",
		vlp_spmi_m_parents/* parent */, VLP_CLK_CFG_0, VLP_CLK_CFG_0_SET,
		VLP_CLK_CFG_0_CLR/* set parent */, 24/* lsb */, 4/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_SPMI_M_MST_SHIFT/* upd shift */),
	/* VLP_CLK_CFG_1 */
	MUX_CLR_SET_UPD(CLK_VLP_DVFSRC_SEL/* dts */, "vlp_dvfsrc_sel",
		vlp_dvfsrc_parents/* parent */, VLP_CLK_CFG_1, VLP_CLK_CFG_1_SET,
		VLP_CLK_CFG_1_CLR/* set parent */, 0/* lsb */, 1/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_DVFSRC_SHIFT/* upd shift */),
	MUX_CLR_SET_UPD(CLK_VLP_PWM_VLP_SEL/* dts */, "vlp_pwm_vlp_sel",
		vlp_pwm_vlp_parents/* parent */, VLP_CLK_CFG_1, VLP_CLK_CFG_1_SET,
		VLP_CLK_CFG_1_CLR/* set parent */, 8/* lsb */, 3/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_PWM_VLP_SHIFT/* upd shift */),
	MUX_CLR_SET_UPD(CLK_VLP_AXI_VLP_SEL/* dts */, "vlp_axi_vlp_sel",
		vlp_axi_vlp_parents/* parent */, VLP_CLK_CFG_1, VLP_CLK_CFG_1_SET,
		VLP_CLK_CFG_1_CLR/* set parent */, 16/* lsb */, 3/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_AXI_VLP_SHIFT/* upd shift */),
	MUX_CLR_SET_UPD(CLK_VLP_DBGAO_26M_SEL/* dts */, "vlp_dbgao_26m_sel",
		vlp_dbgao_26m_parents/* parent */, VLP_CLK_CFG_1, VLP_CLK_CFG_1_SET,
		VLP_CLK_CFG_1_CLR/* set parent */, 24/* lsb */, 1/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_DBGAO_26M_SHIFT/* upd shift */),
	/* VLP_CLK_CFG_2 */
	MUX_CLR_SET_UPD(CLK_VLP_SYSTIMER_26M_SEL/* dts */, "vlp_systimer_26m_sel",
		vlp_systimer_26m_parents/* parent */, VLP_CLK_CFG_2, VLP_CLK_CFG_2_SET,
		VLP_CLK_CFG_2_CLR/* set parent */, 0/* lsb */, 1/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_SYSTIMER_26M_SHIFT/* upd shift */),
	MUX_CLR_SET_UPD(CLK_VLP_SSPM_SEL/* dts */, "vlp_sspm_sel",
		vlp_sspm_parents/* parent */, VLP_CLK_CFG_2, VLP_CLK_CFG_2_SET,
		VLP_CLK_CFG_2_CLR/* set parent */, 8/* lsb */, 3/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_SSPM_SHIFT/* upd shift */),
	MUX_CLR_SET_UPD(CLK_VLP_SSPM_F26M_SEL/* dts */, "vlp_sspm_f26m_sel",
		vlp_sspm_f26m_parents/* parent */, VLP_CLK_CFG_2, VLP_CLK_CFG_2_SET,
		VLP_CLK_CFG_2_CLR/* set parent */, 16/* lsb */, 1/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_SSPM_F26M_SHIFT/* upd shift */),
	MUX_CLR_SET_UPD(CLK_VLP_SRCK_SEL/* dts */, "vlp_srck_sel",
		vlp_srck_parents/* parent */, VLP_CLK_CFG_2, VLP_CLK_CFG_2_SET,
		VLP_CLK_CFG_2_CLR/* set parent */, 24/* lsb */, 1/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_SRCK_SHIFT/* upd shift */),
	/* VLP_CLK_CFG_3 */
	MUX_CLR_SET_UPD(CLK_VLP_SCP_SPI_SEL/* dts */, "vlp_scp_spi_sel",
		vlp_scp_spi_parents/* parent */, VLP_CLK_CFG_3, VLP_CLK_CFG_3_SET,
		VLP_CLK_CFG_3_CLR/* set parent */, 0/* lsb */, 2/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_SCP_SPI_SHIFT/* upd shift */),
	MUX_CLR_SET_UPD(CLK_VLP_SCP_IIC_SEL/* dts */, "vlp_scp_iic_sel",
		vlp_scp_iic_parents/* parent */, VLP_CLK_CFG_3, VLP_CLK_CFG_3_SET,
		VLP_CLK_CFG_3_CLR/* set parent */, 8/* lsb */, 2/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_SCP_IIC_SHIFT/* upd shift */),
	MUX_CLR_SET_UPD(CLK_VLP_SCP_SPI_HS_SEL/* dts */, "vlp_scp_spi_hs_sel",
		vlp_scp_spi_hs_parents/* parent */, VLP_CLK_CFG_3, VLP_CLK_CFG_3_SET,
		VLP_CLK_CFG_3_CLR/* set parent */, 16/* lsb */, 2/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_SCP_SPI_HIGH_SPD_SHIFT/* upd shift */),
	MUX_CLR_SET_UPD(CLK_VLP_SCP_IIC_HS_SEL/* dts */, "vlp_scp_iic_hs_sel",
		vlp_scp_iic_hs_parents/* parent */, VLP_CLK_CFG_3, VLP_CLK_CFG_3_SET,
		VLP_CLK_CFG_3_CLR/* set parent */, 24/* lsb */, 2/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_SCP_IIC_HIGH_SPD_SHIFT/* upd shift */),
	/* VLP_CLK_CFG_4 */
	MUX_CLR_SET_UPD(CLK_VLP_SSPM_ULPOSC_SEL/* dts */, "vlp_sspm_ulposc_sel",
		vlp_sspm_ulposc_parents/* parent */, VLP_CLK_CFG_4, VLP_CLK_CFG_4_SET,
		VLP_CLK_CFG_4_CLR/* set parent */, 0/* lsb */, 1/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_SSPM_ULPOSC_SHIFT/* upd shift */),
	MUX_CLR_SET_UPD(CLK_VLP_APXGPT_26M_SEL/* dts */, "vlp_apxgpt_26m_sel",
		vlp_apxgpt_26m_parents/* parent */, VLP_CLK_CFG_4, VLP_CLK_CFG_4_SET,
		VLP_CLK_CFG_4_CLR/* set parent */, 16/* lsb */, 1/* width */,
		VLP_CLK_CFG_UPDATE/* upd ofs */, VLP_MUX_APXGPT_26M_SHIFT/* upd shift */),
};

static const struct mtk_clk_desc vlpck_desc = {
	.factor_clks = vlp_divs,
	.num_factor_clks = ARRAY_SIZE(vlp_divs),
	.mux_clks = vlp_muxes,
	.num_mux_clks = ARRAY_SIZE(vlp_muxes),
};

static const struct of_device_id of_match_clk_mt6878_vlpck[] = {
	{ .compatible = "mediatek,mt6878-vlpckgen", .data = &vlpck_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6878_vlpck);

static struct platform_driver clk_mt6878_vlpck_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6878-vlpck",
		.of_match_table = of_match_clk_mt6878_vlpck,
	},
};
module_platform_driver(clk_mt6878_vlpck_drv);

MODULE_DESCRIPTION("MediaTek MT6878 VLP clock generator driver");
MODULE_LICENSE("GPL");
