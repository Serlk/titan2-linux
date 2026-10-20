/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 MediaTek Inc.
 * Author: Chuan-wen Chen <chuan-wen.chen@mediatek.com>
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 */

#ifndef __SOC_MEDIATEK_MT6878_PM_DOMAINS_H
#define __SOC_MEDIATEK_MT6878_PM_DOMAINS_H

#include "mtk-pm-domains.h"
#include <dt-bindings/power/mediatek,mt6878-power.h>

/*
 * MT6878 power domain support
 *
 * Register values from the MediaTek downstream kernel (mtk-scpsys-mt6878.c).
 */

/* INFRACFG_AO bus protection, one SET/CLR/STA group per bus */
#define MT6878_TOP_AXI_PROT_EN_MM_SET			0x0c14
#define MT6878_TOP_AXI_PROT_EN_MM_CLR			0x0c18
#define MT6878_TOP_AXI_PROT_EN_MM_STA			0x0c1c
#define  MT6878_TOP_AXI_PROT_EN_MM_DISP			(BIT(0) | BIT(1) | BIT(18))
#define  MT6878_TOP_AXI_PROT_EN_MM_ISP_MAIN		BIT(2)
#define  MT6878_TOP_AXI_PROT_EN_MM_ISP_MAIN_2ND		BIT(3)
#define  MT6878_TOP_AXI_PROT_EN_MM_ISP_MAIN_3RD		BIT(4)
#define  MT6878_TOP_AXI_PROT_EN_MM_ISP_MAIN_4TH		BIT(5)
#define  MT6878_TOP_AXI_PROT_EN_MM_ISP_DIP1		BIT(6)
#define  MT6878_TOP_AXI_PROT_EN_MM_ISP_DIP1_2ND		BIT(7)
#define  MT6878_TOP_AXI_PROT_EN_MM_ISP_VCORE		BIT(8)
#define  MT6878_TOP_AXI_PROT_EN_MM_VEN0			BIT(12)
#define  MT6878_TOP_AXI_PROT_EN_MM_VDE0			BIT(20)
#define  MT6878_TOP_AXI_PROT_EN_MM_CAM_SUBA		BIT(25)
#define  MT6878_TOP_AXI_PROT_EN_MM_CAM_SUBA_2ND		BIT(26)
#define  MT6878_TOP_AXI_PROT_EN_MM_CAM_SUBB		BIT(27)
#define  MT6878_TOP_AXI_PROT_EN_MM_CAM_SUBB_2ND		BIT(28)
#define  MT6878_TOP_AXI_PROT_EN_MM_CAM_MAIN		BIT(29)
#define  MT6878_TOP_AXI_PROT_EN_MM_CAM_MAIN_2ND		BIT(30)
#define  MT6878_TOP_AXI_PROT_EN_MM_CAM_VCORE		BIT(31)

#define MT6878_TOP_AXI_PROT_EN_MM_1_SET			0x0c24
#define MT6878_TOP_AXI_PROT_EN_MM_1_CLR			0x0c28
#define MT6878_TOP_AXI_PROT_EN_MM_1_STA			0x0c2c
#define  MT6878_TOP_AXI_PROT_EN_MM_1_ISP_VCORE		(BIT(7) | BIT(8))
#define  MT6878_TOP_AXI_PROT_EN_MM_1_CAM_VCORE		(BIT(9) | BIT(10))
#define  MT6878_TOP_AXI_PROT_EN_MM_1_VEN0		BIT(12)
#define  MT6878_TOP_AXI_PROT_EN_MM_1_VDE0		BIT(13)

#define MT6878_TOP_AXI_PROT_EN_INFRA_SET		0x0c44
#define MT6878_TOP_AXI_PROT_EN_INFRA_CLR		0x0c48
#define MT6878_TOP_AXI_PROT_EN_INFRA_STA		0x0c4c
#define  MT6878_TOP_AXI_PROT_EN_INFRA_CONN		BIT(8)
#define  MT6878_TOP_AXI_PROT_EN_INFRA_MFG0		BIT(9)
#define  MT6878_TOP_AXI_PROT_EN_INFRA_MD		BIT(11)
#define  MT6878_TOP_AXI_PROT_EN_INFRA_CAM_CCU		BIT(13)

#define MT6878_TOP_AXI_PROT_EN_INFRA_1_SET		0x0c54
#define MT6878_TOP_AXI_PROT_EN_INFRA_1_CLR		0x0c58
#define MT6878_TOP_AXI_PROT_EN_INFRA_1_STA		0x0c5c
#define  MT6878_TOP_AXI_PROT_EN_INFRA_1_MD		BIT(9)
#define  MT6878_TOP_AXI_PROT_EN_INFRA_1_CONN		BIT(12)

#define MT6878_TOP_AXI_PROT_EN_PERI_SET			0x0c84
#define MT6878_TOP_AXI_PROT_EN_PERI_CLR			0x0c88
#define MT6878_TOP_AXI_PROT_EN_PERI_STA			0x0c8c
#define  MT6878_TOP_AXI_PROT_EN_PERI_AUDIO		BIT(6)
#define  MT6878_TOP_AXI_PROT_EN_PERI_SSUSB		BIT(7)

#define MT6878_TOP_AXI_PROT_EN_MCU_SET			0x0c94
#define MT6878_TOP_AXI_PROT_EN_MCU_CLR			0x0c98
#define MT6878_TOP_AXI_PROT_EN_MCU_STA			0x0c9c
#define  MT6878_TOP_AXI_PROT_EN_MCU_CONN_2ND		BIT(0)
#define  MT6878_TOP_AXI_PROT_EN_MCU_CONN		BIT(1)

#define MT6878_TOP_AXI_PROT_EN_MD_SET			0x0ca4
#define MT6878_TOP_AXI_PROT_EN_MD_CLR			0x0ca8
#define MT6878_TOP_AXI_PROT_EN_MD_STA			0x0cac
#define  MT6878_TOP_AXI_PROT_EN_MD_MFG0			BIT(4)

#define MT6878_TOP_AXI_PROT_EN_DRAMC_SET		0x0cc4
#define MT6878_TOP_AXI_PROT_EN_DRAMC_CLR		0x0cc8
#define MT6878_TOP_AXI_PROT_EN_DRAMC_STA		0x0ccc
#define  MT6878_TOP_AXI_PROT_EN_DRAMC_CAM_CCU		(BIT(8) | BIT(10))
#define  MT6878_TOP_AXI_PROT_EN_DRAMC_CAM_CCU_2ND	(BIT(9) | BIT(11) | BIT(12))

/* VLPCFG bus protection */
#define MT6878_VLP_AXI_PROT_EN_SET			0x0214
#define MT6878_VLP_AXI_PROT_EN_CLR			0x0218
#define MT6878_VLP_AXI_PROT_EN_STA			0x0220
#define  MT6878_VLP_AXI_PROT_EN_MM_PROC			BIT(8)
#define  MT6878_VLP_AXI_PROT_EN_MM_PROC_2ND		(BIT(9) | BIT(10))

/* NEMICFG_AO bus protection */
#define MT6878_NEMICFG_AO_MEM_PROT_EN_SET		0x0084
#define MT6878_NEMICFG_AO_MEM_PROT_EN_CLR		0x0088
#define MT6878_NEMICFG_AO_MEM_PROT_EN_STA		0x008c
#define  MT6878_NEMICFG_AO_MEM_PROT_EN_GLITCH_MD	(BIT(6) | BIT(7))

/* Hardware Voter MTCMOS vote registers */
#define MT6878_HWV_MTCMOS_SET				0x0198
#define MT6878_HWV_MTCMOS_CLR				0x019c
#define MT6878_HWV_MTCMOS_EN				0x1410
#define MT6878_HWV_MTCMOS_DONE				0x141c
#define MT6878_HWV_MTCMOS_SET_STA			0x146c
#define MT6878_HWV_MTCMOS_CLR_STA			0x1470

/* All direct control domains report their state in their own PWR_CON */
#define MT6878_PWR_ACK					BIT(30)
#define MT6878_PWR_ACK_2ND				BIT(31)

#define MT6878_SRAM_PDN					BIT(8)
#define MT6878_SRAM_PDN_ACK				BIT(12)
#define MT6878_SRAM_SLP					BIT(9)
#define MT6878_SRAM_SLP_ACK				BIT(13)

#define MT6878_PWR_CON_STA(_ctl)		\
	.ctl_offs = (_ctl),			\
	.sta_mask = MT6878_PWR_ACK,		\
	.sta2nd_mask = MT6878_PWR_ACK_2ND,	\
	.pwr_sta_offs = (_ctl),			\
	.pwr_sta2nd_offs = (_ctl)

#define MT6878_BP_MM(_mask)						\
	BUS_PROT_WR_IGN(INFRA, _mask,					\
			MT6878_TOP_AXI_PROT_EN_MM_SET,			\
			MT6878_TOP_AXI_PROT_EN_MM_CLR,			\
			MT6878_TOP_AXI_PROT_EN_MM_STA)

#define MT6878_BP_MM_1(_mask)						\
	BUS_PROT_WR_IGN(INFRA, _mask,					\
			MT6878_TOP_AXI_PROT_EN_MM_1_SET,		\
			MT6878_TOP_AXI_PROT_EN_MM_1_CLR,		\
			MT6878_TOP_AXI_PROT_EN_MM_1_STA)

#define MT6878_BP_INFRA(_mask)						\
	BUS_PROT_WR_IGN(INFRA, _mask,					\
			MT6878_TOP_AXI_PROT_EN_INFRA_SET,		\
			MT6878_TOP_AXI_PROT_EN_INFRA_CLR,		\
			MT6878_TOP_AXI_PROT_EN_INFRA_STA)

#define MT6878_BP_INFRA_1(_mask)					\
	BUS_PROT_WR_IGN(INFRA, _mask,					\
			MT6878_TOP_AXI_PROT_EN_INFRA_1_SET,		\
			MT6878_TOP_AXI_PROT_EN_INFRA_1_CLR,		\
			MT6878_TOP_AXI_PROT_EN_INFRA_1_STA)

#define MT6878_BP_PERI(_mask)						\
	BUS_PROT_WR_IGN(INFRA, _mask,					\
			MT6878_TOP_AXI_PROT_EN_PERI_SET,		\
			MT6878_TOP_AXI_PROT_EN_PERI_CLR,		\
			MT6878_TOP_AXI_PROT_EN_PERI_STA)

#define MT6878_BP_MCU(_mask)						\
	BUS_PROT_WR_IGN(INFRA, _mask,					\
			MT6878_TOP_AXI_PROT_EN_MCU_SET,			\
			MT6878_TOP_AXI_PROT_EN_MCU_CLR,			\
			MT6878_TOP_AXI_PROT_EN_MCU_STA)

#define MT6878_BP_MD(_mask)						\
	BUS_PROT_WR_IGN(INFRA, _mask,					\
			MT6878_TOP_AXI_PROT_EN_MD_SET,			\
			MT6878_TOP_AXI_PROT_EN_MD_CLR,			\
			MT6878_TOP_AXI_PROT_EN_MD_STA)

#define MT6878_BP_DRAMC(_mask)						\
	BUS_PROT_WR_IGN(INFRA, _mask,					\
			MT6878_TOP_AXI_PROT_EN_DRAMC_SET,		\
			MT6878_TOP_AXI_PROT_EN_DRAMC_CLR,		\
			MT6878_TOP_AXI_PROT_EN_DRAMC_STA)

#define MT6878_BP_VLP(_mask)						\
	BUS_PROT_WR_IGN(VLP, _mask,					\
			MT6878_VLP_AXI_PROT_EN_SET,			\
			MT6878_VLP_AXI_PROT_EN_CLR,			\
			MT6878_VLP_AXI_PROT_EN_STA)

/* Order of the access-controllers phandles */
static enum scpsys_bus_prot_block scpsys_bus_prot_blocks_mt6878[] = {
	BUS_PROT_BLOCK_INFRA, BUS_PROT_BLOCK_VLP, BUS_PROT_BLOCK_EMI,
};

static const struct scpsys_domain_data scpsys_domain_data_mt6878[] = {
	[MT6878_POWER_DOMAIN_MD] = {
		.name = "md",
		/* The modem power sequence only waits for the first ACK */
		.ctl_offs = 0xe00,
		.sta_mask = MT6878_PWR_ACK,
		.pwr_sta_offs = 0xe00,
		.pwr_sta2nd_offs = 0xe00,
		.ext_buck_iso_offs = 0xf24,
		.ext_buck_iso_mask = GENMASK(1, 0),
		.bp_cfg = {
			BUS_PROT_WR(INFRA, MT6878_TOP_AXI_PROT_EN_INFRA_1_MD,
				    MT6878_TOP_AXI_PROT_EN_INFRA_1_SET,
				    MT6878_TOP_AXI_PROT_EN_INFRA_1_CLR,
				    MT6878_TOP_AXI_PROT_EN_INFRA_1_STA),
			BUS_PROT_WR(INFRA, MT6878_TOP_AXI_PROT_EN_INFRA_MD,
				    MT6878_TOP_AXI_PROT_EN_INFRA_SET,
				    MT6878_TOP_AXI_PROT_EN_INFRA_CLR,
				    MT6878_TOP_AXI_PROT_EN_INFRA_STA),
			BUS_PROT_WR(EMI, MT6878_NEMICFG_AO_MEM_PROT_EN_GLITCH_MD,
				    MT6878_NEMICFG_AO_MEM_PROT_EN_SET,
				    MT6878_NEMICFG_AO_MEM_PROT_EN_CLR,
				    MT6878_NEMICFG_AO_MEM_PROT_EN_STA),
		},
		.caps = MTK_SCPD_MODEM_PWRSEQ | MTK_SCPD_EXT_BUCK_ISO |
			MTK_SCPD_SKIP_RESET_B | MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT6878_POWER_DOMAIN_CONN] = {
		.name = "conn",
		MT6878_PWR_CON_STA(0xe04),
		.bp_cfg = {
			MT6878_BP_MCU(MT6878_TOP_AXI_PROT_EN_MCU_CONN),
			MT6878_BP_INFRA_1(MT6878_TOP_AXI_PROT_EN_INFRA_1_CONN),
			MT6878_BP_MCU(MT6878_TOP_AXI_PROT_EN_MCU_CONN_2ND),
			MT6878_BP_INFRA(MT6878_TOP_AXI_PROT_EN_INFRA_CONN),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT6878_POWER_DOMAIN_AUDIO] = {
		.name = "audio",
		MT6878_PWR_CON_STA(0xe18),
		.sram_pdn_bits = MT6878_SRAM_PDN,
		.sram_pdn_ack_bits = MT6878_SRAM_PDN_ACK,
		.bp_cfg = {
			MT6878_BP_PERI(MT6878_TOP_AXI_PROT_EN_PERI_AUDIO),
		},
	},
	[MT6878_POWER_DOMAIN_CSI_RX] = {
		.name = "csi-rx",
		MT6878_PWR_CON_STA(0xe9c),
	},
	[MT6878_POWER_DOMAIN_SSUSB] = {
		.name = "ssusb",
		MT6878_PWR_CON_STA(0xea8),
		.sram_pdn_bits = MT6878_SRAM_PDN,
		.sram_pdn_ack_bits = MT6878_SRAM_PDN_ACK,
		.bp_cfg = {
			MT6878_BP_PERI(MT6878_TOP_AXI_PROT_EN_PERI_SSUSB),
		},
	},
	[MT6878_POWER_DOMAIN_MFG0_SHUTDOWN] = {
		.name = "mfg0-shutdown",
		MT6878_PWR_CON_STA(0xeb4),
		.sram_pdn_bits = MT6878_SRAM_PDN,
		.sram_pdn_ack_bits = MT6878_SRAM_PDN_ACK,
		.bp_cfg = {
			MT6878_BP_MD(MT6878_TOP_AXI_PROT_EN_MD_MFG0),
			MT6878_BP_INFRA(MT6878_TOP_AXI_PROT_EN_INFRA_MFG0),
		},
		.caps = MTK_SCPD_SRAM_ISO | MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT6878_POWER_DOMAIN_DISP] = {
		.name = "disp",
		MT6878_PWR_CON_STA(0xe70),
		.sram_pdn_bits = MT6878_SRAM_PDN,
		.sram_pdn_ack_bits = MT6878_SRAM_PDN_ACK,
		.bp_cfg = {
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_DISP),
		},
	},
	[MT6878_POWER_DOMAIN_VEN0] = {
		.name = "ven0",
		MT6878_PWR_CON_STA(0xe40),
		.sram_pdn_bits = MT6878_SRAM_PDN,
		.sram_pdn_ack_bits = MT6878_SRAM_PDN_ACK,
		.bp_cfg = {
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_VEN0),
			MT6878_BP_MM_1(MT6878_TOP_AXI_PROT_EN_MM_1_VEN0),
		},
	},
	[MT6878_POWER_DOMAIN_CAM_VCORE] = {
		.name = "cam-vcore",
		MT6878_PWR_CON_STA(0xe5c),
		.bp_cfg = {
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_CAM_VCORE),
			MT6878_BP_MM_1(MT6878_TOP_AXI_PROT_EN_MM_1_CAM_VCORE),
		},
		.caps = MTK_SCPD_DOMAIN_SUPPLY,
	},
	[MT6878_POWER_DOMAIN_CAM_CCU] = {
		.name = "cam-ccu",
		MT6878_PWR_CON_STA(0xe60),
		.sram_pdn_bits = MT6878_SRAM_PDN,
		.sram_pdn_ack_bits = MT6878_SRAM_PDN_ACK,
		.bp_cfg = {
			MT6878_BP_DRAMC(MT6878_TOP_AXI_PROT_EN_DRAMC_CAM_CCU),
			MT6878_BP_DRAMC(MT6878_TOP_AXI_PROT_EN_DRAMC_CAM_CCU_2ND),
			MT6878_BP_INFRA(MT6878_TOP_AXI_PROT_EN_INFRA_CAM_CCU),
		},
	},
	[MT6878_POWER_DOMAIN_CAM_CCU_AO] = {
		.name = "cam-ccu-ao",
		MT6878_PWR_CON_STA(0xe64),
	},
	[MT6878_POWER_DOMAIN_CAM_MAIN] = {
		.name = "cam-main",
		MT6878_PWR_CON_STA(0xe48),
		.sram_pdn_bits = MT6878_SRAM_PDN,
		.sram_pdn_ack_bits = MT6878_SRAM_PDN_ACK,
		.bp_cfg = {
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_CAM_MAIN),
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_CAM_MAIN_2ND),
		},
	},
	[MT6878_POWER_DOMAIN_CAM_SUBA] = {
		.name = "cam-suba",
		MT6878_PWR_CON_STA(0xe50),
		.sram_pdn_bits = MT6878_SRAM_PDN,
		.sram_pdn_ack_bits = MT6878_SRAM_PDN_ACK,
		.bp_cfg = {
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_CAM_SUBA),
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_CAM_SUBA_2ND),
		},
	},
	[MT6878_POWER_DOMAIN_CAM_SUBB] = {
		.name = "cam-subb",
		MT6878_PWR_CON_STA(0xe54),
		.sram_pdn_bits = MT6878_SRAM_PDN,
		.sram_pdn_ack_bits = MT6878_SRAM_PDN_ACK,
		.bp_cfg = {
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_CAM_SUBB),
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_CAM_SUBB_2ND),
		},
	},
	[MT6878_POWER_DOMAIN_ISP_VCORE] = {
		.name = "isp-vcore",
		MT6878_PWR_CON_STA(0xe34),
		.bp_cfg = {
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_ISP_VCORE),
			MT6878_BP_MM_1(MT6878_TOP_AXI_PROT_EN_MM_1_ISP_VCORE),
		},
		.caps = MTK_SCPD_DOMAIN_SUPPLY,
	},
	[MT6878_POWER_DOMAIN_ISP_MAIN] = {
		.name = "isp-main",
		MT6878_PWR_CON_STA(0xe28),
		.sram_pdn_bits = MT6878_SRAM_PDN,
		.sram_pdn_ack_bits = MT6878_SRAM_PDN_ACK,
		.bp_cfg = {
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_ISP_MAIN),
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_ISP_MAIN_2ND),
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_ISP_MAIN_3RD),
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_ISP_MAIN_4TH),
		},
	},
	[MT6878_POWER_DOMAIN_ISP_DIP1] = {
		.name = "isp-dip1",
		MT6878_PWR_CON_STA(0xe2c),
		.sram_pdn_bits = MT6878_SRAM_PDN,
		.sram_pdn_ack_bits = MT6878_SRAM_PDN_ACK,
		.bp_cfg = {
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_ISP_DIP1),
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_ISP_DIP1_2ND),
		},
	},
	[MT6878_POWER_DOMAIN_VDE0] = {
		.name = "vde0",
		MT6878_PWR_CON_STA(0xe38),
		.sram_pdn_bits = MT6878_SRAM_PDN,
		.sram_pdn_ack_bits = MT6878_SRAM_PDN_ACK,
		.bp_cfg = {
			MT6878_BP_MM(MT6878_TOP_AXI_PROT_EN_MM_VDE0),
			MT6878_BP_MM_1(MT6878_TOP_AXI_PROT_EN_MM_1_VDE0),
		},
	},
	[MT6878_POWER_DOMAIN_MM_PROC_DORMANT] = {
		.name = "mm-proc-dormant",
		MT6878_PWR_CON_STA(0xe7c),
		/* Dormant: the SRAM is put to sleep instead of powered down */
		.sram_pdn_bits = MT6878_SRAM_SLP,
		.sram_pdn_ack_bits = MT6878_SRAM_SLP_ACK,
		.bp_cfg = {
			MT6878_BP_VLP(MT6878_VLP_AXI_PROT_EN_MM_PROC),
			MT6878_BP_VLP(MT6878_VLP_AXI_PROT_EN_MM_PROC_2ND),
		},
		.caps = MTK_SCPD_SRAM_ISO | MTK_SCPD_SRAM_PDN_INVERTED,
	},
	/*
	 * MT6878_POWER_DOMAIN_MM_INFRA and MT6878_POWER_DOMAIN_SSR are HW
	 * Voter domains, MT6878_POWER_DOMAIN_APU is managed by the APU
	 * firmware and is not supported.
	 */
};

static const struct scpsys_hwv_domain_data scpsys_hwv_domain_data_mt6878[] = {
	[MT6878_POWER_DOMAIN_SSR] = {
		.name = "ssrsys",
		.set = MT6878_HWV_MTCMOS_SET,
		.clr = MT6878_HWV_MTCMOS_CLR,
		.done = MT6878_HWV_MTCMOS_DONE,
		.en = MT6878_HWV_MTCMOS_EN,
		.set_sta = MT6878_HWV_MTCMOS_SET_STA,
		.clr_sta = MT6878_HWV_MTCMOS_CLR_STA,
		.setclr_bit = 1,
		.caps = MTK_SCPD_INFRA_PWR_CTL,
	},
	[MT6878_POWER_DOMAIN_MM_INFRA] = {
		.name = "mm-infra",
		.set = MT6878_HWV_MTCMOS_SET,
		.clr = MT6878_HWV_MTCMOS_CLR,
		.done = MT6878_HWV_MTCMOS_DONE,
		.en = MT6878_HWV_MTCMOS_EN,
		.set_sta = MT6878_HWV_MTCMOS_SET_STA,
		.clr_sta = MT6878_HWV_MTCMOS_CLR_STA,
		.setclr_bit = 0,
		.caps = MTK_SCPD_INFRA_PWR_CTL,
	},
};

static const struct scpsys_soc_data mt6878_scpsys_data = {
	.domains_data = scpsys_domain_data_mt6878,
	.num_domains = ARRAY_SIZE(scpsys_domain_data_mt6878),
	.hwv_domains_data = scpsys_hwv_domain_data_mt6878,
	.num_hwv_domains = ARRAY_SIZE(scpsys_hwv_domain_data_mt6878),
	.bus_prot_blocks = scpsys_bus_prot_blocks_mt6878,
	.num_bus_prot_blocks = ARRAY_SIZE(scpsys_bus_prot_blocks_mt6878),
	.type = SCPSYS_MTCMOS_TYPE_DIRECT_CTL,
};

#endif /* __SOC_MEDIATEK_MT6878_PM_DOMAINS_H */
