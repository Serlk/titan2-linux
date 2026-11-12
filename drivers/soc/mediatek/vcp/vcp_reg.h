/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 MediaTek Inc.
 *
 * Ported to mainline 7.2 for the Unihertz Titan 2 from the MediaTek GPL
 * device-modules tree (kernel_device_modules-6.1,
 * drivers/misc/mediatek/vcp/rv/vcp_reg.h). Only the registers used by
 * vcp_core.c are kept.
 */

#ifndef __MTK_VCP_REG_H__
#define __MTK_VCP_REG_H__

/* "vcp_cfgreg" */
#define VCP_TO_SPM_REG			0x0094
#define R_GIPC_IN_SET			0x0098
#define  B_GIPC3_SETCLR_1		BIT(13)
#define VCP_GPR_DEBUG_HINT		0x0140
#define  B_SERR				BIT(0)
#define VCP_BUS_DEBUG_OUT		0x0150

/* "vcp_cfgreg_core0" / "vcp_cfgreg_core1" */
#define R_CORE_SW_RSTN_CLR		0x0000
#define R_CORE_SW_RSTN_SET		0x0004
#define R_CORE_WDT_IRQ			0x0030
#define  B_WDT_IRQ			BIT(0)
#define R_CORE_GENERAL_REG0		0x0040
#define R_CORE_GENERAL_REG1		0x0044	/* DRAM_RESV_ADDR */
#define R_CORE_GENERAL_REG2		0x0048	/* DRAM_RESV_SIZE */
#define R_CORE_GENERAL_REG5		0x0054	/* hart0 reboot handshake */
#define R_CORE_GENERAL_REG6		0x0058	/* hart1 reboot handshake */
#define  CORE_RDY_TO_REBOOT		0x34
#define  CORE_REBOOT_OK			0x1
#define R_CORE_STATUS			0x0070
#define  B_CORE_GATED			BIT(0)
#define  B_HART0_HALT			BIT(1)
#define  B_HART1_HALT			BIT(2)
#define R_CORE_MON_PC			0x0080
#define R_CORE_MON_LR			0x0084
#define R_CORE_MON_SP			0x0088

#define DRAM_RESV_ADDR_REG		R_CORE_GENERAL_REG1
#define DRAM_RESV_SIZE_REG		R_CORE_GENERAL_REG2

#endif /* __MTK_VCP_REG_H__ */
