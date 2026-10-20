/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 *
 * MediaTek MT6369 SPMI PMIC top level registers
 *
 * Register addresses from the MediaTek downstream kernel.
 */

#ifndef __MFD_MT6369_REGISTERS_H__
#define __MFD_MT6369_REGISTERS_H__

#define MT6369_MISC_TOP_INT_CON0	0x3a
#define MT6369_MISC_TOP_INT_STATUS0	0x46
#define MT6369_TOP_INT_STATUS1		0x51
#define MT6369_HK_TOP_INT_CON0		0xf92
#define MT6369_HK_TOP_INT_STATUS0	0xf9e
#define MT6369_BUCK_TOP_INT_CON0	0x1411
#define MT6369_BUCK_TOP_INT_STATUS0	0x1417
#define MT6369_LDO_TOP_INT_CON0		0x1b10
#define MT6369_LDO_TOP_INT_STATUS0	0x1b22
#define MT6369_AUD_TOP_INT_CON0		0x231d
#define MT6369_AUD_TOP_INT_STATUS0	0x2323

/* Interrupt numbers, as the PMIC interrupt domain hwirqs */
#define MT6369_IRQ_BUCK_BASE		0
#define MT6369_IRQ_LDO_BASE		8
#define MT6369_IRQ_MISC_BASE		32
#define MT6369_IRQ_HK_BASE		48
#define MT6369_IRQ_AUD_BASE		56
#define MT6369_IRQ_NR			64

#endif /* __MFD_MT6369_REGISTERS_H__ */
