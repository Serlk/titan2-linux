/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 *
 * MediaTek MT6363 SPMI PMIC top level registers
 *
 * Register addresses from the MediaTek downstream kernel.
 */

#ifndef __MFD_MT6363_REGISTERS_H__
#define __MFD_MT6363_REGISTERS_H__

#define MT6363_TOPSTATUS		0x1e
#define MT6363_MISC_TOP_INT_CON0	0x37
#define MT6363_MISC_TOP_INT_STATUS0	0x43
#define MT6363_TOP_INT_STATUS1		0x4e
#define MT6363_PSC_TOP_INT_CON0		0x90f
#define MT6363_PSC_TOP_INT_STATUS0	0x91b
#define MT6363_STRUP_CON11		0xa0e
#define MT6363_STRUP_CON12		0xa0f
#define MT6363_HK_TOP_INT_CON0		0xf92
#define MT6363_HK_TOP_INT_STATUS0	0xf9e
#define MT6363_BUCK_TOP_INT_CON0	0x1411
#define MT6363_BUCK_TOP_INT_STATUS0	0x141d
#define MT6363_LDO_TOP_INT_CON0		0x1b11
#define MT6363_LDO_TOP_INT_STATUS0	0x1b29

/* Interrupt numbers, as the PMIC interrupt domain hwirqs */
#define MT6363_IRQ_BUCK_BASE		0
#define MT6363_IRQ_LDO_BASE		16
#define MT6363_IRQ_PSC_BASE		48
#define MT6363_IRQ_PWRKEY		48
#define MT6363_IRQ_HOMEKEY		49
#define MT6363_IRQ_HOMEKEY_2		50
#define MT6363_IRQ_PWRKEY_R		51
#define MT6363_IRQ_HOMEKEY_R		52
#define MT6363_IRQ_HOMEKEY_2_R		53
#define MT6363_IRQ_MISC_BASE		64
#define MT6363_IRQ_HK_BASE		80
#define MT6363_IRQ_NR			96

#endif /* __MFD_MT6363_REGISTERS_H__ */
