/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 MediaTek Inc.
 * Author: Chuan-wen Chen <chuan-wen.chen@mediatek.com>
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 *
 * Power domain indices of the MediaTek MT6878 SCPSYS power controller.
 *
 * The numbering follows the MediaTek downstream kernel, so that device
 * trees written against it map one to one.
 */

#ifndef _DT_BINDINGS_POWER_MEDIATEK_MT6878_POWER_H
#define _DT_BINDINGS_POWER_MEDIATEK_MT6878_POWER_H

#define MT6878_POWER_DOMAIN_MD			0
#define MT6878_POWER_DOMAIN_CONN		1
#define MT6878_POWER_DOMAIN_AUDIO		2
#define MT6878_POWER_DOMAIN_CSI_RX		3
#define MT6878_POWER_DOMAIN_SSR			4
#define MT6878_POWER_DOMAIN_SSUSB		5
#define MT6878_POWER_DOMAIN_MFG0_SHUTDOWN	6
#define MT6878_POWER_DOMAIN_MM_INFRA		7
#define MT6878_POWER_DOMAIN_DISP		8
#define MT6878_POWER_DOMAIN_VEN0		9
#define MT6878_POWER_DOMAIN_CAM_VCORE		10
#define MT6878_POWER_DOMAIN_CAM_CCU		11
#define MT6878_POWER_DOMAIN_CAM_CCU_AO		12
#define MT6878_POWER_DOMAIN_CAM_MAIN		13
#define MT6878_POWER_DOMAIN_CAM_SUBA		14
#define MT6878_POWER_DOMAIN_CAM_SUBB		15
#define MT6878_POWER_DOMAIN_ISP_VCORE		16
#define MT6878_POWER_DOMAIN_ISP_MAIN		17
#define MT6878_POWER_DOMAIN_ISP_DIP1		18
#define MT6878_POWER_DOMAIN_VDE0		19
#define MT6878_POWER_DOMAIN_MM_PROC_DORMANT	20
#define MT6878_POWER_DOMAIN_APU			21

#endif /* _DT_BINDINGS_POWER_MEDIATEK_MT6878_POWER_H */
