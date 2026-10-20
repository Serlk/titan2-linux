/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek CONNINFRA: interface for the connectivity radio drivers
 * (Wi-Fi, Bluetooth, GNSS, FM) of MT6878 class SoCs.
 *
 * Copyright (c) 2026 serlk
 */

#ifndef __SOC_MEDIATEK_MTK_CONNINFRA_H
#define __SOC_MEDIATEK_MTK_CONNINFRA_H

#include <linux/err.h>
#include <linux/types.h>

struct device;
struct mtk_conninfra;

enum mtk_conninfra_radio {
	MTK_CONNINFRA_BT,
	MTK_CONNINFRA_FM,
	MTK_CONNINFRA_GPS,
	MTK_CONNINFRA_WIFI,
	MTK_CONNINFRA_RADIO_MAX,
};

#if IS_ENABLED(CONFIG_MTK_CONNINFRA)

/*
 * Look up the CONNINFRA instance referenced by the "mediatek,conninfra"
 * phandle of @dev. Returns -EPROBE_DEFER until the CONNINFRA driver has
 * probed. The reference is dropped when @dev is unbound.
 */
struct mtk_conninfra *devm_mtk_conninfra_get(struct device *dev);

/*
 * Power CONNINFRA on for @radio. The first user runs the full power-on
 * sequence (PMIC rails, CONN power domain, EMI remapping, D-die and
 * A-die configuration, PLL, low power settings); later users only
 * update the shared radio status. Sleeps.
 */
int mtk_conninfra_radio_on(struct mtk_conninfra *ci, enum mtk_conninfra_radio radio);
void mtk_conninfra_radio_off(struct mtk_conninfra *ci, enum mtk_conninfra_radio radio);

/*
 * Keep the CONN_INFRA "off" domain (bus, clocks) awake for register
 * access from the AP. Calls nest. Only valid while a radio is on.
 */
int mtk_conninfra_wakeup(struct mtk_conninfra *ci);
void mtk_conninfra_sleep(struct mtk_conninfra *ci);

/*
 * Wi-Fi subsystem (WFSYS): power the WFSYS top on, release the WM MCU
 * from reset and wait until the ROM code reports idle. The firmware
 * must already be in EMI (the stock boot loader puts it there).
 * Requires mtk_conninfra_radio_on(MTK_CONNINFRA_WIFI).
 */
int mtk_conninfra_wfsys_on(struct mtk_conninfra *ci);
void mtk_conninfra_wfsys_off(struct mtk_conninfra *ci);
bool mtk_conninfra_wfsys_is_on(struct mtk_conninfra *ci);

/* A-die chip id read over the CONNINFRA RF SPI (0 until the first radio_on) */
u32 mtk_conninfra_adie_id(struct mtk_conninfra *ci);

/* CONNSYS EMI region set up by the boot loader (from "emi-addr"/"emi-size") */
int mtk_conninfra_get_emi(struct mtk_conninfra *ci, phys_addr_t *base, u32 *size);

#else

static inline struct mtk_conninfra *devm_mtk_conninfra_get(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

static inline int mtk_conninfra_radio_on(struct mtk_conninfra *ci,
					 enum mtk_conninfra_radio radio)
{
	return -ENODEV;
}

static inline void mtk_conninfra_radio_off(struct mtk_conninfra *ci,
					   enum mtk_conninfra_radio radio)
{
}

static inline int mtk_conninfra_wakeup(struct mtk_conninfra *ci)
{
	return -ENODEV;
}

static inline void mtk_conninfra_sleep(struct mtk_conninfra *ci)
{
}

static inline int mtk_conninfra_wfsys_on(struct mtk_conninfra *ci)
{
	return -ENODEV;
}

static inline void mtk_conninfra_wfsys_off(struct mtk_conninfra *ci)
{
}

static inline bool mtk_conninfra_wfsys_is_on(struct mtk_conninfra *ci)
{
	return false;
}

static inline u32 mtk_conninfra_adie_id(struct mtk_conninfra *ci)
{
	return 0;
}

static inline int mtk_conninfra_get_emi(struct mtk_conninfra *ci,
					phys_addr_t *base, u32 *size)
{
	return -ENODEV;
}

#endif

#endif /* __SOC_MEDIATEK_MTK_CONNINFRA_H */
