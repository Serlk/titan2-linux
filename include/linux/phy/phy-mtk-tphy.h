/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek T-PHY submodes for phy_set_mode_ext()
 *
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 */

#ifndef __PHY_MTK_TPHY_H__
#define __PHY_MTK_TPHY_H__

/**
 * enum mtk_tphy_submode - extra U2 port states, set with phy_set_mode_ext()
 * @MTK_TPHY_SUBMODE_NONE: no submode, only the mode (role) is set
 * @MTK_TPHY_BC11_SW_SET: hand D+/D- over to the BC1.1/BC1.2 charger
 *	detection path (an external charger runs port detection); the USB
 *	transceiver is disconnected from the lines meanwhile
 * @MTK_TPHY_BC11_SW_CLR: give D+/D- back to the USB transceiver
 *
 * The submodes leave the mode (role) of the PHY alone: callers pass the
 * present mode, phy_get_mode(), along with them.
 */
enum mtk_tphy_submode {
	MTK_TPHY_SUBMODE_NONE,
	MTK_TPHY_BC11_SW_SET,
	MTK_TPHY_BC11_SW_CLR,
};

#endif /* __PHY_MTK_TPHY_H__ */
