# SPDX-License-Identifier: GPL-2.0-only
# Ported to mainline 7.2 for the Unihertz Titan 2 from the MediaTek GPL
# device-modules tree (kernel_device_modules-6.1, drivers/misc/mediatek/vcp/rv).
# Reduced port: vcp_helper.c + vcp_awake.c + vcp_irq.c + vcp_feature_table.c
# are folded into vcp_core.c; vcp_logger.c, vcp_excep.c, vcp_l1c.c,
# vcp_vcpctl.c, vcp_hwvoter_dbg.c and vcp_status.c are not ported.

obj-$(CONFIG_MTK_TINYSYS_VCP_SUPPORT) += mtk-vcp.o
mtk-vcp-y := vcp_core.o
