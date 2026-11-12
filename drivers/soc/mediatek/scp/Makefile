# SPDX-License-Identifier: GPL-2.0
# Ported to mainline 7.2 for the Unihertz Titan 2 from the MediaTek GPL
# device-modules tree (kernel_device_modules-6.1, drivers/misc/mediatek/scp/rv).
# Not built: scp_dvfs.c (stubbed in scp_dvfs.h) and sap.c (no SAP on MT6878).

obj-$(CONFIG_MTK_TINYSYS_SCP_SUPPORT) += scp.o
scp-y += scp_helper.o
scp-y += scp_excep.o
scp-y += scp_wrapper_ipi.o
scp-y += scp_irq.o
scp-y += scp_logger.o
scp-y += scp_l1c.o
scp-y += scp_scpctl.o
scp-y += scp_chre_manager.o
scp-y += scp_feature_table.o
scp-y += scp_awake.o
scp-y += scp_hwvoter_dbg.o
scp-y += scp_low_pwr_dbg.o
scp-y += scp_mbrain_dbg.o

ccflags-y += -I$(src)/include -I$(src)/compat
