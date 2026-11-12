/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 MediaTek Inc.
 *
 * Ported to mainline 7.2 for the Unihertz Titan 2 from the MediaTek GPL
 * device-modules tree (kernel_device_modules-6.1,
 * drivers/misc/mediatek/vcp/include/vcp.h and rv/vcp_helper.h): the IPI,
 * reserved-memory and feature numbering the MT6878 VCP firmware expects,
 * and the API the downstream vcodec/mmdvfs drivers use. Only what the reduced
 * port in vcp_core.c implements is declared.
 */

#ifndef __MTK_VCP_H__
#define __MTK_VCP_H__

#include <linux/notifier.h>
#include <linux/types.h>
#include <linux/soc/mediatek/mtk_tinysys_ipi.h>

/* message sizes in 32-bit words (downstream vcp.h) */
#define PIN_OUT_C_SIZE_SLEEP_0		2
#define PIN_OUT_SIZE_TEST_0		2
#define PIN_IN_SIZE_VCP_READY_0		1
#define PIN_OUT_SIZE_VDEC		18
#define PIN_IN_SIZE_VDEC		18
#define PIN_OUT_SIZE_VENC		18
#define PIN_IN_SIZE_VENC		18

enum vcp_core_id {
	VCP_A_ID = 0,
	VCP_CORE_TOTAL = 1,
};

/*
 * IPI channel numbers. They are the first column of the send-table /
 * recv-table of the factory "mediatek,vcp" node and must not be renumbered.
 */
enum {
	IPI_OUT_VDEC_1			= 0,
	IPI_IN_VDEC_1			= 1,
	IPI_OUT_C_SLEEP_0		= 2,
	IPI_OUT_TEST_0			= 3,
	IPI_IN_VCP_ERROR_INFO_0		= 4,
	IPI_IN_VCP_READY_0		= 5,
	IPI_IN_VCP_RAM_DUMP_0		= 6,
	IPI_OUT_VCP_CONNSYS		= 7,
	IPI_IN_VCP_CONNSYS		= 8,
	IPI_OUT_MMDVFS			= 9,
	IPI_IN_MMDVFS			= 10,
	IPI_OUT_MMQOS			= 11,
	IPI_IN_MMQOS			= 12,
	IPI_OUT_MMDEBUG			= 13,
	IPI_IN_MMDEBUG			= 14,
	IPI_OUT_C_VCP_HWVOTER_DEBUG	= 15,
	IPI_OUT_VENC_0			= 16,
	IPI_IN_VENC_0			= 17,
	IPI_OUT_VCP_MPOOL_0		= 18,
	IPI_IN_VCP_MPOOL_0		= 19,
	IPI_OUT_C_SLEEP_1		= 20,
	IPI_OUT_TEST_1			= 21,
	IPI_OUT_LOGGER_CTRL		= 22,
	IPI_OUT_VCPCTL_1		= 23,
	IPI_IN_VCP_ERROR_INFO_1		= 24,
	IPI_IN_LOGGER_CTRL		= 25,
	IPI_IN_VCP_READY_1		= 26,
	IPI_IN_VCP_RAM_DUMP_1		= 27,
	IPI_OUT_VCP_MPOOL_1		= 28,
	IPI_IN_VCP_MPOOL_1		= 29,
	VCP_IPI_COUNT
};

enum VCP_NOTIFY_EVENT {
	VCP_EVENT_READY = 0,
	VCP_EVENT_STOP,
	VCP_EVENT_SUSPEND,
	VCP_EVENT_RESUME,
};

/* AP/VCP shared memory regions, index = first column of vcp-mem-tbl */
enum vcp_reserve_mem_id_t {
	VDEC_MEM_ID,
	VENC_MEM_ID,
	VCP_A_LOGGER_MEM_ID,
	VDEC_SET_PROP_MEM_ID,
	VENC_SET_PROP_MEM_ID,
	VDEC_VCP_LOG_INFO_ID,
	VENC_VCP_LOG_INFO_ID,
	GCE_MEM_ID,
	MMDVFS_MEM_ID,
	MMQOS_MEM_ID,
	VCP_SECURE_DUMP_ID,
	NUMS_MEM_ID,
};

/* users that keep the VCP powered (vcp_register_feature) */
enum feature_id {
	RTOS_FEATURE_ID,
	VDEC_FEATURE_ID,
	VENC_FEATURE_ID,
	GCE_FEATURE_ID,
	MMDVFS_FEATURE_ID,
	MMQOS_FEATURE_ID,
	MMDEBUG_FEATURE_ID,
	NUM_FEATURE_ID,
};

/* DMA devices, index = vcp-support - 1 of the node that provides it */
enum VCP_IOMMU_DEV {
	VCP_IOMMU_256MB1 = 0,
	VCP_IOMMU_VDEC_512MB1 = 1,
	VCP_IOMMU_VENC_512MB2 = 2,
	VCP_IOMMU_WORK_256MB2 = 3,
	VCP_IOMMU_UBE_LAT = 4,
	VCP_IOMMU_UBE_CORE = 5,
	VCP_IOMMU_SEC = 6,
	VCP_IOMMU_DEV_NUM,
};

/* addresses handed to the VCP: bits 35..32 are folded into bits 3..0 */
#define VCP_PACK_IOVA(addr)	((u32)((addr) | (((addr) >> 32) & 0xF)))
#define VCP_UNPACK_IOVA(addr)	\
	((u64)((addr) & 0xFFFFFFF0) | (((u64)(addr) & 0xF) << 32))

#define VCP_SYNC_TIMEOUT_MS	1000

extern struct mtk_mbox_device vcp_mboxdev;
extern struct mtk_ipi_device vcp_ipidev;

unsigned int is_vcp_ready(enum vcp_core_id id);
unsigned int is_vcp_suspending(void);
unsigned int get_vcp_generation(void);

void vcp_A_register_notify(struct notifier_block *nb);
void vcp_A_unregister_notify(struct notifier_block *nb);

phys_addr_t vcp_get_reserve_mem_phys(enum vcp_reserve_mem_id_t id);
phys_addr_t vcp_get_reserve_mem_virt(enum vcp_reserve_mem_id_t id);
phys_addr_t vcp_get_reserve_mem_size(enum vcp_reserve_mem_id_t id);

int vcp_register_feature(enum feature_id id);
int vcp_deregister_feature(enum feature_id id);

struct device *vcp_get_io_device(enum VCP_IOMMU_DEV io_num);

int vcp_awake_lock(void *_vcp_id);
int vcp_awake_unlock(void *_vcp_id);

#endif /* __MTK_VCP_H__ */
