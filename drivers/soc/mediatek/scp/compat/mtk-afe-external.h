/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stand-in for sound/soc/mediatek/common/mtk-afe-external.h of the MediaTek
 * device-modules tree, for the SCP driver ported to mainline 7.2 for the
 * Unihertz Titan 2. The downstream AFE driver uses this notifier chain to take
 * the SCP/AP/ADSP three-way hardware semaphore around audio register
 * accesses; mainline audio does not, so registering is a no-op.
 * The event numbers are those of the downstream header.
 */
#ifndef __TITAN2_COMPAT_MTK_AFE_EXTERNAL_H__
#define __TITAN2_COMPAT_MTK_AFE_EXTERNAL_H__

#include <linux/notifier.h>

enum {
	NOTIFIER_VOW_ALLOCATE_MEM = 1,
	NOTIFIER_ULTRASOUND_ALLOCATE_MEM,
	NOTIFIER_ADSP_3WAY_SEMAPHORE_GET,
	NOTIFIER_ADSP_3WAY_SEMAPHORE_RELEASE,
	NOTIFIER_SCP_3WAY_SEMAPHORE_GET,
	NOTIFIER_SCP_3WAY_SEMAPHORE_RELEASE,
};

static inline int register_3way_semaphore_notifier(struct notifier_block *nb)
{
	return 0;
}

static inline int unregister_3way_semaphore_notifier(struct notifier_block *nb)
{
	return 0;
}

#endif
