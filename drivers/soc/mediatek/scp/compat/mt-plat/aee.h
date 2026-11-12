/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stand-in for the MediaTek AEE (exception reporting) header of the
 * device-modules tree, for the SCP driver ported to mainline 7.2 for the
 * Unihertz Titan 2. AEE does not exist on mainline: the mode reads as a
 * customer user build (no waiting for an AEE coredump reader) and the
 * report calls do nothing; the SCP driver still prints its own messages.
 */
#ifndef __TITAN2_COMPAT_MT_PLAT_AEE_H__
#define __TITAN2_COMPAT_MT_PLAT_AEE_H__

#define AEE_MODE_MTK_ENG		1
#define AEE_MODE_MTK_USER		2
#define AEE_MODE_CUSTOMER_ENG		3
#define AEE_MODE_CUSTOMER_USER		4

#define DB_OPT_DEFAULT			(0)

static inline int aee_get_mode(void)
{
	return AEE_MODE_CUSTOMER_USER;
}

#define aee_kernel_exception(module, msg...)	do { } while (0)
#define aee_kernel_warning(module, msg...)	do { } while (0)
#define aee_kernel_warning_api(file, line, db_opt, module, msg...) \
	do { } while (0)
#define aee_kernel_exception_api(file, line, db_opt, module, msg...) \
	do { } while (0)

#endif
