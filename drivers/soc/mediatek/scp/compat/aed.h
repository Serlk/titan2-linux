/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stand-in for the MediaTek AED header (see mt-plat/aee.h), for the SCP
 * driver ported to mainline 7.2 for the Unihertz Titan 2.
 * CONFIG_MTK_AEE_FEATURE is never set on mainline, so the one AED call in
 * scp_excep.c is compiled out; only the types are needed here.
 */
#ifndef __TITAN2_COMPAT_AED_H__
#define __TITAN2_COMPAT_AED_H__

#include <mt-plat/aee.h>

#endif
