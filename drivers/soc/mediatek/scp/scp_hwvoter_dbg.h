/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 *
 * Ported to mainline 7.2 for the Unihertz Titan 2 from the MediaTek GPL
 * device-modules tree (kernel_device_modules-6.1).
 */

#ifndef __SCP_HW_VOTER_DBG_H__
#define __SCP_HW_VOTER_DBG_H__

enum {
	HW_VOTER_DBG_CMD_TEST,
	HW_VOTER_DBG_CMD_ENABLE,
};

struct hwvoter_ipi_test_t {
	unsigned char cmd;
	unsigned char type;
	unsigned char op;
	unsigned char clk_category;
	unsigned short clk_id;
	unsigned short val;
};

extern int scp_hw_voter_dbg_init(void);

#endif  /* __SCP_HW_VOTER_DBG_H__ */
