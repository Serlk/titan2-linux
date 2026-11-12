/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 MediaTek Inc.
 *
 * Ported to mainline 7.2 for the Unihertz Titan 2 from the MediaTek GPL
 * device-modules tree (kernel_device_modules-6.1).
 */

#ifndef _CUSTOM_CMD_H_
#define _CUSTOM_CMD_H_

#include "hf_sensor_io.h"

int custom_cmd_comm_with(int sensor_type, struct custom_cmd *cust_cmd);
int custom_cmd_init(void);
void custom_cmd_exit(void);

#endif
