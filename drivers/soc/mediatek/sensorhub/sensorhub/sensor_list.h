/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 MediaTek Inc.
 *
 * Ported to mainline 7.2 for the Unihertz Titan 2 from the MediaTek GPL
 * device-modules tree (kernel_device_modules-6.1).
 */

#ifndef _SENSOR_LIST_H_
#define _SENSOR_LIST_H_

#include "hf_manager.h"

int sensor_list_get_list(struct sensor_info *list, unsigned int size);
int sensor_list_init(void);
void sensor_list_exit(void);

#endif
