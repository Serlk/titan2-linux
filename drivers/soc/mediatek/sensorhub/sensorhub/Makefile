# SPDX-License-Identifier: GPL-2.0
# sap_custom_cmd.o is not built: no SAP on MT6878 (stubs in sap_custom_cmd.h).

ccflags-y += -I$(src)/../core
ccflags-y += -I$(src)/../../scp/include

obj-$(CONFIG_MTK_SENSORHUB) += sensorhub.o
sensorhub-objs += tiny_crc8.o
sensorhub-objs += ready.o
sensorhub-objs += ipi_comm.o
sensorhub-objs += sensor_comm.o
sensorhub-objs += sensor_list.o
sensorhub-objs += transceiver.o
sensorhub-objs += share_memory.o
sensorhub-objs += timesync.o
sensorhub-objs += debug.o
sensorhub-objs += custom_cmd.o
sensorhub-objs += sensor_freq.o
