// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek TF-A (BL31) log buffer, read only: /sys/kernel/debug/mtk-atf-log.
 *
 * The boot loader reserves the buffer ("mediatek,atf-log-reserved"); BL31
 * writes its messages into a ring after a 512 byte control block. The layout
 * of the control block is that of the MediaTek GPL kernel's atf_log driver
 * (kernel_device_modules drivers/misc/mediatek/atf/atf_log.c, GPL-2.0):
 * log address and size, then the write and read offsets. Unlike that
 * driver this one never writes the buffer (the read offset stays as it is)
 * and calls no firmware: the debugfs file shows the control fields and the
 * ring from the oldest to the newest byte.
 *
 * Copyright (c) 2019 MediaTek Inc.
 * Copyright (c) 2026 serlk
 */

#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/types.h>

#define ATF_LOG_CTRL_SIZE	512

struct atf_log_ctrl {
	u64 log_addr;
	u64 log_size;
	u32 write_offset;
	u32 read_offset;
	u64 crash_log_addr;
	u64 crash_log_size;
	u32 total_write_count;
	u32 crash_flag;
} __packed;

static void __iomem *atf_log_base;
static resource_size_t atf_log_len;

static int atf_log_show(struct seq_file *m, void *v)
{
	struct atf_log_ctrl ctrl;
	size_t ring, w, chunk;
	char *buf;

	memcpy_fromio(&ctrl, atf_log_base, sizeof(ctrl));
	seq_printf(m, "# atf log: addr %#llx size %#llx write %#x read %#x total %u crash addr %#llx size %#llx flag %#x\n",
		   ctrl.log_addr, ctrl.log_size, ctrl.write_offset,
		   ctrl.read_offset, ctrl.total_write_count,
		   ctrl.crash_log_addr, ctrl.crash_log_size, ctrl.crash_flag);

	ring = min_t(u64, ctrl.log_size, atf_log_len - ATF_LOG_CTRL_SIZE);
	w = ctrl.write_offset;
	if (!ring || w >= ring)
		return 0;

	buf = kvmalloc(ring, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	memcpy_fromio(buf, atf_log_base + ATF_LOG_CTRL_SIZE, ring);

	/* the ring wrapped: oldest bytes after the write offset */
	if (ctrl.total_write_count > w) {
		chunk = strnlen(buf + w, ring - w);
		seq_write(m, buf + w, chunk);
	}
	seq_write(m, buf, strnlen(buf, w));
	kvfree(buf);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(atf_log);

static int __init mtk_atf_log_init(void)
{
	struct device_node *np;
	struct resource res;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "mediatek,atf-log-reserved");
	if (!np)
		return 0;
	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret || resource_size(&res) <= ATF_LOG_CTRL_SIZE)
		return 0;

	atf_log_len = resource_size(&res);
	atf_log_base = ioremap_wc(res.start, atf_log_len);
	if (!atf_log_base)
		return -ENOMEM;

	debugfs_create_file("mtk-atf-log", 0400, NULL, NULL, &atf_log_fops);
	pr_info("mtk-atf-log: %pR\n", &res);
	return 0;
}
late_initcall(mtk_atf_log_init);
