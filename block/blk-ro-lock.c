// SPDX-License-Identifier: GPL-2.0
/*
 * One-way lock of read-only block devices (CONFIG_BLK_DEV_RO_LOCK).
 *
 * Writing "1" to /sys/module/block/parameters/ro_lock (or booting with
 * block.ro_lock=1) locks every block device that is read-only at that moment.
 * For a locked device, until the next boot: BLKROSET cannot make it writable,
 * write bios to it fail instead of only warning, and for a locked disk BLKPG,
 * BLKRRPART and SCSI pass-through (SG_IO, bsg) are refused. New partitions of
 * a locked disk, and new disks behind the same controller as a locked disk
 * (the nearest platform-bus ancestor, e.g. the UFS host), start read-only and
 * locked, so deleting and rescanning a SCSI device does not undo the lock.
 * Devices that were writable at lock time, and new ones behind other
 * controllers (SD card, USB storage, loop, zram), are not affected.
 *
 * The intended use is a system whose root lives on one partition of a disk
 * that also holds another operating system's partitions: early userspace
 * marks the other partitions and the whole disk read-only, leaves the root
 * partition writable and then sets the lock.
 */
#include <linux/blkdev.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include "blk.h"

#define BLK_RO_LOCK_MAX_CTRL	8

static DEFINE_MUTEX(blk_ro_lock_mutex);
static bool blk_ro_lock_set;
static struct device *blk_ro_lock_ctrl[BLK_RO_LOCK_MAX_CTRL];
static int blk_ro_lock_nr_ctrl;

bool blk_ro_locked(void)
{
	return READ_ONCE(blk_ro_lock_set);
}
EXPORT_SYMBOL_GPL(blk_ro_locked);

/* the controller of a device: its nearest ancestor on the platform bus */
static struct device *blk_ro_lock_ctrl_of(struct device *dev)
{
	for (; dev; dev = dev->parent)
		if (dev->bus == &platform_bus_type)
			return dev;
	return NULL;
}

/* is dev behind the controller of a disk that was locked? */
bool blk_ro_locked_device(struct device *dev)
{
	struct device *ctrl;
	int i, n;

	if (!blk_ro_locked())
		return false;
	ctrl = blk_ro_lock_ctrl_of(dev);
	if (!ctrl)
		return false;
	n = smp_load_acquire(&blk_ro_lock_nr_ctrl);
	for (i = 0; i < n; i++)
		if (blk_ro_lock_ctrl[i] == ctrl)
			return true;
	return false;
}
EXPORT_SYMBOL_GPL(blk_ro_locked_device);

static int blk_ro_lock_one(struct device *dev, void *data)
{
	struct block_device *bdev = dev_to_bdev(dev);
	struct device *ctrl;
	int i;

	if (!bdev_read_only(bdev))
		return 0;
	bdev_set_flag(bdev, BD_RO_LOCKED);
	if (bdev_is_partition(bdev))
		return 0;
	ctrl = blk_ro_lock_ctrl_of(dev->parent);
	if (!ctrl)
		return 0;
	for (i = 0; i < blk_ro_lock_nr_ctrl; i++)
		if (blk_ro_lock_ctrl[i] == ctrl)
			return 0;
	if (blk_ro_lock_nr_ctrl == BLK_RO_LOCK_MAX_CTRL) {
		pr_warn("block: ro_lock: too many controllers, %s not recorded\n",
			dev_name(ctrl));
		return 0;
	}
	blk_ro_lock_ctrl[blk_ro_lock_nr_ctrl] = get_device(ctrl);
	smp_store_release(&blk_ro_lock_nr_ctrl, blk_ro_lock_nr_ctrl + 1);
	pr_info("block: ro_lock: new disks behind %s will be read-only\n", dev_name(ctrl));
	return 0;
}

static int blk_ro_lock_param_set(const char *val, const struct kernel_param *kp)
{
	bool lock;
	int ret;

	ret = kstrtobool(val, &lock);
	if (ret)
		return ret;
	if (!lock)
		return blk_ro_locked() ? -EPERM : 0;
	mutex_lock(&blk_ro_lock_mutex);
	if (!blk_ro_locked()) {
		/* flag first: devices added meanwhile see the lock in bdev_alloc/add_disk */
		WRITE_ONCE(blk_ro_lock_set, true);
		class_for_each_device(&block_class, NULL, NULL, blk_ro_lock_one);
		pr_info("block: read-only block devices locked\n");
	}
	mutex_unlock(&blk_ro_lock_mutex);
	return 0;
}

static int blk_ro_lock_param_get(char *buffer, const struct kernel_param *kp)
{
	return sysfs_emit(buffer, "%c\n", blk_ro_locked() ? 'Y' : 'N');
}

static const struct kernel_param_ops blk_ro_lock_param_ops = {
	.set	= blk_ro_lock_param_set,
	.get	= blk_ro_lock_param_get,
};

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX	"block."

module_param_cb(ro_lock, &blk_ro_lock_param_ops, NULL, 0644);
MODULE_PARM_DESC(ro_lock, "Lock the block devices that are read-only now (one-way)");
