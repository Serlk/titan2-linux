// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: character device ports with a user supplied header.
 *
 * The MediaTek downstream driver hands the modem's file server requests (channel
 * FS_RX, queue 4) to userspace through /dev/ccci_fs, with the CCCI
 * header left in (PORT_F_USER_HEADER). The same minimal ABI here, for
 * an open file server in userspace:
 *
 *  - read(): one modem packet, 16-byte CCCI header included; a short
 *    buffer gets the start of it and the rest on the next read (as the
 *    MediaTek downstream driver does). Blocks unless O_NONBLOCK.
 *  - write(): one packet, header included. The driver sets data[1] to
 *    the length (or data[0] to the magic for a header-only packet), the
 *    TX channel, sequence number and check bit; data[0] and the
 *    reserved word (the request slot) are the caller's.
 *  - poll(): EPOLLIN with a packet queued, EPOLLOUT while writes are
 *    accepted (from HS1 on, as in the MediaTek downstream driver for FS and RPC).
 *  - one opener at a time; no ioctls.
 *
 * Packets are kept while nobody has the device open (stock
 * PORT_F_CLOSE_NO_DROP_PKT): the modem asks for its NVRAM right after
 * the start, maybe before the server is up. Past a limit they stay in
 * the ring, which fills up and holds the modem back.
 *
 * The kernel only passes the packets: it never opens or writes a
 * partition or a file for the modem.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/sched/signal.h>
#include <linux/seq_file.h>
#include <linux/skbuff.h>
#include <linux/uaccess.h>

#include "mtk_ccci.h"

/* packets kept per port before the ring is left to fill (stock 16) */
#define MTK_CCCI_CDEV_RX_MAX	16

static bool mtk_ccci_cdev_tx_allowed(struct mtk_ccci *ccci)
{
	switch (READ_ONCE(ccci->state)) {
	case MTK_CCCI_FSM_WAIT_HS1:
	case MTK_CCCI_FSM_HS1:
	case MTK_CCCI_FSM_WAIT_HS2:
	case MTK_CCCI_FSM_READY:
		return READ_ONCE(ccci->hif_on);
	default:
		return false;
	}
}

/**
 * mtk_ccci_cdev_recv() - queue a modem packet for the reader
 * @cd: port
 * @skb: packet with its CCCI header; always taken
 *
 * Return: 0, or -EBUSY when the port is full: the packet is freed and
 * stays in the ring, to be read again once the reader caught up.
 */
int mtk_ccci_cdev_recv(struct mtk_ccci_cdev *cd, struct sk_buff *skb)
{
	if (skb_queue_len(&cd->rx) >= MTK_CCCI_CDEV_RX_MAX) {
		cd->rx_full++;
		kfree_skb(skb);
		return -EBUSY;
	}
	skb_queue_tail(&cd->rx, skb);
	cd->rx_pkts++;
	wake_up_interruptible(&cd->wq);
	return 0;
}

static struct mtk_ccci_cdev *mtk_ccci_cdev_of(struct file *file)
{
	return container_of(file->private_data, struct mtk_ccci_cdev, misc);
}

static int mtk_ccci_cdev_open(struct inode *inode, struct file *file)
{
	struct mtk_ccci_cdev *cd = mtk_ccci_cdev_of(file);

	if (atomic_cmpxchg(&cd->open, 0, 1))
		return -EBUSY;
	return stream_open(inode, file);
}

static int mtk_ccci_cdev_release(struct inode *inode, struct file *file)
{
	struct mtk_ccci_cdev *cd = mtk_ccci_cdev_of(file);

	atomic_set(&cd->open, 0);
	return 0;
}

static ssize_t mtk_ccci_cdev_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct mtk_ccci_cdev *cd = mtk_ccci_cdev_of(file);
	struct sk_buff *skb;
	size_t len;
	int ret;

	for (;;) {
		skb = skb_dequeue(&cd->rx);
		if (skb)
			break;
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(cd->wq,
					       !skb_queue_empty(&cd->rx));
		if (ret)
			return ret;
	}

	len = min_t(size_t, count, skb->len);
	if (copy_to_user(buf, skb->data, len)) {
		skb_queue_head(&cd->rx, skb);
		return -EFAULT;
	}
	if (len < skb->len) {
		/* the rest on the next read */
		skb_pull(skb, len);
		skb_queue_head(&cd->rx, skb);
	} else {
		consume_skb(skb);
		/* packets may wait in the ring for room here */
		mtk_ccci_hif_kick(cd->ccci);
	}
	return len;
}

static ssize_t mtk_ccci_cdev_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct mtk_ccci_cdev *cd = mtk_ccci_cdev_of(file);
	struct mtk_ccci *ccci = cd->ccci;
	struct mtk_ccci_header *hdr;
	struct sk_buff *skb;
	int ret;

	if (count < sizeof(*hdr))
		return -EINVAL;
	/* the stock limit for a port with a user header (+4 on FS) */
	if (count > MTK_CCCI_MTU + sizeof(*hdr) + 4)
		return -EMSGSIZE;
	if (!mtk_ccci_cdev_tx_allowed(ccci))
		return -ENODEV;

	skb = mtk_ccci_alloc_skb(count, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;
	if (copy_from_user(skb_put(skb, count), buf, count)) {
		kfree_skb(skb);
		return -EFAULT;
	}
	hdr = (void *)skb->data;
	if (count == sizeof(*hdr))
		hdr->data[0] = cpu_to_le32(MTK_CCCI_MAGIC);
	else
		hdr->data[1] = cpu_to_le32(count);
	hdr->status = cpu_to_le32(cd->tx_ch);

	for (;;) {
		ret = mtk_ccci_send_skb(ccci, cd->txq, skb);
		if (ret != -EAGAIN || (file->f_flags & O_NONBLOCK))
			break;
		/* the ring is full: the modem drains it */
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}
		usleep_range(1000, 2000);
		if (!mtk_ccci_cdev_tx_allowed(ccci)) {
			ret = -ENODEV;
			break;
		}
	}
	if (ret) {
		kfree_skb(skb);
		return ret;
	}
	cd->tx_pkts++;
	return count;
}

static __poll_t mtk_ccci_cdev_poll(struct file *file, poll_table *wait)
{
	struct mtk_ccci_cdev *cd = mtk_ccci_cdev_of(file);
	__poll_t mask = 0;

	poll_wait(file, &cd->wq, wait);
	if (!skb_queue_empty(&cd->rx))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (mtk_ccci_cdev_tx_allowed(cd->ccci))
		mask |= EPOLLOUT | EPOLLWRNORM;
	return mask;
}

static const struct file_operations mtk_ccci_cdev_fops = {
	.owner = THIS_MODULE,
	.open = mtk_ccci_cdev_open,
	.release = mtk_ccci_cdev_release,
	.read = mtk_ccci_cdev_read,
	.write = mtk_ccci_cdev_write,
	.poll = mtk_ccci_cdev_poll,
	.llseek = noop_llseek,
};

/* the modem stopped: what it asked for is void */
void mtk_ccci_cdev_reset(struct mtk_ccci *ccci)
{
	skb_queue_purge(&ccci->cdev_fs.rx);
	wake_up_interruptible(&ccci->cdev_fs.wq);
}

void mtk_ccci_cdev_debugfs(struct mtk_ccci *ccci, struct seq_file *s)
{
	const struct mtk_ccci_cdev *cd = &ccci->cdev_fs;

	seq_printf(s, "/dev/%s: %s, rx %lu (queued %u, full %lu) tx %lu\n",
		   cd->misc.name, atomic_read(&cd->open) ? "open" : "closed",
		   cd->rx_pkts, skb_queue_len(&cd->rx), cd->rx_full,
		   cd->tx_pkts);
}

static void mtk_ccci_cdev_cleanup(void *data)
{
	struct mtk_ccci_cdev *cd = data;

	misc_deregister(&cd->misc);
	skb_queue_purge(&cd->rx);
}

static int mtk_ccci_cdev_add(struct mtk_ccci *ccci, struct mtk_ccci_cdev *cd,
			     const char *name, u16 tx_ch, u8 txq)
{
	int ret;

	cd->ccci = ccci;
	cd->tx_ch = tx_ch;
	cd->txq = txq;
	skb_queue_head_init(&cd->rx);
	init_waitqueue_head(&cd->wq);
	atomic_set(&cd->open, 0);

	cd->misc.minor = MISC_DYNAMIC_MINOR;
	cd->misc.name = name;
	cd->misc.fops = &mtk_ccci_cdev_fops;
	cd->misc.parent = ccci->dev;
	cd->misc.mode = 0600;
	ret = misc_register(&cd->misc);
	if (ret)
		return dev_err_probe(ccci->dev, ret, "/dev/%s\n", name);
	return devm_add_action_or_reset(ccci->dev, mtk_ccci_cdev_cleanup, cd);
}

int mtk_ccci_cdev_init(struct mtk_ccci *ccci)
{
	return mtk_ccci_cdev_add(ccci, &ccci->cdev_fs, "ccci_fs",
				 MTK_CCCI_CH_FS_TX, MTK_CCCI_Q_FS);
}
