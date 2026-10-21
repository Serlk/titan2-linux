// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: packet queues on top of CCIF and the CCISM ring buffers.
 *
 * TX: the CCCI header gets the per-channel sequence number and the
 * "check the sequence" bit (cleared for the FS and RPC channels outside
 * the HS1..HS2 window, as the MediaTek downstream driver does), the packet goes into
 * the queue's TX ring and the queue's CCIF channel is raised.
 *
 * RX: one ordered worker drains every RX ring, triggered by the CCIF
 * interrupt and, as a safety net and for boards without the interrupt,
 * by a poll timer. A packet is copied out and handed to its channel's
 * consumer; only once the consumer took it is it dropped from the ring.
 * A consumer that is full (a character device nobody reads) leaves the
 * packet in the ring: the modem sees the queue fill up, as with the
 * MediaTek downstream driver.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>

#include "mtk_ccci.h"

#define MTK_CCCI_RX_BUDGET		64
#define MTK_CCCI_POLL_BOOT_MS		20
#define MTK_CCCI_POLL_READY_MS		200

struct sk_buff *mtk_ccci_alloc_skb(unsigned int len, gfp_t gfp)
{
	return alloc_skb(len, gfp);
}

/**
 * mtk_ccci_send_skb() - queue a packet for the modem
 * @ccci: driver state
 * @qno: ring queue
 * @skb: CCCI header (channel set) and payload; consumed on success only
 *
 * Return: 0, -EAGAIN if the queue is full, another negative error.
 */
int mtk_ccci_send_skb(struct mtk_ccci *ccci, unsigned int qno,
		      struct sk_buff *skb)
{
	struct mtk_ccci_header *hdr = (void *)skb->data;
	struct mtk_ccci_ring *ring;
	unsigned long flags;
	u32 status, ch;
	bool assert = true;
	int ret;

	if (qno >= MTK_CCCI_RINGQ_NUM || skb->len < sizeof(*hdr) ||
	    skb->len > MTK_CCCI_MTU + 4)
		return -EINVAL;
	if (!READ_ONCE(ccci->hif_on))
		return -ENODEV;
	ring = &ccci->ring[qno];
	if (!ring->tx_len)
		return -EINVAL;

	ch = FIELD_GET(MTK_CCCI_H_CHANNEL, le32_to_cpu(hdr->status));
	/* the RPC and FS servers are preemptible on the modem after HS2 */
	if ((ch == MTK_CCCI_CH_RPC_TX || ch == MTK_CCCI_CH_FS_TX) &&
	    READ_ONCE(ccci->state) != MTK_CCCI_FSM_WAIT_HS2)
		assert = false;

	spin_lock_irqsave(&ccci->tx_lock[qno], flags);
	status = FIELD_PREP(MTK_CCCI_H_CHANNEL, ch);
	if (ch < MTK_CCCI_CH_NUM) {
		status |= FIELD_PREP(MTK_CCCI_H_SEQ, ccci->tx_seq[ch]);
		if (assert)
			status |= MTK_CCCI_H_ASSERT;
	}
	hdr->status = cpu_to_le32(status);

	ret = mtk_ccci_ring_write(ring, skb->data, skb->len);
	if (ret >= 0) {
		if (ch < MTK_CCCI_CH_NUM)
			ccci->tx_seq[ch] = (ccci->tx_seq[ch] + 1) &
					   FIELD_MAX(MTK_CCCI_H_SEQ);
		ccci->stats.tx[qno]++;
	}
	spin_unlock_irqrestore(&ccci->tx_lock[qno], flags);
	if (ret < 0)
		return ret == -ENOSPC ? -EAGAIN : ret;

	/* busy: the modem has not taken the last one, it drains the ring */
	mtk_ccci_ccif_notify(ccci, MTK_CCIF_CH_RINGQ(qno));
	consume_skb(skb);
	return 0;
}

/**
 * mtk_ccci_send_msg() - send a header-only message
 * @ccci: driver state
 * @qno: ring queue
 * @tx_ch: channel
 * @msg: message id (data[1])
 * @reserved: its argument
 *
 * The format of the vendor port_send_msg_to_md(): data[0] is the magic,
 * data[1] the message, the reserved word its argument.
 *
 * Return: 0 or a negative error.
 */
int mtk_ccci_send_msg(struct mtk_ccci *ccci, unsigned int qno,
		      unsigned int tx_ch, u32 msg, u32 reserved)
{
	struct mtk_ccci_header *hdr;
	struct sk_buff *skb;
	int ret;

	skb = mtk_ccci_alloc_skb(sizeof(*hdr), GFP_KERNEL);
	if (!skb)
		return -ENOMEM;
	hdr = skb_put(skb, sizeof(*hdr));
	hdr->data[0] = cpu_to_le32(MTK_CCCI_MAGIC);
	hdr->data[1] = cpu_to_le32(msg);
	hdr->status = cpu_to_le32(tx_ch);
	hdr->reserved = cpu_to_le32(reserved);

	ret = mtk_ccci_send_skb(ccci, qno, skb);
	if (ret)
		kfree_skb(skb);
	return ret;
}

/* the MediaTek downstream driver force-asserts the modem here; only count it */
static void mtk_ccci_rx_check_seq(struct mtk_ccci *ccci,
				  const struct mtk_ccci_header *hdr)
{
	u32 status = le32_to_cpu(hdr->status);
	u32 ch = FIELD_GET(MTK_CCCI_H_CHANNEL, status);
	u16 seq = FIELD_GET(MTK_CCCI_H_SEQ, status);

	if (ch >= MTK_CCCI_CH_NUM)
		return;
	if ((status & MTK_CCCI_H_ASSERT) && ccci->rx_seq[ch] &&
	    ((seq - ccci->rx_seq[ch]) & FIELD_MAX(MTK_CCCI_H_SEQ)) != 1) {
		ccci->stats.seq_err++;
		dev_warn_ratelimited(ccci->dev,
				     "channel %u sequence %u after %u\n", ch,
				     seq, ccci->rx_seq[ch]);
	}
	ccci->rx_seq[ch] = seq;
}

/*
 * Hand a packet to its consumer.
 * Return: 0 if taken (or dropped), -EBUSY to leave it in the ring.
 */
static int mtk_ccci_rx_dispatch(struct mtk_ccci *ccci, struct sk_buff *skb)
{
	const struct mtk_ccci_header *hdr = (const void *)skb->data;
	u32 ch = FIELD_GET(MTK_CCCI_H_CHANNEL, le32_to_cpu(hdr->status));

	switch (ch) {
	case MTK_CCCI_CH_CONTROL_RX:
		mtk_ccci_ctl_recv(ccci, skb);
		return 0;
	case MTK_CCCI_CH_SYSTEM_RX:
		mtk_ccci_sys_recv(ccci, skb);
		return 0;
	case MTK_CCCI_CH_STATUS_RX:
		mtk_ccci_status_recv(ccci, skb);
		return 0;
	case MTK_CCCI_CH_FS_RX:
	case MTK_CCCI_CH_RPC_RX:
		ccci->stats.fs_rpc++;
		break;
	}

	if (!mtk_ccci_port_recv(ccci, skb))
		return 0;

	/* mtk_ccci_port_recv() consumed it */
	ccci->stats.dropped++;
	ccci->stats.last_drop_ch = ch;
	dev_dbg_ratelimited(ccci->dev, "no consumer for channel %u\n", ch);
	return 0;
}

/* drain one RX ring; true if there is more to do */
static bool mtk_ccci_rx_queue(struct mtk_ccci *ccci, unsigned int qno)
{
	struct mtk_ccci_ring *ring = &ccci->ring[qno];
	struct mtk_ccci_header hdr;
	unsigned int budget = MTK_CCCI_RX_BUDGET;
	struct sk_buff *skb;
	int len, ret;

	while (budget--) {
		len = mtk_ccci_ring_readable(ring);
		if (len == -EAGAIN)
			return false;
		if (len < 0) {
			/* indices or magics the modem wrote make no sense */
			ccci->stats.rx_err[qno]++;
			dev_err_ratelimited(ccci->dev,
					    "RX queue %u corrupted (%d)\n",
					    qno, len);
			return false;
		}

		skb = mtk_ccci_alloc_skb(len, GFP_KERNEL);
		if (!skb)
			return true;
		ret = mtk_ccci_ring_peek(ring, skb_put(skb, len), len);
		if (ret < 0) {
			kfree_skb(skb);
			ccci->stats.rx_err[qno]++;
			return false;
		}
		memcpy(&hdr, skb->data, sizeof(hdr));

		ret = mtk_ccci_rx_dispatch(ccci, skb);
		if (ret == -EBUSY) {
			/* the consumer did not take it; retried on its read */
			ccci->rxq_blocked |= BIT(qno);
			return false;
		}
		mtk_ccci_rx_check_seq(ccci, &hdr);
		mtk_ccci_ring_consume(ring, len);
		ccci->stats.rx[qno]++;
	}

	return true;
}

static void mtk_ccci_rx_work(struct work_struct *work)
{
	struct mtk_ccci *ccci = container_of(work, struct mtk_ccci, rx_work);
	bool again = false;
	unsigned int qno;
	u32 pending, rch;

	if (!READ_ONCE(ccci->hif_on))
		return;

	pending = atomic_xchg(&ccci->ccif_pending, 0);
	rch = mtk_ccci_ccif_pending(ccci);
	if (rch) {
		mtk_ccci_ccif_ack(ccci, rch);
		pending |= rch;
	}

	if (pending & MTK_CCIF_EX_MASK)
		mtk_ccci_fsm_exception(ccci, pending & MTK_CCIF_EX_MASK);

	if (pending & BIT(MTK_CCIF_CH_SRAM)) {
		/* the boot handshake: the FSM reads the SRAM itself */
		WRITE_ONCE(ccci->hs1_pending, true);
		wake_up_all(&ccci->hs_wq);
	}

	/* all queues: cheap, and nothing is lost to a missed interrupt */
	ccci->rxq_blocked = 0;
	for (qno = 0; qno < MTK_CCCI_RINGQ_NUM; qno++)
		if (ccci->ring[qno].rx_len)
			again |= mtk_ccci_rx_queue(ccci, qno);

	if (again)
		queue_work(ccci->wq, &ccci->rx_work);
}

static void mtk_ccci_poll_work(struct work_struct *work)
{
	struct mtk_ccci *ccci = container_of(to_delayed_work(work),
					     struct mtk_ccci, poll_work);
	enum mtk_ccci_fsm_state state = READ_ONCE(ccci->state);
	unsigned int ms;

	if (!READ_ONCE(ccci->hif_on))
		return;
	ccci->stats.polls++;
	queue_work(ccci->wq, &ccci->rx_work);

	ms = state == MTK_CCCI_FSM_READY ? MTK_CCCI_POLL_READY_MS :
					   MTK_CCCI_POLL_BOOT_MS;
	queue_delayed_work(ccci->wq, &ccci->poll_work, msecs_to_jiffies(ms));
}

/* from interrupt context too */
void mtk_ccci_hif_kick(struct mtk_ccci *ccci)
{
	if (READ_ONCE(ccci->hif_on))
		queue_work(ccci->wq, &ccci->rx_work);
}

/**
 * mtk_ccci_hif_start() - let packets flow, once the rings are laid out
 * @ccci: driver state
 */
void mtk_ccci_hif_start(struct mtk_ccci *ccci)
{
	memset(ccci->tx_seq, 0, sizeof(ccci->tx_seq));
	memset(ccci->rx_seq, 0, sizeof(ccci->rx_seq));
	memset(&ccci->stats, 0, sizeof(ccci->stats));
	atomic_set(&ccci->ccif_pending, 0);
	ccci->rxq_blocked = 0;
	WRITE_ONCE(ccci->hs1_pending, false);

	WRITE_ONCE(ccci->hif_on, true);
	queue_delayed_work(ccci->wq, &ccci->poll_work,
			   msecs_to_jiffies(MTK_CCCI_POLL_BOOT_MS));
}

/**
 * mtk_ccci_hif_stop() - stop the packet flow and drop what is queued
 * @ccci: driver state
 *
 * Interrupts must be released already.
 */
void mtk_ccci_hif_stop(struct mtk_ccci *ccci)
{
	WRITE_ONCE(ccci->hif_on, false);
	cancel_delayed_work_sync(&ccci->poll_work);
	cancel_work_sync(&ccci->rx_work);
}

static int mtk_ccci_queues_show(struct seq_file *s, void *unused)
{
	struct mtk_ccci *ccci = s->private;
	const struct mtk_ccci_stats *st = &ccci->stats;
	unsigned int i;

	seq_printf(s, "hif %s, irqs %lu, polls %lu, seq errors %lu, dropped %lu (last ch %u), fs+rpc %lu\n",
		   ccci->hif_on ? "on" : "off", st->irqs, st->polls,
		   st->seq_err, st->dropped, st->last_drop_ch, st->fs_rpc);
	for (i = 0; i < MTK_CCCI_RINGQ_NUM; i++) {
		const struct mtk_ccci_ring *r = &ccci->ring[i];

		if (!r->rx_len && !r->tx_len)
			continue;
		seq_printf(s, "q%-2u rx %6lu (err %lu) tx %6lu", i, st->rx[i],
			   st->rx_err[i], st->tx[i]);
		if (ccci->hif_on)
			seq_printf(s, "  rx r/w %#x/%#x tx r/w %#x/%#x",
				   readl(r->ctl), readl(r->ctl + 4),
				   readl(r->ctl + 12), readl(r->ctl + 16));
		seq_puts(s, "\n");
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_ccci_queues);

void mtk_ccci_hif_debugfs_init(struct mtk_ccci *ccci)
{
	debugfs_create_file("queues", 0400, ccci->debugfs, ccci,
			    &mtk_ccci_queues_fops);
}

static void mtk_ccci_hif_cleanup(void *data)
{
	struct mtk_ccci *ccci = data;

	WRITE_ONCE(ccci->hif_on, false);
	cancel_delayed_work_sync(&ccci->poll_work);
	cancel_work_sync(&ccci->rx_work);
	destroy_workqueue(ccci->wq);
}

int mtk_ccci_hif_init(struct mtk_ccci *ccci)
{
	unsigned int i;

	for (i = 0; i < MTK_CCCI_RINGQ_NUM; i++)
		spin_lock_init(&ccci->tx_lock[i]);
	spin_lock_init(&ccci->ccif_lock);
	init_waitqueue_head(&ccci->hs_wq);
	INIT_WORK(&ccci->rx_work, mtk_ccci_rx_work);
	INIT_DELAYED_WORK(&ccci->poll_work, mtk_ccci_poll_work);

	ccci->wq = alloc_ordered_workqueue("mtk_ccci",
					   WQ_MEM_RECLAIM | WQ_HIGHPRI);
	if (!ccci->wq)
		return -ENOMEM;
	return devm_add_action_or_reset(ccci->dev, mtk_ccci_hif_cleanup,
					ccci);
}
