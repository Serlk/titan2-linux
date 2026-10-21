// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: ring buffer queues in the CCISM shared memory.
 *
 * The AP lays out 16 queues back to back. Each one is
 *
 *	u32 0xee0000ee, 0xee0000ee
 *	u32 rx_read, rx_write, rx_length	(modem -> AP)
 *	u32 tx_read, tx_write, tx_length	(AP -> modem)
 *	u8  rx_buffer[rx_length]
 *	u8  tx_buffer[tx_length]
 *	u32 0xff0000ff, 0xff0000ff
 *
 * and a packet in a buffer is
 *
 *	u32 0xaabbaabb, u32 len, u8 data[len] padded to 8, u32 0xccddeeff x2
 *
 * The writer owns the write index, the reader the read index; offsets
 * wrap at the buffer length and are kept 8-byte aligned. One byte is
 * always left free so that read == write means empty.
 *
 * The queue sizes are the MediaTek downstream driver's for modem generations from
 * 6298 on; the modem only finds the queues through the layout, so they
 * must match what it expects.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/align.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/sizes.h>

#include "mtk_ccci.h"

#define RB_CTL_MAGIC		0xee0000ee
#define RB_END_MAGIC		0xff0000ff
#define PKT_HEAD_MAGIC		0xaabbaabb
#define PKT_TAIL_MAGIC		0xccddeeff

#define RB_CTL_LEN		(8 + 24 + 8)
#define PKT_OVERHEAD		16	/* head magic + length + tail */

/* control words, from the start of the queue + 8 */
#define RB_RX_READ		0x00
#define RB_RX_WRITE		0x04
#define RB_RX_LEN		0x08
#define RB_TX_READ		0x0c
#define RB_TX_WRITE		0x10
#define RB_TX_LEN		0x14

static const u32 mtk_ccci_rx_size[MTK_CCCI_RINGQ_NUM] = {
	80 * SZ_1K, 80 * SZ_1K, 40 * SZ_1K, 80 * SZ_1K,
	20 * SZ_1K, 20 * SZ_1K, 48 * SZ_1K, 0,
	8 * SZ_1K, 16 * SZ_1K,
};

static const u32 mtk_ccci_tx_size[MTK_CCCI_RINGQ_NUM] = {
	128 * SZ_1K, 40 * SZ_1K, 8 * SZ_1K, 40 * SZ_1K,
	20 * SZ_1K, 20 * SZ_1K, 48 * SZ_1K, 0,
	8 * SZ_1K, 16 * SZ_1K,
};

/* the queues the modem switches to on an exception (CCISM_MCU_EXP) */
static const u32 mtk_ccci_exp_size[MTK_CCCI_RINGQ_NUM] = {
	12 * SZ_1K, 32 * SZ_1K, 8 * SZ_1K, 0, 0, 0, 8 * SZ_1K,
};

static int mtk_ccci_rings_layout(struct mtk_ccci *ccci, const char *what,
				 struct mtk_ccci_ring *ring, void __iomem *base,
				 u32 size, const u32 *rx_size,
				 const u32 *tx_size)
{
	u32 off = 0, total = 0;
	unsigned int i;

	for (i = 0; i < MTK_CCCI_RINGQ_NUM; i++)
		total += RB_CTL_LEN + rx_size[i] + tx_size[i];
	if (total > size) {
		dev_err(ccci->dev, "%s %#x too small for the queues (%#x)\n",
			what, size, total);
		return -ENOSPC;
	}

	/* the MediaTek downstream driver clears the whole region */
	memset_io(base, 0, size);
	for (i = 0; i < MTK_CCCI_RINGQ_NUM; i++) {
		struct mtk_ccci_ring *r = &ring[i];
		u32 len = RB_CTL_LEN + rx_size[i] + tx_size[i];
		void __iomem *q = base + off;

		writel(RB_CTL_MAGIC, q);
		writel(RB_CTL_MAGIC, q + 4);
		writel(RB_END_MAGIC, q + len - 8);
		writel(RB_END_MAGIC, q + len - 4);

		r->ctl = q + 8;
		r->rx_len = rx_size[i];
		r->tx_len = tx_size[i];
		r->rx_buf = r->ctl + 24;
		r->tx_buf = r->rx_buf + r->rx_len;
		writel(r->rx_len, r->ctl + RB_RX_LEN);
		writel(r->tx_len, r->ctl + RB_TX_LEN);

		off += len;
	}

	return total;
}

/**
 * mtk_ccci_rings_init() - lay out empty queues
 * @ccci: driver state, ring[] is filled in and ccism_size set to what the
 *	queues use (the MediaTek downstream driver reports that, not the region size, to
 *	the modem in the runtime data)
 * @base: mapping of the CCISM_MCU region
 * @size: its size
 *
 * Return: 0, or -ENOSPC if the region is too small.
 */
int mtk_ccci_rings_init(struct mtk_ccci *ccci, void __iomem *base, u32 size)
{
	int ret;

	ret = mtk_ccci_rings_layout(ccci, "CCISM", ccci->ring, base, size,
				    mtk_ccci_rx_size, mtk_ccci_tx_size);
	if (ret < 0)
		return ret;
	ccci->ccism_size = ret;
	return 0;
}

/**
 * mtk_ccci_rings_exp_init() - lay out the exception queues
 * @ccci: driver state, ring_exp[] is filled in
 * @base: mapping of the CCISM_MCU_EXP region
 * @size: its size
 *
 * The same queue format, with each queue sharing one buffer size both
 * ways. The driver does not switch to them (no exception handshake yet),
 * but the modem is told where they are and expects them laid out.
 *
 * Return: 0, or -ENOSPC if the region is too small.
 */
int mtk_ccci_rings_exp_init(struct mtk_ccci *ccci, void __iomem *base,
			    u32 size)
{
	int ret;

	ret = mtk_ccci_rings_layout(ccci, "CCISM_EXP", ccci->ring_exp, base,
				    size, mtk_ccci_exp_size,
				    mtk_ccci_exp_size);
	return ret < 0 ? ret : 0;
}

static void mtk_ccci_ring_copy_from(void *dst, const void __iomem *buf,
				    u32 len, u32 pos, u32 n)
{
	u32 first = min(n, len - pos);

	memcpy_fromio(dst, buf + pos, first);
	if (n > first)
		memcpy_fromio(dst + first, buf, n - first);
}

static void mtk_ccci_ring_copy_to(void __iomem *buf, u32 len, u32 pos,
				  const void *src, u32 n)
{
	u32 first = min(n, len - pos);

	memcpy_toio(buf + pos, src, first);
	if (n > first)
		memcpy_toio(buf, src + first, n - first);
}

/* the indices come from memory the modem writes: check before use */
static int mtk_ccci_ring_rx_idx(const struct mtk_ccci_ring *r, u32 *rd,
				u32 *wr)
{
	*rd = readl(r->ctl + RB_RX_READ);
	*wr = readl(r->ctl + RB_RX_WRITE);
	if (!r->rx_len || *rd >= r->rx_len || *wr >= r->rx_len ||
	    (*rd | *wr) & 7)
		return -EIO;
	return 0;
}

/**
 * mtk_ccci_ring_readable() - length of the next modem packet
 * @ring: queue
 *
 * Return: the packet length (CCCI header included), -EAGAIN if the queue
 * is empty or the packet not complete, -EIO if the queue is corrupted.
 */
int mtk_ccci_ring_readable(const struct mtk_ccci_ring *ring)
{
	u32 rd, wr, used, len, frame, pos;
	__le32 w[2];

	if (mtk_ccci_ring_rx_idx(ring, &rd, &wr))
		return -EIO;

	used = wr >= rd ? wr - rd : wr + ring->rx_len - rd;
	if (used < PKT_OVERHEAD + sizeof(struct mtk_ccci_header))
		return -EAGAIN;

	mtk_ccci_ring_copy_from(w, ring->rx_buf, ring->rx_len, rd, 8);
	if (le32_to_cpu(w[0]) != PKT_HEAD_MAGIC)
		return -EIO;
	len = le32_to_cpu(w[1]);
	if (len < sizeof(struct mtk_ccci_header) || len > ring->rx_len)
		return -EIO;

	frame = ALIGN(len + PKT_OVERHEAD, 8);
	if (frame > used)
		return -EAGAIN;

	pos = (rd + frame - 8) % ring->rx_len;
	mtk_ccci_ring_copy_from(w, ring->rx_buf, ring->rx_len, pos, 8);
	if (le32_to_cpu(w[0]) != PKT_TAIL_MAGIC ||
	    le32_to_cpu(w[1]) != PKT_TAIL_MAGIC)
		return -EIO;

	return len;
}

/**
 * mtk_ccci_ring_peek() - copy the next modem packet, leave it queued
 * @ring: queue
 * @buf: destination
 * @len: from mtk_ccci_ring_readable()
 *
 * Return: @len, or a negative error.
 */
int mtk_ccci_ring_peek(const struct mtk_ccci_ring *ring, void *buf, u32 len)
{
	u32 rd, wr;

	if (mtk_ccci_ring_rx_idx(ring, &rd, &wr))
		return -EIO;

	mtk_ccci_ring_copy_from(buf, ring->rx_buf, ring->rx_len,
				(rd + 8) % ring->rx_len, len);
	return len;
}

/**
 * mtk_ccci_ring_consume() - drop the next modem packet from the queue
 * @ring: queue
 * @len: from mtk_ccci_ring_readable(), after mtk_ccci_ring_peek()
 */
void mtk_ccci_ring_consume(struct mtk_ccci_ring *ring, u32 len)
{
	u32 rd, wr;

	if (mtk_ccci_ring_rx_idx(ring, &rd, &wr))
		return;

	rd = (rd + ALIGN(len + PKT_OVERHEAD, 8)) % ring->rx_len;
	/* the data is copied out before the modem may reuse the space */
	mb();
	writel(rd, ring->ctl + RB_RX_READ);
}

/**
 * mtk_ccci_ring_read() - take the next modem packet
 * @ring: queue
 * @buf: destination
 * @len: from mtk_ccci_ring_readable()
 *
 * Return: @len, or a negative error.
 */
int mtk_ccci_ring_read(struct mtk_ccci_ring *ring, void *buf, u32 len)
{
	int ret = mtk_ccci_ring_peek(ring, buf, len);

	if (ret >= 0)
		mtk_ccci_ring_consume(ring, len);
	return ret;
}

/**
 * mtk_ccci_ring_write() - queue a packet for the modem
 * @ring: queue
 * @data: CCCI header and payload
 * @len: its length
 *
 * The caller then raises the queue's CCIF channel.
 *
 * Return: @len, -ENOSPC if the queue is full, another negative error.
 */
int mtk_ccci_ring_write(struct mtk_ccci_ring *ring, const void *data,
			u32 len)
{
	static const __le32 tail[2] = {
		cpu_to_le32(PKT_TAIL_MAGIC), cpu_to_le32(PKT_TAIL_MAGIC),
	};
	u32 rd, wr, free, frame, pad;
	__le32 head[2];

	if (!ring->tx_len || !len)
		return -EINVAL;
	rd = readl(ring->ctl + RB_TX_READ);
	wr = readl(ring->ctl + RB_TX_WRITE);
	if (rd >= ring->tx_len || wr >= ring->tx_len || (rd | wr) & 7)
		return -EIO;

	frame = ALIGN(len + PKT_OVERHEAD, 8);
	free = rd > wr ? rd - wr - 1 : ring->tx_len - wr + rd - 1;
	if (frame >= free)
		return -ENOSPC;

	head[0] = cpu_to_le32(PKT_HEAD_MAGIC);
	head[1] = cpu_to_le32(len);
	mtk_ccci_ring_copy_to(ring->tx_buf, ring->tx_len, wr, head, 8);
	mtk_ccci_ring_copy_to(ring->tx_buf, ring->tx_len,
			      (wr + 8) % ring->tx_len, data, len);
	/* the padding is left as it is, as the MediaTek downstream driver does */
	pad = ALIGN(len, 8) - len;
	mtk_ccci_ring_copy_to(ring->tx_buf, ring->tx_len,
			      (wr + 8 + len + pad) % ring->tx_len, tail, 8);

	/* the packet must be complete before the modem sees the index */
	wmb();
	writel((wr + frame) % ring->tx_len, ring->ctl + RB_TX_WRITE);
	return len;
}
