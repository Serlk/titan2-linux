// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: control ports exported as WWAN ports.
 *
 * A port is a pair of logical channels on one CCIF ring queue. The AT
 * port (stock /dev/ttyC0) and the ten MIPC ports (stock /dev/ttyCMIPC*)
 * become WWAN_PORT_AT and WWAN_PORT_MIPC character devices, as the t7xx
 * driver does for its MIPC port. Queue numbers follow the port
 * table of the MediaTek downstream driver (port_cfg.c).
 *
 * Nothing is registered yet: the ports are created only once the FSM
 * reaches READY, which needs the runtime data step that does not exist
 * yet, and only with the module parameter wwan_ports=1.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2021 Intel Corporation (t7xx port layout)
 * Copyright (C) 2026 serlk
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/wwan.h>

#include "mtk_ccci.h"

static bool wwan_ports;
module_param(wwan_ports, bool, 0444);
MODULE_PARM_DESC(wwan_ports, "create the AT/MIPC WWAN ports once the modem is ready");

/* the modem accepts at most this per packet on the control queues */
#define MTK_CCCI_PORT_MTU	3456

/* ring queue of the AT and MIPC channels (vendor DATA_AT_CMD_Q) */
#define MTK_CCCI_Q_AT		5

struct mtk_ccci_port_conf {
	const char *name;
	u16 tx_ch;
	u16 rx_ch;
	u8 txq;
	u8 rxq;
	enum wwan_port_type type;
};

#define MIPC_PORT(n) {						\
	.name = "mipc" #n,					\
	.tx_ch = MTK_CCCI_CH_MIPC0_TX + 2 * (n),		\
	.rx_ch = MTK_CCCI_CH_MIPC0_RX + 2 * (n),		\
	.txq = MTK_CCCI_Q_AT, .rxq = MTK_CCCI_Q_AT,		\
	.type = WWAN_PORT_MIPC,					\
}

static const struct mtk_ccci_port_conf mtk_ccci_port_confs[] = {
	{
		.name = "at",
		.tx_ch = MTK_CCCI_CH_UART2_TX,
		.rx_ch = MTK_CCCI_CH_UART2_RX,
		.txq = MTK_CCCI_Q_AT, .rxq = MTK_CCCI_Q_AT,
		.type = WWAN_PORT_AT,
	},
	MIPC_PORT(0), MIPC_PORT(1), MIPC_PORT(2), MIPC_PORT(3),
	MIPC_PORT(4), MIPC_PORT(5), MIPC_PORT(6), MIPC_PORT(7),
	MIPC_PORT(8), MIPC_PORT(9),
};

static int mtk_ccci_port_send(struct mtk_ccci_port *port, struct sk_buff *skb)
{
	struct mtk_ccci *ccci = port->ccci;
	struct mtk_ccci_ring *ring = &ccci->ring[port->conf->txq];
	struct mtk_ccci_header *hdr;
	int ret;

	if (ccci->state != MTK_CCCI_FSM_READY || !ring->tx_len)
		return -ENODEV;
	if (skb_cow_head(skb, sizeof(*hdr)))
		return -ENOMEM;

	hdr = skb_push(skb, sizeof(*hdr));
	hdr->data[0] = 0;
	hdr->data[1] = cpu_to_le32(skb->len);
	hdr->status = cpu_to_le32(FIELD_PREP(MTK_CCCI_H_CHANNEL,
					     port->conf->tx_ch) |
				  FIELD_PREP(MTK_CCCI_H_SEQ, port->tx_seq) |
				  MTK_CCCI_H_ASSERT);
	hdr->reserved = 0;

	/* TODO: serialise with the other ports of the queue */
	ret = mtk_ccci_ring_write(ring, skb->data, skb->len);
	if (ret < 0) {
		skb_pull(skb, sizeof(*hdr));
		return ret == -ENOSPC ? -EAGAIN : ret;
	}
	port->tx_seq = (port->tx_seq + 1) & FIELD_MAX(MTK_CCCI_H_SEQ);
	mtk_ccci_ccif_notify(ccci, MTK_CCIF_CH_RINGQ(port->conf->txq));
	consume_skb(skb);
	return 0;
}

static int mtk_ccci_wwan_start(struct wwan_port *wport)
{
	return 0;
}

static void mtk_ccci_wwan_stop(struct wwan_port *wport)
{
}

static int mtk_ccci_wwan_tx(struct wwan_port *wport, struct sk_buff *skb)
{
	struct mtk_ccci_port *port = wwan_port_get_drvdata(wport);

	if (skb->len > MTK_CCCI_PORT_MTU - sizeof(struct mtk_ccci_header))
		return -EMSGSIZE;
	return mtk_ccci_port_send(port, skb);
}

static const struct wwan_port_ops mtk_ccci_wwan_ops = {
	.start = mtk_ccci_wwan_start,
	.stop = mtk_ccci_wwan_stop,
	.tx = mtk_ccci_wwan_tx,
};

/**
 * mtk_ccci_port_recv() - hand a modem packet to its port
 * @ccci: driver state
 * @skb: CCCI header and payload; consumed
 *
 * Return: 0, or -ENOENT if no WWAN port takes the channel.
 */
int mtk_ccci_port_recv(struct mtk_ccci *ccci, struct sk_buff *skb)
{
	const struct mtk_ccci_header *hdr = (const void *)skb->data;
	unsigned int i, ch;

	if (skb->len < sizeof(*hdr))
		goto drop;
	ch = FIELD_GET(MTK_CCCI_H_CHANNEL, le32_to_cpu(hdr->status));

	for (i = 0; i < ccci->num_ports; i++) {
		struct mtk_ccci_port *port = &ccci->ports[i];

		if (port->conf->rx_ch != ch)
			continue;
		if (!port->wwan)
			break;
		skb_pull(skb, sizeof(*hdr));
		wwan_port_rx(port->wwan, skb);
		return 0;
	}
drop:
	dev_kfree_skb_any(skb);
	return -ENOENT;
}

/* create the WWAN ports; the FSM calls this on READY */
int mtk_ccci_port_register(struct mtk_ccci *ccci)
{
	struct wwan_port_caps caps = {
		.frag_len = MTK_CCCI_PORT_MTU - sizeof(struct mtk_ccci_header),
		.headroom_len = sizeof(struct mtk_ccci_header),
	};
	unsigned int i;

	if (!wwan_ports || ccci->ports_registered)
		return 0;

	for (i = 0; i < ccci->num_ports; i++) {
		struct mtk_ccci_port *port = &ccci->ports[i];
		struct wwan_port *wport;

		wport = wwan_create_port(ccci->dev, port->conf->type,
					 &mtk_ccci_wwan_ops, &caps, port);
		if (IS_ERR(wport)) {
			dev_err(ccci->dev, "WWAN port %s: %pe\n",
				port->conf->name, wport);
			continue;
		}
		port->wwan = wport;
	}
	ccci->ports_registered = true;
	return 0;
}

void mtk_ccci_port_unregister(struct mtk_ccci *ccci)
{
	unsigned int i;

	for (i = 0; i < ccci->num_ports; i++) {
		if (!ccci->ports[i].wwan)
			continue;
		wwan_remove_port(ccci->ports[i].wwan);
		ccci->ports[i].wwan = NULL;
	}
	ccci->ports_registered = false;
}

static int mtk_ccci_ports_show(struct seq_file *s, void *unused)
{
	struct mtk_ccci *ccci = s->private;
	unsigned int i;

	for (i = 0; i < ccci->num_ports; i++) {
		const struct mtk_ccci_port *port = &ccci->ports[i];

		seq_printf(s, "%-6s tx ch %3u q%u rx ch %3u q%u %s\n",
			   port->conf->name, port->conf->tx_ch,
			   port->conf->txq, port->conf->rx_ch,
			   port->conf->rxq,
			   port->wwan ? "registered" : "-");
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_ccci_ports);

void mtk_ccci_port_debugfs_init(struct mtk_ccci *ccci)
{
	debugfs_create_file("ports", 0400, ccci->debugfs, ccci,
			    &mtk_ccci_ports_fops);
}

static void mtk_ccci_port_cleanup(void *data)
{
	mtk_ccci_port_unregister(data);
}

int mtk_ccci_port_init(struct mtk_ccci *ccci)
{
	unsigned int i, n = ARRAY_SIZE(mtk_ccci_port_confs);

	ccci->ports = devm_kcalloc(ccci->dev, n, sizeof(*ccci->ports),
				   GFP_KERNEL);
	if (!ccci->ports)
		return -ENOMEM;

	for (i = 0; i < n; i++) {
		ccci->ports[i].ccci = ccci;
		ccci->ports[i].conf = &mtk_ccci_port_confs[i];
	}
	ccci->num_ports = n;

	return devm_add_action_or_reset(ccci->dev, mtk_ccci_port_cleanup,
					ccci);
}
