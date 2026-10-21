// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: WWAN network devices (stock ccmni0..20), skeleton.
 *
 * As t7xx: the WWAN core creates the links ("ip link add ... type wwan
 * linkid N"), link id N is the CCMNI index the modem tags packets with
 * (message PIT/DRB channel id). Raw IP, no link layer. Transmit drops
 * until the DPMAIF data path exists.
 *
 * Copyright (C) 2021 Intel Corporation (t7xx ccmni)
 * Copyright (C) 2026 serlk
 */

#include <linux/if_arp.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/wwan.h>

#include "mtk_ccci.h"

#define MTK_CCCI_NET_LINKS	21	/* ccmni0..20 */
#define MTK_CCCI_NET_MTU_MAX	3000

struct mtk_ccci_net {
	struct mtk_ccci *ccci;
	u32 link_id;
};

static int mtk_ccci_net_open(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

static int mtk_ccci_net_stop(struct net_device *dev)
{
	return 0;
}

static netdev_tx_t mtk_ccci_net_xmit(struct sk_buff *skb,
				     struct net_device *dev)
{
	/* TODO: DPMAIF UL (message DRB + payload DRBs) */
	dev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops mtk_ccci_net_ops = {
	.ndo_open = mtk_ccci_net_open,
	.ndo_stop = mtk_ccci_net_stop,
	.ndo_start_xmit = mtk_ccci_net_xmit,
};

static void mtk_ccci_net_setup(struct net_device *dev)
{
	dev->needed_headroom += sizeof(struct mtk_ccci_header);
	dev->mtu = ETH_DATA_LEN;
	dev->max_mtu = MTK_CCCI_NET_MTU_MAX;
	dev->flags = IFF_POINTOPOINT | IFF_NOARP;
	dev->type = ARPHRD_NONE;
	dev->needs_free_netdev = true;
	dev->netdev_ops = &mtk_ccci_net_ops;
}

static int mtk_ccci_net_newlink(void *ctxt, struct net_device *dev,
				u32 if_id, struct netlink_ext_ack *extack)
{
	struct mtk_ccci_net *net = wwan_netdev_drvpriv(dev);

	if (if_id >= MTK_CCCI_NET_LINKS)
		return -EINVAL;
	net->ccci = ctxt;
	net->link_id = if_id;
	return register_netdevice(dev);
}

static void mtk_ccci_net_dellink(void *ctxt, struct net_device *dev,
				 struct list_head *head)
{
	unregister_netdevice_queue(dev, head);
}

static const struct wwan_ops mtk_ccci_wwan_net_ops = {
	.priv_size = sizeof(struct mtk_ccci_net),
	.setup = mtk_ccci_net_setup,
	.newlink = mtk_ccci_net_newlink,
	.dellink = mtk_ccci_net_dellink,
};

static void mtk_ccci_net_cleanup(void *data)
{
	struct mtk_ccci *ccci = data;

	wwan_unregister_ops(ccci->dev);
}

/* only with the DPMAIF skeleton set up (data=1) */
int mtk_ccci_net_init(struct mtk_ccci *ccci)
{
	int ret;

	ret = wwan_register_ops(ccci->dev, &mtk_ccci_wwan_net_ops, ccci,
				WWAN_NO_DEFAULT_LINK);
	if (ret)
		return ret;
	return devm_add_action_or_reset(ccci->dev, mtk_ccci_net_cleanup, ccci);
}
