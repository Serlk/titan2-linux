// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: WWAN network devices (downstream ccmni0..20).
 *
 * As t7xx: the WWAN core creates the links ("ip link add wwan0-N link
 * wwan0 type wwan linkid N"), link id N is the CCMNI index the modem tags
 * packets with (message PIT on receive, message DRB on transmit). Raw IP,
 * no link layer: the IP version is the first nibble of the packet. The
 * packets go through DPMAIF (mtk_ccci_dpmaif.c); the links are kept by
 * CCMNI index for its receive path.
 *
 * Copyright (C) 2021 Intel Corporation (t7xx ccmni)
 * Copyright (C) 2026 serlk
 */

#include <linux/if_arp.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include <linux/skbuff.h>
#include <linux/wwan.h>

#include "mtk_ccci.h"

/* what the runtime data tells the modem (AP_CCMNI_MTU) */
#define MTK_CCCI_NET_MTU_MAX	0xdf0

struct mtk_ccci_net {
	struct mtk_ccci *ccci;
	u32 link_id;
};

static int mtk_ccci_net_open(struct net_device *dev)
{
	netif_start_queue(dev);
	return 0;
}

static int mtk_ccci_net_stop(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

static netdev_tx_t mtk_ccci_net_xmit(struct sk_buff *skb,
				     struct net_device *dev)
{
	struct mtk_ccci_net *net = wwan_netdev_drvpriv(dev);
	u8 ver = skb->len ? skb->data[0] >> 4 : 0;

	if (ver != 4 && ver != 6) {
		dev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	return mtk_ccci_dpmaif_xmit(net->ccci, dev, skb, net->link_id);
}

static const struct net_device_ops mtk_ccci_net_ops = {
	.ndo_open = mtk_ccci_net_open,
	.ndo_stop = mtk_ccci_net_stop,
	.ndo_start_xmit = mtk_ccci_net_xmit,
};

static void mtk_ccci_net_setup(struct net_device *dev)
{
	dev->mtu = ETH_DATA_LEN;
	dev->min_mtu = ETH_MIN_MTU;
	dev->max_mtu = MTK_CCCI_NET_MTU_MAX;
	dev->hard_header_len = 0;
	dev->addr_len = 0;
	dev->tx_queue_len = 1000;
	dev->flags = IFF_POINTOPOINT | IFF_NOARP;
	dev->type = ARPHRD_NONE;
	/* the hardware checks the received IP/TCP/UDP checksums */
	dev->features = NETIF_F_RXCSUM;
	dev->hw_features = NETIF_F_RXCSUM;
	dev->pcpu_stat_type = NETDEV_PCPU_STAT_TSTATS;
	dev->needs_free_netdev = true;
	dev->netdev_ops = &mtk_ccci_net_ops;
}

static int mtk_ccci_net_newlink(void *ctxt, struct net_device *dev,
				u32 if_id, struct netlink_ext_ack *extack)
{
	struct mtk_ccci_net *net = wwan_netdev_drvpriv(dev);
	struct mtk_ccci *ccci = ctxt;
	int ret;

	if (if_id >= MTK_CCCI_NET_LINKS) {
		NL_SET_ERR_MSG(extack, "link id is the CCMNI index, 0..20");
		return -EINVAL;
	}
	if (rtnl_dereference(ccci->net_link[if_id]))
		return -EBUSY;
	net->ccci = ccci;
	net->link_id = if_id;
	ret = register_netdevice(dev);
	if (ret)
		return ret;
	rcu_assign_pointer(ccci->net_link[if_id], dev);
	return 0;
}

/* unregistering waits for an RCU grace period before the device goes */
static void mtk_ccci_net_dellink(void *ctxt, struct net_device *dev,
				 struct list_head *head)
{
	struct mtk_ccci_net *net = wwan_netdev_drvpriv(dev);
	struct mtk_ccci *ccci = ctxt;

	RCU_INIT_POINTER(ccci->net_link[net->link_id], NULL);
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

/* only with DPMAIF set up (data=1) */
int mtk_ccci_net_init(struct mtk_ccci *ccci)
{
	int ret;

	ret = wwan_register_ops(ccci->dev, &mtk_ccci_wwan_net_ops, ccci,
				WWAN_NO_DEFAULT_LINK);
	if (ret)
		return ret;
	return devm_add_action_or_reset(ccci->dev, mtk_ccci_net_cleanup, ccci);
}
