// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: the kernel side of the modem's remote procedure calls.
 *
 * Between the runtime data and HS2 the modem asks the AP for board
 * facts over the RPC channels (queue 1): which pins its SIM and band
 * detection lines are on, and attributes LK put into the device tree.
 * A request is
 *
 *	CCCI header, u32 op, u32 count, count x { u32 len, data, pad to 4 }
 *
 * and the answer has the same layout with op | 0xffff0000 and the slot
 * (header reserved word) kept. Handled here, as the stock in-kernel RPC
 * port does:
 *
 *  - 0x4002 GET_GPIO_NUM: modem pin number for a pin name, from the
 *    board's "mediatek,md-gpio-names"/"mediatek,md-gpio-pins";
 *  - 0x400E DTSI_QUERY: a u32 property of the "mediatek,md_attr_node"
 *    node LK creates.
 *
 * Every other operation is answered "no such operation", the stock
 * answer to an unknown one; that includes the four the stock system
 * leaves to its userspace server.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/device.h>
#include <linux/of.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "mtk_ccci.h"

#define RPC_GET_GPIO_NUM	0x4002
#define RPC_DTSI_QUERY		0x400e
#define RPC_RESP		0xffff0000

#define RPC_MAX_ARGS		6
#define RPC_MAX_BUF		2048	/* request, header included */
#define RPC_SLOTS		2	/* concurrent requests */
#define RPC_HDR_LEN		(sizeof(struct mtk_ccci_header) + 8)

/* return codes of the modem's FS/RPC API */
#define RPC_NO_OP		-1
#define RPC_PARAM_ERROR		-2
#define RPC_FUNC_FAIL		-5

/* DTSI_QUERY */
#define RPC_REQ_PROP_VALUE	1
#define RPC_DTSI_NAME_LEN	64

struct mtk_ccci_rpc_dtsi_in {
	u8 req;
	u8 index;
	char name[RPC_DTSI_NAME_LEN];
} __packed;

struct mtk_ccci_rpc_dtsi_out {
	__le32 value;
	char str[RPC_DTSI_NAME_LEN];
} __packed;

struct mtk_ccci_rpc_arg {
	u32 len;
	const void *buf;
};

struct mtk_ccci_rpc_reply {
	unsigned int num;
	struct {
		u32 len;
		u8 data[sizeof(struct mtk_ccci_rpc_dtsi_out)];
	} arg[2];
};

static void rpc_reply_u32(struct mtk_ccci_rpc_reply *r, s32 v)
{
	r->arg[r->num].len = 4;
	put_unaligned_le32(v, r->arg[r->num].data);
	r->num++;
}

static int mtk_ccci_rpc_gpio_pin(struct mtk_ccci *ccci, const char *name,
				 u32 len, u32 *pin)
{
	struct device_node *np = ccci->dev->of_node;
	const char *dt_name;
	int i, n;

	len = strnlen(name, len);
	n = of_property_count_strings(np, "mediatek,md-gpio-names");
	for (i = 0; i < n; i++) {
		if (of_property_read_string_index(np, "mediatek,md-gpio-names",
						  i, &dt_name))
			break;
		if (strlen(dt_name) != len || memcmp(dt_name, name, len))
			continue;
		return of_property_read_u32_index(np, "mediatek,md-gpio-pins",
						  i, pin);
	}
	return -ENOENT;
}

static void mtk_ccci_rpc_gpio_num(struct mtk_ccci *ccci,
				  const struct mtk_ccci_rpc_arg *arg,
				  unsigned int num,
				  struct mtk_ccci_rpc_reply *r)
{
	s32 val;
	u32 pin;

	if (num < 2 || !arg[0].len) {
		val = RPC_PARAM_ERROR;
	} else if (mtk_ccci_rpc_gpio_pin(ccci, arg[0].buf, arg[0].len, &pin)) {
		dev_info(ccci->dev, "RPC: no modem pin for %.*s\n",
			 (int)strnlen(arg[0].buf, arg[0].len), (char *)arg[0].buf);
		val = RPC_FUNC_FAIL;
	} else {
		dev_info(ccci->dev, "RPC: %.*s is modem pin %u\n",
			 (int)strnlen(arg[0].buf, arg[0].len),
			 (char *)arg[0].buf, pin);
		val = pin;
	}
	/* the stock answer carries the result twice */
	rpc_reply_u32(r, val);
	rpc_reply_u32(r, val);
}

static void mtk_ccci_rpc_dtsi(struct mtk_ccci *ccci,
			      const struct mtk_ccci_rpc_arg *arg,
			      unsigned int num, struct mtk_ccci_rpc_reply *r)
{
	struct mtk_ccci_rpc_dtsi_out out;
	struct mtk_ccci_rpc_dtsi_in in;
	struct device_node *np;
	char name[RPC_DTSI_NAME_LEN + 1];
	u32 val;

	if (num != 1 || arg[0].len < sizeof(in)) {
		rpc_reply_u32(r, RPC_PARAM_ERROR);
		rpc_reply_u32(r, RPC_PARAM_ERROR);
		return;
	}
	memcpy(&in, arg[0].buf, sizeof(in));
	memcpy(name, in.name, sizeof(in.name));
	name[RPC_DTSI_NAME_LEN] = '\0';

	/* "failure" is a buffer of 0x0f bytes, as in the MediaTek downstream driver */
	memset(&out, 0xf, sizeof(out));
	np = of_find_compatible_node(NULL, NULL, "mediatek,md_attr_node");
	if (np && in.req == RPC_REQ_PROP_VALUE &&
	    !of_property_read_u32(np, name, &val))
		out.value = cpu_to_le32(val);
	of_node_put(np);
	dev_info(ccci->dev, "RPC: %s = %#x%s\n", name, le32_to_cpu(out.value),
		 np ? "" : " (no md_attr_node)");

	rpc_reply_u32(r, 0);
	r->arg[1].len = sizeof(out);
	memcpy(r->arg[1].data, &out, sizeof(out));
	r->num = 2;
}

static int mtk_ccci_rpc_send(struct mtk_ccci *ccci,
			     const struct mtk_ccci_header *req_hdr, u32 op,
			     const struct mtk_ccci_rpc_reply *r)
{
	struct mtk_ccci_header *hdr;
	struct sk_buff *skb;
	unsigned int i, len = RPC_HDR_LEN;
	u8 *p;
	int ret;

	for (i = 0; i < r->num; i++)
		len += 4 + ALIGN(r->arg[i].len, 4);
	skb = mtk_ccci_alloc_skb(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;
	p = skb_put_zero(skb, len);

	hdr = (void *)p;
	hdr->data[0] = req_hdr->data[0];
	hdr->data[1] = cpu_to_le32(len);
	hdr->status = cpu_to_le32(MTK_CCCI_CH_RPC_TX);
	hdr->reserved = req_hdr->reserved;
	put_unaligned_le32(op | RPC_RESP, p + sizeof(*hdr));
	put_unaligned_le32(r->num, p + sizeof(*hdr) + 4);
	p += RPC_HDR_LEN;
	for (i = 0; i < r->num; i++) {
		put_unaligned_le32(r->arg[i].len, p);
		memcpy(p + 4, r->arg[i].data, r->arg[i].len);
		p += 4 + ALIGN(r->arg[i].len, 4);
	}

	ret = mtk_ccci_send_skb(ccci, MTK_CCCI_Q_RPC, skb);
	if (ret)
		kfree_skb(skb);
	return ret;
}

/**
 * mtk_ccci_rpc_recv() - answer a modem RPC request
 * @ccci: driver state
 * @skb: request with its CCCI header; always taken
 *
 * Malformed requests are dropped without an answer, as in the MediaTek downstream
 * driver.
 *
 * Return: 0.
 */
int mtk_ccci_rpc_recv(struct mtk_ccci *ccci, struct sk_buff *skb)
{
	struct mtk_ccci_rpc_arg arg[RPC_MAX_ARGS];
	struct mtk_ccci_rpc_reply *r;
	struct mtk_ccci_header hdr;
	u32 op, num, off, i;
	int ret;

	if (skb->len < RPC_HDR_LEN || skb->len > RPC_MAX_BUF)
		goto bad;
	memcpy(&hdr, skb->data, sizeof(hdr));
	op = get_unaligned_le32(skb->data + sizeof(hdr));
	num = get_unaligned_le32(skb->data + sizeof(hdr) + 4);
	if (le32_to_cpu(hdr.reserved) > RPC_SLOTS || num > RPC_MAX_ARGS)
		goto bad;

	off = RPC_HDR_LEN;
	for (i = 0; i < num; i++) {
		if (off + 4 > skb->len)
			goto bad;
		arg[i].len = get_unaligned_le32(skb->data + off);
		off += 4;
		if (arg[i].len > skb->len - off)
			goto bad;
		arg[i].buf = skb->data + off;
		off += ALIGN(arg[i].len, 4);
	}

	r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (!r)
		goto out;
	switch (op) {
	case RPC_GET_GPIO_NUM:
		mtk_ccci_rpc_gpio_num(ccci, arg, num, r);
		break;
	case RPC_DTSI_QUERY:
		mtk_ccci_rpc_dtsi(ccci, arg, num, r);
		break;
	default:
		dev_info(ccci->dev, "RPC %#x (%u args) not handled\n", op, num);
		rpc_reply_u32(r, RPC_NO_OP);
		break;
	}
	ret = mtk_ccci_rpc_send(ccci, &hdr, op, r);
	if (ret)
		dev_err(ccci->dev, "RPC %#x answer: %d\n", op, ret);
	kfree(r);
	goto out;

bad:
	dev_err_ratelimited(ccci->dev, "malformed RPC request (%u bytes)\n",
			    skb->len);
out:
	consume_skb(skb);
	return 0;
}
