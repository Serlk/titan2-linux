// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: the kernel side of the modem's remote procedure calls.
 *
 * Between the runtime data and HS2 the modem asks the AP for board
 * facts over the RPC channels (queue 1): which pins its SIM and band
 * detection lines are on, their levels, a board ADC reading, and
 * attributes LK put into the device tree.
 * A request is
 *
 *	CCCI header, u32 op, u32 count, count x { u32 len, data, pad to 4 }
 *
 * and the answer has the same layout with op | 0xffff0000 and the slot
 * (header reserved word) kept. Handled here, as the stock in-kernel RPC
 * port does:
 *
 *  - 0x4002 GET_GPIO_NUM: modem pin number for a pin name, from the
 *    board's "mediatek,md-gpio-names"/"mediatek,md-gpios";
 *  - 0x4005 GET_EINT_ATTR: one attribute of a SIM hot plug interrupt,
 *    from the board's "mediatek,md-sim-eints";
 *  - 0x4009 GET_GPIO_ADC: pin numbers and levels for up to 3 (10 in the
 *    second version) pin names, and the board ADC ("mediatek,md-adc");
 *  - 0x400E DTSI_QUERY: a u32 property of the "mediatek,md_attr_node"
 *    node LK creates;
 *  - 0x400D CCCI_LHIF_MAPPING: once the modem is up, and again when it
 *    activates a PDN, it tells which DPMAIF UL queue a CCMNI sends on
 *    (its "logical HIF" queue); the downstream driver records the queue and
 *    answers 0, and the modem repeats the request until it does;
 *  - 0x4016 RF_ECID_DATA: the RF chips' ECIDs; the downstream driver only
 *    acknowledges them.
 *
 * The four operations the kernel does not answer itself
 * (system property, SAR table, DRDI copy, crystal calibration) go to
 * /dev/ccci_rpc unchanged, the answer comes back the same way. Every
 * other operation is answered "no such operation", the downstream answer to
 * an unknown one.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/of.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "mtk_ccci.h"

#define RPC_GET_GPIO_NUM	0x4002
#define RPC_GET_EINT_ATTR	0x4005
#define RPC_GET_GPIO_ADC	0x4009
#define RPC_LHIF_MAPPING	0x400d
#define RPC_DTSI_QUERY		0x400e
#define RPC_RF_ECID_DATA	0x4016
/* served by userspace */
#define RPC_QUERY_AP_SYS_PROP	0x400f
#define RPC_SAR_TABLE_IDX	0x4010
#define RPC_AMMS_DRDI_CONTROL	0x4014
#define RPC_SAVE_MD_CAPID	0x4015
#define RPC_RESP		0xffff0000

#define RPC_MAX_ARGS		6
#define RPC_MAX_BUF		2048	/* request, header included */
#define RPC_SLOTS		2	/* concurrent requests */
#define RPC_HDR_LEN		(sizeof(struct mtk_ccci_header) + 8)

/* return codes of the modem's FS/RPC API */
#define RPC_NO_OP		-1
#define RPC_PARAM_ERROR		-2
#define RPC_FUNC_FAIL		-5

/* GET_EINT_ATTR: attributes per SIM, errors */
#define RPC_EINT_ATTRS		7	/* eint, debounce, polarity, ... */
#define RPC_EINT_SIMS		4
#define RPC_EINT_BAD_TYPE	-12
#define RPC_EINT_BAD_NAME	-11

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

/* GET_GPIO_ADC: what to look up (reqMask) */
#define RPC_REQ_GPIO_PIN	BIT(0)
#define RPC_REQ_GPIO_VALUE	BIT(1)
#define RPC_REQ_ADC_PIN		BIT(4)
#define RPC_REQ_ADC_VALUE	BIT(5)

#define RPC_GPIO_NAME_LEN	34
#define RPC_ADC_NAME_LEN	33
#define RPC_GPIO_ADC_PINS_V1	3
#define RPC_GPIO_ADC_PINS_V2	10

/*
 * The two versions differ in the number of pins and the width of the
 * two masks only; the modem tells them apart by size.
 */
#define DEFINE_RPC_GPIO_ADC(_v, _mask_t, _pins)				\
struct mtk_ccci_rpc_gpio_adc_in_##_v {					\
	_mask_t req;							\
	_mask_t valid_pins;						\
	char pin_name[_pins][RPC_GPIO_NAME_LEN];			\
	__le32 pin[_pins];						\
	char adc_name[RPC_ADC_NAME_LEN];				\
	__le32 adc_ch;							\
	__le32 adc_count;						\
} __packed;								\
struct mtk_ccci_rpc_gpio_adc_out_##_v {					\
	__le32 pin[_pins];						\
	__le32 level[_pins];						\
	__le32 adc_ch;							\
	__le32 adc_sum;							\
} __packed

DEFINE_RPC_GPIO_ADC(v1, u8, RPC_GPIO_ADC_PINS_V1);
DEFINE_RPC_GPIO_ADC(v2, __le16, RPC_GPIO_ADC_PINS_V2);
static_assert(sizeof(struct mtk_ccci_rpc_gpio_adc_in_v1) == 157);
static_assert(sizeof(struct mtk_ccci_rpc_gpio_adc_in_v2) == 425);
static_assert(sizeof(struct mtk_ccci_rpc_gpio_adc_out_v2) == 88);

#define RPC_REPLY_MAX		sizeof(struct mtk_ccci_rpc_gpio_adc_out_v2)
static_assert(RPC_REPLY_MAX >= sizeof(struct mtk_ccci_rpc_dtsi_out));

struct mtk_ccci_rpc_arg {
	u32 len;
	const void *buf;
};

/* CCCI_LHIF_MAPPING: the modem's logical HIF queues */
enum {
	RPC_LHIF_HWQ_SW_DL,
	RPC_LHIF_HWQ_SW_UL,
	RPC_LHIF_HWQ_AP_UL_Q0,		/* DPMAIF UL queue 0: data */
	RPC_LHIF_HWQ_AP_UL_Q1,		/* DPMAIF UL queue 3: IMS */
	RPC_LHIF_HWQ_DIRECT_DL,
};

struct mtk_ccci_rpc_lhif_map {
	__le32 net_if;			/* CCMNI index */
	__le32 lhif_q;
} __packed;

struct mtk_ccci_rpc_reply {
	unsigned int num;
	struct {
		u32 len;
		u8 data[RPC_REPLY_MAX];
	} arg[2];
};

static void rpc_reply_u32(struct mtk_ccci_rpc_reply *r, s32 v)
{
	r->arg[r->num].len = 4;
	put_unaligned_le32(v, r->arg[r->num].data);
	r->num++;
}

/*
 * The pin behind a modem pin name: "mediatek,md-gpios" holds a specifier
 * per name in "mediatek,md-gpio-names". The modem numbers the pins as
 * the GPIO controller does.
 */
static int mtk_ccci_rpc_gpio_spec(struct mtk_ccci *ccci, const char *name,
				  u32 len, struct of_phandle_args *spec)
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
		return of_parse_phandle_with_args(np, "mediatek,md-gpios",
						  "#gpio-cells", i, spec);
	}
	return -ENOENT;
}

static int mtk_ccci_rpc_gpio_pin(struct mtk_ccci *ccci, const char *name,
				 u32 len, u32 *pin)
{
	struct of_phandle_args spec;
	int ret;

	ret = mtk_ccci_rpc_gpio_spec(ccci, name, len, &spec);
	if (ret)
		return ret;
	of_node_put(spec.np);
	if (spec.args_count < 1)
		return -EINVAL;
	*pin = spec.args[0];
	return 0;
}

/*
 * The level of a pin. The pin is read, not requested: requesting it
 * would switch it to its GPIO function, and the SIM lines among these
 * must stay with the modem. Reading the input level does not touch the
 * pin setup.
 */
static int mtk_ccci_rpc_gpio_level(struct mtk_ccci *ccci, u32 pin)
{
	struct device_node *np = ccci->dev->of_node;
	struct of_phandle_args spec;
	struct gpio_device *gdev;
	struct gpio_desc *desc;
	int ret;

	/* all md-gpios are on one controller: take the first one's */
	ret = of_parse_phandle_with_args(np, "mediatek,md-gpios",
					 "#gpio-cells", 0, &spec);
	if (ret)
		return ret;
	gdev = gpio_device_find_by_fwnode(of_fwnode_handle(spec.np));
	of_node_put(spec.np);
	if (!gdev)
		return -EPROBE_DEFER;
	desc = gpio_device_get_desc(gdev, pin);
	ret = IS_ERR(desc) ? PTR_ERR(desc) :
			     gpiod_get_raw_value_cansleep(desc);
	gpio_device_put(gdev);
	return ret;
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

/*
 * The SIM a hot plug interrupt name stands for: the modem asks for
 * "MD1_SIM<n>_HOT_PLUG_EINT" (the factory device tree node is named with
 * dashes, which the downstream driver accepts too). Returns 0-based, or -1.
 */
static int mtk_ccci_rpc_eint_sim(const char *name, u32 len)
{
	char want[24];
	int sim;

	len = strnlen(name, len);
	for (sim = 0; sim < RPC_EINT_SIMS; sim++) {
		snprintf(want, sizeof(want), "MD1_SIM%d_HOT_PLUG_EINT", sim + 1);
		if (len == strlen(want) && !memcmp(name, want, len))
			return sim;
		strreplace(want, '_', '-');
		if (len == strlen(want) && !memcmp(name, want, len))
			return sim;
	}
	return -1;
}

/*
 * "mediatek,md-sim-eints" has one row per SIM of RPC_EINT_ATTRS values
 * in the modem's order and units: interrupt number, debounce (ms),
 * polarity (1 high), sensitivity (1 edge), socket type, dedicated, source
 * pin. A negative value means "not set".
 */
static void mtk_ccci_rpc_eint_attr(struct mtk_ccci *ccci,
				   const struct mtk_ccci_rpc_arg *arg,
				   unsigned int num,
				   struct mtk_ccci_rpc_reply *r)
{
	u32 type, val;
	s32 ret;
	int sim;

	if (num < 3 || !arg[0].len || arg[2].len < 4) {
		rpc_reply_u32(r, RPC_PARAM_ERROR);
		rpc_reply_u32(r, RPC_PARAM_ERROR);
		return;
	}
	type = get_unaligned_le32(arg[2].buf);
	sim = mtk_ccci_rpc_eint_sim(arg[0].buf, arg[0].len);
	if (type >= RPC_EINT_ATTRS)
		ret = RPC_EINT_BAD_TYPE;
	else if (sim < 0 ||
		 of_property_read_u32_index(ccci->dev->of_node,
					    "mediatek,md-sim-eints",
					    sim * RPC_EINT_ATTRS + type, &val) ||
		 (s32)val < 0)
		ret = RPC_EINT_BAD_NAME;
	else
		ret = 0;
	dev_info(ccci->dev, "RPC: EINT %.*s attribute %u -> %d%s\n",
		 (int)strnlen(arg[0].buf, arg[0].len), (char *)arg[0].buf,
		 type, ret ? ret : (s32)val, ret ? " (error)" : "");
	rpc_reply_u32(r, ret);
	if (ret)
		rpc_reply_u32(r, ret);
	else
		rpc_reply_u32(r, val);
}

/* a GET_GPIO_ADC request of either version, and the answer to it */
struct mtk_ccci_rpc_gpio_adc {
	unsigned int pins;
	u16 req;
	u16 valid;
	const char (*name)[RPC_GPIO_NAME_LEN];
	u32 pin_in[RPC_GPIO_ADC_PINS_V2];
	u32 adc_count;
	/* answer, 0x0f bytes where nothing was found (as the downstream driver) */
	u32 pin[RPC_GPIO_ADC_PINS_V2];
	u32 level[RPC_GPIO_ADC_PINS_V2];
	u32 adc_ch;
	u32 adc_sum;
};

/*
 * The board ADC: "mediatek,md-adc" = <channel raw>, the channel number
 * the modem is told and one raw reading, as the downstream driver takes once
 * at boot and repeats for every sample the modem asks for.
 */
static int mtk_ccci_rpc_adc(struct mtk_ccci *ccci, u32 *ch, u32 *raw)
{
	struct device_node *np = ccci->dev->of_node;
	int ret;

	ret = of_property_read_u32_index(np, "mediatek,md-adc", 0, ch);
	if (!ret)
		ret = of_property_read_u32_index(np, "mediatek,md-adc", 1, raw);
	return ret;
}

static void mtk_ccci_rpc_gpio_adc_do(struct mtk_ccci *ccci,
				     struct mtk_ccci_rpc_gpio_adc *g)
{
	bool want_pin = g->req & RPC_REQ_GPIO_PIN;
	bool want_level = g->req & RPC_REQ_GPIO_VALUE;
	unsigned int i;
	u32 ch, raw, pin;
	int level;

	memset(g->pin, 0xf, sizeof(g->pin));
	memset(g->level, 0xf, sizeof(g->level));
	memset(&g->adc_ch, 0xf, sizeof(g->adc_ch));
	memset(&g->adc_sum, 0xf, sizeof(g->adc_sum));

	for (i = 0; i < g->pins; i++) {
		if (!(g->valid & BIT(i)))
			continue;
		if (want_pin) {
			/* by name; with both bits, the level of that pin */
			if (mtk_ccci_rpc_gpio_pin(ccci, g->name[i],
						  RPC_GPIO_NAME_LEN, &pin)) {
				dev_info(ccci->dev, "RPC: no modem pin for %.*s\n",
					 RPC_GPIO_NAME_LEN, g->name[i]);
				continue;
			}
			g->pin[i] = pin;
		} else {
			/* by the number the modem gave */
			pin = g->pin_in[i];
		}
		if (!want_level)
			continue;
		level = mtk_ccci_rpc_gpio_level(ccci, pin);
		if (level < 0) {
			dev_info(ccci->dev, "RPC: pin %u level: %d\n", pin,
				 level);
			continue;
		}
		g->level[i] = level;
	}

	if (!(g->req & (RPC_REQ_ADC_PIN | RPC_REQ_ADC_VALUE)))
		return;
	if (mtk_ccci_rpc_adc(ccci, &ch, &raw)) {
		dev_info(ccci->dev, "RPC: no board ADC described\n");
		return;
	}
	/* the downstream driver has one ADC for the modem and ignores the name */
	if (g->req & RPC_REQ_ADC_PIN)
		g->adc_ch = ch;
	if (g->req & RPC_REQ_ADC_VALUE)
		g->adc_sum = raw * g->adc_count;
}

static void mtk_ccci_rpc_gpio_adc(struct mtk_ccci *ccci,
				  const struct mtk_ccci_rpc_arg *arg,
				  unsigned int num,
				  struct mtk_ccci_rpc_reply *r)
{
	struct mtk_ccci_rpc_gpio_adc *g;
	unsigned int i;

	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g || num != 1)
		goto err;

	if (arg[0].len == sizeof(struct mtk_ccci_rpc_gpio_adc_in_v1)) {
		const struct mtk_ccci_rpc_gpio_adc_in_v1 *in = arg[0].buf;

		g->pins = RPC_GPIO_ADC_PINS_V1;
		g->req = in->req;
		g->valid = in->valid_pins;
		g->name = in->pin_name;
		for (i = 0; i < g->pins; i++)
			g->pin_in[i] = get_unaligned_le32(&in->pin[i]);
		g->adc_count = get_unaligned_le32(&in->adc_count);
	} else if (arg[0].len == sizeof(struct mtk_ccci_rpc_gpio_adc_in_v2)) {
		const struct mtk_ccci_rpc_gpio_adc_in_v2 *in = arg[0].buf;

		g->pins = RPC_GPIO_ADC_PINS_V2;
		g->req = get_unaligned_le16(&in->req);
		g->valid = get_unaligned_le16(&in->valid_pins);
		g->name = in->pin_name;
		for (i = 0; i < g->pins; i++)
			g->pin_in[i] = get_unaligned_le32(&in->pin[i]);
		g->adc_count = get_unaligned_le32(&in->adc_count);
	} else {
		dev_err(ccci->dev, "RPC GPIO/ADC: unknown request size %u\n",
			arg[0].len);
		goto err;
	}

	mtk_ccci_rpc_gpio_adc_do(ccci, g);
	dev_info(ccci->dev, "RPC GPIO/ADC v%u: request %#x pins %#x -> pins %*ph levels %*ph ADC %#x/%#x\n",
		 g->pins == RPC_GPIO_ADC_PINS_V1 ? 1 : 2, g->req, g->valid,
		 (int)(g->pins * 4), g->pin, (int)(g->pins * 4), g->level,
		 g->adc_ch, g->adc_sum);

	/* a status word, then the answer of the request's version */
	rpc_reply_u32(r, 0);
	r->arg[1].len = 2 * 4 * g->pins + 8;
	for (i = 0; i < g->pins; i++) {
		put_unaligned_le32(g->pin[i], r->arg[1].data + 4 * i);
		put_unaligned_le32(g->level[i],
				   r->arg[1].data + 4 * (g->pins + i));
	}
	put_unaligned_le32(g->adc_ch, r->arg[1].data + 8 * g->pins);
	put_unaligned_le32(g->adc_sum, r->arg[1].data + 8 * g->pins + 4);
	r->num = 2;
	kfree(g);
	return;

err:
	kfree(g);
	rpc_reply_u32(r, RPC_PARAM_ERROR);
	rpc_reply_u32(r, RPC_PARAM_ERROR);
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

/*
 * The modem binds a CCMNI to one of the DPMAIF UL queues: the downstream
 * driver puts normal data on queue 0 and the IMS CCMNI on queue 3. The
 * answer is 0 whatever the mapping, as in the MediaTek downstream driver,
 * which only logs a CCMNI or queue it does not know.
 */
static void mtk_ccci_rpc_lhif_map(struct mtk_ccci *ccci,
				  const struct mtk_ccci_rpc_arg *arg,
				  unsigned int num,
				  struct mtk_ccci_rpc_reply *r)
{
	struct mtk_ccci_rpc_lhif_map map;
	u32 net_if, lhif_q;
	int txq = -1;

	if (num != 1 || arg[0].len < sizeof(map)) {
		rpc_reply_u32(r, RPC_PARAM_ERROR);
		rpc_reply_u32(r, RPC_PARAM_ERROR);
		return;
	}
	memcpy(&map, arg[0].buf, sizeof(map));
	net_if = le32_to_cpu(map.net_if);
	lhif_q = le32_to_cpu(map.lhif_q);

	if (lhif_q == RPC_LHIF_HWQ_AP_UL_Q0)
		txq = 0;
	else if (lhif_q == RPC_LHIF_HWQ_AP_UL_Q1)
		txq = 3;

	if (net_if >= MTK_CCCI_NET_LINKS || txq < 0)
		dev_warn(ccci->dev, "RPC: LHIF mapping of CCMNI %u to queue %u ignored\n",
			 net_if, lhif_q);
	else {
		WRITE_ONCE(ccci->net_txq[net_if], txq);
		dev_info(ccci->dev, "RPC: CCMNI %u sends on DPMAIF UL queue %d (LHIF queue %u)\n",
			 net_if, txq, lhif_q);
	}

	rpc_reply_u32(r, 0);
	rpc_reply_u32(r, 0);
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

static bool mtk_ccci_rpc_for_userspace(u32 op)
{
	switch (op) {
	case RPC_QUERY_AP_SYS_PROP:
	case RPC_SAR_TABLE_IDX:
	case RPC_AMMS_DRDI_CONTROL:
	case RPC_SAVE_MD_CAPID:
		return true;
	default:
		return false;
	}
}

/**
 * mtk_ccci_rpc_recv() - answer a modem RPC request
 * @ccci: driver state
 * @skb: request with its CCCI header; always taken
 *
 * Malformed requests are dropped without an answer, as in the MediaTek downstream
 * driver. Requests for the userspace server wait in /dev/ccci_rpc, also
 * while nobody has it open: the modem waits for its answer.
 *
 * Return: 0, or -EBUSY when /dev/ccci_rpc is full (the request stays in
 * the ring).
 */
int mtk_ccci_rpc_recv(struct mtk_ccci *ccci, struct sk_buff *skb)
{
	struct mtk_ccci_rpc_arg arg[RPC_MAX_ARGS];
	struct mtk_ccci_rpc_reply *r;
	struct mtk_ccci_header hdr;
	u32 op, num, off, i;
	int ret;

	if (skb->len < RPC_HDR_LEN)
		goto bad;
	op = get_unaligned_le32(skb->data + sizeof(hdr));
	if (mtk_ccci_rpc_for_userspace(op)) {
		dev_info(ccci->dev, "RPC %#x (%u bytes) to /dev/%s\n", op,
			 skb->len, ccci->cdev_rpc.misc.name);
		return mtk_ccci_cdev_recv(&ccci->cdev_rpc, skb);
	}

	if (skb->len > RPC_MAX_BUF)
		goto bad;
	memcpy(&hdr, skb->data, sizeof(hdr));
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
	case RPC_GET_EINT_ATTR:
		mtk_ccci_rpc_eint_attr(ccci, arg, num, r);
		break;
	case RPC_GET_GPIO_ADC:
		mtk_ccci_rpc_gpio_adc(ccci, arg, num, r);
		break;
	case RPC_DTSI_QUERY:
		mtk_ccci_rpc_dtsi(ccci, arg, num, r);
		break;
	case RPC_LHIF_MAPPING:
		mtk_ccci_rpc_lhif_map(ccci, arg, num, r);
		break;
	case RPC_RF_ECID_DATA:
		/* acknowledged, not kept, as in the MediaTek downstream driver */
		if (num == 1) {
			dev_info(ccci->dev, "RPC: RF ECID data, %u bytes\n",
				 arg[0].len);
			rpc_reply_u32(r, 0);
			rpc_reply_u32(r, 0);
		} else {
			rpc_reply_u32(r, RPC_PARAM_ERROR);
			rpc_reply_u32(r, RPC_PARAM_ERROR);
		}
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
