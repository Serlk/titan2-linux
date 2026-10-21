// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: the control, system and status channels.
 *
 * Control (channel 0): the second boot handshake (HS2) and the modem
 * exception messages. System (channel 2): notifications both ways; only
 * the echo test is answered, the rest is logged. Status (channel 67):
 * the modem's answers to status polls, logged.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/skbuff.h>
#include <linux/wait.h>

#include "mtk_ccci.h"

/**
 * mtk_ccci_fsm_exception() - the modem reported an exception
 * @ccci: driver state
 * @channels: exception CCIF channels raised, 0 for the watchdog
 *
 * The vendor exception handshake (queue switch, dumps) is not done: the
 * modem is only marked, the FSM gives up waiting and "stop" powers the
 * modem off. May be called from interrupt context.
 */
void mtk_ccci_fsm_exception(struct mtk_ccci *ccci, u32 channels)
{
	if (!READ_ONCE(ccci->exception))
		dev_err(ccci->dev, "modem exception (CCIF %#x%s)\n", channels,
			ccci->wdt_fired ? ", watchdog" : "");
	ccci->ex_channels |= channels;
	WRITE_ONCE(ccci->exception, true);
	wake_up_all(&ccci->hs_wq);
}

static void mtk_ccci_hdr_words(const struct sk_buff *skb, u32 w[4])
{
	const struct mtk_ccci_header *hdr = (const void *)skb->data;

	w[0] = le32_to_cpu(hdr->data[0]);
	w[1] = le32_to_cpu(hdr->data[1]);
	w[2] = le32_to_cpu(hdr->status);
	w[3] = le32_to_cpu(hdr->reserved);
}

void mtk_ccci_ctl_recv(struct mtk_ccci *ccci, struct sk_buff *skb)
{
	u32 w[4];

	mtk_ccci_hdr_words(skb, w);
	dev_info(ccci->dev, "control message %#x %#x\n", w[1], w[3]);

	switch (w[1]) {
	case MTK_CCCI_CTL_MD_INIT_START_BOOT:
		if (w[3] == MTK_CCCI_CTL_MD_INIT_CHK_ID)
			dev_warn(ccci->dev, "HS1 on a ring queue, ignored\n");
		else
			/* HS2; the FSM never sends the runtime data yet */
			dev_warn(ccci->dev, "HS2 without runtime data, ignored\n");
		break;
	case MTK_CCCI_CTL_MD_EX:
	case MTK_CCCI_CTL_MD_EX_REC_OK:
	case MTK_CCCI_CTL_MD_EX_PASS:
	case MTK_CCCI_CTL_DRV_VER_ERROR:
		mtk_ccci_fsm_exception(ccci, 0);
		break;
	default:
		break;
	}
	consume_skb(skb);
}

void mtk_ccci_sys_recv(struct mtk_ccci *ccci, struct sk_buff *skb)
{
	u32 w[4];

	mtk_ccci_hdr_words(skb, w);
	dev_info(ccci->dev, "system message (%08x %x %lu %x)\n", w[0], w[1],
		 FIELD_GET(MTK_CCCI_H_CHANNEL, w[2]), w[3]);

	switch (w[1]) {
	case MTK_CCCI_SYS_TEST_MD2AP:
		mtk_ccci_send_msg(ccci, MTK_CCCI_Q_CTRL, MTK_CCCI_CH_SYSTEM_TX,
				  MTK_CCCI_SYS_TEST_AP2MD, w[3]);
		break;
	case MTK_CCCI_SYS_TEST_L1CORE_MD2AP:
		mtk_ccci_send_msg(ccci, MTK_CCCI_Q_CTRL, MTK_CCCI_CH_SYSTEM_TX,
				  MTK_CCCI_SYS_TEST_L1CORE_AP2MD, w[3]);
		break;
	default:
		break;
	}
	consume_skb(skb);
}

void mtk_ccci_status_recv(struct mtk_ccci *ccci, struct sk_buff *skb)
{
	u32 w[4];

	mtk_ccci_hdr_words(skb, w);
	dev_dbg(ccci->dev, "modem status %#x\n", w[2]);
	consume_skb(skb);
}
