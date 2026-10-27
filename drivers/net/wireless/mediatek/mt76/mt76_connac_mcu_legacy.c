// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Legacy (non unified) MCU commands of CONNAC2 firmware without unified
 * command support, such as the MT6878 CONNSYS ("SOC7_0") firmware.
 *
 * Copyright (c) 2026 serlk
 *
 * The command layouts follow MediaTek's gen4m driver
 * (include/nic_cmd_event.h, include/wsys_cmd_handler_fw.h,
 * BSD-2-Clause), Copyright (c) 2021 MediaTek Inc. That firmware takes
 * these structs at their natural alignment and with their exact size,
 * and answers none of the commands here: they are sent without waiting.
 */

#include "mt76_connac_mcu.h"

#define LEGACY_NETWORK_TYPE_AIS		0
#define LEGACY_NETWORK_TYPE_P2P		1

/* CMD_BSS_ACTIVATE_CTRL */
struct mt76_connac_legacy_bss_activate {
	u8 bss_idx;
	u8 active;
	u8 network_type;
	u8 omac_idx;
	u8 addr[ETH_ALEN];
	u8 bmc_wlan_idx;	/* broadcast/multicast WTBL entry, 0..31 */
	u8 mld_link_idx;
} __packed;

/*
 * Bring a BSS up or down: stands in for the unified DEV_INFO and basic
 * BSS_INFO, the firmware sets the own MAC address from it. bmc_wcid is
 * the WTBL entry for group addressed frames; deactivation repeats it.
 */
int mt76_connac_legacy_bss_activate(struct mt76_dev *dev,
				    struct ieee80211_vif *vif,
				    struct mt76_vif_link *mvif,
				    u8 bmc_wcid, bool enable)
{
	struct mt76_connac_legacy_bss_activate req = {
		.bss_idx = mvif->idx,
		.active = enable,
		.omac_idx = mvif->omac_idx,
		.bmc_wlan_idx = bmc_wcid,
	};

	BUILD_BUG_ON(sizeof(req) != 12);

	if (WARN_ON_ONCE(bmc_wcid > 31))
		return -EINVAL;

	switch (vif->type) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_MONITOR:
		req.network_type = LEGACY_NETWORK_TYPE_AIS;
		break;
	default:
		req.network_type = LEGACY_NETWORK_TYPE_P2P;
		break;
	}

	if (enable)
		ether_addr_copy(req.addr, vif->addr);

	return mt76_mcu_send_msg(dev, MCU_CE_CMD(BSS_ACTIVATE_CTRL), &req,
				 sizeof(req), false);
}
EXPORT_SYMBOL_GPL(mt76_connac_legacy_bss_activate);

/* CMD_RX_PACKET_FILTER: a bit mask of MT76_CONNAC_LEGACY_RX_FILTER_* */
int mt76_connac_legacy_set_rx_filter(struct mt76_dev *dev, u32 filter)
{
	struct {
		__le32 filter;
		u8 rsv[64];
	} __packed req = {
		.filter = cpu_to_le32(filter),
	};

	return mt76_mcu_send_msg(dev, MCU_CE_CMD(SET_RX_FILTER), &req,
				 sizeof(req), false);
}
EXPORT_SYMBOL_GPL(mt76_connac_legacy_set_rx_filter);

/*
 * CMD_SCAN_REQ_V2 is not packed: the extended SSID list is aligned to
 * 896, two bytes after where the packed mt76 request has it, and the
 * request is 1192 bytes long. The wildcard BSSID would otherwise be read
 * as ff:ff:ff:ff:00:00.
 */
#define LEGACY_SCAN_REQ_LEN		1192
#define LEGACY_SCAN_EXT_SSIDS		896

int mt76_connac_legacy_hw_scan(struct mt76_phy *phy, struct ieee80211_vif *vif,
			       struct ieee80211_scan_request *scan_req)
{
	const size_t head = offsetof(struct mt76_connac_hw_scan_req, ext_ssids);
	const size_t tail = offsetof(struct mt76_connac_hw_scan_req, pad) - head;
	struct mt76_connac_hw_scan_req *req;
	struct mt76_dev *mdev = phy->dev;
	struct sk_buff *skb;
	u8 *buf;
	int err;

	BUILD_BUG_ON(LEGACY_SCAN_EXT_SSIDS + tail > LEGACY_SCAN_REQ_LEN);

	if (test_bit(MT76_HW_SCANNING, &phy->state))
		return -EBUSY;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	skb = mt76_mcu_msg_alloc(mdev, NULL, LEGACY_SCAN_REQ_LEN);
	if (!skb) {
		kfree(req);
		return -ENOMEM;
	}

	set_bit(MT76_HW_SCANNING, &phy->state);
	mt76_connac_mcu_fill_hw_scan(phy, vif, scan_req, req);

	/* ext SSIDs, BSSID and random MAC address move by the alignment */
	buf = skb_put_zero(skb, LEGACY_SCAN_REQ_LEN);
	memcpy(buf, req, head);
	memcpy(buf + LEGACY_SCAN_EXT_SSIDS, (u8 *)req + head, tail);
	kfree(req);

	err = mt76_mcu_skb_send_msg(mdev, skb, MCU_CE_CMD(START_HW_SCAN),
				    false);
	if (err < 0)
		clear_bit(MT76_HW_SCANNING, &phy->state);

	return err;
}
EXPORT_SYMBOL_GPL(mt76_connac_legacy_hw_scan);
