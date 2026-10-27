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
/*
 * ENUM_SCN_DBDC_SCAN_DIS: scan on one band at a time. gen4m sets it when
 * DBDC is off; the host never sets DBDC up on the SoC (no SET_DBDC_PARMS).
 */
#define LEGACY_SCAN_FUNC_DBDC_DIS	BIT(1)

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
	req->scan_func |= LEGACY_SCAN_FUNC_DBDC_DIS;

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

/* ENUM_BAND of the firmware */
static u8 mt76_connac_legacy_band(enum nl80211_band band)
{
	switch (band) {
	case NL80211_BAND_2GHZ:
		return 1;
	case NL80211_BAND_6GHZ:
		return 3;
	default:
		return 2;
	}
}

#define LEGACY_DBDC_BAND_AUTO		4

/* CMD_CH_PRIVILEGE: ask the firmware to tune to a channel (or give it up) */
struct mt76_connac_legacy_ch_privilege {
	u8 bss_idx;
	u8 token;
	u8 action;		/* 0 request, 1 abort */
	u8 primary_ch;
	u8 sco;			/* SCN 0, SCA 1, SCB 3 */
	u8 band;
	u8 width;		/* CW_20_40MHZ 0, 80 1, 160 2 */
	u8 center_ch1;
	u8 center_ch2;
	u8 req_type;		/* JOIN 0, ROC 1 */
	u8 dbdc_band;
	u8 rsv;
	__le32 max_interval;	/* ms */
	u8 rsv2[8];
};

/*
 * Request (or abort) the channel; the firmware answers a request with
 * the unsolicited CH_PRIVILEGE event once it is on the channel.
 */
int mt76_connac_legacy_ch_privilege(struct mt76_dev *dev, u8 bss_idx,
				    u8 token, bool abort,
				    struct ieee80211_channel *chan,
				    u8 req_type, u32 duration)
{
	struct mt76_connac_legacy_ch_privilege req = {
		.bss_idx = bss_idx,
		.token = token,
		.action = abort,
		.req_type = req_type,
		.dbdc_band = LEGACY_DBDC_BAND_AUTO,
		.max_interval = cpu_to_le32(duration),
	};

	BUILD_BUG_ON(sizeof(req) != 24);

	if (chan) {
		req.primary_ch = chan->hw_value;
		req.center_ch1 = chan->hw_value;
		req.band = mt76_connac_legacy_band(chan->band);
	}

	return mt76_mcu_send_msg(dev, MCU_CE_CMD(SET_ROC), &req, sizeof(req),
				 false);
}
EXPORT_SYMBOL_GPL(mt76_connac_legacy_ch_privilege);

/* PHY_TYPE_BIT_* and the software rate indices (RATE_SET_BIT_*) */
#define LEGACY_PHY_HR_DSSS		BIT(0)
#define LEGACY_PHY_ERP			BIT(1)
#define LEGACY_PHY_OFDM			BIT(3)
#define LEGACY_PHY_HT			BIT(4)
#define LEGACY_PHY_VHT			BIT(5)
#define LEGACY_PHY_HE			BIT(6)
#define LEGACY_PHY_INDEX_HR_DSSS	0
#define LEGACY_PHY_INDEX_ERP		1
#define LEGACY_PHY_INDEX_OFDM		3

/* bitrates in 100 kbps of the software rate indices 0..13 */
static const u16 mt76_connac_legacy_rates[] = {
	10, 20, 55, 110, 220, 330, 60, 90, 120, 180, 240, 360, 480, 540,
};

static u16 mt76_connac_legacy_rate_set(struct mt76_dev *dev,
				       enum nl80211_band band, u32 mask)
{
	struct ieee80211_supported_band *sband;
	u16 set = 0;
	int i, j;

	sband = dev->hw->wiphy->bands[band];
	if (!sband)
		return 0;

	for (i = 0; i < sband->n_bitrates; i++) {
		if (!(mask & BIT(i)))
			continue;
		for (j = 0; j < ARRAY_SIZE(mt76_connac_legacy_rates); j++)
			if (sband->bitrates[i].bitrate ==
			    mt76_connac_legacy_rates[j])
				set |= BIT(j);
	}

	return set;
}

static u8 mt76_connac_legacy_phy_types(struct ieee80211_link_sta *link,
				       enum nl80211_band band)
{
	u8 set = band == NL80211_BAND_2GHZ ?
		 LEGACY_PHY_HR_DSSS | LEGACY_PHY_ERP : LEGACY_PHY_OFDM;

	if (!link)
		return set;
	if (link->ht_cap.ht_supported)
		set |= LEGACY_PHY_HT;
	if (link->vht_cap.vht_supported)
		set |= LEGACY_PHY_VHT;
	if (link->he_cap.has_he)
		set |= LEGACY_PHY_HE;

	return set;
}

/* CMD_UPDATE_STA_RECORD for HE without EHT/MLO, with RX SMM */
struct mt76_connac_legacy_sta_rec {
	u8 sta_idx;
	u8 sta_type;
	u8 addr[ETH_ALEN];
	__le16 aid;
	__le16 listen_interval;
	u8 bss_idx;
	u8 phy_types;
	__le16 rates;
	__le16 basic_rates;
	u8 qos;
	u8 uapsd;
	u8 state;
	u8 mcs_set;
	u8 mcs32;
	u8 version;
	u8 rx_mcs_mask[10];
	__le16 rx_highest;
	__le32 tx_rate_info;
	__le16 ht_cap;
	__le16 ht_ext_cap;
	__le32 txbf_cap;
	u8 ampdu_param;
	u8 asel_cap;
	u8 rcpi;
	u8 need_resp;
	u8 uapsd_ac;
	u8 uapsd_sp;
	u8 wlan_idx;
	u8 bmc_wlan_idx;
	__le32 vht_cap;
	__le16 vht_rx_mcs_map;
	__le16 vht_rx_highest;
	__le16 vht_tx_mcs_map;
	__le16 vht_tx_highest;
	u8 rts_policy;
	u8 vht_op_mode;
	u8 traffic_type;
	u8 tx_gf;
	u8 tx_sgi;
	u8 tx_stbc;
	__le16 fixed_rate_code;
	u8 tx_ampdu;
	u8 rx_ampdu;
	__le32 fixed_phy_rate;
	__le16 max_link_speed;
	__le16 min_link_speed;
	__le32 flags;
	union {
		struct {
			u8 tx;
			u8 rx;
			u8 rsv[2];
		} ht;
		struct {
			__le16 tx;
			__le16 rx;
		} he;
	} ba_size;
	u8 txbf[30];		/* PFMU/sounding block, unused */
	u8 mlr_mode;
	u8 mlr_state;
	u8 tx_amsdu_in_ampdu;
	u8 rx_amsdu_in_ampdu;
	u8 pad2[2];
	__le32 tx_max_amsdu_in_ampdu_len;
	u8 he_mac_cap[6];
	u8 he_phy_cap[11];
	u8 pad3[2];
	__le16 he_mcs[6];	/* rx/tx 80, 160, 80+80 */
	__le16 he_6g_cap;
	u8 pad4[2];
	u8 pad5[28];
	u8 rxsmm_enable;
};

#define LEGACY_STA_TYPE_LEGACY_AP	(BIT(0) | BIT(6))
#define LEGACY_STA_STATE_1		0
#define LEGACY_STA_STATE_3		2
#define LEGACY_BA_SIZE			64

/*
 * The firmware's record of the AP: state 1 before authentication, state
 * 3 once associated. State 3 is answered (EVENT_ID_ACTIVATE_STA_REC); the
 * firmware then lets data through for this WTBL entry.
 */
int mt76_connac_legacy_sta_rec(struct mt76_dev *dev, struct ieee80211_vif *vif,
			       struct mt76_vif_link *mvif,
			       struct ieee80211_sta *sta, u8 wlan_idx, bool assoc)
{
	struct ieee80211_link_sta *link = &sta->deflink;
	enum nl80211_band band;
	struct mt76_connac_legacy_sta_rec req = {
		.sta_idx = wlan_idx,
		.sta_type = LEGACY_STA_TYPE_LEGACY_AP,
		.aid = cpu_to_le16(vif->cfg.aid),
		.listen_interval = cpu_to_le16(dev->hw->conf.listen_interval),
		.bss_idx = mvif->idx,
		.qos = sta->wme,
		.uapsd = !!sta->uapsd_queues,
		.state = assoc ? LEGACY_STA_STATE_3 : LEGACY_STA_STATE_1,
		.need_resp = assoc,
		.uapsd_sp = sta->max_sp,
		.wlan_idx = wlan_idx,
		.bmc_wlan_idx = 0xff,
		.tx_sgi = 1,
		.tx_ampdu = 1,
		.rx_ampdu = 1,
	};
	struct sk_buff *skb;
	int ret;

	BUILD_BUG_ON(sizeof(req) != 204);
	BUILD_BUG_ON(offsetof(typeof(req), wlan_idx) != 54);
	BUILD_BUG_ON(offsetof(typeof(req), fixed_phy_rate) != 80);
	BUILD_BUG_ON(offsetof(typeof(req), ba_size) != 92);
	BUILD_BUG_ON(offsetof(typeof(req), he_mac_cap) != 136);
	BUILD_BUG_ON(offsetof(typeof(req), he_mcs) != 156);
	BUILD_BUG_ON(offsetof(typeof(req), rxsmm_enable) != 200);

	/* the channel context: CNM mode */
	if (!mvif->ctx)
		return -EINVAL;

	band = mvif->ctx->def.chan->band;
	req.phy_types = mt76_connac_legacy_phy_types(link, band);
	ether_addr_copy(req.addr, sta->addr);
	req.rates = cpu_to_le16(mt76_connac_legacy_rate_set(dev, band,
						link->supp_rates[band]));
	req.basic_rates = cpu_to_le16(mt76_connac_legacy_rate_set(dev, band,
						vif->bss_conf.basic_rates));
	req.uapsd_ac = (sta->uapsd_queues & 0xf) |
		       (sta->uapsd_queues & 0xf) << 4;

	if (link->ht_cap.ht_supported) {
		req.mcs_set = link->ht_cap.mcs.rx_mask[0];
		memcpy(req.rx_mcs_mask, link->ht_cap.mcs.rx_mask,
		       sizeof(req.rx_mcs_mask));
		req.rx_highest = link->ht_cap.mcs.rx_highest;
		req.ht_cap = cpu_to_le16(link->ht_cap.cap);
		req.ampdu_param = link->ht_cap.ampdu_factor |
				  link->ht_cap.ampdu_density << 2;
		req.ba_size.ht.tx = LEGACY_BA_SIZE;
		req.ba_size.ht.rx = LEGACY_BA_SIZE;
	}

	if (link->vht_cap.vht_supported) {
		req.vht_cap = cpu_to_le32(link->vht_cap.cap);
		req.vht_rx_mcs_map = link->vht_cap.vht_mcs.rx_mcs_map;
		req.vht_rx_highest = link->vht_cap.vht_mcs.rx_highest;
		req.vht_tx_mcs_map = link->vht_cap.vht_mcs.tx_mcs_map;
		req.vht_tx_highest = link->vht_cap.vht_mcs.tx_highest;
	}

	if (link->he_cap.has_he) {
		const struct ieee80211_he_mcs_nss_supp *mcs;

		mcs = &link->he_cap.he_mcs_nss_supp;
		memcpy(req.he_mac_cap, link->he_cap.he_cap_elem.mac_cap_info,
		       sizeof(req.he_mac_cap));
		memcpy(req.he_phy_cap, link->he_cap.he_cap_elem.phy_cap_info,
		       sizeof(req.he_phy_cap));
		req.he_mcs[0] = mcs->rx_mcs_80;
		req.he_mcs[1] = mcs->tx_mcs_80;
		req.he_mcs[2] = mcs->rx_mcs_160;
		req.he_mcs[3] = mcs->tx_mcs_160;
		req.he_mcs[4] = mcs->rx_mcs_80p80;
		req.he_mcs[5] = mcs->tx_mcs_80p80;
		req.he_6g_cap = link->he_6ghz_capa.capa;
		req.ba_size.he.tx = cpu_to_le16(LEGACY_BA_SIZE);
		req.ba_size.he.rx = cpu_to_le16(LEGACY_BA_SIZE);
	}

	if (!assoc)
		return mt76_mcu_send_msg(dev, MCU_CE_CMD(UPDATE_STA_RECORD),
					 &req, sizeof(req), false);

	ret = mt76_mcu_send_and_get_msg(dev, MCU_CE_CMD(UPDATE_STA_RECORD),
					&req, sizeof(req), true, &skb);
	if (ret)
		return ret;

	dev_kfree_skb(skb);
	return 0;
}
EXPORT_SYMBOL_GPL(mt76_connac_legacy_sta_rec);

/* CMD_REMOVE_STA_RECORD */
int mt76_connac_legacy_remove_sta(struct mt76_dev *dev, u8 sta_idx, u8 bss_idx)
{
	struct {
		u8 action;	/* 0: this STA */
		u8 sta_idx;
		u8 bss_idx;
		u8 rsv;
	} __packed req = {
		.sta_idx = sta_idx,
		.bss_idx = bss_idx,
	};

	return mt76_mcu_send_msg(dev, MCU_CE_CMD(REMOVE_STA_RECORD), &req,
				 sizeof(req), false);
}
EXPORT_SYMBOL_GPL(mt76_connac_legacy_remove_sta);

/* CMD_SET_BSS_RLM_PARAM */
struct mt76_connac_legacy_rlm {
	u8 bss_idx;
	u8 band;
	u8 primary_ch;
	u8 sco;
	u8 erp_protect;
	u8 ht_protect;
	u8 gf_mode;
	u8 tx_rifs;
	__le16 ht_op_info3;
	__le16 ht_op_info2;
	u8 ht_op_info1;
	u8 short_preamble;
	u8 short_slot;
	u8 vht_width;		/* CW_20_40MHZ 0, 80 1, 160 2 */
	u8 vht_center1;
	u8 vht_center2;
	__le16 vht_basic_mcs;
	u8 tx_nss;
	u8 rx_nss;
};

/* CMD_SET_BSS_INFO */
struct mt76_connac_legacy_bss_info {
	u8 bss_idx;
	u8 state;		/* 0 connected, 1 disconnected */
	u8 op_mode;		/* 0 infrastructure */
	u8 ssid_len;
	u8 ssid[32];
	u8 bssid[ETH_ALEN];
	u8 qos;
	u8 version;
	__le16 rates;
	__le16 basic_rates;
	u8 ap_sta_idx;
	u8 pad0;
	__le16 fixed_rate_code;
	u8 non_ht_phy_type;
	u8 auth_mode;
	u8 enc_status;
	u8 phy_types;
	u8 wapi;
	u8 ap_mode;
	u8 bmc_wlan_idx;
	u8 hidden_ssid;
	u8 disconnect_threshold;
	u8 iot_ap_act;
	u8 pad1[2];
	__le32 private_data;
	struct mt76_connac_legacy_rlm rlm;
	u8 dbdc_band;
	u8 wmm_set;
	u8 dbdc_action;
	u8 nss;
	u8 pad2[2];
	u8 he_op[3];
	u8 bss_color;
	__le16 he_basic_mcs;
	u8 max_bssid_indicator;
	u8 mbssid_index;
	u8 pad[12];
};

#define LEGACY_AUTH_MODE_OPEN		0
#define LEGACY_AUTH_MODE_WPA2_PSK	7
#define LEGACY_ENC_DISABLED		1
#define LEGACY_ENC3_ENABLED		6	/* CCMP */
#define LEGACY_STA_REC_NOT_FOUND	0xff
#define LEGACY_RATE_CODE_CCK_1M		0x0000
#define LEGACY_RATE_CODE_OFDM_6M	0x004b

/*
 * Tell the firmware the BSS it is in (or that it left it): BSSID, the AP's
 * STA_REC, rates and the operating channel, from which it runs the BSS.
 */
int mt76_connac_legacy_bss_info(struct mt76_dev *dev, struct ieee80211_vif *vif,
				struct mt76_vif_link *mvif,
				struct ieee80211_sta *ap, u8 ap_wlan_idx,
				u8 bmc_wlan_idx, bool connected)
{
	struct ieee80211_bss_conf *conf = &vif->bss_conf;
	struct ieee80211_link_sta *link = ap ? &ap->deflink : NULL;
	struct mt76_connac_legacy_bss_info req = {
		.bss_idx = mvif->idx,
		.state = !connected,
		.qos = conf->qos,
		.ap_sta_idx = connected ? ap_wlan_idx : LEGACY_STA_REC_NOT_FOUND,
		.bmc_wlan_idx = bmc_wlan_idx,
		.wmm_set = mvif->wmm_idx,
		.dbdc_band = LEGACY_DBDC_BAND_AUTO,
		.nss = 1,
		.rlm = {
			.bss_idx = mvif->idx,
			.short_preamble = conf->use_short_preamble,
			.short_slot = conf->use_short_slot,
			.tx_nss = 1,
			.rx_nss = 1,
		},
	};
	struct cfg80211_chan_def *chandef;
	struct ieee80211_channel *chan;
	enum nl80211_band band;
	int center;

	BUILD_BUG_ON(sizeof(req.rlm) != 22);
	BUILD_BUG_ON(sizeof(req) != 116);
	BUILD_BUG_ON(offsetof(typeof(req), rlm) != 68);
	BUILD_BUG_ON(offsetof(typeof(req), he_basic_mcs) != 100);

	/* the channel context: CNM mode */
	if (!mvif->ctx)
		return -EINVAL;

	chandef = &mvif->ctx->def;
	chan = chandef->chan;
	band = chan->band;
	center = ieee80211_frequency_to_channel(chandef->center_freq1);
	req.phy_types = mt76_connac_legacy_phy_types(link, band);
	req.rlm.band = mt76_connac_legacy_band(band);
	req.rlm.primary_ch = chan->hw_value;

	req.ssid_len = min_t(size_t, vif->cfg.ssid_len, sizeof(req.ssid));
	memcpy(req.ssid, vif->cfg.ssid, req.ssid_len);
	if (conf->bssid)
		ether_addr_copy(req.bssid, conf->bssid);

	if (link)
		req.rates = cpu_to_le16(mt76_connac_legacy_rate_set(dev, band,
						link->supp_rates[band]));
	req.basic_rates = cpu_to_le16(mt76_connac_legacy_rate_set(dev, band,
						conf->basic_rates));

	if (band == NL80211_BAND_2GHZ) {
		req.fixed_rate_code = cpu_to_le16(LEGACY_RATE_CODE_CCK_1M);
		req.non_ht_phy_type = le16_to_cpu(req.basic_rates) & 0xf ?
				      LEGACY_PHY_INDEX_HR_DSSS :
				      LEGACY_PHY_INDEX_ERP;
	} else {
		req.fixed_rate_code = cpu_to_le16(LEGACY_RATE_CODE_OFDM_6M);
		req.non_ht_phy_type = LEGACY_PHY_INDEX_OFDM;
	}

	if (conf->bss && conf->bss->capability & WLAN_CAPABILITY_PRIVACY) {
		req.auth_mode = LEGACY_AUTH_MODE_WPA2_PSK;
		req.enc_status = LEGACY_ENC3_ENABLED;
	} else {
		req.auth_mode = LEGACY_AUTH_MODE_OPEN;
		req.enc_status = LEGACY_ENC_DISABLED;
	}

	if (chandef->width == NL80211_CHAN_WIDTH_40) {
		req.rlm.sco = chan->center_freq < chandef->center_freq1 ? 1 : 3;
	} else if (chandef->width == NL80211_CHAN_WIDTH_80) {
		req.rlm.sco = chan->center_freq < chandef->center_freq1 ? 1 : 3;
		req.rlm.vht_width = 1;
		req.rlm.vht_center1 = center;
	} else if (chandef->width == NL80211_CHAN_WIDTH_160) {
		req.rlm.sco = chan->center_freq < chandef->center_freq1 ? 1 : 3;
		req.rlm.vht_width = 2;
		req.rlm.vht_center1 = center;
	}

	if (link) {
		req.nss = link->rx_nss ?: 1;
		req.rlm.tx_nss = req.nss;
		req.rlm.rx_nss = req.nss;
	}

	if (conf->he_support) {
		memcpy(req.he_op, &conf->he_oper.params, sizeof(req.he_op));
		req.bss_color = conf->he_bss_color.color;
		req.he_basic_mcs = cpu_to_le16(conf->he_oper.nss_set);
	}

	return mt76_mcu_send_msg(dev, MCU_CE_CMD(SET_BSS_INFO), &req,
				 sizeof(req), false);
}
EXPORT_SYMBOL_GPL(mt76_connac_legacy_bss_info);

/* CMD_802_11_KEY */
struct mt76_connac_legacy_key {
	u8 add;
	u8 tx_key;
	u8 pairwise;
	u8 authenticator;
	u8 peer[ETH_ALEN];
	u8 bss_idx;
	u8 cipher;
	u8 key_id;
	u8 key_len;
	u8 wlan_idx;
	u8 mgmt_protection;
	u8 key[32];
	u8 rsc[16];
};

/* CIPHER_SUITE_* of the firmware, 0 if not offloaded */
static u8 mt76_connac_legacy_cipher(u32 cipher)
{
	switch (cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		return 1;
	case WLAN_CIPHER_SUITE_TKIP:
		return 2;
	case WLAN_CIPHER_SUITE_CCMP:
		return 4;
	case WLAN_CIPHER_SUITE_WEP104:
		return 5;
	case WLAN_CIPHER_SUITE_GCMP:
		return 11;
	case WLAN_CIPHER_SUITE_GCMP_256:
		return 12;
	default:
		return 0;
	}
}

/*
 * Install or remove a pairwise key (WTBL entry of the AP) or a group key
 * (the group WTBL entry of the BSS); peer is the AP's address for both.
 */
int mt76_connac_legacy_add_key(struct mt76_dev *dev, struct mt76_vif_link *mvif,
			       struct ieee80211_key_conf *key, const u8 *peer,
			       u8 wlan_idx, bool add)
{
	struct mt76_connac_legacy_key req = {
		.add = add,
		.pairwise = !!(key->flags & IEEE80211_KEY_FLAG_PAIRWISE),
		.bss_idx = mvif->idx,
		.cipher = mt76_connac_legacy_cipher(key->cipher),
		.key_id = key->keyidx,
		.key_len = key->keylen,
		.wlan_idx = wlan_idx,
	};

	BUILD_BUG_ON(sizeof(req) != 64);

	if (!req.cipher || key->keylen > sizeof(req.key))
		return -EOPNOTSUPP;

	req.tx_key = req.pairwise;
	if (peer)
		ether_addr_copy(req.peer, peer);

	if (add) {
		memcpy(req.key, key->key, key->keylen);
		/* mac80211 has the TX MIC key first, the firmware the RX one */
		if (key->cipher == WLAN_CIPHER_SUITE_TKIP) {
			memcpy(req.key + 16, key->key + 24, 8);
			memcpy(req.key + 24, key->key + 16, 8);
		}
	}

	return mt76_mcu_send_msg(dev, MCU_CE_CMD(ADD_REMOVE_KEY), &req,
				 sizeof(req), false);
}
EXPORT_SYMBOL_GPL(mt76_connac_legacy_add_key);
