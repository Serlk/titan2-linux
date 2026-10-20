// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * MT7921 Wi-Fi core of the MediaTek MT6878 CONNSYS ("SOC7_0") on AXI.
 *
 * Copyright (c) 2026 serlk
 *
 * The register map of the AXI window, the WFDMA ring layout and the
 * firmware start command follow MediaTek's gen4m driver for SOC7_0
 * (chips/soc7_0/soc7_0.c, BSD-2-Clause), Copyright (c) 2021 MediaTek Inc.
 * The probe flow follows mt7921/pci.c, Copyright (C) 2020 MediaTek Inc.
 *
 * Unlike the PCIe/USB/SDIO parts, the core is powered by CONNINFRA
 * (drivers/soc/mediatek/mtk-conninfra.c), which also brings up the WFSYS
 * and its WM MCU, and the stock boot loader has already copied the ROM
 * patch and RAM firmware into the CONNSYS EMI. The driver therefore does
 * not download firmware: it asks the ROM to calibrate (PHY_ACTION), sends
 * FW_IMAGE_START and waits for the WM CPU, as MediaTek's driver does with
 * CFG_WLAN_LK_FWDL_SUPPORT.
 *
 * Calibration and RF data: without the stock NVRAM file the firmware runs
 * on its efuse defaults. A copy of the stock file (nvdata
 * APCFG/APRDEB/WIFI) installed as mediatek/mt6878/WIFI is used read-only
 * for the MAC address and the front end tags of the calibration request.
 *
 * Not done: suspend/resume, deep sleep with CONN_INFRA asleep, the
 * connfem front end description, transmit power tables from NVRAM.
 */

#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jhash.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/mtk-conninfra.h>
#include <linux/unaligned.h>

#include "mt7921.h"
#include "../mt76_connac2_mac.h"
#include "../dma.h"
#include "mcu.h"

/* the AP sees CONNSYS at 0x1800_0000..0x186f_ffff (CONN_INFRA, then WFSYS) */
#define MT7921_SOC_WINDOW		0x700000
#define MT7921_SOC_PHYS_BASE		0x18000000

#define MT7921_SOC_REG_INVALID		U32_MAX
#define MT7921_SOC_REG_IGNORE		(U32_MAX - 1)

/* CONN_INFRA_CFG_ON: CONN_INFRA host CSR interrupt enables (WFDMA, WF2AP, ...) */
#define MT7921_SOC_HOST_CSR_IRQ_EN	0x7c001600
#define MT7921_SOC_HOST_CSR_IRQ_ALL	0x3ff
#define MT7921_SOC_CONN_INFRA_RDY	0x7c001210	/* bit 16: cmdbt restored */
#define MT7921_SOC_CONN_IP_VERSION	0x7c011000
#define MT7921_SOC_CONN_IP_MT6878	0x02050401
#define MT7921_SOC_WAKEPU_WF		0x7c0601a4	/* Wi-Fi's force-on of CONN_INFRA */

/* AP2WF CCIF4 (CONN_INFRA on): software interrupts from the WM MCU */
#define MT7921_SOC_CCIF_RCHNUM		0x7c03d010
#define MT7921_SOC_CCIF_ACK		0x7c03d014
#define MT7921_SOC_SW_INT_FW_LOG	BIT(0)
#define MT7921_SOC_SW_INT_SUBSYS_RESET	BIT(1)
#define MT7921_SOC_SW_INT_WHOLE_RESET	BIT(2)

/* CONNSYS EMI: pointer to the release manifest of the RAM code */
#define MT7921_SOC_EMI_FW_VER_PTR	0xb00024
#define MT7921_SOC_EMI_ADDR_MASK	GENMASK(23, 0)
#define MT7921_SOC_FW_VER_LEN		128

/* copy of the stock NVRAM file (nvdata APCFG/APRDEB/WIFI), optional */
#define MT7921_SOC_NVRAM		"mediatek/mt6878/WIFI"
#define MT7921_SOC_NVRAM_MAC		4
#define MT7921_SOC_NVRAM_TLV		256
#define MT7921_SOC_NVRAM_FEM_MAX	1024

/* INIT_CMD_ID_PHY_ACTION (ROM code) */
#define MT7921_SOC_PHY_MAGIC		0x556789aa
#define MT7921_SOC_PHY_VERSION		1
#define MT7921_SOC_PHY_TAG_NVRAM	0
#define MT7921_SOC_PHY_TAG_CAL		1
#define MT7921_SOC_PHY_FORCE_CAL_REQ	0x01
#define MT7921_SOC_PHY_FORCE_CAL_RSP	0x81

/* WF_MCU_CFG_LS: hardware version (CONNAC2X_TOP_HVR of SOC7_0) */
#define MT7921_SOC_TOP_HVR		0x88000000
#define MT7921_SOC_TOP_FVR		0x88000004

/* WFDMA ring layout of SOC7_0 */
#define MT7921_SOC_TXQ_MCU_WM		15
#define MT7921_SOC_TXQ_FWDL		16
#define MT7921_SOC_RXQ_DATA		0	/* band 0 rx data */
#define MT7921_SOC_RXQ_EVENT		2	/* rx event, band 0 tx free done */
#define MT7921_SOC_RXQ_TXDONE1		3	/* band 1 tx free done */
#define MT7921_SOC_RX_EVENT_RING_SIZE	512

#define MT7921_SOC_INT_TX_DONE_MCU_WM	BIT(25)
#define MT7921_SOC_INT_TX_DONE_FWDL	BIT(26)
#define MT7921_SOC_INT_TX_DONE_MCU	(MT7921_SOC_INT_TX_DONE_MCU_WM | \
					 MT7921_SOC_INT_TX_DONE_FWDL)
#define MT7921_SOC_INT_TX_DONE_ALL	(MT7921_SOC_INT_TX_DONE_MCU | \
					 HOST_TX_DONE_INT_ENA0)

struct mt7921_soc_dev {
	struct mt792x_dev dev;		/* must be first, see mt76_alloc_device() */
	struct mtk_conninfra *ci;
	int irq;
	int irq_sw;			/* WF2AP CCIF, optional */
	bool radio_on;
	bool awake;
	bool wfsys_on;			/* gates every register access */

	/* from the stock NVRAM file, if present */
	u8 nvram_mac[ETH_ALEN];
	u8 *fem_tags;			/* tags for the calibration request */
	u32 fem_tags_len;
};

struct mt7921_soc_phy_hdr {
	__le32 magic;
	u8 tag_num;
	u8 version;
	__le16 len;
} __packed;

struct mt7921_soc_phy_tlv {
	__le16 tag;
	__le16 len;
} __packed;

struct mt7921_soc_phy_cal {
	u8 cmd;
	u8 save_result;
	u8 skip_cal;
	u8 rsv;
} __packed;

struct mt7921_soc_phy_rsp {
	u8 event;
	u8 status;
	u8 rsv[2];
	__le32 emi_addr;
	__le32 emi_len;
	__le32 temperature;
} __packed;

static u8 fw_features;
module_param(fw_features, byte, 0444);
MODULE_PARM_DESC(fw_features,
		 "Firmware offload capabilities (BIT(7): CNM), not readable from the EMI image");

static bool runtime_pm;
module_param(runtime_pm, bool, 0444);
MODULE_PARM_DESC(runtime_pm,
		 "Hand the core to the firmware (fw own) when idle; CONN_INFRA stays awake");

static bool phy_action = true;
module_param(phy_action, bool, 0444);
MODULE_PARM_DESC(phy_action,
		 "Ask the ROM code to calibrate before the firmware starts, as the MediaTek downstream driver");

static struct mt7921_soc_dev *to_soc(struct mt792x_dev *dev)
{
	return container_of(dev, struct mt7921_soc_dev, dev);
}

/*
 * The shared mt76 register definitions mix chip addresses with offsets
 * into the PCIe BAR (anything below 0x100000). Both are translated to an
 * offset into the AXI window: BAR offsets through the PCIe column, chip
 * addresses through the chip column. The AXI column is SOC7_0's
 * bus2chip table for _HIF_AXI.
 */
struct mt7921_soc_reg_map {
	u32 chip;
	u32 pcie;	/* MT7921 PCIe BAR offset, U32_MAX if none */
	u32 axi;	/* offset in the AXI window */
	u32 size;
};

#define NO_PCIE	U32_MAX

static const struct mt7921_soc_reg_map mt7921_soc_reg_map[] = {
	{ 0x830c0000, NO_PCIE, 0x400000, 0x01000 }, /* WF_MCU_BUS_CR_REMAP */
	{ 0x54000000, 0x02000, 0x402000, 0x01000 }, /* WFDMA PCIE0 MCU DMA0 */
	{ 0x55000000, 0x03000, 0x403000, 0x01000 }, /* WFDMA PCIE0 MCU DMA1 */
	{ 0x56000000, 0x04000, 0x404000, 0x01000 }, /* WFDMA reserved */
	{ 0x57000000, 0x05000, 0x405000, 0x01000 }, /* WFDMA MCU wrap CR */
	{ 0x58000000, 0x06000, 0x406000, 0x01000 }, /* WFDMA PCIE1 MCU DMA0 (MEM_DMA) */
	{ 0x59000000, 0x07000, 0x407000, 0x01000 }, /* WFDMA PCIE1 MCU DMA1 */
	{ 0x820c0000, 0x08000, 0x408000, 0x04000 }, /* WF_UMAC_TOP (PLE) */
	{ 0x820c8000, 0x0c000, 0x40c000, 0x02000 }, /* WF_UMAC_TOP (PSE) */
	{ 0x820cc000, 0x0e000, 0x40e000, 0x02000 }, /* WF_UMAC_TOP (PP), WF_MDP_TOP */
	{ 0x83000000, NO_PCIE, 0x410000, 0x10000 }, /* WF_PHY_MAP3 */
	{ 0x820e0000, 0x20000, 0x420000, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_CFG) */
	{ 0x820e1000, 0x20400, 0x420400, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_TRB) */
	{ 0x820e2000, 0x20800, 0x420800, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_AGG) */
	{ 0x820e3000, 0x20c00, 0x420c00, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_ARB) */
	{ 0x820e4000, 0x21000, 0x421000, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_TMAC) */
	{ 0x820e5000, 0x21400, 0x421400, 0x00800 }, /* WF_LMAC_TOP BN0 (WF_RMAC) */
	{ 0x820ce000, 0x21c00, 0x421c00, 0x00200 }, /* WF_LMAC_TOP (WF_SEC) */
	{ 0x820e7000, 0x21e00, 0x421e00, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_DMA) */
	{ 0x820cf000, 0x22000, 0x422000, 0x01000 }, /* WF_LMAC_TOP (WF_PF) */
	{ 0x820e9000, 0x23400, 0x423400, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_WTBLOFF) */
	{ 0x820ea000, 0x24000, 0x424000, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_ETBF) */
	{ 0x820eb000, 0x24200, 0x424200, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_LPON) */
	{ 0x820ec000, 0x24600, 0x424600, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_INT) */
	{ 0x820ed000, 0x24800, 0x424800, 0x00800 }, /* WF_LMAC_TOP BN0 (WF_MIB) */
	{ 0x820ca000, 0x26000, 0x426000, 0x02000 }, /* WF_LMAC_TOP BN0 (WF_MUCOP) */
	{ 0x820d0000, 0x30000, 0x430000, 0x10000 }, /* WF_LMAC_TOP (WF_WTBLON) */
	{ 0x83080000, NO_PCIE, 0x450000, 0x10000 }, /* WF_PHY_MAP1 */
	{ 0x83090000, NO_PCIE, 0x460000, 0x10000 }, /* WF_PHY_MAP2 */
	{ 0xe0200000, NO_PCIE, 0x470000, 0x10000 }, /* WF_UMAC_SYSRAM */
	{ 0x00400000, 0x80000, 0x480000, 0x10000 }, /* WF_MCU_SYSRAM */
	{ 0x00410000, 0x90000, 0x490000, 0x10000 }, /* WF_MCU_SYSRAM (config register) */
	{ 0x820f0000, 0xa0000, 0x4a0000, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_CFG) */
	{ 0x820f1000, 0xa0600, 0x4a0600, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_TRB) */
	{ 0x820f2000, 0xa0800, 0x4a0800, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_AGG) */
	{ 0x820f3000, 0xa0c00, 0x4a0c00, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_ARB) */
	{ 0x820f4000, 0xa1000, 0x4a1000, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_TMAC) */
	{ 0x820f5000, 0xa1400, 0x4a1400, 0x00800 }, /* WF_LMAC_TOP BN1 (WF_RMAC) */
	{ 0x820f7000, 0xa1e00, 0x4a1e00, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_DMA) */
	{ 0x820f9000, 0xa3400, 0x4a3400, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_WTBLOFF) */
	{ 0x820fa000, 0xa4000, 0x4a4000, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_ETBF) */
	{ 0x820fb000, 0xa4200, 0x4a4200, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_LPON) */
	{ 0x820fc000, 0xa4600, 0x4a4600, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_INT) */
	{ 0x820fd000, 0xa4800, 0x4a4800, 0x00800 }, /* WF_LMAC_TOP BN1 (WF_MIB) */
	{ 0x820c4000, 0xa8000, 0x4a8000, 0x04000 }, /* WF_LMAC_TOP BN1 (WF_UWTBL) */
	{ 0x820b0000, 0xae000, 0x4ae000, 0x01000 }, /* [APB2] WFSYS_ON */
	{ 0x80020000, 0xb0000, 0x4b0000, 0x10000 }, /* WF_TOP_MISC_OFF */
	{ 0x81020000, 0xc0000, 0x4c0000, 0x10000 }, /* WF_TOP_MISC_ON */
	{ 0x80010000, NO_PCIE, 0x4d4000, 0x01000 }, /* WF_AXIDMA */
	{ 0x83010000, NO_PCIE, 0x4e0000, 0x10000 }, /* WF_PHY_MAP4 */
	{ 0x88000000, NO_PCIE, 0x4f0000, 0x10000 }, /* WF_MCU_CFG_LS */
	/* CONN_INFRA: the PCIe BAR has three 64 KiB windows into it */
	{ 0x7c020000, 0xd0000, 0x020000, 0x10000 }, /* CONN_INFRA, WFDMA */
	{ 0x7c060000, 0xe0000, 0x060000, 0x10000 }, /* CONN_INFRA, conn_host_csr_top */
	{ 0x7c000000, 0xf0000, 0x000000, 0x10000 }, /* CONN_INFRA */
	{ 0x7c000000, NO_PCIE, 0x000000, 0x100000 }, /* CONN_INFRA */
	{ 0x7c500000, NO_PCIE, 0x500000, 0x200000 }, /* remap, for fwdl */
};

static u32 mt7921_soc_reg_addr(struct mt792x_dev *dev, u32 addr)
{
	bool pcie = addr < 0x100000;
	int i;

	/* PCIe MAC registers (interrupt enable, power management): no-op */
	if ((pcie && addr >= 0x10000 && addr < 0x20000) ||
	    (addr >= 0x74030000 && addr < 0x74040000))
		return MT7921_SOC_REG_IGNORE;

	if (!pcie && addr >= MT7921_SOC_PHYS_BASE &&
	    addr < MT7921_SOC_PHYS_BASE + MT7921_SOC_WINDOW)
		return addr - MT7921_SOC_PHYS_BASE;

	for (i = 0; i < ARRAY_SIZE(mt7921_soc_reg_map); i++) {
		const struct mt7921_soc_reg_map *map = &mt7921_soc_reg_map[i];
		u32 base = pcie ? map->pcie : map->chip;

		if (base == NO_PCIE || addr < base || addr - base >= map->size)
			continue;

		return map->axi + (addr - base);
	}

	dev_err_ratelimited(dev->mt76.dev,
			    "access to unmapped register %08x\n", addr);

	return MT7921_SOC_REG_INVALID;
}

/*
 * WFSYS registers hang the AXI bus while the subsystem is off or its
 * bus protection is engaged: no access before CONNINFRA reported WFSYS
 * on, and none after it has been switched off.
 */
static u32 mt7921_soc_rr(struct mt76_dev *mdev, u32 offset)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	u32 addr;

	if (WARN_ON_ONCE(!to_soc(dev)->wfsys_on))
		return ~0;

	addr = mt7921_soc_reg_addr(dev, offset);
	if (addr == MT7921_SOC_REG_INVALID)
		return ~0;
	if (addr == MT7921_SOC_REG_IGNORE)
		return 0;

	return dev->bus_ops->rr(mdev, addr);
}

static void mt7921_soc_wr(struct mt76_dev *mdev, u32 offset, u32 val)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	u32 addr;

	if (WARN_ON_ONCE(!to_soc(dev)->wfsys_on))
		return;

	addr = mt7921_soc_reg_addr(dev, offset);
	if (addr >= MT7921_SOC_REG_IGNORE)
		return;

	dev->bus_ops->wr(mdev, addr, val);
}

static u32 mt7921_soc_rmw(struct mt76_dev *mdev, u32 offset, u32 mask, u32 val)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	u32 addr;

	if (WARN_ON_ONCE(!to_soc(dev)->wfsys_on))
		return ~0;

	addr = mt7921_soc_reg_addr(dev, offset);
	if (addr == MT7921_SOC_REG_INVALID)
		return ~0;
	if (addr == MT7921_SOC_REG_IGNORE)
		return 0;

	return dev->bus_ops->rmw(mdev, addr, mask, val);
}

/* as mt7921e_tx_prepare_skb(): the WFDMA tx path is the same as on PCIe */
static int mt7921_soc_tx_prepare_skb(struct mt76_dev *mdev, void *txwi_ptr,
				     enum mt76_txq_id qid, struct mt76_wcid *wcid,
				     struct ieee80211_sta *sta,
				     struct mt76_tx_info *tx_info)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(tx_info->skb);
	struct ieee80211_key_conf *key = info->control.hw_key;
	struct mt76_connac_hw_txp *txp;
	struct mt76_txwi_cache *t;
	u8 *txwi = (u8 *)txwi_ptr;
	int id, pid;

	if (unlikely(tx_info->skb->len <= ETH_HLEN))
		return -EINVAL;

	if (!wcid)
		wcid = &dev->mt76.global_wcid;

	t = (struct mt76_txwi_cache *)(txwi + mdev->drv->txwi_size);
	t->skb = tx_info->skb;

	id = mt76_token_consume(mdev, &t);
	if (id < 0)
		return id;

	if (sta) {
		struct mt792x_sta *msta = (struct mt792x_sta *)sta->drv_priv;

		if (time_after(jiffies, msta->deflink.last_txs + HZ / 4)) {
			info->flags |= IEEE80211_TX_CTL_REQ_TX_STATUS;
			msta->deflink.last_txs = jiffies;
		}
	}

	pid = mt76_tx_status_skb_add(mdev, wcid, tx_info->skb);
	mt76_connac2_mac_write_txwi(mdev, txwi_ptr, tx_info->skb, wcid, key,
				    pid, qid, 0);

	txp = (struct mt76_connac_hw_txp *)(txwi + MT_TXD_SIZE);
	memset(txp, 0, sizeof(struct mt76_connac_hw_txp));
	mt76_connac_write_hw_txp(mdev, tx_info, txp, id);

	tx_info->skb = NULL;

	return 0;
}

static int mt7921_soc_mcu_send_message(struct mt76_dev *mdev,
				       struct sk_buff *skb, int cmd, int *seq)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	enum mt76_mcuq_id txq = MT_MCUQ_WM;
	int ret;

	ret = mt76_connac2_mcu_fill_message(mdev, skb, cmd, seq);
	if (ret)
		return ret;

	/* the ROM calibration takes a while */
	mdev->mcu.timeout = cmd == MCU_CMD(PHY_ACTION) ? 10 * HZ : 3 * HZ;

	if (cmd == MCU_CMD(FW_SCATTER))
		txq = MT_MCUQ_FWDL;

	return mt76_tx_queue_skb_raw(dev, mdev->q_mcu[txq], skb, 0);
}

/*
 * Before every driver own the MediaTek downstream driver forces the CONN_INFRA host CSR
 * interrupt enables on (soc7_0ForceEnableHostCsrIrq): wake CONN_INFRA
 * for Wi-Fi, check it is up, write 0x3ff, let it go again.
 */
static void mt7921_soc_force_host_csr_irq(struct mt792x_dev *dev)
{
	mt76_set(dev, MT7921_SOC_WAKEPU_WF, BIT(0));
	usleep_range(200, 300);

	if (!mt76_poll_msec_tick(dev, MT7921_SOC_CONN_IP_VERSION, U32_MAX,
				 MT7921_SOC_CONN_IP_MT6878, 11, 1)) {
		dev_err(dev->mt76.dev, "CONN_INFRA not awake (0x%08x)\n",
			mt76_rr(dev, MT7921_SOC_CONN_IP_VERSION));
		goto out;
	}
	if (!mt76_poll_msec_tick(dev, MT7921_SOC_CONN_INFRA_RDY, BIT(16),
				 BIT(16), 6, 1)) {
		dev_err(dev->mt76.dev, "CONN_INFRA restore not done (0x%08x)\n",
			mt76_rr(dev, MT7921_SOC_CONN_INFRA_RDY));
		goto out;
	}

	mt76_wr(dev, MT7921_SOC_HOST_CSR_IRQ_EN, MT7921_SOC_HOST_CSR_IRQ_ALL);
out:
	mt76_clear(dev, MT7921_SOC_WAKEPU_WF, BIT(0));
}

/* hif drv_own: the CONN_INFRA off domain is kept awake while bound */
static int mt7921_soc_drv_own(struct mt792x_dev *dev)
{
	mt7921_soc_force_host_csr_irq(dev);

	return mt792xe_mcu_drv_pmctrl(dev);
}

/*
 * The stock NVRAM file: MAC address at offset 4, then from offset 256 a
 * list of {id, len16} tagged records. The calibration request carries
 * the 2.4/5/6 GHz common and system records and every record with the
 * "valid" bit (0x80) set (wlanGetEpaElnaFromNvram()).
 */
static void mt7921_soc_load_nvram(struct mt7921_soc_dev *soc)
{
	static const u8 need[] = { 7, 8, 9, 21 };
	struct device *dev = soc->dev.mt76.dev;
	const struct firmware *fw;
	u32 off, total, len = 0;
	const u8 *d;
	u8 *tags;

	if (firmware_request_nowarn(&fw, MT7921_SOC_NVRAM, dev))
		return;

	d = fw->data;
	if (fw->size < MT7921_SOC_NVRAM_TLV + 6) {
		dev_warn(dev, "%s: too short (%zu)\n", MT7921_SOC_NVRAM, fw->size);
		goto out;
	}

	ether_addr_copy(soc->nvram_mac, d + MT7921_SOC_NVRAM_MAC);

	tags = devm_kzalloc(dev, MT7921_SOC_NVRAM_FEM_MAX, GFP_KERNEL);
	if (!tags)
		goto out;

	/* record 0 (NVRAM control) holds the length of the tagged part */
	total = MT7921_SOC_NVRAM_TLV;
	if (d[MT7921_SOC_NVRAM_TLV] == 0)
		total += get_unaligned_le16(d + MT7921_SOC_NVRAM_TLV + 4);
	total = min_t(u32, total, fw->size);

	for (off = MT7921_SOC_NVRAM_TLV; off + 3 <= total; ) {
		u8 id = d[off];
		u32 rlen = get_unaligned_le16(d + off + 1) + 3;
		bool add = id & 0x80;
		int i;

		if (!id && rlen == 3)
			break;
		if (off + rlen > fw->size)
			break;

		for (i = 0; i < ARRAY_SIZE(need); i++)
			add |= id == need[i];
		if (add && len + rlen <= MT7921_SOC_NVRAM_FEM_MAX) {
			memcpy(tags + len, d + off, rlen);
			len += rlen;
		}
		off += rlen;
	}

	soc->fem_tags = tags;
	soc->fem_tags_len = min_t(u32, ALIGN(len, 4), MT7921_SOC_NVRAM_FEM_MAX);
	dev_info(dev, "%s: MAC %pM, %u bytes of RF records\n",
		 MT7921_SOC_NVRAM, soc->nvram_mac, len);
out:
	release_firmware(fw);
}

/*
 * INIT_CMD_ID_PHY_ACTION, FORCE_CAL: the ROM code calibrates the RF with
 * the NVRAM records (if any) before the RAM code starts. The result is
 * not saved: without the stock backup in EMI every power-on calibrates.
 */
static int mt7921_soc_phy_action(struct mt7921_soc_dev *soc)
{
	struct mt792x_dev *dev = &soc->dev;
	struct mt7921_soc_phy_hdr *hdr;
	struct mt7921_soc_phy_tlv *tlv;
	struct mt7921_soc_phy_cal *cal;
	struct mt7921_soc_phy_rsp *rsp;
	struct sk_buff *skb;
	u32 len, body;
	u8 *buf, *p;
	int ret;

	body = sizeof(*tlv) + sizeof(*cal);
	if (soc->fem_tags_len)
		body += sizeof(*tlv) + soc->fem_tags_len;
	len = sizeof(*hdr) + body;

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	hdr = (struct mt7921_soc_phy_hdr *)buf;
	hdr->magic = cpu_to_le32(MT7921_SOC_PHY_MAGIC);
	hdr->tag_num = soc->fem_tags_len ? 2 : 1;
	hdr->version = MT7921_SOC_PHY_VERSION;
	hdr->len = cpu_to_le16(body);

	p = buf + sizeof(*hdr);
	tlv = (struct mt7921_soc_phy_tlv *)p;
	tlv->tag = cpu_to_le16(MT7921_SOC_PHY_TAG_CAL);
	tlv->len = cpu_to_le16(sizeof(*cal));
	cal = (struct mt7921_soc_phy_cal *)(tlv + 1);
	cal->cmd = MT7921_SOC_PHY_FORCE_CAL_REQ;
	p += sizeof(*tlv) + sizeof(*cal);

	if (soc->fem_tags_len) {
		tlv = (struct mt7921_soc_phy_tlv *)p;
		tlv->tag = cpu_to_le16(MT7921_SOC_PHY_TAG_NVRAM);
		tlv->len = cpu_to_le16(soc->fem_tags_len);
		memcpy(tlv + 1, soc->fem_tags, soc->fem_tags_len);
	}

	ret = mt76_mcu_send_and_get_msg(&dev->mt76, MCU_CMD(PHY_ACTION), buf,
					len, true, &skb);
	kfree(buf);
	if (ret)
		return ret;

	if (skb->len < sizeof(*hdr) + sizeof(*tlv) + sizeof(*rsp)) {
		ret = -EINVAL;
		goto out;
	}
	hdr = (struct mt7921_soc_phy_hdr *)skb->data;
	tlv = (struct mt7921_soc_phy_tlv *)(hdr + 1);
	rsp = (struct mt7921_soc_phy_rsp *)(tlv + 1);
	if (le32_to_cpu(hdr->magic) != MT7921_SOC_PHY_MAGIC ||
	    le16_to_cpu(tlv->tag) != MT7921_SOC_PHY_TAG_CAL ||
	    rsp->event != MT7921_SOC_PHY_FORCE_CAL_RSP || rsp->status) {
		dev_err(dev->mt76.dev,
			"calibration failed: magic 0x%08x tag %u event 0x%02x status %u\n",
			le32_to_cpu(hdr->magic), le16_to_cpu(tlv->tag),
			rsp->event, rsp->status);
		ret = -EIO;
		goto out;
	}

	dev_info(dev->mt76.dev,
		 "calibrated (result in EMI 0x%x+0x%x, temperature %u)\n",
		 le32_to_cpu(rsp->emi_addr), le32_to_cpu(rsp->emi_len),
		 le32_to_cpu(rsp->temperature));
out:
	dev_kfree_skb(skb);
	return ret;
}

/* the RAM code's release manifest, which the boot loader left in EMI */
static void mt7921_soc_read_manifest(struct mt7921_soc_dev *soc)
{
	struct mt792x_dev *dev = &soc->dev;
	char ver[MT7921_SOC_FW_VER_LEN + 1] = {};
	struct wiphy *wiphy = mt76_hw(dev)->wiphy;
	phys_addr_t base;
	__le32 *ptr;
	void *va;
	u32 size, off;
	size_t n;

	if (mtk_conninfra_get_emi(soc->ci, &base, &size) ||
	    size < MT7921_SOC_EMI_FW_VER_PTR + 4)
		return;

	ptr = memremap(base + MT7921_SOC_EMI_FW_VER_PTR, sizeof(*ptr),
		       MEMREMAP_WB);
	if (!ptr)
		return;
	off = le32_to_cpu(*ptr) & MT7921_SOC_EMI_ADDR_MASK;
	memunmap(ptr);

	if (!off || off + MT7921_SOC_FW_VER_LEN > size)
		return;

	va = memremap(base + off, MT7921_SOC_FW_VER_LEN, MEMREMAP_WB);
	if (!va)
		return;
	memcpy(ver, va, MT7921_SOC_FW_VER_LEN);
	memunmap(va);

	n = strnlen(ver, MT7921_SOC_FW_VER_LEN);
	if (!n)
		return;

	dev_info(dev->mt76.dev, "firmware: %s\n", ver);
	/* the tail (A-die and build date) is the useful part */
	strscpy(wiphy->fw_version,
		ver + (n >= sizeof(wiphy->fw_version) ?
		       n - sizeof(wiphy->fw_version) + 1 : 0),
		sizeof(wiphy->fw_version));
}

/*
 * MAC address: the stock NVRAM copy, else what the firmware reports
 * (efuse), else one derived from per-die A-die efuse words, so that it
 * stays the same across boots. A "local-mac-address" in DT overrides
 * all of them (mt76_eeprom_override()).
 */
static void mt7921_soc_set_mac(struct mt7921_soc_dev *soc)
{
	u8 *mac = soc->dev.mphy.macaddr;
	u32 uid[2], h0, h1;

	if (is_valid_ether_addr(soc->nvram_mac)) {
		ether_addr_copy(mac, soc->nvram_mac);
		return;
	}
	if (is_valid_ether_addr(mac))
		return;
	if (mtk_conninfra_adie_uid(soc->ci, uid))
		return;

	h0 = jhash_2words(uid[0], uid[1], 0x7961);
	h1 = jhash_2words(uid[1], uid[0], 0x6878);
	mac[0] = (h0 & 0xfc) | 0x02;	/* unicast, locally administered */
	mac[1] = h0 >> 8;
	mac[2] = h0 >> 16;
	mac[3] = h1;
	mac[4] = h1 >> 8;
	mac[5] = h1 >> 16;
	dev_info(soc->dev.mt76.dev, "MAC address %pM (from the A-die id)\n", mac);
}

/* calibration, FW_IMAGE_START, mcu ready, nic capabilities */
static int mt7921_soc_start_firmware(struct mt7921_soc_dev *soc)
{
	struct mt792x_dev *dev = &soc->dev;
	int err;

	if (phy_action) {
		err = mt7921_soc_phy_action(soc);
		if (err)
			dev_warn(dev->mt76.dev,
				 "no calibration before start (%d), continuing\n",
				 err);
	}

	/* dev->soc_axi: FW_IMAGE_START instead of the patch/RAM download */
	err = mt7921_run_firmware(dev);
	if (err)
		return err;

	mt7921_soc_read_manifest(soc);
	mt7921_soc_set_mac(soc);

	return 0;
}

static int mt7921_soc_mcu_init(struct mt792x_dev *dev)
{
	static const struct mt76_mcu_ops mt7921_soc_mcu_ops = {
		.headroom = sizeof(struct mt76_connac2_mcu_txd),
		.mcu_skb_send_msg = mt7921_soc_mcu_send_message,
		.mcu_parse_response = mt7921_mcu_parse_response,
	};
	int err;

	dev->mt76.mcu_ops = &mt7921_soc_mcu_ops;

	mt7921_soc_force_host_csr_irq(dev);
	err = __mt792xe_mcu_drv_pmctrl(dev);
	if (err)
		return err;

	return mt7921_soc_start_firmware(to_soc(dev));
}

static int mt7921_soc_init_reset(struct mt792x_dev *dev)
{
	return mt792x_wpdma_reset(dev, true);
}

/*
 * The stock "WF subsys reset": switch the WFSYS off and on again through
 * CONNINFRA (the WM MCU restarts from ROM, the RAM code is still in EMI),
 * then bring the WFDMA and the firmware back as mt7921e_mac_reset() does.
 */
static int mt7921_soc_wfsys_cycle(struct mt7921_soc_dev *soc)
{
	int ret;

	disable_irq(soc->irq);
	if (soc->irq_sw > 0)
		disable_irq(soc->irq_sw);

	WRITE_ONCE(soc->wfsys_on, false);
	mtk_conninfra_wfsys_off(soc->ci);
	ret = mtk_conninfra_wfsys_on(soc->ci);
	if (!ret)
		WRITE_ONCE(soc->wfsys_on, true);

	if (soc->irq_sw > 0)
		enable_irq(soc->irq_sw);
	enable_irq(soc->irq);

	return ret;
}

static int mt7921_soc_mac_reset(struct mt792x_dev *dev)
{
	struct mt7921_soc_dev *soc = to_soc(dev);
	int i, err;

	mt792xe_mcu_drv_pmctrl(dev);

	mt76_connac_free_pending_tx_skbs(&dev->pm, NULL);

	mt76_wr(dev, dev->irq_map->host_irq_enable, 0);

	set_bit(MT76_MCU_RESET, &dev->mphy.state);
	wake_up(&dev->mt76.mcu.wait);
	skb_queue_purge(&dev->mt76.mcu.res_q);

	mt76_txq_schedule_all(&dev->mphy);

	mt76_worker_disable(&dev->mt76.tx_worker);
	mt76_for_each_q_rx(&dev->mt76, i) {
		napi_disable(&dev->mt76.napi[i]);
	}
	napi_disable(&dev->mt76.tx_napi);

	mt76_connac2_tx_token_put(&dev->mt76);
	idr_init(&dev->mt76.token);

	err = mt7921_soc_wfsys_cycle(soc);
	if (err) {
		dev_err(dev->mt76.dev, "WFSYS power cycle failed: %d\n", err);
	} else {
		mt7921_soc_force_host_csr_irq(dev);
		mt792xe_mcu_fw_pmctrl(dev);
		err = __mt792xe_mcu_drv_pmctrl(dev);
		if (!err)
			mt792x_wpdma_reset(dev, true);
	}

	mt76_for_each_q_rx(&dev->mt76, i) {
		napi_enable(&dev->mt76.napi[i]);
	}

	local_bh_disable();
	mt76_for_each_q_rx(&dev->mt76, i) {
		napi_schedule(&dev->mt76.napi[i]);
	}
	local_bh_enable();

	dev->fw_assert = false;
	clear_bit(MT76_MCU_RESET, &dev->mphy.state);

	if (err)
		goto out;

	mt76_wr(dev, dev->irq_map->host_irq_enable,
		dev->irq_map->tx.all_complete_mask |
		MT_INT_RX_DONE_ALL | MT_INT_MCU_CMD);

	err = mt7921_soc_start_firmware(soc);
	if (err)
		goto out;

	err = mt7921_mcu_set_eeprom(dev);
	if (err)
		goto out;

	err = mt7921_mac_init(dev);
	if (err)
		goto out;

	err = __mt7921_start(&dev->phy);
out:
	napi_enable(&dev->mt76.tx_napi);
	local_bh_disable();
	napi_schedule(&dev->mt76.tx_napi);
	local_bh_enable();

	mt76_worker_enable(&dev->mt76.tx_worker);

	return err;
}

/*
 * WF2AP software interrupt (CCIF4): the firmware asks for a subsystem or
 * whole chip reset (after an assert), or announces new log data in EMI.
 */
static irqreturn_t mt7921_soc_sw_irq(int irq, void *data)
{
	struct mt792x_dev *dev = data;
	struct mt7921_soc_dev *soc = to_soc(dev);
	u32 status;

	if (test_bit(MT76_REMOVED, &dev->mt76.phy.state) ||
	    !READ_ONCE(soc->wfsys_on))
		return IRQ_NONE;

	status = mt76_rr(dev, MT7921_SOC_CCIF_RCHNUM);
	mt76_wr(dev, MT7921_SOC_CCIF_ACK, status);
	if (!status || status == U32_MAX)
		return IRQ_NONE;

	if (status & (MT7921_SOC_SW_INT_SUBSYS_RESET |
		      MT7921_SOC_SW_INT_WHOLE_RESET)) {
		dev_err(dev->mt76.dev, "firmware requests a %s reset (0x%x)\n",
			status & MT7921_SOC_SW_INT_WHOLE_RESET ? "chip" :
			"subsystem", status);
		mt792x_reset(&dev->mt76);
	}

	return IRQ_HANDLED;
}

static int mt7921_soc_dma_init(struct mt792x_dev *dev)
{
	int ret;

	mt76_dma_attach(&dev->mt76);

	ret = mt792x_dma_disable(dev, true);
	if (ret)
		return ret;

	/* tx data, band 0 */
	ret = mt76_connac_init_tx_queues(dev->phy.mt76, MT7921_TXQ_BAND0,
					 MT7921_TX_RING_SIZE,
					 MT_TX_RING_BASE, NULL, 0);
	if (ret)
		return ret;

	mt76_wr(dev, MT_WFDMA0_TX_RING0_EXT_CTRL, 0x4);

	/* command to WM */
	ret = mt76_init_mcu_queue(&dev->mt76, MT_MCUQ_WM, MT7921_SOC_TXQ_MCU_WM,
				  MT7921_TX_MCU_RING_SIZE, MT_TX_RING_BASE);
	if (ret)
		return ret;

	/* firmware download ring: unused, the boot loader loaded the firmware */
	ret = mt76_init_mcu_queue(&dev->mt76, MT_MCUQ_FWDL, MT7921_SOC_TXQ_FWDL,
				  MT7921_TX_FWDL_RING_SIZE, MT_TX_RING_BASE);
	if (ret)
		return ret;

	/* rx event and band 0 tx free done */
	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MCU],
			       MT7921_SOC_RXQ_EVENT,
			       MT7921_SOC_RX_EVENT_RING_SIZE,
			       MT_RX_BUF_SIZE, MT_RX_EVENT_RING_BASE);
	if (ret)
		return ret;

	/* band 1 tx free done */
	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MCU_WA],
			       MT7921_SOC_RXQ_TXDONE1,
			       MT7921_RX_MCU_WA_RING_SIZE,
			       MT_RX_BUF_SIZE, MT_RX_EVENT_RING_BASE);
	if (ret)
		return ret;

	/* rx data, band 0 */
	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MAIN],
			       MT7921_SOC_RXQ_DATA, MT7921_RX_RING_SIZE,
			       MT_RX_BUF_SIZE, MT_RX_EVENT_RING_BASE);
	if (ret)
		return ret;

	ret = mt76_init_queues(dev, mt792x_poll_rx);
	if (ret < 0)
		return ret;

	netif_napi_add_tx(dev->mt76.tx_napi_dev, &dev->mt76.tx_napi,
			  mt792x_poll_tx);
	napi_enable(&dev->mt76.tx_napi);

	return mt792x_dma_enable(dev);
}

static void mt7921_soc_power_off(struct mt7921_soc_dev *soc)
{
	if (soc->wfsys_on) {
		soc->wfsys_on = false;
		mtk_conninfra_wfsys_off(soc->ci);
	}
	if (soc->awake) {
		soc->awake = false;
		mtk_conninfra_sleep(soc->ci);
	}
	if (soc->radio_on) {
		soc->radio_on = false;
		mtk_conninfra_radio_off(soc->ci, MTK_CONNINFRA_WIFI);
	}
}

static int mt7921_soc_power_on(struct mt7921_soc_dev *soc)
{
	struct device *dev = soc->dev.mt76.dev;
	int ret;

	ret = mtk_conninfra_radio_on(soc->ci, MTK_CONNINFRA_WIFI);
	if (ret) {
		dev_err(dev, "CONNINFRA power on failed: %d\n", ret);
		return ret;
	}
	soc->radio_on = true;

	ret = mtk_conninfra_wfsys_on(soc->ci);
	if (ret) {
		dev_err(dev, "WFSYS power on failed: %d\n", ret);
		goto err;
	}
	if (!mtk_conninfra_wfsys_is_on(soc->ci)) {
		dev_err(dev, "WFSYS reported off after power on\n");
		ret = -EIO;
		goto err;
	}
	soc->wfsys_on = true;

	/*
	 * Keep the CONN_INFRA off domain awake while the driver is bound:
	 * the WFDMA host registers live there and the interrupt handler
	 * touches them with the core in fw own as well.
	 */
	ret = mtk_conninfra_wakeup(soc->ci);
	if (ret) {
		dev_err(dev, "CONN_INFRA wakeup failed: %d\n", ret);
		goto err;
	}
	soc->awake = true;

	return 0;

err:
	mt7921_soc_power_off(soc);
	return ret;
}

static void mt7921_soc_unregister_device(struct mt792x_dev *dev)
{
	struct mt76_connac_pm *pm = &dev->pm;
	int i;

	cancel_work_sync(&dev->init_work);
	mt76_unregister_device(&dev->mt76);
	mt76_for_each_q_rx(&dev->mt76, i)
		napi_disable(&dev->mt76.napi[i]);
	cancel_delayed_work_sync(&pm->ps_work);
	cancel_work_sync(&pm->wake_work);
	cancel_work_sync(&dev->reset_work);

	mt76_connac2_tx_token_put(&dev->mt76);
	__mt792xe_mcu_drv_pmctrl(dev);
	mt792x_dma_cleanup(dev);
	skb_queue_purge(&dev->mt76.mcu.res_q);

	tasklet_disable(&dev->mt76.irq_tasklet);
}

static int mt7921_soc_probe(struct platform_device *pdev)
{
	static const struct mt76_driver_ops drv_ops = {
		/* txwi_size = txd size + txp size */
		.txwi_size = MT_TXD_SIZE + sizeof(struct mt76_connac_hw_txp),
		.drv_flags = MT_DRV_TXWI_NO_FREE | MT_DRV_HW_MGMT_TXQ |
			     MT_DRV_AMSDU_OFFLOAD,
		.survey_flags = SURVEY_INFO_TIME_TX |
				SURVEY_INFO_TIME_RX |
				SURVEY_INFO_TIME_BSS_RX,
		.token_size = MT7921_TOKEN_SIZE,
		.tx_prepare_skb = mt7921_soc_tx_prepare_skb,
		.tx_complete_skb = mt76_connac_tx_complete_skb,
		.rx_check = mt7921_rx_check,
		.rx_skb = mt7921_queue_rx_skb,
		.rx_poll_complete = mt792x_rx_poll_complete,
		.sta_add = mt7921_mac_sta_add,
		.sta_event = mt7921_mac_sta_event,
		.sta_remove = mt7921_mac_sta_remove,
		.update_survey = mt792x_update_channel,
		.set_channel = mt7921_set_channel,
	};
	static const struct mt792x_hif_ops mt7921_soc_ops = {
		.init_reset = mt7921_soc_init_reset,
		.reset = mt7921_soc_mac_reset,
		.mcu_init = mt7921_soc_mcu_init,
		.drv_own = mt7921_soc_drv_own,
		.fw_own = mt792xe_mcu_fw_pmctrl,
	};
	static const struct mt792x_irq_map irq_map = {
		.host_irq_enable = MT_WFDMA0_HOST_INT_ENA,
		.tx = {
			.all_complete_mask = MT7921_SOC_INT_TX_DONE_ALL,
			.mcu_complete_mask = MT7921_SOC_INT_TX_DONE_MCU,
		},
		.rx = {
			.data_complete_mask = HOST_RX_DONE_INT_ENA0,
			.wm_complete_mask = HOST_RX_DONE_INT_ENA2,
			.wm2_complete_mask = HOST_RX_DONE_INT_ENA3,
		},
	};
	struct device *pdev_dev = &pdev->dev;
	struct mt7921_soc_dev *soc;
	struct ieee80211_ops *ops;
	struct mt76_bus_ops *bus_ops;
	struct mtk_conninfra *ci;
	struct mt792x_dev *dev;
	struct mt76_dev *mdev;
	struct resource *res;
	void __iomem *regs;
	u32 hvr, fvr;
	u8 features;
	int irq, ret;

	ci = devm_mtk_conninfra_get(pdev_dev);
	if (IS_ERR(ci))
		return dev_err_probe(pdev_dev, PTR_ERR(ci),
				     "failed to get CONNINFRA\n");

	/*
	 * The CONNSYS window is shared with CONNINFRA (and later Bluetooth):
	 * map it without claiming it.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) < MT7921_SOC_WINDOW)
		return dev_err_probe(pdev_dev, -EINVAL,
				     "need the 0x%x byte CONNSYS window\n",
				     MT7921_SOC_WINDOW);
	regs = devm_ioremap(pdev_dev, res->start, MT7921_SOC_WINDOW);
	if (!regs)
		return -ENOMEM;

	irq = platform_get_irq_byname(pdev, "wfdma");
	if (irq < 0)
		return irq;

	/*
	 * WFDMA rings: the pool LK carved out (memory-region, written by LK
	 * into this node). Without it the rings would land in memory the
	 * CONNSYS EMI MPU setup may not cover.
	 */
	ret = of_reserved_mem_device_init(pdev_dev);
	if (ret)
		return dev_err_probe(pdev_dev, ret,
				     "no WFDMA pool (memory-region from the boot loader)\n");

	ret = dma_set_mask(pdev_dev, DMA_BIT_MASK(36));
	if (!ret)
		ret = dma_set_coherent_mask(pdev_dev, DMA_BIT_MASK(32));
	if (ret)
		goto err_rmem;

	/*
	 * mt792x_get_mac80211_ops() reads the offload capabilities from the
	 * firmware file, which does not exist here: the image in EMI is a
	 * raw memory image without the feature tag. Default to no CNM
	 * (chanctx emulation, no ROC); fw_features overrides it.
	 */
	ops = devm_kmemdup(pdev_dev, &mt7921_ops, sizeof(*ops), GFP_KERNEL);
	if (!ops) {
		ret = -ENOMEM;
		goto err_rmem;
	}
	features = fw_features;
	if (!(features & MT792x_FW_CAP_CNM)) {
		ops->remain_on_channel = NULL;
		ops->cancel_remain_on_channel = NULL;
		ops->add_chanctx = ieee80211_emulate_add_chanctx;
		ops->remove_chanctx = ieee80211_emulate_remove_chanctx;
		ops->change_chanctx = ieee80211_emulate_change_chanctx;
		ops->switch_vif_chanctx = ieee80211_emulate_switch_vif_chanctx;
		ops->assign_vif_chanctx = NULL;
		ops->unassign_vif_chanctx = NULL;
		ops->mgd_prepare_tx = NULL;
		ops->mgd_complete_tx = NULL;
	}

	mdev = mt76_alloc_device(pdev_dev, sizeof(*soc), ops, &drv_ops);
	if (!mdev) {
		ret = -ENOMEM;
		goto err_rmem;
	}

	dev = container_of(mdev, struct mt792x_dev, mt76);
	soc = to_soc(dev);
	soc->ci = ci;
	soc->irq = irq;
	soc->irq_sw = platform_get_irq_byname_optional(pdev, "sw");
	platform_set_drvdata(pdev, soc);

	mt7921_soc_load_nvram(soc);

	dev->fw_features = features;
	dev->hif_ops = &mt7921_soc_ops;
	dev->irq_map = &irq_map;
	dev->soc_axi = true;
	mt76_mmio_init(&dev->mt76, regs);
	tasklet_init(&mdev->irq_tasklet, mt792x_irq_tasklet, (unsigned long)dev);

	dev->phy.dev = dev;
	dev->phy.mt76 = &dev->mt76.phy;
	dev->mt76.phy.priv = &dev->phy;
	dev->bus_ops = dev->mt76.bus;
	bus_ops = devm_kmemdup(pdev_dev, dev->bus_ops, sizeof(*bus_ops),
			       GFP_KERNEL);
	if (!bus_ops) {
		ret = -ENOMEM;
		goto err_free_dev;
	}

	bus_ops->rr = mt7921_soc_rr;
	bus_ops->wr = mt7921_soc_wr;
	bus_ops->rmw = mt7921_soc_rmw;
	dev->mt76.bus = bus_ops;

	/* CONNINFRA, WFSYS and the WM MCU up to its ROM idle loop */
	ret = mt7921_soc_power_on(soc);
	if (ret)
		goto err_free_dev;

	/*
	 * As soc7_0_McuInit(): the ROM may come up in driver own, hand it to
	 * the firmware first; then, as before every driver own on stock,
	 * force the CONN_INFRA host CSR interrupt enables and take it back.
	 */
	ret = mt792xe_mcu_fw_pmctrl(dev);
	if (ret)
		goto err_power_off;

	mt7921_soc_force_host_csr_irq(dev);
	ret = __mt792xe_mcu_drv_pmctrl(dev);
	if (ret)
		goto err_power_off;

	/* stock: "Chip ID[7961] Version[E1] HW[0x02040300] SW[0x00008a00]" */
	hvr = mt76_rr(dev, MT7921_SOC_TOP_HVR);
	fvr = mt76_rr(dev, MT7921_SOC_TOP_FVR);
	mdev->rev = (0x7961 << 16) | (fvr & 0xff);
	dev_info(mdev->dev, "SOC7_0 HW version 0x%08x, FW version 0x%08x, A-die 0x%08x\n",
		 hvr, fvr, mtk_conninfra_adie_id(ci));

	mt76_wr(dev, irq_map.host_irq_enable, 0);

	ret = devm_request_irq(pdev_dev, irq, mt792x_irq_handler, 0,
			       KBUILD_MODNAME, dev);
	if (ret)
		goto err_power_off;

	if (soc->irq_sw > 0) {
		mt76_wr(dev, MT7921_SOC_CCIF_ACK, mt76_rr(dev, MT7921_SOC_CCIF_RCHNUM));
		ret = devm_request_threaded_irq(pdev_dev, soc->irq_sw, NULL,
						mt7921_soc_sw_irq, IRQF_ONESHOT,
						"mt7921-soc-sw", dev);
		if (ret)
			goto err_free_irq;
	}

	ret = mt7921_soc_dma_init(dev);
	if (ret)
		goto err_free_irq;

	/*
	 * Runtime PM only hands the core to the firmware (fw own); CONN_INFRA
	 * stays awake, the WFDMA interrupt path needs it. No deep sleep.
	 */
	dev->pm.enable_user = runtime_pm;
	dev->pm.enable = runtime_pm;
	dev->pm.ds_enable_user = false;
	dev->pm.ds_enable = false;

	ret = mt7921_register_device(dev);
	if (ret)
		goto err_free_irq;

	return 0;

err_free_irq:
	if (soc->irq_sw > 0)
		devm_free_irq(pdev_dev, soc->irq_sw, dev);
	devm_free_irq(pdev_dev, irq, dev);
err_power_off:
	mt7921_soc_power_off(soc);
err_free_dev:
	mt76_free_device(&dev->mt76);
err_rmem:
	of_reserved_mem_device_release(pdev_dev);

	return ret;
}

static void mt7921_soc_remove(struct platform_device *pdev)
{
	struct mt7921_soc_dev *soc = platform_get_drvdata(pdev);
	struct mt792x_dev *dev = &soc->dev;

	mt7921_soc_unregister_device(dev);
	set_bit(MT76_REMOVED, &dev->mt76.phy.state);
	if (soc->irq_sw > 0)
		devm_free_irq(&pdev->dev, soc->irq_sw, dev);
	devm_free_irq(&pdev->dev, soc->irq, dev);
	mt7921_soc_power_off(soc);
	mt76_free_device(&dev->mt76);
	of_reserved_mem_device_release(&pdev->dev);
}

static const struct of_device_id mt7921_soc_of_match[] = {
	{ .compatible = "mediatek,mt6878-wifi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt7921_soc_of_match);

static struct platform_driver mt7921_soc_driver = {
	.driver = {
		.name = "mt7921-soc",
		.of_match_table = mt7921_soc_of_match,
	},
	.probe = mt7921_soc_probe,
	.remove = mt7921_soc_remove,
};
module_platform_driver(mt7921_soc_driver);

MODULE_DESCRIPTION("MediaTek MT7921 (MT6878 CONNSYS) AXI wireless driver");
MODULE_LICENSE("Dual BSD/GPL");
