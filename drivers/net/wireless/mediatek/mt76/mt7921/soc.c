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
 * not download firmware: it sends FW_IMAGE_START and waits for the WM CPU.
 *
 * Skeleton: no runtime PM, no suspend, no chip reset, no coredump, no
 * CCIF (second interrupt), no NVRAM calibration data.
 */

#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/mtk-conninfra.h>

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
#define MT7921_SOC_HOST_CSR_IRQ_BITS	(BIT(1) | BIT(4) | BIT(8))

/* WF_MCU_CFG_LS: hardware version (CONNAC2X_TOP_HVR of SOC7_0) */
#define MT7921_SOC_TOP_HVR		0x88000000

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
	bool radio_on;
	bool awake;
	bool wfsys_on;			/* gates every register access */
};

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

	mdev->mcu.timeout = 3 * HZ;

	if (cmd == MCU_CMD(FW_SCATTER))
		txq = MT_MCUQ_FWDL;

	return mt76_tx_queue_skb_raw(dev, mdev->q_mcu[txq], skb, 0);
}

/* driver own through CONN_HOST_CSR_TOP LPCR (0x1806_0010), as on PCIe */
static int mt7921_soc_driver_own(struct mt792x_dev *dev)
{
	mt76_wr(dev, MT_TOP_LPCR_HOST_BAND0, MT_TOP_LPCR_HOST_DRV_OWN);
	if (!mt76_poll_msec(dev, MT_TOP_LPCR_HOST_BAND0,
			    MT_TOP_LPCR_HOST_FW_OWN, 0, 500)) {
		dev_err(dev->mt76.dev, "Timeout for driver own\n");
		return -EIO;
	}

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

	err = mt7921_soc_driver_own(dev);
	if (err)
		return err;

	/* dev->soc_axi: FW_IMAGE_START instead of the patch/RAM download */
	return mt7921_run_firmware(dev);
}

static int mt7921_soc_init_reset(struct mt792x_dev *dev)
{
	return mt792x_wpdma_reset(dev, true);
}

static int mt7921_soc_mac_reset(struct mt792x_dev *dev)
{
	/*
	 * TODO: chip reset needs a WFSYS power cycle through CONNINFRA
	 * (stock: conninfra whole-chip / subsys reset); not implemented.
	 */
	dev_warn(dev->mt76.dev, "chip reset is not supported yet\n");

	return -EOPNOTSUPP;
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
	 * the WFDMA host registers live there. TODO: sleep with fw own.
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
		.drv_own = mt792xe_mcu_drv_pmctrl,
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
	u32 val;
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
	 * firmware file, which does not exist here (the image is in EMI).
	 * TODO: read them from the release info the boot loader placed in
	 * EMI; until then run without CNM (chanctx emulation, no ROC).
	 */
	ops = devm_kmemdup(pdev_dev, &mt7921_ops, sizeof(*ops), GFP_KERNEL);
	if (!ops) {
		ret = -ENOMEM;
		goto err_rmem;
	}
	features = 0;
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

	mdev = mt76_alloc_device(pdev_dev, sizeof(*soc), ops, &drv_ops);
	if (!mdev) {
		ret = -ENOMEM;
		goto err_rmem;
	}

	dev = container_of(mdev, struct mt792x_dev, mt76);
	soc = to_soc(dev);
	soc->ci = ci;
	soc->irq = irq;
	platform_set_drvdata(pdev, soc);

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

	/* stock: CONN_INFRA host CSR interrupts 1, 4 and 8 must be enabled */
	val = mt76_rr(dev, MT7921_SOC_HOST_CSR_IRQ_EN);
	if ((val & MT7921_SOC_HOST_CSR_IRQ_BITS) != MT7921_SOC_HOST_CSR_IRQ_BITS) {
		dev_info(pdev_dev, "enabling CONN_INFRA host CSR irqs (0x%08x)\n",
			 val);
		mt76_wr(dev, MT7921_SOC_HOST_CSR_IRQ_EN,
			val | MT7921_SOC_HOST_CSR_IRQ_BITS);
	}

	ret = mt792xe_mcu_fw_pmctrl(dev);
	if (ret)
		goto err_power_off;

	ret = __mt792xe_mcu_drv_pmctrl(dev);
	if (ret)
		goto err_power_off;

	val = mt76_rr(dev, MT7921_SOC_TOP_HVR);
	mdev->rev = (0x7961 << 16) | (val & 0xff);
	dev_info(mdev->dev, "SOC7_0 HW version 0x%08x, A-die 0x%08x\n", val,
		 mtk_conninfra_adie_id(ci));

	mt76_wr(dev, irq_map.host_irq_enable, 0);

	ret = devm_request_irq(pdev_dev, irq, mt792x_irq_handler, 0,
			       KBUILD_MODNAME, dev);
	if (ret)
		goto err_power_off;

	/* TODO: "sw" interrupt (WF2AP CCIF: fw log, coredump, time sync) */

	ret = mt7921_soc_dma_init(dev);
	if (ret)
		goto err_free_irq;

	ret = mt7921_register_device(dev);
	if (ret)
		goto err_free_irq;

	/*
	 * No firmware own / deep sleep yet: going to fw own lets CONN_INFRA
	 * sleep, which the CONNINFRA driver does not handle yet.
	 */
	dev->pm.enable_user = false;
	dev->pm.enable = false;
	dev->pm.ds_enable_user = false;
	dev->pm.ds_enable = false;

	return 0;

err_free_irq:
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
