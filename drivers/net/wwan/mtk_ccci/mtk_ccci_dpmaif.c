// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: DPMAIF v3 data path, hardware setup.
 *
 * DPMAIF moves network packets between the AP and the modem with DMA
 * descriptor rings in AP memory. It is the block the t7xx driver drives
 * over PCIe (same registers, same descriptors); here it is on the SoC
 * bus, set up the way the MediaTek downstream driver sets it up on
 * MT6878 (ccci_dpmaif_drv_v3.c, capability 4: no receive offload):
 *
 *  - DL (modem -> AP), one RX queue: a BAT ring of buffers the AP lends
 *    the hardware, a fragment BAT ring for the rest of large packets and
 *    a PIT ring where the hardware reports what it put where (message
 *    PIT: CCMNI channel, i.e. which network device, and checksum status;
 *    normal PIT: buffer id and length);
 *  - UL (AP -> modem), 4 TX queues: a DRB ring of a message DRB (channel,
 *    length) followed by payload DRBs (address, length).
 *
 * The block is set up before the modem is released and reset after the
 * modem is powered off, as the downstream driver does it.
 *
 * Receive follows t7xx (t7xx_hif_dpmaif_rx.c): NAPI on the one RX queue
 * reads the PITs up to the hardware write index, checks their sequence
 * numbers, puts a packet together from its skb buffer and fragment pages
 * and hands it to the WWAN link of its CCMNI channel; the buffers handed
 * back are lent again. A buffer slot is lent again only once its PIT has
 * been read, so the hardware never writes into a buffer the AP owns, and
 * all buffers are freed only after the block is reset.
 *
 * Transmit follows the downstream driver and t7xx_hif_dpmaif_tx.c: a
 * packet is a message DRB (CCMNI channel, length) and one payload DRB
 * (the links do not do scatter-gather), on TX queue 0, or on queue 1 for
 * a TCP segment that only acknowledges, as the downstream CCMNI does. The
 * hardware read index tells which packets it took; they are released
 * from a BH work item on the TX done interrupt, and the links are woken
 * when a full queue drains.
 *
 * Nothing here runs unless the module parameter data=1.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2021 Intel Corporation (t7xx DPMAIF)
 * Copyright (C) 2026 serlk
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/netdevice.h>
#include <linux/rcupdate.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/skbuff.h>
#include <linux/tcp.h>
#include <linux/workqueue.h>
#include <net/ip.h>

#include "mtk_ccci.h"

static bool data;
module_param(data, bool, 0444);
MODULE_PARM_DESC(data,
		 "set up DPMAIF with the modem: mobile data (default: no)");

#define DPMAIF_TXQ_NUM		4
#define DPMAIF_BAT_CNT		16384
#define DPMAIF_FRG_CNT		16384
#define DPMAIF_PIT_CNT		(DPMAIF_BAT_CNT * 2)
#define DPMAIF_DRB_CNT		2048

/* buffers lent to the hardware at a time (downstream: up to the ring size) */
#define DPMAIF_BAT_LEND		2048
#define DPMAIF_FRG_LEND		1024

/* DL settings of the downstream driver */
#define DPMAIF_BAT_BUF_SIZE	(128 * 13)	/* a BAT buffer */
#define DPMAIF_FRG_BUF_SIZE	(128 * 15)	/* a fragment BAT buffer */
#define DPMAIF_BAT_REMAIN	64	/* less left in a buffer: next one */
#define DPMAIF_PKT_BIDCNT	1	/* packets in one buffer */
#define DPMAIF_HW_MTU		(3 * 1024 + 8)
#define DPMAIF_CHK_BAT_NUM	62	/* fewer free BATs: length error */
#define DPMAIF_CHK_FRG_NUM	62
#define DPMAIF_CHK_PIT_NUM	(DPMAIF_CHK_BAT_NUM * 2)
#define DPMAIF_CHK_RB_PIT_NUM	64

/* UL: TCP ACKs on their own queue, as the downstream CCMNI does */
#define DPMAIF_TXQ_DATA		0
#define DPMAIF_TXQ_ACK		1
#define DPMAIF_TX_DRBS		2	/* message + payload */

/* register poll limits; the downstream driver spins forever */
#define DPMAIF_UL_POLL_US	1000
#define DPMAIF_POLL_US		10000
#define DPMAIF_IDLE_POLL_US	100000

/*
 * "pd" window (0x1022d000): UL +0x000, DL +0x100, AP misc +0x400 and the
 * SRAM copies of the AO settings: DL +0xc00, UL +0xd00, misc +0xe00.
 */
#define PD_UL_DBG_STA2		0x088
#define PD_UL_ADD_DESC_CH(q)	(0x0b0 + 4 * (q))	/* DRB words added */
#define  UL_ADD_UPDATE		BIT(31)	/* reads 1 while the update runs */
#define  UL_DBG_IDLE		BIT(11)
#define PD_DL_BAT_INIT		0x100
#define PD_DL_BAT_ADD		0x104	/* buffers lent, BIT(16): fragment */
#define PD_DL_BAT_INIT_CON0	0x108	/* BAT base 31:0 */
#define PD_DL_BAT_INIT_CON1	0x10c
#define  BAT_CON1_CNT		GENMASK(15, 0)
#define  BAT_CON1_EN		BIT(16)
#define  BAT_CON1_CACHE_PRI	BIT(22)
#define  BAT_CON1_ADDR_HI	GENMASK(31, 24)
#define PD_DL_PIT_INIT		0x120
#define PD_DL_PIT_ADD		0x124	/* PITs read */
#define PD_DL_PIT_INIT_CON0	0x128	/* PIT base 31:0 */
#define PD_DL_PIT_INIT_CON1	0x12c
#define  PIT_CON1_CNT		GENMASK(17, 0)
#define  PIT_CON1_ADDR_HI	GENMASK(31, 24)
#define PD_DL_PIT_INIT_CON3	0x134
#define  PIT_CON3_EN		BIT(0)
#define PD_DL_RESERVE_RW	0x1a8	/* 0 after the PD domain lost power */
#define PD_DL_DBG_STA1		0x1b4
#define  DL_DBG_IDLE		BIT(23)
/* BAT_INIT, PIT_INIT */
#define  DL_INIT_ALLSET		BIT(0)
#define  DL_INIT_FRG		BIT(16)
#define  DL_INIT_EN		BIT(31)	/* reads 1 while the update runs */
/* BAT_ADD, PIT_ADD */
#define  DL_ADD_FRG		BIT(16)
#define  DL_ADD_UPDATE		BIT(31)	/* reads 1 while the update runs */

#define PD_AP_UL_L2TISAR0	0x400	/* UL interrupt status, write 1 to clear */
#define PD_AP_L1TISAR0		0x410
#define PD_AP_DMA_ERR_STA	0x440
#define PD_AP_DL_L2TISAR0	0x450	/* DL interrupt status */
#define PD_AP_IP_BUSY		0x460
#define PD_AP_CG_EN		0x468
#define PD_AP_CODA_VER		0x46c
#define PD_AP_OVERWRITE_CFG	0x490	/* bit 0: sync the settings to SRAM */
#define PD_AP_MEM_CLR		0x494	/* bit 0: clear the SRAM */

#define PD_DL_PKTINFO_CON0	0xc00
#define  PKT0_CHK_RB_PIT	GENMASK(7, 0)
#define  PKT0_REMAIN_MINSZ	GENMASK(15, 8)	/* in 16 bytes */
#define  PKT0_BID_MAXCNT	GENMASK(31, 16)
#define PD_DL_PKTINFO_CON1	0xc04	/* MTU */
#define PD_DL_PKTINFO_CON2	0xc08
#define  PKT2_BAT_RSV_LEN	GENMASK(7, 0)
#define  PKT2_BAT_BUF_SZ	GENMASK(16, 8)	/* in 128 bytes */
#define  PKT2_PIT_CHK_NUM	GENMASK(31, 24)
#define PD_DL_RDY_CHK_THRES	0xc0c
#define  RDY_PIT_BURST		BIT(13)
#define  RDY_BAT_CHK_THRES	GENMASK(21, 16)
#define  RDY_PKT_ALIGN		GENMASK(23, 22)	/* 23: on, 22: 128 bytes */
#define  RDY_PKT_ALIGN_64	BIT(23)
#define  RDY_CHKSUM		BIT(31)
#define PD_DL_RDY_CHK_FRG_THRES	0xc10
#define  FRG_CHK_THRES		GENMASK(7, 0)
#define  FRG_BUF_SZ		GENMASK(15, 8)	/* in 128 bytes */
#define  FRG_FEATURE_EN		BIT(28)
#define PD_DL_PIT_WR_IDX	0xc60
#define  DL_PIT_IDX		GENMASK(17, 0)
#define PD_DL_FRG_RD_IDX	0xc78
#define PD_DL_BAT_RD_IDX	0xcd8
#define PD_DL_BAT_WR_IDX	0xcdc
#define PD_DL_PIT_RD_IDX	0xcec
#define PD_UL_CHNL_CON0(q)	(0xd10 + 0x10 * (q))	/* DRB base 31:0 */
#define PD_UL_CHNL_CON1(q)	(0xd14 + 0x10 * (q))
#define  DRB_CON1_SIZE		GENMASK(15, 0)	/* in 32-bit words */
#define  DRB_CON1_ADDR_HI	GENMASK(31, 24)
#define PD_UL_CH_STA(q)		(0xde0 + 4 * (q))
#define  UL_STA_RD_IDX		GENMASK(31, 16)	/* in 32-bit words */
#define PD_MISC_REMAP(i)	(0xe0c + 4 * (i))	/* 8 registers, 3 windows each */

/* "ao" window (0x10014000): AO UL +0x000, AO DL +0x400 */
#define AO_UL_INIT_SET		0x000
#define AO_UL_CHNL_ARB0		0x01c
#define  ARB0_RDY(q)		BIT(q)
#define  ARB0_ARB(q)		BIT((q) + 8)
#define  ARB0_ARB_ALL		GENMASK(11, 8)
#define AO_UL_AP_L2TIMR0	0x080	/* UL interrupt mask, 1 = masked */
#define AO_UL_AP_L2TIMCR0	0x084	/* write 1: unmask */
#define AO_UL_AP_L2TIMSR0	0x088	/* write 1: mask */
#define AO_UL_AP_L1TIMR0	0x08c
#define AO_UL_APDL_L2TIMR0	0x090	/* DL interrupt mask */
#define AO_UL_APDL_L2TIMCR0	0x094
#define AO_UL_APDL_L2TIMSR0	0x098
#define AO_UL_IP_BUSY_MASK	0x09c
#define AO_DL_INIT_SET		0x400

/* UL interrupts */
#define UL_INT_QDONE		GENMASK(3, 0)	/* per TX queue */
#define UL_INT_PWR_NOTRDY	GENMASK(19, 15)	/* the modem is asleep */
#define UL_INT_LEN_ERR		GENMASK(24, 20)
#define UL_INT_EN		(UL_INT_QDONE | UL_INT_PWR_NOTRDY | UL_INT_LEN_ERR)
/* DL interrupts */
#define DL_INT_QDONE		BIT(0)
#define DL_INT_MTU_ERR		BIT(6)
#define DL_INT_EN		(DL_INT_QDONE | DL_INT_MTU_ERR)
/* L1 sources the downstream driver masks */
#define L1_INT_MASK		(BIT(9) | BIT(10) | BIT(15) | BIT(16))

/* infracfg_ao */
#define INFRA_AO_DPMAIF_CTRL	0x208
#define  INFRA_AO_DPMAIF_EN	BIT(15)	/* set before the setup, cleared before reset */
#define INFRA_AO_DPMAIF_WAKE	0xc00
#define  INFRA_AO_DPMAIF_IP_BUSY_MASK	BIT(12)
#define  INFRA_AO_DPMAIF_MD_WAKE_MASK	BIT(13)	/* the modem-to-AP wakeup event */
#define INFRA_AO_RST_SET	0x130
#define INFRA_AO_RST_CLR	0x134
#define  INFRA_AO_DPMAIF_RST	BIT(0)
/* infracfg (the modem node's "infra" window), shared with the CCIF reset */
#define INFRA_RST_SET		0xf50
#define INFRA_RST_CLR		0xf54
#define  INFRA_DPMAIF_PD_RST	BIT(22)

/*
 * Modem bank remap: the modem side of DPMAIF sees the modem memory (banks
 * 0 and 1, 8 windows of 32 MiB each) at modem addresses, the remap gives
 * the AP address of each window. A window field is bits 33:25 of its AP
 * address, bit 0 of the field enables it.
 */
#define DPMAIF_REMAP_WIN	SZ_32M
#define DPMAIF_REMAP_ON		16	/* banks 0 and 1; bank 4 stays off */
#define DPMAIF_REMAP_FIELD	GENMASK_ULL(33, 25)

/* normal PIT, v3 (16 bytes) */
#define PIT0_PACKET_TYPE	BIT(0)		/* 0: data, 1: message */
#define PIT0_CONT		BIT(1)		/* more PITs for this packet */
#define PIT0_BUF_TYPE		BIT(2)		/* 1: fragment BAT */
#define PIT0_BUF_ID		GENMASK(15, 3)
#define PIT0_DATA_LEN		GENMASK(31, 16)
/* word 1: buffer address low, word 2: high */
#define PIT3_SEQ		GENMASK(7, 0)
#define PIT3_H_BID		GENMASK(10, 8)	/* buffer id bits 15:13 */
#define PIT3_ULQ_DONE		GENMASK(29, 24)
#define PIT3_DLQ_DONE		GENMASK(31, 30)
#define PIT_SEQ_MAX		0xfe		/* without receive offload */

/* message PIT, v3 */
#define MPIT0_CHECKSUM		GENMASK(3, 2)	/* 0: checked good */
#define MPIT0_ERROR		BIT(4)
#define MPIT0_SRC_QID		GENMASK(7, 5)
#define MPIT0_CHANNEL_ID	GENMASK(23, 16)	/* CCMNI index */
#define MPIT0_NETWORK_TYPE	GENMASK(26, 24)
#define MPIT0_DROP		BIT(31)
#define MPIT1_COUNT_L		GENMASK(15, 0)

struct dpmaif_pit {
	__le32 w[4];
};

/* BAT (8 bytes): buffer address low, high byte */
struct dpmaif_bat {
	__le32 addr_lo;
	__le32 addr_hi;
};

/* what is lent in a BAT slot */
struct dpmaif_bat_skb {
	struct sk_buff *skb;
	dma_addr_t dma;
};

struct dpmaif_bat_frg {
	void *data;			/* page fragment */
	dma_addr_t dma;
};

/* DRB (8 bytes) */
#define DRB0_TYPE		GENMASK(1, 0)	/* 0: payload, 1: message */
#define DRB0_CONT		BIT(2)
#define DRB0_ADDR_EXT		GENMASK(15, 8)	/* payload: address bits 39:32 */
#define DRB0_DATA_LEN		GENMASK(31, 16)	/* payload / message: length */
#define DRB1_COUNT_L		GENMASK(15, 0)	/* message: priority count */
#define DRB1_CHANNEL_ID		GENMASK(23, 16)	/* message: CCMNI index */
#define DRB1_NETWORK_TYPE	GENMASK(26, 24)
#define DRB1_IPV4_CSUM		BIT(28)
#define DRB1_L4_CSUM		BIT(29)

struct dpmaif_drb {
	__le32 w[2];
};

/* the packet behind a payload DRB */
struct dpmaif_drb_skb {
	struct sk_buff *skb;
	dma_addr_t dma;
	u32 len;
};

struct dpmaif_txq {
	spinlock_t lock;		/* the indexes, on, the DRB ring */
	bool on;			/* takes packets */
	struct dpmaif_drb_skb *skbs;	/* by DRB slot */
	u32 wr;				/* next DRB to fill */
	u32 rel;			/* next DRB to release */
	unsigned long pkts;
	unsigned long full;
	unsigned long add_err;
	unsigned long rd_err;
};

struct dpmaif_ring {
	void *base;
	dma_addr_t dma;
	u32 cnt;
};

static const char * const dpmaif_clk_names[] = { "main", "blk", "26m" };

struct mtk_ccci_dpmaif {
	struct mtk_ccci *ccci;
	struct device *dev;		/* the modem device, for DMA */
	void __iomem *ao;
	void __iomem *pd;
	struct regmap *infracfg_ao;
	struct clk_bulk_data clks[ARRAY_SIZE(dpmaif_clk_names)];
	int irq;
	u64 md_base;			/* AP address of modem bank 0 */

	struct mutex lock;		/* started, clocks, register access */
	bool started;			/* set up, clocks held */
	struct dpmaif_ring bat;
	struct dpmaif_ring frg;
	struct dpmaif_ring pit;
	struct dpmaif_ring drb[DPMAIF_TXQ_NUM];

	/* DL, only touched by NAPI while started */
	struct net_device *napi_dev;
	struct napi_struct napi;
	struct dpmaif_bat_skb *bat_skb;	/* by BAT slot */
	struct dpmaif_bat_frg *frg_buf;	/* by fragment BAT slot */
	u32 bat_wr;			/* next slot to lend */
	u32 bat_lent;
	u32 frg_wr;
	u32 frg_lent;
	u32 pit_rd;
	u8 pit_seq;			/* the one expected next */
	bool rx_retry;			/* a PIT not written yet, poll again */
	/* the packet being put together */
	struct sk_buff *rx_skb;
	bool rx_msg;			/* its message PIT seen */
	bool rx_drop;
	bool rx_csum_ok;
	u8 rx_ch;

	/* UL */
	struct dpmaif_txq txq[DPMAIF_TXQ_NUM];
	struct work_struct tx_work;
	atomic_t tx_pending;		/* TX queues with done interrupts */

	/* DL counters: per CCMNI channel, and what was dropped */
	unsigned long dl_pkts[MTK_CCCI_NET_LINKS];
	unsigned long dl_nolink[MTK_CCCI_NET_LINKS];
	unsigned long dl_bad_ch;
	u8 dl_last_bad_ch;
	unsigned long dl_dropped;	/* the modem said so */
	unsigned long dl_err;		/* PIT or buffer inconsistent */
	unsigned long pit_seq_err;
	unsigned long bat_nomem;

	/* interrupt counters */
	unsigned long irqs;
	unsigned long ul_done;
	unsigned long ul_pwr_notrdy;
	unsigned long ul_len_err;
	unsigned long dl_done;
	unsigned long dl_mtu_err;
	u32 last_ul_sta;
	u32 last_dl_sta;
};

static void dpmaif_rmem_release(void *dev)
{
	of_reserved_mem_device_release(dev);
}

static int dpmaif_ring_alloc(struct device *dev, struct dpmaif_ring *r,
			     u32 cnt, size_t entry)
{
	r->cnt = cnt;
	r->base = dmam_alloc_coherent(dev, cnt * entry, &r->dma, GFP_KERNEL);
	return r->base ? 0 : -ENOMEM;
}

static void dpmaif_rmw(void __iomem *reg, u32 mask, u32 val)
{
	writel((readl(reg) & ~mask) | (val & mask), reg);
}

/* wait for a self-clearing bit */
static int dpmaif_wait_clear(struct mtk_ccci_dpmaif *dp, void __iomem *reg,
			     u32 bit, const char *what)
{
	u32 v;
	int ret;

	ret = readl_poll_timeout_atomic(reg, v, !(v & bit), 1, DPMAIF_POLL_US);
	if (ret)
		dev_err(dp->dev, "DPMAIF %s stuck: %#x\n", what, v);
	return ret;
}

/* the DL "init" handshake: wait idle, write, wait for the update */
static int dpmaif_dl_init_cmd(struct mtk_ccci_dpmaif *dp, u32 reg, u32 cmd,
			      const char *what)
{
	int ret;

	ret = dpmaif_wait_clear(dp, dp->pd + reg, DL_INIT_EN, what);
	if (ret)
		return ret;
	writel(cmd, dp->pd + reg);
	return dpmaif_wait_clear(dp, dp->pd + reg, DL_INIT_EN, what);
}

static void dpmaif_remap(struct mtk_ccci_dpmaif *dp)
{
	unsigned int i, j, w;
	u32 v;

	for (i = 0; i < 8; i++) {
		v = 0;
		for (j = 0; j < 3; j++) {
			w = i * 3 + j;
			v |= FIELD_GET(DPMAIF_REMAP_FIELD, dp->md_base +
				       (u64)w * DPMAIF_REMAP_WIN) << (10 * j + 1);
			if (w < DPMAIF_REMAP_ON)
				v |= BIT(10 * j);
		}
		writel(v, dp->pd + PD_MISC_REMAP(i));
	}
}

/* unmask the wanted interrupts, wait until the mask shows it */
static int dpmaif_irq_setup(struct mtk_ccci_dpmaif *dp, u32 sta, u32 mask,
			    u32 clr, u32 set, u32 en, const char *what)
{
	u32 v;
	int ret;

	writel(~0U, dp->pd + sta);
	writel(en, dp->ao + clr);
	writel(~en, dp->ao + set);
	ret = readl_poll_timeout_atomic(dp->ao + mask, v, (v & en) != en, 1,
					DPMAIF_POLL_US);
	if (ret)
		dev_err(dp->dev, "DPMAIF %s interrupts stay masked: %#x\n",
			what, v);
	return ret;
}

/* the common part: SRAM, clock gating, bank remap, interrupts */
static int dpmaif_hw_common(struct mtk_ccci_dpmaif *dp)
{
	int ret;

	writel(readl(dp->pd + PD_AP_MEM_CLR) | BIT(0), dp->pd + PD_AP_MEM_CLR);
	ret = dpmaif_wait_clear(dp, dp->pd + PD_AP_MEM_CLR, BIT(0),
				"SRAM clear");
	if (ret)
		return ret;

	writel(0x7f, dp->pd + PD_AP_CG_EN);
	dpmaif_remap(dp);
	writel(L1_INT_MASK, dp->ao + AO_UL_AP_L1TIMR0);
	/* reads 0 again once the PD domain lost its state */
	writel(0xff, dp->pd + PD_DL_RESERVE_RW);

	ret = dpmaif_irq_setup(dp, PD_AP_UL_L2TISAR0, AO_UL_AP_L2TIMR0,
			       AO_UL_AP_L2TIMCR0, AO_UL_AP_L2TIMSR0, UL_INT_EN,
			       "UL");
	if (ret)
		return ret;
	ret = dpmaif_irq_setup(dp, PD_AP_DL_L2TISAR0, AO_UL_APDL_L2TIMR0,
			       AO_UL_APDL_L2TIMCR0, AO_UL_APDL_L2TIMSR0,
			       DL_INT_EN, "DL");
	if (ret)
		return ret;

	writel(~0U, dp->pd + PD_AP_IP_BUSY);
	writel(0, dp->ao + AO_UL_IP_BUSY_MASK);
	return 0;
}

/* RX queue 0: packet parameters and the PIT ring */
static int dpmaif_hw_rxq(struct mtk_ccci_dpmaif *dp)
{
	void __iomem *pd = dp->pd;

	dpmaif_rmw(pd + PD_DL_PKTINFO_CON0,
		   PKT0_REMAIN_MINSZ | PKT0_BID_MAXCNT | PKT0_CHK_RB_PIT,
		   FIELD_PREP(PKT0_REMAIN_MINSZ, DPMAIF_BAT_REMAIN / 16) |
		   FIELD_PREP(PKT0_BID_MAXCNT, DPMAIF_PKT_BIDCNT) |
		   FIELD_PREP(PKT0_CHK_RB_PIT, DPMAIF_CHK_RB_PIT_NUM));
	dpmaif_rmw(pd + PD_DL_RDY_CHK_THRES, RDY_PKT_ALIGN, RDY_PKT_ALIGN_64);
	writel(DPMAIF_HW_MTU, pd + PD_DL_PKTINFO_CON1);
	dpmaif_rmw(pd + PD_DL_PKTINFO_CON2, PKT2_PIT_CHK_NUM,
		   FIELD_PREP(PKT2_PIT_CHK_NUM, DPMAIF_CHK_PIT_NUM));
	/* BAT cache and PIT burst, "performance" in the downstream driver */
	dpmaif_rmw(pd + PD_DL_BAT_INIT_CON1, BAT_CON1_CACHE_PRI,
		   BAT_CON1_CACHE_PRI);
	dpmaif_rmw(pd + PD_DL_RDY_CHK_THRES, RDY_PIT_BURST, RDY_PIT_BURST);

	writel(lower_32_bits(dp->pit.dma), pd + PD_DL_PIT_INIT_CON0);
	dpmaif_rmw(pd + PD_DL_PIT_INIT_CON1, PIT_CON1_ADDR_HI | PIT_CON1_CNT,
		   FIELD_PREP(PIT_CON1_ADDR_HI, upper_32_bits(dp->pit.dma)) |
		   FIELD_PREP(PIT_CON1_CNT, dp->pit.cnt));
	dpmaif_rmw(pd + PD_DL_PIT_INIT_CON3, PIT_CON3_EN, PIT_CON3_EN);
	return dpmaif_dl_init_cmd(dp, PD_DL_PIT_INIT,
				  DL_INIT_EN | DL_INIT_ALLSET, "PIT init");
}

static int dpmaif_hw_bat(struct mtk_ccci_dpmaif *dp, struct dpmaif_ring *r,
			 bool frg)
{
	void __iomem *pd = dp->pd;

	writel(lower_32_bits(r->dma), pd + PD_DL_BAT_INIT_CON0);
	dpmaif_rmw(pd + PD_DL_BAT_INIT_CON1,
		   BAT_CON1_ADDR_HI | BAT_CON1_CNT | BAT_CON1_EN,
		   FIELD_PREP(BAT_CON1_ADDR_HI, upper_32_bits(r->dma)) |
		   FIELD_PREP(BAT_CON1_CNT, r->cnt));
	return dpmaif_dl_init_cmd(dp, PD_DL_BAT_INIT,
				  DL_INIT_EN | DL_INIT_ALLSET |
				  (frg ? DL_INIT_FRG : 0),
				  frg ? "frag BAT init" : "BAT init");
}

/* both BAT rings, queue left disabled: no buffer is lent yet */
static int dpmaif_hw_bats(struct mtk_ccci_dpmaif *dp)
{
	void __iomem *pd = dp->pd;
	int ret;

	dpmaif_rmw(pd + PD_DL_PKTINFO_CON2, PKT2_BAT_BUF_SZ | PKT2_BAT_RSV_LEN,
		   FIELD_PREP(PKT2_BAT_BUF_SZ, DPMAIF_BAT_BUF_SIZE / 128));
	dpmaif_rmw(pd + PD_DL_RDY_CHK_THRES, RDY_BAT_CHK_THRES,
		   FIELD_PREP(RDY_BAT_CHK_THRES, DPMAIF_CHK_BAT_NUM));
	ret = dpmaif_hw_bat(dp, &dp->bat, false);
	if (ret)
		return ret;

	dpmaif_rmw(pd + PD_DL_RDY_CHK_FRG_THRES,
		   FRG_FEATURE_EN | FRG_BUF_SZ | FRG_CHK_THRES,
		   FRG_FEATURE_EN |
		   FIELD_PREP(FRG_BUF_SZ, DPMAIF_FRG_BUF_SIZE / 128) |
		   FIELD_PREP(FRG_CHK_THRES, DPMAIF_CHK_FRG_NUM));
	ret = dpmaif_hw_bat(dp, &dp->frg, true);
	if (ret)
		return ret;

	/* the hardware checks the DL checksums */
	dpmaif_rmw(pd + PD_DL_RDY_CHK_THRES, RDY_CHKSUM, RDY_CHKSUM);
	return 0;
}

static void dpmaif_hw_txqs(struct mtk_ccci_dpmaif *dp)
{
	unsigned int q;

	for (q = 0; q < DPMAIF_TXQ_NUM; q++) {
		struct dpmaif_ring *r = &dp->drb[q];

		dpmaif_rmw(dp->pd + PD_UL_CHNL_CON1(q),
			   DRB_CON1_ADDR_HI | DRB_CON1_SIZE,
			   FIELD_PREP(DRB_CON1_ADDR_HI, upper_32_bits(r->dma)) |
			   FIELD_PREP(DRB_CON1_SIZE,
				      r->cnt * sizeof(struct dpmaif_drb) / 4));
		writel(lower_32_bits(r->dma), dp->pd + PD_UL_CHNL_CON0(q));
		dpmaif_rmw(dp->ao + AO_UL_CHNL_ARB0, ARB0_RDY(q) | ARB0_ARB(q),
			   ARB0_RDY(q) | ARB0_ARB(q));
	}
}

/* copy the settings to the SRAM, then tell UL and DL the setup is done */
static int dpmaif_hw_init_done(struct mtk_ccci_dpmaif *dp)
{
	int ret;

	writel(readl(dp->pd + PD_AP_OVERWRITE_CFG) | BIT(0),
	       dp->pd + PD_AP_OVERWRITE_CFG);
	ret = dpmaif_wait_clear(dp, dp->pd + PD_AP_OVERWRITE_CFG, BIT(0),
				"SRAM sync");
	if (ret)
		return ret;
	writel(BIT(0), dp->ao + AO_UL_INIT_SET);
	writel(BIT(0), dp->ao + AO_DL_INIT_SET);
	return 0;
}

static void dpmaif_clear_ip_busy(struct mtk_ccci_dpmaif *dp)
{
	u32 v = readl(dp->pd + PD_AP_IP_BUSY);

	if (v)
		writel(v, dp->pd + PD_AP_IP_BUSY);
}

/* BAT_ADD or PIT_ADD: wait until the last update is taken, write */
static int dpmaif_dl_add(struct mtk_ccci_dpmaif *dp, u32 reg, u32 val,
			 bool wait_done)
{
	int ret;

	ret = dpmaif_wait_clear(dp, dp->pd + reg, DL_ADD_UPDATE, "DL add");
	if (ret)
		return ret;
	writel(val, dp->pd + reg);
	return wait_done ? dpmaif_wait_clear(dp, dp->pd + reg, DL_ADD_UPDATE,
					     "DL add") : 0;
}

/*
 * Lend skb buffers in ring order, as long as the next slot came back (its
 * PIT was read) and fewer than DPMAIF_BAT_LEND are out.
 */
static void dpmaif_bat_refill(struct mtk_ccci_dpmaif *dp, gfp_t gfp)
{
	struct dpmaif_bat *bat = dp->bat.base;
	u32 n = 0;

	while (dp->bat_lent < DPMAIF_BAT_LEND) {
		struct dpmaif_bat_skb *b = &dp->bat_skb[dp->bat_wr];
		struct sk_buff *skb;
		dma_addr_t dma;

		if (b->skb)
			break;
		skb = __dev_alloc_skb(DPMAIF_BAT_BUF_SIZE, gfp);
		if (!skb) {
			dp->bat_nomem++;
			break;
		}
		dma = dma_map_single(dp->dev, skb->data, DPMAIF_BAT_BUF_SIZE,
				     DMA_FROM_DEVICE);
		if (dma_mapping_error(dp->dev, dma)) {
			dev_kfree_skb_any(skb);
			dp->bat_nomem++;
			break;
		}
		b->skb = skb;
		b->dma = dma;
		bat[dp->bat_wr].addr_lo = cpu_to_le32(lower_32_bits(dma));
		bat[dp->bat_wr].addr_hi = cpu_to_le32(upper_32_bits(dma) & 0xff);
		dp->bat_wr = (dp->bat_wr + 1) % dp->bat.cnt;
		dp->bat_lent++;
		n++;
	}
	/* writel() orders the BAT writes before the register */
	if (n)
		dpmaif_dl_add(dp, PD_DL_BAT_ADD, DL_ADD_UPDATE | n, false);
}

/* the same with page fragments for the fragment BAT */
static void dpmaif_frg_refill(struct mtk_ccci_dpmaif *dp)
{
	struct dpmaif_bat *bat = dp->frg.base;
	u32 n = 0;

	while (dp->frg_lent < DPMAIF_FRG_LEND) {
		struct dpmaif_bat_frg *f = &dp->frg_buf[dp->frg_wr];
		struct page *page;
		dma_addr_t dma;
		void *data;

		if (f->data)
			break;
		data = netdev_alloc_frag(DPMAIF_FRG_BUF_SIZE);
		if (!data) {
			dp->bat_nomem++;
			break;
		}
		page = virt_to_head_page(data);
		dma = dma_map_page(dp->dev, page, data - page_address(page),
				   DPMAIF_FRG_BUF_SIZE, DMA_FROM_DEVICE);
		if (dma_mapping_error(dp->dev, dma)) {
			skb_free_frag(data);
			dp->bat_nomem++;
			break;
		}
		f->data = data;
		f->dma = dma;
		bat[dp->frg_wr].addr_lo = cpu_to_le32(lower_32_bits(dma));
		bat[dp->frg_wr].addr_hi = cpu_to_le32(upper_32_bits(dma) & 0xff);
		dp->frg_wr = (dp->frg_wr + 1) % dp->frg.cnt;
		dp->frg_lent++;
		n++;
	}
	if (n)
		dpmaif_dl_add(dp, PD_DL_BAT_ADD, DL_ADD_UPDATE | DL_ADD_FRG | n,
			      false);
}

/* lend the first buffers, enable both BAT rings (downstream order) */
static int dpmaif_dl_start(struct mtk_ccci_dpmaif *dp)
{
	int ret;

	dpmaif_frg_refill(dp);
	dpmaif_bat_refill(dp, GFP_KERNEL);
	if (!dp->frg_lent || !dp->bat_lent)
		return -ENOMEM;

	dpmaif_rmw(dp->pd + PD_DL_BAT_INIT_CON1, BAT_CON1_EN, BAT_CON1_EN);
	ret = dpmaif_dl_init_cmd(dp, PD_DL_BAT_INIT, DL_INIT_EN | DL_INIT_FRG,
				 "frag BAT enable");
	if (ret)
		return ret;
	dpmaif_rmw(dp->pd + PD_DL_BAT_INIT_CON1, BAT_CON1_EN, BAT_CON1_EN);
	return dpmaif_dl_init_cmd(dp, PD_DL_BAT_INIT, DL_INIT_EN, "BAT enable");
}

/* only with the block reset: nothing is lent to the hardware any more */
static void dpmaif_dl_free(struct mtk_ccci_dpmaif *dp)
{
	u32 i;

	for (i = 0; i < dp->bat.cnt; i++) {
		struct dpmaif_bat_skb *b = &dp->bat_skb[i];

		if (!b->skb)
			continue;
		dma_unmap_single(dp->dev, b->dma, DPMAIF_BAT_BUF_SIZE,
				 DMA_FROM_DEVICE);
		dev_kfree_skb_any(b->skb);
		b->skb = NULL;
	}
	for (i = 0; i < dp->frg.cnt; i++) {
		struct dpmaif_bat_frg *f = &dp->frg_buf[i];

		if (!f->data)
			continue;
		dma_unmap_page(dp->dev, f->dma, DPMAIF_FRG_BUF_SIZE,
			       DMA_FROM_DEVICE);
		skb_free_frag(f->data);
		f->data = NULL;
	}
	dev_kfree_skb_any(dp->rx_skb);
	dp->rx_skb = NULL;
	dp->rx_msg = false;
	dp->bat_wr = 0;
	dp->bat_lent = 0;
	dp->frg_wr = 0;
	dp->frg_lent = 0;
	dp->pit_rd = 0;
	dp->pit_seq = 0;
	memset(dp->bat.base, 0, dp->bat.cnt * sizeof(struct dpmaif_bat));
	memset(dp->frg.base, 0, dp->frg.cnt * sizeof(struct dpmaif_bat));
	memset(dp->pit.base, 0, dp->pit.cnt * sizeof(struct dpmaif_pit));
}

/* hand a finished packet to the link of its channel */
static void dpmaif_rx_deliver(struct mtk_ccci_dpmaif *dp)
{
	struct sk_buff *skb = dp->rx_skb;
	struct net_device *ndev;
	u8 ch = dp->rx_ch;

	dp->rx_skb = NULL;
	dp->rx_msg = false;
	if (dp->rx_drop) {
		dp->dl_dropped++;
		goto drop;
	}
	if (ch >= MTK_CCCI_NET_LINKS) {
		dp->dl_bad_ch++;
		dp->dl_last_bad_ch = ch;
		goto drop;
	}
	dp->dl_pkts[ch]++;
	ndev = rcu_dereference(dp->ccci->net_link[ch]);
	if (!ndev || !netif_running(ndev)) {
		dp->dl_nolink[ch]++;
		goto drop;
	}
	if (!skb_headlen(skb))
		goto drop_dev;
	switch (skb->data[0] >> 4) {
	case 4:
		skb->protocol = htons(ETH_P_IP);
		break;
	case 6:
		skb->protocol = htons(ETH_P_IPV6);
		break;
	default:
		goto drop_dev;
	}
	skb->dev = ndev;
	if (dp->rx_csum_ok && (ndev->features & NETIF_F_RXCSUM))
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	dev_sw_netstats_rx_add(ndev, skb->len);
	napi_gro_receive(&dp->napi, skb);
	return;

drop_dev:
	dev_core_stats_rx_dropped_inc(ndev);
drop:
	dev_kfree_skb_any(skb);
}

/*
 * One PIT. Return 1 when it ends a packet (handed on or dropped). A buffer
 * named by a PIT is taken back from the hardware in any case.
 */
static int dpmaif_rx_pit(struct mtk_ccci_dpmaif *dp,
			 const struct dpmaif_pit *pit, u32 w0, u32 w3)
{
	u32 len = FIELD_GET(PIT0_DATA_LEN, w0);
	u32 bid = FIELD_GET(PIT0_BUF_ID, w0) | FIELD_GET(PIT3_H_BID, w3) << 13;
	u64 addr = le32_to_cpu(pit->w[1]) |
		   (u64)le32_to_cpu(pit->w[2]) << 32;

	if (w0 & PIT0_PACKET_TYPE) {
		if (dp->rx_msg || dp->rx_skb) {
			dp->dl_err++;
			dev_kfree_skb_any(dp->rx_skb);
			dp->rx_skb = NULL;
		}
		dp->rx_msg = true;
		dp->rx_ch = FIELD_GET(MPIT0_CHANNEL_ID, w0);
		dp->rx_drop = w0 & (MPIT0_DROP | MPIT0_ERROR);
		dp->rx_csum_ok = !FIELD_GET(MPIT0_CHECKSUM, w0);
		return 0;
	}

	if (!(w0 & PIT0_BUF_TYPE)) {
		struct dpmaif_bat_skb *b;
		struct sk_buff *skb;

		if (bid >= dp->bat.cnt || !dp->bat_skb[bid].skb)
			goto bad;
		b = &dp->bat_skb[bid];
		skb = b->skb;
		b->skb = NULL;
		dp->bat_lent--;
		dma_unmap_single(dp->dev, b->dma, DPMAIF_BAT_BUF_SIZE,
				 DMA_FROM_DEVICE);
		if (!dp->rx_msg || dp->rx_skb || addr < b->dma ||
		    addr - b->dma + len > DPMAIF_BAT_BUF_SIZE) {
			dev_kfree_skb_any(skb);
			goto bad;
		}
		skb_reserve(skb, addr - b->dma);
		skb_put(skb, len);
		dp->rx_skb = skb;
	} else {
		struct dpmaif_bat_frg *f;
		struct page *page;
		void *data;

		if (bid >= dp->frg.cnt || !dp->frg_buf[bid].data)
			goto bad;
		f = &dp->frg_buf[bid];
		data = f->data;
		f->data = NULL;
		dp->frg_lent--;
		dma_unmap_page(dp->dev, f->dma, DPMAIF_FRG_BUF_SIZE,
			       DMA_FROM_DEVICE);
		if (!dp->rx_skb || addr < f->dma ||
		    addr - f->dma + len > DPMAIF_FRG_BUF_SIZE ||
		    skb_shinfo(dp->rx_skb)->nr_frags >= MAX_SKB_FRAGS) {
			skb_free_frag(data);
			goto bad;
		}
		page = virt_to_head_page(data);
		skb_add_rx_frag(dp->rx_skb, skb_shinfo(dp->rx_skb)->nr_frags,
				page, data - page_address(page) + (addr - f->dma),
				len, DPMAIF_FRG_BUF_SIZE);
	}
	if (w0 & PIT0_CONT)
		return 0;
	dpmaif_rx_deliver(dp);
	return 1;

bad:
	dp->dl_err++;
	dev_kfree_skb_any(dp->rx_skb);
	dp->rx_skb = NULL;
	if (w0 & PIT0_CONT)
		return 0;
	dp->rx_msg = false;
	return 1;
}

/*
 * The write index can run ahead of the PIT contents: give the hardware a
 * little time for the sequence number (t7xx does the same).
 */
static bool dpmaif_pit_ready(struct mtk_ccci_dpmaif *dp,
			     const struct dpmaif_pit *pit, u32 *w3)
{
	unsigned int i;

	for (i = 0; i < 20; i++) {
		*w3 = le32_to_cpu(READ_ONCE(pit->w[3]));
		if (FIELD_GET(PIT3_SEQ, *w3) == dp->pit_seq)
			return true;
		udelay(5);
	}
	return false;
}

/* read the PITs up to the write index; finish a packet once started */
static int dpmaif_rx(struct mtk_ccci_dpmaif *dp, int budget)
{
	struct dpmaif_pit *pits = dp->pit.base;
	u32 hw_wr, done = 0;
	int pkts = 0;

	dp->rx_retry = false;
	hw_wr = readl(dp->pd + PD_DL_PIT_WR_IDX) & DL_PIT_IDX;
	if (hw_wr >= dp->pit.cnt) {
		dp->dl_err++;
		return 0;
	}

	rcu_read_lock();
	while (dp->pit_rd != hw_wr && (pkts < budget || dp->rx_msg)) {
		const struct dpmaif_pit *pit = &pits[dp->pit_rd];
		u32 w0, w3;

		if (!dpmaif_pit_ready(dp, pit, &w3)) {
			dp->pit_seq_err++;
			dp->rx_retry = true;
			break;
		}
		dma_rmb();
		w0 = le32_to_cpu(READ_ONCE(pit->w[0]));
		pkts += dpmaif_rx_pit(dp, pit, w0, w3);
		dp->pit_seq = dp->pit_seq == PIT_SEQ_MAX ? 0 : dp->pit_seq + 1;
		dp->pit_rd = (dp->pit_rd + 1) % dp->pit.cnt;
		done++;
	}
	rcu_read_unlock();

	if (done)
		dpmaif_dl_add(dp, PD_DL_PIT_ADD, DL_ADD_UPDATE | done, true);
	return min(pkts, budget);
}

static int dpmaif_napi_poll(struct napi_struct *napi, int budget)
{
	struct mtk_ccci_dpmaif *dp = container_of(napi, struct mtk_ccci_dpmaif,
						  napi);
	int done;

	done = dpmaif_rx(dp, budget);
	dpmaif_frg_refill(dp);
	dpmaif_bat_refill(dp, GFP_ATOMIC);
	if (dp->rx_retry)
		return budget;
	if (done < budget && napi_complete_done(napi, done))
		writel(DL_INT_QDONE, dp->ao + AO_UL_APDL_L2TIMCR0);
	return done;
}

static u32 dpmaif_txq_used(struct mtk_ccci_dpmaif *dp, unsigned int q)
{
	u32 cnt = dp->drb[q].cnt;

	return (dp->txq[q].wr + cnt - dp->txq[q].rel) % cnt;
}

static u32 dpmaif_txq_free(struct mtk_ccci_dpmaif *dp, unsigned int q)
{
	return dp->drb[q].cnt - 1 - dpmaif_txq_used(dp, q);
}

static void dpmaif_tx_enable(struct mtk_ccci_dpmaif *dp, bool on)
{
	unsigned int q;

	for (q = 0; q < DPMAIF_TXQ_NUM; q++) {
		spin_lock_bh(&dp->txq[q].lock);
		dp->txq[q].on = on;
		spin_unlock_bh(&dp->txq[q].lock);
	}
}

/* wake the links once a queue has room again */
static void dpmaif_tx_wake(struct mtk_ccci_dpmaif *dp)
{
	unsigned int i;

	rcu_read_lock();
	for (i = 0; i < MTK_CCCI_NET_LINKS; i++) {
		struct net_device *ndev = rcu_dereference(dp->ccci->net_link[i]);

		if (ndev && netif_running(ndev) && netif_queue_stopped(ndev))
			netif_wake_queue(ndev);
	}
	rcu_read_unlock();
}

/* release what the hardware has read of TX queue @q */
static bool dpmaif_tx_release(struct mtk_ccci_dpmaif *dp, unsigned int q)
{
	struct dpmaif_txq *txq = &dp->txq[q];
	u32 cnt = dp->drb[q].cnt, hw_rd;
	bool freed = false;

	hw_rd = FIELD_GET(UL_STA_RD_IDX, readl(dp->pd + PD_UL_CH_STA(q))) /
		(sizeof(struct dpmaif_drb) / 4);

	spin_lock_bh(&txq->lock);
	/* never past what was handed over */
	if (hw_rd >= cnt ||
	    (hw_rd + cnt - txq->rel) % cnt > dpmaif_txq_used(dp, q)) {
		txq->rd_err++;
		goto out;
	}
	while (txq->rel != hw_rd) {
		struct dpmaif_drb_skb *e = &txq->skbs[txq->rel];

		if (e->skb) {
			dma_unmap_single(dp->dev, e->dma, e->len,
					 DMA_TO_DEVICE);
			dev_consume_skb_any(e->skb);
			e->skb = NULL;
			freed = true;
		}
		txq->rel = (txq->rel + 1) % cnt;
	}
out:
	spin_unlock_bh(&txq->lock);
	return freed;
}

static void dpmaif_tx_done_work(struct work_struct *work)
{
	struct mtk_ccci_dpmaif *dp = container_of(work, struct mtk_ccci_dpmaif,
						  tx_work);
	unsigned long pending = atomic_xchg(&dp->tx_pending, 0);
	bool freed = false;
	unsigned int q;

	for_each_set_bit(q, &pending, DPMAIF_TXQ_NUM) {
		freed |= dpmaif_tx_release(dp, q);
		/* masked by the interrupt handler */
		writel(BIT(q), dp->ao + AO_UL_AP_L2TIMCR0);
	}
	if (freed)
		dpmaif_tx_wake(dp);
}

/* only with the block reset: the hardware reads no DRB any more */
static void dpmaif_tx_free(struct mtk_ccci_dpmaif *dp)
{
	unsigned int q;
	u32 i;

	for (q = 0; q < DPMAIF_TXQ_NUM; q++) {
		struct dpmaif_txq *txq = &dp->txq[q];

		spin_lock_bh(&txq->lock);
		for (i = 0; i < dp->drb[q].cnt; i++) {
			struct dpmaif_drb_skb *e = &txq->skbs[i];

			if (!e->skb)
				continue;
			dma_unmap_single(dp->dev, e->dma, e->len,
					 DMA_TO_DEVICE);
			dev_kfree_skb_any(e->skb);
			e->skb = NULL;
		}
		txq->wr = 0;
		txq->rel = 0;
		memset(dp->drb[q].base, 0,
		       dp->drb[q].cnt * sizeof(struct dpmaif_drb));
		spin_unlock_bh(&txq->lock);
	}
	atomic_set(&dp->tx_pending, 0);
}

/* stop UL arbitration and DL, wait for both to go idle */
static void dpmaif_hw_quiesce(struct mtk_ccci_dpmaif *dp)
{
	u32 v;

	dpmaif_rmw(dp->ao + AO_UL_CHNL_ARB0, ARB0_ARB_ALL, 0);
	if (readl_poll_timeout_atomic(dp->pd + PD_UL_DBG_STA2, v,
				      v & UL_DBG_IDLE, 1, DPMAIF_IDLE_POLL_US))
		dev_err(dp->dev, "DPMAIF UL not idle: %#x\n", v);

	dpmaif_rmw(dp->pd + PD_DL_BAT_INIT_CON1, BAT_CON1_EN, 0);
	if (dpmaif_dl_init_cmd(dp, PD_DL_BAT_INIT, DL_INIT_EN, "DL disable"))
		return;
	if (readl_poll_timeout_atomic(dp->pd + PD_DL_DBG_STA1, v,
				      v & DL_DBG_IDLE, 1, DPMAIF_IDLE_POLL_US))
		dev_err(dp->dev, "DPMAIF DL not idle: %#x\n", v);
}

/*
 * The downstream reset ("hw-reset-ver" 1 on MT6878): take the bus enable
 * away, reset the PD then the AO part, release AO then PD. Register
 * writes only, the DPMAIF clocks may be off. Sleeps.
 */
static void dpmaif_hw_reset(struct mtk_ccci_dpmaif *dp)
{
	void __iomem *infra = dp->ccci->infra;

	regmap_clear_bits(dp->infracfg_ao, INFRA_AO_DPMAIF_CTRL,
			  INFRA_AO_DPMAIF_EN);
	usleep_range(500, 600);
	writel(INFRA_DPMAIF_PD_RST, infra + INFRA_RST_SET);
	usleep_range(500, 600);
	regmap_write(dp->infracfg_ao, INFRA_AO_RST_SET, INFRA_AO_DPMAIF_RST);
	usleep_range(500, 600);
	regmap_write(dp->infracfg_ao, INFRA_AO_RST_CLR, INFRA_AO_DPMAIF_RST);
	usleep_range(500, 600);
	writel(INFRA_DPMAIF_PD_RST, infra + INFRA_RST_CLR);
}

/*
 * L2 DL mask bits 8..12 sit at 13, 14, 8, 9, 10 in the status register
 * (downstream mapping_rx_reg_from_mask_to_sts()).
 */
static u32 dpmaif_dl_mask_to_sta(u32 mask)
{
	return (mask & GENMASK(7, 0)) |
	       FIELD_PREP(BIT(13), FIELD_GET(BIT(8), mask)) |
	       FIELD_PREP(BIT(14), FIELD_GET(BIT(9), mask)) |
	       FIELD_PREP(GENMASK(10, 8), FIELD_GET(GENMASK(12, 10), mask));
}

/*
 * One line for TX done, RX queue 0 and the errors: acknowledge, mask the
 * queue that has work and schedule it; errors are counted.
 */
static irqreturn_t dpmaif_irq(int irq, void *data)
{
	struct mtk_ccci_dpmaif *dp = data;
	u32 ul, ul_mask, dl, mask;

	ul_mask = readl(dp->ao + AO_UL_AP_L2TIMR0);
	ul = readl(dp->pd + PD_AP_UL_L2TISAR0) & ~ul_mask;
	dl = readl(dp->pd + PD_AP_DL_L2TISAR0) &
	     ~dpmaif_dl_mask_to_sta(readl(dp->ao + AO_UL_APDL_L2TIMR0));
	dpmaif_clear_ip_busy(dp);
	if (!ul && !dl)
		return IRQ_NONE;

	dp->irqs++;
	if (ul) {
		dp->last_ul_sta = ul;
		writel(ul, dp->pd + PD_AP_UL_L2TISAR0);
		if (ul & UL_INT_LEN_ERR)
			dp->ul_len_err++;
		if (ul & UL_INT_PWR_NOTRDY)
			dp->ul_pwr_notrdy++;
		if (ul & UL_INT_QDONE) {
			dp->ul_done++;
			/*
			 * Masked before the work is queued: the work unmasks
			 * it again and may run at once on another CPU.
			 */
			mask = ul & UL_INT_QDONE;
			writel(mask, dp->ao + AO_UL_AP_L2TIMSR0);
			atomic_or(mask, &dp->tx_pending);
			queue_work(system_bh_wq, &dp->tx_work);
			/* the modem is awake again (downstream does this too) */
			if (ul_mask & UL_INT_PWR_NOTRDY)
				writel(UL_INT_PWR_NOTRDY,
				       dp->ao + AO_UL_AP_L2TIMCR0);
		} else if (ul & UL_INT_PWR_NOTRDY) {
			/* the modem sleeps: masked until the next TX done */
			writel(UL_INT_PWR_NOTRDY, dp->ao + AO_UL_AP_L2TIMSR0);
		}
	}
	if (dl) {
		dp->last_dl_sta = dl;
		writel(dl, dp->pd + PD_AP_DL_L2TISAR0);
		if (dl & DL_INT_MTU_ERR)
			dp->dl_mtu_err++;
		if (dl & DL_INT_QDONE) {
			dp->dl_done++;
			/* unmasked again when NAPI is done */
			writel(DL_INT_QDONE, dp->ao + AO_UL_APDL_L2TIMSR0);
			napi_schedule(&dp->napi);
		}
	}
	return IRQ_HANDLED;
}

/**
 * mtk_ccci_dpmaif_start() - set DPMAIF up, before the modem is released
 * @ccci: driver state
 *
 * The downstream order: clocks, bus enable, SRAM clear, bank remap,
 * interrupts, RX queue (PIT), BAT rings, the first buffers lent and DL
 * enabled, TX queues (DRB), SRAM sync and "init done". On a failure the
 * block is reset and left off; the modem starts without a data path.
 *
 * Return: 0 (also without DPMAIF) or a negative error.
 */
int mtk_ccci_dpmaif_start(struct mtk_ccci *ccci)
{
	struct mtk_ccci_dpmaif *dp = ccci->dpmaif;
	int ret;

	if (!dp)
		return 0;

	mutex_lock(&dp->lock);
	if (dp->started) {
		ret = 0;
		goto out;
	}

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(dp->clks), dp->clks);
	if (ret)
		goto out;
	ret = regmap_set_bits(dp->infracfg_ao, INFRA_AO_DPMAIF_CTRL,
			      INFRA_AO_DPMAIF_EN);
	if (!ret)
		/* as the downstream driver: 0x1000 there, 0x3000 out of LK */
		ret = regmap_update_bits(dp->infracfg_ao, INFRA_AO_DPMAIF_WAKE,
					 INFRA_AO_DPMAIF_IP_BUSY_MASK |
					 INFRA_AO_DPMAIF_MD_WAKE_MASK,
					 INFRA_AO_DPMAIF_IP_BUSY_MASK);
	if (ret)
		goto clk_off;

	ret = dpmaif_hw_common(dp);
	if (!ret)
		ret = dpmaif_hw_rxq(dp);
	if (!ret)
		ret = dpmaif_hw_bats(dp);
	if (!ret)
		ret = dpmaif_dl_start(dp);
	if (!ret) {
		dpmaif_hw_txqs(dp);
		ret = dpmaif_hw_init_done(dp);
	}
	if (ret)
		goto reset;

	dpmaif_clear_ip_busy(dp);
	napi_enable(&dp->napi);
	enable_irq(dp->irq);
	dpmaif_tx_enable(dp, true);
	dp->started = true;
	dev_info(ccci->dev, "DPMAIF up (coda %#x), %u+%u buffers lent\n",
		 readl(dp->pd + PD_AP_CODA_VER), dp->bat_lent, dp->frg_lent);
	goto out;

reset:
	dpmaif_hw_reset(dp);
	dpmaif_dl_free(dp);
clk_off:
	clk_bulk_disable_unprepare(ARRAY_SIZE(dp->clks), dp->clks);
out:
	mutex_unlock(&dp->lock);
	return ret;
}

/**
 * mtk_ccci_dpmaif_pre_stop() - stop the queues, before the modem goes down
 * @ccci: driver state
 */
void mtk_ccci_dpmaif_pre_stop(struct mtk_ccci *ccci)
{
	struct mtk_ccci_dpmaif *dp = ccci->dpmaif;

	if (!dp)
		return;
	mutex_lock(&dp->lock);
	if (dp->started) {
		dpmaif_tx_enable(dp, false);
		dpmaif_hw_quiesce(dp);
	}
	mutex_unlock(&dp->lock);
}

/**
 * mtk_ccci_dpmaif_stop() - reset DPMAIF, once the modem is off
 * @ccci: driver state
 */
void mtk_ccci_dpmaif_stop(struct mtk_ccci *ccci)
{
	struct mtk_ccci_dpmaif *dp = ccci->dpmaif;

	if (!dp)
		return;
	mutex_lock(&dp->lock);
	if (dp->started) {
		dpmaif_tx_enable(dp, false);
		disable_irq(dp->irq);
		napi_disable(&dp->napi);
		cancel_work_sync(&dp->tx_work);
		dpmaif_hw_quiesce(dp);
		clk_bulk_disable_unprepare(ARRAY_SIZE(dp->clks), dp->clks);
		dpmaif_hw_reset(dp);
		/* the block is reset: the buffers are the AP's again */
		dpmaif_dl_free(dp);
		dpmaif_tx_free(dp);
		dp->started = false;
		dev_info(ccci->dev, "DPMAIF down\n");
	}
	mutex_unlock(&dp->lock);
}

/* the base of modem bank 0 as LK reports it, checked against the carveouts */
static int dpmaif_md_base(struct mtk_ccci_dpmaif *dp)
{
	struct mtk_ccci *ccci = dp->ccci;
	const __le64 *hdr;
	unsigned int i;
	u32 size;

	hdr = mtk_ccci_lk_find(ccci, "hdr_tbl_inf", &size);
	if (!hdr || size < sizeof(*hdr))
		return -ENODEV;
	dp->md_base = le64_to_cpu(*hdr);

	if (!dp->md_base || !IS_ALIGNED(dp->md_base, DPMAIF_REMAP_WIN) ||
	    dp->md_base + 24ULL * DPMAIF_REMAP_WIN > BIT_ULL(34))
		return -EINVAL;
	for (i = 0; i < ccci->md_mem_num; i++)
		if (ccci->md_mem[i].start == dp->md_base)
			return 0;
	return -EINVAL;
}

static void dpmaif_napi_cleanup(void *data)
{
	struct mtk_ccci_dpmaif *dp = data;

	netif_napi_del(&dp->napi);
	free_netdev(dp->napi_dev);
}

/* NAPI needs a device; the links come and go, as in t7xx use a dummy one */
static int dpmaif_napi_init(struct mtk_ccci_dpmaif *dp)
{
	dp->napi_dev = alloc_netdev_dummy(0);
	if (!dp->napi_dev)
		return -ENOMEM;
	netif_napi_add(dp->napi_dev, &dp->napi, dpmaif_napi_poll);
	return devm_add_action_or_reset(dp->dev, dpmaif_napi_cleanup, dp);
}

static void __iomem *dpmaif_map(struct device *dev, struct device_node *np,
				const char *name)
{
	int i = of_property_match_string(np, "reg-names", name);

	if (i < 0)
		return IOMEM_ERR_PTR(i);
	return devm_of_iomap(dev, np, i, NULL);
}

/* the DPMAIF node: windows, clocks, interrupt, infracfg_ao */
static int dpmaif_get_resources(struct mtk_ccci_dpmaif *dp,
				struct device_node *np)
{
	struct device *dev = dp->dev;
	unsigned int i;

	dp->ao = dpmaif_map(dev, np, "ao");
	if (IS_ERR(dp->ao))
		return PTR_ERR(dp->ao);
	dp->pd = dpmaif_map(dev, np, "pd");
	if (IS_ERR(dp->pd))
		return PTR_ERR(dp->pd);

	for (i = 0; i < ARRAY_SIZE(dpmaif_clk_names); i++) {
		dp->clks[i].id = dpmaif_clk_names[i];
		dp->clks[i].clk =
			devm_get_clk_from_child(dev, np, dpmaif_clk_names[i]);
		if (IS_ERR(dp->clks[i].clk))
			return dev_err_probe(dev, PTR_ERR(dp->clks[i].clk),
					     "DPMAIF clock %s\n",
					     dpmaif_clk_names[i]);
	}

	dp->infracfg_ao = syscon_regmap_lookup_by_phandle(np,
							  "mediatek,infracfg-ao");
	if (IS_ERR(dp->infracfg_ao))
		return PTR_ERR(dp->infracfg_ao);

	dp->irq = of_irq_get(np, 0);
	if (dp->irq <= 0)
		return dp->irq ? dp->irq : -ENXIO;
	return devm_request_irq(dev, dp->irq, dpmaif_irq, IRQF_NO_AUTOEN,
				"mtk-ccci-dpmaif", dp);
}

/**
 * mtk_ccci_dpmaif_init() - find DPMAIF and allocate its rings
 * @ccci: driver state
 *
 * Only with data=1 and the DPMAIF node linked by "mediatek,dpmaif". Touches
 * no register: the block is set up by mtk_ccci_dpmaif_start(). A DPMAIF
 * that cannot be used leaves the modem without a data path, it does not
 * fail the probe.
 *
 * Return: 0 (also when skipped), -EPROBE_DEFER, -ENOMEM or an error of
 * the WWAN registration.
 */
int mtk_ccci_dpmaif_init(struct mtk_ccci *ccci)
{
	struct device *dev = ccci->dev;
	struct mtk_ccci_dpmaif *dp;
	struct device_node *np;
	unsigned int i;
	int ret;

	BUILD_BUG_ON(sizeof(struct dpmaif_pit) != 16);
	BUILD_BUG_ON(sizeof(struct dpmaif_bat) != 8);
	BUILD_BUG_ON(sizeof(struct dpmaif_drb) != 8);

	if (!data)
		return 0;
	np = of_parse_phandle(dev->of_node, "mediatek,dpmaif", 0);
	if (!np) {
		dev_info(dev, "no DPMAIF in the DT, no data path\n");
		return 0;
	}
	if (!ccci->infra) {
		of_node_put(np);
		dev_info(dev, "no infracfg window to reset DPMAIF, no data path\n");
		return 0;
	}

	dp = devm_kzalloc(dev, sizeof(*dp), GFP_KERNEL);
	if (!dp) {
		ret = -ENOMEM;
		goto out;
	}
	dp->ccci = ccci;
	dp->dev = dev;
	mutex_init(&dp->lock);

	ret = dpmaif_md_base(dp);
	if (ret) {
		dev_warn(dev, "no usable modem bank base for DPMAIF\n");
		goto out;
	}

	ret = dpmaif_get_resources(dp, np);
	if (ret)
		goto out;

	/* BAT and DRB addresses have 40 bits */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		goto out;
	/* the rings from the DT pool if there is one, as downstream */
	ret = of_reserved_mem_device_init(dev);
	if (!ret)
		ret = devm_add_action_or_reset(dev, dpmaif_rmem_release, dev);
	else if (ret == -ENODEV)
		ret = 0;
	if (ret)
		goto out;
	ret = dpmaif_ring_alloc(dev, &dp->bat, DPMAIF_BAT_CNT,
				sizeof(struct dpmaif_bat));
	if (!ret)
		ret = dpmaif_ring_alloc(dev, &dp->frg, DPMAIF_FRG_CNT,
					sizeof(struct dpmaif_bat));
	if (!ret)
		ret = dpmaif_ring_alloc(dev, &dp->pit, DPMAIF_PIT_CNT,
					sizeof(struct dpmaif_pit));
	for (i = 0; !ret && i < DPMAIF_TXQ_NUM; i++)
		ret = dpmaif_ring_alloc(dev, &dp->drb[i], DPMAIF_DRB_CNT,
					sizeof(struct dpmaif_drb));
	if (ret)
		goto out;
	dp->bat_skb = devm_kcalloc(dev, DPMAIF_BAT_CNT, sizeof(*dp->bat_skb),
				   GFP_KERNEL);
	dp->frg_buf = devm_kcalloc(dev, DPMAIF_FRG_CNT, sizeof(*dp->frg_buf),
				   GFP_KERNEL);
	if (!dp->bat_skb || !dp->frg_buf) {
		ret = -ENOMEM;
		goto out;
	}
	for (i = 0; i < DPMAIF_TXQ_NUM; i++) {
		spin_lock_init(&dp->txq[i].lock);
		dp->txq[i].skbs = devm_kcalloc(dev, DPMAIF_DRB_CNT,
					       sizeof(*dp->txq[i].skbs),
					       GFP_KERNEL);
		if (!dp->txq[i].skbs) {
			ret = -ENOMEM;
			goto out;
		}
	}
	INIT_WORK(&dp->tx_work, dpmaif_tx_done_work);
	ret = dpmaif_napi_init(dp);
	if (ret)
		goto out;

	ccci->dpmaif = dp;
	dev_info(dev, "DPMAIF: rings allocated, modem bank 0 at %#llx, set up with the modem\n",
		 dp->md_base);
	of_node_put(np);
	return mtk_ccci_net_init(ccci);

out:
	of_node_put(np);
	if (ret == -EPROBE_DEFER || ret == -ENOMEM)
		return ret;
	/* the modem works without a data path */
	dev_warn(dev, "DPMAIF unusable (%d), no data path\n", ret);
	return 0;
}

static void dpmaif_show_regs(struct seq_file *s, struct mtk_ccci_dpmaif *dp)
{
	static const struct {
		const char *name;
		bool ao;
		u16 off;
	} regs[] = {
		{ "coda_ver", false, PD_AP_CODA_VER },
		{ "dl_reserve_rw", false, PD_DL_RESERVE_RW },
		{ "ul_init_set", true, AO_UL_INIT_SET },
		{ "dl_init_set", true, AO_DL_INIT_SET },
		{ "ul_chnl_arb0", true, AO_UL_CHNL_ARB0 },
		{ "ul_l2 sta", false, PD_AP_UL_L2TISAR0 },
		{ "ul_l2 mask", true, AO_UL_AP_L2TIMR0 },
		{ "dl_l2 sta", false, PD_AP_DL_L2TISAR0 },
		{ "dl_l2 mask", true, AO_UL_APDL_L2TIMR0 },
		{ "l1 sta", false, PD_AP_L1TISAR0 },
		{ "l1 mask", true, AO_UL_AP_L1TIMR0 },
		{ "ip_busy", false, PD_AP_IP_BUSY },
		{ "dma_err_sta", false, PD_AP_DMA_ERR_STA },
		{ "pit_con0", false, PD_DL_PIT_INIT_CON0 },
		{ "pit_con1", false, PD_DL_PIT_INIT_CON1 },
		{ "pit_con3", false, PD_DL_PIT_INIT_CON3 },
		{ "bat_con0", false, PD_DL_BAT_INIT_CON0 },
		{ "bat_con1", false, PD_DL_BAT_INIT_CON1 },
		{ "pktinfo_con0", false, PD_DL_PKTINFO_CON0 },
		{ "pktinfo_con1", false, PD_DL_PKTINFO_CON1 },
		{ "pktinfo_con2", false, PD_DL_PKTINFO_CON2 },
		{ "rdy_chk_thres", false, PD_DL_RDY_CHK_THRES },
		{ "rdy_chk_frg", false, PD_DL_RDY_CHK_FRG_THRES },
		{ "pit wr/rd", false, PD_DL_PIT_WR_IDX },
		{ "", false, PD_DL_PIT_RD_IDX },
		{ "bat rd/wr", false, PD_DL_BAT_RD_IDX },
		{ "", false, PD_DL_BAT_WR_IDX },
		{ "frg rd", false, PD_DL_FRG_RD_IDX },
		{ "dl_dbg_sta1", false, PD_DL_DBG_STA1 },
		{ "ul_dbg_sta2", false, PD_UL_DBG_STA2 },
	};
	unsigned int i;

	/* an unnamed register goes on the line of the one before */
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		u32 v = readl((regs[i].ao ? dp->ao : dp->pd) + regs[i].off);

		if (!regs[i].name[0])
			seq_printf(s, " %#010x", v);
		else
			seq_printf(s, "%s%-14s %#010x", i ? "\n" : "",
				   regs[i].name, v);
	}
	seq_puts(s, "\n");
	for (i = 0; i < DPMAIF_TXQ_NUM; i++)
		seq_printf(s, "txq%u con0 %#010x con1 %#010x sta %#010x\n", i,
			   readl(dp->pd + PD_UL_CHNL_CON0(i)),
			   readl(dp->pd + PD_UL_CHNL_CON1(i)),
			   readl(dp->pd + PD_UL_CH_STA(i)));
	seq_puts(s, "remap");
	for (i = 0; i < 8; i++)
		seq_printf(s, " %#010x", readl(dp->pd + PD_MISC_REMAP(i)));
	seq_puts(s, "\n");
}

/*
 * The registers are read only with the clocks on and the bus enable set:
 * while started, or (before the first start) when LK left the enable set,
 * with the clocks held for the read.
 */
static int mtk_ccci_dpmaif_show(struct seq_file *s, void *unused)
{
	struct mtk_ccci_dpmaif *dp = s->private;
	unsigned int i;
	u32 ctrl = 0;
	int ret;

	mutex_lock(&dp->lock);
	ret = regmap_read(dp->infracfg_ao, INFRA_AO_DPMAIF_CTRL, &ctrl);
	seq_printf(s, "state: %s, infracfg_ao+0x208 %#010x (%d), modem bank 0 at %#llx\n",
		   dp->started ? "up" : "down", ctrl, ret, dp->md_base);
	seq_printf(s, "rings: bat %pad+%u frg %pad+%u pit %pad+%u\n",
		   &dp->bat.dma, dp->bat.cnt, &dp->frg.dma, dp->frg.cnt,
		   &dp->pit.dma, dp->pit.cnt);
	for (i = 0; i < DPMAIF_TXQ_NUM; i++)
		seq_printf(s, "       drb%u %pad+%u\n", i, &dp->drb[i].dma,
			   dp->drb[i].cnt);
	seq_printf(s, "irq %d: %lu; ul done %lu pwr-notrdy %lu len-err %lu, dl done %lu mtu-err %lu; last ul %#x dl %#x\n",
		   dp->irq, dp->irqs, dp->ul_done, dp->ul_pwr_notrdy,
		   dp->ul_len_err, dp->dl_done, dp->dl_mtu_err,
		   dp->last_ul_sta, dp->last_dl_sta);
	seq_printf(s, "dl: pit rd %u seq %u, bat wr %u lent %u, frg wr %u lent %u; dropped %lu err %lu seq-err %lu nomem %lu bad-ch %lu (last %u)\n",
		   dp->pit_rd, dp->pit_seq, dp->bat_wr, dp->bat_lent,
		   dp->frg_wr, dp->frg_lent, dp->dl_dropped, dp->dl_err,
		   dp->pit_seq_err, dp->bat_nomem, dp->dl_bad_ch,
		   dp->dl_last_bad_ch);
	for (i = 0; i < DPMAIF_TXQ_NUM; i++)
		seq_printf(s, "ul q%u: wr %u rel %u, %lu packets, full %lu, add-err %lu rd-err %lu\n",
			   i, dp->txq[i].wr, dp->txq[i].rel, dp->txq[i].pkts,
			   dp->txq[i].full, dp->txq[i].add_err,
			   dp->txq[i].rd_err);
	/* the channel a PDN uses shows here, link or not */
	for (i = 0; i < MTK_CCCI_NET_LINKS; i++)
		if (dp->dl_pkts[i])
			seq_printf(s, "dl ch %2u: %lu packets, %lu without a link\n",
				   i, dp->dl_pkts[i], dp->dl_nolink[i]);

	if (dp->started) {
		dpmaif_show_regs(s, dp);
	} else if (!ret && (ctrl & INFRA_AO_DPMAIF_EN) &&
		   !clk_bulk_prepare_enable(ARRAY_SIZE(dp->clks), dp->clks)) {
		dpmaif_show_regs(s, dp);
		clk_bulk_disable_unprepare(ARRAY_SIZE(dp->clks), dp->clks);
	} else {
		seq_puts(s, "registers: not read (bus enable off)\n");
	}
	mutex_unlock(&dp->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_ccci_dpmaif);

void mtk_ccci_dpmaif_debugfs_init(struct mtk_ccci *ccci)
{
	if (ccci->dpmaif)
		debugfs_create_file("dpmaif", 0400, ccci->debugfs,
				    ccci->dpmaif, &mtk_ccci_dpmaif_fops);
}

/* a TCP segment that only acknowledges: downstream sends those on queue 1 */
static bool dpmaif_tx_is_ack(const struct sk_buff *skb)
{
	const struct tcphdr *th;
	unsigned int l3, total;

	switch (skb->data[0] >> 4) {
	case 4: {
		const struct iphdr *iph = (const struct iphdr *)skb->data;

		if (skb->len < sizeof(*iph) || iph->ihl < 5 ||
		    iph->protocol != IPPROTO_TCP ||
		    (iph->frag_off & htons(IP_MF | IP_OFFSET)))
			return false;
		l3 = iph->ihl * 4;
		total = ntohs(iph->tot_len);
		break;
	}
	case 6: {
		const struct ipv6hdr *ip6h = (const struct ipv6hdr *)skb->data;

		if (skb->len < sizeof(*ip6h) || ip6h->nexthdr != IPPROTO_TCP)
			return false;
		l3 = sizeof(*ip6h);
		total = l3 + ntohs(ip6h->payload_len);
		break;
	}
	default:
		return false;
	}
	if (skb->len < l3 + sizeof(*th))
		return false;
	th = (const struct tcphdr *)(skb->data + l3);
	return th->ack && !th->syn && !th->fin && !th->rst &&
	       total == l3 + th->doff * 4;
}

/* tell the hardware about @words more DRB words of TX queue @q */
static int dpmaif_ul_add(struct mtk_ccci_dpmaif *dp, unsigned int q,
			 u32 words)
{
	void __iomem *reg = dp->pd + PD_UL_ADD_DESC_CH(q);
	u32 v;

	if (readl_poll_timeout_atomic(reg, v, !(v & UL_ADD_UPDATE), 1,
				      DPMAIF_UL_POLL_US))
		return -ETIMEDOUT;
	writel(UL_ADD_UPDATE | words, reg);
	/* taken or not, the DRBs are the hardware's now */
	if (readl_poll_timeout_atomic(reg, v, !(v & UL_ADD_UPDATE), 1,
				      DPMAIF_UL_POLL_US))
		dev_err_ratelimited(dp->dev, "DPMAIF UL q%u add slow\n", q);
	return 0;
}

/**
 * mtk_ccci_dpmaif_xmit() - send an IP packet to the modem
 * @ccci: driver state
 * @dev: the WWAN link it comes from
 * @skb: the packet, raw IP
 * @ch: CCMNI index of the link
 *
 * Return: NETDEV_TX_OK, or NETDEV_TX_BUSY with @dev stopped when the TX
 * queue is full.
 */
netdev_tx_t mtk_ccci_dpmaif_xmit(struct mtk_ccci *ccci, struct net_device *dev,
				 struct sk_buff *skb, unsigned int ch)
{
	struct mtk_ccci_dpmaif *dp = ccci->dpmaif;
	struct dpmaif_drb_skb *e;
	struct dpmaif_txq *txq;
	struct dpmaif_drb *drb;
	unsigned int q, len;
	dma_addr_t dma;
	u32 i, j, cnt;

	if (!dp || skb_linearize(skb) || skb->len > FIELD_MAX(DRB0_DATA_LEN))
		goto drop;
	len = skb->len;
	q = dpmaif_tx_is_ack(skb) ? DPMAIF_TXQ_ACK : DPMAIF_TXQ_DATA;
	txq = &dp->txq[q];
	drb = dp->drb[q].base;
	cnt = dp->drb[q].cnt;

	spin_lock_bh(&txq->lock);
	if (!txq->on) {
		spin_unlock_bh(&txq->lock);
		goto drop;
	}
	if (dpmaif_txq_free(dp, q) < DPMAIF_TX_DRBS) {
		/* woken by the TX done work; a full queue has packets out */
		netif_stop_queue(dev);
		txq->full++;
		spin_unlock_bh(&txq->lock);
		return NETDEV_TX_BUSY;
	}
	dma = dma_map_single(dp->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(dp->dev, dma)) {
		spin_unlock_bh(&txq->lock);
		goto drop;
	}

	i = txq->wr;
	j = (i + 1) % cnt;
	drb[i].w[0] = cpu_to_le32(FIELD_PREP(DRB0_TYPE, 1) | DRB0_CONT |
				  FIELD_PREP(DRB0_DATA_LEN, len));
	drb[i].w[1] = cpu_to_le32(FIELD_PREP(DRB1_CHANNEL_ID, ch));
	drb[j].w[0] = cpu_to_le32(FIELD_PREP(DRB0_TYPE, 0) |
				  FIELD_PREP(DRB0_ADDR_EXT,
					     upper_32_bits(dma) & 0xff) |
				  FIELD_PREP(DRB0_DATA_LEN, len));
	drb[j].w[1] = cpu_to_le32(lower_32_bits(dma));
	e = &txq->skbs[j];
	e->skb = skb;
	e->dma = dma;
	e->len = len;

	/* writel() orders the DRB writes before the register */
	if (dpmaif_ul_add(dp, q, DPMAIF_TX_DRBS *
			  sizeof(struct dpmaif_drb) / 4)) {
		e->skb = NULL;
		dma_unmap_single(dp->dev, dma, len, DMA_TO_DEVICE);
		txq->add_err++;
		spin_unlock_bh(&txq->lock);
		goto drop;
	}
	txq->wr = (j + 1) % cnt;
	txq->pkts++;
	if (dpmaif_txq_free(dp, q) < DPMAIF_TX_DRBS) {
		netif_stop_queue(dev);
		txq->full++;
	}
	spin_unlock_bh(&txq->lock);
	dev_sw_netstats_tx_add(dev, 1, len);
	return NETDEV_TX_OK;

drop:
	dev_core_stats_tx_dropped_inc(dev);
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}
