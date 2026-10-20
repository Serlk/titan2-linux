// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bluetooth core (BGF) of the MediaTek MT6878 connectivity subsystem.
 *
 * Copyright (c) 2026 serlk
 *
 * The BGF power sequence, the shared memory HCI transport ("BEIF") and the
 * WMT function control follow MediaTek's mt66xx Bluetooth driver for
 * MT6878 (bt/mt66xx: btif/btmtk_beif.c, btif/btmtk_mt66xx.c,
 * include/btif/platform_6878.h), Copyright (c) 2016-2021 MediaTek Inc.
 *
 * CONNINFRA (drivers/soc/mediatek/mtk-conninfra.c) powers CONNSYS and the
 * RF die. The stock boot loader has copied the BT ROM patch/RAM image into
 * the CONNSYS EMI, so releasing the BGF CPU is enough to start it. HCI
 * packets travel as an H:4 byte stream through two rings in the same EMI
 * (host to chip and chip to host), with software interrupts both ways.
 *
 * Skeleton, not tested on hardware: no low power (the core stays in
 * driver own while up), no firmware log, no coredump, no chip reset.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/usb.h>
#include <linux/soc/mediatek/mtk-conninfra.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "btmtk.h"

/* CONN_INFRA, offsets from 0x1800_0000 */
#define RGU_BGFSYS_ON_TOP_PWR_CTL	0x00020
#define  BGFSYS_TOP_KEY			0x4254
#define  BGFSYS_TOP_PWR_ON		BIT(7)
#define RGU_BGFSYS_CPU_SW_RST		0x00124
#define  BGF_CPU_SW_RST_B		BIT(0)
#define RGU_SEMA_M1_SW_RST_B		0x0015c
#define RGU_BGFSYS_OFF_TOP_PWR_ACK	0x00424
#define  BGF_OFF_PWR_ACK_B		BIT(24)
#define  BGF_OFF_PWR_ACK_S		BIT(0)
#define CFG_ON_CONN_INFRA_RDY		0x01210
#define  CONN_INFRA_RDY			BIT(16)
#define CFG_BT_PWRCTLCR0		0x01208
#define  BT_FUNC_EN			BIT(0)
#define CFG_CONN2BT_GALS_SLP_CTL	0x01450
#define CFG_CONN2BT_GALS_SLP_STATUS	0x01454
#define CFG_BT2CONN_GALS_SLP_CTL	0x01460
#define CFG_BT2CONN_GALS_SLP_STATUS	0x01464
#define  GALS_SLP_PROT_TX_EN		BIT(0)
#define  GALS_SLP_PROT_RX_EN		BIT(4)
#define  GALS_SLP_PROT_RX_ACK		BIT(22)
#define  GALS_SLP_PROT_TX_ACK		BIT(23)
#define WT_SLP_TOP_CK_2			0x03128
#define  TOP_CK_EN			BIT(0)
#define  TOP_CK_BSY			BIT(1)
#define CFG_IP_VERSION			0x11000
#define  MT6878_CONN_IP_VERSION		0x02050401
#define CFG_EMI_CTL_BT			0x11118
#define  BT_EMI_REQ			BIT(0)
#define  BT_SW_CONN_SRCCLKENA		BIT(1)
#define  BT_SW_CONN_AP_BUS_REQ		BIT(2)
#define  BT_SW_CONN_APSRC_REQ		BIT(3)
#define  BT_INFRA_REQ			BIT(5)
#define CONN_DBG_MCU_PC			0x23a00
#define CONN_DBG_MCU_PC_SEL		0x23a04
#define  BGF_MCU_PC_SEL			0xc0010d2a
#define CCIF_BGF_MCUCIRQ		0x3f00c	/* AP to BGF: BEIF data */
#define  BGF_BEIF_WAKEUP		(BIT(0) | BIT(1))
#define CCIF_BGF_SW_IRQ_STATUS		0x3f010
#define CCIF_BGF_SW_IRQ_RESET		0x3f014
#define  BGF_FW_LOG_NOTIFY		BIT(0)
#define  BGF_SUBSYS_CHIP_RESET		BIT(1)
#define  BGF_WHOLE_CHIP_RESET		BIT(2)
#define  BGF_FW2AP_NOTIFY		BIT(3)
#define HOST_BGF_IRQ_STAT		0x60034
#define HOST_BGF_IRQ_STAT2		0x6003c
#define  BGF_IRQ_FW_OWN_SET		BIT(0)
#define HOST_CONN_INFRA_WAKEUP_BT	0x601a8
#define HOST_CONNSYS_PWR_STATES		0x60a10
#define  BGFSYS_ON_ISO_EN		GENMASK(19, 18)
#define  BGFSYS_ON_ISO_EN_DONE		BIT(18)
#define HOST_BGF_MONFLG_ON_OUT		0x60c00
#define  BGF_CONN_INFRA_READY		BIT(15)
#define HOST_HOSTCSR2BGF_ON_DBG_SEL	0x60c04
#define CONN_SEMA_M1_OWN_REL(n)		(0x71200 + 4 * (n))
#define CONN_SEMA_NUM			25

/* BGF, offsets from 0x1880_0000 */
#define BGF_MCCR			0x00100
#define  BGF_CON_CR_AHB_AUTO_DIS	BIT(31)
#define BGF_MCCR_SET			0x00104
#define  BGF_CON_CR_AHB_STOP		(BIT(4) | BIT(5) | BIT(8))
#define BGF_MCU_CFG_SW_DBG_CTL		0x0016c
#define  BGF_ROM_IDLE			0x1d1e
#define BGF_IP_VERSION			0x12000
#define  MT6878_BGF_IP_VERSION		0x02090000
#define BGF_CKGEN0_FRZ_WIN		0x12100
#define  BGF_HCLK_SWITCH_FRZ_WINDOW	BIT(2)
#define BGF_MCUSYS_DLY_CHAIN_CTL	0x20284
#define BGF_WINDOW			0x30000

/* BEIF rings in the CONNSYS EMI */
#define BEIF_EMI_OFFSET			0xba0000
#define BEIF_EMI_SIZE			SZ_32K
#define BEIF_HEADER_KEY			0x55665566
#define BEIF_GUARD_PATTERN		0xdeadbeef
#define BEIF_GUARD_SIZE			32

struct beif_header {
	__le32 key;
	__le32 version;
	__le32 host_ctrl_offset;
	__le32 fw_ctrl_offset;
	__le32 h2c_offset;
	__le32 h2c_size;
	__le32 c2h_offset;
	__le32 c2h_size;
};

struct beif_ctrl {
	__le32 tx_push;
	__le32 rx_pop;
	__le32 unused[5];
	__le32 guard;
};

#define BEIF_HOST_CTRL		sizeof(struct beif_header)
#define BEIF_FW_CTRL		(BEIF_HOST_CTRL + sizeof(struct beif_ctrl))
#define BEIF_H2C		(BEIF_FW_CTRL + sizeof(struct beif_ctrl))
#define BEIF_BUF_SIZE		((BEIF_EMI_SIZE - sizeof(struct beif_header) - \
				  2 * (sizeof(struct beif_ctrl) + BEIF_GUARD_SIZE)) / 2)
#define BEIF_C2H		(BEIF_H2C + BEIF_BUF_SIZE + BEIF_GUARD_SIZE)

/* WMT over HCI (vendor command 0xfc6f, event 0xe4) */
#define WMT_OPCODE_FUNC_CTRL	0x06
#define WMT_DIR_H2C		0x01
#define WMT_EVT_CODE		0xe4

struct btmtkbgf_dev {
	struct device *dev;
	struct hci_dev *hdev;
	struct mtk_conninfra *ci;
	void __iomem *conn;		/* CONN_INFRA, shared with CONNINFRA/Wi-Fi */
	void __iomem *bgf;
	void __iomem *beif;
	int irq;
	bool radio_on;
	bool bgf_on;

	struct mutex tx_lock;		/* H2C ring producer */
	struct work_struct rx_work;

	/* H:4 reassembly of the C2H byte stream */
	struct sk_buff *rx_skb;
	u8 rx_type;
	unsigned int rx_hlen;		/* header length still to come */
	unsigned int rx_dlen;		/* payload length still to come */

	/* WMT command in flight */
	struct completion wmt_done;
	u8 wmt_op;
	int wmt_status;
};

static u32 conn_rd(struct btmtkbgf_dev *b, u32 off)
{
	return readl(b->conn + off);
}

static void conn_wr(struct btmtkbgf_dev *b, u32 off, u32 val)
{
	writel(val, b->conn + off);
}

static void conn_rmw(struct btmtkbgf_dev *b, u32 off, u32 mask, u32 val)
{
	conn_wr(b, off, (conn_rd(b, off) & ~mask) | (val & mask));
}

#define conn_set(b, off, bits)	conn_rmw(b, off, bits, bits)
#define conn_clr(b, off, bits)	conn_rmw(b, off, bits, 0)

static int conn_poll(struct btmtkbgf_dev *b, u32 off, u32 mask, u32 want)
{
	u32 v;

	/* stock: up to 10 tries, 0.5 ms apart */
	return readl_poll_timeout(b->conn + off, v, (v & mask) == want, 500, 5500);
}

/* bgfsys_check_conninfra_ready(): wake CONN_INFRA for BT and check it */
static int btmtkbgf_conninfra_wake(struct btmtkbgf_dev *b)
{
	u32 v;
	int ret;

	conn_set(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));
	ret = readl_poll_timeout(b->conn + CFG_IP_VERSION, v,
				 v == MT6878_CONN_IP_VERSION, 1000, 11000);
	if (!ret)
		ret = conn_poll(b, CFG_ON_CONN_INFRA_RDY, CONN_INFRA_RDY,
				CONN_INFRA_RDY);
	if (ret) {
		dev_err(b->dev, "CONN_INFRA not ready for BT (0x%08x)\n",
			conn_rd(b, CFG_IP_VERSION));
		conn_clr(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));
	}
	return ret;
}

static int btmtkbgf_gals(struct btmtkbgf_dev *b, u32 ctl, u32 sts, u32 en,
			 u32 ack, bool protect)
{
	if (protect) {
		conn_set(b, ctl, en);
		return conn_poll(b, sts, ack, ack);
	}
	conn_clr(b, ctl, en);
	return conn_poll(b, sts, ack, 0);
}

/* bgfsys_power_on() with CFG_BT_ATF_SUPPORT == 0 */
static int btmtkbgf_power_on(struct btmtkbgf_dev *b)
{
	u32 val, pc = 0;
	int i, ret;

	ret = btmtkbgf_conninfra_wake(b);
	if (ret)
		return ret;

	/* N10 CPU in reset, BGFSYS on top on */
	conn_clr(b, RGU_BGFSYS_CPU_SW_RST, BGF_CPU_SW_RST_B);
	conn_rmw(b, RGU_BGFSYS_ON_TOP_PWR_CTL, GENMASK(31, 16) | BGFSYS_TOP_PWR_ON,
		 (BGFSYS_TOP_KEY << 16) | BGFSYS_TOP_PWR_ON);
	ret = conn_poll(b, HOST_CONNSYS_PWR_STATES, BGFSYS_ON_ISO_EN,
			BGFSYS_ON_ISO_EN_DONE);
	if (ret) {
		dev_err(b->dev, "BGFSYS on: PWR_STATES 0x%08x\n",
			conn_rd(b, HOST_CONNSYS_PWR_STATES));
		goto err;
	}
	conn_set(b, CFG_BT_PWRCTLCR0, BT_FUNC_EN);

	ret = conn_poll(b, RGU_BGFSYS_OFF_TOP_PWR_ACK, BGF_OFF_PWR_ACK_B,
			BGF_OFF_PWR_ACK_B);
	if (!ret)
		ret = conn_poll(b, RGU_BGFSYS_OFF_TOP_PWR_ACK, BGF_OFF_PWR_ACK_S,
				BGF_OFF_PWR_ACK_S);
	if (ret) {
		dev_err(b->dev, "BGFSYS off top power ack 0x%08x\n",
			conn_rd(b, RGU_BGFSYS_OFF_TOP_PWR_ACK));
		goto err;
	}

	/*
	 * Until the CONN_INFRA<->BGF bus protection is released an access to
	 * a BGF register would hang the bus: no BGF access before this.
	 */
	ret = btmtkbgf_gals(b, CFG_CONN2BT_GALS_SLP_CTL, CFG_CONN2BT_GALS_SLP_STATUS,
			    GALS_SLP_PROT_RX_EN, GALS_SLP_PROT_RX_ACK, false) ?:
	      btmtkbgf_gals(b, CFG_CONN2BT_GALS_SLP_CTL, CFG_CONN2BT_GALS_SLP_STATUS,
			    GALS_SLP_PROT_TX_EN, GALS_SLP_PROT_TX_ACK, false) ?:
	      btmtkbgf_gals(b, CFG_BT2CONN_GALS_SLP_CTL, CFG_BT2CONN_GALS_SLP_STATUS,
			    GALS_SLP_PROT_RX_EN, GALS_SLP_PROT_RX_ACK, false) ?:
	      btmtkbgf_gals(b, CFG_BT2CONN_GALS_SLP_CTL, CFG_BT2CONN_GALS_SLP_STATUS,
			    GALS_SLP_PROT_TX_EN, GALS_SLP_PROT_TX_ACK, false);
	if (ret) {
		dev_err(b->dev, "BGF bus protection not released (0x%08x 0x%08x)\n",
			conn_rd(b, CFG_CONN2BT_GALS_SLP_STATUS),
			conn_rd(b, CFG_BT2CONN_GALS_SLP_STATUS));
		goto err;
	}
	usleep_range(400, 440);

	val = readl(b->bgf + BGF_IP_VERSION);
	if (val != MT6878_BGF_IP_VERSION) {
		dev_err(b->dev, "BGFSYS IP version 0x%08x, expected 0x%08x\n",
			val, MT6878_BGF_IP_VERSION);
		ret = -ENODEV;
		goto err;
	}

	writel(readl(b->bgf + BGF_MCCR) & ~BGF_CON_CR_AHB_AUTO_DIS, b->bgf + BGF_MCCR);
	writel(BGF_CON_CR_AHB_STOP, b->bgf + BGF_MCCR_SET);
	val = readl(b->bgf + BGF_MCUSYS_DLY_CHAIN_CTL) & ~GENMASK(19, 16);
	writel(val | 0x80000, b->bgf + BGF_MCUSYS_DLY_CHAIN_CTL);
	writel(readl(b->bgf + BGF_CKGEN0_FRZ_WIN) | BGF_HCLK_SWITCH_FRZ_WINDOW,
	       b->bgf + BGF_CKGEN0_FRZ_WIN);
	conn_rmw(b, HOST_HOSTCSR2BGF_ON_DBG_SEL, GENMASK(21, 0), 0x300001);

	ret = conn_poll(b, HOST_BGF_MONFLG_ON_OUT, BGF_CONN_INFRA_READY,
			BGF_CONN_INFRA_READY);
	if (ret) {
		dev_err(b->dev, "conn_infra_rdy not set (0x%08x)\n",
			conn_rd(b, HOST_BGF_MONFLG_ON_OUT));
		goto err;
	}

	/* clear the fw own interrupt, release the CPU */
	conn_wr(b, HOST_BGF_IRQ_STAT, BGF_IRQ_FW_OWN_SET);
	conn_wr(b, HOST_BGF_IRQ_STAT2, BGF_IRQ_FW_OWN_SET);
	conn_set(b, RGU_BGFSYS_CPU_SW_RST, BGF_CPU_SW_RST_B);
	conn_rmw(b, HOST_HOSTCSR2BGF_ON_DBG_SEL, GENMASK(21, 0), 0);

	/* the ROM power-on part ends with 0x1d1e in sw_dbg_ctl */
	conn_wr(b, CONN_DBG_MCU_PC_SEL, BGF_MCU_PC_SEL);
	for (i = 0; i < 100; i++) {
		pc = conn_rd(b, CONN_DBG_MCU_PC);
		if (readl(b->bgf + BGF_MCU_CFG_SW_DBG_CTL) == BGF_ROM_IDLE)
			break;
		usleep_range(5000, 5500);
	}
	if (i == 100) {
		dev_err(b->dev, "BGF MCU not idle: sw_dbg_ctl 0x%08x, pc 0x%08x\n",
			readl(b->bgf + BGF_MCU_CFG_SW_DBG_CTL), pc);
		ret = -ETIMEDOUT;
		goto err;
	}

	/* stale software interrupts, then drop the force-on */
	conn_wr(b, CCIF_BGF_SW_IRQ_RESET, BGF_FW_LOG_NOTIFY | BGF_SUBSYS_CHIP_RESET);
	conn_clr(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));
	dev_info(b->dev, "BGFSYS on, MCU pc 0x%08x\n", pc);
	return 0;

err:
	conn_clr(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));
	return ret ?: -EIO;
}

/* bgfsys_power_off() with CFG_BT_ATF_SUPPORT == 0 */
static void btmtkbgf_power_off(struct btmtkbgf_dev *b)
{
	int i;

	if (btmtkbgf_conninfra_wake(b))
		dev_warn(b->dev, "BGFSYS off without CONN_INFRA wakeup\n");

	if (btmtkbgf_gals(b, CFG_BT2CONN_GALS_SLP_CTL, CFG_BT2CONN_GALS_SLP_STATUS,
			  GALS_SLP_PROT_TX_EN, GALS_SLP_PROT_TX_ACK, true) ||
	    btmtkbgf_gals(b, CFG_BT2CONN_GALS_SLP_CTL, CFG_BT2CONN_GALS_SLP_STATUS,
			  GALS_SLP_PROT_RX_EN, GALS_SLP_PROT_RX_ACK, true) ||
	    btmtkbgf_gals(b, CFG_CONN2BT_GALS_SLP_CTL, CFG_CONN2BT_GALS_SLP_STATUS,
			  GALS_SLP_PROT_TX_EN, GALS_SLP_PROT_TX_ACK, true) ||
	    btmtkbgf_gals(b, CFG_CONN2BT_GALS_SLP_CTL, CFG_CONN2BT_GALS_SLP_STATUS,
			  GALS_SLP_PROT_RX_EN, GALS_SLP_PROT_RX_ACK, true))
		/* stock: conninfra whole chip reset here */
		dev_err(b->dev, "BGF bus protection not engaged (0x%08x 0x%08x)\n",
			conn_rd(b, CFG_CONN2BT_GALS_SLP_STATUS),
			conn_rd(b, CFG_BT2CONN_GALS_SLP_STATUS));

	conn_clr(b, CFG_BT_PWRCTLCR0, BT_FUNC_EN);
	conn_rmw(b, RGU_BGFSYS_ON_TOP_PWR_CTL, GENMASK(31, 16) | BGFSYS_TOP_PWR_ON,
		 BGFSYS_TOP_KEY << 16);
	conn_clr(b, RGU_BGFSYS_CPU_SW_RST, BGF_CPU_SW_RST_B);
	usleep_range(1000, 1100);
	conn_wr(b, CCIF_BGF_SW_IRQ_RESET, conn_rd(b, CCIF_BGF_SW_IRQ_STATUS) & 0xf);

	/* A-die top_ck_en_2 off */
	conn_clr(b, WT_SLP_TOP_CK_2, TOP_CK_EN);
	if (conn_poll(b, WT_SLP_TOP_CK_2, TOP_CK_BSY, 0))
		dev_err(b->dev, "A-die top_ck_en_2 off timed out\n");

	/* release the semaphores the BGF MCU may hold */
	for (i = 0; i < CONN_SEMA_NUM; i++)
		conn_wr(b, CONN_SEMA_M1_OWN_REL(i), 1);
	conn_clr(b, RGU_SEMA_M1_SW_RST_B, BIT(0));
	conn_set(b, RGU_SEMA_M1_SW_RST_B, BIT(0));

	/* clear bt_emi_req and the bus/clock requests */
	conn_set(b, CFG_EMI_CTL_BT, BT_EMI_REQ);
	conn_clr(b, CFG_EMI_CTL_BT, BT_EMI_REQ);
	conn_set(b, CFG_EMI_CTL_BT, BT_SW_CONN_SRCCLKENA);
	conn_clr(b, CFG_EMI_CTL_BT, BT_SW_CONN_SRCCLKENA | BT_SW_CONN_AP_BUS_REQ |
		 BT_SW_CONN_APSRC_REQ);
	conn_set(b, CFG_EMI_CTL_BT, BT_INFRA_REQ);
	conn_clr(b, CFG_EMI_CTL_BT, BT_INFRA_REQ);

	conn_clr(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));
	dev_info(b->dev, "BGFSYS off\n");
}

/* ------------------------------------------------------------------ */
/* BEIF                                                                */

static u32 beif_rd(struct btmtkbgf_dev *b, u32 off)
{
	return readl(b->beif + off);
}

static void beif_wr(struct btmtkbgf_dev *b, u32 off, u32 val)
{
	writel(val, b->beif + off);
}

/* beif_reset(): the host lays out the rings before the first packet */
static void btmtkbgf_beif_reset(struct btmtkbgf_dev *b)
{
	unsigned int i;

	beif_wr(b, offsetof(struct beif_header, key), BEIF_HEADER_KEY);
	beif_wr(b, offsetof(struct beif_header, version), 0);
	beif_wr(b, offsetof(struct beif_header, host_ctrl_offset), BEIF_HOST_CTRL);
	beif_wr(b, offsetof(struct beif_header, fw_ctrl_offset), BEIF_FW_CTRL);
	beif_wr(b, offsetof(struct beif_header, h2c_offset), BEIF_H2C);
	beif_wr(b, offsetof(struct beif_header, h2c_size), BEIF_BUF_SIZE);
	beif_wr(b, offsetof(struct beif_header, c2h_offset), BEIF_C2H);
	beif_wr(b, offsetof(struct beif_header, c2h_size), BEIF_BUF_SIZE);

	for (i = BEIF_HOST_CTRL; i < BEIF_H2C; i += sizeof(struct beif_ctrl)) {
		beif_wr(b, i + offsetof(struct beif_ctrl, tx_push), 0);
		beif_wr(b, i + offsetof(struct beif_ctrl, rx_pop), 0);
		beif_wr(b, i + offsetof(struct beif_ctrl, guard), BEIF_GUARD_PATTERN);
	}

	memset_io(b->beif + BEIF_H2C, 0, BEIF_BUF_SIZE);
	memset_io(b->beif + BEIF_C2H, 0, BEIF_BUF_SIZE);
	for (i = 0; i < BEIF_GUARD_SIZE; i += 4) {
		beif_wr(b, BEIF_H2C + BEIF_BUF_SIZE + i, BEIF_GUARD_PATTERN);
		beif_wr(b, BEIF_C2H + BEIF_BUF_SIZE + i, BEIF_GUARD_PATTERN);
	}

	b->rx_skb = NULL;
	b->rx_hlen = 0;
	b->rx_dlen = 0;
}

static int btmtkbgf_beif_send(struct btmtkbgf_dev *b, const u8 *data, u32 len)
{
	const u32 h_push = BEIF_HOST_CTRL + offsetof(struct beif_ctrl, tx_push);
	const u32 f_pop = BEIF_FW_CTRL + offsetof(struct beif_ctrl, rx_pop);
	u32 push, pop, space, first;
	int retry;

	if (len >= BEIF_BUF_SIZE)
		return -EMSGSIZE;

	mutex_lock(&b->tx_lock);
	for (retry = 0; retry < 100; retry++) {
		push = beif_rd(b, h_push);
		pop = beif_rd(b, f_pop);
		if (push >= BEIF_BUF_SIZE || pop >= BEIF_BUF_SIZE) {
			mutex_unlock(&b->tx_lock);
			dev_err(b->dev, "BEIF H2C ring corrupt (%u/%u)\n", push, pop);
			return -EIO;
		}
		space = push >= pop ? BEIF_BUF_SIZE - (push - pop) - 1 :
				      pop - push - 1;
		if (space >= len)
			break;
		usleep_range(100, 200);
	}
	if (retry == 100) {
		mutex_unlock(&b->tx_lock);
		return -EBUSY;
	}

	first = min(len, BEIF_BUF_SIZE - push);
	memcpy_toio(b->beif + BEIF_H2C + push, data, first);
	if (first < len)
		memcpy_toio(b->beif + BEIF_H2C, data + first, len - first);
	/* data before the producer index */
	wmb();
	beif_wr(b, h_push, (push + len) % BEIF_BUF_SIZE);
	conn_wr(b, CCIF_BGF_MCUCIRQ, BGF_BEIF_WAKEUP);
	mutex_unlock(&b->tx_lock);

	return len;
}

/* ------------------------------------------------------------------ */
/* H:4 stream to HCI frames                                            */

static bool btmtkbgf_wmt_event(struct btmtkbgf_dev *b, struct sk_buff *skb)
{
	struct hci_event_hdr *hdr = (void *)skb->data;
	const u8 *wmt = skb->data + sizeof(*hdr);

	/* e4 <len> | dir 02, op, len16 | status ... */
	if (hci_skb_pkt_type(skb) != HCI_EVENT_PKT || hdr->evt != WMT_EVT_CODE ||
	    skb->len < sizeof(*hdr) + 4 || !b->wmt_op || wmt[1] != b->wmt_op)
		return false;

	b->wmt_status = skb->len > sizeof(*hdr) + 4 ? wmt[4] : 0;
	b->wmt_op = 0;
	complete(&b->wmt_done);
	kfree_skb(skb);
	return true;
}

static void btmtkbgf_frame(struct btmtkbgf_dev *b)
{
	struct sk_buff *skb = b->rx_skb;

	b->rx_skb = NULL;
	if (btmtkbgf_wmt_event(b, skb))
		return;
	hci_recv_frame(b->hdev, skb);
}

static void btmtkbgf_recv(struct btmtkbgf_dev *b, const u8 *p, unsigned int n)
{
	while (n) {
		unsigned int take;

		if (!b->rx_skb) {
			b->rx_type = *p++;
			n--;
			switch (b->rx_type) {
			case HCI_EVENT_PKT:
				b->rx_hlen = HCI_EVENT_HDR_SIZE;
				break;
			case HCI_ACLDATA_PKT:
				b->rx_hlen = HCI_ACL_HDR_SIZE;
				break;
			case HCI_SCODATA_PKT:
				b->rx_hlen = HCI_SCO_HDR_SIZE;
				break;
			case HCI_ISODATA_PKT:
				b->rx_hlen = HCI_ISO_HDR_SIZE;
				break;
			default:
				dev_err_ratelimited(b->dev, "bad H:4 type 0x%02x\n",
						    b->rx_type);
				b->hdev->stat.err_rx++;
				continue;
			}
			b->rx_skb = bt_skb_alloc(HCI_MAX_FRAME_SIZE, GFP_KERNEL);
			if (!b->rx_skb)
				return;
			hci_skb_pkt_type(b->rx_skb) = b->rx_type;
			b->rx_dlen = 0;
			continue;
		}

		if (b->rx_hlen) {
			take = min(n, b->rx_hlen);
			skb_put_data(b->rx_skb, p, take);
			p += take;
			n -= take;
			b->rx_hlen -= take;
			if (b->rx_hlen)
				continue;

			switch (b->rx_type) {
			case HCI_EVENT_PKT:
				b->rx_dlen = hci_event_hdr(b->rx_skb)->plen;
				break;
			case HCI_ACLDATA_PKT:
				b->rx_dlen = __le16_to_cpu(hci_acl_hdr(b->rx_skb)->dlen);
				break;
			case HCI_SCODATA_PKT:
				b->rx_dlen = hci_sco_hdr(b->rx_skb)->dlen;
				break;
			case HCI_ISODATA_PKT:
				b->rx_dlen = hci_iso_data_len(
					__le16_to_cpu(hci_iso_hdr(b->rx_skb)->dlen));
				break;
			}
			if (b->rx_dlen > skb_tailroom(b->rx_skb)) {
				dev_err_ratelimited(b->dev, "H:4 frame too long (%u)\n",
						    b->rx_dlen);
				kfree_skb(b->rx_skb);
				b->rx_skb = NULL;
				b->hdev->stat.err_rx++;
				continue;
			}
			if (!b->rx_dlen)
				btmtkbgf_frame(b);
			continue;
		}

		take = min(n, b->rx_dlen);
		skb_put_data(b->rx_skb, p, take);
		p += take;
		n -= take;
		b->rx_dlen -= take;
		if (!b->rx_dlen)
			btmtkbgf_frame(b);
	}
}

/* beif_data_consummer(): drain the C2H ring */
static void btmtkbgf_rx_work(struct work_struct *work)
{
	struct btmtkbgf_dev *b = container_of(work, struct btmtkbgf_dev, rx_work);
	const u32 h_pop = BEIF_HOST_CTRL + offsetof(struct beif_ctrl, rx_pop);
	const u32 f_push = BEIF_FW_CTRL + offsetof(struct beif_ctrl, tx_push);
	u8 buf[256];
	u32 push, pop, n;

	pop = beif_rd(b, h_pop);
	for (;;) {
		push = beif_rd(b, f_push);
		if (push >= BEIF_BUF_SIZE || pop >= BEIF_BUF_SIZE) {
			dev_err(b->dev, "BEIF C2H ring corrupt (%u/%u)\n", push, pop);
			return;
		}
		if (push == pop)
			break;
		n = push > pop ? push - pop : BEIF_BUF_SIZE - pop;
		n = min_t(u32, n, sizeof(buf));
		/* index before the data it covers */
		rmb();
		memcpy_fromio(buf, b->beif + BEIF_C2H + pop, n);
		btmtkbgf_recv(b, buf, n);
		pop = (pop + n) % BEIF_BUF_SIZE;
		beif_wr(b, h_pop, pop);
	}
}

/* BGF2AP software interrupt */
static irqreturn_t btmtkbgf_irq(int irq, void *data)
{
	struct btmtkbgf_dev *b = data;
	u32 status;

	if (!READ_ONCE(b->bgf_on))
		return IRQ_NONE;

	if (btmtkbgf_conninfra_wake(b))
		return IRQ_NONE;
	status = conn_rd(b, CCIF_BGF_SW_IRQ_STATUS);
	conn_wr(b, CCIF_BGF_SW_IRQ_RESET, status & 0xf);
	conn_clr(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));

	if (status == 0xdeadfeed || !(status & 0xf))
		return IRQ_NONE;

	if (status & BGF_FW2AP_NOTIFY)
		schedule_work(&b->rx_work);
	if (status & (BGF_SUBSYS_CHIP_RESET | BGF_WHOLE_CHIP_RESET)) {
		dev_err(b->dev, "firmware requests a %s reset (0x%x)\n",
			status & BGF_WHOLE_CHIP_RESET ? "chip" : "subsystem",
			status);
		hci_reset_dev(b->hdev);
	}

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* hci_dev                                                             */

static int btmtkbgf_wmt_func_ctrl(struct btmtkbgf_dev *b, bool on)
{
	/* 01 6f fc 06 | 01 06 02 00 | subsys 0, on */
	const u8 cmd[] = { HCI_COMMAND_PKT, 0x6f, 0xfc, 0x06,
			   WMT_DIR_H2C, WMT_OPCODE_FUNC_CTRL, 0x02, 0x00,
			   0x00, on };
	int ret;

	reinit_completion(&b->wmt_done);
	b->wmt_status = -EIO;
	b->wmt_op = WMT_OPCODE_FUNC_CTRL;

	ret = btmtkbgf_beif_send(b, cmd, sizeof(cmd));
	if (ret < 0)
		goto out;

	/* stock: about 0.8 s for "on" */
	if (!wait_for_completion_timeout(&b->wmt_done, msecs_to_jiffies(3000))) {
		ret = -ETIMEDOUT;
		goto out;
	}
	ret = b->wmt_status ? -EIO : 0;
out:
	b->wmt_op = 0;
	if (ret)
		dev_err(b->dev, "WMT func %s failed: %d\n", on ? "on" : "off", ret);
	return ret;
}

static void btmtkbgf_hw_off(struct btmtkbgf_dev *b)
{
	if (b->bgf_on) {
		disable_irq(b->irq);
		WRITE_ONCE(b->bgf_on, false);
		cancel_work_sync(&b->rx_work);
		btmtkbgf_power_off(b);
	}
	if (b->radio_on) {
		mtk_conninfra_radio_off(b->ci, MTK_CONNINFRA_BT);
		b->radio_on = false;
	}
	kfree_skb(b->rx_skb);
	b->rx_skb = NULL;
}

static int btmtkbgf_open(struct hci_dev *hdev)
{
	struct btmtkbgf_dev *b = hci_get_drvdata(hdev);
	int ret;

	ret = mtk_conninfra_radio_on(b->ci, MTK_CONNINFRA_BT);
	if (ret)
		return ret;
	b->radio_on = true;

	ret = btmtkbgf_power_on(b);
	if (ret)
		goto err;

	btmtkbgf_beif_reset(b);
	WRITE_ONCE(b->bgf_on, true);
	enable_irq(b->irq);

	return 0;
err:
	btmtkbgf_hw_off(b);
	return ret;
}

static int btmtkbgf_close(struct hci_dev *hdev)
{
	struct btmtkbgf_dev *b = hci_get_drvdata(hdev);

	if (b->bgf_on)
		btmtkbgf_wmt_func_ctrl(b, false);
	btmtkbgf_hw_off(b);

	return 0;
}

static int btmtkbgf_flush(struct hci_dev *hdev)
{
	return 0;
}

static int btmtkbgf_setup(struct hci_dev *hdev)
{
	struct btmtkbgf_dev *b = hci_get_drvdata(hdev);

	/*
	 * The firmware is already running (boot loader image in EMI); turn
	 * the BT function on as the MediaTek downstream driver does after power on.
	 * TODO: antenna/connfem WMT settings, UTC time sync.
	 */
	return btmtkbgf_wmt_func_ctrl(b, true);
}

static int btmtkbgf_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtkbgf_dev *b = hci_get_drvdata(hdev);
	int ret;

	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;
	case HCI_ISODATA_PKT:
		break;
	default:
		kfree_skb(skb);
		return -EILSEQ;
	}

	memcpy(skb_push(skb, 1), &hci_skb_pkt_type(skb), 1);
	ret = btmtkbgf_beif_send(b, skb->data, skb->len);
	if (ret < 0) {
		hdev->stat.err_tx++;
		kfree_skb(skb);
		return ret;
	}

	hdev->stat.byte_tx += skb->len;
	kfree_skb(skb);
	return 0;
}

static int btmtkbgf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct btmtkbgf_dev *b;
	struct hci_dev *hdev;
	struct resource *res;
	phys_addr_t emi;
	u32 emi_size;
	int ret;

	b = devm_kzalloc(dev, sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;
	b->dev = dev;
	mutex_init(&b->tx_lock);
	INIT_WORK(&b->rx_work, btmtkbgf_rx_work);
	init_completion(&b->wmt_done);

	b->ci = devm_mtk_conninfra_get(dev);
	if (IS_ERR(b->ci))
		return dev_err_probe(dev, PTR_ERR(b->ci), "failed to get CONNINFRA\n");

	/* both windows are shared with CONNINFRA and Wi-Fi: no request */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "bgf");
	if (!res || resource_size(res) < BGF_WINDOW)
		return dev_err_probe(dev, -EINVAL, "need the BGF window\n");
	b->bgf = devm_ioremap(dev, res->start, BGF_WINDOW);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "conninfra");
	if (!res || resource_size(res) < SZ_512K)
		return dev_err_probe(dev, -EINVAL, "need the CONN_INFRA window\n");
	b->conn = devm_ioremap(dev, res->start, resource_size(res));
	if (!b->bgf || !b->conn)
		return -ENOMEM;

	/* BEIF rings at a fixed offset in the CONNSYS EMI of the boot loader */
	ret = mtk_conninfra_get_emi(b->ci, &emi, &emi_size);
	if (ret || emi_size < BEIF_EMI_OFFSET + BEIF_EMI_SIZE)
		return dev_err_probe(dev, -ENOENT,
				     "no CONNSYS EMI (emi-addr from the boot loader)\n");
	b->beif = devm_ioremap(dev, emi + BEIF_EMI_OFFSET, BEIF_EMI_SIZE);
	if (!b->beif)
		return -ENOMEM;

	b->irq = platform_get_irq_byname(pdev, "bgf2ap");
	if (b->irq < 0)
		return b->irq;
	ret = devm_request_threaded_irq(dev, b->irq, NULL, btmtkbgf_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"btmtkbgf", b);
	if (ret)
		return ret;

	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;
	b->hdev = hdev;
	hdev->bus = HCI_VIRTUAL;	/* no HCI_PLATFORM in the bus list */
	hci_set_drvdata(hdev, b);
	hdev->open = btmtkbgf_open;
	hdev->close = btmtkbgf_close;
	hdev->flush = btmtkbgf_flush;
	hdev->setup = btmtkbgf_setup;
	hdev->send = btmtkbgf_send_frame;
	hdev->set_bdaddr = btmtk_set_bdaddr;
	SET_HCIDEV_DEV(hdev, dev);
	/* the BD address is in the stock NVRAM (BT_Addr): take it from DT */
	hci_set_quirk(hdev, HCI_QUIRK_USE_BDADDR_PROPERTY);
	hci_set_quirk(hdev, HCI_QUIRK_NON_PERSISTENT_SETUP);

	ret = hci_register_dev(hdev);
	if (ret) {
		hci_free_dev(hdev);
		return ret;
	}
	platform_set_drvdata(pdev, b);

	return 0;
}

static void btmtkbgf_remove(struct platform_device *pdev)
{
	struct btmtkbgf_dev *b = platform_get_drvdata(pdev);

	hci_unregister_dev(b->hdev);
	btmtkbgf_hw_off(b);
	hci_free_dev(b->hdev);
}

static const struct of_device_id btmtkbgf_of_match[] = {
	{ .compatible = "mediatek,mt6878-bt" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, btmtkbgf_of_match);

static struct platform_driver btmtkbgf_driver = {
	.driver = {
		.name = "btmtkbgf",
		.of_match_table = btmtkbgf_of_match,
	},
	.probe = btmtkbgf_probe,
	.remove = btmtkbgf_remove,
};
module_platform_driver(btmtkbgf_driver);

MODULE_DESCRIPTION("MediaTek MT6878 CONNSYS Bluetooth (BGF) driver");
MODULE_LICENSE("GPL");
