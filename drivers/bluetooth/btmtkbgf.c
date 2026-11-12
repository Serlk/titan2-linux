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
 * the CONNSYS EMI. Before the BGF CPU is released the host lays out the
 * shared parts of that EMI the firmware uses while it boots, as the
 * downstream driver does at probe and power-on: the BEIF rings, the header
 * of the firmware log rings (conninfra connsyslog) and an empty coredump
 * area. HCI packets travel as an H:4 byte stream through the two BEIF
 * rings (host to chip and chip to host), with software interrupts both
 * ways.
 *
 * The firmware log layout and the coredump area follow MediaTek's GPL-2.0
 * conninfra driver (conn_drv/connv2: debug_utility/connsyslog/connsyslog.c,
 * debug_utility/coredump/connsys_coredump.c, platform/mt6878
 * mt6878_connsyslog.h and mt6878_coredump.c), Copyright (c) 2023 MediaTek Inc.
 *
 * Not done: reading the firmware log (it is discarded), coredump, chip
 * reset, antenna/connfem settings, the MD coexistence flag (0x1000_1BF0).
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/ktime.h>
#include <linux/unaligned.h>
#include <linux/time64.h>
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
/* BGF debug bus: select at 0x23a04, read at 0x23a00 */
#define CONN_DBG_MCU_PC			0x23a00
#define CONN_DBG_MCU_PC_SEL		0x23a04
#define  BGF_MCU_PC_SEL			0xc0010d2a
#define CONN_BGF_DUMP			0x23000	/* host csr part: 0xa00..0xa08, 0xe20 */
#define CCIF_BGF_MCUCIRQ		0x3f00c	/* AP to BGF: BEIF data */
#define  BGF_BEIF_WAKEUP		(BIT(0) | BIT(1))
#define CCIF_BGF_SW_IRQ_STATUS		0x3f010
#define CCIF_BGF_SW_IRQ_RESET		0x3f014
#define  BGF_FW_LOG_NOTIFY		BIT(0)
#define  BGF_SUBSYS_CHIP_RESET		BIT(1)
#define  BGF_WHOLE_CHIP_RESET		BIT(2)
#define  BGF_FW2AP_NOTIFY		BIT(3)
#define HOST_BGF_LPCTL			0x60030
#define  BGF_HOST_SET_FW_OWN		BIT(0)
#define  BGF_HOST_CLR_FW_OWN		BIT(1)
#define  BGF_OWNER_STATE_SYNC		BIT(2)
#define HOST_BGF_IRQ_STAT		0x60034
#define HOST_BGF_IRQ_STAT2		0x6003c
#define  BGF_IRQ_FW_OWN_SET		BIT(0)
/* own handshakes: up to 4096 polls of 0.5 ms, as the downstream driver */
#define BGF_OWN_POLL_US			500
#define BGF_OWN_TIMEOUT_US		(4096 * BGF_OWN_POLL_US)
/* idle time after the last frame before the core is given to the firmware */
#define BGF_SLEEP_DELAY_MS		50
#define HOST_DBG_MUX_SEL		0x60128	/* conn_infra/bgf on debug: select */
#define HOST_DBG_MUX_OUT		0x60148
#define HOST_BGF_MCU_PC			0x6022c	/* also LR etc. at 0x230..0x238 */
#define HOST_BGF_MAILBOX		0x60268	/* != 0, != 0x87xxxxxx: BGF bus timed out */
#define HOST_CONN_INFRA_WAKEUP_TOP	0x601a0
#define HOST_CONN_INFRA_WAKEUP_WF	0x601a4
#define HOST_CONN_INFRA_WAKEUP_BT	0x601a8
#define HOST_CONNSYS_PWR_STATES		0x60a10
#define  BGFSYS_ON_ISO_EN		GENMASK(19, 18)
#define  BGFSYS_ON_ISO_EN_DONE		BIT(18)
#define HOST_BGF_MONFLG_ON_OUT		0x60c00
#define  BGF_CONN_INFRA_READY		BIT(15)
#define HOST_HOSTCSR2BGF_ON_DBG_SEL	0x60c04
#define CONN_SEMA_M1_OWN_REL(n)		(0x71200 + 4 * (n))
#define CONN_SEMA_STATUS(m)		(0x70400 + 0x1000 * (m))	/* M0 WF .. M3 */
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

/* the BT firmware version string: a pointer in the log header (downstream driver) */
#define BGF_EMI_FW_VER_PTR		(FWLOG_EMI_OFFSET + FWLOG_FW_VER_PTR)
#define BGF_FW_VER_LEN			64

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

/*
 * BT firmware log rings in the CONNSYS EMI (downstream connsyslog, MT6878 "bt_fw":
 * 64 KiB at 0xb48000, a 32 KiB primary and a 16 KiB MCU block). Header:
 * per block its offset and size (8 bytes each), the AP state at 0x20
 * (1: resumed), the interrupt counter at 0x30 and the ready pattern at
 * 0x38. Each block: read and write offsets in a 32 byte sub-header, the
 * ring, a 32 byte end pattern, 32 bytes reserve. The firmware also keeps
 * the pointer to its version string at 0x24 of this header.
 */
#define FWLOG_EMI_OFFSET		0xb48000
#define FWLOG_EMI_SIZE			SZ_64K
#define FWLOG_HDR_SIZE			64
#define FWLOG_AP_STATE			0x20
#define FWLOG_IRQ_COUNTER		0x30
#define FWLOG_READY			0x38
#define FWLOG_SUB_HDR			32
#define FWLOG_RESERVE			32
#define FWLOG_END_PATTERN		"FWLOGENDFWLOGENDFWLOGENDFWLOGEND"
#define FWLOG_END_SIZE			32
#define FWLOG_READY_PATTERN		"EMIFWLOG"
#define FWLOG_FW_VER_PTR		0x24

static const u32 btmtkbgf_fwlog_blocks[] = { SZ_32K, SZ_16K };

/* BT coredump area (downstream mt6878_coredump.c), cleared before every start */
#define COREDUMP_EMI_OFFSET		0xb58000
#define COREDUMP_EMI_SIZE		0x18000

/* WMT over HCI (vendor command 0xfc6f, event 0xe4) */
#define WMT_OPCODE_FUNC_CTRL	0x06
#define WMT_OPCODE_MISC		0xf0	/* UTC time, screen state */
#define WMT_DIR_H2C		0x01
#define WMT_EVT_CODE		0xe4

struct btmtkbgf_dev {
	struct device *dev;
	struct hci_dev *hdev;
	struct mtk_conninfra *ci;
	void __iomem *conn;		/* CONN_INFRA, shared with CONNINFRA/Wi-Fi */
	void __iomem *bgf;
	void __iomem *beif;
	void __iomem *fwlog;		/* BT firmware log rings in EMI */
	void __iomem *coredump;		/* BT coredump area in EMI */
	u32 fwlog_irqs;
	phys_addr_t emi;
	u32 emi_size;
	u32 emi_crc;			/* of the BT image, first check */
	bool emi_crc_valid;
	int irq;
	bool radio_on;
	bool bgf_on;

	/*
	 * Low power: the firmware owns the core (and sleeps) while the host
	 * has nothing to send. Transmit takes the core back (driver own);
	 * receive works either way. sleep_work hands it over again after a
	 * short idle time, and system suspend hands it over for good.
	 */
	struct mutex own_lock;
	bool fw_own;
	struct delayed_work sleep_work;
	bool irq_wake;

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

/*
 * Start-up variants for one phone run to tell the hypotheses apart; the
 * defaults are the downstream values (bt/mt66xx, platform_6878.h).
 */
static unsigned int idle_polls = 100;
module_param(idle_polls, uint, 0644);
MODULE_PARM_DESC(idle_polls, "Polls for the BGF ROM idle signature (downstream default 100)");

static unsigned int idle_poll_ms = 5;
module_param(idle_poll_ms, uint, 0644);
MODULE_PARM_DESC(idle_poll_ms, "Interval of those polls, ms (downstream default 5)");

static unsigned int wake_settle_us = 200;
module_param(wake_settle_us, uint, 0644);
MODULE_PARM_DESC(wake_settle_us,
		 "Wait after the CONN_INFRA wakeup before trusting its ready bits, us (downstream default 200)");

static unsigned int start_retries;
module_param(start_retries, uint, 0644);
MODULE_PARM_DESC(start_retries,
		 "BGF power-on retries after a full power-off (downstream default 0)");

static bool pc_trace;
module_param(pc_trace, bool, 0644);
MODULE_PARM_DESC(pc_trace, "Log every BGF MCU PC sample while waiting for idle (the downstream driver does)");

static bool emi_check;
module_param(emi_check, bool, 0644);
MODULE_PARM_DESC(emi_check,
		 "CRC32 of the BT image in the CONNSYS EMI before every start (diagnostic; reads the RO part)");

static bool emi_init = true;
module_param(emi_init, bool, 0644);
MODULE_PARM_DESC(emi_init,
		 "Lay out the BEIF rings and the firmware log header and clear the coredump area in EMI before every start, as the downstream driver (default Y; N: leave the EMI as found)");

static unsigned int emi_bt_size = 0x2e0000;
module_param(emi_bt_size, uint, 0644);
MODULE_PARM_DESC(emi_bt_size, "Size of the BT image in EMI (factory device tree ro-bt-emi-size)");

/*
 * bgfsys_check_conninfra_ready(): wake CONN_INFRA for BT and check it.
 * As the downstream driver: wait a little before polling (a "ready" read too
 * early can be a stale one), make sure the wakeup bit took, retry the
 * whole wakeup three times.
 */
static int btmtkbgf_conninfra_wake(struct btmtkbgf_dev *b)
{
	unsigned int try, i;
	u32 v = 0;

	for (try = 0; try < 3; try++) {
		conn_set(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));
		udelay(wake_settle_us);
		for (i = 0; i < 10 && !conn_rd(b, HOST_CONN_INFRA_WAKEUP_BT); i++) {
			conn_set(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));
			usleep_range(1000, 1100);
		}
		if (readl_poll_timeout(b->conn + CFG_IP_VERSION, v,
				       v == MT6878_CONN_IP_VERSION, 1000, 11000))
			continue;
		if (!conn_poll(b, CFG_ON_CONN_INFRA_RDY, CONN_INFRA_RDY,
			       CONN_INFRA_RDY)) {
			if (try)
				dev_info(b->dev, "CONN_INFRA ready for BT after %u wakeups\n",
					 try + 1);
			return 0;
		}
	}

	dev_err(b->dev, "CONN_INFRA not ready for BT (0x%08x, wakeup 0x%x)\n",
		v, conn_rd(b, HOST_CONN_INFRA_WAKEUP_BT));
	conn_clr(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));
	return -ETIMEDOUT;
}

static void btmtkbgf_print_words(struct btmtkbgf_dev *b, const char *what,
				 const u32 *v, unsigned int n)
{
	char line[8 * 9 + 1];
	unsigned int i, j, len;

	for (i = 0; i < n; i += 8) {
		len = 0;
		for (j = i; j < n && j < i + 8; j++)
			len += scnprintf(line + len, sizeof(line) - len, " %08x", v[j]);
		dev_info(b->dev, "%s[%u]:%s\n", what, i, line);
	}
}

/* BGF debug bus words @first..@last (step @step) through 0x23a04/0x23a00 */
static void btmtkbgf_dump_dbg(struct btmtkbgf_dev *b, const char *what,
			      u32 first, u32 last, u32 step)
{
	u32 v[96], sel;
	unsigned int n = 0;

	for (sel = first; sel <= last && n < ARRAY_SIZE(v); sel += step) {
		/* the downstream driver skips these in the MCUSYS flag walk */
		if (step == 0x200 && (sel == 0xc0010d00 || sel == 0xc0011700 ||
				      sel == 0xc0012500 || sel == 0xc0015d00))
			continue;
		conn_wr(b, CONN_DBG_MCU_PC_SEL, sel);
		v[n++] = conn_rd(b, CONN_DBG_MCU_PC);
	}
	btmtkbgf_print_words(b, what, v, n);
}

/*
 * bgfsys_power_on_dump_cr() and bt_dump_bgfsys_all() of the downstream driver
 * (non-ATF path), plus the BT power-on registers. CONN_INFRA must be
 * awake (HOST_CONN_INFRA_WAKEUP_BT) for everything but the host CSR part,
 * and BGF registers proper are read only with the bus protection off.
 */
static void btmtkbgf_dump(struct btmtkbgf_dev *b, bool awake, bool bgf)
{
	static const u32 host[] = {
		HOST_CONN_INFRA_WAKEUP_TOP, HOST_CONN_INFRA_WAKEUP_WF,
		HOST_CONN_INFRA_WAKEUP_BT, HOST_CONNSYS_PWR_STATES,
		HOST_BGF_MONFLG_ON_OUT, HOST_HOSTCSR2BGF_ON_DBG_SEL,
		HOST_BGF_LPCTL, HOST_BGF_IRQ_STAT, HOST_BGF_IRQ_STAT2,
		0x602cc, HOST_BGF_MCU_PC, 0x60230, 0x60234, 0x60238,
		HOST_BGF_MAILBOX, 0x6026c, 0x60790, 0x60794,
	};
	static const u32 conn[] = {
		RGU_BGFSYS_ON_TOP_PWR_CTL, RGU_BGFSYS_CPU_SW_RST, 0x00414,
		RGU_BGFSYS_OFF_TOP_PWR_ACK, CFG_ON_CONN_INFRA_RDY, CFG_BT_PWRCTLCR0,
		CFG_CONN2BT_GALS_SLP_STATUS, CFG_BT2CONN_GALS_SLP_STATUS,
		0x01150, 0x01170, 0x01610, 0x01614, CFG_EMI_CTL_BT, CFG_IP_VERSION,
		CCIF_BGF_SW_IRQ_STATUS, CONN_SEMA_STATUS(0), CONN_SEMA_STATUS(1),
		CONN_SEMA_STATUS(2), CONN_SEMA_STATUS(3),
	};
	u32 v[24], mailbox;
	unsigned int i, n;
	int sel;

	for (i = 0; i < ARRAY_SIZE(host); i++)
		v[i] = conn_rd(b, host[i]);
	btmtkbgf_print_words(b, "host csr (order of host[])", v, ARRAY_SIZE(host));

	/* conn_infra/bgf on-domain debug mux, as the downstream walk */
	n = 0;
	for (sel = 0xf; sel >= 1; sel--) {
		conn_wr(b, HOST_DBG_MUX_SEL, (sel << 16) | 1);
		v[n++] = conn_rd(b, HOST_DBG_MUX_OUT);
	}
	for (sel = 3; sel >= 1; sel--) {
		conn_wr(b, HOST_DBG_MUX_SEL, (sel << 16) | 2);
		v[n++] = conn_rd(b, HOST_DBG_MUX_OUT);
	}
	for (sel = 4; sel >= 1; sel--) {
		conn_wr(b, HOST_DBG_MUX_SEL, (sel << 16) | 3);
		v[n++] = conn_rd(b, HOST_DBG_MUX_OUT);
	}
	btmtkbgf_print_words(b, "debug mux 0x60128->0x60148 (f..1|1, 3..1|2, 4..1|3)", v, n);

	if (!awake) {
		dev_info(b->dev, "CONN_INFRA not awake: host CSR only\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(conn); i++)
		v[i] = conn_rd(b, conn[i]);
	btmtkbgf_print_words(b, "conn (order of conn[])", v, ARRAY_SIZE(conn));

	n = 0;
	for (i = 0xa00; i <= 0xa08; i += 4)
		v[n++] = conn_rd(b, CONN_BGF_DUMP + i);
	v[n++] = conn_rd(b, CONN_BGF_DUMP + 0xe20);
	btmtkbgf_print_words(b, "bgf host csr 23a00-23a08 23e20", v, n);

	btmtkbgf_dump_dbg(b, "BGF MCU PC/LR log", 0xc0010d00, 0xc0010d54, 1);
	btmtkbgf_dump_dbg(b, "BGF MCU core flags", 0xc0015d00, 0xc0015d25, 1);
	btmtkbgf_dump_dbg(b, "BGF MCUSYS flags", 0xc0010100, 0xc0015d00, 0x200);
	btmtkbgf_dump_dbg(b, "BGF bus flags", 0xc0012510, 0xc00125a0, 0x10);
	btmtkbgf_dump_dbg(b, "BGF MCU DMA flags", 0xc0011700, 0xc0011707, 1);

	/* BGF registers only while the BGF bus has not timed out */
	if (!bgf)
		return;
	mailbox = conn_rd(b, HOST_BGF_MAILBOX);
	if (mailbox && (mailbox & 0xff000000) != 0x87000000) {
		dev_info(b->dev, "BGF bus timed out (mailbox 0x%08x): no BGF reads\n",
			 mailbox);
		return;
	}
	n = 0;
	v[n++] = readl(b->bgf + BGF_MCU_CFG_SW_DBG_CTL);
	v[n++] = readl(b->bgf + BGF_MCCR);
	for (i = 0x410; i <= 0x444; i += 4)
		v[n++] = readl(b->bgf + i);
	for (i = 0x2214; i <= 0x2220; i += 4)
		v[n++] = readl(b->bgf + i);
	btmtkbgf_print_words(b, "bgf 16c 100 410-444 2214-2220", v, n);
}

/*
 * The BT image in the CONNSYS EMI: the downstream driver logs the firmware
 * version string the image points to; the optional CRC tells whether
 * anything has overwritten the image since the first start.
 */
static void btmtkbgf_fw_version(struct btmtkbgf_dev *b, const char *when)
{
	char ver[BGF_FW_VER_LEN];
	void __iomem *p;
	u32 off;

	/* the firmware writes the pointer once it runs */
	off = readl(b->fwlog + FWLOG_FW_VER_PTR) & 0x00ffffff;
	memset(ver, 0, sizeof(ver));
	if (off && off + sizeof(ver) <= b->emi_size) {
		p = ioremap(b->emi + off, sizeof(ver));
		if (p) {
			memcpy_fromio(ver, p, sizeof(ver) - 1);
			iounmap(p);
		}
	}
	dev_info(b->dev, "FW version pointer (%s) 0x%06x: \"%*pE\"\n", when, off,
		 (int)strnlen(ver, sizeof(ver)), ver);
}

static void btmtkbgf_emi_check(struct btmtkbgf_dev *b)
{
	void __iomem *p;
	u32 off, crc, size, n;
	u8 *buf;

	if (!emi_check)
		return;
	size = min(emi_bt_size, b->emi_size);
	buf = kmalloc(SZ_4K, GFP_KERNEL);
	p = ioremap(b->emi, size);
	if (!buf || !p)
		goto out;
	crc = ~0;
	for (off = 0; off < size; off += n) {
		n = min_t(u32, SZ_4K, size - off);
		memcpy_fromio(buf, p + off, n);
		crc = crc32_le(crc, buf, n);
	}
	if (!b->emi_crc_valid) {
		b->emi_crc = crc;
		b->emi_crc_valid = true;
		dev_info(b->dev, "BT image in EMI: crc32 %08x over 0x%x bytes\n",
			 crc, size);
	} else if (crc != b->emi_crc) {
		dev_warn(b->dev, "BT image in EMI changed: crc32 %08x, first %08x\n",
			 crc, b->emi_crc);
	} else {
		dev_info(b->dev, "BT image in EMI unchanged (crc32 %08x)\n", crc);
	}
out:
	if (p)
		iounmap(p);
	kfree(buf);
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
	u32 val, pc = 0, pcs[16];
	unsigned int i, npcs = 0;
	bool bgf = false;
	ktime_t t0;
	int ret;

	ret = btmtkbgf_conninfra_wake(b);
	if (ret) {
		btmtkbgf_dump(b, false, false);
		return ret;
	}

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
	bgf = true;
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

	/*
	 * The ROM power-on part ends with 0x1d1e in sw_dbg_ctl. Measured with
	 * the factory firmware on this phone: about 82 ms, PCs 0xf0008910 .. 0xf0008138 (the image
	 * in EMI) and then 0x00902b6a.
	 */
	t0 = ktime_get();
	conn_wr(b, CONN_DBG_MCU_PC_SEL, BGF_MCU_PC_SEL);
	for (i = 0; i < idle_polls; i++) {
		pc = conn_rd(b, CONN_DBG_MCU_PC);
		if (pc_trace)
			dev_info(b->dev, "MCU pc 0x%08x\n", pc);
		if (!npcs || pcs[npcs - 1] != pc) {
			if (npcs < ARRAY_SIZE(pcs))
				pcs[npcs++] = pc;
			else
				pcs[ARRAY_SIZE(pcs) - 1] = pc;
		}
		if (readl(b->bgf + BGF_MCU_CFG_SW_DBG_CTL) == BGF_ROM_IDLE)
			break;
		usleep_range(idle_poll_ms * 1000, idle_poll_ms * 1000 + 500);
	}
	if (i == idle_polls) {
		dev_err(b->dev, "BGF MCU not idle: sw_dbg_ctl 0x%08x, pc 0x%08x after %lld ms\n",
			readl(b->bgf + BGF_MCU_CFG_SW_DBG_CTL), pc,
			ktime_ms_delta(ktime_get(), t0));
		btmtkbgf_print_words(b, "MCU pc changes", pcs, npcs);
		ret = -ETIMEDOUT;
		goto err;
	}

	/* stale software interrupts, then drop the force-on */
	conn_wr(b, CCIF_BGF_SW_IRQ_RESET, BGF_FW_LOG_NOTIFY | BGF_SUBSYS_CHIP_RESET);
	conn_clr(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));
	dev_info(b->dev, "BGFSYS on, MCU pc 0x%08x, ROM idle after %lld ms (%u PCs)\n",
		 pc, ktime_ms_delta(ktime_get(), t0), npcs);
	return 0;

err:
	btmtkbgf_dump(b, true, bgf);
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

/* ------------------------------------------------------------------ */
/* firmware log rings and coredump area in EMI                         */

/* where block @i of the log rings starts (its sub-header) */
static u32 btmtkbgf_fwlog_block(unsigned int i)
{
	u32 base = FWLOG_HDR_SIZE;
	unsigned int j;

	for (j = 0; j < i; j++)
		base += FWLOG_SUB_HDR + btmtkbgf_fwlog_blocks[j] + FWLOG_RESERVE;
	return base;
}

/*
 * Read only: the log header, the ring offsets of both blocks and the BEIF
 * header and control words as the firmware sees them.
 */
static void btmtkbgf_emi_state(struct btmtkbgf_dev *b, const char *when)
{
	u32 v[FWLOG_HDR_SIZE / 4];
	char ready[sizeof(FWLOG_READY_PATTERN)];
	unsigned int i;
	u32 blk;

	for (i = 0; i < ARRAY_SIZE(v); i++)
		v[i] = readl(b->fwlog + 4 * i);
	btmtkbgf_print_words(b, "fw log header", v, ARRAY_SIZE(v));

	memcpy_fromio(ready, b->fwlog + FWLOG_READY, sizeof(ready) - 1);
	ready[sizeof(ready) - 1] = 0;
	for (i = 0; i < ARRAY_SIZE(btmtkbgf_fwlog_blocks); i++) {
		blk = btmtkbgf_fwlog_block(i);
		v[2 * i] = readl(b->fwlog + blk);
		v[2 * i + 1] = readl(b->fwlog + blk + 4);
	}
	dev_info(b->dev,
		 "EMI %s: fw log ready \"%*pE\", AP state %u, irqs %u, ring rd/wr %x/%x %x/%x; BEIF key %08x h2c %u/%u c2h %u/%u\n",
		 when, (int)strnlen(ready, sizeof(ready)), ready,
		 readl(b->fwlog + FWLOG_AP_STATE),
		 readl(b->fwlog + FWLOG_IRQ_COUNTER), v[0], v[1], v[2], v[3],
		 beif_rd(b, offsetof(struct beif_header, key)),
		 beif_rd(b, BEIF_HOST_CTRL + offsetof(struct beif_ctrl, tx_push)),
		 beif_rd(b, BEIF_FW_CTRL + offsetof(struct beif_ctrl, rx_pop)),
		 beif_rd(b, BEIF_FW_CTRL + offsetof(struct beif_ctrl, tx_push)),
		 beif_rd(b, BEIF_HOST_CTRL + offsetof(struct beif_ctrl, rx_pop)));
}

/* connlog_emi_init(), connlog_buffer_init() and connlog_set_ring_ready() for "bt_fw" */
static void btmtkbgf_fwlog_init(struct btmtkbgf_dev *b)
{
	unsigned int i;
	u32 blk, size;

	memset_io(b->fwlog, 0xff, FWLOG_EMI_SIZE);
	memset_io(b->fwlog, 0, FWLOG_HDR_SIZE);
	writel(1, b->fwlog + FWLOG_AP_STATE);
	for (i = 0; i < ARRAY_SIZE(btmtkbgf_fwlog_blocks); i++) {
		blk = btmtkbgf_fwlog_block(i);
		size = btmtkbgf_fwlog_blocks[i];
		/* read and write offsets 0 */
		memset_io(b->fwlog + blk, 0, FWLOG_SUB_HDR);
		writel(blk, b->fwlog + 8 * i);
		writel(size, b->fwlog + 8 * i + 4);
		memcpy_toio(b->fwlog + blk + FWLOG_SUB_HDR + size,
			    FWLOG_END_PATTERN, FWLOG_END_SIZE);
	}
	b->fwlog_irqs = 0;
	memcpy_toio(b->fwlog + FWLOG_READY, FWLOG_READY_PATTERN,
		    sizeof(FWLOG_READY_PATTERN) - 1);
}

/*
 * The firmware log is not read out yet: on its interrupt give the space
 * back to the firmware (read offset = write offset), counting interrupts
 * in the header as the downstream reader does.
 */
static void btmtkbgf_fwlog_drain(struct btmtkbgf_dev *b)
{
	unsigned int i;
	u32 blk, wr;

	for (i = 0; i < ARRAY_SIZE(btmtkbgf_fwlog_blocks); i++) {
		blk = btmtkbgf_fwlog_block(i);
		wr = readl(b->fwlog + blk + 4);
		if (wr < btmtkbgf_fwlog_blocks[i])
			writel(wr, b->fwlog + blk);
	}
	writel(++b->fwlog_irqs, b->fwlog + FWLOG_IRQ_COUNTER);
}

/*
 * What the downstream driver has written to the EMI before the BGF CPU
 * starts: the BEIF rings (beif_init() at probe), the log header
 * (connsys_log_init() at probe) and an empty coredump area
 * (connsys_coredump_init() at probe, connsys_coredump_clean() at every
 * power-on). The EMI keeps its contents over a warm reboot; with a cold
 * boot nothing else writes these.
 */
static void btmtkbgf_emi_prepare(struct btmtkbgf_dev *b)
{
	btmtkbgf_emi_state(b, "before the start");
	if (!emi_init)
		return;
	btmtkbgf_beif_reset(b);
	btmtkbgf_fwlog_init(b);
	memset_io(b->coredump, 0, COREDUMP_EMI_SIZE);
}

/* ------------------------------------------------------------------ */
/* driver own / firmware own                                           */

/* btmtk_cif_fw_own_clr(): take the core back from the firmware */
static int btmtkbgf_drv_own(struct btmtkbgf_dev *b)
{
	unsigned int i;
	int ret = 0;

	mutex_lock(&b->own_lock);
	if (!b->fw_own)
		goto out;
	if (btmtkbgf_conninfra_wake(b)) {
		ret = -EIO;
		goto out;
	}
	/* the request is repeated every 16 polls while the core wakes up */
	for (i = 0; i < BGF_OWN_TIMEOUT_US / BGF_OWN_POLL_US; i++) {
		if (!(i % 16))
			conn_wr(b, HOST_BGF_LPCTL, BGF_HOST_CLR_FW_OWN);
		if (!(conn_rd(b, HOST_BGF_LPCTL) & BGF_OWNER_STATE_SYNC))
			break;
		usleep_range(BGF_OWN_POLL_US, BGF_OWN_POLL_US + 50);
	}
	if (conn_rd(b, HOST_BGF_LPCTL) & BGF_OWNER_STATE_SYNC) {
		dev_err(b->dev, "driver own failed (LPCTL 0x%08x)\n",
			conn_rd(b, HOST_BGF_LPCTL));
		ret = -ETIMEDOUT;
	} else {
		conn_wr(b, HOST_BGF_IRQ_STAT, BGF_IRQ_FW_OWN_SET);
		b->fw_own = false;
	}
	conn_clr(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));
out:
	mutex_unlock(&b->own_lock);
	return ret;
}

static bool btmtkbgf_beif_tx_done(struct btmtkbgf_dev *b)
{
	return beif_rd(b, BEIF_HOST_CTRL + offsetof(struct beif_ctrl, tx_push)) ==
	       beif_rd(b, BEIF_FW_CTRL + offsetof(struct beif_ctrl, rx_pop)) &&
	       beif_rd(b, BEIF_HOST_CTRL + offsetof(struct beif_ctrl, rx_pop)) ==
	       beif_rd(b, BEIF_FW_CTRL + offsetof(struct beif_ctrl, tx_push));
}

/* btmtk_cif_fw_own_set(): hand the core to the firmware once the rings are idle */
static int btmtkbgf_fw_own(struct btmtkbgf_dev *b)
{
	int i, ret = 0;

	mutex_lock(&b->own_lock);
	if (b->fw_own || !b->bgf_on)
		goto out;
	for (i = 0; i < 5 && !btmtkbgf_beif_tx_done(b); i++)
		usleep_range(1000, 1200);
	if (!btmtkbgf_beif_tx_done(b)) {
		ret = -EBUSY;
		goto out;
	}
	if (btmtkbgf_conninfra_wake(b)) {
		ret = -EIO;
		goto out;
	}
	conn_wr(b, HOST_BGF_LPCTL, BGF_HOST_SET_FW_OWN);
	/* the hardware asserts the sync bit at once: wait for the own set irq */
	ret = conn_poll(b, HOST_BGF_IRQ_STAT2, BGF_IRQ_FW_OWN_SET, BGF_IRQ_FW_OWN_SET);
	if (ret) {
		dev_err(b->dev, "fw own failed (LPCTL 0x%08x)\n",
			conn_rd(b, HOST_BGF_LPCTL));
	} else {
		conn_wr(b, HOST_BGF_IRQ_STAT2, BGF_IRQ_FW_OWN_SET);
		b->fw_own = true;
	}
	conn_clr(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));
out:
	mutex_unlock(&b->own_lock);
	return ret;
}

static void btmtkbgf_sleep_work(struct work_struct *work)
{
	struct btmtkbgf_dev *b = container_of(work, struct btmtkbgf_dev,
					      sleep_work.work);

	if (btmtkbgf_fw_own(b) == -EBUSY)
		schedule_delayed_work(&b->sleep_work,
				      msecs_to_jiffies(BGF_SLEEP_DELAY_MS));
}

static void btmtkbgf_sleep_later(struct btmtkbgf_dev *b)
{
	mod_delayed_work(system_dfl_wq, &b->sleep_work,
			 msecs_to_jiffies(BGF_SLEEP_DELAY_MS));
}

static int btmtkbgf_beif_send(struct btmtkbgf_dev *b, const u8 *data, u32 len)
{
	const u32 h_push = BEIF_HOST_CTRL + offsetof(struct beif_ctrl, tx_push);
	const u32 f_pop = BEIF_FW_CTRL + offsetof(struct beif_ctrl, rx_pop);
	u32 push, pop, space, first;
	int retry;

	if (len >= BEIF_BUF_SIZE)
		return -EMSGSIZE;

	if (btmtkbgf_drv_own(b))
		return -EIO;

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
	btmtkbgf_sleep_later(b);

	return len;
}

/* ------------------------------------------------------------------ */
/* H:4 stream to HCI frames                                            */

static bool btmtkbgf_wmt_event(struct btmtkbgf_dev *b, struct sk_buff *skb)
{
	struct hci_event_hdr *hdr = (void *)skb->data;
	const u8 *wmt = skb->data + sizeof(*hdr);

	/*
	 * e4 <len> | dir 02, op, len16 | payload. The payload starts with
	 * the status, except for the misc opcode, where the sub-opcode comes
	 * first (observed: e4 06 02 f0 02 00 <sub> <status>).
	 */
	unsigned int st = b->wmt_op == WMT_OPCODE_MISC ? 5 : 4;

	if (hci_skb_pkt_type(skb) != HCI_EVENT_PKT || hdr->evt != WMT_EVT_CODE ||
	    skb->len < sizeof(*hdr) + 4 || !b->wmt_op || wmt[1] != b->wmt_op)
		return false;

	b->wmt_status = skb->len > sizeof(*hdr) + st ? wmt[st] : 0;
	if (b->wmt_status)
		dev_dbg(b->dev, "WMT op 0x%02x status 0x%02x\n", b->wmt_op,
			b->wmt_status);
	b->wmt_op = 0;
	complete(&b->wmt_done);
	kfree_skb(skb);
	return true;
}

/*
 * The firmware reports Connectionless Slave Broadcast (synchronization
 * train/scan) in page 2 of its extended features but answers Read
 * Synchronization Train Parameters (0x0c77) with "unknown command", which
 * makes the HCI init fail. Take the two bits out of the reply.
 */
static bool csb_broken = true;
module_param(csb_broken, bool, 0644);
MODULE_PARM_DESC(csb_broken,
		 "Hide the synchronization train/scan features the firmware claims but does not implement (default Y)");

static void btmtkbgf_fix_event(struct btmtkbgf_dev *b, struct sk_buff *skb)
{
	struct hci_event_hdr *hdr = (void *)skb->data;
	struct hci_ev_cmd_complete *cc;
	struct hci_rp_read_local_ext_features *rp;

	if (hci_skb_pkt_type(skb) != HCI_EVENT_PKT || hdr->evt != HCI_EV_CMD_COMPLETE ||
	    skb->len < sizeof(*hdr) + sizeof(*cc) + sizeof(*rp))
		return;
	cc = (void *)(skb->data + sizeof(*hdr));
	if (__le16_to_cpu(cc->opcode) != HCI_OP_READ_LOCAL_EXT_FEATURES)
		return;
	rp = (void *)(skb->data + sizeof(*hdr) + sizeof(*cc));
	if (rp->status || rp->page != 2 || !csb_broken)
		return;
	if (rp->features[0] & (LMP_SYNC_TRAIN | LMP_SYNC_SCAN))
		dev_info(b->dev, "hiding the synchronization train/scan features (0x%02x)\n",
			 rp->features[0]);
	rp->features[0] &= ~(LMP_SYNC_TRAIN | LMP_SYNC_SCAN);
}

static void btmtkbgf_frame(struct btmtkbgf_dev *b)
{
	struct sk_buff *skb = b->rx_skb;

	b->rx_skb = NULL;
	if (btmtkbgf_wmt_event(b, skb))
		return;
	btmtkbgf_fix_event(b, skb);
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
	btmtkbgf_sleep_later(b);
}

/* BGF2AP software interrupt */
static irqreturn_t btmtkbgf_irq(int irq, void *data)
{
	struct btmtkbgf_dev *b = data;
	u32 status;

	if (!READ_ONCE(b->bgf_on))
		return IRQ_NONE;

	/* the controller may have woken the system: let the policy know */
	pm_wakeup_event(b->dev, 0);

	if (btmtkbgf_conninfra_wake(b))
		return IRQ_NONE;
	status = conn_rd(b, CCIF_BGF_SW_IRQ_STATUS);
	conn_wr(b, CCIF_BGF_SW_IRQ_RESET, status & 0xf);
	conn_clr(b, HOST_CONN_INFRA_WAKEUP_BT, BIT(0));

	if (status == 0xdeadfeed || !(status & 0xf))
		return IRQ_NONE;

	if (status & BGF_FW2AP_NOTIFY)
		schedule_work(&b->rx_work);
	if (status & BGF_FW_LOG_NOTIFY)
		btmtkbgf_fwlog_drain(b);
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

/* a WMT command (HCI opcode 0xfc6f) and its 0xe4 event, matched by WMT opcode */
static int btmtkbgf_wmt_cmd(struct btmtkbgf_dev *b, const u8 *cmd, size_t len,
			    const char *what)
{
	int ret;

	reinit_completion(&b->wmt_done);
	b->wmt_status = -EIO;
	b->wmt_op = cmd[5];

	ret = btmtkbgf_beif_send(b, cmd, len);
	if (ret < 0)
		goto out;

	/* factory firmware: about 0.8 s for the function on */
	if (!wait_for_completion_timeout(&b->wmt_done, msecs_to_jiffies(3000))) {
		ret = -ETIMEDOUT;
		goto out;
	}
	ret = b->wmt_status ? -EIO : 0;
out:
	b->wmt_op = 0;
	if (ret)
		dev_err(b->dev, "WMT %s failed: %d\n", what, ret);
	return ret;
}

static int btmtkbgf_wmt_func_ctrl(struct btmtkbgf_dev *b, bool on)
{
	/* 01 6f fc 06 | 01 06 02 00 | subsys 0, on */
	const u8 cmd[] = { HCI_COMMAND_PKT, 0x6f, 0xfc, 0x06,
			   WMT_DIR_H2C, WMT_OPCODE_FUNC_CTRL, 0x02, 0x00,
			   0x00, on };

	return btmtkbgf_wmt_cmd(b, cmd, sizeof(cmd), on ? "func on" : "func off");
}

/* the firmware's clock for its logs: UTC seconds and microseconds */
static int btmtkbgf_wmt_utc_sync(struct btmtkbgf_dev *b)
{
	struct timespec64 ts;
	u8 cmd[] = { HCI_COMMAND_PKT, 0x6f, 0xfc, 0x0c,
		     WMT_DIR_H2C, WMT_OPCODE_MISC, 0x09, 0x00, 0x02,
		     0, 0, 0, 0, 0, 0, 0, 0 };

	ktime_get_real_ts64(&ts);
	put_unaligned_le32((u32)ts.tv_sec, cmd + 9);
	put_unaligned_le32((u32)(ts.tv_nsec / NSEC_PER_USEC), cmd + 13);
	return btmtkbgf_wmt_cmd(b, cmd, sizeof(cmd), "UTC sync");
}

/* screen on (1) / off (0): the firmware tunes its scheduling to it */
static int btmtkbgf_wmt_blank_status(struct btmtkbgf_dev *b, bool screen_on)
{
	const u8 cmd[] = { HCI_COMMAND_PKT, 0x6f, 0xfc, 0x06,
			   WMT_DIR_H2C, WMT_OPCODE_MISC, 0x02, 0x00, 0x03,
			   screen_on };

	return btmtkbgf_wmt_cmd(b, cmd, sizeof(cmd), "screen state");
}

static void btmtkbgf_hw_off(struct btmtkbgf_dev *b)
{
	if (b->bgf_on) {
		disable_irq(b->irq);
		cancel_delayed_work_sync(&b->sleep_work);
		WRITE_ONCE(b->bgf_on, false);
		cancel_work_sync(&b->rx_work);
		btmtkbgf_power_off(b);
		b->fw_own = false;
	}
	if (b->radio_on) {
		mtk_conninfra_bt_started(b->ci);
		mtk_conninfra_radio_off(b->ci, MTK_CONNINFRA_BT);
		b->radio_on = false;
	}
	kfree_skb(b->rx_skb);
	b->rx_skb = NULL;
}

static int btmtkbgf_open(struct hci_dev *hdev)
{
	struct btmtkbgf_dev *b = hci_get_drvdata(hdev);
	unsigned int try;
	int ret;

	ret = mtk_conninfra_radio_on(b->ci, MTK_CONNINFRA_BT);
	if (ret)
		return ret;
	b->radio_on = true;

	btmtkbgf_emi_check(b);
	for (try = 0; ; try++) {
		btmtkbgf_emi_prepare(b);
		ret = btmtkbgf_power_on(b);
		if (!ret)
			break;
		btmtkbgf_emi_state(b, "after the failed start");
		btmtkbgf_fw_version(b, "failed start");
		mtk_conninfra_dump(b->ci, "BGF power-on failed");
		/*
		 * As the downstream bt_hw_and_mcu_on(): a failed power-on is
		 * followed by the full power-off, so that no half-started
		 * BGF (CPU out of reset, M1 semaphores, EMI/infra requests)
		 * stays behind on the shared CONN_INFRA.
		 */
		btmtkbgf_power_off(b);
		if (try >= start_retries)
			goto err;
		dev_info(b->dev, "BGF power-on retry %u\n", try + 1);
		msleep(20);
	}
	btmtkbgf_fw_version(b, "running");

	btmtkbgf_beif_reset(b);
	b->fw_own = false;
	WRITE_ONCE(b->bgf_on, true);
	enable_irq(b->irq);
	/* the downstream driver keeps the core awake for a second after on */
	mod_delayed_work(system_dfl_wq, &b->sleep_work, msecs_to_jiffies(1000));

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

/*
 * On the factory firmware the BD address lives in the NVRAM file APCFG/APRDEB/BT_Addr (the
 * Android HAL sets it); a copy installed as mediatek/mt6878/BT_Addr is
 * used here, first six bytes, in the printed (big endian) order. Without
 * it the controller stays unconfigured until "btmgmt public-addr".
 */
#define BTMTKBGF_NVRAM_ADDR	"mediatek/mt6878/BT_Addr"

static bool nvram_addr_le;
module_param(nvram_addr_le, bool, 0444);
MODULE_PARM_DESC(nvram_addr_le, "BT_Addr file holds the address least significant byte first (default: printed order)");

static void btmtkbgf_nvram_bdaddr(struct btmtkbgf_dev *b)
{
	const struct firmware *fw;
	bdaddr_t addr;
	int i;

	if (bacmp(&b->hdev->public_addr, BDADDR_ANY))
		return;
	if (firmware_request_nowarn(&fw, BTMTKBGF_NVRAM_ADDR, b->dev))
		return;
	if (fw->size >= 6) {
		for (i = 0; i < 6; i++)
			addr.b[i] = nvram_addr_le ? fw->data[i] : fw->data[5 - i];
		if (bacmp(&addr, BDADDR_ANY)) {
			bacpy(&b->hdev->public_addr, &addr);
			dev_info(b->dev, "BD address %pMR from %s\n", &addr,
				 BTMTKBGF_NVRAM_ADDR);
		}
	}
	release_firmware(fw);
}

/*
 * HCI init steps that, if the firmware rejects them, end the whole init:
 * switch them off for a test run without rebuilding. Bit 0: erroneous data
 * reporting (0x0c5a/0x0c5b), bit 1: MWS transport configuration (0x0c6f),
 * bit 2: page 2 of the local extended features altogether.
 */
static unsigned int hci_skip;
module_param(hci_skip, uint, 0644);
MODULE_PARM_DESC(hci_skip,
		 "HCI init steps to skip: 1 erroneous data reporting, 2 MWS transport config, 4 extended features page 2");

static int btmtkbgf_setup(struct hci_dev *hdev)
{
	struct btmtkbgf_dev *b = hci_get_drvdata(hdev);
	int ret;

	if (hci_skip & BIT(0))
		hci_set_quirk(hdev, HCI_QUIRK_BROKEN_ERR_DATA_REPORTING);
	if (hci_skip & BIT(1))
		hci_set_quirk(hdev, HCI_QUIRK_BROKEN_MWS_TRANSPORT_CONFIG);
	if (hci_skip & BIT(2))
		hci_set_quirk(hdev, HCI_QUIRK_BROKEN_LOCAL_EXT_FEATURES_PAGE_2);

	/* the HCI core sends it with set_bdaddr once setup is done */
	btmtkbgf_nvram_bdaddr(b);

	/*
	 * The firmware is already running (boot loader image in EMI); turn
	 * the BT function on as the MediaTek downstream driver does after
	 * power on, then tell it the time and that the screen is on (both
	 * are informational: failures are logged, not fatal). Not done:
	 * the antenna/connfem settings, calibration data backup.
	 */
	ret = btmtkbgf_wmt_func_ctrl(b, true);
	/* end of the BT start: a waiting WFSYS power-on may go on */
	mtk_conninfra_bt_started(b->ci);
	if (ret)
		return ret;
	btmtkbgf_wmt_utc_sync(b);
	btmtkbgf_wmt_blank_status(b, true);
	return 0;
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

/* the HCI core keeps the connections through suspend when we can wake */
static bool btmtkbgf_wakeup(struct hci_dev *hdev)
{
	struct btmtkbgf_dev *b = hci_get_drvdata(hdev);

	return device_may_wakeup(b->dev);
}

/*
 * System sleep: the core goes to the firmware (it sleeps on its own and
 * keeps its links), and its software interrupt wakes the system, as the
 * downstream driver has it (enable_irq_wake on BGF2AP). The CONN power
 * domain stays on behind CONNINFRA.
 */
static int btmtkbgf_suspend(struct device *dev)
{
	struct btmtkbgf_dev *b = dev_get_drvdata(dev);

	if (!b->bgf_on)
		return 0;
	cancel_delayed_work_sync(&b->sleep_work);
	flush_work(&b->rx_work);
	if (btmtkbgf_fw_own(b))
		dev_warn(dev, "suspending with the core still in driver own\n");
	if (device_may_wakeup(dev) && !enable_irq_wake(b->irq))
		b->irq_wake = true;
	return 0;
}

static int btmtkbgf_resume(struct device *dev)
{
	struct btmtkbgf_dev *b = dev_get_drvdata(dev);

	if (b->irq_wake) {
		disable_irq_wake(b->irq);
		b->irq_wake = false;
	}
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(btmtkbgf_pm_ops, btmtkbgf_suspend, btmtkbgf_resume);

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
	mutex_init(&b->own_lock);
	INIT_WORK(&b->rx_work, btmtkbgf_rx_work);
	INIT_DELAYED_WORK(&b->sleep_work, btmtkbgf_sleep_work);
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
	b->fwlog = devm_ioremap(dev, emi + FWLOG_EMI_OFFSET, FWLOG_EMI_SIZE);
	b->coredump = devm_ioremap(dev, emi + COREDUMP_EMI_OFFSET, COREDUMP_EMI_SIZE);
	if (!b->beif || !b->fwlog || !b->coredump)
		return -ENOMEM;
	b->emi = emi;
	b->emi_size = emi_size;

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
	hdev->wakeup = btmtkbgf_wakeup;
	hdev->set_bdaddr = btmtk_set_bdaddr;
	SET_HCIDEV_DEV(hdev, dev);
	device_init_wakeup(dev, true);
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
		.pm = pm_sleep_ptr(&btmtkbgf_pm_ops),
	},
	.probe = btmtkbgf_probe,
	.remove = btmtkbgf_remove,
};
module_platform_driver(btmtkbgf_driver);

MODULE_DESCRIPTION("MediaTek MT6878 CONNSYS Bluetooth (BGF) driver");
MODULE_LICENSE("GPL");
