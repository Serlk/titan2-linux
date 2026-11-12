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
 * Power management follows MediaTek's AXI glue. When idle (runtime_pm)
 * the core goes to the firmware (fw own) once the firmware has said it
 * may sleep, and the driver lets go of CONN_INFRA, so that the WFSYS and
 * CONN_INFRA can sleep and drop their 26 MHz/PMIC/VCORE requests. In fw
 * own the host touches nothing but the always-on conn_host_csr_top: the
 * WFDMA interrupt only disables its line and asks for driver own (wake
 * CONN_INFRA, host CSR interrupt enables, LPCTL with the EINT check), as
 * MediaTek's interrupt handler does. After a WFSYS sleep the hardware has
 * restored the WFDMA from its backup but restarted the TX rings, which
 * the driver sets back. System sleep: the firmware is told that the host
 * sleeps (a word in the CONNSYS EMI or a CSR bit it names in its
 * capabilities, and the SUSPEND_MODE command), the interrupts are wake-up
 * sources, and with suspend_fw_own the core goes to fw own for the
 * sleep. With WoWLAN "any" configured the association survives the
 * sleep; without it cfg80211 disconnects before suspend.
 *
 * Not done: wake-on-wireless filters (the firmware wakes the host for
 * every frame it passes up), the connfem front end description, transmit
 * power tables from NVRAM.
 */

#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/firmware.h>
#include <linux/inetdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jhash.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/seq_file.h>
#include <linux/soc/mediatek/mtk-conninfra.h>
#include <linux/swiotlb.h>
#include <linux/unaligned.h>

#include "mt7921.h"
#include "../mt76_connac2_mac.h"
#include "../dma.h"
#include "mcu.h"
#include "regd.h"

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

/*
 * conn_host_csr_top (0x7c06_xxxx) is always on: the only part of the window
 * the host may touch while the firmware owns the core and CONN_INFRA sleeps.
 */
#define MT7921_SOC_AON_BASE		0x060000
#define MT7921_SOC_AON_SIZE		0x010000
#define MT7921_SOC_WFSYS_BASE		0x400000

/* driver own wakes the WM MCU through an EINT (CFG_CHECK_DRVOWN_EINT) */
#define MT7921_SOC_EINT_SEL		0x7c060b00	/* bit 0: EINT CR select */
#define MT7921_SOC_EINT_DBG_SEL		0x7c060b04	/* [4:0] = 6 */
#define MT7921_SOC_EINT_DBG_SEL2	0x7c060b14	/* [2:0] = 1 */
#define MT7921_SOC_WF_MCU_STATE		0x7c060a10	/* bit 4: WM MCU not asleep */
#define MT7921_SOC_WF_AON_DBG_FLAG	0x7c060b10	/* bits 0 and 10: EINT lost */
#define MT7921_SOC_EINT_DELAY_US	156		/* 5 periods of 32 kHz */
#define MT7921_SOC_EINT_RETRIES		2

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

/* power management counters, debugfs <dev>/pm */
struct mt7921_soc_pm_stats {
	u32 idle_fw_own;	/* runtime PM: core handed to the firmware */
	u32 idle_not_sleepy;	/* runtime PM: firmware had not said it may sleep */
	u32 idle_rx_pending;	/* runtime PM: a ring had something for the host */
	u32 fw_own_fail;
	u32 drv_own;		/* driver own handshakes from fw own */
	u32 drv_own_fail;
	u32 eint_retry;
	u32 irq_fw_own;		/* WFDMA interrupts with the core in fw own */
	u32 irq_asleep;		/* WFDMA interrupts in the last system sleep */
	u32 wfdma_kept;		/* WFDMA lost nothing over a WFSYS sleep */
	u32 tx_reinit;		/* TX rings set back after a WFSYS sleep */
	u32 full_reinit;	/* whole WFDMA reset after a WFSYS sleep */
	u32 off_limits;		/* register accesses refused in fw own */
	u32 suspend_fw_own;	/* system sleeps with the core in fw own */
	u32 last_fw_own_us;
	u32 last_drv_own_us;
	u32 max_drv_own_us;
	u32 settle_timeout;	/* suspend: the firmware did not settle in time */
	u32 wakepu_left;	/* suspend: WAKEPU_WF found set in fw own */
	u32 fw_own_int_sta;	/* HOST_INT_STA at the last fw own (before ack) */
	u32 wake_int_sta;	/* HOST_INT_STA at the last driver own from fw own */
	u32 wake_mcu_cmd;	/* MCU2HOST software interrupt with it */
	ktime_t fw_own_since;
	u64 fw_own_ns;		/* total time in fw own */
};

/* the station's IPv4 addresses, collected for the ARP offload */
struct mt7921_soc_ip {
	int bss_idx;			/* -1: no connected station */
	int count;
	__be32 addr[IEEE80211_BSS_ARP_ADDR_LIST_LEN];
	__be32 mask[IEEE80211_BSS_ARP_ADDR_LIST_LEN];
};

struct mt7921_soc_dev {
	struct mt792x_dev dev;		/* must be first, see mt76_alloc_device() */
	bool ip_offloaded;		/* addresses given to the firmware for the sleep */
	u8 ip_bss_idx;
	bool wow_on;			/* the firmware wake filter is set for this sleep */
	bool addba_rejected;		/* ADDBA refused for this sleep */
	bool rx_filter_set;		/* the RX filter was narrowed for this sleep */
	struct mtk_conninfra *ci;
	int irq;
	int irq_sw;			/* WF2AP CCIF, optional */
	bool radio_on;
	bool awake;			/* the driver holds a CONN_INFRA wake-up */
	atomic_t ci_refs;		/* CONN_INFRA wake-ups held, incl. awake */
	bool wfsys_on;			/* gates every register access */
	bool sleeping;			/* in system sleep */
	bool irq_wake;
	/* the firmware owns the core: WFSYS (and CONN_INFRA unless held) off limits */
	bool fw_owned;
	spinlock_t own_lock;		/* irq_defer, irq_masked vs. the IRQ path */
	bool irq_defer;			/* the WFDMA interrupt only asks for driver own */
	bool irq_masked;		/* its line is disabled until driver own */
	struct mt7921_soc_pm_stats pm;
	struct dentry *debugfs;

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

/* the reply has no magic */
struct mt7921_soc_phy_rsp_hdr {
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

/* the legacy channel request (CH_PRIVILEGE) needs the CNM mode */
static u8 fw_features = MT792x_FW_CAP_CNM;
module_param(fw_features, byte, 0444);
MODULE_PARM_DESC(fw_features,
		 "Firmware offload capabilities (BIT(7): CNM), not readable from the EMI image");

/*
 * Runtime power save (mt76 pm.enable): after pm.idle_timeout without
 * activity the core goes to fw own and CONN_INFRA is let go, as MediaTek's
 * AXI driver does when idle. debugfs mt76/runtime-pm switches it later.
 */
static bool runtime_pm;
module_param(runtime_pm, bool, 0444);
MODULE_PARM_DESC(runtime_pm,
		 "Hand the core to the firmware (fw own) when idle, let CONN_INFRA sleep (default false; debugfs mt76/runtime-pm switches it)");

static bool fw_own_sleepy = true;
module_param(fw_own_sleepy, bool, 0644);
MODULE_PARM_DESC(fw_own_sleepy,
		 "Runtime PM: fw own only after the firmware's sleepy event, as MediaTek's driver (default true)");

static bool conninfra_sleep = true;
module_param(conninfra_sleep, bool, 0644);
MODULE_PARM_DESC(conninfra_sleep,
		 "Runtime PM: release the CONN_INFRA wake-up while the firmware owns the core (default true)");

static int wfdma_reinit;
module_param(wfdma_reinit, int, 0644);
MODULE_PARM_DESC(wfdma_reinit,
		 "WFDMA after a WFSYS sleep: 0 set back the rings that lost their index (default), 1 always reset the whole WFDMA (mt7921e)");

static unsigned int suspend_settle_ms = 200;
module_param(suspend_settle_ms, uint, 0644);
MODULE_PARM_DESC(suspend_settle_ms,
		 "System sleep: wait up to this long for the firmware to take the commands and report sleepy before fw own (default 200, 0: no wait)");

static bool suspend_ci_dump;
module_param(suspend_ci_dump, bool, 0644);
MODULE_PARM_DESC(suspend_ci_dump,
		 "System sleep: log the shared CONN_INFRA state (wake-ups, power states, A-die) before fw own (default false)");

/*
 * ARP offload for the sleep: the IPv4 addresses of the station go to the
 * firmware (CMD_ID_SET_IP_ADDRESS, as MediaTek's driver on every address
 * change), so that it can answer ARP requests itself instead of waking the
 * host for them; cleared again on resume.
 */
static bool arp_offload = true;
module_param(arp_offload, bool, 0644);
MODULE_PARM_DESC(arp_offload,
		 "System sleep: give the firmware the IPv4 addresses for ARP offload (default true)");

static unsigned int wake_probe = 6;
module_param(wake_probe, uint, 0644);
MODULE_PARM_DESC(wake_probe,
		 "Log this many packets (events, frames) after each system sleep (default 3)");

static bool drv_own_eint = true;
module_param(drv_own_eint, bool, 0644);
MODULE_PARM_DESC(drv_own_eint,
		 "Driver own: check that the EINT woke the WM MCU, as MediaTek's driver (default true)");

static int deep_sleep = -1;
module_param(deep_sleep, int, 0444);
MODULE_PARM_DESC(deep_sleep,
		 "Firmware deep sleep: -1 firmware default, never sent, as MediaTek's AXI driver (default); 0 KeepFullPwr 1; 1 KeepFullPwr 0");

static bool mcu_trace;
module_param(mcu_trace, bool, 0644);
MODULE_PARM_DESC(mcu_trace, "Log every MCU command (id, sequence, length)");

static bool phy_action = true;
module_param(phy_action, bool, 0444);
MODULE_PARM_DESC(phy_action,
		 "Ask the ROM code to calibrate before the firmware starts, as the MediaTek downstream driver");

/*
 * System sleep: MediaTek's AXI driver does nothing at suspend (the core is
 * in whatever state runtime power save left it, fw own when idle); without
 * runtime PM the core has to be handed over here or CONN_INFRA keeps its
 * clock requests through the sleep. The first version of this (06.11.2026)
 * left the data path dead after resume: it switched LPCTL to fw own without
 * telling mt76 (MT76_STATE_PM), so resume never took the core back. On by
 * default since 11.11.2026 (phone runs B1-B3: CONN_INFRA asleep for 97 % of
 * the sleep, Wi-Fi up 1 s after every resume).
 */
static bool suspend_fw_own = true;
module_param(suspend_fw_own, bool, 0644);
MODULE_PARM_DESC(suspend_fw_own,
		 "Hand the core to the firmware (fw own) for system sleep (default true; false: stays driver-owned unless runtime PM)");

/*
 * The first system sleep after boot (three phone runs, 08.11 to 12.11)
 * found HOST_CONN_INFRA_WAKEPU_WF set after fw own, with no holder on our
 * side (conninfra wake-ups 0): CONN_INFRA then stays awake for the whole
 * sleep. Every path of ours that sets it clears it again; who leaves it is
 * not known. With this, a WAKEPU_WF found set once the core is the
 * firmware's is cleared and logged.
 */
static bool suspend_wakepu_clear = true;
module_param(suspend_wakepu_clear, bool, 0644);
MODULE_PARM_DESC(suspend_wakepu_clear,
		 "System sleep in fw own: clear a CONN_INFRA Wi-Fi wake-up (WAKEPU_WF) left set (default true)");

/*
 * Firmware wake filter for the sleep (CMD_ID_SET_WOWLAN, as gen4m's
 * kalWowProcess with "Wow" on; the downstream AXI build ships with it off):
 * the firmware wakes the host only on the detect types below and passes up
 * only the frames of the filter flags, instead of every frame ("any").
 * Defaults: magic packet, disconnect, GTK rekey failure, beacon lost;
 * incoming IPv4/IPv6 TCP SYN and EAPOL requests; pass up unicast, 802.1X
 * and ARP requests for us (with arp_offload the firmware answers ARP).
 * Values: gen4m include/nic_cmd_event.h WOWLAN_DETECT_TYPE_*,
 * WOWLAN_DETECT_TYPE_EXT_*, WOWLAN_FF_*. How the SoC firmware signals the
 * host in this mode is not known from GPL code: wow_hif (gen4m ENUM_HIF_TYPE,
 * its default 3 GPIO with pin 0xff) is a parameter to try.
 */
/* on by default since 12.11.2026: 299 of 300 s asleep with the router's traffic, a ping wakes */
static bool wowlan = true;
module_param(wowlan, bool, 0644);
MODULE_PARM_DESC(wowlan,
		 "System sleep: firmware wake filter instead of waking on every frame (default true)");

static uint wow_detect = 0x1d;
module_param(wow_detect, uint, 0644);
MODULE_PARM_DESC(wow_detect,
		 "Wake filter detect types: 1 magic, 2 any, 4 disconnect, 8 GTK rekey failure, 0x10 beacon lost (default 0x1d)");

static uint wow_detect_ext = 0x0e;
module_param(wow_detect_ext, uint, 0644);
MODULE_PARM_DESC(wow_detect_ext,
		 "Wake filter extended detect types: 2 IPv4 TCP SYN, 4 IPv6 TCP SYN, 8 EAPOL request, 0x20 IPv6 ICMP (default 0x0e)");

static uint wow_filter = 0x70;
module_param(wow_filter, uint, 0644);
MODULE_PARM_DESC(wow_filter,
		 "Wake filter flags: 1 drop all, 2 magic to host, 4 ARP, 8 broadcast/multicast, 0x10 unicast, 0x20 802.1X, 0x40 ARP requests to us (default 0x70)");

static uint wow_hif = 3;
module_param(wow_hif, uint, 0644);
MODULE_PARM_DESC(wow_hif, "Wake filter host signal: 0 SDIO, 1 USB, 2 PCIe, 3 GPIO (default 3, as gen4m)");

static uint wow_gpio = 0xff;
module_param(wow_gpio, uint, 0644);
MODULE_PARM_DESC(wow_gpio, "Wake filter GPIO pin (default 0xff, as gen4m)");

static uint wow_trigger = 3;
module_param(wow_trigger, uint, 0644);
MODULE_PARM_DESC(wow_trigger, "Wake filter trigger level bits (default 3, as gen4m)");

/*
 * Phone runs 12.11 with the wake filter: every early wake-up was the
 * firmware's RX ADDBA event (the AP setting up a block ack session before
 * sending to us), which the firmware passes up whatever the filter. For the
 * sleep the firmware refuses ADDBA (CMD_ID_ADDBA_REJECT), after the resume
 * it accepts it again; the AP then sends without aggregation.
 */
static bool suspend_addba_reject = true;
module_param(suspend_addba_reject, bool, 0644);
MODULE_PARM_DESC(suspend_addba_reject,
		 "System sleep: the firmware refuses the AP's block ack sessions (default true)");

/*
 * RX filter for the sleep (CMD_ID_SET_RX_FILTER, MT76_CONNAC_LEGACY_RX_FILTER_*
 * bits of what to pass up): -1 leaves it as mac80211 set it; e.g. 1 passes
 * only directed frames, 9 directed and broadcast. Restored on resume.
 */
static int suspend_rx_filter = -1;
module_param(suspend_rx_filter, int, 0644);
MODULE_PARM_DESC(suspend_rx_filter,
		 "System sleep: RX filter bits (1 directed, 2 multicast list, 4 all multicast, 8 broadcast), -1 untouched (default)");

/* the firmware deep sleeps while the system sleeps (as mt7921/pci.c does) */

static bool suspend_deep_sleep;
module_param(suspend_deep_sleep, bool, 0644);
MODULE_PARM_DESC(suspend_deep_sleep,
		 "Tell the firmware to deep sleep during system suspend (default false)");

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
/*
 * In fw own the WFSYS may be powered down and CONN_INFRA asleep: only
 * conn_host_csr_top is always reachable, CONN_INFRA while a wake-up is held.
 */
static bool mt7921_soc_off_limits(struct mt792x_dev *dev, u32 addr)
{
	struct mt7921_soc_dev *soc = to_soc(dev);

	if (likely(!READ_ONCE(soc->fw_owned)) ||
	    addr >= MT7921_SOC_REG_IGNORE ||
	    (addr >= MT7921_SOC_AON_BASE &&
	     addr < MT7921_SOC_AON_BASE + MT7921_SOC_AON_SIZE) ||
	    (addr < MT7921_SOC_WFSYS_BASE && atomic_read(&soc->ci_refs)))
		return false;

	soc->pm.off_limits++;
	WARN_ONCE(1, "mt7921-soc: register 0x%06x accessed in fw own\n", addr);

	return true;
}

static u32 mt7921_soc_rr(struct mt76_dev *mdev, u32 offset)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	u32 addr;

	if (WARN_ON_ONCE(!to_soc(dev)->wfsys_on))
		return ~0;

	addr = mt7921_soc_reg_addr(dev, offset);
	if (addr == MT7921_SOC_REG_INVALID || mt7921_soc_off_limits(dev, addr))
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
	if (addr >= MT7921_SOC_REG_IGNORE || mt7921_soc_off_limits(dev, addr))
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
	if (addr == MT7921_SOC_REG_INVALID || mt7921_soc_off_limits(dev, addr))
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

	if (mcu_trace)
		dev_info(mdev->dev, "mcu cmd 0x%08x seq %d len %u\n", cmd,
			 seq ? *seq : -1, skb->len);

	/*
	 * Nothing says that the command ring has moved on (MediaTek's driver
	 * does not enable a TX done interrupt for it either): free what the
	 * WFDMA has taken before queueing the next one.
	 */
	mt76_queue_tx_cleanup(dev, mdev->q_mcu[txq], false);

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

/* ---- fw own / driver own ------------------------------------------------ */

static int mt7921_soc_ci_get(struct mt7921_soc_dev *soc)
{
	int ret = mtk_conninfra_wakeup(soc->ci);

	if (!ret)
		atomic_inc(&soc->ci_refs);
	return ret;
}

static void mt7921_soc_ci_put(struct mt7921_soc_dev *soc)
{
	if (WARN_ON(!atomic_read(&soc->ci_refs)))
		return;
	atomic_dec(&soc->ci_refs);
	mtk_conninfra_sleep(soc->ci);
}

/* the CONN_INFRA wake-up the driver holds while it owns the core */
static int mt7921_soc_hold_conninfra(struct mt7921_soc_dev *soc)
{
	int ret;

	if (soc->awake)
		return 0;
	ret = mt7921_soc_ci_get(soc);
	if (!ret)
		soc->awake = true;
	return ret;
}

static void mt7921_soc_release_conninfra(struct mt7921_soc_dev *soc)
{
	if (!soc->awake)
		return;
	soc->awake = false;
	mt7921_soc_ci_put(soc);
}

/*
 * asicConnac2xLowPowerOwnClear() with CFG_CHECK_DRVOWN_EINT: clearing the
 * fw own raises an EINT that wakes the WM MCU. If the MCU was asleep and
 * the always-on debug flags say the EINT was not taken, clear it again.
 */
static void mt7921_soc_eint_wake(struct mt792x_dev *dev)
{
	struct mt7921_soc_dev *soc = to_soc(dev);
	int retry = 0;
	u32 mcu, flag;

	for (;;) {
		mt76_set(dev, MT7921_SOC_EINT_SEL, BIT(0));
		mt76_rmw(dev, MT7921_SOC_EINT_DBG_SEL, GENMASK(4, 0), 0x6);
		mt76_rmw(dev, MT7921_SOC_EINT_DBG_SEL2, GENMASK(2, 0), 0x1);
		mcu = mt76_rr(dev, MT7921_SOC_WF_MCU_STATE);
		mt76_wr(dev, MT_CONN_ON_LPCTL, PCIE_LPCR_HOST_CLR_OWN);
		if (mcu & BIT(4))
			return;

		udelay(MT7921_SOC_EINT_DELAY_US);
		flag = mt76_rr(dev, MT7921_SOC_WF_AON_DBG_FLAG);
		if (!(flag & BIT(0)) || !(flag & BIT(10)))
			return;

		soc->pm.eint_retry++;
		dev_warn_ratelimited(dev->mt76.dev,
				     "driver own: EINT not taken (0x%08x), try %d\n",
				     flag, retry + 1);
		if (retry++ == MT7921_SOC_EINT_RETRIES)
			return;
	}
}

/*
 * After a WFSYS sleep the hardware has restored the WFDMA from its backup,
 * and the bit the driver keeps in the dummy CR is gone. MediaTek's AXI
 * driver (CFG_MTK_WIFI_WFDMA_BK_RS) finds the TX rings restarted at index
 * 0 and sets its own indices back; the RX rings keep their state. Here:
 * every ring whose CPU index is not where mt76 left it is set back (TX)
 * or makes the whole WFDMA reset (RX, or wfdma_reinit=1).
 */
static void mt7921_soc_wfdma_reinit_cond(struct mt7921_soc_dev *soc)
{
	struct mt792x_dev *dev = &soc->dev;
	struct mt76_queue *tx[__MT_TXQ_MAX + __MT_MCUQ_MAX];
	unsigned long lost = 0;
	int i, n = 0;

	if (!mt792x_dma_need_reinit(dev))
		return;

	if (wfdma_reinit)
		goto full;

	mt76_for_each_q_rx(&dev->mt76, i) {
		struct mt76_queue *q = &dev->mt76.q_rx[i];

		if (Q_READ(q, cpu_idx) != q->head) {
			dev_info(dev->mt76.dev,
				 "WFDMA lost rx ring %d (cpu index %u, expected %u)\n",
				 i, Q_READ(q, cpu_idx), q->head);
			goto full;
		}
	}

	/* the data queues of all ACs are one ring */
	for (i = 0; i < __MT_TXQ_MAX + __MT_MCUQ_MAX; i++) {
		struct mt76_queue *q = i < __MT_TXQ_MAX ? dev->mphy.q_tx[i] :
				       dev->mt76.q_mcu[i - __MT_TXQ_MAX];
		int j;

		if (!q || !q->ndesc)
			continue;
		for (j = 0; j < n && tx[j] != q; j++)
			;
		if (j < n)
			continue;
		if (Q_READ(q, cpu_idx) != q->head)
			__set_bit(n, &lost);
		tx[n++] = q;
	}

	if (!lost) {
		soc->pm.wfdma_kept++;
	} else {
		/* nothing is in flight: fw own waits for the tx tokens */
		for (i = 0; i < n; i++) {
			if (!test_bit(i, &lost))
				continue;
			mt76_queue_tx_cleanup(dev, tx[i], true);
			mt76_queue_reset(dev, tx[i], true);
		}
		soc->pm.tx_reinit++;
		dev_dbg(dev->mt76.dev, "WFDMA tx rings set back (0x%lx)\n", lost);
	}
	mt76_set(dev, MT_WFDMA_DUMMY_CR, MT_WFDMA_NEED_REINIT);
	return;

full:
	soc->pm.full_reinit++;
	mt792x_wpdma_reinit_cond(dev);
}

/*
 * hif drv_own: take CONN_INFRA, force the host CSR interrupt enables as
 * MediaTek's driver does before every driver own, clear the fw own (EINT
 * check), set the WFDMA back if the WFSYS slept, then serve interrupts.
 */
static int mt7921_soc_drv_own(struct mt792x_dev *dev)
{
	struct mt7921_soc_dev *soc = to_soc(dev);
	struct mt76_phy *mphy = &dev->mt76.phy;
	struct mt76_connac_pm *pm = &dev->pm;
	bool was_fw = READ_ONCE(soc->fw_owned);
	ktime_t start = ktime_get();
	unsigned long flags;
	u32 us;
	int err;

	err = mt7921_soc_hold_conninfra(soc);
	if (err) {
		dev_err(dev->mt76.dev, "driver own: CONN_INFRA wakeup failed: %d\n",
			err);
		soc->pm.drv_own_fail++;
		return err;
	}

	mt7921_soc_force_host_csr_irq(dev);
	if (drv_own_eint)
		mt7921_soc_eint_wake(dev);
	err = __mt792xe_mcu_drv_pmctrl(dev);
	if (err) {
		soc->pm.drv_own_fail++;
		return err;
	}

	if (was_fw)
		soc->pm.fw_own_ns += ktime_to_ns(ktime_sub(start,
							   soc->pm.fw_own_since));
	WRITE_ONCE(soc->fw_owned, false);

	/* what the firmware raised in fw own (the tasklet serves it later) */
	if (was_fw) {
		soc->pm.wake_int_sta = mt76_rr(dev, MT_WFDMA0_HOST_INT_STA);
		soc->pm.wake_mcu_cmd = soc->pm.wake_int_sta & MT_INT_MCU_CMD ?
				       mt76_rr(dev, MT_MCU_CMD) : 0;
	}

	mt7921_soc_wfdma_reinit_cond(soc);

	/* the interrupt is served again before mt76 may touch the core */
	spin_lock_irqsave(&soc->own_lock, flags);
	soc->irq_defer = false;
	if (soc->irq_masked) {
		soc->irq_masked = false;
		enable_irq(soc->irq);
	}
	spin_unlock_irqrestore(&soc->own_lock, flags);

	WRITE_ONCE(dev->fw_sleepy, false);
	if (test_and_clear_bit(MT76_STATE_PM, &mphy->state)) {
		pm->stats.last_wake_event = jiffies;
		pm->stats.doze_time += pm->stats.last_wake_event -
				       pm->stats.last_doze_event;
	}
	/* what came in meanwhile, and the interrupt mask mt76 keeps */
	tasklet_schedule(&dev->mt76.irq_tasklet);

	/* something woke the core while the system goes to sleep */
	if (READ_ONCE(soc->sleeping))
		pm_wakeup_event(dev->mt76.dev, 0);

	us = ktime_us_delta(ktime_get(), start);
	if (was_fw) {
		soc->pm.drv_own++;
		soc->pm.last_drv_own_us = us;
		soc->pm.max_drv_own_us = max(soc->pm.max_drv_own_us, us);
		dev_dbg(dev->mt76.dev, "driver own in %u us\n", us);
	}

	return 0;
}

/*
 * fw own: from here on the host touches the always-on CSRs only. The
 * WFDMA interrupt stays enabled for everything the firmware passes up; it
 * disables its line and asks for driver own (MediaTek: the AXI interrupt
 * handler only disables the IRQ, the main thread owns the core first).
 */
static int __mt7921_soc_fw_own(struct mt7921_soc_dev *soc, bool idle)
{
	struct mt792x_dev *dev = &soc->dev;
	const struct mt792x_irq_map *map = dev->irq_map;
	u32 rx = map->rx.data_complete_mask | map->rx.wm_complete_mask |
		 map->rx.wm2_complete_mask;
	ktime_t start = ktime_get();
	unsigned long flags;
	int err;

	if (idle) {
		/* MediaTek: fw own only in the firmware's sleepy state */
		if (fw_own_sleepy && !READ_ONCE(dev->fw_sleepy)) {
			soc->pm.idle_not_sleepy++;
			goto stay;
		}
		/* MediaTek: not with a frame waiting in a ring */
		if (mt76_rr(dev, MT_WFDMA0_HOST_INT_STA) & rx) {
			soc->pm.idle_rx_pending++;
			goto stay;
		}
	}

	/*
	 * Only what the firmware passes up wakes the host in fw own: the TX
	 * done bits (the MCU command ring right after the last command, the
	 * unused ring 17 bit that stays set from boot) are acknowledged and
	 * left disabled, or the line rises at once. The tasklet enables
	 * them again after driver own.
	 */
	spin_lock_irqsave(&soc->own_lock, flags);
	soc->pm.fw_own_int_sta = mt76_rr(dev, MT_WFDMA0_HOST_INT_STA);
	if (soc->pm.fw_own_int_sta & ~(rx | MT_INT_MCU_CMD))
		mt76_wr(dev, MT_WFDMA0_HOST_INT_STA,
			soc->pm.fw_own_int_sta & ~(rx | MT_INT_MCU_CMD));
	mt76_wr(dev, map->host_irq_enable, rx | MT_INT_MCU_CMD);
	soc->irq_defer = true;
	spin_unlock_irqrestore(&soc->own_lock, flags);

	soc->pm.fw_own_since = ktime_get();
	WRITE_ONCE(soc->fw_owned, true);
	err = mt792xe_mcu_fw_pmctrl(dev);
	if (err) {
		WRITE_ONCE(soc->fw_owned, false);
		spin_lock_irqsave(&soc->own_lock, flags);
		soc->irq_defer = false;
		if (soc->irq_masked) {
			soc->irq_masked = false;
			enable_irq(soc->irq);
		}
		spin_unlock_irqrestore(&soc->own_lock, flags);
		tasklet_schedule(&dev->mt76.irq_tasklet);
		soc->pm.fw_own_fail++;
		return err;
	}

	if (!idle || conninfra_sleep)
		mt7921_soc_release_conninfra(soc);

	soc->pm.last_fw_own_us = ktime_us_delta(ktime_get(), start);
	if (idle) {
		soc->pm.idle_fw_own++;
		dev_dbg(dev->mt76.dev, "fw own in %u us\n",
			soc->pm.last_fw_own_us);
	}

	return 0;

stay:
	/* still driver-owned: the next activity schedules the power save again */
	clear_bit(MT76_STATE_PM, &dev->mphy.state);
	return 0;
}

/* hif fw_own: the runtime power save work (mt792x_mcu_fw_pmctrl) */
static int mt7921_soc_fw_own(struct mt792x_dev *dev)
{
	return __mt7921_soc_fw_own(to_soc(dev), true);
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
	struct mt7921_soc_phy_rsp_hdr *rhdr;
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

	if (skb->len < sizeof(*rhdr) + sizeof(*tlv) + sizeof(*rsp)) {
		dev_err(dev->mt76.dev, "short calibration response (%u bytes)\n",
			skb->len);
		print_hex_dump(KERN_ERR, "cal: ", DUMP_PREFIX_OFFSET, 16, 1,
			       skb->data, min_t(u32, skb->len, 64), false);
		ret = -EINVAL;
		goto out;
	}
	rhdr = (struct mt7921_soc_phy_rsp_hdr *)skb->data;
	tlv = (struct mt7921_soc_phy_tlv *)(rhdr + 1);
	rsp = (struct mt7921_soc_phy_rsp *)(tlv + 1);
	if (!rhdr->tag_num ||
	    le16_to_cpu(tlv->tag) != MT7921_SOC_PHY_TAG_CAL ||
	    rsp->event != MT7921_SOC_PHY_FORCE_CAL_RSP || rsp->status) {
		dev_err(dev->mt76.dev,
			"calibration failed: tags %u tag %u event 0x%02x status %u\n",
			rhdr->tag_num, le16_to_cpu(tlv->tag),
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
	/* the RF is calibrated (or will not be): BT may switch again */
	mtk_conninfra_wifi_starting(soc->ci, false);
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

static int mt7921_soc_wfsys_cycle(struct mt7921_soc_dev *soc);

/*
 * Restart the WFSYS from the ROM: power cycle through CONNINFRA, own the
 * core again and rebuild the rings, which the power cycle has cleared.
 */
static int mt7921_soc_restart(struct mt7921_soc_dev *soc)
{
	struct mt792x_dev *dev = &soc->dev;
	int err;

	err = mt7921_soc_wfsys_cycle(soc);
	if (err) {
		dev_err(dev->mt76.dev, "WFSYS power cycle failed: %d\n", err);
		return err;
	}

	mt7921_soc_force_host_csr_irq(dev);
	mt792xe_mcu_fw_pmctrl(dev);
	err = __mt792xe_mcu_drv_pmctrl(dev);
	if (!err)
		err = mt792x_wpdma_reset(dev, true);
	if (err)
		dev_err(dev->mt76.dev, "WFDMA restart failed: %d\n", err);

	return err;
}

/* after a failed start: from the ROM again, as the MediaTek driver does */
static int mt7921_soc_init_reset(struct mt792x_dev *dev)
{
	return mt7921_soc_restart(to_soc(dev));
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
	/* the firmware start after this calibrates again */
	mtk_conninfra_wifi_starting(soc->ci, true);
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

	mt7921_soc_drv_own(dev);

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

	err = mt7921_soc_restart(soc);

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
 * mt76_set_irq_mask() writes the interrupt enable mask through the mapping
 * directly, at the PCIe BAR offset (0x180d_4204 here, some CONN_INFRA
 * register). Every change of the mask (mt76_connac_irq_enable(), the NAPI
 * and tx done paths) ends up in this tasklet, which keeps the mask only in
 * memory and writes it through the bus ops. Otherwise as
 * mt792x_irq_tasklet().
 */
static void mt7921_soc_irq_serve(struct mt792x_dev *dev)
{
	const struct mt792x_irq_map *irq_map = dev->irq_map;
	u32 intr, mask = 0;

	mt76_wr(dev, irq_map->host_irq_enable, 0);

	intr = mt76_rr(dev, MT_WFDMA0_HOST_INT_STA);
	intr &= dev->mt76.mmio.irqmask;
	mt76_wr(dev, MT_WFDMA0_HOST_INT_STA, intr);

	mask |= intr & (irq_map->rx.data_complete_mask |
			irq_map->rx.wm_complete_mask |
			irq_map->rx.wm2_complete_mask);
	if (intr & irq_map->tx.mcu_complete_mask)
		mask |= irq_map->tx.mcu_complete_mask;

	if (intr & MT_INT_MCU_CMD) {
		u32 intr_sw = mt76_rr(dev, MT_MCU_CMD);

		/* ack MCU2HOST_SW_INT_STA */
		mt76_wr(dev, MT_MCU_CMD, intr_sw);
		if (intr_sw & MT_MCU_CMD_WAKE_RX_PCIE) {
			mask |= irq_map->rx.data_complete_mask;
			intr |= irq_map->rx.data_complete_mask;
		}
	}

	mt76_set_irq_mask(&dev->mt76, 0, mask, 0);

	if (intr & irq_map->tx.all_complete_mask)
		napi_schedule(&dev->mt76.tx_napi);
	if (intr & irq_map->rx.wm_complete_mask)
		napi_schedule(&dev->mt76.napi[MT_RXQ_MCU]);
	if (intr & irq_map->rx.wm2_complete_mask)
		napi_schedule(&dev->mt76.napi[MT_RXQ_MCU_WA]);
	if (intr & irq_map->rx.data_complete_mask)
		napi_schedule(&dev->mt76.napi[MT_RXQ_MAIN]);
}

static void mt7921_soc_irq_tasklet(unsigned long data)
{
	struct mt792x_dev *dev = (struct mt792x_dev *)data;
	struct mt7921_soc_dev *soc = to_soc(dev);
	unsigned long flags;

	/* in fw own nothing is read: driver own schedules this again */
	spin_lock_irqsave(&soc->own_lock, flags);
	if (soc->irq_defer)
		goto out;

	/*
	 * In system sleep with the core driver-owned: the interrupt has woken
	 * the system (the hard handler masked it), the rings are served after
	 * resume.
	 */
	if (READ_ONCE(soc->sleeping)) {
		pm_wakeup_event(dev->mt76.dev, 0);
		goto out;
	}

	mt7921_soc_irq_serve(dev);

	spin_lock(&dev->mt76.mmio.irq_lock);
	if (READ_ONCE(soc->wfsys_on))
		mt76_wr(dev, dev->irq_map->host_irq_enable,
			dev->mt76.mmio.irqmask);
	spin_unlock(&dev->mt76.mmio.irq_lock);
out:
	spin_unlock_irqrestore(&soc->own_lock, flags);
}

/*
 * The WFDMA interrupt. Driver own: as mt792x_irq_handler(). Fw own: the
 * registers are out of reach, disable the line and ask for driver own,
 * which serves the rings and enables the line again.
 */
static irqreturn_t mt7921_soc_irq_handler(int irq, void *data)
{
	struct mt792x_dev *dev = data;
	struct mt7921_soc_dev *soc = to_soc(dev);
	bool sleeping;

	if (test_bit(MT76_REMOVED, &dev->mt76.phy.state))
		return IRQ_NONE;

	spin_lock(&soc->own_lock);
	if (soc->irq_defer) {
		if (!soc->irq_masked) {
			disable_irq_nosync(irq);
			soc->irq_masked = true;
		}
		sleeping = READ_ONCE(soc->sleeping);
		soc->pm.irq_fw_own++;
		if (sleeping)
			soc->pm.irq_asleep++;
		spin_unlock(&soc->own_lock);

		if (sleeping)
			pm_wakeup_event(dev->mt76.dev, 0);
		else
			queue_work(dev->mt76.wq, &dev->pm.wake_work);
		return IRQ_HANDLED;
	}
	if (READ_ONCE(soc->sleeping))
		soc->pm.irq_asleep++;
	mt76_wr(dev, dev->irq_map->host_irq_enable, 0);
	spin_unlock(&soc->own_lock);

	if (!test_bit(MT76_STATE_INITIALIZED, &dev->mphy.state))
		return IRQ_NONE;

	tasklet_schedule(&dev->mt76.irq_tasklet);

	return IRQ_HANDLED;
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

	/* the core may be in fw own with CONN_INFRA asleep */
	if (mt7921_soc_ci_get(soc))
		return IRQ_NONE;
	status = mt76_rr(dev, MT7921_SOC_CCIF_RCHNUM);
	mt76_wr(dev, MT7921_SOC_CCIF_ACK, status);
	mt7921_soc_ci_put(soc);
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
	/*
	 * mt76 accesses the ring registers through the mapping directly, not
	 * through the bus ops: give it their offsets in the AXI window (the
	 * WFDMA is at +0x24000 here, at +0xd4000 in the PCIe BAR).
	 */
	u32 tx_base = mt7921_soc_reg_addr(dev, MT_TX_RING_BASE);
	u32 rx_base = mt7921_soc_reg_addr(dev, MT_RX_EVENT_RING_BASE);
	int ret;

	mt76_dma_attach(&dev->mt76);

	ret = mt792x_dma_disable(dev, true);
	if (ret)
		return ret;

	/* tx data, band 0 */
	ret = mt76_connac_init_tx_queues(dev->phy.mt76, MT7921_TXQ_BAND0,
					 MT7921_TX_RING_SIZE,
					 tx_base, NULL, 0);
	if (ret)
		return ret;

	mt76_wr(dev, MT_WFDMA0_TX_RING0_EXT_CTRL, 0x4);

	/* command to WM */
	ret = mt76_init_mcu_queue(&dev->mt76, MT_MCUQ_WM, MT7921_SOC_TXQ_MCU_WM,
				  MT7921_TX_MCU_RING_SIZE, tx_base);
	if (ret)
		return ret;

	/* firmware download ring: unused, the boot loader loaded the firmware */
	ret = mt76_init_mcu_queue(&dev->mt76, MT_MCUQ_FWDL, MT7921_SOC_TXQ_FWDL,
				  MT7921_TX_FWDL_RING_SIZE, tx_base);
	if (ret)
		return ret;

	/* rx event and band 0 tx free done */
	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MCU],
			       MT7921_SOC_RXQ_EVENT,
			       MT7921_SOC_RX_EVENT_RING_SIZE,
			       MT_RX_BUF_SIZE, rx_base);
	if (ret)
		return ret;

	/* band 1 tx free done */
	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MCU_WA],
			       MT7921_SOC_RXQ_TXDONE1,
			       MT7921_RX_MCU_WA_RING_SIZE,
			       MT_RX_BUF_SIZE, rx_base);
	if (ret)
		return ret;

	/* rx data, band 0 */
	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MAIN],
			       MT7921_SOC_RXQ_DATA, MT7921_RX_RING_SIZE,
			       MT_RX_BUF_SIZE, rx_base);
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
	mtk_conninfra_wifi_starting(soc->ci, false);
	if (soc->wfsys_on) {
		soc->wfsys_on = false;
		mtk_conninfra_wfsys_off(soc->ci);
	}
	mt7921_soc_release_conninfra(soc);
	if (soc->radio_on) {
		soc->radio_on = false;
		mtk_conninfra_radio_off(soc->ci, MTK_CONNINFRA_WIFI);
	}
}

static int mt7921_soc_power_on(struct mt7921_soc_dev *soc)
{
	struct device *dev = soc->dev.mt76.dev;
	int ret;

	/* until the firmware start: keep BT from switching the shared A-die */
	mtk_conninfra_wifi_starting(soc->ci, true);
	ret = mtk_conninfra_radio_on(soc->ci, MTK_CONNINFRA_WIFI);
	if (ret) {
		dev_err(dev, "CONNINFRA power on failed: %d\n", ret);
		mtk_conninfra_wifi_starting(soc->ci, false);
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
	 * Keep the CONN_INFRA off domain awake while the driver owns the
	 * core: the WFDMA host registers live there. Fw own lets it go.
	 */
	ret = mt7921_soc_hold_conninfra(soc);
	if (ret) {
		dev_err(dev, "CONN_INFRA wakeup failed: %d\n", ret);
		goto err;
	}

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
	mt7921_soc_drv_own(dev);
	mt792x_dma_cleanup(dev);
	skb_queue_purge(&dev->mt76.mcu.res_q);

	tasklet_disable(&dev->mt76.irq_tasklet);
}

/* debugfs <dev>/pm: own state and counters of the power management */
static int mt7921_soc_pm_show(struct seq_file *s, void *data)
{
	struct mt7921_soc_dev *soc = dev_get_drvdata(s->private);
	struct mt792x_dev *dev = &soc->dev;
	struct mt76_connac_pm *pm = &dev->pm;
	struct mt7921_soc_pm_stats *st = &soc->pm;
	bool fw = READ_ONCE(soc->fw_owned);
	u64 fw_ns = st->fw_own_ns;

	if (fw)
		fw_ns += ktime_to_ns(ktime_sub(ktime_get(), st->fw_own_since));

	seq_printf(s, "core: %s own, CONN_INFRA wake-ups held %d (driver %s), interrupt %s\n",
		   fw ? "firmware" : "driver", atomic_read(&soc->ci_refs),
		   soc->awake ? "yes" : "no",
		   soc->irq_masked ? "masked until driver own" :
		   soc->irq_defer ? "defers to driver own" : "served");
	seq_printf(s, "runtime pm: %s, idle timeout %u ms, fw_own_sleepy %d, conninfra_sleep %d, firmware sleepy %d (%u events)\n",
		   pm->enable ? "on" : "off", jiffies_to_msecs(pm->idle_timeout),
		   fw_own_sleepy, conninfra_sleep, READ_ONCE(dev->fw_sleepy),
		   READ_ONCE(dev->fw_sleepy_events));
	seq_printf(s, "deep sleep: %s; suspend_fw_own %d, suspend_deep_sleep %d, wfdma_reinit %d, drv_own_eint %d\n",
		   dev->ds_fw_default ? "firmware default" :
		   pm->ds_enable ? "on (KeepFullPwr 0)" : "off (KeepFullPwr 1)",
		   suspend_fw_own, suspend_deep_sleep, wfdma_reinit, drv_own_eint);
	seq_printf(s, "idle fw own: %u (stayed: not sleepy %u, rx pending %u; failed %u), last %u us\n",
		   st->idle_fw_own, st->idle_not_sleepy, st->idle_rx_pending,
		   st->fw_own_fail, st->last_fw_own_us);
	seq_printf(s, "driver own from fw own: %u (failed %u, EINT retries %u), last %u us, max %u us\n",
		   st->drv_own, st->drv_own_fail, st->eint_retry,
		   st->last_drv_own_us, st->max_drv_own_us);
	seq_printf(s, "interrupts in fw own: %u; in the last system sleep: %u\n",
		   st->irq_fw_own, st->irq_asleep);
	seq_printf(s, "INT_STA at the last fw own 0x%08x, at the last driver own 0x%08x (MCU2HOST 0x%x); suspend settle timeouts %u, WAKEPU_WF left set %u\n",
		   st->fw_own_int_sta, st->wake_int_sta, st->wake_mcu_cmd,
		   st->settle_timeout, st->wakepu_left);
	seq_printf(s, "WFDMA after a WFSYS sleep: kept %u, tx rings set back %u, full reset %u\n",
		   st->wfdma_kept, st->tx_reinit, st->full_reinit);
	seq_printf(s, "time in fw own: %llu ms (mt76: awake %u ms, doze %u ms)\n",
		   div_u64(fw_ns, NSEC_PER_MSEC),
		   jiffies_to_msecs(pm->stats.awake_time),
		   jiffies_to_msecs(pm->stats.doze_time));
	seq_printf(s, "system sleeps in fw own: %u; register accesses refused in fw own: %u\n",
		   st->suspend_fw_own, st->off_limits);

	return 0;
}

static void mt7921_soc_debugfs_init(struct mt7921_soc_dev *soc)
{
	struct device *dev = soc->dev.mt76.dev;

	soc->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_devm_seqfile(dev, "pm", soc->debugfs, mt7921_soc_pm_show);
}

static void mt7921_soc_release_rmem(void *data)
{
	of_reserved_mem_device_release(data);
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
		.fw_own = mt7921_soc_fw_own,
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
	 * All DMA of the device stays in the pool LK carved out (memory-region,
	 * written by LK into this node): the EMI MPU setup of LK lets CONNSYS
	 * reach that pool only, and MediaTek's driver copies every command,
	 * event and frame through it. The rings are allocated from it, and as
	 * a restricted DMA pool it bounces the commands and frames mt76 maps
	 * from anywhere else, which the WFDMA could not fetch or fill.
	 */
	ret = of_reserved_mem_device_init(pdev_dev);
	if (ret)
		return dev_err_probe(pdev_dev, ret,
				     "no WFDMA pool (memory-region from the boot loader)\n");
	if (!is_swiotlb_force_bounce(pdev_dev)) {
		of_reserved_mem_device_release(pdev_dev);
		return dev_err_probe(pdev_dev, -EINVAL,
				     "WFDMA pool is not a restricted-dma-pool\n");
	}

	/*
	 * The rings are device managed (dmam_alloc_coherent()) and are freed
	 * after remove(): the pool must stay assigned until then, or they
	 * would be freed as if they came from the page allocator.
	 */
	ret = devm_add_action_or_reset(pdev_dev, mt7921_soc_release_rmem,
				       pdev_dev);
	if (ret)
		return ret;

	ret = dma_set_mask(pdev_dev, DMA_BIT_MASK(36));
	if (!ret)
		ret = dma_set_coherent_mask(pdev_dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/*
	 * mt792x_get_mac80211_ops() reads the offload capabilities from the
	 * firmware file, which does not exist here: the image in EMI is a
	 * raw memory image without the feature tag. Default to CNM, which
	 * the legacy channel request needs; fw_features overrides it.
	 */
	ops = devm_kmemdup(pdev_dev, &mt7921_ops, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return -ENOMEM;
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
	if (!mdev)
		return -ENOMEM;

	dev = container_of(mdev, struct mt792x_dev, mt76);
	soc = to_soc(dev);
	soc->ci = ci;
	soc->irq = irq;
	atomic_set(&soc->ci_refs, 0);
	spin_lock_init(&soc->own_lock);
	soc->irq_sw = platform_get_irq_byname_optional(pdev, "sw");
	platform_set_drvdata(pdev, soc);
	/* a frame for the host wakes the system (downstream: enable_irq_wake) */
	device_init_wakeup(pdev_dev, true);

	mt7921_soc_load_nvram(soc);

	dev->fw_features = features;
	dev->hif_ops = &mt7921_soc_ops;
	dev->irq_map = &irq_map;
	dev->soc_axi = true;
	mt76_mmio_init(&dev->mt76, regs);
	tasklet_init(&mdev->irq_tasklet, mt7921_soc_irq_tasklet, (unsigned long)dev);

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

	ret = devm_request_irq(pdev_dev, irq, mt7921_soc_irq_handler, 0,
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
	 * Runtime PM: fw own when idle, CONN_INFRA let go. Deep sleep: the
	 * firmware default unless asked for (MediaTek's AXI driver never
	 * sends KeepFullPwr).
	 */
	dev->pm.enable_user = runtime_pm;
	dev->pm.enable = runtime_pm;
	dev->ds_fw_default = deep_sleep < 0;
	dev->pm.ds_enable_user = deep_sleep > 0;
	dev->pm.ds_enable = deep_sleep > 0;

	ret = mt7921_register_device(dev);
	if (ret)
		goto err_free_irq;

	mt7921_soc_debugfs_init(soc);

	return 0;

err_free_irq:
	if (soc->irq_sw > 0)
		devm_free_irq(pdev_dev, soc->irq_sw, dev);
	devm_free_irq(pdev_dev, irq, dev);
err_power_off:
	mt7921_soc_power_off(soc);
err_free_dev:
	mt76_free_device(&dev->mt76);

	return ret;
}

static void mt7921_soc_remove(struct platform_device *pdev)
{
	struct mt7921_soc_dev *soc = platform_get_drvdata(pdev);
	struct mt792x_dev *dev = &soc->dev;

	debugfs_remove_recursive(soc->debugfs);
	mt7921_soc_unregister_device(dev);
	set_bit(MT76_REMOVED, &dev->mt76.phy.state);
	if (soc->irq_sw > 0)
		devm_free_irq(&pdev->dev, soc->irq_sw, dev);
	devm_free_irq(&pdev->dev, soc->irq, dev);
	mt7921_soc_power_off(soc);
	mt76_free_device(&dev->mt76);
}

/* ---- system sleep ------------------------------------------------------ */

/*
 * The interrupt enable mask goes through the bus ops (see the tasklet):
 * write what mt76 keeps in mmio.irqmask.
 */
static void mt7921_soc_irq_apply(struct mt792x_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->mt76.mmio.irq_lock, flags);
	mt76_wr(dev, dev->irq_map->host_irq_enable, dev->mt76.mmio.irqmask);
	spin_unlock_irqrestore(&dev->mt76.mmio.irq_lock, flags);
}

/*
 * halSetSuspendFlagToFw(): the firmware names in its capabilities where
 * the host reports that it sleeps: a word in the CONNSYS EMI (0x11111111
 * asleep, 0x22222222 awake), a register bit, or a set/clear register pair.
 */
static void mt7921_soc_host_suspend_flag(struct mt7921_soc_dev *soc, bool suspend)
{
	struct mt792x_dev *dev = &soc->dev;
	u32 val;

	switch (dev->host_suspend.type) {
	case 1: {
		phys_addr_t base;
		u32 size;
		__le32 *word;

		if (mtk_conninfra_get_emi(soc->ci, &base, &size) ||
		    dev->host_suspend.set_addr + 4 > size)
			return;
		word = memremap(base + dev->host_suspend.set_addr, 4, MEMREMAP_WB);
		if (!word)
			return;
		*word = cpu_to_le32(suspend ? 0x11111111 : 0x22222222);
		memunmap(word);
		return;
	}
	case 2:
		val = mt76_rr(dev, dev->host_suspend.set_addr);
		if (suspend)
			val |= dev->host_suspend.mask;
		else
			val &= ~dev->host_suspend.mask;
		mt76_wr(dev, dev->host_suspend.set_addr, val);
		return;
	case 3:
		mt76_wr(dev, suspend ? dev->host_suspend.set_addr :
			dev->host_suspend.clr_addr, dev->host_suspend.mask);
		return;
	default:
		return;
	}
}

/*
 * Before the sleep hand-over: let the firmware fetch the commands just
 * sent (SUSPEND_MODE), answer them and report sleepy, and let the tasklet
 * and NAPI take what it passed up. Whatever is still pending when the
 * core goes to fw own raises the interrupt at once and ends the sleep.
 */
static void mt7921_soc_suspend_settle(struct mt7921_soc_dev *soc)
{
	struct mt792x_dev *dev = &soc->dev;
	const struct mt792x_irq_map *map = dev->irq_map;
	u32 busy_mask = map->rx.data_complete_mask | map->rx.wm_complete_mask |
			map->rx.wm2_complete_mask | MT_INT_MCU_CMD |
			map->tx.mcu_complete_mask;
	unsigned long end = jiffies + msecs_to_jiffies(suspend_settle_ms);
	struct mt76_queue *q = dev->mt76.q_mcu[MT_MCUQ_WM];

	if (!suspend_settle_ms)
		return;

	for (;;) {
		bool busy = mt76_rr(dev, MT_WFDMA0_HOST_INT_STA) & busy_mask;

		if (q && Q_READ(q, dma_idx) != q->head)
			busy = true;
		if (fw_own_sleepy && !READ_ONCE(dev->fw_sleepy))
			busy = true;
		if (!busy)
			return;
		if (time_after(jiffies, end)) {
			soc->pm.settle_timeout++;
			return;
		}
		usleep_range(2000, 3000);
	}
}

/* system sleep: fw own whatever the power save state, mt76 told (MT76_STATE_PM) */
static int mt7921_soc_suspend_fw_own(struct mt7921_soc_dev *soc)
{
	struct mt792x_dev *dev = &soc->dev;
	int err;

	mutex_lock(&dev->pm.mutex);
	set_bit(MT76_STATE_PM, &dev->mphy.state);
	err = __mt7921_soc_fw_own(soc, false);
	if (err)
		/* resume takes the core back in any case */
		set_bit(MT76_STATE_PM, &dev->mphy.state);
	mutex_unlock(&dev->pm.mutex);

	return err;
}

static void mt7921_soc_ip_iter(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt7921_soc_ip *ip = priv;
	struct mt792x_vif *mvif = (struct mt792x_vif *)vif->drv_priv;
	struct wireless_dev *wdev = ieee80211_vif_to_wdev(vif);
	struct in_device *in_dev;
	int i, n;

	if (ip->bss_idx >= 0 || vif->type != NL80211_IFTYPE_STATION ||
	    !vif->cfg.assoc)
		return;

	ip->bss_idx = mvif->bss_conf.mt76.idx;
	n = min_t(int, vif->cfg.arp_addr_cnt, IEEE80211_BSS_ARP_ADDR_LIST_LEN);
	in_dev = wdev && wdev->netdev ? __in_dev_get_rcu(wdev->netdev) : NULL;
	for (i = 0; i < n; i++) {
		const struct in_ifaddr *ifa;

		ip->addr[i] = vif->cfg.arp_addr_list[i];
		ip->mask[i] = htonl(0xffffff00);
		if (!in_dev)
			continue;
		in_dev_for_each_ifa_rcu(ifa, in_dev) {
			if (ifa->ifa_local == ip->addr[i]) {
				ip->mask[i] = ifa->ifa_mask;
				break;
			}
		}
	}
	ip->count = n;
}

static void mt7921_soc_assoc_iter(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	bool *assoc = priv;

	if (vif->type == NL80211_IFTYPE_STATION && vif->cfg.assoc)
		*assoc = true;
}

/* the firmware wake filter and the RX filter for the sleep, or back */
static void mt7921_soc_wow(struct mt7921_soc_dev *soc, bool suspend)
{
	struct mt792x_dev *dev = &soc->dev;
	struct mt76_dev *mdev = &dev->mt76;
	bool assoc = false;
	int err;

	if (!suspend) {
		if (soc->addba_rejected) {
			err = mt76_connac_legacy_addba_reject(mdev, false);
			if (err)
				dev_warn(mdev->dev, "ADDBA accept: %d\n", err);
			soc->addba_rejected = false;
		}
		if (soc->wow_on) {
			err = mt76_connac_legacy_set_wowlan(mdev, false, 0, 0, 0,
							    wow_hif, wow_gpio,
							    wow_trigger);
			if (err)
				dev_warn(mdev->dev, "wake filter stop: %d\n", err);
			soc->wow_on = false;
		}
		if (soc->rx_filter_set) {
			/* what mt7921_configure_filter() keeps set */
			u32 all = MT76_CONNAC_LEGACY_RX_FILTER_DIRECTED |
				  MT76_CONNAC_LEGACY_RX_FILTER_MULTICAST |
				  MT76_CONNAC_LEGACY_RX_FILTER_ALL_MULTICAST |
				  MT76_CONNAC_LEGACY_RX_FILTER_BROADCAST;

			mt76_connac_legacy_set_rx_filter(mdev, all);
			soc->rx_filter_set = false;
		}
		return;
	}

	rcu_read_lock();
	ieee80211_iterate_active_interfaces_atomic(mt76_hw(dev),
						   IEEE80211_IFACE_ITER_NORMAL,
						   mt7921_soc_assoc_iter, &assoc);
	rcu_read_unlock();
	if (!assoc)
		return;

	if (suspend_rx_filter >= 0) {
		err = mt76_connac_legacy_set_rx_filter(mdev, suspend_rx_filter);
		soc->rx_filter_set = !err;
	}
	if (suspend_addba_reject) {
		err = mt76_connac_legacy_addba_reject(mdev, true);
		soc->addba_rejected = !err;
		if (err)
			dev_warn(mdev->dev, "ADDBA reject: %d\n", err);
		else
			dev_info(mdev->dev, "ADDBA refused for the sleep\n");
	}
	if (!wowlan)
		return;
	err = mt76_connac_legacy_set_wowlan(mdev, true, wow_detect,
					    wow_detect_ext, wow_filter,
					    wow_hif, wow_gpio, wow_trigger);
	if (err) {
		dev_warn(mdev->dev, "wake filter: %d\n", err);
		return;
	}
	soc->wow_on = true;
	dev_info(mdev->dev, "wake filter for the sleep: detect 0x%02x ext 0x%04x, pass 0x%04x, host signal %u pin 0x%02x%s\n",
		 wow_detect, wow_detect_ext, wow_filter, wow_hif, wow_gpio,
		 soc->rx_filter_set ? ", RX filter narrowed" : "");
}

/* the station's IPv4 addresses to the firmware for the sleep, or cleared */
static void mt7921_soc_ip_offload(struct mt7921_soc_dev *soc, bool suspend)
{
	struct mt792x_dev *dev = &soc->dev;
	struct mt7921_soc_ip ip = { .bss_idx = -1 };
	int err;

	if (!suspend) {
		if (!soc->ip_offloaded)
			return;
		soc->ip_offloaded = false;
		err = mt76_connac_legacy_set_ip_addr(&dev->mt76, soc->ip_bss_idx,
						     NULL, NULL, 0);
		if (err)
			dev_warn(dev->mt76.dev, "ARP offload: clearing failed: %d\n",
				 err);
		return;
	}

	if (!arp_offload)
		return;

	rcu_read_lock();
	ieee80211_iterate_active_interfaces_atomic(mt76_hw(dev),
						   IEEE80211_IFACE_ITER_NORMAL,
						   mt7921_soc_ip_iter, &ip);
	rcu_read_unlock();
	if (ip.bss_idx < 0 || !ip.count)
		return;

	err = mt76_connac_legacy_set_ip_addr(&dev->mt76, ip.bss_idx, ip.addr,
					     ip.mask, ip.count);
	if (err) {
		dev_warn(dev->mt76.dev, "ARP offload: %d\n", err);
		return;
	}
	soc->ip_offloaded = true;
	soc->ip_bss_idx = ip.bss_idx;
	dev_info(dev->mt76.dev, "ARP offload for the sleep: bss %d, %pI4/%pI4%s\n",
		 ip.bss_idx, &ip.addr[0], &ip.mask[0],
		 ip.count > 1 ? " and more" : "");
}

static const char *mt7921_soc_ds_name(struct mt792x_dev *dev)
{
	if (dev->ds_fw_default && !suspend_deep_sleep)
		return "firmware default";
	return dev->pm.ds_enable || suspend_deep_sleep ? "on" : "off";
}

static int mt7921_soc_suspend(struct device *device)
{
	struct mt7921_soc_dev *soc = dev_get_drvdata(device);
	struct mt792x_dev *dev = &soc->dev;
	struct mt76_dev *mdev = &dev->mt76;
	struct mt76_connac_pm *pm = &dev->pm;
	ktime_t start = ktime_get();
	bool fw_own = false;
	s64 own_us = 0;
	int err;

	/* the core is down (a reset is in progress): nothing to put to sleep */
	if (!READ_ONCE(soc->wfsys_on) || !soc->radio_on)
		return 0;

	pm->suspended = true;
	flush_work(&dev->reset_work);
	cancel_delayed_work_sync(&pm->ps_work);
	cancel_work_sync(&pm->wake_work);
	mt7921_roc_abort_sync(dev);

	/* runtime PM may have handed the core over already */
	err = mt792x_mcu_drv_pmctrl(dev);
	if (err < 0)
		goto restore;

	wait_event_timeout(dev->wait, !dev->regd_in_progress, 5 * HZ);

	/* the host sleeps: listen every DTIM, keep the links */
	WRITE_ONCE(dev->fw_sleepy, false);
	mt7921_soc_ip_offload(soc, true);
	mt76_connac_legacy_suspend_mode(mdev, 0, true, 1, wowlan);
	if (suspend_deep_sleep)
		mt76_connac_mcu_set_deep_sleep(mdev, true);
	/* as gen4m: the wake filter is the last command before the sleep */
	mt7921_soc_wow(soc, true);
	mt7921_soc_host_suspend_flag(soc, true);
	if (suspend_fw_own)
		mt7921_soc_suspend_settle(soc);
	if (suspend_ci_dump)
		mtk_conninfra_dump(soc->ci, "wifi suspend");

	soc->pm.irq_asleep = 0;
	WRITE_ONCE(soc->sleeping, true);
	if (device_may_wakeup(device)) {
		if (!enable_irq_wake(soc->irq))
			soc->irq_wake = true;
		if (soc->irq_sw > 0)
			enable_irq_wake(soc->irq_sw);
	}

	if (suspend_fw_own) {
		ktime_t t = ktime_get();

		err = mt7921_soc_suspend_fw_own(soc);
		own_us = ktime_us_delta(ktime_get(), t);
		if (!err) {
			fw_own = true;
			soc->pm.suspend_fw_own++;
		} else {
			dev_warn(device, "fw own for the sleep failed (%d), staying driver-owned\n",
				 err);
			err = 0;
		}
	}
	/* the firmware wakes CONN_INFRA for the interrupt */
	mt7921_soc_release_conninfra(soc);
	if (fw_own) {
		u32 wakepu = mt76_rr(dev, MT7921_SOC_WAKEPU_WF);

		if (wakepu & BIT(0)) {
			soc->pm.wakepu_left++;
			if (suspend_wakepu_clear)
				mt76_clear(dev, MT7921_SOC_WAKEPU_WF, BIT(0));
			dev_info(device, "suspend: CONN_INFRA Wi-Fi wake-up was left set (0x%08x)%s\n",
				 wakepu, suspend_wakepu_clear ? ", cleared" : "");
		}
	}

	dev_info(device, "asleep in %lld us: %s owns the core (fw own %lld us, INT_STA 0x%08x, firmware sleepy %d, %u sleepy events, settle timeouts %u), CONN_INFRA released (wake-ups held %d), deep sleep %s, wake IRQ %s\n",
		 ktime_us_delta(ktime_get(), start),
		 fw_own ? "firmware" : "driver", own_us,
		 fw_own ? soc->pm.fw_own_int_sta : 0,
		 READ_ONCE(dev->fw_sleepy), READ_ONCE(dev->fw_sleepy_events),
		 soc->pm.settle_timeout,
		 atomic_read(&soc->ci_refs), mt7921_soc_ds_name(dev),
		 soc->irq_wake ? "on" : "off");

	return 0;

restore:
	pm->suspended = false;
	mt792x_reset(mdev);
	return err;
}

static int mt7921_soc_resume(struct device *device)
{
	struct mt7921_soc_dev *soc = dev_get_drvdata(device);
	struct mt792x_dev *dev = &soc->dev;
	struct mt76_dev *mdev = &dev->mt76;
	struct mt76_connac_pm *pm = &dev->pm;
	u32 kept = soc->pm.wfdma_kept, tx = soc->pm.tx_reinit;
	u32 full = soc->pm.full_reinit;
	ktime_t start = ktime_get();
	const char *wfdma;
	bool was_fw;
	int i, err;

	if (!pm->suspended)
		return 0;

	err = mt7921_soc_hold_conninfra(soc);
	if (err) {
		dev_err(device, "CONN_INFRA wakeup after sleep failed: %d\n", err);
		pm->suspended = false;
		return err;
	}

	if (soc->irq_wake) {
		disable_irq_wake(soc->irq);
		soc->irq_wake = false;
		if (soc->irq_sw > 0)
			disable_irq_wake(soc->irq_sw);
	}

	/* what the firmware passes up first tells what woke the system */
	WRITE_ONCE(dev->wake_probe, min(wake_probe, 255U));

	/* take the core back if the firmware has it */
	was_fw = test_bit(MT76_STATE_PM, &dev->mphy.state);
	WRITE_ONCE(soc->sleeping, false);
	err = mt792x_mcu_drv_pmctrl(dev);
	if (err < 0)
		goto failed;

	mt7921_soc_host_suspend_flag(soc, false);
	if (suspend_deep_sleep && !pm->ds_enable && !dev->ds_fw_default)
		mt76_connac_mcu_set_deep_sleep(mdev, false);
	mt76_connac_legacy_suspend_mode(mdev, 0, false, 1, false);
	mt7921_soc_wow(soc, false);
	mt7921_soc_ip_offload(soc, false);

	/* what the firmware raised while asleep was only masked: serve it */
	mt76_connac_irq_enable(mdev, dev->irq_map->tx.all_complete_mask |
			       MT_INT_RX_DONE_ALL | MT_INT_MCU_CMD);
	mt7921_soc_irq_apply(dev);
	local_bh_disable();
	mt76_for_each_q_rx(mdev, i)
		napi_schedule(&mdev->napi[i]);
	napi_schedule(&mdev->tx_napi);
	local_bh_enable();

	mt7921_mcu_regd_update(dev, mdev->alpha2, dev->country_ie_env);

	if (soc->pm.full_reinit != full)
		wfdma = "reset";
	else if (soc->pm.tx_reinit != tx)
		wfdma = "tx rings set back";
	else if (soc->pm.wfdma_kept != kept)
		wfdma = "kept over a WFSYS sleep";
	else
		wfdma = "untouched";
	dev_info(device, "resumed in %lld us: driver owns the core (%s, driver own %u us), WFDMA %s, %u interrupts in the sleep, woken with INT_STA 0x%08x (MCU2HOST 0x%x), IRQ status 0x%08x\n",
		 ktime_us_delta(ktime_get(), start),
		 was_fw ? "was firmware-owned" : "stayed driver-owned",
		 was_fw ? soc->pm.last_drv_own_us : 0, wfdma,
		 soc->pm.irq_asleep, was_fw ? soc->pm.wake_int_sta : 0,
		 was_fw ? soc->pm.wake_mcu_cmd : 0,
		 mt76_rr(dev, MT_WFDMA0_HOST_INT_STA));
failed:
	if (err < 0)
		dev_err(device, "resume: driver own failed: %d\n", err);
	pm->suspended = false;
	if (err < 0)
		mt792x_reset(mdev);
	return err;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mt7921_soc_pm_ops, mt7921_soc_suspend,
				mt7921_soc_resume);

static const struct of_device_id mt7921_soc_of_match[] = {
	{ .compatible = "mediatek,mt6878-wifi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt7921_soc_of_match);

static struct platform_driver mt7921_soc_driver = {
	.driver = {
		.name = "mt7921-soc",
		.of_match_table = mt7921_soc_of_match,
		.pm = pm_sleep_ptr(&mt7921_soc_pm_ops),
	},
	.probe = mt7921_soc_probe,
	.remove = mt7921_soc_remove,
};
module_platform_driver(mt7921_soc_driver);

MODULE_DESCRIPTION("MediaTek MT7921 (MT6878 CONNSYS) AXI wireless driver");
MODULE_LICENSE("Dual BSD/GPL");
