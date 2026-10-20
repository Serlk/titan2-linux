/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek CCCI (Cross Core Communication Interface) for the modem
 * built into MediaTek smartphone SoCs.
 *
 * The data layouts handed over by the bootloader follow the MediaTek
 * ccci_util driver of its GPL downstream kernel; the CCIF mailbox, its SRAM and
 * the shared memory ring buffers follow MediaTek's GPL downstream eccci driver.
 *
 * Copyright (C) 2015-2016 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#ifndef __MTK_CCCI_H__
#define __MTK_CCCI_H__

#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/mutex.h>
#include <linux/types.h>

struct dentry;
struct device;
struct regmap;
struct regulator;
struct wwan_port;
struct sk_buff;

/*
 * "ccci,modem_info_v2" in the modem node, written by the bootloader (LK):
 * where LK left its table of named arguments ("tags"). Little endian, as
 * LK copies the structure into the property without byte swapping.
 */
struct mtk_ccci_lk_info {
	__le64 tags_base;	/* physical address of the tag area */
	__le32 tags_size;	/* bytes used in the tag area */
	__le32 err_no;		/* LK load error, 0 = none */
	__le32 version;		/* tag format; >= 2: 64-byte names */
	__le32 tag_num;
	__le32 ld_flag;		/* bit 0: modem image loaded */
	__le32 ld_md_errno;
};
static_assert(sizeof(struct mtk_ccci_lk_info) == 32);

#define MTK_CCCI_LK_LD_MD		BIT(0)
#define MTK_CCCI_LK_TAGS_MAX_SIZE	0x10000
#define MTK_CCCI_LK_TAG_NAME_LEN	64

/* one entry of the LK tag area (format version 2 and later) */
struct mtk_ccci_lk_tag {
	char name[MTK_CCCI_LK_TAG_NAME_LEN];
	__le32 data_offset;	/* from the start of the tag area */
	__le32 data_size;
	__le32 next_tag_offset;	/* from the start of the tag area */
};
static_assert(sizeof(struct mtk_ccci_lk_tag) == 76);

/* tag "md_mem_layout": modem memory blocks as LK set them up */
struct mtk_ccci_md_mem_blk {
	__le32 md_offset;	/* modem view, from the modem bank 0 base */
	__le32 size;
	__le32 inf_flag;
	__le32 attr_flag;
	__le64 ap_phys;
};
static_assert(sizeof(struct mtk_ccci_md_mem_blk) == 24);

/* tags "nc_smem_layout" and "c_smem_layout": AP/modem shared memory */
struct mtk_ccci_smem_region {
	__le64 ap_phys;
	__le64 ap_virt;		/* LK's own mapping, meaningless here */
	__le32 id;		/* SMEM user, see enum mtk_ccci_smem_id */
	__le32 ap_offset;	/* from the start of the shared memory block */
	__le32 size;
	__le32 align;
	__le32 flags;
	__le32 md_offset;	/* modem view of ap_offset */
};
static_assert(sizeof(struct mtk_ccci_smem_region) == 40);

#define MTK_CCCI_SMEM_F_PADDING		BIT(2)

/* the modem sees the shared memory at md_offset from this address */
#define MTK_CCCI_SMEM_MD_VIEW_BASE	0x40000000

/* SMEM users the driver cares about (vendor enum SMEM_USER_ID) */
enum mtk_ccci_smem_id {
	MTK_CCCI_SMEM_DBM = 0,
	MTK_CCCI_SMEM_CCB_DHL = 1,
	MTK_CCCI_SMEM_CCB_CTRL = 4,
	MTK_CCCI_SMEM_MDSS_DBG = 13,
	MTK_CCCI_SMEM_RUNTIME_DATA = 14,
	MTK_CCCI_SMEM_FORCE_ASSERT = 15,
	MTK_CCCI_SMEM_CCISM_SCP = 16,
	MTK_CCCI_SMEM_CCISM_MCU = 19,
	MTK_CCCI_SMEM_CCISM_MCU_EXP = 20,
	MTK_CCCI_SMEM_MD_CONSYS = 22,
	MTK_CCCI_SMEM_WIFI_PROXY = 31,
	MTK_CCCI_SMEM_NVRAM_CACHE = 32,
	MTK_CCCI_SMEM_LOW_POWER = 33,
	MTK_CCCI_SMEM_DRDI = 41,
};

/* reserved memory LK sets aside for the modem (nodes "mblock-N-<name>") */
enum mtk_ccci_resv {
	MTK_CCCI_RESV_TAGS,	/* "mediatek,ccci_tag_mem" */
	MTK_CCCI_RESV_NC_SMEM,	/* "mediatek,ap_md_nc_smem" */
	MTK_CCCI_RESV_C_SMEM,	/* "mediatek,ap_md_c_smem" */
	MTK_CCCI_RESV_NUM,
};

#define MTK_CCCI_MD_MEM_MAX	8	/* "mediatek,md_mem_usage" nodes */

struct mtk_ccci_lk {
	struct mtk_ccci_lk_info info;
	void *tags;		/* private copy of the tag area */
	u32 tags_size;
	u32 tag_num;		/* tags that passed the checks */

	const struct mtk_ccci_md_mem_blk *md_mem;
	unsigned int md_mem_num;
	const struct mtk_ccci_smem_region *nc_smem;
	unsigned int nc_smem_num;
	const struct mtk_ccci_smem_region *c_smem;
	unsigned int c_smem_num;
};

/*
 * CCCI packet header. The channel word packs the logical channel
 * (bits 15:0), a per-channel sequence number (bits 30:16) and the
 * "check the sequence" bit (31).
 */
struct mtk_ccci_header {
	__le32 data[2];
	__le32 status;
	__le32 reserved;
};
static_assert(sizeof(struct mtk_ccci_header) == 16);

#define MTK_CCCI_H_CHANNEL	GENMASK(15, 0)
#define MTK_CCCI_H_SEQ		GENMASK(30, 16)
#define MTK_CCCI_H_ASSERT	BIT(31)

/* logical channels (vendor enum CCCI_CH), only the ones used here */
enum mtk_ccci_channel {
	MTK_CCCI_CH_CONTROL_RX = 0,
	MTK_CCCI_CH_CONTROL_TX = 1,
	MTK_CCCI_CH_SYSTEM_RX = 2,
	MTK_CCCI_CH_SYSTEM_TX = 3,
	MTK_CCCI_CH_UART2_RX = 10,	/* AT, stock /dev/ttyC0 */
	MTK_CCCI_CH_UART2_TX = 12,
	MTK_CCCI_CH_FS_RX = 14,		/* NVRAM file server */
	MTK_CCCI_CH_FS_TX = 15,
	MTK_CCCI_CH_RPC_RX = 32,
	MTK_CCCI_CH_RPC_TX = 33,
	MTK_CCCI_CH_STATUS_RX = 67,
	MTK_CCCI_CH_STATUS_TX = 68,
	MTK_CCCI_CH_MIPC0_RX = 214,	/* MIPC0..9: 214 + 2 * n */
	MTK_CCCI_CH_MIPC0_TX = 215,
};

/* control channel messages (data[1] of a CONTROL_RX packet) */
#define MTK_CCCI_CTL_MD_INIT_START_BOOT	0x0	/* HS1, and HS2 */
#define MTK_CCCI_CTL_MD_EX		0x4
#define MTK_CCCI_CTL_DRV_VER_ERROR	0x5
#define MTK_CCCI_CTL_MD_EX_REC_OK	0x6
#define MTK_CCCI_CTL_MD_EX_PASS		0x8
#define MTK_CCCI_CTL_MD_INIT_CHK_ID	0x5555ffff	/* reserved of HS1 */

/*
 * CCIF0 mailbox: 24 channels each way, a 512-byte SRAM visible to both
 * sides. Channels 0..7 notify the shared memory ring queues (7 doubles
 * as the CCB wakeup), 15 the SRAM handshake messages, 16..21 exception
 * handling and peer wakeup.
 */
#define MTK_CCIF_CH_NUM			24
#define MTK_CCIF_CH_RINGQ(n)		(n)
#define MTK_CCIF_CH_CCB_WAKEUP		7
#define MTK_CCIF_CH_SRAM		15
#define MTK_CCIF_CH_EX_INIT		16	/* D2H: exception init */
#define MTK_CCIF_CH_EX_INIT_DONE	17
#define MTK_CCIF_CH_EX_CLEARQ_DONE	18
#define MTK_CCIF_CH_EX_ALLQ_RESET	19
#define MTK_CCIF_CH_PEER_WAKEUP		20
#define MTK_CCIF_CH_SEQ_ERROR		21

#define MTK_CCIF_SRAM_SIZE_DEFAULT	512

/* modem runtime feature query, carried with HS1 in the CCIF SRAM */
#define MTK_CCCI_FEATURE_COUNT		64
#define MTK_CCCI_MD_FEATURE_PATTERN	0x49434343
#define MTK_CCCI_AP_FEATURE_PATTERN	0x43434349

struct mtk_ccci_md_feature_query {
	__le32 head_pattern;
	u8 feature[MTK_CCCI_FEATURE_COUNT];	/* support mask 3:0, ver 7:4 */
	__le32 tail_pattern;
	u8 reserved[152];
};
static_assert(sizeof(struct mtk_ccci_md_feature_query) == 224);

/* the start of the CCIF SRAM during the boot handshake */
struct mtk_ccci_ccif_sram_hs {
	struct mtk_ccci_header dl_header;	/* modem -> AP */
	struct mtk_ccci_md_feature_query md_query;
	struct mtk_ccci_header ul_header;	/* AP -> modem */
	/* followed by the AP runtime data reply */
};

/* end of the CCIF SRAM: where the AP points the modem at its debug area */
#define MTK_CCIF_SRAM_DBG_MAGIC		0x7274626e	/* "nbtr" */

/* a read-only picture of CCIF0, see mtk_ccci_ccif_snapshot() */
struct mtk_ccci_ccif_regs {
	u32 con, busy, start, tchnum, rchnum;
};

struct mtk_ccci_ccif_snap {
	bool valid;
	struct mtk_ccci_ccif_regs ap;
	struct mtk_ccci_ccif_regs md;
	u32 sram[MTK_CCIF_SRAM_SIZE_DEFAULT / sizeof(u32)];
	/* outside CCIF, also only read */
	int md_pwr_con;		/* SPM MD PWR_CON, -errno if unknown */
	int md1_clk_mod;	/* topckgen reg 0, -errno if unknown */
};

/* the ring buffer queues in the CCISM shared memory */
#define MTK_CCCI_RINGQ_NUM		16

struct mtk_ccci_ring {
	void __iomem *ctl;	/* rx_control, tx_control */
	void __iomem *rx_buf;	/* modem -> AP */
	void __iomem *tx_buf;	/* AP -> modem */
	u32 rx_len;
	u32 tx_len;
};

enum mtk_ccci_fsm_state {
	MTK_CCCI_FSM_LK_OWNED,	/* as LK left it, never touched */
	MTK_CCCI_FSM_STARTING,
	MTK_CCCI_FSM_WAIT_HS1,
	MTK_CCCI_FSM_HS1,	/* HS1 seen; runtime data not implemented */
	MTK_CCCI_FSM_READY,
	MTK_CCCI_FSM_STOPPED,	/* powered off by the driver */
	MTK_CCCI_FSM_FAILED,
};

enum mtk_ccci_supply {
	MTK_CCCI_SUPPLY_VMODEM,
	MTK_CCCI_SUPPLY_VSRAM,
	MTK_CCCI_SUPPLY_VDIGRF,
	MTK_CCCI_SUPPLY_NUM,
};

struct mtk_ccci_port_conf;

struct mtk_ccci_port {
	struct mtk_ccci *ccci;
	const struct mtk_ccci_port_conf *conf;
	struct wwan_port *wwan;
	u16 tx_seq;
};

struct mtk_ccci {
	struct device *dev;

	struct resource resv[MTK_CCCI_RESV_NUM];
	struct resource md_mem[MTK_CCCI_MD_MEM_MAX];
	unsigned int md_mem_num;

	struct mtk_ccci_lk lk;

	/* CCIF0; nothing is written before an explicit start command */
	void __iomem *ccif_ap;
	void __iomem *ccif_md;
	void __iomem *infra;		/* CCIF reset, optional */
	u32 ccif_sram_size;
	u32 ccif_rst_bit;
	struct clk_bulk_data *clks;
	int num_clks;
	struct clk *ccif_ap_clk;	/* among clks, for the read checks */
	struct clk *ccif_md_clk;
	bool clks_on;

	/* read-only outside CCIF, optional */
	struct regmap *scpsys;
	struct regmap *topckgen;

	/* power control, all dormant unless explicitly enabled */
	struct regulator *supply[MTK_CCCI_SUPPLY_NUM];
	struct mutex fsm_lock;		/* FSM commands */
	enum mtk_ccci_fsm_state state;
	u32 boot_status[2];
	struct mtk_ccci_md_feature_query hs1_query;
	bool hs1_valid;

	void __iomem *ccism;		/* CCISM_MCU, while started */
	u32 ccism_size;
	struct mtk_ccci_ring ring[MTK_CCCI_RINGQ_NUM];

	struct mtk_ccci_port *ports;
	unsigned int num_ports;
	bool ports_registered;

	struct dentry *debugfs;
};

/* mtk_ccci_lk.c */
int mtk_ccci_lk_parse(struct mtk_ccci *ccci);
const void *mtk_ccci_lk_find(const struct mtk_ccci *ccci, const char *name,
			     u32 *size);
const struct mtk_ccci_smem_region *
mtk_ccci_lk_smem(const struct mtk_ccci *ccci, enum mtk_ccci_smem_id id);
void mtk_ccci_lk_debugfs_init(struct mtk_ccci *ccci);

/* mtk_ccci_ccif.c */
int mtk_ccci_ccif_init(struct mtk_ccci *ccci);
bool mtk_ccci_ccif_readable(struct mtk_ccci *ccci);
int mtk_ccci_ccif_snapshot(struct mtk_ccci *ccci,
			   struct mtk_ccci_ccif_snap *snap);
void mtk_ccci_ccif_log_state(struct mtk_ccci *ccci);
void mtk_ccci_ccif_debugfs_init(struct mtk_ccci *ccci);
int mtk_ccci_ccif_start(struct mtk_ccci *ccci);
void mtk_ccci_ccif_stop(struct mtk_ccci *ccci);
u32 mtk_ccci_ccif_pending(struct mtk_ccci *ccci);
void mtk_ccci_ccif_ack(struct mtk_ccci *ccci, u32 channels);
int mtk_ccci_ccif_notify(struct mtk_ccci *ccci, unsigned int channel);
void mtk_ccci_ccif_read_hs(struct mtk_ccci *ccci,
			   struct mtk_ccci_header *hdr,
			   struct mtk_ccci_md_feature_query *query);

/* mtk_ccci_ring.c */
int mtk_ccci_rings_init(struct mtk_ccci *ccci, void __iomem *base, u32 size);
int mtk_ccci_ring_readable(const struct mtk_ccci_ring *ring);
int mtk_ccci_ring_read(struct mtk_ccci_ring *ring, void *buf, u32 len);
int mtk_ccci_ring_write(struct mtk_ccci_ring *ring, const void *data,
			u32 len);

/* mtk_ccci_fsm.c */
int mtk_ccci_fsm_init(struct mtk_ccci *ccci);
void mtk_ccci_fsm_debugfs_init(struct mtk_ccci *ccci);
const char *mtk_ccci_fsm_state_name(enum mtk_ccci_fsm_state state);

/* mtk_ccci_port.c */
int mtk_ccci_port_init(struct mtk_ccci *ccci);
int mtk_ccci_port_register(struct mtk_ccci *ccci);
void mtk_ccci_port_unregister(struct mtk_ccci *ccci);
int mtk_ccci_port_recv(struct mtk_ccci *ccci, struct sk_buff *skb);
void mtk_ccci_port_debugfs_init(struct mtk_ccci *ccci);

#endif /* __MTK_CCCI_H__ */
