// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: the CCIF0 mailbox between the AP and the modem.
 *
 * Each side has a register window: writing a channel number to TCHNUM
 * raises that channel at the peer (BUSY marks it pending until the peer
 * acknowledges), RCHNUM shows the channels the peer raised, ACK clears
 * them. Both windows also show a 512-byte SRAM, used for the boot
 * handshake: the modem leaves HS1 and its feature query there and raises
 * channel 15; the AP answers with its runtime data the same way.
 *
 * Two levels of access:
 *  - passive (module parameter ccif_regs=1): registers and SRAM are only
 *    read, and only while the hardware says the CCIF clocks are on;
 *  - active (start/stop, called by the FSM on an explicit debugfs
 *    command only): reset, SRAM setup, ring buffers, notifications.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>

#include "mtk_ccci.h"

/* CCIF register window, the same layout on both sides */
#define CCIF_CON		0x00
#define CCIF_BUSY		0x04
#define CCIF_START		0x08
#define CCIF_TCHNUM		0x0c
#define CCIF_RCHNUM		0x10
#define CCIF_ACK		0x14
#define CCIF_CHDATA		0x100

/* CCIF reset in infracfg (vendor "ccif-hw-reset-ver" 1) */
#define INFRA_RST_SET		0xf50
#define INFRA_RST_CLR		0xf54

/* SPM: modem power control, PWR_ACK in bit 30 (see mtk-pm-domains) */
#define SPM_MD_PWR_CON		0xe00
#define SPM_PWR_ACK		BIT(30)

/* topckgen register 0: modem clock gates, set = gated */
#define TOPCKGEN_MD1_CLK_MOD	0x0
#define TOPCKGEN_MD1_CLK_MASK	(BIT(8) | BIT(9))

static bool ccif_regs;
module_param(ccif_regs, bool, 0444);
MODULE_PARM_DESC(ccif_regs,
		 "read CCIF0 registers and SRAM (never write) for debugfs; needs the CCIF clocks on");

static int mtk_ccci_read_regmap(struct regmap *map, unsigned int reg)
{
	unsigned int val;
	int ret;

	if (!map)
		return -ENODEV;
	ret = regmap_read(map, reg, &val);
	if (ret)
		return ret;
	/* the bits we look at are below bit 31 */
	return val & INT_MAX;
}

/**
 * mtk_ccci_ccif_readable() - may CCIF0 be read now?
 * @ccci: driver state
 *
 * Without its clocks, an access to CCIF stalls the bus. The clocks are
 * only on because LK started the modem and the kernel runs with
 * clk_ignore_unused, so ask the clock controller (a read of its gate
 * status register) every time.
 */
bool mtk_ccci_ccif_readable(struct mtk_ccci *ccci)
{
	if (!ccif_regs && !ccci->clks_on)
		return false;
	if (ccci->clks_on)
		return true;

	return ccci->ccif_ap_clk && ccci->ccif_md_clk &&
	       __clk_is_enabled(ccci->ccif_ap_clk) &&
	       __clk_is_enabled(ccci->ccif_md_clk);
}

static void mtk_ccci_ccif_read_regs(void __iomem *base,
				    struct mtk_ccci_ccif_regs *r)
{
	r->con = readl(base + CCIF_CON);
	r->busy = readl(base + CCIF_BUSY);
	r->start = readl(base + CCIF_START);
	r->tchnum = readl(base + CCIF_TCHNUM);
	r->rchnum = readl(base + CCIF_RCHNUM);
}

/**
 * mtk_ccci_ccif_snapshot() - read CCIF0 and the modem power state
 * @ccci: driver state
 * @snap: filled in
 *
 * Only reads, and nothing at all unless ccif_regs=1 (or the driver runs
 * CCIF itself). CCIF is read only if mtk_ccci_ccif_readable(); the SPM
 * and topckgen registers are read through their syscon regmaps.
 *
 * Return: 0, or -EPERM/-EAGAIN if CCIF could not be read (the rest of
 * @snap is still filled in).
 */
int mtk_ccci_ccif_snapshot(struct mtk_ccci *ccci,
			   struct mtk_ccci_ccif_snap *snap)
{
	unsigned int i, words;

	memset(snap, 0, sizeof(*snap));
	snap->md_pwr_con = -EPERM;
	snap->md1_clk_mod = -EPERM;
	if (!ccif_regs && !ccci->clks_on)
		return -EPERM;

	snap->md_pwr_con = mtk_ccci_read_regmap(ccci->scpsys, SPM_MD_PWR_CON);
	snap->md1_clk_mod = mtk_ccci_read_regmap(ccci->topckgen,
						 TOPCKGEN_MD1_CLK_MOD);
	if (!mtk_ccci_ccif_readable(ccci))
		return -EAGAIN;

	mtk_ccci_ccif_read_regs(ccci->ccif_ap, &snap->ap);
	mtk_ccci_ccif_read_regs(ccci->ccif_md, &snap->md);

	words = min_t(u32, ccci->ccif_sram_size, sizeof(snap->sram)) / 4;
	for (i = 0; i < words; i++)
		snap->sram[i] = readl(ccci->ccif_ap + CCIF_CHDATA + i * 4);
	snap->valid = true;

	return 0;
}

static const char * const mtk_ccif_ch_names[MTK_CCIF_CH_NUM] = {
	[0] = "ringq0", [1] = "ringq1", [2] = "ringq2", [3] = "ringq3",
	[4] = "ringq4", [5] = "ringq5", [6] = "ringq6", [7] = "ccb",
	[MTK_CCIF_CH_SRAM] = "sram",
	[MTK_CCIF_CH_EX_INIT] = "ex-init",
	[MTK_CCIF_CH_EX_INIT_DONE] = "ex-init-done",
	[MTK_CCIF_CH_EX_CLEARQ_DONE] = "ex-clearq-done",
	[MTK_CCIF_CH_EX_ALLQ_RESET] = "ex-allq-reset",
	[MTK_CCIF_CH_PEER_WAKEUP] = "peer-wakeup",
	[MTK_CCIF_CH_SEQ_ERROR] = "seq-error",
};

static void mtk_ccci_seq_channels(struct seq_file *s, u32 mask)
{
	unsigned int ch;

	if (!mask) {
		seq_puts(s, " -");
		return;
	}
	for (ch = 0; ch < MTK_CCIF_CH_NUM; ch++)
		if (mask & BIT(ch))
			seq_printf(s, " %s", mtk_ccif_ch_names[ch] ?: "?");
}

/*
 * What the snapshot says about the modem. LK powers the modem up and
 * lets it run; the modem then waits for the AP: it raises channel 15
 * with HS1 in the SRAM once it is ready for the runtime data.
 */
enum mtk_ccci_md_guess {
	MTK_CCCI_GUESS_UNKNOWN,
	MTK_CCCI_GUESS_POWERED_OFF,
	MTK_CCCI_GUESS_CLOCK_GATED,
	MTK_CCCI_GUESS_SILENT,
	MTK_CCCI_GUESS_HS1,
	MTK_CCCI_GUESS_EXCEPTION,
	MTK_CCCI_GUESS_TRAFFIC,
};

static const char * const mtk_ccci_guess_text[] = {
	[MTK_CCCI_GUESS_UNKNOWN] = "unknown (CCIF not read)",
	[MTK_CCCI_GUESS_POWERED_OFF] = "modem power domain off",
	[MTK_CCCI_GUESS_CLOCK_GATED] = "modem clocks gated in topckgen",
	[MTK_CCCI_GUESS_SILENT] =
		"powered, no CCIF channel raised: still booting, or waiting before HS1",
	[MTK_CCCI_GUESS_HS1] =
		"HS1 pending: modem waits for the AP runtime data",
	[MTK_CCCI_GUESS_EXCEPTION] = "exception channel raised",
	[MTK_CCCI_GUESS_TRAFFIC] = "channels raised, not a plain HS1",
};

static bool mtk_ccci_snap_is_hs1(const struct mtk_ccci_ccif_snap *snap)
{
	const struct mtk_ccci_ccif_sram_hs *hs = (const void *)snap->sram;
	u32 status = le32_to_cpu(hs->dl_header.status);

	return FIELD_GET(MTK_CCCI_H_CHANNEL, status) == MTK_CCCI_CH_CONTROL_RX &&
	       le32_to_cpu(hs->dl_header.data[1]) ==
			MTK_CCCI_CTL_MD_INIT_START_BOOT &&
	       le32_to_cpu(hs->dl_header.reserved) ==
			MTK_CCCI_CTL_MD_INIT_CHK_ID;
}

static enum mtk_ccci_md_guess
mtk_ccci_guess(const struct mtk_ccci_ccif_snap *snap)
{
	u32 rch = snap->ap.rchnum;

	if (snap->md_pwr_con >= 0 && !(snap->md_pwr_con & SPM_PWR_ACK))
		return MTK_CCCI_GUESS_POWERED_OFF;
	if (snap->md1_clk_mod >= 0 &&
	    (snap->md1_clk_mod & TOPCKGEN_MD1_CLK_MASK))
		return MTK_CCCI_GUESS_CLOCK_GATED;
	if (!snap->valid)
		return MTK_CCCI_GUESS_UNKNOWN;
	if (rch & (BIT(MTK_CCIF_CH_EX_INIT) | BIT(MTK_CCIF_CH_EX_INIT_DONE)))
		return MTK_CCCI_GUESS_EXCEPTION;
	if (rch == BIT(MTK_CCIF_CH_SRAM) && mtk_ccci_snap_is_hs1(snap))
		return MTK_CCCI_GUESS_HS1;
	if (!rch)
		return MTK_CCCI_GUESS_SILENT;
	return MTK_CCCI_GUESS_TRAFFIC;
}

/* one line for the kernel log at probe: survives in pstore/ramoops */
void mtk_ccci_ccif_log_state(struct mtk_ccci *ccci)
{
	struct mtk_ccci_ccif_snap *snap;
	int ret;

	if (!ccif_regs)
		return;
	snap = kmalloc_obj(*snap);
	if (!snap)
		return;

	ret = mtk_ccci_ccif_snapshot(ccci, snap);
	if (ret == -EAGAIN)
		dev_info(ccci->dev, "CCIF clocks off, CCIF not read\n");
	dev_info(ccci->dev,
		 "modem after LK: %s (spm %#x topck %#x rch %#x busy %#x md-rch %#x)\n",
		 mtk_ccci_guess_text[mtk_ccci_guess(snap)],
		 snap->md_pwr_con, snap->md1_clk_mod, snap->ap.rchnum,
		 snap->ap.busy, snap->md.rchnum);
	kfree(snap);
}

static void mtk_ccci_seq_regs(struct seq_file *s, const char *side,
			      const struct mtk_ccci_ccif_regs *r)
{
	seq_printf(s, "%s: con %#010x busy %#010x start %#010x tchnum %#010x rchnum %#010x\n",
		   side, r->con, r->busy, r->start, r->tchnum, r->rchnum);
}

static int mtk_ccci_ccif_show(struct seq_file *s, void *unused)
{
	struct mtk_ccci *ccci = s->private;
	struct mtk_ccci_ccif_snap *snap;
	unsigned int i, words;
	int ret;

	snap = kmalloc_obj(*snap);
	if (!snap)
		return -ENOMEM;

	ret = mtk_ccci_ccif_snapshot(ccci, snap);
	if (ret) {
		seq_printf(s, "CCIF not read: %s\n", ret == -EAGAIN ?
			   "CCIF clocks are off" : "ccif_regs=0");
		goto out;
	}

	mtk_ccci_seq_regs(s, "ap", &snap->ap);
	mtk_ccci_seq_regs(s, "md", &snap->md);
	words = min_t(u32, ccci->ccif_sram_size, sizeof(snap->sram)) / 4;
	for (i = 0; i < words; i += 4)
		seq_printf(s, "sram+%03x %08x %08x %08x %08x\n", i * 4,
			   snap->sram[i], snap->sram[i + 1], snap->sram[i + 2],
			   snap->sram[i + 3]);
out:
	kfree(snap);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_ccci_ccif);

static int mtk_ccci_ccif_state_show(struct seq_file *s, void *unused)
{
	struct mtk_ccci *ccci = s->private;
	const struct mtk_ccci_ccif_sram_hs *hs;
	struct mtk_ccci_ccif_snap *snap;
	unsigned int i, words;
	u32 status, *tail;

	snap = kmalloc_obj(*snap);
	if (!snap)
		return -ENOMEM;
	mtk_ccci_ccif_snapshot(ccci, snap);

	seq_printf(s, "fsm:        %s\n", mtk_ccci_fsm_state_name(ccci->state));
	seq_printf(s, "guess:      %s\n",
		   mtk_ccci_guess_text[mtk_ccci_guess(snap)]);
	if (snap->md_pwr_con >= 0)
		seq_printf(s, "spm md:     PWR_CON %#010x, ack %d\n",
			   snap->md_pwr_con,
			   !!(snap->md_pwr_con & SPM_PWR_ACK));
	else
		seq_printf(s, "spm md:     not read (%d)\n", snap->md_pwr_con);
	if (snap->md1_clk_mod >= 0)
		seq_printf(s, "topckgen:   md1_clk_mod %#010x (%s)\n",
			   snap->md1_clk_mod,
			   snap->md1_clk_mod & TOPCKGEN_MD1_CLK_MASK ?
			   "gated" : "running");
	else
		seq_printf(s, "topckgen:   not read (%d)\n", snap->md1_clk_mod);

	if (!snap->valid) {
		seq_puts(s, "ccif:       not read\n");
		goto out;
	}

	seq_puts(s, "md raised: ");
	mtk_ccci_seq_channels(s, snap->ap.rchnum);
	seq_puts(s, "\nap raised: ");
	mtk_ccci_seq_channels(s, snap->md.rchnum);
	seq_puts(s, "\nap busy:   ");
	mtk_ccci_seq_channels(s, snap->ap.busy);
	seq_puts(s, "\n");

	hs = (const void *)snap->sram;
	status = le32_to_cpu(hs->dl_header.status);
	seq_printf(s, "dl header:  data %#x %#x ch %lu seq %lu reserved %#x%s\n",
		   le32_to_cpu(hs->dl_header.data[0]),
		   le32_to_cpu(hs->dl_header.data[1]),
		   FIELD_GET(MTK_CCCI_H_CHANNEL, status),
		   FIELD_GET(MTK_CCCI_H_SEQ, status),
		   le32_to_cpu(hs->dl_header.reserved),
		   mtk_ccci_snap_is_hs1(snap) ? " (HS1)" : "");
	seq_printf(s, "md query:   head %#010x tail %#010x%s\n",
		   le32_to_cpu(hs->md_query.head_pattern),
		   le32_to_cpu(hs->md_query.tail_pattern),
		   le32_to_cpu(hs->md_query.head_pattern) ==
			MTK_CCCI_MD_FEATURE_PATTERN ? " (valid)" : "");
	if (le32_to_cpu(hs->md_query.head_pattern) ==
	    MTK_CCCI_MD_FEATURE_PATTERN) {
		seq_puts(s, "features:  ");
		for (i = 0; i < MTK_CCCI_FEATURE_COUNT; i++)
			if (hs->md_query.feature[i])
				seq_printf(s, " %u:%#x", i,
					   hs->md_query.feature[i]);
		seq_puts(s, "\n");
	}

	words = min_t(u32, ccci->ccif_sram_size, sizeof(snap->sram)) / 4;
	tail = &snap->sram[words - 3];
	seq_printf(s, "sram tail:  %08x %08x %08x%s\n", tail[0], tail[1],
		   tail[2], tail[0] == MTK_CCIF_SRAM_DBG_MAGIC ?
		   " (debug area set)" : "");
out:
	kfree(snap);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_ccci_ccif_state);

void mtk_ccci_ccif_debugfs_init(struct mtk_ccci *ccci)
{
	debugfs_create_file("ccif_state", 0400, ccci->debugfs, ccci,
			    &mtk_ccci_ccif_state_fops);
	if (ccif_regs)
		debugfs_create_file("ccif", 0400, ccci->debugfs, ccci,
				    &mtk_ccci_ccif_fops);
}

/**
 * mtk_ccci_ccif_init() - find the CCIF resources, touch nothing
 * @ccci: driver state
 *
 * Return: 0 or a negative error.
 */
int mtk_ccci_ccif_init(struct mtk_ccci *ccci)
{
	struct platform_device *pdev = to_platform_device(ccci->dev);
	struct device *dev = ccci->dev;
	struct device_node *np = dev->of_node;
	int i;

	ccci->ccif_ap = devm_platform_ioremap_resource_byname(pdev, "ccif-ap");
	if (IS_ERR(ccci->ccif_ap))
		return PTR_ERR(ccci->ccif_ap);
	ccci->ccif_md = devm_platform_ioremap_resource_byname(pdev, "ccif-md");
	if (IS_ERR(ccci->ccif_md))
		return PTR_ERR(ccci->ccif_md);

	/* only needed to reset CCIF, which the MediaTek downstream driver does on start */
	if (platform_get_resource_byname(pdev, IORESOURCE_MEM, "infra")) {
		ccci->infra = devm_platform_ioremap_resource_byname(pdev,
								    "infra");
		if (IS_ERR(ccci->infra))
			return PTR_ERR(ccci->infra);
	}
	if (of_property_read_u32(np, "mediatek,ccif-reset-bit",
				 &ccci->ccif_rst_bit) ||
	    ccci->ccif_rst_bit > 31)
		ccci->ccif_rst_bit = U32_MAX;

	if (of_property_read_u32(np, "mediatek,ccif-sram-size",
				 &ccci->ccif_sram_size))
		ccci->ccif_sram_size = MTK_CCIF_SRAM_SIZE_DEFAULT;
	if (ccci->ccif_sram_size < sizeof(struct mtk_ccci_ccif_sram_hs) + 12 ||
	    ccci->ccif_sram_size > MTK_CCIF_SRAM_SIZE_DEFAULT ||
	    ccci->ccif_sram_size % 16)
		return dev_err_probe(dev, -EINVAL, "bad CCIF SRAM size %u\n",
				     ccci->ccif_sram_size);

	/* get only: enabling comes with an explicit start */
	ccci->num_clks = devm_clk_bulk_get_all(dev, &ccci->clks);
	if (ccci->num_clks < 0)
		return dev_err_probe(dev, ccci->num_clks,
				     "failed to get clocks\n");
	for (i = 0; i < ccci->num_clks; i++) {
		if (!strcmp(ccci->clks[i].id, "ccif-ap"))
			ccci->ccif_ap_clk = ccci->clks[i].clk;
		else if (!strcmp(ccci->clks[i].id, "ccif-md"))
			ccci->ccif_md_clk = ccci->clks[i].clk;
	}

	/* read-only views of the modem power and clock state */
	ccci->scpsys = syscon_regmap_lookup_by_phandle_optional(np,
							"mediatek,scpsys");
	if (IS_ERR(ccci->scpsys))
		ccci->scpsys = NULL;
	ccci->topckgen = syscon_regmap_lookup_by_phandle_optional(np,
							"mediatek,topckgen");
	if (IS_ERR(ccci->topckgen))
		ccci->topckgen = NULL;

	/* requested only while the driver runs the modem */
	ccci->irq_ccif[0] = platform_get_irq_byname_optional(pdev, "ccif0");
	ccci->irq_ccif[1] = platform_get_irq_byname_optional(pdev, "ccif1");
	ccci->irq_wdt = platform_get_irq_byname_optional(pdev, "wdt");
	for (i = 0; i < ARRAY_SIZE(ccci->irq_ccif); i++)
		if (ccci->irq_ccif[i] == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	if (ccci->irq_wdt == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	return 0;
}

/* --- active part: only reached from an explicit FSM command --- */

u32 mtk_ccci_ccif_pending(struct mtk_ccci *ccci)
{
	return readl(ccci->ccif_ap + CCIF_RCHNUM);
}

void mtk_ccci_ccif_ack(struct mtk_ccci *ccci, u32 channels)
{
	writel(channels, ccci->ccif_ap + CCIF_ACK);
}

/**
 * mtk_ccci_ccif_notify() - raise a channel at the modem
 * @ccci: driver state
 * @channel: CCIF channel
 *
 * Return: 0, or -EBUSY if the modem has not taken the last one yet.
 */
int mtk_ccci_ccif_notify(struct mtk_ccci *ccci, unsigned int channel)
{
	unsigned long flags;
	int ret = 0;

	if (channel >= MTK_CCIF_CH_NUM)
		return -EINVAL;

	spin_lock_irqsave(&ccci->ccif_lock, flags);
	if (readl(ccci->ccif_ap + CCIF_BUSY) & BIT(channel)) {
		ret = -EBUSY;
	} else {
		writel(BIT(channel), ccci->ccif_ap + CCIF_BUSY);
		writel(channel, ccci->ccif_ap + CCIF_TCHNUM);
	}
	spin_unlock_irqrestore(&ccci->ccif_lock, flags);
	return ret;
}

/* HS1 as the modem left it in the SRAM */
void mtk_ccci_ccif_read_hs(struct mtk_ccci *ccci,
			   struct mtk_ccci_header *hdr,
			   struct mtk_ccci_md_feature_query *query)
{
	void __iomem *sram = ccci->ccif_ap + CCIF_CHDATA;
	__le32 *p;
	unsigned int i;

	p = (__le32 *)hdr;
	for (i = 0; i < sizeof(*hdr) / 4; i++)
		p[i] = cpu_to_le32(readl(sram + i * 4));
	sram += sizeof(*hdr);
	p = (__le32 *)query;
	for (i = 0; i < sizeof(*query) / 4; i++)
		p[i] = cpu_to_le32(readl(sram + i * 4));
}

static void mtk_ccci_ccif_sram_clear(void __iomem *base, u32 size)
{
	u32 off;

	for (off = 0; off < size; off += 4)
		writel(0, base + CCIF_CHDATA + off);
}

/*
 * Clear pending/busy channels on both sides (the reset leaves the SRAM
 * alone), clear both SRAM views and tell the modem where its debug area
 * is: the last 12 bytes of the SRAM carry a magic, the modem view of the
 * MDSS_DBG region and its size.
 */
static void mtk_ccci_ccif_reset(struct mtk_ccci *ccci)
{
	const struct mtk_ccci_smem_region *dbg;
	void __iomem *tail;

	if (ccci->infra && ccci->ccif_rst_bit != U32_MAX) {
		writel(BIT(ccci->ccif_rst_bit), ccci->infra + INFRA_RST_SET);
		writel(BIT(ccci->ccif_rst_bit), ccci->infra + INFRA_RST_CLR);
	} else {
		dev_warn(ccci->dev, "no CCIF reset described, only clearing\n");
		writel(readl(ccci->ccif_ap + CCIF_RCHNUM),
		       ccci->ccif_ap + CCIF_ACK);
	}

	mtk_ccci_ccif_sram_clear(ccci->ccif_ap, ccci->ccif_sram_size);
	mtk_ccci_ccif_sram_clear(ccci->ccif_md, ccci->ccif_sram_size);

	dbg = mtk_ccci_lk_smem(ccci, MTK_CCCI_SMEM_MDSS_DBG);
	if (!dbg)
		return;
	tail = ccci->ccif_ap + CCIF_CHDATA + ccci->ccif_sram_size - 12;
	writel(MTK_CCIF_SRAM_DBG_MAGIC, tail);
	writel(mtk_ccci_lk_smem_md_addr(dbg), tail + 4);
	writel(le32_to_cpu(dbg->size), tail + 8);
}

/* map a shared memory region LK described (inside a no-map carveout) */
static void __iomem *mtk_ccci_smem_map(struct mtk_ccci *ccci,
				       enum mtk_ccci_smem_id id, u32 *size)
{
	const struct mtk_ccci_smem_region *r = mtk_ccci_lk_smem(ccci, id);
	void __iomem *p;

	if (!r) {
		dev_err(ccci->dev, "no shared memory region %d from LK\n", id);
		return NULL;
	}
	*size = le32_to_cpu(r->size);
	p = ioremap_wc(le64_to_cpu(r->ap_phys), *size);
	if (!p)
		dev_err(ccci->dev, "cannot map shared memory region %d\n", id);
	return p;
}

/**
 * mtk_ccci_ccif_start() - take CCIF over for a modem (re)start
 * @ccci: driver state
 *
 * The stock order: CCIF clocks, ring buffers laid out in the CCISM shared
 * memory (normal and exception queues), CCIF reset with both SRAM views
 * cleared. The runtime data region is mapped for the answer to HS1.
 *
 * Return: 0 or a negative error.
 */
int mtk_ccci_ccif_start(struct mtk_ccci *ccci)
{
	u32 size;
	int ret;

	if (!ccci->clks_on) {
		ret = clk_bulk_prepare_enable(ccci->num_clks, ccci->clks);
		if (ret)
			return ret;
		ccci->clks_on = true;
	}

	ret = -ENOMEM;
	ccci->ccism = mtk_ccci_smem_map(ccci, MTK_CCCI_SMEM_CCISM_MCU, &size);
	if (!ccci->ccism)
		goto err;
	ret = mtk_ccci_rings_init(ccci, ccci->ccism, size);
	if (ret)
		goto err;

	ret = -ENOMEM;
	ccci->ccism_exp = mtk_ccci_smem_map(ccci, MTK_CCCI_SMEM_CCISM_MCU_EXP,
					    &size);
	if (!ccci->ccism_exp)
		goto err;
	ret = mtk_ccci_rings_exp_init(ccci, ccci->ccism_exp, size);
	if (ret)
		goto err;

	ret = -ENOMEM;
	ccci->rt_data = mtk_ccci_smem_map(ccci, MTK_CCCI_SMEM_RUNTIME_DATA,
					  &ccci->rt_data_size);
	if (!ccci->rt_data)
		goto err;

	mtk_ccci_ccif_reset(ccci);
	return 0;

err:
	mtk_ccci_ccif_stop(ccci);
	return ret;
}

/**
 * mtk_ccci_ccif_stop() - back to quiet after the modem is off
 * @ccci: driver state
 *
 * Unmaps the shared memory and resets CCIF, as the MediaTek downstream driver does
 * once the modem is gated. The CCIF clocks stay on: the stock way to
 * gate CCIF4/5 needs registers this driver does not map, and the
 * clocks were on before the driver started anyway.
 */
void mtk_ccci_ccif_stop(struct mtk_ccci *ccci)
{
	if (ccci->clks_on && ccci->ccism)
		mtk_ccci_ccif_reset(ccci);

	if (ccci->rt_data) {
		iounmap(ccci->rt_data);
		ccci->rt_data = NULL;
	}
	if (ccci->ccism_exp) {
		iounmap(ccci->ccism_exp);
		ccci->ccism_exp = NULL;
		memset(ccci->ring_exp, 0, sizeof(ccci->ring_exp));
	}
	if (ccci->ccism) {
		iounmap(ccci->ccism);
		ccci->ccism = NULL;
		memset(ccci->ring, 0, sizeof(ccci->ring));
	}
}

/*
 * Both CCIF0 lines report the channels the modem raised (the MediaTek downstream
 * driver splits them: data queues on the first, SRAM and exceptions on
 * the second). Take whatever is pending, acknowledge it so the level
 * interrupt drops, and leave the work to the RX worker.
 */
static irqreturn_t mtk_ccci_ccif_isr(int irq, void *data)
{
	struct mtk_ccci *ccci = data;
	u32 rch;

	rch = readl(ccci->ccif_ap + CCIF_RCHNUM);
	if (!rch)
		return IRQ_NONE;
	writel(rch, ccci->ccif_ap + CCIF_ACK);
	atomic_or(rch, &ccci->ccif_pending);
	ccci->stats.irqs++;
	mtk_ccci_hif_kick(ccci);
	return IRQ_HANDLED;
}

static irqreturn_t mtk_ccci_wdt_isr(int irq, void *data)
{
	struct mtk_ccci *ccci = data;

	/* one shot: re-armed on the next start */
	disable_irq_nosync(irq);
	ccci->wdt_fired = true;
	dev_err(ccci->dev, "modem watchdog fired\n");
	mtk_ccci_fsm_exception(ccci, 0);
	return IRQ_HANDLED;
}

/**
 * mtk_ccci_ccif_irq_start() - take the CCIF interrupts
 * @ccci: driver state
 *
 * Only while the driver runs the modem. Without interrupts in the DT the
 * RX worker still polls.
 *
 * Return: 0 or a negative error.
 */
int mtk_ccci_ccif_irq_start(struct mtk_ccci *ccci)
{
	unsigned int i;
	int ret;

	if (ccci->irqs_on)
		return 0;

	for (i = 0; i < ARRAY_SIZE(ccci->irq_ccif); i++) {
		if (ccci->irq_ccif[i] <= 0)
			continue;
		ret = request_irq(ccci->irq_ccif[i], mtk_ccci_ccif_isr, 0,
				  i ? "ccci-ccif1" : "ccci-ccif0", ccci);
		if (ret) {
			dev_err(ccci->dev, "CCIF interrupt %u: %d\n", i, ret);
			while (i--)
				if (ccci->irq_ccif[i] > 0)
					free_irq(ccci->irq_ccif[i], ccci);
			return ret;
		}
	}
	ccci->irqs_on = true;
	return 0;
}

void mtk_ccci_ccif_irq_stop(struct mtk_ccci *ccci)
{
	unsigned int i;

	if (!ccci->irqs_on)
		return;
	for (i = 0; i < ARRAY_SIZE(ccci->irq_ccif); i++)
		if (ccci->irq_ccif[i] > 0)
			free_irq(ccci->irq_ccif[i], ccci);
	ccci->irqs_on = false;
}

/**
 * mtk_ccci_wdt_irq_start() - watch the modem watchdog
 * @ccci: driver state
 *
 * As the MediaTek downstream driver: from the release of the modem until it is
 * stopped. One shot, logged and treated as a modem exception.
 *
 * Return: 0 or a negative error.
 */
int mtk_ccci_wdt_irq_start(struct mtk_ccci *ccci)
{
	int ret;

	if (ccci->irq_wdt <= 0 || ccci->wdt_on)
		return 0;
	ccci->wdt_fired = false;
	ret = request_irq(ccci->irq_wdt, mtk_ccci_wdt_isr, 0, "ccci-md-wdt",
			  ccci);
	if (ret)
		return ret;
	ccci->wdt_on = true;
	return 0;
}

void mtk_ccci_wdt_irq_stop(struct mtk_ccci *ccci)
{
	if (!ccci->wdt_on)
		return;
	free_irq(ccci->irq_wdt, ccci);
	/* as the stock stop: no stale watchdog for the next start */
	irq_set_irqchip_state(ccci->irq_wdt, IRQCHIP_STATE_PENDING, false);
	ccci->wdt_on = false;
}
