// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 MediaTek Inc.
 * Author: James Liao <jamesjj.liao@mediatek.com>
 */

#include <linux/arm-smccc.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/sizes.h>
#include <linux/soc/mediatek/mtk-mmsys.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>

#include "mtk-mmsys.h"
#include "mt6878-mmsys.h"
#include "mt8167-mmsys.h"
#include "mt8173-mmsys.h"
#include "mt8183-mmsys.h"
#include "mt8186-mmsys.h"
#include "mt8188-mmsys.h"
#include "mt8192-mmsys.h"
#include "mt8195-mmsys.h"
#include "mt8365-mmsys.h"

#define MMSYS_SW_RESET_PER_REG 32

/*
 * Experiment (MT6878): when the mmsys registers of the path do not take
 * writes after a power down, issue the MM DEVAPC disable call of the secure
 * firmware (MTK_SIP_CMDQ_CONTROL, CMDQ_MMUEN_SET_DEVAPC_DISABLE: what the
 * MediaTek kernel does at every MM_INFRA power on, mtk-mminfra-debug.c)
 * and write them once more.
 */
static bool devapc_retry = true;
module_param(devapc_retry, bool, 0644);
MODULE_PARM_DESC(devapc_retry, "Retry path writes that do not take after the MM DEVAPC disable call (default 1)");

#define MTK_MMSYS_SIP_CMDQ_CONTROL	MTK_SIP_SMC_CMD(0x518)
#define MTK_MMSYS_SIP_ENABLE_DISP_VA	1
#define MTK_MMSYS_SIP_PREBUILT_ENABLE	3
#define MTK_MMSYS_SIP_PREBUILT_DISABLE	4
#define MTK_MMSYS_SIP_DEVAPC_DISABLE	7
/* GCE-D, the display GCE: hardware id 0 of cmdq-platform-mt6878.c */
#define MTK_MMSYS_GCE_D_HWID		0

/*
 * Experiment (MT6878): when the path registers still do not take writes,
 * run the CMDQ_ENABLE_DISP_VA handshake of the MediaTek kernel from the
 * CPU: the secure firmware call, then the DISP_VA_START token set through
 * the GCE token update register, then wait for DISP_VA_END, and write the
 * registers once more.
 */
static bool disp_va = true;
module_param(disp_va, bool, 0644);
MODULE_PARM_DESC(disp_va, "Try the DISP_VA secure firmware handshake when path writes do not take (default 1)");

/*
 * The DISP_VA handshake above needs the secure firmware's own GCE threads,
 * which the MediaTek GCE driver installs with CMDQ_PREBUILT_ENABLE (hw id)
 * each time it powers the GCE up ("prebuilt-enable" in its DT) and removes
 * with CMDQ_PREBUILT_DISABLE before it powers it down. Without them nothing
 * consumes DISP_VA_START (on the device the start token stayed set). 1: wrap
 * the handshake in PREBUILT_ENABLE/DISABLE for GCE-D. Default 0.
 */
static bool disp_va_prebuilt;
module_param(disp_va_prebuilt, bool, 0644);
MODULE_PARM_DESC(disp_va_prebuilt, "Install the secure GCE prebuilt threads around the DISP_VA handshake (default 0)");

/*
 * GCE-D bring-up from the CPU before the DISP_VA handshake, the steps the
 * MediaTek kernel does whenever it powers the GCE up (we have no GCE
 * driver, and a power down of MM_INFRA resets the GCE), a bit mask:
 * bit 0: GCE_GCTL_VALUE = 0x20002 (mtk-mminfra-debug.c, GENPD_NOTIFY_ON of
 *        MM_INFRA: "enable gce apsrc");
 * bit 1: the core setup of cmdq_mbox_enable() (mtk-cmdq-mailbox-ext.c):
 *        BUS_GCTL ULTRA_EN, TPR_MASK = TPR_EN, THR_SLOT_CYCLES = 0x3200;
 * bit 2: the tokens of cmdq_init_cpu(): every token cleared, then the
 *        "default-tokens" of the GCE DT node set (the prebuilt lock tokens
 *        among them), as before CMDQ_PREBUILT_ENABLE;
 * bit 3: diagnostics: GCE core registers and every enabled GCE thread
 *        (address, status, the token it waits for) before and after.
 * Default 0.
 */
static unsigned int gce_init;
module_param(gce_init, uint, 0644);
MODULE_PARM_DESC(gce_init, "GCE-D setup before the DISP_VA handshake: 1 apsrc, 2 core, 4 tokens, 8 thread dump (default 0)");

/* GCE registers, as the MediaTek GCE driver uses them */
#define GCE_THR_SLOT_CYCLES		0x30
#define GCE_SYNC_TOKEN_ID		0x60
#define GCE_SYNC_TOKEN_VAL		0x64
#define GCE_SYNC_TOKEN_UPD		0x68
#define GCE_SYNC_TOKEN_SET		BIT(16)
#define GCE_BUS_GCTL			0x40
#define GCE_BUS_ULTRA_EN		BIT(0)
#define GCE_GCTL_VALUE			0x48
#define GCE_GCTL_APSRC			0x20002
#define GCE_TPR_MASK			0xd0
#define GCE_TPR_EN			BIT(31)
#define GCE_THR_ACTIVE_SLOT_CYCLES	0x3200
#define GCE_EVENT_MAX			0x3ff
#define GCE_THR_BASE			0x100
#define GCE_THR_SIZE			0x80
#define GCE_THR_COUNT			32
#define GCE_THR_ENABLE			0x04
#define GCE_THR_STATUS			0x0c
#define GCE_THR_CURR_ADDR		0x20
#define GCE_THR_END_ADDR		0x24
#define GCE_THR_WAIT_TOKEN		0x30
#define GCE_MAP_SIZE			SZ_16K

#define GCE_INIT_APSRC			BIT(0)
#define GCE_INIT_CORE			BIT(1)
#define GCE_INIT_TOKENS			BIT(2)
#define GCE_INIT_DUMP			BIT(3)
#define MMINFRA_CG_CON0			0x100
#define MMINFRA_CG_CLR0			0x108
#define MMINFRA_CG_CON1			0x110
#define MMINFRA_CG_CLR1			0x118
#define MMINFRA_CG0_GCE			(BIT(0) | BIT(1))
#define MMINFRA_CG1_GCE26M		BIT(17)

/*
 * MT6878: the boot loader path is programmed by a secure boot stage
 * (bl2_ext, before BL31), and after a power down of the DISP domain parts
 * of the mmsys config (0x0fc, the crossbars 0xc08..0xe48) no longer take
 * non-secure writes. 1: keep the DISP domain powered through system sleep
 * (the mmsys device stays on the awake path, the domain is an active
 * wakeup one), so the path survives; 0: let it go down.
 * TODO: what opens these registers again after a DISP power cycle?
 */
static bool keep_disp_on = true;
module_param(keep_disp_on, bool, 0644);
MODULE_PARM_DESC(keep_disp_on, "Keep the display power domain on through system sleep (default 1: after a DISP power cycle the mmsys crossbars and 0x0fc are no longer writable from the normal world, so the display would freeze)");

/*
 * With keep_disp_on the GCE-D stays powered through the sleep, and its
 * GCE_GCTL_VALUE keeps the value it has while MM_INFRA is on: 0x20002,
 * software mode for the apsrc request with the request set (the MediaTek
 * kernel writes this at every MM_INFRA power on, "enable gce apsrc", and
 * 0 at its PRE_OFF, "disable gce apsrc": mtk-mminfra-debug.c). The SPM
 * then sees a GCE-D request for the whole sleep (gce_d in the mtk-lpm
 * requests). Nothing here uses the GCE: 1 writes 0 for the sleep, as the
 * downstream kernel before MM_INFRA goes down, and the old value back on resume.
 * TODO: 0x20002 is also read right after an MM_INFRA power on, before
 * the kernel writes it: reset value or set by the firmware?
 */
static bool sleep_gce_apsrc_off = true;
module_param(sleep_gce_apsrc_off, bool, 0644);
MODULE_PARM_DESC(sleep_gce_apsrc_off,
		 "With keep_disp_on, drop the GCE-D apsrc request (GCE_GCTL_VALUE 0) for the sleep (default 1)");

/*
 * With keep_disp_on the SPM still sees an MMSYS request for apsrc, ddren
 * and vrf18 through the whole sleep (SPM_REQ_STA_10 bits 10..12, never set
 * in the deep sleep logs of the factory firmware): DRAM cannot go to self refresh, 26M stays
 * on. Two experiments to drop it while DISP stays powered, both off by
 * default and undone on resume:
 * sleep_sodi: a value for MMSYS_SODI_REQ_MASK (0x0f4), the register through
 *   which the MediaTek DRM driver selects/forces the mmsys DRAM requests
 *   (mt6878_mtk_sodi_config: |= 0x6c0; bits 0/1 select apsrc/ddren, 8..11
 *   select and 12..15 force the RDMA PD/CG modes); -1: not touched;
 * sleep_cg_off: gate every mmsys clock that is open (CG_CON0/1) for the
 *   sleep, open the same ones again on resume.
 * MMSYS_MISC (0x0f0), the SODI mask and MMSYS_EMI_REQ_CTL (0x0f8) are logged
 * at every suspend with keep_disp_on.
 */
static int sleep_sodi = -1;
module_param(sleep_sodi, int, 0644);
MODULE_PARM_DESC(sleep_sodi,
		 "With keep_disp_on, MMSYS_SODI_REQ_MASK (0x0f4) for the sleep (experiment; default -1: untouched)");

static bool sleep_cg_off;
module_param(sleep_cg_off, bool, 0644);
MODULE_PARM_DESC(sleep_cg_off,
		 "With keep_disp_on, gate the open mmsys clocks for the sleep (experiment; default 0)");

#define MMSYS_MISC			0x0f0
#define MMSYS_SODI_REQ_MASK		0x0f4
#define MMSYS_EMI_REQ_CTL		0x0f8

static bool keep_boot_path = true;
module_param(keep_boot_path, bool, 0644);
MODULE_PARM_DESC(keep_boot_path,
		 "Write the display path of the boot loader back after a power down (MT6878)");

static const struct mtk_mmsys_driver_data mt2701_mmsys_driver_data = {
	.clk_driver = "clk-mt2701-mm",
	.routes = mmsys_default_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_default_routing_table),
};

static const struct mtk_mmsys_driver_data mt2712_mmsys_driver_data = {
	.clk_driver = "clk-mt2712-mm",
	.routes = mmsys_default_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_default_routing_table),
};

static const struct mtk_mmsys_driver_data mt6779_mmsys_driver_data = {
	.clk_driver = "clk-mt6779-mm",
};

static const struct mtk_mmsys_driver_data mt6795_mmsys_driver_data = {
	.clk_driver = "clk-mt6795-mm",
	.routes = mt8173_mmsys_routing_table,
	.num_routes = ARRAY_SIZE(mt8173_mmsys_routing_table),
	.sw0_rst_offset = MT8183_MMSYS_SW0_RST_B,
	.num_resets = 64,
};

/*
 * No routes: the display path is the one the boot loader programmed
 * (OVL0_2L cascaded through OVL1_2L and OVL2_2L, then the picture quality
 * engines in the state it left them, into DSI0). Its registers are read at
 * probe and written back when the DISP domain has been powered down.
 */
static const struct mtk_mmsys_driver_data mt6878_mmsys_driver_data = {
	.clk_driver = "clk-mt6878-mm",
	.boot_path = &mt6878_boot_path,
};

static const struct mtk_mmsys_driver_data mt6797_mmsys_driver_data = {
	.clk_driver = "clk-mt6797-mm",
};

static const struct mtk_mmsys_driver_data mt8167_mmsys_driver_data = {
	.clk_driver = "clk-mt8167-mm",
	.routes = mt8167_mmsys_routing_table,
	.num_routes = ARRAY_SIZE(mt8167_mmsys_routing_table),
};

static const struct mtk_mmsys_driver_data mt8173_mmsys_driver_data = {
	.clk_driver = "clk-mt8173-mm",
	.routes = mt8173_mmsys_routing_table,
	.num_routes = ARRAY_SIZE(mt8173_mmsys_routing_table),
	.sw0_rst_offset = MT8183_MMSYS_SW0_RST_B,
	.num_resets = 64,
};

static const struct mtk_mmsys_driver_data mt8183_mmsys_driver_data = {
	.clk_driver = "clk-mt8183-mm",
	.routes = mmsys_mt8183_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8183_routing_table),
	.sw0_rst_offset = MT8183_MMSYS_SW0_RST_B,
	.num_resets = 32,
};

static const struct mtk_mmsys_driver_data mt8186_mmsys_driver_data = {
	.clk_driver = "clk-mt8186-mm",
	.routes = mmsys_mt8186_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8186_routing_table),
	.sw0_rst_offset = MT8186_MMSYS_SW0_RST_B,
	.num_resets = 32,
};

static const struct mtk_mmsys_driver_data mt8188_vdosys0_driver_data = {
	.clk_driver = "clk-mt8188-vdo0",
	.routes = mmsys_mt8188_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8188_routing_table),
	.sw0_rst_offset = MT8188_VDO0_SW0_RST_B,
	.rst_tb = mmsys_mt8188_vdo0_rst_tb,
	.num_resets = ARRAY_SIZE(mmsys_mt8188_vdo0_rst_tb),
};

static const struct mtk_mmsys_driver_data mt8188_vdosys1_driver_data = {
	.clk_driver = "clk-mt8188-vdo1",
	.routes = mmsys_mt8188_vdo1_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8188_vdo1_routing_table),
	.sw0_rst_offset = MT8188_VDO1_SW0_RST_B,
	.rst_tb = mmsys_mt8188_vdo1_rst_tb,
	.num_resets = ARRAY_SIZE(mmsys_mt8188_vdo1_rst_tb),
	.vsync_len = 1,
};

static const struct mtk_mmsys_driver_data mt8188_vppsys0_driver_data = {
	.clk_driver = "clk-mt8188-vpp0",
	.is_vppsys = true,
};

static const struct mtk_mmsys_driver_data mt8188_vppsys1_driver_data = {
	.clk_driver = "clk-mt8188-vpp1",
	.is_vppsys = true,
};

static const struct mtk_mmsys_driver_data mt8192_mmsys_driver_data = {
	.clk_driver = "clk-mt8192-mm",
	.routes = mmsys_mt8192_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8192_routing_table),
	.sw0_rst_offset = MT8186_MMSYS_SW0_RST_B,
	.num_resets = 32,
};

static const struct mtk_mmsys_driver_data mt8195_vdosys0_driver_data = {
	.clk_driver = "clk-mt8195-vdo0",
	.routes = mmsys_mt8195_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8195_routing_table),
};

static const struct mtk_mmsys_driver_data mt8195_vdosys1_driver_data = {
	.clk_driver = "clk-mt8195-vdo1",
	.routes = mmsys_mt8195_vdo1_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8195_vdo1_routing_table),
	.sw0_rst_offset = MT8195_VDO1_SW0_RST_B,
	.num_resets = 64,
};

static const struct mtk_mmsys_driver_data mt8195_vppsys0_driver_data = {
	.clk_driver = "clk-mt8195-vpp0",
	.is_vppsys = true,
};

static const struct mtk_mmsys_driver_data mt8195_vppsys1_driver_data = {
	.clk_driver = "clk-mt8195-vpp1",
	.is_vppsys = true,
};

static const struct mtk_mmsys_driver_data mt8365_mmsys_driver_data = {
	.clk_driver = "clk-mt8365-mm",
	.routes = mt8365_mmsys_routing_table,
	.num_routes = ARRAY_SIZE(mt8365_mmsys_routing_table),
};

struct mtk_mmsys_kept {
	void __iomem *regs;
	u32 *vals;
};

struct mtk_mmsys {
	void __iomem *regs;
	const struct mtk_mmsys_driver_data *data;
	/* boot loader path: clock gates and registers found at probe */
	struct mtk_mmsys_kept *kept;
	u32 boot_cg[2];
	unsigned int boot_path_restores;
	/* config registers of boot_path->cfg_ranges at probe */
	u32 *cfg_boot;
	void __iomem *gce;
	void __iomem *mminfra_cfg;
	/* GCE_GCTL_VALUE before the sleep, see sleep_gce_apsrc_off */
	u32 gce_gctl_saved;
	bool gce_gctl_cleared;
	/* sleep_sodi, sleep_cg_off: what to put back on resume */
	u32 sodi_saved;
	bool sodi_set;
	u32 cg_gated[2];
	struct platform_device *clks_pdev;
	struct platform_device *drm_pdev;
	spinlock_t lock; /* protects mmsys_sw_rst_b reg */
	struct reset_controller_dev rcdev;
	struct cmdq_client_reg cmdq_base;
};

static void mtk_mmsys_update_bits(struct mtk_mmsys *mmsys, u32 offset, u32 mask, u32 val,
				  struct cmdq_pkt *cmdq_pkt)
{
	int ret;
	u32 tmp;

	if (mmsys->cmdq_base.size && cmdq_pkt) {
		ret = mmsys->cmdq_base.pkt_write_mask(cmdq_pkt,
						      mmsys->cmdq_base.subsys,
						      mmsys->cmdq_base.pa_base,
						      mmsys->cmdq_base.offset + offset,
						      val, mask);
		if (ret)
			pr_debug("CMDQ unavailable: using CPU write\n");
		else
			return;
	}
	tmp = readl_relaxed(mmsys->regs + offset);
	tmp = (tmp & ~mask) | (val & mask);
	writel_relaxed(tmp, mmsys->regs + offset);
}

/*
 * Display path of the boot loader
 *
 * The DRM driver drives only part of the path some boot loaders set up
 * (MT6878: OVL0_2L and DSI0); the rest (crossbars, the engines in between,
 * their clocks, the larb) stays as the boot loader left it. A power down
 * of the display domain resets all of it, and frames then never reach the
 * output. The registers are read at probe and written back when the path
 * is connected again after such a power down.
 */
static bool mtk_mmsys_kept_gated(struct mtk_mmsys *mmsys,
				 const struct mtk_mmsys_keep_block *blk)
{
	const struct mtk_mmsys_boot_path *path = mmsys->data->boot_path;

	return blk->cg_bit >= 0 &&
	       (readl(mmsys->regs + path->cg_sta[0]) & BIT(blk->cg_bit));
}

static unsigned int mtk_mmsys_cfg_words(const struct mtk_mmsys_boot_path *path)
{
	unsigned int i, n = 0;

	for (i = 0; i < path->num_cfg_ranges; i++)
		n += (path->cfg_ranges[i].end - path->cfg_ranges[i].start) / 4;
	return n;
}

static void mtk_mmsys_cfg_read(struct device *dev, struct mtk_mmsys *mmsys)
{
	const struct mtk_mmsys_boot_path *path = mmsys->data->boot_path;
	unsigned int i, n = 0, words = mtk_mmsys_cfg_words(path);
	u16 off;

	if (!words)
		return;
	mmsys->cfg_boot = devm_kcalloc(dev, words, sizeof(u32), GFP_KERNEL);
	if (!mmsys->cfg_boot)
		return;
	for (i = 0; i < path->num_cfg_ranges; i++)
		for (off = path->cfg_ranges[i].start;
		     off < path->cfg_ranges[i].end; off += 4)
			mmsys->cfg_boot[n++] = readl(mmsys->regs + off);
}

/* mmsys config registers that differ from probe, clock gates left out */
static void mtk_mmsys_cfg_compare(struct device *dev, struct mtk_mmsys *mmsys)
{
	const struct mtk_mmsys_boot_path *path = mmsys->data->boot_path;
	unsigned int i, n = 0, shown = 0, differ = 0;
	u16 off;
	u32 val;

	if (!mmsys->cfg_boot)
		return;
	for (i = 0; i < path->num_cfg_ranges; i++) {
		for (off = path->cfg_ranges[i].start;
		     off < path->cfg_ranges[i].end; off += 4, n++) {
			if (off == path->cg_sta[0] || off == path->cg_sta[1])
				continue;
			val = readl(mmsys->regs + off);
			if (val == mmsys->cfg_boot[n])
				continue;
			differ++;
			if (shown++ < 96)
				dev_info(dev, "boot path: config +%#05x: boot %08x now %08x\n",
					 off, mmsys->cfg_boot[n], val);
		}
	}
	dev_info(dev, "boot path: %u config registers differ from the boot state\n",
		 differ);
}

static u32 mtk_mmsys_gce_token(struct mtk_mmsys *mmsys, u16 token)
{
	writel(token, mmsys->gce + GCE_SYNC_TOKEN_ID);
	return readl(mmsys->gce + GCE_SYNC_TOKEN_VAL);
}

/* GCE-D core registers and its enabled threads, see gce_init bit 3 */
static void mtk_mmsys_gce_dump(struct device *dev, struct mtk_mmsys *mmsys,
			       const char *when)
{
	const struct mtk_mmsys_boot_path *path = mmsys->data->boot_path;
	void __iomem *gce = mmsys->gce;
	unsigned int i, shown = 0;

	dev_info(dev, "boot path: GCE %s: bus gctl %08x, gctl value %08x, slot cycles %08x, tpr mask %08x, tokens start %u end %u disp lock %u\n",
		 when, readl(gce + GCE_BUS_GCTL), readl(gce + GCE_GCTL_VALUE),
		 readl(gce + GCE_THR_SLOT_CYCLES), readl(gce + GCE_TPR_MASK),
		 mtk_mmsys_gce_token(mmsys, path->disp_va_start),
		 mtk_mmsys_gce_token(mmsys, path->disp_va_end),
		 path->num_gce_tokens ?
		 mtk_mmsys_gce_token(mmsys, path->gce_tokens[path->num_gce_tokens - 1]) : 0);

	for (i = 0; i < GCE_THR_COUNT; i++) {
		void __iomem *thr = gce + GCE_THR_BASE + i * GCE_THR_SIZE;

		if (!(readl(thr + GCE_THR_ENABLE) & BIT(0)))
			continue;
		shown++;
		dev_info(dev, "boot path: GCE %s: thread %u pc %08x end %08x status %08x wait %08x\n",
			 when, i, readl(thr + GCE_THR_CURR_ADDR),
			 readl(thr + GCE_THR_END_ADDR), readl(thr + GCE_THR_STATUS),
			 readl(thr + GCE_THR_WAIT_TOKEN));
	}
	if (!shown)
		dev_info(dev, "boot path: GCE %s: no thread enabled (or none visible)\n",
			 when);
}

/* The GCE-D power up steps of the MediaTek kernel, see gce_init */
static void mtk_mmsys_gce_init(struct device *dev, struct mtk_mmsys *mmsys,
			       unsigned int steps)
{
	const struct mtk_mmsys_boot_path *path = mmsys->data->boot_path;
	void __iomem *gce = mmsys->gce;
	unsigned int i;

	if (steps & GCE_INIT_APSRC)
		writel(GCE_GCTL_APSRC, gce + GCE_GCTL_VALUE);

	if (steps & GCE_INIT_CORE) {
		writel(readl(gce + GCE_BUS_GCTL) | GCE_BUS_ULTRA_EN,
		       gce + GCE_BUS_GCTL);
		writel(GCE_TPR_EN, gce + GCE_TPR_MASK);
		writel(GCE_THR_ACTIVE_SLOT_CYCLES, gce + GCE_THR_SLOT_CYCLES);
	}

	if (steps & GCE_INIT_TOKENS) {
		for (i = 0; i <= GCE_EVENT_MAX; i++)
			writel(i, gce + GCE_SYNC_TOKEN_UPD);
		for (i = 0; i < path->num_gce_tokens; i++)
			writel(GCE_SYNC_TOKEN_SET | path->gce_tokens[i],
			       gce + GCE_SYNC_TOKEN_UPD);
	}
}

/* The DISP_VA handshake, see disp_va; returns whether DISP_VA_END came */
static bool mtk_mmsys_disp_va(struct device *dev, struct mtk_mmsys *mmsys)
{
	const struct mtk_mmsys_boot_path *path = mmsys->data->boot_path;
	struct arm_smccc_res res, pre_on = { 0 }, pre_off = { 0 };
	bool prebuilt = READ_ONCE(disp_va_prebuilt);
	unsigned int steps = READ_ONCE(gce_init);
	u32 cg0, cg1, end = 0, end0, start = 0;
	int i;

	if (!mmsys->gce || !mmsys->mminfra_cfg)
		return false;

	/* the GCE clocks, as the MediaTek driver powers the GCE for the call */
	cg0 = readl(mmsys->mminfra_cfg + MMINFRA_CG_CON0);
	cg1 = readl(mmsys->mminfra_cfg + MMINFRA_CG_CON1);
	if (cg0 & MMINFRA_CG0_GCE)
		writel(cg0 & MMINFRA_CG0_GCE, mmsys->mminfra_cfg + MMINFRA_CG_CLR0);
	if (cg1 & MMINFRA_CG1_GCE26M)
		writel(MMINFRA_CG1_GCE26M, mmsys->mminfra_cfg + MMINFRA_CG_CLR1);

	if (steps & GCE_INIT_DUMP)
		mtk_mmsys_gce_dump(dev, mmsys, "before the setup");
	mtk_mmsys_gce_init(dev, mmsys, steps);

	/* a stale DISP_VA_END must not look like an answer */
	end0 = mtk_mmsys_gce_token(mmsys, path->disp_va_end);
	writel(path->disp_va_end, mmsys->gce + GCE_SYNC_TOKEN_UPD);

	if (prebuilt)
		arm_smccc_smc(MTK_MMSYS_SIP_CMDQ_CONTROL,
			      MTK_MMSYS_SIP_PREBUILT_ENABLE, MTK_MMSYS_GCE_D_HWID,
			      0, 0, 0, 0, 0, &pre_on);
	arm_smccc_smc(MTK_MMSYS_SIP_CMDQ_CONTROL, MTK_MMSYS_SIP_ENABLE_DISP_VA,
		      0, 0, 0, 0, 0, 0, &res);
	writel(GCE_SYNC_TOKEN_SET | path->disp_va_start,
	       mmsys->gce + GCE_SYNC_TOKEN_UPD);
	for (i = 0; i < 200; i++) {
		end = mtk_mmsys_gce_token(mmsys, path->disp_va_end);
		if (end)
			break;
		usleep_range(100, 200);
	}
	/* 0: a GCE thread waited for it and took it */
	start = mtk_mmsys_gce_token(mmsys, path->disp_va_start);
	if (steps & GCE_INIT_DUMP)
		mtk_mmsys_gce_dump(dev, mmsys, "after the handshake");
	if (prebuilt)
		arm_smccc_smc(MTK_MMSYS_SIP_CMDQ_CONTROL,
			      MTK_MMSYS_SIP_PREBUILT_DISABLE, MTK_MMSYS_GCE_D_HWID,
			      0, 0, 0, 0, 0, &pre_off);
	dev_info(dev, "boot path: DISP_VA call a0 %#lx, prebuilt %s (a0 %#lx/%#lx), mminfra cg %08x %08x -> %08x %08x, end token before %u, start token %u (0: taken), end token %u after %d polls\n",
		 res.a0, prebuilt ? "on" : "off", pre_on.a0, pre_off.a0,
		 cg0, cg1, readl(mmsys->mminfra_cfg + MMINFRA_CG_CON0),
		 readl(mmsys->mminfra_cfg + MMINFRA_CG_CON1),
		 end0, start, end, i);
	/* a GCE wait would have consumed both tokens */
	writel(path->disp_va_end, mmsys->gce + GCE_SYNC_TOKEN_UPD);
	writel(path->disp_va_start, mmsys->gce + GCE_SYNC_TOKEN_UPD);

	return end;
}

/* Write the mmsys block of the path again; registers that do not read back */
static unsigned int mtk_mmsys_boot_path_rewrite0(struct mtk_mmsys *mmsys)
{
	const struct mtk_mmsys_keep_block *blk = &mmsys->data->boot_path->blocks[0];
	const struct mtk_mmsys_kept *kept = &mmsys->kept[0];
	unsigned int j, stuck = 0;

	for (j = 0; j < blk->num_regs; j++) {
		writel(kept->vals[j], kept->regs + blk->regs[j]);
		if (readl(kept->regs + blk->regs[j]) != kept->vals[j])
			stuck++;
	}
	return stuck;
}

static int mtk_mmsys_boot_path_read(struct device *dev, struct mtk_mmsys *mmsys,
				    struct resource *res)
{
	const struct mtk_mmsys_boot_path *path = mmsys->data->boot_path;
	struct mtk_mmsys_kept *kept;
	unsigned int i, j, routed = 0;

	/* the first block is mmsys itself */
	if (WARN_ON(!path->num_blocks || path->blocks[0].offset))
		return -EINVAL;

	kept = devm_kcalloc(dev, path->num_blocks, sizeof(*kept), GFP_KERNEL);
	if (!kept)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(mmsys->boot_cg); i++)
		mmsys->boot_cg[i] = readl(mmsys->regs + path->cg_sta[i]);

	for (i = 0; i < path->num_blocks; i++) {
		const struct mtk_mmsys_keep_block *blk = &path->blocks[i];

		/* a block with a driver of its own is left to that driver */
		if (blk->owner_compat) {
			struct device_node *np;
			bool owned;

			np = of_find_compatible_node(NULL, NULL, blk->owner_compat);
			owned = np && of_device_is_available(np);
			of_node_put(np);
			if (owned) {
				dev_dbg(dev, "boot path: %s has a driver, not kept\n",
					blk->name);
				continue;
			}
		}

		/* a block left without its clock is not in the path */
		if (blk->cg_bit >= 0 && (mmsys->boot_cg[0] & BIT(blk->cg_bit))) {
			dev_dbg(dev, "boot path: %s is clock gated, not kept\n",
				blk->name);
			continue;
		}

		if (blk->offset) {
			kept[i].regs = devm_ioremap(dev, res->start + blk->offset,
						    SZ_4K);
			if (!kept[i].regs)
				return -ENOMEM;
		} else {
			kept[i].regs = mmsys->regs;
		}

		kept[i].vals = devm_kcalloc(dev, blk->num_regs,
					    sizeof(*kept[i].vals), GFP_KERNEL);
		if (!kept[i].vals)
			return -ENOMEM;

		for (j = 0; j < blk->num_regs; j++) {
			kept[i].vals[j] = readl(kept[i].regs + blk->regs[j]);
			if (!i && kept[i].vals[j])
				routed++;
		}
	}

	if (!routed) {
		dev_info(dev, "no display path left by the boot loader\n");
		return 0;
	}

	mmsys->kept = kept;
	mtk_mmsys_cfg_read(dev, mmsys);
	if (path->gce_base) {
		mmsys->gce = devm_ioremap(dev, path->gce_base, GCE_MAP_SIZE);
		mmsys->mminfra_cfg = devm_ioremap(dev, path->mminfra_cfg_base, SZ_4K);
	}
	dev_info(dev, "keeping the display path of the boot loader (clock gates %08x %08x)\n",
		 mmsys->boot_cg[0], mmsys->boot_cg[1]);

	return 0;
}

/* Number of mmsys registers and clock gates that differ from the boot state */
static unsigned int mtk_mmsys_boot_path_lost(struct device *dev,
					     struct mtk_mmsys *mmsys)
{
	const struct mtk_mmsys_boot_path *path = mmsys->data->boot_path;
	const struct mtk_mmsys_keep_block *blk = &path->blocks[0];
	const struct mtk_mmsys_kept *kept = &mmsys->kept[0];
	unsigned int i, lost = 0;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(mmsys->boot_cg); i++) {
		val = readl(mmsys->regs + path->cg_sta[i]);
		if (val & ~mmsys->boot_cg[i] & path->cg_keep[i]) {
			dev_dbg(dev, "boot path: clock gates %u: %08x, boot loader left %08x\n",
				i, val, mmsys->boot_cg[i]);
			lost++;
		}
	}

	for (i = 0; i < blk->num_regs; i++) {
		val = readl(kept->regs + blk->regs[i]);
		if (val != kept->vals[i]) {
			dev_dbg(dev, "boot path: %s+%#05x: %08x, boot loader left %08x\n",
				blk->name, blk->regs[i], val, kept->vals[i]);
			lost++;
		}
	}

	return lost;
}

static void mtk_mmsys_boot_path_restore(struct device *dev,
					struct mtk_mmsys *mmsys)
{
	const struct mtk_mmsys_boot_path *path = mmsys->data->boot_path;
	char lost_in[320], stuck_in[320];
	unsigned int i, j, len = 0, slen = 0, stuck0 = 0;
	u32 val;

	lost_in[0] = '\0';
	stuck_in[0] = '\0';

	/*
	 * The clocks first: an engine without its clock ignores writes. The
	 * gates come back from a power down at their reset value (gated),
	 * also those of clocks the clock framework still counts as enabled,
	 * so ungate everything the boot loader had left ungated.
	 */
	for (i = 0; i < ARRAY_SIZE(mmsys->boot_cg); i++) {
		u32 now = readl(mmsys->regs + path->cg_sta[i]);

		val = now & ~mmsys->boot_cg[i];
		if (val)
			writel(val, mmsys->regs + path->cg_clr[i]);
		dev_info(dev, "boot path: clock gates %u: %08x -> %08x (boot %08x)\n",
			 i, now, readl(mmsys->regs + path->cg_sta[i]),
			 mmsys->boot_cg[i]);
	}

	for (i = 0; i < path->num_blocks; i++) {
		const struct mtk_mmsys_keep_block *blk = &path->blocks[i];
		const struct mtk_mmsys_kept *kept = &mmsys->kept[i];

		unsigned int differ = 0, stuck = 0;
		u16 stuck_reg = 0;
		u32 stuck_val = 0, stuck_read = 0;

		if (!kept->vals)
			continue;

		for (j = 0; j < blk->num_regs; j++)
			if (readl(kept->regs + blk->regs[j]) != kept->vals[j])
				differ++;
		if (differ && len < sizeof(lost_in))
			len += scnprintf(lost_in + len, sizeof(lost_in) - len,
					 " %s:%u", blk->name, differ);

		for (j = 0; j < blk->num_regs; j++) {
			u16 reg = blk->regs[j];

			val = kept->vals[j];
			if (blk->skip_mask && reg == blk->skip_reg)
				val &= ~blk->skip_mask;
			/* the tables behind the engine are gone: pass pixels on */
			if (blk->relay_mask && reg == blk->relay_reg &&
			    (val & blk->relay_mask) != blk->relay_mask) {
				dev_info(dev, "boot path: %s set to relay, its tables are not kept\n",
					 blk->name);
				val |= blk->relay_mask;
			}
			writel(val, kept->regs + reg);
			if (readl(kept->regs + reg) != val) {
				dev_dbg(dev, "boot path: %s+%#05x reads %08x after writing %08x\n",
					blk->name, reg, readl(kept->regs + reg), val);
				if (!stuck++) {
					stuck_reg = reg;
					stuck_val = val;
					stuck_read = readl(kept->regs + reg);
				}
			}
		}
		if (!i)
			stuck0 = stuck;
		if (stuck && slen < sizeof(stuck_in))
			slen += scnprintf(stuck_in + slen, sizeof(stuck_in) - slen,
					  " %s:%u(+%#05x %08x->%08x)", blk->name,
					  stuck, stuck_reg, stuck_val, stuck_read);
	}

	/* the clock gates were ungated first, so all blocks were readable */
	dev_info(dev, "boot path: registers lost per block:%s\n",
		 len ? lost_in : " none");
	/* registers that read back otherwise right after the write */
	dev_info(dev, "boot path: writes that did not read back:%s\n",
		 slen ? stuck_in : " none");

	if (!stuck0)
		return;

	mtk_mmsys_cfg_compare(dev, mmsys);

	if (READ_ONCE(devapc_retry)) {
		struct arm_smccc_res res;

		arm_smccc_smc(MTK_MMSYS_SIP_CMDQ_CONTROL,
			      MTK_MMSYS_SIP_DEVAPC_DISABLE, 0, 0, 0, 0, 0, 0,
			      &res);
		stuck0 = mtk_mmsys_boot_path_rewrite0(mmsys);
		dev_info(dev, "boot path: after the DEVAPC disable call (a0 %#lx): %u mmsys registers still do not read back\n",
			 res.a0, stuck0);
	}

	if (stuck0 && READ_ONCE(disp_va)) {
		mtk_mmsys_disp_va(dev, mmsys);
		stuck0 = mtk_mmsys_boot_path_rewrite0(mmsys);
		dev_info(dev, "boot path: after the DISP_VA handshake: %u mmsys registers still do not read back\n",
			 stuck0);
		if (!stuck0)
			mtk_mmsys_cfg_compare(dev, mmsys);
	}
}

static void mtk_mmsys_boot_path_check(struct device *dev, struct mtk_mmsys *mmsys)
{
	unsigned int lost;

	lost = mtk_mmsys_boot_path_lost(dev, mmsys);
	if (!lost)
		return;

	if (!keep_boot_path) {
		dev_info(dev, "display path lost the boot loader state (%u registers), not written back\n",
			 lost);
		return;
	}

	mtk_mmsys_boot_path_restore(dev, mmsys);
	mmsys->boot_path_restores++;
	dev_info(dev, "display path lost the boot loader state (%u registers), written back\n",
		 lost);
}

static int mtk_mmsys_boot_path_show(struct seq_file *s, void *data)
{
	struct mtk_mmsys *mmsys = dev_get_drvdata(s->private);
	const struct mtk_mmsys_boot_path *path = mmsys->data->boot_path;
	unsigned int i, j;

	seq_printf(s, "written back: %u times, keep_boot_path=%d\n",
		   mmsys->boot_path_restores, keep_boot_path);

	for (i = 0; i < ARRAY_SIZE(mmsys->boot_cg); i++)
		seq_printf(s, "clock gates %u: boot %08x now %08x\n", i,
			   mmsys->boot_cg[i], readl(mmsys->regs + path->cg_sta[i]));

	for (i = 0; i < path->num_blocks; i++) {
		const struct mtk_mmsys_keep_block *blk = &path->blocks[i];
		const struct mtk_mmsys_kept *kept = &mmsys->kept[i];
		bool gated;

		if (!kept->vals) {
			seq_printf(s, "%s: not kept\n", blk->name);
			continue;
		}

		gated = mtk_mmsys_kept_gated(mmsys, blk);
		for (j = 0; j < blk->num_regs; j++) {
			u32 val;

			seq_printf(s, "%s+%#05x: boot %08x now ", blk->name,
				   blk->regs[j], kept->vals[j]);
			if (gated) {
				seq_puts(s, "(clock gated)\n");
				continue;
			}
			val = readl(kept->regs + blk->regs[j]);
			seq_printf(s, "%08x%s\n", val, val != kept->vals[j] ? " *" : "");
		}
	}

	return 0;
}

static void mtk_mmsys_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static void mtk_mmsys_boot_path_debugfs(struct device *dev)
{
	struct dentry *dir;

	dir = debugfs_create_dir(dev_name(dev), NULL);
	if (devm_add_action_or_reset(dev, mtk_mmsys_debugfs_remove, dir))
		return;
	debugfs_create_devm_seqfile(dev, "boot-path", dir, mtk_mmsys_boot_path_show);
}

void mtk_mmsys_ddp_connect(struct device *dev,
			   enum mtk_ddp_comp_id cur,
			   enum mtk_ddp_comp_id next)
{
	struct mtk_mmsys *mmsys = dev_get_drvdata(dev);
	const struct mtk_mmsys_routes *routes = mmsys->data->routes;
	int i;

	if (mmsys->kept)
		mtk_mmsys_boot_path_check(dev, mmsys);

	for (i = 0; i < mmsys->data->num_routes; i++)
		if (cur == routes[i].from_comp && next == routes[i].to_comp)
			mtk_mmsys_update_bits(mmsys, routes[i].addr, routes[i].mask,
					      routes[i].val, NULL);

	if (mmsys->data->vsync_len)
		mtk_mmsys_update_bits(mmsys, MT8188_VDO1_MIXER_VSYNC_LEN, GENMASK(31, 0),
				      mmsys->data->vsync_len, NULL);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_ddp_connect);

void mtk_mmsys_ddp_disconnect(struct device *dev,
			      enum mtk_ddp_comp_id cur,
			      enum mtk_ddp_comp_id next)
{
	struct mtk_mmsys *mmsys = dev_get_drvdata(dev);
	const struct mtk_mmsys_routes *routes = mmsys->data->routes;
	int i;

	for (i = 0; i < mmsys->data->num_routes; i++)
		if (cur == routes[i].from_comp && next == routes[i].to_comp)
			mtk_mmsys_update_bits(mmsys, routes[i].addr, routes[i].mask, 0, NULL);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_ddp_disconnect);

void mtk_mmsys_merge_async_config(struct device *dev, int idx, int width, int height,
				  struct cmdq_pkt *cmdq_pkt)
{
	mtk_mmsys_update_bits(dev_get_drvdata(dev), MT8195_VDO1_MERGE0_ASYNC_CFG_WD + 0x10 * idx,
			      ~0, height << 16 | width, cmdq_pkt);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_merge_async_config);

void mtk_mmsys_hdr_config(struct device *dev, int be_width, int be_height,
			  struct cmdq_pkt *cmdq_pkt)
{
	mtk_mmsys_update_bits(dev_get_drvdata(dev), MT8195_VDO1_HDRBE_ASYNC_CFG_WD, ~0,
			      be_height << 16 | be_width, cmdq_pkt);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_hdr_config);

void mtk_mmsys_mixer_in_config(struct device *dev, int idx, bool alpha_sel, u16 alpha,
			       u8 mode, u32 biwidth, struct cmdq_pkt *cmdq_pkt)
{
	struct mtk_mmsys *mmsys = dev_get_drvdata(dev);

	mtk_mmsys_update_bits(mmsys, MT8195_VDO1_MIXER_IN1_ALPHA + (idx - 1) * 4, ~0,
			      alpha << 16 | alpha, cmdq_pkt);
	mtk_mmsys_update_bits(mmsys, MT8195_VDO1_HDR_TOP_CFG, BIT(15 + idx), 0, cmdq_pkt);
	mtk_mmsys_update_bits(mmsys, MT8195_VDO1_HDR_TOP_CFG, BIT(19 + idx),
			      alpha_sel << (19 + idx), cmdq_pkt);
	mtk_mmsys_update_bits(mmsys, MT8195_VDO1_MIXER_IN1_PAD + (idx - 1) * 4,
			      GENMASK(31, 16) | GENMASK(1, 0), biwidth << 16 | mode, cmdq_pkt);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_mixer_in_config);

void mtk_mmsys_mixer_in_channel_swap(struct device *dev, int idx, bool channel_swap,
				     struct cmdq_pkt *cmdq_pkt)
{
	mtk_mmsys_update_bits(dev_get_drvdata(dev), MT8195_VDO1_MIXER_IN1_PAD + (idx - 1) * 4,
			      BIT(4), channel_swap << 4, cmdq_pkt);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_mixer_in_channel_swap);

void mtk_mmsys_ddp_dpi_fmt_config(struct device *dev, u32 val)
{
	struct mtk_mmsys *mmsys = dev_get_drvdata(dev);

	switch (val) {
	case MTK_DPI_RGB888_SDR_CON:
		mtk_mmsys_update_bits(mmsys, MT8186_MMSYS_DPI_OUTPUT_FORMAT,
				      MT8186_DPI_FORMAT_MASK, MT8186_DPI_RGB888_SDR_CON, NULL);
		break;
	case MTK_DPI_RGB565_SDR_CON:
		mtk_mmsys_update_bits(mmsys, MT8186_MMSYS_DPI_OUTPUT_FORMAT,
				      MT8186_DPI_FORMAT_MASK, MT8186_DPI_RGB565_SDR_CON, NULL);
		break;
	case MTK_DPI_RGB565_DDR_CON:
		mtk_mmsys_update_bits(mmsys, MT8186_MMSYS_DPI_OUTPUT_FORMAT,
				      MT8186_DPI_FORMAT_MASK, MT8186_DPI_RGB565_DDR_CON, NULL);
		break;
	case MTK_DPI_RGB888_DDR_CON:
	default:
		mtk_mmsys_update_bits(mmsys, MT8186_MMSYS_DPI_OUTPUT_FORMAT,
				      MT8186_DPI_FORMAT_MASK, MT8186_DPI_RGB888_DDR_CON, NULL);
		break;
	}
}
EXPORT_SYMBOL_GPL(mtk_mmsys_ddp_dpi_fmt_config);

void mtk_mmsys_vpp_rsz_merge_config(struct device *dev, u32 id, bool enable,
				    struct cmdq_pkt *cmdq_pkt)
{
	u32 reg;

	switch (id) {
	case 2:
		reg = MT8195_SVPP2_BUF_BF_RSZ_SWITCH;
		break;
	case 3:
		reg = MT8195_SVPP3_BUF_BF_RSZ_SWITCH;
		break;
	default:
		dev_err(dev, "Invalid id %d\n", id);
		return;
	}

	mtk_mmsys_update_bits(dev_get_drvdata(dev), reg, ~0, enable, cmdq_pkt);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_vpp_rsz_merge_config);

void mtk_mmsys_vpp_rsz_dcm_config(struct device *dev, bool enable,
				  struct cmdq_pkt *cmdq_pkt)
{
	u32 client;

	client = MT8195_SVPP1_MDP_RSZ;
	mtk_mmsys_update_bits(dev_get_drvdata(dev),
			      MT8195_VPP1_HW_DCM_1ST_DIS0, client,
			      ((enable) ? client : 0), cmdq_pkt);
	mtk_mmsys_update_bits(dev_get_drvdata(dev),
			      MT8195_VPP1_HW_DCM_2ND_DIS0, client,
			      ((enable) ? client : 0), cmdq_pkt);

	client = MT8195_SVPP2_MDP_RSZ | MT8195_SVPP3_MDP_RSZ;
	mtk_mmsys_update_bits(dev_get_drvdata(dev),
			      MT8195_VPP1_HW_DCM_1ST_DIS1, client,
			      ((enable) ? client : 0), cmdq_pkt);
	mtk_mmsys_update_bits(dev_get_drvdata(dev),
			      MT8195_VPP1_HW_DCM_2ND_DIS1, client,
			      ((enable) ? client : 0), cmdq_pkt);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_vpp_rsz_dcm_config);

static int mtk_mmsys_reset_update(struct reset_controller_dev *rcdev, unsigned long id,
				  bool assert)
{
	struct mtk_mmsys *mmsys = container_of(rcdev, struct mtk_mmsys, rcdev);
	unsigned long flags;
	u32 offset;
	u32 reg;

	if (mmsys->data->rst_tb) {
		if (id >= mmsys->data->num_resets) {
			dev_err(rcdev->dev, "Invalid reset ID: %lu (>=%u)\n",
				id, mmsys->data->num_resets);
			return -EINVAL;
		}
		id = mmsys->data->rst_tb[id];
	}

	offset = (id / MMSYS_SW_RESET_PER_REG) * sizeof(u32);
	id = id % MMSYS_SW_RESET_PER_REG;
	reg = mmsys->data->sw0_rst_offset + offset;

	spin_lock_irqsave(&mmsys->lock, flags);

	if (assert)
		mtk_mmsys_update_bits(mmsys, reg, BIT(id), 0, NULL);
	else
		mtk_mmsys_update_bits(mmsys, reg, BIT(id), BIT(id), NULL);

	spin_unlock_irqrestore(&mmsys->lock, flags);

	return 0;
}

static int mtk_mmsys_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	return mtk_mmsys_reset_update(rcdev, id, true);
}

static int mtk_mmsys_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	return mtk_mmsys_reset_update(rcdev, id, false);
}

static int mtk_mmsys_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	int ret;

	ret = mtk_mmsys_reset_assert(rcdev, id);
	if (ret)
		return ret;

	usleep_range(1000, 1100);

	return mtk_mmsys_reset_deassert(rcdev, id);
}

static const struct reset_control_ops mtk_mmsys_reset_ops = {
	.assert = mtk_mmsys_reset_assert,
	.deassert = mtk_mmsys_reset_deassert,
	.reset = mtk_mmsys_reset,
};

static int mtk_mmsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *clks;
	struct platform_device *drm;
	struct mtk_mmsys *mmsys;
	struct resource *res;
	int ret;

	mmsys = devm_kzalloc(dev, sizeof(*mmsys), GFP_KERNEL);
	if (!mmsys)
		return -ENOMEM;

	mmsys->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(mmsys->regs)) {
		ret = PTR_ERR(mmsys->regs);
		dev_err(dev, "Failed to ioremap mmsys registers: %d\n", ret);
		return ret;
	}

	mmsys->data = of_device_get_match_data(&pdev->dev);

	/* before the clock and DRM drivers come up and change anything */
	if (mmsys->data->boot_path) {
		ret = mtk_mmsys_boot_path_read(dev, mmsys, res);
		if (ret)
			return ret;
	}

	if (mmsys->data->num_resets > 0) {
		spin_lock_init(&mmsys->lock);

		mmsys->rcdev.owner = THIS_MODULE;
		mmsys->rcdev.nr_resets = mmsys->data->num_resets;
		mmsys->rcdev.ops = &mtk_mmsys_reset_ops;
		mmsys->rcdev.of_node = pdev->dev.of_node;
		ret = devm_reset_controller_register(&pdev->dev, &mmsys->rcdev);
		if (ret) {
			dev_err(&pdev->dev, "Couldn't register mmsys reset controller: %d\n", ret);
			return ret;
		}
	}

	/* CMDQ is optional */
	ret = cmdq_dev_get_client_reg(dev, &mmsys->cmdq_base, 0);
	if (ret)
		dev_dbg(dev, "No mediatek,gce-client-reg!\n");

	platform_set_drvdata(pdev, mmsys);

	if (mmsys->kept)
		mtk_mmsys_boot_path_debugfs(dev);

	clks = platform_device_register_data(&pdev->dev, mmsys->data->clk_driver,
					     PLATFORM_DEVID_AUTO, NULL, 0);
	if (IS_ERR(clks))
		return PTR_ERR(clks);
	mmsys->clks_pdev = clks;

	if (mmsys->data->is_vppsys)
		goto out_probe_done;

	drm = platform_device_register_data(&pdev->dev, "mediatek-drm",
					    PLATFORM_DEVID_AUTO, NULL, 0);
	if (IS_ERR(drm)) {
		platform_device_unregister(clks);
		return PTR_ERR(drm);
	}
	mmsys->drm_pdev = drm;

out_probe_done:
	return 0;
}

static void mtk_mmsys_remove(struct platform_device *pdev)
{
	struct mtk_mmsys *mmsys = platform_get_drvdata(pdev);

	platform_device_unregister(mmsys->drm_pdev);
	platform_device_unregister(mmsys->clks_pdev);
}

static const struct of_device_id of_match_mtk_mmsys[] = {
	{ .compatible = "mediatek,mt2701-mmsys", .data = &mt2701_mmsys_driver_data },
	{ .compatible = "mediatek,mt2712-mmsys", .data = &mt2712_mmsys_driver_data },
	{ .compatible = "mediatek,mt6779-mmsys", .data = &mt6779_mmsys_driver_data },
	{ .compatible = "mediatek,mt6795-mmsys", .data = &mt6795_mmsys_driver_data },
	{ .compatible = "mediatek,mt6797-mmsys", .data = &mt6797_mmsys_driver_data },
	{ .compatible = "mediatek,mt6878-mmsys", .data = &mt6878_mmsys_driver_data },
	{ .compatible = "mediatek,mt8167-mmsys", .data = &mt8167_mmsys_driver_data },
	{ .compatible = "mediatek,mt8173-mmsys", .data = &mt8173_mmsys_driver_data },
	{ .compatible = "mediatek,mt8183-mmsys", .data = &mt8183_mmsys_driver_data },
	{ .compatible = "mediatek,mt8186-mmsys", .data = &mt8186_mmsys_driver_data },
	{ .compatible = "mediatek,mt8188-vdosys0", .data = &mt8188_vdosys0_driver_data },
	{ .compatible = "mediatek,mt8188-vdosys1", .data = &mt8188_vdosys1_driver_data },
	{ .compatible = "mediatek,mt8188-vppsys0", .data = &mt8188_vppsys0_driver_data },
	{ .compatible = "mediatek,mt8188-vppsys1", .data = &mt8188_vppsys1_driver_data },
	{ .compatible = "mediatek,mt8192-mmsys", .data = &mt8192_mmsys_driver_data },
	/* "mediatek,mt8195-mmsys" compatible is deprecated */
	{ .compatible = "mediatek,mt8195-mmsys", .data = &mt8195_vdosys0_driver_data },
	{ .compatible = "mediatek,mt8195-vdosys0", .data = &mt8195_vdosys0_driver_data },
	{ .compatible = "mediatek,mt8195-vdosys1", .data = &mt8195_vdosys1_driver_data },
	{ .compatible = "mediatek,mt8195-vppsys0", .data = &mt8195_vppsys0_driver_data },
	{ .compatible = "mediatek,mt8195-vppsys1", .data = &mt8195_vppsys1_driver_data },
	{ .compatible = "mediatek,mt8365-mmsys", .data = &mt8365_mmsys_driver_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_mtk_mmsys);

/* GCE-D apsrc request off for the sleep, see sleep_gce_apsrc_off */
static void mtk_mmsys_gce_apsrc_off(struct device *dev, struct mtk_mmsys *mmsys)
{
	u32 cg0, cg1, val;

	mmsys->gce_gctl_cleared = false;
	if (!READ_ONCE(sleep_gce_apsrc_off) || !mmsys->gce || !mmsys->mminfra_cfg)
		return;

	/* GCE registers only with its clocks running; we do not switch them */
	cg0 = readl(mmsys->mminfra_cfg + MMINFRA_CG_CON0);
	cg1 = readl(mmsys->mminfra_cfg + MMINFRA_CG_CON1);
	if ((cg0 & MMINFRA_CG0_GCE) || (cg1 & MMINFRA_CG1_GCE26M)) {
		dev_info(dev, "GCE-D clocks gated (mminfra cg %08x %08x): apsrc request left alone\n",
			 cg0, cg1);
		return;
	}

	val = readl(mmsys->gce + GCE_GCTL_VALUE);
	if (!val)
		return;
	writel(0, mmsys->gce + GCE_GCTL_VALUE);
	mmsys->gce_gctl_saved = val;
	mmsys->gce_gctl_cleared = true;
	dev_info(dev, "GCE-D apsrc request off for the sleep (gctl value %08x -> %08x)\n",
		 val, readl(mmsys->gce + GCE_GCTL_VALUE));
}

/* the mmsys DRAM requests for the sleep, see sleep_sodi and sleep_cg_off */
static void mtk_mmsys_sleep_reqs(struct device *dev, struct mtk_mmsys *mmsys)
{
	const struct mtk_mmsys_boot_path *path = mmsys->data->boot_path;
	int sodi = READ_ONCE(sleep_sodi);
	unsigned int i;

	mmsys->sodi_set = false;
	memset(mmsys->cg_gated, 0, sizeof(mmsys->cg_gated));
	dev_info(dev, "sleep: misc %08x, sodi req mask %08x, emi req ctl %08x, clock gates %08x %08x\n",
		 readl(mmsys->regs + MMSYS_MISC),
		 readl(mmsys->regs + MMSYS_SODI_REQ_MASK),
		 readl(mmsys->regs + MMSYS_EMI_REQ_CTL),
		 readl(mmsys->regs + path->cg_sta[0]),
		 readl(mmsys->regs + path->cg_sta[1]));

	if (sodi >= 0) {
		mmsys->sodi_saved = readl(mmsys->regs + MMSYS_SODI_REQ_MASK);
		writel(sodi, mmsys->regs + MMSYS_SODI_REQ_MASK);
		mmsys->sodi_set = true;
		dev_info(dev, "sleep: sodi req mask %08x -> %08x\n",
			 mmsys->sodi_saved,
			 readl(mmsys->regs + MMSYS_SODI_REQ_MASK));
	}

	if (READ_ONCE(sleep_cg_off)) {
		for (i = 0; i < ARRAY_SIZE(mmsys->cg_gated); i++) {
			/* the open ones: 0 bits; SET is CLR - 4, as CON + 4 */
			u32 open = ~readl(mmsys->regs + path->cg_sta[i]);

			writel(open, mmsys->regs + path->cg_clr[i] - 4);
			mmsys->cg_gated[i] = open;
		}
		dev_info(dev, "sleep: clock gates closed, now %08x %08x\n",
			 readl(mmsys->regs + path->cg_sta[0]),
			 readl(mmsys->regs + path->cg_sta[1]));
	}
}

static int mtk_mmsys_suspend(struct device *dev)
{
	struct mtk_mmsys *mmsys = dev_get_drvdata(dev);

	if (mmsys->kept && READ_ONCE(keep_disp_on)) {
		device_set_awake_path(dev);
		dev_info(dev, "keeping the display power domain on for the sleep\n");
		mtk_mmsys_gce_apsrc_off(dev, mmsys);
		mtk_mmsys_sleep_reqs(dev, mmsys);
	}
	return 0;
}

static int mtk_mmsys_resume(struct device *dev)
{
	struct mtk_mmsys *mmsys = dev_get_drvdata(dev);
	const struct mtk_mmsys_boot_path *path = mmsys->data->boot_path;
	unsigned int i;

	/* the clocks first: the domain stayed on, its registers kept */
	for (i = 0; path && i < ARRAY_SIZE(mmsys->cg_gated); i++) {
		if (mmsys->cg_gated[i])
			writel(mmsys->cg_gated[i], mmsys->regs + path->cg_clr[i]);
		mmsys->cg_gated[i] = 0;
	}
	if (mmsys->sodi_set) {
		writel(mmsys->sodi_saved, mmsys->regs + MMSYS_SODI_REQ_MASK);
		mmsys->sodi_set = false;
	}

	if (!mmsys->gce_gctl_cleared)
		return 0;
	mmsys->gce_gctl_cleared = false;

	/* the domain should have stayed on, the GCE with its clocks and state */
	if ((readl(mmsys->mminfra_cfg + MMINFRA_CG_CON0) & MMINFRA_CG0_GCE) ||
	    (readl(mmsys->mminfra_cfg + MMINFRA_CG_CON1) & MMINFRA_CG1_GCE26M)) {
		dev_warn(dev, "GCE-D clocks gated after the sleep: apsrc request not restored\n");
		return 0;
	}
	writel(mmsys->gce_gctl_saved, mmsys->gce + GCE_GCTL_VALUE);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mtk_mmsys_pm_ops, mtk_mmsys_suspend,
				mtk_mmsys_resume);

static struct platform_driver mtk_mmsys_drv = {
	.driver = {
		.name = "mtk-mmsys",
		.of_match_table = of_match_mtk_mmsys,
		.pm = pm_sleep_ptr(&mtk_mmsys_pm_ops),
	},
	.probe = mtk_mmsys_probe,
	.remove = mtk_mmsys_remove,
};
module_platform_driver(mtk_mmsys_drv);

MODULE_AUTHOR("Yongqiang Niu <yongqiang.niu@mediatek.com>");
MODULE_DESCRIPTION("MediaTek SoC MMSYS driver");
MODULE_LICENSE("GPL");
