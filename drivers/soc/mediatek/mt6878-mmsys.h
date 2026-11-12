/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * MT6878 display path left by the boot loader, kept across power downs of
 * the DISP domain.
 *
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 *
 * Register offsets as in the MediaTek GPL display driver for MT6878
 * (mtk_drm_ddp.c MT6878_* crossbar registers, mtk_disp_{ovl,rsz,tdshp,c3d,
 * color,ccorr,aal,gamma,postmask,dither}.c, mtk-smi.c) and the engine
 * addresses of its mt6878.dts.
 */

#ifndef __SOC_MEDIATEK_MT6878_MMSYS_H
#define __SOC_MEDIATEK_MT6878_MMSYS_H

#include "mtk-mmsys.h"

#define MT6878_MMSYS_CG_CON0			0x100
#define MT6878_MMSYS_CG_CLR0			0x108
#define MT6878_MMSYS_CG_CON1			0x110
#define MT6878_MMSYS_CG_CLR1			0x118

/* CG_CON0 bits, as in clk-mt6878-mm */
#define MT6878_CG0_OVL1_2L			1
#define MT6878_CG0_OVL2_2L			2
#define MT6878_CG0_RSZ0				6
#define MT6878_CG0_TDSHP0			7
#define MT6878_CG0_C3D0				8
#define MT6878_CG0_COLOR0			9
#define MT6878_CG0_CCORR0			10
#define MT6878_CG0_CCORR1			11
#define MT6878_CG0_AAL0				12
#define MT6878_CG0_GAMMA0			13
#define MT6878_CG0_POSTMASK0			14
#define MT6878_CG0_DITHER0			15
#define MT6878_CG0_APB_BUS			27
/* the "apb" clock of SMI larb0 in the MediaTek device tree */
#define MT6878_CG0_SMI_COMMON			31
/* CG_CON1 */
#define MT6878_CG1_26M				11

/*
 * Clocks no driver owns: the engines between OVL0_2L and DSI0 that the DRM
 * driver does not drive, the bus clocks the MediaTek driver keeps on as
 * "top clocks" (APB_BUS, 26M besides MUTEX0) and the larb0 clock. OVL0_2L,
 * DSI0 and MUTEX0 are left to their drivers.
 */
#define MT6878_CG0_KEEP	(BIT(MT6878_CG0_OVL1_2L) | BIT(MT6878_CG0_OVL2_2L) | \
			 BIT(MT6878_CG0_RSZ0) | BIT(MT6878_CG0_TDSHP0) | \
			 BIT(MT6878_CG0_C3D0) | BIT(MT6878_CG0_COLOR0) | \
			 BIT(MT6878_CG0_CCORR0) | BIT(MT6878_CG0_CCORR1) | \
			 BIT(MT6878_CG0_AAL0) | BIT(MT6878_CG0_GAMMA0) | \
			 BIT(MT6878_CG0_POSTMASK0) | BIT(MT6878_CG0_DITHER0) | \
			 BIT(MT6878_CG0_APB_BUS) | BIT(MT6878_CG0_SMI_COMMON))
#define MT6878_CG1_KEEP	BIT(MT6878_CG1_26M)

/*
 * mmsys: the shadow bypasses first (BYPASS_MUX_SHADOW, then the crossbar
 * bypass bits of CROSSBAR_CON, the order of mtk_crtc_ddp_hw_init() and
 * mtk_ddp_add_comp_to_path() downstream), then every crossbar output enable and
 * every single in/out select of the main display (OVL and PQ crossbars,
 * split and comp crossbars, PQ engine selects).
 */
static const u16 mt6878_keep_mmsys[] = {
	0xc00,					/* BYPASS_MUX_SHADOW */
	0xc0c,					/* MMSYS_CROSSBAR_CON */
	0xc08,					/* MMSYS_OVL_CON */
	0xc10, 0xc14, 0xc18, 0xc1c,		/* OVL_BG_CROSSBAR0..3 */
	0xc34, 0xc38, 0xc3c, 0xc40,		/* OVL_BLEND_CROSSBAR0..3 */
	0xc54, 0xc58,				/* OVL_PQ_IN_CROSSBAR0..1 */
	0xc7c, 0xc80, 0xc84, 0xc88,		/* OVL_PQ_OUT_CROSSBAR0..3 */
	0xd00, 0xd04, 0xd08,			/* PQ_IN_CROSSBAR0..2 */
	0xd20, 0xd24, 0xd28, 0xd2c,		/* PQ_OUT_CROSSBAR0..6 */
	0xd30, 0xd34, 0xd38,
	0xd4c, 0xd50, 0xd54, 0xd58, 0xd5c,	/* SPLIT_OUT_CROSSBAR0..4 */
	0xd60, 0xd64, 0xd68, 0xd6c, 0xd70,	/* SPLIT_OUT_CROSSBAR0..4_SEL_IN */
	0xd74, 0xd78, 0xd7c, 0xd80, 0xd84,	/* COMP_OUT_CROSSBAR0..4 */
	0xe00, 0xe04, 0xe08, 0xe0c,		/* AAL0/C3D0/CCORR0... SOUT/SEL_IN */
	0xe10, 0xe14, 0xe18, 0xe1c,
	0xe20, 0xe24, 0xe28, 0xe2c,
	0xe30, 0xe34, 0xe38, 0xe3c,
	0xe40, 0xe48,
};

/*
 * OVL1_2L and OVL2_2L only pass the background on: OVL_EN (BYPASS_SHADOW
 * is bit 22 of it), ROI size, DATAPATH_CON (BGCLR_IN_SEL), background
 * colour and SRC_CON without its layer enables.
 */
static const u16 mt6878_keep_ovl_2l[] = {
	0x00c, 0x020, 0x024, 0x028, 0x02c,
};

/* SHADOW_CTRL, CONTROL_1/2, in/out size, steps and offsets, ENABLE */
static const u16 mt6878_keep_rsz[] = {
	0x0f0, 0x004, 0x008, 0x010, 0x014, 0x018, 0x01c,
	0x020, 0x024, 0x028, 0x02c, 0x000,
};

/* SHADOW_CTRL, CFG, input size, output offset/size, blank width, CTRL */
static const u16 mt6878_keep_tdshp[] = {
	0x724, 0x110, 0x120, 0x124, 0x128, 0x12c, 0x100,
};

/* SHADOW_CTL, CFG, SIZE, EN */
static const u16 mt6878_keep_c3d[] = {
	0x030, 0x004, 0x024, 0x000,
};

/* SHADOW_CTRL, CFG_MAIN, OUT_SEL, WIDTH, HEIGHT, CM1_EN, CM2_EN, START */
static const u16 mt6878_keep_color[] = {
	0xcb0, 0x400, 0xc0c, 0xc50, 0xc54, 0xc60, 0xca0, 0xc00,
};

/* SHADOW, CFG, SIZE, EN */
static const u16 mt6878_keep_ccorr[] = {
	0x0a0, 0x020, 0x030, 0x000,
};

/* SHADOW_CTRL, CFG, SIZE, CFG_MAIN, OUTPUT_SIZE, OUTPUT_OFFSET, EN */
static const u16 mt6878_keep_aal[] = {
	0x0f0, 0x020, 0x030, 0x200, 0x4d8, 0x4dc, 0x000,
};

/* CFG, SIZE, EN */
static const u16 mt6878_keep_gamma[] = {
	0x020, 0x030, 0x000,
};

/* SHADOW_CTRL, CFG, SIZE, EN */
static const u16 mt6878_keep_postmask[] = {
	0x024, 0x020, 0x030, 0x000,
};

/* DITHER_0 (BYPASS_SHADOW), CFG, SIZE, EN */
static const u16 mt6878_keep_dither[] = {
	0x100, 0x020, 0x030, 0x000,
};

/*
 * SMI larb0 (OVL ports): CMD_THRT_CON, SW_FLAG, DISABLE_ULTRA, the
 * outstanding limits and the non-secure (MMU enable) control of its seven
 * ports, which the MediaTek SMI driver programs on every larb power up.
 */
static const u16 mt6878_keep_larb0[] = {
	0x024, 0x040, 0x070,
	0x200, 0x204, 0x208, 0x20c, 0x210, 0x214, 0x218,
	0x380, 0x384, 0x388, 0x38c, 0x390, 0x394, 0x398,
};

/*
 * SMI common and the two display sub-commons between it and the memory
 * (0x1e801000, 0x1e807000, 0x1e808000), in the MM_INFRA domain, the parent
 * of DISP: L1LEN, the L1 arbiters, the larb to MMU0/1 bus select, the
 * thresholds, DCM and the dummy setting of the MediaTek SMI driver. Its
 * mt6878 smi-common node is optional; without it nothing programs them
 * again after MM_INFRA went down with DISP. Offsets from the mmsys base.
 */
#define MT6878_SMI_COMMON_OFF			(0x1e801000 - 0x14000000)
#define MT6878_SMI_DISP_SSC0_OFF		(0x1e807000 - 0x14000000)
#define MT6878_SMI_DISP_SSC1_OFF		(0x1e808000 - 0x14000000)

static const u16 mt6878_keep_smi_common[] = {
	0x100,					/* L1LEN */
	0x104, 0x108, 0x10c, 0x110,		/* L1ARB0..7 */
	0x114, 0x118, 0x11c, 0x120,
	0x220,					/* BUS_SEL */
	0x234, 0x238, 0x23c,			/* M4U_TH, FIFO_TH1/2 */
	0x300,					/* DCM */
	0x444,					/* DUMMY */
};

static const u16 mt6878_keep_smi_ssc[] = {
	0x100,					/* L1LEN */
	0x244,					/* PREULTRA_MASK1 */
	0x444,					/* DUMMY */
};

#define MT6878_KEEP(_name, _off, _cg, _regs)		\
	.name = _name, .offset = _off, .cg_bit = _cg,	\
	.regs = _regs, .num_regs = ARRAY_SIZE(_regs)

static const struct mtk_mmsys_keep_block mt6878_keep_blocks[] = {
	{ MT6878_KEEP("mmsys", 0x00000, -1, mt6878_keep_mmsys) },
	{ MT6878_KEEP("smi_common", MT6878_SMI_COMMON_OFF, -1, mt6878_keep_smi_common),
	  .owner_compat = "mediatek,mt6878-smi-common" },
	{ MT6878_KEEP("smi_disp_ssc0", MT6878_SMI_DISP_SSC0_OFF, -1, mt6878_keep_smi_ssc) },
	{ MT6878_KEEP("smi_disp_ssc1", MT6878_SMI_DISP_SSC1_OFF, -1, mt6878_keep_smi_ssc) },
	{ MT6878_KEEP("larb0", 0x1e000, MT6878_CG0_SMI_COMMON, mt6878_keep_larb0) },
	{ MT6878_KEEP("ovl1_2l", 0x03000, MT6878_CG0_OVL1_2L, mt6878_keep_ovl_2l),
	  .skip_reg = 0x02c, .skip_mask = GENMASK(3, 0) },
	{ MT6878_KEEP("ovl2_2l", 0x04000, MT6878_CG0_OVL2_2L, mt6878_keep_ovl_2l),
	  .skip_reg = 0x02c, .skip_mask = GENMASK(3, 0) },
	{ MT6878_KEEP("rsz0", 0x08000, MT6878_CG0_RSZ0, mt6878_keep_rsz),
	  .skip_reg = 0x000, .skip_mask = BIT(16) },
	{ MT6878_KEEP("tdshp0", 0x09000, MT6878_CG0_TDSHP0, mt6878_keep_tdshp),
	  .relay_reg = 0x110, .relay_mask = BIT(0) },
	{ MT6878_KEEP("c3d0", 0x0a000, MT6878_CG0_C3D0, mt6878_keep_c3d),
	  .relay_reg = 0x004, .relay_mask = BIT(0) },
	{ MT6878_KEEP("color0", 0x0b000, MT6878_CG0_COLOR0, mt6878_keep_color),
	  .relay_reg = 0x400, .relay_mask = BIT(7) },
	{ MT6878_KEEP("ccorr0", 0x0c000, MT6878_CG0_CCORR0, mt6878_keep_ccorr),
	  .relay_reg = 0x020, .relay_mask = BIT(0) },
	{ MT6878_KEEP("ccorr1", 0x0d000, MT6878_CG0_CCORR1, mt6878_keep_ccorr),
	  .relay_reg = 0x020, .relay_mask = BIT(0) },
	{ MT6878_KEEP("aal0", 0x0e000, MT6878_CG0_AAL0, mt6878_keep_aal),
	  .relay_reg = 0x020, .relay_mask = BIT(0) },
	{ MT6878_KEEP("gamma0", 0x0f000, MT6878_CG0_GAMMA0, mt6878_keep_gamma),
	  .relay_reg = 0x020, .relay_mask = BIT(0) },
	{ MT6878_KEEP("postmask0", 0x10000, MT6878_CG0_POSTMASK0, mt6878_keep_postmask),
	  .relay_reg = 0x020, .relay_mask = BIT(0) },
	{ MT6878_KEEP("dither0", 0x11000, MT6878_CG0_DITHER0, mt6878_keep_dither),
	  .relay_reg = 0x020, .relay_mask = BIT(0) },
};

/*
 * The whole mmsys config window, read only while the DISP domain is on (at
 * probe and when the path is written back); the MediaTek driver itself
 * dumps 0x000..0x90f, 0xb40, 0xbc0 and the crossbars.
 */
static const struct mtk_mmsys_reg_range mt6878_cfg_ranges[] = {
	{ 0x000, 0x1000 },
};

/*
 * CMDQ_ENABLE_DISP_VA handshake of the MediaTek kernel (mtk_drm_crtc.c,
 * mtk_crtc_prepare_instr(), before the path is connected at CRTC enable):
 * the secure firmware runs its own instructions on the display GCE, which
 * waits for the DISP_VA_START token and sets DISP_VA_END when done
 * (tokens of dt-bindings/gce/mt6878-gce.h, GCE 0x1e980000 of its DT).
 */
#define MT6878_GCE_BASE				0x1e980000
#define MT6878_MMINFRA_CFG_BASE			0x1e800000
#define MT6878_GCE_TOKEN_DISP_VA_START		692
#define MT6878_GCE_TOKEN_DISP_VA_END		693

/*
 * "default-tokens" of the factory GCE-D node: set to 1 by the MediaTek GCE
 * driver at every GCE power up (cmdq_init_cpu()); among them the prebuilt
 * lock tokens of mtk-cmdq-ext.h (682 MDP, 685 MML, 688 VFMT, 691 DISP).
 */
static const u16 mt6878_gce_tokens[] = {
	0x2bc, 0x2bd, 0x2be, 0x2bf, 0x2c0, 0x2c6, 0x2c7,
	0x2aa, 0x2ad, 0x2b0, 0x2b3,
};

static const struct mtk_mmsys_boot_path mt6878_boot_path = {
	.cg_sta = { MT6878_MMSYS_CG_CON0, MT6878_MMSYS_CG_CON1 },
	.cg_clr = { MT6878_MMSYS_CG_CLR0, MT6878_MMSYS_CG_CLR1 },
	.cg_keep = { MT6878_CG0_KEEP, MT6878_CG1_KEEP },
	.blocks = mt6878_keep_blocks,
	.num_blocks = ARRAY_SIZE(mt6878_keep_blocks),
	.cfg_ranges = mt6878_cfg_ranges,
	.num_cfg_ranges = ARRAY_SIZE(mt6878_cfg_ranges),
	.gce_base = MT6878_GCE_BASE,
	.mminfra_cfg_base = MT6878_MMINFRA_CFG_BASE,
	.disp_va_start = MT6878_GCE_TOKEN_DISP_VA_START,
	.disp_va_end = MT6878_GCE_TOKEN_DISP_VA_END,
	.gce_tokens = mt6878_gce_tokens,
	.num_gce_tokens = ARRAY_SIZE(mt6878_gce_tokens),
};

#endif /* __SOC_MEDIATEK_MT6878_MMSYS_H */
