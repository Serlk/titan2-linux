// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 MediaTek Inc.
 *		      Chuan-wen Chen <chuan-wen.chen@mediatek.com>
 */

#include <dt-bindings/clock/mediatek,mt6878-clock.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs dip_nr1_dip1_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define GATE_DIP_NR1_DIP1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &dip_nr1_dip1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate dip_nr1_dip1_clks[] = {
	GATE_DIP_NR1_DIP1(CLK_DIP_NR1_DIP1_LARB, "dip_nr1_dip1_larb",
			"img1_ck"/* parent */, 0),
	GATE_DIP_NR1_DIP1(CLK_DIP_NR1_DIP1_DIP_NR1, "dip_nr1_dip1_dip_nr1",
			"img1_ck"/* parent */, 1),
};

static const struct mtk_clk_desc dip_nr1_dip1_mcd = {
	.clks = dip_nr1_dip1_clks,
	.num_clks = ARRAY_SIZE(dip_nr1_dip1_clks),
	.need_runtime_pm = true,
};

static const struct mtk_gate_regs dip_nr2_dip1_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define GATE_DIP_NR2_DIP1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &dip_nr2_dip1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate dip_nr2_dip1_clks[] = {
	GATE_DIP_NR2_DIP1(CLK_DIP_NR2_DIP1_LARB15, "dip_nr2_dip1_larb15",
			"img1_ck"/* parent */, 0),
	GATE_DIP_NR2_DIP1(CLK_DIP_NR2_DIP1_DIP_NR, "dip_nr2_dip1_dip_nr",
			"img1_ck"/* parent */, 1),
};

static const struct mtk_clk_desc dip_nr2_dip1_mcd = {
	.clks = dip_nr2_dip1_clks,
	.num_clks = ARRAY_SIZE(dip_nr2_dip1_clks),
	.need_runtime_pm = true,
};

static const struct mtk_gate_regs dip_top_dip1_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define GATE_DIP_TOP_DIP1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &dip_top_dip1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate dip_top_dip1_clks[] = {
	GATE_DIP_TOP_DIP1(CLK_DIP_TOP_DIP1_DIP_TOP, "dip_dip1_dip_top",
			"img1_ck"/* parent */, 0),
	GATE_DIP_TOP_DIP1(CLK_DIP_TOP_DIP1_DIP_TOP_GALS0, "dip_dip1_dip_gals0",
			"img1_ck"/* parent */, 1),
	GATE_DIP_TOP_DIP1(CLK_DIP_TOP_DIP1_DIP_TOP_GALS1, "dip_dip1_dip_gals1",
			"img1_ck"/* parent */, 2),
	GATE_DIP_TOP_DIP1(CLK_DIP_TOP_DIP1_DIP_TOP_GALS2, "dip_dip1_dip_gals2",
			"img1_ck"/* parent */, 3),
	GATE_DIP_TOP_DIP1(CLK_DIP_TOP_DIP1_DIP_TOP_GALS3, "dip_dip1_dip_gals3",
			"img1_ck"/* parent */, 4),
	GATE_DIP_TOP_DIP1(CLK_DIP_TOP_DIP1_LARB10, "dip_dip1_larb10",
			"img1_ck"/* parent */, 5),
	GATE_DIP_TOP_DIP1(CLK_DIP_TOP_DIP1_LARB15, "dip_dip1_larb15",
			"img1_ck"/* parent */, 6),
	GATE_DIP_TOP_DIP1(CLK_DIP_TOP_DIP1_LARB38, "dip_dip1_larb38",
			"img1_ck"/* parent */, 7),
	GATE_DIP_TOP_DIP1(CLK_DIP_TOP_DIP1_LARB39, "dip_dip1_larb39",
			"img1_ck"/* parent */, 8),
};

static const struct mtk_clk_desc dip_top_dip1_mcd = {
	.clks = dip_top_dip1_clks,
	.num_clks = ARRAY_SIZE(dip_top_dip1_clks),
	.need_runtime_pm = true,
};

static const struct mtk_gate_regs img0_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs img1_cg_regs = {
	.set_ofs = 0x54,
	.clr_ofs = 0x58,
	.sta_ofs = 0x50,
};

#define GATE_IMG0(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &img0_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

#define GATE_IMG1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &img1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate img_clks[] = {
	/* IMG0 */
	GATE_IMG0(CLK_IMG_LARB9, "img_larb9",
			"img1_ck"/* parent */, 0),
	GATE_IMG0(CLK_IMG_TRAW0, "img_traw0",
			"img1_ck"/* parent */, 1),
	GATE_IMG0(CLK_IMG_TRAW1, "img_traw1",
			"img1_ck"/* parent */, 2),
	GATE_IMG0(CLK_IMG_DIP0, "img_dip0",
			"img1_ck"/* parent */, 3),
	GATE_IMG0(CLK_IMG_WPE0, "img_wpe0",
			"img1_ck"/* parent */, 4),
	GATE_IMG0(CLK_IMG_IPE, "img_ipe",
			"ipe_ck"/* parent */, 5),
	GATE_IMG0(CLK_IMG_WPE1, "img_wpe1",
			"img1_ck"/* parent */, 6),
	GATE_IMG0(CLK_IMG_WPE2, "img_wpe2",
			"img1_ck"/* parent */, 7),
	GATE_IMG0(CLK_IMG_SUB_COMMON0, "img_sub_common0",
			"img1_ck"/* parent */, 16),
	GATE_IMG0(CLK_IMG_SUB_COMMON1, "img_sub_common1",
			"img1_ck"/* parent */, 17),
	GATE_IMG0(CLK_IMG_SUB_COMMON3, "img_sub_common3",
			"img1_ck"/* parent */, 19),
	GATE_IMG0(CLK_IMG_SUB_COMMON4, "img_sub_common4",
			"img1_ck"/* parent */, 20),
	GATE_IMG0(CLK_IMG_GALS_RX_DIP0, "img_gals_rx_dip0",
			"img1_ck"/* parent */, 21),
	GATE_IMG0(CLK_IMG_GALS_RX_DIP1, "img_gals_rx_dip1",
			"img1_ck"/* parent */, 22),
	GATE_IMG0(CLK_IMG_GALS_RX_TRAW0, "img_gals_rx_traw0",
			"img1_ck"/* parent */, 23),
	GATE_IMG0(CLK_IMG_GALS_RX_WPE0, "img_gals_rx_wpe0",
			"img1_ck"/* parent */, 24),
	GATE_IMG0(CLK_IMG_GALS_RX_WPE1, "img_gals_rx_wpe1",
			"img1_ck"/* parent */, 25),
	GATE_IMG0(CLK_IMG_GALS_RX_IPE0, "img_gals_rx_ipe0",
			"img1_ck"/* parent */, 27),
	GATE_IMG0(CLK_IMG_GALS_TX_IPE0, "img_gals_tx_ipe0",
			"img1_ck"/* parent */, 28),
	GATE_IMG0(CLK_IMG_GALS, "img_gals",
			"img1_ck"/* parent */, 31),
	/* IMG1 */
	GATE_IMG1(CLK_IMG_FDVT, "img_fdvt",
			"ipe_ck"/* parent */, 0),
	GATE_IMG1(CLK_IMG_ME, "img_me",
			"ipe_ck"/* parent */, 1),
	GATE_IMG1(CLK_IMG_MMG, "img_mmg",
			"ipe_ck"/* parent */, 2),
	GATE_IMG1(CLK_IMG_LARB12, "img_larb12",
			"ipe_ck"/* parent */, 3),
};

static const struct mtk_clk_desc img_mcd = {
	.clks = img_clks,
	.num_clks = ARRAY_SIZE(img_clks),
	.need_runtime_pm = true,
};

static const struct mtk_gate_regs img_v_cg_regs = {
	.set_ofs = 0x0,
	.clr_ofs = 0x0,
	.sta_ofs = 0x0,
};

#define GATE_IMG_V(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &img_v_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_no_setclr,	\
	}

static const struct mtk_gate img_v_clks[] = {
	GATE_IMG_V(CLK_IMG_VCORE_GALS_DISP, "img_vcore_gals_disp",
			"mminfra_ck"/* parent */, 0),
	GATE_IMG_V(CLK_IMG_VCORE_MAIN, "img_vcore_main",
			"mminfra_ck"/* parent */, 1),
	GATE_IMG_V(CLK_IMG_VCORE_SUB0, "img_vcore_sub0",
			"mminfra_ck"/* parent */, 2),
	GATE_IMG_V(CLK_IMG_VCORE_SUB1, "img_vcore_sub1",
			"mminfra_ck"/* parent */, 3),
};

static const struct mtk_clk_desc img_v_mcd = {
	.clks = img_v_clks,
	.num_clks = ARRAY_SIZE(img_v_clks),
	.need_runtime_pm = true,
};

static const struct mtk_gate_regs traw_dip1_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define GATE_TRAW_DIP1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &traw_dip1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate traw_dip1_clks[] = {
	GATE_TRAW_DIP1(CLK_TRAW_DIP1_LARB28, "traw_dip1_larb28",
			"img1_ck"/* parent */, 0),
	GATE_TRAW_DIP1(CLK_TRAW_DIP1_LARB40, "traw_dip1_larb40",
			"img1_ck"/* parent */, 1),
	GATE_TRAW_DIP1(CLK_TRAW_DIP1_TRAW, "traw_dip1_traw",
			"img1_ck"/* parent */, 2),
	GATE_TRAW_DIP1(CLK_TRAW_DIP1_GALS, "traw_dip1_gals",
			"img1_ck"/* parent */, 3),
};

static const struct mtk_clk_desc traw_dip1_mcd = {
	.clks = traw_dip1_clks,
	.num_clks = ARRAY_SIZE(traw_dip1_clks),
	.need_runtime_pm = true,
};

static const struct mtk_gate_regs wpe1_dip1_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define GATE_WPE1_DIP1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &wpe1_dip1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate wpe1_dip1_clks[] = {
	GATE_WPE1_DIP1(CLK_WPE1_DIP1_LARB11, "wpe1_dip1_larb11",
			"img1_ck"/* parent */, 0),
	GATE_WPE1_DIP1(CLK_WPE1_DIP1_WPE, "wpe1_dip1_wpe",
			"img1_ck"/* parent */, 1),
	GATE_WPE1_DIP1(CLK_WPE1_DIP1_GALS0, "wpe1_dip1_gals0",
			"img1_ck"/* parent */, 2),
};

static const struct mtk_clk_desc wpe1_dip1_mcd = {
	.clks = wpe1_dip1_clks,
	.num_clks = ARRAY_SIZE(wpe1_dip1_clks),
	.need_runtime_pm = true,
};

static const struct mtk_gate_regs wpe2_dip1_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define GATE_WPE2_DIP1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &wpe2_dip1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate wpe2_dip1_clks[] = {
	GATE_WPE2_DIP1(CLK_WPE2_DIP1_LARB11, "wpe2_dip1_larb11",
			"img1_ck"/* parent */, 0),
	GATE_WPE2_DIP1(CLK_WPE2_DIP1_WPE, "wpe2_dip1_wpe",
			"img1_ck"/* parent */, 1),
	GATE_WPE2_DIP1(CLK_WPE2_DIP1_GALS0, "wpe2_dip1_gals0",
			"img1_ck"/* parent */, 2),
};

static const struct mtk_clk_desc wpe2_dip1_mcd = {
	.clks = wpe2_dip1_clks,
	.num_clks = ARRAY_SIZE(wpe2_dip1_clks),
	.need_runtime_pm = true,
};

static const struct of_device_id of_match_clk_mt6878_img[] = {
	{ .compatible = "mediatek,mt6878-imgsys1-dip-nr1", .data = &dip_nr1_dip1_mcd },
	{ .compatible = "mediatek,mt6878-imgsys1-dip-nr2", .data = &dip_nr2_dip1_mcd },
	{ .compatible = "mediatek,mt6878-imgsys1-dip-top", .data = &dip_top_dip1_mcd },
	{ .compatible = "mediatek,mt6878-imgsys", .data = &img_mcd },
	{ .compatible = "mediatek,mt6878-imgsys-vcore", .data = &img_v_mcd },
	{ .compatible = "mediatek,mt6878-imgsys-traw", .data = &traw_dip1_mcd },
	{ .compatible = "mediatek,mt6878-imgsys-wpe1", .data = &wpe1_dip1_mcd },
	{ .compatible = "mediatek,mt6878-imgsys-wpe2", .data = &wpe2_dip1_mcd },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6878_img);

static struct platform_driver clk_mt6878_img_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6878-img",
		.of_match_table = of_match_clk_mt6878_img,
	},
};
module_platform_driver(clk_mt6878_img_drv);

MODULE_DESCRIPTION("MediaTek MT6878 imgsys clocks driver");
MODULE_LICENSE("GPL");
