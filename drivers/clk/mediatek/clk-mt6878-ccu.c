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

static const struct mtk_gate_regs ccu_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define GATE_CCU(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &ccu_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate ccu_clks[] = {
	GATE_CCU(CLK_CCU_LARB30_CON, "ccu_larb30_con",
			"ccusys_ck"/* parent */, 0),
	GATE_CCU(CLK_CCU_AHB_CON, "ccu_ahb_con",
			"ccu_ahb_ck"/* parent */, 1),
	GATE_CCU(CLK_CCUSYS_CCU0_CON, "ccusys_ccu0_con",
			"ccusys_ck"/* parent */, 2),
	GATE_CCU(CLK_CCU2MM0_GALS_CON, "ccu2mm0_GCON",
			"ccusys_ck"/* parent */, 4),
};

static const struct mtk_clk_desc ccu_mcd = {
	.clks = ccu_clks,
	.num_clks = ARRAY_SIZE(ccu_clks),
	.need_runtime_pm = true,
};

static const struct of_device_id of_match_clk_mt6878_ccu[] = {
	{ .compatible = "mediatek,mt6878-ccusys", .data = &ccu_mcd },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6878_ccu);

static struct platform_driver clk_mt6878_ccu_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6878-ccu",
		.of_match_table = of_match_clk_mt6878_ccu,
	},
};
module_platform_driver(clk_mt6878_ccu_drv);

MODULE_DESCRIPTION("MediaTek MT6878 ccusys clocks driver");
MODULE_LICENSE("GPL");
