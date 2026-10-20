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

static const struct mtk_gate_regs mdp_cg_regs = {
	.set_ofs = 0x104,
	.clr_ofs = 0x108,
	.sta_ofs = 0x100,
};

#define GATE_MDP(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &mdp_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate mdp_clks[] = {
	GATE_MDP(CLK_MDP_MUTEX0, "mdp_mutex0",
			"disp0_ck"/* parent */, 0),
	GATE_MDP(CLK_MDP_APB_BUS, "mdp_apb_bus",
			"disp0_ck"/* parent */, 1),
	GATE_MDP(CLK_MDP_SMI0, "mdp_smi0",
			"disp0_ck"/* parent */, 2),
	GATE_MDP(CLK_MDP_RDMA0, "mdp_rdma0",
			"disp0_ck"/* parent */, 3),
	GATE_MDP(CLK_MDP_HDR0, "mdp_hdr0",
			"disp0_ck"/* parent */, 5),
	GATE_MDP(CLK_MDP_AAL0, "mdp_aal0",
			"disp0_ck"/* parent */, 6),
	GATE_MDP(CLK_MDP_RSZ0, "mdp_rsz0",
			"disp0_ck"/* parent */, 7),
	GATE_MDP(CLK_MDP_TDSHP0, "mdp_tdshp0",
			"disp0_ck"/* parent */, 8),
	GATE_MDP(CLK_MDP_WROT0, "mdp_wrot0",
			"disp0_ck"/* parent */, 10),
	GATE_MDP(CLK_MDP_RDMA1, "mdp_rdma1",
			"disp0_ck"/* parent */, 15),
	GATE_MDP(CLK_MDP_RSZ1, "mdp_rsz1",
			"disp0_ck"/* parent */, 19),
	GATE_MDP(CLK_MDP_WROT1, "mdp_wrot1",
			"disp0_ck"/* parent */, 22),
};

static const struct mtk_clk_desc mdp_mcd = {
	.clks = mdp_clks,
	.num_clks = ARRAY_SIZE(mdp_clks),
	.need_runtime_pm = true,
};

static const struct of_device_id of_match_clk_mt6878_mdpsys[] = {
	{ .compatible = "mediatek,mt6878-mdpsys", .data = &mdp_mcd },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6878_mdpsys);

static struct platform_driver clk_mt6878_mdpsys_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6878-mdpsys",
		.of_match_table = of_match_clk_mt6878_mdpsys,
	},
};
module_platform_driver(clk_mt6878_mdpsys_drv);

MODULE_DESCRIPTION("MediaTek MT6878 mdpsys clocks driver");
MODULE_LICENSE("GPL");
