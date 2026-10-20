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

static const struct mtk_gate_regs scp_cg_regs = {
	.set_ofs = 0x154,
	.clr_ofs = 0x158,
	.sta_ofs = 0x154,
};

#define GATE_SCP(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &scp_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr_inv,	\
	}

static const struct mtk_gate scp_clks[] = {
	GATE_SCP(CLK_SCP_SET_SPI0, "scp_set_spi0",
			"f26m_ck"/* parent */, 0),
	GATE_SCP(CLK_SCP_SET_SPI1, "scp_set_spi1",
			"f26m_ck"/* parent */, 1),
	GATE_SCP(CLK_SCP_SET_SPI2, "scp_set_spi2",
			"f26m_ck"/* parent */, 2),
	GATE_SCP(CLK_SCP_SET_SPI3, "scp_set_spi3",
			"f26m_ck"/* parent */, 3),
};

static const struct mtk_clk_desc scp_mcd = {
	.clks = scp_clks,
	.num_clks = ARRAY_SIZE(scp_clks),
};

static const struct of_device_id of_match_clk_mt6878_scp[] = {
	{ .compatible = "mediatek,mt6878-scp-cfg", .data = &scp_mcd },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6878_scp);

static struct platform_driver clk_mt6878_scp_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6878-scp",
		.of_match_table = of_match_clk_mt6878_scp,
	},
};
module_platform_driver(clk_mt6878_scp_drv);

MODULE_DESCRIPTION("MediaTek MT6878 SCP configuration clocks driver");
MODULE_LICENSE("GPL");
