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

static const struct mtk_gate_regs mminfra_config0_cg_regs = {
	.set_ofs = 0x104,
	.clr_ofs = 0x108,
	.sta_ofs = 0x100,
};

static const struct mtk_gate_regs mminfra_config0_hwv_regs = {
	.set_ofs = 0x0000,
	.clr_ofs = 0x0004,
	.sta_ofs = 0x1C00,
};

static const struct mtk_gate_regs mminfra_config1_cg_regs = {
	.set_ofs = 0x114,
	.clr_ofs = 0x118,
	.sta_ofs = 0x110,
};

static const struct mtk_gate_regs mminfra_config1_hwv_regs = {
	.set_ofs = 0x0008,
	.clr_ofs = 0x000C,
	.sta_ofs = 0x1C04,
};

#define GATE_HWV_MMINFRA_CONFIG0(_id, _name, _parent, _shift) {	\
		.id = _id,						\
		.name = _name,						\
		.parent_name = _parent,					\
		.regs = &mminfra_config0_cg_regs,			\
		.hwv_regs = &mminfra_config0_hwv_regs,		\
		.shift = _shift,					\
		.ops = &mtk_clk_gate_hwv_ops_setclr,		\
	}

#define GATE_HWV_MMINFRA_CONFIG1(_id, _name, _parent, _shift) {	\
		.id = _id,						\
		.name = _name,						\
		.parent_name = _parent,					\
		.regs = &mminfra_config1_cg_regs,			\
		.hwv_regs = &mminfra_config1_hwv_regs,		\
		.shift = _shift,					\
		.ops = &mtk_clk_gate_hwv_ops_setclr,		\
	}

static const struct mtk_gate mminfra_config_clks[] = {
	/* MMINFRA_CONFIG0 */
	GATE_HWV_MMINFRA_CONFIG0(CLK_MMINFRA_GCE_D, "mminfra_gce_d",
			"mminfra_ck"/* parent */, 0),
	GATE_HWV_MMINFRA_CONFIG0(CLK_MMINFRA_GCE_M, "mminfra_gce_m",
			"mminfra_ck"/* parent */, 1),
	/* MMINFRA_CONFIG1 */
	GATE_HWV_MMINFRA_CONFIG1(CLK_MMINFRA_GCE_26M, "mminfra_gce_26m",
			"mminfra_ck"/* parent */, 17),
};

static const struct mtk_clk_desc mminfra_config_mcd = {
	.clks = mminfra_config_clks,
	.num_clks = ARRAY_SIZE(mminfra_config_clks),
	.need_runtime_pm = true,
};

static const struct of_device_id of_match_clk_mt6878_mminfra[] = {
	{ .compatible = "mediatek,mt6878-mminfra-config", .data = &mminfra_config_mcd },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6878_mminfra);

static struct platform_driver clk_mt6878_mminfra_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6878-mminfra",
		.of_match_table = of_match_clk_mt6878_mminfra,
	},
};
module_platform_driver(clk_mt6878_mminfra_drv);

MODULE_DESCRIPTION("MediaTek MT6878 mminfra_config clocks driver");
MODULE_LICENSE("GPL");
