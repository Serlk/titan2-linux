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

#include "clk-mtk.h"
#include "clk-pll.h"

#define MFGPLL_CON0		0x008
#define MFGPLL_CON1		0x00c
#define MFGSCPLL_CON0		0x008
#define MFGSCPLL_CON1		0x00c

#define MT6878_PLL_FMAX		(3800UL * MHZ)
#define MT6878_PLL_FMIN		(1500UL * MHZ)
#define MT6878_INTEGER_BITS	8

#define PLL(_id, _name, _reg, _en_reg, _en_mask, _pll_en_bit,	\
	    _flags, _rst_bar_mask,				\
	    _pd_reg, _pd_shift, _tuner_reg,			\
	    _tuner_en_reg, _tuner_en_bit,			\
	    _pcw_reg, _pcw_shift, _pcwbits) {			\
		.id = _id,					\
		.name = _name,					\
		.reg = _reg,					\
		.en_reg = _en_reg,				\
		.en_mask = _en_mask,				\
		.pll_en_bit = _pll_en_bit,			\
		.flags = _flags,				\
		.rst_bar_mask = _rst_bar_mask,			\
		.fmax = MT6878_PLL_FMAX,			\
		.fmin = MT6878_PLL_FMIN,			\
		.pd_reg = _pd_reg,				\
		.pd_shift = _pd_shift,				\
		.tuner_reg = _tuner_reg,			\
		.tuner_en_reg = _tuner_en_reg,			\
		.tuner_en_bit = _tuner_en_bit,			\
		.pcw_reg = _pcw_reg,				\
		.pcw_shift = _pcw_shift,			\
		.pcwbits = _pcwbits,				\
		.pcwibits = MT6878_INTEGER_BITS,		\
	}

static const struct mtk_pll_data mfg_ao_plls[] = {
	PLL(CLK_MFG_AO_MFGPLL, "mfg-ao-mfgpll", MFGPLL_CON0, MFGPLL_CON0,
	    0, 0, 0, BIT(0), MFGPLL_CON1, 24, 0, 0, 0, MFGPLL_CON1, 0, 22),
};

static const struct mtk_pll_data mfgsc_ao_plls[] = {
	PLL(CLK_MFGSC_AO_MFGSCPLL, "mfgsc-ao-mfgscpll", MFGSCPLL_CON0,
	    MFGSCPLL_CON0, 0, 0, 0, BIT(0), MFGSCPLL_CON1, 24, 0, 0, 0,
	    MFGSCPLL_CON1, 0, 22),
};

static const struct of_device_id of_match_clk_mt6878_mfg[] = {
	{ .compatible = "mediatek,mt6878-mfgpll-pll-ctrl",
	  .data = &mfg_ao_plls },
	{ .compatible = "mediatek,mt6878-mfgscpll-pll-ctrl",
	  .data = &mfgsc_ao_plls },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6878_mfg);

static int clk_mt6878_mfg_probe(struct platform_device *pdev)
{
	const struct mtk_pll_data *plls;
	struct clk_hw_onecell_data *clk_data;
	struct device_node *node = pdev->dev.of_node;
	const int num_plls = 1;
	int r;

	plls = of_device_get_match_data(&pdev->dev);
	if (!plls)
		return -EINVAL;

	clk_data = mtk_alloc_clk_data(num_plls);
	if (!clk_data)
		return -ENOMEM;

	r = mtk_clk_register_plls(&pdev->dev, plls, num_plls, clk_data);
	if (r)
		goto free_clk_data;

	r = of_clk_add_hw_provider(node, of_clk_hw_onecell_get, clk_data);
	if (r)
		goto unregister_plls;

	platform_set_drvdata(pdev, clk_data);

	return r;

unregister_plls:
	mtk_clk_unregister_plls(plls, num_plls, clk_data);
free_clk_data:
	mtk_free_clk_data(clk_data);

	return r;
}

static void clk_mt6878_mfg_remove(struct platform_device *pdev)
{
	const struct mtk_pll_data *plls = of_device_get_match_data(&pdev->dev);
	struct clk_hw_onecell_data *clk_data = platform_get_drvdata(pdev);
	struct device_node *node = pdev->dev.of_node;

	of_clk_del_provider(node);
	mtk_clk_unregister_plls(plls, 1, clk_data);
	mtk_free_clk_data(clk_data);
}

static struct platform_driver clk_mt6878_mfg_drv = {
	.probe = clk_mt6878_mfg_probe,
	.remove = clk_mt6878_mfg_remove,
	.driver = {
		.name = "clk-mt6878-mfg",
		.of_match_table = of_match_clk_mt6878_mfg,
	},
};
module_platform_driver(clk_mt6878_mfg_drv);

MODULE_DESCRIPTION("MediaTek MT6878 GPU PLL clocks driver");
MODULE_LICENSE("GPL");
