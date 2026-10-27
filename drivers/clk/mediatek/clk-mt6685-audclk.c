// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6685 audio clock buffer
 *
 * The MT6685 clock buffer chip feeds the 26 MHz reference of the audio
 * codec of the second PMIC (MT6369 on MT6878 phones) from its external
 * buffer BBCK5. Unlike the RF/BB buffers, which follow the SRCLKEN
 * signals, the audio buffer is switched by software: mode "register
 * controlled" and the manual enable bit. The codec has no other clock for
 * its digital part and the MTKAIF link to the AFE.
 *
 * Register layout from the MediaTek downstream driver
 * (drivers/mfd/mt6685-audclk.c, Copyright (c) 2021 MediaTek Inc., author
 * Ting-Fang Hou).
 *
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define MT6685_DCXO_EXTBUF5_CW0		0x79e
/* 0: controlled by XO_BBCK5_EN_M, 1: EN_BB, 2: CLK_SEL */
#define XO_BBCK5_MODE_MASK		GENMASK(1, 0)
#define XO_BBCK5_MODE_REG		0
#define XO_BBCK5_EN_M			BIT(2)

/* the downstream driver waits this long after switching the buffer on */
#define XO_BBCK5_SETTLE_US		400

struct mt6685_audclk {
	struct clk_hw hw;
	struct regmap *regmap;
};

static struct mt6685_audclk *to_audclk(struct clk_hw *hw)
{
	return container_of(hw, struct mt6685_audclk, hw);
}

static int mt6685_audclk_prepare(struct clk_hw *hw)
{
	struct mt6685_audclk *ac = to_audclk(hw);
	int ret;

	ret = regmap_update_bits(ac->regmap, MT6685_DCXO_EXTBUF5_CW0,
				 XO_BBCK5_MODE_MASK | XO_BBCK5_EN_M,
				 XO_BBCK5_MODE_REG | XO_BBCK5_EN_M);
	if (ret)
		return ret;

	usleep_range(XO_BBCK5_SETTLE_US, XO_BBCK5_SETTLE_US + 20);

	return 0;
}

static void mt6685_audclk_unprepare(struct clk_hw *hw)
{
	struct mt6685_audclk *ac = to_audclk(hw);

	regmap_update_bits(ac->regmap, MT6685_DCXO_EXTBUF5_CW0,
			   XO_BBCK5_EN_M, 0);
}

static int mt6685_audclk_is_prepared(struct clk_hw *hw)
{
	struct mt6685_audclk *ac = to_audclk(hw);
	unsigned int val;

	if (regmap_read(ac->regmap, MT6685_DCXO_EXTBUF5_CW0, &val))
		return 0;

	return (val & XO_BBCK5_MODE_MASK) == XO_BBCK5_MODE_REG &&
	       (val & XO_BBCK5_EN_M);
}

static const struct clk_ops mt6685_audclk_ops = {
	.prepare = mt6685_audclk_prepare,
	.unprepare = mt6685_audclk_unprepare,
	.is_prepared = mt6685_audclk_is_prepared,
};

static int mt6685_audclk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_init_data init = {
		.name = "mt6685_xo_bbck5",
		.ops = &mt6685_audclk_ops,
		.flags = CLK_IGNORE_UNUSED,
	};
	struct mt6685_audclk *ac;
	int ret;

	ac = devm_kzalloc(dev, sizeof(*ac), GFP_KERNEL);
	if (!ac)
		return -ENOMEM;

	ac->regmap = dev_get_regmap(dev->parent, NULL);
	if (!ac->regmap)
		return dev_err_probe(dev, -ENODEV, "no MT6685 regmap\n");

	of_property_read_string(dev->of_node, "clock-output-names", &init.name);
	ac->hw.init = &init;

	ret = devm_clk_hw_register(dev, &ac->hw);
	if (ret)
		return ret;

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &ac->hw);
}

static const struct of_device_id mt6685_audclk_of_match[] = {
	{ .compatible = "mediatek,mt6685-audclk" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6685_audclk_of_match);

static struct platform_driver mt6685_audclk_driver = {
	.probe = mt6685_audclk_probe,
	.driver = {
		.name = "mt6685-audclk",
		.of_match_table = mt6685_audclk_of_match,
	},
};
module_platform_driver(mt6685_audclk_driver);

MODULE_DESCRIPTION("MediaTek MT6685 audio clock buffer (BBCK5)");
MODULE_LICENSE("GPL");
