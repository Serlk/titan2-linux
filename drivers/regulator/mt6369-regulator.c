// SPDX-License-Identifier: GPL-2.0
//
// MediaTek MT6369 SPMI PMIC regulators
//
// Register layout and voltage tables from the MediaTek downstream driver
// for this PMIC.

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/mt6369-regulator.h>
#include <linux/regulator/of_regulator.h>

#define MT6369_REGULATOR_MODE_NORMAL	0
#define MT6369_REGULATOR_MODE_FCCM	1
#define MT6369_REGULATOR_MODE_LP	2

/* The buck enable registers come with SET and CLR companions */
#define MT6369_EN_SET_OFFSET		0x1
#define MT6369_EN_CLR_OFFSET		0x2

#define MT6369_BUCK_TOP_UNLOCK_VALUE	0x5543

enum {
	MT6369_ID_VBUCK1,
	MT6369_ID_VPA,
	MT6369_ID_VSRAM_CORE,
	MT6369_ID_VDIGRF,
	MT6369_ID_VAUX18,
	MT6369_ID_VUSB,
	MT6369_ID_VIBR,
	MT6369_ID_VIO28,
	MT6369_ID_VFP,
	MT6369_ID_VTP,
	MT6369_ID_VMCH,
	MT6369_ID_VMC,
	MT6369_ID_VCN33_1,
	MT6369_ID_VCN33_2,
	MT6369_ID_VAUD28,
	MT6369_ID_VANT18,
	MT6369_ID_VEFUSE,
	MT6369_ID_MAX,
};

/**
 * struct mt6369_regulator_info - MT6369 regulator information
 * @desc: Regulator description
 * @lp_mode_reg: Low Power mode register
 * @lp_mode_mask: Low Power mode bit
 * @modeset_reg: Forced PWM (FCCM) mode register, bucks only
 * @modeset_mask: Forced PWM (FCCM) mode bit
 */
struct mt6369_regulator_info {
	struct regulator_desc desc;
	u16 lp_mode_reg;
	u8 lp_mode_mask;
	u16 modeset_reg;
	u8 modeset_mask;
};

#define MT6369_BUCK(match, vreg, min, max, step, mset_reg, mset_bit)	\
[MT6369_ID_##vreg] = {							\
	.desc = {							\
		.name = match,						\
		.of_match = of_match_ptr(match),			\
		.ops = &mt6369_buck_ops,				\
		.type = REGULATOR_VOLTAGE,				\
		.id = MT6369_ID_##vreg,					\
		.owner = THIS_MODULE,					\
		.n_voltages = ((max) - (min)) / (step) + 1,		\
		.min_uV = min,						\
		.uV_step = step,					\
		.enable_reg = MT6369_RG_BUCK_##vreg##_EN_ADDR,		\
		.enable_mask = BIT(MT6369_RG_BUCK_##vreg##_EN_BIT),	\
		.vsel_reg = MT6369_RG_BUCK_##vreg##_VOSEL_ADDR,		\
		.vsel_mask = MT6369_RG_BUCK_##vreg##_VOSEL_MASK,	\
		.of_map_mode = mt6369_map_mode,				\
	},								\
	.lp_mode_reg = MT6369_RG_BUCK_##vreg##_LP_ADDR,			\
	.lp_mode_mask = BIT(MT6369_RG_BUCK_##vreg##_LP_BIT),		\
	.modeset_reg = mset_reg,					\
	.modeset_mask = BIT(mset_bit),					\
}

#define MT6369_LDO_LINEAR(match, vreg, min, max, step)			\
[MT6369_ID_##vreg] = {							\
	.desc = {							\
		.name = match,						\
		.of_match = of_match_ptr(match),			\
		.ops = &mt6369_ldo_linear_ops,				\
		.type = REGULATOR_VOLTAGE,				\
		.id = MT6369_ID_##vreg,					\
		.owner = THIS_MODULE,					\
		.n_voltages = ((max) - (min)) / (step) + 1,		\
		.min_uV = min,						\
		.uV_step = step,					\
		.enable_reg = MT6369_RG_LDO_##vreg##_ADDR,		\
		.enable_mask = BIT(MT6369_RG_LDO_##vreg##_EN_BIT),	\
		.vsel_reg = MT6369_RG_LDO_##vreg##_VOSEL_ADDR,		\
		.vsel_mask = MT6369_RG_LDO_##vreg##_VOSEL_MASK,		\
		.of_map_mode = mt6369_map_mode,				\
	},								\
	.lp_mode_reg = MT6369_RG_LDO_##vreg##_ADDR,			\
	.lp_mode_mask = BIT(MT6369_RG_LDO_##vreg##_LP_BIT),		\
}

/*
 * The table LDOs select a nominal voltage with VOSEL; the calibration
 * field next to it (VOCAL) is left alone, as the downstream driver does.
 */
#define MT6369_LDO_TABLE(match, vreg, table)				\
[MT6369_ID_##vreg] = {							\
	.desc = {							\
		.name = match,						\
		.of_match = of_match_ptr(match),			\
		.ops = &mt6369_ldo_table_ops,				\
		.type = REGULATOR_VOLTAGE,				\
		.id = MT6369_ID_##vreg,					\
		.owner = THIS_MODULE,					\
		.n_voltages = ARRAY_SIZE(table),			\
		.volt_table = table,					\
		.enable_reg = MT6369_RG_LDO_##vreg##_ADDR,		\
		.enable_mask = BIT(MT6369_RG_LDO_##vreg##_EN_BIT),	\
		.vsel_reg = MT6369_RG_##vreg##_VOSEL_ADDR,		\
		.vsel_mask = MT6369_RG_LDO_VOSEL_MASK,			\
		.of_map_mode = mt6369_map_mode,				\
	},								\
	.lp_mode_reg = MT6369_RG_LDO_##vreg##_ADDR,			\
	.lp_mode_mask = BIT(MT6369_RG_LDO_##vreg##_LP_BIT),		\
}

static const unsigned int ldo_volt_table1[] = {
	1800000, 1900000, 2000000, 2100000, 2200000, 2300000, 2400000, 2500000,
	2600000, 2700000, 2800000, 2900000, 3000000, 3100000, 3200000, 3300000,
};

static const unsigned int ldo_volt_table2[] = {
	1200000, 1300000, 1500000, 1700000, 1800000, 2000000, 2500000, 2600000,
	2700000, 2800000, 2900000, 3000000, 3100000, 3300000, 3400000, 3500000,
};

static const unsigned int ldo_volt_table3[] = {
	600000, 700000, 800000, 900000, 1000000, 1100000, 1200000, 1300000,
	1400000, 1500000, 1600000, 1700000, 1800000, 1900000, 2000000, 2100000,
};

static inline unsigned int mt6369_map_mode(unsigned int mode)
{
	switch (mode) {
	case MT6369_REGULATOR_MODE_NORMAL:
		return REGULATOR_MODE_NORMAL;
	case MT6369_REGULATOR_MODE_FCCM:
		return REGULATOR_MODE_FAST;
	case MT6369_REGULATOR_MODE_LP:
		return REGULATOR_MODE_IDLE;
	default:
		return REGULATOR_MODE_INVALID;
	}
}

static int mt6369_buck_enable(struct regulator_dev *rdev)
{
	return regmap_write(rdev->regmap, rdev->desc->enable_reg + MT6369_EN_SET_OFFSET,
			    rdev->desc->enable_mask);
}

static int mt6369_buck_disable(struct regulator_dev *rdev)
{
	return regmap_write(rdev->regmap, rdev->desc->enable_reg + MT6369_EN_CLR_OFFSET,
			    rdev->desc->enable_mask);
}

static unsigned int mt6369_regulator_get_mode(struct regulator_dev *rdev)
{
	struct mt6369_regulator_info *info = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	if (info->modeset_reg) {
		ret = regmap_read(rdev->regmap, info->modeset_reg, &val);
		if (ret)
			return REGULATOR_MODE_INVALID;

		if (val & info->modeset_mask)
			return REGULATOR_MODE_FAST;
	}

	ret = regmap_read(rdev->regmap, info->lp_mode_reg, &val);
	if (ret)
		return REGULATOR_MODE_INVALID;

	return (val & info->lp_mode_mask) ? REGULATOR_MODE_IDLE : REGULATOR_MODE_NORMAL;
}

static int mt6369_buck_unlock(struct regmap *map, bool unlock)
{
	u8 buf[2];

	buf[0] = unlock ? MT6369_BUCK_TOP_UNLOCK_VALUE & 0xff : 0;
	buf[1] = unlock ? MT6369_BUCK_TOP_UNLOCK_VALUE >> 8 : 0;

	return regmap_bulk_write(map, MT6369_BUCK_TOP_KEY_PROT_LO, buf, sizeof(buf));
}

static int mt6369_set_fccm(struct regulator_dev *rdev, bool fccm)
{
	struct mt6369_regulator_info *info = rdev_get_drvdata(rdev);
	int ret, ret_lock;

	ret = mt6369_buck_unlock(rdev->regmap, true);
	if (ret)
		return ret;

	ret = regmap_update_bits(rdev->regmap, info->modeset_reg, info->modeset_mask,
				 fccm ? info->modeset_mask : 0);

	ret_lock = mt6369_buck_unlock(rdev->regmap, false);

	return ret ? ret : ret_lock;
}

static int mt6369_regulator_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct mt6369_regulator_info *info = rdev_get_drvdata(rdev);
	unsigned int cur_mode;
	int ret;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		if (!info->modeset_reg)
			return -EINVAL;

		return mt6369_set_fccm(rdev, true);
	case REGULATOR_MODE_NORMAL:
		cur_mode = mt6369_regulator_get_mode(rdev);
		if (cur_mode == REGULATOR_MODE_FAST)
			return mt6369_set_fccm(rdev, false);

		if (cur_mode == REGULATOR_MODE_IDLE) {
			ret = regmap_clear_bits(rdev->regmap, info->lp_mode_reg,
						info->lp_mode_mask);
			if (ret)
				return ret;

			/* Let the regulator settle in normal mode */
			udelay(100);
		}

		return 0;
	case REGULATOR_MODE_IDLE:
		return regmap_set_bits(rdev->regmap, info->lp_mode_reg, info->lp_mode_mask);
	default:
		return -EINVAL;
	}
}

static const struct regulator_ops mt6369_buck_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.enable = mt6369_buck_enable,
	.disable = mt6369_buck_disable,
	.is_enabled = regulator_is_enabled_regmap,
	.set_mode = mt6369_regulator_set_mode,
	.get_mode = mt6369_regulator_get_mode,
};

static const struct regulator_ops mt6369_ldo_linear_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_mode = mt6369_regulator_set_mode,
	.get_mode = mt6369_regulator_get_mode,
};

static const struct regulator_ops mt6369_ldo_table_ops = {
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_iterate,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_mode = mt6369_regulator_set_mode,
	.get_mode = mt6369_regulator_get_mode,
};

static const struct mt6369_regulator_info mt6369_regulators[] = {
	MT6369_BUCK("vbuck1", VBUCK1, 0, 1193750, 6250,
		    MT6369_RG_VBUCK1_FCCM_ADDR, MT6369_RG_VBUCK1_FCCM_BIT),
	MT6369_BUCK("vpa", VPA, 500000, 3600000, 50000,
		    MT6369_RG_VPA_MODESET_ADDR, MT6369_RG_VPA_MODESET_BIT),
	MT6369_LDO_LINEAR("vsram-core", VSRAM_CORE, 400000, 1193750, 6250),
	MT6369_LDO_LINEAR("vdigrf", VDIGRF, 400000, 1193750, 6250),
	MT6369_LDO_TABLE("vaux18", VAUX18, ldo_volt_table1),
	MT6369_LDO_TABLE("vusb", VUSB, ldo_volt_table2),
	MT6369_LDO_TABLE("vibr", VIBR, ldo_volt_table2),
	MT6369_LDO_TABLE("vio28", VIO28, ldo_volt_table2),
	MT6369_LDO_TABLE("vfp", VFP, ldo_volt_table2),
	MT6369_LDO_TABLE("vtp", VTP, ldo_volt_table2),
	MT6369_LDO_TABLE("vmch", VMCH, ldo_volt_table2),
	MT6369_LDO_TABLE("vmc", VMC, ldo_volt_table2),
	MT6369_LDO_TABLE("vcn33-1", VCN33_1, ldo_volt_table2),
	MT6369_LDO_TABLE("vcn33-2", VCN33_2, ldo_volt_table2),
	MT6369_LDO_TABLE("vaud28", VAUD28, ldo_volt_table1),
	MT6369_LDO_TABLE("vant18", VANT18, ldo_volt_table3),
	MT6369_LDO_TABLE("vefuse", VEFUSE, ldo_volt_table2),
};

static int mt6369_regulator_probe(struct platform_device *pdev)
{
	struct regulator_config config = {};
	struct device *dev = &pdev->dev;
	struct regulator_dev *rdev;
	unsigned int i;

	config.dev = dev;
	config.regmap = dev_get_regmap(dev->parent, NULL);
	if (!config.regmap)
		return dev_err_probe(dev, -ENODEV, "Cannot get parent regmap\n");

	for (i = 0; i < ARRAY_SIZE(mt6369_regulators); i++) {
		const struct mt6369_regulator_info *info = &mt6369_regulators[i];

		config.driver_data = (void *)info;

		rdev = devm_regulator_register(dev, &info->desc, &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev), "Cannot register %s\n",
					     info->desc.name);
	}

	return 0;
}

static const struct of_device_id mt6369_regulator_match[] = {
	{ .compatible = "mediatek,mt6369-regulator" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6369_regulator_match);

static struct platform_driver mt6369_regulator_driver = {
	.driver = {
		.name = "mt6369-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = mt6369_regulator_match,
	},
	.probe = mt6369_regulator_probe,
};
module_platform_driver(mt6369_regulator_driver);

MODULE_DESCRIPTION("Regulator driver for the MediaTek MT6369 PMIC");
MODULE_LICENSE("GPL");
