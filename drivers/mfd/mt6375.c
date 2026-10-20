// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6375 SubPMIC core driver
 *
 * The MT6375 answers on six I2C addresses (Type-C/PD, PMU, battery
 * monitor, two housekeeping banks and a test bank). A register is
 * addressed as (bank << 8 | offset), the bank selecting the I2C client,
 * the same scheme the MediaTek BSP driver uses. The interrupt events of
 * all function blocks are collected in sixteen PMU registers at 0x1d0
 * (write 1 to clear) with matching masks at 0x1f0.
 *
 * Register layout and slave addresses follow the GPL MediaTek BSP driver
 * (drivers/mfd/mt6375.c, Copyright (c) 2021 MediaTek Inc., ShuFan Lee
 * <shufan_lee@richtek.com>).
 */

#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/regmap.h>

#define MT6375_REG_DEV_INFO	0x100
#define MT6375_REG_EVT_BASE	0x1d0
#define MT6375_REG_MASK_BASE	0x1f0
#define MT6375_NUM_IRQ_REGS	16
#define MT6375_REG_MAX		0x4ff

#define MT6375_VENID_MASK	GENMASK(7, 4)
#define MT6375_REVID_MASK	GENMASK(3, 0)
#define MT6375_VENID		0x7

#define MT6375_ADDR_LEN		2

enum {
	MT6375_BANK_TCPC,
	MT6375_BANK_PMU,
	MT6375_BANK_BM,
	MT6375_BANK_HK1,
	MT6375_BANK_HK2,
	MT6375_NUM_BANKS,
};

static const u8 mt6375_bank_addr[MT6375_NUM_BANKS] = {
	[MT6375_BANK_TCPC] = 0x4e,
	[MT6375_BANK_PMU] = 0x34,
	[MT6375_BANK_BM] = 0x1a,
	[MT6375_BANK_HK1] = 0x4a,
	[MT6375_BANK_HK2] = 0x64,
};

struct mt6375_data {
	struct i2c_client *i2c[MT6375_NUM_BANKS];
	struct regmap_irq_chip_data *irq_data;
};

/* PMU event numbers: register (n / 8) at 0x1d0, bit (n % 8) */
#define MT6375_IRQ(_n)	REGMAP_IRQ_REG((_n), (_n) / 8, BIT((_n) % 8))

static const struct regmap_irq mt6375_irqs[] = {
	MT6375_IRQ(0),	/* fl_pwr_rdy */
	MT6375_IRQ(1),	/* fl_detach */
	MT6375_IRQ(2),	/* fl_rechg */
	MT6375_IRQ(3),	/* fl_chg_done */
	MT6375_IRQ(4),	/* fl_bk_chg */
	MT6375_IRQ(5),	/* fl_ieoc */
	MT6375_IRQ(6),	/* fl_chg_rdy */
	MT6375_IRQ(7),	/* fl_vbus_gd */
	MT6375_IRQ(8),	/* fl_vbus_ov */
	MT6375_IRQ(9),	/* fl_chg_batov */
	MT6375_IRQ(10),	/* fl_chg_sysov */
	MT6375_IRQ(11),	/* fl_chg_tout */
	MT6375_IRQ(12),	/* fl_chg_busuv */
	MT6375_IRQ(13),	/* fl_chg_threg */
	MT6375_IRQ(14),	/* fl_chg_aicr */
	MT6375_IRQ(15),	/* fl_chg_mivr */
	MT6375_IRQ(16),	/* fl_sys_short */
	MT6375_IRQ(17),	/* fl_sys_min */
	MT6375_IRQ(18),	/* fl_aicc_done */
	MT6375_IRQ(19),	/* fl_pe_done */
	MT6375_IRQ(21),	/* pp_pgb_evt */
	MT6375_IRQ(22),	/* ft_dig_thr */
	MT6375_IRQ(23),	/* fl_wdt */
	MT6375_IRQ(24),	/* fl_otg_fault */
	MT6375_IRQ(25),	/* fl_otg_bat_lbp */
	MT6375_IRQ(26),	/* fl_otg_cc */
	MT6375_IRQ(27),	/* fl_batpro_done */
	MT6375_IRQ(29),	/* fl_dcd_done */
	MT6375_IRQ(30),	/* fl_bc12_hvdcp */
	MT6375_IRQ(31),	/* fl_bc12_dn */
	MT6375_IRQ(32),	/* int_chrdet_uv */
	MT6375_IRQ(33),	/* int_chrdet_ov */
	MT6375_IRQ(34),	/* int_chrdet_ext */
	MT6375_IRQ(43),	/* adc_vbat_mon_ov */
	MT6375_IRQ(44),	/* adc_donei */
	MT6375_IRQ(64),	/* otp0_evt */
	MT6375_IRQ(65),	/* otp1_evt */
	MT6375_IRQ(67),	/* qonb_rst_evt */
	MT6375_IRQ(68),	/* mrstb_evt */
	MT6375_IRQ(70),	/* vddaov_evt */
	MT6375_IRQ(112),	/* usbid_evt */
	MT6375_IRQ(120),	/* pd_evt */
};

static const struct regmap_irq_chip mt6375_irq_chip = {
	.name = "mt6375-irqs",
	.status_base = MT6375_REG_EVT_BASE,
	.mask_base = MT6375_REG_MASK_BASE,
	.ack_base = MT6375_REG_EVT_BASE,
	.init_ack_masked = true,
	.num_regs = MT6375_NUM_IRQ_REGS,
	.irqs = mt6375_irqs,
	.num_irqs = ARRAY_SIZE(mt6375_irqs),
};

static const struct mfd_cell mt6375_devices[] = {
	MFD_CELL_OF("mt6375-adc", NULL, NULL, 0, 0, "mediatek,mt6375-adc"),
	MFD_CELL_OF("mt6375-charger", NULL, NULL, 0, 0, "mediatek,mt6375-chg"),
	MFD_CELL_OF("mt6375-tcpc", NULL, NULL, 0, 0, "mediatek,mt6375-tcpc"),
};

static struct i2c_client *mt6375_bank_client(struct mt6375_data *data, u8 bank)
{
	if (bank >= MT6375_NUM_BANKS)
		return NULL;

	return data->i2c[bank];
}

static int mt6375_regmap_read(void *context, const void *reg_buf,
			      size_t reg_size, void *val_buf, size_t val_size)
{
	struct mt6375_data *data = context;
	const u8 *addr = reg_buf;
	struct i2c_client *client;
	int ret;

	client = mt6375_bank_client(data, addr[0]);
	if (!client)
		return -EINVAL;

	ret = i2c_smbus_read_i2c_block_data(client, addr[1], val_size, val_buf);
	if (ret < 0)
		return ret;

	return ret == val_size ? 0 : -EIO;
}

static int mt6375_regmap_write(void *context, const void *buf, size_t count)
{
	struct mt6375_data *data = context;
	const u8 *addr = buf;
	struct i2c_client *client;

	if (count <= MT6375_ADDR_LEN)
		return -EINVAL;

	client = mt6375_bank_client(data, addr[0]);
	if (!client)
		return -EINVAL;

	return i2c_smbus_write_i2c_block_data(client, addr[1],
					      count - MT6375_ADDR_LEN,
					      addr + MT6375_ADDR_LEN);
}

static const struct regmap_bus mt6375_regmap_bus = {
	.read = mt6375_regmap_read,
	.write = mt6375_regmap_write,
	.max_raw_read = I2C_SMBUS_BLOCK_MAX,
	.max_raw_write = I2C_SMBUS_BLOCK_MAX,
};

static const struct regmap_config mt6375_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = MT6375_REG_MAX,
};

static int mt6375_check_id(struct device *dev, struct regmap *regmap)
{
	unsigned int val;
	int ret;

	ret = regmap_read(regmap, MT6375_REG_DEV_INFO, &val);
	if (ret)
		return ret;

	if (FIELD_GET(MT6375_VENID_MASK, val) != MT6375_VENID)
		return dev_err_probe(dev, -ENODEV,
				     "unexpected device info 0x%02x\n", val);

	dev_info(dev, "MT6375 revision 0x%lx\n",
		 FIELD_GET(MT6375_REVID_MASK, val));
	return 0;
}

static int mt6375_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct mt6375_data *data;
	struct regmap *regmap;
	int i, ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	for (i = 0; i < MT6375_NUM_BANKS; i++) {
		if (i == MT6375_BANK_PMU) {
			data->i2c[i] = i2c;
			continue;
		}

		data->i2c[i] = devm_i2c_new_dummy_device(dev, i2c->adapter,
							 mt6375_bank_addr[i]);
		if (IS_ERR(data->i2c[i]))
			return dev_err_probe(dev, PTR_ERR(data->i2c[i]),
					     "failed to add I2C client 0x%02x\n",
					     mt6375_bank_addr[i]);
	}

	regmap = devm_regmap_init(dev, &mt6375_regmap_bus, data,
				  &mt6375_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to init regmap\n");

	ret = mt6375_check_id(dev, regmap);
	if (ret)
		return ret;

	ret = devm_regmap_add_irq_chip(dev, regmap, i2c->irq, IRQF_ONESHOT, 0,
				       &mt6375_irq_chip, &data->irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add irq chip\n");

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, mt6375_devices,
				    ARRAY_SIZE(mt6375_devices), NULL, 0,
				    regmap_irq_get_domain(data->irq_data));
}

static const struct of_device_id mt6375_of_match[] = {
	{ .compatible = "mediatek,mt6375" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6375_of_match);

static struct i2c_driver mt6375_driver = {
	.driver = {
		.name = "mt6375",
		.of_match_table = mt6375_of_match,
	},
	.probe = mt6375_probe,
};
module_i2c_driver(mt6375_driver);

MODULE_DESCRIPTION("MediaTek MT6375 SubPMIC core driver");
MODULE_LICENSE("GPL");
