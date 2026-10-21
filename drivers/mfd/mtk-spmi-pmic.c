// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 *
 * MediaTek SPMI PMICs (MT6363, MT6369) core driver
 *
 * The PMIC is a slave on the SPMI bus; its interrupts are grouped in a few
 * blocks (BUCK, LDO, PSC, ...) and signalled to the SPMI controller with a
 * Request Capable Slave (RCS) message. A top status register says which
 * block has pending interrupts, each block has one enable and one status
 * (write 1 to clear) register for every 8 interrupts.
 *
 * Based on the MediaTek downstream driver (drivers/mfd/mtk-spmi-pmic.c,
 * Copyright (c) 2021 MediaTek Inc.).
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/core.h>
#include <linux/mfd/mt6363/registers.h>
#include <linux/mfd/mt6369/registers.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/spmi.h>

#define MTK_SPMI_PMIC_SWCID		0x0b
#define MTK_SPMI_PMIC_RCS_INT_DONE	0x41b
#define MTK_SPMI_PMIC_IRQS_PER_REG	8

/* The enable registers come with SET and CLR companions: CON, SET, CLR */
#define MTK_SPMI_PMIC_INT_CON_STRIDE	3

/**
 * struct mtk_spmi_pmic_irq_group - A block of PMIC interrupts
 * @hwirq_base: First hwirq number of the block
 * @num_regs:   Number of 8-bit enable/status registers of the block
 * @en_reg:     First enable register
 * @sta_reg:    First status register
 * @top_bit:    Bit of the block in the top interrupt status register
 */
struct mtk_spmi_pmic_irq_group {
	u16 hwirq_base;
	u8 num_regs;
	u16 en_reg;
	u16 sta_reg;
	u8 top_bit;
};

#define MTK_SPMI_PMIC_IRQ_GROUP(_chip, _grp, _regs, _bit)		\
{									\
	.hwirq_base = _chip##_IRQ_##_grp##_BASE,			\
	.num_regs = _regs,						\
	.en_reg = _chip##_##_grp##_TOP_INT_CON0,			\
	.sta_reg = _chip##_##_grp##_TOP_INT_STATUS0,			\
	.top_bit = _bit,						\
}

struct mtk_spmi_pmic_data {
	const struct mfd_cell *cells;
	unsigned int num_cells;
	const struct mtk_spmi_pmic_irq_group *irq_groups;
	unsigned int num_irq_groups;
	unsigned int num_irqs;
	u16 top_int_status_reg;
	unsigned int max_register;
};

struct mtk_spmi_pmic {
	struct device *dev;
	struct spmi_device *sdev;
	struct regmap *regmap;
	const struct mtk_spmi_pmic_data *data;
	struct irq_domain *domain;
	struct mutex irq_lock;
	unsigned long *irq_enable;
	unsigned long *irq_enable_cur;
};

static const struct resource mt6363_keys_resources[] = {
	DEFINE_RES_IRQ_NAMED(MT6363_IRQ_PWRKEY, "powerkey"),
	DEFINE_RES_IRQ_NAMED(MT6363_IRQ_HOMEKEY, "homekey"),
	DEFINE_RES_IRQ_NAMED(MT6363_IRQ_PWRKEY_R, "powerkey_r"),
	DEFINE_RES_IRQ_NAMED(MT6363_IRQ_HOMEKEY_R, "homekey_r"),
};

static const struct mfd_cell mt6363_cells[] = {
	MFD_CELL_OF("mt6363-auxadc", NULL, NULL, 0, 0, "mediatek,mt6363-auxadc"),
	MFD_CELL_OF("mtk-pmic-keys", mt6363_keys_resources, NULL, 0, 0,
		    "mediatek,mt6363-keys"),
	MFD_CELL_OF("mt6363-regulator", NULL, NULL, 0, 0, "mediatek,mt6363-regulator"),
};

static const struct mfd_cell mt6369_cells[] = {
	MFD_CELL_OF("mt6369-regulator", NULL, NULL, 0, 0, "mediatek,mt6369-regulator"),
	MFD_CELL_OF("mt6369-sound", NULL, NULL, 0, 0, "mediatek,mt6369-sound"),
};

/*
 * The BM (fuel gauge) block of MT6363 is left out: its hwirq numbers would
 * overlap the second register of the HK block.
 */
static const struct mtk_spmi_pmic_irq_group mt6363_irq_groups[] = {
	MTK_SPMI_PMIC_IRQ_GROUP(MT6363, BUCK, 2, 0),
	MTK_SPMI_PMIC_IRQ_GROUP(MT6363, LDO, 4, 1),
	MTK_SPMI_PMIC_IRQ_GROUP(MT6363, PSC, 2, 2),
	MTK_SPMI_PMIC_IRQ_GROUP(MT6363, MISC, 2, 3),
	MTK_SPMI_PMIC_IRQ_GROUP(MT6363, HK, 2, 4),
};

static const struct mtk_spmi_pmic_irq_group mt6369_irq_groups[] = {
	MTK_SPMI_PMIC_IRQ_GROUP(MT6369, BUCK, 1, 0),
	MTK_SPMI_PMIC_IRQ_GROUP(MT6369, LDO, 3, 1),
	MTK_SPMI_PMIC_IRQ_GROUP(MT6369, MISC, 2, 3),
	MTK_SPMI_PMIC_IRQ_GROUP(MT6369, HK, 1, 4),
	MTK_SPMI_PMIC_IRQ_GROUP(MT6369, AUD, 1, 7),
};

static const struct mtk_spmi_pmic_irq_group *
mtk_spmi_pmic_find_group(const struct mtk_spmi_pmic_data *data, irq_hw_number_t hwirq)
{
	unsigned int i;

	for (i = 0; i < data->num_irq_groups; i++) {
		const struct mtk_spmi_pmic_irq_group *grp = &data->irq_groups[i];

		if (hwirq >= grp->hwirq_base &&
		    hwirq < grp->hwirq_base + grp->num_regs * MTK_SPMI_PMIC_IRQS_PER_REG)
			return grp;
	}

	return NULL;
}

static void mtk_spmi_pmic_irq_enable(struct irq_data *d)
{
	struct mtk_spmi_pmic *pmic = irq_data_get_irq_chip_data(d);

	__set_bit(irqd_to_hwirq(d), pmic->irq_enable);
}

static void mtk_spmi_pmic_irq_disable(struct irq_data *d)
{
	struct mtk_spmi_pmic *pmic = irq_data_get_irq_chip_data(d);

	__clear_bit(irqd_to_hwirq(d), pmic->irq_enable);
}

static void mtk_spmi_pmic_irq_lock(struct irq_data *d)
{
	struct mtk_spmi_pmic *pmic = irq_data_get_irq_chip_data(d);

	mutex_lock(&pmic->irq_lock);
}

static void mtk_spmi_pmic_irq_sync_unlock(struct irq_data *d)
{
	struct mtk_spmi_pmic *pmic = irq_data_get_irq_chip_data(d);
	const struct mtk_spmi_pmic_irq_group *grp;
	unsigned int hwirq, offset, reg;
	int ret;

	for (hwirq = 0; hwirq < pmic->data->num_irqs; hwirq++) {
		bool en = test_bit(hwirq, pmic->irq_enable);

		if (en == test_bit(hwirq, pmic->irq_enable_cur))
			continue;

		grp = mtk_spmi_pmic_find_group(pmic->data, hwirq);
		if (!grp)
			continue;

		offset = hwirq - grp->hwirq_base;
		reg = grp->en_reg + (offset / MTK_SPMI_PMIC_IRQS_PER_REG) *
		      MTK_SPMI_PMIC_INT_CON_STRIDE;

		ret = regmap_update_bits(pmic->regmap, reg,
					 BIT(offset % MTK_SPMI_PMIC_IRQS_PER_REG),
					 en ? BIT(offset % MTK_SPMI_PMIC_IRQS_PER_REG) : 0);
		if (ret) {
			dev_err(pmic->dev, "Cannot %s hwirq %u: %d\n",
				en ? "enable" : "disable", hwirq, ret);
			continue;
		}

		__assign_bit(hwirq, pmic->irq_enable_cur, en);
	}

	mutex_unlock(&pmic->irq_lock);
}

static const struct irq_chip mtk_spmi_pmic_irq_chip = {
	.name = "mtk-spmi-pmic",
	.flags = IRQCHIP_SKIP_SET_WAKE,
	.irq_enable = mtk_spmi_pmic_irq_enable,
	.irq_disable = mtk_spmi_pmic_irq_disable,
	.irq_bus_lock = mtk_spmi_pmic_irq_lock,
	.irq_bus_sync_unlock = mtk_spmi_pmic_irq_sync_unlock,
};

static void mtk_spmi_pmic_handle_group(struct mtk_spmi_pmic *pmic,
				       const struct mtk_spmi_pmic_irq_group *grp)
{
	unsigned int i, status, reg;
	unsigned long pending;
	int bit, ret;

	for (i = 0; i < grp->num_regs; i++) {
		reg = grp->sta_reg + i;

		ret = regmap_read(pmic->regmap, reg, &status);
		if (ret) {
			dev_err(pmic->dev, "Cannot read IRQ status 0x%x: %d\n", reg, ret);
			return;
		}

		if (!status)
			continue;

		pending = status;
		for_each_set_bit(bit, &pending, MTK_SPMI_PMIC_IRQS_PER_REG) {
			unsigned int virq;

			virq = irq_find_mapping(pmic->domain, grp->hwirq_base +
						i * MTK_SPMI_PMIC_IRQS_PER_REG + bit);
			if (virq)
				handle_nested_irq(virq);
		}

		/* Write 1 to clear */
		regmap_write(pmic->regmap, reg, status);
	}
}

static irqreturn_t mtk_spmi_pmic_irq_thread(int irq, void *data)
{
	struct mtk_spmi_pmic *pmic = data;
	unsigned int i, top_status;
	int ret;

	ret = regmap_read(pmic->regmap, pmic->data->top_int_status_reg, &top_status);
	if (ret) {
		dev_err(pmic->dev, "Cannot read top IRQ status: %d\n", ret);
		return IRQ_NONE;
	}

	for (i = 0; i < pmic->data->num_irq_groups; i++) {
		const struct mtk_spmi_pmic_irq_group *grp = &pmic->data->irq_groups[i];

		if (top_status & BIT(grp->top_bit))
			mtk_spmi_pmic_handle_group(pmic, grp);
	}

	/* Allow the PMIC to send the next RCS message */
	ret = regmap_write(pmic->regmap, MTK_SPMI_PMIC_RCS_INT_DONE, 1);
	if (ret)
		dev_err(pmic->dev, "Cannot acknowledge RCS: %d\n", ret);

	return IRQ_HANDLED;
}

static int mtk_spmi_pmic_irq_translate(struct irq_domain *d, struct irq_fwspec *fwspec,
				       unsigned long *hwirq, unsigned int *type)
{
	struct mtk_spmi_pmic *pmic = d->host_data;

	/* Cells: SPMI slave ID, PMIC interrupt number, trigger type */
	if (fwspec->param_count != 3)
		return -EINVAL;

	if (fwspec->param[0] != pmic->sdev->usid)
		return -EINVAL;

	if (!mtk_spmi_pmic_find_group(pmic->data, fwspec->param[1]))
		return -EINVAL;

	*hwirq = fwspec->param[1];
	*type = fwspec->param[2] & IRQ_TYPE_SENSE_MASK;

	return 0;
}

static int mtk_spmi_pmic_irq_map(struct irq_domain *d, unsigned int virq,
				 irq_hw_number_t hwirq)
{
	struct mtk_spmi_pmic *pmic = d->host_data;

	irq_set_chip_data(virq, pmic);
	irq_set_chip_and_handler(virq, &mtk_spmi_pmic_irq_chip, handle_level_irq);
	irq_set_nested_thread(virq, true);
	irq_set_noprobe(virq);

	return 0;
}

static const struct irq_domain_ops mtk_spmi_pmic_irq_domain_ops = {
	.map = mtk_spmi_pmic_irq_map,
	.translate = mtk_spmi_pmic_irq_translate,
};

static void mtk_spmi_pmic_irq_domain_remove(void *data)
{
	irq_domain_remove(data);
}

static int mtk_spmi_pmic_irq_init(struct mtk_spmi_pmic *pmic)
{
	const struct mtk_spmi_pmic_data *data = pmic->data;
	struct device *dev = pmic->dev;
	unsigned int i, j;
	int irq, ret;

	irq = of_irq_get(dev->of_node, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "Cannot get IRQ\n");
	if (!irq)
		return dev_err_probe(dev, -ENXIO, "Cannot map IRQ\n");

	mutex_init(&pmic->irq_lock);

	pmic->irq_enable = devm_bitmap_zalloc(dev, data->num_irqs, GFP_KERNEL);
	pmic->irq_enable_cur = devm_bitmap_zalloc(dev, data->num_irqs, GFP_KERNEL);
	if (!pmic->irq_enable || !pmic->irq_enable_cur)
		return -ENOMEM;

	/* Start from a known state: everything masked and acknowledged */
	for (i = 0; i < data->num_irq_groups; i++) {
		const struct mtk_spmi_pmic_irq_group *grp = &data->irq_groups[i];

		for (j = 0; j < grp->num_regs; j++) {
			ret = regmap_write(pmic->regmap,
					   grp->en_reg + j * MTK_SPMI_PMIC_INT_CON_STRIDE, 0);
			if (ret)
				return ret;

			ret = regmap_write(pmic->regmap, grp->sta_reg + j, GENMASK(7, 0));
			if (ret)
				return ret;
		}
	}

	ret = regmap_write(pmic->regmap, MTK_SPMI_PMIC_RCS_INT_DONE, 1);
	if (ret)
		return ret;

	pmic->domain = irq_domain_create_linear(of_fwnode_handle(dev->of_node),
						data->num_irqs,
						&mtk_spmi_pmic_irq_domain_ops, pmic);
	if (!pmic->domain)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, mtk_spmi_pmic_irq_domain_remove, pmic->domain);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, irq, NULL, mtk_spmi_pmic_irq_thread,
					IRQF_ONESHOT, dev_name(dev), pmic);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot request IRQ %d\n", irq);

	return 0;
}

static int mtk_spmi_pmic_probe(struct spmi_device *sdev)
{
	struct regmap_config regmap_config = {
		.reg_bits = 16,
		.val_bits = 8,
		.fast_io = true,
	};
	struct device *dev = &sdev->dev;
	struct mtk_spmi_pmic *pmic;
	unsigned int i, id;
	int ret;

	pmic = devm_kzalloc(dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic)
		return -ENOMEM;

	pmic->dev = dev;
	pmic->sdev = sdev;
	pmic->data = of_device_get_match_data(dev);
	if (!pmic->data)
		return -ENODEV;

	regmap_config.max_register = pmic->data->max_register;
	pmic->regmap = devm_regmap_init_spmi_ext(sdev, &regmap_config);
	if (IS_ERR(pmic->regmap))
		return dev_err_probe(dev, PTR_ERR(pmic->regmap), "Cannot init regmap\n");

	/*
	 * The first access may fail if the bootloader left the PMIC in sleep
	 * mode: it wakes the PMIC up, so ignore its result.
	 */
	regmap_read(pmic->regmap, MTK_SPMI_PMIC_SWCID, &id);

	ret = regmap_read(pmic->regmap, MTK_SPMI_PMIC_SWCID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot read chip ID\n");

	dev_dbg(dev, "Chip ID 0x%02x\n", id);

	dev_set_drvdata(dev, pmic);

	ret = mtk_spmi_pmic_irq_init(pmic);
	if (ret)
		return ret;

	/* Only instantiate the functions that are described in the devicetree */
	for (i = 0; i < pmic->data->num_cells; i++) {
		const struct mfd_cell *cell = &pmic->data->cells[i];
		struct device_node *np;

		np = of_get_compatible_child(dev->of_node, cell->of_compatible);
		if (!np)
			continue;
		of_node_put(np);

		ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, cell, 1,
					   NULL, 0, pmic->domain);
		if (ret)
			return dev_err_probe(dev, ret, "Cannot add %s\n", cell->name);
	}

	return 0;
}

static const struct mtk_spmi_pmic_data mt6363_data = {
	.cells = mt6363_cells,
	.num_cells = ARRAY_SIZE(mt6363_cells),
	.irq_groups = mt6363_irq_groups,
	.num_irq_groups = ARRAY_SIZE(mt6363_irq_groups),
	.num_irqs = MT6363_IRQ_NR,
	.top_int_status_reg = MT6363_TOP_INT_STATUS1,
	.max_register = 0x1fff,
};

static const struct mtk_spmi_pmic_data mt6369_data = {
	.cells = mt6369_cells,
	.num_cells = ARRAY_SIZE(mt6369_cells),
	.irq_groups = mt6369_irq_groups,
	.num_irq_groups = ARRAY_SIZE(mt6369_irq_groups),
	.num_irqs = MT6369_IRQ_NR,
	.top_int_status_reg = MT6369_TOP_INT_STATUS1,
	/* up to the end of the audio block (AUDDEC/AUDENC/ACCDET) */
	.max_register = 0x27ff,
};

static const struct of_device_id mtk_spmi_pmic_of_match[] = {
	{ .compatible = "mediatek,mt6363", .data = &mt6363_data },
	{ .compatible = "mediatek,mt6369", .data = &mt6369_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_spmi_pmic_of_match);

static struct spmi_driver mtk_spmi_pmic_driver = {
	.driver = {
		.name = "mtk-spmi-pmic",
		.of_match_table = mtk_spmi_pmic_of_match,
	},
	.probe = mtk_spmi_pmic_probe,
};
module_spmi_driver(mtk_spmi_pmic_driver);

MODULE_DESCRIPTION("MediaTek SPMI PMIC core driver");
MODULE_LICENSE("GPL");
