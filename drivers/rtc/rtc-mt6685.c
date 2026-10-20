// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 *
 * RTC of the MediaTek MT6685 clock buffer / RTC chip (SPMI)
 *
 * The register layout follows the MediaTek downstream driver
 * (drivers/rtc/rtc-mt6685.c, Copyright (C) 2020 MediaTek Inc., author
 * Amber Lin). The RTC block starts at 0x580 in the chip's 8 bit register
 * space; its registers are 16 bit wide, little endian.
 *
 * This driver deliberately does less than the downstream one. The time,
 * the power-on alarm kept in the spare registers and the power control
 * bits (BBPU) belong to the bootloader and to the stock system on the other
 * slot, so at probe nothing is written at all, the time can not be set,
 * and only the alarm (time, mask, interrupt enable) is ever programmed:
 *
 *  - read_time: like downstream, the RTC clocks are switched on for a
 *    moment first (gate set/clear registers of the chip top, the state
 *    found is restored), then the counters are read;
 *  - set_alarm / alarm_irq_enable: alarm registers, alarm interrupt enable
 *    and the RTC interrupt enable in the chip top, latched with the write
 *    trigger;
 *  - interrupt: the status is read (read to clear), the alarm interrupt
 *    disabled (one shot) and, as downstream, the alarm status in BBPU reset
 *    when the power enable bit is set.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>
#include <linux/rtc.h>

/* chip top, absolute addresses */
#define MT6685_SCK_TOP_CKPDN_CON0_L	0x514
#define MT6685_SCK_TOP_CKPDN_CON0_L_SET	0x515
#define MT6685_SCK_TOP_CKPDN_CON0_L_CLR	0x516
#define MT6685_RTC_MCLK_PDN		BIT(4)
#define MT6685_RTC_32K_CK_PDN		BIT(5)
#define MT6685_SCK_TOP_INT_CON0		0x528
#define MT6685_SCK_TOP_INT_CON0_SET	0x52a
#define MT6685_INT_EN_RTC		BIT(0)
#define MT6685_SCK_TOP_INT_STATUS0	0x534

#define MT6685_RTC_BASE			0x580

/* RTC block, offsets from MT6685_RTC_BASE, 16 bit registers */
#define RTC_BBPU		0x08
#define RTC_BBPU_KEY		0x4300
#define RTC_BBPU_PWREN		BIT(0)
#define RTC_BBPU_RESET_AL	BIT(3)
#define RTC_BBPU_CBUSY		BIT(6)
#define RTC_IRQ_STA		0x0a
#define RTC_IRQ_STA_AL		BIT(0)
#define RTC_IRQ_EN		0x0c
#define RTC_IRQ_EN_AL		BIT(0)
#define RTC_AL_MASK		0x10
#define RTC_AL_MASK_DOW		BIT(4)
#define RTC_TC_SEC		0x12
#define RTC_AL_SEC		0x20
#define RTC_WRTGR		0x42

/* time and alarm: SEC, MIN, HOU, DOM, DOW, MTH, YEA, 16 bit each */
enum {
	RTC_OFFSET_SEC,
	RTC_OFFSET_MIN,
	RTC_OFFSET_HOUR,
	RTC_OFFSET_DOM,
	RTC_OFFSET_DOW,
	RTC_OFFSET_MTH,
	RTC_OFFSET_YEAR,
	RTC_OFFSET_COUNT
};

static const u16 mt6685_rtc_field_mask[RTC_OFFSET_COUNT] = {
	[RTC_OFFSET_SEC] = 0x3f,
	[RTC_OFFSET_MIN] = 0x3f,
	[RTC_OFFSET_HOUR] = 0x1f,
	[RTC_OFFSET_DOM] = 0x1f,
	[RTC_OFFSET_DOW] = 0x07,
	[RTC_OFFSET_MTH] = 0x0f,
	[RTC_OFFSET_YEAR] = 0x7f,
};

/* the 7 bit year counts from 1968 */
#define RTC_MIN_YEAR		1968
#define RTC_MIN_YEAR_OFFSET	(RTC_MIN_YEAR - 1900)

#define RTC_TRIGGER_TIMEOUT_US	1000000

struct mt6685_rtc {
	struct device *dev;
	struct rtc_device *rtc_dev;
	struct regmap *regmap;
	struct mutex lock;
	int irq;
	bool alarm_armed;
};

static int mt6685_rtc_read16(struct mt6685_rtc *rtc, unsigned int reg, u16 *val)
{
	__le16 v;
	int ret;

	ret = regmap_bulk_read(rtc->regmap, MT6685_RTC_BASE + reg, &v, sizeof(v));
	if (!ret)
		*val = le16_to_cpu(v);

	return ret;
}

static int mt6685_rtc_write16(struct mt6685_rtc *rtc, unsigned int reg, u16 val)
{
	__le16 v = cpu_to_le16(val);

	return regmap_bulk_write(rtc->regmap, MT6685_RTC_BASE + reg, &v, sizeof(v));
}

static int mt6685_rtc_read_fields(struct mt6685_rtc *rtc, unsigned int reg,
				  u16 data[RTC_OFFSET_COUNT])
{
	__le16 raw[RTC_OFFSET_COUNT];
	int i, ret;

	ret = regmap_bulk_read(rtc->regmap, MT6685_RTC_BASE + reg, raw, sizeof(raw));
	if (ret)
		return ret;

	for (i = 0; i < RTC_OFFSET_COUNT; i++)
		data[i] = le16_to_cpu(raw[i]);

	return 0;
}

/*
 * Switch the RTC main and 32k clocks on (downstream does this around every
 * access). Returns the gate bits that were off, for mt6685_rtc_clk_restore().
 */
static int mt6685_rtc_clk_on(struct mt6685_rtc *rtc, unsigned int *was_off)
{
	unsigned int val;
	int ret;

	ret = regmap_read(rtc->regmap, MT6685_SCK_TOP_CKPDN_CON0_L, &val);
	if (ret)
		return ret;

	*was_off = val & (MT6685_RTC_MCLK_PDN | MT6685_RTC_32K_CK_PDN);
	if (!*was_off)
		return 0;

	ret = regmap_write(rtc->regmap, MT6685_SCK_TOP_CKPDN_CON0_L_CLR, *was_off);
	if (ret)
		return ret;
	usleep_range(1000, 1500);

	return 0;
}

static void mt6685_rtc_clk_restore(struct mt6685_rtc *rtc, unsigned int was_off)
{
	if (was_off)
		regmap_write(rtc->regmap, MT6685_SCK_TOP_CKPDN_CON0_L_SET, was_off);
}

/* latch the written RTC registers into the RTC domain */
static int mt6685_rtc_write_trigger(struct mt6685_rtc *rtc)
{
	unsigned int bbpu;
	int ret;

	ret = mt6685_rtc_write16(rtc, RTC_WRTGR, 1);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(rtc->regmap, MT6685_RTC_BASE + RTC_BBPU, bbpu,
				       !(bbpu & RTC_BBPU_CBUSY), 50, RTC_TRIGGER_TIMEOUT_US);
	if (ret)
		dev_err(rtc->dev, "write trigger timed out (BBPU %#x)\n", bbpu);

	return ret;
}

static int mt6685_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct mt6685_rtc *rtc = dev_get_drvdata(dev);
	u16 data[RTC_OFFSET_COUNT], sec;
	unsigned int was_off;
	int ret, tries = 3;

	mutex_lock(&rtc->lock);

	ret = mt6685_rtc_clk_on(rtc, &was_off);
	if (ret)
		goto out;
	udelay(60);
	mt6685_rtc_clk_restore(rtc, was_off);

	/* re-read when the seconds wrapped while reading the other fields */
	do {
		ret = mt6685_rtc_read_fields(rtc, RTC_TC_SEC, data);
		if (ret)
			goto out;
		ret = mt6685_rtc_read16(rtc, RTC_TC_SEC, &sec);
		if (ret)
			goto out;
		sec &= mt6685_rtc_field_mask[RTC_OFFSET_SEC];
	} while (sec < (data[RTC_OFFSET_SEC] & mt6685_rtc_field_mask[RTC_OFFSET_SEC]) &&
		 --tries);

	tm->tm_sec = data[RTC_OFFSET_SEC] & mt6685_rtc_field_mask[RTC_OFFSET_SEC];
	tm->tm_min = data[RTC_OFFSET_MIN] & mt6685_rtc_field_mask[RTC_OFFSET_MIN];
	tm->tm_hour = data[RTC_OFFSET_HOUR] & mt6685_rtc_field_mask[RTC_OFFSET_HOUR];
	tm->tm_mday = data[RTC_OFFSET_DOM] & mt6685_rtc_field_mask[RTC_OFFSET_DOM];
	tm->tm_mon = (data[RTC_OFFSET_MTH] & mt6685_rtc_field_mask[RTC_OFFSET_MTH]) - 1;
	tm->tm_year = (data[RTC_OFFSET_YEAR] & mt6685_rtc_field_mask[RTC_OFFSET_YEAR]) +
		      RTC_MIN_YEAR_OFFSET;
	tm->tm_wday = (div_s64(rtc_tm_to_time64(tm), 86400) + 4) % 7;

out:
	mutex_unlock(&rtc->lock);
	return ret;
}

static int mt6685_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct mt6685_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &alm->time;
	u16 data[RTC_OFFSET_COUNT], irqen;
	int ret;

	mutex_lock(&rtc->lock);
	ret = mt6685_rtc_read16(rtc, RTC_IRQ_EN, &irqen);
	if (!ret)
		ret = mt6685_rtc_read_fields(rtc, RTC_AL_SEC, data);
	mutex_unlock(&rtc->lock);
	if (ret)
		return ret;

	alm->enabled = !!(irqen & RTC_IRQ_EN_AL);
	/* RTC_IRQ_STA is read to clear: never looked at outside the handler */
	alm->pending = 0;

	tm->tm_sec = data[RTC_OFFSET_SEC] & mt6685_rtc_field_mask[RTC_OFFSET_SEC];
	tm->tm_min = data[RTC_OFFSET_MIN] & mt6685_rtc_field_mask[RTC_OFFSET_MIN];
	tm->tm_hour = data[RTC_OFFSET_HOUR] & mt6685_rtc_field_mask[RTC_OFFSET_HOUR];
	tm->tm_mday = data[RTC_OFFSET_DOM] & mt6685_rtc_field_mask[RTC_OFFSET_DOM];
	tm->tm_mon = (data[RTC_OFFSET_MTH] & mt6685_rtc_field_mask[RTC_OFFSET_MTH]) - 1;
	tm->tm_year = (data[RTC_OFFSET_YEAR] & mt6685_rtc_field_mask[RTC_OFFSET_YEAR]) +
		      RTC_MIN_YEAR_OFFSET;

	return 0;
}

/* called with rtc->lock held and the RTC clocks on */
static int mt6685_rtc_set_alarm_irq(struct mt6685_rtc *rtc, bool enable)
{
	u16 irqen;
	int ret;

	ret = mt6685_rtc_read16(rtc, RTC_IRQ_EN, &irqen);
	if (ret)
		return ret;

	if (enable) {
		unsigned int top;

		/* the RTC interrupt of the chip top must be on to get RCS */
		ret = regmap_read(rtc->regmap, MT6685_SCK_TOP_INT_CON0, &top);
		if (!ret && !(top & MT6685_INT_EN_RTC))
			ret = regmap_write(rtc->regmap, MT6685_SCK_TOP_INT_CON0_SET,
					   MT6685_INT_EN_RTC);
		if (ret)
			return ret;
		irqen |= RTC_IRQ_EN_AL;
	} else {
		irqen &= ~RTC_IRQ_EN_AL;
	}

	ret = mt6685_rtc_write16(rtc, RTC_IRQ_EN, irqen);
	if (ret)
		return ret;
	ret = mt6685_rtc_write_trigger(rtc);
	if (ret)
		return ret;

	rtc->alarm_armed = enable;

	return 0;
}

static int mt6685_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct mt6685_rtc *rtc = dev_get_drvdata(dev);
	unsigned int was_off;
	int ret;

	mutex_lock(&rtc->lock);
	ret = mt6685_rtc_clk_on(rtc, &was_off);
	if (!ret) {
		ret = mt6685_rtc_set_alarm_irq(rtc, enabled);
		mt6685_rtc_clk_restore(rtc, was_off);
	}
	mutex_unlock(&rtc->lock);

	return ret;
}

static int mt6685_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct mt6685_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &alm->time;
	u16 data[RTC_OFFSET_COUNT], mask;
	unsigned int was_off;
	int i, ret;

	if (tm->tm_year < RTC_MIN_YEAR_OFFSET ||
	    tm->tm_year > RTC_MIN_YEAR_OFFSET + mt6685_rtc_field_mask[RTC_OFFSET_YEAR])
		return -EINVAL;

	mutex_lock(&rtc->lock);
	ret = mt6685_rtc_clk_on(rtc, &was_off);
	if (ret)
		goto out_unlock;

	/* no alarm while the registers are half written */
	ret = mt6685_rtc_set_alarm_irq(rtc, false);
	if (ret)
		goto out;

	ret = mt6685_rtc_read_fields(rtc, RTC_AL_SEC, data);
	if (ret)
		goto out;

	/* the bits around the fields are kept, as downstream does */
#define SET_FIELD(i, v)	(data[i] = (data[i] & ~mt6685_rtc_field_mask[i]) | \
				   ((v) & mt6685_rtc_field_mask[i]))
	SET_FIELD(RTC_OFFSET_SEC, tm->tm_sec);
	SET_FIELD(RTC_OFFSET_MIN, tm->tm_min);
	SET_FIELD(RTC_OFFSET_HOUR, tm->tm_hour);
	SET_FIELD(RTC_OFFSET_DOM, tm->tm_mday);
	SET_FIELD(RTC_OFFSET_MTH, tm->tm_mon + 1);
	SET_FIELD(RTC_OFFSET_YEAR, tm->tm_year - RTC_MIN_YEAR_OFFSET);
#undef SET_FIELD

	for (i = 0; i < RTC_OFFSET_COUNT; i++) {
		if (i == RTC_OFFSET_DOW)
			continue;
		ret = mt6685_rtc_write16(rtc, RTC_AL_SEC + 2 * i, data[i]);
		if (ret)
			goto out;
	}

	/* compare all fields but the day of week */
	ret = mt6685_rtc_read16(rtc, RTC_AL_MASK, &mask);
	if (!ret && mask != RTC_AL_MASK_DOW)
		ret = mt6685_rtc_write16(rtc, RTC_AL_MASK, RTC_AL_MASK_DOW);
	if (ret)
		goto out;

	if (alm->enabled)
		ret = mt6685_rtc_set_alarm_irq(rtc, true);
	else
		ret = mt6685_rtc_write_trigger(rtc);

	dev_dbg(dev, "alarm %ptR %s\n", tm, alm->enabled ? "on" : "off");
out:
	mt6685_rtc_clk_restore(rtc, was_off);
out_unlock:
	mutex_unlock(&rtc->lock);
	return ret;
}

static irqreturn_t mt6685_rtc_irq(int irq, void *data)
{
	struct mt6685_rtc *rtc = data;
	unsigned int was_off, top = 0;
	u16 sta = 0, bbpu;
	bool alarm = false;

	mutex_lock(&rtc->lock);
	if (mt6685_rtc_clk_on(rtc, &was_off))
		goto out_unlock;

	mt6685_rtc_read16(rtc, RTC_IRQ_STA, &sta);	/* read to clear */

	/* chip top status, write 1 to clear */
	if (!regmap_read(rtc->regmap, MT6685_SCK_TOP_INT_STATUS0, &top) && top)
		regmap_write(rtc->regmap, MT6685_SCK_TOP_INT_STATUS0, top);

	if (sta & RTC_IRQ_STA_AL) {
		alarm = true;
		mt6685_rtc_set_alarm_irq(rtc, false);

		/* downstream resets the alarm status, keeping power enable */
		if (!mt6685_rtc_read16(rtc, RTC_BBPU, &bbpu) && (bbpu & RTC_BBPU_PWREN) &&
		    !mt6685_rtc_write16(rtc, RTC_BBPU,
					RTC_BBPU_KEY | RTC_BBPU_PWREN | RTC_BBPU_RESET_AL))
			mt6685_rtc_write_trigger(rtc);
	}

	mt6685_rtc_clk_restore(rtc, was_off);
out_unlock:
	mutex_unlock(&rtc->lock);

	if (alarm)
		rtc_update_irq(rtc->rtc_dev, 1, RTC_IRQF | RTC_AF);

	return (alarm || top) ? IRQ_HANDLED : IRQ_NONE;
}

static const struct rtc_class_ops mt6685_rtc_ops = {
	.read_time = mt6685_rtc_read_time,
	.read_alarm = mt6685_rtc_read_alarm,
	.set_alarm = mt6685_rtc_set_alarm,
	.alarm_irq_enable = mt6685_rtc_alarm_irq_enable,
};

static int mt6685_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6685_rtc *rtc;
	int ret;

	rtc = devm_kzalloc(dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->dev = dev;
	mutex_init(&rtc->lock);
	platform_set_drvdata(pdev, rtc);

	rtc->regmap = dev_get_regmap(dev->parent, NULL);
	if (!rtc->regmap)
		return dev_err_probe(dev, -ENODEV, "no regmap from the MT6685 core\n");

	rtc->rtc_dev = devm_rtc_allocate_device(dev);
	if (IS_ERR(rtc->rtc_dev))
		return PTR_ERR(rtc->rtc_dev);

	rtc->irq = platform_get_irq(pdev, 0);
	if (rtc->irq < 0)
		return rtc->irq;

	ret = devm_request_threaded_irq(dev, rtc->irq, NULL, mt6685_rtc_irq,
					IRQF_ONESHOT, "mt6685-rtc", rtc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	device_init_wakeup(dev, true);
	ret = devm_pm_set_wake_irq(dev, rtc->irq);
	if (ret)
		dev_warn(dev, "no wake IRQ: %d\n", ret);

	rtc->rtc_dev->ops = &mt6685_rtc_ops;
	rtc->rtc_dev->range_min = mktime64(RTC_MIN_YEAR, 1, 1, 0, 0, 0);
	rtc->rtc_dev->range_max = mktime64(RTC_MIN_YEAR + 127, 12, 31, 23, 59, 59);
	clear_bit(RTC_FEATURE_UPDATE_INTERRUPT, rtc->rtc_dev->features);

	return devm_rtc_register_device(rtc->rtc_dev);
}

/* an alarm set from here must not switch the phone on after power off */
static void mt6685_rtc_shutdown(struct platform_device *pdev)
{
	struct mt6685_rtc *rtc = platform_get_drvdata(pdev);
	unsigned int was_off;

	disable_irq(rtc->irq);

	mutex_lock(&rtc->lock);
	if (rtc->alarm_armed && !mt6685_rtc_clk_on(rtc, &was_off)) {
		mt6685_rtc_set_alarm_irq(rtc, false);
		mt6685_rtc_clk_restore(rtc, was_off);
	}
	mutex_unlock(&rtc->lock);
}

static const struct of_device_id mt6685_rtc_of_match[] = {
	{ .compatible = "mediatek,mt6685-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6685_rtc_of_match);

static struct platform_driver mt6685_rtc_driver = {
	.driver = {
		.name = "mt6685-rtc",
		.of_match_table = mt6685_rtc_of_match,
	},
	.probe = mt6685_rtc_probe,
	.shutdown = mt6685_rtc_shutdown,
};
module_platform_driver(mt6685_rtc_driver);

MODULE_DESCRIPTION("MediaTek MT6685 RTC driver");
MODULE_LICENSE("GPL");
