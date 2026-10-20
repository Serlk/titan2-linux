// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6375 switching charger
 *
 * Register fields follow the GPL MediaTek BSP driver
 * (drivers/power/supply/mt6375-charger.c, Copyright (c) 2021 MediaTek Inc.,
 * ShuFan Lee <shufan_lee@richtek.com>).
 *
 * The driver is deliberately conservative:
 *  - without a monitored-battery (constant charge voltage and current) it
 *    only reports state and never writes the charger;
 *  - charging is enabled only while the battery fuel gauge reports a
 *    temperature inside the battery operating range and a voltage not
 *    above the charge voltage, and is switched off when it cannot be read;
 *  - charge voltage and current requests are clamped to the battery limits;
 *  - BC1.2 detection, OTG boost, PE/AICC and the charger watchdog are not
 *    used. The detector needs the USB PHY to hand D+/D- over to the
 *    charger (the BSP PHY "BC11" switch, which mainline PHY drivers do not
 *    have), so the result of the boot loader BC1.2 run is only read back,
 *    and only until the cable is first removed.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#define MT6375_REG_CHG_TOP1	0x120
#define MT6375_REG_CHG_AICR	0x122
#define MT6375_REG_CHG_MIVR	0x123
#define MT6375_REG_CHG_VCHG	0x125
#define MT6375_REG_CHG_ICHG	0x126
#define MT6375_REG_CHG_TMR	0x127
#define MT6375_REG_CHG_EOC	0x128
#define MT6375_REG_CHG_WDT	0x12a
#define MT6375_REG_BAT_COMP	0x133
#define MT6375_REG_CHG_STAT	0x134
#define MT6375_REG_BC12_STAT	0x151
#define MT6375_REG_CHG_STAT0	0x1e0

#define MT6375_CHG_EN		BIT(0)
#define MT6375_ILIM_EN		BIT(7)
#define MT6375_AICR_MASK	GENMASK(6, 0)
#define MT6375_MIVR_MASK	GENMASK(6, 0)
#define MT6375_CV_MASK		GENMASK(6, 0)
#define MT6375_ICHG_MASK	GENMASK(5, 0)
#define MT6375_TMR_EN		BIT(7)
#define MT6375_TMR_MASK		GENMASK(5, 4)
#define MT6375_IEOC_MASK	GENMASK(7, 4)
#define MT6375_TE		BIT(1)
#define MT6375_WDT_EN		BIT(3)
#define MT6375_IRCMP_MASK	(GENMASK(6, 4) | GENMASK(2, 0))
#define MT6375_IC_STAT_MASK	GENMASK(3, 0)
#define MT6375_PORT_STAT_MASK	GENMASK(3, 0)
#define MT6375_PWR_RDY		BIT(0)

/* Field ranges in mA / mV, (value - min) / step + offset */
#define MT6375_AICR_MIN		100
#define MT6375_AICR_MAX		3225
#define MT6375_AICR_STEP	25
#define MT6375_AICR_OFFS	2
#define MT6375_CV_MIN		3900
#define MT6375_CV_MAX		4710
#define MT6375_CV_STEP		10
#define MT6375_ICHG_MIN		300
#define MT6375_ICHG_MAX		3150
#define MT6375_ICHG_STEP	50
#define MT6375_ICHG_OFFS	6
#define MT6375_IEOC_MIN		100
#define MT6375_IEOC_MAX		800
#define MT6375_IEOC_STEP	50
#define MT6375_IEOC_OFFS	1
#define MT6375_MIVR_MIN		3900
#define MT6375_MIVR_STEP	100

/* Defaults when the battery description leaves them out (stock values) */
#define MT6375_DEF_AICR_UA	500000
#define MT6375_MAX_AICR_UA	3200000
#define MT6375_DEF_IEOC_UA	300000
#define MT6375_DEF_TEMP_MIN	0	/* degC */
#define MT6375_DEF_TEMP_MAX	50	/* degC */
#define MT6375_TEMP_MIN_HYST	60	/* deci-degC */
#define MT6375_TEMP_MAX_HYST	30	/* deci-degC */
#define MT6375_OV_MARGIN_UV	50000
#define MT6375_OV_RELEASE_UV	100000
#define MT6375_SAFETY_TIMER_H	10
#define MT6375_MONITOR_MS	10000

enum mt6375_ic_stat {
	MT6375_STAT_SLEEP,
	MT6375_STAT_VBUS_RDY,
	MT6375_STAT_TRICKLE,
	MT6375_STAT_PRE,
	MT6375_STAT_FAST,
	MT6375_STAT_EOC,
	MT6375_STAT_BKGND,
	MT6375_STAT_DONE,
	MT6375_STAT_FAULT,
	MT6375_STAT_OTG = 15,
};

enum mt6375_port_stat {
	MT6375_PORT_NOINFO,
	MT6375_PORT_APPLE_10W = 8,
	MT6375_PORT_SAMSUNG,
	MT6375_PORT_APPLE_5W,
	MT6375_PORT_APPLE_12W,
	MT6375_PORT_UNKNOWN_TA,
	MT6375_PORT_SDP,
	MT6375_PORT_CDP,
	MT6375_PORT_DCP,
};

struct mt6375_chg {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply *bat;
	struct delayed_work monitor;
	struct mutex lock;
	struct iio_channel *vbus_adc;
	struct iio_channel *ibus_adc;

	bool configured;
	u32 cv_max_uv;
	u32 ichg_max_ua;
	int temp_min;		/* deci-degC */
	int temp_max;		/* deci-degC */
	int behaviour;

	/* BC12_STAT belongs to the present attach (boot loader run or event) */
	bool bc12_valid;

	bool temp_blocked;
	bool ov_blocked;
	bool chg_on;
	const char *reason;
};

static unsigned int mt6375_to_sel(u32 val, u32 min, u32 max, u32 step,
				  u32 offs)
{
	val = clamp(val, min, max);
	return (val - min) / step + offs;	/* rounds down */
}

static u32 mt6375_from_sel(unsigned int sel, u32 min, u32 step, u32 offs)
{
	return sel < offs ? min : min + (sel - offs) * step;
}

static int mt6375_read_field(struct mt6375_chg *chg, unsigned int reg,
			     unsigned int mask, unsigned int *sel)
{
	unsigned int val;
	int ret;

	ret = regmap_read(chg->regmap, reg, &val);
	if (ret)
		return ret;

	*sel = (val & mask) >> __ffs(mask);
	return 0;
}

static int mt6375_write_field(struct mt6375_chg *chg, unsigned int reg,
			      unsigned int mask, unsigned int sel)
{
	return regmap_update_bits(chg->regmap, reg, mask,
				  (sel << __ffs(mask)) & mask);
}

static int mt6375_get_aicr(struct mt6375_chg *chg, int *ua)
{
	unsigned int sel;
	int ret;

	ret = mt6375_read_field(chg, MT6375_REG_CHG_AICR, MT6375_AICR_MASK, &sel);
	if (ret)
		return ret;

	*ua = 1000 * mt6375_from_sel(sel, MT6375_AICR_MIN, MT6375_AICR_STEP,
				     MT6375_AICR_OFFS);
	return 0;
}

static int mt6375_set_aicr(struct mt6375_chg *chg, u32 ua)
{
	ua = min(ua, MT6375_MAX_AICR_UA);
	return mt6375_write_field(chg, MT6375_REG_CHG_AICR, MT6375_AICR_MASK,
				  mt6375_to_sel(ua / 1000, MT6375_AICR_MIN,
						MT6375_AICR_MAX,
						MT6375_AICR_STEP,
						MT6375_AICR_OFFS));
}

static int mt6375_get_cv(struct mt6375_chg *chg, int *uv)
{
	unsigned int sel;
	int ret;

	ret = mt6375_read_field(chg, MT6375_REG_CHG_VCHG, MT6375_CV_MASK, &sel);
	if (ret)
		return ret;

	*uv = 1000 * mt6375_from_sel(sel, MT6375_CV_MIN, MT6375_CV_STEP, 0);
	return 0;
}

static int mt6375_set_cv(struct mt6375_chg *chg, u32 uv)
{
	/* never above the battery limit; the field rounds down */
	uv = min(uv, chg->cv_max_uv);
	return mt6375_write_field(chg, MT6375_REG_CHG_VCHG, MT6375_CV_MASK,
				  mt6375_to_sel(uv / 1000, MT6375_CV_MIN,
						MT6375_CV_MAX, MT6375_CV_STEP,
						0));
}

static int mt6375_get_ichg(struct mt6375_chg *chg, int *ua)
{
	unsigned int sel;
	int ret;

	ret = mt6375_read_field(chg, MT6375_REG_CHG_ICHG, MT6375_ICHG_MASK, &sel);
	if (ret)
		return ret;

	*ua = 1000 * mt6375_from_sel(sel, MT6375_ICHG_MIN, MT6375_ICHG_STEP,
				     MT6375_ICHG_OFFS);
	return 0;
}

static int mt6375_set_ichg(struct mt6375_chg *chg, u32 ua)
{
	ua = min(ua, chg->ichg_max_ua);
	return mt6375_write_field(chg, MT6375_REG_CHG_ICHG, MT6375_ICHG_MASK,
				  mt6375_to_sel(ua / 1000, MT6375_ICHG_MIN,
						MT6375_ICHG_MAX,
						MT6375_ICHG_STEP,
						MT6375_ICHG_OFFS));
}

static int mt6375_get_ieoc(struct mt6375_chg *chg, int *ua)
{
	unsigned int sel;
	int ret;

	ret = mt6375_read_field(chg, MT6375_REG_CHG_EOC, MT6375_IEOC_MASK, &sel);
	if (ret)
		return ret;

	*ua = 1000 * mt6375_from_sel(sel, MT6375_IEOC_MIN, MT6375_IEOC_STEP,
				     MT6375_IEOC_OFFS);
	return 0;
}

static int mt6375_get_mivr(struct mt6375_chg *chg, int *uv)
{
	unsigned int sel;
	int ret;

	ret = mt6375_read_field(chg, MT6375_REG_CHG_MIVR, MT6375_MIVR_MASK, &sel);
	if (ret)
		return ret;

	*uv = 1000 * mt6375_from_sel(sel, MT6375_MIVR_MIN, MT6375_MIVR_STEP, 0);
	return 0;
}

static int mt6375_get_online(struct mt6375_chg *chg, int *online)
{
	unsigned int val;
	int ret;

	ret = regmap_read(chg->regmap, MT6375_REG_CHG_STAT0, &val);
	if (ret)
		return ret;

	*online = !!(val & MT6375_PWR_RDY);
	return 0;
}

static int mt6375_get_status(struct mt6375_chg *chg, int *status)
{
	unsigned int stat, top;
	int online, ret;

	ret = mt6375_get_online(chg, &online);
	if (ret)
		return ret;

	if (!online) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	ret = regmap_read(chg->regmap, MT6375_REG_CHG_TOP1, &top);
	if (ret)
		return ret;

	ret = mt6375_read_field(chg, MT6375_REG_CHG_STAT, MT6375_IC_STAT_MASK,
				&stat);
	if (ret)
		return ret;

	switch (stat) {
	case MT6375_STAT_TRICKLE:
	case MT6375_STAT_PRE:
	case MT6375_STAT_FAST:
	case MT6375_STAT_EOC:
	case MT6375_STAT_BKGND:
		*status = (top & MT6375_CHG_EN) ? POWER_SUPPLY_STATUS_CHARGING :
						  POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case MT6375_STAT_DONE:
		*status = POWER_SUPPLY_STATUS_FULL;
		break;
	case MT6375_STAT_OTG:
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	}

	return 0;
}

static int mt6375_get_charge_type(struct mt6375_chg *chg, int *type)
{
	unsigned int stat;
	int ret;

	ret = mt6375_read_field(chg, MT6375_REG_CHG_STAT, MT6375_IC_STAT_MASK,
				&stat);
	if (ret)
		return ret;

	switch (stat) {
	case MT6375_STAT_TRICKLE:
	case MT6375_STAT_PRE:
		*type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case MT6375_STAT_FAST:
	case MT6375_STAT_EOC:
	case MT6375_STAT_BKGND:
		*type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	default:
		*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	}

	return 0;
}

/*
 * Only read back what a BC1.2 run found; the driver never starts one. A
 * result is used only while the attach it was taken on lasts.
 */
static int mt6375_get_usb_type(struct mt6375_chg *chg, int *type)
{
	unsigned int port;
	int online, ret;

	ret = mt6375_get_online(chg, &online);
	if (ret)
		return ret;

	if (!online)
		WRITE_ONCE(chg->bc12_valid, false);

	if (!READ_ONCE(chg->bc12_valid)) {
		*type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		return 0;
	}

	ret = mt6375_read_field(chg, MT6375_REG_BC12_STAT,
				MT6375_PORT_STAT_MASK, &port);
	if (ret)
		return ret;

	switch (port) {
	case MT6375_PORT_SDP:
		*type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case MT6375_PORT_CDP:
		*type = POWER_SUPPLY_USB_TYPE_CDP;
		break;
	case MT6375_PORT_DCP:
	case MT6375_PORT_APPLE_10W ... MT6375_PORT_UNKNOWN_TA:
		*type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	default:
		*type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	}

	return 0;
}

/* VBUS in uV and input current in uA from the MT6375 ADC */
static int mt6375_read_adc(struct iio_channel *chan, int *val)
{
	if (!chan)
		return -ENODATA;

	return iio_read_channel_processed_scale(chan, val, 1000);
}

static int mt6375_set_chg_en(struct mt6375_chg *chg, bool on,
			     const char *reason)
{
	int ret;

	ret = regmap_update_bits(chg->regmap, MT6375_REG_CHG_TOP1,
				 MT6375_CHG_EN, on ? MT6375_CHG_EN : 0);
	if (ret)
		return ret;

	if (on != chg->chg_on || reason != chg->reason)
		dev_info(chg->dev, "charging %s (%s)\n",
			 on ? "enabled" : "disabled", reason);

	chg->chg_on = on;
	chg->reason = reason;
	return 0;
}

struct mt6375_bat_lookup {
	const char *self;
	char name[64];
	bool strong;
	int count;
};

static int mt6375_bat_lookup_fn(struct power_supply *psy, void *data)
{
	struct mt6375_bat_lookup *lk = data;
	size_t i;

	if (psy->desc->type != POWER_SUPPLY_TYPE_BATTERY)
		return 0;

	lk->count++;
	for (i = 0; i < psy->num_supplies; i++) {
		if (!strcmp(psy->supplied_from[i], lk->self)) {
			strscpy(lk->name, psy->desc->name, sizeof(lk->name));
			lk->strong = true;
			return 1;
		}
	}

	if (!lk->name[0])
		strscpy(lk->name, psy->desc->name, sizeof(lk->name));

	return 0;
}

/*
 * The battery is the power supply that names us in power-supplies, or else
 * the only battery in the system. Several unrelated batteries: none.
 */
static struct power_supply *mt6375_find_battery(struct mt6375_chg *chg)
{
	struct mt6375_bat_lookup lk = { .self = chg->psy->desc->name };
	struct power_supply *bat;

	power_supply_for_each_psy(&lk, mt6375_bat_lookup_fn);
	if (!lk.name[0] || (!lk.strong && lk.count != 1))
		return NULL;

	bat = power_supply_get_by_name(lk.name);
	if (bat)
		dev_info(chg->dev, "monitoring battery %s\n", lk.name);

	return bat;
}

static void mt6375_put_battery(void *data)
{
	struct mt6375_chg *chg = data;

	if (chg->bat)
		power_supply_put(chg->bat);
}

static void mt6375_monitor_work(struct work_struct *work)
{
	struct mt6375_chg *chg = container_of(to_delayed_work(work),
					      struct mt6375_chg, monitor);
	union power_supply_propval temp, vbat;
	const char *reason = "battery in range";
	bool allow = true;
	int cv, ret;

	mutex_lock(&chg->lock);

	if (chg->behaviour != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) {
		allow = false;
		reason = "inhibited by user";
		goto apply;
	}

	if (!chg->bat)
		chg->bat = mt6375_find_battery(chg);
	if (!chg->bat) {
		allow = false;
		reason = "no battery fuel gauge";
		goto apply;
	}

	ret = power_supply_get_property(chg->bat, POWER_SUPPLY_PROP_TEMP, &temp);
	if (ret) {
		allow = false;
		reason = "battery temperature unknown";
		goto apply;
	}

	if (chg->temp_blocked) {
		if (temp.intval >= chg->temp_min + MT6375_TEMP_MIN_HYST &&
		    temp.intval <= chg->temp_max - MT6375_TEMP_MAX_HYST)
			chg->temp_blocked = false;
	} else if (temp.intval < chg->temp_min || temp.intval >= chg->temp_max) {
		chg->temp_blocked = true;
	}
	if (chg->temp_blocked) {
		allow = false;
		reason = "battery temperature out of range";
		goto apply;
	}

	ret = power_supply_get_property(chg->bat, POWER_SUPPLY_PROP_VOLTAGE_NOW,
					&vbat);
	if (!ret)
		ret = mt6375_get_cv(chg, &cv);
	if (ret) {
		allow = false;
		reason = "battery voltage unknown";
		goto apply;
	}

	if (chg->ov_blocked) {
		if (vbat.intval < cv - MT6375_OV_RELEASE_UV)
			chg->ov_blocked = false;
	} else if (vbat.intval > cv + MT6375_OV_MARGIN_UV) {
		dev_warn(chg->dev, "battery %d uV above charge voltage %d uV\n",
			 vbat.intval, cv);
		chg->ov_blocked = true;
	}
	if (chg->ov_blocked) {
		allow = false;
		reason = "battery voltage above charge voltage";
	}

apply:
	if (mt6375_set_chg_en(chg, allow, reason))
		dev_err(chg->dev, "failed to switch charging\n");

	mutex_unlock(&chg->lock);

	queue_delayed_work(system_freezable_wq, &chg->monitor,
			   msecs_to_jiffies(MT6375_MONITOR_MS));
}

static int mt6375_chg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct mt6375_chg *chg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		return mt6375_get_online(chg, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		return mt6375_get_status(chg, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		return mt6375_get_charge_type(chg, &val->intval);
	case POWER_SUPPLY_PROP_USB_TYPE:
		return mt6375_get_usb_type(chg, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return mt6375_read_adc(chg->vbus_adc, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return mt6375_read_adc(chg->ibus_adc, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return mt6375_get_ichg(chg, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		if (!chg->configured)
			return -ENODATA;
		val->intval = chg->ichg_max_ua;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return mt6375_get_cv(chg, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		if (!chg->configured)
			return -ENODATA;
		val->intval = chg->cv_max_uv;
		return 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return mt6375_get_aicr(chg, &val->intval);
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		return mt6375_get_mivr(chg, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return mt6375_get_ieoc(chg, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = chg->behaviour;
		return 0;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "MediaTek";
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "MT6375";
		return 0;
	default:
		return -EINVAL;
	}
}

static int mt6375_chg_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct mt6375_chg *chg = power_supply_get_drvdata(psy);
	int ret;

	if (!chg->configured)
		return -EPERM;
	if (psp != POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR && val->intval < 0)
		return -EINVAL;

	mutex_lock(&chg->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = mt6375_set_ichg(chg, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = mt6375_set_cv(chg, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = mt6375_set_aicr(chg, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
		    val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE) {
			ret = -EINVAL;
			break;
		}
		chg->behaviour = val->intval;
		ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&chg->lock);

	mod_delayed_work(system_freezable_wq, &chg->monitor, 0);
	return ret;
}

static int mt6375_chg_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		return 1;
	default:
		return 0;
	}
}

static const enum power_supply_property mt6375_chg_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static const struct power_supply_desc mt6375_chg_psy_desc = {
	.name = "mt6375-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = mt6375_chg_properties,
	.num_properties = ARRAY_SIZE(mt6375_chg_properties),
	.get_property = mt6375_chg_get_property,
	.set_property = mt6375_chg_set_property,
	.property_is_writeable = mt6375_chg_property_is_writeable,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		     BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
			     BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
};

static void mt6375_dump_state(struct mt6375_chg *chg, const char *when)
{
	unsigned int top, wdt, comp, tmr, eoc, stat, port;
	int cv, ichg, aicr, mivr, ieoc, online;
	int vbus = -1000, ibus = -1000;

	if (regmap_read(chg->regmap, MT6375_REG_CHG_TOP1, &top) ||
	    regmap_read(chg->regmap, MT6375_REG_CHG_WDT, &wdt) ||
	    regmap_read(chg->regmap, MT6375_REG_BAT_COMP, &comp) ||
	    regmap_read(chg->regmap, MT6375_REG_CHG_TMR, &tmr) ||
	    regmap_read(chg->regmap, MT6375_REG_CHG_EOC, &eoc) ||
	    regmap_read(chg->regmap, MT6375_REG_CHG_STAT, &stat) ||
	    regmap_read(chg->regmap, MT6375_REG_BC12_STAT, &port) ||
	    mt6375_get_cv(chg, &cv) || mt6375_get_ichg(chg, &ichg) ||
	    mt6375_get_aicr(chg, &aicr) || mt6375_get_mivr(chg, &mivr) ||
	    mt6375_get_ieoc(chg, &ieoc) || mt6375_get_online(chg, &online)) {
		dev_err(chg->dev, "%s: failed to read charger state\n", when);
		return;
	}

	/* -1 when there is no ADC */
	mt6375_read_adc(chg->vbus_adc, &vbus);
	mt6375_read_adc(chg->ibus_adc, &ibus);

	dev_info(chg->dev,
		 "%s: online %d top1 0x%02x cv %d ichg %d aicr %d mivr %d ieoc %d (eoc 0x%02x) tmr 0x%02x wdt 0x%02x comp 0x%02x stat %lu port %lu vbus %d ibus %d\n",
		 when, online, top, cv / 1000, ichg / 1000, aicr / 1000,
		 mivr / 1000, ieoc / 1000, eoc, tmr, wdt, comp,
		 FIELD_GET(MT6375_IC_STAT_MASK, stat),
		 FIELD_GET(MT6375_PORT_STAT_MASK, port), vbus / 1000,
		 ibus / 1000);
}

static int mt6375_get_limits(struct mt6375_chg *chg, u32 *ieoc_ua)
{
	struct power_supply_battery_info *info;
	int ret;

	ret = power_supply_get_battery_info(chg->psy, &info);
	if (ret)
		return ret;

	if (info->constant_charge_voltage_max_uv < MT6375_CV_MIN * 1000 ||
	    info->constant_charge_voltage_max_uv > MT6375_CV_MAX * 1000 ||
	    info->constant_charge_current_max_ua < MT6375_ICHG_MIN * 1000) {
		dev_err(chg->dev, "battery charge voltage/current missing or out of range\n");
		ret = -EINVAL;
		goto out;
	}

	chg->cv_max_uv = info->constant_charge_voltage_max_uv;
	chg->ichg_max_ua = min(info->constant_charge_current_max_ua,
			       MT6375_ICHG_MAX * 1000);
	chg->temp_min = 10 * (info->temp_min > INT_MIN ? info->temp_min :
			      MT6375_DEF_TEMP_MIN);
	chg->temp_max = 10 * (info->temp_max < INT_MAX ? info->temp_max :
			      MT6375_DEF_TEMP_MAX);
	*ieoc_ua = info->charge_term_current_ua > 0 ?
		   info->charge_term_current_ua : MT6375_DEF_IEOC_UA;
out:
	power_supply_put_battery_info(chg->psy, info);
	return ret;
}

static int mt6375_init_charger(struct mt6375_chg *chg, u32 ieoc_ua)
{
	struct regmap *rm = chg->regmap;
	int ret;

	/* Off until the monitor has seen a sane battery */
	ret = mt6375_set_chg_en(chg, false, "probe");
	/* The watchdog is off in the stock firmware as well */
	ret = ret ?: regmap_update_bits(rm, MT6375_REG_CHG_WDT, MT6375_WDT_EN, 0);
	/* No IR compensation: the battery never sees more than CV */
	ret = ret ?: regmap_update_bits(rm, MT6375_REG_BAT_COMP,
					MT6375_IRCMP_MASK, 0);
	ret = ret ?: mt6375_set_cv(chg, chg->cv_max_uv);
	ret = ret ?: mt6375_set_ichg(chg, chg->ichg_max_ua);
	ret = ret ?: regmap_update_bits(rm, MT6375_REG_CHG_AICR, MT6375_ILIM_EN, 0);
	ret = ret ?: mt6375_set_aicr(chg, MT6375_DEF_AICR_UA);
	ret = ret ?: mt6375_write_field(chg, MT6375_REG_CHG_EOC, MT6375_IEOC_MASK,
					mt6375_to_sel(ieoc_ua / 1000,
						      MT6375_IEOC_MIN,
						      MT6375_IEOC_MAX,
						      MT6375_IEOC_STEP,
						      MT6375_IEOC_OFFS));
	ret = ret ?: regmap_update_bits(rm, MT6375_REG_CHG_EOC, MT6375_TE,
					MT6375_TE);
	ret = ret ?: regmap_update_bits(rm, MT6375_REG_CHG_TMR,
					MT6375_TMR_EN | MT6375_TMR_MASK,
					MT6375_TMR_EN |
					FIELD_PREP(MT6375_TMR_MASK,
						   (MT6375_SAFETY_TIMER_H - 5) / 5));
	return ret;
}

static irqreturn_t mt6375_chg_irq(int irq, void *data)
{
	struct mt6375_chg *chg = data;
	int online;

	/* a new attach invalidates the BC1.2 result of the previous one */
	if (mt6375_get_online(chg, &online) || !online)
		WRITE_ONCE(chg->bc12_valid, false);

	power_supply_changed(chg->psy);
	/* Without battery limits the charger is never written */
	if (chg->configured)
		mod_delayed_work(system_freezable_wq, &chg->monitor, 0);
	return IRQ_HANDLED;
}

/* Only reached if something else (the boot loader) ran the detector */
static irqreturn_t mt6375_chg_bc12_irq(int irq, void *data)
{
	struct mt6375_chg *chg = data;

	WRITE_ONCE(chg->bc12_valid, true);
	power_supply_changed(chg->psy);
	return IRQ_HANDLED;
}

static irqreturn_t mt6375_chg_fault_irq(int irq, void *data)
{
	struct mt6375_chg *chg = data;

	dev_warn(chg->dev, "charger fault event (irq %d)\n", irq);
	return mt6375_chg_irq(irq, data);
}

static void mt6375_request_irqs(struct mt6375_chg *chg)
{
	static const struct {
		const char *name;
		irq_handler_t handler;
	} irqs[] = {
		{ "pwr_rdy", mt6375_chg_irq },
		{ "detach", mt6375_chg_irq },
		{ "vbus_ov", mt6375_chg_fault_irq },
		{ "chg_tout", mt6375_chg_fault_irq },
		{ "chg_batov", mt6375_chg_fault_irq },
		{ "bc12_dn", mt6375_chg_bc12_irq },
	};
	struct platform_device *pdev = to_platform_device(chg->dev);
	int i, irq, ret;

	/* Events only speed things up; the monitor polls anyway */
	for (i = 0; i < ARRAY_SIZE(irqs); i++) {
		irq = platform_get_irq_byname_optional(pdev, irqs[i].name);
		if (irq < 0)
			continue;

		ret = devm_request_threaded_irq(chg->dev, irq, NULL,
						irqs[i].handler, IRQF_ONESHOT,
						irqs[i].name, chg);
		if (ret)
			dev_warn(chg->dev, "failed to request irq %s: %d\n",
				 irqs[i].name, ret);
	}
}

static int mt6375_chg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = {};
	struct mt6375_chg *chg;
	u32 ieoc_ua;
	int ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	chg->behaviour = POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
	chg->reason = "boot";

	chg->regmap = dev_get_regmap(dev->parent, NULL);
	if (!chg->regmap)
		return dev_err_probe(dev, -ENODEV, "no parent regmap\n");

	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, mt6375_put_battery, chg);
	if (ret)
		return ret;

	chg->vbus_adc = devm_iio_channel_get(dev, "vbus");
	if (IS_ERR(chg->vbus_adc)) {
		if (PTR_ERR(chg->vbus_adc) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(chg->vbus_adc),
					     "failed to get vbus ADC\n");
		chg->vbus_adc = NULL;
	}

	chg->ibus_adc = devm_iio_channel_get(dev, "ibus");
	if (IS_ERR(chg->ibus_adc)) {
		if (PTR_ERR(chg->ibus_adc) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(chg->ibus_adc),
					     "failed to get ibus ADC\n");
		chg->ibus_adc = NULL;
	}

	/* The boot loader BC1.2 result is about the cable present now */
	if (!mt6375_get_online(chg, &ret) && ret)
		chg->bc12_valid = true;

	mt6375_dump_state(chg, "boot");

	cfg.fwnode = dev_fwnode(dev);
	cfg.drv_data = chg;
	chg->psy = devm_power_supply_register(dev, &mt6375_chg_psy_desc, &cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy),
				     "failed to register power supply\n");

	/* Cancelled before the power supply goes away and the battery is put */
	ret = devm_delayed_work_autocancel(dev, &chg->monitor,
					   mt6375_monitor_work);
	if (ret)
		return ret;

	ret = mt6375_get_limits(chg, &ieoc_ua);
	if (ret) {
		dev_warn(dev, "no usable monitored-battery (%d): reporting only, charger left as the boot loader set it\n",
			 ret);
		/* events only refresh the reported state here */
		mt6375_request_irqs(chg);
		return 0;
	}

	ret = mt6375_init_charger(chg, ieoc_ua);
	if (ret)
		return dev_err_probe(dev, ret, "failed to configure charger\n");

	chg->configured = true;
	mt6375_dump_state(chg, "configured");
	dev_info(dev, "limits: cv %u uV, ichg %u uA, battery %d..%d deci-degC\n",
		 chg->cv_max_uv, chg->ichg_max_ua, chg->temp_min, chg->temp_max);

	mt6375_request_irqs(chg);
	queue_delayed_work(system_freezable_wq, &chg->monitor, 0);
	return 0;
}

static const struct of_device_id mt6375_chg_of_match[] = {
	{ .compatible = "mediatek,mt6375-chg" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6375_chg_of_match);

static struct platform_driver mt6375_chg_driver = {
	.driver = {
		.name = "mt6375-charger",
		.of_match_table = mt6375_chg_of_match,
	},
	.probe = mt6375_chg_probe,
};
module_platform_driver(mt6375_chg_driver);

MODULE_DESCRIPTION("MediaTek MT6375 charger driver");
MODULE_LICENSE("GPL");
