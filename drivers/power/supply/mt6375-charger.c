// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: ShuFan Lee <shufan_lee@richtek.com>
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 *
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
 *  - optional JEITA bands (DT mediatek,jeita-*): reduced charge voltage
 *    and current in the cool and warm bands, nothing outside
 *    operating-range-celsius;
 *  - in system sleep charging is switched off, unless the DT (or the
 *    charge_in_suspend parameter, also at run time) allows it; then it goes on only in the
 *    normal band, at a reduced current, with a fresh battery check taken
 *    just before the sleep, and a boot-time alarm wakes the system every
 *    suspend_check_s seconds to look at the battery again, as the downstream
 *    mtk_charger does (its 10 s ALARM_BOOTTIME poll). A wake-up alarm that
 *    is found unfired well after its expiry switches the feature off for
 *    the boot;
 *  - PE/AICC and the charger watchdog are not used. BC1.2 detection runs
 *    only with the bc12_detect parameter (read at each attach) and a
 *    usb2-phy in the DT: it needs
 *    the USB PHY to hand D+/D- over to the charger (the downstream driver's "BC11" switch,
 *    here a submode of the MediaTek T-PHY), and gives them back right
 *    after. Without it the result of the boot loader BC1.2 run is only
 *    read back, and only until the cable is first removed.
 */

#include <linux/alarmtimer.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mtk-tphy.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/suspend.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#define MT6375_REG_TM_PAS_CODE1	0x107	/* hidden mode: 0x69 opens, 0 closes */
#define MT6375_REG_CHG_TOP1	0x120
#define MT6375_REG_CHG_TOP2	0x121
#define MT6375_REG_CHG_AICR	0x122
#define MT6375_REG_CHG_MIVR	0x123
#define MT6375_REG_CHG_VCHG	0x125
#define MT6375_REG_CHG_ICHG	0x126
#define MT6375_REG_CHG_TMR	0x127
#define MT6375_REG_CHG_EOC	0x128
#define MT6375_REG_CHG_VSYS	0x129
#define MT6375_REG_CHG_WDT	0x12a
#define MT6375_REG_OTG_V	0x131
#define MT6375_REG_OTG_C	0x132
#define MT6375_REG_BAT_COMP	0x133
#define MT6375_REG_CHG_DUMY0	0x135
#define MT6375_REG_CHG_HD_BUCK5	0x140
#define MT6375_REG_CHG_STAT	0x134
#define MT6375_REG_BC12_FUNC	0x150
#define MT6375_REG_BC12_STAT	0x151
#define MT6375_REG_CHG_STAT0	0x1e0

#define MT6375_CHG_EN		BIT(0)
#define MT6375_OTG_EN		BIT(2)
#define MT6375_ILIM_EN		BIT(7)
/* the buck/boost switch-over of the OTG boost (downstream mt6375_set_boost_param) */
#define MT6375_CLK_FREQ_MASK	GENMASK(7, 6)	/* CHG_TOP2 */
#define MT6375_COMP_CLAMP_MASK	GENMASK(1, 0)	/* CHG_DUMY0 */
#define MT6375_BUCK_RAMPOFT_MASK GENMASK(7, 6)	/* CHG_HD_BUCK5 */
#define MT6375_BLEED_DIS_EN	BIT(7)		/* CHG_VSYS */
#define MT6375_OTG_CV_MASK	GENMASK(5, 0)
#define MT6375_OTG_CC_MASK	GENMASK(2, 0)
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
/* BC12_FUNC (downstream F_DCDT_SEL 0/300/600 ms, F_SPEC_TA_EN, F_BC12_EN) */
#define MT6375_DCDT_SEL_MASK	GENMASK(5, 4)
#define MT6375_DCDT_SEL_600MS	2		/* factory DT dcdt-sel = 600 */
#define MT6375_SPEC_TA_EN	BIT(6)		/* factory DT spec-ta-en */
#define MT6375_BC12_EN		BIT(7)

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
#define MT6375_MIVR_MAX		13400
#define MT6375_MIVR_STEP	100
#define MT6375_VBUS_OV_MASK	GENMASK(1, 0)	/* CHG_TOP2: 5.8/6.5/11/14.5 V */

/* Defaults when the battery description leaves them out (stock values) */
#define MT6375_DEF_AICR_UA	500000
#define MT6375_MAX_AICR_UA	3200000
/* input limits by what the port says: BC1.2 only, or the Type-C/PD source */
#define MT6375_AICR_SDP_UA	500000
#define MT6375_AICR_CDP_UA	1500000
#define MT6375_AICR_DCP_UA	1500000
#define MT6375_TCPM_PSY_PREFIX	"tcpm-source-psy-"
/*
 * A PD contract, as the downstream PD algorithm with enable-inductor-protect
 * (GPL mtk_pd.c mtk_pd_input_current_protection(), set in the Titan 2
 * DT): input at most 3 A at 5 V and 1.5 A at 9 V, never above the
 * contract. MIVR above 5 V: contract voltage - 1.2 V
 * (PD_VBUS_IR_DROP_THRESHOLD), at least min-charger-voltage 4.6 V
 * (__mtk_pdc_setup()).
 */
#define MT6375_PD_HV_UV		5500000		/* above: a 9 V class contract */
#define MT6375_AICR_PD_5V_UA	3000000
#define MT6375_AICR_PD_HV_UA	1500000
#define MT6375_PD_IR_DROP_UV	1200000
#define MT6375_PD_MIVR_MIN_UV	4600000
#define MT6375_DEF_IEOC_UA	300000
#define MT6375_DEF_TEMP_MIN	0	/* degC */
#define MT6375_DEF_TEMP_MAX	50	/* degC */
#define MT6375_TEMP_MIN_HYST	60	/* deci-degC */
#define MT6375_TEMP_MAX_HYST	30	/* deci-degC */
#define MT6375_OV_MARGIN_UV	50000
#define MT6375_OV_RELEASE_UV	100000
#define MT6375_SAFETY_TIMER_H	10
#define MT6375_MONITOR_MS	10000
/* JEITA band hysteresis toward the normal band (factory: 6 degC) */
#define MT6375_DEF_JEITA_HYST	3	/* degC */
/* charging in system sleep */
#define MT6375_DEF_SUS_ICHG_UA	500000
#define MT6375_DEF_SUS_TEMP_MAX	40	/* degC */
#define MT6375_SUS_CHECK_MIN_S	10	/* the downstream poll */
#define MT6375_SUS_CHECK_MAX_S	600
#define MT6375_SUS_SLACK_MIN_S	30
#define MT6375_SUS_WAKE_MS	3000	/* awake for the monitor after a check */
/* BC1.2: DCD (600 ms) plus primary and secondary detection */
#define MT6375_BC12_TIMEOUT_MS	2000

static int charge_in_suspend = -1;
module_param(charge_in_suspend, int, 0644);
MODULE_PARM_DESC(charge_in_suspend,
		 "Charge in system sleep: -1 as the DT says (mediatek,charge-in-suspend), 0 never, 1 allowed (normal temperature band, suspend current, periodic wake-up)");

static unsigned int suspend_check_s = 60;
module_param(suspend_check_s, uint, 0644);
MODULE_PARM_DESC(suspend_check_s,
		 "Seconds between battery checks while charging in system sleep (10..600)");

static bool bc12_detect;
module_param(bc12_detect, bool, 0644);
MODULE_PARM_DESC(bc12_detect,
		 "Run BC1.2 port detection on the next attach; needs usb2-phy in the DT (default: only read the boot loader result)");

enum mt6375_band {
	MT6375_BAND_NORMAL,
	MT6375_BAND_COOL,
	MT6375_BAND_WARM,
};

static const char * const mt6375_band_names[] = {
	[MT6375_BAND_NORMAL] = "normal",
	[MT6375_BAND_COOL] = "cool",
	[MT6375_BAND_WARM] = "warm",
};

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
	/* diagnostics only (the boot/configured state lines) */
	struct iio_channel *vbat_adc;
	struct iio_channel *ibat_adc;
	struct iio_channel *tjc_adc;
	struct iio_channel *ts_adc;

	bool configured;
	bool suspended;
	bool stopping;
	bool fault_latched;
	u32 cv_max_uv;
	u32 ichg_max_ua;
	int temp_min;		/* deci-degC */
	int temp_max;		/* deci-degC */
	int behaviour;

	/* the port type found for the present attach (boot loader run or ours) */
	bool bc12_valid;
	unsigned int bc12_port;

	/* BC1.2 detection by this driver (bc12_detect) */
	struct phy *phy;		/* NULL: no detection */
	struct work_struct bc12_work;
	struct completion bc12_done;
	bool bc12_running;		/* D+/D- may be with the charger */
	bool was_online;

	bool temp_blocked;
	bool ov_blocked;
	bool chg_on;
	const char *reason;

	/* what userspace asked for, before the band and sleep limits */
	u32 user_cv_uv;
	u32 user_ichg_ua;

	/* JEITA bands inside operating-range-celsius (deci-degC) */
	bool jeita;
	int jeita_cool;		/* below: cool band */
	int jeita_warm;		/* from here: warm band */
	int jeita_hyst;
	u32 cool_cv_uv, cool_ichg_ua;	/* ichg 0: no charging in the band */
	u32 warm_cv_uv, warm_ichg_ua;
	enum mt6375_band band;

	/* charging in system sleep */
	bool sus_dt;		/* mediatek,charge-in-suspend */
	bool sus_broken;	/* the wake-up timer did not fire: off this boot */
	bool sus_charging;	/* this sleep */
	bool sus_logged;	/* "not in sleep" reasons are logged once */
	bool sus_fired;		/* the alarm of this sleep went off */
	u32 sus_ichg_ua;
	int sus_temp_max;	/* deci-degC */
	ktime_t sus_start;
	struct alarm sus_alarm;
	struct notifier_block pm_nb;

	/* the last monitor run: charging allowed, at this battery temperature */
	bool last_ok;
	int last_temp;		/* deci-degC */
	unsigned long last_check;	/* jiffies */

	/* the input current the port allows: Type-C/PD contract or BC1.2 */
	struct notifier_block psy_nb;
	struct work_struct src_work;
	u32 src_limit_ua;		/* 0: nothing known, the boot limit */
	bool src_pd;
	int src_pd_uv;			/* the PD contract voltage, 0: none */
	u32 aicr_set_ua;		/* the limit last written, 0: boot limit */
	u32 base_mivr_uv;		/* MIVR without a high-voltage contract */
	u32 mivr_set_uv;		/* MIVR last written */

	/* the OTG boost: VBUS out for a host role */
	struct regulator_dev *otg_rdev;
	struct mutex hm_lock;
	unsigned int hm_use;
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
	/* never above what the port negotiated; the boot limit until it did */
	ua = min(ua, chg->src_limit_ua ?: MT6375_DEF_AICR_UA);
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

static int mt6375_set_mivr(struct mt6375_chg *chg, u32 uv)
{
	return mt6375_write_field(chg, MT6375_REG_CHG_MIVR, MT6375_MIVR_MASK,
				  mt6375_to_sel(uv / 1000, MT6375_MIVR_MIN,
						MT6375_MIVR_MAX,
						MT6375_MIVR_STEP, 0));
}

static const u32 mt6375_vbus_ov_mv[] = { 5800, 6500, 11000, 14500 };

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
 * What a BC1.2 run found (the boot loader's, or ours with bc12_detect). A
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

	port = READ_ONCE(chg->bc12_port);
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

/* The port type in BC12_STAT; NOINFO while no detection has finished */
static int mt6375_read_bc12_port(struct mt6375_chg *chg, unsigned int *port)
{
	return mt6375_read_field(chg, MT6375_REG_BC12_STAT,
				 MT6375_PORT_STAT_MASK, port);
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
	unsigned int top;
	int ret;

	ret = regmap_update_bits(chg->regmap, MT6375_REG_CHG_TOP1,
				 MT6375_CHG_EN, on ? MT6375_CHG_EN : 0);
	if (ret)
		return ret;

	ret = regmap_read(chg->regmap, MT6375_REG_CHG_TOP1, &top);
	if (ret)
		return ret;
	if (!!(top & MT6375_CHG_EN) != on)
		return -EIO;

	if (on != chg->chg_on || reason != chg->reason)
		dev_info(chg->dev, "charging %s (%s)\n",
			 on ? "enabled" : "disabled", reason);

	chg->chg_on = on;
	chg->reason = reason;
	return 0;
}

/* The charge voltage and current for the present band, and for sleep */
static void mt6375_targets(struct mt6375_chg *chg, bool sleep, u32 *cv,
			   u32 *ichg)
{
	*cv = chg->user_cv_uv;
	*ichg = chg->user_ichg_ua;

	switch (chg->band) {
	case MT6375_BAND_COOL:
		*cv = min(*cv, chg->cool_cv_uv);
		*ichg = min(*ichg, chg->cool_ichg_ua);
		break;
	case MT6375_BAND_WARM:
		*cv = min(*cv, chg->warm_cv_uv);
		*ichg = min(*ichg, chg->warm_ichg_ua);
		break;
	default:
		break;
	}

	if (sleep)
		*ichg = min(*ichg, chg->sus_ichg_ua);
}

/*
 * Program the targets and read them back: the fields round down, so a
 * read-back above a target means the write did not take.
 */
static int mt6375_apply_targets(struct mt6375_chg *chg, bool sleep)
{
	u32 cv, ichg;
	int got_cv, got_ichg, ret;

	mt6375_targets(chg, sleep, &cv, &ichg);
	if (!ichg)		/* a band without charging: CHG_EN stays off */
		return 0;

	ret = mt6375_set_cv(chg, cv);
	ret = ret ?: mt6375_set_ichg(chg, ichg);
	ret = ret ?: mt6375_get_cv(chg, &got_cv);
	ret = ret ?: mt6375_get_ichg(chg, &got_ichg);
	if (ret)
		return ret;
	if (got_cv > cv || got_ichg > ichg) {
		dev_err(chg->dev, "cv %d uV / ichg %d uA read back above %u / %u\n",
			got_cv, got_ichg, cv, ichg);
		return -EIO;
	}
	return 0;
}

/* The JEITA band of a battery temperature, with hysteresis toward normal */
static void mt6375_update_band(struct mt6375_chg *chg, int temp)
{
	enum mt6375_band band = chg->band;
	u32 cv, ichg;

	if (!chg->jeita) {
		chg->band = MT6375_BAND_NORMAL;
		return;
	}

	if (band == MT6375_BAND_COOL &&
	    temp >= chg->jeita_cool + chg->jeita_hyst)
		band = MT6375_BAND_NORMAL;
	else if (band == MT6375_BAND_WARM &&
		 temp < chg->jeita_warm - chg->jeita_hyst)
		band = MT6375_BAND_NORMAL;

	if (temp < chg->jeita_cool)
		band = MT6375_BAND_COOL;
	else if (temp >= chg->jeita_warm)
		band = MT6375_BAND_WARM;

	if (band == chg->band)
		return;

	chg->band = band;
	mt6375_targets(chg, false, &cv, &ichg);
	dev_info(chg->dev, "battery %d deci-degC: %s band, cv %u uV, ichg %u uA\n",
		 temp, mt6375_band_names[band], cv, ichg);
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
	unsigned int stat;

	mutex_lock(&chg->lock);

	if (chg->suspended || chg->stopping) {
		mutex_unlock(&chg->lock);
		return;
	}

	ret = mt6375_read_field(chg, MT6375_REG_CHG_STAT,
				MT6375_IC_STAT_MASK, &stat);
	if (!ret && stat == MT6375_STAT_FAULT)
		chg->fault_latched = true;
	if (ret || chg->fault_latched) {
		allow = false;
		reason = "charger fault or unreadable status";
		goto apply;
	}

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
	chg->last_temp = temp.intval;

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

	/* the band's charge voltage and current before charging is allowed */
	mt6375_update_band(chg, temp.intval);
	if (mt6375_apply_targets(chg, false)) {
		chg->fault_latched = true;
		allow = false;
		reason = "cannot set charge voltage/current";
		goto apply;
	}
	if ((chg->band == MT6375_BAND_COOL && !chg->cool_ichg_ua) ||
	    (chg->band == MT6375_BAND_WARM && !chg->warm_ichg_ua)) {
		allow = false;
		reason = "no charging in this temperature band";
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
	chg->last_ok = false;
	if (mt6375_set_chg_en(chg, allow, reason)) {
		chg->fault_latched = true;
		dev_err(chg->dev, "failed to switch charging; fault latched\n");
		if (mt6375_set_chg_en(chg, false, "switch failure"))
			dev_err(chg->dev, "cannot confirm charging is disabled\n");
	} else {
		chg->last_ok = allow;
	}
	chg->last_check = jiffies;

	queue_delayed_work(system_freezable_wq, &chg->monitor,
			   msecs_to_jiffies(MT6375_MONITOR_MS));
	mutex_unlock(&chg->lock);
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
	if (chg->stopping || chg->suspended) {
		mutex_unlock(&chg->lock);
		return -EBUSY;
	}
	switch (psp) {
	/* the band limits still apply on top (and the battery maximum) */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		chg->user_ichg_ua = clamp_t(u32, val->intval,
					    MT6375_ICHG_MIN * 1000,
					    chg->ichg_max_ua);
		ret = mt6375_apply_targets(chg, false);
		if (ret)
			chg->fault_latched = true;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		chg->user_cv_uv = clamp_t(u32, val->intval,
					  MT6375_CV_MIN * 1000,
					  chg->cv_max_uv);
		ret = mt6375_apply_targets(chg, false);
		if (ret)
			chg->fault_latched = true;
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
		ret = val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE ?
			mt6375_set_chg_en(chg, false, "inhibited by user") : 0;
		if (ret)
			chg->fault_latched = true;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (!chg->stopping && !chg->suspended)
		mod_delayed_work(system_freezable_wq, &chg->monitor, 0);
	mutex_unlock(&chg->lock);
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
	unsigned int top, top2, wdt, comp, tmr, eoc, stat, port;
	int cv, ichg, aicr, mivr, ieoc, online;
	int vbus = -1000, ibus = -1000;

	if (regmap_read(chg->regmap, MT6375_REG_CHG_TOP1, &top) ||
	    regmap_read(chg->regmap, MT6375_REG_CHG_TOP2, &top2) ||
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
		 "%s: online %d top1 0x%02x cv %d ichg %d aicr %d mivr %d vbus_ov %u ieoc %d (eoc 0x%02x) tmr 0x%02x wdt 0x%02x comp 0x%02x stat %lu port %lu vbus %d ibus %d\n",
		 when, online, top, cv / 1000, ichg / 1000, aicr / 1000,
		 mivr / 1000,
		 mt6375_vbus_ov_mv[FIELD_GET(MT6375_VBUS_OV_MASK, top2)],
		 ieoc / 1000, eoc, tmr, wdt, comp,
		 FIELD_GET(MT6375_IC_STAT_MASK, stat),
		 FIELD_GET(MT6375_PORT_STAT_MASK, port), vbus / 1000,
		 ibus / 1000);

	/* the battery side as the charger sees it, when the DT names them */
	if (chg->vbat_adc || chg->ibat_adc || chg->tjc_adc || chg->ts_adc) {
		int vbat = -1000, ibat = -1000, tjc = -1000000, ts = -1000;

		mt6375_read_adc(chg->vbat_adc, &vbat);
		mt6375_read_adc(chg->ibat_adc, &ibat);
		mt6375_read_adc(chg->ts_adc, &ts);
		/* temperatures come in milli-degC, scaled: micro-degC */
		mt6375_read_adc(chg->tjc_adc, &tjc);
		dev_info(chg->dev, "%s: adc vbat %d mV ibat %d mA ts %d mV temp_jc %d degC\n",
			 when, vbat / 1000, ibat / 1000, ts / 1000,
			 tjc / 1000000);
	}
}

static struct iio_channel *mt6375_get_adc(struct device *dev,
					  const char *name, bool *defer)
{
	struct iio_channel *chan = devm_iio_channel_get(dev, name);

	if (!IS_ERR(chan))
		return chan;
	if (PTR_ERR(chan) == -EPROBE_DEFER)
		*defer = true;
	return NULL;
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

/*
 * A band current: 0 means no charging in the band; the field cannot go
 * below 300 mA, so smaller requests mean no charging too. Never above the
 * battery maximum.
 */
static u32 mt6375_band_ichg(struct mt6375_chg *chg, const char *prop)
{
	u32 ua;

	if (device_property_read_u32(chg->dev, prop, &ua))
		return chg->ichg_max_ua;
	if (ua < MT6375_ICHG_MIN * 1000) {
		if (ua)
			dev_warn(chg->dev, "%s %u below %u: no charging in the band\n",
				 prop, ua, MT6375_ICHG_MIN * 1000);
		return 0;
	}
	return min(ua, chg->ichg_max_ua);
}

/* A band voltage below the field minimum is refused: the band charges not */
static u32 mt6375_band_cv(struct mt6375_chg *chg, const char *prop, u32 *ichg)
{
	u32 uv;

	if (device_property_read_u32(chg->dev, prop, &uv))
		return chg->cv_max_uv;
	if (uv < MT6375_CV_MIN * 1000) {
		dev_warn(chg->dev, "%s %u below %u: no charging in the band\n",
			 prop, uv, MT6375_CV_MIN * 1000);
		*ichg = 0;
		return MT6375_CV_MIN * 1000;
	}
	return min(uv, chg->cv_max_uv);
}

/* mediatek,jeita-<band>-charge-{voltage-microvolt,current-microamp} */
static void mt6375_band_limits(struct mt6375_chg *chg, const char *band,
			       u32 *cv, u32 *ichg)
{
	char prop[64];

	snprintf(prop, sizeof(prop), "mediatek,jeita-%s-charge-current-microamp",
		 band);
	*ichg = mt6375_band_ichg(chg, prop);
	snprintf(prop, sizeof(prop), "mediatek,jeita-%s-charge-voltage-microvolt",
		 band);
	*cv = mt6375_band_cv(chg, prop, ichg);
}

/* Charging in sleep asked for: the parameter, or else the DT */
static bool mt6375_sus_wanted(struct mt6375_chg *chg)
{
	int param = READ_ONCE(charge_in_suspend);

	if (!chg->sus_ichg_ua)
		return false;
	return param < 0 ? chg->sus_dt : param > 0;
}

/* JEITA bands and charging in sleep, from the charger node */
static void mt6375_get_policy(struct mt6375_chg *chg)
{
	struct device *dev = chg->dev;
	u32 cool, warm, hyst = MT6375_DEF_JEITA_HYST, val;

	chg->user_cv_uv = chg->cv_max_uv;
	chg->user_ichg_ua = chg->ichg_max_ua;
	chg->band = MT6375_BAND_NORMAL;

	if (!device_property_read_u32(dev, "mediatek,jeita-cool-celsius", &cool) &&
	    !device_property_read_u32(dev, "mediatek,jeita-warm-celsius", &warm)) {
		device_property_read_u32(dev, "mediatek,jeita-hysteresis-celsius",
					 &hyst);
		/* the properties are s32 cells */
		chg->jeita_cool = 10 * (s32)cool;
		chg->jeita_warm = 10 * (s32)warm;
		chg->jeita_hyst = 10 * hyst;
		if (hyst < 1 || hyst > 10 ||
		    chg->jeita_cool < chg->temp_min ||
		    chg->jeita_warm > chg->temp_max ||
		    chg->jeita_warm - chg->jeita_cool <= 2 * chg->jeita_hyst) {
			dev_warn(dev, "JEITA %d/%d/%u degC invalid: ignored\n",
				 (s32)cool, (s32)warm, hyst);
		} else {
			mt6375_band_limits(chg, "cool", &chg->cool_cv_uv,
					   &chg->cool_ichg_ua);
			mt6375_band_limits(chg, "warm", &chg->warm_cv_uv,
					   &chg->warm_ichg_ua);
			chg->jeita = true;
			dev_info(dev, "JEITA: cool < %d degC (cv %u ichg %u), warm >= %d degC (cv %u ichg %u), hysteresis %u degC\n",
				 (s32)cool, chg->cool_cv_uv, chg->cool_ichg_ua,
				 (s32)warm, chg->warm_cv_uv, chg->warm_ichg_ua,
				 hyst);
		}
	}

	chg->sus_dt = device_property_read_bool(dev, "mediatek,charge-in-suspend");
	chg->sus_ichg_ua = MT6375_DEF_SUS_ICHG_UA;
	if (!device_property_read_u32(dev, "mediatek,suspend-charge-current-microamp",
				      &val))
		chg->sus_ichg_ua = val;
	chg->sus_ichg_ua = min(chg->sus_ichg_ua, chg->ichg_max_ua);
	chg->sus_temp_max = 10 * MT6375_DEF_SUS_TEMP_MAX;
	if (!device_property_read_u32(dev, "mediatek,suspend-max-celsius", &val))
		chg->sus_temp_max = 10 * (s32)val;
	chg->sus_temp_max = min(chg->sus_temp_max, chg->temp_max);

	if (chg->sus_ichg_ua < MT6375_ICHG_MIN * 1000) {
		dev_warn(dev, "suspend charge current %u below %u: no charging in sleep\n",
			 chg->sus_ichg_ua, MT6375_ICHG_MIN * 1000);
		chg->sus_ichg_ua = 0;
	}
	/* the parameter can change later and the wake-up RTC come later */
	dev_info(dev, "charging in sleep %s (DT %d, charge_in_suspend %d): up to %u uA, battery below %d deci-degC\n",
		 mt6375_sus_wanted(chg) ? "on" : "off", chg->sus_dt,
		 READ_ONCE(charge_in_suspend), chg->sus_ichg_ua,
		 chg->sus_temp_max);
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
	bool was_online;
	int online;

	/* a new attach invalidates the BC1.2 result of the previous one */
	if (mt6375_get_online(chg, &online))
		online = 0;
	if (!online)
		WRITE_ONCE(chg->bc12_valid, false);

	was_online = xchg(&chg->was_online, !!online);
	if (chg->phy) {
		if (!online && READ_ONCE(chg->bc12_running))
			complete(&chg->bc12_done);	/* stop waiting */
		else if (online && !was_online && READ_ONCE(bc12_detect))
			schedule_work(&chg->bc12_work);
	}

	power_supply_changed(chg->psy);
	schedule_work(&chg->src_work);
	/* Without battery limits the charger is never written */
	mutex_lock(&chg->lock);
	if (chg->configured && !chg->stopping && !chg->suspended)
		mod_delayed_work(system_freezable_wq, &chg->monitor, 0);
	mutex_unlock(&chg->lock);
	return IRQ_HANDLED;
}

/* Our detection (bc12_detect) or one something else (the boot loader) ran */
static irqreturn_t mt6375_chg_bc12_irq(int irq, void *data)
{
	struct mt6375_chg *chg = data;
	unsigned int port;
	int online;

	if (READ_ONCE(chg->bc12_running)) {
		complete(&chg->bc12_done);
		return IRQ_HANDLED;
	}

	if (mt6375_get_online(chg, &online) || !online ||
	    mt6375_read_bc12_port(chg, &port) || port == MT6375_PORT_NOINFO)
		return IRQ_HANDLED;

	WRITE_ONCE(chg->bc12_port, port);
	WRITE_ONCE(chg->bc12_valid, true);
	power_supply_changed(chg->psy);
	schedule_work(&chg->src_work);
	return IRQ_HANDLED;
}

static irqreturn_t mt6375_chg_fault_irq(int irq, void *data)
{
	struct mt6375_chg *chg = data;

	dev_warn(chg->dev, "charger fault event (irq %d)\n", irq);
	mutex_lock(&chg->lock);
	chg->fault_latched = true;
	if (chg->configured &&
	    mt6375_set_chg_en(chg, false, "latched charger fault"))
		dev_err(chg->dev, "cannot disable charging after fault\n");
	mutex_unlock(&chg->lock);
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


/* ---- what the port allows: Type-C / PD contract, else BC1.2 ------------ */

static int mt6375_psy_notify(struct notifier_block *nb, unsigned long evt,
			     void *data)
{
	struct mt6375_chg *chg = container_of(nb, struct mt6375_chg, psy_nb);
	struct power_supply *psy = data;

	if (evt != PSY_EVENT_PROP_CHANGED || !psy || !psy->desc ||
	    strncmp(psy->desc->name, MT6375_TCPM_PSY_PREFIX,
		    strlen(MT6375_TCPM_PSY_PREFIX)))
		return NOTIFY_DONE;
	schedule_work(&chg->src_work);
	return NOTIFY_OK;
}

struct mt6375_tcpm_find {
	struct device *mfd;
	struct power_supply *psy;
};

static int mt6375_match_tcpm_psy(struct power_supply *psy, void *data)
{
	struct mt6375_tcpm_find *f = data;

	if (!psy->desc || !psy->desc->name ||
	    strncmp(psy->desc->name, MT6375_TCPM_PSY_PREFIX,
		    strlen(MT6375_TCPM_PSY_PREFIX)))
		return 0;
	/* the port manager of the TCPC cell of this MT6375 */
	if (!psy->dev.parent || psy->dev.parent->parent != f->mfd)
		return 0;
	get_device(&psy->dev);
	f->psy = psy;
	return 1;
}

/*
 * The port manager names its supply after the TCPC device, and the MFD
 * cells get automatic ids ("mt6375-tcpc.N.auto"): look it up by prefix
 * and parent, not by a fixed name. Put with put_device().
 */
static struct power_supply *mt6375_find_tcpm_psy(struct mt6375_chg *chg)
{
	struct mt6375_tcpm_find f = { .mfd = chg->dev->parent };

	if (!f.mfd)
		return NULL;
	power_supply_for_each_psy(&f, mt6375_match_tcpm_psy);
	return f.psy;
}

static void mt6375_src_work(struct work_struct *work)
{
	struct mt6375_chg *chg = container_of(work, struct mt6375_chg, src_work);
	union power_supply_propval online = {}, cur = {}, type = {}, volt = {};
	struct power_supply *psy = mt6375_find_tcpm_psy(chg);
	u32 limit = 0, mivr;
	const char *how = "boot limit";
	char pd_how[64];
	bool pd = false;
	int pd_uv = 0;
	unsigned int top2;
	int usb_type, ret;

	if (psy) {
		if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &online) &&
		    online.intval &&
		    !power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &cur) &&
		    cur.intval > 0) {
			limit = cur.intval;
			power_supply_get_property(psy, POWER_SUPPLY_PROP_USB_TYPE, &type);
			pd = type.intval == POWER_SUPPLY_USB_TYPE_PD ||
			     type.intval == POWER_SUPPLY_USB_TYPE_PD_PPS;
			how = pd ? "PD contract" : "Type-C current";
		}
		if (pd) {
			/* the contract voltage; unknown: treat as 9 V class */
			if (power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
						      &volt) || volt.intval <= 0)
				volt.intval = MT6375_PD_HV_UV + 1;
			pd_uv = volt.intval;
			snprintf(pd_how, sizeof(pd_how), "PD contract %d mV / %d mA",
				 pd_uv / 1000, limit / 1000);
			how = pd_how;
			if (pd_uv > MT6375_PD_HV_UV)
				limit = min_t(u32, limit, MT6375_AICR_PD_HV_UA);
			else
				limit = min_t(u32, limit, MT6375_AICR_PD_5V_UA);
		}
		put_device(&psy->dev);
	}
	if (!limit) {
		/* nothing from the port manager: the BC1.2 result */
		mutex_lock(&chg->lock);
		ret = mt6375_get_usb_type(chg, &usb_type);
		mutex_unlock(&chg->lock);
		if (!ret) {
			switch (usb_type) {
			case POWER_SUPPLY_USB_TYPE_CDP:
				limit = MT6375_AICR_CDP_UA;
				how = "BC1.2 CDP";
				break;
			case POWER_SUPPLY_USB_TYPE_DCP:
				limit = MT6375_AICR_DCP_UA;
				how = "BC1.2 DCP";
				break;
			case POWER_SUPPLY_USB_TYPE_SDP:
				limit = MT6375_AICR_SDP_UA;
				how = "BC1.2 SDP";
				break;
			default:
				break;
			}
		}
	}

	mutex_lock(&chg->lock);
	chg->src_limit_ua = limit;
	chg->src_pd = pd;
	chg->src_pd_uv = pd ? pd_uv : 0;
	/* above 5 V the downstream PD setup: contract - 1.2 V, at least 4.6 V */
	mivr = chg->base_mivr_uv;
	if (pd && pd_uv > MT6375_PD_HV_UV)
		mivr = max_t(u32, MT6375_PD_MIVR_MIN_UV,
			     pd_uv - MT6375_PD_IR_DROP_UV);
	if (chg->configured && !chg->stopping && !chg->suspended) {
		/*
		 * Going down (contract lost or back to 5 V): MIVR first, so
		 * the input never sits at a 9 V MIVR on 5 V; going up: AICR
		 * at the new limit, then MIVR.
		 */
		if (mivr < chg->mivr_set_uv) {
			ret = mt6375_set_mivr(chg, mivr);
			dev_info(chg->dev, "MIVR %u mV (%s)%s\n", mivr / 1000,
				 pd ? "PD contract at 5 V" : "no PD contract",
				 ret ? ", FAILED to set" : "");
			if (!ret)
				chg->mivr_set_uv = mivr;
		}
		if (limit != chg->aicr_set_ua) {
			ret = mt6375_set_aicr(chg, limit ?: MT6375_DEF_AICR_UA);
			dev_info(chg->dev, "input current limit %u mA (%s)%s\n",
				 (limit ?: MT6375_DEF_AICR_UA) / 1000, how,
				 ret ? ", FAILED to set" : "");
			if (!ret)
				chg->aicr_set_ua = limit;
		}
		if (mivr > chg->mivr_set_uv) {
			ret = mt6375_set_mivr(chg, mivr);
			dev_info(chg->dev, "MIVR %u mV (PD contract %d mV)%s\n",
				 mivr / 1000, pd_uv / 1000,
				 ret ? ", FAILED to set" : "");
			if (!ret)
				chg->mivr_set_uv = mivr;
		}
		/* the hardware VBUS OVP is left as found (the downstream driver sets 14.5 V) */
		if (pd && !regmap_read(chg->regmap, MT6375_REG_CHG_TOP2, &top2) &&
		    pd_uv / 1000 >= mt6375_vbus_ov_mv[FIELD_GET(MT6375_VBUS_OV_MASK, top2)])
			dev_warn(chg->dev, "PD contract %d mV at or above the VBUS OVP %u mV\n",
				 pd_uv / 1000,
				 mt6375_vbus_ov_mv[FIELD_GET(MT6375_VBUS_OV_MASK, top2)]);
	}
	mutex_unlock(&chg->lock);
	power_supply_changed(chg->psy);
}

/* ---- BC1.2 port detection by the charger (bc12_detect) ----------------- */

static const char *mt6375_port_name(unsigned int port)
{
	switch (port) {
	case MT6375_PORT_SDP:
		return "SDP";
	case MT6375_PORT_CDP:
		return "CDP";
	case MT6375_PORT_DCP:
		return "DCP";
	case MT6375_PORT_APPLE_10W:
	case MT6375_PORT_APPLE_5W:
	case MT6375_PORT_APPLE_12W:
		return "Apple charger (DCP)";
	case MT6375_PORT_SAMSUNG:
		return "Samsung charger (DCP)";
	case MT6375_PORT_UNKNOWN_TA:
		return "non-standard charger (DCP)";
	default:
		return "no result";
	}
}

/* D+/D- to the charger (true) or back to the USB transceiver */
static int mt6375_bc12_switch(struct mt6375_chg *chg, bool to_charger)
{
	return phy_set_mode_ext(chg->phy, phy_get_mode(chg->phy),
				to_charger ? MTK_TPHY_BC11_SW_SET :
					     MTK_TPHY_BC11_SW_CLR);
}

/*
 * As the downstream flow on an attach: D+/D- to the charger, the detector on
 * (BC12_EN, with the factory DCD timeout and special-charger detection),
 * wait for bc12_dn, read the port type, detector off, D+/D- back to USB.
 * The lines go back to USB on every path; while they are with the charger
 * the USB link is down (a host sees a short disconnect).
 */
static void mt6375_bc12_work(struct work_struct *work)
{
	struct mt6375_chg *chg = container_of(work, struct mt6375_chg,
					      bc12_work);
	unsigned int port = MT6375_PORT_NOINFO, top;
	bool event = false;
	int online, ret, ret2;

	/* the detector is a charger write: not in the reporting-only mode */
	if (!READ_ONCE(bc12_detect) || !chg->configured ||
	    mt6375_get_online(chg, &online) || !online)
		return;
	/* VBUS is ours (host role): there is no charger port to detect */
	if (regmap_read(chg->regmap, MT6375_REG_CHG_TOP1, &top) ||
	    (top & MT6375_OTG_EN))
		return;

	reinit_completion(&chg->bc12_done);
	WRITE_ONCE(chg->bc12_running, true);

	ret = mt6375_bc12_switch(chg, true);
	if (ret) {
		dev_warn(chg->dev, "BC1.2: USB PHY does not hand over D+/D-: %d\n",
			 ret);
		goto restore;
	}

	/* the detector starts on a 0 -> 1 of BC12_EN */
	ret = regmap_update_bits(chg->regmap, MT6375_REG_BC12_FUNC,
				 MT6375_DCDT_SEL_MASK | MT6375_SPEC_TA_EN |
				 MT6375_BC12_EN,
				 FIELD_PREP(MT6375_DCDT_SEL_MASK,
					    MT6375_DCDT_SEL_600MS) |
				 MT6375_SPEC_TA_EN);
	ret = ret ?: regmap_update_bits(chg->regmap, MT6375_REG_BC12_FUNC,
					MT6375_BC12_EN, MT6375_BC12_EN);
	if (ret) {
		dev_warn(chg->dev, "BC1.2: cannot start the detector: %d\n", ret);
		goto restore;
	}

	/* bc12_dn, or a detach; without the event the status is polled once */
	event = wait_for_completion_timeout(&chg->bc12_done,
					    msecs_to_jiffies(MT6375_BC12_TIMEOUT_MS));
	ret = mt6375_read_bc12_port(chg, &port);

restore:
	ret2 = regmap_update_bits(chg->regmap, MT6375_REG_BC12_FUNC,
				  MT6375_BC12_EN, 0);
	if (ret2)
		dev_err(chg->dev, "BC1.2: cannot stop the detector: %d\n", ret2);
	ret2 = mt6375_bc12_switch(chg, false);
	if (ret2) {
		/* once more: USB data must not stay cut off */
		ret2 = mt6375_bc12_switch(chg, false);
		if (ret2)
			dev_err(chg->dev, "BC1.2: D+/D- NOT given back to USB: %d\n",
				ret2);
	}
	WRITE_ONCE(chg->bc12_running, false);

	if (ret)
		return;
	if (mt6375_get_online(chg, &online) || !online) {
		dev_info(chg->dev, "BC1.2: cable removed during detection\n");
		return;
	}
	if (port == MT6375_PORT_NOINFO) {
		dev_warn(chg->dev, "BC1.2: no result in %u ms\n",
			 MT6375_BC12_TIMEOUT_MS);
		return;
	}

	dev_info(chg->dev, "BC1.2: %s%s\n", mt6375_port_name(port),
		 event ? "" : " (no done event, status polled)");
	WRITE_ONCE(chg->bc12_port, port);
	WRITE_ONCE(chg->bc12_valid, true);
	power_supply_changed(chg->psy);
	schedule_work(&chg->src_work);
}

/* The USB PHY from the DT; without it bc12_detect has no effect */
static int mt6375_get_bc12_phy(struct mt6375_chg *chg)
{
	struct device *dev = chg->dev;
	struct phy *phy;
	int ret;

	phy = devm_phy_optional_get(dev, "usb2-phy");
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy), "cannot get usb2-phy\n");
	if (!phy) {
		dev_info(dev, "no usb2-phy (phys/phy-names) in the DT: no BC1.2 detection, bc12_detect ignored\n");
		return 0;
	}

	init_completion(&chg->bc12_done);
	/* the work gives D+/D- back before it ends; cancelled before the put */
	ret = devm_work_autocancel(dev, &chg->bc12_work, mt6375_bc12_work);
	if (ret)
		return ret;
	chg->phy = phy;
	dev_info(dev, "BC1.2 detection on attach %s (bc12_detect)\n",
		 READ_ONCE(bc12_detect) ? "on" : "off");
	return 0;
}

/* ---- the OTG boost: VBUS for a host role (downstream mt6375_chg_otg_rdesc) ---- */

static int mt6375_hidden_mode(struct mt6375_chg *chg, bool open)
{
	int ret = 0;

	mutex_lock(&chg->hm_lock);
	if (open) {
		if (!chg->hm_use)
			ret = regmap_write(chg->regmap, MT6375_REG_TM_PAS_CODE1, 0x69);
		if (!ret)
			chg->hm_use++;
	} else if (chg->hm_use) {
		if (chg->hm_use == 1)
			ret = regmap_write(chg->regmap, MT6375_REG_TM_PAS_CODE1, 0);
		if (!ret)
			chg->hm_use--;
	}
	mutex_unlock(&chg->hm_lock);
	return ret;
}

/* the converter's clock, clamp, ramp and bleed differ between buck and boost */
static int mt6375_set_boost_param(struct mt6375_chg *chg, bool boost)
{
	static const struct {
		unsigned int reg, mask;
		u8 buck, bst;
	} p[] = {
		{ MT6375_REG_CHG_TOP2, MT6375_CLK_FREQ_MASK, 1, 0 },
		{ MT6375_REG_CHG_DUMY0, MT6375_COMP_CLAMP_MASK, 0, 3 },
		{ MT6375_REG_CHG_HD_BUCK5, MT6375_BUCK_RAMPOFT_MASK, 1, 3 },
		{ MT6375_REG_CHG_VSYS, MT6375_BLEED_DIS_EN, 1, 0 },
	};
	int i, ret;

	ret = mt6375_hidden_mode(chg, true);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(p); i++) {
		ret = regmap_update_bits(chg->regmap, p[i].reg, p[i].mask,
					 (boost ? p[i].bst : p[i].buck) <<
					 (ffs(p[i].mask) - 1));
		if (ret) {
			dev_err(chg->dev, "boost parameter 0x%x failed: %d\n",
				p[i].reg, ret);
			while (--i >= 0)
				regmap_update_bits(chg->regmap, p[i].reg, p[i].mask,
						   (boost ? p[i].buck : p[i].bst) <<
						   (ffs(p[i].mask) - 1));
			break;
		}
	}
	mt6375_hidden_mode(chg, false);
	return ret;
}

static int mt6375_otg_enable(struct regulator_dev *rdev)
{
	struct mt6375_chg *chg = rdev_get_drvdata(rdev);
	int ret;

	ret = mt6375_set_boost_param(chg, true);
	if (ret)
		return ret;
	ret = regulator_enable_regmap(rdev);
	if (ret)
		mt6375_set_boost_param(chg, false);
	else
		dev_info(chg->dev, "OTG boost on\n");
	return ret;
}

static int mt6375_otg_disable(struct regulator_dev *rdev)
{
	struct mt6375_chg *chg = rdev_get_drvdata(rdev);
	int ret;

	ret = regulator_disable_regmap(rdev);
	if (ret)
		return ret;
	mt6375_set_boost_param(chg, false);
	dev_info(chg->dev, "OTG boost off\n");
	return 0;
}

static const struct regulator_ops mt6375_otg_ops = {
	.enable = mt6375_otg_enable,
	.disable = mt6375_otg_disable,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_current_limit = regulator_set_current_limit_regmap,
	.get_current_limit = regulator_get_current_limit_regmap,
};

static const unsigned int mt6375_otg_cc_ua[] = {
	500000, 700000, 1100000, 1300000, 1800000, 2100000, 2400000,
};

/* OTG_V: 4.85 V + 25 mV per step from code 20 (downstream: 47 steps) */
static const struct regulator_desc mt6375_otg_desc = {
	.name = "usb-otg-vbus",
	.of_match = "otg",
	.ops = &mt6375_otg_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.min_uV = 4850000,
	.uV_step = 25000,
	.n_voltages = 47,
	.linear_min_sel = 20,
	.curr_table = mt6375_otg_cc_ua,
	.n_current_limits = ARRAY_SIZE(mt6375_otg_cc_ua),
	.csel_reg = MT6375_REG_OTG_C,
	.csel_mask = MT6375_OTG_CC_MASK,
	.vsel_reg = MT6375_REG_OTG_V,
	.vsel_mask = MT6375_OTG_CV_MASK,
	.enable_reg = MT6375_REG_CHG_TOP1,
	.enable_mask = MT6375_OTG_EN,
};

/* ---- charging in system sleep (charge_in_suspend) ---------------------- */

static unsigned int mt6375_sus_check_s(void)
{
	return clamp(READ_ONCE(suspend_check_s), MT6375_SUS_CHECK_MIN_S,
		     MT6375_SUS_CHECK_MAX_S);
}

/* Runs in the alarm (hrtimer) context, also right after a resume */
static void mt6375_sus_alarm_fn(struct alarm *alarm, ktime_t now)
{
	struct mt6375_chg *chg = container_of(alarm, struct mt6375_chg,
					      sus_alarm);

	WRITE_ONCE(chg->sus_fired, true);
	/* awake long enough for the monitor; aborts a sleep on its way */
	pm_wakeup_dev_event(chg->dev, MT6375_SUS_WAKE_MS, true);
}

/*
 * Before tasks freeze: a fresh monitor run, so the sleep decision rests
 * on a battery reading of now (the fuel gauge may be asleep by the time
 * the charger suspends).
 */
static int mt6375_pm_notify(struct notifier_block *nb, unsigned long evt,
			    void *data)
{
	struct mt6375_chg *chg = container_of(nb, struct mt6375_chg, pm_nb);

	if (evt != PM_SUSPEND_PREPARE || !mt6375_sus_wanted(chg))
		return NOTIFY_DONE;

	mutex_lock(&chg->lock);
	if (chg->configured && !chg->stopping && !chg->suspended)
		mod_delayed_work(system_freezable_wq, &chg->monitor, 0);
	mutex_unlock(&chg->lock);
	flush_delayed_work(&chg->monitor);
	return NOTIFY_OK;
}

static void mt6375_unreg_pm_notifier(void *data)
{
	unregister_pm_notifier(data);
}

static void mt6375_cancel_alarm(void *data)
{
	struct mt6375_chg *chg = data;

	alarm_cancel(&chg->sus_alarm);
}

/* Why charging cannot go on in this sleep, or NULL; under chg->lock */
static const char *mt6375_sus_veto(struct mt6375_chg *chg)
{
	int online;

	if (!mt6375_sus_wanted(chg))
		return "not enabled";
	if (chg->sus_broken)
		return "the wake-up alarm did not fire earlier";
	if (!alarmtimer_get_rtcdev())
		return "no wake-up RTC for the alarm timer";
	if (chg->fault_latched)
		return "charger fault";
	if (chg->behaviour != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO)
		return "inhibited by user";
	if (!chg->last_ok || !chg->chg_on ||
	    time_after(jiffies, chg->last_check +
				msecs_to_jiffies(2 * MT6375_MONITOR_MS)))
		return "no fresh battery check allowing charging";
	if (chg->band != MT6375_BAND_NORMAL)
		return "battery outside the normal temperature band";
	if (chg->last_temp >= chg->sus_temp_max)
		return "battery too warm for charging in sleep";
	if (mt6375_get_online(chg, &online) || !online)
		return "no input";
	return NULL;
}

/* Charging on at the sleep current and the alarm armed, or an error */
static int mt6375_sus_start(struct mt6375_chg *chg)
{
	unsigned int check_s = mt6375_sus_check_s();
	int ret;

	ret = mt6375_apply_targets(chg, true);
	ret = ret ?: mt6375_set_chg_en(chg, true, "charging in sleep");
	if (ret)
		return ret;

	WRITE_ONCE(chg->sus_fired, false);
	chg->sus_start = ktime_get_boottime();
	alarm_start_timer(&chg->sus_alarm, ktime_set(check_s, 0), true);
	chg->sus_charging = true;
	chg->sus_logged = false;
	dev_dbg(chg->dev, "charging in sleep, next check in %u s\n", check_s);
	return 0;
}

/* After the sleep: alarm off, and was it seen to work? */
static void mt6375_sus_stop(struct mt6375_chg *chg)
{
	unsigned int check_s = mt6375_sus_check_s();
	s64 slack_ms = 1000LL * max_t(unsigned int, MT6375_SUS_SLACK_MIN_S,
				      check_s / 2);
	ktime_t left = alarm_expires_remaining(&chg->sus_alarm);
	s64 slept_ms;

	alarm_cancel(&chg->sus_alarm);
	chg->sus_charging = false;
	slept_ms = ktime_ms_delta(ktime_get_boottime(), chg->sus_start);

	/*
	 * An alarm past its expiry by far more than a resume takes and still
	 * unfired did not wake the system: the RTC wake-up does not work, so
	 * the battery would go unchecked in the next sleeps.
	 */
	if (!READ_ONCE(chg->sus_fired) && ktime_to_ms(left) < -slack_ms) {
		chg->sus_broken = true;
		dev_warn(chg->dev, "wake-up alarm %lld ms overdue and unfired after %lld ms of sleep: no charging in sleep from now on\n",
			 -ktime_to_ms(left), slept_ms);
		return;
	}
	dev_dbg(chg->dev, "charged in sleep for %lld ms (alarm %s)\n",
		slept_ms, READ_ONCE(chg->sus_fired) ? "fired" : "pending");
}

static void mt6375_psy_unreg_notifier(void *data)
{
	power_supply_unreg_notifier(data);
}

static int mt6375_chg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = {};
	struct mt6375_chg *chg;
	unsigned int port;
	bool defer = false;
	u32 ieoc_ua;
	int online, mivr, ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	platform_set_drvdata(pdev, chg);
	chg->behaviour = POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
	chg->reason = "boot";

	chg->regmap = dev_get_regmap(dev->parent, NULL);
	if (!chg->regmap)
		return dev_err_probe(dev, -ENODEV, "no parent regmap\n");

	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return ret;
	ret = devm_mutex_init(dev, &chg->hm_lock);
	if (ret)
		return ret;
	ret = devm_work_autocancel(dev, &chg->src_work, mt6375_src_work);
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

	/* the battery side, for the state lines only */
	chg->vbat_adc = mt6375_get_adc(dev, "vbat", &defer);
	chg->ibat_adc = mt6375_get_adc(dev, "ibat", &defer);
	chg->tjc_adc = mt6375_get_adc(dev, "temp_jc", &defer);
	chg->ts_adc = mt6375_get_adc(dev, "ts", &defer);
	if (defer)
		return -EPROBE_DEFER;

	/* before anything is written: it may defer */
	ret = mt6375_get_bc12_phy(chg);
	if (ret)
		return ret;

	/* The boot loader BC1.2 result is about the cable present now */
	if (!mt6375_get_online(chg, &online) && online) {
		chg->was_online = true;
		if (!mt6375_read_bc12_port(chg, &port) &&
		    port != MT6375_PORT_NOINFO) {
			chg->bc12_port = port;
			chg->bc12_valid = true;
		}
	}

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

	/* the OTG boost, only when the DT names it (a host-capable port) */
	{
		struct regulator_config rcfg = {
			.dev = dev, .regmap = chg->regmap, .driver_data = chg,
		};

		chg->otg_rdev = devm_regulator_register(dev, &mt6375_otg_desc, &rcfg);
		if (IS_ERR(chg->otg_rdev))
			return dev_err_probe(dev, PTR_ERR(chg->otg_rdev),
					     "failed to register the OTG boost\n");
	}

	/* the input limit follows the Type-C port manager and BC1.2 */
	chg->psy_nb.notifier_call = mt6375_psy_notify;
	ret = power_supply_reg_notifier(&chg->psy_nb);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, mt6375_psy_unreg_notifier, &chg->psy_nb);
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
	/* init wrote the boot AICR; MIVR stays as the boot loader set it */
	chg->aicr_set_ua = 0;
	if (mt6375_get_mivr(chg, &mivr)) {
		chg->mivr_set_uv = U32_MAX;	/* unknown: written on first use */
		chg->base_mivr_uv = MT6375_PD_MIVR_MIN_UV;
	} else {
		chg->mivr_set_uv = mivr;
		/* never a high-voltage value left from an earlier contract */
		chg->base_mivr_uv = min_t(u32, mivr, MT6375_PD_MIVR_MIN_UV);
	}

	mt6375_get_policy(chg);

	/* charging in sleep: charge_in_suspend may be switched at run time */
	alarm_init(&chg->sus_alarm, ALARM_BOOTTIME, mt6375_sus_alarm_fn);
	ret = devm_add_action_or_reset(dev, mt6375_cancel_alarm, chg);
	ret = ret ?: devm_device_init_wakeup(dev);
	if (ret)
		return ret;
	chg->pm_nb.notifier_call = mt6375_pm_notify;
	ret = register_pm_notifier(&chg->pm_nb);
	ret = ret ?: devm_add_action_or_reset(dev, mt6375_unreg_pm_notifier,
					      &chg->pm_nb);
	if (ret)
		return ret;

	chg->configured = true;
	mt6375_dump_state(chg, "configured");
	dev_info(dev, "limits: cv %u uV, ichg %u uA, battery %d..%d deci-degC\n",
		 chg->cv_max_uv, chg->ichg_max_ua, chg->temp_min, chg->temp_max);

	mt6375_request_irqs(chg);
	queue_delayed_work(system_freezable_wq, &chg->monitor, 0);
	schedule_work(&chg->src_work);
	/* a cable from boot without a boot loader result */
	if (chg->phy && READ_ONCE(bc12_detect) && chg->was_online &&
	    !chg->bc12_valid)
		schedule_work(&chg->bc12_work);
	return 0;
}

/* Faults stay latched for this driver lifetime; auto cannot clear them. */
static int mt6375_chg_suspend(struct device *dev)
{
	struct mt6375_chg *chg = dev_get_drvdata(dev);
	const char *veto;
	int ret = 0;

	/* D+/D- back with USB before the sleep */
	if (chg->phy)
		flush_work(&chg->bc12_work);

	mutex_lock(&chg->lock);
	chg->suspended = true;
	if (chg->configured) {
		veto = mt6375_sus_veto(chg);
		if (!veto) {
			ret = mt6375_sus_start(chg);
			if (ret)
				dev_warn(dev, "cannot charge in sleep: %d\n", ret);
		}
		if (veto || ret) {
			if (veto && mt6375_sus_wanted(chg) && !chg->sus_logged) {
				dev_info(dev, "no charging in sleep: %s\n", veto);
				chg->sus_logged = true;
			}
			ret = mt6375_set_chg_en(chg, false, "suspend");
		}
	}
	if (ret) {
		chg->fault_latched = true;
		chg->suspended = false;
	}
	mutex_unlock(&chg->lock);
	if (!ret)
		cancel_delayed_work_sync(&chg->monitor);
	else
		mod_delayed_work(system_freezable_wq, &chg->monitor, 0);
	return ret;
}

static int mt6375_chg_resume(struct device *dev)
{
	struct mt6375_chg *chg = dev_get_drvdata(dev);

	mutex_lock(&chg->lock);
	if (chg->sus_charging)
		mt6375_sus_stop(chg);
	chg->suspended = false;
	/* the monitor puts the awake current back and checks the battery */
	if (chg->configured && !chg->stopping)
		mod_delayed_work(system_freezable_wq, &chg->monitor, 0);
	mutex_unlock(&chg->lock);
	/* a contract change in sleep was not written yet */
	schedule_work(&chg->src_work);
	return 0;
}

static void mt6375_chg_stop(struct platform_device *pdev)
{
	struct mt6375_chg *chg = platform_get_drvdata(pdev);

	mutex_lock(&chg->lock);
	chg->stopping = true;
	if (chg->configured && mt6375_set_chg_en(chg, false, "driver stopped"))
		dev_err(chg->dev, "cannot disable charging on stop\n");
	/* no high-voltage MIVR left behind for a 5 V input */
	if (chg->configured && chg->mivr_set_uv != chg->base_mivr_uv &&
	    mt6375_set_mivr(chg, chg->base_mivr_uv))
		dev_err(chg->dev, "cannot restore MIVR on stop\n");
	mutex_unlock(&chg->lock);
	cancel_delayed_work_sync(&chg->monitor);
	/* a detection in progress gives D+/D- back before it ends */
	if (chg->phy)
		cancel_work_sync(&chg->bc12_work);
	/* both report to the power supply, which goes before their cancel */
	cancel_work_sync(&chg->src_work);
}

static DEFINE_SIMPLE_DEV_PM_OPS(mt6375_chg_pm_ops,
			      mt6375_chg_suspend, mt6375_chg_resume);

static const struct of_device_id mt6375_chg_of_match[] = {
	{ .compatible = "mediatek,mt6375-chg" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6375_chg_of_match);

static struct platform_driver mt6375_chg_driver = {
	.driver = {
		.name = "mt6375-charger",
		.of_match_table = mt6375_chg_of_match,
		.pm = pm_sleep_ptr(&mt6375_chg_pm_ops),
	},
	.probe = mt6375_chg_probe,
	.remove = mt6375_chg_stop,
	.shutdown = mt6375_chg_stop,
};
module_platform_driver(mt6375_chg_driver);

MODULE_DESCRIPTION("MediaTek MT6375 charger driver");
MODULE_LICENSE("GPL");
