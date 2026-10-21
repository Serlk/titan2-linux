// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: modem power sequence and boot state machine.
 *
 * LK powers the modem up and lets it run on every boot. The stock kernel
 * does not pick that modem up: on its first start it waits for the modem
 * boot ROM to finish (TF-A "check done"), powers the modem off and then
 * on again its own way:
 *
 *   power off: MD power domain off, 4 ms, flight mode on (TF-A),
 *              modem clocks gated in topckgen
 *   power on:  supplies at their voltages, modem clocks ungated, flight
 *              mode off (TF-A), MD power domain on, "kernel boot up"
 *              (TF-A applies the boot vector and releases the modem)
 *
 * after which the modem raises HS1 through the CCIF SRAM. This file has
 * that sequence, but runs nothing on its own: every step is behind the
 * power_ctrl module parameter AND an explicit command written to the
 * debugfs file "fsm". Probing never powers, clocks or calls TF-A.
 *
 * The runtime data answer to HS1 (and so HS2 and READY) is not here yet:
 * the FSM reads HS1, records what the modem asked for and then powers
 * the modem off again the stock way, so that a start always ends with
 * the modem either off or as LK left it.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "mtk_ccci.h"

static bool power_ctrl;
module_param(power_ctrl, bool, 0444);
MODULE_PARM_DESC(power_ctrl,
		 "allow the debugfs FSM commands to call TF-A and switch the modem (default: no)");

/* TF-A SiP service of the CCCI driver */
#define MTK_SIP_KERNEL_CCCI_CONTROL	MTK_SIP_SMC_CMD(0x505)

enum {
	CCCI_SIP_MD_CLOCK_REQUEST = 5,
	CCCI_SIP_MD_POWER_CONFIG = 6,
	CCCI_SIP_MD_FLIGHT_MODE_SET = 7,
	CCCI_SIP_CCIF_CLK_REQUEST = 14,
	CCCI_SIP_CCIF_CLK_RELEASE = 15,
};

/* sub-commands of CCCI_SIP_MD_POWER_CONFIG */
enum {
	CCCI_SIP_PWR_KERNEL_BOOT_UP = 0,
	CCCI_SIP_PWR_CHECK_FLAG = 2,
	CCCI_SIP_PWR_CHECK_DONE = 3,
	CCCI_SIP_PWR_BOOT_STATUS = 4,
};

/* expected boot status of MT6878 and later ("boot-status-config" 1) */
#define MTK_CCCI_BOOT_STATUS_OK		0x544300f1

#define SPM_MD_PWR_CON		0xe00
#define SPM_MD_PWR_ACK		BIT(30)

#define TOPCKGEN_MD1_CLK_MOD	0x0
#define TOPCKGEN_MD1_CLK_MASK	(BIT(8) | BIT(9))

#define MTK_CCCI_POLL_MS		20
#define MTK_CCCI_BROM_TIMEOUT_MS	2000
#define MTK_CCCI_HS1_TIMEOUT_MS		30000

static const char * const mtk_ccci_supply_names[MTK_CCCI_SUPPLY_NUM] = {
	[MTK_CCCI_SUPPLY_VMODEM] = "md-vmodem",
	[MTK_CCCI_SUPPLY_VSRAM] = "md-vsram",
	[MTK_CCCI_SUPPLY_VDIGRF] = "md-vdigrf",
};

/*
 * The MediaTek downstream driver only sets these voltages (vmodem and the modem SRAM
 * at 0.8 V, digital RF at 0.7 V on Titan 2) and never switches the
 * supplies: they are always on. Here they are only checked; the board
 * describes them always-on without voltage limits, so Linux cannot
 * change them at all.
 */
static const int mtk_ccci_supply_uv[MTK_CCCI_SUPPLY_NUM] = {
	[MTK_CCCI_SUPPLY_VMODEM] = 800000,
	[MTK_CCCI_SUPPLY_VSRAM] = 800000,
	[MTK_CCCI_SUPPLY_VDIGRF] = 700000,
};

static const char * const mtk_ccci_fsm_names[] = {
	[MTK_CCCI_FSM_LK_OWNED] = "lk-owned",
	[MTK_CCCI_FSM_STARTING] = "starting",
	[MTK_CCCI_FSM_WAIT_HS1] = "wait-hs1",
	[MTK_CCCI_FSM_HS1] = "hs1",
	[MTK_CCCI_FSM_WAIT_HS2] = "wait-hs2",
	[MTK_CCCI_FSM_EXCEPTION] = "exception",
	[MTK_CCCI_FSM_READY] = "ready",
	[MTK_CCCI_FSM_STOPPED] = "stopped",
	[MTK_CCCI_FSM_FAILED] = "failed",
};

const char *mtk_ccci_fsm_state_name(enum mtk_ccci_fsm_state state)
{
	if (state >= ARRAY_SIZE(mtk_ccci_fsm_names))
		return "?";
	return mtk_ccci_fsm_names[state];
}

static void mtk_ccci_sip(struct mtk_ccci *ccci, unsigned long cmd,
			 unsigned long a1, unsigned long a2,
			 struct arm_smccc_res *res)
{
	arm_smccc_smc(MTK_SIP_KERNEL_CCCI_CONTROL, cmd, a1, a2, 0, 0, 0, 0,
		      res);
	dev_dbg(ccci->dev, "sip %lu/%lu/%lu: %#lx %#lx %#lx %#lx\n", cmd, a1,
		a2, res->a0, res->a1, res->a2, res->a3);
}

static void mtk_ccci_query_boot_status(struct mtk_ccci *ccci)
{
	struct arm_smccc_res res;

	mtk_ccci_sip(ccci, CCCI_SIP_MD_POWER_CONFIG, CCCI_SIP_PWR_BOOT_STATUS,
		     0, &res);
	ccci->boot_status[0] = res.a1;
	ccci->boot_status[1] = res.a2;
	dev_info(ccci->dev, "boot status %#lx: %#lx %#lx (expect %#x)\n",
		 res.a0, res.a1, res.a2, MTK_CCCI_BOOT_STATUS_OK);
}

/* what has to be in place before anything is switched, as text */
struct mtk_ccci_check {
	char *buf;
	size_t len, size;
	bool ok;
};

static __printf(3, 4) void mtk_ccci_check_add(struct mtk_ccci_check *c,
					      bool ok, const char *fmt, ...)
{
	va_list args;

	if (!ok)
		c->ok = false;
	if (c->len >= c->size)
		return;
	c->len += scnprintf(c->buf + c->len, c->size - c->len, "%s ",
			    ok ? "[ok]  " : "[MISS]");
	va_start(args, fmt);
	c->len += vscnprintf(c->buf + c->len, c->size - c->len, fmt, args);
	va_end(args);
	c->len += scnprintf(c->buf + c->len, c->size - c->len, "\n");
}

static void mtk_ccci_fsm_check(struct mtk_ccci *ccci,
			       struct mtk_ccci_check *c)
{
	unsigned int i;

	c->ok = true;
	mtk_ccci_check_add(c, power_ctrl, "module parameter power_ctrl=1");
	mtk_ccci_check_add(c, ccci->md_domain,
			   "MD power domain attached (mediatek,modem-power-domain in DT)");
	mtk_ccci_check_add(c, ccci->topckgen,
			   "topckgen regmap (mediatek,topckgen)");
	mtk_ccci_check_add(c, ccci->infra && ccci->ccif_rst_bit != U32_MAX,
			   "CCIF reset (reg \"infra\", mediatek,ccif-reset-bit)");
	mtk_ccci_check_add(c, mtk_ccci_lk_smem(ccci, MTK_CCCI_SMEM_CCISM_MCU),
			   "CCISM_MCU shared memory from LK");
	for (i = 0; i < MTK_CCCI_SUPPLY_NUM; i++) {
		int uv = ccci->supply[i] ?
			 regulator_get_voltage(ccci->supply[i]) : -ENODEV;

		mtk_ccci_check_add(c, uv == mtk_ccci_supply_uv[i],
				   "%s-supply at %d uV (now %d)",
				   mtk_ccci_supply_names[i],
				   mtk_ccci_supply_uv[i], uv);
	}
}

/* SPM PWR_CON of the modem: 1 = powered, 0 = off, < 0 unknown */
static int mtk_ccci_md_pwr_ack(struct mtk_ccci *ccci)
{
	unsigned int val;

	if (!ccci->scpsys || regmap_read(ccci->scpsys, SPM_MD_PWR_CON, &val))
		return -ENODEV;
	return !!(val & SPM_MD_PWR_ACK);
}

static int mtk_ccci_topckgen_md_clk(struct mtk_ccci *ccci, bool on)
{
	return regmap_update_bits(ccci->topckgen, TOPCKGEN_MD1_CLK_MOD,
				  TOPCKGEN_MD1_CLK_MASK,
				  on ? 0 : TOPCKGEN_MD1_CLK_MASK);
}

/* wait for the modem boot ROM, as the stock first start does */
static int mtk_ccci_wait_brom(struct mtk_ccci *ccci)
{
	struct arm_smccc_res res;
	unsigned int waited;

	for (waited = 0; waited < MTK_CCCI_BROM_TIMEOUT_MS;
	     waited += MTK_CCCI_POLL_MS) {
		mtk_ccci_sip(ccci, CCCI_SIP_MD_POWER_CONFIG,
			     CCCI_SIP_PWR_CHECK_DONE, 0, &res);
		if (!res.a0)
			break;
		msleep(MTK_CCCI_POLL_MS);
	}

	mtk_ccci_sip(ccci, CCCI_SIP_MD_POWER_CONFIG, CCCI_SIP_PWR_CHECK_FLAG,
		     0, &res);
	dev_info(ccci->dev, "modem BROM %s after %u ms, flags %#lx %#lx %#lx %#lx\n",
		 waited < MTK_CCCI_BROM_TIMEOUT_MS ? "done" : "NOT done",
		 waited, res.a0, res.a1, res.a2, res.a3);
	mtk_ccci_query_boot_status(ccci);

	return waited < MTK_CCCI_BROM_TIMEOUT_MS ? 0 : -ETIMEDOUT;
}

static int mtk_ccci_power_off(struct mtk_ccci *ccci)
{
	struct arm_smccc_res res;
	int ret;

	/*
	 * The modem LK started: genpd sees the domain off. Take it over
	 * first (on a running domain this only sets bits that are set).
	 */
	if (!ccci->md_domain_held) {
		ret = pm_runtime_resume_and_get(ccci->dev);
		if (ret) {
			dev_err(ccci->dev, "MD power domain take-over: %d\n",
				ret);
			return ret;
		}
		ccci->md_domain_held = true;
	}

	/*
	 * The MediaTek downstream driver also blocks UFS I/O around the domain switch
	 * (ufs_mtk_cali_hold); the mainline UFS driver has no such hook.
	 */
	mtk_ccci_wdt_irq_stop(ccci);
	ret = pm_runtime_put_sync(ccci->dev);
	if (ret < 0)
		dev_warn(ccci->dev, "MD power domain off: %d\n", ret);
	ccci->md_domain_held = false;
	/* genpd does not report a failed power off: ask SPM */
	if (ret >= 0 && mtk_ccci_md_pwr_ack(ccci) > 0) {
		dev_err(ccci->dev, "MD power domain still on\n");
		ret = -EIO;
	}
	/* required by modem generation 6299 after the domain is off */
	mdelay(4);

	mtk_ccci_sip(ccci, CCCI_SIP_MD_FLIGHT_MODE_SET, 1, 0, &res);
	mtk_ccci_topckgen_md_clk(ccci, false);
	if (ret >= 0)
		ccci->lk_modem_gone = true;
	dev_info(ccci->dev, "modem powered off\n");

	return ret < 0 ? ret : 0;
}

static int mtk_ccci_power_on(struct mtk_ccci *ccci)
{
	struct arm_smccc_res res;
	int ret;

	ret = mtk_ccci_topckgen_md_clk(ccci, true);
	if (ret)
		return ret;

	mtk_ccci_sip(ccci, CCCI_SIP_MD_FLIGHT_MODE_SET, 0, 0, &res);

	ret = pm_runtime_resume_and_get(ccci->dev);
	if (ret) {
		dev_err(ccci->dev, "MD power domain on: %d\n", ret);
		return ret;
	}
	ccci->md_domain_held = true;

	/* make the boot vector take effect: the modem starts */
	mtk_ccci_sip(ccci, CCCI_SIP_MD_POWER_CONFIG,
		     CCCI_SIP_PWR_KERNEL_BOOT_UP, 0, &res);
	dev_info(ccci->dev, "modem released: %#lx %#lx %#lx %#lx\n", res.a0,
		 res.a1, res.a2, res.a3);

	/* as the MediaTek downstream driver: the watchdog is watched from here on */
	ret = mtk_ccci_wdt_irq_start(ccci);
	if (ret)
		dev_warn(ccci->dev, "modem watchdog interrupt: %d\n", ret);
	return 0;
}

/* wait for HS1: the RX worker sees the SRAM channel (interrupt or poll) */
static int mtk_ccci_wait_hs1(struct mtk_ccci *ccci)
{
	struct mtk_ccci_header hdr;
	unsigned long t0 = jiffies;
	long left;
	u32 status;

	left = wait_event_timeout(ccci->hs_wq,
				  READ_ONCE(ccci->hs1_pending) ||
				  READ_ONCE(ccci->exception),
				  msecs_to_jiffies(MTK_CCCI_HS1_TIMEOUT_MS));
	if (READ_ONCE(ccci->exception)) {
		dev_err(ccci->dev, "modem exception before HS1\n");
		return -EIO;
	}
	if (!left) {
		dev_err(ccci->dev, "no HS1 in %u ms\n", MTK_CCCI_HS1_TIMEOUT_MS);
		return -ETIMEDOUT;
	}

	mtk_ccci_ccif_read_hs(ccci, &hdr, &ccci->hs1_query);
	status = le32_to_cpu(hdr.status);
	if (FIELD_GET(MTK_CCCI_H_CHANNEL, status) != MTK_CCCI_CH_CONTROL_RX ||
	    le32_to_cpu(hdr.data[1]) != MTK_CCCI_CTL_MD_INIT_START_BOOT ||
	    le32_to_cpu(hdr.reserved) != MTK_CCCI_CTL_MD_INIT_CHK_ID) {
		dev_err(ccci->dev, "unexpected SRAM message %*ph\n",
			(int)sizeof(hdr), &hdr);
		return -EPROTO;
	}

	ccci->hs1_valid = le32_to_cpu(ccci->hs1_query.head_pattern) ==
			  MTK_CCCI_MD_FEATURE_PATTERN &&
			  le32_to_cpu(ccci->hs1_query.tail_pattern) ==
			  MTK_CCCI_MD_FEATURE_PATTERN;
	dev_info(ccci->dev, "HS1 after %u ms, feature query %s\n",
		 jiffies_to_msecs(jiffies - t0),
		 ccci->hs1_valid ? "valid" : "INVALID");
	return 0;
}

/* stop talking to the modem; it must be off (or never touched) */
static void mtk_ccci_hif_down(struct mtk_ccci *ccci)
{
	struct arm_smccc_res res;

	mtk_ccci_ccif_irq_stop(ccci);
	mtk_ccci_hif_stop(ccci);
	mtk_ccci_ccif_stop(ccci);
	mtk_ccci_sip(ccci, CCCI_SIP_CCIF_CLK_RELEASE, 0, 0, &res);
}

/*
 * The stock first start, in the stock order: CCIF taken over (while the
 * LK modem still runs), boot ROM wait, power off, power on, HS1. Then,
 * as the runtime data answer does not exist yet, the modem is powered
 * off again: the start reports -EOPNOTSUPP with the HS1 details kept.
 */
static int mtk_ccci_fsm_start(struct mtk_ccci *ccci)
{
	struct arm_smccc_res res;
	/* until the LK modem is powered off once, start the stock first way */
	bool first = !ccci->lk_modem_gone;
	int ret;

	if (ccci->state != MTK_CCCI_FSM_LK_OWNED &&
	    ccci->state != MTK_CCCI_FSM_STOPPED &&
	    !(ccci->state == MTK_CCCI_FSM_FAILED && !ccci->md_domain_held))
		return -EBUSY;

	WRITE_ONCE(ccci->state, MTK_CCCI_FSM_STARTING);
	ccci->hs1_valid = false;
	WRITE_ONCE(ccci->exception, false);
	ccci->ex_channels = 0;

	mtk_ccci_sip(ccci, CCCI_SIP_CCIF_CLK_REQUEST, 0, 0, &res);

	ret = mtk_ccci_ccif_start(ccci);
	if (ret) {
		dev_err(ccci->dev, "CCIF start: %d\n", ret);
		goto fail;
	}
	mtk_ccci_hif_start(ccci);
	ret = mtk_ccci_ccif_irq_start(ccci);
	if (ret)
		dev_warn(ccci->dev, "no CCIF interrupts (%d), polling\n", ret);

	if (first) {
		ret = mtk_ccci_wait_brom(ccci);
		if (ret)
			goto fail;
		ret = mtk_ccci_power_off(ccci);
		if (ret)
			goto fail;
	}

	ret = mtk_ccci_power_on(ccci);
	if (ret)
		goto fail;

	WRITE_ONCE(ccci->state, MTK_CCCI_FSM_WAIT_HS1);
	ret = mtk_ccci_wait_hs1(ccci);
	if (ret)
		goto fail;
	WRITE_ONCE(ccci->state, MTK_CCCI_FSM_HS1);
	mtk_ccci_query_boot_status(ccci);

	/*
	 * runtime data: not implemented. Without the answer the modem
	 * would time out waiting and assert; power it off now instead.
	 */
	dev_warn(ccci->dev, "runtime data: not implemented, stopping the modem at HS1\n");
	ret = -EOPNOTSUPP;

fail:
	if (ret != -EOPNOTSUPP)
		mtk_ccci_query_boot_status(ccci);
	if (ccci->md_domain_held)
		mtk_ccci_power_off(ccci);
	mtk_ccci_hif_down(ccci);
	/* FAILED with the domain held: "stop" retries the power off */
	WRITE_ONCE(ccci->state, ret == -EOPNOTSUPP && !ccci->md_domain_held ?
			 MTK_CCCI_FSM_STOPPED : MTK_CCCI_FSM_FAILED);
	return ret;
}

static int mtk_ccci_fsm_stop(struct mtk_ccci *ccci)
{
	int ret;

	switch (ccci->state) {
	case MTK_CCCI_FSM_WAIT_HS1:
	case MTK_CCCI_FSM_HS1:
	case MTK_CCCI_FSM_WAIT_HS2:
	case MTK_CCCI_FSM_READY:
	case MTK_CCCI_FSM_EXCEPTION:
	case MTK_CCCI_FSM_FAILED:
		break;
	default:
		return -EINVAL;
	}
	/* FAILED before the power on: nothing is held */
	if (!ccci->md_domain_held)
		return -EINVAL;

	mtk_ccci_port_unregister(ccci);
	ret = mtk_ccci_power_off(ccci);
	mtk_ccci_hif_down(ccci);
	WRITE_ONCE(ccci->state, ret ? MTK_CCCI_FSM_FAILED :
			 MTK_CCCI_FSM_STOPPED);
	return ret;
}

static int mtk_ccci_fsm_show(struct seq_file *s, void *unused)
{
	struct mtk_ccci *ccci = s->private;
	struct mtk_ccci_check c = { };
	unsigned int i;

	c.size = PAGE_SIZE;
	c.buf = kzalloc(c.size, GFP_KERNEL);
	if (!c.buf)
		return -ENOMEM;

	mutex_lock(&ccci->fsm_lock);
	seq_printf(s, "state: %s\n", mtk_ccci_fsm_state_name(ccci->state));
	seq_printf(s, "boot status: %#x %#x\n", ccci->boot_status[0],
		   ccci->boot_status[1]);
	if (ccci->hs1_valid) {
		seq_puts(s, "hs1 features:");
		for (i = 0; i < MTK_CCCI_FEATURE_COUNT; i++)
			if (ccci->hs1_query.feature[i])
				seq_printf(s, " %u:%#x", i,
					   ccci->hs1_query.feature[i]);
		seq_puts(s, "\n");
	}
	mtk_ccci_fsm_check(ccci, &c);
	mutex_unlock(&ccci->fsm_lock);

	seq_printf(s, "start needs:\n%s", c.buf);
	seq_puts(s, "commands: dry-run | query | start | stop\n");
	kfree(c.buf);
	return 0;
}

static void mtk_ccci_fsm_dry_run(struct mtk_ccci *ccci)
{
	struct mtk_ccci_check c = { };
	char *line, *p;

	c.size = PAGE_SIZE;
	c.buf = kzalloc(c.size, GFP_KERNEL);
	if (!c.buf)
		return;
	mtk_ccci_fsm_check(ccci, &c);

	dev_info(ccci->dev, "dry run, state %s, nothing is switched:\n",
		 mtk_ccci_fsm_state_name(ccci->state));
	for (p = c.buf; (line = strsep(&p, "\n")) && *line;)
		dev_info(ccci->dev, "  %s\n", line);
	dev_info(ccci->dev,
		 "  would: SIP CCIF_CLK_REQUEST; CCIF clocks on, rings in CCISM, CCIF reset, CCIF interrupts%s; topckgen md1 clocks on; SIP FLIGHT_MODE 0; MD domain on; SIP KERNEL_BOOT_UP; wait for HS1 up to %u ms; then (runtime data not implemented) MD domain off, 4 ms, SIP FLIGHT_MODE 1, topckgen md1 clocks off\n",
		 !ccci->lk_modem_gone ?
		 "; SIP CHECK_DONE poll; take the MD domain over, off, 4 ms, SIP FLIGHT_MODE 1, topckgen md1 clocks off" : "",
		 MTK_CCCI_HS1_TIMEOUT_MS);
	kfree(c.buf);
}

static ssize_t mtk_ccci_fsm_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct mtk_ccci *ccci = s->private;
	struct mtk_ccci_check c = { };
	char cmd[16];
	int ret = 0;

	if (count >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, ubuf, count))
		return -EFAULT;
	cmd[count] = '\0';
	strim(cmd);

	mutex_lock(&ccci->fsm_lock);
	if (!strcmp(cmd, "dry-run")) {
		mtk_ccci_fsm_dry_run(ccci);
	} else if (!strcmp(cmd, "query")) {
		/* TF-A reads only, but still TF-A: behind power_ctrl */
		if (power_ctrl)
			mtk_ccci_query_boot_status(ccci);
		else
			ret = -EPERM;
	} else if (!strcmp(cmd, "start")) {
		c.size = PAGE_SIZE;
		c.buf = kzalloc(c.size, GFP_KERNEL);
		if (!c.buf) {
			ret = -ENOMEM;
		} else {
			mtk_ccci_fsm_check(ccci, &c);
			ret = c.ok ? mtk_ccci_fsm_start(ccci) : -EPERM;
			kfree(c.buf);
		}
	} else if (!strcmp(cmd, "stop")) {
		ret = power_ctrl ? mtk_ccci_fsm_stop(ccci) : -EPERM;
	} else {
		ret = -EINVAL;
	}
	mutex_unlock(&ccci->fsm_lock);

	return ret ?: count;
}

static int mtk_ccci_fsm_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_ccci_fsm_show, inode->i_private);
}

static const struct file_operations mtk_ccci_fsm_fops = {
	.owner = THIS_MODULE,
	.open = mtk_ccci_fsm_open,
	.read = seq_read,
	.write = mtk_ccci_fsm_write,
	.llseek = seq_lseek,
	.release = single_release,
};

void mtk_ccci_fsm_debugfs_init(struct mtk_ccci *ccci)
{
	debugfs_create_file("fsm", 0600, ccci->debugfs, ccci,
			    &mtk_ccci_fsm_fops);
}

static void mtk_ccci_md_domain_detach(void *data)
{
	struct device *dev = data;

	/* never switches the domain: genpd keeps whatever state it had */
	pm_runtime_disable(dev);
	pm_genpd_remove_device(dev);
}

/*
 * The MD power domain, from "mediatek,modem-power-domain" and not from
 * "power-domains" on purpose: the bus code powers a "power-domains"
 * domain on at bind and off at unbind or when probe fails, which would
 * stop the modem LK started. Here the device is only added to the
 * domain, which the power controller registers default-off (genpd sees
 * it off while LK's modem runs). The domain is switched by the FSM
 * alone, through runtime PM: on in the power on step, off in the power
 * off step; the first power off adopts LK's modem (a power on of the
 * running domain, which only sets bits that are set already) and then
 * really switches it off. The module is pinned: with the domain held,
 * nothing may tear the device down.
 */
static int mtk_ccci_md_domain_attach(struct mtk_ccci *ccci)
{
	struct device *dev = ccci->dev;
	struct of_phandle_args args;
	int ret;

	ret = of_parse_phandle_with_args(dev->of_node,
					 "mediatek,modem-power-domain",
					 "#power-domain-cells", 0, &args);
	if (ret == -ENOENT)
		return 0;
	if (ret)
		return dev_err_probe(dev, ret, "modem power domain\n");
	if (dev->pm_domain) {
		of_node_put(args.np);
		return dev_err_probe(dev, -EINVAL,
				     "use mediatek,modem-power-domain, not power-domains\n");
	}

	ret = of_genpd_add_device(&args, dev);
	of_node_put(args.np);
	if (ret == -ENOENT)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "modem power domain not registered\n");
	if (ret)
		return dev_err_probe(dev, ret, "modem power domain\n");

	pm_runtime_enable(dev);
	ret = devm_add_action_or_reset(dev, mtk_ccci_md_domain_detach, dev);
	if (ret)
		return ret;

	ccci->md_domain = true;
	__module_get(THIS_MODULE);
	return 0;
}

/**
 * mtk_ccci_fsm_init() - get the power resources, switch nothing
 * @ccci: driver state
 *
 * Return: 0 or a negative error.
 */
int mtk_ccci_fsm_init(struct mtk_ccci *ccci)
{
	struct device *dev = ccci->dev;
	unsigned int i;
	int ret;

	mutex_init(&ccci->fsm_lock);
	ccci->state = MTK_CCCI_FSM_LK_OWNED;

	/* getting a supply does not switch it; it is never enabled here */
	for (i = 0; i < MTK_CCCI_SUPPLY_NUM; i++) {
		struct regulator *reg;

		reg = devm_regulator_get_optional(dev,
						  mtk_ccci_supply_names[i]);
		if (IS_ERR(reg)) {
			if (PTR_ERR(reg) != -ENODEV)
				return dev_err_probe(dev, PTR_ERR(reg),
						     "%s supply\n",
						     mtk_ccci_supply_names[i]);
			reg = NULL;
		}
		ccci->supply[i] = reg;
	}

	ret = mtk_ccci_md_domain_attach(ccci);
	if (ret)
		return ret;

	return 0;
}
