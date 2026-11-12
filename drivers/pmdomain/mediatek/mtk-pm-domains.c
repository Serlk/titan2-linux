// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Collabora Ltd.
 */
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_clk.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/sizes.h>
#include <linux/soc/mediatek/infracfg.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/spinlock.h>

#include "mt6735-pm-domains.h"
#include "mt6795-pm-domains.h"
#include "mt6878-pm-domains.h"
#include "mt6893-pm-domains.h"
#include "mt8167-pm-domains.h"
#include "mt8173-pm-domains.h"
#include "mt8183-pm-domains.h"
#include "mt8186-pm-domains.h"
#include "mt8188-pm-domains.h"
#include "mt8189-pm-domains.h"
#include "mt8192-pm-domains.h"
#include "mt8195-pm-domains.h"
#include "mt8196-pm-domains.h"
#include "mt8365-pm-domains.h"

#define MTK_POLL_DELAY_US		10
#define MTK_POLL_TIMEOUT		USEC_PER_SEC

#define MTK_HWV_POLL_DELAY_US		5
#define MTK_HWV_POLL_TIMEOUT		(300 * USEC_PER_MSEC)

#define MTK_HWV_PREPARE_DELAY_US	1
#define MTK_HWV_PREPARE_TIMEOUT		(3 * USEC_PER_MSEC)

#define PWR_RST_B_BIT			BIT(0)
#define PWR_ISO_BIT			BIT(1)
#define PWR_ON_BIT			BIT(2)
#define PWR_ON_2ND_BIT			BIT(3)
#define PWR_CLK_DIS_BIT			BIT(4)
#define PWR_SRAM_CLKISO_BIT		BIT(5)
#define PWR_SRAM_ISOINT_B_BIT		BIT(6)

#define PWR_RTFF_SAVE			BIT(24)
#define PWR_RTFF_NRESTORE		BIT(25)
#define PWR_RTFF_CLK_DIS		BIT(26)
#define PWR_RTFF_SAVE_FLAG		BIT(27)
#define PWR_RTFF_UFS_CLK_DIS		BIT(28)

#define MTK_SIP_KERNEL_HWCCF_CONTROL	MTK_SIP_SMC_CMD(0x540)

/*
 * MT6878 MM_INFRA power on, as the MediaTek GPL kernel does it in its
 * mminfra driver (drivers/misc/mediatek/mminfra/mtk-mminfra-debug.c,
 * mtk_mminfra_pd_callback() on GENPD_NOTIFY_ON, and the probe; requests of
 * drivers/misc/mediatek/cmdq/mailbox/cmdq-util.h): the secure firmware
 * call that drops the MM DEVAPC protection again (without it the boot
 * loader display path registers in mmsys ignore non-secure writes after a
 * power cycle), and the check of the SMI and GCE clock gates.
 */
#define MTK_SIP_CMDQ_CONTROL		MTK_SIP_SMC_CMD(0x518)
#define CMDQ_SIP_MMINFRA_CMD		6
#define CMDQ_SIP_MMUEN_DEVAPC_DISABLE	7

#define MMINFRA_CG_CON0			0x100
#define MMINFRA_CG_CLR0			0x108
#define MMINFRA_CG_CON1			0x110
#define MMINFRA_CG0_GCED		BIT(0)
#define MMINFRA_CG0_GCEM		BIT(1)
#define MMINFRA_CG0_SMI			BIT(2)
#define MMINFRA_CG1_GCE26M		BIT(17)

#define MMINFRA_ON_DEVAPC		BIT(0)
#define MMINFRA_ON_INIT_CMDS		BIT(1)
#define MMINFRA_ON_SMI_CG		BIT(2)
#define MMINFRA_ON_DISP_DEVAPC		BIT(3)

/*
 * Steps after an MT6878 MM_INFRA power on, a bit mask:
 * bit 0: the DEVAPC disable call at every power on (as the downstream kernel does);
 * bit 1: the mminfra commands 0 and 3 (HRE / read FIFO init) at the first
 *        power on (the downstream driver does them once, at its probe);
 * bit 2: ungate the SMI clock if it came back gated (the downstream driver only warns);
 * bit 3: the DEVAPC disable call after every DISP power on as well, an
 *        experiment: MM_INFRA may go down in the SPM sleep through the
 *        hardware voter without its power domain seeing it, and the mmsys
 *        path registers ignore writes after a DISP power cycle.
 * The clock gates are logged either way.
 */
static unsigned int mminfra_on = MMINFRA_ON_DEVAPC | MMINFRA_ON_SMI_CG |
				 MMINFRA_ON_DISP_DEVAPC;
module_param(mminfra_on, uint, 0644);
MODULE_PARM_DESC(mminfra_on, "MT6878 MM_INFRA power on steps: 1 DEVAPC, 2 init commands, 4 SMI clock, 8 DEVAPC after DISP (default 13)");

/*
 * MT6878 DISP power on: the steps of the MediaTek GPL scpsys driver
 * (mtk-scpsys.c scpsys_power_on() with the "disp" data of
 * mtk-scpsys-mt6878.c) that this driver does differently, a bit mask:
 * bit 0: switch PWR_ON, wait for its ack, 50 us, then PWR_ON_2ND and wait
 *        (downstream); this driver sets both switches at once;
 * bit 1: open the "disp0-0"/"disp0-1" subsys clocks of the downstream driver (mmsys
 *        SMI_COMMON, mdpsys SMI0) after the reset release, before the SRAM
 *        and the bus protection are released, and again before the bus
 *        protection is set at power off: the downstream driver keeps them enabled across
 *        both. The protection handshake of the larb ports needs the SMI
 *        clock; with an SMI larb driver that gates SMI_COMMON at runtime
 *        suspend (the -iommu DTB), setting the protection timed out after
 *        1 s and genpd silently kept DISP and MM_INFRA on, with the
 *        protection left set (the display froze after resume). Default on;
 * bit 2: diagnostics after power on: DISP bus protection status, mmsys
 *        0x0fc, 0xc00, 0xc0c and whether the unused crossbar output
 *        enable 0xc18 takes a write, before and after the firmware call of
 *        mminfra_on bit 3.
 * Default 2 (bit 1). A failed DISP power off is logged in any case, and
 * its half set bus protection is released again.
 */
#define DISP_ON_STAGED			BIT(0)
#define DISP_ON_SUBSYS_CG		BIT(1)
#define DISP_ON_PROBE			BIT(2)
#define DISP_ON_DISP_VA			BIT(3)

/*
 * bit 3 (experiment, off by default): after every DISP power on, the
 * "display VA" handshake of the downstream display driver
 * (mediatek_v2/mtk_drm_crtc.c mtk_crtc_enable_iommu_runtime(), which lists
 * MT6878: "set smi_larb_sec_con reg as 1"): the firmware arms a prebuilt
 * GCE-D program (CMDQ_PREBUILT_ENABLE, hwid 0) and the display VA request
 * (CMDQ_ENABLE_DISP_VA); the GCE token DISP_VA_START (692) starts it and it
 * signals DISP_VA_END (693) when done. The downstream driver sets the start
 * token from a GCE packet; here the CPU sets it through the GCE token update
 * register.
 * See scpsys_disp_va(); the disp_va parameter runs it once on demand.
 */
static unsigned int disp_on = DISP_ON_SUBSYS_CG;
module_param(disp_on, uint, 0644);
MODULE_PARM_DESC(disp_on, "MT6878 DISP power on/off: 1 staged power switches, 2 subsys clocks around the bus protection, 4 log crossbar writability, 8 display VA handshake (default 2)");

/* GCE-D tokens for DISP_ON_DISP_VA (cmdq mailbox register layout) */
#define MT6878_GCE_D_PA			0x1e980000
#define GCE_VA_MODE_CTL			0x54	/* bit 17 VA mode, 16 debug */
#define GCE_SYNC_TOKEN_ID		0x60
#define GCE_SYNC_TOKEN_VAL		0x64
#define GCE_SYNC_TOKEN_UPD		0x68
#define GCE_TOKEN_SET			BIT(16)
#define GCE_TOKEN_DISP_VA_START		692
#define GCE_TOKEN_DISP_VA_END		693
#define CMDQ_SIP_ENABLE_DISP_VA		1
#define CMDQ_SIP_PREBUILT_ENABLE	3
#define GCE_D_HWID			0
#define MMINFRA_CG_CLR1			0x118
#define DISP_VA_TIMEOUT_US		20000
/* SPM register the firmware checks before CMDQ_ENABLE_DISP_VA */
#define SPM_DISP_VA_STA			0xe70
#define SPM_DISP_VA_STA_ON		GENMASK(31, 30)

#define SUBSYS_CG_CON			0x100
#define SUBSYS_CG_CLR			0x108
/* mmsys config, for DISP_ON_PROBE */
#define MMSYS_SEC_STA			0x0fc
#define MMSYS_BYPASS_MUX_SHADOW		0xc00
#define MMSYS_CROSSBAR_CON		0xc0c
#define MMSYS_OVL_BG_CROSSBAR2		0xc18

struct scpsys_domain {
	struct generic_pm_domain genpd;
	const struct scpsys_domain_data *data;
	const struct scpsys_hwv_domain_data *hwv_data;
	struct scpsys *scpsys;
	int num_clks;
	struct clk_bulk_data *clks;
	int num_subsys_clks;
	struct clk_bulk_data *subsys_clks;
	struct regulator *supply;
	struct notifier_block rail_nb;
	void __iomem *cg_base;
	unsigned int power_ons;
	void __iomem *subsys_cg[2];
	/* found on at probe (left on by the boot loader) */
	bool boot_on;
	/*
	 * Switched on by scpsys_power_on() and not switched off since. genpd
	 * ignores a failed power on in its system resume path
	 * (genpd_sync_power_on() marks the domain on regardless), and would
	 * then power off a domain whose clocks this driver had already
	 * released: "already unprepared" for every clock of the domain.
	 */
	bool powered;
};

struct scpsys {
	struct device *dev;
	struct regmap *base;
	struct regmap *hwv_base;
	const struct scpsys_soc_data *soc_data;
	u8 bus_prot_index[BUS_PROT_BLOCK_COUNT];
	struct regmap **bus_prot;
	struct genpd_onecell_data pd_data;
	struct generic_pm_domain *domains[];
};

#define to_scpsys_domain(gpd) container_of(gpd, struct scpsys_domain, genpd)

static bool scpsys_domain_is_on(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;
	u32 mask = pd->data->sta_mask;
	u32 status, status2, mask2;

	mask2 = pd->data->sta2nd_mask ? pd->data->sta2nd_mask : mask;

	regmap_read(scpsys->base, pd->data->pwr_sta_offs, &status);
	status &= mask;

	regmap_read(scpsys->base, pd->data->pwr_sta2nd_offs, &status2);
	status2 &= mask2;

	/* A domain is on when both status bits are set. */
	return status && status2;
}

static bool scpsys_hwv_domain_is_disable_done(struct scpsys_domain *pd)
{
	const struct scpsys_hwv_domain_data *hwv = pd->hwv_data;
	u32 regs[2] = { hwv->done, hwv->clr_sta };
	u32 val[2];
	u32 mask = BIT(hwv->setclr_bit);

	regmap_multi_reg_read(pd->scpsys->hwv_base, regs, val, 2);

	/* Disable is done when the bit is set in DONE, cleared in CLR_STA */
	return (val[0] & mask) && !(val[1] & mask);
}

static bool scpsys_hwv_domain_is_enable_done(struct scpsys_domain *pd)
{
	const struct scpsys_hwv_domain_data *hwv = pd->hwv_data;
	u32 regs[3] = { hwv->done, hwv->en, hwv->set_sta };
	u32 val[3];
	u32 mask = BIT(hwv->setclr_bit);

	regmap_multi_reg_read(pd->scpsys->hwv_base, regs, val, 3);

	/* Enable is done when the bit is set in DONE and EN, cleared in SET_STA */
	return (val[0] & mask) && (val[1] & mask) && !(val[2] & mask);
}

static DEFINE_SPINLOCK(scpsys_sec_infra_lock);
static unsigned int scpsys_sec_infra_refcnt;

static int scpsys_sec_infra_power_on(bool on)
{
	struct arm_smccc_res res;
	unsigned long flags;
	int ret = 0;

	/*
	 * The infra power request is global: more than one power domain can
	 * be in the middle of a power sequence that needs it, so only issue
	 * the request for the first user and drop it with the last one.
	 */
	spin_lock_irqsave(&scpsys_sec_infra_lock, flags);

	if (on) {
		if (scpsys_sec_infra_refcnt++ == 0) {
			arm_smccc_smc(MTK_SIP_KERNEL_HWCCF_CONTROL, 1, 0, 0, 0, 0, 0, 0, &res);
			ret = res.a0;
			if (ret)
				scpsys_sec_infra_refcnt--;
		}
	} else if (!WARN_ON(!scpsys_sec_infra_refcnt)) {
		if (--scpsys_sec_infra_refcnt == 0) {
			arm_smccc_smc(MTK_SIP_KERNEL_HWCCF_CONTROL, 0, 0, 0, 0, 0, 0, 0, &res);
			ret = res.a0;
		}
	}

	spin_unlock_irqrestore(&scpsys_sec_infra_lock, flags);

	return ret;
}

static int scpsys_sram_enable(struct scpsys_domain *pd)
{
	u32 expected_ack, pdn_ack = pd->data->sram_pdn_ack_bits;
	struct scpsys *scpsys = pd->scpsys;
	unsigned int tmp;
	int ret;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_SRAM_PDN_INVERTED)) {
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, pd->data->sram_pdn_bits);
		expected_ack = pdn_ack;
	} else {
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, pd->data->sram_pdn_bits);
		expected_ack = 0;
	}

	/* Either wait until SRAM_PDN_ACK all 1 or 0 */
	ret = regmap_read_poll_timeout(scpsys->base, pd->data->ctl_offs, tmp,
				       (tmp & pdn_ack) == expected_ack,
				       MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
	if (ret < 0)
		return ret;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_SRAM_ISO)) {
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_SRAM_ISOINT_B_BIT);
		udelay(1);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_SRAM_CLKISO_BIT);
	}

	return 0;
}

static int scpsys_sram_disable(struct scpsys_domain *pd)
{
	u32 expected_ack, pdn_ack = pd->data->sram_pdn_ack_bits;
	struct scpsys *scpsys = pd->scpsys;
	unsigned int tmp;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_SRAM_ISO)) {
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_SRAM_CLKISO_BIT);
		udelay(1);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_SRAM_ISOINT_B_BIT);
	}

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_SRAM_PDN_INVERTED)) {
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, pd->data->sram_pdn_bits);
		expected_ack = 0;
	} else {
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, pd->data->sram_pdn_bits);
		expected_ack = pdn_ack;
	}

	/* Either wait until SRAM_PDN_ACK all 1 or 0 */
	return regmap_read_poll_timeout(scpsys->base, pd->data->ctl_offs, tmp,
					(tmp & pdn_ack) == expected_ack,
					MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
}

static struct regmap *scpsys_bus_protect_get_regmap(struct scpsys_domain *pd,
						    const struct scpsys_bus_prot_data *bpd)
{
	struct scpsys *scpsys = pd->scpsys;
	unsigned short block_idx = scpsys->bus_prot_index[bpd->bus_prot_block];

	return scpsys->bus_prot[block_idx];
}

static struct regmap *scpsys_bus_protect_get_sta_regmap(struct scpsys_domain *pd,
							const struct scpsys_bus_prot_data *bpd)
{
	struct scpsys *scpsys = pd->scpsys;
	int block_idx = scpsys->bus_prot_index[bpd->bus_prot_sta_block];

	return scpsys->bus_prot[block_idx];
}

static int scpsys_bus_protect_clear(struct scpsys_domain *pd,
				    const struct scpsys_bus_prot_data *bpd)
{
	struct regmap *sta_regmap = scpsys_bus_protect_get_sta_regmap(pd, bpd);
	struct regmap *regmap = scpsys_bus_protect_get_regmap(pd, bpd);
	u32 sta_mask = bpd->bus_prot_sta_mask;
	u32 expected_ack;
	u32 val;

	expected_ack = (bpd->bus_prot_sta_block == BUS_PROT_BLOCK_INFRA_NAO ? sta_mask : 0);

	if (bpd->flags & BUS_PROT_REG_UPDATE)
		regmap_clear_bits(regmap, bpd->bus_prot_clr, bpd->bus_prot_set_clr_mask);
	else
		regmap_write(regmap, bpd->bus_prot_clr, bpd->bus_prot_set_clr_mask);

	if (bpd->flags & BUS_PROT_IGNORE_CLR_ACK)
		return 0;

	return regmap_read_poll_timeout(sta_regmap, bpd->bus_prot_sta,
					val, (val & sta_mask) == expected_ack,
					MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
}

static int scpsys_bus_protect_set(struct scpsys_domain *pd,
				  const struct scpsys_bus_prot_data *bpd)
{
	struct regmap *sta_regmap = scpsys_bus_protect_get_sta_regmap(pd, bpd);
	struct regmap *regmap = scpsys_bus_protect_get_regmap(pd, bpd);
	u32 sta_mask = bpd->bus_prot_sta_mask;
	u32 val;

	if (bpd->flags & BUS_PROT_REG_UPDATE)
		regmap_set_bits(regmap, bpd->bus_prot_set, bpd->bus_prot_set_clr_mask);
	else
		regmap_write(regmap, bpd->bus_prot_set, bpd->bus_prot_set_clr_mask);

	return regmap_read_poll_timeout(sta_regmap, bpd->bus_prot_sta,
					val, (val & sta_mask) == sta_mask,
					MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
}

static int scpsys_bus_protect_enable(struct scpsys_domain *pd, u8 flags)
{
	for (int i = 0; i < SPM_MAX_BUS_PROT_DATA; i++) {
		const struct scpsys_bus_prot_data *bpd = &pd->data->bp_cfg[i];
		int ret;

		if (!bpd->bus_prot_set_clr_mask)
			break;

		if ((bpd->flags & BUS_PROT_IGNORE_SUBCLK) !=
		    (flags & BUS_PROT_IGNORE_SUBCLK))
			continue;

		if (bpd->flags & BUS_PROT_INVERTED)
			ret = scpsys_bus_protect_clear(pd, bpd);
		else
			ret = scpsys_bus_protect_set(pd, bpd);
		if (ret)
			return ret;
	}

	return 0;
}

static int scpsys_bus_protect_disable(struct scpsys_domain *pd, u8 flags)
{
	for (int i = SPM_MAX_BUS_PROT_DATA - 1; i >= 0; i--) {
		const struct scpsys_bus_prot_data *bpd = &pd->data->bp_cfg[i];
		int ret;

		if (!bpd->bus_prot_set_clr_mask)
			continue;

		if ((bpd->flags & BUS_PROT_IGNORE_SUBCLK) !=
		    (flags & BUS_PROT_IGNORE_SUBCLK))
			continue;

		if (bpd->flags & BUS_PROT_INVERTED)
			ret = scpsys_bus_protect_set(pd, bpd);
		else
			ret = scpsys_bus_protect_clear(pd, bpd);
		if (ret)
			return ret;
	}

	return 0;
}

static int scpsys_regulator_enable(struct regulator *supply)
{
	return supply ? regulator_enable(supply) : 0;
}

/*
 * Isolation of an external rail shared by several domains (MT6878 VMM).
 * Released once the rail is up, in the order of the table, and set again in
 * the reverse order before the rail goes down: done from the regulator
 * notifier, so that it follows the rail and not the single domain, plus once
 * more in power on for a rail that was already up.
 */
static void scpsys_rail_iso_release(struct scpsys *scpsys,
				    const struct scpsys_rail_iso *iso)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(iso->bits); i++)
		regmap_write(scpsys->base, iso->clr_offs, BIT(iso->bits[i]));
}

static void scpsys_rail_iso_set(struct scpsys *scpsys,
				const struct scpsys_rail_iso *iso)
{
	int i;

	for (i = ARRAY_SIZE(iso->bits) - 1; i >= 0; i--)
		regmap_write(scpsys->base, iso->set_offs, BIT(iso->bits[i]));
}

static u32 scpsys_rail_iso_mask(const struct scpsys_rail_iso *iso)
{
	return BIT(iso->bits[0]) | BIT(iso->bits[1]) | BIT(iso->bits[2]);
}

static int scpsys_rail_event(struct notifier_block *nb, unsigned long event,
			     void *data)
{
	struct scpsys_domain *pd = container_of(nb, struct scpsys_domain, rail_nb);
	const struct scpsys_rail_iso *iso = pd->data->rail_iso;

	switch (event) {
	case REGULATOR_EVENT_ENABLE:
	case REGULATOR_EVENT_ABORT_DISABLE:
		scpsys_rail_iso_release(pd->scpsys, iso);
		break;
	case REGULATOR_EVENT_PRE_DISABLE:
		scpsys_rail_iso_set(pd->scpsys, iso);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static int scpsys_regulator_disable(struct regulator *supply)
{
	return supply ? regulator_disable(supply) : 0;
}

static unsigned long scpsys_cmdq_sip(unsigned long req, unsigned long arg)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_CMDQ_CONTROL, req, arg, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

/* for DISP_ON_DISP_VA: the MM_INFRA clock gates and the DISP domain */
static void __iomem *scpsys_mminfra_cg;
static void __iomem *scpsys_gce_d;
static struct scpsys_domain *scpsys_disp_pd;

static u32 scpsys_gce_token(void __iomem *gce, u32 id)
{
	writel(id, gce + GCE_SYNC_TOKEN_ID);
	return readl(gce + GCE_SYNC_TOKEN_VAL);
}

/*
 * The display VA handshake (DISP_ON_DISP_VA), in the order of the downstream
 * kernel (mtk_crtc_exec_atf_prebuilt_instr(), cmdq_util_enable_disp_va()).
 * Called with DISP, and with it MM_INFRA that holds GCE-D, powered; the GCE
 * clocks are ungated first and stay so (GCE registers are not to be touched
 * gated):
 *   1. CMDQ_PREBUILT_ENABLE (hwid 0): the firmware sets the VA mode of
 *      GCE-D (0x54 bit 17) and starts nothing;
 *   2. token DISP_VA_END cleared; the SPM status the firmware requires
 *      (0xe70 bits 31:30 both set, else it skips and only logs) and the
 *      GCE-D VA mode logged;
 *   3. CMDQ_ENABLE_DISP_VA: the firmware places its program in the boot
 *      loader block me_cmdq_reserved and starts GCE-D secure thread 29;
 *   4. token DISP_VA_START set. Downstream sets it from a display GCE packet
 *      built before and flushed after the call; this kernel has no GCE
 *      mailbox for MT6878, so the CPU sets it through the token update
 *      register, after the call as the flushed packet would;
 *   5. wait for DISP_VA_END (the downstream packet waits for it), consume it.
 * The firmware returns 0 whether it did the work or not; its own log is
 * in the boot loader block atf-log-reserved (mtk-atf-log debugfs).
 */
static void scpsys_disp_va(struct scpsys_domain *pd)
{
	struct device *dev = pd->scpsys->dev;
	void __iomem *gce = scpsys_gce_d;
	unsigned long r_pre, r_va;
	u32 cg0, cg1, va0, va1, spm = 0, end = 0, start;
	unsigned int us;

	if (!gce || !scpsys_mminfra_cg) {
		dev_warn(dev, "disp va: GCE-D or MM_INFRA clock gates not mapped\n");
		return;
	}

	cg0 = readl(scpsys_mminfra_cg + MMINFRA_CG_CON0);
	cg1 = readl(scpsys_mminfra_cg + MMINFRA_CG_CON1);
	if (cg0 & (MMINFRA_CG0_GCED | MMINFRA_CG0_GCEM))
		writel(MMINFRA_CG0_GCED | MMINFRA_CG0_GCEM,
		       scpsys_mminfra_cg + MMINFRA_CG_CLR0);
	if (cg1 & MMINFRA_CG1_GCE26M)
		writel(MMINFRA_CG1_GCE26M, scpsys_mminfra_cg + MMINFRA_CG_CLR1);

	va0 = readl(gce + GCE_VA_MODE_CTL);
	r_pre = scpsys_cmdq_sip(CMDQ_SIP_PREBUILT_ENABLE, GCE_D_HWID);
	va1 = readl(gce + GCE_VA_MODE_CTL);

	writel(GCE_TOKEN_DISP_VA_END, gce + GCE_SYNC_TOKEN_UPD);	/* clear */
	regmap_read(pd->scpsys->base, SPM_DISP_VA_STA, &spm);
	r_va = scpsys_cmdq_sip(CMDQ_SIP_ENABLE_DISP_VA, 0);
	writel(GCE_TOKEN_SET | GCE_TOKEN_DISP_VA_START, gce + GCE_SYNC_TOKEN_UPD);

	for (us = 0; us < DISP_VA_TIMEOUT_US; us += 100) {
		end = scpsys_gce_token(gce, GCE_TOKEN_DISP_VA_END);
		if (end)
			break;
		udelay(100);
	}
	start = scpsys_gce_token(gce, GCE_TOKEN_DISP_VA_START);
	/*
	 * The secure thread registers (0xf80..) are not read: they may belong
	 * to the secure world.
	 */
	writel(GCE_TOKEN_DISP_VA_END, gce + GCE_SYNC_TOKEN_UPD);	/* consumed */

	dev_info(dev, "%s: disp va: spm 0xe70 %08x%s, gce 0x54 %08x -> %08x (prebuilt %#lx), enable %#lx, end token %u after %u us%s, start token %u, mminfra cg %08x %08x\n",
		 pd->genpd.name, spm,
		 (spm & SPM_DISP_VA_STA_ON) == SPM_DISP_VA_STA_ON ? "" :
		 " (bits 31:30 not both set: the firmware skips)",
		 va0, va1, r_pre, r_va, end, us, end ? "" : " (timeout)", start,
		 cg0, cg1);
}

static int scpsys_disp_va_set(const char *val, const struct kernel_param *kp)
{
	struct scpsys_domain *pd = READ_ONCE(scpsys_disp_pd);
	bool on;
	int ret;

	ret = kstrtobool(val, &on);
	if (ret)
		return ret;
	if (!on)
		return 0;
	/* only with the display powered (GCE-D and the larbs are under it) */
	if (!pd || !pd->powered)
		return -ENODEV;
	scpsys_disp_va(pd);
	return 0;
}

static const struct kernel_param_ops scpsys_disp_va_ops = {
	.set = scpsys_disp_va_set,
};
module_param_cb(disp_va, &scpsys_disp_va_ops, NULL, 0200);
MODULE_PARM_DESC(disp_va, "MT6878 experiment: write 1 to run the display VA handshake now (DISP on)");

/* MT6878 MM_INFRA is on again: firmware calls and clock gates, see above */
static void scpsys_mminfra_powered_on(struct scpsys_domain *pd)
{
	struct device *dev = pd->scpsys->dev;
	unsigned int steps = READ_ONCE(mminfra_on);
	unsigned long r0 = 0, r3 = 0, rd = 0;
	u32 cg0 = 0, cg1 = 0, cg0_after = 0;
	bool first = !pd->power_ons++;
	bool hwv = pd->hwv_data;

	if (!hwv) {
		/* a domain below MM_INFRA (DISP) */
		if (!(steps & MMINFRA_ON_DISP_DEVAPC))
			return;
		rd = scpsys_cmdq_sip(CMDQ_SIP_MMUEN_DEVAPC_DISABLE, 0);
		dev_info(dev, "%s on (#%u): devapc %#lx\n", pd->genpd.name,
			 pd->power_ons, rd);
		return;
	}

	if (first && (steps & MMINFRA_ON_INIT_CMDS)) {
		r0 = scpsys_cmdq_sip(CMDQ_SIP_MMINFRA_CMD, 0);
		r3 = scpsys_cmdq_sip(CMDQ_SIP_MMINFRA_CMD, 3);
	}
	if (steps & MMINFRA_ON_DEVAPC)
		rd = scpsys_cmdq_sip(CMDQ_SIP_MMUEN_DEVAPC_DISABLE, 0);

	if (pd->cg_base) {
		cg0 = readl(pd->cg_base + MMINFRA_CG_CON0);
		cg1 = readl(pd->cg_base + MMINFRA_CG_CON1);
		if ((cg0 & MMINFRA_CG0_SMI) && (steps & MMINFRA_ON_SMI_CG))
			writel(MMINFRA_CG0_SMI, pd->cg_base + MMINFRA_CG_CLR0);
		cg0_after = readl(pd->cg_base + MMINFRA_CG_CON0);
	}

	dev_info(dev, "%s on (#%u): steps %#x, init cmds %#lx/%#lx, devapc %#lx, cg %08x->%08x %08x%s\n",
		 pd->genpd.name, pd->power_ons, steps, r0, r3, rd, cg0,
		 cg0_after, cg1,
		 (cg0_after & MMINFRA_CG0_SMI) ? " (SMI gated)" : "");
}

static int scpsys_hwv_power_on(struct generic_pm_domain *genpd)
{
	struct scpsys_domain *pd = container_of(genpd, struct scpsys_domain, genpd);
	const struct scpsys_hwv_domain_data *hwv = pd->hwv_data;
	struct scpsys *scpsys = pd->scpsys;
	u32 val;
	int ret;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_INFRA_PWR_CTL)) {
		ret = scpsys_sec_infra_power_on(true);
		if (ret)
			return ret;
	}

	ret = scpsys_regulator_enable(pd->supply);
	if (ret)
		goto err_infra;

	ret = clk_bulk_prepare_enable(pd->num_clks, pd->clks);
	if (ret)
		goto err_reg;

	/* For HWV the subsys clocks refer to the HWV low power subsystem */
	ret = clk_bulk_prepare_enable(pd->num_subsys_clks, pd->subsys_clks);
	if (ret)
		goto err_disable_clks;

	/* Make sure the HW Voter is idle and able to accept commands */
	ret = regmap_read_poll_timeout_atomic(scpsys->hwv_base, hwv->done, val,
					      val & BIT(hwv->setclr_bit),
					      MTK_HWV_POLL_DELAY_US,
					      MTK_HWV_POLL_TIMEOUT);
	if (ret) {
		dev_err(scpsys->dev, "Failed to power on: HW Voter busy.\n");
		goto err_disable_subsys_clks;
	}

	/*
	 * Instruct the HWV to power on the MTCMOS (power domain): after that,
	 * the same bit will be unset immediately by the hardware.
	 */
	regmap_write(scpsys->hwv_base, hwv->set, BIT(hwv->setclr_bit));

	/*
	 * Wait until the HWV sets the bit again, signalling that its internal
	 * state machine was started and it now processing the vote command.
	 */
	ret = regmap_read_poll_timeout_atomic(scpsys->hwv_base, hwv->set, val,
					      val & BIT(hwv->setclr_bit),
					      MTK_HWV_PREPARE_DELAY_US,
					      MTK_HWV_PREPARE_TIMEOUT);
	if (ret) {
		dev_err(scpsys->dev, "Failed to power on: HW Voter not starting.\n");
		goto err_disable_subsys_clks;
	}

	/* Wait for ACK, signalling that the MTCMOS was enabled */
	ret = readx_poll_timeout_atomic(scpsys_hwv_domain_is_enable_done, pd, val, val,
					MTK_HWV_POLL_DELAY_US, MTK_HWV_POLL_TIMEOUT);
	if (ret) {
		dev_err(scpsys->dev, "Failed to power on: HW Voter ACK timeout.\n");
		goto err_disable_subsys_clks;
	}

	/* It's done! Disable the HWV low power subsystem clocks */
	clk_bulk_disable_unprepare(pd->num_subsys_clks, pd->subsys_clks);

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_MMINFRA_ON_SIP))
		scpsys_mminfra_powered_on(pd);

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_INFRA_PWR_CTL))
		scpsys_sec_infra_power_on(false);

	return 0;

err_disable_subsys_clks:
	clk_bulk_disable_unprepare(pd->num_subsys_clks, pd->subsys_clks);
err_disable_clks:
	clk_bulk_disable_unprepare(pd->num_clks, pd->clks);
err_reg:
	scpsys_regulator_disable(pd->supply);
err_infra:
	if (MTK_SCPD_CAPS(pd, MTK_SCPD_INFRA_PWR_CTL))
		scpsys_sec_infra_power_on(false);
	return ret;
};

static int scpsys_hwv_power_off_internal(struct scpsys_domain *pd)
{
	const struct scpsys_hwv_domain_data *hwv = pd->hwv_data;
	struct scpsys *scpsys = pd->scpsys;
	u32 val;
	int ret;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_INFRA_PWR_CTL)) {
		ret = scpsys_sec_infra_power_on(true);
		if (ret)
			return ret;
	}

	ret = clk_bulk_prepare_enable(pd->num_subsys_clks, pd->subsys_clks);
	if (ret)
		goto err_infra;

	/* Make sure the HW Voter is idle and able to accept commands */
	ret = regmap_read_poll_timeout_atomic(scpsys->hwv_base, hwv->done, val,
					      val & BIT(hwv->setclr_bit),
					      MTK_HWV_POLL_DELAY_US,
					      MTK_HWV_POLL_TIMEOUT);
	if (ret)
		goto err_disable_subsys_clks;


	/*
	 * Instruct the HWV to power off the MTCMOS (power domain): differently
	 * from poweron, the bit will be kept set.
	 */
	regmap_write(scpsys->hwv_base, hwv->clr, BIT(hwv->setclr_bit));

	/*
	 * Wait until the HWV clears the bit, signalling that its internal
	 * state machine was started and it now processing the clear command.
	 */
	ret = regmap_read_poll_timeout_atomic(scpsys->hwv_base, hwv->clr, val,
					      !(val & BIT(hwv->setclr_bit)),
					      MTK_HWV_PREPARE_DELAY_US,
					      MTK_HWV_PREPARE_TIMEOUT);
	if (ret)
		goto err_disable_subsys_clks;

	/* Poweroff needs 100us for the HW to stabilize */
	udelay(100);

	/* Wait for ACK, signalling that the MTCMOS was disabled */
	ret = readx_poll_timeout_atomic(scpsys_hwv_domain_is_disable_done, pd, val, val,
					MTK_HWV_POLL_DELAY_US, MTK_HWV_POLL_TIMEOUT);
	if (ret)
		goto err_disable_subsys_clks;

	clk_bulk_disable_unprepare(pd->num_subsys_clks, pd->subsys_clks);
	clk_bulk_disable_unprepare(pd->num_clks, pd->clks);

	scpsys_regulator_disable(pd->supply);

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_INFRA_PWR_CTL))
		scpsys_sec_infra_power_on(false);

	return 0;

err_disable_subsys_clks:
	clk_bulk_disable_unprepare(pd->num_subsys_clks, pd->subsys_clks);
err_infra:
	if (MTK_SCPD_CAPS(pd, MTK_SCPD_INFRA_PWR_CTL))
		scpsys_sec_infra_power_on(false);
	return ret;
};

static int scpsys_hwv_power_off(struct generic_pm_domain *genpd)
{
	struct scpsys_domain *pd = container_of(genpd, struct scpsys_domain, genpd);
	int ret;

	ret = scpsys_hwv_power_off_internal(pd);
	/* the power on logs its steps; tell whether the vote was dropped */
	if (MTK_SCPD_CAPS(pd, MTK_SCPD_MMINFRA_ON_SIP))
		dev_info(pd->scpsys->dev, "%s off: %d\n", genpd->name, ret);
	return ret;
}

static int scpsys_ctl_pwrseq_on(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;
	bool do_rtff_nrestore, tmp;
	int ret;

	/* subsys power on */
	regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_ON_BIT);
	if (MTK_SCPD_CAPS(pd, MTK_SCPD_MT6878_DISP_ON) &&
	    (READ_ONCE(disp_on) & DISP_ON_STAGED)) {
		unsigned int val;

		/* as the downstream driver: the first switch, its ack, 50 us */
		ret = regmap_read_poll_timeout(scpsys->base, pd->data->pwr_sta_offs,
					       val, val & pd->data->sta_mask,
					       MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
		if (ret < 0)
			return ret;
		usleep_range(50, 60);
	}
	regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_ON_2ND_BIT);

	/* wait until PWR_ACK = 1 */
	ret = readx_poll_timeout(scpsys_domain_is_on, pd, tmp, tmp, MTK_POLL_DELAY_US,
				 MTK_POLL_TIMEOUT);
	if (ret < 0)
		return ret;

	regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_CLK_DIS_BIT);
	regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_ISO_BIT);
	regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RST_B_BIT);

	/*
	 * RTFF HW state may be modified by secure world or remote processors.
	 *
	 * With the only exception of STOR_UFS, which always needs save/restore,
	 * check if this power domain's RTFF is already on before trying to do
	 * the NRESTORE procedure, otherwise the system will lock up.
	 */
	switch (pd->data->rtff_type) {
	case SCPSYS_RTFF_TYPE_GENERIC:
	case SCPSYS_RTFF_TYPE_PCIE_PHY:
	{
		u32 ctl_status;

		regmap_read(scpsys->base, pd->data->ctl_offs, &ctl_status);
		do_rtff_nrestore = ctl_status & PWR_RTFF_SAVE_FLAG;
		break;
	}
	case SCPSYS_RTFF_TYPE_STOR_UFS:
		/* STOR_UFS always needs NRESTORE */
		do_rtff_nrestore = true;
		break;
	default:
		do_rtff_nrestore = false;
		break;
	}

	/* Return early if RTFF NRESTORE shall not be done */
	if (!do_rtff_nrestore)
		return 0;

	switch (pd->data->rtff_type) {
	case SCPSYS_RTFF_TYPE_GENERIC:
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE_FLAG);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_CLK_DIS);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_NRESTORE);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_NRESTORE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_CLK_DIS);
		break;
	case SCPSYS_RTFF_TYPE_PCIE_PHY:
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE_FLAG);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_NRESTORE);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_NRESTORE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_CLK_DIS);
		break;
	case SCPSYS_RTFF_TYPE_STOR_UFS:
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_UFS_CLK_DIS);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_NRESTORE);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_NRESTORE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_UFS_CLK_DIS);
		break;
	default:
		break;
	}

	return 0;
}

static void scpsys_ctl_pwrseq_off(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;

	switch (pd->data->rtff_type) {
	case SCPSYS_RTFF_TYPE_GENERIC:
	case SCPSYS_RTFF_TYPE_PCIE_PHY:
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_CLK_DIS);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_CLK_DIS);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE_FLAG);
		break;
	case SCPSYS_RTFF_TYPE_STOR_UFS:
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_UFS_CLK_DIS);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_UFS_CLK_DIS);
		break;
	default:
		break;
	}

	/* subsys power off */
	regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_ISO_BIT);

	/* Wait for RTFF HW to sync buck isolation state if this is PCIe PHY RTFF */
	if (pd->data->rtff_type == SCPSYS_RTFF_TYPE_PCIE_PHY)
		udelay(1);

	regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_CLK_DIS_BIT);
	regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RST_B_BIT);
	regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_ON_2ND_BIT);
	regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_ON_BIT);
}

static int scpsys_modem_pwrseq_on(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;
	bool tmp;
	int ret;

	if (!MTK_SCPD_CAPS(pd, MTK_SCPD_SKIP_RESET_B))
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RST_B_BIT);

	regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_ON_BIT);

	/* wait until PWR_ACK = 1 */
	ret = readx_poll_timeout(scpsys_domain_is_on, pd, tmp, tmp, MTK_POLL_DELAY_US,
				 MTK_POLL_TIMEOUT);
	if (ret < 0)
		return ret;

	return 0;
}

static void scpsys_modem_pwrseq_off(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;

	regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_ON_BIT);

	if (!MTK_SCPD_CAPS(pd, MTK_SCPD_SKIP_RESET_B))
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RST_B_BIT);
}

/* MT6878 DISP: open the downstream subsys clock gates, see disp_on */
static void scpsys_open_subsys_cg(struct scpsys_domain *pd)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pd->subsys_cg); i++)
		if (pd->subsys_cg[i] && pd->data->subsys_cg_mask[i])
			writel(pd->data->subsys_cg_mask[i],
			       pd->subsys_cg[i] + SUBSYS_CG_CLR);
}

/* MT6878 DISP: does the mmsys crossbar block take a write? See disp_on */
static void scpsys_disp_probe(struct scpsys_domain *pd, unsigned int n,
			      const char *when)
{
	const struct scpsys_bus_prot_data *bpd = &pd->data->bp_cfg[0];
	void __iomem *mm = pd->subsys_cg[0];
	u32 sta = 0, cg1 = 0, test;

	if (!mm)
		return;
	if (bpd->bus_prot_set_clr_mask)
		regmap_read(scpsys_bus_protect_get_sta_regmap(pd, bpd),
			    bpd->bus_prot_sta, &sta);
	if (pd->subsys_cg[1])
		cg1 = readl(pd->subsys_cg[1] + SUBSYS_CG_CON);

	/* an output enable no path uses; 0 at boot, put back at once */
	writel(0x00010001, mm + MMSYS_OVL_BG_CROSSBAR2);
	test = readl(mm + MMSYS_OVL_BG_CROSSBAR2);
	writel(0, mm + MMSYS_OVL_BG_CROSSBAR2);

	dev_info(pd->scpsys->dev, "%s on (#%u) %s: bus prot sta %08x (domain bits %08x), mmsys cg %08x, mdpsys cg %08x, 0x0fc %08x, 0xc00 %08x, 0xc0c %08x, 0xc18 written 00010001 reads %08x\n",
		 pd->genpd.name, n, when, sta,
		 sta & bpd->bus_prot_sta_mask, readl(mm + SUBSYS_CG_CON), cg1,
		 readl(mm + MMSYS_SEC_STA), readl(mm + MMSYS_BYPASS_MUX_SHADOW),
		 readl(mm + MMSYS_CROSSBAR_CON), test);
}

static int scpsys_power_on_internal(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;
	unsigned int n;
	bool probe;
	int ret;

	ret = scpsys_regulator_enable(pd->supply);
	if (ret)
		return ret;

	if (pd->supply && pd->data->rail_iso) {
		u32 val;

		scpsys_rail_iso_release(scpsys, pd->data->rail_iso);
		regmap_read(scpsys->base, pd->data->rail_iso->sta_offs, &val);
		if (val & scpsys_rail_iso_mask(pd->data->rail_iso)) {
			dev_err(scpsys->dev, "%s: rail isolation still set: %#x\n",
				pd->genpd.name, val);
			ret = -EIO;
			goto err_reg;
		}
	}

	ret = clk_bulk_prepare_enable(pd->num_clks, pd->clks);
	if (ret)
		goto err_reg;

	if (pd->data->ext_buck_iso_offs && MTK_SCPD_CAPS(pd, MTK_SCPD_EXT_BUCK_ISO))
		regmap_clear_bits(scpsys->base, pd->data->ext_buck_iso_offs,
				  pd->data->ext_buck_iso_mask);

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_MODEM_PWRSEQ))
		ret = scpsys_modem_pwrseq_on(pd);
	else
		ret = scpsys_ctl_pwrseq_on(pd);

	if (ret)
		goto err_pwr_ack;

	/*
	 * In MT8189 mminfra power domain, the bus protect policy separates
	 * into two parts, one is set before subsys clocks enabled, and another
	 * need to enable after subsys clocks enable.
	 */
	ret = scpsys_bus_protect_disable(pd, BUS_PROT_IGNORE_SUBCLK);
	if (ret < 0)
		goto err_pwr_ack;

	/*
	 * In few Mediatek platforms(e.g. MT6779), the bus protect policy is
	 * stricter, which leads to bus protect release must be prior to bus
	 * access.
	 */
	if (!MTK_SCPD_CAPS(pd, MTK_SCPD_STRICT_BUS_PROTECTION)) {
		ret = clk_bulk_prepare_enable(pd->num_subsys_clks,
					      pd->subsys_clks);
		if (ret)
			goto err_pwr_ack;
	}

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_MT6878_DISP_ON) &&
	    (READ_ONCE(disp_on) & DISP_ON_SUBSYS_CG))
		scpsys_open_subsys_cg(pd);

	ret = scpsys_sram_enable(pd);
	if (ret < 0)
		goto err_disable_subsys_clks;

	ret = scpsys_bus_protect_disable(pd, 0);
	if (ret < 0)
		goto err_disable_sram;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_STRICT_BUS_PROTECTION)) {
		ret = clk_bulk_prepare_enable(pd->num_subsys_clks,
					      pd->subsys_clks);
		if (ret)
			goto err_enable_bus_protect;
	}

	probe = MTK_SCPD_CAPS(pd, MTK_SCPD_MT6878_DISP_ON) &&
		(READ_ONCE(disp_on) & DISP_ON_PROBE);
	n = pd->power_ons + 1;
	if (probe)
		scpsys_disp_probe(pd, n, "before the firmware call");

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_MMINFRA_ON_SIP))
		scpsys_mminfra_powered_on(pd);

	if (probe)
		scpsys_disp_probe(pd, n, "after the firmware call");

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_MT6878_DISP_ON) &&
	    (READ_ONCE(disp_on) & DISP_ON_DISP_VA))
		scpsys_disp_va(pd);

	return 0;

err_enable_bus_protect:
	scpsys_bus_protect_enable(pd, 0);
err_disable_sram:
	scpsys_sram_disable(pd);
err_disable_subsys_clks:
	if (!MTK_SCPD_CAPS(pd, MTK_SCPD_STRICT_BUS_PROTECTION))
		clk_bulk_disable_unprepare(pd->num_subsys_clks,
					   pd->subsys_clks);
err_pwr_ack:
	clk_bulk_disable_unprepare(pd->num_clks, pd->clks);
err_reg:
	scpsys_regulator_disable(pd->supply);
	return ret;
}

/* MT6878 DISP: the bus protection could not be set, see disp_on */
static void scpsys_disp_off_failed(struct scpsys_domain *pd, int err)
{
	const struct scpsys_bus_prot_data *bpd = &pd->data->bp_cfg[0];
	u32 sta = 0;

	regmap_read(scpsys_bus_protect_get_sta_regmap(pd, bpd),
		    bpd->bus_prot_sta, &sta);
	dev_warn(pd->scpsys->dev, "%s off failed: %d, bus prot sta %08x (domain bits %08x of %08x), mmsys cg %08x; protection released again, the domain stays on\n",
		 pd->genpd.name, err, sta, sta & bpd->bus_prot_sta_mask,
		 bpd->bus_prot_sta_mask,
		 pd->subsys_cg[0] ? readl(pd->subsys_cg[0] + SUBSYS_CG_CON) : 0);
	/* the masters would stay cut off from the bus while the domain is on */
	scpsys_bus_protect_disable(pd, 0);
}

static int scpsys_power_off_internal(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;
	bool tmp;
	int ret;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_MT6878_DISP_ON) &&
	    (READ_ONCE(disp_on) & DISP_ON_SUBSYS_CG))
		scpsys_open_subsys_cg(pd);

	ret = scpsys_bus_protect_enable(pd, 0);
	if (ret < 0) {
		if (MTK_SCPD_CAPS(pd, MTK_SCPD_MT6878_DISP_ON))
			scpsys_disp_off_failed(pd, ret);
		return ret;
	}

	ret = scpsys_sram_disable(pd);
	if (ret < 0)
		return ret;

	if (pd->data->ext_buck_iso_offs && MTK_SCPD_CAPS(pd, MTK_SCPD_EXT_BUCK_ISO))
		regmap_set_bits(scpsys->base, pd->data->ext_buck_iso_offs,
				pd->data->ext_buck_iso_mask);

	clk_bulk_disable_unprepare(pd->num_subsys_clks, pd->subsys_clks);

	/*
	 * A failure from here on leaves the domain on for genpd: take the
	 * subsys clocks again so that a later power off releases them once.
	 */
	ret = scpsys_bus_protect_enable(pd, BUS_PROT_IGNORE_SUBCLK);
	if (ret < 0)
		goto err_subsys_clks;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_MODEM_PWRSEQ))
		scpsys_modem_pwrseq_off(pd);
	else
		scpsys_ctl_pwrseq_off(pd);

	/* wait until PWR_ACK = 0 */
	ret = readx_poll_timeout(scpsys_domain_is_on, pd, tmp, !tmp, MTK_POLL_DELAY_US,
				 MTK_POLL_TIMEOUT);
	if (ret < 0)
		goto err_subsys_clks;

	clk_bulk_disable_unprepare(pd->num_clks, pd->clks);

	scpsys_regulator_disable(pd->supply);

	return 0;

err_subsys_clks:
	if (clk_bulk_prepare_enable(pd->num_subsys_clks, pd->subsys_clks))
		dev_err(scpsys->dev, "%s: subsys clocks not restored\n",
			pd->genpd.name);
	return ret;
}

static int scpsys_power_on(struct generic_pm_domain *genpd)
{
	struct scpsys_domain *pd = container_of(genpd, struct scpsys_domain, genpd);
	int ret;

	if (pd->powered)
		return 0;

	ret = scpsys_power_on_internal(pd);
	if (ret) {
		/* genpd does not report it in its system resume path */
		dev_err(pd->scpsys->dev, "%s: power on failed: %d\n",
			genpd->name, ret);
		return ret;
	}
	pd->powered = true;
	return 0;
}

static int scpsys_power_off(struct generic_pm_domain *genpd)
{
	struct scpsys_domain *pd = container_of(genpd, struct scpsys_domain, genpd);
	int ret;

	/* marked on by genpd after a power on that failed: nothing to undo */
	if (!pd->powered) {
		dev_dbg(pd->scpsys->dev, "%s: was not powered on\n", genpd->name);
		return 0;
	}

	ret = scpsys_power_off_internal(pd);
	if (!ret)
		pd->powered = false;
	return ret;
}

static bool scpsys_domain_data_is_on(struct scpsys *scpsys,
				     const struct scpsys_domain_data *data)
{
	u32 mask2 = data->sta2nd_mask ? data->sta2nd_mask : data->sta_mask;
	u32 status, status2;

	regmap_read(scpsys->base, data->pwr_sta_offs, &status);
	regmap_read(scpsys->base, data->pwr_sta2nd_offs, &status2);

	return (status & data->sta_mask) && (status2 & mask2);
}

/*
 * A domain on an isolated external rail: follow the rail with the
 * isolation. If the rail is up while no domain on it is (left so by the
 * boot loader), set the isolation now: a later disable of the unused rail
 * (regulator late cleanup) does not go through the notifier.
 */
static int scpsys_rail_iso_init(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;
	const struct scpsys_rail_iso *iso = pd->data->rail_iso;
	const struct scpsys_soc_data *soc = scpsys->soc_data;
	bool users_on = false;
	u32 val;
	int i, ret;

	for (i = 0; i < soc->num_domains; i++)
		if (soc->domains_data[i].rail_iso == iso &&
		    scpsys_domain_data_is_on(scpsys, &soc->domains_data[i]))
			users_on = true;

	ret = regulator_is_enabled(pd->supply);
	regmap_read(scpsys->base, iso->sta_offs, &val);
	dev_info(scpsys->dev, "%s: rail %s, isolation %#x, domains on the rail %s\n",
		 pd->data->name, ret > 0 ? "on" : "off", val & scpsys_rail_iso_mask(iso),
		 users_on ? "on" : "off");

	if (ret > 0 && !users_on)
		scpsys_rail_iso_set(scpsys, iso);

	pd->rail_nb.notifier_call = scpsys_rail_event;
	return devm_regulator_register_notifier(pd->supply, &pd->rail_nb);
}

static struct
generic_pm_domain *scpsys_add_one_domain(struct scpsys *scpsys, struct device_node *node)
{
	const struct scpsys_domain_data *domain_data;
	const struct scpsys_hwv_domain_data *hwv_domain_data;
	struct scpsys_domain *pd;
	struct property *prop;
	const char *clk_name;
	int i, ret, num_clks;
	struct clk *clk;
	int clk_ind = 0;
	u32 id;

	ret = of_property_read_u32(node, "reg", &id);
	if (ret) {
		dev_err(scpsys->dev, "%pOF: failed to retrieve domain id from reg: %d\n",
			node, ret);
		return ERR_PTR(-EINVAL);
	}

	switch (scpsys->soc_data->type) {
	case SCPSYS_MTCMOS_TYPE_DIRECT_CTL:
		if (id >= scpsys->soc_data->num_domains) {
			dev_err(scpsys->dev, "%pOF: invalid domain id %d\n", node, id);
			return ERR_PTR(-EINVAL);
		}

		domain_data = &scpsys->soc_data->domains_data[id];
		hwv_domain_data = NULL;

		/*
		 * Some controllers have both direct control and HW Voter
		 * domains in the same index space: an id that has no direct
		 * control data may be a HWV domain.
		 */
		if (domain_data->sta_mask == 0 &&
		    id < scpsys->soc_data->num_hwv_domains &&
		    scpsys->soc_data->hwv_domains_data[id].name) {
			domain_data = NULL;
			hwv_domain_data = &scpsys->soc_data->hwv_domains_data[id];
		}

		if (domain_data && domain_data->sta_mask == 0) {
			dev_err(scpsys->dev, "%pOF: undefined domain id %d\n", node, id);
			return ERR_PTR(-EINVAL);
		}

		break;
	case SCPSYS_MTCMOS_TYPE_HW_VOTER:
		if (id >= scpsys->soc_data->num_hwv_domains) {
			dev_err(scpsys->dev, "%pOF: invalid HWV domain id %d\n", node, id);
			return ERR_PTR(-EINVAL);
		}

		domain_data = NULL;
		hwv_domain_data = &scpsys->soc_data->hwv_domains_data[id];

		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	pd = devm_kzalloc(scpsys->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return ERR_PTR(-ENOMEM);

	pd->data = domain_data;
	pd->hwv_data = hwv_domain_data;
	pd->scpsys = scpsys;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_DOMAIN_SUPPLY)) {
		pd->supply = devm_of_regulator_get_optional(scpsys->dev, node, "domain");
		if (IS_ERR(pd->supply))
			return dev_err_cast_probe(scpsys->dev, pd->supply,
				      "%pOF: failed to get power supply.\n",
				      node);

		if (pd->data && pd->data->rail_iso) {
			ret = scpsys_rail_iso_init(pd);
			if (ret)
				return dev_err_ptr_probe(scpsys->dev, ret,
					"%pOF: failed to watch the supply\n", node);
		}
	}

	num_clks = of_clk_get_parent_count(node);
	if (num_clks > 0) {
		/* Calculate number of subsys_clks */
		of_property_for_each_string(node, "clock-names", prop, clk_name) {
			char *subsys;

			subsys = strchr(clk_name, '-');
			if (subsys)
				pd->num_subsys_clks++;
			else
				pd->num_clks++;
		}

		pd->clks = devm_kcalloc(scpsys->dev, pd->num_clks, sizeof(*pd->clks), GFP_KERNEL);
		if (!pd->clks)
			return ERR_PTR(-ENOMEM);

		pd->subsys_clks = devm_kcalloc(scpsys->dev, pd->num_subsys_clks,
					       sizeof(*pd->subsys_clks), GFP_KERNEL);
		if (!pd->subsys_clks)
			return ERR_PTR(-ENOMEM);

	}

	for (i = 0; i < pd->num_clks; i++) {
		clk = of_clk_get(node, i);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			dev_err_probe(scpsys->dev, ret,
				      "%pOF: failed to get clk at index %d\n", node, i);
			goto err_put_clocks;
		}

		pd->clks[clk_ind++].clk = clk;
	}

	for (i = 0; i < pd->num_subsys_clks; i++) {
		clk = of_clk_get(node, i + clk_ind);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			dev_err_probe(scpsys->dev, ret,
				      "%pOF: failed to get clk at index %d\n", node,
				      i + clk_ind);
			goto err_put_subsys_clocks;
		}

		pd->subsys_clks[i].clk = clk;
	}

	if (scpsys->domains[id]) {
		ret = -EINVAL;
		dev_err(scpsys->dev,
			"power domain with id %d already exists, check your device-tree\n", id);
		goto err_put_subsys_clocks;
	}

	if (pd->data && MTK_SCPD_CAPS(pd, MTK_SCPD_MT6878_DISP_ON)) {
		for (i = 0; i < ARRAY_SIZE(pd->subsys_cg); i++) {
			if (!pd->data->subsys_cg_base[i])
				continue;
			pd->subsys_cg[i] = devm_ioremap(scpsys->dev,
							pd->data->subsys_cg_base[i],
							SZ_4K);
			if (!pd->subsys_cg[i])
				dev_warn(scpsys->dev, "%pOF: cannot map clock gates %d\n",
					 node, i);
		}
	}

	if (pd->hwv_data && pd->hwv_data->cg_base) {
		pd->cg_base = devm_ioremap(scpsys->dev, pd->hwv_data->cg_base, SZ_4K);
		if (!pd->cg_base)
			dev_warn(scpsys->dev, "%pOF: cannot map the clock gates\n", node);
		else if (MTK_SCPD_CAPS(pd, MTK_SCPD_MMINFRA_ON_SIP))
			scpsys_mminfra_cg = pd->cg_base;
	}

	/* DISP_ON_DISP_VA: the GCE-D tokens */
	if (pd->data && MTK_SCPD_CAPS(pd, MTK_SCPD_MT6878_DISP_ON)) {
		scpsys_gce_d = devm_ioremap(scpsys->dev, MT6878_GCE_D_PA, SZ_4K);
		scpsys_disp_pd = pd;
	}

	if (pd->data && pd->data->name)
		pd->genpd.name = pd->data->name;
	else if (pd->hwv_data && pd->hwv_data->name)
		pd->genpd.name = pd->hwv_data->name;
	else
		pd->genpd.name = node->name;

	if (!pd->hwv_data) {
		pd->genpd.power_off = scpsys_power_off;
		pd->genpd.power_on = scpsys_power_on;
	} else {
		pd->genpd.power_off = scpsys_hwv_power_off;
		pd->genpd.power_on = scpsys_hwv_power_on;

		/*
		 * HW-Voter code can be invoked in atomic context, but not in
		 * a mixed controller: there the HWV domains are parents of
		 * direct control ones and take clocks that need to sleep.
		 */
		if (scpsys->soc_data->type == SCPSYS_MTCMOS_TYPE_HW_VOTER)
			pd->genpd.flags |= GENPD_FLAG_IRQ_SAFE;
	}

	/*
	 * Initially turn on all domains to make the domains usable
	 * with !CONFIG_PM and to get the hardware in sync with the
	 * software.  The unused domains will be switched off during
	 * late_init time.
	 */
	if (MTK_SCPD_CAPS(pd, MTK_SCPD_KEEP_DEFAULT_OFF)) {
		bool domain_is_on;

		if (pd->hwv_data)
			domain_is_on = scpsys_hwv_domain_is_enable_done(pd);
		else
			domain_is_on = scpsys_domain_is_on(pd);

		if (domain_is_on)
			dev_warn(scpsys->dev,
				 "%pOF: A default off power domain has been ON\n", node);
	} else {
		if (pd->hwv_data)
			pd->boot_on = scpsys_hwv_domain_is_enable_done(pd);
		else
			pd->boot_on = scpsys_domain_is_on(pd);

		ret = pd->genpd.power_on(&pd->genpd);
		if (ret < 0) {
			dev_err(scpsys->dev, "%pOF: failed to power on domain: %d\n", node, ret);
			goto err_put_subsys_clocks;
		}

		if (MTK_SCPD_CAPS(pd, MTK_SCPD_ALWAYS_ON))
			pd->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
	}

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_ACTIVE_WAKEUP))
		pd->genpd.flags |= GENPD_FLAG_ACTIVE_WAKEUP;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_KEEP_DEFAULT_OFF))
		pm_genpd_init(&pd->genpd, NULL, true);
	else
		pm_genpd_init(&pd->genpd, NULL, false);

	scpsys->domains[id] = &pd->genpd;

	return scpsys->pd_data.domains[id];

err_put_subsys_clocks:
	clk_bulk_put(pd->num_subsys_clks, pd->subsys_clks);
err_put_clocks:
	clk_bulk_put(pd->num_clks, pd->clks);
	return ERR_PTR(ret);
}

static int scpsys_add_subdomain(struct scpsys *scpsys, struct device_node *parent)
{
	struct generic_pm_domain *child_pd, *parent_pd;
	struct device_node *child;
	int ret;

	/* a disabled child domain is skipped with its whole subtree */
	for_each_available_child_of_node(parent, child) {
		u32 id;

		ret = of_property_read_u32(parent, "reg", &id);
		if (ret) {
			dev_err(scpsys->dev, "%pOF: failed to get parent domain id\n", child);
			goto err_put_node;
		}

		if (!scpsys->pd_data.domains[id]) {
			ret = -EINVAL;
			dev_err(scpsys->dev, "power domain with id %d does not exist\n", id);
			goto err_put_node;
		}

		parent_pd = scpsys->pd_data.domains[id];

		child_pd = scpsys_add_one_domain(scpsys, child);
		if (IS_ERR(child_pd)) {
			ret = PTR_ERR(child_pd);
			dev_err_probe(scpsys->dev, ret, "%pOF: failed to get child domain id\n",
				      child);
			goto err_put_node;
		}

		/* recursive call to add all subdomains */
		ret = scpsys_add_subdomain(scpsys, child);
		if (ret)
			goto err_put_node;

		ret = pm_genpd_add_subdomain(parent_pd, child_pd);
		if (ret) {
			dev_err(scpsys->dev, "failed to add %s subdomain to parent %s\n",
				child_pd->name, parent_pd->name);
			goto err_put_node;
		} else {
			dev_dbg(scpsys->dev, "%s add subdomain: %s\n", parent_pd->name,
				child_pd->name);
		}
	}

	return 0;

err_put_node:
	of_node_put(child);
	return ret;
}

static void scpsys_remove_one_domain(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;
	int ret;

	/*
	 * We're in the error cleanup already, so we only complain,
	 * but won't emit another error on top of the original one.
	 */
	ret = pm_genpd_remove(&pd->genpd);
	if (ret < 0)
		dev_err(scpsys->dev,
			"failed to remove domain '%s' : %d - state may be inconsistent\n",
			pd->genpd.name, ret);

	/*
	 * A default-off domain that Linux never turned on is left as it was
	 * found: what is behind it may have been started by the boot loader
	 * (e.g. a modem) and is not ours to switch off.
	 */
	if (MTK_SCPD_CAPS(pd, MTK_SCPD_KEEP_DEFAULT_OFF) &&
	    pd->genpd.status == GENPD_STATE_OFF)
		goto put_clocks;

	/*
	 * Neither is a domain that was already on when the probe found it: a
	 * probe that fails (most often deferred, e.g. for the regulator of a
	 * later domain) would otherwise power cycle what the boot loader runs
	 * behind it, such as the display path, before the next attempt. Its
	 * clocks keep the references this probe took, so they stay running
	 * as the boot loader left them. A domain with a supply is switched off
	 * as before: its regulator reference must not be dropped while enabled.
	 */
	if (pd->boot_on && !pd->supply) {
		dev_dbg(scpsys->dev, "%s: left on, it was on at probe\n",
			pd->genpd.name);
		return;
	}

	if (pd->hwv_data) {
		if (scpsys_hwv_domain_is_enable_done(pd))
			scpsys_hwv_power_off_internal(pd);
	} else {
		if (scpsys_domain_is_on(pd))
			scpsys_power_off_internal(pd);
	}

put_clocks:
	clk_bulk_put(pd->num_clks, pd->clks);
	clk_bulk_put(pd->num_subsys_clks, pd->subsys_clks);
}

static void scpsys_domain_cleanup(struct scpsys *scpsys)
{
	struct generic_pm_domain *genpd;
	struct scpsys_domain *pd;
	int i;

	for (i = scpsys->pd_data.num_domains - 1; i >= 0; i--) {
		genpd = scpsys->pd_data.domains[i];
		if (genpd) {
			pd = to_scpsys_domain(genpd);
			scpsys_remove_one_domain(pd);
		}
	}
}

static int scpsys_get_bus_protection_legacy(struct device *dev, struct scpsys *scpsys)
{
	const u8 bp_blocks[3] = {
		BUS_PROT_BLOCK_INFRA, BUS_PROT_BLOCK_SMI, BUS_PROT_BLOCK_INFRA_NAO
	};
	struct device_node *np = dev->of_node;
	struct device_node *node, *smi_np;
	int num_regmaps = 0, i, j;
	struct regmap *regmap[3];
	int ret = 0;

	/*
	 * Legacy code retrieves a maximum of three bus protection handles:
	 * some may be optional, or may not be, so the array of bp blocks
	 * that is normally passed in as platform data must be dynamically
	 * built in this case.
	 *
	 * Here, try to retrieve all of the regmaps that the legacy code
	 * supported and then count the number of the ones that are present,
	 * this makes it then possible to allocate the array of bus_prot
	 * regmaps and convert all to the new style handling.
	 */
	of_node_get(np);
	node = of_find_node_with_property(np, "mediatek,infracfg");
	if (node) {
		regmap[0] = syscon_regmap_lookup_by_phandle(node, "mediatek,infracfg");
		num_regmaps++;
		if (IS_ERR(regmap[0])) {
			ret = dev_err_probe(dev, PTR_ERR(regmap[0]),
					     "%pOF: failed to get infracfg regmap\n",
					     node);
			of_node_put(node);
			return ret;
		}
		of_node_put(node);
	} else {
		regmap[0] = NULL;
	}

	of_node_get(np);
	node = of_find_node_with_property(np, "mediatek,smi");
	if (node) {
		smi_np = of_parse_phandle(node, "mediatek,smi", 0);
		if (!smi_np) {
			of_node_put(node);
			return -ENODEV;
		}

		regmap[1] = device_node_to_regmap(smi_np);
		num_regmaps++;
		of_node_put(smi_np);
		if (IS_ERR(regmap[1])) {
			ret = dev_err_probe(dev, PTR_ERR(regmap[1]),
					     "%pOF: failed to get SMI regmap\n",
					     node);
			of_node_put(node);
			return ret;
		}
		of_node_put(node);
	} else {
		regmap[1] = NULL;
	}

	of_node_get(np);
	node = of_find_node_with_property(np, "mediatek,infracfg-nao");
	if (node) {
		regmap[2] = syscon_regmap_lookup_by_phandle(node, "mediatek,infracfg-nao");
		num_regmaps++;
		if (IS_ERR(regmap[2])) {
			ret = dev_err_probe(dev, PTR_ERR(regmap[2]),
					     "%pOF: failed to get infracfg regmap\n",
					     node);
			of_node_put(node);
			return ret;
		}
		of_node_put(node);
	} else {
		regmap[2] = NULL;
	}

	scpsys->bus_prot = devm_kmalloc_array(dev, num_regmaps,
					      sizeof(*scpsys->bus_prot), GFP_KERNEL);
	if (!scpsys->bus_prot)
		return -ENOMEM;

	for (i = 0, j = 0; i < ARRAY_SIZE(bp_blocks); i++) {
		enum scpsys_bus_prot_block bp_type;

		if (!regmap[i])
			continue;

		bp_type = bp_blocks[i];
		scpsys->bus_prot_index[bp_type] = j;
		scpsys->bus_prot[j] = regmap[i];

		j++;
	}

	return 0;
}

static int scpsys_get_bus_protection(struct device *dev, struct scpsys *scpsys)
{
	const struct scpsys_soc_data *soc = scpsys->soc_data;
	struct device_node *np = dev->of_node;
	int i, num_handles;

	num_handles = of_count_phandle_with_args(np, "access-controllers", NULL);
	if (num_handles < 0 || num_handles != soc->num_bus_prot_blocks)
		return dev_err_probe(dev, -EINVAL,
				     "Cannot get access controllers: expected %u, got %d\n",
				     soc->num_bus_prot_blocks, num_handles);

	scpsys->bus_prot = devm_kmalloc_array(dev, soc->num_bus_prot_blocks,
					      sizeof(*scpsys->bus_prot), GFP_KERNEL);
	if (!scpsys->bus_prot)
		return -ENOMEM;

	for (i = 0; i < soc->num_bus_prot_blocks; i++) {
		enum scpsys_bus_prot_block bp_type;
		struct device_node *node;

		node = of_parse_phandle(np, "access-controllers", i);
		if (!node)
			return -EINVAL;

		/*
		 * Index the bus protection regmaps so that we don't have to
		 * find the right one by type with a loop at every execution
		 * of power sequence(s).
		 */
		bp_type = soc->bus_prot_blocks[i];
		scpsys->bus_prot_index[bp_type] = i;

		scpsys->bus_prot[i] = device_node_to_regmap(node);
		of_node_put(node);
		if (IS_ERR_OR_NULL(scpsys->bus_prot[i]))
			return dev_err_probe(dev, scpsys->bus_prot[i] ?
					     PTR_ERR(scpsys->bus_prot[i]) : -ENXIO,
					     "Cannot get regmap for access controller %d\n", i);
	}

	return 0;
}

static const struct of_device_id scpsys_of_match[] = {
	{
		.compatible = "mediatek,mt6735-power-controller",
		.data = &mt6735_scpsys_data,
	},
	{
		.compatible = "mediatek,mt6795-power-controller",
		.data = &mt6795_scpsys_data,
	},
	{
		.compatible = "mediatek,mt6878-power-controller",
		.data = &mt6878_scpsys_data,
	},
	{
		.compatible = "mediatek,mt6893-power-controller",
		.data = &mt6893_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8167-power-controller",
		.data = &mt8167_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8173-power-controller",
		.data = &mt8173_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8183-power-controller",
		.data = &mt8183_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8186-power-controller",
		.data = &mt8186_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8188-power-controller",
		.data = &mt8188_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8189-power-controller",
		.data = &mt8189_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8192-power-controller",
		.data = &mt8192_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8195-power-controller",
		.data = &mt8195_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8196-power-controller",
		.data = &mt8196_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8196-hwv-hfrp-power-controller",
		.data = &mt8196_hfrpsys_hwv_data,
	},
	{
		.compatible = "mediatek,mt8196-hwv-scp-power-controller",
		.data = &mt8196_scpsys_hwv_data,
	},
	{
		.compatible = "mediatek,mt8365-power-controller",
		.data = &mt8365_scpsys_data,
	},
	{ }
};

static int scpsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct scpsys_soc_data *soc;
	struct device *parent;
	struct scpsys *scpsys;
	int num_domains, ret;

	soc = of_device_get_match_data(&pdev->dev);
	if (!soc) {
		dev_err(&pdev->dev, "no power controller data\n");
		return -EINVAL;
	}

	/* In a mixed controller both kinds of domains share the index space */
	num_domains = max(soc->num_domains, soc->num_hwv_domains);

	scpsys = devm_kzalloc(dev, struct_size(scpsys, domains, num_domains), GFP_KERNEL);
	if (!scpsys)
		return -ENOMEM;

	scpsys->dev = dev;
	scpsys->soc_data = soc;

	scpsys->pd_data.domains = scpsys->domains;
	scpsys->pd_data.num_domains = num_domains;

	parent = dev->parent;
	if (!parent) {
		dev_err(dev, "no parent for syscon devices\n");
		return -ENODEV;
	}

	scpsys->base = syscon_node_to_regmap(parent->of_node);
	if (IS_ERR(scpsys->base)) {
		dev_err(dev, "no regmap available\n");
		return PTR_ERR(scpsys->base);
	}

	/*
	 * HWV controllers live in the HWV register space; a direct control
	 * controller that also has HWV domains points to the voter instead.
	 */
	if (soc->type == SCPSYS_MTCMOS_TYPE_DIRECT_CTL && soc->num_hwv_domains) {
		scpsys->hwv_base = syscon_regmap_lookup_by_phandle(np, "mediatek,hardware-voter");
		if (IS_ERR(scpsys->hwv_base))
			return dev_err_probe(dev, PTR_ERR(scpsys->hwv_base),
					     "no hardware voter regmap available\n");
	} else {
		scpsys->hwv_base = scpsys->base;
	}

	if (of_find_property(np, "access-controllers", NULL))
		ret = scpsys_get_bus_protection(dev, scpsys);
	else
		ret = scpsys_get_bus_protection_legacy(dev, scpsys);

	if (ret)
		return ret;

	ret = -ENODEV;
	for_each_available_child_of_node_scoped(np, node) {
		struct generic_pm_domain *domain;

		domain = scpsys_add_one_domain(scpsys, node);
		if (IS_ERR(domain)) {
			ret = PTR_ERR(domain);
			goto err_cleanup_domains;
		}

		ret = scpsys_add_subdomain(scpsys, node);
		if (ret)
			goto err_cleanup_domains;
	}

	if (ret) {
		dev_dbg(dev, "no power domains present\n");
		return ret;
	}

	ret = of_genpd_add_provider_onecell(np, &scpsys->pd_data);
	if (ret) {
		dev_err(dev, "failed to add provider: %d\n", ret);
		goto err_cleanup_domains;
	}

	return 0;

err_cleanup_domains:
	scpsys_domain_cleanup(scpsys);
	return ret;
}

static struct platform_driver scpsys_pm_domain_driver = {
	.probe = scpsys_probe,
	.driver = {
		.name = "mtk-power-controller",
		.suppress_bind_attrs = true,
		.of_match_table = scpsys_of_match,
	},
};
builtin_platform_driver(scpsys_pm_domain_driver);
