// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6878 low power manager: the kernel side of the system low
 * power handshake with TF-A (BL31) and the SPM for suspend-to-idle.
 *
 * On these SoCs system sleep is suspend-to-idle: every CPU enters the PSCI
 * state 0x020180ff and TF-A, seeing all online cores in it, asks the SPM
 * firmware for the deepest "constraint" the hardware allows (bus26m,
 * syspll or vcore: 26 MHz off, PLLs off, DRAM self refresh, vcore low).
 * The kernel has to tell TF-A a few things for that to work the way the
 * downstream kernel does:
 *
 *  - which CPU is entering suspend (SUSPEND_SRC) around each s2idle entry,
 *  - that the platform is ready for the CPU power states (VALIDATE_PWR_STATE),
 *  - the SPM interrupt TF-A sets pending on wake-up so that the wake-up is
 *    seen by the kernel (SET_PENDING_IRQ_INIT),
 *  - the edge/level interrupts to mirror into the SYS_CIRQ while the GIC is
 *    powered down ("irq remain"),
 *  - the resource requests and constraints to enable (DOE_RESOURCE_CTRL,
 *    DOE_RC).
 *
 * After every wake-up the SPM backup registers say what woke the system
 * and how deep it went; they are printed and kept in sysfs for the
 * user space sleep policy.
 *
 * The SiP interface (function 0x507, the 0xDA000000 user ids and their
 * actions) and the SPM register layout are those of MediaTek's GPL-2.0
 * mtk_lpm driver in kernel_device_modules-6.1 (drivers/misc/mediatek/lpm,
 * Copyright (c) 2019, 2023 MediaTek Inc.) and of the BSD-3-Clause
 * plat/mediatek headers of Trusted Firmware-A (Copyright (c) 2023-2025
 * MediaTek Inc.).
 *
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 */

#include <linux/arm-smccc.h>
#include <linux/clk-provider.h>
#include <linux/bitops.h>
#include <linux/cpuidle.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/seq_buf.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/sysfs.h>

/*
 * Switches, for a kernel built with the driver in: enable=0 keeps the
 * firmware as the boot loader left it (no handshake at all); runtime=0
 * (the default) leaves out what changes the behaviour of the ordinary idle
 * as well, not only of system sleep: the platform-ready word to TF-A, the
 * constraints (26M/PLL/vcore off at MCUSYS off) and the interrupts kept
 * through MCUSYS off. The suspend source, the SPM wake-up interrupt and
 * the wake-up report stay. Constraints can also be switched one by one
 * through sysfs (lpm/constraints) once sleep has been seen working.
 */
static bool enable = true;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Talk to the firmware at all (default: yes)");

static bool runtime;
module_param(runtime, bool, 0444);
MODULE_PARM_DESC(runtime, "Also the settings that change ordinary idle: platform ready, constraints, irq remain (default: no)");

/*
 * With runtime=0 the constraints are left as the firmware booted, which
 * may be off: then no suspend reaches 26 MHz off whoever else lets go.
 * The downstream kernel enables vcore, bus26m and syspll at boot (DOE_RC,
 * device tree "constraint-list", value 1). suspend_rc=1 enables the ones
 * found off for the time of a suspend only (between the late suspend and
 * the early resume), and switches them back off afterwards.
 */
static bool suspend_rc = true;
module_param(suspend_rc, bool, 0644);
MODULE_PARM_DESC(suspend_rc, "With runtime=0: enable the constraints found off for the time of a suspend (default: yes)");

/*
 * The downstream kernel releases the eight SPM resources TF-A can be told to
 * keep (DOE_RESOURCE_CTRL id 0..7, value 0 from the factory device tree's
 * resource-ctrl-list) at boot, whatever else it does. res_ctrl=1 sends
 * the same before every suspend (idempotent, also with runtime=0);
 * 0 leaves them as the firmware booted, as before.
 */
static bool res_ctrl = true;
module_param(res_ctrl, bool, 0644);
MODULE_PARM_DESC(res_ctrl, "Release the eight SPM resources before a suspend, as the factory boot does (default: yes)");

/*
 * The downstream late init also tells TF-A that the platform is ready for the
 * CPU power states (VALIDATE_PWR_STATE_CTRL). With runtime=0 that is not
 * sent; validate=1 sends it once, at the first suspend. It stays for the
 * ordinary idle too (no way back), so it is off by default.
 */
static bool validate;
module_param(validate, bool, 0644);
MODULE_PARM_DESC(validate, "With runtime=0: send the platform-ready word at the first suspend (default: no)");

/*
 * The board DT boots with clk_ignore_unused (the display path and the USB
 * gadget of the boot loader need clocks no driver holds), the downstream kernel
 * without ("clk: Disabling unused clocks"). Every clock LK left on stays on,
 * and the SPM's cg_check keeps 26M for the infra/peri gates among them.
 * sleep_gate_unused: comma separated clock name prefixes whose unused gates
 * are gated from the end of the device suspend to the start of the resume
 * (clk_sleep_gate_unused()); empty (default): nothing. Pick always-on
 * providers only (infracfg_ao "ifrao_", pericfg_ao "peraop_"): their gate
 * state is read. sleep_gate_exclude: name prefixes never touched.
 */
static char *sleep_gate_unused = "";
module_param(sleep_gate_unused, charp, 0644);
MODULE_PARM_DESC(sleep_gate_unused,
		 "Clock name prefixes whose unused gates are closed for the sleep, e.g. ifrao_,peraop_ (default empty: none)");

/*
 * The SPM's own hardware request sources besides the subsystem lines:
 * clock gate (MODULE_SW_CG 0..3 of infracfg_ao), power status (PWR_STATUS,
 * PWR_STATUS_MSB) and module busy, each with a mask per resource
 * (REG_MODULE_SW_CG_*_REQ_MASK_0..3 0x460..0x4ac, REG_PWR_STATUS[_MSB]_*
 * 0x4b0..0x4dc, REG_MODULE_BUSY_* 0x4e0..0x4f4; spm_reg.h of MT6878).
 * These have no "_mask_b" suffix like SPM_SRC_MASK: whether a 1 masks or
 * honours is not stated in the GPL headers. Experiment, a bit mask:
 * bit 0: all clock gate masks, bit 1: all power status masks, bit 2: all
 * busy masks set to ~0 for the sleep (TF-A may write them itself on its
 * suspend path), the old values back at the wake-up. Default 0.
 */
static unsigned int sleep_req_mask;
module_param(sleep_req_mask, uint, 0644);
MODULE_PARM_DESC(sleep_req_mask,
		 "Experiment: set the SPM clock-gate (1), power-status (2), busy (4) request masks to ~0 for the sleep (default 0)");

/*
 * Experiment: drop requesters from SPM_SRC_MASK for the sleep (their
 * "_mask_b" bits to 0: the SPM ignores them), the old masks back at the
 * wake-up. Bit 0: perisys (SRC_MASK_10 bits 23..30), bit 1: sspm
 * (SRC_MASK_11 bits 13..19), bit 2: mcupm (SRC_MASK_4 bits 16..22) and mcu
 * (SRC_MASK_7 bits 28..30). Default 0. A masked requester that still needs
 * its resource in the sleep loses it: run with t2-s2idle-test SAFETY on.
 */
static unsigned int sleep_src_unmask;
module_param(sleep_src_unmask, uint, 0644);
MODULE_PARM_DESC(sleep_src_unmask,
		 "Experiment: SPM ignores perisys (1), sspm (2), mcupm+mcu (4) for the sleep (default 0)");

/*
 * The downstream kernel starts the DVFSRC (vcore/DRAM DVFS) at boot: mtk-dvfsrc
 * sends MTK_SIP_VCOREFS_CONTROL (0x506) DVFSRC_INIT (0, flags 0/0) at probe,
 * mtk-dvfsrc-start DVFSRC_START at late_initcall_sync ("Vcore DVFS Enable." in
 * the boot log of the Titan 2 factory firmware). We send neither; TF-A then sleeps with
 * SPM flags 0x00090f77 instead of the factory 0x00090077: DISABLE_VCORE_DVS,
 * DISABLE_DDR_DFS, DISABLE_EMI_DFS, DISABLE_BUS_DFS (lpm_spm_comm.h).
 * Experiment: 1 sends both once, at the next suspend (one way: no kernel
 * driver votes bandwidth then, so DRAM may run at its lowest level while
 * awake too). Default 0.
 */
static bool dvfsrc_start;
module_param(dvfsrc_start, bool, 0644);
MODULE_PARM_DESC(dvfsrc_start,
		 "Experiment: start the DVFSRC as the downstream kernel does (DVFSRC_INIT, DVFSRC_START), once, at the next suspend (default 0)");

#define MTK_SIP_VCOREFS_CONTROL	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, \
						   ARM_SMCCC_SMC_64, \
						   ARM_SMCCC_OWNER_SIP, 0x506)
#define MTK_SIP_DVFSRC_INIT	0x0
#define MTK_SIP_DVFSRC_START	0x1

static char *sleep_gate_exclude = "";
module_param(sleep_gate_exclude, charp, 0644);
MODULE_PARM_DESC(sleep_gate_exclude,
		 "Clock name prefixes sleep_gate_unused leaves alone (default empty)");

#define MTK_SIP_LPM_CONTROL	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, \
						   ARM_SMCCC_SMC_64, \
						   ARM_SMCCC_OWNER_SIP, 0x507)

/* a1: 0xDA000000 | user << 16 | id */
#define LPM_SMC_MAGIC		0xDA000000
#define LPM_SMC_USER(user, id)	(LPM_SMC_MAGIC | ((user) << 16) | (id))

#define LPM_USER_CPU_PM		0
#define LPM_USER_SPM_DBG	1
#define LPM_USER_SPM		2
#define LPM_USER_CPU_PM_LP	3

/* a2 */
#define LPM_ACT_SET		BIT(0)
#define LPM_ACT_CLR		BIT(1)
#define LPM_ACT_GET		BIT(2)
#define LPM_ACT_PUSH		BIT(3)
#define LPM_ACT_POP		BIT(4)
#define LPM_ACT_SUBMIT		BIT(5)

/* user 0, cpu_pm: the legacy interface puts its payload into a2 */
#define CPU_PM_IRQ_REMAIN_LIST_ALLOC	2
#define CPU_PM_IRQ_REMAIN_IRQ_ADD	3
#define CPU_PM_IRQ_REMAIN_IRQ_SUBMIT	4
#define CPU_PM_VALIDATE_PWR_STATE_CTRL	5

/* user 3, cpu_pm_lp */
#define CPU_PM_LP_IRQS_REMAIN_ALLOC	1
#define CPU_PM_LP_IRQS_REMAIN_CTRL	2
#define CPU_PM_LP_IRQS_REMAIN_IRQ	3
#define CPU_PM_LP_IRQS_REMAIN_WAKEUP_CAT 4
#define CPU_PM_LP_IRQS_REMAIN_WAKEUP_SRC 5
#define CPU_PM_LP_SUSPEND_SRC		6
#define CPU_PM_LP_RECORD_CTRL		8
#define CPU_PM_LP_SUSPEND_ABORT_REASON	9

/* user 2, spm */
#define SPM_FW_TYPE			3
#define SPM_SET_PENDING_IRQ_INIT	5

/* user 1, spm_dbg */
#define SPM_DBG_SUSPEND_DBG_CTRL	0x03
#define SPM_DBG_RC_SWITCH		0x05
#define SPM_DBG_RC_CNT			0x06
#define SPM_DBG_COND_CHECK		0x07
#define SPM_DBG_COND_BLOCK		0x08
#define SPM_DBG_RES_NUM			0x0b
#define SPM_DBG_RES_USAGE		0x0d
#define SPM_DBG_RES_USER_NUM		0x0e
#define SPM_DBG_RES_USER_VALID		0x0f
#define SPM_DBG_RES_USER_NAME		0x10
#define SPM_DBG_RC_RES_CTRL		0x14
#define SPM_DBG_RC_RES_INFO		0x15
#define SPM_DBG_DOE_RESOURCE_CTRL	0x11
#define SPM_DBG_DOE_RC			0x12
#define SPM_DBG_IDLE_PWR_STAT		0x2A
#define SPM_DBG_SUSPEND_PWR_STAT	0x2B
#define SPM_DBG_LP_REQ_STAT		0x2C

/* RES_USAGE: a3 = this id for the usage of all resources at once */
#define SPM_RES_USAGE_ALL		(~0UL)
/* RES_USER_NAME: four characters, one per byte, low byte first */
#define SPM_RES_USER_NAME_LEN		4

/* IDLE/SUSPEND_PWR_STAT a3 and a4 */
#define SPM_STAT_MCUSYS		0
#define SPM_STAT_F26M		1
#define SPM_STAT_VCORE		2
#define SPM_SLP_COUNT		0
#define SPM_SLP_DURATION	1

/* RC_SWITCH GET result */
#define SPM_RC_VALID_SW		BIT(0)
#define SPM_RC_VALID_FW		BIT(1)

/* SPM registers (offsets in the 0x1c001000 block) */
#define SPM_CLK_SETTLE		0x028
#define MD32PCM_SCU_STA0	0x110
#define SPM_IRQ_STA		0x128
#define MD32PCM_WAKEUP_STA	0x130
#define MD32PCM_EVENT_STA	0x134
#define SYS_TIMER_VALUE_L	0x504
#define SYS_TIMER_VALUE_H	0x508
#define SPM_SW_FLAG_0		0x600
#define SPM_SW_FLAG_1		0x608
#define SPM_SW_RSV(n)		(0x610 + 4 * (n))
#define SPM_BK_WAKE_EVENT	0x634
#define SPM_BK_VTCXO_DUR	0x638
#define SPM_BK_WAKE_MISC	0x63c
#define SPM_BK_PCM_TIMER	0x640
#define SPM_WAKEUP_STA		0x800
#define SPM_WAKEUP_EXT_STA	0x804
#define SPM_SRC_REQ		0x818
#define SPM_SRC_MASK(n)		(0x81c + 4 * (n))
/* the SPM's HW CG check: infracfg_ao MODULE_SW_CG_0..3 that request 26M */
#define SPM_SW_CG_F26M_MASK(n)	(0x490 + 4 * (n))
#define SPM_SW_CG_DDREN_MASK(n)	(0x460 + 4 * (n))
#define SPM_PWR_STA_F26M_MASK	0x4bc
#define SPM_PWR_STA_MSB_F26M_MASK 0x4d4
#define SPM_BUSY_F26M_MASK	0x4ec
#define MT6878_INFRACFG_AO_BASE	0x10001000
#define SPM_HW_REQ_MASK_FIRST	0x460
#define SPM_HW_REQ_MASK_NUM	((0x4f4 - 0x460) / 4 + 1)
#define SPM_PWR_STATUS		0xf40
#define SPM_PWR_STATUS_2ND	0xf44
#define SPM_PWR_STATUS_MSB	0xf48
#define SPM_PWR_STATUS_MSB_2ND	0xf4c
#define SPM_PWR_CON(n)		(0xe00 + 4 * (n))	/* n = PWR_STATUS bit, MSB: 32 + bit */
#define SPM_PWR_CON_ACK		BIT(30)
#define SPM_REQ_STA(n)		(0x850 + 4 * (n))
#define SPM_REQ_STA_NUM		13
#define PCM_WDT_LATCH_SPARE(n)	(0x980 + 4 * (n))

#define SPM_TICKS_PER_SEC	32768

/* SPM_BK_WAKE_MISC */
#define WAKE_MISC_SRCLKEN_RC_ERR_INT	BIT(0)
#define WAKE_MISC_DVFSRC_IRQ		BIT(4)
#define WAKE_MISC_TWAM_IRQ_B		BIT(5)
#define WAKE_MISC_SPM_ACK_CHK_WAKEUP_0	BIT(6)
#define WAKE_MISC_SPM_ACK_CHK_WAKEUP_1	BIT(7)
#define WAKE_MISC_SPM_ACK_CHK_WAKEUP_2	BIT(8)
#define WAKE_MISC_SPM_ACK_CHK_WAKEUP_3	BIT(9)
#define WAKE_MISC_SPM_ACK_CHK_WAKEUP_ALL BIT(10)
#define WAKE_MISC_VLP_BUS_TIMEOUT_IRQ	BIT(11)
#define WAKE_MISC_PCM_TIMER_EVENT	BIT(16)
#define WAKE_MISC_PMIC_EINT_OUT		(BIT(19) | BIT(20))
#define WAKE_MISC_PMIC_IRQ_ACK		BIT(30)
#define WAKE_MISC_PMIC_SCP_IRQ		BIT(31)

/* PCM_WDT_LATCH_SPARE_0: the firmware reached the deepest state */
#define DEBUG_FLAG_DEEPEST		BIT(9)

/* R12 of the MT6878 SPM: what woke the system */
static const char * const mtk_lpm_r12_names[32] = {
	[0] = "PCM_TIMER",
	[1] = "TWAM_PMSR_DVFSRC_IRQ",
	[2] = "KP_IRQ_B",
	[3] = "APWDT_EVENT_B",
	[4] = "APXGPT1_EVENT_B",
	[5] = "CONN2AP_SPM_WAKEUP_B",
	[6] = "EINT_EVENT_B",
	[7] = "CONN_WDT_IRQ_B",
	[8] = "CCIF0_EVENT_B",
	[9] = "CCIF1_EVENT_B",
	[10] = "SC_SSPM2SPM_WAKEUP_B",
	[11] = "SC_SCP2SPM_WAKEUP_B",
	[13] = "PCM_WDT_WAKEUP_B",
	[14] = "USB_CDSC_B",
	[15] = "USB_POWERDWN_B",
	[17] = "UART2SPM_IRQ_B",
	[18] = "SYSTIMER_EVENT_B",
	[19] = "EINT_EVENT_SECURED",
	[20] = "AFE_IRQ_MCU_B",
	[21] = "THERM_CTRL_EVENT_B",
	[22] = "SYS_CIRQ_IRQ_B",
	[23] = "MD2AP_PEER_EVENT_B",
	[24] = "CSYSPWREQ_B",
	[25] = "MD1_WDT_B",
	[26] = "AP2AP_PEER_WAKEUPEVENT_B",
	[27] = "SEJ_EVENT_B",
	[28] = "SPM_CPU_WAKEUPEVENT_B",
	[29] = "APUSYS_WAKE_HOST",
	[31] = "MSDC_WAKEUPEVENT_B",
};

/* Who keeps the SPM from its deepest state: subsystem resource requests */
struct mtk_lpm_subsys_req {
	const char *name;
	u32 reg1, mask1;
	u32 reg2, mask2;
	const char * const *res;	/* the resource of each bit, in order */
};

/*
 * The nine requests of the modem and of CONNSYS, in the order of their
 * SPM_SRC_MASK bits in the downstream lpm logger of the MT6878 (apsrc
 * .. vrf18). That the REQ_STA bits follow the same order is an
 * assumption; "26m" is SRCCLKENA, the 26 MHz clock.
 */
#define MTK_LPM_SUBSYS_RES	9

static const char * const mtk_lpm_md_res[MTK_LPM_SUBSYS_RES] = {
	"apsrc", "ddren", "emi", "infra", "pmic", "26m", "26m1", "vcore",
	"vrf18",
};

static const char * const mtk_lpm_conn_res[MTK_LPM_SUBSYS_RES] = {
	"apsrc", "ddren", "emi", "infra", "pmic", "26m", "26mb", "vcore",
	"vrf18",
};

static const struct mtk_lpm_subsys_req mtk_lpm_subsys_reqs[] = {
	{ "md", SPM_REQ_STA(9), 0xff << 24, SPM_REQ_STA(10), 0x1,
	  mtk_lpm_md_res },
	{ "conn", SPM_REQ_STA(4), 0x1ff << 7, 0, 0, mtk_lpm_conn_res },
	{ "disp", SPM_REQ_STA(4), 0x7f << 23 },
	{ "scp", SPM_REQ_STA(10), BIT(31), SPM_REQ_STA(11), 0x7f },
	{ "ufs", SPM_REQ_STA(11), 0x1f << 27, SPM_REQ_STA(12), 0x3 },
	{ "apu", SPM_REQ_STA(0), 0x7f },
	{ "gce_d", SPM_REQ_STA(6), 0x7f << 19 },
	{ "gce_m", SPM_REQ_STA(6), 0x3f << 26, SPM_REQ_STA(7), 0x1 },
	{ "srclkeni", SPM_REQ_STA(11), 0x7 << 7 },
	{ "spm", SPM_SRC_REQ, 0x18f6 },
};

/*
 * Every request line of SPM_REQ_STA_0..12, with the requester and the
 * resource, as the MT6878 register description in the downstream lpm
 * headers (modules/include/mt6878/spm_reg.h) lists them. SPM_SRC_MASK_0..12
 * have the same layout: a request counts for the SPM only where its
 * "_mask_b" bit is 1. Multi-bit fields (ccif, dpm, mcusys, srcclkeni:
 * one bit per channel or core) are one entry each.
 */
struct mtk_lpm_req_bit {
	u8 reg, bit, width, who, res;
};

static const char * const mtk_lpm_req_who[] = {
	"apu", "audio_dsp", "cam", "ccif", "ccu", "cg_check", "conn", "mcupm",
	"disp0", "disp1", "dpm", "dpmaif", "dvfsrc", "emisys", "gce_d", "gce_m",
	"gpueb", "hwccf", "img", "infrasys", "ipic", "mcu", "mcusys", "md", "mdp",
	"mm_proc", "mmsys", "pcie0", "pcie1", "perisys", "scp", "srcclkeni", "sspm",
	"ssr", "ufs", "vdec", "venc",
};

static const char * const mtk_lpm_req_res[] = {
	"apsrc", "ddren", "emi", "infra", "pmic", "26m", "26m1", "26mb", "vcore", "vrf18", "level",
};

static const struct mtk_lpm_req_bit mtk_lpm_req_bits[] = {
	{ 0, 0, 1, 0, 0 },
	{ 0, 1, 1, 0, 1 },
	{ 0, 2, 1, 0, 2 },
	{ 0, 3, 1, 0, 3 },
	{ 0, 4, 1, 0, 4 },
	{ 0, 5, 1, 0, 5 },
	{ 0, 6, 1, 0, 9 },
	{ 0, 7, 1, 1, 0 },
	{ 0, 8, 1, 1, 1 },
	{ 0, 9, 1, 1, 2 },
	{ 0, 10, 1, 1, 3 },
	{ 0, 11, 1, 1, 4 },
	{ 0, 12, 1, 1, 5 },
	{ 0, 13, 1, 1, 8 },
	{ 0, 14, 1, 1, 9 },
	{ 0, 15, 1, 2, 0 },
	{ 0, 16, 1, 2, 1 },
	{ 0, 17, 1, 2, 2 },
	{ 0, 18, 1, 2, 3 },
	{ 0, 19, 1, 2, 4 },
	{ 0, 20, 1, 2, 5 },
	{ 0, 21, 1, 2, 9 },
	{ 1, 0, 12, 3, 0 },
	{ 1, 12, 12, 3, 2 },
	{ 2, 0, 12, 3, 3 },
	{ 2, 12, 12, 3, 4 },
	{ 3, 0, 12, 3, 5 },
	{ 3, 12, 12, 3, 9 },
	{ 3, 24, 1, 4, 0 },
	{ 3, 25, 1, 4, 1 },
	{ 3, 26, 1, 4, 2 },
	{ 3, 27, 1, 4, 3 },
	{ 3, 28, 1, 4, 4 },
	{ 3, 29, 1, 4, 5 },
	{ 3, 30, 1, 4, 9 },
	{ 3, 31, 1, 5, 0 },
	{ 4, 0, 1, 5, 1 },
	{ 4, 1, 1, 5, 2 },
	{ 4, 2, 1, 5, 3 },
	{ 4, 3, 1, 5, 4 },
	{ 4, 4, 1, 5, 5 },
	{ 4, 5, 1, 5, 8 },
	{ 4, 6, 1, 5, 9 },
	{ 4, 7, 1, 6, 0 },
	{ 4, 8, 1, 6, 1 },
	{ 4, 9, 1, 6, 2 },
	{ 4, 10, 1, 6, 3 },
	{ 4, 11, 1, 6, 4 },
	{ 4, 12, 1, 6, 5 },
	{ 4, 13, 1, 6, 7 },
	{ 4, 14, 1, 6, 8 },
	{ 4, 15, 1, 6, 9 },
	{ 4, 16, 1, 7, 0 },
	{ 4, 17, 1, 7, 1 },
	{ 4, 18, 1, 7, 2 },
	{ 4, 19, 1, 7, 3 },
	{ 4, 20, 1, 7, 4 },
	{ 4, 21, 1, 7, 5 },
	{ 4, 22, 1, 7, 9 },
	{ 4, 23, 1, 8, 0 },
	{ 4, 24, 1, 8, 1 },
	{ 4, 25, 1, 8, 2 },
	{ 4, 26, 1, 8, 3 },
	{ 4, 27, 1, 8, 4 },
	{ 4, 28, 1, 8, 5 },
	{ 4, 29, 1, 8, 9 },
	{ 4, 30, 1, 9, 0 },
	{ 4, 31, 1, 9, 1 },
	{ 5, 0, 1, 9, 2 },
	{ 5, 1, 1, 9, 3 },
	{ 5, 2, 1, 9, 4 },
	{ 5, 3, 1, 9, 5 },
	{ 5, 4, 1, 9, 9 },
	{ 5, 5, 4, 10, 0 },
	{ 5, 9, 4, 10, 1 },
	{ 5, 13, 4, 10, 2 },
	{ 5, 17, 4, 10, 3 },
	{ 5, 21, 4, 10, 4 },
	{ 5, 25, 4, 10, 5 },
	{ 6, 0, 4, 10, 8 },
	{ 6, 4, 4, 10, 9 },
	{ 6, 8, 1, 11, 0 },
	{ 6, 9, 1, 11, 1 },
	{ 6, 10, 1, 11, 2 },
	{ 6, 11, 1, 11, 3 },
	{ 6, 12, 1, 11, 4 },
	{ 6, 13, 1, 11, 5 },
	{ 6, 14, 1, 11, 9 },
	{ 6, 15, 1, 12, 10 },
	{ 6, 16, 1, 13, 0 },
	{ 6, 17, 1, 13, 1 },
	{ 6, 18, 1, 13, 2 },
	{ 6, 19, 1, 14, 0 },
	{ 6, 20, 1, 14, 1 },
	{ 6, 21, 1, 14, 2 },
	{ 6, 22, 1, 14, 3 },
	{ 6, 23, 1, 14, 4 },
	{ 6, 24, 1, 14, 5 },
	{ 6, 25, 1, 14, 9 },
	{ 6, 26, 1, 15, 0 },
	{ 6, 27, 1, 15, 1 },
	{ 6, 28, 1, 15, 2 },
	{ 6, 29, 1, 15, 3 },
	{ 6, 30, 1, 15, 4 },
	{ 6, 31, 1, 15, 5 },
	{ 7, 0, 1, 15, 9 },
	{ 7, 1, 1, 16, 0 },
	{ 7, 2, 1, 16, 1 },
	{ 7, 3, 1, 16, 2 },
	{ 7, 4, 1, 16, 3 },
	{ 7, 5, 1, 16, 4 },
	{ 7, 6, 1, 16, 5 },
	{ 7, 7, 1, 16, 9 },
	{ 7, 8, 1, 17, 0 },
	{ 7, 9, 1, 17, 1 },
	{ 7, 10, 1, 17, 2 },
	{ 7, 11, 1, 17, 3 },
	{ 7, 12, 1, 17, 4 },
	{ 7, 13, 1, 17, 5 },
	{ 7, 14, 1, 17, 8 },
	{ 7, 15, 1, 17, 9 },
	{ 7, 16, 1, 18, 0 },
	{ 7, 17, 1, 18, 1 },
	{ 7, 18, 1, 18, 2 },
	{ 7, 19, 1, 18, 3 },
	{ 7, 20, 1, 18, 4 },
	{ 7, 21, 1, 18, 5 },
	{ 7, 22, 1, 18, 9 },
	{ 7, 23, 1, 19, 0 },
	{ 7, 24, 1, 19, 1 },
	{ 7, 25, 1, 19, 2 },
	{ 7, 26, 1, 20, 3 },
	{ 7, 27, 1, 20, 9 },
	{ 7, 28, 1, 21, 0 },
	{ 7, 29, 1, 21, 1 },
	{ 7, 30, 1, 21, 2 },
	{ 8, 0, 8, 22, 0 },
	{ 8, 8, 8, 22, 1 },
	{ 8, 16, 8, 22, 2 },
	{ 8, 24, 8, 22, 3 },
	{ 9, 0, 8, 22, 4 },
	{ 9, 8, 8, 22, 5 },
	{ 9, 16, 8, 22, 9 },
	{ 9, 24, 1, 23, 0 },
	{ 9, 25, 1, 23, 1 },
	{ 9, 26, 1, 23, 2 },
	{ 9, 27, 1, 23, 3 },
	{ 9, 28, 1, 23, 4 },
	{ 9, 29, 1, 23, 5 },
	{ 9, 30, 1, 23, 6 },
	{ 9, 31, 1, 23, 8 },
	{ 10, 0, 1, 23, 9 },
	{ 10, 1, 1, 24, 0 },
	{ 10, 2, 1, 24, 1 },
	{ 10, 3, 1, 25, 0 },
	{ 10, 4, 1, 25, 1 },
	{ 10, 5, 1, 25, 2 },
	{ 10, 6, 1, 25, 3 },
	{ 10, 7, 1, 25, 4 },
	{ 10, 8, 1, 25, 5 },
	{ 10, 9, 1, 25, 9 },
	{ 10, 10, 1, 26, 0 },
	{ 10, 11, 1, 26, 1 },
	{ 10, 12, 1, 26, 9 },
	{ 10, 13, 1, 27, 0 },
	{ 10, 14, 1, 27, 1 },
	{ 10, 15, 1, 27, 3 },
	{ 10, 16, 1, 27, 5 },
	{ 10, 17, 1, 27, 9 },
	{ 10, 18, 1, 28, 0 },
	{ 10, 19, 1, 28, 1 },
	{ 10, 20, 1, 28, 3 },
	{ 10, 21, 1, 28, 5 },
	{ 10, 22, 1, 28, 9 },
	{ 10, 23, 1, 29, 0 },
	{ 10, 24, 1, 29, 1 },
	{ 10, 25, 1, 29, 2 },
	{ 10, 26, 1, 29, 3 },
	{ 10, 27, 1, 29, 4 },
	{ 10, 28, 1, 29, 5 },
	{ 10, 29, 1, 29, 8 },
	{ 10, 30, 1, 29, 9 },
	{ 10, 31, 1, 30, 0 },
	{ 11, 0, 1, 30, 1 },
	{ 11, 1, 1, 30, 2 },
	{ 11, 2, 1, 30, 3 },
	{ 11, 3, 1, 30, 4 },
	{ 11, 4, 1, 30, 5 },
	{ 11, 5, 1, 30, 8 },
	{ 11, 6, 1, 30, 9 },
	{ 11, 7, 2, 31, 3 },
	{ 11, 9, 2, 31, 4 },
	{ 11, 11, 2, 31, 5 },
	{ 11, 13, 1, 32, 0 },
	{ 11, 14, 1, 32, 1 },
	{ 11, 15, 1, 32, 2 },
	{ 11, 16, 1, 32, 3 },
	{ 11, 17, 1, 32, 4 },
	{ 11, 18, 1, 32, 5 },
	{ 11, 19, 1, 32, 9 },
	{ 11, 20, 1, 33, 0 },
	{ 11, 21, 1, 33, 1 },
	{ 11, 22, 1, 33, 2 },
	{ 11, 23, 1, 33, 3 },
	{ 11, 24, 1, 33, 4 },
	{ 11, 25, 1, 33, 5 },
	{ 11, 26, 1, 33, 9 },
	{ 11, 27, 1, 34, 0 },
	{ 11, 28, 1, 34, 1 },
	{ 11, 29, 1, 34, 2 },
	{ 11, 30, 1, 34, 3 },
	{ 11, 31, 1, 34, 4 },
	{ 12, 0, 1, 34, 5 },
	{ 12, 1, 1, 34, 9 },
	{ 12, 2, 1, 35, 0 },
	{ 12, 3, 1, 35, 1 },
	{ 12, 4, 1, 35, 2 },
	{ 12, 5, 1, 35, 3 },
	{ 12, 6, 1, 35, 4 },
	{ 12, 7, 1, 35, 5 },
	{ 12, 8, 1, 35, 9 },
	{ 12, 9, 1, 36, 0 },
	{ 12, 10, 1, 36, 1 },
	{ 12, 11, 1, 36, 2 },
	{ 12, 12, 1, 36, 3 },
	{ 12, 13, 1, 36, 4 },
	{ 12, 14, 1, 36, 5 },
	{ 12, 15, 1, 36, 9 },
};

/*
 * COND_BLOCK: bits 0..15 clock gate groups, from bit 16 the PLLs in the
 * order of TF-A's common code, as the downstream lpm debug driver (v2,
 * the one the downstream kernel of the Titan 2 loads) decodes them.
 */
#define SPM_COND_BLOCKED_PLL_IDX	16
static const char * const mtk_lpm_cond_plls[] = {
	"UNIVPLL", "UNIVPLL2", "MFGPLL", "MFGSCPLL", "SENSONPLL", "MSDCPLL",
	"UFSPLL", "TVDPLL", "MMPLL", "MMPLL2", "MAINPLL2", "IMGPLL", "USBPLL",
	"ADSPPLL", "APLL1", "APLL2",
};

/*
 * The SPM resources of TF-A's software resource requests (mt_lp_rq: the
 * kernel drivers and TF-A itself ask the SPM to keep these through a
 * sleep), in the order of the downstream lpm logger (MT_SPM_RES_*).
 */
static const char * const mtk_lpm_sw_res[] = {
	"XO_FPM", "CK_26M", "INFRA", "SYSPLL", "DRAM_S0", "DRAM_S1", "VCORE",
	"EMI", "PMIC",
};

/* LP_REQ_STAT: the scenarios that keep the SPM up (downstream order) */
static const char * const mtk_lpm_lp_scenes[] = {
	"AUDIO_AFE", "AUDIO_DSP", "USB_HEADSET",
};

/* MT6878: the SPM resources TF-A knows (DOE_RESOURCE_CTRL ids) */
#define MTK_LPM_NUM_RESOURCES	8

/* MT6878: the constraints to enable at boot (DOE_RC ids, the TF-A order) */
static const struct {
	u8 id;
	const char *name;
} mtk_lpm_constraints[] = {
	{ 0, "vcore" },
	{ 1, "bus26m" },
	{ 2, "syspll" },
};

#define MTK_LPM_NUM_RC		3
static_assert(ARRAY_SIZE(mtk_lpm_constraints) == MTK_LPM_NUM_RC);

/* the state at the last suspend entry, after all devices suspended */
struct mtk_lpm_entry {
	u32 req_sta[SPM_REQ_STA_NUM], src_req;
	u32 src_mask[SPM_REQ_STA_NUM];
	unsigned long rc_sw[MTK_LPM_NUM_RC], rc_block[MTK_LPM_NUM_RC];
	unsigned long rc_check[MTK_LPM_NUM_RC];	/* COND_CHECK: blocked */
	unsigned long rc_cnt[MTK_LPM_NUM_RC];	/* RC_CNT before the sleep */
	unsigned long stat[2][3][2];	/* suspend/idle x mcusys/26m/vcore x count/duration */
	char sw_req[160];		/* TF-A's software resource requests */
	u8 rc_forced;		/* constraints enabled for this suspend */
	u64 seq;
};

struct mtk_lpm_wake {
	u32 r12, r12_ext, raw_sta, raw_ext_sta, wake_misc, timer_out, r13;
	u32 vtcxo_dur, vcore_dur;
	u32 debug_flag, debug_flag1;
	u32 req_sta[SPM_REQ_STA_NUM], src_req;
	u32 src_mask[SPM_REQ_STA_NUM];
	u32 sw_flag0, sw_flag1, isr;
	unsigned long abort_reason;
	unsigned long rc_entered[MTK_LPM_NUM_RC];	/* RC_CNT delta */
	unsigned long rc_res[MTK_LPM_NUM_RC];	/* RC residency, 32 kHz ticks */
	u64 seq;
};

struct mtk_lpm {
	struct device *dev;
	struct regmap *spm;
	int irq;
	unsigned int hwirq;

	/* original cpuidle entries, index by state */
	int (*orig_enter_s2idle[CPUIDLE_STATE_MAX])(struct cpuidle_device *,
						    struct cpuidle_driver *, int);
	struct cpuidle_driver *idle_drv;

	spinlock_t lock;		/* wake, entry and the counters */
	struct mtk_lpm_wake wake;
	struct mtk_lpm_entry entry;
	u8 rc_forced;			/* bit n: constraint n enabled by us */
	u8 rc_forced_switch;		/* bit n: ... through RC_SWITCH */
	bool validated;			/* VALIDATE_PWR_STATE sent */
	u64 suspends;
	bool in_suspend;
	void __iomem *rc;		/* SRCLKEN_RC, read only */
	void __iomem *dcm[21];		/* mtk_lpm_dcm_regs, read only */
	void __iomem *infracfg;		/* MODULE_SW_CG status, read only */
	u32 req_mask_saved[SPM_HW_REQ_MASK_NUM];
	bool req_mask_set;
	u32 src_mask_saved[SPM_REQ_STA_NUM];
	bool src_mask_set;
	bool dvfsrc_started;
};

/*
 * The power domains of the MT6878 SPM by PWR_CON offset (downstream clkchk and
 * scpsys tables): PWR_STATUS bit n is assumed to be the domain at 0xe00 +
 * 4n (bit n of PWR_STATUS_MSB: 0xe80 + 4n), checked against each named
 * domain's own PWR_ACK at every log.
 */
static const char * const mtk_lpm_pwr_names[64] = {
	[0] = "md1", [1] = "conn", [4] = "ufs0", [5] = "ufs0-phy",
	[6] = "audio", [10] = "isp-main", [11] = "isp-dip1",
	[13] = "isp-vcore", [14] = "vde0", [16] = "ven0", [18] = "cam-main",
	[20] = "cam-suba", [21] = "cam-subb", [23] = "cam-vcore",
	[24] = "cam-ccu", [25] = "cam-ccu-ao", [28] = "disp",
	[30] = "mm-infra", [31] = "mm-proc", [39] = "csi-rx", [40] = "ssrsys",
	[42] = "ssusb", [45] = "mfg0",
};

/* infracfg_ao MODULE_SW_CG_0..3 status (1: gated), as the SPM watches them */
static const u16 mtk_lpm_infra_cg_sta[] = { 0x90, 0x94, 0xac, 0xc8 };

static struct mtk_lpm *mtk_lpm_instance;

/*
 * SRCLKEN_RC (0x1c00d000 on MT6878, "mediatek,srclken-rc" of the factory device tree,
 * not in ours): the RC collects the 26 MHz requests of the SPM ("suspend",
 * sub-id 0) and of the subsystems and drives the PMIC DCXO. Register
 * layout of the downstream clkbuf driver (clkbuf/src/src-data-v1.c); the
 * sub-ids are those of the factory device tree. Only read, to show who keeps the DCXO
 * in full power mode (FPM) around a sleep.
 */
#define MT6878_SRCLKEN_RC_BASE		0x1c00d000
#define MT6878_SRCLKEN_RC_SIZE		0x800
#define RC_M_SRCLKEN_CFG(n)		(0x020 + 4 * (n))
#define  RC_CFG_SW_BBLPM_EN		BIT(5)
#define  RC_CFG_SW_FPM_EN		BIT(4)
#define  RC_CFG_SW_SRCLKEN_RC_EN	BIT(3)
#define  RC_CFG_SUBSYS_RC_EN		BIT(0)
#define RC_FSM_STA_0			0x100
#define  RC_FSM_ANY_DCXO_REQ		BIT(3)
#define RC_CMD_STA_0			0x104
#define RC_CMD_STA_1			0x108	/* [12:0] latched requests */
#define RC_M_REQ_STA(n)			(0x114 + 4 * (n))
#define  RC_REQ_FPM			BIT(0)
#define  RC_REQ_BBLPM			BIT(2)
#define RC_TRACE(n)			(0x100 + 0x600 + 8 * (n))	/* LSB, MSB at + 4 */
#define RC_TIMER(n)			(0x100 + 0x640 + 8 * (n))

/*
 * DCM (dynamic clock management) of the infra, peri, vlp and mcusys
 * buses: an idle bus only lets go of its SPM requests with its DCM on. On
 * the Titan 2 factory firmware the boot loader sets these and the downstream kernel
 * changes nothing ("[dcm_restore]type:0x00000000, INIT_DCM_TYPE_BY_K=0x0");
 * the values are those the downstream mt6878_dcm module logged at boot
 * on the factory firmware, the addresses those of
 * its register header (dcm/include/mt6878_dcm_autogen.h). Only read.
 */
struct mtk_lpm_dcm_reg {
	const char *name;
	u32 pa, boot;
};

static const struct mtk_lpm_dcm_reg mtk_lpm_dcm_regs[] = {
	{ "BUS_PLLDIV_CFG", 0x0c53a2e0, 0x800002ff },
	{ "CPU_PLLDIV_CFG0", 0x0c53a2a0, 0x030012ff },
	{ "CPU_PLLDIV_CFG1", 0x0c53a2a4, 0x030012ff },
	{ "EMI_WFIFO", 0x0c53a900, 0x0000000f },
	{ "INFRA_AXIMEM_IDLE_BIT_EN_0", 0x10001a30, 0x000c0000 },
	{ "INFRA_BUS_DCM_CTRL", 0x10001070, 0x40f80603 },
	{ "MCSI_DCM0", 0x0c53a440, 0x0000ffff },
	{ "MCUSYS_DCM_CFG0", 0x0c53a5c0, 0x00078100 },
	{ "MP0_DCM_CFG0", 0x0c53c880, 0x0001000f },
	{ "MP0_DCM_CFG7", 0x0c53c89c, 0x00001011 },
	{ "MP_ADB_DCM_CFG0", 0x0c53a500, 0x00020000 },
	{ "MP_ADB_DCM_CFG4", 0x0c53a510, 0x00278000 },
	{ "MP_MISC_DCM_CFG0", 0x0c53a518, 0x0000007a },
	{ "P2P_RX_CLK_ON", 0x100010a0, 0x00000000 },
	{ "PERI_BUS_DCM_CTRL", 0x10001074, 0xa47f83e3 },
	{ "INFRA_PAR_BUS_CTRL_0", 0x10022030, 0x000202cd },
	{ "INFRA_PAR_BUS_CTRL_3", 0x1002203c, 0x01004028 },
	{ "PERI_PAR_BUS_CTRL_0", 0x11035018, 0x02201493 },
	{ "PERI_PAR_BUS_CTRL_1", 0x1103501c, 0x4403f010 },
	{ "PERI_PAR_BUS_CTRL_2", 0x11035020, 0x04403010 },
	{ "VLP_PAR_BUS_spm_CTRL_0", 0x1c0170b4, 0x80802027 },
};

static const char * const mtk_lpm_rc_subsys[] = {
	"suspend", "md1", "md2", "md3", "mdrf", "mmw", "gps", "bt", "wf",
	"mcu", "co-ant", "nfc", "suspend-rsv", "ufs",
};

static void mtk_lpm_rc_names(struct mtk_lpm *lpm, bool cfg, u32 bit,
			     struct seq_buf *s)
{
	unsigned int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(mtk_lpm_rc_subsys); i++) {
		u32 v = readl(lpm->rc + (cfg ? RC_M_SRCLKEN_CFG(i) :
					 RC_M_REQ_STA(i)));

		if (v & bit)
			seq_buf_printf(s, "%s%s", n++ ? "," : "",
				       mtk_lpm_rc_subsys[i]);
	}
	if (!n)
		seq_buf_printf(s, "none");
}

static u32 mtk_lpm_rd(struct mtk_lpm *lpm, u32 reg);

/* the open infra clock gates the SPM's cg_check counts for 26M and ddren */
static void mtk_lpm_log_cg_check(struct mtk_lpm *lpm, const char *when)
{
	char buf[384];
	struct seq_buf s;
	unsigned int i;

	if (!lpm->infracfg)
		return;
	seq_buf_init(&s, buf, sizeof(buf));
	for (i = 0; i < ARRAY_SIZE(mtk_lpm_infra_cg_sta); i++) {
		u32 open = ~readl(lpm->infracfg + mtk_lpm_infra_cg_sta[i]);
		u32 m26 = mtk_lpm_rd(lpm, SPM_SW_CG_F26M_MASK(i));
		u32 mdd = mtk_lpm_rd(lpm, SPM_SW_CG_DDREN_MASK(i));

		/* a mask bit 1 ignores the gate (run 63: ~0 took cg_check away) */
		seq_buf_printf(&s, "%scg%u open %08x 26m-mask %08x counted %08x",
			       i ? "; " : "", i, open, m26, open & ~m26);
		seq_buf_printf(&s, ", ddren %08x counted %08x", mdd, open & ~mdd);
	}
	dev_info(lpm->dev, "%s, cg_check: %s; pwr-status 26m-mask %08x %08x, busy 26m-mask %08x\n",
		 when, seq_buf_str(&s), mtk_lpm_rd(lpm, SPM_PWR_STA_F26M_MASK),
		 mtk_lpm_rd(lpm, SPM_PWR_STA_MSB_F26M_MASK),
		 mtk_lpm_rd(lpm, SPM_BUSY_F26M_MASK));
}

static void mtk_lpm_pwr_list(u64 bits, struct seq_buf *s)
{
	unsigned int i, n = 0;

	for (i = 0; i < 64; i++) {
		if (!(bits & BIT_ULL(i)))
			continue;
		if (mtk_lpm_pwr_names[i])
			seq_buf_printf(s, "%s%s", n++ ? "," : "", mtk_lpm_pwr_names[i]);
		else
			seq_buf_printf(s, "%s%s%u", n++ ? "," : "",
				       i < 32 ? "lsb" : "msb", i % 32);
	}
	if (!n)
		seq_buf_printf(s, "none");
}

/* domains powered now, and those under the power status 26M mask */
static void mtk_lpm_log_pwr(struct mtk_lpm *lpm, const char *when)
{
	u64 on, on2, mask, ack = 0, named = 0;
	char buf[640];
	struct seq_buf s;
	unsigned int i;

	on = mtk_lpm_rd(lpm, SPM_PWR_STATUS) |
	     (u64)mtk_lpm_rd(lpm, SPM_PWR_STATUS_MSB) << 32;
	on2 = mtk_lpm_rd(lpm, SPM_PWR_STATUS_2ND) |
	      (u64)mtk_lpm_rd(lpm, SPM_PWR_STATUS_MSB_2ND) << 32;
	mask = mtk_lpm_rd(lpm, SPM_PWR_STA_F26M_MASK) |
	       (u64)mtk_lpm_rd(lpm, SPM_PWR_STA_MSB_F26M_MASK) << 32;
	for (i = 0; i < 64; i++) {
		if (!mtk_lpm_pwr_names[i])
			continue;
		named |= BIT_ULL(i);
		if (mtk_lpm_rd(lpm, SPM_PWR_CON(i)) & SPM_PWR_CON_ACK)
			ack |= BIT_ULL(i);
	}

	seq_buf_init(&s, buf, sizeof(buf));
	seq_buf_printf(&s, "status %016llx (2nd %016llx), on: ", on, on2);
	mtk_lpm_pwr_list(on, &s);
	seq_buf_printf(&s, "; on, ignored (26m-mask 1): ");
	mtk_lpm_pwr_list(on & mask, &s);
	seq_buf_printf(&s, "; on, counted for 26M (26m-mask 0): ");
	mtk_lpm_pwr_list(on & ~mask, &s);
	if ((on ^ ack) & named) {
		seq_buf_printf(&s, "; PWR_ACK differs for: ");
		mtk_lpm_pwr_list((on ^ ack) & named, &s);
	}
	dev_info(lpm->dev, "%s, power domains: %s\n", when, seq_buf_str(&s));
}

/* see sleep_req_mask */
static void mtk_lpm_req_mask(struct mtk_lpm *lpm, bool sleep)
{
	unsigned int bits = READ_ONCE(sleep_req_mask), i, kept = 0;

	if (!sleep) {
		if (!lpm->req_mask_set)
			return;
		for (i = 0; i < SPM_HW_REQ_MASK_NUM; i++)
			regmap_write(lpm->spm, SPM_HW_REQ_MASK_FIRST + 4 * i,
				     lpm->req_mask_saved[i]);
		lpm->req_mask_set = false;
		return;
	}
	if (!bits)
		return;
	for (i = 0; i < SPM_HW_REQ_MASK_NUM; i++) {
		u32 off = SPM_HW_REQ_MASK_FIRST + 4 * i;
		bool cg = off < 0x4b0, pwr = off >= 0x4b0 && off < 0x4e0;

		lpm->req_mask_saved[i] = mtk_lpm_rd(lpm, off);
		if ((cg && (bits & BIT(0))) || (pwr && (bits & BIT(1))) ||
		    (!cg && !pwr && (bits & BIT(2)))) {
			regmap_write(lpm->spm, off, ~0U);
			if (mtk_lpm_rd(lpm, off) == ~0U)
				kept++;
		}
	}
	lpm->req_mask_set = true;
	dev_info(lpm->dev, "suspend entry: SPM request masks 0x%x set to ~0 for the sleep, %u registers took it\n",
		 bits, kept);
}

/* see sleep_src_unmask */
static void mtk_lpm_src_unmask(struct mtk_lpm *lpm, bool sleep)
{
	static const struct { u8 bit, reg; u32 mask; } drop[] = {
		{ 0, 10, GENMASK(30, 23) },	/* perisys */
		{ 1, 11, GENMASK(19, 13) },	/* sspm */
		{ 2, 4, GENMASK(22, 16) },	/* mcupm */
		{ 2, 7, GENMASK(30, 28) },	/* mcu */
	};
	unsigned int bits = READ_ONCE(sleep_src_unmask), i;
	u32 now[SPM_REQ_STA_NUM];

	if (!sleep) {
		if (!lpm->src_mask_set)
			return;
		for (i = 0; i < SPM_REQ_STA_NUM; i++)
			regmap_write(lpm->spm, SPM_SRC_MASK(i), lpm->src_mask_saved[i]);
		lpm->src_mask_set = false;
		return;
	}
	if (!bits)
		return;
	for (i = 0; i < SPM_REQ_STA_NUM; i++) {
		lpm->src_mask_saved[i] = mtk_lpm_rd(lpm, SPM_SRC_MASK(i));
		now[i] = lpm->src_mask_saved[i];
	}
	for (i = 0; i < ARRAY_SIZE(drop); i++)
		if (bits & BIT(drop[i].bit))
			now[drop[i].reg] &= ~drop[i].mask;
	for (i = 0; i < SPM_REQ_STA_NUM; i++)
		if (now[i] != lpm->src_mask_saved[i])
			regmap_write(lpm->spm, SPM_SRC_MASK(i), now[i]);
	lpm->src_mask_set = true;
	dev_info(lpm->dev, "suspend entry: SPM ignores 0x%x for the sleep: SRC_MASK_4/7/10/11 %08x %08x %08x %08x -> %08x %08x %08x %08x\n",
		 bits, lpm->src_mask_saved[4], lpm->src_mask_saved[7],
		 lpm->src_mask_saved[10], lpm->src_mask_saved[11],
		 mtk_lpm_rd(lpm, SPM_SRC_MASK(4)), mtk_lpm_rd(lpm, SPM_SRC_MASK(7)),
		 mtk_lpm_rd(lpm, SPM_SRC_MASK(10)), mtk_lpm_rd(lpm, SPM_SRC_MASK(11)));
}

/* the DCM registers that differ from the factory boot, see mtk_lpm_dcm_regs */
static void mtk_lpm_log_dcm(struct mtk_lpm *lpm, const char *when)
{
	char buf[640];
	struct seq_buf s;
	unsigned int i, n = 0;

	seq_buf_init(&s, buf, sizeof(buf));
	for (i = 0; i < ARRAY_SIZE(mtk_lpm_dcm_regs); i++) {
		u32 v;

		if (!lpm->dcm[i])
			continue;
		v = readl(lpm->dcm[i]);
		if (v == mtk_lpm_dcm_regs[i].boot)
			continue;
		seq_buf_printf(&s, "%s%s %08x (boot %08x)", n++ ? ", " : "",
			       mtk_lpm_dcm_regs[i].name, v,
			       mtk_lpm_dcm_regs[i].boot);
	}
	if (!n)
		seq_buf_printf(&s, "all as at boot");
	dev_info(lpm->dev, "%s, DCM: %s\n", when, seq_buf_str(&s));
}

static void mtk_lpm_log_rc(struct mtk_lpm *lpm, const char *when)
{
	char buf[256];
	struct seq_buf s;
	u32 fsm;

	if (!lpm->rc)
		return;
	seq_buf_init(&s, buf, sizeof(buf));
	fsm = readl(lpm->rc + RC_FSM_STA_0);
	seq_buf_printf(&s, "fsm 0x%08x (DCXO request %s), cmd 0x%08x, latched 0x%04x",
		       fsm, fsm & RC_FSM_ANY_DCXO_REQ ? "yes" : "no",
		       readl(lpm->rc + RC_CMD_STA_0),
		       readl(lpm->rc + RC_CMD_STA_1) & 0x1fff);
	seq_buf_printf(&s, "; FPM requests: ");
	mtk_lpm_rc_names(lpm, false, RC_REQ_FPM, &s);
	seq_buf_printf(&s, "; software FPM: ");
	mtk_lpm_rc_names(lpm, true, RC_CFG_SW_FPM_EN, &s);
	seq_buf_printf(&s, "; software mode: ");
	mtk_lpm_rc_names(lpm, true, RC_CFG_SW_SRCLKEN_RC_EN, &s);
	dev_info(lpm->dev, "%s, SRCLKEN_RC: %s\n", when, seq_buf_str(&s));

	/*
	 * The RC's own history: eight trace words (request state) and eight
	 * timers, STA + 0x600/0x640 (broadcast variant of src-data-v1.c; the
	 * factory device tree maps STA 0x1c00d100 + 0x700). What changed between entry
	 * and wake-up is what the RC saw during the sleep.
	 */
	seq_buf_init(&s, buf, sizeof(buf));
	for (fsm = 0; fsm < 8; fsm++)
		seq_buf_printf(&s, "%s%08x%08x", fsm ? " " : "",
			       readl(lpm->rc + RC_TRACE(fsm) + 4),
			       readl(lpm->rc + RC_TRACE(fsm)));
	dev_info(lpm->dev, "%s, SRCLKEN_RC trace: %s\n", when, seq_buf_str(&s));
	seq_buf_init(&s, buf, sizeof(buf));
	for (fsm = 0; fsm < 8; fsm++)
		seq_buf_printf(&s, "%s%08x%08x", fsm ? " " : "",
			       readl(lpm->rc + RC_TIMER(fsm) + 4),
			       readl(lpm->rc + RC_TIMER(fsm)));
	dev_info(lpm->dev, "%s, SRCLKEN_RC timers: %s\n", when, seq_buf_str(&s));
}

static unsigned long mtk_lpm_smc(u32 user, u32 id, u32 act, u32 v1, u32 v2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_LPM_CONTROL, LPM_SMC_USER(user, id), act, v1, v2,
		      0, 0, 0, &res);
	return res.a0;
}

static unsigned long mtk_lpm_smc_ul(u32 user, u32 id, u32 act,
				    unsigned long v1, unsigned long v2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_LPM_CONTROL, LPM_SMC_USER(user, id), act, v1, v2,
		      0, 0, 0, &res);
	return res.a0;
}

#define lpm_smc_cpu_pm(id, act, v1, v2) \
	mtk_lpm_smc(LPM_USER_CPU_PM, id, act, v1, v2)
#define lpm_smc_cpu_pm_lp(id, act, v1, v2) \
	mtk_lpm_smc(LPM_USER_CPU_PM_LP, id, act, v1, v2)
#define lpm_smc_spm(id, act, v1, v2) \
	mtk_lpm_smc(LPM_USER_SPM, id, act, v1, v2)
#define lpm_smc_spm_dbg(id, act, v1, v2) \
	mtk_lpm_smc(LPM_USER_SPM_DBG, id, act, v1, v2)

static u32 mtk_lpm_rd(struct mtk_lpm *lpm, u32 reg)
{
	u32 val = 0;

	regmap_read(lpm->spm, reg, &val);
	return val;
}

/* ---- s2idle: the CPU suspend source around every entry ---------------- */

static int __cpuidle mtk_lpm_enter_s2idle(struct cpuidle_device *dev,
					  struct cpuidle_driver *drv, int index)
{
	struct mtk_lpm *lpm = mtk_lpm_instance;
	int cpu = dev->cpu;
	int ret;

	lpm_smc_cpu_pm_lp(CPU_PM_LP_SUSPEND_SRC, LPM_ACT_SET, cpu, 0);
	ret = lpm->orig_enter_s2idle[index](dev, drv, index);
	lpm_smc_cpu_pm_lp(CPU_PM_LP_SUSPEND_SRC, LPM_ACT_CLR, cpu, 0);

	return ret;
}

static int mtk_lpm_hook_cpuidle(struct mtk_lpm *lpm)
{
	struct cpuidle_driver *drv;
	int i, n = 0;

	cpuidle_pause_and_lock();
	drv = cpuidle_get_driver();
	if (!drv) {
		cpuidle_resume_and_unlock();
		return -EPROBE_DEFER;
	}
	for (i = 1; i < drv->state_count; i++) {
		if (!drv->states[i].enter_s2idle)
			continue;
		lpm->orig_enter_s2idle[i] = drv->states[i].enter_s2idle;
		drv->states[i].enter_s2idle = mtk_lpm_enter_s2idle;
		n++;
	}
	lpm->idle_drv = drv;
	cpuidle_resume_and_unlock();

	if (!n) {
		dev_warn(lpm->dev, "no cpuidle state with an s2idle entry\n");
		return -ENODEV;
	}
	dev_dbg(lpm->dev, "suspend source hooked into %d idle states\n", n);
	return 0;
}

static void mtk_lpm_unhook_cpuidle(struct mtk_lpm *lpm)
{
	struct cpuidle_driver *drv = lpm->idle_drv;
	int i;

	if (!drv)
		return;
	cpuidle_pause_and_lock();
	for (i = 1; i < drv->state_count; i++)
		if (lpm->orig_enter_s2idle[i])
			drv->states[i].enter_s2idle = lpm->orig_enter_s2idle[i];
	cpuidle_resume_and_unlock();
}

/* ---- the wake-up status ------------------------------------------------- */

static void mtk_lpm_read_wake(struct mtk_lpm *lpm, struct mtk_lpm_wake *w)
{
	int i;

	w->r12 = mtk_lpm_rd(lpm, SPM_BK_WAKE_EVENT);
	w->raw_sta = mtk_lpm_rd(lpm, SPM_WAKEUP_STA);
	w->r12_ext = w->raw_sta;
	w->raw_ext_sta = mtk_lpm_rd(lpm, SPM_WAKEUP_EXT_STA);
	w->wake_misc = mtk_lpm_rd(lpm, SPM_BK_WAKE_MISC);
	w->timer_out = mtk_lpm_rd(lpm, SPM_BK_PCM_TIMER);
	w->r13 = mtk_lpm_rd(lpm, MD32PCM_SCU_STA0);
	w->vtcxo_dur = mtk_lpm_rd(lpm, SPM_BK_VTCXO_DUR);
	w->vcore_dur = mtk_lpm_rd(lpm, SPM_SW_RSV(4));
	w->debug_flag = mtk_lpm_rd(lpm, PCM_WDT_LATCH_SPARE(0));
	w->debug_flag1 = mtk_lpm_rd(lpm, PCM_WDT_LATCH_SPARE(1));
	for (i = 0; i < SPM_REQ_STA_NUM; i++) {
		w->req_sta[i] = mtk_lpm_rd(lpm, SPM_REQ_STA(i));
		w->src_mask[i] = mtk_lpm_rd(lpm, SPM_SRC_MASK(i));
	}
	w->src_req = mtk_lpm_rd(lpm, SPM_SRC_REQ);
	w->sw_flag0 = mtk_lpm_rd(lpm, SPM_SW_FLAG_0);
	w->sw_flag1 = mtk_lpm_rd(lpm, SPM_SW_FLAG_1);
	w->isr = mtk_lpm_rd(lpm, SPM_IRQ_STA);
}

static u32 mtk_lpm_req_reg(const u32 *req_sta, u32 src_req, u32 reg)
{
	if (reg == SPM_SRC_REQ)
		return src_req;
	if (reg >= SPM_REQ_STA(0) && reg <= SPM_REQ_STA(SPM_REQ_STA_NUM - 1))
		return req_sta[(reg - SPM_REQ_STA(0)) / 4];
	return 0;
}

static void mtk_lpm_wake_sources(const struct mtk_lpm_wake *w, struct seq_buf *s)
{
	int i, n = 0;

	if ((w->r12 & BIT(0)) && (w->wake_misc & WAKE_MISC_PCM_TIMER_EVENT))
		seq_buf_printf(s, "%sPCM_TIMER", n++ ? " " : "");
	if (w->r12 & BIT(1)) {
		static const struct { u32 bit; const char *name; } misc[] = {
			{ WAKE_MISC_DVFSRC_IRQ, "DVFSRC" },
			{ WAKE_MISC_TWAM_IRQ_B, "TWAM" },
			{ WAKE_MISC_SPM_ACK_CHK_WAKEUP_0, "SPM_ACK_CHK_0" },
			{ WAKE_MISC_SPM_ACK_CHK_WAKEUP_1, "SPM_ACK_CHK_1" },
			{ WAKE_MISC_SPM_ACK_CHK_WAKEUP_2, "SPM_ACK_CHK_2" },
			{ WAKE_MISC_SPM_ACK_CHK_WAKEUP_3, "SPM_ACK_CHK_3" },
			{ WAKE_MISC_SPM_ACK_CHK_WAKEUP_ALL, "SPM_ACK_CHK_ALL" },
			{ WAKE_MISC_SRCLKEN_RC_ERR_INT, "SRCLKEN_RC_ERR" },
			{ WAKE_MISC_VLP_BUS_TIMEOUT_IRQ, "VLP_BUS_TIMEOUT" },
			{ WAKE_MISC_PMIC_EINT_OUT, "PMIC_EINT_OUT" },
			{ WAKE_MISC_PMIC_IRQ_ACK, "PMIC_IRQ_ACK" },
			{ WAKE_MISC_PMIC_SCP_IRQ, "PMIC_SCP_IRQ" },
		};

		for (i = 0; i < ARRAY_SIZE(misc); i++)
			if (w->wake_misc & misc[i].bit)
				seq_buf_printf(s, "%s%s", n++ ? " " : "",
					       misc[i].name);
	}
	for (i = 1; i < 32; i++)
		if ((w->r12 & BIT(i)) && mtk_lpm_r12_names[i])
			seq_buf_printf(s, "%s%s", n++ ? " " : "",
				       mtk_lpm_r12_names[i]);
	if (!n)
		seq_buf_printf(s, "none(r12=0x%x)", w->r12);
}

/*
 * All request lines of @req_sta by requester: those the SPM honours
 * (SPM_SRC_MASK bit set) or, with @masked, those it ignores.
 */
static void mtk_lpm_req_decode(const u32 *req_sta, const u32 *src_mask,
			       bool masked, struct seq_buf *s)
{
	unsigned int i, j, n = 0, res = 0;
	int who = -1;

	for (i = 0; i <= ARRAY_SIZE(mtk_lpm_req_bits); i++) {
		const struct mtk_lpm_req_bit *b = NULL;
		u32 field, mask;

		if (i < ARRAY_SIZE(mtk_lpm_req_bits))
			b = &mtk_lpm_req_bits[i];
		if (!b || b->who != who) {
			if (res) {
				int m = 0;

				seq_buf_printf(s, "%s%s(", n++ ? " " : "",
					       mtk_lpm_req_who[who]);
				for (j = 0; j < ARRAY_SIZE(mtk_lpm_req_res); j++)
					if (res & BIT(j))
						seq_buf_printf(s, "%s%s", m++ ? "," : "",
							       mtk_lpm_req_res[j]);
				seq_buf_printf(s, ")");
			}
			if (!b)
				break;
			who = b->who;
			res = 0;
		}
		mask = GENMASK(b->width - 1, 0);
		field = (req_sta[b->reg] >> b->bit) & mask;
		mask &= src_mask[b->reg] >> b->bit;
		if (masked ? field & ~mask : field & mask)
			res |= BIT(b->res);
	}
	if (!n)
		seq_buf_printf(s, "none");
}

static void mtk_lpm_blockers(const u32 *req_sta, u32 src_req,
			     struct seq_buf *s)
{
	int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(mtk_lpm_subsys_reqs); i++) {
		const struct mtk_lpm_subsys_req *r = &mtk_lpm_subsys_reqs[i];
		u32 v1 = mtk_lpm_req_reg(req_sta, src_req, r->reg1) & r->mask1;
		u32 v2 = r->reg2 ?
			 mtk_lpm_req_reg(req_sta, src_req, r->reg2) & r->mask2 : 0;
		u32 bits;
		int j, m = 0;

		if (!v1 && !v2)
			continue;
		seq_buf_printf(s, "%s%s", n++ ? " " : "", r->name);
		if (!r->res)
			continue;
		/* the request bits as one field: reg1's, then reg2's */
		bits = (v1 >> __ffs(r->mask1)) | (v2 << hweight32(r->mask1));
		seq_buf_printf(s, "(");
		for (j = 0; j < MTK_LPM_SUBSYS_RES; j++)
			if (bits & BIT(j))
				seq_buf_printf(s, "%s%s", m++ ? "," : "",
					       r->res[j]);
		seq_buf_printf(s, ")");
	}
	if (!n)
		seq_buf_printf(s, "none");
}

/*
 * Read after the wake-up these are the requests of the woken system: the
 * subsystem that woke it (md for CCIF) and the SPM's own requests for the
 * running AP (idle logs of the Titan 2 factory firmware show spm_src_req 0x18f618f6
 * after a system_vcore sleep) are always there. What held the SPM during
 * the sleep shows in the entry snapshot and in the 26M/vcore-off times.
 */
static void mtk_lpm_wake_blockers(const struct mtk_lpm_wake *w, struct seq_buf *s)
{
	mtk_lpm_blockers(w->req_sta, w->src_req, s);
}

static void mtk_lpm_block_names(unsigned long block, struct seq_buf *s)
{
	int i, n = 0;

	for (i = 0; i < SPM_COND_BLOCKED_PLL_IDX; i++)
		if (block & BIT_ULL(i))
			seq_buf_printf(s, "%scg%d", n++ ? "," : "", i);
	for (i = 0; i < ARRAY_SIZE(mtk_lpm_cond_plls); i++)
		if (block & BIT_ULL(SPM_COND_BLOCKED_PLL_IDX + i))
			seq_buf_printf(s, "%s%s", n++ ? "," : "",
				       mtk_lpm_cond_plls[i]);
	if (!n)
		seq_buf_printf(s, "-");
}

/* one log line: every honoured request line, then the masked ones */
static void mtk_lpm_log_req_lines(struct mtk_lpm *lpm, const char *when,
				  const u32 *req_sta, const u32 *src_mask)
{
	char *buf = kmalloc(1024, GFP_ATOMIC);
	struct seq_buf s;

	if (!buf)
		return;
	seq_buf_init(&s, buf, 1024);
	mtk_lpm_req_decode(req_sta, src_mask, false, &s);
	seq_buf_printf(&s, "; masked: ");
	mtk_lpm_req_decode(req_sta, src_mask, true, &s);
	dev_info(lpm->dev, "%s, all request lines the SPM honours: %s\n",
		 when, seq_buf_str(&s));
	kfree(buf);
}

/* TF-A's own counters (SUSPEND/IDLE_PWR_STAT): [suspend, idle][mcusys, 26m, vcore][count, ticks] */
static void mtk_lpm_read_stats(unsigned long st[2][3][2])
{
	static const u32 ids[2] = { SPM_DBG_SUSPEND_PWR_STAT, SPM_DBG_IDLE_PWR_STAT };
	int i, j;

	for (i = 0; i < 2; i++)
		for (j = 0; j < 3; j++) {
			st[i][j][0] = lpm_smc_spm_dbg(ids[i], LPM_ACT_GET, j, SPM_SLP_COUNT);
			st[i][j][1] = lpm_smc_spm_dbg(ids[i], LPM_ACT_GET, j, SPM_SLP_DURATION);
		}
}

static void mtk_lpm_log_stats(struct mtk_lpm *lpm, unsigned long before[2][3][2])
{
	static const char * const kind[2] = { "suspend", "idle" };
	static const char * const what[3] = { "mcusys", "26m", "vcore" };
	unsigned long now[2][3][2];
	char buf[320];
	struct seq_buf s;
	int i, j;

	mtk_lpm_read_stats(now);
	seq_buf_init(&s, buf, sizeof(buf));
	for (i = 0; i < 2; i++)
		for (j = 0; j < 3; j++) {
			unsigned long dc = now[i][j][0] - before[i][j][0];
			unsigned long dt = now[i][j][1] - before[i][j][1];

			seq_buf_printf(&s, "%s%s %s +%lu (%lu ms)", i || j ? ", " : "",
				       kind[i], what[j], dc,
				       dt * 1000 / SPM_TICKS_PER_SEC);
		}
	dev_info(lpm->dev, "suspend wake up, TF-A counters during the sleep: %s\n",
		 seq_buf_str(&s));
}

static void mtk_lpm_log_wake(struct mtk_lpm *lpm)
{
	struct mtk_lpm_wake w;
	char buf[256], blk[192], rcb[128];
	struct seq_buf s, b, r;
	u32 sec, ms, pct26m = 0, pctvcore = 0;
	int i, n = 0;

	mtk_lpm_read_wake(lpm, &w);
	w.abort_reason = lpm_smc_cpu_pm_lp(CPU_PM_LP_SUSPEND_ABORT_REASON,
					   LPM_ACT_GET, 0, 0);

	/* which constraint TF-A took for this sleep, and for how long */
	seq_buf_init(&r, rcb, sizeof(rcb));
	for (i = 0; i < ARRAY_SIZE(mtk_lpm_constraints); i++) {
		u8 id = mtk_lpm_constraints[i].id;
		unsigned long cnt;

		cnt = lpm_smc_spm_dbg(SPM_DBG_RC_CNT, LPM_ACT_GET, id, 0);
		w.rc_entered[i] = cnt - lpm->entry.rc_cnt[i];
		w.rc_res[i] = lpm_smc_spm_dbg(SPM_DBG_RC_RES_INFO,
					      LPM_ACT_GET | LPM_ACT_CLR, id, 0);
		lpm_smc_spm_dbg(SPM_DBG_RC_RES_CTRL, LPM_ACT_CLR, id, 0);
		if (w.rc_entered[i] || w.rc_res[i])
			seq_buf_printf(&r, "%s%s x%lu %lu ms", n++ ? ", " : "",
				       mtk_lpm_constraints[i].name,
				       w.rc_entered[i],
				       w.rc_res[i] * 1000 / SPM_TICKS_PER_SEC);
	}
	if (!n)
		seq_buf_printf(&r, "none");

	seq_buf_init(&s, buf, sizeof(buf));
	mtk_lpm_wake_sources(&w, &s);
	seq_buf_init(&b, blk, sizeof(blk));
	mtk_lpm_wake_blockers(&w, &b);

	sec = w.timer_out / SPM_TICKS_PER_SEC;
	ms = (w.timer_out % SPM_TICKS_PER_SEC) * 1000 / SPM_TICKS_PER_SEC;
	if (w.timer_out) {
		pct26m = div_u64((u64)w.vtcxo_dur * 100, w.timer_out);
		pctvcore = div_u64((u64)w.vcore_dur * 100, w.timer_out);
	}

	dev_info(lpm->dev,
		 "suspend wake up by %s, slept %u.%03u s, 26M off %u%%, vcore off %u%%, deepest %s, requests: %s\n",
		 buf, sec, ms, pct26m, pctvcore,
		 w.debug_flag & DEBUG_FLAG_DEEPEST ? "yes" : "no", blk);
	dev_dbg(lpm->dev,
		"r12 0x%08x r12_ext 0x%08x raw_sta 0x%08x raw_ext 0x%08x misc 0x%08x r13 0x%08x debug 0x%08x 0x%08x src_req 0x%08x sw_flag 0x%08x 0x%08x isr 0x%08x\n",
		w.r12, w.r12_ext, w.raw_sta, w.raw_ext_sta, w.wake_misc, w.r13,
		w.debug_flag, w.debug_flag1, w.src_req, w.sw_flag0, w.sw_flag1,
		w.isr);
	mtk_lpm_log_req_lines(lpm, "suspend wake up", w.req_sta, w.src_mask);
	mtk_lpm_log_rc(lpm, "suspend wake up");
	mtk_lpm_log_cg_check(lpm, "suspend wake up");
	mtk_lpm_log_pwr(lpm, "suspend wake up");
	dev_info(lpm->dev, "suspend constraints entered: %s\n", rcb);
	mtk_lpm_log_stats(lpm, lpm->entry.stat);
	if (w.abort_reason)
		dev_warn(lpm->dev, "TF-A suspend abort reason %lu\n",
			 w.abort_reason);

	spin_lock(&lpm->lock);
	w.seq = ++lpm->wake.seq;
	lpm->wake = w;
	spin_unlock(&lpm->lock);
}

/* ---- platform s2idle ops ------------------------------------------------ */

static int mtk_lpm_s2idle_begin(void)
{
	struct mtk_lpm *lpm = mtk_lpm_instance;

	spin_lock(&lpm->lock);
	lpm->suspends++;
	lpm->in_suspend = true;
	spin_unlock(&lpm->lock);
	return 0;
}

/* enable the constraints the firmware has off, for this suspend only */
static void mtk_lpm_force_rc(struct mtk_lpm *lpm)
{
	int i;

	lpm->rc_forced = 0;
	lpm->rc_forced_switch = 0;
	if (runtime || !suspend_rc)
		return;

	for (i = 0; i < ARRAY_SIZE(mtk_lpm_constraints); i++) {
		u8 id = mtk_lpm_constraints[i].id;

		if (lpm_smc_spm_dbg(SPM_DBG_RC_SWITCH, LPM_ACT_GET, id, 0) &
		    SPM_RC_VALID_SW)
			continue;
		/* the downstream boot-time enable */
		lpm_smc_spm_dbg(SPM_DBG_DOE_RC, LPM_ACT_SET, id, 0);
		lpm->rc_forced |= BIT(i);
		if (lpm_smc_spm_dbg(SPM_DBG_RC_SWITCH, LPM_ACT_GET, id, 0) &
		    SPM_RC_VALID_SW)
			continue;
		lpm_smc_spm_dbg(SPM_DBG_RC_SWITCH, LPM_ACT_SET, id, 0);
		lpm->rc_forced_switch |= BIT(i);
	}
}

static void mtk_lpm_unforce_rc(struct mtk_lpm *lpm)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mtk_lpm_constraints); i++) {
		u8 id = mtk_lpm_constraints[i].id;

		if (lpm->rc_forced_switch & BIT(i))
			lpm_smc_spm_dbg(SPM_DBG_RC_SWITCH, LPM_ACT_CLR, id, 0);
		if (lpm->rc_forced & BIT(i))
			lpm_smc_spm_dbg(SPM_DBG_DOE_RC, LPM_ACT_CLR, id, 0);
	}
	lpm->rc_forced = 0;
	lpm->rc_forced_switch = 0;
}

/*
 * What the downstream kernel sends at boot and ours, with runtime=0, did not:
 * the eight resources released (factory device tree values) and, if asked for, the
 * platform-ready word. Before every suspend, after the constraints.
 */
static void mtk_lpm_boot_init(struct mtk_lpm *lpm)
{
	int i;

	if (res_ctrl && !runtime) {
		for (i = 0; i < MTK_LPM_NUM_RESOURCES; i++)
			lpm_smc_spm_dbg(SPM_DBG_DOE_RESOURCE_CTRL, LPM_ACT_SET,
					i, 0);
	}
	if (validate && !runtime && !lpm->validated) {
		lpm_smc_cpu_pm(CPU_PM_VALIDATE_PWR_STATE_CTRL, LPM_ACT_SET, 0, 0);
		lpm->validated = true;
		dev_info(lpm->dev, "platform-ready word sent (mtk_lpm.validate=1)\n");
	}
}

/*
 * TF-A's software resource requests (mt_lp_rq): who asks the SPM to keep
 * which resource through the sleep, e.g. "CK_26M(ufs) VCORE(ufs)", as the
 * downstream logger's "suspend warning: <res> request: <users>". These
 * are not in SPM_REQ_STA; a kernel driver holds them through TF-A.
 */
static void mtk_lpm_sw_requests(struct seq_buf *s)
{
	unsigned long rnum, unum, usage, valid, per, name;
	int i, j, k, n = 0;
	char uname[SPM_RES_USER_NAME_LEN + 1];

	rnum = lpm_smc_spm_dbg(SPM_DBG_RES_NUM, LPM_ACT_GET, 0, 0);
	unum = lpm_smc_spm_dbg(SPM_DBG_RES_USER_NUM, LPM_ACT_GET, 0, 0);
	if (!rnum || rnum > 32 || unum > 32) {
		seq_buf_printf(s, "unknown (res %lu users %lu)", rnum, unum);
		return;
	}
	usage = mtk_lpm_smc_ul(LPM_USER_SPM_DBG, SPM_DBG_RES_USAGE,
			       LPM_ACT_GET, SPM_RES_USAGE_ALL, 0);
	valid = lpm_smc_spm_dbg(SPM_DBG_RES_USER_VALID, LPM_ACT_GET, 0, 0);
	for (i = 0; i < rnum; i++) {
		if (!(usage & BIT(i)))
			continue;
		seq_buf_printf(s, "%s%s(", n++ ? " " : "",
			       i < ARRAY_SIZE(mtk_lpm_sw_res) ?
			       mtk_lpm_sw_res[i] : "res?");
		per = lpm_smc_spm_dbg(SPM_DBG_RES_USAGE, LPM_ACT_GET, i, 0);
		for (j = 0, k = 0; j < unum; j++) {
			int c;

			if (!(per & BIT(j)))
				continue;
			name = lpm_smc_spm_dbg(SPM_DBG_RES_USER_NAME,
					       LPM_ACT_GET, j, 0);
			for (c = 0; c < SPM_RES_USER_NAME_LEN; c++) {
				u8 ch = (name >> (8 * c)) & 0xff;

				uname[c] = ch >= 0x20 && ch < 0x7f ? ch : '\0';
				if (!uname[c])
					break;
			}
			uname[c] = '\0';
			seq_buf_printf(s, "%s%s%s", k++ ? "," : "",
				       uname[0] ? uname : "?",
				       valid & BIT(j) ? "" : "[bypassed]");
		}
		seq_buf_printf(s, ")");
	}
	if (!n)
		seq_buf_printf(s, "none");
	usage = lpm_smc_spm_dbg(SPM_DBG_LP_REQ_STAT, LPM_ACT_GET, 0, 0);
	for (i = 0; i < ARRAY_SIZE(mtk_lpm_lp_scenes); i++)
		if (usage & BIT(i))
			seq_buf_printf(s, " scene:%s", mtk_lpm_lp_scenes[i]);
}

/*
 * All devices are suspended (noirq), the CPUs not yet idle: who requests
 * SPM resources now and what blocks each constraint is what the sleep
 * starts with.
 */
static void mtk_lpm_log_entry(struct mtk_lpm *lpm)
{
	struct mtk_lpm_entry e = { };
	char buf[384], blk[192];
	struct seq_buf s, b;
	int i;

	for (i = 0; i < SPM_REQ_STA_NUM; i++) {
		e.req_sta[i] = mtk_lpm_rd(lpm, SPM_REQ_STA(i));
		e.src_mask[i] = mtk_lpm_rd(lpm, SPM_SRC_MASK(i));
	}
	e.src_req = mtk_lpm_rd(lpm, SPM_SRC_REQ);
	e.rc_forced = lpm->rc_forced;
	mtk_lpm_read_stats(e.stat);

	seq_buf_init(&b, blk, sizeof(blk));
	mtk_lpm_blockers(e.req_sta, e.src_req, &b);
	seq_buf_init(&s, buf, sizeof(buf));
	for (i = 0; i < ARRAY_SIZE(mtk_lpm_constraints); i++) {
		u8 id = mtk_lpm_constraints[i].id;

		e.rc_sw[i] = lpm_smc_spm_dbg(SPM_DBG_RC_SWITCH, LPM_ACT_GET,
					     id, 0);
		e.rc_block[i] = lpm_smc_spm_dbg(SPM_DBG_COND_BLOCK,
						LPM_ACT_GET, id, 0);
		e.rc_check[i] = lpm_smc_spm_dbg(SPM_DBG_COND_CHECK,
						LPM_ACT_GET, id, 0);
		e.rc_cnt[i] = lpm_smc_spm_dbg(SPM_DBG_RC_CNT, LPM_ACT_GET,
					      id, 0);
		/* residency of this sleep: cleared now, read at the wake-up */
		lpm_smc_spm_dbg(SPM_DBG_RC_RES_CTRL, LPM_ACT_SET, id, 0);
		lpm_smc_spm_dbg(SPM_DBG_RC_RES_INFO, LPM_ACT_GET | LPM_ACT_CLR,
				id, 0);
		seq_buf_printf(&s, "%s%s sw=%d%s fw=%d check=0x%lx block=0x%lx(",
			       i ? ", " : "", mtk_lpm_constraints[i].name,
			       !!(e.rc_sw[i] & SPM_RC_VALID_SW),
			       e.rc_forced & BIT(i) ? " (enabled for suspend)" : "",
			       !!(e.rc_sw[i] & SPM_RC_VALID_FW),
			       e.rc_check[i], e.rc_block[i]);
		mtk_lpm_block_names(e.rc_block[i], &s);
		seq_buf_printf(&s, ")");
	}

	seq_buf_init(&b, e.sw_req, sizeof(e.sw_req));
	mtk_lpm_sw_requests(&b);

	dev_info(lpm->dev, "suspend entry: requests %s; %s\n", blk, buf);
	dev_info(lpm->dev, "suspend entry: software requests to the SPM: %s; resources %s\n",
		 e.sw_req, res_ctrl && !runtime ? "released (factory device tree values)" :
		 runtime ? "set at boot" : "as booted (mtk_lpm.res_ctrl=0)");
	mtk_lpm_log_req_lines(lpm, "suspend entry", e.req_sta, e.src_mask);
	mtk_lpm_log_rc(lpm, "suspend entry");
	mtk_lpm_log_dcm(lpm, "suspend entry");
	mtk_lpm_log_cg_check(lpm, "suspend entry");
	mtk_lpm_log_pwr(lpm, "suspend entry");

	spin_lock(&lpm->lock);
	e.seq = lpm->entry.seq + 1;
	lpm->entry = e;
	spin_unlock(&lpm->lock);
}

/* see dvfsrc_start */
static void mtk_lpm_dvfsrc_start(struct mtk_lpm *lpm)
{
	struct arm_smccc_res init, start;

	if (!READ_ONCE(dvfsrc_start) || lpm->dvfsrc_started)
		return;
	lpm->dvfsrc_started = true;
	arm_smccc_smc(MTK_SIP_VCOREFS_CONTROL, MTK_SIP_DVFSRC_INIT, 0, 0, 0, 0,
		      0, 0, &init);
	arm_smccc_smc(MTK_SIP_VCOREFS_CONTROL, MTK_SIP_DVFSRC_START, 0, 0, 0, 0,
		      0, 0, &start);
	dev_info(lpm->dev, "DVFSRC started as the downstream kernel does: init a0 %#lx a1 %#lx (DRAM type), start a0 %#lx; SPM flags %08x\n",
		 init.a0, init.a1, start.a0, mtk_lpm_rd(lpm, SPM_SW_FLAG_0));
}

/* see sleep_gate_unused */
static void mtk_lpm_sleep_gate(struct mtk_lpm *lpm)
{
	const char *prefixes = READ_ONCE(sleep_gate_unused);
	char *log;
	unsigned int n;

	if (!prefixes || !*prefixes)
		return;
	log = kmalloc(1024, GFP_KERNEL);
	n = clk_sleep_gate_unused(prefixes, READ_ONCE(sleep_gate_exclude),
				  log, log ? 1024 : 0);
	dev_info(lpm->dev, "suspend entry: %u unused clocks gated for the sleep (%s): %s\n",
		 n, prefixes, log && n ? log : "-");
	kfree(log);
}

static int mtk_lpm_s2idle_prepare_late(void)
{
	struct mtk_lpm *lpm = mtk_lpm_instance;

	mtk_lpm_force_rc(lpm);
	mtk_lpm_boot_init(lpm);
	mtk_lpm_dvfsrc_start(lpm);
	mtk_lpm_sleep_gate(lpm);
	mtk_lpm_req_mask(lpm, true);
	mtk_lpm_src_unmask(lpm, true);
	mtk_lpm_log_entry(lpm);
	return 0;
}

static void mtk_lpm_s2idle_restore_early(void)
{
	struct mtk_lpm *lpm = mtk_lpm_instance;

	/* before the devices resume and change the request status */
	mtk_lpm_log_wake(lpm);
	mtk_lpm_req_mask(lpm, false);
	mtk_lpm_src_unmask(lpm, false);
	clk_sleep_ungate_unused();
	mtk_lpm_unforce_rc(lpm);
}

static void mtk_lpm_s2idle_end(void)
{
	struct mtk_lpm *lpm = mtk_lpm_instance;

	spin_lock(&lpm->lock);
	lpm->in_suspend = false;
	spin_unlock(&lpm->lock);
}

static const struct platform_s2idle_ops mtk_lpm_s2idle_ops = {
	.begin = mtk_lpm_s2idle_begin,
	.prepare_late = mtk_lpm_s2idle_prepare_late,
	.restore_early = mtk_lpm_s2idle_restore_early,
	.end = mtk_lpm_s2idle_end,
};

/* ---- the SPM interrupt: TF-A sets it pending on wake-up ----------------- */

static irqreturn_t mtk_lpm_spm_irq(int irq, void *data)
{
	/* nothing to acknowledge: the firmware raised it to mark the wake-up */
	return IRQ_HANDLED;
}

static int mtk_lpm_init_spm_irq(struct mtk_lpm *lpm)
{
	struct irq_data *d;
	int ret;

	ret = devm_request_irq(lpm->dev, lpm->irq, mtk_lpm_spm_irq, 0,
			       "mtk-lpm-spm", lpm);
	if (ret)
		return ret;
	d = irq_get_irq_data(lpm->irq);
	if (!d)
		return -EINVAL;
	lpm->hwirq = irqd_to_hwirq(d);
	ret = enable_irq_wake(lpm->irq);
	if (ret)
		return ret;
	lpm_smc_spm(SPM_SET_PENDING_IRQ_INIT, LPM_ACT_SET, lpm->hwirq, 0);
	return 0;
}

/* ---- irq remain: interrupts mirrored into the SYS_CIRQ ----------------- */

static int mtk_lpm_init_irq_remain(struct mtk_lpm *lpm)
{
	struct device_node *np = lpm->dev->of_node;
	struct of_phandle_args args;
	unsigned int *hwirqs;
	int count = 0, i, n = 0;

	/* <&node irq-index> pairs */
	while (!of_parse_phandle_with_fixed_args(np, "mediatek,irq-remain", 1,
						 count, &args)) {
		of_node_put(args.np);
		count++;
	}
	if (!count)
		return 0;

	hwirqs = kcalloc(count, sizeof(*hwirqs), GFP_KERNEL);
	if (!hwirqs)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		struct irq_data *d;
		int irq;

		if (of_parse_phandle_with_fixed_args(np, "mediatek,irq-remain",
						     1, i, &args))
			continue;
		irq = of_irq_get(args.np, args.args[0]);
		of_node_put(args.np);
		if (irq == -EPROBE_DEFER) {
			kfree(hwirqs);
			return irq;
		}
		if (irq <= 0) {
			dev_warn(lpm->dev, "irq-remain entry %d has no interrupt (%d)\n",
				 i, irq);
			continue;
		}
		d = irq_get_irq_data(irq);
		if (!d)
			continue;
		hwirqs[n++] = irqd_to_hwirq(d);
	}

	/* both interfaces, as the downstream driver does */
	lpm_smc_cpu_pm(CPU_PM_IRQ_REMAIN_LIST_ALLOC, n, 0, 0);
	lpm_smc_cpu_pm_lp(CPU_PM_LP_IRQS_REMAIN_ALLOC, LPM_ACT_SET, n, 0);
	for (i = 0; i < n; i++) {
		lpm_smc_cpu_pm(CPU_PM_IRQ_REMAIN_IRQ_ADD, hwirqs[i], 0, 0);
		lpm_smc_cpu_pm_lp(CPU_PM_LP_IRQS_REMAIN_IRQ, LPM_ACT_SET, hwirqs[i], 0);
		lpm_smc_cpu_pm_lp(CPU_PM_LP_IRQS_REMAIN_WAKEUP_CAT, LPM_ACT_SET, 0, 0);
		lpm_smc_cpu_pm_lp(CPU_PM_LP_IRQS_REMAIN_WAKEUP_SRC, LPM_ACT_SET, 0, 0);
		lpm_smc_cpu_pm_lp(CPU_PM_LP_IRQS_REMAIN_CTRL, LPM_ACT_PUSH, 0, 0);
	}
	lpm_smc_cpu_pm(CPU_PM_IRQ_REMAIN_IRQ_SUBMIT, 0, 0, 0);
	lpm_smc_cpu_pm_lp(CPU_PM_LP_IRQS_REMAIN_CTRL, LPM_ACT_SUBMIT, 0, 0);

	dev_dbg(lpm->dev, "%d interrupts remain through MCUSYS off\n", n);
	kfree(hwirqs);
	return 0;
}

/* ---- resources and constraints ----------------------------------------- */

static void mtk_lpm_init_constraints(struct mtk_lpm *lpm)
{
	int i;

	/* no resource is forced on by the kernel */
	for (i = 0; i < MTK_LPM_NUM_RESOURCES; i++)
		lpm_smc_spm_dbg(SPM_DBG_DOE_RESOURCE_CTRL, LPM_ACT_SET, i, 0);

	for (i = 0; i < ARRAY_SIZE(mtk_lpm_constraints); i++) {
		unsigned long st;

		lpm_smc_spm_dbg(SPM_DBG_DOE_RC, LPM_ACT_SET,
				mtk_lpm_constraints[i].id, 0);
		st = lpm_smc_spm_dbg(SPM_DBG_RC_SWITCH, LPM_ACT_GET,
				     mtk_lpm_constraints[i].id, 0);
		dev_dbg(lpm->dev, "constraint %s: sw %s, fw %s\n",
			mtk_lpm_constraints[i].name,
			st & SPM_RC_VALID_SW ? "on" : "off",
			st & SPM_RC_VALID_FW ? "ready" : "not ready");
	}
}

/* ---- sysfs ------------------------------------------------------------- */

static ssize_t last_wake_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct mtk_lpm *lpm = dev_get_drvdata(dev);
	struct mtk_lpm_wake w;
	struct seq_buf s;
	int i;

	spin_lock(&lpm->lock);
	w = lpm->wake;
	spin_unlock(&lpm->lock);

	seq_buf_init(&s, buf, PAGE_SIZE);
	seq_buf_printf(&s, "seq %llu\nsources ", w.seq);
	mtk_lpm_wake_sources(&w, &s);
	seq_buf_printf(&s, "\nr12 0x%08x\nr12_ext 0x%08x\nraw_sta 0x%08x\nraw_ext_sta 0x%08x\nwake_misc 0x%08x\nr13 0x%08x\n",
		       w.r12, w.r12_ext, w.raw_sta, w.raw_ext_sta, w.wake_misc,
		       w.r13);
	seq_buf_printf(&s, "timer_out %u\nslept_ms %u\n26m_off_ticks %u\nvcore_off_ticks %u\n",
		       w.timer_out,
		       (u32)div_u64((u64)w.timer_out * 1000, SPM_TICKS_PER_SEC),
		       w.vtcxo_dur, w.vcore_dur);
	seq_buf_printf(&s, "deepest %d\ndebug_flag 0x%08x 0x%08x\nabort_reason %lu\nrequests ",
		       !!(w.debug_flag & DEBUG_FLAG_DEEPEST), w.debug_flag,
		       w.debug_flag1, w.abort_reason);
	mtk_lpm_wake_blockers(&w, &s);
	seq_buf_printf(&s, "\nspm_honours ");
	mtk_lpm_req_decode(w.req_sta, w.src_mask, false, &s);
	seq_buf_printf(&s, "\nspm_masked ");
	mtk_lpm_req_decode(w.req_sta, w.src_mask, true, &s);
	seq_buf_printf(&s, "\nreq_sta");
	for (i = 0; i < SPM_REQ_STA_NUM; i++)
		seq_buf_printf(&s, " 0x%08x", w.req_sta[i]);
	seq_buf_printf(&s, "\nsrc_mask");
	for (i = 0; i < SPM_REQ_STA_NUM; i++)
		seq_buf_printf(&s, " 0x%08x", w.src_mask[i]);
	seq_buf_printf(&s, "\nsrc_req 0x%08x\nsw_flag 0x%08x 0x%08x\nisr 0x%08x\n",
		       w.src_req, w.sw_flag0, w.sw_flag1, w.isr);
	for (i = 0; i < ARRAY_SIZE(mtk_lpm_constraints); i++)
		seq_buf_printf(&s, "rc_%s entered %lu residency_ms %lu\n",
			       mtk_lpm_constraints[i].name, w.rc_entered[i],
			       w.rc_res[i] * 1000 / SPM_TICKS_PER_SEC);
	return seq_buf_used(&s);
}
static DEVICE_ATTR_RO(last_wake);

/* the snapshot at the last suspend entry */
static ssize_t last_entry_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mtk_lpm *lpm = dev_get_drvdata(dev);
	struct mtk_lpm_entry e;
	struct seq_buf s;
	int i;

	spin_lock(&lpm->lock);
	e = lpm->entry;
	spin_unlock(&lpm->lock);

	seq_buf_init(&s, buf, PAGE_SIZE);
	seq_buf_printf(&s, "seq %llu\nrequests ", e.seq);
	mtk_lpm_blockers(e.req_sta, e.src_req, &s);
	seq_buf_printf(&s, "\nspm_honours ");
	mtk_lpm_req_decode(e.req_sta, e.src_mask, false, &s);
	seq_buf_printf(&s, "\nspm_masked ");
	mtk_lpm_req_decode(e.req_sta, e.src_mask, true, &s);
	seq_buf_printf(&s, "\nreq_sta");
	for (i = 0; i < SPM_REQ_STA_NUM; i++)
		seq_buf_printf(&s, " 0x%08x", e.req_sta[i]);
	seq_buf_printf(&s, "\nsrc_mask");
	for (i = 0; i < SPM_REQ_STA_NUM; i++)
		seq_buf_printf(&s, " 0x%08x", e.src_mask[i]);
	seq_buf_printf(&s, "\nsrc_req 0x%08x\nsw_requests %s\n", e.src_req,
		       e.sw_req);
	for (i = 0; i < ARRAY_SIZE(mtk_lpm_constraints); i++) {
		seq_buf_printf(&s, "%s sw=%d fw=%d forced=%d check=0x%lx block=0x%lx ",
			       mtk_lpm_constraints[i].name,
			       !!(e.rc_sw[i] & SPM_RC_VALID_SW),
			       !!(e.rc_sw[i] & SPM_RC_VALID_FW),
			       !!(e.rc_forced & BIT(i)), e.rc_check[i],
			       e.rc_block[i]);
		mtk_lpm_block_names(e.rc_block[i], &s);
		seq_buf_printf(&s, "\n");
	}
	return seq_buf_used(&s);
}
static DEVICE_ATTR_RO(last_entry);

static ssize_t mtk_lpm_stat_show(char *buf, u32 id)
{
	static const char * const names[] = { "mcusys", "26m", "vcore" };
	struct seq_buf s;
	int i;

	seq_buf_init(&s, buf, PAGE_SIZE);
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		unsigned long count, dur;

		count = lpm_smc_spm_dbg(id, LPM_ACT_GET, i, SPM_SLP_COUNT);
		dur = lpm_smc_spm_dbg(id, LPM_ACT_GET, i, SPM_SLP_DURATION);
		seq_buf_printf(&s, "%s %lu %lu.%03lu\n", names[i], count,
			       dur / SPM_TICKS_PER_SEC,
			       (dur % SPM_TICKS_PER_SEC) * 1000 / SPM_TICKS_PER_SEC);
	}
	return seq_buf_used(&s);
}

/* "<state> <count> <seconds>" of the firmware's system idle statistics */
static ssize_t idle_stat_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	return mtk_lpm_stat_show(buf, SPM_DBG_IDLE_PWR_STAT);
}
static DEVICE_ATTR_RO(idle_stat);

static ssize_t suspend_stat_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return mtk_lpm_stat_show(buf, SPM_DBG_SUSPEND_PWR_STAT);
}
static DEVICE_ATTR_RO(suspend_stat);

static ssize_t constraints_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct seq_buf s;
	int i;

	seq_buf_init(&s, buf, PAGE_SIZE);
	for (i = 0; i < ARRAY_SIZE(mtk_lpm_constraints); i++) {
		u8 id = mtk_lpm_constraints[i].id;
		unsigned long st, cnt, blk;

		st = lpm_smc_spm_dbg(SPM_DBG_RC_SWITCH, LPM_ACT_GET, id, 0);
		cnt = lpm_smc_spm_dbg(SPM_DBG_RC_CNT, LPM_ACT_GET, id, 0);
		blk = lpm_smc_spm_dbg(SPM_DBG_COND_BLOCK, LPM_ACT_GET, id, 0);
		seq_buf_printf(&s, "%s sw=%d fw=%d count=%lu block=0x%lx ",
			       mtk_lpm_constraints[i].name,
			       !!(st & SPM_RC_VALID_SW), !!(st & SPM_RC_VALID_FW),
			       cnt, blk);
		mtk_lpm_block_names(blk, &s);
		seq_buf_printf(&s, "\n");
	}
	return seq_buf_used(&s);
}
/* "<name> on|off" or "all on|off": switch a constraint at run time */
static ssize_t constraints_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	char name[16], val[8];
	bool on, found = false;
	int i;

	if (sscanf(buf, "%15s %7s", name, val) != 2)
		return -EINVAL;
	if (!strcmp(val, "on") || !strcmp(val, "1"))
		on = true;
	else if (!strcmp(val, "off") || !strcmp(val, "0"))
		on = false;
	else
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(mtk_lpm_constraints); i++) {
		if (strcmp(name, "all") && strcmp(name, mtk_lpm_constraints[i].name))
			continue;
		lpm_smc_spm_dbg(SPM_DBG_RC_SWITCH, on ? LPM_ACT_SET : LPM_ACT_CLR,
				mtk_lpm_constraints[i].id, 0);
		dev_info(dev, "constraint %s %s\n", mtk_lpm_constraints[i].name,
			 on ? "on" : "off");
		found = true;
	}
	return found ? count : -ENOENT;
}
static DEVICE_ATTR_RW(constraints);

static ssize_t suspends_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct mtk_lpm *lpm = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%llu\n", lpm->suspends);
}
static DEVICE_ATTR_RO(suspends);

static struct attribute *mtk_lpm_attrs[] = {
	&dev_attr_last_wake.attr,
	&dev_attr_last_entry.attr,
	&dev_attr_idle_stat.attr,
	&dev_attr_suspend_stat.attr,
	&dev_attr_constraints.attr,
	&dev_attr_suspends.attr,
	NULL
};

static const struct attribute_group mtk_lpm_group = {
	.name = "lpm",
	.attrs = mtk_lpm_attrs,
};
__ATTRIBUTE_GROUPS(mtk_lpm);

/* ---- probe ------------------------------------------------------------- */

static int mtk_lpm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_lpm *lpm;
	unsigned long fw;
	unsigned int i;
	int ret;

	if (mtk_lpm_instance)
		return -EBUSY;
	if (!enable) {
		dev_info(dev, "disabled (mtk_lpm.enable=0): firmware left as booted\n");
		return -ENODEV;
	}

	lpm = devm_kzalloc(dev, sizeof(*lpm), GFP_KERNEL);
	if (!lpm)
		return -ENOMEM;
	lpm->dev = dev;
	spin_lock_init(&lpm->lock);

	/* only for the log; the sleep works without it */
	lpm->rc = devm_ioremap(dev, MT6878_SRCLKEN_RC_BASE, MT6878_SRCLKEN_RC_SIZE);
	lpm->infracfg = devm_ioremap(dev, MT6878_INFRACFG_AO_BASE, SZ_4K);
	BUILD_BUG_ON(ARRAY_SIZE(mtk_lpm_dcm_regs) != ARRAY_SIZE(lpm->dcm));
	for (i = 0; i < ARRAY_SIZE(mtk_lpm_dcm_regs); i++)
		lpm->dcm[i] = devm_ioremap(dev, mtk_lpm_dcm_regs[i].pa, 4);
	lpm->spm = syscon_node_to_regmap(dev->of_node->parent);
	if (IS_ERR(lpm->spm))
		return dev_err_probe(dev, PTR_ERR(lpm->spm),
				     "no SPM regmap from the parent syscon\n");

	lpm->irq = platform_get_irq(pdev, 0);
	if (lpm->irq < 0)
		return lpm->irq;

	if (runtime) {
		ret = mtk_lpm_init_irq_remain(lpm);
		if (ret)
			return ret;
	}

	ret = mtk_lpm_hook_cpuidle(lpm);
	if (ret)
		return ret;
	mtk_lpm_instance = lpm;
	platform_set_drvdata(pdev, lpm);

	ret = mtk_lpm_init_spm_irq(lpm);
	if (ret) {
		dev_err(dev, "SPM interrupt: %d\n", ret);
		goto err_unhook;
	}

	if (runtime) {
		mtk_lpm_init_constraints(lpm);

		/* the platform is ready for the CPU power states: no boot-up hold-off */
		cpuidle_pause_and_lock();
		lpm_smc_cpu_pm(CPU_PM_VALIDATE_PWR_STATE_CTRL, LPM_ACT_SET, 0, 0);
		cpuidle_resume_and_unlock();
	}

	s2idle_set_ops(&mtk_lpm_s2idle_ops);

	fw = lpm_smc_spm(SPM_FW_TYPE, LPM_ACT_GET, 0, 0);
	dev_info(dev, "SPM firmware type %lu, suspend source on %d states, SPM irq %d (hwirq %u), runtime settings %s\n",
		 fw, lpm->idle_drv->state_count - 1, lpm->irq, lpm->hwirq,
		 runtime ? "applied" : "left to the firmware (mtk_lpm.runtime=0)");
	return 0;

err_unhook:
	mtk_lpm_instance = NULL;
	mtk_lpm_unhook_cpuidle(lpm);
	return ret;
}

static void mtk_lpm_remove(struct platform_device *pdev)
{
	struct mtk_lpm *lpm = platform_get_drvdata(pdev);

	s2idle_set_ops(NULL);
	disable_irq_wake(lpm->irq);
	mtk_lpm_unhook_cpuidle(lpm);
	mtk_lpm_instance = NULL;
}

static const struct of_device_id mtk_lpm_of_match[] = {
	{ .compatible = "mediatek,mt6878-lpm" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_lpm_of_match);

static struct platform_driver mtk_lpm_driver = {
	.probe = mtk_lpm_probe,
	.remove = mtk_lpm_remove,
	.driver = {
		.name = "mtk-lpm",
		.of_match_table = mtk_lpm_of_match,
		.dev_groups = mtk_lpm_groups,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(mtk_lpm_driver);

MODULE_DESCRIPTION("MediaTek MT6878 low power manager");
MODULE_LICENSE("GPL");
