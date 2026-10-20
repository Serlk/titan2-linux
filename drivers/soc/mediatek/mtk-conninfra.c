// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CONNINFRA: power and bus infrastructure of the connectivity
 * subsystem (CONNSYS) shared by Wi-Fi, Bluetooth, GNSS and FM.
 *
 * Copyright (c) 2026 serlk
 *
 * The power-on sequence, the register offsets and values, the expected
 * IP versions and the command batch table follow MediaTek's GPL-2.0
 * conninfra driver (conn_drv/connv2, platform mt6878, generated from the
 * MT6878 "POS" tables), Copyright (c) 2023 MediaTek Inc. The Wi-Fi
 * subsystem power-on follows MediaTek's wlan gen4m driver (chips/soc7_0,
 * BSD-2-Clause), Copyright (c) 2021 MediaTek Inc.
 *
 * On probe the driver only checks that CONN_INFRA answers (PMIC rails, CONN
 * power domain, IP version) and switches it off again. The full power-on
 * (EMI remapping, D-die and A-die configuration over the RF SPI, PLL, low
 * power settings) runs when a radio driver asks for it through
 * mtk_conninfra_radio_on(), or on an explicit debugfs command. Only the
 * MT6637 A-die is supported so far.
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <linux/soc/mediatek/mtk-conninfra.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "mtk-conninfra-mt6878.h"

/* ------------------------------------------------------------------ */
/* CONN_INFRA register map, offsets from 0x1800_0000                   */

/* conn_infra_rgu_on */
#define RGU_ON				0x00000
#define  RGU_CONN_INFRA_OFF_TOP_PWR_CTL	(RGU_ON + 0x000)
#define  RGU_WFSYS_ON_TOP_PWR_CTL	(RGU_ON + 0x010)
#define   TOP_PWR_CTL_KEY		GENMASK(31, 16)
#define   TOP_PWR_CTL_ON		BIT(7)
#define   WFSYS_TOP_KEY			0x5746
#define  RGU_BGFSYS_ON_TOP_PWR_CTL	(RGU_ON + 0x020)
#define  RGU_SYSRAM_HWCTL_PDN		(RGU_ON + 0x050)
#define  RGU_SYSRAM_HWCTL_SLP		(RGU_ON + 0x054)
#define  RGU_CO_EXT_MEM_HWCTL_PDN	(RGU_ON + 0x070)
#define  RGU_CO_EXT_MEM_HWCTL_SLP	(RGU_ON + 0x074)
#define  RGU_WFSYS_CPU_SW_RST_B		(RGU_ON + 0x120)
#define  RGU_SEMA_M0_SW_RST_B		(RGU_ON + 0x158)

/* conn_infra_cfg_on */
#define CFG_ON				0x01000
#define  CFG_ON_ADIE_CTL		(CFG_ON + 0x010)
#define  CFG_ON_PWRCTRL0		(CFG_ON + 0x200)
#define  CFG_ON_PWRCTRL1		(CFG_ON + 0x210)
#define   PWRCTRL1_CONN_INFRA_RDY	BIT(16)
#define  CFG_ON_OSC_CTL_0		(CFG_ON + 0x300)
#define  CFG_ON_OSC_CTL_1		(CFG_ON + 0x304)
#define  CFG_ON_RC_CTL_0		(CFG_ON + 0x340)
#define  CFG_ON_RC_CTL_1		(CFG_ON + 0x348)
#define  CFG_ON_RC_CTL_0_GPS		(CFG_ON + 0x350)
#define  CFG_ON_RC_CTL_1_GPS		(CFG_ON + 0x354)
#define  CFG_ON_RC_CTL_0_BT		(CFG_ON + 0x360)
#define  CFG_ON_RC_CTL_1_BT		(CFG_ON + 0x364)
#define  CFG_ON_RC_CTL_0_WF		(CFG_ON + 0x370)
#define  CFG_ON_RC_CTL_1_WF		(CFG_ON + 0x374)
#define  CFG_ON_RC_CTL_0_TOP		(CFG_ON + 0x380)
#define  CFG_ON_RC_CTL_1_TOP		(CFG_ON + 0x384)
#define  CFG_ON_WF_SLP_CTRL		(CFG_ON + 0x440)
#define   WF_SLP_PROT_SW_EN		BIT(0)
#define  CFG_ON_WF_SLP_STATUS		(CFG_ON + 0x444)
#define   WF2CONN_SLP_PROT_RDY		BIT(31)
#define   WF2CONN_SLP_PROT_HW_EN	BIT(30)
#define   CONN2WF_SLP_PROT_RDY		BIT(29)
#define   WFDMA2CONN_SLP_PROT_RDY	BIT(25)

/* conn_wt_slp_ctl_reg: A-die sleep/wakeup controller */
#define WT_SLP				0x03000
#define  WB_SLP_CTL			(WT_SLP + 0x004)
#define  WB_BG_ADDR(n)			(WT_SLP + 0x010 + 4 * ((n) - 1))
#define  WB_BG_ON(n)			(WT_SLP + 0x030 + 4 * ((n) - 1))
#define  WB_BG_OFF(n)			(WT_SLP + 0x050 + 4 * ((n) - 1))
#define  WB_WF_CK_ADDR			(WT_SLP + 0x070)
#define  WB_WF_WAKE_ADDR		(WT_SLP + 0x074)
#define  WB_WF_ZPS_ADDR			(WT_SLP + 0x078)
#define  WB_BT_CK_ADDR			(WT_SLP + 0x07c)
#define  WB_BT_WAKE_ADDR		(WT_SLP + 0x080)
#define  WB_TOP_CK_ADDR			(WT_SLP + 0x084)
#define  WB_GPS_CK_ADDR			(WT_SLP + 0x088)
#define  WB_WF_B0_CMD_ADDR		(WT_SLP + 0x08c)
#define  WB_WF_B1_CMD_ADDR		(WT_SLP + 0x090)
#define  WB_GPS_RFBUF_ADR		(WT_SLP + 0x094)
#define  WB_GPS_L5_EN_ADDR		(WT_SLP + 0x098)
#define  WB_SLP_TOP_CK_0		(WT_SLP + 0x120)
#define  WB_SLP_TOP_CK_1		(WT_SLP + 0x124)
#define   WB_SLP_TOP_CK_EN		BIT(0)
#define   WB_SLP_TOP_CK_BSY		BIT(1)

/* conn_infra_bus_cr_on */
#define BUS_CR_ON			0x0e000
#define  BUS_CR_ON_VON_BUS_TIMEOUT_CTRL	(BUS_CR_ON + 0x024)
#define  BUS_CR_ON_ON_BUS_TIMEOUT_CTRL	(BUS_CR_ON + 0x038)
#define  BUS_CR_ON_VON_BUS_DCM_CTL_1	(BUS_CR_ON + 0x104)
#define  BUS_CR_ON_OFF_BUS_DCM_CTL_1	(BUS_CR_ON + 0x110)
#define  BUS_CR_ON_CKSYS_CK_CTRL	(BUS_CR_ON + 0x120)

/* conn_infra_cfg */
#define CFG				0x11000
#define  CFG_IP_VERSION			(CFG + 0x000)
#define  CFG_EFUSE			(CFG + 0x020)
#define  CFG_CMDBT_FETCH_START_ADDR0	(CFG + 0x050)
#define  CFG_EMI_CTL_0			(CFG + 0x100)
#define  CFG_EMI_CTL_WF			(CFG + 0x114)

/* conn_infra_clkgen_top */
#define CLKGEN				0x12000
#define  CKGEN_BUS_BPLL_DIV_1		(CLKGEN + 0x000)
#define  CKGEN_BUS_BPLL_DIV_2		(CLKGEN + 0x004)
#define  CKGEN_BUS_WPLL_DIV_1		(CLKGEN + 0x008)
#define  CKGEN_BUS_WPLL_DIV_2		(CLKGEN + 0x00c)
#define  CKGEN_RFSPI_INST2_SW_MODE	(CLKGEN + 0x034)
#define  CKGEN_RFSPI_CK_CTRL		(CLKGEN + 0x038)
#define  CKGEN_BUS			(CLKGEN + 0x050)
#define  CKGEN_COEX_0_SET		(CLKGEN + 0x064)
#define  CKGEN_COEX_0_CLR		(CLKGEN + 0x068)
#define  CKGEN_COEX_1_SET		(CLKGEN + 0x074)
#define  CKGEN_COEX_1_CLR		(CLKGEN + 0x078)
/* PTA/UART-PTA HCLK and OSC clocks, BSI and FDD coex clocks */
#define  COEX_0_WF_PTA			(BIT(24) | BIT(16) | BIT(8) | BIT(0))
#define  COEX_1_WF_PTA			(BIT(8) | BIT(0))

/* conn_dbg_ctl: bus clock detector */
#define DBG_CTL_CLOCK_DETECT		0x23000

/* conn_therm_ctl, conn_afe_ctl */
#define THERM_AADDR			(0x40000 + 0x018)
#define AFE				0x41000
#define  AFE_RG_DIG_EN_01		(AFE + 0x000)
#define  AFE_RG_DIG_EN_02		(AFE + 0x004)
#define  AFE_RG_DIG_EN_03		(AFE + 0x008)
#define  AFE_RG_DIG_TOP_01		(AFE + 0x00c)
#define  AFE_RG_PLL_STB_TIME		(AFE + 0x0f4)
#define  AFE_RG_DIG_EN_04		(AFE + 0x15c)

/* conn_rf_spi_mst_reg: SPI master towards the A-die */
#define RF_SPI				0x42000
#define  RF_SPI_STA			(RF_SPI + 0x000)
#define  RF_SPI_FM_CTRL			(RF_SPI + 0x00c)
#define  RF_SPI_MCU_RECORD_CTL		(RF_SPI + 0x320)

/* conn_infra_bus_cr */
#define BUS_CR				0x4b000
#define  BUS_CR_OFF_BUS_TIMEOUT_CTRL	(BUS_CR + 0x024)
#define  BUS_CR_EMI_PATH_START		(BUS_CR + 0x070)
#define  BUS_CR_EMI_PATH_END		(BUS_CR + 0x074)
#define  BUS_CR_LIGHT_SECURITY_CTRL	(BUS_CR + 0x374)

/* conn_infra_off_debug_ctrl_ao */
#define OFF_DEBUG_CTRL_AO_CTRL0		0x4d000

/* conn_infra_sysram and the software-defined words at its end */
#define SYSRAM				0x50000
#define  SYSRAM_SIZE			(20 * 1024)
#define  SYSRAM_CMDBT_FETCH_START	0x18050200
#define SYSRAM_SW_CR			0x53800
#define  SW_CR_A_DIE_CHIP_ID		(SYSRAM_SW_CR + 0x000)
#define  SW_CR_A_DIE_EFUSE(n)		(SYSRAM_SW_CR + 0x004 + 4 * (n))
#define  SW_CR_D_DIE_EFUSE		(SYSRAM_SW_CR + 0x020)
#define  SW_CR_RADIO_STATUS		(SYSRAM_SW_CR + 0x034)
#define  SW_CR_CLOCK_TYPE		(SYSRAM_SW_CR + 0x03c)

/* AP2WF CCIF (WF to AP software interrupt and its acknowledge) */
#define CCIF4_AP2WF_PCCIF_ACK		(0x3d000 + 0x014)

/* conn_host_csr_top */
#define HOST_CSR			0x60000
#define  HOST_CONN_INFRA_WAKEPU_TOP	(HOST_CSR + 0x1a0)
#define  HOST_CONN_INFRA_WAKEPU_WF	(HOST_CSR + 0x1a4)
#define  HOST_CONN2AP_REMAP_MCU_EMI	(HOST_CSR + 0x354)
#define   CONN2AP_REMAP_BASE		GENMASK(19, 0)	/* address >> 16 */
#define  HOST_CONN_INFRA_SLP_CNT_CTL	(HOST_CSR + 0x380)
#define  HOST_CONNSYS_PWR_STATES	(HOST_CSR + 0xa10)
#define   PWR_STATES_WFSYS_RGU_OFF_RST_B BIT(30)
#define  HOST_WF_ON_MONFLG_EN		(HOST_CSR + 0xb00)
#define  HOST_WF_ON_MONFLG_SEL		(HOST_CSR + 0xb04)
#define  HOST_WF_ON_MONFLG_OUT		(HOST_CSR + 0xb10)
#define   WF_ROM_IDLE			0x1d1e

/* conn_semaphore, as master M2 (the AP) */
#define SEMA_M2_OWN_STA(n)		(0x70000 + 0x2000 + 4 * (n))
#define SEMA_M2_OWN_REL(n)		(0x70000 + 0x2200 + 4 * (n))
#define SEMA_RFSPI			12
#define SEMA_COMMON_SYSRAM		23
#define SEMA_TIMEOUT_US			1000

/* WFSYS registers, offsets from 0x1840_0000 */
#define WF_MCU_BUS_CR_AP2WF_REMAP_1	0x000120
#define WF_TOP_CFG_IP_VERSION		0x0b0010
#define WF_TOP_CFG_ON_ROMCODE_INDEX	0x0c1604
#define WF_TOP_SLPPROT_ON_STATUS	0x0c300c
#define  WF_SLPPROT_EN_SOURCE_1		BIT(23)	/* WFDMA to CONN_INFRA, AXI TX */
#define  WF_SLPPROT_EN_SOURCE_2		BIT(21)	/* WFDMA to CONN_INFRA, AXI RX */
#define WF_MCU_CONFG_LS_BUSHANGCR	0x0f0440
#define  BUSHANG_DEBUG_EN		BIT(31)
#define  BUSHANG_DEBUG_CG_EN		BIT(28)
#define  BUSHANG_TIME_LIMIT		GENMASK(7, 0)
/* remap window 0 at 0x1850_0000, pointed at the WF MCU debug_ctrl_ao */
#define WF_REMAP0			0x100000
#define  WF_DEBUG_CTRL_AO_BASE		0x810f0000
#define  WF_PWA_CTRL0			(WF_REMAP0 + 0x000)
#define   PWA_TIMEOUT_THRES		GENMASK(31, 16)
#define   PWA_TIMEOUT_CLR		BIT(9)
#define   PWA_DEBUG_EN_DEBUGTOP		BIT(4)
#define   PWA_DEBUG_CKEN		BIT(3)
#define   PWA_DEBUG_EN			BIT(2)
#define  WF_PWA_CTRL3			(WF_REMAP0 + 0x00c)

/* MT6637 A-die registers (RF SPI, "TOP" subsystem) */
#define ADIE6637_CHIP_ID		0x02c
#define  ADIE6637_ID			0x6637
#define  ADIE6637_HW_VER_E1		0x8a00
#define  ADIE6637_HW_VER_E2		0x8a01
#define ADIE6637_RG_TOP_THADC_BG	0x034
#define ADIE6637_RG_TOP_THADC		0x038
#define ADIE6637_WRI_CTR2		0x064
#define ADIE6637_RG_ENCAL_WBTAC_IF_SW	0x070
#define ADIE6637_SMCTK11		0x0bc
#define ADIE6637_EFUSE_CTRL		0x108
#define  EFUSE_CTRL_KICK		BIT(30)
#define  EFUSE_CTRL_VALID		BIT(29)
#define  EFUSE_CTRL_ADDR		GENMASK(25, 16)
#define ADIE6637_EFUSE_RDATA(n)		(0x130 + 4 * (n))
#define ADIE6637_RG_WF0_BG		0x344
#define ADIE6637_RG_WF1_BG		0x374
#define ADIE6637_RG_TOP_XO_02		0xb04
#define ADIE6637_RG_TOP_XO_03		0xb08
#define ADIE6637_RG_TOP_XO_07		0xb18

/* CONNSYS SiP service of the stock ATF: EMI remapping (CONN2AP) */
#define MTK_SIP_KERNEL_CONNSYS_CONTROL	MTK_SIP_SMC_CMD(0x534)
#define  CONNSYS_SMC_EMI_REMAP		70

/* SRCLKEN_RC: is the PMIC/clock request controller in RC mode? */
#define SRCLKEN_RC_CENTRAL_CFG1		0x004
#define  SRCLKEN_RC_EN			BIT(0)

/* TOPCKGEN: keep the host clock request of CONN_VON_TOP out of "always on" */
#define TOPCKGEN_CONN_CK_CTRL		0x504
#define  CONN_VON_TOP_HOST_CK_REQ_EN	BIT(3)

#define CONNINFRA_IP_VER_POLL_US	1000
#define CONNINFRA_IP_VER_TIMEOUT_US	11000

#define MT6878_WF_IP_VERSION		0x02040300

enum mtk_conninfra_adie {
	MTK_ADIE_MT6637,
	MTK_ADIE_MT6631,
	MTK_ADIE_MT6631_MT6686,
};

/* RF SPI channels (only "TOP" is used for the MT6637 sequences) */
enum mtk_conninfra_spi {
	SPI_WF1, SPI_WF, SPI_BT, SPI_FM, SPI_GPS, SPI_TOP, SPI_WF2, SPI_WF3,
};

struct mtk_conninfra_spi_op {
	u8 busy_bit;
	u16 addr_reg;
	u16 read_fmt;
	u16 write_fmt;
	u16 wdat_reg;
	u16 rdat_reg;
	u32 rdat_mask;
};

static const struct mtk_conninfra_spi_op mtk_conninfra_spi_ops[] = {
	[SPI_WF1] = { 1, 0x010, 0x1000, 0x0000, 0x014, 0x018, 0xffffffff },
	[SPI_WF]  = { 1, 0x010, 0x3000, 0x2000, 0x014, 0x018, 0xffffffff },
	[SPI_BT]  = { 2, 0x020, 0x5000, 0x4000, 0x024, 0x028, 0x000000ff },
	[SPI_FM]  = { 3, 0x030, 0x7000, 0x6000, 0x034, 0x038, 0x0000ffff },
	[SPI_GPS] = { 4, 0x210, 0x9000, 0x8000, 0x214, 0x218, 0x0000ffff },
	[SPI_TOP] = { 5, 0x050, 0xb000, 0xa000, 0x054, 0x058, 0xffffffff },
	[SPI_WF2] = { 1, 0x010, 0xd000, 0xc000, 0x014, 0x018, 0xffffffff },
	[SPI_WF3] = { 1, 0x010, 0xf000, 0xe000, 0x014, 0x018, 0xffffffff },
};

struct mtk_conninfra_data {
	u32 ip_version;
	enum mtk_conninfra_adie adie;
	const char *adie_name;
	bool full_power_on;	/* the POS below is implemented for this A-die */
};

static const char * const mtk_conninfra_supplies[] = {
	"vcn13",	/* D-die AFE/WBG, A-die digital */
	"vrfio18",	/* A-die I/O, crystal */
};

struct mtk_conninfra {
	struct device *dev;
	void __iomem *base;		/* CONN_INFRA, 0x1800_0000 */
	void __iomem *wfsys;		/* WFSYS, 0x1840_0000 (optional) */
	bool rc_mode;
	struct regmap *topckgen;
	struct regulator_bulk_data supplies[ARRAY_SIZE(mtk_conninfra_supplies)];
	struct regulator *vcn33_1;	/* Wi-Fi and BT front end */
	struct regulator *vcn33_2;	/* Wi-Fi front end */
	struct regulator *vant18;	/* GNSS LNA */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_idle;
	struct pinctrl_state *pins_adie_cfg;
	const struct mtk_conninfra_data *data;
	struct dentry *debugfs;

	phys_addr_t emi_base;		/* from LK ("emi-addr"), 0 if absent */
	u32 emi_size;
	bool emi_remapped;
	bool co_clock_52m;

	struct mutex lock;		/* protects everything below */
	bool hw_on;			/* rails, CONN domain and pins on */
	unsigned long radios;		/* BIT(enum mtk_conninfra_radio) */
	unsigned int wake_cnt;
	bool wfsys_on;
	u32 adie_id;
	u32 adie_efuse[4];
	u32 d_die_efuse;

	spinlock_t spi_lock;		/* RF SPI semaphore holder, IRQs off */
	unsigned long spi_flags;
};

static bool allow_no_emi;
module_param(allow_no_emi, bool, 0644);
MODULE_PARM_DESC(allow_no_emi,
		 "Start WFSYS even if the CONNSYS EMI is not known to be remapped (unsafe)");

/* ------------------------------------------------------------------ */
/* register helpers                                                    */

static u32 ci_rd(struct mtk_conninfra *ci, u32 off)
{
	return readl(ci->base + off);
}

static void ci_wr(struct mtk_conninfra *ci, u32 off, u32 val)
{
	writel(val, ci->base + off);
}

static void ci_rmw(struct mtk_conninfra *ci, u32 off, u32 mask, u32 val)
{
	u32 v = readl(ci->base + off);

	writel((v & ~mask) | (val & mask), ci->base + off);
}

static void ci_set(struct mtk_conninfra *ci, u32 off, u32 bits)
{
	ci_rmw(ci, off, bits, bits);
}

static void ci_clr(struct mtk_conninfra *ci, u32 off, u32 bits)
{
	ci_rmw(ci, off, bits, 0);
}

static int ci_poll(struct mtk_conninfra *ci, u32 off, u32 mask, u32 want,
		   unsigned int delay_us, unsigned int timeout_us)
{
	u32 v;

	return readl_poll_timeout_atomic(ci->base + off, v, (v & mask) == want,
					 delay_us, timeout_us);
}

static u32 wf_rd(struct mtk_conninfra *ci, u32 off)
{
	return readl(ci->wfsys + off);
}

static void wf_wr(struct mtk_conninfra *ci, u32 off, u32 val)
{
	writel(val, ci->wfsys + off);
}

static void wf_rmw(struct mtk_conninfra *ci, u32 off, u32 mask, u32 val)
{
	u32 v = readl(ci->wfsys + off);

	writel((v & ~mask) | (val & mask), ci->wfsys + off);
}

/* ------------------------------------------------------------------ */
/* hardware semaphores and the RF SPI master                           */

static int mtk_conninfra_sema_get(struct mtk_conninfra *ci, unsigned int idx)
{
	unsigned int i;

	/* reading the M2 status register takes the semaphore if it is free */
	for (i = 0; i < SEMA_TIMEOUT_US; i++) {
		if (ci_rd(ci, SEMA_M2_OWN_STA(idx)) & BIT(0))
			return 0;
		udelay(1);
	}

	dev_err(ci->dev, "timeout on hardware semaphore %u\n", idx);
	return -ETIMEDOUT;
}

static void mtk_conninfra_sema_put(struct mtk_conninfra *ci, unsigned int idx)
{
	ci_wr(ci, SEMA_M2_OWN_REL(idx), BIT(0));
}

/*
 * The firmware of the radio MCUs shares the RF SPI master; like the MediaTek downstream
 * driver, hold its semaphore with interrupts off and keep it short.
 */
static int mtk_conninfra_spi_lock(struct mtk_conninfra *ci)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ci->spi_lock, flags);
	ret = mtk_conninfra_sema_get(ci, SEMA_RFSPI);
	if (ret) {
		spin_unlock_irqrestore(&ci->spi_lock, flags);
		return ret;
	}
	ci->spi_flags = flags;
	return 0;
}

static void mtk_conninfra_spi_unlock(struct mtk_conninfra *ci)
{
	mtk_conninfra_sema_put(ci, SEMA_RFSPI);
	spin_unlock_irqrestore(&ci->spi_lock, ci->spi_flags);
}

static int mtk_conninfra_spi_wait(struct mtk_conninfra *ci,
				  const struct mtk_conninfra_spi_op *op)
{
	/* stock: 100 polls, 50 us apart */
	return ci_poll(ci, RF_SPI_STA, BIT(op->busy_bit), 0, 50, 5000);
}

/* caller holds the RF SPI semaphore */
static int mtk_conninfra_spi_read(struct mtk_conninfra *ci,
				  enum mtk_conninfra_spi sys, u32 addr, u32 *val)
{
	const struct mtk_conninfra_spi_op *op = &mtk_conninfra_spi_ops[sys];
	int ret;

	ret = mtk_conninfra_spi_wait(ci, op);
	if (ret)
		goto err;
	ci_wr(ci, RF_SPI + op->addr_reg, op->read_fmt | addr);
	ci_wr(ci, RF_SPI + op->wdat_reg, 0);
	udelay(1);
	ret = mtk_conninfra_spi_wait(ci, op);
	if (ret)
		goto err;
	*val = ci_rd(ci, RF_SPI + op->rdat_reg) & op->rdat_mask;
	return 0;

err:
	dev_err(ci->dev, "RF SPI read 0x%03x timed out (status 0x%08x)\n",
		addr, ci_rd(ci, RF_SPI_STA));
	return ret;
}

static int mtk_conninfra_spi_write(struct mtk_conninfra *ci,
				   enum mtk_conninfra_spi sys, u32 addr, u32 val)
{
	const struct mtk_conninfra_spi_op *op = &mtk_conninfra_spi_ops[sys];
	int ret;

	ret = mtk_conninfra_spi_wait(ci, op);
	if (ret)
		goto err;
	ci_wr(ci, RF_SPI + op->addr_reg, op->write_fmt | addr);
	ci_wr(ci, RF_SPI + op->wdat_reg, val);
	udelay(1);
	ret = mtk_conninfra_spi_wait(ci, op);
	if (ret)
		goto err;
	return 0;

err:
	dev_err(ci->dev, "RF SPI write 0x%03x timed out (status 0x%08x)\n",
		addr, ci_rd(ci, RF_SPI_STA));
	return ret;
}

/* replace @reg_bits of an A-die register with bits of @val starting at @val_lsb */
static void mtk_conninfra_spi_copy_bits(struct mtk_conninfra *ci, u32 addr,
					u32 val, unsigned int reg_lsb,
					unsigned int val_lsb, unsigned int width)
{
	u32 mask = GENMASK(reg_lsb + width - 1, reg_lsb);
	u32 data;

	if (mtk_conninfra_spi_read(ci, SPI_TOP, addr, &data))
		return;
	data &= ~mask;
	data |= ((val >> val_lsb) << reg_lsb) & mask;
	mtk_conninfra_spi_write(ci, SPI_TOP, addr, data);
}

/* ------------------------------------------------------------------ */
/* low level helpers of the power-on sequence (MT6878 + MT6637)        */

static bool mtk_conninfra_bus_clock_ok(struct mtk_conninfra *ci)
{
	/* bit 1: conn_infra off bus clock, bit 2: OSC clock */
	ci_set(ci, DBG_CTL_CLOCK_DETECT, BIT(0));
	udelay(20);
	return (ci_rd(ci, DBG_CTL_CLOCK_DETECT) & GENMASK(2, 1)) == GENMASK(2, 1);
}

static int mtk_conninfra_poll_ip_version(struct mtk_conninfra *ci, bool atomic)
{
	u32 ver;
	int ret;

	if (atomic)
		ret = readl_poll_timeout_atomic(ci->base + CFG_IP_VERSION, ver,
						ver == ci->data->ip_version,
						CONNINFRA_IP_VER_POLL_US,
						CONNINFRA_IP_VER_TIMEOUT_US);
	else
		ret = readl_poll_timeout(ci->base + CFG_IP_VERSION, ver,
					 ver == ci->data->ip_version,
					 CONNINFRA_IP_VER_POLL_US,
					 CONNINFRA_IP_VER_TIMEOUT_US);
	if (ret)
		dev_err(ci->dev, "CONN_INFRA IP version 0x%08x, expected 0x%08x\n",
			ver, ci->data->ip_version);
	return ret ? -ENODEV : 0;
}

static int mtk_conninfra_adie_top_ck(struct mtk_conninfra *ci, bool on)
{
	int ret;

	if (on)
		ci_set(ci, WB_SLP_TOP_CK_0, WB_SLP_TOP_CK_EN);
	else
		ci_clr(ci, WB_SLP_TOP_CK_0, WB_SLP_TOP_CK_EN);

	/* the busy bit drops once the A-die has taken the request */
	ret = ci_poll(ci, WB_SLP_TOP_CK_0, WB_SLP_TOP_CK_BSY, 0, 5, 500);
	if (ret)
		dev_err(ci->dev, "A-die top_ck_en_0 %s timed out (0x%08x)\n",
			on ? "on" : "off", ci_rd(ci, WB_SLP_TOP_CK_0));
	return ret;
}

/* D-die: shared SRAM, EMI window, memory power control */
static void mtk_conninfra_d_die_cfg(struct mtk_conninfra *ci)
{
	/* conn_von_top_host_ck_req to HW mode */
	ci_rmw(ci, BUS_CR_ON_CKSYS_CK_CTRL, 0x3, 0x3);
	memset_io(ci->base + SYSRAM, 0, SYSRAM_SIZE);
	/* CONN2AP EMI path: the 64 MB CONNSYS view of EMI at 0x0400_0000 */
	ci_wr(ci, BUS_CR_EMI_PATH_START, 0x04000000);
	ci_wr(ci, BUS_CR_EMI_PATH_END, 0x43ffffff);

	ci->d_die_efuse = ci_rd(ci, CFG_EFUSE);
	ci_wr(ci, SW_CR_D_DIE_EFUSE, ci->d_die_efuse);

	/* sysram and co-ext memory: no HW power down, HW sleep on */
	ci_wr(ci, RGU_SYSRAM_HWCTL_PDN, 0x0);
	ci_wr(ci, RGU_SYSRAM_HWCTL_SLP, 0x3);
	ci_wr(ci, RGU_CO_EXT_MEM_HWCTL_PDN, 0x0);
	ci_wr(ci, RGU_CO_EXT_MEM_HWCTL_SLP, 0x1);
	/* do not mask the MTCMOS memory ack (avoids fake acks); key 0x494e */
	ci_rmw(ci, RGU_CONN_INFRA_OFF_TOP_PWR_CTL, 0xffff0040, 0x494e0000);
}

/* where the A-die sleep controller finds the ck_en/wake_en registers */
static void mtk_conninfra_spi_master_cfg_6637(struct mtk_conninfra *ci)
{
	ci_rmw(ci, WB_WF_CK_ADDR, 0x0fff0fff, 0x0af40a04);
	ci_rmw(ci, WB_WF_WAKE_ADDR, 0x0fff0fff, 0x00a00090);
	ci_rmw(ci, WB_WF_ZPS_ADDR, 0x0fff0fff, 0x009c008c);
	ci_rmw(ci, WB_BT_CK_ADDR, 0xfff, 0xa08);
	ci_rmw(ci, WB_BT_WAKE_ADDR, 0xfff, 0x094);
	ci_rmw(ci, WB_TOP_CK_ADDR, 0xfff, 0xa00);
	ci_rmw(ci, WB_GPS_CK_ADDR, 0x0fff0fff, 0x0afc0a0c);
	ci_rmw(ci, WB_WF_B0_CMD_ADDR, 0xfff, 0x0f0);
	ci_rmw(ci, WB_WF_B1_CMD_ADDR, 0xfff, 0x0f4);
	ci_rmw(ci, WB_GPS_RFBUF_ADR, 0xfff, 0x0fc);
	ci_rmw(ci, WB_GPS_L5_EN_ADDR, 0xfff, 0x0f8);
}

/* A-die power saving through the sleep controller (sleep modes 1 and 3) */
static void mtk_conninfra_adie_power_saving_6637(struct mtk_conninfra *ci)
{
	static const u32 bg_addr[] = { 0xa03c, 0xa03c, 0xab00, 0xab00, 0xab00, 0xa0c8 };
	static const u32 bg_on[] = { 0x0, 0x0, 0xc0000000, 0xf8000000, 0xfc000000, 0x0 };
	static const u32 bg_off[] = { 0x57400000, 0x57400000, 0x0, 0x0, 0xc4000000, 0x1 };
	unsigned int i;

	ci_rmw(ci, WB_SLP_CTL, 0x1f, 0x6);
	for (i = 0; i < ARRAY_SIZE(bg_addr); i++) {
		ci_wr(ci, WB_BG_ADDR(i + 1), bg_addr[i]);
		ci_wr(ci, WB_BG_ON(i + 1), bg_on[i]);
		ci_wr(ci, WB_BG_OFF(i + 1), bg_off[i]);
	}
}

static int mtk_conninfra_adie_pins(struct mtk_conninfra *ci, bool adie_cfg)
{
	struct pinctrl_state *st = adie_cfg ? ci->pins_adie_cfg : ci->pins_active;

	if (IS_ERR_OR_NULL(ci->pinctrl) || IS_ERR_OR_NULL(st))
		return 0;
	return pinctrl_select_state(ci->pinctrl, st);
}

/* MT6637 efuse block at @addr: kick a read, return whether it is valid */
static bool mtk_conninfra_adie_efuse_read(struct mtk_conninfra *ci, u32 addr)
{
	u32 ctrl = 0;
	unsigned int i;

	mtk_conninfra_spi_read(ci, SPI_TOP, ADIE6637_EFUSE_CTRL, &ctrl);
	ctrl &= ~EFUSE_CTRL_KICK;
	mtk_conninfra_spi_write(ci, SPI_TOP, ADIE6637_EFUSE_CTRL, ctrl);

	mtk_conninfra_spi_read(ci, SPI_TOP, ADIE6637_EFUSE_CTRL, &ctrl);
	ctrl &= ~(EFUSE_CTRL_KICK | EFUSE_CTRL_ADDR | GENMASK(7, 6));
	ctrl |= EFUSE_CTRL_KICK | FIELD_PREP(EFUSE_CTRL_ADDR, addr);
	mtk_conninfra_spi_write(ci, SPI_TOP, ADIE6637_EFUSE_CTRL, ctrl);

	/* busy (= the kick bit) drops within 2 ms */
	for (i = 0; i < 70; i++) {
		mtk_conninfra_spi_read(ci, SPI_TOP, ADIE6637_EFUSE_CTRL, &ctrl);
		if (!(ctrl & EFUSE_CTRL_KICK))
			break;
		udelay(30);
	}
	if (ctrl & EFUSE_CTRL_KICK)
		dev_warn(ci->dev, "A-die efuse 0x%x read still busy\n", addr);

	mtk_conninfra_spi_read(ci, SPI_TOP, ADIE6637_EFUSE_CTRL, &ctrl);
	return ctrl & EFUSE_CTRL_VALID;
}

static void mtk_conninfra_adie_efuse_6637(struct mtk_conninfra *ci)
{
	u32 *e = ci->adie_efuse;
	unsigned int i;

	/* WF0/WF1 SX calibration map, DBDC low power commands, FM read bit */
	mtk_conninfra_spi_write(ci, SPI_TOP, ADIE6637_WRI_CTR2, 0x7700);
	mtk_conninfra_spi_write(ci, SPI_TOP, ADIE6637_SMCTK11, 0x21);
	ci_rmw(ci, RF_SPI_FM_CTRL, 0x80ff, 0x0);
	/* where the thermal sensor of the MT6637 lives */
	ci_wr(ci, THERM_AADDR, 0x50305a00);

	memset(ci->adie_efuse, 0, sizeof(ci->adie_efuse));
	if (!mtk_conninfra_adie_efuse_read(ci, 0x0)) {
		dev_warn(ci->dev, "A-die efuse is not valid\n");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(ci->adie_efuse); i++) {
		mtk_conninfra_spi_read(ci, SPI_TOP, ADIE6637_EFUSE_RDATA(i), &e[i]);
		/* the firmware reads its copy from the shared SRAM */
		ci_wr(ci, SW_CR_A_DIE_EFUSE(i), e[i]);
	}

	/* thermal ADC trimming, when burnt */
	if (e[1] & BIT(7)) {
		mtk_conninfra_spi_copy_bits(ci, ADIE6637_RG_TOP_THADC_BG, e[1], 12, 3, 4);
		mtk_conninfra_spi_copy_bits(ci, ADIE6637_RG_TOP_THADC, e[1], 23, 0, 3);
	}
	if (e[1] & BIT(15))
		mtk_conninfra_spi_copy_bits(ci, ADIE6637_RG_TOP_THADC, e[1], 26, 13, 2);
}

/* MT6637 default value updates ("PART2" of the stock sequence) */
static void mtk_conninfra_adie_defaults_6637(struct mtk_conninfra *ci, u32 hw_ver)
{
	static const struct {
		u16 addr;
		u32 val;
	} tbl[] = {
		{ 0x380, 0x000e8002 },	/* WF2_POS_01: CAL LDO off */
		{ 0x390, 0x000e8002 },	/* WF2_POS_02: CAL LDO off */
		{ 0x384, 0x00002008 },	/* WF2_TIME_COUNT_01: WF0 THADC IR */
		{ 0x394, 0x00002008 },	/* WF2_TIME_COUNT_02: WF1 THADC IR */
		{ 0x358, 0x000f000b },	/* WF1_RFLDO_TIME */
		{ 0xdf8, 0x000c000c },	/* WF1_RFDIG_OFF_TIME */
		{ 0x304, 0x00020003 },	/* BT0_BG_TIME */
		{ 0x308, 0x0001000b },	/* BT0_ISO_TIME */
		{ 0x430, 0x00010004 },	/* BT0_XOBUF_TIME */
		{ 0x438, 0x00020003 },	/* BT0_CKEN_TIME */
		{ 0xffc, 0x00150008 },	/* WAKEUPSLEEP_PON_OFF: 8 us */
		{ 0xb04, 0x88000000 },	/* TOP_XO_02: WF XO buffer off */
		{ 0xb08, 0x00000000 },	/* TOP_XO_03 */
		{ 0xb18, 0x00000000 },	/* TOP_XO_07: BT/FM/GNSS XO buffer off */
		{ 0x070, 0x00000005 },	/* ENCAL_WFBT_IF_SW_01: BT AIQ normal */
		{ 0x160, 0x00000035 },	/* FMCTL4: FMSYS always on */
		{ 0x00c, 0xa6aaaa44 },	/* TOP_PULL: no pulls on the SPI clocks */
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tbl); i++)
		mtk_conninfra_spi_write(ci, SPI_TOP, tbl[i].addr, tbl[i].val);

	if (hw_ver == ADIE6637_HW_VER_E2) {
		mtk_conninfra_spi_write(ci, SPI_TOP, ADIE6637_RG_WF0_BG, 0xe008);
		mtk_conninfra_spi_write(ci, SPI_TOP, ADIE6637_RG_WF1_BG, 0xe008);
	}

	/* WF0/WF1 CAL/TRX LDO bleeding current */
	mtk_conninfra_spi_write(ci, SPI_TOP, 0x754, 0x000e440a);
	mtk_conninfra_spi_write(ci, SPI_TOP, 0x758, 0x000e401a);
}

static int mtk_conninfra_a_die_cfg_6637(struct mtk_conninfra *ci)
{
	u32 id = 0;
	int ret;

	/* FW only cares about 26 MHz (1) or 52 MHz (2) co-clock */
	ci_wr(ci, SW_CR_CLOCK_TYPE, ci->co_clock_52m ? 2 : 1);

	/*
	 * CONN_WF_CTRL2 (GPIO180) as a GPIO while the A-die comes out of
	 * reset: low for a 26 MHz, high for a 52 MHz co-clock.
	 */
	ret = mtk_conninfra_adie_pins(ci, true);
	if (ret)
		dev_warn(ci->dev, "failed to select adie-cfg pins: %d\n", ret);

	ret = mtk_conninfra_adie_top_ck(ci, true);
	if (ret)
		goto out_pins;

	ret = mtk_conninfra_spi_lock(ci);
	if (ret)
		goto out_ck;

	ci_set(ci, CFG_ON_ADIE_CTL, BIT(0));	/* A-die reset de-asserted */

	ret = mtk_conninfra_spi_read(ci, SPI_TOP, ADIE6637_CHIP_ID, &id);
	if (!ret && (id >> 16) != ADIE6637_ID)
		ret = -ENODEV;
	if (ret) {
		mtk_conninfra_spi_unlock(ci);
		dev_err(ci->dev, "no MT6637 A-die (chip id 0x%08x): %d\n", id, ret);
		goto out_ck;
	}
	ci->adie_id = id;
	ci_wr(ci, SW_CR_A_DIE_CHIP_ID, id);

	mtk_conninfra_adie_efuse_6637(ci);
	mtk_conninfra_adie_defaults_6637(ci, id & 0xffff);
	mtk_conninfra_spi_unlock(ci);

	dev_info(ci->dev, "A-die MT6637 chip id 0x%08x, efuse %08x %08x %08x %08x\n",
		 id, ci->adie_efuse[0], ci->adie_efuse[1], ci->adie_efuse[2],
		 ci->adie_efuse[3]);

	mtk_conninfra_adie_pins(ci, false);
	mtk_conninfra_adie_power_saving_6637(ci);
	return mtk_conninfra_adie_top_ck(ci, false);

out_ck:
	mtk_conninfra_adie_top_ck(ci, false);
out_pins:
	mtk_conninfra_adie_pins(ci, false);
	return ret;
}

/* D-die AFE (WBG): default values and the one-time RC/TX calibration */
static int mtk_conninfra_afe_cfg(struct mtk_conninfra *ci, bool calibrate)
{
	int ret;

	ci_clr(ci, AFE_RG_DIG_EN_04, BIT(31));
	ci_rmw(ci, AFE_RG_DIG_EN_03, 0x7700, 0x4400);

	if (!calibrate)
		return 0;

	/* WF_PAD to high-Z during the calibration */
	ret = mtk_conninfra_spi_lock(ci);
	if (ret)
		return ret;
	mtk_conninfra_spi_write(ci, SPI_TOP, ADIE6637_RG_ENCAL_WBTAC_IF_SW, 0x80000000);
	mtk_conninfra_spi_unlock(ci);

	ci_set(ci, AFE_RG_DIG_EN_01, BIT(0));		/* RG_WBG_EN_RCK */
	udelay(60);
	ci_clr(ci, AFE_RG_DIG_EN_01, BIT(0));
	ci_set(ci, AFE_RG_DIG_EN_03, BIT(21));		/* RG_WBG_EN_BPLL_UP */
	udelay(30);
	ci_set(ci, AFE_RG_DIG_EN_03, BIT(20));		/* RG_WBG_EN_WPLL_UP */
	udelay(60);
	/* RG_WBG_EN_TXCAL_BT, _WF0, _WF1 */
	ci_set(ci, AFE_RG_DIG_EN_01, BIT(21));
	ci_set(ci, AFE_RG_DIG_EN_01, BIT(20));
	ci_set(ci, AFE_RG_DIG_EN_01, BIT(19));
	udelay(800);
	ci_clr(ci, AFE_RG_DIG_EN_01, BIT(21));
	ci_clr(ci, AFE_RG_DIG_EN_01, BIT(20));
	ci_clr(ci, AFE_RG_DIG_EN_01, BIT(19));
	ci_clr(ci, AFE_RG_DIG_EN_03, BIT(21));
	ci_clr(ci, AFE_RG_DIG_EN_03, BIT(20));

	ret = mtk_conninfra_spi_lock(ci);
	if (ret)
		return ret;
	mtk_conninfra_spi_write(ci, SPI_TOP, ADIE6637_RG_ENCAL_WBTAC_IF_SW, 0x5);
	mtk_conninfra_spi_unlock(ci);
	return 0;
}

static void mtk_conninfra_pll_init(struct mtk_conninfra *ci)
{
	/* 26 MHz crystal */
	ci_rmw(ci, AFE_RG_PLL_STB_TIME, 0x7fff7fff, 0x03140521);
	ci_rmw(ci, AFE_RG_DIG_EN_02, 0x000300cf, 0x00030047);
	ci_rmw(ci, AFE_RG_DIG_TOP_01, 0x00078000, 0x00068000);
}

static int mtk_conninfra_radio_status(struct mtk_conninfra *ci,
				      enum mtk_conninfra_radio radio, bool on)
{
	int ret;

	ret = mtk_conninfra_sema_get(ci, SEMA_COMMON_SYSRAM);
	if (ret)
		return ret;
	if (on)
		ci_set(ci, SW_CR_RADIO_STATUS, BIT(radio));
	else
		ci_clr(ci, SW_CR_RADIO_STATUS, BIT(radio));
	mtk_conninfra_sema_put(ci, SEMA_COMMON_SYSRAM);
	return 0;
}

/* XO buffers of the MT6637 for the radios that change state */
static int mtk_conninfra_adie_xo_buffers(struct mtk_conninfra *ci,
					 unsigned long cur, unsigned long next)
{
	const unsigned long bgf = BIT(MTK_CONNINFRA_BT) | BIT(MTK_CONNINFRA_GPS) |
				  BIT(MTK_CONNINFRA_FM);
	unsigned long changed = cur ^ next;
	int ret;

	ret = mtk_conninfra_adie_top_ck(ci, true);
	if (ret)
		return ret;
	ret = mtk_conninfra_spi_lock(ci);
	if (ret)
		goto out;

	if (changed & BIT(MTK_CONNINFRA_WIFI)) {
		bool on = next & BIT(MTK_CONNINFRA_WIFI);

		mtk_conninfra_spi_write(ci, SPI_TOP, ADIE6637_RG_TOP_XO_02,
					on ? 0xfe000000 : 0x88000000);
		mtk_conninfra_spi_write(ci, SPI_TOP, ADIE6637_RG_TOP_XO_03,
					on ? 0xe0000000 : 0x0);
	}
	if ((changed & next) & bgf)
		mtk_conninfra_spi_write(ci, SPI_TOP, ADIE6637_RG_TOP_XO_07, 0x7);
	if ((cur & bgf) && !(next & bgf))
		mtk_conninfra_spi_write(ci, SPI_TOP, ADIE6637_RG_TOP_XO_07, 0x0);

	mtk_conninfra_spi_unlock(ci);
out:
	mtk_conninfra_adie_top_ck(ci, false);
	return ret;
}

/*
 * CONN_INFRA low power setup, last step of the first power-on: OSC/RC
 * control, bus DCM and timeouts, the command batch the hardware replays
 * when it restores the "off" domain, and finally HW controlled sleep.
 * From here on the AP must wake CONN_INFRA up before touching it.
 */
static void mtk_conninfra_low_power_cfg(struct mtk_conninfra *ci)
{
	unsigned int i;

	/* unmask the off->on slpprot_rdy / slpprot_rdy_dis checkers */
	ci_rmw(ci, CFG_ON_PWRCTRL0, 0xf000, 0x1000);
	ci_rmw(ci, CFG_ON_PWRCTRL0, 0xf0000, 0xd0000);

	if (!ci->rc_mode) {
		ci_clr(ci, CFG_ON_RC_CTL_0, BIT(7));
		ci_rmw(ci, CFG_ON_OSC_CTL_0, 0xffffff, 0x80706);
		ci_clr(ci, CFG_ON_OSC_CTL_1, BIT(16));
	} else {
		static const u32 rc[][2] = {
			{ CFG_ON_RC_CTL_1_GPS, CFG_ON_RC_CTL_0_GPS },
			{ CFG_ON_RC_CTL_1_BT, CFG_ON_RC_CTL_0_BT },
			{ CFG_ON_RC_CTL_1_WF, CFG_ON_RC_CTL_0_WF },
			{ CFG_ON_RC_CTL_1_TOP, CFG_ON_RC_CTL_0_TOP },
		};

		/* RC OSC stable time and conn_srcclkena_ack per requester */
		for (i = 0; i < ARRAY_SIZE(rc); i++) {
			ci_wr(ci, rc[i][0], 0x2080706);
			ci_clr(ci, rc[i][1], BIT(15));
		}
		ci_set(ci, CFG_ON_RC_CTL_1, BIT(3));	/* osc_ctl_top output */
		udelay(30);
		ci_set(ci, CFG_ON_RC_CTL_1, BIT(0));	/* gps output */
		ci_set(ci, CFG_ON_RC_CTL_1, BIT(1));	/* bt output */
		ci_set(ci, CFG_ON_RC_CTL_1, BIT(2));	/* wf output */
		ci_set(ci, CFG_ON_RC_CTL_0, BIT(4));	/* gps */
		ci_set(ci, CFG_ON_RC_CTL_0, BIT(5));	/* bt */
		ci_set(ci, CFG_ON_RC_CTL_0, BIT(6));	/* wf */
		ci_set(ci, CFG_ON_RC_CTL_1, BIT(20));	/* srcclkena by emi_ctl */
		ci_clr(ci, CFG_ON_RC_CTL_0, BIT(31));	/* legacy osc output off */
		ci_clr(ci, CFG_ON_RC_CTL_0, BIT(0));	/* legacy osc ctl off */
		ci_rmw(ci, CFG_ON_OSC_CTL_0, 0xffffff, 0x80706);
		ci_clr(ci, CFG_ON_OSC_CTL_1, BIT(16));
		ci_set(ci, CFG_ON_RC_CTL_0, BIT(30));	/* osc_rc_en_bk */
		ci_rmw(ci, CFG_ON_RC_CTL_1, 0xf0, 0xc0);
		ci_set(ci, CFG_ON_RC_CTL_1, BIT(21));	/* emi_bt_only_rc_en */
	}

	/* no subsystem power cycling in a short interval */
	ci_rmw(ci, RGU_BGFSYS_ON_TOP_PWR_CTL, 0xffff0040, 0x42540000);
	ci_rmw(ci, RGU_WFSYS_ON_TOP_PWR_CTL, 0xffff0040, 0x57460000);

	/* conn2ap slpprot release: bypass ddr_en_ack, ddr_en timeout 1023 T */
	ci_set(ci, CFG_EMI_CTL_0, BIT(18));
	ci_rmw(ci, CFG_EMI_CTL_0, 0x7ff0, 0x3ff0);
	ci_set(ci, CFG_EMI_CTL_0, BIT(15));

	/* bus DCM of the off and von domains */
	ci_set(ci, BUS_CR_ON_OFF_BUS_DCM_CTL_1, BIT(14));
	ci_set(ci, BUS_CR_ON_OFF_BUS_DCM_CTL_1, BIT(16));
	ci_set(ci, BUS_CR_ON_VON_BUS_DCM_CTL_1, BIT(14));
	ci_set(ci, BUS_CR_ON_VON_BUS_DCM_CTL_1, BIT(16));

	/* BT/WF/GPS may not access each other's address space */
	ci_rmw(ci, BUS_CR_LIGHT_SECURITY_CTRL, 0xc63, 0xc63);

	/* bus timeouts: off, on and von layers */
	ci_rmw(ci, BUS_CR_OFF_BUS_TIMEOUT_CTRL, 0x7f8, 0x240);
	ci_set(ci, BUS_CR_OFF_BUS_TIMEOUT_CTRL, BIT(0));
	ci_rmw(ci, BUS_CR_ON_ON_BUS_TIMEOUT_CTRL, 0x7f8, 0x188);
	ci_set(ci, BUS_CR_ON_ON_BUS_TIMEOUT_CTRL, BIT(0));
	ci_rmw(ci, BUS_CR_ON_VON_BUS_TIMEOUT_CTRL, 0x7f8, 0x498);
	ci_set(ci, BUS_CR_ON_VON_BUS_TIMEOUT_CTRL, BIT(0));

	/* off bus debug: auto-generated timeout */
	ci_set(ci, OFF_DEBUG_CTRL_AO_CTRL0, BIT(9));
	ci_rmw(ci, OFF_DEBUG_CTRL_AO_CTRL0, 0xffff0000, 0x11df0000);
	ci_set(ci, OFF_DEBUG_CTRL_AO_CTRL0, BIT(2));
	ci_set(ci, OFF_DEBUG_CTRL_AO_CTRL0, BIT(3));
	ci_set(ci, OFF_DEBUG_CTRL_AO_CTRL0, BIT(4));
	ci_clr(ci, OFF_DEBUG_CTRL_AO_CTRL0, BIT(9));

	/* bus BPLL/WPLL dividers, RF SPI clock from BPLL / 13 */
	ci_set(ci, CKGEN_BUS_BPLL_DIV_1, BIT(0));
	ci_set(ci, CKGEN_BUS_BPLL_DIV_2, BIT(0));
	ci_set(ci, CKGEN_BUS_WPLL_DIV_1, BIT(0));
	ci_set(ci, CKGEN_BUS_WPLL_DIV_2, BIT(0));
	ci_clr(ci, CKGEN_RFSPI_CK_CTRL, BIT(7));
	ci_set(ci, CKGEN_RFSPI_CK_CTRL, BIT(6));
	ci_set(ci, CKGEN_RFSPI_CK_CTRL, BIT(4));
	ci_set(ci, CKGEN_RFSPI_INST2_SW_MODE, BIT(0));

	/* keep the last four SPI write commands for debugging */
	ci_rmw(ci, RF_SPI_MCU_RECORD_CTL, 0x31ff, 0x1ff);

	/* sleep counters, host side */
	ci_set(ci, HOST_CONN_INFRA_SLP_CNT_CTL, BIT(31));
	ci_set(ci, HOST_CONN_INFRA_SLP_CNT_CTL, BIT(0));

	/* coex clocks on, bus clock under HW control */
	ci_wr(ci, CKGEN_COEX_0_SET, 0xffffffff);
	ci_rmw(ci, CKGEN_COEX_1_SET, 0xffff, 0xffff);
	ci_clr(ci, CKGEN_BUS, BIT(0));

	/* command batch the hardware runs to restore the off domain */
	ci_wr(ci, CFG_CMDBT_FETCH_START_ADDR0, SYSRAM_CMDBT_FETCH_START);
	for (i = 0; i < MT6878_CMDBT_WORDS; i++)
		ci_wr(ci, SYSRAM + 4 * i,
		      i < ARRAY_SIZE(mt6878_cmdbt) ? mt6878_cmdbt[i] : 0);

	/* restore before the first wakeup, then HW controlled deep sleep */
	ci_set(ci, CFG_ON_PWRCTRL0, BIT(6));
	ci_set(ci, CFG_ON_PWRCTRL0, BIT(0));
}

/*
 * CONN2AP EMI remapping: the stock kernel asks ATF to program it (the
 * addresses came from LK through another SiP call). Read it back; fall
 * back to a direct write if ATF did nothing.
 */
static void mtk_conninfra_emi_remap(struct mtk_conninfra *ci)
{
	struct arm_smccc_res res;
	u32 want, got;

	ci->emi_remapped = false;
	if (!ci->emi_base) {
		dev_warn(ci->dev, "no emi-addr from the boot loader: CONNSYS firmware not loaded\n");
		return;
	}

	want = FIELD_PREP(CONN2AP_REMAP_BASE, ci->emi_base >> 16);
	arm_smccc_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, CONNSYS_SMC_EMI_REMAP,
		      0, 0, 0, 0, 0, 0, &res);
	got = ci_rd(ci, HOST_CONN2AP_REMAP_MCU_EMI) & CONN2AP_REMAP_BASE;
	if (got != want) {
		dev_warn(ci->dev, "EMI remap after SMC (ret %lu): 0x%05x, writing 0x%05x\n",
			 res.a0, got, want);
		ci_rmw(ci, HOST_CONN2AP_REMAP_MCU_EMI, CONN2AP_REMAP_BASE, want);
		got = ci_rd(ci, HOST_CONN2AP_REMAP_MCU_EMI) & CONN2AP_REMAP_BASE;
	}

	ci->emi_remapped = got == want;
	if (ci->emi_remapped)
		dev_info(ci->dev, "CONNSYS EMI at %pa (0x%x), remap 0x%05x\n",
			 &ci->emi_base, ci->emi_size, got);
	else
		dev_err(ci->dev, "CONN2AP EMI remap stays 0x%05x\n", got);
}

/* ------------------------------------------------------------------ */
/* power: rails, domain, pins                                          */

static void mtk_conninfra_select_pins(struct mtk_conninfra *ci, bool active)
{
	struct pinctrl_state *state = active ? ci->pins_active : ci->pins_idle;
	int ret;

	if (IS_ERR_OR_NULL(ci->pinctrl) || IS_ERR_OR_NULL(state))
		return;

	ret = pinctrl_select_state(ci->pinctrl, state);
	if (ret)
		dev_warn(ci->dev, "failed to select %s pins: %d\n",
			 active ? "active" : "idle", ret);
}

static int mtk_conninfra_hw_on(struct mtk_conninfra *ci)
{
	int ret;

	/* PMIC: rails of the D-die AFE, the crystal and the A-die */
	ret = regulator_bulk_enable(ARRAY_SIZE(ci->supplies), ci->supplies);
	if (ret) {
		dev_err(ci->dev, "failed to enable supplies: %d\n", ret);
		return ret;
	}
	if (ci->vant18) {
		ret = regulator_enable(ci->vant18);
		if (ret)
			goto err_supplies;
	}

	if (ci->topckgen)
		regmap_clear_bits(ci->topckgen, TOPCKGEN_CONN_CK_CTRL,
				  CONN_VON_TOP_HOST_CK_REQ_EN);

	/* D-die <-> A-die interface: GPIO172..182 to their CONN functions */
	mtk_conninfra_select_pins(ci, true);

	/*
	 * CONN power domain: power, clock, isolation and reset in SPM, then
	 * the AP<->CONNSYS bus protection in INFRACFG_AO.
	 */
	ret = pm_runtime_resume_and_get(ci->dev);
	if (ret) {
		dev_err(ci->dev, "failed to power on the CONN domain: %d\n", ret);
		goto err_pins;
	}

	ret = mtk_conninfra_poll_ip_version(ci, false);
	if (ret)
		goto err_pm;

	ci->hw_on = true;
	return 0;

err_pm:
	if (!mtk_conninfra_bus_clock_ok(ci))
		dev_err(ci->dev, "CONN_INFRA bus/OSC clock missing\n");
	pm_runtime_put_sync(ci->dev);
err_pins:
	mtk_conninfra_select_pins(ci, false);
	if (ci->vant18)
		regulator_disable(ci->vant18);
err_supplies:
	regulator_bulk_disable(ARRAY_SIZE(ci->supplies), ci->supplies);
	return ret;
}

static void mtk_conninfra_hw_off(struct mtk_conninfra *ci)
{
	pm_runtime_put_sync(ci->dev);
	mtk_conninfra_select_pins(ci, false);
	if (ci->vant18)
		regulator_disable(ci->vant18);
	regulator_bulk_disable(ARRAY_SIZE(ci->supplies), ci->supplies);
	ci->hw_on = false;
	ci->wake_cnt = 0;
}

static int mtk_conninfra_radio_rails(struct mtk_conninfra *ci,
				     enum mtk_conninfra_radio radio, bool on)
{
	struct regulator *regs[2] = { };
	unsigned int i;
	int ret = 0;

	/* stock "legacy mode" rails; in RC mode the PMIC follows the RC */
	if (radio == MTK_CONNINFRA_WIFI) {
		regs[0] = ci->vcn33_1;
		regs[1] = ci->vcn33_2;
	} else if (radio == MTK_CONNINFRA_BT) {
		regs[0] = ci->vcn33_1;
	}

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		if (!regs[i])
			continue;
		if (on) {
			ret = regulator_enable(regs[i]);
			if (ret) {
				while (i--)
					if (regs[i])
						regulator_disable(regs[i]);
				return ret;
			}
		} else {
			regulator_disable(regs[i]);
		}
	}
	return ret;
}

/* ------------------------------------------------------------------ */
/* CONN_INFRA off-domain wakeup                                        */

static int __mtk_conninfra_wakeup(struct mtk_conninfra *ci)
{
	int ret;

	if (!ci->hw_on)
		return -ENODEV;
	if (ci->wake_cnt++)
		return 0;

	ci_wr(ci, HOST_CONN_INFRA_WAKEPU_TOP, 0x1);
	udelay(200);
	ret = mtk_conninfra_poll_ip_version(ci, false);
	if (ret) {
		ci_wr(ci, HOST_CONN_INFRA_WAKEPU_TOP, 0x0);
		ci->wake_cnt--;
		return ret;
	}
	/* the command batch has restored the off domain */
	if (ci_poll(ci, CFG_ON_PWRCTRL1, PWRCTRL1_CONN_INFRA_RDY,
		    PWRCTRL1_CONN_INFRA_RDY, 500, 5000))
		dev_warn(ci->dev, "CONN_INFRA restore not done (0x%08x)\n",
			 ci_rd(ci, CFG_ON_PWRCTRL1));
	return 0;
}

static void __mtk_conninfra_sleep(struct mtk_conninfra *ci)
{
	if (WARN_ON(!ci->wake_cnt))
		return;
	if (!--ci->wake_cnt)
		ci_wr(ci, HOST_CONN_INFRA_WAKEPU_TOP, 0x0);
}

/* ------------------------------------------------------------------ */
/* radios                                                              */

static int mtk_conninfra_first_on(struct mtk_conninfra *ci,
				  enum mtk_conninfra_radio radio)
{
	unsigned long next = BIT(radio);
	int ret;

	ret = mtk_conninfra_hw_on(ci);
	if (ret)
		return ret;

	mtk_conninfra_emi_remap(ci);
	mtk_conninfra_d_die_cfg(ci);
	mtk_conninfra_spi_master_cfg_6637(ci);

	ret = mtk_conninfra_a_die_cfg_6637(ci);
	if (ret)
		goto err;

	ret = mtk_conninfra_afe_cfg(ci, true);
	if (ret)
		goto err;
	mtk_conninfra_pll_init(ci);

	ret = mtk_conninfra_radio_status(ci, radio, true);
	if (ret)
		goto err;

	ret = mtk_conninfra_adie_xo_buffers(ci, 0, next);
	if (ret)
		goto err;
	mtk_conninfra_low_power_cfg(ci);
	return 0;

err:
	if (!mtk_conninfra_bus_clock_ok(ci))
		dev_err(ci->dev, "CONN_INFRA bus/OSC clock missing\n");
	mtk_conninfra_hw_off(ci);
	return ret;
}

int mtk_conninfra_radio_on(struct mtk_conninfra *ci, enum mtk_conninfra_radio radio)
{
	unsigned long cur;
	int ret;

	if (radio >= MTK_CONNINFRA_RADIO_MAX)
		return -EINVAL;
	if (!ci->data->full_power_on) {
		dev_err(ci->dev, "power-on sequence for the %s A-die not implemented\n",
			ci->data->adie_name);
		return -EOPNOTSUPP;
	}

	mutex_lock(&ci->lock);
	cur = ci->radios;
	if (cur & BIT(radio)) {
		ret = -EBUSY;
		goto out;
	}

	ret = mtk_conninfra_radio_rails(ci, radio, true);
	if (ret)
		goto out;

	if (!cur) {
		ret = mtk_conninfra_first_on(ci, radio);
	} else {
		ret = __mtk_conninfra_wakeup(ci);
		if (!ret) {
			ret = mtk_conninfra_radio_status(ci, radio, true);
			if (!ret)
				ret = mtk_conninfra_adie_xo_buffers(ci, cur,
								    cur | BIT(radio));
			__mtk_conninfra_sleep(ci);
		}
	}
	if (ret) {
		mtk_conninfra_radio_rails(ci, radio, false);
		goto out;
	}

	ci->radios |= BIT(radio);
	dev_info(ci->dev, "radio %d on, radios 0x%lx\n", radio, ci->radios);
out:
	mutex_unlock(&ci->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_conninfra_radio_on);

void mtk_conninfra_radio_off(struct mtk_conninfra *ci, enum mtk_conninfra_radio radio)
{
	unsigned long cur, next;

	if (radio >= MTK_CONNINFRA_RADIO_MAX)
		return;

	mutex_lock(&ci->lock);
	cur = ci->radios;
	if (WARN_ON(!(cur & BIT(radio))))
		goto out;
	next = cur & ~BIT(radio);

	if (radio == MTK_CONNINFRA_WIFI && ci->wfsys_on)
		dev_warn(ci->dev, "Wi-Fi radio off with WFSYS still on\n");

	if (next) {
		if (!__mtk_conninfra_wakeup(ci)) {
			mtk_conninfra_radio_status(ci, radio, false);
			mtk_conninfra_adie_xo_buffers(ci, cur, next);
			__mtk_conninfra_sleep(ci);
		}
	} else {
		ci->wfsys_on = false;
		mtk_conninfra_hw_off(ci);
	}
	mtk_conninfra_radio_rails(ci, radio, false);
	ci->radios = next;
	dev_info(ci->dev, "radio %d off, radios 0x%lx\n", radio, ci->radios);
out:
	mutex_unlock(&ci->lock);
}
EXPORT_SYMBOL_GPL(mtk_conninfra_radio_off);

int mtk_conninfra_wakeup(struct mtk_conninfra *ci)
{
	int ret;

	mutex_lock(&ci->lock);
	ret = ci->radios ? __mtk_conninfra_wakeup(ci) : -ENODEV;
	mutex_unlock(&ci->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_conninfra_wakeup);

void mtk_conninfra_sleep(struct mtk_conninfra *ci)
{
	mutex_lock(&ci->lock);
	__mtk_conninfra_sleep(ci);
	mutex_unlock(&ci->lock);
}
EXPORT_SYMBOL_GPL(mtk_conninfra_sleep);

/* ------------------------------------------------------------------ */
/* WFSYS                                                               */

static int mtk_conninfra_wf_poll(struct mtk_conninfra *ci, u32 off, u32 mask,
				 u32 want, unsigned int delay_us,
				 unsigned int timeout_us)
{
	u32 v;

	return readl_poll_timeout_atomic(ci->wfsys + off, v, (v & mask) == want,
					 delay_us, timeout_us);
}

/* Wi-Fi's own wakeup of the CONN_INFRA off domain (stock wake_up_conninfra_off) */
static int mtk_conninfra_wf_wake(struct mtk_conninfra *ci)
{
	unsigned int i;
	int ret;

	for (i = 0; i < 10; i++) {
		ci_set(ci, HOST_CONN_INFRA_WAKEPU_WF, BIT(0));
		usleep_range(1000, 1100);
		if (ci_rd(ci, HOST_CONN_INFRA_WAKEPU_WF) & BIT(0))
			break;
	}

	ret = mtk_conninfra_poll_ip_version(ci, false);
	if (ret)
		return ret;

	ret = ci_poll(ci, CFG_ON_PWRCTRL1, PWRCTRL1_CONN_INFRA_RDY,
		      PWRCTRL1_CONN_INFRA_RDY, 500, 5500);
	if (ret)
		dev_err(ci->dev, "CONN_INFRA restore not done (0x%08x)\n",
			ci_rd(ci, CFG_ON_PWRCTRL1));
	return ret;
}

static void mtk_conninfra_wfsys_dump(struct mtk_conninfra *ci)
{
	dev_err(ci->dev, "PWR_STATES 0x%08x WF_SLP_STATUS 0x%08x MONFLG 0x%08x RST_B 0x%08x TOP_PWR 0x%08x\n",
		ci_rd(ci, HOST_CONNSYS_PWR_STATES), ci_rd(ci, CFG_ON_WF_SLP_STATUS),
		ci_rd(ci, HOST_WF_ON_MONFLG_OUT), ci_rd(ci, RGU_WFSYS_CPU_SW_RST_B),
		ci_rd(ci, RGU_WFSYS_ON_TOP_PWR_CTL));
}

/* back to the state before mtk_conninfra_wfsys_on: protection on, top off */
static void mtk_conninfra_wfsys_top_off(struct mtk_conninfra *ci)
{
	u32 val;

	ci_set(ci, CFG_ON_WF_SLP_CTRL, WF_SLP_PROT_SW_EN);
	if (ci_poll(ci, CFG_ON_WF_SLP_STATUS, WFDMA2CONN_SLP_PROT_RDY,
		    WFDMA2CONN_SLP_PROT_RDY, 500, 50000))
		dev_err(ci->dev, "WFDMA to CONN_INFRA protection not on (0x%08x)\n",
			ci_rd(ci, CFG_ON_WF_SLP_STATUS));
	if (readl_poll_timeout_atomic(ci->base + CFG_ON_WF_SLP_STATUS, val,
				      val & (WF2CONN_SLP_PROT_RDY | WF2CONN_SLP_PROT_HW_EN),
				      500, 50000))
		dev_err(ci->dev, "WFSYS to CONN_INFRA protection not on (0x%08x)\n",
			ci_rd(ci, CFG_ON_WF_SLP_STATUS));

	ci_rmw(ci, RGU_WFSYS_ON_TOP_PWR_CTL, TOP_PWR_CTL_KEY | TOP_PWR_CTL_ON,
	       FIELD_PREP(TOP_PWR_CTL_KEY, WFSYS_TOP_KEY));
	if (ci_poll(ci, HOST_CONNSYS_PWR_STATES, PWR_STATES_WFSYS_RGU_OFF_RST_B,
		    0, 500, 5500))
		dev_err(ci->dev, "WFSYS RGU still out of reset (0x%08x)\n",
			ci_rd(ci, HOST_CONNSYS_PWR_STATES));
}

static int __mtk_conninfra_wfsys_on(struct mtk_conninfra *ci)
{
	u32 val;
	int ret;

	ret = mtk_conninfra_wf_wake(ci);
	if (ret)
		return ret;

	/* PTA clocks */
	ci_wr(ci, CKGEN_COEX_0_SET, COEX_0_WF_PTA);
	ci_wr(ci, CKGEN_COEX_1_SET, COEX_1_WF_PTA);

	/* WM CPU held in reset, conn<->wf bus protection off, WFSYS top on */
	ci_clr(ci, RGU_WFSYS_CPU_SW_RST_B, BIT(0));
	ci_clr(ci, CFG_ON_WF_SLP_CTRL, WF_SLP_PROT_SW_EN);
	ci_rmw(ci, RGU_WFSYS_ON_TOP_PWR_CTL, TOP_PWR_CTL_KEY | TOP_PWR_CTL_ON,
	       FIELD_PREP(TOP_PWR_CTL_KEY, WFSYS_TOP_KEY) | TOP_PWR_CTL_ON);

	ret = ci_poll(ci, HOST_CONNSYS_PWR_STATES, PWR_STATES_WFSYS_RGU_OFF_RST_B,
		      PWR_STATES_WFSYS_RGU_OFF_RST_B, 500, 5500);
	if (ret) {
		dev_err(ci->dev, "WFSYS RGU does not come out of reset\n");
		goto err_top;
	}

	/*
	 * Until both directions of the CONN_INFRA<->WFSYS protection are
	 * released, an AP access to a WFSYS register queues at the
	 * protection and hangs the bus: do not touch WFSYS before this.
	 */
	ret = ci_poll(ci, CFG_ON_WF_SLP_STATUS,
		      WF2CONN_SLP_PROT_RDY | CONN2WF_SLP_PROT_RDY, 0, 500, 50000);
	if (ret) {
		dev_err(ci->dev, "CONN_INFRA<->WFSYS protection not released\n");
		goto err_top;
	}

	/* WFDMA to CONN_INFRA AXI TX/RX protection */
	ret = mtk_conninfra_wf_poll(ci, WF_TOP_SLPPROT_ON_STATUS,
				    WF_SLPPROT_EN_SOURCE_1, 0, 500, 50000);
	if (!ret)
		ret = mtk_conninfra_wf_poll(ci, WF_TOP_SLPPROT_ON_STATUS,
					    WF_SLPPROT_EN_SOURCE_2, 0, 500, 50000);
	if (ret) {
		dev_err(ci->dev, "WFDMA protection not released (0x%08x)\n",
			wf_rd(ci, WF_TOP_SLPPROT_ON_STATUS));
		goto err_top;
	}

	ret = mtk_conninfra_wf_poll(ci, WF_TOP_CFG_IP_VERSION, U32_MAX,
				    MT6878_WF_IP_VERSION, 500, 5500);
	if (ret) {
		dev_err(ci->dev, "WFSYS IP version 0x%08x, expected 0x%08x\n",
			wf_rd(ci, WF_TOP_CFG_IP_VERSION), MT6878_WF_IP_VERSION);
		goto err_top;
	}

	/* WFSYS AHB/APB bus timeout */
	wf_rmw(ci, WF_MCU_CONFG_LS_BUSHANGCR, BUSHANG_TIME_LIMIT, 0x03);
	wf_rmw(ci, WF_MCU_CONFG_LS_BUSHANGCR, BUSHANG_DEBUG_EN | BUSHANG_DEBUG_CG_EN,
	       BUSHANG_DEBUG_EN | BUSHANG_DEBUG_CG_EN);

	/* remap window 0 to the WF MCU debug_ctrl_ao: bus timeout there too */
	wf_wr(ci, WF_MCU_BUS_CR_AP2WF_REMAP_1, WF_DEBUG_CTRL_AO_BASE);
	wf_rmw(ci, WF_PWA_CTRL0, PWA_DEBUG_CKEN, PWA_DEBUG_CKEN);
	wf_rmw(ci, WF_PWA_CTRL0, PWA_TIMEOUT_CLR, PWA_TIMEOUT_CLR);
	wf_rmw(ci, WF_PWA_CTRL0, PWA_TIMEOUT_CLR, 0);
	wf_rmw(ci, WF_PWA_CTRL0, PWA_TIMEOUT_THRES, 0x04f50000);
	wf_rmw(ci, WF_PWA_CTRL3, BIT(8), BIT(8));	/* mask wfdma+umac busy */
	wf_rmw(ci, WF_PWA_CTRL0, PWA_DEBUG_EN | PWA_DEBUG_CKEN | PWA_DEBUG_EN_DEBUGTOP,
	       PWA_DEBUG_EN | PWA_DEBUG_CKEN | PWA_DEBUG_EN_DEBUGTOP);

	/* WF mailbox flags readable from the host side */
	ci_set(ci, HOST_WF_ON_MONFLG_EN, BIT(0));
	ci_rmw(ci, HOST_WF_ON_MONFLG_SEL, GENMASK(4, 0), 0xc);

	/* release the WM CPU and wait for the ROM to reach its idle loop */
	ci_set(ci, RGU_WFSYS_CPU_SW_RST_B, BIT(0));
	ret = readl_poll_timeout(ci->base + HOST_WF_ON_MONFLG_OUT, val,
				 val == WF_ROM_IDLE, 1000, 1000000);
	if (ret) {
		dev_err(ci->dev, "WM MCU not idle: MONFLG 0x%08x, ROM index 0x%08x\n",
			val, wf_rd(ci, WF_TOP_CFG_ON_ROMCODE_INDEX));
		ci_clr(ci, RGU_WFSYS_CPU_SW_RST_B, BIT(0));
		goto err_top;
	}

	/* drop Wi-Fi's force-on of the off domain */
	ci_clr(ci, HOST_CONN_INFRA_WAKEPU_WF, BIT(0));
	return 0;

err_top:
	mtk_conninfra_wfsys_dump(ci);
	mtk_conninfra_wfsys_top_off(ci);
	ci_wr(ci, CKGEN_COEX_0_CLR, COEX_0_WF_PTA);
	ci_wr(ci, CKGEN_COEX_1_CLR, COEX_1_WF_PTA);
	ci_clr(ci, HOST_CONN_INFRA_WAKEPU_WF, BIT(0));
	return ret ?: -EIO;
}

int mtk_conninfra_wfsys_on(struct mtk_conninfra *ci)
{
	int ret;

	if (!ci->wfsys)
		return -ENODEV;

	mutex_lock(&ci->lock);
	if (!(ci->radios & BIT(MTK_CONNINFRA_WIFI))) {
		ret = -ENODEV;
		goto out;
	}
	if (ci->wfsys_on) {
		ret = -EBUSY;
		goto out;
	}
	if (!ci->emi_remapped && !allow_no_emi) {
		dev_err(ci->dev, "CONNSYS EMI not remapped: not starting the WM MCU\n");
		ret = -ENOENT;
		goto out;
	}

	dev_info(ci->dev, "WFSYS power-on\n");
	ret = __mtk_conninfra_wfsys_on(ci);
	if (!ret) {
		ci->wfsys_on = true;
		dev_info(ci->dev, "WFSYS on, WM MCU in ROM idle\n");
	}
out:
	mutex_unlock(&ci->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_conninfra_wfsys_on);

void mtk_conninfra_wfsys_off(struct mtk_conninfra *ci)
{
	mutex_lock(&ci->lock);
	if (!ci->wfsys_on)
		goto out;

	if (mtk_conninfra_wf_wake(ci))
		dev_warn(ci->dev, "WFSYS power-off without CONN_INFRA wakeup\n");

	mtk_conninfra_wfsys_top_off(ci);

	/* release the semaphores WFSYS may still hold */
	ci_clr(ci, RGU_SEMA_M0_SW_RST_B, BIT(0));
	ci_set(ci, RGU_SEMA_M0_SW_RST_B, BIT(0));
	/* acknowledge pending WF2AP CCIF interrupts */
	ci_rmw(ci, CCIF4_AP2WF_PCCIF_ACK, 0xff, 0xff);

	/* A-die top_ck_en_1 off */
	ci_clr(ci, WB_SLP_TOP_CK_1, WB_SLP_TOP_CK_EN);
	if (ci_poll(ci, WB_SLP_TOP_CK_1, WB_SLP_TOP_CK_BSY, 0, 500, 50000))
		dev_err(ci->dev, "A-die top_ck_en_1 off timed out\n");

	/* clear wf_emi_req and wf_infra_req */
	ci_set(ci, CFG_EMI_CTL_WF, BIT(0));
	ci_clr(ci, CFG_EMI_CTL_WF, BIT(0));
	ci_set(ci, CFG_EMI_CTL_WF, BIT(5));
	ci_clr(ci, CFG_EMI_CTL_WF, BIT(5));

	ci_wr(ci, CKGEN_COEX_0_CLR, COEX_0_WF_PTA);
	ci_wr(ci, CKGEN_COEX_1_CLR, COEX_1_WF_PTA);
	ci_clr(ci, HOST_CONN_INFRA_WAKEPU_WF, BIT(0));

	ci->wfsys_on = false;
	dev_info(ci->dev, "WFSYS off\n");
out:
	mutex_unlock(&ci->lock);
}
EXPORT_SYMBOL_GPL(mtk_conninfra_wfsys_off);

bool mtk_conninfra_wfsys_is_on(struct mtk_conninfra *ci)
{
	return READ_ONCE(ci->wfsys_on);
}
EXPORT_SYMBOL_GPL(mtk_conninfra_wfsys_is_on);

u32 mtk_conninfra_adie_id(struct mtk_conninfra *ci)
{
	return READ_ONCE(ci->adie_id);
}
EXPORT_SYMBOL_GPL(mtk_conninfra_adie_id);

int mtk_conninfra_get_emi(struct mtk_conninfra *ci, phys_addr_t *base, u32 *size)
{
	if (!ci->emi_base)
		return -ENOENT;
	*base = ci->emi_base;
	*size = ci->emi_size;
	return 0;
}
EXPORT_SYMBOL_GPL(mtk_conninfra_get_emi);

static void mtk_conninfra_put_device(void *data)
{
	put_device(data);
}

struct mtk_conninfra *devm_mtk_conninfra_get(struct device *dev)
{
	struct platform_device *pdev;
	struct device_node *np;
	struct mtk_conninfra *ci;
	int ret;

	np = of_parse_phandle(dev->of_node, "mediatek,conninfra", 0);
	if (!np)
		return ERR_PTR(-ENODEV);
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	ret = devm_add_action_or_reset(dev, mtk_conninfra_put_device, &pdev->dev);
	if (ret)
		return ERR_PTR(ret);

	ci = platform_get_drvdata(pdev);
	if (!ci)
		return ERR_PTR(-EPROBE_DEFER);
	if (!device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return ERR_PTR(-EINVAL);
	return ci;
}
EXPORT_SYMBOL_GPL(devm_mtk_conninfra_get);

/* ------------------------------------------------------------------ */
/* debugfs: explicit bring-up commands                                 */

static const char * const mtk_conninfra_radio_names[] = {
	[MTK_CONNINFRA_BT] = "bt",
	[MTK_CONNINFRA_FM] = "fm",
	[MTK_CONNINFRA_GPS] = "gps",
	[MTK_CONNINFRA_WIFI] = "wifi",
};

static int mtk_conninfra_status_show(struct seq_file *s, void *unused)
{
	struct mtk_conninfra *ci = s->private;

	mutex_lock(&ci->lock);
	seq_printf(s, "A-die (compatible): %s\n", ci->data->adie_name);
	seq_printf(s, "radios: 0x%lx, hw_on %d, wake_cnt %u, wfsys_on %d\n",
		   ci->radios, ci->hw_on, ci->wake_cnt, ci->wfsys_on);
	seq_printf(s, "A-die chip id: 0x%08x, efuse %08x %08x %08x %08x\n",
		   ci->adie_id, ci->adie_efuse[0], ci->adie_efuse[1],
		   ci->adie_efuse[2], ci->adie_efuse[3]);
	seq_printf(s, "D-die efuse: 0x%08x\n", ci->d_die_efuse);
	seq_printf(s, "co-clock: %s, RC mode: %d\n",
		   ci->co_clock_52m ? "52 MHz" : "26 MHz", ci->rc_mode);
	seq_printf(s, "EMI: %pa size 0x%x, remapped %d\n",
		   &ci->emi_base, ci->emi_size, ci->emi_remapped);

	if (ci->radios && !__mtk_conninfra_wakeup(ci)) {
		seq_printf(s, "IP version 0x%08x, radio status 0x%08x, remap 0x%08x\n",
			   ci_rd(ci, CFG_IP_VERSION), ci_rd(ci, SW_CR_RADIO_STATUS),
			   ci_rd(ci, HOST_CONN2AP_REMAP_MCU_EMI));
		seq_printf(s, "PWR_STATES 0x%08x WF_SLP_STATUS 0x%08x MONFLG 0x%08x\n",
			   ci_rd(ci, HOST_CONNSYS_PWR_STATES),
			   ci_rd(ci, CFG_ON_WF_SLP_STATUS),
			   ci_rd(ci, HOST_WF_ON_MONFLG_OUT));
		__mtk_conninfra_sleep(ci);
	}
	mutex_unlock(&ci->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_conninfra_status);

/* "<radio> on|off" (radio: bt fm gps wifi), "wfsys on|off" */
static ssize_t mtk_conninfra_ctl_write(struct file *file, const char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	struct mtk_conninfra *ci = file->private_data;
	char buf[32], *p, *what, *op;
	bool on;
	int i, ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	p = strim(buf);
	what = strsep(&p, " ");
	op = p ? strim(p) : NULL;
	if (!op)
		return -EINVAL;
	if (!strcmp(op, "on"))
		on = true;
	else if (!strcmp(op, "off"))
		on = false;
	else
		return -EINVAL;

	if (!strcmp(what, "wfsys")) {
		if (!on) {
			mtk_conninfra_wfsys_off(ci);
			return count;
		}
		ret = mtk_conninfra_wfsys_on(ci);
		return ret ?: count;
	}

	i = match_string(mtk_conninfra_radio_names,
			 ARRAY_SIZE(mtk_conninfra_radio_names), what);
	if (i < 0)
		return -EINVAL;
	if (on) {
		ret = mtk_conninfra_radio_on(ci, i);
	} else {
		mtk_conninfra_radio_off(ci, i);
		ret = 0;
	}
	return ret ?: count;
}

static const struct file_operations mtk_conninfra_ctl_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = mtk_conninfra_ctl_write,
	.llseek = noop_llseek,
};

static void mtk_conninfra_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static void mtk_conninfra_debugfs_init(struct mtk_conninfra *ci)
{
	ci->debugfs = debugfs_create_dir(dev_name(ci->dev), NULL);
	debugfs_create_file("status", 0400, ci->debugfs, ci, &mtk_conninfra_status_fops);
	debugfs_create_file("ctl", 0200, ci->debugfs, ci, &mtk_conninfra_ctl_fops);
	devm_add_action_or_reset(ci->dev, mtk_conninfra_debugfs_remove, ci->debugfs);
}

/* ------------------------------------------------------------------ */

static void mtk_conninfra_get_emi_from_dt(struct mtk_conninfra *ci)
{
	struct device_node *np = ci->dev->of_node;
	u64 addr;

	/* written by LK when it loaded the CONNSYS firmware */
	if (of_property_read_u64(np, "emi-addr", &addr) || !addr)
		return;
	if (of_property_read_u32(np, "emi-size", &ci->emi_size))
		return;
	ci->emi_base = addr;
}

static int mtk_conninfra_get_optional_supply(struct device *dev, const char *id,
					     struct regulator **reg)
{
	struct regulator *r = devm_regulator_get_optional(dev, id);

	if (IS_ERR(r)) {
		if (PTR_ERR(r) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(r), "failed to get %s\n", id);
		r = NULL;
	}
	*reg = r;
	return 0;
}

static int mtk_conninfra_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_conninfra *ci;
	struct resource *res;
	void __iomem *rc;
	unsigned int i;
	int ret;

	ci = devm_kzalloc(dev, sizeof(*ci), GFP_KERNEL);
	if (!ci)
		return -ENOMEM;

	ci->dev = dev;
	ci->data = of_device_get_match_data(dev);
	if (!ci->data)
		return -EINVAL;
	spin_lock_init(&ci->spi_lock);

	ret = devm_mutex_init(dev, &ci->lock);
	if (ret)
		return ret;

	/*
	 * The CONN_INFRA and WFSYS windows are shared with the Wi-Fi and
	 * Bluetooth blocks, which map them as well: do not claim them.
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "conninfra");
	if (!res)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	ci->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!ci->base)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "wfsys");
	if (res) {
		ci->wfsys = devm_ioremap(dev, res->start, resource_size(res));
		if (!ci->wfsys)
			return -ENOMEM;
	}

	/* SRCLKEN_RC is shared too; it is only read once here */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "srclken-rc");
	if (res) {
		rc = ioremap(res->start, resource_size(res));
		if (!rc)
			return -ENOMEM;
		ci->rc_mode = readl(rc + SRCLKEN_RC_CENTRAL_CFG1) & SRCLKEN_RC_EN;
		iounmap(rc);
	}

	ci->topckgen = syscon_regmap_lookup_by_phandle_optional(dev->of_node,
								"mediatek,topckgen");
	if (IS_ERR(ci->topckgen))
		return dev_err_probe(dev, PTR_ERR(ci->topckgen),
				     "invalid mediatek,topckgen\n");

	for (i = 0; i < ARRAY_SIZE(ci->supplies); i++)
		ci->supplies[i].supply = mtk_conninfra_supplies[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ci->supplies),
				      ci->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	ret = mtk_conninfra_get_optional_supply(dev, "vcn33-1", &ci->vcn33_1);
	if (!ret)
		ret = mtk_conninfra_get_optional_supply(dev, "vcn33-2", &ci->vcn33_2);
	if (!ret)
		ret = mtk_conninfra_get_optional_supply(dev, "vant18", &ci->vant18);
	if (ret)
		return ret;

	ci->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ci->pinctrl)) {
		ret = PTR_ERR(ci->pinctrl);
		if (ret == -EPROBE_DEFER)
			return ret;
		ci->pinctrl = NULL;
	} else {
		ci->pins_active = pinctrl_lookup_state(ci->pinctrl, "active");
		ci->pins_idle = pinctrl_lookup_state(ci->pinctrl, "idle");
		ci->pins_adie_cfg = pinctrl_lookup_state(ci->pinctrl, "adie-cfg");
	}

	ci->co_clock_52m = of_property_read_bool(dev->of_node, "mediatek,co-clock-52mhz");
	mtk_conninfra_get_emi_from_dt(ci);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	/* Show that CONN_INFRA answers, then leave it off until asked. */
	mutex_lock(&ci->lock);
	ret = mtk_conninfra_hw_on(ci);
	if (!ret) {
		dev_info(dev, "CONN_INFRA IP version 0x%08x, A-die %s\n",
			 ci_rd(ci, CFG_IP_VERSION), ci->data->adie_name);
		mtk_conninfra_hw_off(ci);
	}
	mutex_unlock(&ci->lock);
	if (ret)
		return ret;

	mtk_conninfra_debugfs_init(ci);
	/* from here on devm_mtk_conninfra_get() hands it out */
	platform_set_drvdata(pdev, ci);
	return 0;
}

static void mtk_conninfra_remove(struct platform_device *pdev)
{
	struct mtk_conninfra *ci = platform_get_drvdata(pdev);
	int radio;

	/* consumers are unbound first (device link); drop debugfs users */
	mtk_conninfra_wfsys_off(ci);
	for (radio = 0; radio < MTK_CONNINFRA_RADIO_MAX; radio++)
		if (ci->radios & BIT(radio))
			mtk_conninfra_radio_off(ci, radio);
}

static const struct mtk_conninfra_data mt6878_6637_data = {
	.ip_version = 0x02050401,
	.adie = MTK_ADIE_MT6637,
	.adie_name = "MT6637",
	.full_power_on = true,
};

static const struct mtk_conninfra_data mt6878_6631_data = {
	.ip_version = 0x02050401,
	.adie = MTK_ADIE_MT6631,
	.adie_name = "MT6631",
};

static const struct mtk_conninfra_data mt6878_6631_6686_data = {
	.ip_version = 0x02050401,
	.adie = MTK_ADIE_MT6631_MT6686,
	.adie_name = "MT6631+MT6686",
};

/*
 * The stock LK rewrites the compatible of /soc/consys (or /consys) with
 * the A-die it detects through the hwid-node GPIOs; the three variants
 * below are the only values it writes on MT6878.
 */
static const struct of_device_id mtk_conninfra_of_match[] = {
	{ .compatible = "mediatek,mt6878-6637-consys", .data = &mt6878_6637_data },
	{ .compatible = "mediatek,mt6878-6631-consys", .data = &mt6878_6631_data },
	{ .compatible = "mediatek,mt6878-6631-6686-consys", .data = &mt6878_6631_6686_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_conninfra_of_match);

static struct platform_driver mtk_conninfra_driver = {
	.probe = mtk_conninfra_probe,
	.remove = mtk_conninfra_remove,
	.driver = {
		.name = "mtk-conninfra",
		.of_match_table = mtk_conninfra_of_match,
	},
};
module_platform_driver(mtk_conninfra_driver);

MODULE_DESCRIPTION("MediaTek CONNINFRA power driver");
MODULE_LICENSE("GPL");
