// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 MediaTek Inc.
 *
 * MediaTek VCP (video companion processor, "MMuP", RV55) host driver.
 *
 * Ported to mainline 7.2 for the Unihertz Titan 2 from the MediaTek GPL
 * device-modules tree (kernel_device_modules-6.1,
 * drivers/misc/mediatek/vcp/rv/vcp_helper.c, vcp_awake.c, vcp_irq.c and
 * vcp_feature_table.c). Reduced port: the downstream power/reset sequence,
 * mailbox/IPI tables, ready handshake, feature reference counting and
 * notifier are kept; logger, exception dump, recovery, hardware voter,
 * semaphores, SMMU and MTK dma-heap handling are left out.
 *
 * As on the downstream kernel, the boot loader loads the VCP firmware
 * ("tinysys-vcp-RV55_A" from the vcp_<slot> partition) into
 * me_vcp_reserved when the "mediatek,vcp" node has vcp-support = <1>.
 * This driver only powers the VCP, tells it where the AP shared memory is
 * and releases it from reset through the SiP call, then waits for the
 * firmware's ready IPI.
 *
 * Deviation from the downstream driver: the shared memory table (vcp-mem-tbl) is carved
 * from one dma_alloc_coherent() block of the VCP device instead of per
 * region MTK dma-heap buffers mapped through the MM IOMMU, because MT6878
 * has no mainline IOMMU driver yet. The VCP then sees physical addresses;
 * whether its DRAM port is in IOMMU bypass is to be checked on hardware.
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/timer.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/soc/mediatek/mtk-mbox.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/soc/mediatek/mtk_tinysys_ipi.h>

#include "vcp.h"
#include "vcp_reg.h"

#ifndef MTK_SIP_TINYSYS_VCP_CONTROL
#define MTK_SIP_TINYSYS_VCP_CONTROL	MTK_SIP_SMC_CMD(0x52C)
#endif
#ifndef MTK_SIP_KERNEL_DAPC_MMUP_CONTROL
#define MTK_SIP_KERNEL_DAPC_MMUP_CONTROL	MTK_SIP_SMC_CMD(0x52E)
#endif

enum mtk_tinysys_vcp_kernel_op {
	MTK_TINYSYS_VCP_KERNEL_OP_DUMP_START = 0,
	MTK_TINYSYS_VCP_KERNEL_OP_DUMP_POLLING,
	MTK_TINYSYS_VCP_KERNEL_OP_RESET_SET,
	MTK_TINYSYS_VCP_KERNEL_OP_RESET_RELEASE,
	MTK_TINYSYS_VCP_KERNEL_OP_SET_SRAMLOGBUF_INFO,
	MTK_TINYSYS_VCP_KERNEL_OP_NUM,
};

/* reset_vcp() argument, as in the downstream driver */
#define VCP_ALL_ENABLE		0x00
#define VCP_ALL_SUSPEND		0x10

#define VCP_READY_TIMEOUT	(HZ / 5 * 4)	/* 800 ms */
#define VCP_AWAKE_TIMEOUT	200000		/* us, reboot handshake */

struct slp_ctrl_data {
	u32 feature;
	u32 cmd;
};

enum {
	SLP_WAKE_LOCK = 0,
	SLP_WAKE_UNLOCK,
	SLP_STATUS_DBG,
};

struct vcp_regs {
	void __iomem *sram;
	void __iomem *cfg;
	void __iomem *cfg_core0;
	void __iomem *cfg_core1;
	int irq_wdt;
	unsigned int total_tcmsize;
	unsigned int vcp_tcmsize;
	unsigned int core_nums;
	unsigned int twohart;
};

struct vcp_reserve_mblock {
	u64 start_phys;		/* address as seen by the VCP */
	void *start_virt;
	u32 size;
};

static struct vcp_regs vcpreg;
static struct device *vcp_dev;
static struct device *vcp_io_devs[VCP_IOMMU_DEV_NUM];
static struct clk *vcpsel, *vcpclk, *vcp26m;

static bool vcp_enable;
static unsigned int vcp_ready;
static unsigned int vcp_generation;
static bool is_suspending;
static int pwclkcnt;
static int vcp_awake_count;
static unsigned int feature_enable[NUM_FEATURE_ID];

static DEFINE_MUTEX(vcp_pw_clk_mutex);
static DEFINE_MUTEX(vcp_feature_mutex);
static DEFINE_MUTEX(vcp_notify_mutex);
static DEFINE_SPINLOCK(vcp_awake_spinlock);
static BLOCKING_NOTIFIER_HEAD(vcp_notifier_list);

static struct vcp_reserve_mblock vcp_mblock[NUMS_MEM_ID];
static void *vcp_mem_virt;
static dma_addr_t vcp_mem_dma;
static size_t vcp_mem_size;

static struct timer_list vcp_ready_timer;
static struct work_struct vcp_ready_work;
static struct workqueue_struct *vcp_wq;

static u32 slp_ipi_ack_data;
static u32 msg_vcp_ready0, msg_vcp_ready1;

static bool boot_test;
module_param(boot_test, bool, 0444);
MODULE_PARM_DESC(boot_test,
		 "Power the VCP and release it from reset at probe (bring-up test)");

static int vcp_clr_spm_reg(void *unused);

struct mtk_mbox_device vcp_mboxdev = {
	.name = "vcp_mboxdev",
	.post_cb = (mbox_rx_cb_t)vcp_clr_spm_reg,
};
EXPORT_SYMBOL_GPL(vcp_mboxdev);

struct mtk_ipi_device vcp_ipidev = {
	.name = "vcp_ipidev",
	.id = IPI_DEV_VCP,
	.mbdev = &vcp_mboxdev,
	.pre_cb = (ipi_tx_cb_t)vcp_awake_lock,
	.post_cb = (ipi_tx_cb_t)vcp_awake_unlock,
	.prdata = 0,
};
EXPORT_SYMBOL_GPL(vcp_ipidev);

static inline u32 core0_rd(u32 ofs)
{
	return readl(vcpreg.cfg_core0 + ofs);
}

static inline u32 core1_rd(u32 ofs)
{
	return vcpreg.cfg_core1 ? readl(vcpreg.cfg_core1 + ofs) : 0;
}

/* --- status API ------------------------------------------------------ */

unsigned int is_vcp_ready(enum vcp_core_id id)
{
	return id == VCP_A_ID ? READ_ONCE(vcp_ready) : 0;
}
EXPORT_SYMBOL_GPL(is_vcp_ready);

unsigned int is_vcp_suspending(void)
{
	return is_suspending;
}
EXPORT_SYMBOL_GPL(is_vcp_suspending);

/* changes whenever the VCP was powered off; users compare it */
unsigned int get_vcp_generation(void)
{
	return vcp_generation;
}
EXPORT_SYMBOL_GPL(get_vcp_generation);

void vcp_A_register_notify(struct notifier_block *nb)
{
	mutex_lock(&vcp_notify_mutex);
	blocking_notifier_chain_register(&vcp_notifier_list, nb);
	if (is_vcp_ready(VCP_A_ID))
		nb->notifier_call(nb, VCP_EVENT_READY, NULL);
	mutex_unlock(&vcp_notify_mutex);
}
EXPORT_SYMBOL_GPL(vcp_A_register_notify);

void vcp_A_unregister_notify(struct notifier_block *nb)
{
	mutex_lock(&vcp_notify_mutex);
	blocking_notifier_chain_unregister(&vcp_notifier_list, nb);
	mutex_unlock(&vcp_notify_mutex);
}
EXPORT_SYMBOL_GPL(vcp_A_unregister_notify);

static void vcp_extern_notify(enum VCP_NOTIFY_EVENT ev)
{
	mutex_lock(&vcp_notify_mutex);
	blocking_notifier_call_chain(&vcp_notifier_list, ev, NULL);
	mutex_unlock(&vcp_notify_mutex);
}

struct device *vcp_get_io_device(enum VCP_IOMMU_DEV io_num)
{
	return io_num < VCP_IOMMU_DEV_NUM ? vcp_io_devs[io_num] : NULL;
}
EXPORT_SYMBOL_GPL(vcp_get_io_device);

/* --- shared memory ---------------------------------------------------- */

/*
 * VDEC, VENC and the logger share one region in this order (downstream layout:
 * the vcodec drivers address VSI buffers relative to VDEC_MEM_ID).
 */
phys_addr_t vcp_get_reserve_mem_phys(enum vcp_reserve_mem_id_t id)
{
	if (id >= NUMS_MEM_ID)
		return 0;
	return vcp_mblock[id].start_phys;
}
EXPORT_SYMBOL_GPL(vcp_get_reserve_mem_phys);

phys_addr_t vcp_get_reserve_mem_virt(enum vcp_reserve_mem_id_t id)
{
	if (id >= NUMS_MEM_ID)
		return 0;
	return (phys_addr_t)(uintptr_t)vcp_mblock[id].start_virt;
}
EXPORT_SYMBOL_GPL(vcp_get_reserve_mem_virt);

phys_addr_t vcp_get_reserve_mem_size(enum vcp_reserve_mem_id_t id)
{
	if (id >= NUMS_MEM_ID)
		return 0;
	return vcp_mblock[id].size;
}
EXPORT_SYMBOL_GPL(vcp_get_reserve_mem_size);

static int vcp_reserve_memory_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int n, i, ret;
	u32 idx, size;
	size_t ofs = 0;

	n = of_property_count_u32_elems(dev->of_node, "vcp-mem-tbl");
	if (n <= 0 || n % 2) {
		dev_err(dev, "vcp-mem-tbl missing or malformed\n");
		return -EINVAL;
	}

	for (i = 0; i < n / 2; i++) {
		of_property_read_u32_index(dev->of_node, "vcp-mem-tbl", 2 * i, &idx);
		of_property_read_u32_index(dev->of_node, "vcp-mem-tbl", 2 * i + 1, &size);
		if (idx >= NUMS_MEM_ID)
			continue;
		/* the secure dump lives in me_vcp_reserved; not used here */
		if (idx == VCP_SECURE_DUMP_ID)
			size = 0;
		vcp_mblock[idx].size = size;
	}

	/* layout: VDEC | VENC | LOGGER contiguous, then the rest page aligned */
	for (i = 0; i < NUMS_MEM_ID; i++) {
		if (!vcp_mblock[i].size)
			continue;
		if (i > VCP_A_LOGGER_MEM_ID)
			ofs = ALIGN(ofs, PAGE_SIZE);
		ofs += vcp_mblock[i].size;
	}
	vcp_mem_size = ALIGN(ofs, PAGE_SIZE);
	if (!vcp_mem_size)
		return 0;

	/* conservative: below 4 GiB while the larbs/IOMMU are in bypass */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	vcp_mem_virt = dmam_alloc_coherent(dev, vcp_mem_size, &vcp_mem_dma,
					   GFP_KERNEL);
	if (!vcp_mem_virt)
		return -ENOMEM;

	ofs = 0;
	for (i = 0; i < NUMS_MEM_ID; i++) {
		if (!vcp_mblock[i].size)
			continue;
		if (i > VCP_A_LOGGER_MEM_ID)
			ofs = ALIGN(ofs, PAGE_SIZE);
		vcp_mblock[i].start_phys = vcp_mem_dma + ofs;
		vcp_mblock[i].start_virt = vcp_mem_virt + ofs;
		ofs += vcp_mblock[i].size;
		dev_dbg(dev, "mem[%d] 0x%llx size 0x%x\n", i,
			vcp_mblock[i].start_phys, vcp_mblock[i].size);
	}

	dev_info(dev, "shared memory %pad size 0x%zx\n", &vcp_mem_dma, vcp_mem_size);
	return 0;
}

/* --- awake / mailbox callbacks ---------------------------------------- */

int vcp_awake_lock(void *_vcp_id)
{
	unsigned long flags;

	if (!is_vcp_ready(VCP_A_ID))
		return -1;
	spin_lock_irqsave(&vcp_awake_spinlock, flags);
	vcp_awake_count++;
	spin_unlock_irqrestore(&vcp_awake_spinlock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(vcp_awake_lock);

int vcp_awake_unlock(void *_vcp_id)
{
	unsigned long flags;

	if (!is_vcp_ready(VCP_A_ID))
		return -1;
	spin_lock_irqsave(&vcp_awake_spinlock, flags);
	if (vcp_awake_count > 0)
		vcp_awake_count--;
	spin_unlock_irqrestore(&vcp_awake_spinlock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(vcp_awake_unlock);

static void vcp_wait_awake_count(void)
{
	unsigned long flags;
	int i;

	for (i = 0; i < 200 && READ_ONCE(vcp_awake_count); i++)
		mdelay(1);
	spin_lock_irqsave(&vcp_awake_spinlock, flags);
	vcp_awake_count = 0;
	spin_unlock_irqrestore(&vcp_awake_spinlock, flags);
}

/* mailbox rx post callback: ack the VCP->SPM wake request */
static int vcp_clr_spm_reg(void *unused)
{
	writel(0x1, vcpreg.cfg + VCP_TO_SPM_REG);
	return 0;
}

/* --- IRQs ------------------------------------------------------------- */

static void vcp_dump_regs(const char *why)
{
	dev_warn(vcp_dev, "%s: status 0x%x pc 0x%x lr 0x%x sp 0x%x hint 0x%x bus 0x%x\n",
		 why, core0_rd(R_CORE_STATUS), core0_rd(R_CORE_MON_PC),
		 core0_rd(R_CORE_MON_LR), core0_rd(R_CORE_MON_SP),
		 readl(vcpreg.cfg + VCP_GPR_DEBUG_HINT),
		 readl(vcpreg.cfg + VCP_BUS_DEBUG_OUT));
}

/*
 * Watchdog: the downstream driver dumps and restarts the VCP. Restart is not
 * ported (as with the SCP port, recovery is left to the user); log and
 * clear the interrupt.
 */
static irqreturn_t vcp_wdt_irq_thread(int irq, void *data)
{
	vcp_dump_regs("watchdog");
	if (core0_rd(R_CORE_WDT_IRQ))
		writel(B_WDT_IRQ, vcpreg.cfg_core0 + R_CORE_WDT_IRQ);
	if (vcpreg.core_nums == 2 && core1_rd(R_CORE_WDT_IRQ))
		writel(B_WDT_IRQ, vcpreg.cfg_core1 + R_CORE_WDT_IRQ);
	WRITE_ONCE(vcp_ready, 0);
	return IRQ_HANDLED;
}

static void vcp_enable_irqs(void)
{
	unsigned int i;

	for (i = 0; i < vcp_mboxdev.count; i++)
		if (vcp_mboxdev.info_table[i].irq_num > 0)
			enable_irq(vcp_mboxdev.info_table[i].irq_num);
	if (vcpreg.irq_wdt > 0)
		enable_irq(vcpreg.irq_wdt);
}

static void vcp_disable_irqs(void)
{
	unsigned int i;

	if (vcpreg.irq_wdt > 0)
		disable_irq(vcpreg.irq_wdt);
	for (i = 0; i < vcp_mboxdev.count; i++)
		if (vcp_mboxdev.info_table[i].irq_num > 0)
			disable_irq(vcp_mboxdev.info_table[i].irq_num);
}

/* --- ready handshake -------------------------------------------------- */

static void vcp_ready_ws(struct work_struct *ws)
{
	writel(0xff, vcpreg.cfg + VCP_TO_SPM_REG);	/* downstream: clear SPM irq */
	WRITE_ONCE(vcp_ready, 1);
	dev_info(vcp_dev, "VCP ready\n");
	vcp_extern_notify(VCP_EVENT_READY);
}

static int vcp_ready_ipi_handler(unsigned int id, void *prdata, void *data,
				 unsigned int len)
{
	u32 image_size = *(u32 *)data;

	timer_delete(&vcp_ready_timer);
	if (!is_vcp_ready(VCP_A_ID))
		queue_work(vcp_wq, &vcp_ready_work);
	if (image_size != vcpreg.vcp_tcmsize)
		dev_warn(vcp_dev, "image size AP 0x%x VCP 0x%x\n",
			 vcpreg.vcp_tcmsize, image_size);
	return 0;
}

static void vcp_ready_timeout(struct timer_list *t)
{
	/* downstream: schedule a reset; here: report only */
	vcp_dump_regs("no ready IPI within 800 ms");
}

/* --- power / reset ---------------------------------------------------- */

static void vcp_set_clk(void)
{
	int ret;

	if (IS_ERR_OR_NULL(vcpsel))
		return;
	/* downstream: park on 26 MHz first so the mux state is in sync */
	if (!IS_ERR_OR_NULL(vcp26m)) {
		ret = clk_set_parent(vcpsel, vcp26m);
		if (ret)
			dev_warn(vcp_dev, "set parent 26m: %d\n", ret);
	}
	if (!IS_ERR_OR_NULL(vcpclk)) {
		ret = clk_set_parent(vcpsel, vcpclk);
		if (ret)
			dev_warn(vcp_dev, "set parent clk: %d\n", ret);
	}
}

static void vcp_wait_core_stop_timeout(void)
{
	u32 want = B_CORE_GATED | B_HART0_HALT |
		   (vcpreg.twohart ? B_HART1_HALT : 0);
	u32 st0 = 0, st1 = 0;
	int timeout = 500;

	while (timeout--) {
		st0 = core0_rd(R_CORE_STATUS);
		st1 = vcpreg.core_nums == 2 ? core1_rd(R_CORE_STATUS) : 0;
		if (st0 == want && (vcpreg.core_nums != 2 ||
		    st1 == (B_CORE_GATED | B_HART0_HALT | B_HART1_HALT)))
			return;
		mdelay(1);
	}
	dev_notice(vcp_dev, "core stop timeout: 0x%x 0x%x\n", st0, st1);
}

static void vcp_wait_ready_to_reboot(void)
{
	int retry;

	for (retry = VCP_AWAKE_TIMEOUT; retry > 0; retry--) {
		if (core0_rd(R_CORE_GENERAL_REG5) == CORE_RDY_TO_REBOOT &&
		    (!vcpreg.twohart ||
		     core0_rd(R_CORE_GENERAL_REG6) == CORE_RDY_TO_REBOOT))
			break;
		udelay(1);
	}
	if (!retry)
		dev_notice(vcp_dev, "reboot handshake timeout, status 0x%x\n",
			   core0_rd(R_CORE_STATUS));
	udelay(10);
}

static void vcp_enable_dapc(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_KERNEL_DAPC_MMUP_CONTROL, 1, 0, 0, 0, 0, 0, 0, &res);
}

static void vcp_disable_dapc(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_TINYSYS_VCP_CONTROL,
		      MTK_TINYSYS_VCP_KERNEL_OP_RESET_SET, 1, 0, 0, 0, 0, 0, &res);
	mb();
	arm_smccc_smc(MTK_SIP_KERNEL_DAPC_MMUP_CONTROL, 0, 0, 0, 0, 0, 0, 0, &res);
}

static void reset_vcp(int reset)
{
	struct arm_smccc_res res;
	u32 addr = VCP_PACK_IOVA((u64)vcp_mem_dma);

	if (!vcp_enable)
		return;

	/* tell the VCP where the AP shared memory is (it sets up its MPU) */
	writel(addr, vcpreg.cfg_core0 + DRAM_RESV_ADDR_REG);
	writel((u32)vcp_mem_size, vcpreg.cfg_core0 + DRAM_RESV_SIZE_REG);
	vcp_set_clk();

	mod_timer(&vcp_ready_timer, jiffies + VCP_READY_TIMEOUT);
	arm_smccc_smc(MTK_SIP_TINYSYS_VCP_CONTROL,
		      MTK_TINYSYS_VCP_KERNEL_OP_RESET_RELEASE,
		      reset == VCP_ALL_SUSPEND ? 0 : 1, 0, 0, 0, 0, 0, &res);
	dev_info(vcp_dev, "reset release: resv 0x%x/0x%x ret %lu\n",
		 core0_rd(DRAM_RESV_ADDR_REG), core0_rd(DRAM_RESV_SIZE_REG), res.a0);
}

static int vcp_power_up(int reset)
{
	int ret;

	ret = clk_prepare_enable(vcp26m);
	if (ret)
		return ret;
	ret = clk_prepare_enable(vcpclk);
	if (ret)
		goto err_26m;
	ret = clk_prepare_enable(vcpsel);
	if (ret)
		goto err_clk;
	ret = pm_runtime_resume_and_get(vcp_dev);
	if (ret)
		goto err_sel;

	vcp_enable_dapc();
	vcp_enable_irqs();
	if (!is_vcp_ready(VCP_A_ID))
		reset_vcp(reset);
	return 0;

err_sel:
	clk_disable_unprepare(vcpsel);
err_clk:
	clk_disable_unprepare(vcpclk);
err_26m:
	clk_disable_unprepare(vcp26m);
	return ret;
}

static void vcp_power_down(void)
{
	vcp_disable_irqs();
	flush_workqueue(vcp_wq);
	WRITE_ONCE(vcp_ready, 0);
	vcp_generation++;

	/* trigger the halt ISR, the VCP parks in wfi */
	writel(B_GIPC3_SETCLR_1, vcpreg.cfg + R_GIPC_IN_SET);
	vcp_wait_ready_to_reboot();
	timer_delete_sync(&vcp_ready_timer);
	vcp_wait_core_stop_timeout();
	vcp_wait_awake_count();
	vcp_disable_dapc();

	pm_runtime_put_sync(vcp_dev);
	clk_disable_unprepare(vcpsel);
	clk_disable_unprepare(vcpclk);
	clk_disable_unprepare(vcp26m);
}

static int vcp_enable_pm_clk(enum feature_id id)
{
	int ret = 0;

	mutex_lock(&vcp_pw_clk_mutex);
	while (is_suspending) {
		mutex_unlock(&vcp_pw_clk_mutex);
		usleep_range(10000, 20000);
		mutex_lock(&vcp_pw_clk_mutex);
	}
	if (pwclkcnt == 0)
		ret = vcp_power_up(VCP_ALL_ENABLE);
	if (!ret)
		pwclkcnt++;
	mutex_unlock(&vcp_pw_clk_mutex);
	return ret;
}

static int vcp_disable_pm_clk(enum feature_id id)
{
	mutex_lock(&vcp_pw_clk_mutex);
	while (is_suspending) {
		mutex_unlock(&vcp_pw_clk_mutex);
		usleep_range(10000, 20000);
		mutex_lock(&vcp_pw_clk_mutex);
	}
	if (pwclkcnt > 0 && --pwclkcnt == 0) {
		vcp_extern_notify(VCP_EVENT_STOP);
		vcp_power_down();
	}
	mutex_unlock(&vcp_pw_clk_mutex);
	return 0;
}

int vcp_register_feature(enum feature_id id)
{
	int ret;

	if (!vcp_enable || id >= NUM_FEATURE_ID)
		return -EINVAL;
	mutex_lock(&vcp_feature_mutex);
	ret = vcp_enable_pm_clk(id);
	if (!ret)
		feature_enable[id]++;
	mutex_unlock(&vcp_feature_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(vcp_register_feature);

int vcp_deregister_feature(enum feature_id id)
{
	int ret = -EINVAL;

	if (!vcp_enable || id >= NUM_FEATURE_ID)
		return -EINVAL;
	mutex_lock(&vcp_feature_mutex);
	if (feature_enable[id]) {
		feature_enable[id]--;
		ret = vcp_disable_pm_clk(id);
	} else {
		pr_warn("vcp: unbalanced deregister of feature %d\n", id);
	}
	mutex_unlock(&vcp_feature_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(vcp_deregister_feature);

static int vcp_pm_event(struct notifier_block *nb, unsigned long ev, void *unused)
{
	switch (ev) {
	case PM_SUSPEND_PREPARE:
		vcp_extern_notify(VCP_EVENT_SUSPEND);
		mutex_lock(&vcp_pw_clk_mutex);
		if (!is_suspending && pwclkcnt)
			vcp_power_down();
		is_suspending = true;
		mutex_unlock(&vcp_pw_clk_mutex);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		mutex_lock(&vcp_pw_clk_mutex);
		if (is_suspending && pwclkcnt)
			vcp_power_up(VCP_ALL_SUSPEND);
		is_suspending = false;
		mutex_unlock(&vcp_pw_clk_mutex);
		vcp_extern_notify(VCP_EVENT_RESUME);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block vcp_pm_nb = {
	.notifier_call = vcp_pm_event,
};

/* --- mailbox tables from DT ------------------------------------------- */

/*
 * send-table: <ipi_id mbox msg_size> triples,
 * recv-table: <ipi_id mbox msg_size recv_opt> quads (downstream binding).
 */
static int vcp_ipi_table_init(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct mtk_mbox_pin_send *ps;
	struct mtk_mbox_pin_recv *pr;
	unsigned int i;
	u32 v[4];
	int n;

	if (of_property_read_u32(np, "mbox-count", &vcp_mboxdev.count) ||
	    !vcp_mboxdev.count)
		return -EINVAL;

	n = of_property_count_u32_elems(np, "send-table");
	if (n <= 0 || n % 3)
		return -EINVAL;
	vcp_mboxdev.send_count = n / 3;

	n = of_property_count_u32_elems(np, "recv-table");
	if (n <= 0 || n % 4)
		return -EINVAL;
	vcp_mboxdev.recv_count = n / 4;

	vcp_mboxdev.info_table = devm_kcalloc(dev, vcp_mboxdev.count,
					      sizeof(*vcp_mboxdev.info_table), GFP_KERNEL);
	vcp_mboxdev.pin_send_table = devm_kcalloc(dev, vcp_mboxdev.send_count,
						  sizeof(*ps), GFP_KERNEL);
	vcp_mboxdev.pin_recv_table = devm_kcalloc(dev, vcp_mboxdev.recv_count,
						  sizeof(*pr), GFP_KERNEL);
	if (!vcp_mboxdev.info_table || !vcp_mboxdev.pin_send_table ||
	    !vcp_mboxdev.pin_recv_table)
		return -ENOMEM;

	for (i = 0; i < vcp_mboxdev.count; i++) {
		vcp_mboxdev.info_table[i].id = i;
		vcp_mboxdev.info_table[i].slot = 64;
		vcp_mboxdev.info_table[i].enable = 1;
		vcp_mboxdev.info_table[i].is64d = 1;
	}

	for (i = 0; i < vcp_mboxdev.send_count; i++) {
		ps = &vcp_mboxdev.pin_send_table[i];
		if (of_property_read_u32_index(np, "send-table", 3 * i, &v[0]) ||
		    of_property_read_u32_index(np, "send-table", 3 * i + 1, &v[1]) ||
		    of_property_read_u32_index(np, "send-table", 3 * i + 2, &v[2]))
			return -EINVAL;
		ps->chan_id = v[0];
		ps->mbox = v[1];
		ps->msg_size = v[2];
	}

	for (i = 0; i < vcp_mboxdev.recv_count; i++) {
		pr = &vcp_mboxdev.pin_recv_table[i];
		if (of_property_read_u32_index(np, "recv-table", 4 * i, &v[0]) ||
		    of_property_read_u32_index(np, "recv-table", 4 * i + 1, &v[1]) ||
		    of_property_read_u32_index(np, "recv-table", 4 * i + 2, &v[2]) ||
		    of_property_read_u32_index(np, "recv-table", 4 * i + 3, &v[3]))
			return -EINVAL;
		pr->chan_id = v[0];
		pr->mbox = v[1];
		pr->msg_size = v[2];
		pr->recv_opt = v[3];
	}
	return 0;
}

/*
 * Slot offset and pin index of every pin of one mailbox: send pins first,
 * then receive pins, each pin taking msg_size words rounded up to 64-bit
 * slots (downstream mbox_setup_pin_table()).
 */
static void vcp_mbox_setup_pin_table(unsigned int mbox)
{
	struct mtk_mbox_info *mi = &vcp_mboxdev.info_table[mbox];
	int last_ofs = 0, last_idx = 0, last_slot = 0, last_sz = 0;
	unsigned int i;

#define VCP_PIN_PLACE(p) do {						\
		(p)->offset = last_ofs + last_slot;			\
		(p)->pin_index = last_idx + last_sz;			\
		last_idx = (p)->pin_index;				\
		if (mi->is64d) {					\
			last_sz = DIV_ROUND_UP((p)->msg_size, 2);	\
			last_ofs = last_sz * 2;				\
			last_slot = last_idx * 2;			\
		} else {						\
			last_sz = (p)->msg_size;			\
			last_ofs = last_sz;				\
			last_slot = last_idx;				\
		}							\
	} while (0)

	for (i = 0; i < vcp_mboxdev.send_count; i++) {
		struct mtk_mbox_pin_send *p = &vcp_mboxdev.pin_send_table[i];

		if (p->mbox == mbox)
			VCP_PIN_PLACE(p);
		else if (mbox < p->mbox)
			break;
	}
	for (i = 0; i < vcp_mboxdev.recv_count; i++) {
		struct mtk_mbox_pin_recv *p = &vcp_mboxdev.pin_recv_table[i];

		if (p->mbox == mbox)
			VCP_PIN_PLACE(p);
		else if (mbox < p->mbox)
			break;
	}
#undef VCP_PIN_PLACE

	if (last_idx > 32 || (last_ofs + last_slot) > (mi->is64d + 1) * 32)
		dev_warn(vcp_dev, "mbox%u ofs(%d)/slot(%d) exceed the maximum\n",
			 mbox, last_idx, last_ofs + last_slot);
}

/* --- sysfs ------------------------------------------------------------ */

static ssize_t vcp_status_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	ssize_t len;

	len = sysfs_emit(buf, "ready %u gen %u users %d suspending %d\n",
			 is_vcp_ready(VCP_A_ID), vcp_generation, pwclkcnt,
			 is_suspending);
	if (pwclkcnt)
		len += sysfs_emit_at(buf, len,
				     "status 0x%x pc 0x%x lr 0x%x resv 0x%x/0x%x\n",
				     core0_rd(R_CORE_STATUS), core0_rd(R_CORE_MON_PC),
				     core0_rd(R_CORE_MON_LR),
				     core0_rd(DRAM_RESV_ADDR_REG),
				     core0_rd(DRAM_RESV_SIZE_REG));
	return len;
}
static DEVICE_ATTR_RO(vcp_status);

/* echo 1 > vcp_rtos: hold the VCP powered (RTOS feature), 0: release */
static ssize_t vcp_rtos_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;
	ret = on ? vcp_register_feature(RTOS_FEATURE_ID) :
		   vcp_deregister_feature(RTOS_FEATURE_ID);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(vcp_rtos);

static struct attribute *vcp_attrs[] = {
	&dev_attr_vcp_status.attr,
	&dev_attr_vcp_rtos.attr,
	NULL,
};
static const struct attribute_group vcp_group = {
	.attrs = vcp_attrs,
};

/* --- probe ------------------------------------------------------------ */

static void __iomem *vcp_ioremap_byname(struct platform_device *pdev,
					const char *name, bool optional,
					unsigned int *size)
{
	struct resource *res;
	void __iomem *base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return optional ? NULL : IOMEM_ERR_PTR(-ENODEV);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (size && !IS_ERR(base))
		*size = resource_size(res);
	return base;
}

static int vcp_main_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *core0 = NULL;
	unsigned int i;
	int ret;

	vcp_dev = dev;
	vcp_io_devs[VCP_IOMMU_256MB1] = dev;

	vcpreg.sram = vcp_ioremap_byname(pdev, "vcp_sram_base", false,
					 &vcpreg.total_tcmsize);
	vcpreg.cfg = vcp_ioremap_byname(pdev, "vcp_cfgreg", false, NULL);
	vcpreg.cfg_core0 = vcp_ioremap_byname(pdev, "vcp_cfgreg_core0", false, NULL);
	vcpreg.cfg_core1 = vcp_ioremap_byname(pdev, "vcp_cfgreg_core1", true, NULL);
	if (IS_ERR(vcpreg.sram) || IS_ERR(vcpreg.cfg) || IS_ERR(vcpreg.cfg_core0) ||
	    IS_ERR(vcpreg.cfg_core1))
		return -ENODEV;

	of_property_read_u32(dev->of_node, "vcp-sram-size", &vcpreg.vcp_tcmsize);
	of_property_read_u32(dev->of_node, "core-nums", &vcpreg.core_nums);
	of_property_read_u32(dev->of_node, "twohart", &vcpreg.twohart);
	if (!vcpreg.vcp_tcmsize || !vcpreg.core_nums)
		return -ENODEV;
	if (!of_property_read_string(dev->of_node, "core-0", &core0) &&
	    !strcmp(core0, "enable"))
		vcp_enable = true;
	if (!vcp_enable) {
		dev_notice(dev, "core-0 not enabled\n");
		return 0;
	}

	vcpsel = devm_clk_get_optional(dev, "mmup-sel");
	vcpclk = devm_clk_get_optional(dev, "mmup-clk");
	vcp26m = devm_clk_get_optional(dev, "mmup-26m");
	if (IS_ERR(vcpsel))
		return dev_err_probe(dev, PTR_ERR(vcpsel), "mmup-sel\n");
	if (IS_ERR(vcpclk))
		return dev_err_probe(dev, PTR_ERR(vcpclk), "mmup-clk\n");
	if (IS_ERR(vcp26m))
		return dev_err_probe(dev, PTR_ERR(vcp26m), "mmup-26m\n");

	vcp_wq = alloc_ordered_workqueue("vcp_wq", 0);
	if (!vcp_wq)
		return -ENOMEM;
	INIT_WORK(&vcp_ready_work, vcp_ready_ws);
	timer_setup(&vcp_ready_timer, vcp_ready_timeout, 0);

	vcpreg.irq_wdt = platform_get_irq_byname_optional(pdev, "wdt");
	if (vcpreg.irq_wdt > 0) {
		ret = devm_request_threaded_irq(dev, vcpreg.irq_wdt, NULL,
						vcp_wdt_irq_thread, IRQF_ONESHOT,
						"vcp-wdt", NULL);
		if (ret)
			vcpreg.irq_wdt = 0;
	}

	ret = vcp_ipi_table_init(dev);
	if (ret)
		goto err_wq;

	for (i = 0; i < vcp_mboxdev.count; i++) {
		vcp_mboxdev.info_table[i].mbdev = &vcp_mboxdev;
		ret = mtk_mbox_probe(pdev, &vcp_mboxdev, i);
		if (ret < 0 || vcp_mboxdev.info_table[i].irq_num < 0) {
			dev_warn(dev, "mbox%u probe fail %d\n", i, ret);
			vcp_mboxdev.info_table[i].irq_num = 0;
			continue;
		}
		vcp_mbox_setup_pin_table(i);
	}

	ret = mtk_ipi_device_register(&vcp_ipidev, pdev, &vcp_mboxdev, VCP_IPI_COUNT);
	if (ret) {
		dev_err(dev, "ipi device register: %d\n", ret);
		goto err_wq;
	}
	mtk_ipi_register(&vcp_ipidev, IPI_OUT_C_SLEEP_0, NULL, NULL, &slp_ipi_ack_data);
	mtk_ipi_register(&vcp_ipidev, IPI_IN_VCP_READY_0, vcp_ready_ipi_handler,
			 NULL, &msg_vcp_ready0);
	mtk_ipi_register(&vcp_ipidev, IPI_IN_VCP_READY_1, vcp_ready_ipi_handler,
			 NULL, &msg_vcp_ready1);

	ret = vcp_reserve_memory_init(pdev);
	if (ret)
		goto err_wq;

	pm_runtime_enable(dev);
	ret = register_pm_notifier(&vcp_pm_nb);
	if (ret)
		goto err_pm;
	ret = devm_device_add_group(dev, &vcp_group);
	if (ret)
		dev_warn(dev, "sysfs: %d\n", ret);

	/* powered off until a user registers a feature */
	vcp_disable_irqs();
	dev_info(dev, "VCP probed: sram 0x%x (fw 0x%x), %u mbox, %u/%u pins\n",
		 vcpreg.total_tcmsize, vcpreg.vcp_tcmsize, vcp_mboxdev.count,
		 vcp_mboxdev.send_count, vcp_mboxdev.recv_count);

	if (boot_test)
		vcp_register_feature(RTOS_FEATURE_ID);
	return 0;

err_pm:
	pm_runtime_disable(dev);
err_wq:
	destroy_workqueue(vcp_wq);
	vcp_wq = NULL;
	return ret;
}

/* "mediatek,vcp-io-*": DMA devices for the vcodec buffers */
static int vcp_io_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	u32 support = 0;
	int ret;

	of_property_read_u32(dev->of_node, "vcp-support", &support);
	if (support < 2 || support > VCP_IOMMU_DEV_NUM)
		return -ENODEV;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	ret = dma_set_max_seg_size(dev, UINT_MAX);
	if (ret)
		return ret;
	vcp_io_devs[support - 1] = dev;
	return 0;
}

static int vcp_probe(struct platform_device *pdev)
{
	u32 support = 0;

	of_property_read_u32(pdev->dev.of_node, "vcp-support", &support);
	if (support == 0) {
		dev_info(&pdev->dev, "vcp-support = 0, VCP not used\n");
		return 0;
	}
	if (support == 1)
		return vcp_main_probe(pdev);
	return vcp_io_probe(pdev);
}

static void vcp_remove(struct platform_device *pdev)
{
	if (&pdev->dev != vcp_dev)
		return;

	mutex_lock(&vcp_pw_clk_mutex);
	if (pwclkcnt) {
		vcp_extern_notify(VCP_EVENT_STOP);
		vcp_power_down();
		pwclkcnt = 0;
	}
	mutex_unlock(&vcp_pw_clk_mutex);
	unregister_pm_notifier(&vcp_pm_nb);
	pm_runtime_disable(&pdev->dev);
	if (vcp_wq)
		destroy_workqueue(vcp_wq);
	vcp_wq = NULL;
	vcp_dev = NULL;
}

static const struct of_device_id vcp_of_ids[] = {
	{ .compatible = "mediatek,vcp" },
	{ .compatible = "mediatek,vcp-io-vdec" },
	{ .compatible = "mediatek,vcp-io-venc" },
	{ .compatible = "mediatek,vcp-io-work" },
	{ .compatible = "mediatek,vcp-io-ube-lat" },
	{ .compatible = "mediatek,vcp-io-ube-core" },
	{ .compatible = "mediatek,vcp-io-sec" },
	{ }
};
MODULE_DEVICE_TABLE(of, vcp_of_ids);

static struct platform_driver mtk_vcp_driver = {
	.probe = vcp_probe,
	.remove = vcp_remove,
	.driver = {
		.name = "mtk-vcp",
		.of_match_table = vcp_of_ids,
	},
};
module_platform_driver(mtk_vcp_driver);

MODULE_DESCRIPTION("MediaTek VCP (video companion processor) host driver");
MODULE_AUTHOR("MediaTek");
MODULE_LICENSE("GPL");
