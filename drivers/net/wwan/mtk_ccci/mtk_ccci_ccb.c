// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: the CCB ("CCCI buffer") rings of the modem loggers.
 *
 * The modem's DHL logger, its monitor (MDM) and META talk to the AP
 * through rings in shared memory: the data in the CCB data regions, the
 * ring headers in the 4 KiB CCB control area (SMEM user CCB_CTRL). One
 * header pair (DL: modem to AP, UL: AP to modem) of 64 bytes per buffer,
 * buffer i at i * 0x40, in the order of the table below.
 *
 * Before the modem is started the AP fills in each pair: indexes zero,
 * the page and buffer sizes of both directions and the end guard
 * 0xEEFF0011 ("AP initialised"). When the modem takes a buffer over it
 * writes 0xDDBBCCAA into the start guards. Nothing else is signalled: no
 * CCIF channel, no message. A logger attached later (not done here)
 * would set the end guards to 0x1100FFEE.
 *
 * The table is the one of MediaTek's GPL downstream eccci driver
 * (port_smem.c); the headers the modem expects to find before its first
 * start are written here by the kernel.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/align.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/sizes.h>

#include "mtk_ccci.h"

/*
 * The headers are written on the first modem start after
 * the AP booted and never again: on a restart the modem finds the headers
 * as it left them (start guards set). Whether it would also accept fresh
 * ones is not known, so that is the default.
 */
static bool ccb_init_every_start;
module_param(ccb_init_every_start, bool, 0644);
MODULE_PARM_DESC(ccb_init_every_start,
		 "write the CCB ring headers on every modem start (default: first start only)");

#define MTK_CCCI_CCB_P_CORE		0

/* header pair of one buffer in the control area (downstream buffer_header) */
#define MTK_CCCI_CCB_HDR_SIZE		0x40
#define MTK_CCCI_CCB_DL			0x00
#define MTK_CCCI_CCB_UL			0x20
/* within one direction */
#define MTK_CCCI_CCB_GUARD_START	0x00	/* the modem: 0xDDBBCCAA */
#define MTK_CCCI_CCB_PAGE_SIZE		0x14
#define MTK_CCCI_CCB_BUF_SIZE		0x18	/* bytes, not pages */
#define MTK_CCCI_CCB_GUARD_END		0x1c
#define MTK_CCCI_CCB_DIR_SIZE		0x20

#define MTK_CCCI_CCB_AP_INIT		0xeeff0011

/* one CCB buffer: DL is modem to AP, UL is AP to modem */
struct mtk_ccci_ccb_config {
	u8 user;		/* SMEM user of the data: CCB_DHL..CCB_META */
	u8 core;
	u32 dl_page_size;
	u32 ul_page_size;
	u32 dl_buf_size;
	u32 ul_buf_size;
};

#define CCB_CFG(u, dlp, ulp, dlb, ulb) {				\
	.user = MTK_CCCI_SMEM_##u, .core = MTK_CCCI_CCB_P_CORE,		\
	.dl_page_size = (dlp), .ul_page_size = (ulp),			\
	.dl_buf_size = (dlb), .ul_buf_size = (ulb) }

/* the buffers of one user must follow each other */
static const struct mtk_ccci_ccb_config mtk_ccci_ccb_configs[] = {
	CCB_CFG(CCB_DHL, 1024, 1024, 0x8000, 0x8000),		/* control */
	CCB_CFG(CCB_DHL, 20480, 128, 0x1e000, 0x80),		/* exception */
	CCB_CFG(CCB_DHL, 512, 1024, 0x9000, 0x8000),		/* PS */
	CCB_CFG(CCB_DHL, 512, 128, 0x8000, 0x80),		/* HW logger 1 */
	CCB_CFG(CCB_DHL, 512, 128, 0x8000, 0x80),		/* HW logger 2 */
	CCB_CFG(CCB_DHL, 512, 128, 0x20000, 0x80),		/* HW logger 3 */
	/* dummy buffers 6..16 */
	CCB_CFG(CCB_DHL, 128, 128, 0x80, 0x80),
	CCB_CFG(CCB_DHL, 128, 128, 0x80, 0x80),
	CCB_CFG(CCB_DHL, 128, 128, 0x80, 0x80),
	CCB_CFG(CCB_DHL, 128, 128, 0x80, 0x80),
	CCB_CFG(CCB_DHL, 128, 128, 0x80, 0x80),
	CCB_CFG(CCB_DHL, 128, 128, 0x80, 0x80),
	CCB_CFG(CCB_DHL, 128, 128, 0x80, 0x80),
	CCB_CFG(CCB_DHL, 128, 128, 0x80, 0x80),
	CCB_CFG(CCB_DHL, 128, 128, 0x80, 0x80),
	CCB_CFG(CCB_DHL, 128, 128, 0x80, 0x80),
	CCB_CFG(CCB_DHL, 128, 128, 0x80, 0x80),
	CCB_CFG(CCB_DHL, 128, 128, 0x280, 0x80),		/* padding */
	CCB_CFG(CCB_MD_MONITOR, 512, 1024, 0x4000, 0x4000),	/* MDM */
	CCB_CFG(CCB_META, 66560, 66560, 0x82000, 0x82000),	/* META */
};

static void mtk_ccci_ccb_dir_init(void __iomem *p, u32 page_size,
				  u32 buf_size)
{
	unsigned int off;

	/* the indexes and the start guard first */
	for (off = 0; off < MTK_CCCI_CCB_DIR_SIZE; off += 4)
		writel(0, p + off);
	writel(page_size, p + MTK_CCCI_CCB_PAGE_SIZE);
	writel(buf_size, p + MTK_CCCI_CCB_BUF_SIZE);
	writel(MTK_CCCI_CCB_AP_INIT, p + MTK_CCCI_CCB_GUARD_END);
}

/**
 * mtk_ccci_ccb_init() - write the CCB ring headers before a modem start
 * @ccci: driver state, the modem not running
 *
 * Writes the header pair of every buffer of the table into the CCB
 * control area; the rest of the area is left alone. Only on the first
 * start unless the module parameter ccb_init_every_start is set.
 *
 * Return: 0 or a negative error. The modem may still be started
 * whether this worked or not.
 */
int mtk_ccci_ccb_init(struct mtk_ccci *ccci)
{
	const struct mtk_ccci_smem_region *r;
	const struct mtk_ccci_ccb_config *c;
	u32 size = ARRAY_SIZE(mtk_ccci_ccb_configs) * MTK_CCCI_CCB_HDR_SIZE;
	void __iomem *p;
	unsigned int i;

	if (ccci->ccb_ctrl_init && !ccb_init_every_start)
		return 0;

	r = mtk_ccci_lk_smem(ccci, MTK_CCCI_SMEM_CCB_CTRL);
	if (!r)
		return -ENODEV;
	if (le32_to_cpu(r->size) < size) {
		dev_err(ccci->dev, "CCB control area of %#x bytes, %#x needed\n",
			le32_to_cpu(r->size), size);
		return -ENOSPC;
	}

	/* inside the no-map shared memory carveout */
	p = ioremap_wc(le64_to_cpu(r->ap_phys), size);
	if (!p)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(mtk_ccci_ccb_configs); i++) {
		void __iomem *hdr = p + i * MTK_CCCI_CCB_HDR_SIZE;

		c = &mtk_ccci_ccb_configs[i];
		mtk_ccci_ccb_dir_init(hdr + MTK_CCCI_CCB_DL, c->dl_page_size,
				      c->dl_buf_size);
		mtk_ccci_ccb_dir_init(hdr + MTK_CCCI_CCB_UL, c->ul_page_size,
				      c->ul_buf_size);
	}
	/* all of it in memory before the modem is released */
	wmb();
	iounmap(p);

	if (!ccci->ccb_ctrl_init)
		dev_info(ccci->dev, "CCB: %zu ring headers, %#x bytes at %#llx\n",
			 ARRAY_SIZE(mtk_ccci_ccb_configs), size,
			 le64_to_cpu(r->ap_phys));
	ccci->ccb_ctrl_init = true;
	return 0;
}

/**
 * mtk_ccci_ccb_data_size() - the CCB data the buffers of the table need
 *
 * The users DHL, MDM and META back to back from the start of the CCB data,
 * each the sum of its DL and UL buffers rounded up to 4 KiB, as the
 * downstream driver places them (0x17c000 with the table above).
 *
 * Return: the size in bytes.
 */
u32 mtk_ccci_ccb_data_size(void)
{
	const struct mtk_ccci_ccb_config *c;
	u32 total = 0, user_size = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_ccci_ccb_configs); i++) {
		c = &mtk_ccci_ccb_configs[i];
		user_size += c->dl_buf_size + c->ul_buf_size;
		if (i + 1 == ARRAY_SIZE(mtk_ccci_ccb_configs) ||
		    mtk_ccci_ccb_configs[i + 1].user != c->user) {
			total += ALIGN(user_size, SZ_4K);
			user_size = 0;
		}
	}
	return total;
}
