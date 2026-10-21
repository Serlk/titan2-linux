// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: DPMAIF v3 data path, skeleton.
 *
 * DPMAIF moves network packets between the AP and the modem with DMA
 * descriptor rings in AP memory, the way the t7xx DPMAIF does:
 *
 *  - DL (modem -> AP), per RX queue: a BAT ring of buffers the AP lends
 *    the hardware and a PIT ring where the hardware reports what it put
 *    where (normal PIT: buffer id and length; message PIT: CCMNI
 *    channel, i.e. which network device, and checksum status);
 *  - UL (AP -> modem), per TX queue: a DRB ring of a message DRB
 *    (channel, length) followed by payload DRBs (address, length).
 *
 * MT6878 has DPMAIF v3: 2 RX and 4 TX queues, 16-byte PITs. This file
 * only describes the descriptors and allocates the rings; the hardware
 * setup (vendor ccci_dpmaif_drv_v3.c: SRAM init, bus remap, BAT/PIT/DRB
 * base registers, interrupt masks) is not written yet, so the data path
 * never starts. Nothing here runs unless the module parameter data=1.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2021 Intel Corporation (t7xx DPMAIF)
 * Copyright (C) 2026 serlk
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "mtk_ccci.h"

static bool data;
module_param(data, bool, 0444);
MODULE_PARM_DESC(data, "set up the DPMAIF data path skeleton (default: no; it does not start)");

#define DPMAIF_RXQ_NUM		2
#define DPMAIF_TXQ_NUM		4
#define DPMAIF_BAT_CNT		16384
#define DPMAIF_PIT_CNT		(DPMAIF_BAT_CNT * 2)
#define DPMAIF_DRB_CNT		2048
#define DPMAIF_BUF_SIZE		1638	/* vendor DPMAIF_PKT_SIZE, 128-aligned */

/* normal PIT, v3 (16 bytes) */
#define PIT0_PACKET_TYPE	BIT(0)		/* 0: data, 1: message */
#define PIT0_CONT		BIT(1)		/* more PITs for this packet */
#define PIT0_BUF_TYPE		BIT(2)		/* 1: fragment BAT */
#define PIT0_BUF_ID		GENMASK(15, 3)
#define PIT0_DATA_LEN		GENMASK(31, 16)
/* word 1: buffer address low, word 2: high */
#define PIT3_SEQ		GENMASK(7, 0)
#define PIT3_HEADER_OFFSET	GENMASK(23, 19)
#define PIT3_ULQ_DONE		GENMASK(29, 24)
#define PIT3_DLQ_DONE		GENMASK(31, 30)

/* message PIT, v3 */
#define MPIT0_CHECKSUM		GENMASK(3, 2)
#define MPIT0_ERROR		BIT(4)
#define MPIT0_SRC_QID		GENMASK(7, 5)
#define MPIT0_CHANNEL_ID	GENMASK(23, 16)	/* CCMNI index */
#define MPIT0_NETWORK_TYPE	GENMASK(26, 24)
#define MPIT1_COUNT_L		GENMASK(15, 0)

struct dpmaif_pit {
	__le32 w[4];
};

/* BAT (8 bytes): buffer address low, high byte */
struct dpmaif_bat {
	__le32 addr_lo;
	__le32 addr_hi;
};

/* DRB (8 bytes) */
#define DRB0_TYPE		GENMASK(1, 0)	/* 0: payload, 1: message */
#define DRB0_CONT		BIT(2)
#define DRB0_ADDR_EXT		GENMASK(15, 8)	/* payload: address bits 39:32 */
#define DRB0_DATA_LEN		GENMASK(31, 16)	/* payload / message: length */
#define DRB1_CHANNEL_ID		GENMASK(23, 16)	/* message: CCMNI index */
#define DRB1_NETWORK_TYPE	GENMASK(26, 24)
#define DRB1_IPV4_CSUM		BIT(28)
#define DRB1_L4_CSUM		BIT(29)

struct dpmaif_drb {
	__le32 w[2];
};

struct dpmaif_ring {
	void *base;
	dma_addr_t dma;
	size_t size;
};

struct mtk_ccci_dpmaif {
	struct mtk_ccci *ccci;
	/* register windows (stock node dpmaif@10014000) */
	void __iomem *ao_ul;
	void __iomem *pd_ul;
	void __iomem *pd_md_misc;
	void __iomem *sram;
	struct dpmaif_ring bat[DPMAIF_RXQ_NUM];
	struct dpmaif_ring pit[DPMAIF_RXQ_NUM];
	struct dpmaif_ring drb[DPMAIF_TXQ_NUM];
};

static int dpmaif_ring_alloc(struct device *dev, struct dpmaif_ring *r,
			     size_t size)
{
	r->size = size;
	r->base = dmam_alloc_coherent(dev, size, &r->dma, GFP_KERNEL);
	return r->base ? 0 : -ENOMEM;
}

/**
 * mtk_ccci_dpmaif_init() - map DPMAIF and allocate its rings
 * @ccci: driver state
 *
 * Only with data=1 and the DPMAIF windows described ("dpmaif-ao-ul",
 * "dpmaif-pd-ul", "dpmaif-pd-md-misc", "dpmaif-sram"). Touches no
 * register.
 *
 * Return: 0 (also when skipped) or a negative error.
 */
int mtk_ccci_dpmaif_init(struct mtk_ccci *ccci)
{
	struct platform_device *pdev = to_platform_device(ccci->dev);
	struct device *dev = ccci->dev;
	struct mtk_ccci_dpmaif *dp;
	unsigned int i;
	int ret;

	BUILD_BUG_ON(sizeof(struct dpmaif_pit) != 16);
	BUILD_BUG_ON(sizeof(struct dpmaif_bat) != 8);
	BUILD_BUG_ON(sizeof(struct dpmaif_drb) != 8);

	if (!data)
		return 0;
	if (!platform_get_resource_byname(pdev, IORESOURCE_MEM, "dpmaif-ao-ul")) {
		dev_info(dev, "no DPMAIF in the DT, no data path\n");
		return 0;
	}

	dp = devm_kzalloc(dev, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;
	dp->ccci = ccci;

	dp->ao_ul = devm_platform_ioremap_resource_byname(pdev, "dpmaif-ao-ul");
	if (IS_ERR(dp->ao_ul))
		return PTR_ERR(dp->ao_ul);
	dp->pd_ul = devm_platform_ioremap_resource_byname(pdev, "dpmaif-pd-ul");
	if (IS_ERR(dp->pd_ul))
		return PTR_ERR(dp->pd_ul);
	dp->pd_md_misc = devm_platform_ioremap_resource_byname(pdev,
							"dpmaif-pd-md-misc");
	if (IS_ERR(dp->pd_md_misc))
		return PTR_ERR(dp->pd_md_misc);
	dp->sram = devm_platform_ioremap_resource_byname(pdev, "dpmaif-sram");
	if (IS_ERR(dp->sram))
		return PTR_ERR(dp->sram);

	/* the MediaTek downstream driver uses a 36-bit DMA mask */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		return ret;

	for (i = 0; i < DPMAIF_RXQ_NUM; i++) {
		ret = dpmaif_ring_alloc(dev, &dp->bat[i],
					DPMAIF_BAT_CNT * sizeof(struct dpmaif_bat));
		if (ret)
			return ret;
		ret = dpmaif_ring_alloc(dev, &dp->pit[i],
					DPMAIF_PIT_CNT * sizeof(struct dpmaif_pit));
		if (ret)
			return ret;
	}
	for (i = 0; i < DPMAIF_TXQ_NUM; i++) {
		ret = dpmaif_ring_alloc(dev, &dp->drb[i],
					DPMAIF_DRB_CNT * sizeof(struct dpmaif_drb));
		if (ret)
			return ret;
	}

	ccci->dpmaif = dp;
	dev_info(dev, "DPMAIF rings allocated; the data path is not implemented\n");
	return mtk_ccci_net_init(ccci);
}

/**
 * mtk_ccci_dpmaif_start() - start the data path, once the modem is ready
 * @ccci: driver state
 *
 * TODO: the v3 hardware setup (vendor drv3_* sequence: SRAM init, bus
 * remap, BAT/PIT/DRB base and size registers, BAT fill, interrupts).
 *
 * Return: -EOPNOTSUPP.
 */
int mtk_ccci_dpmaif_start(struct mtk_ccci *ccci)
{
	return ccci->dpmaif ? -EOPNOTSUPP : 0;
}

void mtk_ccci_dpmaif_stop(struct mtk_ccci *ccci)
{
}
