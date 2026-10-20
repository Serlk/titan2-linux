/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek CCCI (Cross Core Communication Interface) for the modem
 * built into MediaTek smartphone SoCs.
 *
 * The data layouts handed over by the bootloader follow the MediaTek
 * ccci_util driver of its GPL downstream kernel.
 *
 * Copyright (C) 2015 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#ifndef __MTK_CCCI_H__
#define __MTK_CCCI_H__

#include <linux/build_bug.h>
#include <linux/ioport.h>
#include <linux/types.h>

struct dentry;
struct device;

/*
 * "ccci,modem_info_v2" in the modem node, written by the bootloader (LK):
 * where LK left its table of named arguments ("tags"). Little endian, as
 * LK copies the structure into the property without byte swapping.
 */
struct mtk_ccci_lk_info {
	__le64 tags_base;	/* physical address of the tag area */
	__le32 tags_size;	/* bytes used in the tag area */
	__le32 err_no;		/* LK load error, 0 = none */
	__le32 version;		/* tag format; >= 2: 64-byte names */
	__le32 tag_num;
	__le32 ld_flag;		/* bit 0: modem image loaded */
	__le32 ld_md_errno;
};
static_assert(sizeof(struct mtk_ccci_lk_info) == 32);

#define MTK_CCCI_LK_LD_MD		BIT(0)
#define MTK_CCCI_LK_TAGS_MAX_SIZE	0x10000
#define MTK_CCCI_LK_TAG_NAME_LEN	64

/* one entry of the LK tag area (format version 2 and later) */
struct mtk_ccci_lk_tag {
	char name[MTK_CCCI_LK_TAG_NAME_LEN];
	__le32 data_offset;	/* from the start of the tag area */
	__le32 data_size;
	__le32 next_tag_offset;	/* from the start of the tag area */
};
static_assert(sizeof(struct mtk_ccci_lk_tag) == 76);

/* tag "md_mem_layout": modem memory blocks as LK set them up */
struct mtk_ccci_md_mem_blk {
	__le32 md_offset;	/* modem view, from the modem bank 0 base */
	__le32 size;
	__le32 inf_flag;
	__le32 attr_flag;
	__le64 ap_phys;
};
static_assert(sizeof(struct mtk_ccci_md_mem_blk) == 24);

/* tags "nc_smem_layout" and "c_smem_layout": AP/modem shared memory */
struct mtk_ccci_smem_region {
	__le64 ap_phys;
	__le64 ap_virt;		/* LK's own mapping, meaningless here */
	__le32 id;		/* SMEM user, see enum mtk_ccci_smem_id */
	__le32 ap_offset;	/* from the start of the shared memory block */
	__le32 size;
	__le32 align;
	__le32 flags;
	__le32 md_offset;	/* modem view of ap_offset */
};
static_assert(sizeof(struct mtk_ccci_smem_region) == 40);

#define MTK_CCCI_SMEM_F_PADDING		BIT(2)

/* the SMEM users stage 1 cares about (vendor enum SMEM_USER_ID) */
enum mtk_ccci_smem_id {
	MTK_CCCI_SMEM_DBM = 0,
	MTK_CCCI_SMEM_CCB_DHL = 1,
	MTK_CCCI_SMEM_CCB_CTRL = 4,
	MTK_CCCI_SMEM_RUNTIME_DATA = 14,
	MTK_CCCI_SMEM_FORCE_ASSERT = 15,
	MTK_CCCI_SMEM_CCISM_SCP = 16,
	MTK_CCCI_SMEM_MD_CONSYS = 22,
	MTK_CCCI_SMEM_WIFI_PROXY = 31,
	MTK_CCCI_SMEM_NVRAM_CACHE = 32,
	MTK_CCCI_SMEM_LOW_POWER = 33,
	MTK_CCCI_SMEM_DRDI = 41,
};

/* reserved memory LK sets aside for the modem (nodes "mblock-N-<name>") */
enum mtk_ccci_resv {
	MTK_CCCI_RESV_TAGS,	/* "mediatek,ccci_tag_mem" */
	MTK_CCCI_RESV_NC_SMEM,	/* "mediatek,ap_md_nc_smem" */
	MTK_CCCI_RESV_C_SMEM,	/* "mediatek,ap_md_c_smem" */
	MTK_CCCI_RESV_NUM,
};

#define MTK_CCCI_MD_MEM_MAX	8	/* "mediatek,md_mem_usage" nodes */

struct mtk_ccci_lk {
	struct mtk_ccci_lk_info info;
	void *tags;		/* private copy of the tag area */
	u32 tags_size;
	u32 tag_num;		/* tags that passed the checks */

	const struct mtk_ccci_md_mem_blk *md_mem;
	unsigned int md_mem_num;
	const struct mtk_ccci_smem_region *nc_smem;
	unsigned int nc_smem_num;
	const struct mtk_ccci_smem_region *c_smem;
	unsigned int c_smem_num;
};

struct mtk_ccci {
	struct device *dev;

	struct resource resv[MTK_CCCI_RESV_NUM];
	struct resource md_mem[MTK_CCCI_MD_MEM_MAX];
	unsigned int md_mem_num;

	struct mtk_ccci_lk lk;

	void __iomem *ccif_ap;	/* AP side of CCIF0; never written in stage 1 */
	u32 ccif_sram_size;

	struct dentry *debugfs;
};

int mtk_ccci_lk_parse(struct mtk_ccci *ccci);
const void *mtk_ccci_lk_find(const struct mtk_ccci *ccci, const char *name,
			     u32 *size);
void mtk_ccci_lk_debugfs_init(struct mtk_ccci *ccci);

#endif /* __MTK_CCCI_H__ */
