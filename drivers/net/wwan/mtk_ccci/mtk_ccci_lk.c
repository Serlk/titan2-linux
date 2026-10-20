// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: arguments the bootloader leaves for the modem driver.
 *
 * The stock LK loads and verifies the modem firmware, carves out the
 * modem and shared memory, programs the EMI MPU and the modem address
 * remapping, and describes all of it in a table of named "tags" in a
 * reserved memory block. The location of that table is written into
 * the "ccci,modem_info_v2" property of the modem node.
 *
 * Everything here only reads: the tag area is copied once and unmapped,
 * nothing is written back (the MediaTek downstream driver clears the area for tag
 * formats older than 3; LK on MT6878 uses format 3).
 *
 * Copyright (C) 2015 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "mtk_ccci.h"

static bool mtk_ccci_resv_contains(const struct resource *r, u64 base, u64 size)
{
	u64 end;

	if (!r->end || check_add_overflow(base, size, &end))
		return false;

	return base >= r->start && end - 1 <= r->end;
}

static int mtk_ccci_lk_walk_tags(struct mtk_ccci *ccci)
{
	struct mtk_ccci_lk *lk = &ccci->lk;
	u32 tag_num = le32_to_cpu(lk->info.tag_num);
	u32 off = 0, i;

	for (i = 0; i < tag_num; i++) {
		const struct mtk_ccci_lk_tag *tag;
		u32 data_off, data_size, next, end;

		if (off > lk->tags_size - sizeof(*tag)) {
			dev_err(ccci->dev, "LK tag %u out of bounds\n", i);
			return -EINVAL;
		}
		tag = lk->tags + off;

		if (!memchr(tag->name, '\0', sizeof(tag->name))) {
			dev_err(ccci->dev, "LK tag %u has no name\n", i);
			return -EINVAL;
		}

		data_off = le32_to_cpu(tag->data_offset);
		data_size = le32_to_cpu(tag->data_size);
		if (check_add_overflow(data_off, data_size, &end) ||
		    end > lk->tags_size) {
			dev_err(ccci->dev, "LK tag %s: data out of bounds\n",
				tag->name);
			return -EINVAL;
		}

		next = le32_to_cpu(tag->next_tag_offset);
		if (i + 1 < tag_num && next <= off) {
			dev_err(ccci->dev, "LK tag %s: bad next offset\n",
				tag->name);
			return -EINVAL;
		}
		off = next;
	}

	lk->tag_num = tag_num;
	return 0;
}

/**
 * mtk_ccci_lk_find() - look up an LK tag by name
 * @ccci: driver state
 * @name: tag name
 * @size: returns the size of the tag data
 *
 * Return: the tag data (in the private copy of the tag area) or NULL.
 */
const void *mtk_ccci_lk_find(const struct mtk_ccci *ccci, const char *name,
			     u32 *size)
{
	const struct mtk_ccci_lk *lk = &ccci->lk;
	u32 off = 0, i;

	for (i = 0; i < lk->tag_num; i++) {
		const struct mtk_ccci_lk_tag *tag = lk->tags + off;

		if (!strcmp(tag->name, name)) {
			*size = le32_to_cpu(tag->data_size);
			return lk->tags + le32_to_cpu(tag->data_offset);
		}
		off = le32_to_cpu(tag->next_tag_offset);
	}

	return NULL;
}

static int mtk_ccci_lk_find_u32(const struct mtk_ccci *ccci, const char *name,
				u32 *val)
{
	const __le32 *p;
	u32 size;

	p = mtk_ccci_lk_find(ccci, name, &size);
	if (!p || size < sizeof(*p))
		return -ENOENT;

	*val = le32_to_cpu(*p);
	return 0;
}

static const struct mtk_ccci_smem_region *
mtk_ccci_lk_smem_table(struct mtk_ccci *ccci, const char *name,
		       const char *num_name, unsigned int *num)
{
	const struct mtk_ccci_smem_region *tbl;
	u32 n, size;

	*num = 0;
	if (mtk_ccci_lk_find_u32(ccci, num_name, &n))
		return NULL;

	tbl = mtk_ccci_lk_find(ccci, name, &size);
	if (!tbl || size < n * sizeof(*tbl)) {
		dev_warn(ccci->dev, "LK tag %s: %u entries do not fit\n",
			 name, n);
		return NULL;
	}

	*num = n;
	return tbl;
}

/**
 * mtk_ccci_lk_smem() - look up a shared memory region by SMEM user
 * @ccci: driver state
 * @id: SMEM user
 *
 * Return: the region from the non-cached or the cached table (both checked
 * to lie inside their carveouts), or NULL if LK did not set it up.
 */
const struct mtk_ccci_smem_region *
mtk_ccci_lk_smem(const struct mtk_ccci *ccci, enum mtk_ccci_smem_id id)
{
	const struct mtk_ccci_lk *lk = &ccci->lk;
	unsigned int i;

	for (i = 0; i < lk->nc_smem_num; i++)
		if (le32_to_cpu(lk->nc_smem[i].id) == id &&
		    le32_to_cpu(lk->nc_smem[i].size))
			return &lk->nc_smem[i];
	for (i = 0; i < lk->c_smem_num; i++)
		if (le32_to_cpu(lk->c_smem[i].id) == id &&
		    le32_to_cpu(lk->c_smem[i].size))
			return &lk->c_smem[i];

	return NULL;
}

/* every shared memory region LK describes must lie in its carveout */
static int mtk_ccci_lk_check_smem(struct mtk_ccci *ccci, const char *what,
				  const struct mtk_ccci_smem_region *tbl,
				  unsigned int num, const struct resource *r)
{
	unsigned int i;

	for (i = 0; i < num; i++) {
		u64 base = le64_to_cpu(tbl[i].ap_phys);
		u32 size = le32_to_cpu(tbl[i].size);

		if (!size)
			continue;
		if (!mtk_ccci_resv_contains(r, base, size)) {
			dev_err(ccci->dev,
				"%s smem id %d [%#llx+%#x] outside %pR\n", what,
				le32_to_cpu(tbl[i].id), base, size, r);
			return -EINVAL;
		}
	}

	return 0;
}

/**
 * mtk_ccci_lk_parse() - read and check what LK passed for the modem
 * @ccci: driver state, with the reserved memory regions filled in
 *
 * Return: 0 when LK loaded the modem and its description is consistent,
 * -ENODEV when LK did not load a modem, another error otherwise.
 */
int mtk_ccci_lk_parse(struct mtk_ccci *ccci)
{
	const struct resource *tag_resv = &ccci->resv[MTK_CCCI_RESV_TAGS];
	struct mtk_ccci_lk *lk = &ccci->lk;
	struct device *dev = ccci->dev;
	const void *prop;
	void *map;
	u64 base;
	u32 size;
	int len, ret;

	prop = of_get_property(dev->of_node, "ccci,modem_info_v2", &len);
	if (!prop || len < sizeof(lk->info)) {
		dev_info(dev, "no modem arguments from the bootloader\n");
		return -ENODEV;
	}
	memcpy(&lk->info, prop, sizeof(lk->info));

	base = le64_to_cpu(lk->info.tags_base);
	size = le32_to_cpu(lk->info.tags_size);
	dev_info(dev,
		 "LK: tags %#llx+%#x v%u n=%u ld_flag=%#x err=%d md_err=%d\n",
		 base, size, le32_to_cpu(lk->info.version),
		 le32_to_cpu(lk->info.tag_num), le32_to_cpu(lk->info.ld_flag),
		 (s32)le32_to_cpu(lk->info.err_no),
		 (s32)le32_to_cpu(lk->info.ld_md_errno));

	if (!base || !(le32_to_cpu(lk->info.ld_flag) & MTK_CCCI_LK_LD_MD) ||
	    lk->info.err_no || lk->info.ld_md_errno) {
		dev_info(dev, "LK did not load the modem\n");
		return -ENODEV;
	}

	if (le32_to_cpu(lk->info.version) < 2) {
		dev_err(dev, "LK tag format %u not supported\n",
			le32_to_cpu(lk->info.version));
		return -EINVAL;
	}

	/* never map anything LK did not reserve for exactly this */
	if (size < sizeof(struct mtk_ccci_lk_tag) ||
	    size > MTK_CCCI_LK_TAGS_MAX_SIZE ||
	    !mtk_ccci_resv_contains(tag_resv, base, size)) {
		dev_err(dev, "LK tag area %#llx+%#x not in %pR\n", base, size,
			tag_resv);
		return -EINVAL;
	}

	/*
	 * tags_size counts the bytes LK used; the MediaTek downstream driver maps the
	 * whole 64 KiB window, so allow tag data anywhere in it as long as
	 * it stays inside the carveout.
	 */
	size = min_t(u64, MTK_CCCI_LK_TAGS_MAX_SIZE, tag_resv->end - base + 1);

	/* no-map carveout: memremap() falls back to ioremap_wc() */
	map = memremap(base, size, MEMREMAP_WC);
	if (!map)
		return -ENOMEM;
	lk->tags = devm_kmalloc(dev, size, GFP_KERNEL);
	if (lk->tags)
		memcpy(lk->tags, map, size);
	memunmap(map);
	if (!lk->tags)
		return -ENOMEM;
	lk->tags_size = size;

	ret = mtk_ccci_lk_walk_tags(ccci);
	if (ret)
		return ret;

	lk->md_mem = mtk_ccci_lk_find(ccci, "md_mem_layout", &size);
	lk->md_mem_num = lk->md_mem ? size / sizeof(*lk->md_mem) : 0;

	lk->nc_smem = mtk_ccci_lk_smem_table(ccci, "nc_smem_layout",
					     "nc_smem_layout_num",
					     &lk->nc_smem_num);
	lk->c_smem = mtk_ccci_lk_smem_table(ccci, "c_smem_layout",
					    "c_smem_layout_num",
					    &lk->c_smem_num);
	if (!lk->nc_smem || !lk->c_smem) {
		dev_err(dev, "LK did not describe the shared memory\n");
		return -EINVAL;
	}

	ret = mtk_ccci_lk_check_smem(ccci, "nc", lk->nc_smem, lk->nc_smem_num,
				     &ccci->resv[MTK_CCCI_RESV_NC_SMEM]);
	if (ret)
		return ret;
	ret = mtk_ccci_lk_check_smem(ccci, "c", lk->c_smem, lk->c_smem_num,
				     &ccci->resv[MTK_CCCI_RESV_C_SMEM]);
	if (ret)
		return ret;

	dev_info(dev, "LK: %u tags, %u modem blocks, smem %u nc + %u c\n",
		 lk->tag_num, lk->md_mem_num, lk->nc_smem_num, lk->c_smem_num);
	return 0;
}

static int mtk_ccci_lk_tags_show(struct seq_file *s, void *unused)
{
	const struct mtk_ccci *ccci = s->private;
	const struct mtk_ccci_lk *lk = &ccci->lk;
	u32 off = 0, i;

	for (i = 0; i < lk->tag_num; i++) {
		const struct mtk_ccci_lk_tag *tag = lk->tags + off;
		u32 size = le32_to_cpu(tag->data_size);
		const u8 *data = lk->tags + le32_to_cpu(tag->data_offset);

		seq_printf(s, "%-32s %5u %*ph\n", tag->name, size,
			   (int)min(size, 16U), data);
		off = le32_to_cpu(tag->next_tag_offset);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_ccci_lk_tags);

static void mtk_ccci_smem_show(struct seq_file *s, const char *what,
			       const struct mtk_ccci_smem_region *tbl,
			       unsigned int num)
{
	unsigned int i;

	for (i = 0; i < num; i++)
		seq_printf(s, "%-2s id %2d ap %#011llx md +%#09x size %#09x flags %#x\n",
			   what, le32_to_cpu(tbl[i].id),
			   le64_to_cpu(tbl[i].ap_phys),
			   le32_to_cpu(tbl[i].md_offset),
			   le32_to_cpu(tbl[i].size),
			   le32_to_cpu(tbl[i].flags));
}

static int mtk_ccci_lk_layout_show(struct seq_file *s, void *unused)
{
	const struct mtk_ccci *ccci = s->private;
	const struct mtk_ccci_lk *lk = &ccci->lk;
	unsigned int i;

	for (i = 0; i < lk->md_mem_num; i++)
		seq_printf(s, "md ap %#011llx md %#010x size %#010x inf %#x attr %#x\n",
			   le64_to_cpu(lk->md_mem[i].ap_phys),
			   le32_to_cpu(lk->md_mem[i].md_offset),
			   le32_to_cpu(lk->md_mem[i].size),
			   le32_to_cpu(lk->md_mem[i].inf_flag),
			   le32_to_cpu(lk->md_mem[i].attr_flag));
	mtk_ccci_smem_show(s, "nc", lk->nc_smem, lk->nc_smem_num);
	mtk_ccci_smem_show(s, "c", lk->c_smem, lk->c_smem_num);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_ccci_lk_layout);

void mtk_ccci_lk_debugfs_init(struct mtk_ccci *ccci)
{
	debugfs_create_file("lk_tags", 0400, ccci->debugfs, ccci,
			    &mtk_ccci_lk_tags_fops);
	debugfs_create_file("lk_layout", 0400, ccci->debugfs, ccci,
			    &mtk_ccci_lk_layout_fops);
}
