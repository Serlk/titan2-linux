// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: the runtime data, the AP answer to HS1.
 *
 * With HS1 the modem lists the runtime features it knows (64 feature
 * bytes: support level and version). The AP answers in two places:
 *
 *  - the RUNTIME_DATA shared memory (first 2 KiB, the AP half): one record
 *    per feature, in feature order, with the support level agreed on and
 *    the data the feature needs (where a shared memory region is, the
 *    time, a random seed...);
 *  - the CCIF SRAM, after the modem's query: a control header and the AP
 *    feature block, which tells the modem where the runtime data is and
 *    which shared memory it may use (its MPU windows). CCIF channel 15
 *    then tells the modem the answer is there.
 *
 * The modem then asks the AP for files (FS) and board data (RPC) and
 * reports HS2 on the control channel: it is ready.
 *
 * The agreement and the records follow the MediaTek downstream driver
 * (ccci_fsm.c: config_ap_side_feature, ccci_md_prepare_runtime_data;
 * modem_sys1.c: config_ap_runtime_data_v2_1), the values of a normal
 * boot of the Titan 2 are the defaults of the
 * module parameters below. Where the data comes from LK, a missing tag is
 * treated as the downstream driver does.
 *
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <linux/timekeeping.h>

#include <asm/arch_timer.h>

#include "mtk_ccci.h"

static bool runtime_data;
module_param(runtime_data, bool, 0444);
MODULE_PARM_DESC(runtime_data,
		 "answer HS1 and let the modem boot to READY (default: stop at HS1)");

/* what a normal boot passes to the modem */
static uint sbp_code;
module_param(sbp_code, uint, 0444);
MODULE_PARM_DESC(sbp_code, "operator (SBP) code given to the modem (default 0)");

static uint rat_bitmap = 0x7d;
module_param(rat_bitmap, uint, 0444);
MODULE_PARM_DESC(rat_bitmap,
		 "radio technologies the modem may use, modem bit order (default 0x7d: N/C/Lf/Lt/W/G)");

static bool ccb_data_lk;
module_param(ccb_data_lk, bool, 0444);
MODULE_PARM_DESC(ccb_data_lk,
		 "announce the whole CCB data LK set aside (default: what the CCB buffers use)");

/* modem side RAT bits (0 | N | Lf | Lt | W | C | T | G) */
#define MTK_CCCI_RAT_CDMA2000		BIT(2)

/* runtime feature ids (downstream MD_CCCI_RUNTIME_FEATURE_ID) */
enum {
	FT_BOOT_INFO = 0,
	FT_EXCEPTION_SHARE_MEMORY = 1,
	FT_CCIF_SHARE_MEMORY = 2,
	FT_SMART_LOGGING_SHARE_MEMORY = 3,
	FT_MD1MD3_SHARE_MEMORY = 4,
	FT_MISC_INFO_HIF_DMA_REMAP = 5,
	FT_MISC_INFO_RTC_32K_LESS = 6,
	FT_MISC_INFO_RANDOM_SEED_NUM = 7,
	FT_MISC_INFO_GPS_COCLOCK = 8,
	FT_MISC_INFO_SBP_ID = 9,
	FT_MISC_INFO_CCCI = 10,
	FT_MISC_INFO_CLIB_TIME = 11,
	FT_MISC_INFO_C2K = 12,
	FT_MD_IMAGE_START_MEMORY = 13,
	FT_CCISM_SHARE_MEMORY = 14,
	FT_CCB_SHARE_MEMORY = 15,
	FT_DHL_RAW_SHARE_MEMORY = 16,
	FT_DT_NETD_SHARE_MEMORY = 17,
	FT_DT_USB_SHARE_MEMORY = 18,
	FT_EE_AFTER_EPOF = 19,
	FT_AP_CCMNI_MTU = 20,
	FT_MISC_INFO_CUSTOMER_VAL = 21,
	FT_CCCI_FAST_HEADER = 22,
	FT_MISC_INFO_C2K_MEID = 24,
	FT_LWA_SHARE_MEMORY = 25,
	FT_AUDIO_RAW_SHARE_MEMORY = 26,
	FT_MULTI_MD_MPU = 27,
	FT_CCISM_SHARE_MEMORY_EXP = 28,
	FT_MD_PHY_CAPTURE = 29,
	FT_MD_CONSYS_SHARE_MEMORY = 30,
	FT_MD_USIP_SHARE_MEMORY = 31,
	FT_MD_MTEE_SHARE_MEMORY_ENABLE = 32,
	FT_MD_POS_SHARE_MEMORY = 33,
	FT_UDC_RAW_SHARE_MEMORY = 34,
	FT_MD_WIFI_PROXY_SHARE_MEMORY = 35,
	FT_NVRAM_CACHE_SHARE_MEMORY = 36,
	FT_SECURITY_SHARE_MEMORY = 37,
	FT_MD_MEM_AP_VIEW_INF = 38,
	FT_AMMS_DRDI_COPY = 45,
	FT_MD_POST_DUMP = 55,
};

/* version of MISC_INFO_CLIB_TIME with microseconds and the AP counter */
#define FT_CLIB_TIME_HIRES		1

/* BOOT_INFO */
#define MTK_CCCI_BOOT_MODE_NORMAL	0
#define MTK_CCCI_BOOT_ATTR_PHONE	BIT(1)	/* not an EVB */

/* MISC_INFO_CCCI */
#define MTK_CCCI_MISC_SEQ_CHECK		BIT(0)
#define MTK_CCCI_MISC_POLL_MD_STATUS	BIT(1)

/* MISC_INFO_C2K */
#define MTK_CCCI_C2K_ON			BIT(2)

/* the exception area the modem fills in MDCCCI_DBG */
#define MTK_CCCI_EE_SMEM_SIZE		SZ_64K

/* largest packet the AP takes on a CCMNI port, header excluded */
#define MTK_CCCI_CCMNI_MTU		(MTK_CCCI_MTU + 128 - \
					 sizeof(struct mtk_ccci_header))

/* the second half of RUNTIME_DATA is the modem's */
#define MTK_CCCI_RT_MD_SIZE		0x800

/* share_memory_support: an AP that sets the MPU windows of the modem */
#define MTK_CCCI_SHM_MULTI_MD_MPU	2

/* one runtime data record, followed by data_len bytes */
struct mtk_ccci_rt_feature {
	u8 feature_id;
	u8 support;		/* MTK_CCCI_FT_MASK | MTK_CCCI_FT_VER */
	u8 reserved[2];
	__le32 data_len;
};
static_assert(sizeof(struct mtk_ccci_rt_feature) == 8);

struct mtk_ccci_rt_shm {
	__le32 addr;		/* modem view */
	__le32 size;
};

/* the AP feature block in the CCIF SRAM (downstream ap_query_md_feature_v2_1) */
struct mtk_ccci_ap_feature {
	__le32 head_pattern;
	u8 feature[MTK_CCCI_FEATURE_COUNT];
	__le32 share_memory_support;
	__le32 ap_runtime_data_addr;
	__le32 ap_runtime_data_size;
	__le32 md_runtime_data_addr;
	__le32 md_runtime_data_size;
	__le32 noncached_mpu_start_addr;
	__le32 noncached_mpu_total_size;
	__le32 cached_mpu_start_addr;
	__le32 cached_mpu_total_size;
	__le32 reserve_addr[12];
	__le32 tail_pattern;
};
static_assert(sizeof(struct mtk_ccci_ap_feature) == 156);

/* LK tags read here (downstream ccci_util), only the leading fields used */
struct mtk_ccci_lk_hdr_tbl {		/* "hdr_tbl_inf": modem banks 0/1 */
	__le64 base;
	__le32 size;
};

struct mtk_ccci_lk_smem_layout {	/* "smem_layout": non-cached smem */
	__le64 base;
	__le32 md1_offset;
	__le32 md1_size;
};

struct mtk_ccci_lk_csmem {		/* "md1_bank4_cache_info" */
	__le64 base;
	__le32 md_offset;
	__le32 size;
};

struct mtk_ccci_lk_sib {		/* "md1_sib_info" */
	__le64 addr;
	__le32 size;
};

/* a record of the modem view of one bank (MD_MEM_AP_VIEW_INF) */
struct mtk_ccci_rt_mem_view {
	__le32 md_addr;
	__le32 size;
	__le32 ap_lo;
	__le32 ap_hi;
};

static u8 ft_byte(unsigned int support, unsigned int ver)
{
	return FIELD_PREP(MTK_CCCI_FT_MASK, support) |
	       FIELD_PREP(MTK_CCCI_FT_VER, ver);
}

static const void *rt_tag(struct mtk_ccci *ccci, const char *name,
			  u32 min_size)
{
	const void *p;
	u32 size;

	p = mtk_ccci_lk_find(ccci, name, &size);
	return p && size >= min_size ? p : NULL;
}

static u32 rt_tag_u32(struct mtk_ccci *ccci, const char *name, u32 def)
{
	const __le32 *p = rt_tag(ccci, name, sizeof(*p));

	return p ? le32_to_cpu(*p) : def;
}

static u32 rt_phy_cap_size(struct mtk_ccci *ccci)
{
	return rt_tag_u32(ccci, "md1_phy_cap", 0);
}

static u32 rt_sib_size(struct mtk_ccci *ccci)
{
	const struct mtk_ccci_lk_sib *sib = rt_tag(ccci, "md1_sib_info",
						   sizeof(*sib));

	return sib && le64_to_cpu(sib->addr) ? le32_to_cpu(sib->size) : 0;
}

static u32 rt_smem_size(struct mtk_ccci *ccci, enum mtk_ccci_smem_id id)
{
	const struct mtk_ccci_smem_region *r = mtk_ccci_lk_smem(ccci, id);

	return r ? le32_to_cpu(r->size) : 0;
}

/*
 * The CCB data announced. The downstream driver places the CCB users back
 * to back at the start of the CCB data LK handed over, each as large as
 * its buffers (mtk_ccci_ccb.c), and announces their sum, not the size of
 * the LK block (on Titan 2: 0x17c000 of 0x1600000; the rest of the block
 * is the raw DHL/MDM logging memory).
 */
static u32 rt_ccb_data_size(struct mtk_ccci *ccci)
{
	u32 lk = 0, need = mtk_ccci_ccb_data_size();
	unsigned int id;

	for (id = MTK_CCCI_SMEM_CCB_DHL; id <= MTK_CCCI_SMEM_CCB_META; id++)
		lk += rt_smem_size(ccci, id);
	if (ccb_data_lk)
		return lk;
	if (need > lk) {
		dev_warn(ccci->dev, "CCB data of %#x bytes, %#x needed\n",
			 lk, need);
		return lk;
	}
	return need;
}

/* what this AP supports, as the downstream driver configures it */
static void rt_ap_features(struct mtk_ccci *ccci, u8 ap[MTK_CCCI_FEATURE_COUNT])
{
	static const u8 must[] = {
		FT_BOOT_INFO, FT_EXCEPTION_SHARE_MEMORY, FT_CCIF_SHARE_MEMORY,
		FT_CCISM_SHARE_MEMORY, FT_CCISM_SHARE_MEMORY_EXP,
		FT_MD_CONSYS_SHARE_MEMORY, FT_CCB_SHARE_MEMORY,
		FT_DHL_RAW_SHARE_MEMORY, FT_LWA_SHARE_MEMORY,
		FT_DT_NETD_SHARE_MEMORY, FT_DT_USB_SHARE_MEMORY,
		FT_AUDIO_RAW_SHARE_MEMORY, FT_MISC_INFO_HIF_DMA_REMAP,
		FT_MULTI_MD_MPU, FT_MISC_INFO_RTC_32K_LESS,
		FT_MISC_INFO_RANDOM_SEED_NUM, FT_MISC_INFO_GPS_COCLOCK,
		FT_MISC_INFO_SBP_ID, FT_MISC_INFO_CCCI, FT_MISC_INFO_CLIB_TIME,
		FT_MISC_INFO_C2K, FT_EE_AFTER_EPOF, FT_AP_CCMNI_MTU,
		FT_NVRAM_CACHE_SHARE_MEMORY, FT_AMMS_DRDI_COPY,
	};
	static const u8 optional[] = {
		FT_MD_IMAGE_START_MEMORY, FT_MD_USIP_SHARE_MEMORY,
		FT_MD_MTEE_SHARE_MEMORY_ENABLE, FT_MD_WIFI_PROXY_SHARE_MEMORY,
		FT_MD_MEM_AP_VIEW_INF, FT_MD_POST_DUMP,
	};
	static const u8 unsupported[] = {
		FT_MD1MD3_SHARE_MEMORY, FT_MISC_INFO_CUSTOMER_VAL,
		FT_MISC_INFO_C2K_MEID, FT_SMART_LOGGING_SHARE_MEMORY,
		FT_SECURITY_SHARE_MEMORY,
	};
	unsigned int i, md_ver;
	bool on;

	memset(ap, 0, MTK_CCCI_FEATURE_COUNT);
	for (i = 0; i < ARRAY_SIZE(must); i++)
		ap[must[i]] = ft_byte(MTK_CCCI_FT_MUST_SUPPORT, 0);
	for (i = 0; i < ARRAY_SIZE(optional); i++)
		ap[optional[i]] = ft_byte(MTK_CCCI_FT_OPTIONAL_SUPPORT, 0);
	for (i = 0; i < ARRAY_SIZE(unsupported); i++)
		ap[unsupported[i]] = ft_byte(MTK_CCCI_FT_NOT_SUPPORT, 0);

	/* the regions LK may or may not have set up */
	on = rt_phy_cap_size(ccci) || rt_sib_size(ccci);
	ap[FT_MD_PHY_CAPTURE] = ft_byte(on ? MTK_CCCI_FT_MUST_SUPPORT :
					MTK_CCCI_FT_NOT_SUPPORT, 0);
	on = rt_smem_size(ccci, MTK_CCCI_SMEM_UDC_DATA) &&
	     rt_smem_size(ccci, MTK_CCCI_SMEM_UDC_DESCTAB);
	ap[FT_UDC_RAW_SHARE_MEMORY] = ft_byte(on ? MTK_CCCI_FT_MUST_SUPPORT :
					      MTK_CCCI_FT_NOT_SUPPORT, 0);
	on = rt_tag_u32(ccci, "smem_amms_pos_size", 0);
	ap[FT_MD_POS_SHARE_MEMORY] = ft_byte(on ? MTK_CCCI_FT_MUST_SUPPORT :
					     MTK_CCCI_FT_NOT_SUPPORT, 0);

	/* the high resolution time, if the modem knows it */
	md_ver = FIELD_GET(MTK_CCCI_FT_VER,
			   ccci->hs1_query.feature[FT_MISC_INFO_CLIB_TIME]);
	if (md_ver == FT_CLIB_TIME_HIRES)
		ap[FT_MISC_INFO_CLIB_TIME] = ft_byte(MTK_CCCI_FT_MUST_SUPPORT,
						     FT_CLIB_TIME_HIRES);
}

/*
 * Agree on one feature. Return the support byte of the record, or -EPROTO
 * if the modem must have a feature this AP does not support.
 */
static int rt_agree(u8 md, u8 ap)
{
	unsigned int md_sup = FIELD_GET(MTK_CCCI_FT_MASK, md);
	unsigned int md_ver = FIELD_GET(MTK_CCCI_FT_VER, md);
	unsigned int ap_sup = FIELD_GET(MTK_CCCI_FT_MASK, ap);
	unsigned int ap_ver = FIELD_GET(MTK_CCCI_FT_VER, ap);
	bool ok;

	switch (md_sup) {
	case MTK_CCCI_FT_NOT_EXIST:
		return md;
	case MTK_CCCI_FT_MUST_SUPPORT:
		return ap_sup < MTK_CCCI_FT_MUST_SUPPORT ? -EPROTO : md;
	case MTK_CCCI_FT_OPTIONAL_SUPPORT:
		ok = md_ver == ap_ver && ap_sup >= MTK_CCCI_FT_MUST_SUPPORT;
		break;
	case MTK_CCCI_FT_BACKWARD_COMPAT:
		ok = md_ver >= ap_ver;
		break;
	default:
		/* the downstream driver leaves the record empty */
		return 0;
	}
	return ft_byte(ok ? MTK_CCCI_FT_MUST_SUPPORT : MTK_CCCI_FT_NOT_SUPPORT,
		       ap_ver);
}

struct rt_writer {
	u8 *p;
	u32 left;
	int err;
};

static void rt_append(struct rt_writer *w, u8 id, u8 support,
		      const void *data, u32 len)
{
	struct mtk_ccci_rt_feature f = {
		.feature_id = id,
		.support = support,
		.data_len = cpu_to_le32(len),
	};

	if (w->err)
		return;
	if (sizeof(f) + len > w->left) {
		w->err = -ENOSPC;
		return;
	}
	memcpy(w->p, &f, sizeof(f));
	w->p += sizeof(f);
	if (len)
		memcpy(w->p, data, len);
	w->p += len;
	w->left -= sizeof(f) + len;
}

static void rt_shm_of(struct mtk_ccci *ccci, enum mtk_ccci_smem_id id,
		      struct mtk_ccci_rt_shm *shm)
{
	const struct mtk_ccci_smem_region *r = mtk_ccci_lk_smem(ccci, id);

	shm->addr = cpu_to_le32(r ? mtk_ccci_lk_smem_md_addr(r) : 0);
	shm->size = r ? r->size : 0;
}

/* the modem view of its banks (bank 0/1, non-cached and cached smem) */
static u32 rt_mem_view(struct mtk_ccci *ccci, struct mtk_ccci_rt_mem_view *v)
{
	const struct mtk_ccci_lk_hdr_tbl *md;
	const struct mtk_ccci_lk_smem_layout *nc;
	const struct mtk_ccci_lk_csmem *c;
	u64 ap;

	md = rt_tag(ccci, "hdr_tbl_inf", sizeof(*md));
	nc = rt_tag(ccci, "smem_layout", sizeof(*nc));
	c = rt_tag(ccci, "md1_bank4_cache_info", sizeof(*c));
	if (!md || !nc || !c)
		return 0;

	ap = le64_to_cpu(md->base);
	v[0] = (struct mtk_ccci_rt_mem_view){
		.md_addr = 0,
		.size = md->size,
		.ap_lo = cpu_to_le32(lower_32_bits(ap)),
		.ap_hi = cpu_to_le32(upper_32_bits(ap)),
	};
	ap = le64_to_cpu(nc->base) + le32_to_cpu(nc->md1_offset);
	v[1] = (struct mtk_ccci_rt_mem_view){
		.md_addr = cpu_to_le32(MTK_CCCI_SMEM_MD_VIEW_BASE),
		.size = nc->md1_size,
		.ap_lo = cpu_to_le32(lower_32_bits(ap)),
		.ap_hi = cpu_to_le32(upper_32_bits(ap)),
	};
	ap = le64_to_cpu(c->base);
	v[2] = (struct mtk_ccci_rt_mem_view){
		.md_addr = cpu_to_le32(MTK_CCCI_SMEM_MD_VIEW_BASE +
				       rt_tag_u32(ccci, "md1_smem_cahce_offset",
						  SZ_128M)),
		.size = c->size,
		.ap_lo = cpu_to_le32(lower_32_bits(ap)),
		.ap_hi = cpu_to_le32(upper_32_bits(ap)),
	};
	return 3 * sizeof(*v);
}

/*
 * The data of one feature agreed on, into buf (16 words). Return its length,
 * or -ENODATA when the downstream driver leaves the record out entirely.
 */
static int rt_data_of(struct mtk_ccci *ccci, u8 id, u8 support, void *buf)
{
	struct mtk_ccci_rt_shm *shm = buf;
	__le32 *w = buf;
	const struct mtk_ccci_smem_region *r;
	struct timespec64 ts;
	u64 usec, cnt;
	unsigned int j;

	/* most features are a shared memory region */
	static const u8 region[][2] = {
		{ FT_CCISM_SHARE_MEMORY, MTK_CCCI_SMEM_CCISM_SCP },
		{ FT_DHL_RAW_SHARE_MEMORY, MTK_CCCI_SMEM_RAW_DHL },
		{ FT_LWA_SHARE_MEMORY, MTK_CCCI_SMEM_RAW_LWA },
		{ FT_DT_NETD_SHARE_MEMORY, MTK_CCCI_SMEM_RAW_NETD },
		{ FT_DT_USB_SHARE_MEMORY, MTK_CCCI_SMEM_RAW_USB },
		{ FT_SMART_LOGGING_SHARE_MEMORY, MTK_CCCI_SMEM_SMART_LOGGING },
		{ FT_MD1MD3_SHARE_MEMORY, MTK_CCCI_SMEM_RAW_MD2MD },
		{ FT_AUDIO_RAW_SHARE_MEMORY, MTK_CCCI_SMEM_RAW_AUDIO },
		{ FT_CCISM_SHARE_MEMORY_EXP, MTK_CCCI_SMEM_CCISM_MCU_EXP },
		{ FT_MD_CONSYS_SHARE_MEMORY, MTK_CCCI_SMEM_MD_CONSYS },
		{ FT_MD_USIP_SHARE_MEMORY, MTK_CCCI_SMEM_USIP },
		{ FT_MD_POS_SHARE_MEMORY, MTK_CCCI_SMEM_AMMS_POS },
		{ FT_MD_WIFI_PROXY_SHARE_MEMORY, MTK_CCCI_SMEM_WIFI_PROXY },
		{ FT_NVRAM_CACHE_SHARE_MEMORY, MTK_CCCI_SMEM_NVRAM_CACHE },
		{ FT_AMMS_DRDI_COPY, MTK_CCCI_SMEM_DRDI },
		{ FT_MD_POST_DUMP, MTK_CCCI_SMEM_POST_DUMP },
	};

	for (j = 0; j < ARRAY_SIZE(region); j++) {
		if (region[j][0] == id) {
			rt_shm_of(ccci, region[j][1], shm);
			return sizeof(*shm);
		}
	}

	/* the "misc info" features carry four words, zero unless set */
	memset(buf, 0, 8 * sizeof(*w));

	switch (id) {
	case FT_BOOT_INFO:
		w[0] = cpu_to_le32(MTK_CCCI_CH_CONTROL_RX);
		/* log mode 0, normal boot, no wait time for a memory dump */
		w[1] = cpu_to_le32(MTK_CCCI_BOOT_MODE_NORMAL);
		w[2] = cpu_to_le32(MTK_CCCI_BOOT_ATTR_PHONE);
		return 4 * sizeof(*w);
	case FT_EXCEPTION_SHARE_MEMORY:
		/* the downstream driver leaves the record out without it */
		r = mtk_ccci_lk_smem(ccci, MTK_CCCI_SMEM_MDCCCI_DBG);
		if (!r)
			return -ENODATA;
		shm->addr = cpu_to_le32(mtk_ccci_lk_smem_md_addr(r));
		shm->size = cpu_to_le32(MTK_CCCI_EE_SMEM_SIZE);
		return sizeof(*shm);
	case FT_CCIF_SHARE_MEMORY:
		/* CCISM_MCU: what the ring queues use of it */
		rt_shm_of(ccci, MTK_CCCI_SMEM_CCISM_MCU, shm);
		shm->size = cpu_to_le32(ccci->ccism_size);
		return sizeof(*shm);
	case FT_CCB_SHARE_MEMORY:
		r = mtk_ccci_lk_smem(ccci, MTK_CCCI_SMEM_CCB_CTRL);
		if (r) {
			w[0] = cpu_to_le32(mtk_ccci_lk_smem_md_addr(r));
			w[1] = r->size;
		}
		r = mtk_ccci_lk_smem(ccci, MTK_CCCI_SMEM_CCB_DHL);
		if (r) {
			w[2] = cpu_to_le32(mtk_ccci_lk_smem_md_addr(r));
			w[3] = cpu_to_le32(rt_ccb_data_size(ccci));
		}
		return 4 * sizeof(*w);
	case FT_MULTI_MD_MPU:
		return 0;
	case FT_MISC_INFO_RANDOM_SEED_NUM:
		w[0] = cpu_to_le32(get_random_u32());
		return 4 * sizeof(*w);
	case FT_MISC_INFO_SBP_ID:
		w[0] = cpu_to_le32(sbp_code);
		w[1] = cpu_to_le32(rat_bitmap);
		return 4 * sizeof(*w);
	case FT_MISC_INFO_CCCI:
		w[0] = cpu_to_le32(MTK_CCCI_MISC_SEQ_CHECK |
				   MTK_CCCI_MISC_POLL_MD_STATUS);
		return 4 * sizeof(*w);
	case FT_MISC_INFO_CLIB_TIME:
		/* seconds, time zone (minutes west, none set), DST flag */
		ktime_get_real_ts64(&ts);
		w[0] = cpu_to_le32(lower_32_bits(ts.tv_sec));
		w[1] = cpu_to_le32(upper_32_bits(ts.tv_sec));
		if (FIELD_GET(MTK_CCCI_FT_VER, support) != FT_CLIB_TIME_HIRES)
			return 4 * sizeof(*w);
		/* then microseconds and the AP system counter */
		usec = ts.tv_nsec / NSEC_PER_USEC;
		cnt = arch_timer_read_counter();
		w[4] = cpu_to_le32(lower_32_bits(usec));
		w[5] = cpu_to_le32(upper_32_bits(usec));
		w[6] = cpu_to_le32(lower_32_bits(cnt));
		w[7] = cpu_to_le32(upper_32_bits(cnt));
		return 8 * sizeof(*w);
	case FT_MISC_INFO_C2K:
		if (rat_bitmap & MTK_CCCI_RAT_CDMA2000)
			w[0] = cpu_to_le32(MTK_CCCI_C2K_ON);
		return 4 * sizeof(*w);
	case FT_MISC_INFO_HIF_DMA_REMAP:
	case FT_MISC_INFO_RTC_32K_LESS:
	case FT_MISC_INFO_GPS_COCLOCK:
	case FT_EE_AFTER_EPOF:
		return 4 * sizeof(*w);
	case FT_MD_IMAGE_START_MEMORY:
		/* the downstream driver: address 0, the raw image size from LK */
		shm->addr = 0;
		shm->size = cpu_to_le32(rt_tag_u32(ccci, "md1img", 0));
		return sizeof(*shm);
	case FT_AP_CCMNI_MTU:
		w[0] = cpu_to_le32(MTK_CCCI_CCMNI_MTU);
		return sizeof(*w);
	case FT_MD_PHY_CAPTURE:
		shm->addr = 0;
		shm->size = cpu_to_le32(rt_sib_size(ccci));
		return sizeof(*shm);
	case FT_UDC_RAW_SHARE_MEMORY:
		r = mtk_ccci_lk_smem(ccci, MTK_CCCI_SMEM_UDC_DATA);
		if (r) {
			w[0] = cpu_to_le32(mtk_ccci_lk_smem_md_addr(r));
			w[1] = r->size;
		}
		r = mtk_ccci_lk_smem(ccci, MTK_CCCI_SMEM_UDC_DESCTAB);
		if (r) {
			w[2] = cpu_to_le32(mtk_ccci_lk_smem_md_addr(r));
			w[3] = r->size;
		}
		return 4 * sizeof(*w);
	case FT_MD_MTEE_SHARE_MEMORY_ENABLE:
		w[0] = cpu_to_le32(rt_tag_u32(ccci, "mtee_support", 0));
		return sizeof(*w);
	case FT_MD_MEM_AP_VIEW_INF:
		return rt_mem_view(ccci, buf);
	default:
		/* agreed on, but not handled downstream: no record */
		return -ENODATA;
	}
}

/* the MPU windows: the whole non-cached and cached shared memory */
static void rt_mpu_window(const struct mtk_ccci_smem_region *tbl,
			  unsigned int num, __le32 *start, __le32 *total)
{
	u32 size = 0;
	unsigned int i;

	if (!num)
		return;
	for (i = 0; i < num; i++)
		size += le32_to_cpu(tbl[i].size);
	*start = cpu_to_le32(mtk_ccci_lk_smem_md_addr(&tbl[0]));
	*total = cpu_to_le32(size);
}

/**
 * mtk_ccci_rt_enabled() - whether start answers HS1
 *
 * Return: the runtime_data module parameter.
 */
bool mtk_ccci_rt_enabled(void)
{
	return runtime_data;
}

/**
 * mtk_ccci_rt_send() - answer HS1
 * @ccci: driver state, with the HS1 query read and the shared memory
 *	mapped (mtk_ccci_ccif_start)
 *
 * Writes the runtime data records, then the AP feature block into the
 * CCIF SRAM, and notifies the modem on the SRAM channel. What was agreed
 * on is kept in ccci->rt_answer for debugfs.
 *
 * Return: 0, -EPROTO if the modem needs a feature this AP does not have,
 * or another negative error.
 */
int mtk_ccci_rt_send(struct mtk_ccci *ccci)
{
	u8 ap[MTK_CCCI_FEATURE_COUNT];
	struct mtk_ccci_ap_feature af = { };
	const struct mtk_ccci_smem_region *rt;
	struct rt_writer w = {
		.p = ccci->rt_copy,
		.left = MTK_CCCI_RT_AP_SIZE,
	};
	__le32 buf[16];
	unsigned int i;
	u32 rt_md;

	rt = mtk_ccci_lk_smem(ccci, MTK_CCCI_SMEM_RUNTIME_DATA);
	if (!rt || !ccci->rt_data || !ccci->hs1_valid ||
	    ccci->rt_data_size < MTK_CCCI_RT_AP_SIZE + MTK_CCCI_RT_MD_SIZE)
		return -EINVAL;

	rt_ap_features(ccci, ap);
	for (i = 0; i < MTK_CCCI_FEATURE_COUNT; i++) {
		int sup, len = 0;

		sup = rt_agree(ccci->hs1_query.feature[i], ap[i]);
		if (sup < 0) {
			dev_err(ccci->dev,
				"modem needs runtime feature %u (%#x), not supported\n",
				i, ccci->hs1_query.feature[i]);
			return sup;
		}
		ccci->rt_answer[i] = sup;
		if (FIELD_GET(MTK_CCCI_FT_MASK, sup) == MTK_CCCI_FT_MUST_SUPPORT) {
			len = rt_data_of(ccci, i, sup, buf);
			if (len < 0) {
				dev_dbg(ccci->dev, "runtime feature %u: no record\n", i);
				continue;
			}
		}
		rt_append(&w, i, sup, buf, len);
	}
	if (w.err) {
		dev_err(ccci->dev, "runtime data does not fit in %#x bytes\n",
			MTK_CCCI_RT_AP_SIZE);
		return w.err;
	}
	ccci->rt_len = MTK_CCCI_RT_AP_SIZE - w.left;
	memset_io(ccci->rt_data, 0, MTK_CCCI_RT_AP_SIZE);
	memcpy_toio(ccci->rt_data, ccci->rt_copy, ccci->rt_len);

	rt_md = mtk_ccci_lk_smem_md_addr(rt);
	af.head_pattern = cpu_to_le32(MTK_CCCI_AP_FEATURE_PATTERN);
	af.share_memory_support = cpu_to_le32(MTK_CCCI_SHM_MULTI_MD_MPU);
	af.ap_runtime_data_addr = cpu_to_le32(rt_md);
	af.ap_runtime_data_size = cpu_to_le32(MTK_CCCI_RT_AP_SIZE);
	af.md_runtime_data_addr = cpu_to_le32(rt_md + MTK_CCCI_RT_AP_SIZE);
	af.md_runtime_data_size = cpu_to_le32(MTK_CCCI_RT_MD_SIZE);
	rt_mpu_window(ccci->lk.nc_smem, ccci->lk.nc_smem_num,
		      &af.noncached_mpu_start_addr, &af.noncached_mpu_total_size);
	rt_mpu_window(ccci->lk.c_smem, ccci->lk.c_smem_num,
		      &af.cached_mpu_start_addr, &af.cached_mpu_total_size);
	/* feature 1: support 0, version 1 = MPU size is AP-MD1 + MD1-MD3 */
	af.feature[1] = ft_byte(0, 1);
	af.tail_pattern = cpu_to_le32(MTK_CCCI_AP_FEATURE_PATTERN);

	dev_info(ccci->dev,
		 "runtime data: %u bytes at %#x, MPU nc %#x+%#x c %#x+%#x\n",
		 ccci->rt_len, rt_md, le32_to_cpu(af.noncached_mpu_start_addr),
		 le32_to_cpu(af.noncached_mpu_total_size),
		 le32_to_cpu(af.cached_mpu_start_addr),
		 le32_to_cpu(af.cached_mpu_total_size));

	mtk_ccci_ccif_write_rt(ccci, &af, sizeof(af));
	return mtk_ccci_ccif_notify(ccci, MTK_CCIF_CH_SRAM);
}

/* debugfs: what was agreed on and the AP half of the runtime data */
void mtk_ccci_rt_show(struct mtk_ccci *ccci, struct seq_file *s)
{
	unsigned int i;

	if (!ccci->rt_len)
		return;
	seq_puts(s, "runtime answer:");
	for (i = 0; i < MTK_CCCI_FEATURE_COUNT; i++)
		if (ccci->rt_answer[i])
			seq_printf(s, " %u:%#x", i, ccci->rt_answer[i]);
	seq_printf(s, "\nruntime data: %u bytes\n", ccci->rt_len);
	seq_hex_dump(s, "  ", DUMP_PREFIX_OFFSET, 16, 4, ccci->rt_copy,
		     ccci->rt_len, false);
}
