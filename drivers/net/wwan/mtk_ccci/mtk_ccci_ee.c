// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CCCI: the modem exception handshake and its report.
 *
 * On an exception the modem raises CCIF channel 16 and then walks the
 * AP through the MediaTek downstream sequence:
 *
 *	modem			AP
 *	EX_INIT (16)	->
 *			<-	EX_ACK (17)
 *	EX_INIT_DONE (17)
 *	EX_CLEARQ_DONE (18) ->	switch every queue to its exception ring
 *				(CCISM_MCU_EXP), emptied
 *			<-	EX_CLEARQ_ACK (18)
 *	EX_ALLQ_RESET (19)
 *	MD_EX on CONTROL ->
 *			<-	MD_EX
 *	MD_EX_REC_OK	->	(carries the exception overview)
 *	MD_EX_PASS	->
 *
 * The overview (format version 6) says which core failed, the exception
 * type and either the assert file, line and parameters or the fatal
 * error codes. Before HS2 the modem leaves it in the MDSS_DBG shared
 * memory; the copy in MD_EX_REC_OK is preferred when it is valid.
 *
 * Nothing here dumps modem memory or restarts the modem: the report is
 * logged and kept for debugfs ("ee"), then the FSM powers the modem off.
 *
 * Copyright (C) 2019 MediaTek Inc.
 * Copyright (C) 2026 serlk
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/wait.h>

#include "mtk_ccci.h"

#define EE_MD_EX_CHK_ID		0x45584350	/* "EXCP" */
#define EE_MD_EX_REC_OK_CHK_ID	0x45524543	/* "EREC" */

/* how long the modem gets for the handshake and its report */
#define EE_REPORT_TIMEOUT_MS	10000

/* exception overview, format 6 */
#define EE_OVERVIEW_VER_MASK	0xffff0000
#define EE_OVERVIEW_VER		0xab000006
#define EE_CORES		5
#define EE_CORE_NAME_LEN	11
#define EE_VPE_NUM		15

enum {
	EE_CLASS_ASSERT,
	EE_CLASS_FATAL,
	EE_CLASS_CUSTOM,
};

struct mtk_ccci_ee_assert {
	char file[256];
	__le32 line;
	__le32 param[3];
	__le32 lr;
} __packed;

struct mtk_ccci_ee_fatal {
	__le32 code[3];
	char offender[8];
	u8 cadefa;
	u8 has_file;
	u8 section;
	u8 dispatch_valid;
	u8 pad[4];
	__le32 status;
	__le32 sp;
	__le32 pc;
	__le32 lr;
	__le32 addr;
	__le32 cause;
	char file[];	/* when has_file, to the end of the union */
} __packed;

struct mtk_ccci_ee_overview {
	__le32 version;
	__le32 cores;
	struct {
		char name[EE_CORE_NAME_LEN];
		u8 offender;
	} __packed core[EE_CORES];
	/* the main record */
	__le16 type;
	u8 type_format;
	u8 class;
	u8 elm_status;
	u8 vpe;
	u8 tc;
	u8 pad0;
	union {
		struct mtk_ccci_ee_assert assert;
		struct mtk_ccci_ee_fatal fatal;
	} __packed info;
	__le32 vpes;
	struct {
		__le32 step;
		__le32 time;
	} __packed steps[EE_VPE_NUM];
	u8 rest[116];
} __packed;
static_assert(sizeof(struct mtk_ccci_ee_overview) == 592);

/* a few exception types worth a name; the rest is printed as a number */
static const char *mtk_ccci_ee_type_name(u16 type)
{
	switch (type) {
	case 0x50: return "assert";
	case 0x51: return "assert (extended record)";
	case 0x52: return "assert (native)";
	case 0x53 ... 0x55: return "assert (custom)";
	case 0x30: return "stack access";
	case 0x31: return "fatal error (task)";
	case 0x32: return "fatal error (buffer)";
	case 0x60 ... 0x67: return "cross core";
	case 0x70: return "EMI MPU violation";
	case 0x00 ... 0x2f: return "CPU exception";
	default: return "unknown";
	}
}

/* format the overview into ccci->ee_report; false if it is not one */
static bool mtk_ccci_ee_parse(struct mtk_ccci *ccci, const void *buf,
			      size_t len, const char *from)
{
	const struct mtk_ccci_ee_overview *ov = buf;
	char *p = ccci->ee_report, *end = p + sizeof(ccci->ee_report);
	u32 ver, cores, i;
	u16 type;

	if (len < sizeof(*ov))
		return false;
	ver = le32_to_cpu(ov->version);
	cores = le32_to_cpu(ov->cores);
	if (ver != EE_OVERVIEW_VER || cores > EE_CORES)
		return false;

	type = le16_to_cpu(ov->type);
	p += scnprintf(p, end - p, "from %s: type %#x (%s), class %u", from,
		       type, mtk_ccci_ee_type_name(type), ov->class);
	for (i = 0; i < cores; i++)
		if (ov->core[i].offender) {
			p += scnprintf(p, end - p, ", core %.*s",
				       EE_CORE_NAME_LEN, ov->core[i].name);
			break;
		}
	p += scnprintf(p, end - p, " (vpe %u tc %u)\n", ov->vpe, ov->tc);

	switch (ov->class) {
	case EE_CLASS_ASSERT: {
		const struct mtk_ccci_ee_assert *a = &ov->info.assert;

		p += scnprintf(p, end - p,
			       "assert %.*s:%u params %#x %#x %#x lr %#x\n",
			       (int)sizeof(a->file), a->file,
			       le32_to_cpu(a->line), le32_to_cpu(a->param[0]),
			       le32_to_cpu(a->param[1]),
			       le32_to_cpu(a->param[2]), le32_to_cpu(a->lr));
		break;
	}
	case EE_CLASS_FATAL:
	case EE_CLASS_CUSTOM: {
		const struct mtk_ccci_ee_fatal *f = &ov->info.fatal;
		size_t flen = sizeof(ov->info) - sizeof(*f);

		p += scnprintf(p, end - p,
			       "fatal %#x %#x %#x offender %.*s section %u pc %#x lr %#x addr %#x cause %#x\n",
			       le32_to_cpu(f->code[0]), le32_to_cpu(f->code[1]),
			       le32_to_cpu(f->code[2]),
			       f->offender[0] == (char)0xcc ? 0 :
			       (int)sizeof(f->offender), f->offender,
			       f->section, le32_to_cpu(f->pc),
			       le32_to_cpu(f->lr), le32_to_cpu(f->addr),
			       le32_to_cpu(f->cause));
		if (f->has_file == 1)
			p += scnprintf(p, end - p, "file %.*s\n", (int)flen,
				       f->file);
		break;
	}
	default:
		break;
	}
	return true;
}

/* the overview the modem left in MDSS_DBG */
static bool mtk_ccci_ee_parse_smem(struct mtk_ccci *ccci)
{
	const struct mtk_ccci_smem_region *r;
	void __iomem *map;
	void *buf;
	size_t len;
	bool ok;

	r = mtk_ccci_lk_smem(ccci, MTK_CCCI_SMEM_MDSS_DBG);
	if (!r)
		return false;
	len = min_t(size_t, le32_to_cpu(r->size), MTK_CCCI_EE_PKT_MAX);
	if (len < sizeof(struct mtk_ccci_ee_overview))
		return false;
	buf = kmalloc(len, GFP_KERNEL);
	map = ioremap_wc(le64_to_cpu(r->ap_phys), len);
	ok = buf && map;
	if (ok) {
		memcpy_fromio(buf, map, len);
		ok = mtk_ccci_ee_parse(ccci, buf, len, "MDSS_DBG");
		if (!ok)
			print_hex_dump(KERN_INFO, "MDSS_DBG: ",
				       DUMP_PREFIX_OFFSET, 16, 4, buf, 128,
				       false);
	}
	if (map)
		iounmap(map);
	kfree(buf);
	return ok;
}

/* the queues the modem uses from EX_CLEARQ_DONE on */
static void mtk_ccci_ee_switch_rings(struct mtk_ccci *ccci)
{
	unsigned long flags[MTK_CCCI_RINGQ_NUM];
	unsigned int i;

	for (i = 0; i < MTK_CCCI_RINGQ_NUM; i++)
		spin_lock_irqsave(&ccci->tx_lock[i], flags[i]);
	for (i = 0; i < MTK_CCCI_RINGQ_NUM; i++) {
		struct mtk_ccci_ring *r = &ccci->ring[i];

		*r = ccci->ring_exp[i];
		if (!r->ctl)
			continue;
		/* empty both ways, as the downstream queue reset */
		writel(0, r->ctl + 0x00);
		writel(0, r->ctl + 0x04);
		writel(0, r->ctl + 0x0c);
		writel(0, r->ctl + 0x10);
	}
	memset(ccci->tx_seq, 0, sizeof(ccci->tx_seq));
	memset(ccci->rx_seq, 0, sizeof(ccci->rx_seq));
	for (i = MTK_CCCI_RINGQ_NUM; i-- > 0;)
		spin_unlock_irqrestore(&ccci->tx_lock[i], flags[i]);
}

/**
 * mtk_ccci_ee_ccif() - the exception channels the modem raised
 * @ccci: driver state
 * @channels: CCIF channels, MTK_CCIF_EX_MASK bits only
 *
 * Called from the RX worker, in the modem's order when several came at
 * once.
 */
void mtk_ccci_ee_ccif(struct mtk_ccci *ccci, u32 channels)
{
	if (channels & BIT(MTK_CCIF_CH_EX_INIT)) {
		ccci->ee_stage |= BIT(MTK_CCIF_CH_EX_INIT);
		mtk_ccci_ccif_notify(ccci, MTK_CCIF_CH_EX_ACK);
	}
	if (channels & BIT(MTK_CCIF_CH_EX_INIT_DONE))
		ccci->ee_stage |= BIT(MTK_CCIF_CH_EX_INIT_DONE);
	if (channels & BIT(MTK_CCIF_CH_EX_CLEARQ_DONE)) {
		ccci->ee_stage |= BIT(MTK_CCIF_CH_EX_CLEARQ_DONE);
		mtk_ccci_ee_switch_rings(ccci);
		mtk_ccci_ccif_notify(ccci, MTK_CCIF_CH_EX_CLEARQ_ACK);
	}
	if (channels & BIT(MTK_CCIF_CH_EX_ALLQ_RESET))
		ccci->ee_stage |= BIT(MTK_CCIF_CH_EX_ALLQ_RESET);
	dev_info(ccci->dev, "exception handshake: CCIF %#x, seen %#x\n",
		 channels, ccci->ee_stage);
	mtk_ccci_fsm_exception(ccci, channels);
}

/**
 * mtk_ccci_ee_ctl() - an exception message on the control channel
 * @ccci: driver state
 * @msg: data[1] of the message
 * @id: its reserved word
 * @skb: the message, header included; not taken
 */
void mtk_ccci_ee_ctl(struct mtk_ccci *ccci, u32 msg, u32 id,
		     const struct sk_buff *skb)
{
	u32 len;

	switch (msg) {
	case MTK_CCCI_CTL_MD_EX:
		if (id != EE_MD_EX_CHK_ID)
			break;
		mtk_ccci_send_msg(ccci, MTK_CCCI_Q_CTRL,
				  MTK_CCCI_CH_CONTROL_TX, MTK_CCCI_CTL_MD_EX,
				  EE_MD_EX_CHK_ID);
		break;
	case MTK_CCCI_CTL_MD_EX_REC_OK:
		if (id != EE_MD_EX_REC_OK_CHK_ID)
			break;
		len = skb->len - sizeof(struct mtk_ccci_header);
		len = min_t(u32, len, MTK_CCCI_EE_PKT_MAX);
		memcpy(ccci->ee_pkt, skb->data + sizeof(struct mtk_ccci_header),
		       len);
		ccci->ee_pkt_len = len;
		WRITE_ONCE(ccci->ee_rec_ok, true);
		wake_up_all(&ccci->hs_wq);
		break;
	case MTK_CCCI_CTL_MD_EX_PASS:
		WRITE_ONCE(ccci->ee_pass, true);
		wake_up_all(&ccci->hs_wq);
		break;
	default:
		break;
	}
	dev_info(ccci->dev, "exception message %#x %#x\n", msg, id);
	mtk_ccci_fsm_exception(ccci, 0);
}

/* a new start: forget the last report */
void mtk_ccci_ee_reset(struct mtk_ccci *ccci)
{
	ccci->ee_stage = 0;
	ccci->ee_pkt_len = 0;
	ccci->ee_rec_ok = false;
	ccci->ee_pass = false;
	ccci->ee_report[0] = '\0';
}

/**
 * mtk_ccci_ee_report() - let the modem finish its exception and log it
 * @ccci: driver state, the modem marked as in exception
 *
 * Waits for MD_EX_PASS (or the timeout), then logs the overview. The
 * modem stays powered: the caller switches it off.
 */
void mtk_ccci_ee_report(struct mtk_ccci *ccci)
{
	const char *line, *next;

	wait_event_timeout(ccci->hs_wq, READ_ONCE(ccci->ee_pass),
			   msecs_to_jiffies(EE_REPORT_TIMEOUT_MS));
	dev_err(ccci->dev, "exception handshake %#x, MD_EX_REC_OK %s (%u bytes), MD_EX_PASS %s\n",
		ccci->ee_stage, ccci->ee_rec_ok ? "seen" : "missing",
		ccci->ee_pkt_len, ccci->ee_pass ? "seen" : "missing");

	if (!(ccci->ee_rec_ok &&
	      mtk_ccci_ee_parse(ccci, ccci->ee_pkt, ccci->ee_pkt_len,
				"MD_EX_REC_OK")) &&
	    !mtk_ccci_ee_parse_smem(ccci)) {
		strscpy(ccci->ee_report, "no valid exception overview\n");
		if (ccci->ee_pkt_len)
			print_hex_dump(KERN_INFO, "MD_EX_REC_OK: ",
				       DUMP_PREFIX_OFFSET, 16, 4, ccci->ee_pkt,
				       min_t(u32, ccci->ee_pkt_len, 128),
				       false);
	}
	for (line = ccci->ee_report; *line; line = next) {
		next = strchrnul(line, '\n');
		dev_err(ccci->dev, "modem exception: %.*s\n",
			(int)(next - line), line);
		if (*next)
			next++;
	}
}

static int mtk_ccci_ee_show(struct seq_file *s, void *unused)
{
	struct mtk_ccci *ccci = s->private;

	seq_printf(s, "handshake %#x, MD_EX_REC_OK %s (%u bytes), MD_EX_PASS %s\n",
		   ccci->ee_stage, ccci->ee_rec_ok ? "seen" : "missing",
		   ccci->ee_pkt_len, ccci->ee_pass ? "seen" : "missing");
	seq_puts(s, ccci->ee_report);
	if (ccci->ee_pkt_len)
		seq_hex_dump(s, "pkt ", DUMP_PREFIX_OFFSET, 16, 4,
			     ccci->ee_pkt, ccci->ee_pkt_len, true);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_ccci_ee);

void mtk_ccci_ee_debugfs_init(struct mtk_ccci *ccci)
{
	debugfs_create_file("ee", 0400, ccci->debugfs, ccci,
			    &mtk_ccci_ee_fops);
}

int mtk_ccci_ee_init(struct mtk_ccci *ccci)
{
	ccci->ee_pkt = devm_kzalloc(ccci->dev, MTK_CCCI_EE_PKT_MAX,
				    GFP_KERNEL);
	return ccci->ee_pkt ? 0 : -ENOMEM;
}
