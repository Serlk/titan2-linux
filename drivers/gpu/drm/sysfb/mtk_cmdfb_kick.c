// SPDX-License-Identifier: GPL-2.0-only
/*
 * Frame trigger for a boot framebuffer on a MediaTek command-mode DSI panel
 *
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 *
 * A DSI panel in command mode keeps the picture in its own frame memory and
 * only shows new framebuffer contents after the display pipeline has been
 * started for one frame. The bootloader sets the pipeline up (OVL -> ... ->
 * DSI, one mutex) and leaves it idle; simpledrm then writes into the
 * framebuffer, but nothing ever reaches the panel.
 *
 * The register layout and the frame start are those of mainline
 * mediatek-drm (mtk_dsi.c, mtk-mutex.c, mtk_disp_ovl.c) and of the MediaTek
 * downstream MT6878 DSI driver (mtk_dsi_config_trigger); the pipeline
 * state checked at probe is what the hardware reads after the bootloader.
 *
 * This driver keeps the bootloader's pipeline and only starts frames:
 *
 *  - at probe it checks that the hardware is in the state the bootloader
 *    leaves (DSI in command mode with write_memory_start queued, mutex idle,
 *    OVL scanning out the simple-framebuffer) and does nothing otherwise;
 *  - it turns off the OVL layers that do not scan out the framebuffer
 *    (bootloader overlays such as the "orange state" warning);
 *  - after each simpledrm update (drm_sysfb flush notifier) it waits until
 *    the previous frame has finished and starts one frame: mutex enable,
 *    then DSI START 0 -> 1.
 *
 * Timings, PLLs and the panel are never touched. Power domains and clocks
 * are taken from DT if given (they are already on from the bootloader).
 *
 * This is a stop-gap for boards without a mediatek-drm driver; once the real
 * display driver exists it must not be used together with this one.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include <drm/drm_device.h>

#include "drm_sysfb_helper.h"

/* DSI */
#define DSI_START		0x000
#define DSI_START_BIT		BIT(0)
#define DSI_INTSTA		0x00c
#define DSI_BUSY		BIT(31)
#define DSI_MODE_CTRL		0x014
#define DSI_CMDQ_SIZE		0x060
#define DSI_CMDQ_SIZE_MASK	GENMASK(7, 0)
#define DSI_CMDQ(n)		(0xd00 + 4 * (n))
/* DCS long write (0x39) of write_memory_start (0x2c), as queued by LK */
#define DSI_CMDQ_WMS		0x002c3909

/* DISP mutex */
#define MUTEX_EN(n)		(0x020 + 0x20 * (n))
#define MUTEX_EN_BIT		BIT(0)
#define MUTEX_MOD(n)		(0x030 + 0x20 * (n))

/* OVL */
#define OVL_ROI_SIZE		0x020
#define OVL_SRC_CON		0x02c
#define OVL_PITCH(n)		(0x044 + 0x20 * (n))
#define OVL_PITCH_MASK		GENMASK(15, 0)
#define OVL_ADDR(n)		(0xf40 + 0x20 * (n))

#define KICK_POLL_US		500
#define KICK_TIMEOUT_US		100000
#define KICK_MAX_FAILURES	10

static unsigned int refresh_hz;
module_param(refresh_hz, uint, 0444);
MODULE_PARM_DESC(refresh_hz,
		 "Also start frames periodically at this rate (0 = only on updates, max 120)");

struct mtk_cmdfb_kick_data {
	unsigned int mutex_id;
	unsigned int ovl_layers;
};

struct mtk_cmdfb_kick {
	struct device *dev;
	void __iomem *mutex;
	void __iomem *ovl;
	void __iomem *dsi;
	const struct mtk_cmdfb_kick_data *data;
	struct device_node *fb_np;

	struct notifier_block nb;
	struct work_struct work;
	struct delayed_work refresh;
	unsigned long refresh_jiffies;

	atomic_t pending;
	bool stopped;
	unsigned int failures;
	unsigned long frames;
	unsigned long timeouts;
};

static bool mtk_cmdfb_kick_idle(struct mtk_cmdfb_kick *k)
{
	return !(readl(k->dsi + DSI_INTSTA) & DSI_BUSY) &&
	       !(readl(k->mutex + MUTEX_EN(k->data->mutex_id)) & MUTEX_EN_BIT);
}

/* The DSI must still be in command mode with write_memory_start queued. */
static bool mtk_cmdfb_kick_dsi_ok(struct mtk_cmdfb_kick *k)
{
	return readl(k->dsi + DSI_MODE_CTRL) == 0 &&
	       FIELD_GET(DSI_CMDQ_SIZE_MASK, readl(k->dsi + DSI_CMDQ_SIZE)) >= 1 &&
	       readl(k->dsi + DSI_CMDQ(0)) == DSI_CMDQ_WMS;
}

static void mtk_cmdfb_kick_stop(struct mtk_cmdfb_kick *k, const char *why)
{
	k->stopped = true;
	dev_err(k->dev,
		"%s, no more frames: DSI START %#x INTSTA %#x MODE %#x CMDQ0 %#x, MUTEX EN %#x\n",
		why, readl(k->dsi + DSI_START), readl(k->dsi + DSI_INTSTA),
		readl(k->dsi + DSI_MODE_CTRL), readl(k->dsi + DSI_CMDQ(0)),
		readl(k->mutex + MUTEX_EN(k->data->mutex_id)));
}

static void mtk_cmdfb_kick_work(struct work_struct *work)
{
	struct mtk_cmdfb_kick *k = container_of(work, struct mtk_cmdfb_kick, work);
	bool idle;
	int ret;

	if (k->stopped)
		return;

	/* never start a frame on top of one that is still being sent */
	ret = read_poll_timeout(mtk_cmdfb_kick_idle, idle, idle, KICK_POLL_US,
				KICK_TIMEOUT_US, false, k);
	if (ret) {
		k->timeouts++;
		if (++k->failures >= KICK_MAX_FAILURES)
			mtk_cmdfb_kick_stop(k, "previous frame never finished");
		else
			queue_work(system_dfl_wq, &k->work);	/* keep the request */
		dev_warn_ratelimited(k->dev, "previous frame not finished after %u ms\n",
				     KICK_TIMEOUT_US / 1000);
		return;
	}
	k->failures = 0;

	if (!atomic_xchg(&k->pending, 0))
		return;

	if (!mtk_cmdfb_kick_dsi_ok(k)) {
		mtk_cmdfb_kick_stop(k, "DSI no longer in LK's command mode state");
		return;
	}

	writel(MUTEX_EN_BIT, k->mutex + MUTEX_EN(k->data->mutex_id));
	writel(0, k->dsi + DSI_START);
	writel(DSI_START_BIT, k->dsi + DSI_START);
	k->frames++;
}

static void mtk_cmdfb_kick_request(struct mtk_cmdfb_kick *k)
{
	atomic_set(&k->pending, 1);
	queue_work(system_dfl_wq, &k->work);
}

static int mtk_cmdfb_kick_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	struct mtk_cmdfb_kick *k = container_of(nb, struct mtk_cmdfb_kick, nb);
	struct drm_sysfb_flush_event *event = data;

	if (event->dev->dev->of_node != k->fb_np)
		return NOTIFY_DONE;

	/*
	 * The framebuffer is mapped write-combined; make the writes visible to
	 * the OVL before another CPU starts the frame.
	 */
	wmb();
	mtk_cmdfb_kick_request(k);

	return NOTIFY_OK;
}

static void mtk_cmdfb_kick_refresh(struct work_struct *work)
{
	struct mtk_cmdfb_kick *k = container_of(to_delayed_work(work),
						struct mtk_cmdfb_kick, refresh);

	if (k->stopped)
		return;

	mtk_cmdfb_kick_request(k);
	schedule_delayed_work(&k->refresh, k->refresh_jiffies);
}

/*
 * Check the pipeline against the state LK leaves and find the OVL layer that
 * scans out the framebuffer. Returns the layer or a negative errno.
 */
static int mtk_cmdfb_kick_check(struct mtk_cmdfb_kick *k, const struct resource *fb,
				u32 width, u32 height, u32 stride)
{
	unsigned int mutex_id = k->data->mutex_id;
	u32 src_con, layers_mask = GENMASK(k->data->ovl_layers - 1, 0);
	u32 mode = readl(k->dsi + DSI_MODE_CTRL);
	u32 cmdq_size = readl(k->dsi + DSI_CMDQ_SIZE);
	u32 cmdq0 = readl(k->dsi + DSI_CMDQ(0));
	u32 mutex_en = readl(k->mutex + MUTEX_EN(mutex_id));
	u32 mutex_mod = readl(k->mutex + MUTEX_MOD(mutex_id));
	u32 roi, pitch;
	int i, layer = -1;

	src_con = readl(k->ovl + OVL_SRC_CON);

	dev_info(k->dev,
		 "LK state: DSI MODE %#x CMDQ_SIZE %#x CMDQ0 %#x; MUTEX%u EN %#x MOD %#x; OVL SRC_CON %#x\n",
		 mode, cmdq_size, cmdq0, mutex_id, mutex_en, mutex_mod, src_con);

	if (mode != 0) {
		dev_err(k->dev, "DSI not in command mode (MODE_CTRL %#x)\n", mode);
		return -ENODEV;
	}
	if (!FIELD_GET(DSI_CMDQ_SIZE_MASK, cmdq_size) || cmdq0 != DSI_CMDQ_WMS) {
		dev_err(k->dev, "DSI command queue is not write_memory_start\n");
		return -ENODEV;
	}
	if (!mutex_mod) {
		dev_err(k->dev, "mutex %u has no modules\n", mutex_id);
		return -ENODEV;
	}
	if (mutex_en & MUTEX_EN_BIT) {
		dev_err(k->dev, "mutex %u still enabled, pipeline not idle\n", mutex_id);
		return -ENODEV;
	}
	if (upper_32_bits(fb->start)) {
		dev_err(k->dev, "framebuffer %pR above 4 GiB\n", fb);
		return -ENODEV;
	}
	if (src_con & GENMASK(3, 0) & ~layers_mask) {
		dev_err(k->dev, "unexpected OVL layers enabled (SRC_CON %#x)\n", src_con);
		return -ENODEV;
	}

	for (i = 0; i < k->data->ovl_layers; i++) {
		u32 addr = readl(k->ovl + OVL_ADDR(i));

		dev_dbg(k->dev, "OVL L%d %s addr %#x\n", i,
			src_con & BIT(i) ? "on" : "off", addr);
		if ((src_con & BIT(i)) && addr == lower_32_bits(fb->start)) {
			layer = i;
			break;
		}
	}
	if (layer < 0) {
		dev_err(k->dev, "no enabled OVL layer scans out %pR\n", fb);
		return -ENODEV;
	}

	/* informational: register layout as on other MediaTek OVLs */
	roi = readl(k->ovl + OVL_ROI_SIZE);
	pitch = FIELD_GET(OVL_PITCH_MASK, readl(k->ovl + OVL_PITCH(layer)));
	if (roi != (height << 16 | width) || pitch != stride)
		dev_warn(k->dev, "OVL ROI %#x pitch %u, framebuffer %ux%u stride %u\n",
			 roi, pitch, width, height, stride);

	return layer;
}

static ssize_t frames_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_cmdfb_kick *k = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%lu\n", k->frames);
}
static DEVICE_ATTR_RO(frames);

static ssize_t timeouts_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_cmdfb_kick *k = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%lu\n", k->timeouts);
}
static DEVICE_ATTR_RO(timeouts);

static ssize_t stopped_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_cmdfb_kick *k = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", k->stopped);
}
static DEVICE_ATTR_RO(stopped);

static struct attribute *mtk_cmdfb_kick_attrs[] = {
	&dev_attr_frames.attr,
	&dev_attr_timeouts.attr,
	&dev_attr_stopped.attr,
	NULL
};
ATTRIBUTE_GROUPS(mtk_cmdfb_kick);

static void mtk_cmdfb_kick_unregister(void *data)
{
	struct mtk_cmdfb_kick *k = data;

	drm_sysfb_unregister_flush_notifier(&k->nb);
}

static void mtk_cmdfb_kick_cancel(void *data)
{
	struct mtk_cmdfb_kick *k = data;

	k->stopped = true;
	cancel_delayed_work_sync(&k->refresh);
	cancel_work_sync(&k->work);
}

static void mtk_cmdfb_kick_put_fb(void *data)
{
	of_node_put(data);
}

static int mtk_cmdfb_kick_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_cmdfb_kick *k;
	struct resource fb;
	struct clk_bulk_data *clks;
	u32 width, height, stride, src_con;
	int layer, ret;

	k = devm_kzalloc(dev, sizeof(*k), GFP_KERNEL);
	if (!k)
		return -ENOMEM;
	k->dev = dev;
	k->data = device_get_match_data(dev);
	INIT_WORK(&k->work, mtk_cmdfb_kick_work);
	INIT_DELAYED_WORK(&k->refresh, mtk_cmdfb_kick_refresh);
	platform_set_drvdata(pdev, k);

	k->fb_np = of_parse_phandle(dev->of_node, "mediatek,framebuffer", 0);
	if (!k->fb_np)
		return dev_err_probe(dev, -EINVAL, "no mediatek,framebuffer\n");
	ret = devm_add_action_or_reset(dev, mtk_cmdfb_kick_put_fb, k->fb_np);
	if (ret)
		return ret;

	ret = of_address_to_resource(k->fb_np, 0, &fb);
	if (ret)
		return dev_err_probe(dev, ret, "framebuffer has no reg\n");
	if (of_property_read_u32(k->fb_np, "width", &width) ||
	    of_property_read_u32(k->fb_np, "height", &height) ||
	    of_property_read_u32(k->fb_np, "stride", &stride))
		return dev_err_probe(dev, -EINVAL, "framebuffer without width/height/stride\n");

	k->mutex = devm_platform_ioremap_resource_byname(pdev, "mutex");
	if (IS_ERR(k->mutex))
		return PTR_ERR(k->mutex);
	k->ovl = devm_platform_ioremap_resource_byname(pdev, "ovl");
	if (IS_ERR(k->ovl))
		return PTR_ERR(k->ovl);
	k->dsi = devm_platform_ioremap_resource_byname(pdev, "dsi");
	if (IS_ERR(k->dsi))
		return PTR_ERR(k->dsi);

	/* already on from LK; holding them keeps them on without clk_ignore_unused */
	ret = devm_clk_bulk_get_all_enabled(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");

	layer = mtk_cmdfb_kick_check(k, &fb, width, height, stride);
	if (layer < 0) {
		dev_err(dev, "hardware not as LK leaves it, not touching the display\n");
		return layer;
	}

	src_con = readl(k->ovl + OVL_SRC_CON);
	if (src_con & GENMASK(k->data->ovl_layers - 1, 0) & ~BIT(layer)) {
		writel(src_con & ~(GENMASK(k->data->ovl_layers - 1, 0) & ~BIT(layer)),
		       k->ovl + OVL_SRC_CON);
		dev_info(dev, "OVL SRC_CON %#x -> %#x (only layer %d)\n",
			 src_con, readl(k->ovl + OVL_SRC_CON), layer);
	}

	ret = devm_add_action_or_reset(dev, mtk_cmdfb_kick_cancel, k);
	if (ret)
		return ret;

	k->nb.notifier_call = mtk_cmdfb_kick_notify;
	ret = drm_sysfb_register_flush_notifier(&k->nb);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register flush notifier\n");
	ret = devm_add_action_or_reset(dev, mtk_cmdfb_kick_unregister, k);
	if (ret)
		return ret;

	/* show what is in the framebuffer now, without the LK overlays */
	mtk_cmdfb_kick_request(k);

	if (refresh_hz) {
		k->refresh_jiffies = max(1UL, msecs_to_jiffies(1000 / min(refresh_hz, 120U)));
		schedule_delayed_work(&k->refresh, k->refresh_jiffies);
	}

	dev_info(dev, "frame trigger for %pR on OVL layer %d, mutex %u%s\n", &fb, layer,
		 k->data->mutex_id, refresh_hz ? ", periodic refresh" : "");

	return 0;
}

static const struct mtk_cmdfb_kick_data mt6878_data = {
	.mutex_id = 0,
	.ovl_layers = 2,	/* OVL0 is a 2-layer OVL */
};

static const struct of_device_id mtk_cmdfb_kick_of_match[] = {
	{ .compatible = "mediatek,mt6878-cmdmode-fb-kick", .data = &mt6878_data },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_cmdfb_kick_of_match);

static struct platform_driver mtk_cmdfb_kick_driver = {
	.probe = mtk_cmdfb_kick_probe,
	.driver = {
		.name = "mtk-cmdfb-kick",
		.of_match_table = mtk_cmdfb_kick_of_match,
		.dev_groups = mtk_cmdfb_kick_groups,
	},
};
module_platform_driver(mtk_cmdfb_kick_driver);

MODULE_DESCRIPTION("Frame trigger for boot framebuffers on MediaTek command-mode DSI");
MODULE_LICENSE("GPL");
