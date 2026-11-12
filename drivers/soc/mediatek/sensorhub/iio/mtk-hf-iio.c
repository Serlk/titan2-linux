// SPDX-License-Identifier: GPL-2.0-only
/*
 * IIO devices for the sensors of the MediaTek SCP sensor hub
 *
 * Copyright (C) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 *
 * On the MT6878 (Unihertz Titan 2) the accelerometer/gyroscope, the
 * magnetometer and the ambient light/proximity sensor sit behind the SCP;
 * its firmware delivers samples to the kernel's hf_manager (the MediaTek
 * sensor hub core). This driver is an in-kernel hf_manager client that
 * exposes the sensors the hub lists as ordinary IIO devices, so that Linux
 * user space (iio-sensor-proxy, the desktop) reads them like any IIO
 * sensor:
 *
 *   accelerometer   in_accel_{x,y,z}_raw     * in_accel_scale      m/s^2
 *   gyroscope       in_anglvel_{x,y,z}_raw   * in_anglvel_scale    rad/s
 *   magnetometer    in_magn_{x,y,z}_raw      * in_magn_scale       gauss
 *   light           in_illuminance_raw       * in_illuminance_scale lux
 *   proximity       in_proximity_raw: closeness 0..5 (5 - distance in cm),
 *                   in_proximity_nearlevel = 5 - near_cm
 *
 * The hub reports value * gain (the gain is per sensor, from the hub's
 * sensor list): the scale is 1 / gain (and / 100 for the magnetometer,
 * whose hub unit is the microtesla). The proximity sensor reports a
 * distance in centimetres (0 or 5 for the stk3a5x); IIO proximity grows as
 * an object comes closer, so raw is 5 - distance, clamped to 0..5, and
 * near is below near_cm (module parameter). iio-sensor-proxy takes
 * raw > nearlevel * 1.1 as near.
 *
 * There is no buffer or trigger: a sensor is enabled at the hub when one
 * of its values is read, the read waits for the first sample, and the
 * sensor is disabled again after idle_ms without reads. That suits the
 * polling backends of iio-sensor-proxy.
 */

#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "hf_manager.h"
#include "hf_sensor_io.h"
#include "hf_sensor_type.h"

static unsigned int idle_ms = 3000;
module_param(idle_ms, uint, 0644);
MODULE_PARM_DESC(idle_ms, "Disable a sensor at the hub after this many ms without reads");

static unsigned int period_ms = 100;
module_param(period_ms, uint, 0644);
MODULE_PARM_DESC(period_ms, "Sampling period requested from the hub (ms)");

static unsigned int first_sample_ms = 500;
module_param(first_sample_ms, uint, 0644);
MODULE_PARM_DESC(first_sample_ms, "How long a read waits for the first sample after enabling (ms)");

static unsigned int near_cm = 1;
module_param(near_cm, uint, 0644);
MODULE_PARM_DESC(near_cm, "Proximity: a reported distance below this many cm is near");

struct hf_iio_sensor;

struct hf_iio {
	struct device *dev;
	struct hf_client *client;
	struct task_struct *reader;
	struct delayed_work idle_work;
	struct delayed_work find_work;	/* waits for the hub's sensor list */
	unsigned int find_tries;
	struct mutex lock;		/* enable state of all sensors */
	struct hf_iio_sensor *sensors[SENSOR_TYPE_SENSOR_MAX];
};

struct hf_iio_sensor {
	struct hf_iio *hub;
	struct iio_dev *indio_dev;
	u8 type;
	u32 gain;
	int scale_div;			/* extra divisor of the scale (magnetometer) */
	bool enabled;
	unsigned long last_read;	/* jiffies */
	wait_queue_head_t wq;
	spinlock_t sample_lock;
	bool have_sample;
	s64 timestamp;
	s32 word[6];
};

/* ---- the hub ---- */

static int hf_iio_enable(struct hf_iio_sensor *s, bool on)
{
	struct hf_manager_cmd cmd = { };
	struct hf_manager_batch batch = {
		.delay = (s64)period_ms * NSEC_PER_MSEC,
		.latency = 0,
	};
	int ret;

	cmd.sensor_type = s->type;
	cmd.action = on ? HF_MANAGER_SENSOR_ENABLE : HF_MANAGER_SENSOR_DISABLE;
	if (on) {
		BUILD_BUG_ON(sizeof(batch) > sizeof(cmd.data));
		memcpy(cmd.data, &batch, sizeof(batch));
		cmd.length = sizeof(batch);
	}
	ret = hf_client_control_sensor(s->hub->client, &cmd);
	if (ret < 0)
		dev_warn(s->hub->dev, "sensor %u %s failed: %d\n", s->type,
			 on ? "enable" : "disable", ret);
	return ret;
}

/* enable the sensor if needed and wait for a sample; called with hub->lock */
static int hf_iio_get(struct hf_iio_sensor *s, s32 word[6])
{
	unsigned long flags;
	long left;
	int ret;

	s->last_read = jiffies;
	if (!s->enabled) {
		spin_lock_irqsave(&s->sample_lock, flags);
		s->have_sample = false;
		spin_unlock_irqrestore(&s->sample_lock, flags);
		ret = hf_iio_enable(s, true);
		if (ret < 0)
			return ret;
		s->enabled = true;
		mod_delayed_work(system_wq, &s->hub->idle_work,
				 msecs_to_jiffies(idle_ms));
	}
	left = wait_event_interruptible_timeout(s->wq, READ_ONCE(s->have_sample),
						msecs_to_jiffies(first_sample_ms));
	if (left < 0)
		return left;
	if (!left)
		return -EAGAIN;
	spin_lock_irqsave(&s->sample_lock, flags);
	memcpy(word, s->word, sizeof(s->word));
	spin_unlock_irqrestore(&s->sample_lock, flags);
	return 0;
}

static void hf_iio_idle_work(struct work_struct *work)
{
	struct hf_iio *hub = container_of(to_delayed_work(work), struct hf_iio,
					  idle_work);
	unsigned long idle = msecs_to_jiffies(idle_ms), next = 0;
	bool any = false;
	int i;

	mutex_lock(&hub->lock);
	for (i = 0; i < SENSOR_TYPE_SENSOR_MAX; i++) {
		struct hf_iio_sensor *s = hub->sensors[i];

		if (!s || !s->enabled)
			continue;
		if (time_after_eq(jiffies, s->last_read + idle)) {
			hf_iio_enable(s, false);
			s->enabled = false;
		} else {
			unsigned long left = s->last_read + idle - jiffies;

			if (!any || left < next)
				next = left;
			any = true;
		}
	}
	if (any)
		mod_delayed_work(system_wq, &hub->idle_work, next + 1);
	mutex_unlock(&hub->lock);
}

static int hf_iio_reader(void *data)
{
	struct hf_iio *hub = data;
	struct hf_manager_event ev[8];
	int n, i;

	while (!kthread_should_stop()) {
		n = hf_client_poll_sensor_timeout(hub->client, ev, ARRAY_SIZE(ev),
						  msecs_to_jiffies(500));
		if (n <= 0)
			continue;	/* timeout, signal or nothing */
		for (i = 0; i < n; i++) {
			struct hf_iio_sensor *s;
			unsigned long flags;

			if (ev[i].action != DATA_ACTION ||
			    ev[i].sensor_type >= SENSOR_TYPE_SENSOR_MAX)
				continue;
			s = hub->sensors[ev[i].sensor_type];
			if (!s)
				continue;
			spin_lock_irqsave(&s->sample_lock, flags);
			memcpy(s->word, ev[i].word, sizeof(s->word));
			s->timestamp = ev[i].timestamp;
			s->have_sample = true;
			spin_unlock_irqrestore(&s->sample_lock, flags);
			wake_up_interruptible(&s->wq);
		}
	}
	return 0;
}

/* ---- IIO ---- */

#define HF_IIO_AXIS(_type, _axis, _idx) {				\
	.type = (_type),						\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.address = (_idx),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.ext_info = hf_iio_ext_info,					\
}

static ssize_t hf_iio_read_words(struct iio_dev *indio_dev, uintptr_t priv,
				 const struct iio_chan_spec *chan, char *buf)
{
	struct hf_iio_sensor *s = iio_priv(indio_dev);
	s32 w[6];
	int ret;

	mutex_lock(&s->hub->lock);
	ret = hf_iio_get(s, w);
	mutex_unlock(&s->hub->lock);
	if (ret)
		return ret;
	return sysfs_emit(buf, "%d %d %d %d %d %d\n", w[0], w[1], w[2], w[3], w[4], w[5]);
}

/*
 * hub_cali: calibration words for the hub, as the user-space HAL sends them
 * (HF_MANAGER_SENSOR_CONFIG_CALI: up to 12 32-bit words, e.g. the x/y/z
 * offsets of the accelerometer). Written as decimal numbers separated by
 * spaces; the hub keeps them while it runs.
 */
static ssize_t hf_iio_write_cali(struct iio_dev *indio_dev, uintptr_t priv,
				 const struct iio_chan_spec *chan,
				 const char *buf, size_t len)
{
	struct hf_iio_sensor *s = iio_priv(indio_dev);
	struct hf_manager_cmd cmd = { };
	s32 w[12];
	char *copy, *p, *tok;
	int n = 0, ret;

	copy = kstrndup(buf, len, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;
	p = copy;
	while ((tok = strsep(&p, " \t\n")) && n < ARRAY_SIZE(w)) {
		if (!*tok)
			continue;
		ret = kstrtos32(tok, 0, &w[n]);
		if (ret) {
			kfree(copy);
			return ret;
		}
		n++;
	}
	kfree(copy);
	if (!n)
		return -EINVAL;

	cmd.sensor_type = s->type;
	cmd.action = HF_MANAGER_SENSOR_CONFIG_CALI;
	cmd.length = n * sizeof(s32);
	memcpy(cmd.data, w, cmd.length);
	mutex_lock(&s->hub->lock);
	ret = hf_client_control_sensor(s->hub->client, &cmd);
	mutex_unlock(&s->hub->lock);
	if (ret < 0)
		return ret;
	dev_info(s->hub->dev, "%s: %d calibration word(s) sent to the hub\n",
		 indio_dev->name, n);
	return len;
}

static const struct iio_mount_matrix hf_iio_identity = {
	.rotation = { "1", "0", "0", "0", "1", "0", "0", "0", "1" },
};

static const struct iio_mount_matrix *
hf_iio_get_mount_matrix(const struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan)
{
	return &hf_iio_identity;
}

/* hub_words: the raw sample words, for checks against the user-space HAL */
static const struct iio_chan_spec_ext_info hf_iio_ext_info[] = {
	{ .name = "hub_words", .shared = IIO_SHARED_BY_ALL, .read = hf_iio_read_words },
	{ .name = "hub_cali", .shared = IIO_SHARED_BY_ALL, .write = hf_iio_write_cali },
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, hf_iio_get_mount_matrix),
	{ }
};

/* in_proximity_nearlevel: iio-sensor-proxy takes raw > 1.1 * nearlevel as near */
static ssize_t hf_iio_read_nearlevel(struct iio_dev *indio_dev, uintptr_t priv,
				     const struct iio_chan_spec *chan, char *buf)
{
	return sysfs_emit(buf, "%u\n", 5 - min(near_cm, 5U));
}

static const struct iio_chan_spec_ext_info hf_iio_prox_ext_info[] = {
	{ .name = "hub_words", .shared = IIO_SHARED_BY_ALL, .read = hf_iio_read_words },
	{ .name = "hub_cali", .shared = IIO_SHARED_BY_ALL, .write = hf_iio_write_cali },
	{ .name = "nearlevel", .shared = IIO_SEPARATE, .read = hf_iio_read_nearlevel },
	{ }
};

static const struct iio_chan_spec hf_iio_accel[] = {
	HF_IIO_AXIS(IIO_ACCEL, X, 0), HF_IIO_AXIS(IIO_ACCEL, Y, 1), HF_IIO_AXIS(IIO_ACCEL, Z, 2),
};
static const struct iio_chan_spec hf_iio_gyro[] = {
	HF_IIO_AXIS(IIO_ANGL_VEL, X, 0), HF_IIO_AXIS(IIO_ANGL_VEL, Y, 1), HF_IIO_AXIS(IIO_ANGL_VEL, Z, 2),
};
static const struct iio_chan_spec hf_iio_magn[] = {
	HF_IIO_AXIS(IIO_MAGN, X, 0), HF_IIO_AXIS(IIO_MAGN, Y, 1), HF_IIO_AXIS(IIO_MAGN, Z, 2),
};
static const struct iio_chan_spec hf_iio_light[] = {
	{
		.type = IIO_LIGHT,
		.address = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.ext_info = hf_iio_ext_info,
	},
};
static const struct iio_chan_spec hf_iio_prox[] = {
	{
		.type = IIO_PROXIMITY,
		.address = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.ext_info = hf_iio_prox_ext_info,
	},
};

static int hf_iio_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct hf_iio_sensor *s = iio_priv(indio_dev);
	s32 w[6];
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&s->hub->lock);
		ret = hf_iio_get(s, w);
		mutex_unlock(&s->hub->lock);
		if (ret)
			return ret;
		if (chan->type == IIO_PROXIMITY)
			*val = clamp_t(s32, 5 - w[0] / (s32)s->gain, 0, 5);
		else
			*val = w[chan->address];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		*val2 = s->gain * s->scale_div;
		return IIO_VAL_FRACTIONAL;
	}
	return -EINVAL;
}

static const struct iio_info hf_iio_info = {
	.read_raw = hf_iio_read_raw,
};

static const struct {
	u8 type;
	const char *name;
	const struct iio_chan_spec *chans;
	int n;
	int scale_div;
} hf_iio_types[] = {
	{ SENSOR_TYPE_ACCELEROMETER, "hub-accel", hf_iio_accel, ARRAY_SIZE(hf_iio_accel), 1 },
	{ SENSOR_TYPE_GYROSCOPE, "hub-gyro", hf_iio_gyro, ARRAY_SIZE(hf_iio_gyro), 1 },
	/* the hub reports microtesla, IIO wants gauss: 1 G = 100 uT */
	{ SENSOR_TYPE_MAGNETIC_FIELD, "hub-magn", hf_iio_magn, ARRAY_SIZE(hf_iio_magn), 100 },
	{ SENSOR_TYPE_LIGHT, "hub-light", hf_iio_light, ARRAY_SIZE(hf_iio_light), 1 },
	{ SENSOR_TYPE_PROXIMITY, "hub-prox", hf_iio_prox, ARRAY_SIZE(hf_iio_prox), 1 },
};

static void hf_iio_stop(void *data)
{
	struct hf_iio *hub = data;
	int i;

	cancel_delayed_work_sync(&hub->find_work);
	cancel_delayed_work_sync(&hub->idle_work);
	if (hub->reader)
		kthread_stop(hub->reader);
	mutex_lock(&hub->lock);
	for (i = 0; i < SENSOR_TYPE_SENSOR_MAX; i++)
		if (hub->sensors[i] && hub->sensors[i]->enabled) {
			hf_iio_enable(hub->sensors[i], false);
			hub->sensors[i]->enabled = false;
		}
	mutex_unlock(&hub->lock);
	hf_client_destroy(hub->client);
}

/* one IIO device per sensor the hub lists; returns how many */
static int hf_iio_find(struct hf_iio *hub)
{
	struct device *dev = hub->dev;
	int i, ret, found = 0;

	for (i = 0; i < ARRAY_SIZE(hf_iio_types); i++) {
		struct sensor_info info = { };
		struct iio_dev *indio_dev;
		struct hf_iio_sensor *s;
		u8 type = hf_iio_types[i].type;

		if (hub->sensors[type] || hf_client_find_sensor(hub->client, type))
			continue;
		if (hf_client_get_sensor_info(hub->client, type, &info) || !info.gain) {
			dev_warn(dev, "sensor %u: no info from the hub\n", type);
			continue;
		}
		indio_dev = devm_iio_device_alloc(dev, sizeof(*s));
		if (!indio_dev)
			return -ENOMEM;
		s = iio_priv(indio_dev);
		s->hub = hub;
		s->indio_dev = indio_dev;
		s->type = type;
		s->gain = info.gain;
		s->scale_div = hf_iio_types[i].scale_div;
		init_waitqueue_head(&s->wq);
		spin_lock_init(&s->sample_lock);

		indio_dev->name = hf_iio_types[i].name;
		indio_dev->info = &hf_iio_info;
		indio_dev->modes = INDIO_DIRECT_MODE;
		indio_dev->channels = hf_iio_types[i].chans;
		indio_dev->num_channels = hf_iio_types[i].n;
		hub->sensors[type] = s;
		ret = devm_iio_device_register(dev, indio_dev);
		if (ret) {
			hub->sensors[type] = NULL;
			dev_err(dev, "%s: IIO register failed: %d\n", indio_dev->name, ret);
			continue;
		}
		dev_info(dev, "%s: hub sensor %u '%.16s' (%.16s), gain %u\n",
			 hf_iio_types[i].name, type, info.name, info.vendor, info.gain);
		found++;
	}
	return found;
}

/* the hub lists its sensors once the SCP firmware is ready: look again for a minute */
static void hf_iio_find_work(struct work_struct *work)
{
	struct hf_iio *hub = container_of(to_delayed_work(work), struct hf_iio,
					  find_work);
	int found = hf_iio_find(hub);

	if (found > 0)
		return;
	if (++hub->find_tries < 60)
		schedule_delayed_work(&hub->find_work, HZ);
	else
		dev_warn(hub->dev, "the sensor hub listed no sensors (SCP not ready?)\n");
}

static int hf_iio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hf_iio *hub;
	int ret;

	hub = devm_kzalloc(dev, sizeof(*hub), GFP_KERNEL);
	if (!hub)
		return -ENOMEM;
	hub->dev = dev;
	mutex_init(&hub->lock);
	INIT_DELAYED_WORK(&hub->idle_work, hf_iio_idle_work);
	INIT_DELAYED_WORK(&hub->find_work, hf_iio_find_work);

	hub->client = hf_client_create();
	if (!hub->client)
		return -ENOMEM;
	hub->reader = kthread_run(hf_iio_reader, hub, "hf-iio");
	if (IS_ERR(hub->reader)) {
		ret = PTR_ERR(hub->reader);
		hf_client_destroy(hub->client);
		return ret;
	}
	ret = devm_add_action_or_reset(dev, hf_iio_stop, hub);
	if (ret)
		return ret;

	if (hf_iio_find(hub) <= 0)
		schedule_delayed_work(&hub->find_work, HZ);
	return 0;
}

static struct platform_driver hf_iio_driver = {
	.probe = hf_iio_probe,
	.driver = {
		.name = "mtk-hf-iio",
	},
};

/*
 * No DT node of its own: the device is created with the module; the IIO
 * devices appear when the sensor hub has listed its sensors.
 */
static struct platform_device *hf_iio_pdev;

static int __init hf_iio_init(void)
{
	int ret;

	ret = platform_driver_register(&hf_iio_driver);
	if (ret)
		return ret;
	hf_iio_pdev = platform_device_register_simple("mtk-hf-iio", -1, NULL, 0);
	if (IS_ERR(hf_iio_pdev)) {
		platform_driver_unregister(&hf_iio_driver);
		return PTR_ERR(hf_iio_pdev);
	}
	return 0;
}
module_init(hf_iio_init);

static void __exit hf_iio_exit(void)
{
	platform_device_unregister(hf_iio_pdev);
	platform_driver_unregister(&hf_iio_driver);
}
module_exit(hf_iio_exit);

MODULE_DESCRIPTION("IIO devices for the MediaTek SCP sensor hub (hf_manager client)");
MODULE_LICENSE("GPL");
