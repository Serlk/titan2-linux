// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keyboard matrix scanned through an Awinic AW9523B I/O expander
 *
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 *
 * Matrix wiring, reset timing and key map are derived from the GPL module
 * aw9523_key.ko shipped with Titan 2 V01.00.14; the AW9523B registers are
 * those of the Awinic datasheet.
 *
 * The AW9523B has 16 I/O lines in two 8-bit ports. Some lines sense the
 * matrix rows (inputs, pulled up on the board), the others drive the
 * columns. While idle every column is driven low and a level change on any
 * row raises the expander interrupt. The matrix is then scanned one column
 * at a time and polled while any key is held; once all keys are released
 * the columns go back low and the row interrupts are re-armed.
 *
 * Each column costs two I2C transfers: a 2-byte write of the port direction
 * registers (only the scanned column is an output) and a 2-byte read of the
 * input registers. The chip auto-increments the register address.
 *
 * The interrupt line is level triggered and only deasserts once the input
 * registers have been read, so the interrupt handler does no I2C at all: it
 * disables the interrupt and leaves the chip to the scan work, which
 * enables it again once the chip is re-armed and the interrupt cleared.
 * All chip accesses therefore happen in the work. Any I2C error, an
 * interrupt that keeps firing without a key change, or a chip that lost
 * its setup (seen by a periodic check while idle) resets and reinitialises
 * the chip, with the interrupt kept off and an increasing back-off.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

/* Register pairs: port 0 at the even address, port 1 at the next one */
#define AW9523_REG_INPUT	0x00	/* reading clears the interrupt */
#define AW9523_REG_OUTPUT	0x02
#define AW9523_REG_CONFIG	0x04	/* 1: input, 0: output */
#define AW9523_REG_INTDIS	0x06	/* 1: interrupt disabled */
#define AW9523_REG_ID		0x10
#define AW9523_REG_GCR		0x11
#define AW9523_GCR_P0_PUSH_PULL	BIT(4)
#define AW9523_REG_MODE		0x12	/* 1: GPIO, 0: LED current sink */
#define AW9523_REG_SOFT_RESET	0x7f

#define AW9523_CHIP_ID		0x23
#define AW9523_NUM_LINES	16
#define AW9523_ALL_LINES	GENMASK(AW9523_NUM_LINES - 1, 0)

#define AW9523_DEF_DEBOUNCE_MS	1
#define AW9523_DEF_POLL_MS	5

/* Idle health check of the chip setup */
#define AW9523_CHECK_MS		10000
/* Interrupts without any key change within a second that mean a stuck line */
#define AW9523_MAX_SPURIOUS	20
/* Reset back-off: 10 ms doubling up to 2 s */
#define AW9523_RESET_MIN_MS	10
#define AW9523_RESET_MAX_MS	2000
/* Every that many failed resets in a row, power cycle the chip first */
#define AW9523_POWER_CYCLE_EVERY	3

/* The interrupt is disabled (by the handler or the recovery path) */
#define AW9523_IRQ_OFF		0

struct aw9523_keypad {
	struct i2c_client *client;
	struct regmap *regmap;
	struct input_dev *input;
	struct gpio_desc *reset;
	struct regulator *vio;
	bool vio_on;
	struct delayed_work work;
	unsigned long flags;

	unsigned int rows;
	unsigned int cols;
	unsigned int row_shift;
	u8 row_line[AW9523_NUM_LINES];
	u8 col_line[AW9523_NUM_LINES];
	u16 row_mask;
	u16 col_mask;

	unsigned int debounce_ms;
	unsigned int poll_ms;
	unsigned int col_scan_delay_us;

	/* Work state, only touched by the work and with the work stopped */
	bool polling;
	bool need_reset;
	unsigned int resets;
	unsigned int spurious;
	unsigned long spurious_start;
	unsigned long next_check;

	/* Pressed rows (bit per row) of every column, as last reported */
	u16 state[AW9523_NUM_LINES];
};

static int aw9523_keypad_write16(struct aw9523_keypad *kp, unsigned int reg,
				 u16 val)
{
	u8 buf[2] = { val & 0xff, val >> 8 };

	return regmap_bulk_write(kp->regmap, reg, buf, sizeof(buf));
}

static int aw9523_keypad_read16(struct aw9523_keypad *kp, unsigned int reg,
				u16 *val)
{
	u8 buf[2];
	int error;

	error = regmap_bulk_read(kp->regmap, reg, buf, sizeof(buf));
	if (error)
		return error;

	*val = buf[0] | buf[1] << 8;
	return 0;
}

static void aw9523_keypad_irq_off(struct aw9523_keypad *kp)
{
	if (!test_and_set_bit(AW9523_IRQ_OFF, &kp->flags))
		disable_irq_nosync(kp->client->irq);
}

static void aw9523_keypad_irq_on(struct aw9523_keypad *kp)
{
	if (test_and_clear_bit(AW9523_IRQ_OFF, &kp->flags))
		enable_irq(kp->client->irq);
}

/* Rows in, columns driven low, row interrupts on, pending interrupt cleared */
static int aw9523_keypad_arm(struct aw9523_keypad *kp, u16 *input)
{
	int error;

	error = aw9523_keypad_write16(kp, AW9523_REG_CONFIG,
				      AW9523_ALL_LINES & ~kp->col_mask);
	if (error)
		return error;

	error = aw9523_keypad_write16(kp, AW9523_REG_INTDIS,
				      AW9523_ALL_LINES & ~kp->row_mask);
	if (error)
		return error;

	return aw9523_keypad_read16(kp, AW9523_REG_INPUT, input);
}

static int aw9523_keypad_scan(struct aw9523_keypad *kp, u16 *state)
{
	unsigned int row, col;
	u16 input;
	int error;

	for (col = 0; col < kp->cols; col++) {
		/* Only the scanned column is an output, the others float */
		error = aw9523_keypad_write16(kp, AW9523_REG_CONFIG,
					      AW9523_ALL_LINES &
					      ~BIT(kp->col_line[col]));
		if (error)
			return error;

		if (kp->col_scan_delay_us)
			fsleep(kp->col_scan_delay_us);

		error = aw9523_keypad_read16(kp, AW9523_REG_INPUT, &input);
		if (error)
			return error;

		state[col] = 0;
		for (row = 0; row < kp->rows; row++)
			if (!(input & BIT(kp->row_line[row])))
				state[col] |= BIT(row);
	}

	return 0;
}

/* Returns true if any key changed state */
static bool aw9523_keypad_report(struct aw9523_keypad *kp, const u16 *state)
{
	const unsigned short *keycodes = kp->input->keycode;
	struct input_dev *input = kp->input;
	unsigned int row, col, code;
	unsigned long changed;
	bool any = false;

	for (col = 0; col < kp->cols; col++) {
		changed = state[col] ^ kp->state[col];
		for_each_set_bit(row, &changed, kp->rows) {
			code = MATRIX_SCAN_CODE(row, col, kp->row_shift);
			input_event(input, EV_MSC, MSC_SCAN, code);
			input_report_key(input, keycodes[code],
					 state[col] & BIT(row));
			any = true;
		}
		kp->state[col] = state[col];
	}

	if (any)
		input_sync(input);

	return any;
}

static void aw9523_keypad_release_all(struct aw9523_keypad *kp)
{
	u16 state[AW9523_NUM_LINES] = { };

	aw9523_keypad_report(kp, state);
}

static void aw9523_keypad_hw_reset(struct aw9523_keypad *kp)
{
	if (kp->reset) {
		gpiod_set_value_cansleep(kp->reset, 1);
		fsleep(1000);
		gpiod_set_value_cansleep(kp->reset, 0);
		fsleep(6000);
	} else {
		/* Best effort: the chip may not even answer */
		regmap_write(kp->regmap, AW9523_REG_SOFT_RESET, 0);
		fsleep(1000);
	}
}

/*
 * As aw9523_power_ctrl in the factory driver: reset asserted while the supply
 * is off, so that the chip is not powered through the reset line.
 */
static int aw9523_keypad_power_cycle(struct aw9523_keypad *kp)
{
	int error;

	if (kp->reset)
		gpiod_set_value_cansleep(kp->reset, 1);
	fsleep(5000);

	if (kp->vio_on) {
		error = regulator_disable(kp->vio);
		if (error)
			return error;
		kp->vio_on = false;
		fsleep(20000);
	}

	error = regulator_enable(kp->vio);
	if (error)
		return error;
	kp->vio_on = true;
	fsleep(5000);

	return 0;
}

static int aw9523_keypad_hw_init(struct aw9523_keypad *kp)
{
	u16 lines = kp->row_mask | kp->col_mask;
	unsigned int id, try;
	u16 input;
	int error;

	/* The factory driver retries the ID read too, right after reset */
	for (try = 0; try < 5; try++) {
		error = regmap_read(kp->regmap, AW9523_REG_ID, &id);
		if (!error)
			break;
		fsleep(5000);
	}
	if (error)
		return error;
	if (id != AW9523_CHIP_ID)
		return -ENODEV;

	error = aw9523_keypad_write16(kp, AW9523_REG_INTDIS, AW9523_ALL_LINES);
	if (error)
		return error;

	/* Matrix lines in GPIO mode, not LED current sink mode */
	error = regmap_bulk_write(kp->regmap, AW9523_REG_MODE,
				  (u8 []){ lines & 0xff, lines >> 8 }, 2);
	if (error)
		return error;

	/* Port 0 push-pull, as set up by the factory driver */
	error = regmap_update_bits(kp->regmap, AW9523_REG_GCR,
				   AW9523_GCR_P0_PUSH_PULL,
				   AW9523_GCR_P0_PUSH_PULL);
	if (error)
		return error;

	error = aw9523_keypad_write16(kp, AW9523_REG_OUTPUT, 0);
	if (error)
		return error;

	return aw9523_keypad_arm(kp, &input);
}

/*
 * The registers a chip reset (power glitch, bus noise hitting the soft
 * reset register) would change and nothing else writes. The direction and
 * interrupt masks are only known while armed, not in the middle of a scan.
 */
static int aw9523_keypad_check(struct aw9523_keypad *kp, bool armed)
{
	unsigned int gcr;
	u8 regs[6];
	u16 output, config, intdis;
	int error;

	/* OUTPUT, CONFIG and INTDIS in one go */
	error = regmap_bulk_read(kp->regmap, AW9523_REG_OUTPUT, regs,
				 sizeof(regs));
	if (error)
		return error;

	output = regs[0] | regs[1] << 8;
	config = regs[2] | regs[3] << 8;
	intdis = regs[4] | regs[5] << 8;
	if (output & kp->col_mask)
		return -EIO;
	if (armed && ((config & kp->col_mask) || (~config & kp->row_mask) ||
		      (intdis & kp->row_mask)))
		return -EIO;

	error = regmap_read(kp->regmap, AW9523_REG_GCR, &gcr);
	if (error)
		return error;
	if (!(gcr & AW9523_GCR_P0_PUSH_PULL))
		return -EIO;

	return 0;
}

/* Called from the work: interrupt off, reset the chip after a back-off */
static void aw9523_keypad_fail(struct aw9523_keypad *kp, const char *what,
			       int error)
{
	unsigned int delay;

	aw9523_keypad_irq_off(kp);

	delay = min(AW9523_RESET_MIN_MS << min(kp->resets, 8U),
		    AW9523_RESET_MAX_MS);
	kp->resets++;
	kp->need_reset = true;
	kp->polling = false;

	dev_warn_ratelimited(&kp->client->dev,
			     "%s (%d), resetting the chip in %u ms\n",
			     what, error, delay);

	mod_delayed_work(system_freezable_wq, &kp->work,
			 msecs_to_jiffies(delay));
}

static void aw9523_keypad_work(struct work_struct *work)
{
	struct aw9523_keypad *kp = container_of(to_delayed_work(work),
						struct aw9523_keypad, work);
	u16 state[AW9523_NUM_LINES];
	bool pressed = false, changed, from_irq;
	unsigned int col;
	u16 input;
	int error;

	if (kp->need_reset) {
		/* Whatever was held is lost, do not leave keys stuck down */
		aw9523_keypad_release_all(kp);

		/* A plain reset did not help a few times: power cycle too */
		if (kp->resets % AW9523_POWER_CYCLE_EVERY == 0) {
			dev_warn(&kp->client->dev, "power cycling the chip\n");
			error = aw9523_keypad_power_cycle(kp);
			if (error) {
				aw9523_keypad_fail(kp, "power cycle failed", error);
				return;
			}
		}

		aw9523_keypad_hw_reset(kp);
		error = aw9523_keypad_hw_init(kp);
		if (error) {
			aw9523_keypad_fail(kp, "chip init failed", error);
			return;
		}

		kp->need_reset = false;
		kp->spurious = 0;
		dev_info(&kp->client->dev, "chip reinitialised\n");
		/* Scan for keys held through the reset */
	} else if (!test_bit(AW9523_IRQ_OFF, &kp->flags)) {
		/*
		 * Idle with the interrupt on: the periodic check. A
		 * concurrent interrupt queues the scan behind this.
		 */
		error = aw9523_keypad_check(kp, true);
		if (error) {
			aw9523_keypad_fail(kp, "chip setup lost", error);
			return;
		}

		kp->resets = 0;
		queue_delayed_work(system_freezable_wq, &kp->work,
				   msecs_to_jiffies(AW9523_CHECK_MS));
		return;
	}

	from_irq = !kp->polling;

	error = aw9523_keypad_scan(kp, state);
	if (error) {
		aw9523_keypad_fail(kp, "matrix scan failed", error);
		return;
	}

	changed = aw9523_keypad_report(kp, state);

	for (col = 0; col < kp->cols; col++)
		pressed |= state[col];

	if (pressed)
		goto poll;

	error = aw9523_keypad_arm(kp, &input);
	if (error) {
		aw9523_keypad_fail(kp, "re-arming failed", error);
		return;
	}

	/* A key went down while re-arming: keep polling */
	if ((input & kp->row_mask) != kp->row_mask)
		goto poll;

	/*
	 * An interrupt that brought no key change: bounce, or a line that
	 * stays asserted. Too many of them mean the latter.
	 */
	if (from_irq && !changed) {
		if (time_after(jiffies, kp->spurious_start + HZ)) {
			kp->spurious_start = jiffies;
			kp->spurious = 0;
		}
		if (++kp->spurious > AW9523_MAX_SPURIOUS) {
			kp->spurious = 0;
			aw9523_keypad_fail(kp, "interrupt stuck", -EIO);
			return;
		}
	}

	if (changed)
		kp->resets = 0;

	/* All rows high and the interrupt cleared: wait for the next one */
	kp->polling = false;
	aw9523_keypad_irq_on(kp);
	queue_delayed_work(system_freezable_wq, &kp->work,
			   msecs_to_jiffies(AW9523_CHECK_MS));
	return;

poll:
	/* Keys held for long: check the chip now and then all the same */
	if (from_irq) {
		kp->next_check = jiffies + msecs_to_jiffies(AW9523_CHECK_MS);
	} else if (time_after(jiffies, kp->next_check)) {
		kp->next_check = jiffies + msecs_to_jiffies(AW9523_CHECK_MS);
		error = aw9523_keypad_check(kp, false);
		if (error) {
			aw9523_keypad_fail(kp, "chip setup lost", error);
			return;
		}
	}

	kp->polling = true;
	if (changed)
		kp->resets = 0;
	mod_delayed_work(system_freezable_wq, &kp->work,
			 msecs_to_jiffies(kp->poll_ms));
}

static irqreturn_t aw9523_keypad_irq(int irq, void *data)
{
	struct aw9523_keypad *kp = data;

	/*
	 * Level triggered, and only reading the chip clears it: keep it off
	 * until the work has scanned and re-armed the chip.
	 */
	aw9523_keypad_irq_off(kp);
	mod_delayed_work(system_freezable_wq, &kp->work,
			 msecs_to_jiffies(kp->debounce_ms));

	/* A key pressed while suspending aborts the suspend */
	if (device_may_wakeup(&kp->client->dev))
		pm_wakeup_event(&kp->client->dev, 0);

	return IRQ_HANDLED;
}

static int aw9523_keypad_read_lines(struct device *dev, const char *prop,
				    u8 *lines, unsigned int *num, u16 *mask)
{
	u32 val[AW9523_NUM_LINES];
	int count, error, i;

	count = device_property_count_u32(dev, prop);
	if (count <= 0 || count > AW9523_NUM_LINES)
		return dev_err_probe(dev, -EINVAL, "invalid %s\n", prop);

	error = device_property_read_u32_array(dev, prop, val, count);
	if (error)
		return dev_err_probe(dev, error, "failed to read %s\n", prop);

	*mask = 0;
	for (i = 0; i < count; i++) {
		if (val[i] >= AW9523_NUM_LINES || (*mask & BIT(val[i])))
			return dev_err_probe(dev, -EINVAL,
					     "invalid line %u in %s\n",
					     val[i], prop);
		lines[i] = val[i];
		*mask |= BIT(val[i]);
	}
	*num = count;

	return 0;
}

static void aw9523_keypad_disable_vio(void *data)
{
	struct aw9523_keypad *kp = data;

	if (kp->vio_on)
		regulator_disable(kp->vio);
}

static void aw9523_keypad_stop(void *data)
{
	struct aw9523_keypad *kp = data;

	/* The work may enable the interrupt again, but not past this */
	disable_irq(kp->client->irq);
	cancel_delayed_work_sync(&kp->work);
}

static const struct regmap_config aw9523_keypad_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AW9523_REG_SOFT_RESET,
};

static int aw9523_keypad_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct aw9523_keypad *kp;
	struct input_dev *input;
	const char *label;
	int error;

	if (!client->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	kp = devm_kzalloc(dev, sizeof(*kp), GFP_KERNEL);
	if (!kp)
		return -ENOMEM;

	kp->client = client;
	kp->spurious_start = jiffies;
	INIT_DELAYED_WORK(&kp->work, aw9523_keypad_work);

	error = aw9523_keypad_read_lines(dev, "awinic,row-lines", kp->row_line,
					 &kp->rows, &kp->row_mask);
	if (error)
		return error;

	error = aw9523_keypad_read_lines(dev, "awinic,column-lines",
					 kp->col_line, &kp->cols,
					 &kp->col_mask);
	if (error)
		return error;

	if (kp->row_mask & kp->col_mask)
		return dev_err_probe(dev, -EINVAL,
				     "a line is both a row and a column\n");

	kp->debounce_ms = AW9523_DEF_DEBOUNCE_MS;
	device_property_read_u32(dev, "debounce-delay-ms", &kp->debounce_ms);
	kp->poll_ms = AW9523_DEF_POLL_MS;
	device_property_read_u32(dev, "poll-interval", &kp->poll_ms);
	device_property_read_u32(dev, "col-scan-delay-us",
				 &kp->col_scan_delay_us);

	kp->regmap = devm_regmap_init_i2c(client, &aw9523_keypad_regmap_config);
	if (IS_ERR(kp->regmap))
		return PTR_ERR(kp->regmap);

	kp->vio = devm_regulator_get(dev, "vio");
	if (IS_ERR(kp->vio))
		return dev_err_probe(dev, PTR_ERR(kp->vio), "failed to get vio\n");

	error = regulator_enable(kp->vio);
	if (error)
		return dev_err_probe(dev, error, "failed to enable vio\n");
	kp->vio_on = true;

	error = devm_add_action_or_reset(dev, aw9523_keypad_disable_vio, kp);
	if (error)
		return error;

	/* Held in reset until aw9523_keypad_hw_reset() */
	kp->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(kp->reset))
		return dev_err_probe(dev, PTR_ERR(kp->reset),
				     "failed to get reset\n");

	aw9523_keypad_hw_reset(kp);

	error = aw9523_keypad_hw_init(kp);
	if (error)
		return dev_err_probe(dev, error, "failed to initialise\n");

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	kp->input = input;
	input->name = "aw9523-keypad";
	if (!device_property_read_string(dev, "label", &label))
		input->name = label;
	input->phys = "aw9523-keypad/input0";
	input->id.bustype = BUS_I2C;

	kp->row_shift = get_count_order(kp->cols);
	error = matrix_keypad_build_keymap(NULL, NULL, kp->rows, kp->cols,
					   NULL, input);
	if (error)
		return dev_err_probe(dev, error, "failed to build keymap\n");

	input_set_capability(input, EV_MSC, MSC_SCAN);
	if (device_property_read_bool(dev, "autorepeat"))
		__set_bit(EV_REP, input->evbit);

	error = input_register_device(input);
	if (error)
		return error;

	/*
	 * The trigger type comes from the firmware node. Not threaded: the
	 * handler only disables the interrupt and queues the scan.
	 */
	error = devm_request_irq(dev, client->irq, aw9523_keypad_irq, 0,
				 dev_name(dev), kp);
	if (error)
		return dev_err_probe(dev, error, "failed to request irq\n");

	error = devm_add_action_or_reset(dev, aw9523_keypad_stop, kp);
	if (error)
		return error;

	i2c_set_clientdata(client, kp);
	device_init_wakeup(dev, device_property_read_bool(dev, "wakeup-source"));

	/* Start the periodic check */
	queue_delayed_work(system_freezable_wq, &kp->work,
			   msecs_to_jiffies(AW9523_CHECK_MS));

	return 0;
}

static int aw9523_keypad_suspend(struct device *dev)
{
	struct aw9523_keypad *kp = dev_get_drvdata(dev);
	u16 input;

	disable_irq(kp->client->irq);
	cancel_delayed_work_sync(&kp->work);
	/* Only the disable_irq() above is left */
	if (test_and_clear_bit(AW9523_IRQ_OFF, &kp->flags))
		enable_irq(kp->client->irq);
	kp->polling = false;

	/* Leave the chip armed so that any key can wake the system up */
	if (kp->need_reset) {
		aw9523_keypad_hw_reset(kp);
		if (!aw9523_keypad_hw_init(kp))
			kp->need_reset = false;
	} else {
		aw9523_keypad_arm(kp, &input);
	}

	if (device_may_wakeup(dev)) {
		enable_irq_wake(kp->client->irq);
		enable_irq(kp->client->irq);
	}

	return 0;
}

static int aw9523_keypad_resume(struct device *dev)
{
	struct aw9523_keypad *kp = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(kp->client->irq);
	else
		enable_irq(kp->client->irq);

	/* Report whatever changed while asleep, including the wakeup key */
	aw9523_keypad_irq_off(kp);
	mod_delayed_work(system_freezable_wq, &kp->work, 0);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(aw9523_keypad_pm_ops,
				aw9523_keypad_suspend, aw9523_keypad_resume);

static const struct of_device_id aw9523_keypad_of_match[] = {
	{ .compatible = "awinic,aw9523b-keypad" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw9523_keypad_of_match);

static const struct i2c_device_id aw9523_keypad_id[] = {
	{ "aw9523b-keypad" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw9523_keypad_id);

static struct i2c_driver aw9523_keypad_driver = {
	.driver = {
		.name = "aw9523b-keypad",
		.of_match_table = aw9523_keypad_of_match,
		.pm = pm_sleep_ptr(&aw9523_keypad_pm_ops),
	},
	.probe = aw9523_keypad_probe,
	.id_table = aw9523_keypad_id,
};
module_i2c_driver(aw9523_keypad_driver);

MODULE_DESCRIPTION("Awinic AW9523B keyboard matrix driver");
MODULE_LICENSE("GPL");
