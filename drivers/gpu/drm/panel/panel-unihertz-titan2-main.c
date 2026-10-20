// SPDX-License-Identifier: GPL-2.0-only
/*
 * Main display panel of the Unihertz Titan 2 (Agold part "A61QWERTY46"):
 * 1440x1440 LCD on a 4-lane MIPI DSI link in command mode, with its own
 * frame memory and a tearing-effect output.
 *
 * The power sequence, the command set and the timings are derived from
 * the GPL v2 module panel-A61QWERTY46.ko shipped with Titan 2 V01.00.14
 * (its source is not published); nothing in the controller is programmed
 * beyond standard DCS commands.
 *
 * The boot loader leaves the panel on, showing its picture from the panel
 * frame memory: the first prepare takes it over without a reset. Once on,
 * the panel only goes to sleep and back (DCS sleep in/out) with the
 * display; its power is cut when the driver goes away.
 *
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#define TITAN2_MAIN_WIDTH	1440
#define TITAN2_MAIN_HEIGHT	1440

/*
 * Power switches in the order the factory driver turns them on, with the
 * delay it waits after each one (microseconds).
 */
enum { SUPPLY_PM, SUPPLY_VCI, SUPPLY_LCM, SUPPLY_LCD };

static const struct {
	const char *name;
	unsigned int on_delay_us;
} titan2_main_supplies[] = {
	[SUPPLY_PM]  = { "pm",  1000 },
	[SUPPLY_VCI] = { "vci", 1000 },
	[SUPPLY_LCM] = { "lcm", 1000 },
	[SUPPLY_LCD] = { "lcd", 5000 },
};

#define TITAN2_MAIN_NUM_SUPPLIES ARRAY_SIZE(titan2_main_supplies)

enum titan2_main_state {
	TITAN2_MAIN_OFF,
	/* on and initialized by the boot loader, not touched yet */
	TITAN2_MAIN_BOOT_ON,
	/* powered and initialized, in sleep mode */
	TITAN2_MAIN_SLEEP,
	TITAN2_MAIN_ON,
};

struct titan2_main_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *supplies[TITAN2_MAIN_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *touch_reset_gpio;
	/* supplies held by us; set at probe when the boot loader left them on */
	bool powered;
	enum titan2_main_state state;
};

static inline struct titan2_main_panel *to_titan2_main(struct drm_panel *panel)
{
	return container_of(panel, struct titan2_main_panel, panel);
}

static int titan2_main_power_on(struct titan2_main_panel *ctx)
{
	unsigned int i;
	int ret;

	/* supplies already on (boot loader, sleep): only reset and initialize */
	if (ctx->powered)
		goto reset;

	for (i = 0; i < TITAN2_MAIN_NUM_SUPPLIES; i++) {
		ret = regulator_enable(ctx->supplies[i]);
		if (ret) {
			dev_err(&ctx->dsi->dev, "failed to enable %s supply: %d\n",
				titan2_main_supplies[i].name, ret);
			goto err_disable;
		}
		fsleep(titan2_main_supplies[i].on_delay_us);
	}
	ctx->powered = true;

reset:
	/* the touch controller sharing the panel is reset with it */
	if (ctx->touch_reset_gpio) {
		gpiod_direction_output(ctx->touch_reset_gpio, 1);
		usleep_range(2000, 3000);
		gpiod_set_value_cansleep(ctx->touch_reset_gpio, 0);
		usleep_range(2000, 3000);
	}

	/*
	 * The factory driver only releases the reset line here (it is held
	 * from power down). Assert it first as well, so that the controller
	 * is reset when the boot loader left the panel running.
	 */
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);

	return 0;

err_disable:
	while (i--)
		regulator_disable(ctx->supplies[i]);
	return ret;
}

/* the stock power down order: VCI, LCM, LCD, then the main switch */
static void titan2_main_power_off(struct titan2_main_panel *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	if (ctx->touch_reset_gpio)
		gpiod_set_value_cansleep(ctx->touch_reset_gpio, 1);
	msleep(19);

	regulator_disable(ctx->supplies[SUPPLY_VCI]);
	usleep_range(500, 1000);
	regulator_disable(ctx->supplies[SUPPLY_LCM]);
	msleep(19);
	regulator_disable(ctx->supplies[SUPPLY_LCD]);
	regulator_disable(ctx->supplies[SUPPLY_PM]);
	ctx->powered = false;
	ctx->state = TITAN2_MAIN_OFF;
}

static int titan2_main_on(struct titan2_main_panel *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	/* brightness control on, display dimming and backlight enabled */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x2c);
	mipi_dsi_dcs_set_column_address_multi(&dsi_ctx, 0, TITAN2_MAIN_WIDTH - 1);
	mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0, TITAN2_MAIN_HEIGHT - 1);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_ADDRESS_MODE, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_PIXEL_FORMAT,
				     MIPI_DCS_PIXEL_FMT_24BIT << 4 |
				     MIPI_DCS_PIXEL_FMT_24BIT);
	/*
	 * The panel brightness stays at its maximum: the backlight level is
	 * set by the SoC PWM that drives the LED driver.
	 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0xff);
	mipi_dsi_dcs_set_tear_scanline_multi(&dsi_ctx, 1000);
	/* content adaptive brightness: user interface mode, no lower limit */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_CABC_MIN_BRIGHTNESS, 0x00);

	/* the factory driver turns the display on before leaving sleep mode */
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	/*
	 * The factory driver narrows the column window to 1080 pixels here and
	 * relies on the host to send the window again with every frame.
	 * Restore the full frame instead.
	 */
	mipi_dsi_dcs_set_column_address_multi(&dsi_ctx, 0, TITAN2_MAIN_WIDTH - 1);
	mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0, TITAN2_MAIN_HEIGHT - 1);

	return dsi_ctx.accum_err;
}

/*
 * Sleep out, then display on: the order of the standard DCS wake up, seen
 * working on the device; the panel keeps its frame memory meanwhile.
 */
static int titan2_main_exit_sleep(struct titan2_main_panel *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int titan2_main_prepare(struct drm_panel *panel)
{
	struct titan2_main_panel *ctx = to_titan2_main(panel);
	int ret;

	switch (ctx->state) {
	case TITAN2_MAIN_ON:
		return 0;
	case TITAN2_MAIN_BOOT_ON:
		/* keep the boot picture until the first frame */
		dev_info(&ctx->dsi->dev, "taking over the panel from the boot loader\n");
		ctx->state = TITAN2_MAIN_ON;
		return 0;
	case TITAN2_MAIN_SLEEP:
		ret = titan2_main_exit_sleep(ctx);
		if (!ret) {
			ctx->state = TITAN2_MAIN_ON;
			return 0;
		}
		/* reset and initialize it again, the supplies stay on */
		dev_err(&ctx->dsi->dev, "failed to wake the panel up: %d\n", ret);
		break;
	case TITAN2_MAIN_OFF:
		break;
	}

	ret = titan2_main_power_on(ctx);
	if (ret)
		return ret;

	ret = titan2_main_on(ctx);
	if (ret) {
		dev_err(&ctx->dsi->dev, "failed to initialize panel: %d\n", ret);
		titan2_main_power_off(ctx);
		return ret;
	}
	ctx->state = TITAN2_MAIN_ON;

	return 0;
}

/* The stock sequence and delays, without cutting the power afterwards */
static int titan2_main_enter_sleep(struct titan2_main_panel *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 50);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 150);
	if (dsi_ctx.accum_err)
		dev_err(&ctx->dsi->dev, "failed to put panel to sleep: %d\n",
			dsi_ctx.accum_err);

	return dsi_ctx.accum_err;
}

static int titan2_main_unprepare(struct drm_panel *panel)
{
	struct titan2_main_panel *ctx = to_titan2_main(panel);

	if (ctx->state != TITAN2_MAIN_ON)
		return 0;

	titan2_main_enter_sleep(ctx);
	ctx->state = TITAN2_MAIN_SLEEP;

	return 0;
}

/* 60 Hz; the porches only matter for the link rate, the panel paces itself */
static const struct drm_display_mode titan2_main_mode = {
	.clock = (1440 + 25 + 10 + 25) * (1440 + 10 + 4 + 2) * 60 / 1000,
	.hdisplay = 1440,
	.hsync_start = 1440 + 25,
	.hsync_end = 1440 + 25 + 10,
	.htotal = 1440 + 25 + 10 + 25,
	.vdisplay = 1440,
	.vsync_start = 1440 + 10,
	.vsync_end = 1440 + 10 + 4,
	.vtotal = 1440 + 10 + 4 + 2,
	/* 4.5-inch diagonal; the factory driver reports 73 x 88 mm */
	.width_mm = 81,
	.height_mm = 81,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int titan2_main_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &titan2_main_mode);
}

/*
 * The boot loader leaves the panel on. Hold the supplies when all of them
 * are still enabled, so that they are not switched off as unused before
 * the display pipeline comes up; prepare() then skips the power-up.
 */
static int titan2_main_hold_boot_supplies(struct titan2_main_panel *ctx)
{
	unsigned int i;
	int ret;

	for (i = 0; i < TITAN2_MAIN_NUM_SUPPLIES; i++)
		if (regulator_is_enabled(ctx->supplies[i]) <= 0)
			return 0;

	/* powered, but held in reset: not initialized */
	if (gpiod_get_value_cansleep(ctx->reset_gpio) != 0)
		return 0;

	for (i = 0; i < TITAN2_MAIN_NUM_SUPPLIES; i++) {
		ret = regulator_enable(ctx->supplies[i]);
		if (ret) {
			while (i--)
				regulator_disable(ctx->supplies[i]);
			return ret;
		}
	}
	ctx->powered = true;
	ctx->state = TITAN2_MAIN_BOOT_ON;

	return 0;
}

/* Give the supplies back without changing anything at the panel */
static void titan2_main_release_supplies(struct titan2_main_panel *ctx)
{
	unsigned int i;

	if (!ctx->powered)
		return;

	for (i = 0; i < TITAN2_MAIN_NUM_SUPPLIES; i++)
		regulator_disable(ctx->supplies[i]);
	ctx->powered = false;
}

static const struct drm_panel_funcs titan2_main_panel_funcs = {
	.prepare = titan2_main_prepare,
	.unprepare = titan2_main_unprepare,
	.get_modes = titan2_main_get_modes,
};

static int titan2_main_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct titan2_main_panel *ctx;
	unsigned int i;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct titan2_main_panel, panel,
				   &titan2_main_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	for (i = 0; i < TITAN2_MAIN_NUM_SUPPLIES; i++) {
		ctx->supplies[i] = devm_regulator_get(dev,
						      titan2_main_supplies[i].name);
		if (IS_ERR(ctx->supplies[i]))
			return dev_err_probe(dev, PTR_ERR(ctx->supplies[i]),
					     "failed to get %s supply\n",
					     titan2_main_supplies[i].name);
	}

	/*
	 * The boot loader leaves the panel on: take the reset lines as they
	 * are, so that probing does not blank the screen. The state of the
	 * panel reset tells whether the panel has been brought up.
	 */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset GPIO\n");

	ctx->touch_reset_gpio = devm_gpiod_get_optional(dev, "touch-reset",
							GPIOD_ASIS);
	if (IS_ERR(ctx->touch_reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->touch_reset_gpio),
				     "failed to get touch reset GPIO\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* lane bit rate of the factory driver (PLL 430 MHz), as left by LK */
	dsi->hs_rate = 860000000;
	/* command mode: no MIPI_DSI_MODE_VIDEO */
	dsi->mode_flags = MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get backlight\n");

	ret = titan2_main_hold_boot_supplies(ctx);
	if (ret)
		return dev_err_probe(dev, ret, "failed to hold the supplies\n");

	/* keep the panel running, or in reset until it is powered up */
	ret = gpiod_direction_output(ctx->reset_gpio,
				     ctx->state == TITAN2_MAIN_BOOT_ON ? 0 : 1);
	if (ret) {
		titan2_main_release_supplies(ctx);
		return dev_err_probe(dev, ret, "failed to set up reset GPIO\n");
	}

	drm_panel_add(&ctx->panel);

	/*
	 * On failure (or deferral) leave the panel as the boot loader left
	 * it: it keeps showing the boot picture.
	 */
	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		titan2_main_release_supplies(ctx);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	return 0;
}

static void titan2_main_remove(struct mipi_dsi_device *dsi)
{
	struct titan2_main_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);

	if (ctx->powered)
		titan2_main_power_off(ctx);
}

static const struct of_device_id titan2_main_of_match[] = {
	{ .compatible = "unihertz,titan2-main-panel" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, titan2_main_of_match);

static struct mipi_dsi_driver titan2_main_driver = {
	.probe = titan2_main_probe,
	.remove = titan2_main_remove,
	.driver = {
		.name = "panel-unihertz-titan2-main",
		.of_match_table = titan2_main_of_match,
	},
};
module_mipi_dsi_driver(titan2_main_driver);

MODULE_AUTHOR("serlk <22211586+Serlk@users.noreply.github.com>");
MODULE_DESCRIPTION("DRM driver for the Unihertz Titan 2 main display panel");
MODULE_LICENSE("GPL");
