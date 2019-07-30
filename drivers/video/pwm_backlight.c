// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <dm.h>
#include <backlight.h>
#include <pwm.h>
#include <asm/gpio.h>
#include <power/regulator.h>

struct pwm_backlight_priv {
	struct udevice *reg;
	struct gpio_desc enable;
	struct udevice *pwm;
	uint channel;
	uint period_ns;
	uint default_level;
	uint min_level;
	uint max_level;
};

static int pwm_backlight_enable(struct udevice *dev)
{
	struct pwm_backlight_priv *priv = dev_get_priv(dev);
	struct dm_regulator_uclass_platdata *plat;
	uint duty_cycle;
	int ret;

	if (priv->reg) {
		plat = dev_get_uclass_platdata(priv->reg);
		debug("%s: Enable '%s', regulator '%s'/'%s'\n", __func__,
		      dev->name, priv->reg->name, plat->name);
		ret = regulator_set_enable(priv->reg, true);
		if (ret) {
			debug("%s: Cannot enable regulator for PWM '%s'\n",
			      __func__, dev->name);
			return ret;
		}
		mdelay(120);
	}

	duty_cycle = priv->period_ns * (priv->default_level - priv->min_level) /
		(priv->max_level - priv->min_level + 1);
	ret = pwm_set_config(priv->pwm, priv->channel, priv->period_ns,
			     duty_cycle);
	if (ret)
		return ret;
	ret = pwm_set_enable(priv->pwm, priv->channel, true);
	if (ret)
		return ret;
	mdelay(10);
	dm_gpio_set_value(&priv->enable, 1);

	return 0;
}

static int pwm_backlight_ofdata_to_platdata(struct udevice *dev)
{
	struct pwm_backlight_priv *priv = dev_get_priv(dev);
	struct ofnode_phandle_args args;
	int index, ret, count, len;
	const u32 *cell;

	debug("%s: start\n", __func__);
	ret = uclass_get_device_by_phandle(UCLASS_REGULATOR, dev,
					   "power-supply", &priv->reg);
	if (ret)
		debug("%s: Cannot get power supply: ret=%d\n", __func__, ret);
	ret = gpio_request_by_name(dev, "enable-gpios", 0, &priv->enable,
				   GPIOD_IS_OUT);
	if (ret) {
		debug("%s: Warning: cannot get enable GPIO: ret=%d\n",
		      __func__, ret);
		if (ret != -ENOENT)
			return ret;
	}
	ret = dev_read_phandle_with_args(dev, "pwms", "#pwm-cells", 0, 0,
					 &args);
	if (ret) {
		debug("%s: Cannot get PWM phandle: ret=%d\n", __func__, ret);
		return ret;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_PWM, args.node, &priv->pwm);
	if (ret) {
		debug("%s: Cannot get PWM: ret=%d\n", __func__, ret);
		return ret;
	}
	priv->channel = args.args[0];
	priv->period_ns = args.args[1];

	index = dev_read_u32_default(dev, "default-brightness-level", 255);
	cell = dev_read_prop(dev, "brightness-levels", &len);
	count = len / sizeof(u32);
	if (cell && count > index) {
		priv->default_level = fdt32_to_cpu(cell[index]);
		priv->max_level = fdt32_to_cpu(cell[count - 1]);
	} else {
		priv->default_level = index;
		priv->max_level = 255;
	}
	debug("%s: done\n", __func__);


	return 0;
}

static int pwm_backlight_probe(struct udevice *dev)
{
	return 0;
}

static const struct backlight_ops pwm_backlight_ops = {
	.enable	= pwm_backlight_enable,
};

static const struct udevice_id pwm_backlight_ids[] = {
	{ .compatible = "pwm-backlight" },
	{ }
};

U_BOOT_DRIVER(pwm_backlight) = {
	.name	= "pwm_backlight",
	.id	= UCLASS_PANEL_BACKLIGHT,
	.of_match = pwm_backlight_ids,
	.ops	= &pwm_backlight_ops,
	.ofdata_to_platdata	= pwm_backlight_ofdata_to_platdata,
	.probe		= pwm_backlight_probe,
	.priv_auto_alloc_size	= sizeof(struct pwm_backlight_priv),
};
