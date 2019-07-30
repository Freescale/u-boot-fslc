// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 Socionext Inc.
 *   Author: Masahiro Yamada <yamada.masahiro@socionext.com>
 */

#include <common.h>
#include <clk.h>
#include <fdtdec.h>
#include <mmc.h>
#include <dm.h>
#include <linux/compat.h>
#include <linux/dma-direction.h>
#include <linux/io.h>
#include <linux/sizes.h>
#include <power/regulator.h>
#include <asm/unaligned.h>

#include "tmio-common.h"

static const struct dm_mmc_ops uniphier_sd_ops = {
	.send_cmd = tmio_sd_send_cmd,
	.set_ios = tmio_sd_set_ios,
	.get_cd = tmio_sd_get_cd,
};

static const struct udevice_id uniphier_sd_match[] = {
	{ .compatible = "socionext,uniphier-sdhc", .data = 0 },
	{ /* sentinel */ }
};

static int uniphier_sd_probe(struct udevice *dev)
{
	struct tmio_sd_priv *priv = dev_get_priv(dev);
#ifndef CONFIG_SPL_BUILD
	struct clk clk;
	int ret;

	ret = clk_get_by_index(dev, 0, &clk);
	if (ret < 0) {
		dev_err(dev, "failed to get host clock\n");
		return ret;
	}

	/* set to max rate */
	priv->mclk = clk_set_rate(&clk, ULONG_MAX);
	if (IS_ERR_VALUE(priv->mclk)) {
		dev_err(dev, "failed to set rate for host clock\n");
		clk_free(&clk);
		return priv->mclk;
	}

	ret = clk_enable(&clk);
	clk_free(&clk);
	if (ret) {
		dev_err(dev, "failed to enable host clock\n");
		return ret;
	}
#else
	priv->mclk = 100000000;
#endif

	return tmio_sd_probe(dev, 0);
}

U_BOOT_DRIVER(uniphier_mmc) = {
	.name = "uniphier-mmc",
	.id = UCLASS_MMC,
	.of_match = uniphier_sd_match,
	.bind = tmio_sd_bind,
	.probe = uniphier_sd_probe,
	.priv_auto_alloc_size = sizeof(struct tmio_sd_priv),
	.platdata_auto_alloc_size = sizeof(struct tmio_sd_plat),
	.ops = &uniphier_sd_ops,
};
