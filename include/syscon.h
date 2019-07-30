/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2015 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 */

#ifndef __SYSCON_H
#define __SYSCON_H

#include <fdtdec.h>

/**
 * struct syscon_uc_info - Information stored by the syscon UCLASS_UCLASS
 *
 * @regmap:	Register map for this controller
 */
struct syscon_uc_info {
	struct regmap *regmap;
};

/* So far there are no ops so this is a placeholder */
struct syscon_ops {
};

#define syscon_get_ops(dev)        ((struct syscon_ops *)(dev)->driver->ops)

#if CONFIG_IS_ENABLED(OF_PLATDATA)
/*
 * We don't support 64-bit machines. If they are so resource-contrained that
 * they need to use OF_PLATDATA, something is horribly wrong with the
 * education of our hardware engineers.
 *
 * Update: 64-bit is now supported and we have an education crisis.
 */
struct syscon_base_platdata {
	fdt_val_t reg[2];
};
#endif

/**
 * syscon_get_regmap() - Get access to a register map
 *
 * @dev:	Device to check (UCLASS_SCON)
 * @info:	Returns regmap for the device
 * @return 0 if OK, -ve on error
 */
struct regmap *syscon_get_regmap(struct udevice *dev);

/**
 * syscon_get_regmap_by_driver_data() - Look up a controller by its ID
 *
 * Each system controller can be accessed by its driver data, which is
 * assumed to be unique through the scope of all system controllers that
 * are in use. This function looks up the controller given this driver data.
 *
 * @driver_data:	Driver data value to look up
 * @devp:		Returns the controller correponding to @driver_data
 * @return 0 on success, -ENODEV if the ID was not found, or other -ve error
 *	   code
 */
int syscon_get_by_driver_data(ulong driver_data, struct udevice **devp);

/**
 * syscon_get_regmap_by_driver_data() - Look up a controller by its ID
 *
 * Each system controller can be accessed by its driver data, which is
 * assumed to be unique through the scope of all system controllers that
 * are in use. This function looks up the regmap given this driver data.
 *
 * @driver_data:	Driver data value to look up
 * @return register map correponding to @driver_data, or -ve error code
 */
struct regmap *syscon_get_regmap_by_driver_data(ulong driver_data);

/**
 * syscon_get_first_range() - get the first memory range from a syscon regmap
 *
 * @driver_data:	Driver data value to look up
 * @return first region of register map correponding to @driver_data, or
 *			-ve error code
 */
void *syscon_get_first_range(ulong driver_data);

#endif
