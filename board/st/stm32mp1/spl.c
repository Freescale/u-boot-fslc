// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Copyright (C) 2018, STMicroelectronics - All Rights Reserved
 */

#include <config.h>
#include <common.h>
#include <spl.h>
#include <dm.h>
#include <ram.h>
#include <asm/io.h>
#include <post.h>
#include <power/pmic.h>
#include <power/stpmu1.h>
#include <asm/arch/ddr.h>

void spl_board_init(void)
{
	/* Keep vdd on during the reset cycle */
#if defined(CONFIG_PMIC_STPMU1) && defined(CONFIG_SPL_POWER_SUPPORT)
	struct udevice *dev;
	int ret;

	ret = uclass_get_device_by_driver(UCLASS_PMIC,
					  DM_GET_DRIVER(pmic_stpmu1), &dev);
	if (!ret)
		pmic_clrsetbits(dev,
				STPMU1_MASK_RESET_BUCK,
				STPMU1_MASK_RESET_BUCK3,
				STPMU1_MASK_RESET_BUCK3);
#endif
}
