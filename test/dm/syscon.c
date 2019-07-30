// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2015 Google, Inc
 */

#include <common.h>
#include <dm.h>
#include <syscon.h>
#include <asm/test.h>
#include <dm/test.h>
#include <test/ut.h>

/* Base test of system controllers */
static int dm_test_syscon_base(struct unit_test_state *uts)
{
	struct udevice *dev;

	ut_assertok(uclass_get_device(UCLASS_SYSCON, 0, &dev));
	ut_asserteq(SYSCON0, dev->driver_data);

	ut_assertok(uclass_get_device(UCLASS_SYSCON, 1, &dev));
	ut_asserteq(SYSCON1, dev->driver_data);

	ut_asserteq(-ENODEV, uclass_get_device(UCLASS_SYSCON, 2, &dev));

	return 0;
}
DM_TEST(dm_test_syscon_base, DM_TESTF_SCAN_PDATA | DM_TESTF_SCAN_FDT);

/* Test system controller finding */
static int dm_test_syscon_by_driver_data(struct unit_test_state *uts)
{
	struct udevice *dev;

	ut_assertok(syscon_get_by_driver_data(SYSCON0, &dev));
	ut_asserteq(SYSCON0, dev->driver_data);

	ut_assertok(syscon_get_by_driver_data(SYSCON1, &dev));
	ut_asserteq(SYSCON1, dev->driver_data);

	ut_asserteq(-ENODEV, syscon_get_by_driver_data(2, &dev));

	return 0;
}
DM_TEST(dm_test_syscon_by_driver_data, DM_TESTF_SCAN_PDATA | DM_TESTF_SCAN_FDT);
