/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for ODROID-C2
 * (C) Copyright 2016 Beniamino Galvani <b.galvani@gmail.com>
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_MISC_INIT_R

/* Serial setup */

#define MESON_FDTFILE_SETTING "fdtfile=amlogic/meson-gxbb-odroidc2.dtb\0"

#include <configs/meson-gxbb-common.h>

#endif /* __CONFIG_H */
