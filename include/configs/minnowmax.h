/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2015 Google, Inc
 */

/*
 * board/config.h - configuration options, board specific
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <configs/x86-common.h>

#define CONFIG_SYS_MONITOR_LEN		(1 << 20)

#define CONFIG_MISC_INIT_R

#define CONFIG_STD_DEVICES_SETTINGS	"stdin=usbkbd,serial\0" \
					"stdout=vidconsole,serial\0" \
					"stderr=vidconsole,serial\0" \
					"usb_pgood_delay=40\0"

#define VIDEO_IO_OFFSET				0
#define CONFIG_X86EMU_RAW_IO

#define CONFIG_ENV_SECT_SIZE		0x1000
#define CONFIG_ENV_OFFSET		0x006ef000

#endif	/* __CONFIG_H */
