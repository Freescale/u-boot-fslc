/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2008-2010
 * Gražvydas Ignotas <notasas@gmail.com>
 *
 * Configuration settings for the OMAP3 Pandora.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_NR_DRAM_BANKS	2	/* CS1 may or may not be populated */

/* override base for compatibility with MLO the device ships with */

#include <configs/ti_omap3_common.h>

#define CONFIG_MISC_INIT_R
#define CONFIG_REVISION_TAG		1

#define CONFIG_ENV_SIZE			(128 << 10)	/* 128 KiB */

#define CONFIG_SYS_DEVICE_NULLDEV	1

/*
 * Hardware drivers
 */

/* TWL4030 LED */
#define CONFIG_TWL4030_LED

/*
 * NS16550 Configuration
 */
#undef CONFIG_SYS_NS16550_CLK
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_CLK		V_NS16550_CLK
#define CONFIG_SYS_NS16550_COM3		OMAP34XX_UART3
#define CONFIG_SERIAL3			3

/* commands to include */

/*
 * Board NAND Info.
 */
#define CONFIG_SYS_NAND_ADDR		NAND_BASE	/* physical address */
							/* to access nand */
#define CONFIG_NAND_OMAP_ECCSCHEME	OMAP_ECC_HAM1_CODE_SW
#define CONFIG_SYS_NAND_PAGE_SIZE	2048
#define CONFIG_SYS_NAND_OOBSIZE		64

#ifdef CONFIG_NAND
#define CONFIG_MTD_PARTITIONS	/* required for UBI partition support */
#endif


#define CONFIG_BOOTCOMMAND \
	"run distro_bootcmd; " \
	"setenv bootargs ${bootargs_ubi}; " \
	"if mmc rescan && load mmc 0:1 ${loadaddr} autoboot.scr; then " \
		"source ${loadaddr}; " \
	"fi; " \
	"ubi part boot && ubifsmount ubi:boot && " \
		"ubifsload ${loadaddr} uImage && bootm ${loadaddr}"

#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0) \

#include <config_distro_bootcmd.h>

#define CONFIG_EXTRA_ENV_SETTINGS \
	DEFAULT_LINUX_BOOT_ENV \
	"usbtty=cdc_acm\0" \
	"bootargs_ubi=ubi.mtd=4 ubi.mtd=3 root=ubi0:rootfs rootfstype=ubifs " \
		"rw rootflags=bulk_read vram=6272K omapfb.vram=0:3000K\0" \
	"mtdparts=" CONFIG_MTDPARTS_DEFAULT "\0" \
	BOOTENV \

/* memtest works on */
#define CONFIG_SYS_MEMTEST_START	(OMAP34XX_SDRC_CS0)
#define CONFIG_SYS_MEMTEST_END		(OMAP34XX_SDRC_CS0 + \
					0x01F00000) /* 31MB */

#if defined(CONFIG_NAND)
#define CONFIG_SYS_FLASH_BASE		NAND_BASE
#endif

/* Monitor at start of flash */
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_FLASH_BASE


#define CONFIG_SYS_ENV_SECT_SIZE	(128 << 10)	/* 128 KiB */
#define CONFIG_ENV_OFFSET		0x260000
#define CONFIG_ENV_ADDR			0x260000

#endif				/* __CONFIG_H */
