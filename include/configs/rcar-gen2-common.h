/* SPDX-License-Identifier: GPL-2.0 */
/*
 * include/configs/rcar-gen2-common.h
 *
 * Copyright (C) 2013,2014 Renesas Electronics Corporation
 */

#ifndef __RCAR_GEN2_COMMON_H
#define __RCAR_GEN2_COMMON_H

#include <asm/arch/rmobile.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG

#ifdef CONFIG_SPL
#define CONFIG_SPL_TARGET	"spl/u-boot-spl.srec"
#endif

#undef	CONFIG_SHOW_BOOT_PROGRESS

#define CONFIG_ARCH_CPU_INIT

#define CONFIG_TMU_TIMER
#ifndef CONFIG_PINCTRL_PFC
#define CONFIG_SH_GPIO_PFC
#endif

/* console */
#define CONFIG_SYS_PBSIZE		256
#define CONFIG_SYS_BAUDRATE_TABLE	{ 38400, 115200 }

#define CONFIG_SYS_SDRAM_BASE		(RCAR_GEN2_SDRAM_BASE)
#define CONFIG_SYS_SDRAM_SIZE		(RCAR_GEN2_UBOOT_SDRAM_SIZE)
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + 0x7fc0)
#define CONFIG_NR_DRAM_BANKS		1

#define CONFIG_SYS_MONITOR_BASE		0x00000000
#define CONFIG_SYS_MONITOR_LEN		(256 * 1024)
#define CONFIG_SYS_MALLOC_LEN		(1 * 1024 * 1024)
#define CONFIG_SYS_BOOTMAPSZ		(8 * 1024 * 1024)

/* ENV setting */
#define CONFIG_ENV_ADDR	0xC0000

/* Common ENV setting */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_SECT_SIZE	(256 * 1024)
#define CONFIG_ENV_OFFSET	(CONFIG_ENV_ADDR)
#define CONFIG_ENV_SIZE		(CONFIG_ENV_SECT_SIZE)
#define CONFIG_ENV_SIZE_REDUND	(CONFIG_SYS_MONITOR_LEN)

#endif	/* __RCAR_GEN2_COMMON_H */
