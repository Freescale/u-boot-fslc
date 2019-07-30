/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuation settings for the Hitachi Solution Engine 7750
 *
 * Copyright (C) 2007 Nobuhiro Iwamatsu <iwamatsu@nigauri.org>
 */

#ifndef __MS7750SE_H
#define __MS7750SE_H

#define CONFIG_CPU_SH7750	1
/* #define CONFIG_CPU_SH7751	1 */
/* #define CONFIG_CPU_TYPE_R	1 */
#define __LITTLE_ENDIAN__	1

#define CONFIG_DISPLAY_BOARDINFO

/*
 * Command line configuration.
 */
#define CONFIG_CONS_SCIF1	1

#define CONFIG_ENV_OVERWRITE	1

/* SDRAM */
#define CONFIG_SYS_SDRAM_BASE		(0x8C000000)
#define CONFIG_SYS_SDRAM_SIZE		(64 * 1024 * 1024)

#define CONFIG_SYS_PBSIZE		256

#define CONFIG_SYS_MEMTEST_START	(CONFIG_SYS_SDRAM_BASE)
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_TEXT_BASE - 0x100000)

/* NOR Flash */
/* #define CONFIG_SYS_FLASH_BASE		(0xA1000000)*/
#define CONFIG_SYS_FLASH_BASE		(0xA0000000)
#define CONFIG_SYS_MAX_FLASH_BANKS	(1)	/* Max number of
					 * Flash memory banks
					 */
#define CONFIG_SYS_MAX_FLASH_SECT	142
#define CONFIG_SYS_FLASH_BANKS_LIST	{ CONFIG_SYS_FLASH_BASE }

#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + 4 * 1024 * 1024)
#define CONFIG_SYS_MONITOR_BASE	(CONFIG_SYS_FLASH_BASE)	/* Address of u-boot image in Flash */
#define CONFIG_SYS_MONITOR_LEN		(128 * 1024)
#define CONFIG_SYS_MALLOC_LEN		(256 * 1024)		/* Size of DRAM reserved for malloc() use */

#define CONFIG_SYS_BOOTMAPSZ		(8 * 1024 * 1024)
#define CONFIG_SYS_RX_ETH_BUFFER	(8)

#define CONFIG_SYS_FLASH_CFI
#define CONFIG_FLASH_CFI_DRIVER
#undef CONFIG_SYS_FLASH_CFI_BROKEN_TABLE
#undef  CONFIG_SYS_FLASH_QUIET_TEST
#define CONFIG_SYS_FLASH_EMPTY_INFO				/* print 'E' for empty sector on flinfo */

#define CONFIG_ENV_SECT_SIZE	0x20000
#define CONFIG_ENV_SIZE		(CONFIG_ENV_SECT_SIZE)
#define CONFIG_ENV_ADDR		(CONFIG_SYS_MONITOR_BASE + CONFIG_SYS_MONITOR_LEN)
#define CONFIG_SYS_FLASH_ERASE_TOUT	120000
#define CONFIG_SYS_FLASH_WRITE_TOUT	500

/* Board Clock */
#define CONFIG_SYS_CLK_FREQ	33333333
#define CONFIG_SH_TMU_CLK_FREQ CONFIG_SYS_CLK_FREQ
#define CONFIG_SH_SCIF_CLK_FREQ CONFIG_SYS_CLK_FREQ
#define CONFIG_SYS_TMU_CLK_DIV		4

#endif /* __MS7750SE_H */
