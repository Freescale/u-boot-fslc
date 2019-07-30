/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuation settings for the Hitachi Solution Engine 7722
 *
 * Copyright (C) 2007 Nobuhiro Iwamatsu <iwamatsu@nigauri.org>
 */

#ifndef __MS7722SE_H
#define __MS7722SE_H

#define CONFIG_CPU_SH7722	1

#define CONFIG_DISPLAY_BOARDINFO
#undef  CONFIG_SHOW_BOOT_PROGRESS

/* SMC9111 */
#define CONFIG_SMC91111
#define CONFIG_SMC91111_BASE    (0xB8000000)

/* MEMORY */
#define MS7722SE_SDRAM_BASE	(0x8C000000)
#define MS7722SE_FLASH_BASE_1	(0xA0000000)
#define MS7722SE_FLASH_BANK_SIZE	(8*1024 * 1024)

#define CONFIG_SYS_PBSIZE		256		/* Buffer size for Console output */
#define CONFIG_SYS_BAUDRATE_TABLE	{ 115200 }	/* List of legal baudrate settings for this board */

/* SCIF */
#define CONFIG_CONS_SCIF0	1

#define CONFIG_SYS_MEMTEST_START	(MS7722SE_SDRAM_BASE)
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + (60 * 1024 * 1024))

#undef  CONFIG_SYS_MEMTEST_SCRATCH	/* Scratch address used by the alternate memory test */

#undef  CONFIG_SYS_LOADS_BAUD_CHANGE	/* Enable temporary baudrate change while serial download */

#define CONFIG_SYS_SDRAM_BASE	(MS7722SE_SDRAM_BASE)
#define CONFIG_SYS_SDRAM_SIZE	(64 * 1024 * 1024)	/* maybe more, but if so u-boot doesn't know about it... */

#define CONFIG_SYS_LOAD_ADDR	(CONFIG_SYS_SDRAM_BASE + 4 * 1024 * 1024)	/* default load address for scripts ?!? */

#define CONFIG_SYS_MONITOR_BASE	(MS7722SE_FLASH_BASE_1)	/* Address of u-boot image
							in Flash (NOT run time address in SDRAM) ?!? */
#define CONFIG_SYS_MONITOR_LEN	(128 * 1024)		/* */
#define CONFIG_SYS_MALLOC_LEN	(256 * 1024)		/* Size of DRAM reserved for malloc() use */
#define CONFIG_SYS_BOOTMAPSZ	(8 * 1024 * 1024)

/* FLASH */
#define CONFIG_SYS_FLASH_CFI
#define CONFIG_FLASH_CFI_DRIVER
#undef  CONFIG_SYS_FLASH_QUIET_TEST
#define CONFIG_SYS_FLASH_EMPTY_INFO			/* print 'E' for empty sector on flinfo */

#define CONFIG_SYS_FLASH_BASE		(MS7722SE_FLASH_BASE_1)	/* Physical start address of Flash memory */

#define CONFIG_SYS_MAX_FLASH_SECT	150		/* Max number of sectors on each
							Flash chip */

/* if you use all NOR Flash , you change dip-switch. Please see MS7722SE01 Manual. */
#define CONFIG_SYS_MAX_FLASH_BANKS	2
#define CONFIG_SYS_FLASH_BANKS_LIST	{ CONFIG_SYS_FLASH_BASE + (0 * MS7722SE_FLASH_BANK_SIZE), \
				  CONFIG_SYS_FLASH_BASE + (1 * MS7722SE_FLASH_BANK_SIZE), \
				}

#define CONFIG_SYS_FLASH_ERASE_TOUT	(3 * 1000)	/* Timeout for Flash erase operations (in ms) */
#define CONFIG_SYS_FLASH_WRITE_TOUT	(3 * 1000)	/* Timeout for Flash write operations (in ms) */
#define CONFIG_SYS_FLASH_LOCK_TOUT	(3 * 1000)	/* Timeout for Flash set sector lock bit operations (in ms) */
#define CONFIG_SYS_FLASH_UNLOCK_TOUT	(3 * 1000)	/* Timeout for Flash clear lock bit operations (in ms) */

#undef  CONFIG_SYS_FLASH_PROTECTION			/* Use hardware flash sectors protection instead of U-Boot software protection */

#undef  CONFIG_SYS_DIRECT_FLASH_TFTP

#define CONFIG_ENV_OVERWRITE	1
#define CONFIG_ENV_SECT_SIZE	(8 * 1024)
#define CONFIG_ENV_SIZE		(CONFIG_ENV_SECT_SIZE)
#define CONFIG_ENV_ADDR		(CONFIG_SYS_FLASH_BASE + (1 * CONFIG_ENV_SECT_SIZE))
#define CONFIG_ENV_OFFSET		(CONFIG_ENV_ADDR - CONFIG_SYS_FLASH_BASE)	/* Offset of env Flash sector relative to CONFIG_SYS_FLASH_BASE */
#define CONFIG_ENV_SIZE_REDUND	(CONFIG_ENV_SECT_SIZE)
#define CONFIG_ENV_ADDR_REDUND	(CONFIG_SYS_FLASH_BASE + (2 * CONFIG_ENV_SECT_SIZE))

/* Board Clock */
#define CONFIG_SYS_CLK_FREQ	33333333
#define CONFIG_SH_TMU_CLK_FREQ CONFIG_SYS_CLK_FREQ
#define CONFIG_SH_SCIF_CLK_FREQ CONFIG_SYS_CLK_FREQ
#define CONFIG_SYS_TMU_CLK_DIV		(4)	/* 4 (default), 16, 64, 256 or 1024 */

#endif	/* __MS7722SE_H */
