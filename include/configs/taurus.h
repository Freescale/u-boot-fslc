/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Common board functions for Siemens TAURUS (AT91SAM9G20) based boards
 * (C) Copyright 2013 Siemens AG
 *
 * Based on:
 * U-Boot file: include/configs/at91sam9260ek.h
 *
 * (C) Copyright 2007-2008
 * Stelian Pop <stelian@popies.net>
 * Lead Tech Design <www.leadtechdesign.com>
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * SoC must be defined first, before hardware.h is included.
 * In this case SoC is defined in boards.cfg.
 */
#include <asm/hardware.h>
#include <linux/sizes.h>

#if defined(CONFIG_SPL_BUILD)
#define CONFIG_SYS_ICACHE_OFF
#define CONFIG_SYS_DCACHE_OFF
#endif
/*
 * Warning: changing CONFIG_SYS_TEXT_BASE requires
 * adapting the initial boot program.
 * Since the linker has to swallow that define, we must use a pure
 * hex number here!
 */

/* ARM asynchronous clock */
#define CONFIG_SYS_AT91_SLOW_CLOCK	32768		/* slow clock xtal */
#define CONFIG_SYS_AT91_MAIN_CLOCK	18432000	/* main clock xtal */

/* Misc CPU related */
#define CONFIG_ARCH_CPU_INIT
#define CONFIG_CMDLINE_TAG		/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_SKIP_LOWLEVEL_INIT_ONLY

/* general purpose I/O */
#define CONFIG_ATMEL_LEGACY		/* required until (g)pio is fixed */
#define CONFIG_AT91_GPIO
#define CONFIG_AT91_GPIO_PULLUP	1	/* keep pullups on peripheral pins */

/* serial console */
#define CONFIG_ATMEL_USART
#define CONFIG_USART_BASE		ATMEL_BASE_DBGU
#define CONFIG_USART_ID			ATMEL_ID_SYS


/*
 * SDRAM: 1 bank, min 32, max 128 MB
 * Initialized before u-boot gets started.
 */
#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_SDRAM_BASE		ATMEL_BASE_CS1
#define CONFIG_SYS_SDRAM_SIZE		(128 * SZ_1M)

/*
 * Initial stack pointer: 4k - GENERATED_GBL_DATA_SIZE in internal SRAM,
 * leaving the correct space for initial global data structure above
 * that address while providing maximum stack area below.
 */
#define CONFIG_SYS_INIT_SP_ADDR \
	(ATMEL_BASE_SRAM1 + 0x1000 - GENERATED_GBL_DATA_SIZE)

/* NAND flash */
#ifdef CONFIG_CMD_NAND
#define CONFIG_NAND_ATMEL
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		ATMEL_BASE_CS3
#define CONFIG_SYS_NAND_DBW_8
#define CONFIG_SYS_NAND_MASK_ALE	(1 << 21)
#define CONFIG_SYS_NAND_MASK_CLE	(1 << 22)
#define CONFIG_SYS_NAND_ENABLE_PIN	AT91_PIN_PC14
#define CONFIG_SYS_NAND_READY_PIN	AT91_PIN_PC13
#endif

/* Ethernet */
#define CONFIG_MACB
#define CONFIG_RMII
#define CONFIG_AT91_WANTS_COMMON_PHY

#define CONFIG_AT91SAM9_WATCHDOG
#define CONFIG_AT91_HW_WDT_TIMEOUT	15
#if !defined(CONFIG_SPL_BUILD)
/* Enable the watchdog */
#define CONFIG_HW_WATCHDOG
#endif

/* USB */
#if defined(CONFIG_BOARD_TAURUS)
#define CONFIG_USB_ATMEL
#define CONFIG_USB_ATMEL_CLK_SEL_PLLB
#define CONFIG_USB_OHCI_NEW
#define CONFIG_SYS_USB_OHCI_CPU_INIT
#define CONFIG_SYS_USB_OHCI_REGS_BASE		0x00500000
#define CONFIG_SYS_USB_OHCI_SLOT_NAME		"at91sam9260"
#define CONFIG_SYS_USB_OHCI_MAX_ROOT_PORTS	2

/* USB DFU support */
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS

#define CONFIG_USB_GADGET_AT91

/* DFU class support */
#define CONFIG_SYS_DFU_DATA_BUF_SIZE	(SZ_1M)
#define DFU_MANIFEST_POLL_TIMEOUT	25000
#endif

/* SPI EEPROM */
#define TAURUS_SPI_MASK (1 << 4)
#define TAURUS_SPI_CS_PIN	AT91_PIN_PA3

#if defined(CONFIG_SPL_BUILD)
/* SPL related */
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x20000

#define CONFIG_SF_DEFAULT_BUS 0
#define CONFIG_SF_DEFAULT_SPEED 1000000
#define CONFIG_SF_DEFAULT_MODE SPI_MODE_3
#endif

/* load address */
#define CONFIG_SYS_LOAD_ADDR			0x22000000

/* bootstrap in spi flash , u-boot + env + linux in nandflash */
#define CONFIG_ENV_OFFSET		0x100000
#define CONFIG_ENV_OFFSET_REDUND	0x180000
#define CONFIG_ENV_SIZE		(SZ_128K)	/* 1 sector = 128 kB */
#define CONFIG_BOOTCOMMAND	"nand read 0x22000000 0x200000 0x300000; bootm"

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN \
	ROUND(3 * CONFIG_ENV_SIZE + SZ_4M, 0x1000)

/* Defines for SPL */
#define CONFIG_SPL_TEXT_BASE		0x0
#define CONFIG_SPL_MAX_SIZE		(31 * SZ_512)
#define	CONFIG_SPL_STACK		(ATMEL_BASE_SRAM1 + SZ_16K)
#define CONFIG_SYS_SPL_MALLOC_START     (CONFIG_SYS_TEXT_BASE - \
					CONFIG_SYS_MALLOC_LEN)
#define CONFIG_SYS_SPL_MALLOC_SIZE      CONFIG_SYS_MALLOC_LEN

#define CONFIG_SPL_BSS_START_ADDR	CONFIG_SPL_MAX_SIZE
#define CONFIG_SPL_BSS_MAX_SIZE		(3 * SZ_512)

#define CONFIG_SYS_NAND_ENABLE_PIN_SPL	(2*32 + 14)
#define CONFIG_SYS_USE_NANDFLASH	1
#define CONFIG_SPL_NAND_DRIVERS
#define CONFIG_SPL_NAND_BASE
#define CONFIG_SPL_NAND_ECC
#define CONFIG_SPL_NAND_RAW_ONLY
#define CONFIG_SPL_NAND_SOFTECC
#define CONFIG_SYS_NAND_U_BOOT_OFFS	0x20000
#define CONFIG_SYS_NAND_U_BOOT_SIZE	SZ_512K
#define	CONFIG_SYS_NAND_U_BOOT_START	CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_NAND_U_BOOT_DST	CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_NAND_5_ADDR_CYCLE

#define CONFIG_SYS_NAND_SIZE		(256 * SZ_1M)
#define CONFIG_SYS_NAND_PAGE_SIZE	SZ_2K
#define CONFIG_SYS_NAND_BLOCK_SIZE	(SZ_128K)
#define CONFIG_SYS_NAND_PAGE_COUNT	(CONFIG_SYS_NAND_BLOCK_SIZE / \
					 CONFIG_SYS_NAND_PAGE_SIZE)
#define CONFIG_SYS_NAND_BAD_BLOCK_POS	NAND_LARGE_BADBLOCK_POS
#define CONFIG_SYS_NAND_ECCSIZE		256
#define CONFIG_SYS_NAND_ECCBYTES	3
#define CONFIG_SYS_NAND_OOBSIZE		64
#define CONFIG_SYS_NAND_ECCPOS		{ 40, 41, 42, 43, 44, 45, 46, 47, \
					  48, 49, 50, 51, 52, 53, 54, 55, \
					  56, 57, 58, 59, 60, 61, 62, 63, }

#define CONFIG_SPL_ATMEL_SIZE
#define CONFIG_SYS_MASTER_CLOCK		132096000
#define AT91_PLL_LOCK_TIMEOUT		1000000
#define CONFIG_SYS_AT91_PLLA		0x202A3F01
#define CONFIG_SYS_MCKR			0x1300
#define CONFIG_SYS_MCKR_CSS		(0x02 | CONFIG_SYS_MCKR)
#define CONFIG_SYS_AT91_PLLB		0x10193F05

#endif
