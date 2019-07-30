/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Aeronix Zipit Z2 configuration file
 *
 * Copyright (C) 2009-2010 Marek Vasut <marek.vasut@gmail.com>
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * High Level Board Configuration Options
 */
#define	CONFIG_CPU_PXA27X		1	/* Marvell PXA270 CPU */

#undef	CONFIG_SKIP_LOWLEVEL_INIT
#define	CONFIG_PREBOOT

/*
 * Environment settings
 */
#define	CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_ADDR			0x40000
#define CONFIG_ENV_SIZE			0x10000

#define	CONFIG_SYS_MALLOC_LEN		(128*1024)
#define	CONFIG_ARCH_CPU_INIT

#define	CONFIG_BOOTCOMMAND						\
	"if mmc rescan && ext2load mmc 0 0xa0000000 boot/uboot.script ;"\
	"then "								\
		"source 0xa0000000; "					\
	"else "								\
		"bootm 0x50000; "					\
	"fi; "
#define	CONFIG_TIMESTAMP
#define	CONFIG_CMDLINE_TAG
#define	CONFIG_SETUP_MEMORY_TAGS

/*
 * Serial Console Configuration
 * STUART - the lower serial port on Colibri board
 */
#define	CONFIG_STUART			1

/*
 * Bootloader Components Configuration
 */

/*
 * MMC Card Configuration
 */
#ifdef	CONFIG_CMD_MMC
#define	CONFIG_PXA_MMC_GENERIC
#define	CONFIG_SYS_MMC_BASE		0xF0000000
#endif

/*
 * SPI and LCD
 */
#ifdef	CONFIG_CMD_SPI
#define	CONFIG_SOFT_SPI
#define	CONFIG_LCD_ROTATION
#define	CONFIG_PXA_LCD
#define	CONFIG_LMS283GF05

#define	SPI_DELAY	udelay(10)
#define	SPI_SDA(val)	zipitz2_spi_sda(val)
#define	SPI_SCL(val)	zipitz2_spi_scl(val)
#define	SPI_READ	zipitz2_spi_read()
#ifndef	__ASSEMBLY__
void zipitz2_spi_sda(int);
void zipitz2_spi_scl(int);
unsigned char zipitz2_spi_read(void);
#endif
#endif

#define	CONFIG_SYS_DEVICE_NULLDEV	1

/*
 * Clock Configuration
 */
#define CONFIG_SYS_CPUSPEED		0x190		/* standard setting for 312MHz; L=16, N=1.5, A=0, SDCLK!=SystemBus */

/*
 * SRAM Map
 */
#define	PHYS_SRAM			0x5c000000	/* SRAM Bank #1 */
#define	PHYS_SRAM_SIZE			0x00040000	/* 256k */

/*
 * DRAM Map
 */
#define	CONFIG_NR_DRAM_BANKS		1		/* We have 1 bank of DRAM */
#define	PHYS_SDRAM_1			0xa0000000	/* SDRAM Bank #1 */
#define	PHYS_SDRAM_1_SIZE		0x02000000	/* 32 MB */

#define	CONFIG_SYS_DRAM_BASE		0xa0000000	/* CS0 */
#define	CONFIG_SYS_DRAM_SIZE		0x02000000	/* 32 MB DRAM */

#define CONFIG_SYS_MEMTEST_START	0xa0400000	/* memtest works on */
#define CONFIG_SYS_MEMTEST_END		0xa0800000	/* 4 ... 8 MB in DRAM */

#define	CONFIG_SYS_LOAD_ADDR		CONFIG_SYS_DRAM_BASE

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1
#define	CONFIG_SYS_INIT_SP_ADDR		(GENERATED_GBL_DATA_SIZE + PHYS_SRAM + 2048)

/*
 * NOR FLASH
 */
#define PHYS_FLASH_1			0x00000000	/* Flash Bank #1 */
#define PHYS_FLASH_SIZE			0x00800000	/* 8 MB */
#define PHYS_FLASH_SECT_SIZE		0x00010000	/* 64 KB sectors */
#define CONFIG_SYS_FLASH_BASE		PHYS_FLASH_1

#define CONFIG_SYS_FLASH_CFI
#define CONFIG_FLASH_CFI_DRIVER		1
#define CONFIG_SYS_FLASH_CFI_WIDTH      FLASH_CFI_16BIT

#define CONFIG_SYS_MONITOR_BASE		PHYS_FLASH_1
#define CONFIG_SYS_MONITOR_LEN		PHYS_FLASH_SECT_SIZE

#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define CONFIG_SYS_MAX_FLASH_SECT	256

#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE	1

#define CONFIG_SYS_FLASH_ERASE_TOUT	240000
#define CONFIG_SYS_FLASH_WRITE_TOUT	240000
#define CONFIG_SYS_FLASH_LOCK_TOUT	240000
#define CONFIG_SYS_FLASH_UNLOCK_TOUT	240000
#define CONFIG_SYS_FLASH_PROTECTION

/*
 * GPIO settings
 */
#define CONFIG_SYS_GAFR0_L_VAL	0x02000140
#define CONFIG_SYS_GAFR0_U_VAL	0x59188000
#define CONFIG_SYS_GAFR1_L_VAL	0x63900002
#define CONFIG_SYS_GAFR1_U_VAL	0xaaa03950
#define CONFIG_SYS_GAFR2_L_VAL	0x0aaaaaaa
#define CONFIG_SYS_GAFR2_U_VAL	0x29000308
#define CONFIG_SYS_GAFR3_L_VAL	0x54000000
#define CONFIG_SYS_GAFR3_U_VAL	0x000000d5
#define CONFIG_SYS_GPCR0_VAL	0x00000000
#define CONFIG_SYS_GPCR1_VAL	0x00000020
#define CONFIG_SYS_GPCR2_VAL	0x00000000
#define CONFIG_SYS_GPCR3_VAL	0x00000000
#define CONFIG_SYS_GPDR0_VAL	0xdafcee00
#define CONFIG_SYS_GPDR1_VAL	0xffa3aaab
#define CONFIG_SYS_GPDR2_VAL	0x8fe9ffff
#define CONFIG_SYS_GPDR3_VAL	0x001b1f8a
#define CONFIG_SYS_GPSR0_VAL	0x06080400
#define CONFIG_SYS_GPSR1_VAL	0x007f0000
#define CONFIG_SYS_GPSR2_VAL	0x032a0000
#define CONFIG_SYS_GPSR3_VAL	0x00000180

#define CONFIG_SYS_PSSR_VAL	0x30

/*
 * Clock settings
 */
#define CONFIG_SYS_CKEN		0x00511220
#define CONFIG_SYS_CCCR		0x00000190

/*
 * Memory settings
 */
#define CONFIG_SYS_MSC0_VAL	0x2ffc38f8
#define CONFIG_SYS_MSC1_VAL	0x0000ccd1
#define CONFIG_SYS_MSC2_VAL	0x0000b884
#define CONFIG_SYS_MDCNFG_VAL	0x08000ba9
#define CONFIG_SYS_MDREFR_VAL	0x2011a01e
#define CONFIG_SYS_MDMRS_VAL	0x00000000
#define CONFIG_SYS_FLYCNFG_VAL	0x00010001
#define CONFIG_SYS_SXCNFG_VAL	0x40044004

/*
 * PCMCIA and CF Interfaces
 */
#define CONFIG_SYS_MECR_VAL	0x00000001
#define CONFIG_SYS_MCMEM0_VAL	0x00014307
#define CONFIG_SYS_MCMEM1_VAL	0x00014307
#define CONFIG_SYS_MCATT0_VAL	0x0001c787
#define CONFIG_SYS_MCATT1_VAL	0x0001c787
#define CONFIG_SYS_MCIO0_VAL	0x0001430f
#define CONFIG_SYS_MCIO1_VAL	0x0001430f

#include "pxa-common.h"

#endif	/* __CONFIG_H */
