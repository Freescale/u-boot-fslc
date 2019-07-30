/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2013 CompuLab, Ltd.
 * Author: Igor Grinberg <grinberg@compulab.co.il>
 *
 * Configuration settings for the CompuLab CM-T3517 board
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * High Level Configuration Options
 */
#define CONFIG_CM_T3517	/* working with CM-T3517 */

/*
 * This is needed for the DMA stuff.
 * Although the default iss 64, we still define it
 * to be on the safe side once the default is changed.
 */

#include <asm/arch/cpu.h>		/* get chip and board defs */
#include <asm/arch/omap.h>

#define CONFIG_MACH_TYPE                MACH_TYPE_CM_T3517

/* Clock Defines */
#define V_OSCK			26000000	/* Clock output from T2 */
#define V_SCLK			(V_OSCK >> 1)

#define CONFIG_MISC_INIT_R

/*
 * The early kernel mapping on ARM currently only maps from the base of DRAM
 * to the end of the kernel image.  The kernel is loaded at DRAM base + 0x8000.
 * The early kernel pagetable uses DRAM base + 0x4000 to DRAM base + 0x8000,
 * so that leaves DRAM base to DRAM base + 0x4000 available.
 */
#define CONFIG_SYS_BOOTMAPSZ	        0x4000

#define CONFIG_CMDLINE_TAG		/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG
#define CONFIG_SERIAL_TAG

/*
 * Size of malloc() pool
 */
#define CONFIG_ENV_SIZE		(128 << 10)	/* 128 KiB */
#define CONFIG_SYS_MALLOC_LEN	(CONFIG_ENV_SIZE + (128 << 10))

/*
 * Hardware drivers
 */

/*
 * NS16550 Configuration
 */
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_CLK		48000000	/* 48MHz (APLL96/2) */

/*
 * select serial console configuration
 */
#define CONFIG_SYS_NS16550_COM3		OMAP34XX_UART3
#define CONFIG_SERIAL3			3	/* UART3 */

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_SYS_BAUDRATE_TABLE	{4800, 9600, 19200, 38400, 57600,\
					115200}

/* USB */

#ifndef CONFIG_USB_MUSB_AM35X
#define CONFIG_OMAP_EHCI_PHY1_RESET_GPIO 146
#define CONFIG_OMAP_EHCI_PHY2_RESET_GPIO 147
#endif /* CONFIG_USB_MUSB_AM35X */

/* commands to include */
#define CONFIG_MTD_DEVICE	/* needed for mtdparts commands */
#define CONFIG_MTD_PARTITIONS

#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_EEPROM_ADDR	0x50
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	1
#define CONFIG_SYS_I2C_EEPROM_BUS	0
#define CONFIG_I2C_MULTI_BUS

/*
 * Board NAND Info.
 */
#define CONFIG_SYS_NAND_ADDR		NAND_BASE	/* physical address */
							/* to access nand */
#define CONFIG_SYS_NAND_BASE		NAND_BASE	/* physical address */
							/* to access nand at */
							/* CS0 */
#define CONFIG_SYS_MAX_NAND_DEVICE	1		/* Max number of NAND */
							/* devices */

/* Environment information */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"loadaddr=0x82000000\0" \
	"baudrate=115200\0" \
	"console=ttyO2,115200n8\0" \
	"netretry=yes\0" \
	"mpurate=auto\0" \
	"vram=12M\0" \
	"dvimode=1024x768MR-16@60\0" \
	"defaultdisplay=dvi\0" \
	"mmcdev=0\0" \
	"mmcroot=/dev/mmcblk0p2 rw rootwait\0" \
	"mmcrootfstype=ext4\0" \
	"nandroot=/dev/mtdblock4 rw\0" \
	"nandrootfstype=ubifs\0" \
	"mmcargs=setenv bootargs console=${console} " \
		"mpurate=${mpurate} " \
		"vram=${vram} " \
		"omapfb.mode=dvi:${dvimode} " \
		"omapdss.def_disp=${defaultdisplay} " \
		"root=${mmcroot} " \
		"rootfstype=${mmcrootfstype}\0" \
	"nandargs=setenv bootargs console=${console} " \
		"mpurate=${mpurate} " \
		"vram=${vram} " \
		"omapfb.mode=dvi:${dvimode} " \
		"omapdss.def_disp=${defaultdisplay} " \
		"root=${nandroot} " \
		"rootfstype=${nandrootfstype}\0" \
	"loadbootscript=fatload mmc ${mmcdev} ${loadaddr} boot.scr\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source ${loadaddr}\0" \
	"loaduimage=fatload mmc ${mmcdev} ${loadaddr} uImage\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"bootm ${loadaddr}\0" \
	"nandboot=echo Booting from nand ...; " \
		"run nandargs; " \
		"nand read ${loadaddr} 2a0000 400000; " \
		"bootm ${loadaddr}\0" \

#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev}; if mmc rescan; then " \
		"if run loadbootscript; then " \
			"run bootscript; " \
		"else " \
			"if run loaduimage; then " \
				"run mmcboot; " \
			"else run nandboot; " \
			"fi; " \
		"fi; " \
	"else run nandboot; fi"

/*
 * Miscellaneous configurable options
 */
#define CONFIG_TIMESTAMP
#define CONFIG_SYS_AUTOLOAD		"no"
#define CONFIG_SYS_CBSIZE		512	/* Console I/O Buffer Size */
#define CONFIG_SYS_MAXARGS		32	/* max number of command args */

#define CONFIG_SYS_LOAD_ADDR		(OMAP34XX_SDRC_CS0 + 0x02000000)

/*
 * AM3517 has 12 GP timers, they can be driven by the system clock
 * (12/13/16.8/19.2/38.4MHz) or by 32KHz clock. We use 13MHz (V_SCLK).
 * This rate is divided by a local divisor.
 */
#define CONFIG_SYS_TIMERBASE		(OMAP34XX_GPT2)
#define CONFIG_SYS_PTV			2       /* Divisor: 2^(PTV+1) => 8 */
#define CONFIG_SYS_HZ			1000

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	1	/* CM-T3517 DRAM is only on CS0 */
#define PHYS_SDRAM_1		OMAP34XX_SDRC_CS0
#define CONFIG_SYS_CS0_SIZE		(256 << 20)

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */

/* **** PISMO SUPPORT *** */
/* Monitor at start of flash */
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_FLASH_BASE
#define CONFIG_SYS_MONITOR_LEN		(256 << 10)	/* Reserve 2 sectors */

#define CONFIG_ENV_OFFSET		0x260000
#define CONFIG_ENV_ADDR			0x260000

#if defined(CONFIG_CMD_NET)
#define CONFIG_DRIVER_TI_EMAC
#define CONFIG_DRIVER_TI_EMAC_USE_RMII
#define CONFIG_MII
#define CONFIG_ARP_TIMEOUT		200UL
#define CONFIG_NET_RETRY_COUNT		5
#endif /* CONFIG_CMD_NET */

/* additions for new relocation code, must be added to all boards */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1
#define CONFIG_SYS_INIT_RAM_ADDR	0x4020f800
#define CONFIG_SYS_INIT_RAM_SIZE	0x800
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_INIT_RAM_ADDR +	\
					 CONFIG_SYS_INIT_RAM_SIZE -	\
					 GENERATED_GBL_DATA_SIZE)

/* Status LED */
#define GREEN_LED_GPIO			186 /* CM-T3517 Green LED is GPIO186 */

/* Display Configuration */
#define CONFIG_VIDEO_OMAP3
#define LCD_BPP		LCD_COLOR16

#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASHIMAGE_GUARD
#define CONFIG_BMP_16BPP
#define CONFIG_SCF0403_LCD

/* EEPROM */
#define CONFIG_ENV_EEPROM_IS_ON_I2C
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN		1
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS	4
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	5
#define CONFIG_SYS_EEPROM_SIZE			256

#endif /* __CONFIG_H */
