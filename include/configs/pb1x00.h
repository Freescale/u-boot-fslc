/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 */

/*
 * This file contains the configuration parameters for the dbau1x00 board.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_SOC_AU1X00	1  /* alchemy series cpu */

#ifdef CONFIG_PB1000
#define CONFIG_SOC_AU1000	1
#else
#ifdef CONFIG_PB1100
#define CONFIG_SOC_AU1100	1
#else
#ifdef CONFIG_PB1500
#define CONFIG_SOC_AU1500	1
#else
#error "No valid board set"
#endif
#endif
#endif

#define	CONFIG_TIMESTAMP		/* Print image info with timestamp */

#define	CONFIG_EXTRA_ENV_SETTINGS					\
	"addmisc=setenv bootargs ${bootargs} "				\
		"console=ttyS0,${baudrate} "				\
		"panic=1\0"						\
	"bootfile=/vmlinux.img\0"				\
	"load=tftp 80500000 ${u-boot}\0"				\
	""
/* Boot from NFS root */
#define CONFIG_BOOTCOMMAND	"bootp; setenv bootargs root=/dev/nfs rw nfsroot=${serverip}:${rootpath} ip=${ipaddr}:${serverip}:${gatewayip}:${netmask}:${hostname}::off; bootm"

/*
 * Miscellaneous configurable options
 */

#define CONFIG_SYS_MALLOC_LEN		128*1024

#define CONFIG_SYS_BOOTPARAMS_LEN	128*1024

#define CONFIG_SYS_MIPS_TIMER_FREQ	396000000

#define CONFIG_SYS_SDRAM_BASE		0x80000000     /* Cached addr */

#define	CONFIG_SYS_LOAD_ADDR		0x81000000     /* default load address	*/

#define CONFIG_SYS_MEMTEST_START	0x80100000
#undef CONFIG_SYS_MEMTEST_START
#define CONFIG_SYS_MEMTEST_START       0x80200000
#define CONFIG_SYS_MEMTEST_END		0x83800000

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#define CONFIG_SYS_MAX_FLASH_BANKS	2	/* max number of memory banks */
#define CONFIG_SYS_MAX_FLASH_SECT	(128)	/* max number of sectors on one chip */

#define PHYS_FLASH_1		0xbec00000 /* Flash Bank #1 */
#define PHYS_FLASH_2		0xbfc00000 /* Flash Bank #2 */

#define	CONFIG_SYS_MONITOR_BASE	CONFIG_SYS_TEXT_BASE
#define	CONFIG_SYS_MONITOR_LEN		(192 << 10)

#define CONFIG_SYS_INIT_SP_OFFSET	0x4000000

/* We boot from this flash, selected with dip switch */
#define CONFIG_SYS_FLASH_BASE		PHYS_FLASH_2

/* timeout values are in ticks */
#define CONFIG_SYS_FLASH_ERASE_TOUT	(2 * CONFIG_SYS_HZ) /* Timeout for Flash Erase */
#define CONFIG_SYS_FLASH_WRITE_TOUT	(2 * CONFIG_SYS_HZ) /* Timeout for Flash Write */

/* Address and size of Primary Environment Sector	*/
#define CONFIG_ENV_ADDR		0xB0030000
#define CONFIG_ENV_SIZE		0x10000

#define CONFIG_FLASH_16BIT

#define CONFIG_NR_DRAM_BANKS	2

#define CONFIG_MEMSIZE_IN_BYTES

/*---USB -------------------------------------------*/
#if 0
#define CONFIG_USB_OHCI
#endif

/*---ATA PCMCIA ------------------------------------*/
#if 0
#define CONFIG_SYS_PCMCIA_MEM_SIZE 0x4000000 /* Offset to slot 1 FIXME!!! */
#define CONFIG_SYS_PCMCIA_MEM_ADDR 0x20000000
#define CONFIG_PCMCIA_SLOT_A

#define CONFIG_ATAPI 1

/* We run CF in "true ide" mode or a harddrive via pcmcia */
#define CONFIG_IDE_PCMCIA 1

/* We only support one slot for now */
#define CONFIG_SYS_IDE_MAXBUS		1	/* max. 1 IDE bus		*/
#define CONFIG_SYS_IDE_MAXDEVICE	1	/* max. 1 drive per IDE bus	*/

#undef	CONFIG_IDE_RESET		/* reset for ide not supported	*/

#define CONFIG_SYS_ATA_IDE0_OFFSET	0x0000

#define CONFIG_SYS_ATA_BASE_ADDR       CONFIG_SYS_PCMCIA_MEM_ADDR

/* Offset for data I/O			*/
#define CONFIG_SYS_ATA_DATA_OFFSET     8

/* Offset for normal register accesses  */
#define CONFIG_SYS_ATA_REG_OFFSET      0

/* Offset for alternate registers       */
#define CONFIG_SYS_ATA_ALT_OFFSET      0x0100

#endif

/*
 * BOOTP options
 */
#define CONFIG_BOOTP_BOOTFILESIZE

/*
 * Command line configuration.
 */

#endif	/* __CONFIG_H */
