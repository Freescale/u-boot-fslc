/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for Xilinx ZynqMP
 * (C) Copyright 2014 - 2015 Xilinx, Inc.
 * Michal Simek <michal.simek@xilinx.com>
 *
 * Based on Configuration for Versatile Express
 */

#ifndef __XILINX_ZYNQMP_H
#define __XILINX_ZYNQMP_H

#define CONFIG_REMAKE_ELF

/* #define CONFIG_ARMV8_SWITCH_TO_EL1 */

/* Generic Interrupt Controller Definitions */
#define CONFIG_GICV2
#define GICD_BASE	0xF9010000
#define GICC_BASE	0xF9020000

#ifndef CONFIG_SYS_MEMTEST_SCRATCH
# define CONFIG_SYS_MEMTEST_SCRATCH	0x10800000
#endif

#ifndef CONFIG_NR_DRAM_BANKS
# define CONFIG_NR_DRAM_BANKS		2
#endif
#define CONFIG_SYS_MEMTEST_START	0
#define CONFIG_SYS_MEMTEST_END		1000

#define CONFIG_SYS_INIT_SP_ADDR		CONFIG_SYS_TEXT_BASE

/* Generic Timer Definitions - setup in EL3. Setup by ATF for other cases */
#if !defined(COUNTER_FREQUENCY)
# define COUNTER_FREQUENCY		100000000
#endif

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + 0x2000000)

/* Serial setup */
#define CONFIG_ARM_DCC
#define CONFIG_CPU_ARMV8

#define CONFIG_SYS_BAUDRATE_TABLE \
	{ 4800, 9600, 19200, 38400, 57600, 115200 }

/* Command line configuration */
#define CONFIG_MP

/* BOOTP options */
#define CONFIG_BOOTP_BOOTFILESIZE
#define CONFIG_BOOTP_MAY_FAIL

#if defined(CONFIG_MMC_SDHCI_ZYNQ)
# define CONFIG_SUPPORT_EMMC_BOOT
#endif

#ifdef CONFIG_NAND_ARASAN
# define CONFIG_SYS_MAX_NAND_DEVICE	1
# define CONFIG_SYS_NAND_ONFI_DETECTION
# define CONFIG_MTD_DEVICE
#endif

#if defined(CONFIG_SPL_BUILD)
#define CONFIG_ZYNQMP_PSU_INIT_ENABLED
#endif

/* Miscellaneous configurable options */
#define CONFIG_SYS_LOAD_ADDR		0x8000000

#if defined(CONFIG_ZYNQMP_USB)
#define CONFIG_SYS_DFU_DATA_BUF_SIZE	0x1800000
#define DFU_DEFAULT_POLL_TIMEOUT	300
#define CONFIG_USB_CABLE_CHECK
#define CONFIG_THOR_RESET_OFF
#define DFU_ALT_INFO_RAM \
	"dfu_ram_info=" \
	"setenv dfu_alt_info " \
	"Image ram $kernel_addr $kernel_size\\\\;" \
	"system.dtb ram $fdt_addr $fdt_size\0" \
	"dfu_ram=run dfu_ram_info && dfu 0 ram 0\0" \
	"thor_ram=run dfu_ram_info && thordown 0 ram 0\0"

#define DFU_ALT_INFO  \
		DFU_ALT_INFO_RAM

#ifndef CONFIG_SPL_BUILD
# define PARTS_DEFAULT \
	"partitions=uuid_disk=${uuid_gpt_disk};" \
	"name=""boot"",size=16M,uuid=${uuid_gpt_boot};" \
	"name=""Linux"",size=-M,uuid=${uuid_gpt_Linux}\0"
#endif
#endif

#if !defined(DFU_ALT_INFO)
# define DFU_ALT_INFO
#endif

#if !defined(PARTS_DEFAULT)
# define PARTS_DEFAULT
#endif

/* Do not preserve environment */
#define CONFIG_ENV_SIZE			0x8000

/* Monitor Command Prompt */
/* Console I/O Buffer Size */
#define CONFIG_SYS_CBSIZE		2048
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE
#define CONFIG_PANIC_HANG
#define CONFIG_SYS_MAXARGS		64

/* Ethernet driver */
#if defined(CONFIG_ZYNQ_GEM)
# define CONFIG_MII
# define CONFIG_SYS_FAULT_ECHO_LINK_DOWN
# define PHY_ANEG_TIMEOUT       20000
#endif

/* I2C */
#if defined(CONFIG_SYS_I2C_ZYNQ)
# define CONFIG_SYS_I2C
#endif

/* EEPROM */
#ifdef CONFIG_ZYNQMP_EEPROM
# define CONFIG_SYS_I2C_EEPROM_ADDR_LEN		2
# define CONFIG_SYS_I2C_EEPROM_ADDR		0x54
# define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS	4
# define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	5
# define CONFIG_SYS_EEPROM_SIZE			(64 * 1024)
#endif

#ifdef CONFIG_SATA_CEVA
#define CONFIG_SYS_SCSI_MAX_SCSI_ID	2
#define CONFIG_SYS_SCSI_MAX_LUN		1
#define CONFIG_SYS_SCSI_MAX_DEVICE	(CONFIG_SYS_SCSI_MAX_SCSI_ID * \
					 CONFIG_SYS_SCSI_MAX_LUN)
#endif

#define CONFIG_SYS_BOOTM_LEN	(60 * 1024 * 1024)

#define CONFIG_CLOCKS

#define ENV_MEM_LAYOUT_SETTINGS \
	"fdt_high=10000000\0" \
	"initrd_high=10000000\0" \
	"fdt_addr_r=0x40000000\0" \
	"pxefile_addr_r=0x10000000\0" \
	"kernel_addr_r=0x18000000\0" \
	"scriptaddr=0x02000000\0" \
	"ramdisk_addr_r=0x02100000\0" \

#if defined(CONFIG_MMC_SDHCI_ZYNQ)
# define BOOT_TARGET_DEVICES_MMC(func)	func(MMC, mmc, 0) func(MMC, mmc, 1)
#else
# define BOOT_TARGET_DEVICES_MMC(func)
#endif

#if defined(CONFIG_SATA_CEVA)
# define BOOT_TARGET_DEVICES_SCSI(func)	func(SCSI, scsi, 0)
#else
# define BOOT_TARGET_DEVICES_SCSI(func)
#endif

#if defined(CONFIG_ZYNQMP_USB)
# define BOOT_TARGET_DEVICES_USB(func)	func(USB, usb, 0) func(USB, usb, 1)
#else
# define BOOT_TARGET_DEVICES_USB(func)
#endif

#if defined(CONFIG_CMD_PXE) && defined(CONFIG_CMD_DHCP)
# define BOOT_TARGET_DEVICES_PXE(func)	func(PXE, pxe, na)
#else
# define BOOT_TARGET_DEVICES_PXE(func)
#endif

#if defined(CONFIG_CMD_DHCP)
# define BOOT_TARGET_DEVICES_DHCP(func)	func(DHCP, dhcp, na)
#else
# define BOOT_TARGET_DEVICES_DHCP(func)
#endif

#define BOOT_TARGET_DEVICES(func) \
	BOOT_TARGET_DEVICES_MMC(func) \
	BOOT_TARGET_DEVICES_USB(func) \
	BOOT_TARGET_DEVICES_SCSI(func) \
	BOOT_TARGET_DEVICES_PXE(func) \
	BOOT_TARGET_DEVICES_DHCP(func)

#include <config_distro_bootcmd.h>

/* Initial environment variables */
#ifndef CONFIG_EXTRA_ENV_SETTINGS
#define CONFIG_EXTRA_ENV_SETTINGS \
	ENV_MEM_LAYOUT_SETTINGS \
	BOOTENV \
	DFU_ALT_INFO
#endif

/* SPL can't handle all huge variables - define just DFU */
#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_DFU_SUPPORT)
#undef CONFIG_EXTRA_ENV_SETTINGS
# define CONFIG_EXTRA_ENV_SETTINGS \
	"dfu_alt_info_ram=uboot.bin ram 0x8000000 0x1000000;" \
			  "atf-uboot.ub ram 0x10000000 0x1000000;" \
			  "Image ram 0x80000 0x3f80000;" \
			  "system.dtb ram 0x4000000 0x100000\0" \
	"dfu_bufsiz=0x1000\0"
#endif

#define CONFIG_SPL_TEXT_BASE		0xfffc0000
#define CONFIG_SPL_STACK		0xfffffffc
#define CONFIG_SPL_MAX_SIZE		0x40000

/* Just random location in OCM */
#define CONFIG_SPL_BSS_START_ADDR	0x0
#define CONFIG_SPL_BSS_MAX_SIZE		0x80000

#if defined(CONFIG_SPL_SPI_FLASH_SUPPORT)
# define CONFIG_SYS_SPI_KERNEL_OFFS	0x80000
# define CONFIG_SYS_SPI_ARGS_OFFS	0xa0000
# define CONFIG_SYS_SPI_ARGS_SIZE	0xa0000

# define CONFIG_SYS_SPI_U_BOOT_OFFS	0x170000
#endif

/* u-boot is like dtb */
#define CONFIG_SPL_FS_LOAD_ARGS_NAME	"u-boot.bin"
#define CONFIG_SYS_SPL_ARGS_ADDR	0x8000000

/* ATF is my kernel image */
#define CONFIG_SPL_FS_LOAD_KERNEL_NAME	"atf-uboot.ub"

/* FIT load address for RAM boot */
#define CONFIG_SPL_LOAD_FIT_ADDRESS	0x10000000

/* MMC support */
#ifdef CONFIG_MMC_SDHCI_ZYNQ
# define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION	1
# define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR	0 /* unused */
# define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS	0 /* unused */
# define CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR	0 /* unused */
# define CONFIG_SPL_FS_LOAD_PAYLOAD_NAME	"u-boot.img"
#endif

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_DFU_SUPPORT)
# undef CONFIG_CMD_BOOTD
# define CONFIG_SPL_ENV_SUPPORT
# define CONFIG_SPL_HASH_SUPPORT
# define CONFIG_ENV_MAX_ENTRIES	10
#endif

#define CONFIG_SYS_SPL_MALLOC_START	0x20000000
#define CONFIG_SYS_SPL_MALLOC_SIZE	0x100000

#ifdef CONFIG_SPL_SYS_MALLOC_SIMPLE
# error "Disable CONFIG_SPL_SYS_MALLOC_SIMPLE. Full malloc needs to be used"
#endif

#define CONFIG_BOARD_EARLY_INIT_F

#endif /* __XILINX_ZYNQMP_H */
