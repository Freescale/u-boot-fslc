/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2017
 * Lukasz Majewski, DENX Software Engineering, lukma@denx.de
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

/* Falcon Mode */
#define CONFIG_CMD_SPL
#define CONFIG_SYS_SPL_ARGS_ADDR	0x18000000
#define CONFIG_CMD_SPL_WRITE_SIZE	(44 * SZ_1K)

/* Falcon Mode - MMC support */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR	0x3F00
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS	\
	(CONFIG_CMD_SPL_WRITE_SIZE / 512)
#define CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR	0x100	/* 128KiB */

/*
 * display5 SPI-NOR memory layout
 *
 * The definition can be found in Kconfig's
 * CONFIG_MTDIDS_DEFAULT and CONFIG_MTDPARTS_DEFAULT
 *
 * 0x000000 - 0x020000 : SPI.SPL (128KiB)
 * 0x020000 - 0x120000 : SPI.u-boot (1MiB)
 * 0x120000 - 0x130000 : SPI.u-boot-env1 (64KiB)
 * 0x130000 - 0x140000 : SPI.u-boot-env2 (64KiB)
 * 0x140000 - 0x940000 : SPI.fitImage-recovery (8MiB)
 * 0x940000 - 0xD40000 : SPI.swupdate-kernel-FIT (4MiB)
 * 0xD40000 - 0x1540000 : SPI.swupdate-initramfs  (8MiB)
 */

#ifndef CONFIG_SPL_BUILD
#define CONFIG_MTD_DEVICE
#define CONFIG_SPI_FLASH_MTD
#define CONFIG_MTD_PARTITIONS
#endif

/* Below values are "dummy" - only to avoid build break */
#define CONFIG_SYS_SPI_KERNEL_OFFS      0x150000
#define CONFIG_SYS_SPI_ARGS_OFFS        0x140000
#define CONFIG_SYS_SPI_ARGS_SIZE        0x10000

#include "imx6_spl.h"
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x20000

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * 1024 * 1024)
#define CONFIG_MISC_INIT_R

/*#define CONFIG_MXC_UART*/
#define CONFIG_MXC_UART_BASE		UART5_BASE

/* SPI NOR Flash */
#ifdef CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_BUS		1
#define CONFIG_SF_DEFAULT_CS		(0 | (IMX_GPIO_NR(5, 29) << 8))
#define CONFIG_SF_DEFAULT_SPEED		50000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#endif

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1
#define CONFIG_SYS_I2C_MXC_I2C2
#define CONFIG_SYS_I2C_MXC_I2C3
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_I2C_EDID
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN  2

/* Ethernet */
#ifdef CONFIG_FEC_MXC
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		0
#define CONFIG_MII
#endif

/* MMC Configs */
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SUPPORT_EMMC_BOOT

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_BAUDRATE			115200

#ifndef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND "run boot_mmc"
#endif

#define PARTS_DEFAULT \
	/* Linux partitions */ \
	"partitions=" \
	"uuid_disk=${uuid_gpt_disk};" \
	"name=kernel_raw1,start=128K,size=8M,uuid=${uuid_gpt_kernel_raw1};" \
	"name=rootfs1,size=1528M,uuid=${uuid_gpt_rootfs1};" \
	"name=kernel_raw2,size=8M,uuid=${uuid_gpt_kernel_raw2};" \
	"name=rootfs2,size=1528M,uuid=${uuid_gpt_rootfs2};" \
	"name=data,size=-,uuid=${uuid_gpt_data}\0"

#define FACTORY_PROCEDURE \
	"echo '#######################';" \
	"echo '# Factory Boot        #';" \
	"echo '#######################';" \
	"env default -a;" \
	"saveenv;" \
	"gpt write mmc ${mmcdev} ${partitions};" \
	"run tftp_sf_SPL;" \
	"run tftp_sf_uboot;" \
	TFTP_UPDATE_KERNEL \
	"run tftp_sf_fitImg_recovery;" \
	"run tftp_sf_fitImg_SWU;" \
	"run tftp_sf_initramfs_SWU;" \
	TFTP_UPDATE_ROOTFS \
	"echo '#######################';" \
	"echo '# END - OK            #';" \
	"echo '#######################';" \
	"setenv bootcmd 'env default -a; saveenv; run falcon_setup; reset';" \
	"setenv boot_os 'n';" \
	"saveenv;" \
	"reset;"

#define SWUPDATE_RECOVERY_PROCEDURE \
	"echo '#######################';" \
	"echo '# RECOVERY SWUupdate  #';" \
	"echo '#######################';" \
	"setenv loadaddr_swu_initramfs 0x14000000;" \
	"setenv bootargs console=${console} " \
		"ip=${ipaddr}:${serverip}:${gatewayip}:${netmask}" \
		":${hostname}::off root=/dev/ram rw;" \
	"sf probe;" \
	"sf read ${loadaddr} swu-kernel;" \
	"sf read ${loadaddr_swu_initramfs} swu-initramfs;" \
	"bootm ${loadaddr} ${loadaddr_swu_initramfs};"

#define KERNEL_RECOVERY_PROCEDURE \
	"echo '#######################';" \
	"echo '# RECOVERY KERNEL IMG #';" \
	"echo '#######################';" \
	"sf probe;" \
	"sf read ${loadaddr} lin-recovery;" \
	"bootm;"

#define SETUP_BOOTARGS \
	"run set_rootfs_part;" \
	"setenv bootargs ${bootargs} console=${console} "	  \
		      "root=/dev/mmcblk${mmcdev}p${rootfs_part} " \
		      "rootwait rootfstype=ext4 rw; " \
	"run set_kernel_part;" \
	"part start mmc ${mmcdev} ${kernel_part} lba_start; " \
	"mmc read ${loadaddr} ${lba_start} 0x2000; " \
	"setenv fdt_conf imx6q-${board}-${display}.dtb; "

/* All the numbers are in LBAs */
#define __TFTP_UPDATE_KERNEL \
	"tftp_mmc_fitImg=" \
	   "if test ! -n ${kernel_part}; then " \
	       "setenv kernel_part ${kernel_part_active};" \
	   "fi;" \
	   "if tftp ${loadaddr} ${kernel_file}; then " \
	       "setexpr fw_sz ${filesize} / 0x200; " \
	       "setexpr fw_sz ${fw_sz} + 1; "  \
	       "part start mmc ${mmcdev} ${kernel_part} lba_start; " \
	       "mmc write ${loadaddr} ${lba_start} ${fw_sz}; " \
	   "; fi\0" \

#define TFTP_UPDATE_KERNEL \
	"setenv kernel_part ${kernel_part_active};" \
	"run tftp_mmc_fitImg;" \
	"setenv kernel_part ${kernel_part_backup};" \
	"run tftp_mmc_fitImg;" \

#define __TFTP_UPDATE_ROOTFS \
	"tftp_mmc_rootfs=" \
	   "if test ! -n ${rootfs_part}; then " \
	       "setenv rootfs_part ${rootfs_part_active};" \
	   "fi;" \
	   "if tftp ${loadaddr} ${rootfs_file}; then " \
	       "setexpr fw_sz ${filesize} / 0x200; " \
	       "setexpr fw_sz ${fw_sz} + 1; "  \
	       "part start mmc ${mmcdev} ${rootfs_part} lba_start; " \
	       "mmc write ${loadaddr} ${lba_start} ${fw_sz}; " \
	   "; fi\0" \

/* To save some considerable time, we only once download the rootfs image */
/* and store it on 'active' and 'backup' rootfs partitions */
#define TFTP_UPDATE_ROOTFS \
	"setenv rootfs_part ${rootfs_part_active};" \
	"run tftp_mmc_rootfs;" \
	"part start mmc ${mmcdev} ${rootfs_part_backup} lba_start;" \
	"mmc write ${loadaddr} ${lba_start} ${fw_sz};" \

#define TFTP_UPDATE_RECOVERY_SWU_KERNEL \
	"tftp_sf_fitImg_SWU=" \
	    "if tftp ${loadaddr} ${kernel_file}; then " \
		"sf probe;" \
		"sf erase swu-kernel +${filesize};" \
		"sf write ${loadaddr} swu-kernel ${filesize};" \
	"; fi\0"	  \

#define TFTP_UPDATE_RECOVERY_SWU_INITRAMFS \
	"swu_initramfs_file=swupdate-image-display5.ext3.gz.u-boot\0" \
	"tftp_sf_initramfs_SWU=" \
	    "if tftp ${loadaddr} ${swu_initramfs_file}; then " \
		"sf probe;" \
		"sf erase swu-initramfs +${filesize};" \
		"sf write ${loadaddr} swu-initramfs ${filesize};" \
	"; fi\0"	  \

#define TFTP_UPDATE_RECOVERY_KERNEL_INITRAMFS \
	"kernel_recovery_file=fitImage-initramfs\0" \
	"tftp_sf_fitImg_recovery=" \
	    "if tftp ${loadaddr} ${kernel_recovery_file}; then " \
		"sf probe;" \
		"sf erase lin-recovery +${filesize};" \
		"sf write ${loadaddr} lin-recovery ${filesize};" \
	"; fi\0"	  \

#define TFTP_UPDATE_BOOTLOADER \
	"ubootfile=u-boot.img\0" \
	"ubootfileSPL=SPL\0" \
	"tftp_sf_uboot=" \
	    "if tftp ${loadaddr} ${ubootfile}; then " \
		"sf probe;" \
		"sf erase u-boot +${filesize};" \
		"sf write ${loadaddr} u-boot ${filesize}" \
	"; fi\0"	  \
	"tftp_sf_SPL="	  \
	    "if tftp ${loadaddr} ${ubootfileSPL}; then " \
		"sf probe;" \
		"setexpr uboot_SPL_size ${filesize} + 0x400;" \
		"sf erase 0x0 +${uboot_SPL_size};" \
		"sf write ${loadaddr} 0x400 ${filesize};" \
	"fi\0" \

#define CONFIG_EXTRA_ENV_SETTINGS	  \
	PARTS_DEFAULT \
	"display=tianma-tm070-800x480\0" \
	"board=display5\0" \
	"mmcdev=0\0" \
	"altbootcmd=run recovery\0" \
	"bootdelay=1\0" \
	"baudrate=115200\0" \
	"bootcmd=" CONFIG_BOOTCOMMAND "\0" \
	"factory=" FACTORY_PROCEDURE "\0" \
	"bootlimit=3\0" \
	"ethact=FEC\0" \
	"netdev=eth0\0" \
	"boot_os=y\0" \
	"hostname=display5\0" \
	"loadaddr=0x12000000\0" \
	"fdtaddr=0x12800000\0" \
	"console=ttymxc4,115200 quiet\0" \
	"fdtfile=imx6q-display5.dtb\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"kernel_file=fitImage\0" \
	"up=run tftp_sf_SPL; run tftp_sf_uboot\0" \
	"download_kernel=" \
		"tftpboot ${loadaddr} ${kernel_file};\0" \
	"boot_kernel_recovery=" KERNEL_RECOVERY_PROCEDURE "\0" \
	"boot_swu_recovery=" SWUPDATE_RECOVERY_PROCEDURE "\0" \
	"recovery=" \
	"if test ${BOOT_FROM_RECOVERY} = SWU; then " \
	     "echo BOOT: RECOVERY: SWU;" \
	     "run boot_swu_recovery;" \
	"else " \
	     "echo BOOT: RECOVERY: Linux;" \
	     "run boot_kernel_recovery;" \
	"fi\0" \
	"boot_tftp=" \
	"if run download_kernel; then "	  \
	     "setenv bootargs console=${console} " \
	     "root=/dev/mmcblk0p2 rootwait;" \
	     "bootm ${loadaddr} - ${fdtaddr};" \
	"fi\0" \
	"addip=setenv bootargs ${bootargs} " \
	"ip=${ipaddr}:${serverip}:${gatewayip}:${netmask}:" \
	    "${hostname}:eth0:on"	  \
	"\0"	  \
	"nfsargs=setenv bootargs " \
	"root=/dev/nfs rw "	  \
	"nfsroot=${serverip}:${rootpath},nolock,nfsvers=3" \
	"\0" \
	"rootpath=/srv/tftp/DISP5/rootfs\0" \
	"boot_nfs=" \
	"if run download_kernel; then "	  \
	     "run nfsargs;"	  \
	     "run addip;"	  \
	     "setenv bootargs ${bootargs} console=${console};"	  \
	     "setenv fdt_conf imx6q-${board}-${display}.dtb; " \
	     "bootm ${loadaddr}#conf@${fdt_conf};" \
	"fi\0" \
	"falcon_setup=" \
	"if mmc dev ${mmcdev}; then "	  \
	     SETUP_BOOTARGS \
	     "spl export fdt ${loadaddr}#conf@${fdt_conf};" \
	     "setexpr fw_sz ${fdtargslen} / 0x200; " \
	     "setexpr fw_sz ${fw_sz} + 1; "  \
	     "mmc write ${fdtargsaddr} " \
	     __stringify(CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR)" ${fw_sz}; " \
	"fi\0" \
	"boot_mmc=" \
	"if mmc dev ${mmcdev}; then "	  \
	     SETUP_BOOTARGS \
	     "bootm ${loadaddr}#conf@${fdt_conf};" \
	"fi\0" \
	"set_kernel_part=" \
	"if test ${BOOT_FROM} = ACTIVE; then " \
	     "setenv kernel_part ${kernel_part_active};" \
	     "echo BOOT: ACTIVE;" \
	"else if test ${BOOT_FROM} = BACKUP; then " \
	     "setenv kernel_part ${kernel_part_backup};" \
	     "echo BOOT: BACKUP;" \
	"else " \
	     "run recovery;" \
	"fi;fi\0" \
	"set_rootfs_part=" \
	"if test ${BOOT_FROM} = ACTIVE; then " \
	     "setenv rootfs_part ${rootfs_part_active};" \
	"else if test ${BOOT_FROM} = BACKUP; then " \
	     "setenv rootfs_part ${rootfs_part_backup};" \
	"else " \
	     "run recovery;" \
	"fi;fi\0" \
	"BOOT_FROM=ACTIVE\0" \
	"BOOT_FROM_RECOVERY=Linux\0" \
	TFTP_UPDATE_BOOTLOADER \
	"kernel_part_active=1\0" \
	"kernel_part_backup=3\0" \
	__TFTP_UPDATE_KERNEL \
	"rootfs_part_active=2\0" \
	"rootfs_part_backup=4\0" \
	"rootfs_file=core-image-lwn-display5.ext4\0" \
	__TFTP_UPDATE_ROOTFS \
	TFTP_UPDATE_RECOVERY_KERNEL_INITRAMFS \
	TFTP_UPDATE_RECOVERY_SWU_KERNEL \
	TFTP_UPDATE_RECOVERY_SWU_INITRAMFS \
	"\0" \

/* Miscellaneous configurable options */
#undef CONFIG_SYS_CBSIZE
#define CONFIG_SYS_CBSIZE		2048

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					 sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		32
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE

#define CONFIG_STANDALONE_LOAD_ADDR	0x10001000
#define CONFIG_SYS_HZ			1000

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM

#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Commands */
#define CONFIG_MTD_PARTITIONS
#define CONFIG_MTD_DEVICE

/* ENV config */
#ifdef CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_SIZE		(SZ_64K)
/* The 0x120000 value corresponds to above SPI-NOR memory MAP */
#define CONFIG_ENV_OFFSET		(0x120000)
#define CONFIG_ENV_SECT_SIZE		(SZ_64K)
#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + \
						CONFIG_ENV_SECT_SIZE)
#define CONFIG_ENV_SIZE_REDUND		CONFIG_ENV_SIZE

#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#endif

#define CONFIG_MXC_USB_PORTSC           (PORT_PTS_UTMI | PORT_PTS_PTW)
#endif /* __CONFIG_H */
