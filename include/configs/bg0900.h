/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2013 Marek Vasut <marex@denx.de>
 */
#ifndef __CONFIGS_BG0900_H__
#define __CONFIGS_BG0900_H__

/* Memory configuration */
#define CONFIG_NR_DRAM_BANKS		1		/* 1 bank of DRAM */
#define PHYS_SDRAM_1			0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE		0x10000000	/* Max 256 MB RAM */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1

/* Environment */
#define CONFIG_ENV_SIZE			(16 * 1024)
#define CONFIG_ENV_OVERWRITE

/* FEC Ethernet on SoC */
#ifdef	CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#endif

/* SPI */
#ifdef CONFIG_CMD_SPI
#define CONFIG_DEFAULT_SPI_BUS		2
#define CONFIG_DEFAULT_SPI_MODE		SPI_MODE_0

/* SPI FLASH */
#ifdef CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_BUS		2
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		40000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0

#define CONFIG_ENV_SPI_BUS		2
#define CONFIG_ENV_SPI_CS		0
#define CONFIG_ENV_SPI_MAX_HZ		40000000
#define CONFIG_ENV_SPI_MODE		SPI_MODE_0
#endif

#endif

/* Boot Linux */
#define CONFIG_BOOTFILE		"uImage"
#define CONFIG_BOOTCOMMAND	"bootm"
#define CONFIG_LOADADDR		0x42000000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR

/* Extra Environment */
#define CONFIG_EXTRA_ENV_SETTINGS					\
	"update_spi_firmware_filename=u-boot.sb\0"			\
	"update_spi_firmware_maxsz=0x80000\0"				\
	"update_spi_firmware="	/* Update the SPI flash firmware */	\
		"if sf probe 2:0 ; then "				\
		"if tftp ${update_spi_firmware_filename} ; then "	\
		"sf erase 0x0 +${filesize} ; "				\
		"sf write ${loadaddr} 0x0 ${filesize} ; "		\
		"fi ; "							\
		"fi\0"

/* The rest of the configuration is shared */
#include <configs/mxs.h>

#endif /* __CONFIGS_BG0900_H__ */
