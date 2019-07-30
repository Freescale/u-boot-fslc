/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
 * Copyright (C) 2010 Ilya Yanok, Emcraft Systems, yanok@emcraft.com
 *
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * High Level Configuration Options
 */
#define CONFIG_E300		1 /* E300 family */
#define CONFIG_MPC830x		1 /* MPC830x family */
#define CONFIG_MPC8308		1 /* MPC8308 CPU specific */

#define CONFIG_MISC_INIT_R

#ifdef CONFIG_MMC
#define CONFIG_SYS_FSL_ESDHC_ADDR	CONFIG_SYS_MPC83xx_ESDHC_ADDR
#define CONFIG_SYS_FSL_ESDHC_USE_PIO
#endif

/*
 * On-board devices
 *
 * TSEC1 is SoC TSEC
 * TSEC2 is VSC switch
 */
#define CONFIG_TSEC1
#define CONFIG_VSC7385_ENET

/*
 * System Clock Setup
 */
#define CONFIG_83XX_CLKIN	33333333 /* in Hz */
#define CONFIG_SYS_CLK_FREQ	CONFIG_83XX_CLKIN

/*
 * Hardware Reset Configuration Word
 * if CLKIN is 66.66MHz, then
 * CSB = 133MHz, DDRC = 266MHz, LBC = 133MHz
 * We choose the A type silicon as default, so the core is 400Mhz.
 */
#define CONFIG_SYS_HRCW_LOW (\
	HRCWL_LCL_BUS_TO_SCB_CLK_1X1 |\
	HRCWL_DDR_TO_SCB_CLK_2X1 |\
	HRCWL_SVCOD_DIV_2 |\
	HRCWL_CSB_TO_CLKIN_4X1 |\
	HRCWL_CORE_TO_CSB_3X1)
/*
 * There are neither HRCWH_PCI_HOST nor HRCWH_PCI1_ARBITER_ENABLE bits
 * in 8308's HRCWH according to the manual, but original Freescale's
 * code has them and I've expirienced some problems using the board
 * with BDI3000 attached when I've tried to set these bits to zero
 * (UART doesn't work after the 'reset run' command).
 */
#define CONFIG_SYS_HRCW_HIGH (\
	HRCWH_PCI_HOST |\
	HRCWH_PCI1_ARBITER_ENABLE |\
	HRCWH_CORE_ENABLE |\
	HRCWH_FROM_0X00000100 |\
	HRCWH_BOOTSEQ_DISABLE |\
	HRCWH_SW_WATCHDOG_DISABLE |\
	HRCWH_ROM_LOC_LOCAL_16BIT |\
	HRCWH_RL_EXT_LEGACY |\
	HRCWH_TSEC1M_IN_RGMII |\
	HRCWH_TSEC2M_IN_RGMII |\
	HRCWH_BIG_ENDIAN)

/*
 * System IO Config
 */
#define CONFIG_SYS_SICRH (\
	SICRH_ESDHC_A_SD |\
	SICRH_ESDHC_B_SD |\
	SICRH_ESDHC_C_SD |\
	SICRH_GPIO_A_TSEC2 |\
	SICRH_GPIO_B_TSEC2_GTX_CLK125 |\
	SICRH_IEEE1588_A_GPIO |\
	SICRH_USB |\
	SICRH_GTM_GPIO |\
	SICRH_IEEE1588_B_GPIO |\
	SICRH_ETSEC2_CRS |\
	SICRH_GPIOSEL_1 |\
	SICRH_TMROBI_V3P3 |\
	SICRH_TSOBI1_V2P5 |\
	SICRH_TSOBI2_V2P5)	/* 0x01b7d103 */
#define CONFIG_SYS_SICRL (\
	SICRL_SPI_PF0 |\
	SICRL_UART_PF0 |\
	SICRL_IRQ_PF0 |\
	SICRL_I2C2_PF0 |\
	SICRL_ETSEC1_GTX_CLK125)	/* 0x00000040 */

/*
 * IMMR new address
 */
#define CONFIG_SYS_IMMR		0xE0000000

/*
 * SERDES
 */
#define CONFIG_FSL_SERDES
#define CONFIG_FSL_SERDES1	0xe3000

/*
 * Arbiter Setup
 */
#define CONFIG_SYS_ACR_PIPE_DEP	3 /* Arbiter pipeline depth is 4 */
#define CONFIG_SYS_ACR_RPTCNT	3 /* Arbiter repeat count is 4 */
#define CONFIG_SYS_SPCR_TSECEP	3 /* eTSEC emergency priority is highest */

/*
 * DDR Setup
 */
#define CONFIG_SYS_DDR_BASE		0x00000000 /* DDR is system memory */
#define CONFIG_SYS_SDRAM_BASE		CONFIG_SYS_DDR_BASE
#define CONFIG_SYS_DDR_SDRAM_BASE	CONFIG_SYS_DDR_BASE
#define CONFIG_SYS_DDR_SDRAM_CLK_CNTL	DDR_SDRAM_CLK_CNTL_CLK_ADJUST_05
#define CONFIG_SYS_DDRCDR_VALUE	(DDRCDR_EN \
				| DDRCDR_PZ_LOZ \
				| DDRCDR_NZ_LOZ \
				| DDRCDR_ODT \
				| DDRCDR_Q_DRN)
				/* 0x7b880001 */
/*
 * Manually set up DDR parameters
 * consist of two chips HY5PS12621BFP-C4 from HYNIX
 */

#define CONFIG_SYS_DDR_SIZE		128 /* MB */

#define CONFIG_SYS_DDR_CS0_BNDS	0x00000007
#define CONFIG_SYS_DDR_CS0_CONFIG	(CSCONFIG_EN \
				| CSCONFIG_ODT_RD_NEVER \
				| CSCONFIG_ODT_WR_ONLY_CURRENT \
				| CSCONFIG_ROW_BIT_13 | CSCONFIG_COL_BIT_10)
				/* 0x80010102 */
#define CONFIG_SYS_DDR_TIMING_3	0x00000000
#define CONFIG_SYS_DDR_TIMING_0	((0 << TIMING_CFG0_RWT_SHIFT) \
				| (0 << TIMING_CFG0_WRT_SHIFT) \
				| (0 << TIMING_CFG0_RRT_SHIFT) \
				| (0 << TIMING_CFG0_WWT_SHIFT) \
				| (2 << TIMING_CFG0_ACT_PD_EXIT_SHIFT) \
				| (2 << TIMING_CFG0_PRE_PD_EXIT_SHIFT) \
				| (8 << TIMING_CFG0_ODT_PD_EXIT_SHIFT) \
				| (2 << TIMING_CFG0_MRS_CYC_SHIFT))
				/* 0x00220802 */
#define CONFIG_SYS_DDR_TIMING_1	((2 << TIMING_CFG1_PRETOACT_SHIFT) \
				| (7 << TIMING_CFG1_ACTTOPRE_SHIFT) \
				| (2 << TIMING_CFG1_ACTTORW_SHIFT) \
				| (5 << TIMING_CFG1_CASLAT_SHIFT) \
				| (6 << TIMING_CFG1_REFREC_SHIFT) \
				| (2 << TIMING_CFG1_WRREC_SHIFT) \
				| (2 << TIMING_CFG1_ACTTOACT_SHIFT) \
				| (2 << TIMING_CFG1_WRTORD_SHIFT))
				/* 0x27256222 */
#define CONFIG_SYS_DDR_TIMING_2	((1 << TIMING_CFG2_ADD_LAT_SHIFT) \
				| (4 << TIMING_CFG2_CPO_SHIFT) \
				| (2 << TIMING_CFG2_WR_LAT_DELAY_SHIFT) \
				| (2 << TIMING_CFG2_RD_TO_PRE_SHIFT) \
				| (2 << TIMING_CFG2_WR_DATA_DELAY_SHIFT) \
				| (3 << TIMING_CFG2_CKE_PLS_SHIFT) \
				| (5 << TIMING_CFG2_FOUR_ACT_SHIFT))
				/* 0x121048c5 */
#define CONFIG_SYS_DDR_INTERVAL	((0x0360 << SDRAM_INTERVAL_REFINT_SHIFT) \
				| (0x0100 << SDRAM_INTERVAL_BSTOPRE_SHIFT))
				/* 0x03600100 */
#define CONFIG_SYS_DDR_SDRAM_CFG	(SDRAM_CFG_SREN \
				| SDRAM_CFG_SDRAM_TYPE_DDR2 \
				| SDRAM_CFG_DBW_32)
				/* 0x43080000 */

#define CONFIG_SYS_DDR_SDRAM_CFG2	0x00401000 /* 1 posted refresh */
#define CONFIG_SYS_DDR_MODE		((0x0448 << SDRAM_MODE_ESD_SHIFT) \
				| (0x0232 << SDRAM_MODE_SD_SHIFT))
				/* ODT 150ohm CL=3, AL=1 on SDRAM */
#define CONFIG_SYS_DDR_MODE2		0x00000000

/*
 * Memory test
 */
#define CONFIG_SYS_MEMTEST_START	0x00001000 /* memtest region */
#define CONFIG_SYS_MEMTEST_END		0x07f00000

/*
 * The reserved memory
 */
#define CONFIG_SYS_MONITOR_BASE	CONFIG_SYS_TEXT_BASE /* start of monitor */

#define CONFIG_SYS_MONITOR_LEN	(512 * 1024) /* Reserve 512 kB for Mon */
#define CONFIG_SYS_MALLOC_LEN	(512 * 1024) /* Reserved for malloc */

/*
 * Initial RAM Base Address Setup
 */
#define CONFIG_SYS_INIT_RAM_LOCK	1
#define CONFIG_SYS_INIT_RAM_ADDR	0xE6000000 /* Initial RAM address */
#define CONFIG_SYS_INIT_RAM_SIZE	0x1000 /* Size of used area in RAM */
#define CONFIG_SYS_GBL_DATA_OFFSET	\
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)

/*
 * Local Bus Configuration & Clock Setup
 */
#define CONFIG_SYS_LCRR_DBYP		LCRR_DBYP
#define CONFIG_SYS_LCRR_CLKDIV		LCRR_CLKDIV_2
#define CONFIG_SYS_LBC_LBCR		0x00040000

/*
 * FLASH on the Local Bus
 */
#define CONFIG_SYS_FLASH_CFI		/* use the Common Flash Interface */
#define CONFIG_FLASH_CFI_DRIVER		/* use the CFI driver */
#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT

#define CONFIG_SYS_FLASH_BASE		0xFE000000 /* FLASH base address */
#define CONFIG_SYS_FLASH_SIZE		8 /* FLASH size is 8M */
#define CONFIG_SYS_FLASH_PROTECTION	1 /* Use h/w Flash protection. */

/* Window base at flash base */
#define CONFIG_SYS_LBLAWBAR0_PRELIM	CONFIG_SYS_FLASH_BASE
#define CONFIG_SYS_LBLAWAR0_PRELIM	(LBLAWAR_EN | LBLAWAR_8MB)

#define CONFIG_SYS_BR0_PRELIM	(CONFIG_SYS_FLASH_BASE \
				| BR_PS_16	/* 16 bit port */ \
				| BR_MS_GPCM	/* MSEL = GPCM */ \
				| BR_V)		/* valid */
#define CONFIG_SYS_OR0_PRELIM	(MEG_TO_AM(CONFIG_SYS_FLASH_SIZE) \
				| OR_UPM_XAM \
				| OR_GPCM_CSNT \
				| OR_GPCM_ACS_DIV2 \
				| OR_GPCM_XACS \
				| OR_GPCM_SCY_15 \
				| OR_GPCM_TRLX_SET \
				| OR_GPCM_EHTR_SET)

#define CONFIG_SYS_MAX_FLASH_BANKS	1 /* number of banks */
/* 127 64KB sectors and 8 8KB top sectors per device */
#define CONFIG_SYS_MAX_FLASH_SECT	135

#define CONFIG_SYS_FLASH_ERASE_TOUT	60000 /* Flash Erase Timeout (ms) */
#define CONFIG_SYS_FLASH_WRITE_TOUT	500 /* Flash Write Timeout (ms) */

/*
 * NAND Flash on the Local Bus
 */
#define CONFIG_SYS_NAND_BASE	0xE0600000		/* 0xE0600000 */
#define CONFIG_SYS_NAND_WINDOW_SIZE	(32 * 1024)	/* 0x00008000 */
#define CONFIG_SYS_BR1_PRELIM	(CONFIG_SYS_NAND_BASE \
				| BR_DECC_CHK_GEN	/* Use HW ECC */ \
				| BR_PS_8		/* 8 bit Port */ \
				| BR_MS_FCM		/* MSEL = FCM */ \
				| BR_V)			/* valid */
#define CONFIG_SYS_OR1_PRELIM	(P2SZ_TO_AM(CONFIG_SYS_NAND_WINDOW_SIZE) \
				| OR_FCM_CSCT \
				| OR_FCM_CST \
				| OR_FCM_CHT \
				| OR_FCM_SCY_1 \
				| OR_FCM_TRLX \
				| OR_FCM_EHTR)
				/* 0xFFFF8396 */

#define CONFIG_SYS_LBLAWBAR1_PRELIM	CONFIG_SYS_NAND_BASE
#define CONFIG_SYS_LBLAWAR1_PRELIM	(LBLAWAR_EN | LBLAWAR_32KB)

#ifdef CONFIG_VSC7385_ENET
#define CONFIG_TSEC2
					/* VSC7385 Base address on CS2 */
#define CONFIG_SYS_VSC7385_BASE		0xF0000000
#define CONFIG_SYS_VSC7385_SIZE		(128 * 1024) /* 0x00020000 */
#define CONFIG_SYS_BR2_PRELIM		(CONFIG_SYS_VSC7385_BASE \
					| BR_PS_8	/* 8-bit port */ \
					| BR_MS_GPCM	/* MSEL = GPCM */ \
					| BR_V)		/* valid */
					/* 0xF0000801 */
#define CONFIG_SYS_OR2_PRELIM		(P2SZ_TO_AM(CONFIG_SYS_VSC7385_SIZE) \
					| OR_GPCM_CSNT \
					| OR_GPCM_XACS \
					| OR_GPCM_SCY_15 \
					| OR_GPCM_SETA \
					| OR_GPCM_TRLX_SET \
					| OR_GPCM_EHTR_SET)
					/* 0xFFFE09FF */
/* Access window base at VSC7385 base */
#define CONFIG_SYS_LBLAWBAR2_PRELIM	CONFIG_SYS_VSC7385_BASE
/* Access window size 128K */
#define CONFIG_SYS_LBLAWAR2_PRELIM	(LBLAWAR_EN | LBLAWAR_128KB)
/* The flash address and size of the VSC7385 firmware image */
#define CONFIG_VSC7385_IMAGE		0xFE7FE000
#define CONFIG_VSC7385_IMAGE_SIZE	8192
#endif
/*
 * Serial Port
 */
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	1
#define CONFIG_SYS_NS16550_CLK		get_bus_freq(0)

#define CONFIG_SYS_BAUDRATE_TABLE  \
	{300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200}

#define CONFIG_SYS_NS16550_COM1	(CONFIG_SYS_IMMR + 0x4500)
#define CONFIG_SYS_NS16550_COM2	(CONFIG_SYS_IMMR + 0x4600)

/* I2C */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_FSL
#define CONFIG_SYS_FSL_I2C_SPEED	400000
#define CONFIG_SYS_FSL_I2C_SLAVE	0x7F
#define CONFIG_SYS_FSL_I2C_OFFSET	0x3000
#define CONFIG_SYS_FSL_I2C2_SPEED	400000
#define CONFIG_SYS_FSL_I2C2_SLAVE	0x7F
#define CONFIG_SYS_FSL_I2C2_OFFSET	0x3100
#define CONFIG_SYS_I2C_NOPROBES		{ {0, 0x51} }

/*
 * SPI on header J8
 *
 * WARNING: enabling this will break TSEC2 (connected to the Vitesse switch)
 * due to a pinmux conflict between GPIO9 (SPI chip select )and the TSEC2 pins.
 */
#ifdef CONFIG_MPC8XXX_SPI
#define CONFIG_USE_SPIFLASH
#endif

/*
 * Board info - revision and where boot from
 */
#define CONFIG_SYS_I2C_PCF8574A_ADDR	0x39

/*
 * Config on-board RTC
 */
#define CONFIG_RTC_DS1337	/* ds1339 on board, use ds1337 rtc via i2c */
#define CONFIG_SYS_I2C_RTC_ADDR	0x68 /* at address 0x68 */

/*
 * General PCI
 * Addresses are mapped 1-1.
 */
#define CONFIG_SYS_PCIE1_BASE		0xA0000000
#define CONFIG_SYS_PCIE1_MEM_BASE	0xA0000000
#define CONFIG_SYS_PCIE1_MEM_PHYS	0xA0000000
#define CONFIG_SYS_PCIE1_MEM_SIZE	0x10000000
#define CONFIG_SYS_PCIE1_CFG_BASE	0xB0000000
#define CONFIG_SYS_PCIE1_CFG_SIZE	0x01000000
#define CONFIG_SYS_PCIE1_IO_BASE	0x00000000
#define CONFIG_SYS_PCIE1_IO_PHYS	0xB1000000
#define CONFIG_SYS_PCIE1_IO_SIZE	0x00800000

/* enable PCIE clock */
#define CONFIG_SYS_SCCR_PCIEXP1CM	1

#define CONFIG_PCI_INDIRECT_BRIDGE
#define CONFIG_PCIE

#define CONFIG_SYS_PCI_SUBSYS_VENDORID 0x1957	/* Freescale */
#define CONFIG_83XX_GENERIC_PCIE_REGISTER_HOSES 1

/*
 * TSEC
 */
#define CONFIG_SYS_TSEC1_OFFSET	0x24000
#define CONFIG_SYS_TSEC1	(CONFIG_SYS_IMMR+CONFIG_SYS_TSEC1_OFFSET)
#define CONFIG_SYS_TSEC2_OFFSET	0x25000
#define CONFIG_SYS_TSEC2	(CONFIG_SYS_IMMR+CONFIG_SYS_TSEC2_OFFSET)

/*
 * TSEC ethernet configuration
 */
#define CONFIG_MII		1 /* MII PHY management */
#define CONFIG_TSEC1_NAME	"eTSEC0"
#define CONFIG_TSEC2_NAME	"eTSEC1"
#define TSEC1_PHY_ADDR		2
#define TSEC2_PHY_ADDR		1
#define TSEC1_PHYIDX		0
#define TSEC2_PHYIDX		0
#define TSEC1_FLAGS		TSEC_GIGABIT
#define TSEC2_FLAGS		TSEC_GIGABIT

/* Options are: eTSEC[0-1] */
#define CONFIG_ETHPRIME		"eTSEC0"

/*
 * Environment
 */
#define CONFIG_ENV_ADDR		(CONFIG_SYS_MONITOR_BASE + \
				 CONFIG_SYS_MONITOR_LEN)
#define CONFIG_ENV_SECT_SIZE	0x10000 /* 64K(one sector) for env */
#define CONFIG_ENV_SIZE		0x2000
#define CONFIG_ENV_ADDR_REDUND	(CONFIG_ENV_ADDR + CONFIG_ENV_SECT_SIZE)
#define CONFIG_ENV_SIZE_REDUND	CONFIG_ENV_SIZE

#define CONFIG_LOADS_ECHO	1	/* echo on for serial download */
#define CONFIG_SYS_LOADS_BAUD_CHANGE	1	/* allow baudrate change */

/*
 * BOOTP options
 */
#define CONFIG_BOOTP_BOOTFILESIZE

/*
 * Command line configuration.
 */

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LOAD_ADDR		0x2000000 /* default load address */

#define CONFIG_SYS_CBSIZE	1024 /* Console I/O Buffer Size */

/* Boot Argument Buffer Size */
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE

/*
 * For booting Linux, the board info and command line data
 * have to be in the first 256 MB of memory, since this is
 * the maximum mapped by the Linux kernel during initialization.
 */
#define CONFIG_SYS_BOOTMAPSZ	(256 << 20) /* Initial Memory map for Linux */
#define CONFIG_SYS_BOOTM_LEN	(64 << 20)	/* Increase max gunzip size */

/*
 * Core HID Setup
 */
#define CONFIG_SYS_HID0_INIT	0x000000000
#define CONFIG_SYS_HID0_FINAL	(HID0_ENABLE_MACHINE_CHECK | \
				 HID0_ENABLE_INSTRUCTION_CACHE | \
				 HID0_ENABLE_DYNAMIC_POWER_MANAGMENT)
#define CONFIG_SYS_HID2		HID2_HBE

/*
 * MMU Setup
 */

/* DDR: cache cacheable */
#define CONFIG_SYS_IBAT0L	(CONFIG_SYS_SDRAM_BASE | BATL_PP_RW | \
					BATL_MEMCOHERENCE)
#define CONFIG_SYS_IBAT0U	(CONFIG_SYS_SDRAM_BASE | BATU_BL_128M | \
					BATU_VS | BATU_VP)
#define CONFIG_SYS_DBAT0L	CONFIG_SYS_IBAT0L
#define CONFIG_SYS_DBAT0U	CONFIG_SYS_IBAT0U

/* IMMRBAR, PCI IO and NAND: cache-inhibit and guarded */
#define CONFIG_SYS_IBAT1L	(CONFIG_SYS_IMMR | BATL_PP_RW | \
			BATL_CACHEINHIBIT | BATL_GUARDEDSTORAGE)
#define CONFIG_SYS_IBAT1U	(CONFIG_SYS_IMMR | BATU_BL_8M | BATU_VS | \
					BATU_VP)
#define CONFIG_SYS_DBAT1L	CONFIG_SYS_IBAT1L
#define CONFIG_SYS_DBAT1U	CONFIG_SYS_IBAT1U

/* FLASH: icache cacheable, but dcache-inhibit and guarded */
#define CONFIG_SYS_IBAT2L	(CONFIG_SYS_FLASH_BASE | BATL_PP_RW | \
					BATL_MEMCOHERENCE)
#define CONFIG_SYS_IBAT2U	(CONFIG_SYS_FLASH_BASE | BATU_BL_8M | \
					BATU_VS | BATU_VP)
#define CONFIG_SYS_DBAT2L	(CONFIG_SYS_FLASH_BASE | BATL_PP_RW | \
					BATL_CACHEINHIBIT | \
					BATL_GUARDEDSTORAGE)
#define CONFIG_SYS_DBAT2U	CONFIG_SYS_IBAT2U

/* Stack in dcache: cacheable, no memory coherence */
#define CONFIG_SYS_IBAT3L	(CONFIG_SYS_INIT_RAM_ADDR | BATL_PP_RW)
#define CONFIG_SYS_IBAT3U	(CONFIG_SYS_INIT_RAM_ADDR | BATU_BL_128K | \
					BATU_VS | BATU_VP)
#define CONFIG_SYS_DBAT3L	CONFIG_SYS_IBAT3L
#define CONFIG_SYS_DBAT3U	CONFIG_SYS_IBAT3U

/*
 * Environment Configuration
 */

#define CONFIG_ENV_OVERWRITE

#if defined(CONFIG_TSEC_ENET)
#define CONFIG_HAS_ETH0
#define CONFIG_HAS_ETH1
#endif

#define CONFIG_LOADADDR	800000	/* default location for tftp and bootm */


#define	CONFIG_EXTRA_ENV_SETTINGS					\
	"netdev=eth0\0"							\
	"consoledev=ttyS0\0"						\
	"nfsargs=setenv bootargs root=/dev/nfs rw "			\
		"nfsroot=${serverip}:${rootpath}\0"			\
	"ramargs=setenv bootargs root=/dev/ram rw\0"			\
	"addip=setenv bootargs ${bootargs} "				\
		"ip=${ipaddr}:${serverip}:${gatewayip}:${netmask}"	\
		":${hostname}:${netdev}:off panic=1\0"			\
	"addtty=setenv bootargs ${bootargs}"				\
		" console=${consoledev},${baudrate}\0"			\
	"addmtd=setenv bootargs ${bootargs} ${mtdparts}\0"		\
	"addmisc=setenv bootargs ${bootargs}\0"				\
	"kernel_addr=FE080000\0"					\
	"fdt_addr=FE280000\0"						\
	"ramdisk_addr=FE290000\0"					\
	"u-boot=mpc8308rdb/u-boot.bin\0"				\
	"kernel_addr_r=1000000\0"					\
	"fdt_addr_r=C00000\0"						\
	"hostname=mpc8308rdb\0"						\
	"bootfile=mpc8308rdb/uImage\0"					\
	"fdtfile=mpc8308rdb/mpc8308rdb.dtb\0"				\
	"rootpath=/opt/eldk-4.2/ppc_6xx\0"				\
	"flash_self=run ramargs addip addtty addmtd addmisc;"		\
		"bootm ${kernel_addr} ${ramdisk_addr} ${fdt_addr}\0"	\
	"flash_nfs=run nfsargs addip addtty addmtd addmisc;"		\
		"bootm ${kernel_addr} - ${fdt_addr}\0"			\
	"net_nfs=tftp ${kernel_addr_r} ${bootfile};"			\
		"tftp ${fdt_addr_r} ${fdtfile};"			\
		"run nfsargs addip addtty addmtd addmisc;"		\
		"bootm ${kernel_addr_r} - ${fdt_addr_r}\0"		\
	"bootcmd=run flash_self\0"					\
	"load=tftp ${loadaddr} ${u-boot}\0"				\
	"update=protect off " __stringify(CONFIG_SYS_MONITOR_BASE)	\
		" +${filesize};era " __stringify(CONFIG_SYS_MONITOR_BASE)\
		" +${filesize};cp.b ${fileaddr} "			\
		__stringify(CONFIG_SYS_MONITOR_BASE) " ${filesize}\0"	\
	"upd=run load update\0"						\

#endif	/* __CONFIG_H */
