/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2013 Google, Inc
 *
 * (C) Copyright 2012
 * Pavel Herrmann <morpheus.ibis@gmail.com>
 */

#ifndef _DM_UCLASS_ID_H
#define _DM_UCLASS_ID_H

/* TODO(sjg@chromium.org): this could be compile-time generated */
enum uclass_id {
	/* These are used internally by driver model */
	UCLASS_ROOT = 0,
	UCLASS_DEMO,
	UCLASS_TEST,
	UCLASS_TEST_FDT,
	UCLASS_TEST_BUS,
	UCLASS_TEST_PROBE,
	UCLASS_TEST_DUMMY,
	UCLASS_SPI_EMUL,	/* sandbox SPI device emulator */
	UCLASS_I2C_EMUL,	/* sandbox I2C device emulator */
	UCLASS_PCI_EMUL,	/* sandbox PCI device emulator */
	UCLASS_USB_EMUL,	/* sandbox USB bus device emulator */
	UCLASS_SIMPLE_BUS,	/* bus with child devices */

	/* U-Boot uclasses start here - in alphabetical order */
	UCLASS_ADC,		/* Analog-to-digital converter */
	UCLASS_AHCI,		/* SATA disk controller */
	UCLASS_BLK,		/* Block device */
	UCLASS_CLK,		/* Clock source, e.g. used by peripherals */
	UCLASS_CPU,		/* CPU, typically part of an SoC */
	UCLASS_CROS_EC,		/* Chrome OS EC */
	UCLASS_DISPLAY,		/* Display (e.g. DisplayPort, HDMI) */
	UCLASS_DMA,		/* Direct Memory Access */
	UCLASS_EFI,		/* EFI managed devices */
	UCLASS_ETH,		/* Ethernet device */
	UCLASS_GPIO,		/* Bank of general-purpose I/O pins */
	UCLASS_FIRMWARE,	/* Firmware */
	UCLASS_I2C,		/* I2C bus */
	UCLASS_I2C_EEPROM,	/* I2C EEPROM device */
	UCLASS_I2C_GENERIC,	/* Generic I2C device */
	UCLASS_I2C_MUX,		/* I2C multiplexer */
	UCLASS_IDE,		/* IDE device */
	UCLASS_IRQ,		/* Interrupt controller */
	UCLASS_KEYBOARD,	/* Keyboard input device */
	UCLASS_LED,		/* Light-emitting diode (LED) */
	UCLASS_LPC,		/* x86 'low pin count' interface */
	UCLASS_MAILBOX,		/* Mailbox controller */
	UCLASS_MASS_STORAGE,	/* Mass storage device */
	UCLASS_MISC,		/* Miscellaneous device */
	UCLASS_MMC,		/* SD / MMC card or chip */
	UCLASS_MOD_EXP,		/* RSA Mod Exp device */
	UCLASS_MTD,		/* Memory Technology Device (MTD) device */
	UCLASS_NORTHBRIDGE,	/* Intel Northbridge / SDRAM controller */
	UCLASS_NVME,		/* NVM Express device */
	UCLASS_PANEL,		/* Display panel, such as an LCD */
	UCLASS_PANEL_BACKLIGHT,	/* Backlight controller for panel */
	UCLASS_PCH,		/* x86 platform controller hub */
	UCLASS_PCI,		/* PCI bus */
	UCLASS_PCI_GENERIC,	/* Generic PCI bus device */
	UCLASS_PHY,		/* Physical Layer (PHY) device */
	UCLASS_PINCONFIG,	/* Pin configuration node device */
	UCLASS_PINCTRL,		/* Pinctrl (pin muxing/configuration) device */
	UCLASS_PMIC,		/* PMIC I/O device */
	UCLASS_PWM,		/* Pulse-width modulator */
	UCLASS_POWER_DOMAIN,	/* (SoC) Power domains */
	UCLASS_PWRSEQ,		/* Power sequence device */
	UCLASS_RAM,		/* RAM controller */
	UCLASS_REGULATOR,	/* Regulator device */
	UCLASS_REMOTEPROC,	/* Remote Processor device */
	UCLASS_RESET,		/* Reset controller device */
	UCLASS_RTC,		/* Real time clock device */
	UCLASS_SCSI,		/* SCSI device */
	UCLASS_SERIAL,		/* Serial UART */
	UCLASS_SPI,		/* SPI bus */
	UCLASS_SPMI,		/* System Power Management Interface bus */
	UCLASS_SPI_FLASH,	/* SPI flash */
	UCLASS_SPI_GENERIC,	/* Generic SPI flash target */
	UCLASS_SYSCON,		/* System configuration device */
	UCLASS_SYSRESET,	/* System reset device */
	UCLASS_THERMAL,		/* Thermal sensor */
	UCLASS_TIMER,		/* Timer device */
	UCLASS_TPM,		/* Trusted Platform Module TIS interface */
	UCLASS_USB,		/* USB bus */
	UCLASS_USB_DEV_GENERIC,	/* USB generic device */
	UCLASS_USB_HUB,		/* USB hub */
	UCLASS_VIDEO,		/* Video or LCD device */
	UCLASS_VIDEO_BRIDGE,	/* Video bridge, e.g. DisplayPort to LVDS */
	UCLASS_VIDEO_CONSOLE,	/* Text console driver for video device */
	UCLASS_WDT,		/* Watchdot Timer driver */

	UCLASS_COUNT,
	UCLASS_INVALID = -1,
};

#endif
