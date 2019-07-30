// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016 Google, Inc
 */

#include <common.h>
#include <debug_uart.h>
#include <spl.h>
#include <asm/cpu.h>
#include <asm/mtrr.h>
#include <asm/processor.h>
#include <asm-generic/sections.h>

DECLARE_GLOBAL_DATA_PTR;

__weak int arch_cpu_init_dm(void)
{
	return 0;
}

static int x86_spl_init(void)
{
	/*
	 * TODO(sjg@chromium.org): We use this area of RAM for the stack
	 * and global_data in SPL. Once U-Boot starts up and releocates it
	 * is not needed. We could make this a CONFIG option or perhaps
	 * place it immediately below CONFIG_SYS_TEXT_BASE.
	 */
	char *ptr = (char *)0x110000;
	int ret;

	debug("%s starting\n", __func__);
	ret = spl_init();
	if (ret) {
		debug("%s: spl_init() failed\n", __func__);
		return ret;
	}
	ret = arch_cpu_init();
	if (ret) {
		debug("%s: arch_cpu_init() failed\n", __func__);
		return ret;
	}
	ret = arch_cpu_init_dm();
	if (ret) {
		debug("%s: arch_cpu_init_dm() failed\n", __func__);
		return ret;
	}
	preloader_console_init();
	ret = print_cpuinfo();
	if (ret) {
		debug("%s: print_cpuinfo() failed\n", __func__);
		return ret;
	}
	ret = dram_init();
	if (ret) {
		debug("%s: dram_init() failed\n", __func__);
		return ret;
	}
	memset(&__bss_start, 0, (ulong)&__bss_end - (ulong)&__bss_start);

	/* TODO(sjg@chromium.org): Consider calling cpu_init_r() here */
	ret = interrupt_init();
	if (ret) {
		debug("%s: interrupt_init() failed\n", __func__);
		return ret;
	}

	/*
	 * The stack grows down from ptr. Put the global data at ptr. This
	 * will only be used for SPL. Once SPL loads U-Boot proper it will
	 * set up its own stack.
	 */
	gd->new_gd = (struct global_data *)ptr;
	memcpy(gd->new_gd, gd, sizeof(*gd));
	arch_setup_gd(gd->new_gd);
	gd->start_addr_sp = (ulong)ptr;

	/* Cache the SPI flash. Otherwise copying the code to RAM takes ages */
	ret = mtrr_add_request(MTRR_TYPE_WRBACK,
			       (1ULL << 32) - CONFIG_XIP_ROM_SIZE,
			       CONFIG_XIP_ROM_SIZE);
	if (ret) {
		debug("%s: SPI cache setup failed\n", __func__);
		return ret;
	}

	return 0;
}

void board_init_f(ulong flags)
{
	int ret;

	ret = x86_spl_init();
	if (ret) {
		debug("Error %d\n", ret);
		hang();
	}

	/* Uninit CAR and jump to board_init_f_r() */
	board_init_f_r_trampoline(gd->start_addr_sp);
}

void board_init_f_r(void)
{
	init_cache_f_r();
	gd->flags &= ~GD_FLG_SERIAL_READY;
	debug("cache status %d\n", dcache_status());
	board_init_r(gd, 0);
}

u32 spl_boot_device(void)
{
	return BOOT_DEVICE_BOARD;
}

int spl_start_uboot(void)
{
	return 0;
}

void spl_board_announce_boot_device(void)
{
	printf("SPI flash");
}

static int spl_board_load_image(struct spl_image_info *spl_image,
				struct spl_boot_device *bootdev)
{
	spl_image->size = CONFIG_SYS_MONITOR_LEN;
	spl_image->entry_point = CONFIG_SYS_TEXT_BASE;
	spl_image->load_addr = CONFIG_SYS_TEXT_BASE;
	spl_image->os = IH_OS_U_BOOT;
	spl_image->name = "U-Boot";

	debug("Loading to %lx\n", spl_image->load_addr);

	return 0;
}
SPL_LOAD_IMAGE_METHOD("SPI", 0, BOOT_DEVICE_BOARD, spl_board_load_image);

int spl_spi_load_image(void)
{
	return -EPERM;
}

void __noreturn jump_to_image_no_args(struct spl_image_info *spl_image)
{
	int ret;

	printf("Jumping to 64-bit U-Boot: Note many features are missing\n");
	ret = cpu_jump_to_64bit_uboot(spl_image->entry_point);
	debug("ret=%d\n", ret);
	while (1)
		;
}
