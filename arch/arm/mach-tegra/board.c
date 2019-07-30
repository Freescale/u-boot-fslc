// SPDX-License-Identifier: GPL-2.0+
/*
 *  (C) Copyright 2010-2015
 *  NVIDIA Corporation <www.nvidia.com>
 */

#include <common.h>
#include <dm.h>
#include <ns16550.h>
#include <spl.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/funcmux.h>
#include <asm/arch/mc.h>
#include <asm/arch/tegra.h>
#include <asm/arch-tegra/ap.h>
#include <asm/arch-tegra/board.h>
#include <asm/arch-tegra/pmc.h>
#include <asm/arch-tegra/sys_proto.h>
#include <asm/arch-tegra/warmboot.h>

void save_boot_params_ret(void);

DECLARE_GLOBAL_DATA_PTR;

enum {
	/* UARTs which we can enable */
	UARTA	= 1 << 0,
	UARTB	= 1 << 1,
	UARTC	= 1 << 2,
	UARTD	= 1 << 3,
	UARTE	= 1 << 4,
	UART_COUNT = 5,
};

static bool from_spl __attribute__ ((section(".data")));

#ifndef CONFIG_SPL_BUILD
void save_boot_params(u32 r0, u32 r1, u32 r2, u32 r3)
{
	from_spl = r0 != UBOOT_NOT_LOADED_FROM_SPL;
	save_boot_params_ret();
}
#endif

bool spl_was_boot_source(void)
{
	return from_spl;
}

#if defined(CONFIG_TEGRA_SUPPORT_NON_SECURE)
#if !defined(CONFIG_TEGRA124)
#error tegra_cpu_is_non_secure has only been validated on Tegra124
#endif
bool tegra_cpu_is_non_secure(void)
{
	/*
	 * This register reads 0xffffffff in non-secure mode. This register
	 * only implements bits 31:20, so the lower bits will always read 0 in
	 * secure mode. Thus, the lower bits are an indicator for secure vs.
	 * non-secure mode.
	 */
	struct mc_ctlr *mc = (struct mc_ctlr *)NV_PA_MC_BASE;
	uint32_t mc_s_cfg0 = readl(&mc->mc_security_cfg0);
	return (mc_s_cfg0 & 1) == 1;
}
#endif

/* Read the RAM size directly from the memory controller */
static phys_size_t query_sdram_size(void)
{
	struct mc_ctlr *const mc = (struct mc_ctlr *)NV_PA_MC_BASE;
	u32 emem_cfg;
	phys_size_t size_bytes;

	emem_cfg = readl(&mc->mc_emem_cfg);
#if defined(CONFIG_TEGRA20)
	debug("mc->mc_emem_cfg (MEM_SIZE_KB) = 0x%08x\n", emem_cfg);
	size_bytes = get_ram_size((void *)PHYS_SDRAM_1, emem_cfg * 1024);
#else
	debug("mc->mc_emem_cfg (MEM_SIZE_MB) = 0x%08x\n", emem_cfg);
#ifndef CONFIG_PHYS_64BIT
	/*
	 * If >=4GB RAM is present, the byte RAM size won't fit into 32-bits
	 * and will wrap. Clip the reported size to the maximum that a 32-bit
	 * variable can represent (rounded to a page).
	 */
	if (emem_cfg >= 4096) {
		size_bytes = U32_MAX & ~(0x1000 - 1);
	} else
#endif
	{
		/* RAM size EMC is programmed to. */
		size_bytes = (phys_size_t)emem_cfg * 1024 * 1024;
#ifndef CONFIG_ARM64
		/*
		 * If all RAM fits within 32-bits, it can be accessed without
		 * LPAE, so go test the RAM size. Otherwise, we can't access
		 * all the RAM, and get_ram_size() would get confused, so
		 * avoid using it. There's no reason we should need this
		 * validation step anyway.
		 */
		if (emem_cfg <= (0 - PHYS_SDRAM_1) / (1024 * 1024))
			size_bytes = get_ram_size((void *)PHYS_SDRAM_1,
						  size_bytes);
#endif
	}
#endif

#if defined(CONFIG_TEGRA30) || defined(CONFIG_TEGRA114)
	/* External memory limited to 2047 MB due to IROM/HI-VEC */
	if (size_bytes == SZ_2G)
		size_bytes -= SZ_1M;
#endif

	return size_bytes;
}

int dram_init(void)
{
	/* We do not initialise DRAM here. We just query the size */
	gd->ram_size = query_sdram_size();
	return 0;
}

static int uart_configs[] = {
#if defined(CONFIG_TEGRA20)
 #if defined(CONFIG_TEGRA_UARTA_UAA_UAB)
	FUNCMUX_UART1_UAA_UAB,
 #elif defined(CONFIG_TEGRA_UARTA_GPU)
	FUNCMUX_UART1_GPU,
 #elif defined(CONFIG_TEGRA_UARTA_SDIO1)
	FUNCMUX_UART1_SDIO1,
 #else
	FUNCMUX_UART1_IRRX_IRTX,
#endif
	FUNCMUX_UART2_UAD,
	-1,
	FUNCMUX_UART4_GMC,
	-1,
#elif defined(CONFIG_TEGRA30)
	FUNCMUX_UART1_ULPI,	/* UARTA */
	-1,
	-1,
	-1,
	-1,
#elif defined(CONFIG_TEGRA114)
	-1,
	-1,
	-1,
	FUNCMUX_UART4_GMI,	/* UARTD */
	-1,
#elif defined(CONFIG_TEGRA124)
	FUNCMUX_UART1_KBC,	/* UARTA */
	-1,
	-1,
	FUNCMUX_UART4_GPIO,	/* UARTD */
	-1,
#else	/* Tegra210 */
	FUNCMUX_UART1_UART1,	/* UARTA */
	-1,
	-1,
	FUNCMUX_UART4_UART4,	/* UARTD */
	-1,
#endif
};

/**
 * Set up the specified uarts
 *
 * @param uarts_ids	Mask containing UARTs to init (UARTx)
 */
static void setup_uarts(int uart_ids)
{
	static enum periph_id id_for_uart[] = {
		PERIPH_ID_UART1,
		PERIPH_ID_UART2,
		PERIPH_ID_UART3,
		PERIPH_ID_UART4,
		PERIPH_ID_UART5,
	};
	size_t i;

	for (i = 0; i < UART_COUNT; i++) {
		if (uart_ids & (1 << i)) {
			enum periph_id id = id_for_uart[i];

			funcmux_select(id, uart_configs[i]);
			clock_ll_start_uart(id);
		}
	}
}

void board_init_uart_f(void)
{
	int uart_ids = 0;	/* bit mask of which UART ids to enable */

#ifdef CONFIG_TEGRA_ENABLE_UARTA
	uart_ids |= UARTA;
#endif
#ifdef CONFIG_TEGRA_ENABLE_UARTB
	uart_ids |= UARTB;
#endif
#ifdef CONFIG_TEGRA_ENABLE_UARTC
	uart_ids |= UARTC;
#endif
#ifdef CONFIG_TEGRA_ENABLE_UARTD
	uart_ids |= UARTD;
#endif
#ifdef CONFIG_TEGRA_ENABLE_UARTE
	uart_ids |= UARTE;
#endif
	setup_uarts(uart_ids);
}

#if !CONFIG_IS_ENABLED(OF_CONTROL)
static struct ns16550_platdata ns16550_com1_pdata = {
	.base = CONFIG_SYS_NS16550_COM1,
	.reg_shift = 2,
	.clock = CONFIG_SYS_NS16550_CLK,
	.fcr = UART_FCR_DEFVAL,
};

U_BOOT_DEVICE(ns16550_com1) = {
	"ns16550_serial", &ns16550_com1_pdata
};
#endif

#if !defined(CONFIG_SYS_DCACHE_OFF) && !defined(CONFIG_ARM64)
void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif
