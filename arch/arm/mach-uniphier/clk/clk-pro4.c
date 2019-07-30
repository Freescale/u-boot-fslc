// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2011-2015 Panasonic Corporation
 * Copyright (C) 2015-2016 Socionext Inc.
 *   Author: Masahiro Yamada <yamada.masahiro@socionext.com>
 */

#include <linux/io.h>

#include "../init.h"
#include "../sc-regs.h"

void uniphier_pro4_clk_init(void)
{
	u32 tmp;

	/* deassert reset */
	tmp = readl(SC_RSTCTRL);
#ifdef CONFIG_USB_DWC3_UNIPHIER
	tmp |= SC_RSTCTRL_NRST_USB3B0 | SC_RSTCTRL_NRST_USB3C0 |
		SC_RSTCTRL_NRST_GIO;
#endif
#ifdef CONFIG_NAND_DENALI
	tmp |= SC_RSTCTRL_NRST_NAND;
#endif
	writel(tmp, SC_RSTCTRL);
	readl(SC_RSTCTRL); /* dummy read */

#ifdef CONFIG_USB_DWC3_UNIPHIER
	tmp = readl(SC_RSTCTRL2);
	tmp |= SC_RSTCTRL2_NRST_USB3B1 | SC_RSTCTRL2_NRST_USB3C1;
	writel(tmp, SC_RSTCTRL2);
	readl(SC_RSTCTRL2); /* dummy read */
#endif

	/* provide clocks */
	tmp = readl(SC_CLKCTRL);
#ifdef CONFIG_USB_DWC3_UNIPHIER
	tmp |= SC_CLKCTRL_CEN_USB31 | SC_CLKCTRL_CEN_USB30 |
		SC_CLKCTRL_CEN_GIO;
#endif
#ifdef CONFIG_USB_EHCI_HCD
	tmp |= SC_CLKCTRL_CEN_MIO | SC_CLKCTRL_CEN_STDMAC;
#endif
#ifdef CONFIG_NAND_DENALI
	tmp |= SC_CLKCTRL_CEN_NAND;
#endif
	writel(tmp, SC_CLKCTRL);
	readl(SC_CLKCTRL); /* dummy read */
}
