// SPDX-License-Identifier: GPL-2.0+
/*
 * SoC-specific code for TMS320DM646x chips
 */

#include <asm/arch/hardware.h>

void davinci_enable_uart0(void)
{
	lpsc_on(DAVINCI_DM646X_LPSC_UART0);
}

#ifdef CONFIG_DRIVER_TI_EMAC
void davinci_enable_emac(void)
{
	lpsc_on(DAVINCI_DM646X_LPSC_EMAC);
}
#endif

#ifdef CONFIG_SYS_I2C_DAVINCI
void davinci_enable_i2c(void)
{
	lpsc_on(DAVINCI_DM646X_LPSC_I2C);
}
#endif
