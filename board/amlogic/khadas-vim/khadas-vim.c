// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

#include <common.h>
#include <dm.h>
#include <environment.h>
#include <asm/io.h>
#include <asm/arch/gxbb.h>
#include <asm/arch/mem.h>
#include <asm/arch/sm.h>
#include <asm/arch/eth.h>

#define EFUSE_SN_OFFSET		20
#define EFUSE_SN_SIZE		16
#define EFUSE_MAC_OFFSET	52
#define EFUSE_MAC_SIZE		6

int board_init(void)
{
	return 0;
}

int misc_init_r(void)
{
	u8 mac_addr[EFUSE_MAC_SIZE];
	char serial[EFUSE_SN_SIZE];
	ssize_t len;

	meson_gx_eth_init(PHY_INTERFACE_MODE_RMII,
			  MESON_GXL_USE_INTERNAL_RMII_PHY);

	if (!eth_env_get_enetaddr("ethaddr", mac_addr)) {
		len = meson_sm_read_efuse(EFUSE_MAC_OFFSET,
					  mac_addr, EFUSE_MAC_SIZE);
		if (len == EFUSE_MAC_SIZE && is_valid_ethaddr(mac_addr))
			eth_env_set_enetaddr("ethaddr", mac_addr);
	}

	if (!env_get("serial#")) {
		len = meson_sm_read_efuse(EFUSE_SN_OFFSET, serial,
					  EFUSE_SN_SIZE);
		if (len == EFUSE_SN_SIZE)
			env_set("serial#", serial);
	}

	return 0;
}

int ft_board_setup(void *blob, bd_t *bd)
{
	meson_gx_init_reserved_memory(blob);

	return 0;
}
