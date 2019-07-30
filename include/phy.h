/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2011 Freescale Semiconductor, Inc.
 *	Andy Fleming <afleming@gmail.com>
 *
 * This file pretty much stolen from Linux's mii.h/ethtool.h/phy.h
 */

#ifndef _PHY_H
#define _PHY_H

#include <linux/list.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/mdio.h>

#define PHY_FIXED_ID		0xa5a55a5a

#define PHY_MAX_ADDR 32

#define PHY_FLAG_BROKEN_RESET	(1 << 0) /* soft reset not supported */

#define PHY_DEFAULT_FEATURES	(SUPPORTED_Autoneg | \
				 SUPPORTED_TP | \
				 SUPPORTED_MII)

#define PHY_10BT_FEATURES	(SUPPORTED_10baseT_Half | \
				 SUPPORTED_10baseT_Full)

#define PHY_100BT_FEATURES	(SUPPORTED_100baseT_Half | \
				 SUPPORTED_100baseT_Full)

#define PHY_1000BT_FEATURES	(SUPPORTED_1000baseT_Half | \
				 SUPPORTED_1000baseT_Full)

#define PHY_BASIC_FEATURES	(PHY_10BT_FEATURES | \
				 PHY_100BT_FEATURES | \
				 PHY_DEFAULT_FEATURES)

#define PHY_GBIT_FEATURES	(PHY_BASIC_FEATURES | \
				 PHY_1000BT_FEATURES)

#define PHY_10G_FEATURES	(PHY_GBIT_FEATURES | \
				SUPPORTED_10000baseT_Full)

#ifndef PHY_ANEG_TIMEOUT
#define PHY_ANEG_TIMEOUT	4000
#endif


typedef enum {
	PHY_INTERFACE_MODE_MII,
	PHY_INTERFACE_MODE_GMII,
	PHY_INTERFACE_MODE_SGMII,
	PHY_INTERFACE_MODE_SGMII_2500,
	PHY_INTERFACE_MODE_QSGMII,
	PHY_INTERFACE_MODE_TBI,
	PHY_INTERFACE_MODE_RMII,
	PHY_INTERFACE_MODE_RGMII,
	PHY_INTERFACE_MODE_RGMII_ID,
	PHY_INTERFACE_MODE_RGMII_RXID,
	PHY_INTERFACE_MODE_RGMII_TXID,
	PHY_INTERFACE_MODE_RTBI,
	PHY_INTERFACE_MODE_XGMII,
	PHY_INTERFACE_MODE_XAUI,
	PHY_INTERFACE_MODE_RXAUI,
	PHY_INTERFACE_MODE_SFI,
	PHY_INTERFACE_MODE_NONE,	/* Must be last */

	PHY_INTERFACE_MODE_COUNT,
} phy_interface_t;

static const char *phy_interface_strings[] = {
	[PHY_INTERFACE_MODE_MII]		= "mii",
	[PHY_INTERFACE_MODE_GMII]		= "gmii",
	[PHY_INTERFACE_MODE_SGMII]		= "sgmii",
	[PHY_INTERFACE_MODE_SGMII_2500]		= "sgmii-2500",
	[PHY_INTERFACE_MODE_QSGMII]		= "qsgmii",
	[PHY_INTERFACE_MODE_TBI]		= "tbi",
	[PHY_INTERFACE_MODE_RMII]		= "rmii",
	[PHY_INTERFACE_MODE_RGMII]		= "rgmii",
	[PHY_INTERFACE_MODE_RGMII_ID]		= "rgmii-id",
	[PHY_INTERFACE_MODE_RGMII_RXID]		= "rgmii-rxid",
	[PHY_INTERFACE_MODE_RGMII_TXID]		= "rgmii-txid",
	[PHY_INTERFACE_MODE_RTBI]		= "rtbi",
	[PHY_INTERFACE_MODE_XGMII]		= "xgmii",
	[PHY_INTERFACE_MODE_XAUI]		= "xaui",
	[PHY_INTERFACE_MODE_RXAUI]		= "rxaui",
	[PHY_INTERFACE_MODE_SFI]		= "sfi",
	[PHY_INTERFACE_MODE_NONE]		= "",
};

static inline const char *phy_string_for_interface(phy_interface_t i)
{
	/* Default to unknown */
	if (i > PHY_INTERFACE_MODE_NONE)
		i = PHY_INTERFACE_MODE_NONE;

	return phy_interface_strings[i];
}


struct phy_device;

#define MDIO_NAME_LEN 32

struct mii_dev {
	struct list_head link;
	char name[MDIO_NAME_LEN];
	void *priv;
	int (*read)(struct mii_dev *bus, int addr, int devad, int reg);
	int (*write)(struct mii_dev *bus, int addr, int devad, int reg,
			u16 val);
	int (*reset)(struct mii_dev *bus);
	struct phy_device *phymap[PHY_MAX_ADDR];
	u32 phy_mask;
};

/* struct phy_driver: a structure which defines PHY behavior
 *
 * uid will contain a number which represents the PHY.  During
 * startup, the driver will poll the PHY to find out what its
 * UID--as defined by registers 2 and 3--is.  The 32-bit result
 * gotten from the PHY will be masked to
 * discard any bits which may change based on revision numbers
 * unimportant to functionality
 *
 */
struct phy_driver {
	char *name;
	unsigned int uid;
	unsigned int mask;
	unsigned int mmds;

	u32 features;

	/* Called to do any driver startup necessities */
	/* Will be called during phy_connect */
	int (*probe)(struct phy_device *phydev);

	/* Called to configure the PHY, and modify the controller
	 * based on the results.  Should be called after phy_connect */
	int (*config)(struct phy_device *phydev);

	/* Called when starting up the controller */
	int (*startup)(struct phy_device *phydev);

	/* Called when bringing down the controller */
	int (*shutdown)(struct phy_device *phydev);

	int (*readext)(struct phy_device *phydev, int addr, int devad, int reg);
	int (*writeext)(struct phy_device *phydev, int addr, int devad, int reg,
			u16 val);
	struct list_head list;
};

struct phy_device {
	/* Information about the PHY type */
	/* And management functions */
	struct mii_dev *bus;
	struct phy_driver *drv;
	void *priv;

#ifdef CONFIG_DM_ETH
	struct udevice *dev;
#else
	struct eth_device *dev;
#endif

	/* forced speed & duplex (no autoneg)
	 * partner speed & duplex & pause (autoneg)
	 */
	int speed;
	int duplex;

	/* The most recently read link state */
	int link;
	int port;
	phy_interface_t interface;

	u32 advertising;
	u32 supported;
	u32 mmds;

	int autoneg;
	int addr;
	int pause;
	int asym_pause;
	u32 phy_id;
	u32 flags;
};

struct fixed_link {
	int phy_id;
	int duplex;
	int link_speed;
	int pause;
	int asym_pause;
};

static inline int phy_read(struct phy_device *phydev, int devad, int regnum)
{
	struct mii_dev *bus = phydev->bus;

	return bus->read(bus, phydev->addr, devad, regnum);
}

static inline int phy_write(struct phy_device *phydev, int devad, int regnum,
			u16 val)
{
	struct mii_dev *bus = phydev->bus;

	return bus->write(bus, phydev->addr, devad, regnum, val);
}

#ifdef CONFIG_PHYLIB_10G
extern struct phy_driver gen10g_driver;

/* For now, XGMII is the only 10G interface */
static inline int is_10g_interface(phy_interface_t interface)
{
	return interface == PHY_INTERFACE_MODE_XGMII;
}

#endif

int phy_init(void);
int phy_reset(struct phy_device *phydev);
struct phy_device *phy_find_by_mask(struct mii_dev *bus, unsigned phy_mask,
		phy_interface_t interface);
#ifdef CONFIG_DM_ETH
void phy_connect_dev(struct phy_device *phydev, struct udevice *dev);
struct phy_device *phy_connect(struct mii_dev *bus, int addr,
				struct udevice *dev,
				phy_interface_t interface);
#else
void phy_connect_dev(struct phy_device *phydev, struct eth_device *dev);
struct phy_device *phy_connect(struct mii_dev *bus, int addr,
				struct eth_device *dev,
				phy_interface_t interface);
#endif
int phy_startup(struct phy_device *phydev);
int phy_config(struct phy_device *phydev);
int phy_shutdown(struct phy_device *phydev);
int phy_register(struct phy_driver *drv);
int phy_set_supported(struct phy_device *phydev, u32 max_speed);
int genphy_config_aneg(struct phy_device *phydev);
int genphy_restart_aneg(struct phy_device *phydev);
int genphy_update_link(struct phy_device *phydev);
int genphy_parse_link(struct phy_device *phydev);
int genphy_config(struct phy_device *phydev);
int genphy_startup(struct phy_device *phydev);
int genphy_shutdown(struct phy_device *phydev);
int gen10g_config(struct phy_device *phydev);
int gen10g_startup(struct phy_device *phydev);
int gen10g_shutdown(struct phy_device *phydev);
int gen10g_discover_mmds(struct phy_device *phydev);

int phy_b53_init(void);
int phy_mv88e61xx_init(void);
int phy_aquantia_init(void);
int phy_atheros_init(void);
int phy_broadcom_init(void);
int phy_cortina_init(void);
int phy_davicom_init(void);
int phy_et1011c_init(void);
int phy_lxt_init(void);
int phy_marvell_init(void);
int phy_micrel_ksz8xxx_init(void);
int phy_micrel_ksz90x1_init(void);
int phy_meson_gxl_init(void);
int phy_natsemi_init(void);
int phy_realtek_init(void);
int phy_smsc_init(void);
int phy_teranetics_init(void);
int phy_ti_init(void);
int phy_vitesse_init(void);
int phy_xilinx_init(void);
int phy_mscc_init(void);
int phy_fixed_init(void);

int board_phy_config(struct phy_device *phydev);
int get_phy_id(struct mii_dev *bus, int addr, int devad, u32 *phy_id);

/**
 * phy_get_interface_by_name() - Look up a PHY interface name
 *
 * @str:	PHY interface name, e.g. "mii"
 * @return PHY_INTERFACE_MODE_... value, or -1 if not found
 */
int phy_get_interface_by_name(const char *str);

/**
 * phy_interface_is_rgmii - Convenience function for testing if a PHY interface
 * is RGMII (all variants)
 * @phydev: the phy_device struct
 */
static inline bool phy_interface_is_rgmii(struct phy_device *phydev)
{
	return phydev->interface >= PHY_INTERFACE_MODE_RGMII &&
		phydev->interface <= PHY_INTERFACE_MODE_RGMII_TXID;
}

/**
 * phy_interface_is_sgmii - Convenience function for testing if a PHY interface
 * is SGMII (all variants)
 * @phydev: the phy_device struct
 */
static inline bool phy_interface_is_sgmii(struct phy_device *phydev)
{
	return phydev->interface >= PHY_INTERFACE_MODE_SGMII &&
		phydev->interface <= PHY_INTERFACE_MODE_QSGMII;
}

/* PHY UIDs for various PHYs that are referenced in external code */
#define PHY_UID_CS4340  0x13e51002
#define PHY_UID_TN2020	0x00a19410

#endif
