// SPDX-License-Identifier: GPL-2.0
/*
 * TI PHY drivers
 *
 */
#include <common.h>
#include <phy.h>
#include <linux/compat.h>
#include <malloc.h>

#include <fdtdec.h>
#include <dm.h>
#include <dt-bindings/net/ti-dp83867.h>

DECLARE_GLOBAL_DATA_PTR;

/* TI DP83867 */
#define DP83867_DEVADDR		0x1f

#define MII_DP83867_PHYCTRL	0x10
#define MII_DP83867_MICR	0x12
#define MII_DP83867_CFG2	0x14
#define MII_DP83867_BISCR	0x16
#define DP83867_CTRL		0x1f

/* Extended Registers */
#define DP83867_RGMIICTL	0x0032
#define DP83867_RGMIIDCTL	0x0086
#define DP83867_IO_MUX_CFG	0x0170

#define DP83867_SW_RESET	BIT(15)
#define DP83867_SW_RESTART	BIT(14)

/* MICR Interrupt bits */
#define MII_DP83867_MICR_AN_ERR_INT_EN		BIT(15)
#define MII_DP83867_MICR_SPEED_CHNG_INT_EN	BIT(14)
#define MII_DP83867_MICR_DUP_MODE_CHNG_INT_EN	BIT(13)
#define MII_DP83867_MICR_PAGE_RXD_INT_EN	BIT(12)
#define MII_DP83867_MICR_AUTONEG_COMP_INT_EN	BIT(11)
#define MII_DP83867_MICR_LINK_STS_CHNG_INT_EN	BIT(10)
#define MII_DP83867_MICR_FALSE_CARRIER_INT_EN	BIT(8)
#define MII_DP83867_MICR_SLEEP_MODE_CHNG_INT_EN	BIT(4)
#define MII_DP83867_MICR_WOL_INT_EN		BIT(3)
#define MII_DP83867_MICR_XGMII_ERR_INT_EN	BIT(2)
#define MII_DP83867_MICR_POL_CHNG_INT_EN	BIT(1)
#define MII_DP83867_MICR_JABBER_INT_EN		BIT(0)

/* RGMIICTL bits */
#define DP83867_RGMII_TX_CLK_DELAY_EN		BIT(1)
#define DP83867_RGMII_RX_CLK_DELAY_EN		BIT(0)

/* PHY CTRL bits */
#define DP83867_PHYCR_FIFO_DEPTH_SHIFT		14
#define DP83867_MDI_CROSSOVER		5
#define DP83867_MDI_CROSSOVER_AUTO	2
#define DP83867_MDI_CROSSOVER_MDIX	2
#define DP83867_PHYCTRL_SGMIIEN			0x0800
#define DP83867_PHYCTRL_RXFIFO_SHIFT	12
#define DP83867_PHYCTRL_TXFIFO_SHIFT	14

/* RGMIIDCTL bits */
#define DP83867_RGMII_TX_CLK_DELAY_SHIFT	4

/* CFG2 bits */
#define MII_DP83867_CFG2_SPEEDOPT_10EN		0x0040
#define MII_DP83867_CFG2_SGMII_AUTONEGEN	0x0080
#define MII_DP83867_CFG2_SPEEDOPT_ENH		0x0100
#define MII_DP83867_CFG2_SPEEDOPT_CNT		0x0800
#define MII_DP83867_CFG2_SPEEDOPT_INTLOW	0x2000
#define MII_DP83867_CFG2_MASK			0x003F

#define MII_MMD_CTRL	0x0d /* MMD Access Control Register */
#define MII_MMD_DATA	0x0e /* MMD Access Data Register */

/* MMD Access Control register fields */
#define MII_MMD_CTRL_DEVAD_MASK	0x1f /* Mask MMD DEVAD*/
#define MII_MMD_CTRL_ADDR	0x0000 /* Address */
#define MII_MMD_CTRL_NOINCR	0x4000 /* no post increment */
#define MII_MMD_CTRL_INCR_RDWT	0x8000 /* post increment on reads & writes */
#define MII_MMD_CTRL_INCR_ON_WT	0xC000 /* post increment on writes only */

/* User setting - can be taken from DTS */
#define DEFAULT_RX_ID_DELAY	DP83867_RGMIIDCTL_2_25_NS
#define DEFAULT_TX_ID_DELAY	DP83867_RGMIIDCTL_2_75_NS
#define DEFAULT_FIFO_DEPTH	DP83867_PHYCR_FIFO_DEPTH_4_B_NIB

/* IO_MUX_CFG bits */
#define DP83867_IO_MUX_CFG_IO_IMPEDANCE_CTRL	0x1f

#define DP83867_IO_MUX_CFG_IO_IMPEDANCE_MAX	0x0
#define DP83867_IO_MUX_CFG_IO_IMPEDANCE_MIN	0x1f

struct dp83867_private {
	int rx_id_delay;
	int tx_id_delay;
	int fifo_depth;
	int io_impedance;
};

/**
 * phy_read_mmd_indirect - reads data from the MMD registers
 * @phydev: The PHY device bus
 * @prtad: MMD Address
 * @devad: MMD DEVAD
 * @addr: PHY address on the MII bus
 *
 * Description: it reads data from the MMD registers (clause 22 to access to
 * clause 45) of the specified phy address.
 * To read these registers we have:
 * 1) Write reg 13 // DEVAD
 * 2) Write reg 14 // MMD Address
 * 3) Write reg 13 // MMD Data Command for MMD DEVAD
 * 3) Read  reg 14 // Read MMD data
 */
int phy_read_mmd_indirect(struct phy_device *phydev, int prtad,
			  int devad, int addr)
{
	int value = -1;

	/* Write the desired MMD Devad */
	phy_write(phydev, addr, MII_MMD_CTRL, devad);

	/* Write the desired MMD register address */
	phy_write(phydev, addr, MII_MMD_DATA, prtad);

	/* Select the Function : DATA with no post increment */
	phy_write(phydev, addr, MII_MMD_CTRL, (devad | MII_MMD_CTRL_NOINCR));

	/* Read the content of the MMD's selected register */
	value = phy_read(phydev, addr, MII_MMD_DATA);
	return value;
}

/**
 * phy_write_mmd_indirect - writes data to the MMD registers
 * @phydev: The PHY device
 * @prtad: MMD Address
 * @devad: MMD DEVAD
 * @addr: PHY address on the MII bus
 * @data: data to write in the MMD register
 *
 * Description: Write data from the MMD registers of the specified
 * phy address.
 * To write these registers we have:
 * 1) Write reg 13 // DEVAD
 * 2) Write reg 14 // MMD Address
 * 3) Write reg 13 // MMD Data Command for MMD DEVAD
 * 3) Write reg 14 // Write MMD data
 */
void phy_write_mmd_indirect(struct phy_device *phydev, int prtad,
			    int devad, int addr, u32 data)
{
	/* Write the desired MMD Devad */
	phy_write(phydev, addr, MII_MMD_CTRL, devad);

	/* Write the desired MMD register address */
	phy_write(phydev, addr, MII_MMD_DATA, prtad);

	/* Select the Function : DATA with no post increment */
	phy_write(phydev, addr, MII_MMD_CTRL, (devad | MII_MMD_CTRL_NOINCR));

	/* Write the data into MMD's selected register */
	phy_write(phydev, addr, MII_MMD_DATA, data);
}

#if defined(CONFIG_DM_ETH)
/**
 * dp83867_data_init - Convenience function for setting PHY specific data
 *
 * @phydev: the phy_device struct
 */
static int dp83867_of_init(struct phy_device *phydev)
{
	struct dp83867_private *dp83867 = phydev->priv;
	struct udevice *dev = phydev->dev;
	int node = dev_of_offset(dev);
	const void *fdt = gd->fdt_blob;

	if (fdtdec_get_bool(fdt, node, "ti,max-output-impedance"))
		dp83867->io_impedance = DP83867_IO_MUX_CFG_IO_IMPEDANCE_MAX;
	else if (fdtdec_get_bool(fdt, node, "ti,min-output-impedance"))
		dp83867->io_impedance = DP83867_IO_MUX_CFG_IO_IMPEDANCE_MIN;
	else
		dp83867->io_impedance = -EINVAL;

	dp83867->rx_id_delay = fdtdec_get_uint(gd->fdt_blob, dev_of_offset(dev),
				 "ti,rx-internal-delay", -1);

	dp83867->tx_id_delay = fdtdec_get_uint(gd->fdt_blob, dev_of_offset(dev),
				 "ti,tx-internal-delay", -1);

	dp83867->fifo_depth = fdtdec_get_uint(gd->fdt_blob, dev_of_offset(dev),
				 "ti,fifo-depth", -1);

	return 0;
}
#else
static int dp83867_of_init(struct phy_device *phydev)
{
	struct dp83867_private *dp83867 = phydev->priv;

	dp83867->rx_id_delay = DEFAULT_RX_ID_DELAY;
	dp83867->tx_id_delay = DEFAULT_TX_ID_DELAY;
	dp83867->fifo_depth = DEFAULT_FIFO_DEPTH;
	dp83867->io_impedance = -EINVAL;

	return 0;
}
#endif

static int dp83867_config(struct phy_device *phydev)
{
	struct dp83867_private *dp83867;
	unsigned int val, delay, cfg2;
	int ret;

	if (!phydev->priv) {
		dp83867 = kzalloc(sizeof(*dp83867), GFP_KERNEL);
		if (!dp83867)
			return -ENOMEM;

		phydev->priv = dp83867;
		ret = dp83867_of_init(phydev);
		if (ret)
			goto err_out;
	} else {
		dp83867 = (struct dp83867_private *)phydev->priv;
	}

	/* Restart the PHY.  */
	val = phy_read(phydev, MDIO_DEVAD_NONE, DP83867_CTRL);
	phy_write(phydev, MDIO_DEVAD_NONE, DP83867_CTRL,
		  val | DP83867_SW_RESTART);

	if (phy_interface_is_rgmii(phydev)) {
		ret = phy_write(phydev, MDIO_DEVAD_NONE, MII_DP83867_PHYCTRL,
			(DP83867_MDI_CROSSOVER_AUTO << DP83867_MDI_CROSSOVER) |
			(dp83867->fifo_depth << DP83867_PHYCR_FIFO_DEPTH_SHIFT));
		if (ret)
			goto err_out;
	} else if (phy_interface_is_sgmii(phydev)) {
		phy_write(phydev, MDIO_DEVAD_NONE, MII_BMCR,
			  (BMCR_ANENABLE | BMCR_FULLDPLX | BMCR_SPEED1000));

		cfg2 = phy_read(phydev, phydev->addr, MII_DP83867_CFG2);
		cfg2 &= MII_DP83867_CFG2_MASK;
		cfg2 |= (MII_DP83867_CFG2_SPEEDOPT_10EN |
			 MII_DP83867_CFG2_SGMII_AUTONEGEN |
			 MII_DP83867_CFG2_SPEEDOPT_ENH |
			 MII_DP83867_CFG2_SPEEDOPT_CNT |
			 MII_DP83867_CFG2_SPEEDOPT_INTLOW);
		phy_write(phydev, MDIO_DEVAD_NONE, MII_DP83867_CFG2, cfg2);

		phy_write_mmd_indirect(phydev, DP83867_RGMIICTL,
				       DP83867_DEVADDR, phydev->addr, 0x0);

		phy_write(phydev, MDIO_DEVAD_NONE, MII_DP83867_PHYCTRL,
			  DP83867_PHYCTRL_SGMIIEN |
			  (DP83867_MDI_CROSSOVER_MDIX <<
			  DP83867_MDI_CROSSOVER) |
			  (dp83867->fifo_depth << DP83867_PHYCTRL_RXFIFO_SHIFT) |
			  (dp83867->fifo_depth << DP83867_PHYCTRL_TXFIFO_SHIFT));
		phy_write(phydev, MDIO_DEVAD_NONE, MII_DP83867_BISCR, 0x0);
	}

	if (phy_interface_is_rgmii(phydev)) {
		val = phy_read_mmd_indirect(phydev, DP83867_RGMIICTL,
					    DP83867_DEVADDR, phydev->addr);

		if (phydev->interface == PHY_INTERFACE_MODE_RGMII_ID)
			val |= (DP83867_RGMII_TX_CLK_DELAY_EN |
				DP83867_RGMII_RX_CLK_DELAY_EN);

		if (phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID)
			val |= DP83867_RGMII_TX_CLK_DELAY_EN;

		if (phydev->interface == PHY_INTERFACE_MODE_RGMII_RXID)
			val |= DP83867_RGMII_RX_CLK_DELAY_EN;

		phy_write_mmd_indirect(phydev, DP83867_RGMIICTL,
				       DP83867_DEVADDR, phydev->addr, val);

		delay = (dp83867->rx_id_delay |
			 (dp83867->tx_id_delay << DP83867_RGMII_TX_CLK_DELAY_SHIFT));

		phy_write_mmd_indirect(phydev, DP83867_RGMIIDCTL,
				       DP83867_DEVADDR, phydev->addr, delay);

		if (dp83867->io_impedance >= 0) {
			val = phy_read_mmd_indirect(phydev,
						    DP83867_IO_MUX_CFG,
						    DP83867_DEVADDR,
						    phydev->addr);
			val &= ~DP83867_IO_MUX_CFG_IO_IMPEDANCE_CTRL;
			val |= dp83867->io_impedance &
			       DP83867_IO_MUX_CFG_IO_IMPEDANCE_CTRL;
			phy_write_mmd_indirect(phydev, DP83867_IO_MUX_CFG,
					       DP83867_DEVADDR, phydev->addr,
					       val);
		}
	}

	genphy_config_aneg(phydev);
	return 0;

err_out:
	kfree(dp83867);
	return ret;
}

static struct phy_driver DP83867_driver = {
	.name = "TI DP83867",
	.uid = 0x2000a231,
	.mask = 0xfffffff0,
	.features = PHY_GBIT_FEATURES,
	.config = &dp83867_config,
	.startup = &genphy_startup,
	.shutdown = &genphy_shutdown,
};

int phy_ti_init(void)
{
	phy_register(&DP83867_driver);
	return 0;
}
