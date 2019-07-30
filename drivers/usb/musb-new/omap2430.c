// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005-2007 by Texas Instruments
 * Some code has been taken from tusb6010.c
 * Copyrights for that are attributable to:
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * This file is part of the Inventra Controller Driver for Linux.
 */
#ifndef __UBOOT__
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/err.h>
#include <linux/usb/musb-omap.h>
#else
#include <common.h>
#include <asm/omap_common.h>
#include <asm/omap_musb.h>
#include <twl4030.h>
#include <twl6030.h>
#include "linux-compat.h"
#endif

#include "musb_core.h"
#include "omap2430.h"

#ifndef __UBOOT__
struct omap2430_glue {
	struct device		*dev;
	struct platform_device	*musb;
	enum omap_musb_vbus_id_status status;
	struct work_struct	omap_musb_mailbox_work;
};
#define glue_to_musb(g)		platform_get_drvdata(g->musb)

struct omap2430_glue		*_glue;

static struct timer_list musb_idle_timer;

static void musb_do_idle(unsigned long _musb)
{
	struct musb	*musb = (void *)_musb;
	unsigned long	flags;
	u8	power;
	u8	devctl;

	spin_lock_irqsave(&musb->lock, flags);

	switch (musb->xceiv->state) {
	case OTG_STATE_A_WAIT_BCON:

		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_BDEVICE) {
			musb->xceiv->state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
		} else {
			musb->xceiv->state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(musb);
		}
		break;
	case OTG_STATE_A_SUSPEND:
		/* finish RESUME signaling? */
		if (musb->port1_status & MUSB_PORT_STAT_RESUME) {
			power = musb_readb(musb->mregs, MUSB_POWER);
			power &= ~MUSB_POWER_RESUME;
			dev_dbg(musb->controller, "root port resume stopped, power %02x\n", power);
			musb_writeb(musb->mregs, MUSB_POWER, power);
			musb->is_active = 1;
			musb->port1_status &= ~(USB_PORT_STAT_SUSPEND
						| MUSB_PORT_STAT_RESUME);
			musb->port1_status |= USB_PORT_STAT_C_SUSPEND << 16;
			usb_hcd_poll_rh_status(musb_to_hcd(musb));
			/* NOTE: it might really be A_WAIT_BCON ... */
			musb->xceiv->state = OTG_STATE_A_HOST;
		}
		break;
	case OTG_STATE_A_HOST:
		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl &  MUSB_DEVCTL_BDEVICE)
			musb->xceiv->state = OTG_STATE_B_IDLE;
		else
			musb->xceiv->state = OTG_STATE_A_WAIT_BCON;
	default:
		break;
	}
	spin_unlock_irqrestore(&musb->lock, flags);
}


static void omap2430_musb_try_idle(struct musb *musb, unsigned long timeout)
{
	unsigned long		default_timeout = jiffies + msecs_to_jiffies(3);
	static unsigned long	last_timer;

	if (timeout == 0)
		timeout = default_timeout;

	/* Never idle if active, or when VBUS timeout is not set as host */
	if (musb->is_active || ((musb->a_wait_bcon == 0)
			&& (musb->xceiv->state == OTG_STATE_A_WAIT_BCON))) {
		dev_dbg(musb->controller, "%s active, deleting timer\n",
			otg_state_string(musb->xceiv->state));
		del_timer(&musb_idle_timer);
		last_timer = jiffies;
		return;
	}

	if (time_after(last_timer, timeout)) {
		if (!timer_pending(&musb_idle_timer))
			last_timer = timeout;
		else {
			dev_dbg(musb->controller, "Longer idle timer already pending, ignoring\n");
			return;
		}
	}
	last_timer = timeout;

	dev_dbg(musb->controller, "%s inactive, for idle timer for %lu ms\n",
		otg_state_string(musb->xceiv->state),
		(unsigned long)jiffies_to_msecs(timeout - jiffies));
	mod_timer(&musb_idle_timer, timeout);
}

static void omap2430_musb_set_vbus(struct musb *musb, int is_on)
{
	struct usb_otg	*otg = musb->xceiv->otg;
	u8		devctl;
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	int ret = 1;
	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	if (is_on) {
		if (musb->xceiv->state == OTG_STATE_A_IDLE) {
			/* start the session */
			devctl |= MUSB_DEVCTL_SESSION;
			musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
			/*
			 * Wait for the musb to set as A device to enable the
			 * VBUS
			 */
			while (musb_readb(musb->mregs, MUSB_DEVCTL) & 0x80) {

				cpu_relax();

				if (time_after(jiffies, timeout)) {
					dev_err(musb->controller,
					"configured as A device timeout");
					ret = -EINVAL;
					break;
				}
			}

			if (ret && otg->set_vbus)
				otg_set_vbus(otg, 1);
		} else {
			musb->is_active = 1;
			otg->default_a = 1;
			musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
			devctl |= MUSB_DEVCTL_SESSION;
			MUSB_HST_MODE(musb);
		}
	} else {
		musb->is_active = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		otg->default_a = 0;
		musb->xceiv->state = OTG_STATE_B_IDLE;
		devctl &= ~MUSB_DEVCTL_SESSION;

		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	dev_dbg(musb->controller, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		otg_state_string(musb->xceiv->state),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}

static int omap2430_musb_set_mode(struct musb *musb, u8 musb_mode)
{
	u8	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	devctl |= MUSB_DEVCTL_SESSION;
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	return 0;
}
#endif

static inline void omap2430_low_level_exit(struct musb *musb)
{
	u32 l;

	/* in any role */
	l = musb_readl(musb->mregs, OTG_FORCESTDBY);
	l |= ENABLEFORCE;	/* enable MSTANDBY */
	musb_writel(musb->mregs, OTG_FORCESTDBY, l);
}

static inline void omap2430_low_level_init(struct musb *musb)
{
	u32 l;

	l = musb_readl(musb->mregs, OTG_FORCESTDBY);
	l &= ~ENABLEFORCE;	/* disable MSTANDBY */
	musb_writel(musb->mregs, OTG_FORCESTDBY, l);
}

#ifndef __UBOOT__
void omap_musb_mailbox(enum omap_musb_vbus_id_status status)
{
	struct omap2430_glue	*glue = _glue;
	struct musb		*musb = glue_to_musb(glue);

	glue->status = status;
	if (!musb) {
		dev_err(glue->dev, "musb core is not yet ready\n");
		return;
	}

	schedule_work(&glue->omap_musb_mailbox_work);
}
EXPORT_SYMBOL_GPL(omap_musb_mailbox);

static void omap_musb_set_mailbox(struct omap2430_glue *glue)
{
	struct musb *musb = glue_to_musb(glue);
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *pdata = dev->platform_data;
	struct omap_musb_board_data *data = pdata->board_data;
	struct usb_otg *otg = musb->xceiv->otg;

	switch (glue->status) {
	case OMAP_MUSB_ID_GROUND:
		dev_dbg(dev, "ID GND\n");

		otg->default_a = true;
		musb->xceiv->state = OTG_STATE_A_IDLE;
		musb->xceiv->last_event = USB_EVENT_ID;
		if (!is_otg_enabled(musb) || musb->gadget_driver) {
			pm_runtime_get_sync(dev);
			usb_phy_init(musb->xceiv);
			omap2430_musb_set_vbus(musb, 1);
		}
		break;

	case OMAP_MUSB_VBUS_VALID:
		dev_dbg(dev, "VBUS Connect\n");

		otg->default_a = false;
		musb->xceiv->state = OTG_STATE_B_IDLE;
		musb->xceiv->last_event = USB_EVENT_VBUS;
		if (musb->gadget_driver)
			pm_runtime_get_sync(dev);
		usb_phy_init(musb->xceiv);
		break;

	case OMAP_MUSB_ID_FLOAT:
	case OMAP_MUSB_VBUS_OFF:
		dev_dbg(dev, "VBUS Disconnect\n");

		musb->xceiv->last_event = USB_EVENT_NONE;
		if (is_otg_enabled(musb) || is_peripheral_enabled(musb))
			if (musb->gadget_driver) {
				pm_runtime_mark_last_busy(dev);
				pm_runtime_put_autosuspend(dev);
			}

		if (data->interface_type == MUSB_INTERFACE_UTMI) {
			if (musb->xceiv->otg->set_vbus)
				otg_set_vbus(musb->xceiv->otg, 0);
		}
		usb_phy_shutdown(musb->xceiv);
		break;
	default:
		dev_dbg(dev, "ID float\n");
	}
}


static void omap_musb_mailbox_work(struct work_struct *mailbox_work)
{
	struct omap2430_glue *glue = container_of(mailbox_work,
				struct omap2430_glue, omap_musb_mailbox_work);
	omap_musb_set_mailbox(glue);
}
#endif

static int omap2430_musb_init(struct musb *musb)
{
	u32 l;
	int status = 0;
	unsigned long int start;
#ifndef __UBOOT__
	struct device *dev = musb->controller;
	struct omap2430_glue *glue = dev_get_drvdata(dev->parent);
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
#else
	struct omap_musb_board_data *data =
		(struct omap_musb_board_data *)musb->controller;
#endif

	/* Reset the controller */
	musb_writel(musb->mregs, OTG_SYSCONFIG, SOFTRST);

	start = get_timer(0);

	while (1) {
		l = musb_readl(musb->mregs, OTG_SYSCONFIG);
		if ((l & SOFTRST) == 0)
			break;

		if (get_timer(start) > (CONFIG_SYS_HZ / 1000)) {
			dev_err(musb->controller, "MUSB reset is taking too long\n");
			return -ENODEV;
		}
	}

#ifndef __UBOOT__
	/* We require some kind of external transceiver, hooked
	 * up through ULPI.  TWL4030-family PMICs include one,
	 * which needs a driver, drivers aren't always needed.
	 */
	musb->xceiv = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (IS_ERR_OR_NULL(musb->xceiv)) {
		pr_err("HS USB OTG: no transceiver configured\n");
		return -ENODEV;
	}

	status = pm_runtime_get_sync(dev);
	if (status < 0) {
		dev_err(dev, "pm_runtime_get_sync FAILED %d\n", status);
		goto err1;
	}
#endif

	l = musb_readl(musb->mregs, OTG_INTERFSEL);

	if (data->interface_type == MUSB_INTERFACE_UTMI) {
		/* OMAP4 uses Internal PHY GS70 which uses UTMI interface */
		l &= ~ULPI_12PIN;       /* Disable ULPI */
		l |= UTMI_8BIT;         /* Enable UTMI  */
	} else {
		l |= ULPI_12PIN;
	}

	musb_writel(musb->mregs, OTG_INTERFSEL, l);

	pr_debug("HS USB OTG: revision 0x%x, sysconfig 0x%02x, "
			"sysstatus 0x%x, intrfsel 0x%x, simenable  0x%x\n",
			musb_readl(musb->mregs, OTG_REVISION),
			musb_readl(musb->mregs, OTG_SYSCONFIG),
			musb_readl(musb->mregs, OTG_SYSSTATUS),
			musb_readl(musb->mregs, OTG_INTERFSEL),
			musb_readl(musb->mregs, OTG_SIMENABLE));

#ifndef __UBOOT__
	setup_timer(&musb_idle_timer, musb_do_idle, (unsigned long) musb);

	if (glue->status != OMAP_MUSB_UNKNOWN)
		omap_musb_set_mailbox(glue);

	pm_runtime_put_noidle(musb->controller);
#endif
	return 0;

err1:
	return status;
}

#ifndef __UBOOT__
static void omap2430_musb_enable(struct musb *musb)
#else
static int omap2430_musb_enable(struct musb *musb)
#endif
{
#ifndef __UBOOT__
	u8		devctl;
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	struct device *dev = musb->controller;
	struct omap2430_glue *glue = dev_get_drvdata(dev->parent);
	struct musb_hdrc_platform_data *pdata = dev->platform_data;
	struct omap_musb_board_data *data = pdata->board_data;

	switch (glue->status) {

	case OMAP_MUSB_ID_GROUND:
		usb_phy_init(musb->xceiv);
		if (data->interface_type != MUSB_INTERFACE_UTMI)
			break;
		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		/* start the session */
		devctl |= MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
		while (musb_readb(musb->mregs, MUSB_DEVCTL) &
				MUSB_DEVCTL_BDEVICE) {
			cpu_relax();

			if (time_after(jiffies, timeout)) {
				dev_err(dev, "configured as A device timeout");
				break;
			}
		}
		break;

	case OMAP_MUSB_VBUS_VALID:
		usb_phy_init(musb->xceiv);
		break;

	default:
		break;
	}
#else
#ifdef CONFIG_TWL4030_USB
	if (twl4030_usb_ulpi_init()) {
		serial_printf("ERROR: %s Could not initialize PHY\n",
				__PRETTY_FUNCTION__);
	}
#endif

#ifdef CONFIG_TWL6030_POWER
	twl6030_usb_device_settings();
#endif

#ifdef CONFIG_OMAP44XX
	u32 *usbotghs_control = (u32 *)((*ctrl)->control_usbotghs_ctrl);
	*usbotghs_control = USBOTGHS_CONTROL_AVALID |
		USBOTGHS_CONTROL_VBUSVALID | USBOTGHS_CONTROL_IDDIG;
#endif

	return 0;
#endif
}

static void omap2430_musb_disable(struct musb *musb)
{
#ifndef __UBOOT__
	struct device *dev = musb->controller;
	struct omap2430_glue *glue = dev_get_drvdata(dev->parent);

	if (glue->status != OMAP_MUSB_UNKNOWN)
		usb_phy_shutdown(musb->xceiv);
#endif
}

static int omap2430_musb_exit(struct musb *musb)
{
	del_timer_sync(&musb_idle_timer);

	omap2430_low_level_exit(musb);

	return 0;
}

#ifndef __UBOOT__
static const struct musb_platform_ops omap2430_ops = {
#else
const struct musb_platform_ops omap2430_ops = {
#endif
	.init		= omap2430_musb_init,
	.exit		= omap2430_musb_exit,

#ifndef __UBOOT__
	.set_mode	= omap2430_musb_set_mode,
	.try_idle	= omap2430_musb_try_idle,

	.set_vbus	= omap2430_musb_set_vbus,
#endif

	.enable		= omap2430_musb_enable,
	.disable	= omap2430_musb_disable,
};

#ifndef __UBOOT__
static u64 omap2430_dmamask = DMA_BIT_MASK(32);

static int __devinit omap2430_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	*pdata = pdev->dev.platform_data;
	struct platform_device		*musb;
	struct omap2430_glue		*glue;
	int				ret = -ENOMEM;

	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		dev_err(&pdev->dev, "failed to allocate glue context\n");
		goto err0;
	}

	musb = platform_device_alloc("musb-hdrc", -1);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		goto err0;
	}

	musb->dev.parent		= &pdev->dev;
	musb->dev.dma_mask		= &omap2430_dmamask;
	musb->dev.coherent_dma_mask	= omap2430_dmamask;

	glue->dev			= &pdev->dev;
	glue->musb			= musb;
	glue->status			= OMAP_MUSB_UNKNOWN;

	pdata->platform_ops		= &omap2430_ops;

	platform_set_drvdata(pdev, glue);

	/*
	 * REVISIT if we ever have two instances of the wrapper, we will be
	 * in big trouble
	 */
	_glue	= glue;

	INIT_WORK(&glue->omap_musb_mailbox_work, omap_musb_mailbox_work);

	ret = platform_device_add_resources(musb, pdev->resource,
			pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto err1;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err1;
	}

	pm_runtime_enable(&pdev->dev);

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err1;
	}

	return 0;

err1:
	platform_device_put(musb);

err0:
	return ret;
}

static int __devexit omap2430_remove(struct platform_device *pdev)
{
	struct omap2430_glue		*glue = platform_get_drvdata(pdev);

	cancel_work_sync(&glue->omap_musb_mailbox_work);
	platform_device_del(glue->musb);
	platform_device_put(glue->musb);

	return 0;
}

#ifdef CONFIG_PM

static int omap2430_runtime_suspend(struct device *dev)
{
	struct omap2430_glue		*glue = dev_get_drvdata(dev);
	struct musb			*musb = glue_to_musb(glue);

	if (musb) {
		musb->context.otg_interfsel = musb_readl(musb->mregs,
				OTG_INTERFSEL);

		omap2430_low_level_exit(musb);
		usb_phy_set_suspend(musb->xceiv, 1);
	}

	return 0;
}

static int omap2430_runtime_resume(struct device *dev)
{
	struct omap2430_glue		*glue = dev_get_drvdata(dev);
	struct musb			*musb = glue_to_musb(glue);

	if (musb) {
		omap2430_low_level_init(musb);
		musb_writel(musb->mregs, OTG_INTERFSEL,
				musb->context.otg_interfsel);

		usb_phy_set_suspend(musb->xceiv, 0);
	}

	return 0;
}

static struct dev_pm_ops omap2430_pm_ops = {
	.runtime_suspend = omap2430_runtime_suspend,
	.runtime_resume = omap2430_runtime_resume,
};

#define DEV_PM_OPS	(&omap2430_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif

static struct platform_driver omap2430_driver = {
	.probe		= omap2430_probe,
	.remove		= __devexit_p(omap2430_remove),
	.driver		= {
		.name	= "musb-omap2430",
		.pm	= DEV_PM_OPS,
	},
};

MODULE_DESCRIPTION("OMAP2PLUS MUSB Glue Layer");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("GPL v2");

static int __init omap2430_init(void)
{
	return platform_driver_register(&omap2430_driver);
}
subsys_initcall(omap2430_init);

static void __exit omap2430_exit(void)
{
	platform_driver_unregister(&omap2430_driver);
}
module_exit(omap2430_exit);
#endif
