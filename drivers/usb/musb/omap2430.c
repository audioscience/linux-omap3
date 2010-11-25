/*
 * Copyright (C) 2005-2010 Texas Instruments Incorporated - http://www.ti.com/
 * Some code has been taken from tusb6010.c
 * Copyrights for that are attributable to:
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include "musb_core.h"
#include "omap2430.h"
#include "musbhsdma.h"

struct omap_musb_glue {
	struct clk	*clk;
	struct device	*dev;
};

#define musb_to_omap(d)	dev_get_drvdata(musb->controller->parent)

static struct timer_list musb_idle_timer;
static u64 musb_dmamask = DMA_BIT_MASK(32);

static void omap2430_musb_do_idle(unsigned long _musb)
{
	struct musb	*musb = (void *)_musb;
	unsigned long	flags;
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	u8	power;
#endif
	u8	devctl;

	spin_lock_irqsave(&musb->lock, flags);

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	switch (musb->xceiv->state) {
	case OTG_STATE_A_WAIT_BCON:
		devctl &= ~MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_BDEVICE) {
			musb->xceiv->state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
		} else {
			musb->xceiv->state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(musb);
		}
		break;
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	case OTG_STATE_A_SUSPEND:
		/* finish RESUME signaling? */
		if (musb->port1_status & MUSB_PORT_STAT_RESUME) {
			power = musb_readb(musb->mregs, MUSB_POWER);
			power &= ~MUSB_POWER_RESUME;
			DBG(1, "root port resume stopped, power %02x\n", power);
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
#endif
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	case OTG_STATE_A_HOST:
		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl &  MUSB_DEVCTL_BDEVICE)
			musb->xceiv->state = OTG_STATE_B_IDLE;
		else
			musb->xceiv->state = OTG_STATE_A_WAIT_BCON;
#endif
	default:
		break;
	}
	spin_unlock_irqrestore(&musb->lock, flags);
}

#define MUSB_TIMEOUT_A_WAIT_BCON	1100

static void omap2430_musb_try_idle(struct musb *musb, unsigned long timeout)
{
	unsigned long		default_timeout = jiffies + msecs_to_jiffies(3);
	static unsigned long	last_timer;

	if (timeout == 0)
		timeout = default_timeout;

	/* Never idle if active, or when VBUS timeout is not set as host */
	if (musb->is_active || ((musb->a_wait_bcon == 0)
			&& (musb->xceiv->state == OTG_STATE_A_WAIT_BCON))) {
		DBG(4, "%s active, deleting timer\n", otg_state_string(musb));
		del_timer(&musb_idle_timer);
		last_timer = jiffies;
		return;
	}

	if (time_after(last_timer, timeout)) {
		if (!timer_pending(&musb_idle_timer))
			last_timer = timeout;
		else {
			DBG(4, "Longer idle timer already pending, ignoring\n");
			return;
		}
	}
	last_timer = timeout;

	DBG(4, "%s inactive, for idle timer for %lu ms\n",
		otg_state_string(musb),
		(unsigned long)jiffies_to_msecs(timeout - jiffies));
	mod_timer(&musb_idle_timer, timeout);
}

static int omap2430_musb_set_vbus(struct musb *musb, int is_on)
{
	u8		devctl;
	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	if (is_on) {
		musb->is_active = 1;
		musb->xceiv->default_a = 1;
		musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
		devctl |= MUSB_DEVCTL_SESSION;

		MUSB_HST_MODE(musb);
	} else {
		musb->is_active = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		musb->xceiv->default_a = 0;
		musb->xceiv->state = OTG_STATE_B_IDLE;
		devctl &= ~MUSB_DEVCTL_SESSION;

		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	DBG(1, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		otg_state_string(musb),
		musb_readb(musb->mregs, MUSB_DEVCTL));

	return 0;
}

static int omap2430_musb_set_mode(struct musb *musb, u8 musb_mode)
{
	u8	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	devctl |= MUSB_DEVCTL_SESSION;
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	return 0;
}

#ifdef CONFIG_PM
static void omap2430_musb_save_context(struct musb *musb)
{
	struct musb_context_registers *ctx = &musb->context;

	ctx->otg_sysconfig = musb_readl(musb->mregs, OTG_SYSCONFIG);
	ctx->otg_forcestandby = musb_readl(musb->mregs, OTG_FORCESTDBY);
}

static void omap2430_musb_restore_context(struct musb *musb)
{
	struct musb_context_registers *ctx = &musb->context;

	musb_writel(musb->mregs, OTG_SYSCONFIG, ctx->otg_sysconfig);
	musb_writel(musb->mregs, OTG_FORCESTDBY, ctx->otg_forcestandby);
}

static int omap2430_musb_suspend(struct musb *musb)
{
	struct omap_musb_glue	*omap = musb_to_omap(musb);
	u32 l;

	/* in any role */
	l = musb_readl(musb->mregs, OTG_FORCESTDBY);
	l |= ENABLEFORCE;	/* enable MSTANDBY */
	musb_writel(musb->mregs, OTG_FORCESTDBY, l);

	l = musb_readl(musb->mregs, OTG_SYSCONFIG);
	l |= ENABLEWAKEUP;	/* enable wakeup */
	musb_writel(musb->mregs, OTG_SYSCONFIG, l);

	omap2430_musb_save_context(musb);

	otg_set_suspend(musb->xceiv, 1);

	clk_disable(omap->clk);

	return 0;
}

static int omap2430_musb_resume(struct musb *musb)
{
	u32 l;

	otg_set_suspend(musb->xceiv, 0);

	omap2430_musb_restore_context(musb);

	l = musb_readl(musb->mregs, OTG_SYSCONFIG);
	l &= ~ENABLEWAKEUP;	/* disable wakeup */
	musb_writel(musb->mregs, OTG_SYSCONFIG, l);

	l = musb_readl(musb->mregs, OTG_FORCESTDBY);
	l &= ~ENABLEFORCE;	/* disable MSTANDBY */
	musb_writel(musb->mregs, OTG_FORCESTDBY, l);

	return 0;
}
#else
#define omap2430_musb_suspend	NULL
#define omap2430_musb_resume	NULL
#endif

static int omap2430_musb_init(struct musb *musb, void *board_data)
{
	struct omap_musb_board_data *data = board_data;
	u32 l;

	/* We require some kind of external transceiver, hooked
	 * up through ULPI.  TWL4030-family PMICs include one,
	 * which needs a driver, drivers aren't always needed.
	 */
	musb->xceiv = otg_get_transceiver();
	if (!musb->xceiv) {
		pr_err("HS USB OTG: no transceiver configured\n");
		return -ENODEV;
	}

	omap2430_musb_resume(musb);

	l = musb_readl(musb->mregs, OTG_SYSCONFIG);
	l &= ~ENABLEWAKEUP;	/* disable wakeup */
	l &= ~NOSTDBY;		/* remove possible nostdby */
	l |= SMARTSTDBY;	/* enable smart standby */
	l &= ~AUTOIDLE;		/* disable auto idle */
	l &= ~NOIDLE;		/* remove possible noidle */

	/* SMARTIDLE is blocking core to enter off mode in 3630 */
	if (cpu_is_omap3630())
		l |= FORCEIDLE;		/* enable force idle */
	else
		l |= SMARTIDLE;		/* enable smart idle */
	/*
	 * MUSB AUTOIDLE don't work in 3430.
	 * Workaround by Richard Woodruff/TI
	 */
	if (!cpu_is_omap3430())
		l |= AUTOIDLE;		/* enable auto idle */
	musb_writel(musb->mregs, OTG_SYSCONFIG, l);

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

	musb->a_wait_bcon = MUSB_TIMEOUT_A_WAIT_BCON;
	setup_timer(&musb_idle_timer, omap2430_musb_do_idle,
			(unsigned long) musb);

	musb->inventra = 1;
	otg_put_transceiver(musb->xceiv);
	return 0;
}

static int omap2430_musb_exit(struct musb *musb)
{
	return omap2430_musb_suspend(musb);
}

static struct musb_platform_ops omap2430_musb_ops = {
	.init		= omap2430_musb_init,
	.exit		= omap2430_musb_exit,
	.suspend	= omap2430_musb_suspend,
	.resume		= omap2430_musb_resume,
	.try_idle	= omap2430_musb_try_idle,
	.set_vbus	= omap2430_musb_set_vbus,
	.set_mode	= omap2430_musb_set_mode,
#ifndef CONFIG_MUSB_PIO_ONLY
	.dma_create	= inventra_dma_controller_create,
	.dma_destroy	= inventra_dma_controller_destroy,
#endif
	.read_fifo	= generic_musb_read_fifo,
	.write_fifo	= generic_musb_write_fifo,

	.read_long		= generic_musb_readl,
	.read_word		= generic_musb_readw,
	.read_byte		= generic_musb_readb,

	.write_long		= generic_musb_writel,
	.write_word		= generic_musb_writew,
	.write_byte		= generic_musb_writeb,
};

static int __init omap2430_musb_probe(struct platform_device *pdev)
{
	struct omap_musb_glue		*omap;
	int				ret;
	struct platform_device		*musb;
	struct musb_hdrc_platform_data	*pdata = pdev->dev.platform_data;

	omap = kzalloc(sizeof(*omap), GFP_KERNEL);
	if (!omap) {
		dev_err(&pdev->dev, "unable to allocate memory\n");
		return -ENOMEM;
	}

	omap->clk = clk_get(&pdev->dev, "ick");
	if (IS_ERR(omap->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		ret = PTR_ERR(omap->clk);
		goto err0;
	}

	musb = platform_device_alloc("musb_hdrc", -1);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		ret = -ENOMEM;
		goto err1;
	}

	pdata->ops = &omap2430_musb_ops;

	musb->dev.dma_mask = &musb_dmamask;
	musb->dev.coherent_dma_mask = musb_dmamask;

	platform_device_add_resources(musb, pdev->resource,
					pdev->num_resources);

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err2;
	}

	ret = clk_enable(omap->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		goto err2;
	}

	platform_set_drvdata(pdev, omap);
	omap->dev = &pdev->dev;

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err3;
	}

	return 0;

err3:
	clk_disable(omap->clk);

err2:
	platform_device_put(musb);

err1:
	clk_put(omap->clk);

err0:
	kfree(omap);

	return ret;
}

static int __exit omap2430_musb_remove(struct platform_device *pdev)
{
	struct omap_musb_glue		*omap = platform_get_drvdata(pdev);

	clk_disable(omap->clk);
	clk_put(omap->clk);
	kfree(omap);

	return 0;
}

static struct platform_driver omap2430_musb_driver = {
	.remove		= __exit_p(omap2430_musb_remove),
	.driver		= {
		.name	= "musb-omap2430",
	},
};

MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_DESCRIPTION("OMAP2+ MUSB Glue Layer");
MODULE_LICENSE("GPL v2");

static int __init omap2430_glue_init(void)
{
	return platform_driver_probe(&omap2430_musb_driver,
			omap2430_musb_probe);
}
subsys_initcall(omap2430_glue_init);

static void __exit omap2430_glue_exit(void)
{
	platform_driver_unregister(&omap2430_musb_driver);
}
module_exit(omap2430_glue_exit);
