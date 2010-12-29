/*
 * linux/arch/arm/mach-omap2/usb-musb.c
 *
 * This file will contain the board specific details for the
 * MENTOR USB OTG controller on OMAP3430
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Vikram Pandita
 *
 * Generalization by:
 * Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>

#include <linux/usb/musb.h>

#include <asm/sizes.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/am35xx.h>
#include <plat/usb.h>

#define OTG_SYSCONFIG	   0x404
#define OTG_SYSC_SOFTRESET BIT(1)
#define OTG_SYSSTATUS     0x408
#define OTG_SYSS_RESETDONE BIT(0)

static struct platform_device dummy_pdev = {
	.dev = {
		.bus = &platform_bus_type,
	},
};

static void __iomem *otg_base;
static struct clk *otg_clk;

static void __init usb_musb_pm_init(void)
{
	struct device *dev = &dummy_pdev.dev;

	if (!cpu_is_omap34xx())
		return;

	otg_base = ioremap(OMAP34XX_HSUSB_OTG_BASE, SZ_4K);
	if (WARN_ON(!otg_base))
		return;

	dev_set_name(dev, "musb-omap2430");
	otg_clk = clk_get(dev, "ick");

	if (otg_clk && clk_enable(otg_clk)) {
		printk(KERN_WARNING
			"%s: Unable to enable clocks for MUSB, "
			"cannot reset.\n",  __func__);
	} else {
		/* Reset OTG controller. After reset, it will be in
		 * force-idle, force-standby mode. */
		__raw_writel(OTG_SYSC_SOFTRESET, otg_base + OTG_SYSCONFIG);

		while (!(OTG_SYSS_RESETDONE &
					__raw_readl(otg_base + OTG_SYSSTATUS)))
			cpu_relax();
	}

	if (otg_clk)
		clk_disable(otg_clk);
}

void usb_musb_disable_autoidle(void)
{
	if (otg_clk) {
		unsigned long reg;

		clk_enable(otg_clk);
		reg = __raw_readl(otg_base + OTG_SYSCONFIG);
		__raw_writel(reg & ~1, otg_base + OTG_SYSCONFIG);
		clk_disable(otg_clk);
	}
}

#ifdef CONFIG_USB_MUSB_HDRC

static struct resource musb_resources[] = {
	[0] = { /* start and end set dynamically */
		.flags	= IORESOURCE_MEM,
	},
	[1] = {	/* general IRQ */
		.start	= INT_243X_HS_USB_MC,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {	/* DMA IRQ */
		.start	= INT_243X_HS_USB_DMA,
		.flags	= IORESOURCE_IRQ,
	},
	[3] = { /* MEM for TI81x's second musb */
		.flags  = IORESOURCE_MEM,
		.start	= TI81XX_USB1_BASE,
		.end	= TI81XX_USB1_BASE + SZ_2K - 1,
	},
	[4] = {	/* IRQ for TI81x's second musb */
		.start	= TI81XX_IRQ_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct musb_hdrc_config musb_config = {
	.fifo_mode	= 4,
	.multipoint	= 1,
	.dyn_fifo	= 1,
	.num_eps	= 16,
	.ram_bits	= 12,
};

static struct musb_hdrc_platform_data musb_plat[] = {
	{
		.config         = &musb_config,
		.clock          = "ick",
	},
	{
		.config         = &musb_config,
		.clock          = "ick",
	},
};

static u64 musb_dmamask = DMA_BIT_MASK(32);

static struct platform_device musb_device[] = {
	{
		.name		= "musb-omap2430",
		.id		= 0,
		.dev = {
			.dma_mask		= &musb_dmamask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
			.platform_data		= &musb_plat[0],
		},
		.num_resources	= 3,
		.resource	= &musb_resources[0],
	},
	{
		.name		= "musb-omap2430",
		.id		= 1,
		.dev = {
			.dma_mask		= &musb_dmamask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
			.platform_data		= &musb_plat[1],
		},
		.num_resources	= 3,
		.resource	= &musb_resources[3],
	},

};

void __init usb_musb_init(struct omap_musb_board_data *board_data)
{
	int i;

	if (cpu_is_omap243x()) {
		musb_resources[0].start = OMAP243X_HS_BASE;
	} else if (cpu_is_omap3517() || cpu_is_omap3505()) {
		musb_device[0].name = "musb-am35x";
		musb_resources[0].start = AM35XX_IPSS_USBOTGSS_BASE;
		musb_resources[1].start = INT_35XX_USBOTG_IRQ;
	} else if (cpu_is_omap34xx()) {
		musb_resources[0].start = OMAP34XX_HSUSB_OTG_BASE;
	} else if (cpu_is_omap44xx()) {
		musb_resources[0].start = OMAP44XX_HSUSB_OTG_BASE;
		musb_resources[1].start = OMAP44XX_IRQ_HS_USB_MC_N;
		musb_resources[2].start = OMAP44XX_IRQ_HS_USB_DMA_N;
	} else if (cpu_is_ti81xx()) {

		/* disable musb multipoint support for ti8168 */
		if (cpu_is_ti816x())
			musb_config.multipoint = 0;

		/* only usb0 port enabled in peripheral mode*/
		if (board_data->mode == MUSB_PERIPHERAL)
			board_data->instances = 0;

		musb_resources[0].start = TI81XX_USB0_BASE;
		musb_resources[1].start = TI81XX_IRQ_USB0;
		musb_resources[0].end = musb_resources[0].start + SZ_2K - 1;

		for (i = 0; i <= board_data->instances; i++) {
			musb_device[i].name = "musb-ti81xx";
			musb_device[i].num_resources = 2;
		}

		musb_config.fifo_mode = 4;
	}

	if (cpu_is_omap3517() || cpu_is_omap3505())
		musb_resources[0].end = musb_resources[0].start + SZ_32K - 1;
	else if (!cpu_is_ti81xx())
		musb_resources[0].end = musb_resources[0].start + SZ_4K - 1;

	/*
	 * OMAP3630/AM35x platform has MUSB RTL-1.8 which has the fix for the
	 * issue restricting active endpoints to use first 8K of FIFO space.
	 * This issue restricts OMAP35x platform to use fifo_mode '5'.
	 */
	if (cpu_is_omap3430())
		musb_config.fifo_mode = 5;

	for (i = 0; i <= board_data->instances; i++) {
		if (cpu_is_ti816x())
			musb_plat[i].clock = "usbotg_ick";
		else if (cpu_is_ti814x())
			musb_plat[i].clock = "usb_ick";

		musb_plat[i].board_data = board_data;
		musb_plat[i].power = board_data->power >> 1;
		musb_plat[i].mode = board_data->mode;
		musb_plat[i].extvbus = board_data->extvbus;

		if (platform_device_register(&musb_device[i]) < 0)
			printk(KERN_ERR "Unable to register HS-USB (MUSB) device\n");
	}

	usb_musb_pm_init();
}

#else
void __init usb_musb_init(struct omap_musb_board_data *board_data)
{
	usb_musb_pm_init();
}
#endif  /* CONFIG_USB_MUSB_SOC */
