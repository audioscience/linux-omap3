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

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/am35xx.h>
#include <plat/mux.h>
#include <plat/usb.h>
#include <plat/irqs-ti816x.h>

#ifdef CONFIG_USB_MUSB_SOC

#undef MULTI_MUSB_INSTANCE

#ifdef MULTI_MUSB_INSTANCE
#define MAX_MUSB_CONTROLLERS	2
#else
#define MAX_MUSB_CONTROLLERS	1
#endif

static struct resource musb_resources[] = {
	[0] = { /* start and end set dynamically */
		.flags	= IORESOURCE_MEM,
	},
	[1] = {	/* general IRQ */
		.start	= INT_243X_HS_USB_MC,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {	/* DMA IRQ */
		.flags	= IORESOURCE_MEM,
	},
	[3] = {	/* DMA IRQ */
		.start	= INT_243X_HS_USB_DMA,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct musb_hdrc_config musb_config = {
	.multipoint	= 0,
	.dyn_fifo	= 1,
	.num_eps	= 16,
	.ram_bits	= 12,
};

static struct musb_hdrc_platform_data musb_plat = {
#ifdef CONFIG_USB_MUSB_OTG
	.mode		= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode		= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode		= MUSB_PERIPHERAL,
#endif
	/* .clock is set dynamically */
	.config		= &musb_config,

	/* REVISIT charge pump on TWL4030 can supply up to
	 * 100 mA ... but this value is board-specific, like
	 * "mode", and should be passed to usb_musb_init().
	 */
	.power		= 50,			/* up to 100 mA */
};

static u64 musb_dmamask = DMA_BIT_MASK(32);

static struct platform_device musb_devices[] = {
	{
		.name		= "musb_hdrc.0",
		.id		= -1,
		.dev = {
			.dma_mask		= &musb_dmamask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
			.platform_data		= &musb_plat,
		},
		.num_resources	= ARRAY_SIZE(musb_resources)/2,
		.resource	= &musb_resources[0],
	},
	{
		.name		= "musb_hdrc.1",
		.id		= -1,
		.dev = {
			.dma_mask		= &musb_dmamask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
			.platform_data		= &musb_plat,
		},
		.num_resources	= ARRAY_SIZE(musb_resources)/2,
		.resource	= &musb_resources[2],
	},
};

struct clk *usbotg_clk;

void __init usb_musb_init(struct omap_musb_board_data *board_data)
{
	int i;

	usbotg_clk = clk_get(NULL, "usbotg_ick");
	if (IS_ERR(usbotg_clk)) {
		pr_err("usb : Failed to get usbotg clock\n");
		return ;
	}

	if (clk_enable(usbotg_clk)) {
		pr_err("usb : Clock Enable Failed\n");
		return ;
	}

	if (cpu_is_omap243x()) {
		musb_resources[0].start = OMAP243X_HS_BASE;
		musb_resources[0].end = musb_resources[0].start + SZ_4K - 1;
	} else if (cpu_is_omap3517()) {
		musb_resources[0].start = AM35XX_IPSS_USBOTGSS_BASE;
		musb_resources[1].start = INT_35XX_USBOTG_IRQ;
		musb_resources[0].end = musb_resources[0].start
						+ (2 * SZ_16K) - 1;
	} else if (cpu_is_omap34xx()) {
		musb_resources[0].start = OMAP34XX_HSUSB_OTG_BASE;
		musb_resources[0].end = musb_resources[0].start + SZ_4K - 1;
	} else if (cpu_is_omap44xx()) {
		musb_resources[0].start = OMAP44XX_HSUSB_OTG_BASE;
		musb_resources[1].start = OMAP44XX_IRQ_HS_USB_MC_N;
		musb_resources[2].start = OMAP44XX_IRQ_HS_USB_DMA_N;
		musb_resources[0].end = musb_resources[0].start + SZ_4K - 1;
	} else if (cpu_is_ti816x()) {
		musb_resources[0].start = TI816X_USB0_BASE;
		musb_resources[1].start = TI816X_IRQ_USB0;
		musb_resources[0].end = musb_resources[0].start + SZ_2K - 1;
#ifdef MULTI_MUSB_INSTANCE
		musb_resources[2].start = TI816X_USB1_BASE;
		musb_resources[3].start = TI816X_IRQ_USB1;
		musb_resources[2].end = musb_resources[2].start + SZ_2K - 1;
#endif
	}

	/*
	 * REVISIT: This line can be removed once all the platforms using
	 * musb_core.c have been converted to use use clkdev.
	 */
	musb_plat.clock = "usbotg_ick";
	musb_plat.board_data = board_data;
	musb_plat.power = board_data->power >> 1;
	musb_plat.mode = board_data->mode;
	musb_plat.extvbus = board_data->extvbus;

	for (i = 0; i < MAX_MUSB_CONTROLLERS; i++) {
		musb_devices[i].id = i;
		if (platform_device_register(&musb_devices[i]) < 0)
			printk(KERN_ERR "Unable to register HS-USB (MUSB) device\n");
	}
}

#else
void __init usb_musb_init(struct omap_musb_board_data *board_data)
{
}
#endif /* CONFIG_USB_MUSB_SOC */
