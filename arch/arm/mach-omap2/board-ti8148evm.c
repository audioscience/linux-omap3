/*
 * Code for TI8148 EVM.
 *
 * Copyright (C) 2010 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>

#include "clock.h"
#include "clockdomains.h"
#include "powerdomains.h"
#include "hsmmc.h"


static struct omap2_hsmmc_info mmc[] = {
       {
               .mmc            = 1,
               .caps           = MMC_CAP_4_BIT_DATA,
               .gpio_cd        = -EINVAL,/* Dedicated pins for CD and WP */
               .gpio_wp        = -EINVAL,
               .ocr_mask       = MMC_VDD_33_34,
       },
       {}      /* Terminator */
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode           = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode           = MUSB_PERIPHERAL,
#endif
	.power			= 500,
	.instances              = 1,
};

static void __init ti8148_evm_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL);
	omap_init_irq();
}

static void __init ti8148_evm_init(void)
{
	omap_serial_init();

	omap2_hsmmc_init(mmc);

	/* initialize usb */
	usb_musb_init(&musb_board_data);
}

static void __init ti8148_evm_map_io(void)
{
	omap2_set_globals_ti814x();
	ti81xx_map_common_io();
}

MACHINE_START(TI8148EVM, "ti8148evm")
	/* Maintainer: Texas Instruments */
	.boot_params	= 0x80000100,
	.map_io		= ti8148_evm_map_io,
	.init_irq	= ti8148_evm_init_irq,
	.init_machine	= ti8148_evm_init,
	.timer		= &omap_timer,
MACHINE_END
