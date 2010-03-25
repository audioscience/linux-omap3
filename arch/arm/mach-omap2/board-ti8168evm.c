/*
 * arch/arm/mach-omap2/board-ti8168evm.c
 * Code for TI8168 EVM. Also supports Simulator and Zebu environemtns.
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/irqs.h>
#include <plat/mux.h>
#include <plat/usb.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/timer-gp.h>

#include "clock.h"
#include "clockdomains.h"
#include "powerdomains.h"

static void __init ti8168_evm_init_irq(void)
{
	omap2_gp_clockevent_set_gptimer(2);
	omap2_init_common_hw(NULL, NULL);
	omap_init_irq();
}

static struct omap_board_config_kernel generic_config[] = {
};

static void __init ti8168_evm_init(void)
{
	omap_board_config = generic_config;
	omap_board_config_size = ARRAY_SIZE(generic_config);
	omap_serial_init();
	omap_register_i2c_bus(1, 100, NULL, 0);
	omap_register_i2c_bus(2, 100, NULL, 0);
}

static void __init ti8168_evm_map_io(void)
{
	omap2_set_globals_ti816x();
	ti816x_map_common_io();
}

MACHINE_START(TI8168_EVM, "ti8168evm")
	/* Maintainer: Paul Mundt <paul.mundt@nokia.com> */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= ti8168_evm_map_io,
	.init_irq	= ti8168_evm_init_irq,
	.init_machine	= ti8168_evm_init,
	.timer		= &omap_timer,
MACHINE_END
