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
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <plat/irqs.h>
#include <plat/mux.h>
#include <plat/usb.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/timer-gp.h>

#include "clock.h"
#include "clockdomains.h"
#include "powerdomains.h"
#include "mux.h"
#include "hsmmc.h"

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,		/* FIXME: Should this be 8? */
		.gpio_cd	= -EINVAL,	/* FIXME: GPIO numbers */
		.gpio_wp	= 63,
	},
	{}	/* Terminator */
};

static struct omap2_mcspi_device_config m25p32_mcspi_config = {
    .turbo_mode = 0,
    .single_channel = 1,    /* 0: slave, 1: master */
};

static struct flash_platform_data m25p32_flash_data = {
	.type			= "m25p32",
};

static struct spi_board_info ti8168_evm_spi_info[] __initconst = {
	{
		.modalias       = "m25p32",
		.max_speed_hz   = 10 * 1000 * 1000,
		.bus_num        = 0,
		.chip_select    = 0,
		.mode           = SPI_MODE_0,
		.controller_data = &m25p32_mcspi_config,
		.platform_data = &m25p32_flash_data,
		.irq			= TI816X_IRQ_SPI,
	},
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 500,
};

static void __init ti8168_evm_init_irq(void)
{
	omap2_gp_clockevent_set_gptimer(2);
	omap2_init_common_hw(NULL, NULL);
	omap_init_irq();
}

static struct omap_board_config_kernel generic_config[] = {
};

int __init ti_ahci_register(u8 num_inst);

static void __init ti8168_evm_init(void)
{
	omap_board_config = generic_config;
	omap_board_config_size = ARRAY_SIZE(generic_config);

	spi_register_board_info(ti8168_evm_spi_info,
				ARRAY_SIZE(ti8168_evm_spi_info));

	omap_serial_init();
/*
There are two instances of I2C in TI 816x but currently only one instance
is used by TI 816x EVM. Registering a single isntance
*/
	omap_register_i2c_bus(1, 100, NULL, 0);
	/* TODO: Decide on the GPIO pin number */
	omap_mux_init_gpio(63, OMAP_PIN_INPUT);
	omap2_hsmmc_init(mmc);

	/* initialize usb */
	usb_musb_init(&musb_board_data);

	/* register ahci interface for 2 SATA ports */
	ti_ahci_register(2);
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
