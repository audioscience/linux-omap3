/*
 * linux/arch/arm/mach-omap2/board-omap3517evm.c
 *
 * Copyright (C) 2009 Texas Instruments Incorporated
 *
 * Based on mach-omap2/board-omap3evm.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/input.h>
#include <linux/leds.h>

#include <linux/spi/spi.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/mux.h>
#include <mach/usb.h>
#include <mach/common.h>
#include <mach/mcspi.h>


/*
 * UART
 */
static struct omap_uart_config omap3517_evm_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

/*
 * Ethernet
 */
static int __init omap3517_evm_ethernet_init (void)
{

	return 0 ;
}

/*
 * I2C
 */
static int __init omap3517_evm_i2c_init(void)
{
	return 0;
}

/*
 * Power Management IC
 */


/*
 * MTD
 */

/*
 * SPI
 */

/*
 * MTD
 */

/*
 * Board initialization
 */
static struct omap_board_config_kernel omap3517_evm_config[] __initdata = {
	{ OMAP_TAG_UART,	&omap3517_evm_uart_config },
};

static struct platform_device *omap3517_evm_devices[] __initdata = {
};

static void __init omap3517_evm_init_irq(void)
{
	omap2_init_common_hw(NULL);
	omap_init_irq();
	omap_gpio_init();
}

static void __init omap3517_evm_init(void)
{
	omap3517_evm_i2c_init();

	platform_add_devices(omap3517_evm_devices,
				ARRAY_SIZE(omap3517_evm_devices));

	omap_board_config = omap3517_evm_config;
	omap_board_config_size = ARRAY_SIZE(omap3517_evm_config);

	omap_serial_init();
	omap3517_evm_ethernet_init();
}

static void __init omap3517_evm_map_io(void)
{
	omap2_set_globals_35xx();
	omap2_map_common_io();
}

MACHINE_START(OMAP3517EVM, "OMAP3517 EVM")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3517_evm_map_io,
	.init_irq	= omap3517_evm_init_irq,
	.init_machine	= omap3517_evm_init,
	.timer		= &omap_timer,
MACHINE_END

