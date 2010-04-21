/*
 * arch/arm/mach-omap2/board-ti8168evm.c
 *
 * Code for TI8168 EVM.
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
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/pcf857x.h>
#include <linux/i2c/at24.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/phy.h>

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


#if defined(CONFIG_MTD_PHYSMAP) || \
    defined(CONFIG_MTD_PHYSMAP_MODULE)
#define HAS_NOR 1
#else
#define HAS_NOR 0
#endif

static struct mtd_partition ti816x_evm_norflash_partitions[] = {
	/* bootloader (U-Boot, etc) in first 5 sectors */
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= 5 * SZ_128K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params in the next 1 sectors */
	{
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= 0,
	},
	/* kernel */
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_2M,
		.mask_flags	= 0
	},
	/* file system */
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0
	}
};

static struct physmap_flash_data ti816x_evm_norflash_data = {
	.width		= 2,
	.parts		= ti816x_evm_norflash_partitions,
	.nr_parts	= ARRAY_SIZE(ti816x_evm_norflash_partitions),
};

#define TI816X_EVM_NOR_BASE			0x0000000
static struct resource ti816x_evm_norflash_resource = {
	.start		= TI816X_EVM_NOR_BASE,
	.end		= TI816X_EVM_NOR_BASE + SZ_64M - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device ti816x_evm_norflash_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &ti816x_evm_norflash_data,
	},
	.num_resources	= 1,
	.resource	= &ti816x_evm_norflash_resource,
};

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
		.irq		= TI816X_IRQ_SPI,
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

/* FIX ME: Check the address of I2C expander */

static struct i2c_board_info __initdata ti816x_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("eeprom", 0x50),
	},

	{
		I2C_BOARD_INFO("cpld", 0x23),
	},
};

/* FIX ME: Check on the Bit Value */

#define TI816X_EVM_CIR_UART BIT(5)

static struct i2c_client *cpld_reg0_client;
/* CPLD Register 0 Client: used for I/O Control */
static int cpld_reg0_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	u8 data;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &data,
		},
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &data,
		},
	};

	cpld_reg0_client = client;

	/* Clear UART CIR to enable cir operation. */
		i2c_transfer(client->adapter, msg, 1);
		data &= ~(TI816X_EVM_CIR_UART);
		i2c_transfer(client->adapter, msg + 1, 1);
	return 0;
}


static const struct i2c_device_id cpld_reg_ids[] = {
		{ "cpld_reg0", 0, },
		{ },
};

static struct i2c_driver ti816xevm_cpld_driver = {
	.driver.name    = "cpld_reg0",
	.id_table       = cpld_reg_ids,
	.probe          = cpld_reg0_probe,
};


static int __init ti816x_evm_i2c_init(void)
{
/*
There are two instances of I2C in TI 816x but currently only one instance
is used by TI 816x EVM. Registering a single isntance
*/
	omap_register_i2c_bus(1, 100, ti816x_i2c_boardinfo,
		ARRAY_SIZE(ti816x_i2c_boardinfo));
	return 0;
}



static void __init ti8168_evm_init(void)
{
	omap_board_config = generic_config;
	omap_board_config_size = ARRAY_SIZE(generic_config);

	spi_register_board_info(ti8168_evm_spi_info,
				ARRAY_SIZE(ti8168_evm_spi_info));

	omap_serial_init();
	/* TODO: Decide on the GPIO pin number */
	omap_mux_init_gpio(63, OMAP_PIN_INPUT);
	omap2_hsmmc_init(mmc);

	/* initialize usb */
	usb_musb_init(&musb_board_data);

	/* register ahci interface for 2 SATA ports */
	ti_ahci_register(2);
	ti816x_evm_i2c_init();
	i2c_add_driver(&ti816xevm_cpld_driver);

	if (HAS_NOR)
		platform_device_register(&ti816x_evm_norflash_device);
}


static void __init ti8168_evm_map_io(void)
{
	omap2_set_globals_ti816x();
	ti816x_map_common_io();
}

MACHINE_START(TI8168_EVM, "ti8168evm")
	/* Maintainer: Texas Instruments */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= ti8168_evm_map_io,
	.init_irq	= ti8168_evm_init_irq,
	.init_machine	= ti8168_evm_init,
	.timer		= &omap_timer,
MACHINE_END




