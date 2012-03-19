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
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/phy.h>
#include <linux/regulator/machine.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/asp.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/hdmi_lib.h>
#include <plat/ti81xx_ram.h>
#include <mach/board-ti814x.h>

#include "board-flash.h"
#include "clock.h"
#include "mux.h"
#include "hsmmc.h"
#include "control.h"

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux     NULL
#endif

static struct i2c_board_info __initdata asi1230_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("lm75", 0x4B),
	},
};

static void __init asi1230_i2c_init(void)
{
	omap_register_i2c_bus(1, 100, asi1230_i2c_boardinfo,
				ARRAY_SIZE(asi1230_i2c_boardinfo));
}

#ifndef CONFIG_MACH_TI8148EVM
/* I must define the following two functions when TI8148EVM support is not configured in 
 * or usb-ehci.c (!!!) will be missing symbols
 */
int vps_ti814x_select_video_decoder(int vid_decoder_id) { return 0; }
int vps_ti814x_set_tvp7002_filter(enum fvid2_standard standard) { return 0; }
#endif

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL, /* Dedicated pins for CD and WP */
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_33_34,
	},
	{}	/* Terminator */
};

const struct flash_platform_data asi1230_spi_flash = {
	.type		= "m25p16",
	.name		= "spi_flash",
	.parts		= NULL,
	.nr_parts	= 0,
};

struct spi_board_info __initdata asi1230_spi_slave_info[] = {
	{
		.modalias	= "m25p80",
		.platform_data	= &asi1230_spi_flash,
		.irq		= -1,
		.max_speed_hz	= 75000000,
		.bus_num	= 1,
		.chip_select	= 0,
	},
};

void __init asi1230_spi_init(void)
{
	spi_register_board_info(asi1230_spi_slave_info,
				ARRAY_SIZE(asi1230_spi_slave_info));
}

static void __init asi1230_init_irq(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
	gpmc_init();
}

#define PHY_VSC8601_ID 0x00070421
#define PHY_VSC8601_MASK 0xFFFFFFFF
#define PHY_VSC8601_EXCTRL1_REG 0x17
#define PHY_VSC8601_RXCLKSKEW 0x100

static int asi1230_vsc_phy_fixup(struct phy_device *phydev)
{
	unsigned int val;

    /* Enable RGMII RX/TX clock skew */
    val = phy_read(phydev, PHY_VSC8601_EXCTRL1_REG);
    val |= PHY_VSC8601_RXCLKSKEW;
    phy_write(phydev, PHY_VSC8601_EXCTRL1_REG, val);
    val = phy_read(phydev, PHY_VSC8601_EXCTRL1_REG);
	return 0;
}

static void __init asi1230_init(void)
{
	ti814x_mux_init(board_mux);
	omap_serial_init();
	asi1230_i2c_init();
	omap2_hsmmc_init(mmc);

	asi1230_spi_init();
	regulator_use_dummy_regulator();

	/* Register a clock skew ETH PHY fix for */
	phy_register_fixup_for_uid(PHY_VSC8601_ID, PHY_VSC8601_MASK, asi1230_vsc_phy_fixup);
}

static void __init asi1230_map_io(void)
{
	omap2_set_globals_ti814x();
	ti81xx_map_common_io();
}

#if 1
void  __init asi1230_reserve(void)
{
	//ti81xx_set_sdram_vram(0, 0);
	ti81xxfb_reserve_sdram_memblock();
//	ti81xx_pcie_mem_reserve_sdram_memblock();
}
#endif

MACHINE_START(ASI1230, "asi1230")
	/* Maintainer: Audioscience Inc */
	.boot_params	= 0x80000100,
	.map_io		= asi1230_map_io,
	.reserve        = asi1230_reserve, //ti81xx_reserve,
	.init_irq       = asi1230_init_irq,
	.init_machine   = asi1230_init,
	.timer          = &omap_timer,
MACHINE_END
