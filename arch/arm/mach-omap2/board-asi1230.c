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
#include <linux/mtd/nand.h>
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

/* I must define the following two functions or usb-ehci.c (!!!) will be missing symbols */
int vps_ti814x_select_video_decoder(int vid_decoder_id) { return 0; }
int vps_ti814x_set_tvp7002_filter(enum fvid2_standard standard) { return 0; }

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
	.type		= "w25x16",
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

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode           = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode           = MUSB_PERIPHERAL,
#endif
	.power		= 500,
	.instances	= 1,
};

static void __init asi1230_evm_init_irq(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
	gpmc_init();
}

static void __init asi1230_evm_init(void)
{
	ti814x_mux_init(board_mux);
	omap_serial_init();

	omap2_hsmmc_init(mmc);

	/* initialize usb */
	usb_musb_init(&musb_board_data);

    asi1230_spi_init();
	regulator_use_dummy_regulator();
}

static void __init asi1230_evm_map_io(void)
{
	omap2_set_globals_ti814x();
	ti81xx_map_common_io();
}

#if 1
void  __init asi1230_reserve(void)
{
	ti81xx_set_sdram_vram(0, 0);
	ti81xxfb_reserve_sdram_memblock();
//	ti81xx_pcie_mem_reserve_sdram_memblock();
}
#endif

MACHINE_START(ASI1230, "asi1230")
	/* Maintainer: Texas Instruments */
	.boot_params	= 0x80000100,
	.map_io		= asi1230_evm_map_io,
	.reserve         = asi1230_reserve, //ti81xx_reserve,
	.init_irq	= asi1230_evm_init_irq,
	.init_machine	= asi1230_evm_init,
	.timer		= &omap_timer,
MACHINE_END
