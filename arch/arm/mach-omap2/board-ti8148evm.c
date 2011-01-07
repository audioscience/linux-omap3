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
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/gpmc.h>
#include <plat/nand.h>


#include "hsmmc.h"
#include "board-flash.h"


/* NAND flash information */
static struct mtd_partition ti814x_nand_partitions[] = {
       /* All the partition sizes are listed in terms of NAND block size */
       {
               .name           = "U-Boot-min",
               .offset         = 0,    /* Offset = 0x0 */
               .size           = SZ_128K,
               .mask_flags     = MTD_WRITEABLE,        /* force read-only */
       },
       {
               .name           = "U-Boot",
               .offset         = MTDPART_OFS_APPEND,   /* Offset = 0x0 + 128K */
               .size           = 18 * SZ_128K,
               .mask_flags     = MTD_WRITEABLE,        /* force read-only */
       },
       {
               .name           = "U-Boot Env",
               .offset         = MTDPART_OFS_APPEND,   /* Offset = 0x260000 */
               .size           = 1 * SZ_128K,
       },
       {
               .name           = "Kernel",
               .offset         = MTDPART_OFS_APPEND,   /* Offset = 0x280000 */
               .size           = 34 * SZ_128K,
       },
       {
               .name           = "File System",
               .offset         = MTDPART_OFS_APPEND,   /* Offset = 0x6C0000 */
               .size           = 1601 * SZ_128K,
       },
       {
               .name           = "Reserved",
               .offset         = MTDPART_OFS_APPEND,   /* Offset = 0xCEE0000 */
               .size           = MTDPART_SIZ_FULL,
       },

};


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

int __init ti_ahci_register(u8 num_inst);

static void __init ti8148_evm_init(void)
{
	omap_serial_init();

	board_nand_init(ti814x_nand_partitions,
		ARRAY_SIZE(ti814x_nand_partitions), 0);

	omap2_hsmmc_init(mmc);

	/* initialize usb */
	usb_musb_init(&musb_board_data);

	/* register ahci interface for 1 SATA ports */
	ti_ahci_register(1);

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
