/*
 * linux/arch/arm/mach-omap2/board-ldp-flash.c
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-2430sdp-flash.c
 * Author: Rohit Choraria <rohitkc@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/types.h>
#include <linux/io.h>

#include <asm/mach/flash.h>
#include <asm/arch/board.h>
#include <asm/arch/gpmc.h>
#include <asm/arch/nand.h>

#define GPMC_CS0_BASE	0x60
#define GPMC_CS_SIZE	0x30

static struct mtd_partition ldp_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader-NAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Boot Env-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x100000 */
		.size		= 2 * (64 * 2048),
	},
	{
		.name		= "Kernel-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x140000 */
		.size		= 32 * (64 * 2048),
	},
	{
		.name		= "File System - NAND",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x540000 */
	},
};

/* NAND chip access: 16 bit */
static struct omap_nand_platform_data ldp_nand_data = {
	.parts		= ldp_nand_partitions,
	.nr_parts	= ARRAY_SIZE(ldp_nand_partitions),
	.nand_setup	= NULL,
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.dev_ready	= NULL,
};

static struct resource ldp_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device ldp_nand_device = {
	.name		= "omap2-nand",
	.id		= 0,
	.dev		= {
	.platform_data	= &ldp_nand_data,
	},
	.num_resources	= 1,
	.resource	= &ldp_nand_resource,
};

void __init ldp_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;
	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}
	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				"in GPMC\n ");
		return;
	}
	if (nandcs < GPMC_CS_NUM) {
		ldp_nand_data.cs = nandcs;
		ldp_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add +
					GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
		ldp_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

		if (platform_device_register(&ldp_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}
