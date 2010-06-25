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
#include <linux/err.h>
#include <linux/clk.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/cacheflush.h>
#include <asm/fiq.h>

#include <plat/mcspi.h>
#include <plat/irqs.h>
#include <plat/mux.h>
#include <plat/usb.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/timer-gp.h>
#include <plat/asp.h>
#include <plat/mmc.h>
#include <plat/dmtimer.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
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

#define NAND_BLOCK_SIZE		SZ_128K

static struct mtd_partition ti816x_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "U-Boot",
		.offset		= 0,	/* Offset = 0x0 */
		.size		= 19 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 34 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x6C0000 */
		.size		= 1601 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Reserved",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0xCEE0000 */
		.size		= MTDPART_SIZ_FULL,
	},

};

static struct omap_nand_platform_data ti816x_nand_data = {
	.options	= NAND_BUSWIDTH_16,
	.parts		= ti816x_nand_partitions,
	.nr_parts	= ARRAY_SIZE(ti816x_nand_partitions),
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.nand_setup	= NULL,
	.dev_ready	= NULL,
};

static struct resource ti816x_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device ti816x_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &ti816x_nand_data,
	},
	.num_resources	= 1,
	.resource	= &ti816x_nand_resource,
};


#if 0
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
		.wires		= 4,
		.gpio_cd	= -EINVAL,/* Dedicated pins for CD and WP */
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	},
	{}	/* Terminator */
};

static struct omap2_mcspi_device_config w25x32_mcspi_config = {
    .turbo_mode = 0,
    .single_channel = 1,    /* 0: slave, 1: master */
};

static struct flash_platform_data w25x32_flash_data = {
	.type			= "w25x32",
};

static struct spi_board_info ti8168_evm_spi_info[] __initconst = {
	{
		.modalias       = "w25x32",
		.max_speed_hz   = 10 * 1000 * 1000,
		.bus_num        = 0,
		.chip_select    = 0,
		.mode           = SPI_MODE_0,
		.controller_data = &w25x32_mcspi_config,
		.platform_data = &w25x32_flash_data,
		.irq		= TI816X_IRQ_SPI,
	},
};

#endif

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


static struct at24_platform_data eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
};


/* FIX ME: Check the address of I2C expander */

static struct i2c_board_info __initdata ti816x_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("eeprom", 0x50),
		.platform_data	= &eeprom_info,
	},
	{
		I2C_BOARD_INFO("cpld", 0x23),
	},
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
	{
		I2C_BOARD_INFO("IO Expander", 0x20),
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

#if 0
static u8 ti8168_iis_serializer_direction[] = {
	RX_MODE,	TX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data ti8168_evm_snd_data = {
	.tx_dma_offset = 0x2000,
	.rx_dma_offset = 0x2000,
	.op_mode = DAVINCI_MCASP_IIS_MODE,
	.num_serializer = ARRAY_SIZE(ti8168_iis_serializer_direction),
	.tdm_slots = 2,
	.serial_dir = ti8168_iis_serializer_direction,
	.eventq_no = EVENTQ_3,
	.version = MCASP_VERSION_2,
	.txnumevt = 1,
	.rxnumevt = 1,
};
#endif

/*
 * dmtimer FIQ block: Use DMTIMER3 to generate FIQ every __us.
 */
#define ARM_FIQ_VEC	0xffff001c
#define FIQ_HDLR_SIZE	(12*4)		/* FIXME */


/**
 * dmtimer_setup() - Setup specified timer in periodic mode with given rate
 */
static int dmtimer_setup(int dmtimer_id, struct omap_dm_timer *dmtimer, unsigned
			int rate)
{
	u32 clk_rate, period;
	int src = OMAP_TIMER_SRC_SYS_CLK;

	WARN(IS_ERR_VALUE(omap_dm_timer_set_source(dmtimer, src)),
			"dmtimer_setup: set_source() failed\n");

	clk_rate = clk_get_rate(omap_dm_timer_get_fclk(dmtimer));

	pr_info("OMAP clockevent source: DMTIMER%d at %u Hz\n",
			dmtimer_id, clk_rate);

	period = clk_rate / rate;

	omap_dm_timer_set_int_enable(dmtimer, OMAP_TIMER_INT_OVERFLOW);
	omap_dm_timer_set_load_start(dmtimer, 1, 0xffffffff - period);

	return 0;
}

#define INTC_MIR_CLEAR0		0x0088
#define INTC_CONTROL		0x0048
#define INTC_ILR(irq)		(0x100 + (irq * 4))

#define INTC_FIQ_RST_BIT	BIT(1)
#define INTC_MAP_FIQ_BIT	BIT(0)

/**
 * intc_fiq_setup() - Hook up specified interrupt to FIQ
 */
static int intc_fiq_setup(int irq)
{
	void __iomem *base = TI816X_L4_SLOW_IO_ADDRESS(TI816X_ARM_INTC_BASE);
	int offset = irq & (~(32 - 1));

	__raw_writel(INTC_MAP_FIQ_BIT, base + INTC_ILR(irq));

	irq &= (32 - 1);
	__raw_writel((1 << irq), base + INTC_MIR_CLEAR0 + offset);

	return 0;
}

/**
 * dmtimer_fiq_hdlr() - FIQ handler installed @FIQ VECTOR
 *
 * Note1: Uses __attribute__ ((naked)) mainly to prevent clobber
 * Note2: Ignores interrupt status, assumes dmtimer4 as source
 * Note3: Forcing to use banked FIQ registers to avoid context save
 */
static void __naked dmtimer_fiq_hdlr(void)
{
#if 0
	volatile register unsigned int *reg1 asm("r8");
	volatile register unsigned int *reg2 asm("r11");

	reg1 = TI816X_L4_SLOW_IO_ADDRESS(0x48042000) + (0x28 & 0xff);
	reg2 = TI816X_L4_SLOW_IO_ADDRESS(TI816X_ARM_INTC_BASE) + INTC_CONTROL;
#endif

	/* FIXME: Assuming dmtimer4 in non-posted mode */

	asm __volatile__ (
		/*"ldr r9, [%[reg1]];\n\t"*/
		"mov r12, #0xfa000000\n\t"
		"orr r9, r12, #0x00040000\n\t"
		"orr r9, r9, #0x00002000\n\t"
		"orr r9, r9, #0x00000028\n\t"
		"mov r10, #2;\n\t"
		"str r10, [r9];\n\t"
		/*:: [reg1] "r" (reg1)*/
		);

	asm __volatile__ (
		/*"ldr r9, [%[reg2]];\n\t"*/
		"orr r9, r12, #0x00200000\n\t"
		"orr r9, r9, #0x00000048\n\t"
		"mov r10, #2;\n\t"
		"str r10, [r9];\n\t"
		/*:: [reg2] "r" (reg2)*/
		);

	asm __volatile__ (
		"subs pc, lr, #4\n\t"	/* FIXME */
		);
}

/**
 * axi2ocp_fiq_fixup() - Install and start 125us FIQ
 */
static int axi2ocp_fiq_fixup(void)
{
	struct omap_dm_timer *dmtimer_fiq;

	dmtimer_fiq = omap_dm_timer_request_specific(4);
	if (dmtimer_fiq == NULL) {
		pr_err("axi2ocp_fiq_fixup: failed to get dmtimer\n");
		return -1;
	}

	local_fiq_disable();

	intc_fiq_setup(omap_dm_timer_get_irq(dmtimer_fiq));

	/* FIXME: Put actual rate for 125us */
	dmtimer_setup(4, dmtimer_fiq, 10000);

	memcpy((void *)ARM_FIQ_VEC, (void *)dmtimer_fiq_hdlr, FIQ_HDLR_SIZE);
	flush_icache_range(ARM_FIQ_VEC, ARM_FIQ_VEC + FIQ_HDLR_SIZE);

	/*
	 * FIXME: Use CONFIG_FIQ and then init_FIQ(), generic set_fiq_handler(),
	 * set_fiq_regs(), enable_fiq() (take care of avoiding any side effects
	 * though)
	 */

	local_fiq_enable();

	pr_info("axi2ocp_fiq_fixup: FIQ for DMTIMER installed successfully\n");

	return 0;
}

static void __init ti816x_nand_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	u32 gpmc_base_add = TI816X_GPMC_VIRT;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
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
		ti816x_nand_data.cs = nandcs;
		ti816x_nand_data.gpmc_cs_baseaddr = (void *)
			(gpmc_base_add + GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
		ti816x_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (platform_device_register(&ti816x_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}



static void __init ti8168_evm_init(void)
{
	omap_board_config = generic_config;
	omap_board_config_size = ARRAY_SIZE(generic_config);
#if 0
	spi_register_board_info(ti8168_evm_spi_info,
				ARRAY_SIZE(ti8168_evm_spi_info));
#endif

	omap_serial_init();
#if 0
	omap2_hsmmc_init(mmc);
#endif
	/* initialize usb */
	usb_musb_init(&musb_board_data);
	/* register ahci interface for 2 SATA ports */
	ti_ahci_register(2);

	ti816x_evm_i2c_init();
	i2c_add_driver(&ti816xevm_cpld_driver);
	ti816x_nand_init();
#if 0
	if (HAS_NOR)
		platform_device_register(&ti816x_evm_norflash_device);
	ti816x_register_mcasp(0, &ti8168_evm_snd_data);

	/* FIXME: Move further up in initialization sequence */
	pr_info("AXI2OCP: Entering fixup code...\n");
	axi2ocp_fiq_fixup();
	pr_info("AXI2OCP: done with fixup\n");
#endif
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
