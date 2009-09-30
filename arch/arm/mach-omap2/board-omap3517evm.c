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
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/input.h>
#include <linux/leds.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>

#include <linux/spi/spi.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/omap3517.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/mux.h>
#include <mach/usb.h>
#include <mach/common.h>
#include <mach/mcspi.h>
#include <mach/display.h>
#include <mach/emac.h>
#include <mach/gpmc.h>
#include <mach/nand.h>

#include <media/davinci/vpfe_capture.h>
#include <media/tvp514x-sd.h>
#include <linux/can/platform/ti_hecc.h>

#include "mmc-omap3517evm.h"

#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE         SZ_128K

static struct mtd_partition omap3517evm_nand_partitions[] = {
        /* All the partition sizes are listed in terms of NAND block size */
        {
                .name           = "xloader-nand",
                .offset         = 0,
                .size           = 4*(NAND_BLOCK_SIZE),
                .mask_flags     = MTD_WRITEABLE
        },
        {
                .name           = "uboot-nand",
                .offset         = MTDPART_OFS_APPEND,
                .size           = 14*(NAND_BLOCK_SIZE),
                .mask_flags     = MTD_WRITEABLE
        },
        {
                .name           = "params-nand",

                .offset         = MTDPART_OFS_APPEND,
                .size           = 2*(NAND_BLOCK_SIZE)
        },
        {
                .name           = "linux-nand",
                .offset         = MTDPART_OFS_APPEND,
                .size           = 40*(NAND_BLOCK_SIZE)
        },
        {
                .name           = "jffs2-nand",
                .size           = MTDPART_SIZ_FULL,
                .offset         = MTDPART_OFS_APPEND,
        },
};

static struct omap_nand_platform_data omap3517evm_nand_data = {
        .parts          = omap3517evm_nand_partitions,
        .nr_parts       = ARRAY_SIZE(omap3517evm_nand_partitions),
        .nand_setup     = NULL,
        .dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
        .dev_ready      = NULL,
};

static struct resource omap3517evm_nand_resource = {
        .flags          = IORESOURCE_MEM,
};

static struct platform_device omap3517evm_nand_device = {
        .name           = "omap2-nand",
        .id             = 0,
        .dev            = {
                .platform_data  = &omap3517evm_nand_data,
        },
        .num_resources  = 1,
        .resource       = &omap3517evm_nand_resource,
};



extern void omap35x_pmic_init(void);

/*
 * HECC information 
 */

#define OMAP3517_HECC_BASE      0x5C050000
#define INT_3517_HECC_0         24
#define INT_3517_HECC_1         28

static struct resource omap3517_hecc_resources[] = {
        {
                .start  = OMAP3517_HECC_BASE,
                .end    = OMAP3517_HECC_BASE + 0x3FFF,
                .flags  = IORESOURCE_MEM,
        },
        {
                .start  = INT_3517_HECC_0,
                .end    = INT_3517_HECC_0,
                .flags  = IORESOURCE_IRQ,
        },
};

static struct platform_device omap3517_hecc_device = {
        .name           = "ti_hecc",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(omap3517_hecc_resources),
        .resource       = omap3517_hecc_resources,
};

#define OMAP3517_HECC_SCC_HECC_OFFSET   0
#define OMAP3517_HECC_SCC_RAM_OFFSET    0x3000
#define OMAP3517_HECC_RAM_OFFSET        0x3000
#define OMAP3517_HECC_MBOX_OFFSET       0x2000
#define OMAP3517_HECC_INT_LINE          0
#define OMAP3517_HECC_VERSION           1

static struct ti_hecc_platform_data omap3517_evm_hecc_pdata = {
        .scc_hecc_offset        = OMAP3517_HECC_SCC_HECC_OFFSET,
        .scc_ram_offset         = OMAP3517_HECC_SCC_RAM_OFFSET,
        .hecc_ram_offset        = OMAP3517_HECC_RAM_OFFSET,
        .mbox_offset            = OMAP3517_HECC_MBOX_OFFSET,
        .int_line               = OMAP3517_HECC_INT_LINE,
        .version                = OMAP3517_HECC_VERSION,
};

static void omap3517_evm_hecc_init(struct ti_hecc_platform_data *pdata)
{
        omap3517_hecc_device.dev.platform_data = pdata;
        platform_device_register(&omap3517_hecc_device);
}

/*
 * Ethernet
 */


#define OMAP3517_EVM_PHY_MASK          (0xF)
#define OMAP3517_EVM_MDIO_FREQUENCY    (1000000) /*PHY bus frequency */

static struct emac_platform_data omap3517_evm_emac_pdata = {
        .phy_mask       = OMAP3517_EVM_PHY_MASK,
        .mdio_max_freq  = OMAP3517_EVM_MDIO_FREQUENCY,
        .rmii_en        = 1,
};

static struct resource omap3517_emac_resources[] = {
        {
                .start  = OMAP3517_EMAC_BASE,
                .end    = OMAP3517_EMAC_BASE + 0x3FFFF,
                .flags  = IORESOURCE_MEM,
        },
        {
                .start  = INT_3517_EMAC_RXTHRESH_IRQ,
                .end    = INT_3517_EMAC_RXTHRESH_IRQ,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .start  = INT_3517_EMAC_RX_IRQ,
                .end    = INT_3517_EMAC_RX_IRQ,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .start  = INT_3517_EMAC_TX_IRQ,
                .end    = INT_3517_EMAC_TX_IRQ,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .start  = INT_3517_EMAC_MISC_IRQ,
                .end    = INT_3517_EMAC_MISC_IRQ,
                .flags  = IORESOURCE_IRQ,
        },
};

static struct platform_device omap3517_emac_device = {
        .name           = "davinci_emac",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(omap3517_emac_resources),
        .resource       = omap3517_emac_resources,
};

void omap3517_evm_ethernet_init(struct emac_platform_data *pdata)
{
	unsigned int regval;

        pdata->ctrl_reg_offset          = OMAP3517_EMAC_CNTRL_OFFSET;
        pdata->ctrl_mod_reg_offset      = OMAP3517_EMAC_CNTRL_MOD_OFFSET;
        pdata->ctrl_ram_offset          = OMAP3517_EMAC_CNTRL_RAM_OFFSET;
        pdata->mdio_reg_offset          = OMAP3517_EMAC_MDIO_OFFSET;
        pdata->ctrl_ram_size            = OMAP3517_EMAC_CNTRL_RAM_SIZE;
        pdata->version                  = EMAC_VERSION_2;
        omap3517_emac_device.dev.platform_data     = pdata;
        platform_device_register(&omap3517_emac_device);
	regval = ioread32(OMAP2_IO_ADDRESS(OMAP3517_IP_SW_RESET));
	regval = regval & (~0x2);
	iowrite32(regval,OMAP2_IO_ADDRESS(OMAP3517_IP_SW_RESET));

	return ;
}

/*
 * I2C
 */
static struct i2c_board_info __initdata omap3517evm_i2c_boardinfo[] = {
	{
	I2C_BOARD_INFO("tlv320aic23", 0x1A),
	},
};

static int __init omap3517_evm_i2c_init(void)
{
	/* I2C bus 1 is getting registered from omap35x_pmic_init */

	omap_register_i2c_bus(2, 400, omap3517evm_i2c_boardinfo,
			ARRAY_SIZE(omap3517evm_i2c_boardinfo));
	omap_register_i2c_bus(3, 400, NULL, 0);

	return 0;
}

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
 * DSS
 */
/*
 * Please note that USB2 port pins (D3 & D6) are muxed with
 * LCD panel Backlight enable and PWM signals, making them mutual exclusive
 * to each other.
 */
#define LCD_PANEL_PWR		176
#define LCD_PANEL_BKLIGHT_PWR	182
#define LCD_PANEL_PWM		181

static int lcd_enabled;
static int dvi_enabled;

static void __init omap3517_evm_display_init(void)
{
#if defined(CONFIG_PANEL_SHARP_LQ043T1DG01) || \
	defined(CONFIG_PANEL_SHARP_LQ043T1DG01_MODULE)
	int r;

	omap_cfg_reg(AA16_34XX_GPI181);
	omap_cfg_reg(AE17_34XX_GPIO182);
	omap_cfg_reg(AE15_34XX_GPIO176);
	/*
	 * Enable GPIO 182 = LCD Backlight Power
	 */
	r = gpio_request(LCD_PANEL_BKLIGHT_PWR, "lcd_backlight_pwr");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_BKLIGHT_PWR\n");
		return;
	}
	gpio_direction_output(LCD_PANEL_BKLIGHT_PWR, 1);

	/*
	 * Enable GPIO 181 = LCD Panel PWM
	 *
	 * We are not supporting configuration range for backlight
	 * driver, so make it high (by default).
	 *
	 */
	r = gpio_request(LCD_PANEL_PWM, "lcd_pwm");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_PWM\n");
		goto err_1;
	}
	gpio_direction_output(LCD_PANEL_PWM, 1);

	/*
	 * Enable GPIO 176 = LCD Panel Power enable pin
	 */
	r = gpio_request(LCD_PANEL_PWR, "lcd_panel_pwr");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_PWR\n");
		goto err_2;
	}
	gpio_direction_output(LCD_PANEL_PWR, 1);

	printk(KERN_INFO "Display initialized successfully\n");
	return;

err_2:
	gpio_free(LCD_PANEL_PWM);
err_1:
	gpio_free(LCD_PANEL_BKLIGHT_PWR);
#endif
}

static int omap3517_evm_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}
	gpio_direction_output(LCD_PANEL_PWR, 1);
	lcd_enabled = 1;
	return 0;
}

static void omap3517_evm_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_direction_output(LCD_PANEL_PWR, 0);
	lcd_enabled = 0;
}

/*
 * TODO: Need to change depending on LCD Panel being used
 */
static struct omap_dss_device omap3517_evm_lcd_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "sharp_lq_panel",
	.phy.dpi.data_lines 	= 16,
	.platform_enable	= omap3517_evm_panel_enable_lcd,
	.platform_disable	= omap3517_evm_panel_disable_lcd,
};

/*
 * TV Out: Only used for OMAP3517TEB Board
 */
static int omap3517_evm_panel_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void omap3517_evm_panel_disable_tv(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device omap3517_evm_tv_device = {
	.type 			= OMAP_DISPLAY_TYPE_VENC,
	.name 			= "tv",
	.driver_name		= "venc",
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable	= omap3517_evm_panel_enable_tv,
	.platform_disable	= omap3517_evm_panel_disable_tv,
};


static int omap3517_evm_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}
	dvi_enabled = 1;

	return 0;
}

static void omap3517_evm_panel_disable_dvi(struct omap_dss_device *dssdev)
{
	dvi_enabled = 0;
}

static struct omap_dss_device omap3517_evm_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "panel-generic",
	.phy.dpi.data_lines	= 16,
	.platform_enable	= omap3517_evm_panel_enable_dvi,
	.platform_disable	= omap3517_evm_panel_disable_dvi,
};

static struct omap_dss_device *omap3517_evm_dss_devices[] = {
	&omap3517_evm_lcd_device,
	&omap3517_evm_tv_device,
	&omap3517_evm_dvi_device,
};

static struct omap_dss_board_info omap3517_evm_dss_data = {
	.num_devices	= ARRAY_SIZE(omap3517_evm_dss_devices),
	.devices	= omap3517_evm_dss_devices,
#if defined(CONFIG_PANEL_SHARP_LQ043T1DG01) || \
		defined(CONFIG_PANEL_SHARP_LQ043T1DG01_MODULE)
	.default_device	= &omap3517_evm_lcd_device,
#else
	.default_device = &omap3517_evm_tv_device,
#endif
};

struct platform_device omap3517_evm_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data	= &omap3517_evm_dss_data,
	},
};

static struct resource vpfe_resources[] = {
	{
		.start          = INT_3517_CCDC_VD0,
		.end            = INT_3517_CCDC_VD0,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = INT_3517_CCDC_VD1,
		.end            = INT_3517_CCDC_VD1,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = OMAP3517_IPSS_VPFE_BASE,
		.end            = OMAP3517_IPSS_VPFE_BASE + 0xffff,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = OMAP3517_LVL_INTR_CLEAR,
		.end            = OMAP3517_LVL_INTR_CLEAR + 0x4,
		.flags          = IORESOURCE_MEM,
	},
};

static u64 vpfe_dma_mask = DMA_BIT_MASK(32);

static struct platform_device vpfe_capture_dev = {
	.name		= CAPTURE_DRV_NAME,
	.id		= -1,
	.num_resources	= ARRAY_SIZE(vpfe_resources),
	.resource	= vpfe_resources,
	.dev = {
		.dma_mask		= &vpfe_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

static void omap3517evm_set_vpfe_config(struct vpfe_config *cfg)
{
	vpfe_capture_dev.dev.platform_data = cfg;
}

#define TVP514X_STD_ALL	(V4L2_STD_NTSC | V4L2_STD_PAL)
/* Inputs available at the TVP5146 */
static struct v4l2_input tvp5146_inputs[] = {
	{
		.index = 0,
		.name = "Composite",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = TVP514X_STD_ALL,
	},
	{
		.index = 1,
		.name = "S-Video",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = TVP514X_STD_ALL,
	},
};

/*
 * this is the route info for connecting each input to decoder
 * ouput that goes to vpfe. There is a one to one correspondence
 * with tvp5146_inputs
 */
static struct vpfe_route tvp5146_routes[] = {
	{
		.input = INPUT_CVBS_VI1A,
		.output = OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
	{
		.input = INPUT_SVIDEO_VI2C_VI1C,
		.output = OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
};

static struct tvp514x_platform_data tvp5146_pdata = {
	.clk_polarity = 0,
	.hs_polarity = 1,
	.vs_polarity = 1
};

static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
		.module_name = TVP514X_MODULE_NAME,
		.grp_id = VPFE_SUBDEV_TVP5146,
		.num_inputs = ARRAY_SIZE(tvp5146_inputs),
		.inputs = tvp5146_inputs,
		.routes = tvp5146_routes,
		.can_route = 1,
		.ccdc_if_params = {
			.if_type = VPFE_BT656,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {
			I2C_BOARD_INFO("tvp5146", 0x5C),
			.platform_data = &tvp5146_pdata,
		},
	},
};

static struct vpfe_config vpfe_cfg = {
	.num_subdevs	= ARRAY_SIZE(vpfe_sub_devs),
	.sub_devs	= vpfe_sub_devs,
	.card_name	= "OMAP3517 EVM",
	.ccdc		= "DM6446 CCDC",
	.num_clocks	= 2,
	.clocks		= {"vpfe_ck", "vpfe_pck"},
	.i2c_adapter_id	= 3,
};
/*
 * Board initialization
 */
static struct omap_board_config_kernel omap3517_evm_config[] __initdata = {
};

static struct platform_device *omap3517_evm_devices[] __initdata = {
	&omap3517_evm_dss_device,
	&vpfe_capture_dev,
};

static void __init omap3517_evm_init_irq(void)
{
	omap_board_config = omap3517_evm_config;
	omap_board_config_size = ARRAY_SIZE(omap3517_evm_config);

	omap2_init_common_hw(NULL, NULL, NULL, NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

static struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
#if defined(CONFIG_PANEL_SHARP_LQ043T1DG01) || \
		defined(CONFIG_PANEL_SHARP_LQ043T1DG01_MODULE)
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
#else
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
#endif
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.chargepump = false,
	.phy_reset  = true,
	.reset_gpio_port[0]  = 57,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};


void __init omap3517evm_flash_init(void)
{
        u8              cs = 0;
        u8              nandcs = GPMC_CS_NUM + 1;
        u32             gpmc_base_add = OMAP34XX_GPMC_VIRT;

        while (cs < GPMC_CS_NUM) {
                u32 ret = 0;
                ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

                if ((ret & 0xC00) == 0x800) {
                        /* Found it!! */
                        if (nandcs > GPMC_CS_NUM)
                                nandcs = cs;
                }
                cs++;
        }
        if ((nandcs > GPMC_CS_NUM)) {
                printk(KERN_INFO "NAND: Unable to find configuration "
                                " in GPMC\n ");
                return;
        }

        if (nandcs < GPMC_CS_NUM) {
                omap3517evm_nand_data.cs        = nandcs;
                omap3517evm_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add +
                                        GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
                omap3517evm_nand_data.gpmc_baseaddr   = (void *) (gpmc_base_add);

                if (platform_device_register(&omap3517evm_nand_device) < 0) {
                        printk(KERN_ERR "Unable to register NAND device\n");
                }
        }
}


static struct omap3517_hsmmc_info mmc[] = {
       {
               .mmc            = 1,
               .wires          = 4,
               /*TODO: Need to change*/
               .gpio_cd        = 127,
               .gpio_wp        = 126,
       },
       {
               .mmc            = 2,
               .wires          = 4,
               /*TODO: Need to change*/
               .gpio_cd        = 128,
               .gpio_wp        = 129,
       },

       {}      /* Terminator */
};


static void __init omap3517_evm_init(void)
{
	omap35x_pmic_init();

	omap3517_evm_i2c_init();

	platform_add_devices(omap3517_evm_devices,
				ARRAY_SIZE(omap3517_evm_devices));

	omap_serial_init();
        omap3517_evm_ethernet_init(&omap3517_evm_emac_pdata);
        omap3517_evm_hecc_init(&omap3517_evm_hecc_pdata);

	usb_musb_init();
	/* Setup EHCI phy reset padconfig for port1 using GPIO57 */
	omap_cfg_reg(N5_3517_GPIO57_OUT);
	usb_ehci_init(&ehci_pdata);

	omap3517evm_flash_init();

	omap3517_evm_display_init();

	/* MMC init function */
	omap3517_mmc_init(mmc);
}

static void __init omap3517_evm_map_io(void)
{
	/* setup input configuration for VPFE input devices */
	omap3517evm_set_vpfe_config(&vpfe_cfg);

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
