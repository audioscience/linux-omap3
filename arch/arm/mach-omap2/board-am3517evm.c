/*
 * linux/arch/arm/mach-omap2/board-am3517evm.c
 *
 * Copyright (C) 2009 Texas Instruments Incorporated
 * Authot: Ranjith Lohithakshan <ranjithl@ti.com>
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
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/display.h>

#include <media/ti-media/vpfe_capture.h>
#include <media/tvp514x.h>

/*
 * VPFE - Video Decoder interface
 */
#define TVP514X_STD_ALL	(V4L2_STD_NTSC | V4L2_STD_PAL)
/* Inputs available at the TVP5146 */
static struct v4l2_input tvp5146_inputs[] = {
	{
		.index	= 0,
		.name	= "Composite",
		.type	= V4L2_INPUT_TYPE_CAMERA,
		.std	= TVP514X_STD_ALL,
	},
	{
		.index = 1,
		.name = "S-Video",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = TVP514X_STD_ALL,
	},
};

static struct vpfe_route tvp5146_routes[] = {
	{
		.input	= INPUT_CVBS_VI1A,
		.output	= OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
	{
		.input	= INPUT_SVIDEO_VI2C_VI1C,
		.output	= OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
};

static struct tvp514x_platform_data tvp5146_pdata = {
	.clk_polarity = 0,
	.hs_polarity = 1,
	.vs_polarity = 1
};

static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
		.module_name	= TVP514X_MODULE_NAME,
		.grp_id		= VPFE_SUBDEV_TVP5146,
		.num_inputs = ARRAY_SIZE(tvp5146_inputs),
		.inputs = tvp5146_inputs,
		.routes = tvp5146_routes,
		.can_route = 1,
		.ccdc_if_params = {
			.if_type	= VPFE_BT656_10BIT,
			.hdpol		= VPFE_PINPOL_POSITIVE,
			.vdpol		= VPFE_PINPOL_POSITIVE,
		},
		.board_info = {
			I2C_BOARD_INFO("tvp5146", 0x5C),
			.platform_data	= &tvp5146_pdata,
		},
	},
};

static void am3517_evm_clear_vpfe_intr(int vdint)
{
	unsigned int vpfe_int_clr;

	vpfe_int_clr = omap_ctrl_readl(OMAP3517_CONTROL_LVL_INTR_CLEAR);

	switch (vdint) {
	/* VD0 interrrupt */
	case INT_3517_CCDC_VD0_IRQ:
		vpfe_int_clr &= ~AM3517_VPFE_VD0_INT_CLR;
		vpfe_int_clr |= AM3517_VPFE_VD0_INT_CLR;
		break;
		/* VD1 interrrupt */
	case INT_3517_CCDC_VD1_IRQ:
		vpfe_int_clr &= ~AM3517_VPFE_VD1_INT_CLR;
		vpfe_int_clr |= AM3517_VPFE_VD1_INT_CLR;
		break;
		/* VD2 interrrupt */
	case INT_3517_CCDC_VD2_IRQ:
		vpfe_int_clr &= ~AM3517_VPFE_VD2_INT_CLR;
		vpfe_int_clr |= AM3517_VPFE_VD2_INT_CLR;
		break;
		/* Clear all interrrupts */
	default:
		vpfe_int_clr &= ~(AM3517_VPFE_VD0_INT_CLR |
				AM3517_VPFE_VD1_INT_CLR |
				AM3517_VPFE_VD2_INT_CLR);
		vpfe_int_clr |= (AM3517_VPFE_VD0_INT_CLR |
				AM3517_VPFE_VD1_INT_CLR |
				AM3517_VPFE_VD2_INT_CLR);
		break;
	}

	omap_ctrl_writel(vpfe_int_clr, OMAP3517_CONTROL_LVL_INTR_CLEAR);
	vpfe_int_clr = omap_ctrl_readl(OMAP3517_CONTROL_LVL_INTR_CLEAR);
}

static struct resource vpfe_resources[] = {
	{
		.start          = INT_3517_CCDC_VD0_IRQ,
		.end            = INT_3517_CCDC_VD0_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start          = INT_3517_CCDC_VD1_IRQ,
		.end            = INT_3517_CCDC_VD1_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start          = AM3517_IPSS_VPFE_BASE,
		.end            = AM3517_IPSS_VPFE_BASE + 0xffff,
		.flags		= IORESOURCE_MEM,
	},

	{
		.start          = OMAP3517_CONTROL_LVL_INTR_CLEAR,
		.end            = OMAP3517_CONTROL_LVL_INTR_CLEAR + 0x4,
		.flags		= IORESOURCE_MEM,
	},
};

static struct vpfe_config vpfe_cfg = {
	.num_subdevs	= ARRAY_SIZE(vpfe_sub_devs),
	.sub_devs	= vpfe_sub_devs,
	.card_name	= "AM3517 EVM",
	.ccdc		= "DM6446 CCDC",
	.clr_intr	= am3517_evm_clear_vpfe_intr,
	.num_clocks	= 2,
	.clocks		= {"vpfe_ck", "vpfe_pck"},
	.i2c_adapter_id	= 3,
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
		.platform_data		= &vpfe_cfg,
	},
};

/*
 * I2C
 */
static struct i2c_board_info __initdata am3517evm_i2c_boardinfo[] = {
	{
	I2C_BOARD_INFO("tlv320aic23", 0x1A),
	},
};

static int __init am3517_evm_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, NULL, 0);
	omap_register_i2c_bus(2, 400, am3517evm_i2c_boardinfo,
			ARRAY_SIZE(am3517evm_i2c_boardinfo));
	omap_register_i2c_bus(3, 400, NULL, 0);

	return 0;
}


#define LCD_PANEL_PWR		176
#define LCD_PANEL_BKLIGHT_PWR	182
#define LCD_PANEL_PWM		181

static int lcd_enabled;
static int dvi_enabled;

static void __init am3517_evm_display_init(void)
{
	int r;

	/*
	 * Enable GPIO 182 = LCD Backlight Power
	 */
	r = gpio_request(LCD_PANEL_BKLIGHT_PWR, "lcd_backlight_pwr");
	if (r) {
		printk(KERN_ERR "failed to get lcd_backlight_pwr\n");
		return;
	}
	gpio_direction_output(LCD_PANEL_BKLIGHT_PWR, 1);
	/*
	 * Enable GPIO 181 = LCD Panel PWM
	 */
	r = gpio_request(LCD_PANEL_PWM, "lcd_pwm");
	if (r) {
		printk(KERN_ERR "failed to get lcd_pwm\n");
		goto err_1;
	}
	gpio_direction_output(LCD_PANEL_PWM, 1);
	/*
	 * Enable GPIO 176 = LCD Panel Power enable pin
	 */
	r = gpio_request(LCD_PANEL_PWR, "lcd_panel_pwr");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_pwr\n");
		goto err_2;
	}
	gpio_direction_output(LCD_PANEL_PWR, 1);

	printk(KERN_INFO "Display initialized successfully\n");
	return;

err_2:
	gpio_free(LCD_PANEL_PWM);
err_1:
	gpio_free(LCD_PANEL_BKLIGHT_PWR);
}

static int am3517_evm_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}
	gpio_set_value(LCD_PANEL_PWR, 1);
	lcd_enabled = 1;

	return 0;
}

static void am3517_evm_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_PANEL_PWR, 0);
	lcd_enabled = 0;
}

static struct omap_dss_device am3517_evm_lcd_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "sharp_lq_panel",
	.phy.dpi.data_lines 	= 16,
	.platform_enable	= am3517_evm_panel_enable_lcd,
	.platform_disable	= am3517_evm_panel_disable_lcd,
};

static int am3517_evm_panel_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void am3517_evm_panel_disable_tv(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device am3517_evm_tv_device = {
	.type 			= OMAP_DISPLAY_TYPE_VENC,
	.name 			= "tv",
	.driver_name		= "venc",
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable	= am3517_evm_panel_enable_tv,
	.platform_disable	= am3517_evm_panel_disable_tv,
};

static int am3517_evm_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}
	dvi_enabled = 1;

	return 0;
}

static void am3517_evm_panel_disable_dvi(struct omap_dss_device *dssdev)
{
	dvi_enabled = 0;
}

static struct omap_dss_device am3517_evm_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.phy.dpi.data_lines	= 24,
	.platform_enable	= am3517_evm_panel_enable_dvi,
	.platform_disable	= am3517_evm_panel_disable_dvi,
};

static struct omap_dss_device *am3517_evm_dss_devices[] = {
	&am3517_evm_lcd_device,
	&am3517_evm_tv_device,
	&am3517_evm_dvi_device,
};

static struct omap_dss_board_info am3517_evm_dss_data = {
	.num_devices	= ARRAY_SIZE(am3517_evm_dss_devices),
	.devices	= am3517_evm_dss_devices,
	.default_device	= &am3517_evm_lcd_device,
};

struct platform_device am3517_evm_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data	= &am3517_evm_dss_data,
	},
};
/*
 * Board initialization
 */
static struct omap_board_config_kernel am3517_evm_config[] __initdata = {
};

static struct platform_device *am3517_evm_devices[] __initdata = {
	&am3517_evm_dss_device,
	&vpfe_capture_dev,
};

static void __init am3517_evm_init_irq(void)
{
	omap_board_config = am3517_evm_config;
	omap_board_config_size = ARRAY_SIZE(am3517_evm_config);

	omap2_init_common_hw(NULL, NULL, NULL, NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

static void __init am3517_evm_init(void)
{
	am3517_evm_i2c_init();

	platform_add_devices(am3517_evm_devices,
				ARRAY_SIZE(am3517_evm_devices));
	omap_serial_init();

	am3517_evm_display_init();

	usb_musb_init();
}

static void __init am3517_evm_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP3517EVM, "OMAP3517/AM3517 EVM")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= am3517_evm_map_io,
	.init_irq	= am3517_evm_init_irq,
	.init_machine	= am3517_evm_init,
	.timer		= &omap_timer,
MACHINE_END
