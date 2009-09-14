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

#include <media/davinci/vpfe_capture.h>
#include <media/tvp514x-sd.h>

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
	omap_register_i2c_bus(1, 400, NULL, 0);
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);

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

static struct platform_device omap3517_evm_dss_device = {
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
	.num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
	.sub_devs = vpfe_sub_devs,
	.card_name = "OMAP3517 EVM",
	.ccdc = "DM6446 CCDC",
	.num_clocks = 2,
	.clocks = {"vpfe_ck", "vpfe_pck"},
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

static void __init omap3517_evm_init(void)
{
	omap3517_evm_i2c_init();

	platform_add_devices(omap3517_evm_devices,
				ARRAY_SIZE(omap3517_evm_devices));

	omap_serial_init();
	omap3517_evm_ethernet_init();

	usb_musb_init();
	/* Setup EHCI phy reset padconfig for port1 using GPIO57 */
	omap_cfg_reg(N5_3517_GPIO57_OUT);
	usb_ehci_init(&ehci_pdata);

	omap3517_evm_display_init();
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
