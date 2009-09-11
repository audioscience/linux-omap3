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
#include <mach/display.h>

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

/*
 * Board initialization
 */
static struct omap_board_config_kernel omap3517_evm_config[] __initdata = {
};

static struct platform_device *omap3517_evm_devices[] __initdata = {
	&omap3517_evm_dss_device,
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
