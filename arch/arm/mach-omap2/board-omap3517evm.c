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

extern int oma35x_pmic_init(void);

#include "mmc-twl4030.h"
/*
 * UART
 */
static struct omap_uart_config omap3517_evm_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

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
 * DSS2
 */
#define LCD_BACKLIGHT_PWR	182
/*
 * TODO: Need to define depending on Schematics
 */
#define LCD_PANEL_PWR		0

static int lcd_enabled;
static int dvi_enabled;

static void __init omap3517_evm_display_init(void)
{
	int r;

	/*
	 * Enable GPIO 182 = LCD Backlight Power
	 */
	r = gpio_request(LCD_BACKLIGHT_PWR, "lcd_backlight_pwr");
	if (r) {
		printk(KERN_ERR "failed to get LCD_BACKLIGHT_PWR\n");
		return;
	}
	/*
	 * Assumption: This pin is being interfaced to some LED driver
	 * (TPS61048).
	 * 	1 - enable
	 *	0 - disable
	 */
	gpio_direction_output(LCD_BACKLIGHT_PWR, 1);


	/*
	 * TODO: PWM feature, same goes to backlight driver
	 *
	 *	- Need interface details of gptx_pwm_evt/LCD_PWM0 pin
	 */

	/*
	 * TODO: LCD panel power
	 * Currently this pin is floating, not connected to any 3517/05 pins
	 */
	r = gpio_request(LCD_PANEL_PWR, "lcd_panel_pwr");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_PWR\n");
		goto err_1;
	}
	/*
	 * TODO: Verify the polarity of signal depending on LCD panel
	 * connected
 */
	gpio_direction_output(LCD_PANEL_PWR, 1);

	/*
	 * TODO: DVI/HDMI (TFP410) interface control
	 */
	printk("Display initialized successfully\n");
	return;

err_1:
	gpio_free(LCD_BACKLIGHT_PWR);

}

static int omap3517_evm_panel_enable_lcd(struct omap_display *display)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}
	gpio_direction_output(LCD_PANEL_PWR, 1);
	lcd_enabled = 1;
	return 0;
}

static void omap3517_evm_panel_disable_lcd(struct omap_display *display)
{
	gpio_direction_output(LCD_PANEL_PWR, 0);
	lcd_enabled = 0;
}

/*
 * TODO: Need to change depending on LCD Panel being used
 */
static struct omap_dss_display_config omap3517_evm_display_data = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.panel_name		= "sharp-ls037v7dw01",
	.u.dpi.data_lines 	= 18,
	.panel_enable		= omap3517_evm_panel_enable_lcd,
	.panel_disable		= omap3517_evm_panel_disable_lcd,
};

/*
 * TV Out: Only used for OMAP3517TEB Board
 */
static int omap3517_evm_panel_enable_tv(struct omap_display *display)
{
	return 0;
}

static void omap3517_evm_panel_disable_tv(struct omap_display *display)
{
}

static struct omap_dss_display_config omap3517_evm_display_data_tv = {
	.type = OMAP_DISPLAY_TYPE_VENC,
	.name = "tv",
	.u.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
	.panel_enable = omap3517_evm_panel_enable_tv,
	.panel_disable = omap3517_evm_panel_disable_tv,
};


static int omap3517_evm_panel_enable_dvi(struct omap_display *display)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}
	dvi_enabled = 1;

	return 0;
}

static void omap3517_evm_panel_disable_dvi(struct omap_display *display)
{
	dvi_enabled = 0;
}

static struct omap_dss_display_config omap3517_evm_display_data_dvi = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.panel_name		= "panel-generic",
	.u.dpi.data_lines	= 24,
	.panel_enable		= omap3517_evm_panel_enable_dvi,
	.panel_disable		= omap3517_evm_panel_disable_dvi,
};

static struct omap_dss_board_info omap3517_evm_dss_data = {
	.num_displays = 3,
	.displays = {
		&omap3517_evm_display_data,
		&omap3517_evm_display_data_tv,
		&omap3517_evm_display_data_dvi,
	}
};
static struct platform_device omap3517_evm_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev            = {
		.platform_data = &omap3517_evm_dss_data,
	},
};

/*
 * Board initialization
 */
static struct omap_board_config_kernel omap3517_evm_config[] __initdata = {
	{ OMAP_TAG_UART,	&omap3517_evm_uart_config },
};

static struct platform_device *omap3517_evm_devices[] __initdata = {
	&omap3517_evm_dss_device,
};

static void __init omap3517_evm_init_irq(void)
{
	omap2_init_common_hw(NULL);
	omap_init_irq();
	omap_gpio_init();
}
/*
 * TODO: Need to change the GPIO pins for
 *	Card Detect:
 *	Write Protect:
 */
static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc            = 1,
		.wires          = 4,
		/*TODO: Need to change*/
		.gpio_cd	= 63,
		.gpio_wp	= 63,
	},
	{}      /* Terminator */
};

static void __init omap3517_evm_init(void)
{
	/* Initialize the PMIC */
	oma35x_pmic_init();
	
	omap3517_evm_i2c_init();

	platform_add_devices(omap3517_evm_devices,
				ARRAY_SIZE(omap3517_evm_devices));

	omap_board_config = omap3517_evm_config;
	omap_board_config_size = ARRAY_SIZE(omap3517_evm_config);

	omap_serial_init();
	omap3517_evm_ethernet_init();
	omap3517_evm_display_init();

	omap3517evm_flash_init();
	usb_musb_init();
	/*
	 * MMC init function
	 */
	twl4030_mmc_init(mmc);
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

