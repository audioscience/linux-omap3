/*
 * linux/arch/arm/mach-omap2/board-omap3evm.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/backlight.h>

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c/twl4030.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/gpio.h>
#include <mach/keypad.h>
#include <mach/board.h>
#include <mach/usb-musb.h>
#include <mach/usb-ehci.h>
#include <mach/common.h>
#include <mach/mcspi.h>
#include <mach/mux.h>
#include <mach/display.h>
#include <mach/omap-pm.h>
#include <mach/clock.h>

#include "sdram-micron-mt46h32m32lf-6.h"
#include "twl4030-generic-scripts.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "omap3-opp.h"
#include "board-omap3evm-dc.h"
#include <linux/regulator/machine.h>
#include <linux/smsc911x.h>

#if defined(CONFIG_OMAP3EVM_PR785) && defined(CONFIG_TWL4030_CORE)
#error config err : only one of OMAP3EVM_PR785 or TWL4030_CORE can be defined
#endif

#if defined(CONFIG_TWL4030_CORE)
static int omap3evm_twl_gpio_setup(struct device *dev,
               unsigned gpio, unsigned ngpio);
#endif

static int omap3evm_board_version;

int get_omap3evm_board_rev(void)
{
	return omap3evm_board_version;
}
EXPORT_SYMBOL(get_omap3evm_board_rev);
static void omap3evm_board_rev(void)
{
	void __iomem *ioaddr;
	unsigned int smsc_id;
	/*
	 * The run time detection of EVM revision is done by reading Ethernet
	 * PHY ID -
	 *	GEN_1	= 0x
	 *	GEN_2	= 0x92200000
	 */
	ioaddr = ioremap_nocache(OMAP3EVM_ETHR_START + 0x50, 0x4);
	smsc_id = readl(ioaddr) & 0xFFFF0000;
	iounmap(ioaddr);

	switch (smsc_id) {
	/*SMSC9115 chipset*/
	case 0x01150000:
		omap3evm_board_version = OMAP3EVM_BOARD_GEN_1;
		break;
	/*SMSC 9220 chipset*/
	case 0x92200000:
	default:
		omap3evm_board_version = OMAP3EVM_BOARD_GEN_2;
	}
}

static struct resource omap3evm_smc911x_resources[] = {
	[0] =	{
		.start  = OMAP3EVM_ETHR_START,
		.end    = (OMAP3EVM_ETHR_START + OMAP3EVM_ETHR_SIZE - 1),
		.flags  = IORESOURCE_MEM,
	},
	[1] =	{
		.start  = OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
		.end    = OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct smsc911x_platform_config smsc911x_config = {
        .phy_interface  = PHY_INTERFACE_MODE_MII,
        .irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
        .irq_type       = SMSC911X_IRQ_TYPE_OPEN_DRAIN,
        .flags          = SMSC911X_USE_32BIT,
};



static struct platform_device omap3evm_smc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap3evm_smc911x_resources),
	.resource	= &omap3evm_smc911x_resources [0],
	.dev  = {
		  .platform_data = &smsc911x_config,
	},
};

static inline void __init omap3evm_init_smc911x(void)
{
	int eth_cs;
	struct clk *l3ck;
	unsigned int rate;

	eth_cs = OMAP3EVM_SMC911X_CS;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (gpio_request(OMAP3EVM_ETHR_GPIO_IRQ, "SMC911x irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc911x IRQ\n",
			OMAP3EVM_ETHR_GPIO_IRQ);
		return;
	}

	gpio_direction_input(OMAP3EVM_ETHR_GPIO_IRQ);
}

static struct omap_uart_config omap3_evm_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

#if defined(CONFIG_TWL4030_CORE)
static struct twl4030_gpio_platform_data omap3evm_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
        .pulldowns      = BIT(2) | BIT(6) | BIT(8) | BIT(13)
                                | BIT(16) | BIT(17),
        .setup          = omap3evm_twl_gpio_setup,

};

static struct twl4030_usb_data omap3evm_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static int omap3evm_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_A),
	KEY(0, 3, KEY_B),
	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_E),
	KEY(1, 3, KEY_F),
	KEY(2, 0, KEY_ENTER),
	KEY(2, 1, KEY_I),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_K),
	KEY(3, 0, KEY_M),
	KEY(3, 1, KEY_N),
	KEY(3, 2, KEY_O),
	KEY(3, 3, KEY_P)
};

static struct twl4030_keypad_data omap3evm_kp_data = {
	.rows		= 4,
	.cols		= 4,
	.keymap		= omap3evm_keymap,
	.keymapsize	= ARRAY_SIZE(omap3evm_keymap),
	.rep		= 1,
};

static struct twl4030_madc_platform_data omap3evm_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data omap3evm_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.keypad		= &omap3evm_kp_data,
	.madc		= &omap3evm_madc_data,
	.usb		= &omap3evm_usb_data,
	.power		= GENERIC3430_T2SCRIPTS_DATA,
	.gpio		= &omap3evm_gpio_data,
};

static struct i2c_board_info __initdata omap3evm_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &omap3evm_twldata,
	},
};


#endif

#if defined(CONFIG_OMAP3EVM_PR785)
/* CORE voltage regulator */
struct regulator_consumer_supply tps62352_core_consumers = {
       .supply = "vdd2",
};

/* MPU voltage regulator */
struct regulator_consumer_supply tps62352_mpu_consumers = {
       .supply = "vdd1",
};

struct regulator_init_data vdd2_tps_regulator_data = {
               .constraints = {
                       .min_uV = 750000,
                       .max_uV = 1537000,
                       .valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
                               REGULATOR_CHANGE_STATUS),
               },
               .num_consumer_supplies  = 1,
               .consumer_supplies      = &tps62352_core_consumers,
};

struct regulator_init_data vdd1_tps_regulator_data = {
               .constraints = {
                       .min_uV = 750000,
                       .max_uV = 1537000,
                       .valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
                               REGULATOR_CHANGE_STATUS),
               },
               .num_consumer_supplies  = 1,
               .consumer_supplies      = &tps62352_mpu_consumers,
};

static struct i2c_board_info __initdata tps_6235x_i2c_board_info[] = {
       {
               I2C_BOARD_INFO("tps62352", 0x4A),
               .flags = I2C_CLIENT_WAKE,
               .platform_data = &vdd2_tps_regulator_data,
       },
       {
               I2C_BOARD_INFO("tps62353", 0x48),
               .flags = I2C_CLIENT_WAKE,
               .platform_data = &vdd1_tps_regulator_data,
       },
};
#endif


static int __init omap3_evm_i2c_init(void)
{
#if defined(CONFIG_OMAP3EVM_PR785)
       omap_register_i2c_bus(1, 2600, tps_6235x_i2c_board_info,
               ARRAY_SIZE(tps_6235x_i2c_board_info));

#endif
#if defined(CONFIG_TWL4030_CORE)

	omap_register_i2c_bus(1, 2600, omap3evm_i2c_boardinfo,
			ARRAY_SIZE(omap3evm_i2c_boardinfo));
#endif
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}
/*
 * For new frame buffer driver based on DSS2 library
 */
#ifdef CONFIG_OMAP2_DSS

#ifdef CONFIG_FB_OMAP2
static struct resource omap3evm_vout_resource[3 - CONFIG_FB_OMAP2_NUM_FBS] = {
};
#else /* CONFIG_FB_OMAP2 */
static struct resource omap3evm_vout_resource[2] = {
};
#endif /* CONFIG_FB_OMAP2 */
static struct platform_device omap3evm_vout_device = {
	.name			= "omap_vout",
	.num_resources	= ARRAY_SIZE(omap3evm_vout_resource),
	.resource 		= &omap3evm_vout_resource[0],
	.id		= -1,
};

#define LCD_PANEL_LR		2
#define LCD_PANEL_UD		3
#define LCD_PANEL_INI		152
#define LCD_PANEL_ENABLE_GPIO	153
#define LCD_PANEL_QVGA		154
#define LCD_PANEL_RESB		155

#define ENABLE_VDAC_DEDICATED	0x03
#define ENABLE_VDAC_DEV_GRP	0x20
#define ENABLE_VPLL2_DEDICATED	0x05
#define ENABLE_VPLL2_DEV_GRP	0xE0

static int lcd_enabled;
static int dvi_enabled;

static void __init omap3_evm_display_init(void)
{
	int r;
	r = gpio_request(LCD_PANEL_LR, "lcd_panel_lr");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_LR\n");
		return;
	}
	r = gpio_request(LCD_PANEL_UD, "lcd_panel_ud");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_UD\n");
		goto err_1;
	}

	r = gpio_request(LCD_PANEL_INI, "lcd_panel_ini");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_INI\n");
		goto err_2;
	}
	r = gpio_request(LCD_PANEL_RESB, "lcd_panel_resb");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_RESB\n");
		goto err_3;
	}
	r = gpio_request(LCD_PANEL_QVGA, "lcd_panel_qvga");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_QVGA\n");
		goto err_4;
	}

	gpio_direction_output(LCD_PANEL_RESB, 1);
	gpio_direction_output(LCD_PANEL_INI, 1);
	gpio_direction_output(LCD_PANEL_QVGA, 0);
	gpio_direction_output(LCD_PANEL_LR, 1);
	gpio_direction_output(LCD_PANEL_UD, 1);

	return;

err_4:
	gpio_free(LCD_PANEL_RESB);
err_3:
	gpio_free(LCD_PANEL_INI);
err_2:
	gpio_free(LCD_PANEL_UD);
err_1:
	gpio_free(LCD_PANEL_LR);

}

static int omap3_evm_panel_enable_lcd(struct omap_display *display)
{
	if (dvi_enabled) {
		return -EINVAL;
	}
#if defined(CONFIG_TWL4030_CORE)
	if (omap_rev() > OMAP3430_REV_ES1_0) {
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				ENABLE_VPLL2_DEDICATED, TWL4030_PLL2_DEDICATED);
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				ENABLE_VPLL2_DEV_GRP, TWL4030_VPLL2_DEV_GRP);
	}
#endif
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 0);
	lcd_enabled = 1;
	return 0;
}

static void omap3_evm_panel_disable_lcd(struct omap_display *display)
{
#if defined(CONFIG_TWL4030_CORE)
	if (omap_rev() > OMAP3430_REV_ES1_0) {
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x0,
				TWL4030_PLL2_DEDICATED);
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x0,
				TWL4030_VPLL2_DEV_GRP);
	}
#endif
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 1);
	lcd_enabled = 0;
}

static struct omap_display_data omap3_evm_display_data = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.panel_name = "sharp-ls037v7dw01",
	.u.dpi.data_lines = 18,
	.panel_enable = omap3_evm_panel_enable_lcd,
	.panel_disable = omap3_evm_panel_disable_lcd,
};

static int omap3_evm_panel_enable_tv(struct omap_display *display)
{
#if defined(CONFIG_TWL4030_CORE)
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEDICATED, TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEV_GRP, TWL4030_VDAC_DEV_GRP);
#endif
	return 0;
}

static void omap3_evm_panel_disable_tv(struct omap_display *display)
{
#if defined(CONFIG_TWL4030_CORE)
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEV_GRP);
#endif
}

static struct omap_display_data omap3_evm_display_data_tv = {
	.type = OMAP_DISPLAY_TYPE_VENC,
	.name = "tv",
#if defined(CONFIG_OMAP2_VENC_OUT_TYPE_SVIDEO)
	.u.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
#elif defined(CONFIG_OMAP2_VENC_OUT_TYPE_COMPOSITE)
	.u.venc.type = OMAP_DSS_VENC_TYPE_COMPOSITE,
#endif
	.panel_enable = omap3_evm_panel_enable_tv,
	.panel_disable = omap3_evm_panel_disable_tv,
};

static int omap3_evm_panel_enable_dvi(struct omap_display *display)
{
	unsigned char val;

	if (lcd_enabled) {
		return -EINVAL;
	}

#if defined(CONFIG_TWL4030_CORE)
	if (omap_rev() > OMAP3430_REV_ES1_0) {
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VPLL2_DEDICATED, TWL4030_PLL2_DEDICATED);
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VPLL2_DEV_GRP, TWL4030_VPLL2_DEV_GRP);
	}
	twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &val,
			REG_GPIODATADIR1);
	val |= 0x80;
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, val,
			REG_GPIODATADIR1);
	twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &val,
			REG_GPIODATAOUT1);
	val |= 0x80;
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, val,
			REG_GPIODATAOUT1);
#endif
	dvi_enabled = 1;

	return 0;
}

static void omap3_evm_panel_disable_dvi(struct omap_display *display)
{
#if defined(CONFIG_TWL4030_CORE)
	unsigned char val;

	if (omap_rev() > OMAP3430_REV_ES1_0) {
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x0,
				TWL4030_PLL2_DEDICATED);
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x0,
				TWL4030_VPLL2_DEV_GRP);
	}

	twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &val,
			REG_GPIODATADIR1);
	val &= ~0x80;
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, val,
			REG_GPIODATADIR1);
	twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &val,
			REG_GPIODATAOUT1);
	val &= ~0x80;
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, val,
			REG_GPIODATAOUT1);
#endif
	dvi_enabled = 0;
}

static struct omap_display_data omap3_evm_display_data_dvi = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.panel_name = "panel-generic",
	.u.dpi.data_lines = 24,
	.panel_enable = omap3_evm_panel_enable_dvi,
	.panel_disable = omap3_evm_panel_disable_dvi,
};

static struct omap_dss_platform_data omap3_evm_dss_data = {
	.num_displays = 3,
	.displays = {
		&omap3_evm_display_data,
		&omap3_evm_display_data_dvi,
		&omap3_evm_display_data_tv,
	}
};
static struct platform_device omap3_evm_dss_device = {
	.name		= "omap-dss",
	.id		= -1,
	.dev            = {
		.platform_data = &omap3_evm_dss_data,
	},
};
#else /* CONFIG_OMAP2_DSS */
static struct platform_device omap3_evm_lcd_device = {
	.name		= "omap3evm_lcd",
	.id		= -1,
};
static struct omap_lcd_config omap3_evm_lcd_config __initdata = {
	.ctrl_name	= "internal",
};
#endif /* CONFIG_OMAP2_DSS */

static void omap3evm_set_bl_intensity(int intensity)
{
	unsigned char c;

	if (intensity > 100)
		return;

#if defined(CONFIG_TWL4030_CORE)
	/*
	 * Enable LEDA for backlight
	 */
	twl4030_i2c_write_u8(TWL4030_MODULE_LED, 0x11, TWL4030_LED_EN);

	if (get_omap3evm_board_rev() >= OMAP3EVM_BOARD_GEN_2) {
		c = ((125 * (100 - intensity)) / 100) + 1;
		twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, 0x7F,
							TWL4030_LED_PWMOFF);
		twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, c,
							TWL4030_LED_PWMON);
	} else {
		c = ((125 * (100 - intensity)) / 100) + 2;
		twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, 0x1,
							TWL4030_LED_PWMON);
		twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, c,
							TWL4030_LED_PWMOFF);
	}
#endif
}

static struct generic_bl_info omap3evm_bl_platform_data = {
	.name			= "omap3evm-bklight",
	.max_intensity		= 100,
	.default_intensity	= 70,
	.limit_mask		= 0,
	.set_bl_intensity	= omap3evm_set_bl_intensity,
	.kick_battery		= NULL,
};

static struct platform_device omap3evm_bklight_device = {
	.name		= "generic-bl",
	.id		= -1,
	.dev		= {
		.parent		= &omap3_evm_dss_device.dev,
		.platform_data	= &omap3evm_bl_platform_data,
	},
};

static void ads7846_dev_init(void)
{
	if (gpio_request(OMAP3_EVM_TS_GPIO, "ADS7846 pendown") < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");

	gpio_direction_input(OMAP3_EVM_TS_GPIO);

	omap_set_gpio_debounce(OMAP3_EVM_TS_GPIO, 1);
	omap_set_gpio_debounce_time(OMAP3_EVM_TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(OMAP3_EVM_TS_GPIO);
}

struct ads7846_platform_data ads7846_config = {
	.x_max                  = 0x0fff,
	.y_max                  = 0x0fff,
	.x_plate_ohms           = 180,
	.pressure_max           = 255,
	.debounce_max           = 10,
	.debounce_tol           = 3,
	.debounce_rep           = 1,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.settle_delay_usecs     = 150,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,  /* 0: slave, 1: master */
};

struct spi_board_info omap3evm_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(OMAP3_EVM_TS_GPIO),
		.platform_data		= &ads7846_config,
	},
};

static void __init omap3_evm_init_irq(void)
{
	omap2_init_common_hw(mt46h32m32lf6_sdrc_params, omap3_mpu_rate_table,
	                     omap3_dsp_rate_table, omap3_l3_rate_table);
	omap_init_irq();
	omap_gpio_init();
	omap3evm_init_smc911x();
}

static struct omap_board_config_kernel omap3_evm_config[] __initdata = {
	{ OMAP_TAG_UART,	&omap3_evm_uart_config },
#ifndef CONFIG_OMAP2_DSS
	{ OMAP_TAG_LCD,		&omap3_evm_lcd_config },
#endif
};

static struct platform_device *omap3_evm_devices[] __initdata = {
#ifdef CONFIG_OMAP2_DSS
	&omap3_evm_dss_device,
	&omap3evm_vout_device,
#else /* CONFIG_OMAP2_DSS */
	&omap3_evm_lcd_device,
#endif /* CONFIG_OMAP2_DSS */
	&omap3evm_smc911x_device,
	&omap3evm_bklight_device,

};

#if defined(CONFIG_TWL4030_CORE)
static struct twl4030_hsmmc_info mmc[] __initdata = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= 63,
	},
	{}	/* Terminator */
};

static struct gpio_led gpio_leds[] = {
       {
               .name                   = "omap3evm::ledb",
               /* normally not visible (board underside) */
               .default_trigger        = "default-on",
               .gpio                   = -EINVAL,      /* gets replaced */
               .active_low             = true,
       },
};

static struct gpio_led_platform_data gpio_led_info = {
       .leds           = gpio_leds,
       .num_leds       = ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
       .name   = "leds-gpio",
       .id     = -1,
       .dev    = {
               .platform_data  = &gpio_led_info,
       },
};


static int omap3evm_twl_gpio_setup(struct device *dev,
               unsigned gpio, unsigned ngpio)
{
       /* gpio + 0 is "mmc0_cd" (input/IRQ) */
       mmc[0].gpio_cd = gpio + 0;
       twl4030_mmc_init(mmc);

       /* Most GPIOs are for USB OTG.  Some are mostly sent to
        * the P2 connector; notably LEDA for the LCD backlight.
        */

       /* TWL4030_GPIO_MAX + 1 == ledB (out, active low LED) */
       gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

       platform_device_register(&leds_gpio);

       return 0;
}

static void omap_init_twl4030(void)
{
       if (cpu_is_omap343x()) {
               omap_cfg_reg(AF26_34XX_GPIO0);
               omap_cfg_reg(L8_34XX_GPIO63);
       }
}

#define TWL4030_VAUX2_1P8V 0x5
#define ENABLE_VAUX2_DEV_GRP 0x20

/* This is called from twl4030-core.c and is required by
 * MUSB and EHCI on new OMAP3EVM.
 */
void usb_gpio_settings(void)
{
	unsigned char val;

	if (get_omap3evm_board_rev() < OMAP3EVM_BOARD_GEN_2)
		return;

	/* enable VAUX2 for EHCI */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			TWL4030_VAUX2_1P8V, TWL4030_VAUX2_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX2_DEV_GRP, TWL4030_VAUX2_DEV_GRP);

	/* Enable TWL GPIO Module */
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x04, REG_GPIO_CTRL);

	/*
	 * Configure GPIO-6 as output
	 */
	twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &val, REG_GPIODATADIR1);
	val |= 0x4;
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, val, REG_GPIODATADIR1);

	/* Set GPIO6 = 1 */
	twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &val, REG_GPIODATAOUT1);
	val |= 0x40;
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, val, REG_GPIODATAOUT1);

}
EXPORT_SYMBOL(usb_gpio_settings);

#endif

#if defined(CONFIG_OMAP3EVM_PR785)
static void omap_init_pr785(void)
{
       /* Initialize the mux settings for PR785 power module board */
       if (cpu_is_omap343x()) {
               omap_cfg_reg(AF26_34XX_GPIO0);
               omap_cfg_reg(AF22_34XX_GPIO9);
               omap_cfg_reg(AF6_34XX_GPIO140_UP);
               omap_cfg_reg(AE6_34XX_GPIO141);
               omap_cfg_reg(AF5_34XX_GPIO142);
               omap_cfg_reg(AE5_34XX_GPIO143);
       }
}
#endif

static void __init omap3_evm_init(void)
{
	int dec_i2c_id, is_dec_onboard;

	/* Get EVM board version and save it */
	omap3evm_board_rev();

	omap3_evm_i2c_init();

	platform_add_devices(omap3_evm_devices, ARRAY_SIZE(omap3_evm_devices));
	omap_board_config = omap3_evm_config;
	omap_board_config_size = ARRAY_SIZE(omap3_evm_config);

	spi_register_board_info(omap3evm_spi_board_info,
				ARRAY_SIZE(omap3evm_spi_board_info));

	omap_serial_init();
#if defined(CONFIG_TWL4030_CORE)
        omap_init_twl4030();
#endif
#if defined(CONFIG_OMAP3EVM_PR785)
       omap_init_pr785();
#endif

	usb_musb_init();

	if (get_omap3evm_board_rev() >= OMAP3EVM_BOARD_GEN_2) {
		/* enable EHCI VBUS using GPIO22 */
		omap_cfg_reg(AF9_34XX_GPIO22);
		gpio_request(22, "enable EHCI VBUS");
		gpio_direction_output(22, 0);
		gpio_set_value(22, 1);

		/* enable 1.8V using GPIO61 */
		omap_cfg_reg(U3_34XX_GPIO61);
		gpio_request(61, "enable 1.8V for EHCI");
		gpio_direction_output(61, 0);
		gpio_set_value(61, 0);

		/* setup EHCI phy reset config */
		omap_cfg_reg(AH14_34XX_GPIO21);
		omap3_ehci_phy_reset_gpio = 21;

		/* enable MUSB VBUS */
	#if 0
		/* Don't enable GPIO based VBUS when MUSB
		 * PHY is programmed to use EXT VBUS
		 */
		omap_cfg_reg(Y21_34XX_GPIO156);
		gpio_request(156, "enable MUSB VBUS");
		gpio_direction_output(156, 0);
		gpio_set_value(156, 1);
	#endif
	} else {
		/* setup EHCI phy reset on MDC */
		omap_cfg_reg(AF4_34XX_GPIO135);
		omap3_ehci_phy_reset_gpio = 135;
	}
	usb_ehci_init();
	omap3evm_flash_init();
	ads7846_dev_init();
#ifdef CONFIG_OMAP2_DSS
	omap3_evm_display_init();
#endif /* CONFIG_OMAP2_DSS */

	if (get_omap3evm_board_rev() >= OMAP3EVM_BOARD_GEN_2) {
		dec_i2c_id = 0x5C;
		is_dec_onboard = 1;
	} else {
		dec_i2c_id = 0x5D;
		is_dec_onboard = 0;
	}
	omap3evmdc_init(is_dec_onboard, 3, dec_i2c_id);
}

static void __init omap3_evm_map_io(void)
{
	omap2_set_globals_35xx();
	omap2_map_common_io();
}

MACHINE_START(OMAP3EVM, "OMAP3 EVM")
	/* Maintainer: Syed Mohammed Khasim - Texas Instruments */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_evm_map_io,
	.init_irq	= omap3_evm_init_irq,
	.init_machine	= omap3_evm_init,
	.timer		= &omap_timer,
MACHINE_END
