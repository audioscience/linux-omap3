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
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/leds.h>
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
#include <plat/led.h>
#include <plat/mmc.h>
#include <plat/gpmc.h>
#include <plat/ti81xx_ram.h>
#include <mach/board-ti814x.h>

#include "board-flash.h"
#include "clock.h"
#include "mux.h"
#include "hsmmc.h"
#include "control.h"

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{.reg_offset = OMAP_MUX_TERMINATOR},
};
#else
#define board_mux     NULL
#endif

static struct i2c_board_info __initdata asi1230_i2c_boardinfo[] = {
	{
	 I2C_BOARD_INFO("lm75", 0x4B),
	 },
};

static void __init asi1230_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, asi1230_i2c_boardinfo, /* note 400kHz bus speed */
			      ARRAY_SIZE(asi1230_i2c_boardinfo));
}

#ifdef CONFIG_MACH_TI8148EVM
#error Cannot compile ASI1230 support together with TI8148EVM support
/* The following two functions must be defined when TI8148EVM support is not configured in
 * or usb-ehci.c (!!!) will be missing symbols. The problem goes away after tweaking Makefiles
 * to exclude USB support when deselected but a proper fix would require refactoring
 * EVM and platform code.
int vps_ti814x_select_video_decoder(int vid_decoder_id) { return 0; }
int vps_ti814x_set_tvp7002_filter(enum fvid2_standard standard) { return 0; }
 */
#endif

static struct omap2_hsmmc_info mmc[] = {
	{
	 .mmc = 1,
	 .caps = MMC_CAP_4_BIT_DATA,
	 .gpio_cd = -EINVAL,	/* Dedicated pins for CD and WP */
	 .gpio_wp = -EINVAL,
	 .ocr_mask = MMC_VDD_33_34,
	 },
	{}			/* Terminator */
};
#if 0 /* Disable SPI Flash for revA HW */
const struct flash_platform_data asi1230_spi_flash = {
	.type = "m25p16",
	.name = "spi_flash",
	.parts = NULL,
	.nr_parts = 0,
};
#endif
struct spi_board_info __initdata asi1230_spi_slave_info[] = {
#if 0 /* Disable SPI Flash for revA HW */
	{
	 .modalias = "m25p80",
	 .platform_data = &asi1230_spi_flash,
	 .irq = -1,
	 .max_speed_hz = 48000000,	// max speed of TI814x SPI is 48MHz
	 .bus_num = 1,
	 .chip_select = 0,
	 },
#endif
	{
	 .modalias = "spidev",
	 .irq = -1,
	 .max_speed_hz = 48000000,
	 .bus_num = 1,
	 .chip_select = 1,
	 },
	{
	 .modalias = "spidev",
	 .irq = -1,
	 .max_speed_hz = 48000000,
	 .bus_num = 1,
	 .chip_select = 2,
	 },
	{
	 .modalias = "spidev",
	 .irq = -1,
	 .max_speed_hz = 48000000,
	 .bus_num = 2,
	 .chip_select = 0,
	 },
};

void __init asi1230_spi_init(void)
{
	spi_register_board_info(asi1230_spi_slave_info,
				ARRAY_SIZE(asi1230_spi_slave_info));
}

#define PHY_VSC8601_ID 0x00070421
#define PHY_VSC8601_MASK 0xFFFFFFFF
#define PHY_VSC8601_EXCTRL1_REG 0x17
#define PHY_VSC8601_LEDCTRL_REG 0x1B
#define PHY_VSC8601_RXCLKSKEW 0x100
#define PHY_VSC8601_LEDBLINK_EN 0x04
#define PHY_VSC8601_LEDBLINK_10HZ 0x02

static int asi1230_vsc_phy_fixup(struct phy_device *phydev)
{
	unsigned int val;

	/* Enable RGMII RX/TX clock skew */
	val = phy_read(phydev, PHY_VSC8601_EXCTRL1_REG);
	val |= PHY_VSC8601_RXCLKSKEW;
	phy_write(phydev, PHY_VSC8601_EXCTRL1_REG, val);
	val = phy_read(phydev, PHY_VSC8601_EXCTRL1_REG);

	/* Blink RJ-45 activity LED at a 10Hz rate */
	val = phy_read(phydev, PHY_VSC8601_LEDCTRL_REG);
	val |= PHY_VSC8601_LEDBLINK_EN | PHY_VSC8601_LEDBLINK_10HZ;
	phy_write(phydev, PHY_VSC8601_LEDCTRL_REG, val);
	return 0;
}

#define LED0_GPIO 1
#define LED1_GPIO 2
#define LED2_GPIO 3
#define LED3_GPIO 4
#define LED4_GPIO 15
#define LED5_GPIO 16
#define LED6_GPIO 17
#define LED7_GPIO 18

#define J2_9_GPIO 5

static struct gpio_led asi1230_led_config[] = {
	{
	 .name = "asi1230:led0:sys_heartbeat",
	 .default_trigger = "heartbeat",
	 .active_low = true,
	 .gpio = LED0_GPIO,
	 },
	{
	 .name = "asi1230:led1:mmc0_act",
	 .default_trigger = "mmc0",
	 .active_low = true,
	 .gpio = LED1_GPIO,
	 },
	{
	 .name = "asi1230:led2",
	 .active_low = true,
	 .gpio = LED2_GPIO,
	 },
	{
	 .name = "asi1230:led3",
	 .active_low = true,
	 .gpio = LED3_GPIO,
	 },
	/* led 4..7 are for the DSP to use */
	{
	 .name = "asi1230:led4",
	 .active_low = true,
	 .gpio = LED4_GPIO,
	 },
	{
	 .name = "asi1230:led5",
	 .active_low = true,
	 .gpio = LED5_GPIO,
	 },
	{
	 .name = "asi1230:led6",
	 .active_low = true,
	 .gpio = LED6_GPIO,
	 },
	{
	 .name = "asi1230:led7",
	 .active_low = true,
	 .gpio = LED7_GPIO,
	 },
};

static struct gpio_led_platform_data asi1230_led_data = {
	.leds = asi1230_led_config,
	.num_leds = ARRAY_SIZE(asi1230_led_config),
};

static struct platform_device asi1230_led_device = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &asi1230_led_data,
		},
};

static struct gpio_keys_button asi1230_gpio_buttons[] = {
	{
	 .code = KEY_VENDOR,
	 .gpio = J2_9_GPIO,
	 .desc = "eng_mode_jumper",
	 .type = EV_KEY,
	 .active_low = true,
	 .wakeup = 1,
	 },
};

static struct gpio_keys_platform_data asi1230_gpio_key_data = {
	.buttons = asi1230_gpio_buttons,
	.nbuttons = ARRAY_SIZE(asi1230_gpio_buttons),
	.rep = false,
};

static struct platform_device asi1230_keys_device = {
	.name = "gpio-keys",
	.id = -1,
	.dev = {
		.platform_data = &asi1230_gpio_key_data,
		},
};

static struct platform_device *asi1230_devices[] __initdata = {
	&asi1230_led_device,
	&asi1230_keys_device,
};

/* Board Init */

static void __init asi1230_init_irq(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
	gpmc_init();
}

static void __init asi1230_map_io(void)
{
	omap2_set_globals_ti814x();
	ti81xx_map_common_io();
}

void __init asi1230_reserve(void)
{
	ti81xx_reserve();
}

static void __init asi1230_init(void)
{
	struct clk *dev_clk;

	ti814x_mux_init(board_mux);
	omap_serial_init();
	asi1230_i2c_init();
	omap2_hsmmc_init(mmc);

	asi1230_spi_init();
	platform_add_devices(asi1230_devices, ARRAY_SIZE(asi1230_devices));
	regulator_use_dummy_regulator();

	/* Enable McASP0 clock so it can be accessed by other CPUs */
	dev_clk = clk_get_sys("davinci-mcasp.0", NULL);
	BUG_ON(IS_ERR(dev_clk));
	clk_enable(dev_clk);

	phy_register_fixup_for_uid(PHY_VSC8601_ID, PHY_VSC8601_MASK,
				   asi1230_vsc_phy_fixup);
}

MACHINE_START(ASI1230, "asi1230")
	/* Maintainer: Audioscience Inc */
	.boot_params	= 0x80000100,
	.map_io		= asi1230_map_io,
	.reserve        = asi1230_reserve,
	.init_irq       = asi1230_init_irq,
	.init_machine   = asi1230_init,
	.timer          = &omap_timer,
MACHINE_END
