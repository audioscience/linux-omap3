/*
 * linux/arch/arm/mach-omap2/board-ldp.c
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 * Nishant Kamat <nskamat@ti.com>
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c/twl4030.h>
#include <linux/i2c/twl4030-rtc.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/mcspi.h>
#include <asm/arch/gpio.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>
#include <asm/arch/keypad.h>
#include <asm/arch/gpmc.h>
#include <asm/arch/hsmmc.h>
#include <asm/arch/usb-musb.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/arch/control.h>
#ifdef CONFIG_OMAP3_PM
#include "prcm-regs.h"
#include "ti-compat.h"
#include <asm/arch/prcm_34xx.h>
#endif

#define	SDP3430_SMC91X_CS	3
#define ENABLE_VAUX1_DEDICATED	0x03
#define ENABLE_VAUX1_DEV_GRP	0x20

#define ENABLE_VAUX3_DEDICATED	0x03
#define ENABLE_VAUX3_DEV_GRP	0x20
#define TWL4030_MSECURE_GPIO	22

#ifdef CONFIG_OMAP3_PM
#define CONTROL_SYSC_SMARTIDLE  (0x2 << 3)
#define CONTROL_SYSC_AUTOIDLE   (0x1)

#define PRCM_INTERRUPT_MASK     (1 << 11)
#endif

#define UART1_INTERRUPT_MASK    (1 << 8)
#define UART2_INTERRUPT_MASK    (1 << 9)
#define UART3_INTERRUPT_MASK    (1 << 10)
#define TWL4030_MSECURE_GPIO    22

#define OMAP_CTRL_READADDR	0x48002000
#define OMAP_MMC1_DAT4		0x150
#define OMAP_MMC1_DAT5		0x152

#define OMAP_MMC1_DAT4_GPIO	126
#define OMAP_MMC1_DAT5_GPIO	127

int console_detect(char *str);
unsigned int uart_interrupt_mask_value;
u32 *console_padconf_reg;
bool console_detected;

static struct resource ldp_smc911x_resources[] = {
	[0] = {
		.start	= OMAP34XX_ETHR_START,
		.end	= OMAP34XX_ETHR_START + SZ_4K,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device ldp_smc911x_device = {
	.name		= "smc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ldp_smc911x_resources),
	.resource	= ldp_smc911x_resources,
};

#ifdef CONFIG_OMAP3_PM
/*
 * Board DDR timings used during frequency changes
 */
struct dvfs_config omap3_vdd2_config[PRCM_NO_VDD2_OPPS] = {
#ifdef CONFIG_OMAP3_CORE_166MHZ
	{
	/* SDRC CS0/CS1 values 83MHZ*/
	/* not optimized at 1/2 speed except for RFR */
	{{0x00025801, 0x629db4c6, 0x00012214},    /* cs 0 */
	 {0x00025801, 0x629db4c6, 0x00012214} },  /* cs 1 */
	},

	/* SDRC CS0/CS1 values 166MHZ*/
	{
	{{0x0004e201, 0xaa9db4c6, 0x00011517},
	 {0x0004e201, 0xaa9db4c6, 0x00011517} },
	},
#elif defined(CONFIG_OMAP3_CORE_133MHZ)
	{
	/* SDRC CS0/CS1 values 66MHZ*/
	{{0x0001ef01, 0x8a99b485, 0x00011412},
	 {0x0001ef01, 0x8a99b485, 0x00011412} },
	},

	/* SDRC CS0/CS1 values 133MHZ*/
	{
	{{0x0003de01, 0x8a99b485, 0x00011412},
	 {0x0003de01, 0x8a99b485, 0x00011412} },
	},
#endif
};

static u32 *uart_detect(void){
	char str[7];

	if (console_detected)
		return console_padconf_reg;

	console_padconf_reg = NULL;
	console_detected = 0;

	if (console_detect(str))
		printk(KERN_INFO"Invalid console paramter\n");

	if (!strcmp(str, "ttyS0")) {
		console_padconf_reg = (u32 *)(&CONTROL_PADCONF_UART1_CTS);
		uart_interrupt_mask_value = UART1_INTERRUPT_MASK;
		}
	else if (!strcmp(str, "ttyS1")) {
		console_padconf_reg = (u32 *)(&CONTROL_PADCONF_UART2_TX);
		uart_interrupt_mask_value = UART2_INTERRUPT_MASK;
		}
	else if (!strcmp(str, "ttyS2")) {
		console_padconf_reg = (u32 *)(&CONTROL_PADCONF_UART3_RTS_SD);
		uart_interrupt_mask_value = UART3_INTERRUPT_MASK;
		}
	else
		printk(KERN_INFO
		"Unable to recongnize Console UART!\n");

	if (console_padconf_reg)
		console_detected = 1;

	return console_padconf_reg;
}

void  init_wakeupconfig(void)
{
	u32 *ptr;
	ptr = uart_detect();
	*ptr |= (u32)((IO_PAD_WAKEUPENABLE | IO_PAD_OFFPULLUDENABLE |
			IO_PAD_OFFOUTENABLE | IO_PAD_OFFENABLE |
			IO_PAD_INPUTENABLE | IO_PAD_PULLUDENABLE)
				<<  IO_PAD_HIGH_SHIFT);

}

bool is_console_wakeup(void)
{
	if ((*uart_detect() >> IO_PAD_HIGH_SHIFT) & IO_PAD_WAKEUPEVENT)
		return 1;
	return 0;
}
void uart_padconf_control(void)
{
	u32 *ptr;
	ptr = (u32 *)uart_detect();
		*ptr |= (u32)((IO_PAD_WAKEUPENABLE)
			<< IO_PAD_HIGH_SHIFT);
}

void setup_board_wakeup_source(u32 wakeup_source)
{
	if ((wakeup_source & PRCM_WAKEUP_T2_KEYPAD) ||
		(wakeup_source & PRCM_WAKEUP_TOUCHSCREEN)) {
		PRCM_GPIO1_SYSCONFIG = 0x15;
		PM_WKEN_WKUP |= 0x8;
		PM_MPUGRPSEL_WKUP = 0x8;
		/* Unmask GPIO interrupt*/
		INTC_MIR_0 = ~((1<<29));
	}
	if (wakeup_source & PRCM_WAKEUP_T2_KEYPAD) {
		CONTROL_PADCONF_SYS_NIRQ &= 0xFFFFFFF8;
		CONTROL_PADCONF_SYS_NIRQ |= 0x4;
		GPIO1_SETIRQENABLE1 |= 0x1;
		GPIO1_SETWKUENA |= 0x1;
		GPIO1_FALLINGDETECT |= 0x1;
	}
	/* Unmasking the PRCM interrupts */
	INTC_MIR_0 &= ~PRCM_INTERRUPT_MASK;
	if (wakeup_source & PRCM_WAKEUP_UART) {
		/* Unmasking the UART interrupts */
		INTC_MIR_2 = ~uart_interrupt_mask_value;
	}
}

static void scm_clk_init(void)
{
	struct clk *p_omap_ctrl_clk = NULL;

	p_omap_ctrl_clk = clk_get(NULL, "omapctrl_ick");
	if (p_omap_ctrl_clk != NULL) {
		if (clk_enable(p_omap_ctrl_clk) != 0) {
			printk(KERN_ERR "failed to enable scm clks\n");
			clk_put(p_omap_ctrl_clk);
		}
	}
	/* Sysconfig set to SMARTIDLE and AUTOIDLE */
	CONTROL_SYSCONFIG = (CONTROL_SYSC_SMARTIDLE | CONTROL_SYSC_AUTOIDLE);
}
#endif

static int ldp_twl4030_keymap[] = {
	KEY(0, 0, KEY_1),
	KEY(1, 0, KEY_2),
	KEY(2, 0, KEY_3),
	KEY(3, 0, KEY_ENTER),
	KEY(4, 0, KEY_F1),
	KEY(5, 0, KEY_F2),
	KEY(0, 1, KEY_4),
	KEY(1, 1, KEY_5),
	KEY(2, 1, KEY_6),
	KEY(3, 1, KEY_F5),
	KEY(4, 1, KEY_F3),
	KEY(5, 1, KEY_F4),
	KEY(0, 2, KEY_7),
	KEY(1, 2, KEY_8),
	KEY(2, 2, KEY_9),
	KEY(3, 2, KEY_F6),
	KEY(0, 3, KEY_F7),
	KEY(1, 3, KEY_0),
	KEY(2, 3, KEY_F8),
	KEY(0, 4, KEY_RIGHT),
	KEY(1, 4, KEY_UP),
	KEY(2, 4, KEY_DOWN),
	KEY(3, 4, KEY_LEFT),
	KEY(5, 4, KEY_MUTE),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(5, 5, KEY_VOLUMEDOWN),
	0
};

#define GPIO_KEY(gpio, key)  ((gpio<<16) | (key))

static unsigned int ldp_omap_gpio_keymap[] = {
	GPIO_KEY(101, KEY_ENTER), /*select*/
	GPIO_KEY(102, KEY_F1),    /*S7*/
	GPIO_KEY(103, KEY_F2),    /*S3*/
	GPIO_KEY(104, KEY_F3),    /*S1*/
	GPIO_KEY(105, KEY_F4),    /*S2*/
	GPIO_KEY(106, KEY_LEFT),  /*left*/
	GPIO_KEY(107, KEY_RIGHT), /*right*/
	GPIO_KEY(108, KEY_UP),    /*up*/
	GPIO_KEY(109, KEY_DOWN),  /*down*/
	0
};

/* OMAP3430 LDP Keymaps:
 * OMAP LDP uses both TWL4030 GPIO's and OMAP GPIO's to get key presses.
 * This is why there are two keymaps:
 *  - ldp_twl4030_keymap (TWL4030)
 *  - ldp_omap_gpio_keymap (OMAP GPIO's)
 * Unfortunately the input subsystem requires all the keymaps to be
 * listed in one place (.keymap) in order for a key to be a valid input.
 * This is why some keys appear in both keymaps.
 * If a key does appear in both keymaps then its entry in
 * ldp_twl4030_keymap is purely to keep the input subsystem happy and
 * its row/col values have no meaning.
 */
static struct omap_kp_platform_data ldp_kp_data = {
	.rows		= 6,
	.cols		= 6,
	.keymap 	= ldp_twl4030_keymap,
	.keymapsize 	= ARRAY_SIZE(ldp_twl4030_keymap),
	.rep		= 1,
	/* Use row_gpios as a way to pass the OMAP GPIO keymap pointer */
	.row_gpios	= ldp_omap_gpio_keymap,
};

static struct platform_device ldp_kp_device = {
	.name		= "omap_twl4030keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &ldp_kp_data,
	},
};

static int ts_gpio;

#ifdef CONFIG_RTC_DRV_TWL4030
static int twl4030_rtc_init(void)
{
	int ret = 0;

	/* 3430ES2.0 doesn't have msecure/gpio-22 line connected to T2 */
	if (is_device_type_gp() && is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		u32 msecure_pad_config_reg = omap_ctrl_base_get() + 0xA3C;
		int mux_mask = 0x04;
		u16 tmp;

		ret = omap_request_gpio(TWL4030_MSECURE_GPIO);
		if (ret < 0) {
			printk(KERN_ERR "twl4030_rtc_init: can't"
				"reserve GPIO:%d !\n", TWL4030_MSECURE_GPIO);
			goto out;
		}
		/*
		 * TWL4030 will be in secure mode if msecure line from OMAP
		 * is low. Make msecure line high in order to change the
		 * TWL4030 RTC time and calender registers.
		 */
		omap_set_gpio_direction(TWL4030_MSECURE_GPIO, 0);

		tmp = omap_readw(msecure_pad_config_reg);
		tmp &= 0xF8;	/* To enable mux mode 03/04 = GPIO_RTC */
		tmp |= mux_mask;/* To enable mux mode 03/04 = GPIO_RTC */
		omap_writew(tmp, msecure_pad_config_reg);

		omap_set_gpio_dataout(TWL4030_MSECURE_GPIO, 1);
	}
out:
	return ret;
}

static void twl4030_rtc_exit(void)
{
	omap_free_gpio(TWL4030_MSECURE_GPIO);
}

static struct twl4030rtc_platform_data ldp_twl4030rtc_data = {
	.init = &twl4030_rtc_init,
	.exit = &twl4030_rtc_exit,
};

static struct platform_device ldp_twl4030rtc_device = {
	.name		= "twl4030_rtc",
	.id		= -1,
	.dev		= {
		.platform_data	= &ldp_twl4030rtc_data,
	},
};
#endif

/**
 * @brief ads7846_dev_init : Requests & sets GPIO line for pen-irq
 *
 * @return - void. If request gpio fails then Flag KERN_ERR.
 */
static void ads7846_dev_init(void)
{
	if (omap_request_gpio(ts_gpio) < 0) {
		printk(KERN_ERR "can't get ads746 pen down GPIO\n");
		return;
	}

	omap_set_gpio_direction(ts_gpio, 1);

	omap_set_gpio_debounce(ts_gpio, 1);
	omap_set_gpio_debounce_time(ts_gpio, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !omap_get_gpio_datain(ts_gpio);
}

/*
 * This enable(1)/disable(0) the voltage for TS: uses twl4030 calls
 */
static int ads7846_vaux_control(int vaux_cntrl)
{
	int ret = 0;

#ifdef CONFIG_TWL4030_CORE
	/* check for return value of ldo_use: if success it returns 0 */
	if (vaux_cntrl == VAUX_ENABLE) {
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX1_DEDICATED, TWL4030_VAUX1_DEDICATED))
			return -EIO;
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX1_DEV_GRP, TWL4030_VAUX1_DEV_GRP))
			return -EIO;
	} else if (vaux_cntrl == VAUX_DISABLE) {
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			0x00, TWL4030_VAUX1_DEDICATED))
			return -EIO;
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			0x00, TWL4030_VAUX1_DEV_GRP))
			return -EIO;
	}
#else
	ret = -EIO;
#endif
	return ret;
}

/**
 * @brief modem_uart_config : Request & sets GPIO IO pads
 *
 * @return - void
 */
static void modem_uart_config(void)
{
	/* Set the padconfig */
	omap_writew(0x011c, OMAP_CTRL_READADDR + OMAP_MMC1_DAT4);
	omap_writew(0x010c, OMAP_CTRL_READADDR + OMAP_MMC1_DAT5);

	/* Set GPIO for conection with Neptune modem */

	/* GPIO_126 is modem_nRst */
	omap_request_gpio(OMAP_MMC1_DAT4_GPIO);
	omap_set_gpio_direction(OMAP_MMC1_DAT4_GPIO, 1);
	omap_set_gpio_dataout(OMAP_MMC1_DAT4_GPIO, 1);

	/* GPIO_127 s  modem_nPwron */
	omap_request_gpio(OMAP_MMC1_DAT5_GPIO);
	omap_set_gpio_direction(OMAP_MMC1_DAT5_GPIO, 0);
	omap_set_gpio_dataout(OMAP_MMC1_DAT5_GPIO, 0);
}

static struct ads7846_platform_data tsc2046_config __initdata = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 10,
	.debounce_tol		= 10,
	.debounce_rep		= 1,
 	.get_pendown_state	= ads7846_get_pendown_state,
 	.keep_vref_on		= 1,
 	.vaux_control		= ads7846_vaux_control,
	.settle_delay_usecs	= 100,
};


static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,  /* 0: slave, 1: master */
};

#ifdef CONFIG_FB_OMAP_LCD_WVGA
static struct omap2_mcspi_device_config wvgalcd_mcspi_config = {
	.turbo_mode		= 0,
	.single_channel		= 1,  /* 0: slave, 1: master */
};
#endif

static struct spi_board_info ldp_spi_board_info[] __initdata = {
	[0] = {
		/*
		 * TSC2046 operates at a max freqency of 2MHz, so
		 * operate slightly below at 1.5MHz
		 */
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &tsc2046_mcspi_config,
		.irq			= 0,
		.platform_data		= &tsc2046_config,
	},
#ifdef CONFIG_FB_OMAP_LCD_WVGA
	[1] = {
		.modalias		= "wvgalcd",
		.bus_num		= 1,
		.chip_select		= 2,
		.max_speed_hz		= 375000,
		.controller_data 	= &wvgalcd_mcspi_config,
	},
#endif
};

static struct platform_device *ldp_devices[] __initdata = {
	&ldp_smc911x_device,
	&ldp_kp_device,
#ifdef CONFIG_RTC_DRV_TWL4030
	&ldp_twl4030rtc_device,
#endif
};

static inline void __init ldp_init_smc911x(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	int eth_gpio = 0;

	eth_cs = LDP_SMC911X_CS;

	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for smc911x\n");
		return;
	}

	ldp_smc911x_resources[0].start = cs_mem_base + 0x0;
	ldp_smc911x_resources[0].end   = cs_mem_base + 0xf;
	udelay(100);

	eth_gpio = LDP_SMC911X_GPIO;

	ldp_smc911x_resources[1].start = OMAP_GPIO_IRQ(eth_gpio);

	if (omap_request_gpio(eth_gpio) < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc911x IRQ\n",
			eth_gpio);
		return;
	}
	omap_set_gpio_direction(eth_gpio, 1);
}

static void __init omap_ldp_init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
#ifdef CONFIG_OMAP3_PM
	/* System Control module clock initialization */
	scm_clk_init();
#endif
	omap_gpio_init();
	ldp_init_smc911x();
}

static struct omap_uart_config ldp_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_mmc_config ldp_mmc_config __initdata = {
	.mmc [0] = {
		.enabled	= 1,
		.wire4		= 1,
	},
};

static struct omap_board_config_kernel ldp_config[] __initdata = {
	{ OMAP_TAG_UART,	&ldp_uart_config },
	{ OMAP_TAG_MMC,		&ldp_mmc_config },
};

static int __init omap_i2c_init(void)
{
	omap_register_i2c_bus(1, 100, NULL, 0);
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static void __init omap_ldp_init(void)
{
	platform_add_devices(ldp_devices, ARRAY_SIZE(ldp_devices));
	omap_board_config = ldp_config;
	omap_board_config_size = ARRAY_SIZE(ldp_config);
	ts_gpio = 54;
	ldp_spi_board_info[0].irq = OMAP_GPIO_IRQ(ts_gpio);
	spi_register_board_info(ldp_spi_board_info,
				ARRAY_SIZE(ldp_spi_board_info));
#ifdef CONFIG_OMAP3_PM
	prcm_init();
#endif
	ads7846_dev_init();
	ldp_flash_init();
	omap_serial_init();
	usb_musb_init();
	hsmmc_init();
	modem_uart_config();
}

static void __init omap_ldp_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}
arch_initcall(omap_i2c_init);

MACHINE_START(OMAP_LDP, "OMAP LDP board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_ldp_map_io,
	.init_irq	= omap_ldp_init_irq,
	.init_machine	= omap_ldp_init,
	.timer		= &omap_timer,
MACHINE_END
