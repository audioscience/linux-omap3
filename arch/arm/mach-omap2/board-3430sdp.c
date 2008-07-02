/*
 * linux/arch/arm/mach-omap2/board-3430sdp.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
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
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/i2c/twl4030.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/mcspi.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/irda.h>
#include <asm/arch/board.h>
#include <asm/arch/usb-musb.h>
#include <asm/arch/usb-ehci.h>
#include <asm/arch/hsmmc.h>
#include <asm/arch/common.h>
#include <asm/arch/keypad.h>
#include <asm/arch/dma.h>
#include <asm/arch/gpmc.h>
#include <linux/i2c/twl4030-rtc.h>
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/mt9p012.h>
#include <../drivers/media/video/omap34xxcam.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/arch/control.h>

#define	SDP3430_SMC91X_CS	3

#define ENABLE_VAUX1_DEDICATED	0x03
#define ENABLE_VAUX1_DEV_GRP	0x20

#define ENABLE_VAUX3_DEDICATED	0x03
#define ENABLE_VAUX3_DEV_GRP	0x20


#define TWL4030_MSECURE_GPIO	22

static struct resource sdp3430_smc91x_resources[] = {
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

static struct platform_device sdp3430_smc91x_device = {
	.name		= "smc91x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sdp3430_smc91x_resources),
	.resource	= sdp3430_smc91x_resources,
};

/* IrDA
 */
#if defined(CONFIG_OMAP_IR) || defined(CONFIG_OMAP_IR_MODULE)

#define	IRDA_SD	164	/* gpio 164 */
#define	IRDA_TX	166	/* gpio 166 */
#define	IRDA_SD_PIN	T21_3430_GPIO164
#define	IRDA_TX_PIN	V21_3430_GPIO166

#define IRDA_VAUX_EN	1
#define IRDA_VAUX_DIS	0

/*
 * This enable(1)/disable(0) the voltage for IrDA: uses twl4030 calls
 */
static int irda_vaux_control(int vaux_cntrl)
{
	int ret = 0;

#ifdef CONFIG_TWL4030_CORE
	/* check for return value of ldo_use: if success it returns 0 */
	if (vaux_cntrl == IRDA_VAUX_EN) {
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX1_DEDICATED, TWL4030_VAUX1_DEDICATED))
			return -EIO;
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX1_DEV_GRP, TWL4030_VAUX1_DEV_GRP))
			return -EIO;
	} else if (vaux_cntrl == IRDA_VAUX_DIS) {
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

static int select_irda(struct device *dev, int state)
{
	int err;
	if (state == IR_SEL) {
		err = irda_vaux_control(IRDA_VAUX_EN);
		if (err != 0) {
			printk(KERN_ERR "OMAP: IrDA vaux enable failed\n");
			return err;
		}

		omap_cfg_reg(R21_3430_UART3_CTS_RCTX);
		omap_cfg_reg(T21_3430_UART3_RTS_SD);
		omap_cfg_reg(U21_3430_UART3_RX_IRRX);
		omap_cfg_reg(V21_3430_UART3_TX_IRTX);

		omap_request_gpio(IRDA_SD);
		omap_request_gpio(IRDA_TX);
		omap_cfg_reg(IRDA_SD_PIN);
		omap_set_gpio_direction(IRDA_SD, GPIO_DIR_OUTPUT);
		omap_set_gpio_direction(IRDA_TX, GPIO_DIR_OUTPUT);
		omap_set_gpio_dataout(IRDA_SD, 0);
	} else {
		omap_free_gpio(IRDA_SD);
		omap_free_gpio(IRDA_TX);
		err = irda_vaux_control(IRDA_VAUX_EN);
		if (err != 0) {
			printk(KERN_ERR "OMAP: IrDA vaux Enable failed\n");
			return err;
		}
	}

	return 0;
}

static int transceiver_mode(struct device *dev, int mode)
{
	omap_cfg_reg(IRDA_SD_PIN);
	omap_cfg_reg(IRDA_TX_PIN);

	if (mode & IR_SIRMODE) {
		/* SIR */
		omap_set_gpio_dataout(IRDA_SD, 1);
		udelay(1);
		omap_set_gpio_dataout(IRDA_TX, 0);
		udelay(1);
		omap_set_gpio_dataout(IRDA_SD, 0);
		udelay(1);
	} else {
		/* MIR/FIR */
		omap_set_gpio_dataout(IRDA_SD, 1);
		udelay(1);
		omap_set_gpio_dataout(IRDA_TX, 1);
		udelay(1);
		omap_set_gpio_dataout(IRDA_SD, 0);
		udelay(1);
		omap_set_gpio_dataout(IRDA_TX, 0);
		udelay(1);
	}

	omap_cfg_reg(T21_3430_UART3_RTS_SD);
	omap_cfg_reg(V21_3430_UART3_TX_IRTX);
	return 0;
}
#else
static int select_irda(struct device *dev, int state) { return 0; }
static int transceiver_mode(struct device *dev, int mode) { return 0; }
#endif

static struct omap_irda_config irda_data = {
	.transceiver_cap	= IR_SIRMODE | IR_MIRMODE | IR_FIRMODE,
	.transceiver_mode	= transceiver_mode,
	.select_irda	 	= select_irda,
	.rx_channel		= OMAP24XX_DMA_UART3_RX,
	.tx_channel		= OMAP24XX_DMA_UART3_TX,
	.dest_start		= OMAP_UART3_BASE,
	.src_start		= OMAP_UART3_BASE,
	.tx_trigger		= OMAP24XX_DMA_UART3_TX,
	.rx_trigger		= OMAP24XX_DMA_UART3_RX,
};

static struct resource irda_resources[] = {
	[0] = {
		.start	= INT_24XX_UART3_IRQ,
		.end	= INT_24XX_UART3_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device irda_device = {
	.name		= "omapirda",
	.id		= -1,
	.dev		= {
		.platform_data	= &irda_data,
	},
	.num_resources	= 1,
	.resource	= irda_resources,
};

static int sdp3430_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_A),
	KEY(0, 3, KEY_B),
	KEY(0, 4, KEY_C),
	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_E),
	KEY(1, 3, KEY_F),
	KEY(1, 4, KEY_G),
	KEY(2, 0, KEY_ENTER),
	KEY(2, 1, KEY_I),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_K),
	KEY(2, 4, KEY_3),
	KEY(3, 0, KEY_M),
	KEY(3, 1, KEY_N),
	KEY(3, 2, KEY_O),
	KEY(3, 3, KEY_P),
	KEY(3, 4, KEY_Q),
	KEY(4, 0, KEY_R),
	KEY(4, 1, KEY_4),
	KEY(4, 2, KEY_T),
	KEY(4, 3, KEY_U),
	KEY(4, 4, KEY_D),
	KEY(5, 0, KEY_V),
	KEY(5, 1, KEY_W),
	KEY(5, 2, KEY_L),
	KEY(5, 3, KEY_S),
	KEY(5, 4, KEY_H),
	0
};

static struct omap_kp_platform_data sdp3430_kp_data = {
	.rows		= 5,
	.cols		= 6,
	.keymap 	= sdp3430_keymap,
	.keymapsize 	= ARRAY_SIZE(sdp3430_keymap),
	.rep		= 1,
};

static struct platform_device sdp3430_kp_device = {
	.name		= "omap_twl4030keypad",
	.id		= -1,
	.dev		= {
		.platform_data	= &sdp3430_kp_data,
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

static struct twl4030rtc_platform_data sdp3430_twl4030rtc_data = {
	.init = &twl4030_rtc_init,
	.exit = &twl4030_rtc_exit,
};

static struct platform_device sdp3430_twl4030rtc_device = {
	.name		= "twl4030_rtc",
	.id		= -1,
	.dev		= {
		.platform_data	= &sdp3430_twl4030rtc_data,
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
			ENABLE_VAUX3_DEDICATED, TWL4030_VAUX3_DEDICATED))
			return -EIO;
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX3_DEV_GRP, TWL4030_VAUX3_DEV_GRP))
			return -EIO;
	} else if (vaux_cntrl == VAUX_DISABLE) {
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			0x00, TWL4030_VAUX3_DEDICATED))
			return -EIO;
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			0x00, TWL4030_VAUX3_DEV_GRP))
			return -EIO;
	}
#else
	ret = -EIO;
#endif
	return ret;
}

static struct ads7846_platform_data tsc2046_config __initdata = {
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.vaux_control		= ads7846_vaux_control,
};


static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,  /* 0: slave, 1: master */
};

static struct spi_board_info sdp3430_spi_board_info[] __initdata = {
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
};

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
static void __iomem *fpga_map_addr;
/*
 * Common MT9P012 register initialization for all image sizes, pixel formats,
 * and frame rates
 */
const static struct mt9p012_reg mt9p012_common[] = {
	{MT9P012_8BIT, REG_SOFTWARE_RESET, 0x01},
	{MT9P012_TOK_DELAY, 0x00, 5}, /* Delay = 5ms, min 2400 xcks */
	{MT9P012_16BIT, REG_RESET_REGISTER, 0x10C8},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9P012_16BIT, REG_ANALOG_GAIN_GREENR, 0x0020},
	{MT9P012_16BIT, REG_ANALOG_GAIN_RED, 0x0020},
	{MT9P012_16BIT, REG_ANALOG_GAIN_BLUE, 0x0020},
	{MT9P012_16BIT, REG_ANALOG_GAIN_GREENB, 0x0020},
	{MT9P012_16BIT, REG_DIGITAL_GAIN_GREENR, 0x0100},
	{MT9P012_16BIT, REG_DIGITAL_GAIN_RED, 0x0100},
	{MT9P012_16BIT, REG_DIGITAL_GAIN_BLUE, 0x0100},
	{MT9P012_16BIT, REG_DIGITAL_GAIN_GREENB, 0x0100},
	/* Recommended values for image quality, sensor Rev 1 */
	{MT9P012_16BIT, 0x3088, 0x6FFB},
	{MT9P012_16BIT, 0x308E, 0x2020},
	{MT9P012_16BIT, 0x309E, 0x4400},
	{MT9P012_16BIT, 0x30D4, 0x9080},
	{MT9P012_16BIT, 0x3126, 0x00FF},
	{MT9P012_16BIT, 0x3154, 0x1482},
	{MT9P012_16BIT, 0x3158, 0x97C7},
	{MT9P012_16BIT, 0x315A, 0x97C6},
	{MT9P012_16BIT, 0x3162, 0x074C},
	{MT9P012_16BIT, 0x3164, 0x0756},
	{MT9P012_16BIT, 0x3166, 0x0760},
	{MT9P012_16BIT, 0x316E, 0x8488},
	{MT9P012_16BIT, 0x3172, 0x0003},
	{MT9P012_16BIT, 0x30EA, 0x3F06},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update all at once */
	{MT9P012_TOK_TERM, 0, 0}

};

static struct omap34xxcam_hw_config cam_hwc = {
	.sensor_isp = V4L2_IF_CAP_RAW,
	.xclk = OMAP34XXCAM_XCLK_A,
};

static void enable_fpga_vio_1v8(u8 enable)
{
	u16 reg_val;

	fpga_map_addr = ioremap(DEBUG_BASE, 4096);
	reg_val = readw(fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);

	/* Ensure that the SPR_GPIO1_3v3 is 0 - powered off.. 1 is on */
	if (reg_val & FPGA_SPR_GPIO1_3v3) {
		reg_val |= FPGA_SPR_GPIO1_3v3;
		reg_val |= FPGA_GPIO6_DIR_CTRL; /* output mode */
		writew(reg_val, fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);
		/* give a few milli sec to settle down
		 * Let the sensor also settle down.. if required..
		 */
		if (enable)
			mdelay(10);
	}

	if (enable) {
		reg_val |= FPGA_SPR_GPIO1_3v3 | FPGA_GPIO6_DIR_CTRL;
		writew(reg_val, fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);
	}
	/* Vrise time for the voltage - should be less than 1 ms */
	mdelay(1);
}

static int mt9p012_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->sensor_isp = cam_hwc.sensor_isp;
	hwc->xclk = cam_hwc.xclk;
	return 0;
}

static int mt9p012_sensor_power_set(enum v4l2_power power)
{
	switch (power) {
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
#ifdef CONFIG_TWL4030_CORE
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX2_DEV_GRP);
#else
#error "no power companion board defined!"
#endif
		enable_fpga_vio_1v8(0);
		omap_free_gpio(MT9P012_RESET_GPIO);
		iounmap(fpga_map_addr);
		omap_free_gpio(MT9P012_STANDBY_GPIO);
		break;
	case V4L2_POWER_ON:
		/* Power Up Sequence */

		/* Request and configure gpio pins */
		if (omap_request_gpio(MT9P012_STANDBY_GPIO) != 0) {
			printk(KERN_WARNING "Could not request GPIO %d for "
					"AF D88\n", MT9P012_STANDBY_GPIO);
			return -EIO;
		}

		/* Request and configure gpio pins */
		if (omap_request_gpio(MT9P012_RESET_GPIO) != 0)
			return -EIO;

		/* STANDBY_GPIO is active HIGH for set LOW to release */
		omap_set_gpio_dataout(MT9P012_STANDBY_GPIO, 1);

		/* nRESET is active LOW. set HIGH to release reset */
		omap_set_gpio_dataout(MT9P012_RESET_GPIO, 1);

		/* set to output mode */
		omap_set_gpio_direction(MT9P012_STANDBY_GPIO, GPIO_DIR_OUTPUT);
		/* set to output mode */
		omap_set_gpio_direction(MT9P012_RESET_GPIO, GPIO_DIR_OUTPUT);

		/* turn on digital power */
		enable_fpga_vio_1v8(1);
#ifdef CONFIG_TWL4030_CORE
		/* turn on analog power */
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_2_8_V, TWL4030_VAUX2_DEDICATED);
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX2_DEV_GRP);
#else
#error "no power companion board defined!"
#endif

		omap_set_gpio_dataout(MT9P012_STANDBY_GPIO, 0);

		udelay(1000);

		/* have to put sensor to reset to guarantee detection */
		omap_set_gpio_dataout(MT9P012_RESET_GPIO, 0);

		udelay(1500);

		/* nRESET is active LOW. set HIGH to release reset */
		omap_set_gpio_dataout(MT9P012_RESET_GPIO, 1);
		/* give sensor sometime to get out of the reset. Datasheet says
		   2400 xclks. At 6 MHz, 400 usec are enough */
		udelay(300);
		CONTROL_PADCONF_CAM_FLD = 0x01003B1C;
		omap_set_gpio_direction(MT9P012_RESET_GPIO, GPIO_DIR_INPUT);
		break;
	case V4L2_POWER_STANDBY:
		/* stand by */
		omap_set_gpio_dataout(MT9P012_STANDBY_GPIO, 1);
		break;
	case V4L2_POWER_RESUME:
		/* out of standby */
		omap_set_gpio_dataout(MT9P012_STANDBY_GPIO, 0);
		udelay(1000);
		break;
	}

    return 0;
}

static struct v4l2_ifparm ifparm = {
	.capability = V4L2_IF_CAP_RAW,
	.if_type = V4L2_IF_TYPE_BT656,
	.u = {
		.bt656 = {
			.frame_start_on_rising_vs = 1,
			.latch_clk_inv = 0,
			.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_10BIT,
			.clock_min = MT9P012_XCLK_MIN,
			.clock_max = MT9P012_XCLK_MAX,
		},
	},
};

static int mt9p012_ifparm(struct v4l2_ifparm *p)
{
	*p = ifparm;
	return 0;
}

static struct mt9p012_platform_data sdp3430_mt9p012_platform_data = {
	.power_set      = mt9p012_sensor_power_set,
	.priv_data_set  = mt9p012_sensor_set_prv_data,
	.default_regs   = mt9p012_common,
	.ifparm         = mt9p012_ifparm,
};

static struct i2c_board_info __initdata sdp3430_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("mt9p012", MT9P012_I2C_ADDR),
		.platform_data = &sdp3430_mt9p012_platform_data,
	},
};

#endif

static struct platform_device sdp3430_lcd_device = {
	.name		= "sdp2430_lcd",
	.id		= -1,
};

static struct platform_device *sdp3430_devices[] __initdata = {
	&sdp3430_smc91x_device,
	&irda_device,
	&sdp3430_kp_device,
	&sdp3430_lcd_device,
#ifdef CONFIG_RTC_DRV_TWL4030
	&sdp3430_twl4030rtc_device,
#endif
};

static inline void __init sdp3430_init_smc91x(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	int eth_gpio = 0;

	eth_cs = SDP3430_SMC91X_CS;

	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for smc91x\n");
		return;
	}

	sdp3430_smc91x_resources[0].start = cs_mem_base + 0x0;
	sdp3430_smc91x_resources[0].end   = cs_mem_base + 0xf;
	udelay(100);

	if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
		eth_gpio = OMAP34XX_ETHR_GPIO_IRQ_SDPV2;
	else
		eth_gpio = OMAP34XX_ETHR_GPIO_IRQ_SDPV1;

	sdp3430_smc91x_resources[1].start = OMAP_GPIO_IRQ(eth_gpio);

	if (omap_request_gpio(eth_gpio) < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc91x IRQ\n",
			eth_gpio);
		return;
	}
	omap_set_gpio_direction(eth_gpio, 1);
}

static void __init omap_3430sdp_init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
	sdp3430_init_smc91x();
}

static struct omap_uart_config sdp3430_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_lcd_config sdp3430_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct omap_mmc_config sdp3430_mmc_config __initdata = {
	.mmc [0] = {
		.enabled	= 1,
		.wire4		= 1,
	},
};

static struct omap_board_config_kernel sdp3430_config[] __initdata = {
	{ OMAP_TAG_UART,	&sdp3430_uart_config },
	{OMAP_TAG_LCD,		&sdp3430_lcd_config},
	{OMAP_TAG_MMC,		&sdp3430_mmc_config },
};

static int __init omap3430_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, NULL, 0);
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
	omap_register_i2c_bus(2, 400, sdp3430_i2c_board_info,
			      ARRAY_SIZE(sdp3430_i2c_board_info));
#else
	omap_register_i2c_bus(2, 400, NULL, 0);
#endif
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

extern void __init sdp3430_flash_init(void);

static void __init omap_3430sdp_init(void)
{
	platform_add_devices(sdp3430_devices, ARRAY_SIZE(sdp3430_devices));
	omap_board_config = sdp3430_config;
	omap_board_config_size = ARRAY_SIZE(sdp3430_config);
	if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
		ts_gpio = OMAP34XX_TS_GPIO_IRQ_SDPV2;
	else
		ts_gpio = OMAP34XX_TS_GPIO_IRQ_SDPV1;
	sdp3430_spi_board_info[0].irq = OMAP_GPIO_IRQ(ts_gpio);
	spi_register_board_info(sdp3430_spi_board_info,
				ARRAY_SIZE(sdp3430_spi_board_info));
	ads7846_dev_init();
	sdp3430_flash_init();
	omap_serial_init();
	usb_musb_init();
	usb_ehci_init();
	hsmmc_init();
}

static void __init omap_3430sdp_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}
arch_initcall(omap3430_i2c_init);

MACHINE_START(OMAP_3430SDP, "OMAP3430 3430SDP board")
	/* Maintainer: Syed Khasim - Texas Instruments Inc */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_3430sdp_map_io,
	.init_irq	= omap_3430sdp_init_irq,
	.init_machine	= omap_3430sdp_init,
	.timer		= &omap_timer,
MACHINE_END
