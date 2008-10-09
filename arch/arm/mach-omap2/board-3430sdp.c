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
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c/twl4030.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/irda.h>
#include <mach/board.h>
#include <mach/usb-musb.h>
#include <mach/usb-ehci.h>
#include <mach/hsmmc.h>
#include <mach/common.h>
#include <mach/keypad.h>
#include <mach/dma.h>
#include <mach/gpmc.h>
#include "ti-compat.h"

#ifdef CONFIG_VIDEO_OMAP3
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
#include <../drivers/media/video/mt9p012.h>
#endif
#endif

#ifdef CONFIG_VIDEO_DW9710
#include <../drivers/media/video/dw9710.h>
#endif

#ifdef CONFIG_OMAP3_PM
#include "prcm-regs.h"
#include <mach/prcm_34xx.h>
#endif

#include <asm/io.h>
#include <asm/delay.h>
#include <mach/control.h>

#include "sdram-qimonda-hyb18m512160af-6.h"

#define	SDP3430_SMC91X_CS	3

#define ENABLE_VAUX1_DEDICATED	0x03
#define ENABLE_VAUX1_DEV_GRP	0x20

#define ENABLE_VAUX3_DEDICATED	0x03
#define ENABLE_VAUX3_DEV_GRP	0x20


#define TWL4030_MSECURE_GPIO 22

#ifdef CONFIG_OMAP3_PM
#define CONTROL_SYSC_SMARTIDLE	(0x2 << 3)
#define CONTROL_SYSC_AUTOIDLE	(0x1)

#define SDP3430_SMC91X_CS	3
#define PRCM_INTERRUPT_MASK	(1 << 11)
#define UART1_INTERRUPT_MASK	(1 << 8)
#define UART2_INTERRUPT_MASK	(1 << 9)
#define UART3_INTERRUPT_MASK	(1 << 10)
#define TWL4030_MSECURE_GPIO	22
int console_detect(char *str);
unsigned int uart_interrupt_mask_value;
#endif

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

#ifdef CONFIG_OMAP3_PM
/*
 * Board DDR timings used during frequency changes
 */
struct dvfs_config omap3_vdd2_config[PRCM_NO_VDD2_OPPS] = {
#ifdef CONFIG_OMAP3_CORE_166MHZ
	{
	/* SDRC CS0/CS1 values 83MHZ*/
	/* not optimized at 1/2 speed except for RFR */
	{{0x00025801, 0x629db4c6, 0x00012214},   /* cs 0 */
	 {0x00025801, 0x629db4c6, 0x00012214} }, /* cs 1*/
	},

	/* SDRC CS0/CS1 values 166MHZ*/
	{
	{{0x0004e201, 0x629db4c6, 0x00012214},
	 {0x0004e201, 0x629db4c6, 0x00012214} },
	},
#endif
};

#endif /* CONFIG_OMAP3_PM */

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

#ifdef CONFIG_OMAP3_PM
static u32 *uart_detect(void)
{
	char str[7];
	u32 *temp_ptr = 0;
	if (console_detect(str))
		printk(KERN_INFO"Invalid console paramter....\n");

	if (!strcmp(str, "ttyS0")) {
		temp_ptr = (u32 *)(&CONTROL_PADCONF_UART1_CTS);
		uart_interrupt_mask_value = UART1_INTERRUPT_MASK;
		}
	else if (!strcmp(str, "ttyS1")) {
		temp_ptr = (u32 *)(&CONTROL_PADCONF_UART2_TX);
		uart_interrupt_mask_value = UART2_INTERRUPT_MASK;
		}
	else if (!strcmp(str, "ttyS2")) {
		temp_ptr = (u32 *)(&CONTROL_PADCONF_UART3_RTS_SD);
		uart_interrupt_mask_value = UART3_INTERRUPT_MASK;
		}
	else
		printk(KERN_INFO
		"!!!!!!!!! Unable to recongnize Console UART........\n");
	return (u32 *)(temp_ptr);
}

void  init_wakeupconfig(void)
{
	u32 *ptr;
	ptr = (u32 *)uart_detect();
	*ptr |= (u32)((IO_PAD_WAKEUPENABLE | IO_PAD_OFFPULLUDENABLE |
			IO_PAD_OFFOUTENABLE | IO_PAD_OFFENABLE |
			IO_PAD_INPUTENABLE | IO_PAD_PULLUDENABLE)
				<<  IO_PAD_HIGH_SHIFT);

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
#endif /* CONFIG_OMAP3_PM */

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

static struct twl4030_keypad_data sdp3430_kp_data = {
	.rows		= 5,
	.cols		= 6,
	.keymap		= sdp3430_keymap,
	.keymapsize	= ARRAY_SIZE(sdp3430_keymap),
	.rep		= 1,
	.irq		= TWL4030_MODIRQ_KEYPAD,
};

static int ts_gpio;

static int __init msecure_init(void)
{
	int ret = 0;

#ifdef CONFIG_RTC_DRV_TWL4030
	/* 3430ES2.0 doesn't have msecure/gpio-22 line connected to T2 */
	if (omap_type() == OMAP2_DEVICE_TYPE_GP &&
			system_rev < OMAP3430_REV_ES2_0) {
		u32 msecure_pad_config_reg = omap_ctrl_base_get() + 0xA3C;
		int mux_mask = 0x04;
		u16 tmp;

		ret = gpio_request(TWL4030_MSECURE_GPIO, "msecure");
		if (ret < 0) {
			printk(KERN_ERR "msecure_init: can't"
				"reserve GPIO:%d !\n", TWL4030_MSECURE_GPIO);
			goto out;
		}
		/*
		 * TWL4030 will be in secure mode if msecure line from OMAP
		 * is low. Make msecure line high in order to change the
		 * TWL4030 RTC time and calender registers.
		 */
		tmp = omap_readw(msecure_pad_config_reg);
		tmp &= 0xF8;	/* To enable mux mode 03/04 = GPIO_RTC */
		tmp |= mux_mask;/* To enable mux mode 03/04 = GPIO_RTC */
		omap_writew(tmp, msecure_pad_config_reg);

		gpio_direction_output(TWL4030_MSECURE_GPIO, 1);
	}
out:
#endif
	return ret;
}

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

#ifdef CONFIG_VIDEO_DW9710
static int dw9710_lens_power_set(enum v4l2_power power)
{

	return 0;
}

static int dw9710_lens_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_LENS;

	return 0;
}

static struct dw9710_platform_data sdp3430_dw9710_platform_data = {
	.power_set      = dw9710_lens_power_set,
	.priv_data_set  = dw9710_lens_set_prv_data,
};
#endif

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
static void __iomem *fpga_map_addr;

static struct omap34xxcam_sensor_config cam_hwc = {
	.sensor_isp = 0,
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

	hwc->u.sensor.xclk = cam_hwc.xclk;
	hwc->u.sensor.sensor_isp = cam_hwc.sensor_isp;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static struct isp_interface_config mt9p012_if_config = {
	.ccdc_par_ser = ISP_PARLL,
	.dataline_shift = 0x1,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.vdint0_timing = 0x0,
	.vdint1_timing = 0x0,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.u.par.par_bridge = 0x0,
	.u.par.par_clk_pol = 0x0,
};

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
		isp_configure_interface(&mt9p012_if_config);

		/* Request and configure gpio pins */
		if (omap_request_gpio(MT9P012_STANDBY_GPIO) != 0) {
			printk(KERN_WARNING "Could not request GPIO %d for "
					"AF D88\n", MT9P012_STANDBY_GPIO);
			return -EIO;
		}

		/* Request and configure gpio pins */
		if (omap_request_gpio(MT9P012_RESET_GPIO) != 0)
			return -EIO;

		/* set to output mode */
		omap_set_gpio_direction(MT9P012_STANDBY_GPIO, 0);
		/* set to output mode */
		omap_set_gpio_direction(MT9P012_RESET_GPIO, 0);

		/* STANDBY_GPIO is active HIGH for set LOW to release */
		omap_set_gpio_dataout(MT9P012_STANDBY_GPIO, 1);

		/* nRESET is active LOW. set HIGH to release reset */
		omap_set_gpio_dataout(MT9P012_RESET_GPIO, 1);

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
	.default_regs   = NULL,
	.ifparm         = mt9p012_ifparm,
};


static struct i2c_board_info __initdata sdp3430_i2c2_boardinfo[] = {
#ifdef CONFIG_VIDEO_DW9710
	{
		I2C_BOARD_INFO(DW9710_NAME,  DW9710_AF_I2C_ADDR),
		.platform_data = &sdp3430_dw9710_platform_data,
	},
#endif
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
	&sdp3430_lcd_device,
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

	if (system_rev > OMAP3430_REV_ES1_0)
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
	omap2_init_common_hw(hyb18m512160af6_sdrc_params);
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

static struct omap_board_config_kernel sdp3430_config[] __initdata = {
	{ OMAP_TAG_UART,	&sdp3430_uart_config },
	{ OMAP_TAG_LCD,		&sdp3430_lcd_config },
};

static int sdp3430_batt_table[] = {
/* 0 C*/
30800, 29500, 28300, 27100,
26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
11600, 11200, 10800, 10400, 10000, 9630,   9280,   8950,   8620,   8310,
8020,   7730,   7460,   7200,   6950,   6710,   6470,   6250,   6040,   5830,
5640,   5450,   5260,   5090,   4920,   4760,   4600,   4450,   4310,   4170,
4040,   3910,   3790,   3670,   3550
};

static struct twl4030_bci_platform_data sdp3430_bci_data = {
      .battery_tmp_tbl	= sdp3430_batt_table,
      .tblsize		= ARRAY_SIZE(sdp3430_batt_table),
};

static struct twl4030_gpio_platform_data sdp3430_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
};

static struct twl4030_usb_data sdp3430_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_madc_platform_data sdp3430_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data sdp3430_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci		= &sdp3430_bci_data,
	.gpio		= &sdp3430_gpio_data,
	.madc		= &sdp3430_madc_data,
	.keypad		= &sdp3430_kp_data,
	.usb		= &sdp3430_usb_data,
};

static struct i2c_board_info __initdata sdp3430_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &sdp3430_twldata,
	},
};

static int __init omap3430_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, sdp3430_i2c_boardinfo,
			ARRAY_SIZE(sdp3430_i2c_boardinfo));
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
	omap_register_i2c_bus(2, 400, sdp3430_i2c2_boardinfo,
			      ARRAY_SIZE(sdp3430_i2c2_boardinfo));
#else
	omap_register_i2c_bus(2, 400, NULL, 0);
#endif
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

extern void __init sdp3430_flash_init(void);

static void __init omap_3430sdp_init(void)
{
	omap3430_i2c_init();
	platform_add_devices(sdp3430_devices, ARRAY_SIZE(sdp3430_devices));
	omap_board_config = sdp3430_config;
	omap_board_config_size = ARRAY_SIZE(sdp3430_config);
	if (system_rev > OMAP3430_REV_ES1_0)
		ts_gpio = OMAP34XX_TS_GPIO_IRQ_SDPV2;
	else
		ts_gpio = OMAP34XX_TS_GPIO_IRQ_SDPV1;
	sdp3430_spi_board_info[0].irq = OMAP_GPIO_IRQ(ts_gpio);
	spi_register_board_info(sdp3430_spi_board_info,
				ARRAY_SIZE(sdp3430_spi_board_info));
	ads7846_dev_init();
	sdp3430_flash_init();
	msecure_init();
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
