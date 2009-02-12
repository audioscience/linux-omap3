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
#include <linux/mm.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/board.h>
#include <mach/usb-musb.h>
#include <mach/usb-ehci.h>
#include <mach/common.h>
#include <mach/keypad.h>
#include <mach/dma.h>
#include <mach/gpmc.h>
#include <mach/omap-pm.h>
#include <mach/display.h>

#ifdef CONFIG_VIDEO_OMAP3_CAM
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#define REG_SDP3430_FPGA_GPIO_2 (0x50)
#define FPGA_SPR_GPIO1_3v3	(0x1 << 14)
#define FPGA_GPIO6_DIR_CTRL	(0x1 << 6)
static void __iomem *fpga_map_addr;
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
/* Sensor specific GPIO signals */
#define MT9P012_RESET_GPIO  	98
#define MT9P012_STANDBY_GPIO	58

#define MT9P012_USE_XCLKA  	0
#define MT9P012_USE_XCLKB  	1

#define VAUX_2_8_V		0x09
#define VAUX_DEV_GRP_P1		0x20
#define VAUX_DEV_GRP_NONE	0x00

#include <media/mt9p012.h>
static enum v4l2_power mt9p012_previous_power = V4L2_POWER_OFF;
#endif
#if defined(CONFIG_VIDEO_OV3640) || defined(CONFIG_VIDEO_OV3640_MODULE)
#include <../drivers/media/video/ov3640.h>
#include <../drivers/media/video/isp/ispcsi2.h>
static	struct omap34xxcam_hw_config *hwc;
#define OV3640_CSI2_CLOCK_POLARITY	0	/* +/- pin order */
#define OV3640_CSI2_DATA0_POLARITY	0	/* +/- pin order */
#define OV3640_CSI2_DATA1_POLARITY	0	/* +/- pin order */
#define OV3640_CSI2_CLOCK_LANE		1	 /* Clock lane position: 1 */
#define OV3640_CSI2_DATA0_LANE		2	 /* Data0 lane position: 2 */
#define OV3640_CSI2_DATA1_LANE		3	 /* Data1 lane position: 3 */
#define OV3640_CSI2_PHY_THS_TERM	4
#define OV3640_CSI2_PHY_THS_SETTLE	14
#define OV3640_CSI2_PHY_TCLK_TERM	0
#define OV3640_CSI2_PHY_TCLK_MISS	1
#define OV3640_CSI2_PHY_TCLK_SETTLE	14
#endif
#endif

#ifdef CONFIG_VIDEO_DW9710
#include <media/dw9710.h>
#endif

#include <asm/io.h>
#include <asm/delay.h>
#include <mach/control.h>
#include <mach/clock.h>

#include "sdram-qimonda-hyb18m512160af-6.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "omap3-opp.h"

#define CONFIG_DISABLE_HFCLK 1

#define	SDP3430_SMC91X_CS	3

#define ENABLE_VAUX3_DEDICATED	0x03
#define ENABLE_VAUX3_DEV_GRP	0x20

#define TWL4030_MSECURE_GPIO 22

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
};

static int ts_gpio;

static int __init msecure_init(void)
{
	int ret = 0;

#ifdef CONFIG_RTC_DRV_TWL4030
	/* 3430ES2.0 doesn't have msecure/gpio-22 line connected to T2 */
	if (omap_type() == OMAP2_DEVICE_TYPE_GP &&
			omap_rev() < OMAP3430_REV_ES2_0) {
		void __iomem *msecure_pad_config_reg = omap_ctrl_base_get() +
			0xA3C;
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
		tmp = __raw_readw(msecure_pad_config_reg);
		tmp &= 0xF8; /* To enable mux mode 03/04 = GPIO_RTC */
		tmp |= mux_mask;/* To enable mux mode 03/04 = GPIO_RTC */
		__raw_writew(tmp, msecure_pad_config_reg);

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
	if (gpio_request(ts_gpio, "ADS7846 pendown") < 0) {
		printk(KERN_ERR "can't get ads746 pen down GPIO\n");
		return;
	}

	gpio_direction_input(ts_gpio);

	omap_set_gpio_debounce(ts_gpio, 1);
	omap_set_gpio_debounce_time(ts_gpio, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(ts_gpio);
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

#ifdef CONFIG_VIDEO_OMAP3_CAM
#define DEBUG_BASE		0x08000000
#define REG_SDP3430_FPGA_GPIO_2 (0x50)
#define FPGA_SPR_GPIO1_3v3	(0x1 << 14)
#define FPGA_GPIO6_DIR_CTRL	(0x1 << 6)

static void __iomem *fpga_map_addr;

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
#endif

#ifdef CONFIG_VIDEO_DW9710
static int dw9710_lens_power_set(enum v4l2_power power)
{

	/* The power change depends on MT9P012 powerup, so if we request a
	 * power state different from sensor, we should return error
	 */
	if ((mt9p012_previous_power != V4L2_POWER_OFF) &&
					(power != mt9p012_previous_power))
		return -EIO;

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
		/* Request and configure gpio pins */
		if (omap_request_gpio(MT9P012_STANDBY_GPIO) != 0) {
			printk(KERN_WARNING "Could not request GPIO %d"
						" for MT9P012\n",
						MT9P012_STANDBY_GPIO);
			return -EIO;
		}

		/* Request and configure gpio pins */
		if (omap_request_gpio(MT9P012_RESET_GPIO) != 0)
			return -EIO;

		/* set to output mode */
		gpio_direction_output(MT9P012_STANDBY_GPIO, true);
		/* set to output mode */
		gpio_direction_output(MT9P012_RESET_GPIO, true);

		/* STANDBY_GPIO is active HIGH for set LOW to release */
		gpio_set_value(MT9P012_STANDBY_GPIO, 1);

		/* nRESET is active LOW. set HIGH to release reset */
		gpio_set_value(MT9P012_RESET_GPIO, 1);

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
		/* out of standby */
		gpio_set_value(MT9P012_STANDBY_GPIO, 0);
		udelay(1000);

		/* have to put sensor to reset to guarantee detection */
		gpio_set_value(MT9P012_RESET_GPIO, 0);

		udelay(1500);

		/* nRESET is active LOW. set HIGH to release reset */
		gpio_set_value(MT9P012_RESET_GPIO, 1);
		/* give sensor sometime to get out of the reset.
		 * Datasheet says 2400 xclks. At 6 MHz, 400 usec is
		 * enough
		 */
		udelay(300);
		break;
	case V4L2_POWER_STANDBY:
		break;
	}
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
static struct omap34xxcam_sensor_config cam_hwc = {
	.sensor_isp = 0,
	.xclk = OMAP34XXCAM_XCLK_A,
	.capture_mem = PAGE_ALIGN(2592 * 1944 * 2) * 4,
};

static int mt9p012_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.xclk = cam_hwc.xclk;
	hwc->u.sensor.sensor_isp = cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = cam_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static struct isp_interface_config mt9p012_if_config = {
	.ccdc_par_ser = ISP_PARLL,
	.dataline_shift = 0x1,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.prev_sph = 2,
	.prev_slv = 0,
	.wenlog = ISPCCDC_CFG_WENLOG_OR,
	.wait_hs_vs = 2,
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
		if (mt9p012_previous_power == V4L2_POWER_OFF) {
			/* Power Up Sequence */
			isp_configure_interface(&mt9p012_if_config);

			/* Request and configure gpio pins */
			if (omap_request_gpio(MT9P012_STANDBY_GPIO) != 0) {
				printk(KERN_WARNING "Could not request GPIO %d"
							" for MT9P012\n",
							MT9P012_STANDBY_GPIO);
				return -EIO;
			}

			/* Request and configure gpio pins */
			if (omap_request_gpio(MT9P012_RESET_GPIO) != 0)
				return -EIO;

			/* set to output mode */
			gpio_direction_output(MT9P012_STANDBY_GPIO, true);
			/* set to output mode */
			gpio_direction_output(MT9P012_RESET_GPIO, true);

			/* STANDBY_GPIO is active HIGH for set LOW to release */
			gpio_set_value(MT9P012_STANDBY_GPIO, 1);

			/* nRESET is active LOW. set HIGH to release reset */
			gpio_set_value(MT9P012_RESET_GPIO, 1);

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
		}

		/* out of standby */
		gpio_set_value(MT9P012_STANDBY_GPIO, 0);
		udelay(1000);

		if (mt9p012_previous_power == V4L2_POWER_OFF) {
			/* have to put sensor to reset to guarantee detection */
			gpio_set_value(MT9P012_RESET_GPIO, 0);

			udelay(1500);

			/* nRESET is active LOW. set HIGH to release reset */
			gpio_set_value(MT9P012_RESET_GPIO, 1);
			/* give sensor sometime to get out of the reset.
			 * Datasheet says 2400 xclks. At 6 MHz, 400 usec is
			 * enough
			 */
			udelay(300);
		}
		break;
	case V4L2_POWER_STANDBY:
		/* stand by */
		gpio_set_value(MT9P012_STANDBY_GPIO, 1);
		break;
	}
	/* Save powerstate to know what was before calling POWER_ON. */
	mt9p012_previous_power = power;
	return 0;
}

static u32 mt9p012_sensor_set_xclk(u32 xclkfreq)
{
	return isp_set_xclk(xclkfreq, MT9P012_USE_XCLKA);
}

static struct mt9p012_platform_data sdp3430_mt9p012_platform_data = {
	.power_set      = mt9p012_sensor_power_set,
	.priv_data_set  = mt9p012_sensor_set_prv_data,
	.set_xclk       = mt9p012_sensor_set_xclk,
};

#endif

#if defined(CONFIG_VIDEO_OV3640) || defined(CONFIG_VIDEO_OV3640_MODULE)

static struct omap34xxcam_sensor_config ov3640_hwc = {
	.sensor_isp = 0,
#if defined(CONFIG_VIDEO_OV3640_CSI2)
	.xclk = OMAP34XXCAM_XCLK_B,
#else
	.xclk = OMAP34XXCAM_XCLK_A,
#endif
	.capture_mem = PAGE_ALIGN(2048 * 1536 * 2) * 2,
};

static struct isp_interface_config ov3640_if_config = {
	.ccdc_par_ser = ISP_CSIA,
	.dataline_shift = 0x0,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.prev_sph = 2,
	.prev_slv = 1,
	.wenlog = ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs = 2,
	.u.csi.crc = 0x0,
	.u.csi.mode = 0x0,
	.u.csi.edge = 0x0,
	.u.csi.signalling = 0x0,
	.u.csi.strobe_clock_inv = 0x0,
	.u.csi.vs_edge = 0x0,
	.u.csi.channel = 0x1,
	.u.csi.vpclk = 0x1,
	.u.csi.data_start = 0x0,
	.u.csi.data_size = 0x0,
	.u.csi.format = V4L2_PIX_FMT_SGRBG10,
};

static int ov3640_sensor_set_prv_data(void *priv)
{

	hwc = priv;
	hwc->u.sensor.xclk = ov3640_hwc.xclk;
	hwc->u.sensor.sensor_isp = ov3640_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = ov3640_hwc.capture_mem;
	hwc->dev_index = 1;
	hwc->dev_minor = 4;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static int ov3640_sensor_power_set(enum v4l2_power power)
{
	struct isp_csi2_lanes_cfg lanecfg;
	struct isp_csi2_phy_cfg phyconfig;
	static enum v4l2_power previous_power = V4L2_POWER_OFF;
	switch (power) {
	case V4L2_POWER_ON:
		if (previous_power == V4L2_POWER_OFF)
			isp_csi2_reset();

		lanecfg.clk.pol = OV3640_CSI2_CLOCK_POLARITY;
		lanecfg.clk.pos = OV3640_CSI2_CLOCK_LANE;
	 	lanecfg.data[0].pol = OV3640_CSI2_DATA0_POLARITY;
		lanecfg.data[0].pos = OV3640_CSI2_DATA0_LANE;
		lanecfg.data[1].pol = OV3640_CSI2_DATA1_POLARITY;
		lanecfg.data[1].pos = OV3640_CSI2_DATA1_LANE;
		lanecfg.data[2].pol = 0;
		lanecfg.data[2].pos = 0;
		lanecfg.data[3].pol = 0;
		lanecfg.data[3].pos = 0;
		isp_csi2_complexio_lanes_config(&lanecfg);
		isp_csi2_complexio_lanes_update(true);

		phyconfig.ths_term = OV3640_CSI2_PHY_THS_TERM;
		phyconfig.ths_settle = OV3640_CSI2_PHY_THS_SETTLE;
		phyconfig.tclk_term = OV3640_CSI2_PHY_TCLK_TERM;
		phyconfig.tclk_miss = OV3640_CSI2_PHY_TCLK_MISS;
		phyconfig.tclk_settle = OV3640_CSI2_PHY_TCLK_SETTLE;
		isp_csi2_phy_config(&phyconfig);
		isp_csi2_phy_update(true);

		isp_configure_interface(&ov3640_if_config);

		if (previous_power == V4L2_POWER_OFF) {

#ifdef CONFIG_TWL4030_CORE
			/* turn on analog power */
#if defined(CONFIG_VIDEO_OV3640_CSI2)
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_1_8_V, TWL4030_VAUX4_DEDICATED);
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX4_DEV_GRP);
#else
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_2_8_V, TWL4030_VAUX2_DEDICATED);
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX2_DEV_GRP);
#endif
			udelay(100);
#else
#error "no power companion board defined!"
#endif
			/* Request and configure gpio pins */
			if (omap_request_gpio(OV3640_RESET_GPIO) != 0) {
				printk(KERN_ERR "Could not request GPIO %d",
					OV3640_RESET_GPIO);
				return -EIO;
			}
			if (omap_request_gpio(OV3640_STANDBY_GPIO) != 0) {
				printk(KERN_ERR "Could not request GPIO %d",
					OV3640_STANDBY_GPIO);
				return -EIO;
			}
			/* set to output mode */
			gpio_direction_output(OV3640_RESET_GPIO, true);
			gpio_direction_output(OV3640_STANDBY_GPIO, true);

			/* Turn ON Omnivision sensor */
			gpio_set_value(OV3640_RESET_GPIO, 1);
			gpio_set_value(OV3640_STANDBY_GPIO, 0);
			udelay(100);

			/* RESET Omnivision sensor */
			gpio_set_value(OV3640_RESET_GPIO, 0);
			udelay(100);
			gpio_set_value(OV3640_RESET_GPIO, 1);

			/* Wait 10 ms */
			mdelay(10);
			enable_fpga_vio_1v8(1);
			udelay(100);
		}
		break;
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		isp_csi2_complexio_power(ISP_CSI2_POWER_OFF);
#ifdef CONFIG_TWL4030_CORE
#if defined(CONFIG_VIDEO_OV3640_CSI2)
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX4_DEV_GRP);
#else
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX2_DEV_GRP);
#endif
#else
#error "no power companion board defined!"
#endif
		enable_fpga_vio_1v8(0);
		omap_free_gpio(OV3640_RESET_GPIO);
		iounmap(fpga_map_addr);
		omap_free_gpio(OV3640_STANDBY_GPIO);
		break;
	case V4L2_POWER_STANDBY:
		break;
	}
	previous_power = power;
	return 0;
}

static struct ov3640_platform_data sdp3430_ov3640_platform_data = {
	.power_set	 = ov3640_sensor_power_set,
	.priv_data_set	 = ov3640_sensor_set_prv_data,
	.default_regs	 = ov3640_common[0],
};

#endif

#define SDP2430_LCD_PANEL_BACKLIGHT_GPIO	91
#define SDP2430_LCD_PANEL_ENABLE_GPIO		154
#define SDP3430_LCD_PANEL_BACKLIGHT_GPIO	24
#define SDP3430_LCD_PANEL_ENABLE_GPIO		28

#define PM_RECEIVER             TWL4030_MODULE_PM_RECEIVER
#define ENABLE_VAUX2_DEDICATED  0x09
#define ENABLE_VAUX2_DEV_GRP    0x20
#define ENABLE_VAUX3_DEDICATED	0x03
#define ENABLE_VAUX3_DEV_GRP	0x20

#define ENABLE_VPLL2_DEDICATED	0x05
#define ENABLE_VPLL2_DEV_GRP	0xE0
#define TWL4030_VPLL2_DEV_GRP	0x33
#define TWL4030_VPLL2_DEDICATED	0x36

#define t2_out(c, r, v) twl4030_i2c_write_u8(c, r, v)

static unsigned backlight_gpio;
static unsigned enable_gpio;
static int lcd_enabled;
static int dvi_enabled;

static void __init sdp3430_display_init(void)
{
	int r;

	enable_gpio    = SDP3430_LCD_PANEL_ENABLE_GPIO;
	backlight_gpio = SDP3430_LCD_PANEL_BACKLIGHT_GPIO;

	r = gpio_request(enable_gpio, "LCD reset");
	if (r) {
		printk(KERN_ERR "failed to get LCD reset GPIO\n");
		goto err0;
	}

	r = gpio_request(backlight_gpio, "LCD Backlight");
	if (r) {
		printk(KERN_ERR "failed to get LCD backlight GPIO\n");
		goto err1;
	}

	gpio_direction_output(enable_gpio, 0);
	gpio_direction_output(backlight_gpio, 0);

	return;
err1:
	gpio_free(enable_gpio);
err0:
	return;
}

static int sdp3430_panel_enable_lcd(struct omap_display *display)
{
	u8 ded_val, ded_reg;
	u8 grp_val, grp_reg;

	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}

	if (omap_rev() > OMAP3430_REV_ES1_0) {
		t2_out(PM_RECEIVER, ENABLE_VPLL2_DEDICATED,
				TWL4030_VPLL2_DEDICATED);
		t2_out(PM_RECEIVER, ENABLE_VPLL2_DEV_GRP,
				TWL4030_VPLL2_DEV_GRP);
	}

	ded_reg = TWL4030_VAUX3_DEDICATED;
	ded_val = ENABLE_VAUX3_DEDICATED;
	grp_reg = TWL4030_VAUX3_DEV_GRP;
	grp_val = ENABLE_VAUX3_DEV_GRP;

	gpio_direction_output(enable_gpio, 1);
	gpio_direction_output(backlight_gpio, 1);

	if (0 != t2_out(PM_RECEIVER, ded_val, ded_reg))
		return -EIO;
	if (0 != t2_out(PM_RECEIVER, grp_val, grp_reg))
		return -EIO;

	lcd_enabled = 1;

	return 0;
}

static void sdp3430_panel_disable_lcd(struct omap_display *display)
{
	lcd_enabled = 0;

	gpio_direction_output(enable_gpio, 0);
	gpio_direction_output(backlight_gpio, 0);

	if (omap_rev() > OMAP3430_REV_ES1_0) {
		t2_out(PM_RECEIVER, 0x0, TWL4030_VPLL2_DEDICATED);
		t2_out(PM_RECEIVER, 0x0, TWL4030_VPLL2_DEV_GRP);
		mdelay(4);
	}
}

static struct omap_display_data sdp3430_display_data = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.panel_name = "sharp-ls037v7dw01",
	.u.dpi.data_lines = 16,
	.panel_enable = sdp3430_panel_enable_lcd,
	.panel_disable = sdp3430_panel_disable_lcd,
};

static int sdp3430_panel_enable_dvi(struct omap_display *display)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}

	if (omap_rev() > OMAP3430_REV_ES1_0) {
		t2_out(PM_RECEIVER, ENABLE_VPLL2_DEDICATED,
				TWL4030_VPLL2_DEDICATED);
		t2_out(PM_RECEIVER, ENABLE_VPLL2_DEV_GRP,
				TWL4030_VPLL2_DEV_GRP);
	}

	dvi_enabled = 1;

	return 0;
}

static void sdp3430_panel_disable_dvi(struct omap_display *display)
{
	dvi_enabled = 0;

	if (omap_rev() > OMAP3430_REV_ES1_0) {
		t2_out(PM_RECEIVER, 0x0, TWL4030_VPLL2_DEDICATED);
		t2_out(PM_RECEIVER, 0x0, TWL4030_VPLL2_DEV_GRP);
		mdelay(4);
	}
}

static struct omap_display_data sdp3430_display_data_dvi = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.panel_name = "panel-generic",
	.u.dpi.data_lines = 24,
	.panel_enable = sdp3430_panel_enable_dvi,
	.panel_disable = sdp3430_panel_disable_dvi,
>>>>>>> 91be542... DSS: Support for OMAP3 SDP board:arch/arm/mach-omap2/board-3430sdp.c
};

static int sdp3430_panel_enable_tv(struct omap_display *display)
{
#define ENABLE_VDAC_DEDICATED           0x03
#define ENABLE_VDAC_DEV_GRP             0x20

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEDICATED,
			TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEV_GRP, TWL4030_VDAC_DEV_GRP);

	return 0;
}

static void sdp3430_panel_disable_tv(struct omap_display *display)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEV_GRP);
}

static struct omap_display_data sdp3430_display_data_tv = {
	.type = OMAP_DISPLAY_TYPE_VENC,
	.name = "tv",
	.u.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
	.panel_enable = sdp3430_panel_enable_tv,
	.panel_disable = sdp3430_panel_disable_tv,
};

static struct omap_dss_platform_data sdp3430_dss_data = {
	.num_displays = 3,
	.displays = {
		&sdp3430_display_data,
		&sdp3430_display_data_dvi,
		&sdp3430_display_data_tv,
	}
};

static struct platform_device sdp3430_dss_device = {
	.name          = "omap-dss",
	.id            = -1,
	.dev            = {
		.platform_data = &sdp3430_dss_data,
	},
};

static struct platform_device *sdp3430_devices[] __initdata = {
	&sdp3430_smc91x_device,
	&sdp3430_dss_device,
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

	if (omap_rev() > OMAP3430_REV_ES1_0)
		eth_gpio = OMAP34XX_ETHR_GPIO_IRQ_SDPV2;
	else
		eth_gpio = OMAP34XX_ETHR_GPIO_IRQ_SDPV1;

	sdp3430_smc91x_resources[1].start = gpio_to_irq(eth_gpio);

	if (gpio_request(eth_gpio, "SMC91x irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc91x IRQ\n",
			eth_gpio);
		return;
	}
	gpio_direction_input(eth_gpio);
}

static void __init omap_3430sdp_init_irq(void)
{
	omap2_init_common_hw(hyb18m512160af6_sdrc_params, omap3_mpu_rate_table,
			     omap3_dsp_rate_table, omap3_l3_rate_table);
	omap_init_irq();
	omap_gpio_init();
	sdp3430_init_smc91x();
}

static struct omap_uart_config sdp3430_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_board_config_kernel sdp3430_config[] __initdata = {
	{ OMAP_TAG_UART,	&sdp3430_uart_config },
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

static struct twl4030_ins __initdata sleep_on_seq[] = {
/*
 * Turn off VDD1 and VDD2.
 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_OFF), 4},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_OFF), 2},
#ifdef CONFIG_DISABLE_HFCLK
/*
 * And also turn off the OMAP3 PLLs and the sysclk output.
 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_OFF), 3},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_OFF), 3},
#endif
};

static struct twl4030_script sleep_on_script __initdata = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TRITON_SLEEP_SCRIPT,
};

static struct twl4030_ins wakeup_seq[] __initdata = {
#ifndef CONFIG_DISABLE_HFCLK
/*
 * Wakeup VDD1 and VDD2.
 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_ACTIVE), 4},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_ACTIVE), 2},
#else
/*
 * Reenable the OMAP3 PLLs.
 * Wakeup VDD1 and VDD2.
 * Reenable sysclk output.
 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_ACTIVE), 0x30},
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_ACTIVE), 0x30},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_ACTIVE), 0x37},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 3},
#endif /* #ifndef CONFIG_DISABLE_HFCLK */
};

static struct twl4030_script wakeup_script __initdata = {
	.script	= wakeup_seq,
	.size	= ARRAY_SIZE(wakeup_seq),
	.flags	= TRITON_WAKEUP12_SCRIPT | TRITON_WAKEUP3_SCRIPT,
};

static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset VDD1 regulator.
 * Reset VDD2 regulator.
 * Reset VPLL1 regulator.
 * Enable sysclk output.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_WRST), 0x60},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_ACTIVE), 2},
};
static struct twl4030_script wrst_script __initdata = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wakeup_seq),
	.flags  = TRITON_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] __initdata = {
	&sleep_on_script,
	&wakeup_script,
	&wrst_script,
};

static struct twl4030_power_data sdp3430_t2scripts_data __initdata = {
	.scripts	= twl4030_scripts,
	.size		= ARRAY_SIZE(twl4030_scripts),
};

static struct twl4030_platform_data sdp3430_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci		= &sdp3430_bci_data,
	.gpio		= &sdp3430_gpio_data,
	.madc		= &sdp3430_madc_data,
	.keypad		= &sdp3430_kp_data,
	.power		= &sdp3430_t2scripts_data,
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

static struct i2c_board_info __initdata sdp3430_i2c_boardinfo_2[] = {
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
	{
		I2C_BOARD_INFO("mt9p012", MT9P012_I2C_ADDR),
		.platform_data = &sdp3430_mt9p012_platform_data,
	},
#ifdef CONFIG_VIDEO_DW9710
	{
		I2C_BOARD_INFO(DW9710_NAME, DW9710_AF_I2C_ADDR),
		.platform_data = &sdp3430_dw9710_platform_data,
	},
#endif
#endif
#if defined(CONFIG_VIDEO_OV3640) || defined(CONFIG_VIDEO_OV3640_MODULE)
	{
		I2C_BOARD_INFO("ov3640", OV3640_I2C_ADDR),
		.platform_data = &sdp3430_ov3640_platform_data,
	},
#endif
};

#define TWL4030_VAUX4_DEV_GRP	0x23
#define TWL4030_VAUX4_DEDICATED	0x26

static int __init omap3430_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, sdp3430_i2c_boardinfo,
			ARRAY_SIZE(sdp3430_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, sdp3430_i2c_boardinfo_2,
			ARRAY_SIZE(sdp3430_i2c_boardinfo_2));
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static struct twl4030_hsmmc_info mmc[] __initdata = {
	{
		.mmc		= 1,
		.wires		= 8,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 2,
		.wires		= 8,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}	/* Terminator */
};

extern void __init sdp3430_flash_init(void);

static void __init omap_3430sdp_init(void)
{
	omap3430_i2c_init();
	platform_add_devices(sdp3430_devices, ARRAY_SIZE(sdp3430_devices));
	omap_board_config = sdp3430_config;
	omap_board_config_size = ARRAY_SIZE(sdp3430_config);
	if (omap_rev() > OMAP3430_REV_ES1_0)
		ts_gpio = OMAP34XX_TS_GPIO_IRQ_SDPV2;
	else
		ts_gpio = OMAP34XX_TS_GPIO_IRQ_SDPV1;
	sdp3430_spi_board_info[0].irq = gpio_to_irq(ts_gpio);
	spi_register_board_info(sdp3430_spi_board_info,
				ARRAY_SIZE(sdp3430_spi_board_info));
	ads7846_dev_init();
	sdp3430_flash_init();
	msecure_init();
	omap_serial_init();
	usb_musb_init();
	usb_ehci_init();
	twl4030_mmc_init(mmc);
	sdp3430_display_init();
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
