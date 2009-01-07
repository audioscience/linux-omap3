/*
 * arch/arm/mach-omap2/board-omap3evm-dc.c
 *
 * Driver for OMAP3 EVM Daughter Card
 *
 * Copyright (C) 2008 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * Contributors:
 *     Anuj Aggarwal <anuj.aggarwal@ti.com>
 *     Sivaraj R <sivaraj@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <mach/io.h>
#include <mach/mux.h>

#if defined(CONFIG_VIDEO_TVP514X) || defined(CONFIG_VIDEO_TVP514X_MODULE)
#include <linux/videodev2.h>
#include <media/v4l2-int-device.h>
#include <media/tvp514x.h>
/* include V4L2 camera driver related header file */
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#endif				/* #ifdef CONFIG_VIDEO_OMAP3 */
#endif				/* #ifdef CONFIG_VIDEO_TVP514X*/

#include "board-omap3evm-dc.h"

#define MODULE_NAME			"omap3evmdc"

#ifdef DEBUG
#define dprintk(fmt, args...) printk(KERN_ERR MODULE_NAME ": " fmt, ## args)
#else
#define dprintk(fmt, args...)
#endif				/* #ifdef DEBUG */

/* Macro Definitions */

/* System control module register offsets */
#define REG_CONTROL_PADCONF_I2C2_SDA	(0x480021C0u)
#define REG_CONTROL_PADCONF_I2C3_SDA	(0x480021C4u)

#define PADCONF_I2C3_SCL_MASK		(0xFFFF0000u)
#define PADCONF_I2C3_SDA_MASK		(0x0000FFFFu)

/* mux mode 0 (enable I2C3 SCL), pull-up enable, input enable */
#define PADCONF_I2C3_SCL_DEF		(0x01180000u)
/* mux mode 0 (enable I2C3 SDA), pull-up enable, input enable */
#define PADCONF_I2C3_SDA_DEF		(0x00000118u)

/* GPIO pins */
#define GPIO134_SEL_Y                   (134)
#define GPIO54_SEL_EXP_CAM              (54)
#define GPIO136_SEL_CAM                 (136)

/* board internal information (BEGIN) */

/* I2C bus to which all I2C slave devices are attached */
#define BOARD_I2C_BUSNUM		(3)

/* I2C address of chips present in board */
#define TVP5146_I2C_ADDR		(0x5D)

/* Register offsets */
#define REG_BUS_CTRL1			(0x00000180u)
#define REG_BUS_CTRL2			(0x000001C0u)

/* Bit defines for Bus Control 1 register */
#define TVP5146_EN_SHIFT		(0x0000u)
#define TVP5146_EN_MASK			(1u << TVP5146_EN_SHIFT)

#define CAMERA_SENSOR_EN_SHIFT		(0x0008u)
#define CAMERA_SENSOR_EN_MASK		(1u << CAMERA_SENSOR_EN_SHIFT)

/* default value for bus control registers */
#define BUS_CONTROL1_DEF		(0x0141u)	/* Disable all mux */
#define BUS_CONTROL2_DEF		(0x010Au)	/* Disable all mux */

#if defined(CONFIG_VIDEO_TVP514X) || defined(CONFIG_VIDEO_TVP514X_MODULE)
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
static struct omap34xxcam_hw_config decoder_hwc = {
	.dev_index = 0,
	.dev_minor = 0,
	.dev_type = OMAP34XXCAM_SLAVE_SENSOR,
	.u.sensor.xclk = OMAP34XXCAM_XCLK_NONE,
	.u.sensor.sensor_isp = 1,
};

static struct isp_interface_config tvp5146_if_config = {
	.ccdc_par_ser = ISP_PARLL_YUV_BT,
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
#endif

static struct v4l2_ifparm ifparm = {
	.if_type = V4L2_IF_TYPE_BT656,
	.u = {
	      .bt656 = {
			.frame_start_on_rising_vs = 1,
			.bt_sync_correct = 0,
			.swap = 0,
			.latch_clk_inv = 0,
			.nobt_hs_inv = 0,	/* active high */
			.nobt_vs_inv = 0,	/* active high */
			.mode = V4L2_IF_TYPE_BT656_MODE_BT_8BIT,
			.clock_min = TVP514X_XCLK_BT656,
			.clock_max = TVP514X_XCLK_BT656,
			},
	      },
};

/**
 * @brief tvp5146_ifparm - Returns the TVP5146 decoder interface parameters
 *
 * @param p - pointer to v4l2_ifparm structure
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_ifparm(struct v4l2_ifparm *p)
{
	if (p == NULL)
		return -EINVAL;

	*p = ifparm;
	return 0;
}

/**
 * @brief tvp5146_set_prv_data - Returns tvp5146 omap34xx driver private data
 *
 * @param priv - pointer to omap34xxcam_hw_config structure
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_set_prv_data(void *priv)
{
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
	struct omap34xxcam_hw_config *hwc = priv;

	if (priv == NULL)
		return -EINVAL;

	hwc->u.sensor.sensor_isp = decoder_hwc.u.sensor.sensor_isp;
	hwc->u.sensor.xclk = decoder_hwc.u.sensor.xclk;
	hwc->dev_index = decoder_hwc.dev_index;
	hwc->dev_minor = decoder_hwc.dev_minor;
	hwc->dev_type = decoder_hwc.dev_type;
	return 0;
#else
	return -EINVAL;
#endif
}

/**
 * @brief omap3evmdc_set_mux - Sets mux to enable/disable signal routing to
 *                             different peripherals present in board
 * IMPORTANT - This function will take care of writing appropriate values for
 * active low signals as well
 *
 * @param mux_id - enum, mux id to enable/disable
 * @param value - enum, ENABLE_MUX for enabling and DISABLE_MUX for disabling
 *
 * @return result of operation - 0 is success
 */
static int omap3evmdc_set_mux(enum omap3evmdc_mux mux_id, enum config_mux value)
{
	int err = 0;

	if (unlikely(mux_id >= NUM_MUX)) {
		dprintk("Invalid mux id\n");
		return -EPERM;
	}


	switch (mux_id) {
	case MUX_TVP5146:
		/* active low signal. set 0 to enable, 1 to disable */
		if (ENABLE_MUX == value) {
			/* pull down the GPIO GPIO134 = 0 */
			gpio_set_value(GPIO134_SEL_Y, 0);
			/* pull up the GPIO GPIO54 = 1 */
			gpio_set_value(GPIO54_SEL_EXP_CAM, 1);
			/* pull up the GPIO GPIO136 = 1 */
			gpio_set_value(GPIO136_SEL_CAM, 1);
		} else
			/* pull up the GPIO GPIO134 = 0 */
			gpio_set_value(GPIO134_SEL_Y, 1);

		break;

	case MUX_CAMERA_SENSOR:
		/* active low signal. set 0 to enable, 1 to disable */
		if (ENABLE_MUX == value) {
			/* pull up the GPIO GPIO134 = 0 */
			gpio_set_value(GPIO134_SEL_Y, 1);
			/* pull up the GPIO GPIO54 = 1 */
			gpio_set_value(GPIO54_SEL_EXP_CAM, 1);
			/* pull down the GPIO GPIO136 = 1 */
			gpio_set_value(GPIO136_SEL_CAM, 0);
		} else
			/* pull up the GPIO GPIO136 = 1 */
			gpio_set_value(GPIO136_SEL_CAM, 1);

		break;

	case MUX_EXP_CAMERA_SENSOR:
		/* active low signal. set 0 to enable, 1 to disable */
		if (ENABLE_MUX == value) {
			/* pull up the GPIO GPIO134 = 1 */
			gpio_set_value(GPIO134_SEL_Y, 1);
			/* pull down the GPIO GPIO54 = 1 */
			gpio_set_value(GPIO54_SEL_EXP_CAM, 0);
			/* pull up the GPIO GPIO136 = 1 */
			gpio_set_value(GPIO136_SEL_CAM, 1);
		} else
			/* pull up the GPIO GPIO54 = 1 */
			gpio_set_value(GPIO54_SEL_EXP_CAM, 1);

		break;

	case NUM_MUX:
	default:
		dprintk("Invalid mux id\n");
		err = -EPERM;
	}

	return err;
}
/**
 * @brief tvp5146_power_set - Power-on or power-off TVP5146 device
 *
 * @param power - enum, Power on/off, resume/standby
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_power_set(enum v4l2_power power)
{
	switch (power) {
	case V4L2_POWER_OFF:
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
		if (isp_free_interface(ISP_PARLL_YUV_BT))
			return -ENODEV;
#endif
		/* Disable mux for TVP5146 decoder data path */
		if (omap3evmdc_set_mux(MUX_TVP5146, DISABLE_MUX))
			return -ENODEV;
		break;

	case V4L2_POWER_STANDBY:
		break;

	case V4L2_POWER_ON:
		/* Enable mux for TVP5146 decoder data path */
		if (omap3evmdc_set_mux(MUX_TVP5146, ENABLE_MUX))
			return -ENODEV;

#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
		if (isp_request_interface(ISP_PARLL_YUV_BT))
			return -ENODEV;

		isp_configure_interface(&tvp5146_if_config);
#endif
		break;

	default:
		return -ENODEV;
		break;
	}
	return 0;
}

static struct tvp514x_platform_data tvp5146_pdata = {
	.master = "omap34xxcam",
	.power_set = tvp5146_power_set,
	.priv_data_set = tvp5146_set_prv_data,
	.ifparm = tvp5146_ifparm,

	/* Some interface dependent params */
	.clk_polarity = 0, /* data clocked out on falling edge */
	.hs_polarity = 1, /* 0 - Active low, 1- Active high */
	.vs_polarity = 1, /* 0 - Active low, 1- Active high */
};

static struct i2c_board_info __initdata tvp5146_i2c_board_info = {
	I2C_BOARD_INFO("tvp5146m2", TVP5146_I2C_ADDR),
	.platform_data = &tvp5146_pdata,
};

#endif				/* #ifdef CONFIG_VIDEO_TVP514X */

/**
 * @brief omap3evmdc_mdc_config - GPIO configuration for
 *                          GPIO 134, 54 and 136
 *
 * @return result of operation - 0 is success
 */
static int omap3evmdc_mdc_config(void)
{
	/* Setting the MUX configuration */
	omap_cfg_reg(GPIO134_VDIN_SEL_Y);
	omap_cfg_reg(GPIO54_VDIN_SEL_EXP_CAM);
	omap_cfg_reg(GPIO136_VDIN_SEL_CAM);

	if (gpio_request(GPIO134_SEL_Y, "GPIO134_SEL_Y") < 0) {
		dprintk("can't get GPIO 134\n");
		return -EINVAL;
	}

	if (gpio_request(GPIO54_SEL_EXP_CAM, "GPIO54_SEL_EXP_CAM") < 0) {
		dprintk("can't get GPIO 54\n");
		return -EINVAL;
	}

	if (gpio_request(GPIO136_SEL_CAM, "GPIO136_SEL_CAM") < 0) {
		dprintk("can't get GPIO 136\n");
		return -EINVAL;
	}

	/* Make GPIO as output */
	gpio_direction_output(GPIO134_SEL_Y, 0);
	/* Make GPIO as output */
	gpio_direction_output(GPIO54_SEL_EXP_CAM, 0);
	/* Make GPIO as output */
	gpio_direction_output(GPIO136_SEL_CAM, 0);

	return 0;
}

/**
 * @brief omap3evmdc_init - module init function. Should be called before any
 *                          client driver init call
 *
 * @return result of operation - 0 is success
 */
static int __init omap3evmdc_init(void)
{
	int err;

	/*
	 * I2C3 SCL pin mux settings - mux mode 0, pull-up enable, input enable
	 * Uses the MSB 16-bit of this register, retain the LSB 16-bit.
	 * This pin is shared with gpio_184 (mux mode 4)
	 */
	omap_writel(((omap_readl(REG_CONTROL_PADCONF_I2C2_SDA) &
		      ~PADCONF_I2C3_SCL_MASK) | PADCONF_I2C3_SCL_DEF),
		    REG_CONTROL_PADCONF_I2C2_SDA);

	/*
	 * I2C3 SDA pin mux settings - mux mode 0, pull-up enable, input enable
	 * Uses the LSB 16-bit of this register, retain the MSB 16-bit.
	 * This pin is shared with gpio_185 (mux mode 4)
	 */
	omap_writel(((omap_readl(REG_CONTROL_PADCONF_I2C3_SDA) &
		      ~PADCONF_I2C3_SDA_MASK) | PADCONF_I2C3_SDA_DEF),
		    REG_CONTROL_PADCONF_I2C3_SDA);

	err = omap3evmdc_mdc_config();
	if (err) {
		dprintk("MDC configuration failed \n");
		return err;
	}

	/*
	 * Register each of the I2C devices present in the board to the I2C
	 * framework.
	 * If more I2C devices are added, then each device information should
	 * be registered with I2C using i2c_register_board_info().
	 */
#if defined(CONFIG_VIDEO_TVP514X) || defined(CONFIG_VIDEO_TVP514X_MODULE)
	err = i2c_register_board_info(BOARD_I2C_BUSNUM,
					&tvp5146_i2c_board_info, 1);
	if (err) {
		dprintk("TVP5146 I2C Board Registration failed \n");
		return err;
	}
#endif

	printk(KERN_INFO MODULE_NAME ": Driver registration complete \n");

	return 0;
}

arch_initcall(omap3evmdc_init);
