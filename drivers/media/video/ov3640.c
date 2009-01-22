/*
 * drivers/media/video/ov3640.c
 *
 * ov3640 sensor driver
 *
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Leverage ov3640.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>
#include "ov3640.h"
#include "omap34xxcam.h"
#include "isp/isp.h"
#include "isp/ispcsi2.h"

#define OV3640_DRIVER_NAME  "ov3640"
#define MOD_NAME "OV3640: "

#define I2C_M_WR 0

/**
 * struct ov3640_sensor - main structure for storage of sensor information
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @i2c_client: iic client device structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @isize: base image size
 * @ver: ov3640 chip version
 * @width: configured width
 * @height: configuredheight
 * @vsize: vertical size for the image
 * @hsize: horizontal size for the image
 * @crop_rect: crop rectangle specifying the left,top and width and height
 */
struct ov3640_sensor {
	const struct ov3640_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	int isize;
	int ver;
	int fps;
	unsigned long width;
	unsigned long height;
	unsigned long vsize;
	unsigned long hsize;
	struct v4l2_rect crop_rect;
	int state;
};

static struct ov3640_sensor ov3640;
static struct i2c_driver ov3640sensor_i2c_driver;
static unsigned long xclk_current = OV3640_XCLK_MIN;

/* List of image formats supported by OV3640 sensor */
const static struct v4l2_fmtdesc ov3640_formats[] = {
#if OV3640_RAW_MODE
	{
		.description	= "RAW10",
		.pixelformat	= V4L2_PIX_FMT_SGRBG10,
	},
#else
	{
		/* Note:  V4L2 defines RGB565 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 r4 r3 r2 r1 r0	  b4 b3 b2 b1 b0 g5 g4 g3
		 *
		 * We interpret RGB565 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 b4 b3 b2 b1 b0	  r4 r3 r2 r1 r0 g5 g4 g3
		 */
		.description	= "RGB565, le",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
	},
	{
		/* Note:  V4L2 defines RGB565X as:
		 *
		 *	Byte 0			  Byte 1
		 *	b4 b3 b2 b1 b0 g5 g4 g3	  g2 g1 g0 r4 r3 r2 r1 r0
		 *
		 * We interpret RGB565X as:
		 *
		 *	Byte 0			  Byte 1
		 *	r4 r3 r2 r1 r0 g5 g4 g3	  g2 g1 g0 b4 b3 b2 b1 b0
		 */
		.description	= "RGB565, be",
		.pixelformat	= V4L2_PIX_FMT_RGB565X,
	},
	{
		.description	= "YUYV (YUV 4:2:2), packed",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
	},
	{
		.description	= "UYVY, packed",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
	},
	{
		/* Note:  V4L2 defines RGB555 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 r4 r3 r2 r1 r0	  x  b4 b3 b2 b1 b0 g4 g3
		 *
		 * We interpret RGB555 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 b4 b3 b2 b1 b0	  x  r4 r3 r2 r1 r0 g4 g3
		 */
		.description	= "RGB555, le",
		.pixelformat	= V4L2_PIX_FMT_RGB555,
	},
	{
		/* Note:  V4L2 defines RGB555X as:
		 *
		 *	Byte 0			  Byte 1
		 *	x  b4 b3 b2 b1 b0 g4 g3	  g2 g1 g0 r4 r3 r2 r1 r0
		 *
		 * We interpret RGB555X as:
		 *
		 *	Byte 0			  Byte 1
		 *	x  r4 r3 r2 r1 r0 g4 g3	  g2 g1 g0 b4 b3 b2 b1 b0
		 */
		.description	= "RGB555, be",
		.pixelformat	= V4L2_PIX_FMT_RGB555X,
	},
#endif
};

#define NUM_CAPTURE_FORMATS (sizeof(ov3640_formats) / sizeof(ov3640_formats[0]))

/* register initialization tables for ov3640 */
#define OV3640_REG_TERM 0xFFFF	/* terminating list entry for reg */
#define OV3640_VAL_TERM 0xFF	/* terminating list entry for val */

const static struct ov3640_reg ov3640_out_xga[] = {
	{OV3640_ISP_XOUT_H, 0x04},  /* ISP_XOUT */
	{OV3640_ISP_XOUT_L, 0x00},  /* ISP_XOUT */
	{OV3640_ISP_YOUT_H, 0x03},  /* ISP_YOUT */
	{OV3640_ISP_YOUT_L, 0x00},  /* ISP_YOUT */
	{OV3640_REG_TERM, OV3640_VAL_TERM}
};

const static struct ov3640_reg ov3640_out_qxga[] = {
	{OV3640_ISP_XOUT_H, 0x08},  /* ISP_XOUT */
	{OV3640_ISP_XOUT_L, 0x00},  /* ISP_XOUT */
	{OV3640_ISP_YOUT_H, 0x06},  /* ISP_YOUT */
	{OV3640_ISP_YOUT_L, 0x00},  /* ISP_YOUT */
	{OV3640_REG_TERM, OV3640_VAL_TERM}
};

/* Brightness Settings - 7 levels */
const static struct ov3640_reg brightness[7][5] = {
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_SGNSET, 0x09},
		{OV3640_YBRIGHT, 0x30},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_SGNSET, 0x09},
		{OV3640_YBRIGHT, 0x20},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_SGNSET, 0x09},
		{OV3640_YBRIGHT, 0x10},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_SGNSET, 0x01},
		{OV3640_YBRIGHT, 0x00},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_SGNSET, 0x01},
		{OV3640_YBRIGHT, 0x10},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_SGNSET, 0x01},
		{OV3640_YBRIGHT, 0x20},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_SGNSET, 0x01},
		{OV3640_YBRIGHT, 0x30},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
};

/* Contrast Settings - 7 levels */
const static struct ov3640_reg contrast[7][5] = {
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_YOFFSET, 0x14},
		{OV3640_YGAIN, 0x14},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_YOFFSET, 0x18},
		{OV3640_YGAIN, 0x18},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_YOFFSET, 0x1c},
		{OV3640_YGAIN, 0x1c},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_YOFFSET, 0x20},
		{OV3640_YGAIN, 0x20},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_YOFFSET, 0x24},
		{OV3640_YGAIN, 0x24},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_YOFFSET, 0x28},
		{OV3640_YGAIN, 0x28},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x04},
		{OV3640_YOFFSET, 0x2c},
		{OV3640_YGAIN, 0x2c},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
};

/* Color Settings - 3 colors */
const static struct ov3640_reg colors[3][5] = {
	{
		{OV3640_SDE_CTRL, 0x00},
		{OV3640_UREG, 0x80},
		{OV3640_VREG, 0x80},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x18},
		{OV3640_UREG, 0x40},
		{OV3640_VREG, 0xa6},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	{
		{OV3640_SDE_CTRL, 0x18},
		{OV3640_UREG, 0x80},
		{OV3640_VREG, 0x80},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
};

/* Average Based Algorithm - Based on target Luminance */
const static struct ov3640_reg exposure_avg[11][5] = {
	/* -1.7EV */
	{
		{OV3640_HISTO7, 0x00},
		{OV3640_WPT_HISH, 0x10},
		{OV3640_BPT_HISL, 0x08},
		{OV3640_VPT, 0x21},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* -1.3EV */
	{
		{OV3640_HISTO7, 0x00},
		{OV3640_WPT_HISH, 0x18},
		{OV3640_BPT_HISL, 0x10},
		{OV3640_VPT, 0x31},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* -1.0EV */
	{
		{OV3640_HISTO7, 0x00},
		{OV3640_WPT_HISH, 0x20},
		{OV3640_BPT_HISL, 0x18},
		{OV3640_VPT, 0x41},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* -0.7EV */
	{
		{OV3640_HISTO7, 0x00},
		{OV3640_WPT_HISH, 0x28},
		{OV3640_BPT_HISL, 0x20},
		{OV3640_VPT, 0x51},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* -0.3EV */
	{
		{OV3640_HISTO7, 0x00},
		{OV3640_WPT_HISH, 0x30},
		{OV3640_BPT_HISL, 0x28},
		{OV3640_VPT, 0x61},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* default */
	{
		{OV3640_HISTO7, 0x00},
		{OV3640_WPT_HISH, 0x38},
		{OV3640_BPT_HISL, 0x30},
		{OV3640_VPT, 0x61},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* 0.3EV */
	{
		{OV3640_HISTO7, 0x00},
		{OV3640_WPT_HISH, 0x40},
		{OV3640_BPT_HISL, 0x38},
		{OV3640_VPT, 0x71},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* 0.7EV */
	{
		{OV3640_HISTO7, 0x00},
		{OV3640_WPT_HISH, 0x48},
		{OV3640_BPT_HISL, 0x40},
		{OV3640_VPT, 0x81},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* 1.0EV */
	{
		{OV3640_HISTO7, 0x00},
		{OV3640_WPT_HISH, 0x50},
		{OV3640_BPT_HISL, 0x48},
		{OV3640_VPT, 0x91},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* 1.3EV */
	{
		{OV3640_HISTO7, 0x00},
		{OV3640_WPT_HISH, 0x58},
		{OV3640_BPT_HISL, 0x50},
		{OV3640_VPT, 0x91},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* 1.7EV */
	{
		{OV3640_HISTO7, 0x00},
		{OV3640_WPT_HISH, 0x60},
		{OV3640_BPT_HISL, 0x58},
		{OV3640_VPT, 0xa1},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
};

/* Histogram Based Algorithm - Based on histogram and probability */
const static struct ov3640_reg exposure_hist[11][5] = {
	/* -1.7EV */
	{
		{OV3640_HISTO7, 0x80},
		{OV3640_WPT_HISH, 0x58},
		{OV3640_BPT_HISL, 0x38},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* -1.3EV */
	{
		{OV3640_HISTO7, 0x80},
		{OV3640_WPT_HISH, 0x60},
		{OV3640_BPT_HISL, 0x40},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* -1.0EV */
	{
		{OV3640_HISTO7, 0x80},
		{OV3640_WPT_HISH, 0x68},
		{OV3640_BPT_HISL, 0x48},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* -0.7EV */
	{
		{OV3640_HISTO7, 0x80},
		{OV3640_WPT_HISH, 0x70},
		{OV3640_BPT_HISL, 0x50},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* -0.3EV */
	{
		{OV3640_HISTO7, 0x80},
		{OV3640_WPT_HISH, 0x78},
		{OV3640_BPT_HISL, 0x58},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* default */
	{
		{OV3640_HISTO7, 0x80},
		{OV3640_WPT_HISH, 0x80},
		{OV3640_BPT_HISL, 0x60},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* 0.3EV */
	{
		{OV3640_HISTO7, 0x80},
		{OV3640_WPT_HISH, 0x88},
		{OV3640_BPT_HISL, 0x68},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* 0.7EV */
	{
		{OV3640_HISTO7, 0x80},
		{OV3640_WPT_HISH, 0x90},
		{OV3640_BPT_HISL, 0x70},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* 1.0EV */
	{
		{OV3640_HISTO7, 0x80},
		{OV3640_WPT_HISH, 0x98},
		{OV3640_BPT_HISL, 0x78},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* 1.3EV */
	{
		{OV3640_HISTO7, 0x80},
		{OV3640_WPT_HISH, 0xa0},
		{OV3640_BPT_HISL, 0x80},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
	/* 1.7EV */
	{
		{OV3640_HISTO7, 0x80},
		{OV3640_WPT_HISH, 0xa8},
		{OV3640_BPT_HISL, 0x88},
		{OV3640_REG_TERM, OV3640_VAL_TERM}
	},
};

/* ov3640 register configuration for combinations of pixel format and
 * image size
 */

const static struct ov3640_reg qxga_yuv[] = {
	{OV3640_SC_CTRL0, 0x02},
	{OV3640_DSP_CTRL_4, 0xFC},
	{OV3640_FMT_MUX_CTRL0, 0x00},
	{OV3640_FMT_CTRL00, 0x00},
	{OV3640_OUT_CTRL01, OV3640_OUT_CTRL01_MIPIBIT8},
	{OV3640_VTS_H, 0x06},
	{OV3640_VTS_L, 0x20},
	{OV3640_REG_TERM, OV3640_VAL_TERM}
};

const static struct ov3640_reg qxga_565[] = {
	{OV3640_SC_CTRL0, 0x02},
	{OV3640_DSP_CTRL_4, 0xFC},
	{OV3640_FMT_MUX_CTRL0, 0x01},
	{OV3640_FMT_CTRL00, 0x11},
	{OV3640_OUT_CTRL01, OV3640_OUT_CTRL01_MIPIBIT8},
	{OV3640_VTS_H, 0x06},
	{OV3640_VTS_L, 0x20},
	{OV3640_REG_TERM, OV3640_VAL_TERM}
};

const static struct ov3640_reg qxga_555[] = {
	{OV3640_SC_CTRL0, 0x02},
	{OV3640_DSP_CTRL_4, 0xFC},
	{OV3640_FMT_MUX_CTRL0, 0x01},
	{OV3640_FMT_CTRL00, 0x13},
	{OV3640_OUT_CTRL01, OV3640_OUT_CTRL01_MIPIBIT8},
	{OV3640_VTS_H, 0x06},
	{OV3640_VTS_L, 0x20},
	{OV3640_REG_TERM, OV3640_VAL_TERM}
};

const static struct ov3640_reg qxga_raw10[] = {
	{OV3640_SC_CTRL0, 0x22},
	{OV3640_DSP_CTRL_4, 0x01},
	{OV3640_FMT_MUX_CTRL0, 0x04},
	{OV3640_FMT_CTRL00, 0x18},
	{OV3640_OUT_CTRL01, 0x00},
	{OV3640_VTS_H, 0x06},
	{OV3640_VTS_L, 0x20},
	{OV3640_REG_TERM, OV3640_VAL_TERM}
};

const static struct ov3640_reg xga_yuv[] = {
	{OV3640_SC_CTRL0, 0x02},
	{OV3640_DSP_CTRL_4, 0xFC},
	{OV3640_FMT_MUX_CTRL0, 0x00},
	{OV3640_FMT_CTRL00, 0x00},
	{OV3640_OUT_CTRL01, OV3640_OUT_CTRL01_MIPIBIT8},
	{OV3640_VTS_H, 0x03},
	{OV3640_VTS_L, 0x10},
	{OV3640_REG_TERM, OV3640_VAL_TERM}
};

const static struct ov3640_reg xga_565[] = {
	{OV3640_SC_CTRL0, 0x02},
	{OV3640_DSP_CTRL_4, 0xFC},
	{OV3640_FMT_MUX_CTRL0, 0x01},
	{OV3640_FMT_CTRL00, 0x11},
	{OV3640_OUT_CTRL01, OV3640_OUT_CTRL01_MIPIBIT8},
	{OV3640_VTS_H, 0x03},
	{OV3640_VTS_L, 0x10},
	{OV3640_REG_TERM, OV3640_VAL_TERM}
};

const static struct ov3640_reg xga_555[] = {
	{OV3640_SC_CTRL0, 0x02},
	{OV3640_DSP_CTRL_4, 0xFC},
	{OV3640_FMT_MUX_CTRL0, 0x01},
	{OV3640_FMT_CTRL00, 0x13},
	{OV3640_OUT_CTRL01, OV3640_OUT_CTRL01_MIPIBIT8},
	{OV3640_VTS_H, 0x03},
	{OV3640_VTS_L, 0x10},
	{OV3640_REG_TERM, OV3640_VAL_TERM}
};

const static struct ov3640_reg xga_raw10[] = {
	{OV3640_SC_CTRL0, 0x22},
	{OV3640_DSP_CTRL_4, 0x01},
	{OV3640_FMT_MUX_CTRL0, 0x04},
	{OV3640_FMT_CTRL00, 0x18},
	{OV3640_OUT_CTRL01, 0x00},
	{OV3640_VTS_H, 0x03},
	{OV3640_VTS_L, 0x10},
	{OV3640_REG_TERM, OV3640_VAL_TERM}
};

const static struct ov3640_reg
	*ov3640_reg_init[OV_NUM_PIXEL_FORMATS][OV_NUM_IMAGE_SIZES] = {
	{xga_yuv, qxga_yuv},
	{xga_565, qxga_565},
	{xga_555, qxga_555},
	{xga_raw10, qxga_raw10}
};

/*
 * struct vcontrol - Video controls
 * @v4l2_queryctrl: V4L2 VIDIOC_QUERYCTRL ioctl structure
 * @current_value: current value of this control
 */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {
#if (OV3640_RAW_MODE == 0)
	{
		{
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Brightness",
		.minimum = OV3640_MIN_BRIGHT,
		.maximum = OV3640_MAX_BRIGHT,
		.step = OV3640_BRIGHT_STEP,
		.default_value = OV3640_DEF_BRIGHT,
		},
	.current_value = OV3640_DEF_BRIGHT,
	},
	{
		{
		.id = V4L2_CID_CONTRAST,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Contrast",
		.minimum = OV3640_MIN_CONTRAST,
		.maximum = OV3640_MAX_CONTRAST,
		.step = OV3640_CONTRAST_STEP,
		.default_value = OV3640_DEF_CONTRAST,
		},
	.current_value = OV3640_DEF_CONTRAST,
	},
	{
		{
		.id = V4L2_CID_PRIVATE_BASE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Color Effects",
		.minimum = OV3640_MIN_COLOR,
		.maximum = OV3640_MAX_COLOR,
		.step = OV3640_COLOR_STEP,
		.default_value = OV3640_DEF_COLOR,
		},
	.current_value = OV3640_DEF_COLOR,
	}
#endif
};

/*
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array.
 *
 * Returns the index of the requested ID from the control structure array
 */
static int find_vctrl(int id)
{
	int i = 0;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/*
 * Read a value from a register in ov3640 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov3640_read_reg(struct i2c_client *client, u16 data_length, u16 reg,
								u32 *val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;

	/* High byte goes out first */
	data[0] = (u8) (reg >> 8);
	data[1] = (u8) (reg & 0xff);

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0) {
		mdelay(3);
		msg->flags = I2C_M_RD;
		msg->len = data_length;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = 0;
		/* High byte comes first */
		if (data_length == 1)
			*val = data[0];
		else if (data_length == 2)
			*val = data[1] + (data[0] << 8);
		else
			*val = data[3] + (data[2] << 8) +
				(data[1] << 16) + (data[0] << 24);
		return 0;
	}
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

/* Write a value to a register in ov3640 sensor device.
 * @client: i2c driver client structure.
 * @reg: Address of the register to read value from.
 * @val: Value to be written to a specific register.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov3640_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[3];
	int retries = 0;

	if (!client->adapter)
		return -ENODEV;
retry:
	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 3;
	msg->buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);
	data[1] = (u8) (reg & 0xff);
	data[2] = val;

	err = i2c_transfer(client->adapter, msg, 1);
	udelay(50);

	if (err >= 0)
		return 0;

	if (retries <= 5) {
		dev_dbg(&client->dev, "Retrying I2C... %d", retries);
		retries++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto retry;
	}

	return err;
}

/*
 * Initialize a list of ov3640 registers.
 * The list of registers is terminated by the pair of values
 * {OV3640_REG_TERM, OV3640_VAL_TERM}.
 * @client: i2c driver client structure.
 * @reglist[]: List of address of the registers to write data.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov3640_write_regs(struct i2c_client *client,
					const struct ov3640_reg reglist[])
{
	int err = 0;
	const struct ov3640_reg *next = reglist;

	while (!((next->reg == OV3640_REG_TERM)
		&& (next->val == OV3640_VAL_TERM))) {
		err = ov3640_write_reg(client, next->reg, next->val);
		udelay(100);
		if (err)
			return err;
		next++;
	}
	return 0;
}

/* Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum image_size_ov
ov3640_find_size(unsigned int width, unsigned int height)
{
	if ((width > ov3640_sizes[XGA].width) ||
		(height > ov3640_sizes[XGA].height))
		return QXGA;
	return XGA;
}

/*
 * Set CSI2 Virtual ID.
 */
static int ov3640_set_virtual_id(struct i2c_client *client, u32 id)
{
	return ov3640_write_reg(client, OV3640_MIPI_CTRL0C, (0x3 & id) << 6 |
									0x02);
}


/*
 * Calculates the MIPIClk.
 * 1) Calculate fclk
 *     fclk = (64 - OV3640_PLL_1[5:0]) * N * Bit8Div * MCLK / M
 *    where N = 1/1.5/2/3 for OV3640_PLL_2[7:6]=0~3
 *          M = 1/1.5/2/3 for OV3640_PLL_2[1:0]=0~3
 *    Bit8Div = 1/1/4/5 for OV3640_PLL_2[5:4]
 * 2) Calculate MIPIClk
 *     MIPIClk = fclk / ScaleDiv / MIPIDiv
 *             = fclk * (1/ScaleDiv) / MIPIDiv
 *    where 1/ScaleDiv = 0x3010[3:0]*2
 *          MIPIDiv = 0x3010[5] + 1
 * NOTE:
 *  - The lookup table 'lut1' has been multiplied by 2 so all its values
 *    are integers. Since both N & M use the same table, and they are
 *    used as a ratio then the factor of 2 is already take into account.
 *    i.e.  2N/2M = N/M
 */
static u32 ov3640_calc_mipiclk(struct v4l2_int_device *s)
{
	struct ov3640_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	u32 rxpll, val, n, m, bit8div;
	u32 sdiv_inv, mipidiv;
	u32 fclk, mipiclk, mclk = 24000000;
	u8 lut1[4] = {2, 3, 4, 6};
	u8 lut2[4] = {1, 1, 4, 5};

	/* Calculate fclk */
	ov3640_read_reg(client, 1, OV3640_PLL_1, &val);
	rxpll = val & 0x3F;

	ov3640_read_reg(client, 1, OV3640_PLL_2, &val);
	n = lut1[(val >> 6) & 0x3];
	m = lut1[val & 0x3];
	bit8div = lut2[(val >> 4) & 0x3];
	fclk = (64 - rxpll) * n * bit8div * mclk / m;

	ov3640_read_reg(client, 1, OV3640_PLL_3, &val);
	mipidiv = ((val >> 5) & 1) + 1;
	sdiv_inv = (val & 0xF) * 2;

	if ((val & 0xF) >= 1)
		mipiclk = fclk / sdiv_inv / mipidiv;
	else
		mipiclk = fclk / mipidiv;
	dev_dbg(&client->dev, "mipiclk=%u  fclk=%u  val&0xF=%u  sdiv_inv=%u  "
							"mipidiv=%u\n",
							mipiclk, fclk, val&0xF,
							sdiv_inv, mipidiv);
	return mipiclk;
}

/**
 * ov3640_set_framerate
 **/
static int ov3640_set_framerate(struct i2c_client *client,
						struct v4l2_fract *fper,
						enum image_size_ov isize)
{
	u32 tempfps1, tempfps2;
	u8 clkval;
/*
	u32 origvts, newvts, templineperiod;
	u32 origvts_h, origvts_l, newvts_h, newvts_l;
*/
	int err = 0;

	/* FIXME: QXGA framerate setting forced to 15 FPS */
	if (isize == QXGA) {
		err = ov3640_write_reg(client, OV3640_PLL_1, 0x32);
		err = ov3640_write_reg(client, OV3640_PLL_2, 0x21);
		err = ov3640_write_reg(client, OV3640_PLL_3, 0x21);
		err = ov3640_write_reg(client, OV3640_CLK, 0x01);
		err = ov3640_write_reg(client, 0x304c, 0x81);
		return err;
	}

	tempfps1 = fper->denominator * 10000;
	tempfps1 /= fper->numerator;
	tempfps2 = fper->denominator / fper->numerator;
	if ((tempfps1 % 10000) != 0)
		tempfps2++;
	clkval = (u8)((30 / tempfps2) - 1);

	err = ov3640_write_reg(client, OV3640_CLK, clkval);
	/* RxPLL = 50d = 32h */
	err = ov3640_write_reg(client, OV3640_PLL_1, 0x32);
	/* RxPLL = 50d = 32h */
	err = ov3640_write_reg(client, OV3640_PLL_2,
					OV3640_PLL_2_BIT8DIV_4 |
					OV3640_PLL_2_INDIV_1_5);
	/*
	 * NOTE: Sergio's Fix for MIPI CLK timings, not suggested by OV
	 */
	err = ov3640_write_reg(client, OV3640_PLL_3, 0x21 +
							(clkval & 0xF));
	/* Setting DVP divisor value */
	err = ov3640_write_reg(client, 0x304c, 0x82);
/* FIXME: Time adjustment to add granularity to the available fps */
/*
	ov3640_read_reg(client, 1, OV3640_VTS_H, &origvts_h);
	ov3640_read_reg(client, 1, OV3640_VTS_L, &origvts_l);
	origvts = (u32)((origvts_h << 8) + origvts_l);
	templineperiod = 1000000 / (tempfps2 * origvts);
	newvts = 1000000 / (tempfps2 * templineperiod);
	newvts_h = (u8)((newvts & 0xFF00) >> 8);
	newvts_l = (u8)(newvts & 0xFF);
	err = ov3640_write_reg(client, OV3640_VTS_H, newvts_h);
	err = ov3640_write_reg(client, OV3640_VTS_L, newvts_l);
*/
	return err;
}

/*
 * Configure the ov3640 for a specified image size, pixel format, and frame
 * period.  xclk is the frequency (in Hz) of the xclk input to the OV3640.
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int ov3640_configure(struct v4l2_int_device *s)
{
	struct ov3640_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &sensor->pix;
	struct i2c_client *client = sensor->i2c_client;
	enum image_size_ov isize = XGA;
	unsigned char hsize_l = 0, hsize_h = 0;
	unsigned char vsize_l = 0, vsize_h = 0;
	int vsize = 0, hsize = 0, height_l = 0, height_h = 0, width_l = 0;
	int width_h = 0, ratio = 0, err = 0;
	u32 mipiclk;
	enum pixel_format_ov pfmt = YUV;
	u32 min_hs_zero_nui, min_hs_zero, min_hs_zero_total;
	u32 min_hs_prepare_nui, min_hs_prepare, min_hs_prepare_total;
	u32 max_hs_prepare_nui, max_hs_prepare, max_hs_prepare_total;
	u32 ubound_hs_settle, lbound_hs_settle;
	u32 val;

	switch (pix->pixelformat) {

	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
		pfmt = RGB565;
		break;

	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB555X:
		pfmt = RGB555;
		break;

	case V4L2_PIX_FMT_SGRBG10:
		pfmt = RAW10;
		break;

	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pfmt = YUV;
	}

	/* Set receivers virtual channel before sensor setup starts.
	 * Only set the sensors virtual channel after all other setup
	 * for the sensor is complete.
	 */
	isp_csi2_ctx_config_virtual_id(0, OV3640_CSI2_VIRTUAL_ID);
	isp_csi2_ctx_update(0, false);

	if (ov3640_find_size(pix->width, pix->height) == XGA)
		isize = XGA;
	else
		isize = QXGA;

	/* Reset */
	ov3640_write_reg(client, OV3640_SYS, 0x80);
	mdelay(5);

	/* Common registers */
	err = ov3640_write_regs(client, ov3640_common[isize]);

	/* Configure image size and pixel format */
	err = ov3640_write_regs(client, ov3640_reg_init[pfmt][isize]);

	/* Setting of frame rate (OV suggested way) */
	err = ov3640_set_framerate(client, &sensor->timeperframe, isize);
#ifdef CONFIG_VIDEO_OV3640_CSI2
	/* Set CSI2 common register settings */
	err = ov3640_write_regs(client, ov3640_common_csi2);
#endif

	sensor->isize = isize;

	/* Scale image if needed*/
	if (((pix->width == ov3640_sizes[QXGA].width) &&
		(pix->height == ov3640_sizes[QXGA].height) && (isize == QXGA))
		|| ((pix->width == ov3640_sizes[XGA].width) &&
		(pix->height == ov3640_sizes[XGA].height) &&
		(isize == XGA)) || (pfmt == RAW10)) {

		/* if the image size correspond to one of the base image sizes
			then we don't need to scale the image */
		sensor->hsize = pix->width;
		sensor->vsize = pix->height;

		if (isize == XGA)
			ov3640_write_regs(client, ov3640_out_xga);
		else
			ov3640_write_regs(client, ov3640_out_qxga);

	} else {
	/* Default Ver and Hor sizes for QXGA and XGA*/
		if (isize == QXGA) {
			vsize = 0x600;/* 0x60c; */
			hsize = 0x800;/* 0x818; */
		} else {
			vsize = 0x304;
			hsize = 0x40c;
		}
		/* Scaling */
		/* Adjust V and H sizes for image sizes not derived form VGA*/
		ratio = (pix->width * 1000) / pix->height;

		if  (((vsize * ratio + 500) / 1000) > hsize)
			vsize = (hsize * 1000) / ratio ;

		else
			hsize = (vsize * ratio + 500) / 1000;

		/* We need even numbers */
		if (vsize & 1)
			vsize--;
		if (hsize & 1)
			hsize--;

		/* Adjusting numbers to set registers correctly */
		hsize_l = (0xFF & hsize);
		hsize_h = (0xF00 & hsize) >> 8;
		vsize_l = (0xFF & vsize);
		vsize_h = (0x700 & vsize) >> 4;

		/* According to Software app notes we have to add 0x08 and 0x04
		 * in order to scale correctly
		 */
		width_l = (0xFF & pix->width) + 0x08;
		width_h = (0xF00 & pix->width) >> 8;
		height_l = (0xFF & pix->height) + 0x04;
		height_h = (0x700 & pix->height) >> 4;

		err = ov3640_write_reg(client, OV3640_SIZE_IN_MISC,
							(vsize_h | hsize_h));
		err = ov3640_write_reg(client, OV3640_HSIZE_IN_L, hsize_l);
		err = ov3640_write_reg(client, OV3640_VSIZE_IN_L, vsize_l);
		err = ov3640_write_reg(client, OV3640_SIZE_OUT_MISC,
							(height_h | width_h));
		err = ov3640_write_reg(client, OV3640_HSIZE_OUT_L, width_l);
		err = ov3640_write_reg(client, OV3640_VSIZE_OUT_L, height_l);
		err = ov3640_write_reg(client, OV3640_ISP_PAD_CTR2, 0x42);
		err = ov3640_write_reg(client, OV3640_ISP_XOUT_H, width_h);
		err = ov3640_write_reg(client, OV3640_ISP_XOUT_L,
							(width_l  - 0x08));
		err = ov3640_write_reg(client, OV3640_ISP_YOUT_H,
							(height_h >> 4));
		err = ov3640_write_reg(client, OV3640_ISP_YOUT_L,
							(height_l - 0x04));

		sensor->hsize = hsize;
		sensor->vsize = vsize;

		dev_dbg(&client->dev, "HSIZE_IN =%i  VSIZE_IN =%i\n", hsize,
									vsize);
		dev_dbg(&client->dev, "HSIZE_OUT=%u  VSIZE_OUT=%u\n",
							(pix->width + 8),
							(pix->height + 4));
		dev_dbg(&client->dev, "ISP_XOUT =%u  ISP_YOUT =%u\n",
								pix->width,
								pix->height);
	}

	/* Setup the ISP VP based on image format */
	if (pix->pixelformat == V4L2_PIX_FMT_SGRBG10) {
		isp_csi2_ctrl_config_vp_out_ctrl(2);
		isp_csi2_ctrl_update(false);
	} else {
		isp_csi2_ctrl_config_vp_out_ctrl(1);
		isp_csi2_ctrl_update(false);
	}

	/* Store image size */
	sensor->width = pix->width;
	sensor->height = pix->height;

	sensor->crop_rect.left = 0;
	sensor->crop_rect.width = pix->width;
	sensor->crop_rect.top = 0;
	sensor->crop_rect.height = pix->height;

#ifdef CONFIG_VIDEO_OV3640_CSI2
	mipiclk = ov3640_calc_mipiclk(s);

	/* Calculate Valid bounds for High speed settle timing in UIs */
	ov3640_read_reg(client, 1, OV3640_MIPI_CTRL22, &val);
	min_hs_zero_nui = ((val & OV3640_MIPI_CTRL22_MIN_HS_ZERO_NUI_MASK) >>
				OV3640_MIPI_CTRL22_MIN_HS_ZERO_NUI_SHIFT);
	min_hs_zero = ((val & OV3640_MIPI_CTRL22_MIN_HS_ZERO_H_MASK) << 8);
	ov3640_read_reg(client, 1, OV3640_MIPI_CTRL23, &val);
	min_hs_zero |= (val & OV3640_MIPI_CTRL23_MIN_HS_ZERO_L_MASK);
	min_hs_zero_total = ((min_hs_zero_nui * 1000000 * 1000) / mipiclk) +
								min_hs_zero;

	ov3640_read_reg(client, 1, OV3640_MIPI_CTRL32, &val);
	min_hs_prepare_nui = ((val &
				OV3640_MIPI_CTRL32_MIN_HS_PREPARE_NUI_MASK) >>
				OV3640_MIPI_CTRL32_MIN_HS_PREPARE_NUI_SHIFT);
	min_hs_prepare = ((val &
				OV3640_MIPI_CTRL32_MIN_HS_PREPARE_H_MASK) << 8);
	ov3640_read_reg(client, 1, OV3640_MIPI_CTRL33, &val);
	min_hs_prepare |= (val & OV3640_MIPI_CTRL33_MIN_HS_PREPARE_L_MASK);
	min_hs_prepare_total = ((min_hs_prepare_nui * 1000000 * 1000) /
						mipiclk) + min_hs_prepare;

	ov3640_read_reg(client, 1, OV3640_MIPI_CTRL34, &val);
	max_hs_prepare_nui = ((val &
				OV3640_MIPI_CTRL34_MAX_HS_PREPARE_NUI_MASK) >>
				OV3640_MIPI_CTRL34_MAX_HS_PREPARE_NUI_SHIFT);
	max_hs_prepare = ((val &
				OV3640_MIPI_CTRL34_MAX_HS_PREPARE_H_MASK) << 8);
	ov3640_read_reg(client, 1, OV3640_MIPI_CTRL35, &val);
	max_hs_prepare |= (val & OV3640_MIPI_CTRL35_MAX_HS_PREPARE_L_MASK);
	max_hs_prepare_total = ((max_hs_prepare_nui * 1000000 * 1000) /
						mipiclk) + max_hs_prepare;

	ubound_hs_settle = ((min_hs_zero_total + min_hs_prepare_total) *
					((mipiclk >> 1) / 1000000)) / 1000;
	lbound_hs_settle = (max_hs_prepare_total * ((mipiclk >> 1) /
							1000000)) / 1000;

	/* Send settings to ISP-CSI2 Receiver PHY */
	isp_csi2_calc_phy_cfg0(mipiclk, lbound_hs_settle, ubound_hs_settle);

	/* Set sensors virtual channel*/
	ov3640_set_virtual_id(client, OV3640_CSI2_VIRTUAL_ID);
#endif
	return err;
}


/* Detect if an ov3640 is present, returns a negative error number if no
 * device is detected, or pidl as version number if a device is detected.
 */
static int ov3640_detect(struct i2c_client *client)
{
	u32 pidh, pidl;

	if (!client)
		return -ENODEV;

	if (ov3640_read_reg(client, 1, OV3640_PIDH, &pidh))
		return -ENODEV;

	if (ov3640_read_reg(client, 1, OV3640_PIDL, &pidl))
		return -ENODEV;

	if ((pidh == OV3640_PIDH_MAGIC) && ((pidl == OV3640_PIDL_MAGIC1) ||
						(pidl == OV3640_PIDL_MAGIC2))) {
		dev_dbg(&client->dev, "Detect success (%02X,%02X)\n", pidh,
									pidl);
		return pidl;
	}

	return -ENODEV;
}

/* To get the cropping capabilities of ov3640 sensor
 * Returns zero if successful, or non-zero otherwise.
 */
static int ioctl_cropcap(struct v4l2_int_device *s,
						struct v4l2_cropcap *cropcap)
{
	struct ov3640_sensor *sensor = s->priv;

	cropcap->bounds.top = 0;
	cropcap->bounds.left = 0;
	cropcap->bounds.width = sensor->width;
	cropcap->bounds.height = sensor->height;
	cropcap->defrect = cropcap->bounds;
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;
	return 0;
}

/* To get the current crop window for of ov3640 sensor
 * Returns zero if successful, or non-zero otherwise.
 */
static int ioctl_g_crop(struct v4l2_int_device *s, struct  v4l2_crop *crop)
{
	struct ov3640_sensor *sensor = s->priv;

	crop->c = sensor->crop_rect;
	return 0;
}

/* To set the crop window for of ov3640 sensor
 * Returns zero if successful, or non-zero otherwise.
 */
static int ioctl_s_crop(struct v4l2_int_device *s, struct  v4l2_crop *crop)
{
	struct ov3640_sensor *sensor = s->priv;
	/* FIXME: Temporary workaround for avoiding Zoom setting */
	/* struct i2c_client *client = sensor->i2c_client; */
	struct v4l2_rect *cur_rect;
	unsigned long *cur_width, *cur_height;
	int hstart, vstart, hsize, vsize, hsize_l, vsize_l, hsize_h, vsize_h;
	int hratio, vratio, zoomfactor, err = 0;

	cur_rect = &sensor->crop_rect;
	cur_width = &sensor->width;
	cur_height = &sensor->height;

	if ((crop->c.left == cur_rect->left) &&
	    (crop->c.width == cur_rect->width) &&
	    (crop->c.top == cur_rect->top) &&
	    (crop->c.height == cur_rect->height))
		return 0;

	/* out of range? then return the current crop rectangle */
	if ((crop->c.left + crop->c.width) > sensor->width ||
	    (crop->c.top + crop->c.height) > sensor->height) {
		crop->c = *cur_rect;
		return 0;
	}

	if (sensor->isize == QXGA)
		zoomfactor = 1;
	else
		zoomfactor = 2;

	hratio = (sensor->hsize * 1000) / sensor->width;
	vratio = (sensor->vsize * 1000) / sensor->height;
	hstart = (((crop->c.left * hratio + 500) / 1000) * zoomfactor) + 0x11d;
	vstart = (((crop->c.top * vratio + 500) / 1000) + 0x0a);
	hsize  = (crop->c.width * hratio + 500) / 1000;
	vsize  = (crop->c.height * vratio + 500) / 1000;

	if (vsize & 1)
		vsize--;
	if (hsize & 1)
		hsize--;

	/* Adjusting numbers to set register correctly */
	hsize_l = (0xFF & hsize);
	hsize_h = (0xF00 & hsize) >> 8;
	vsize_l = (0xFF & vsize);
	vsize_h = (0x700 & vsize) >> 4;

	if ((sensor->height > vsize) || (sensor->width > hsize))
		return -EINVAL;

	hsize = hsize * zoomfactor;
/*
	err = ov3640_write_reg(client, OV3640_DSP_CTRL_2, 0xEF);
	err = ov3640_write_reg(client, OV3640_SIZE_IN_MISC, (vsize_h |
								hsize_h));
	err = ov3640_write_reg(client, OV3640_HSIZE_IN_L, hsize_l);
	err = ov3640_write_reg(client, OV3640_VSIZE_IN_L, vsize_l);
	err = ov3640_write_reg(client, OV3640_HS_H, (hstart >> 8) & 0xFF);
	err = ov3640_write_reg(client, OV3640_HS_L, hstart & 0xFF);
	err = ov3640_write_reg(client, OV3640_VS_H, (vstart >> 8) & 0xFF);
	err = ov3640_write_reg(client, OV3640_VS_L, vstart & 0xFF);
	err = ov3640_write_reg(client, OV3640_HW_H, ((hsize) >> 8) & 0xFF);
	err = ov3640_write_reg(client, OV3640_HW_L, hsize & 0xFF);
	err = ov3640_write_reg(client, OV3640_VH_H, ((vsize) >> 8) & 0xFF);
	err = ov3640_write_reg(client, OV3640_VH_L, vsize & 0xFF);
*/
	if (err)
		return err;

	/* save back */
	*cur_rect = crop->c;

	/* Setting crop too fast can cause frame out-of-sync. */

	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(20));
	return 0;
}


/*
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
						struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = video_control[i].qc;
	return 0;
}

/*
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */

static int ioctl_g_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	struct vcontrol *lvc;
	int i;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_PRIVATE_BASE:
		vc->value = lvc->current_value;
		break;
	}
	return 0;
}

/*
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	int retval = -EINVAL;
	int i;
	struct ov3640_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct vcontrol *lvc;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &video_control[i];

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		if (vc->value >= 0 && vc->value <= 6) {
			retval = ov3640_write_regs(client,
							brightness[vc->value]);
		} else {
			dev_err(&client->dev, "BRIGHTNESS LEVEL NOT SUPPORTED");
			return -EINVAL;
		}
		break;
	case V4L2_CID_CONTRAST:
		if (vc->value >= 0 && vc->value <= 6)
			retval = ov3640_write_regs(client, contrast[vc->value]);
		else {
			dev_err(&client->dev, "CONTRAST LEVEL NOT SUPPORTED");
			return -EINVAL;
		}
		break;
	case V4L2_CID_PRIVATE_BASE:
		if (vc->value >= 0 && vc->value <= 2)
			retval = ov3640_write_regs(client, colors[vc->value]);
		else {
			dev_err(&client->dev, "COLOR LEVEL NOT SUPPORTED");
			return -EINVAL;
		}
		break;
	}
	if (!retval)
		lvc->current_value = vc->value;
	return retval;
}

/*
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
 static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
				   struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= NUM_CAPTURE_FORMATS)
			return -EINVAL;
	break;
	default:
		return -EINVAL;
	}

	fmt->flags = ov3640_formats[index].flags;
	strlcpy(fmt->description, ov3640_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = ov3640_formats[index].pixelformat;

	return 0;
}


/*
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */

static int ioctl_try_fmt_cap(struct v4l2_int_device *s,
			     struct v4l2_format *f)
{
	int ifmt;
	enum image_size_ov isize;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (pix->width > ov3640_sizes[QXGA].width)
		pix->width = ov3640_sizes[QXGA].width;
	if (pix->height > ov3640_sizes[QXGA].height)
		pix->height = ov3640_sizes[QXGA].height;

	isize = ov3640_find_size(pix->width, pix->height);
	pix->width = ov3640_sizes[isize].width;
	pix->height = ov3640_sizes[isize].height;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == ov3640_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	pix->pixelformat = ov3640_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width*2;
	pix->sizeimage = pix->bytesperline*pix->height;
	pix->priv = 0;
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB555X:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	}
	return 0;
}


/*
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
 static int ioctl_s_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct ov3640_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;

	rval = ioctl_try_fmt_cap(s, f);
	if (rval)
		return rval;

	sensor->pix = *pix;

	return 0;
}

/*
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct ov3640_sensor *sensor = s->priv;
	f->fmt.pix = sensor->pix;

	return 0;
}

/*
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	struct ov3640_sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = sensor->timeperframe;

	return 0;
}

/*
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	int rval = 0;
	struct ov3640_sensor *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	struct v4l2_fract timeperframe_old;
	int desired_fps;
	timeperframe_old = sensor->timeperframe;
	sensor->timeperframe = *timeperframe;

	desired_fps = timeperframe->denominator / timeperframe->numerator;
	if ((desired_fps < OV3640_MIN_FPS) || (desired_fps > OV3640_MAX_FPS))
		rval = -EINVAL;

	if (rval)
		sensor->timeperframe = timeperframe_old;
	else
		*timeperframe = sensor->timeperframe;

	return rval;
}

/*
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct ov3640_sensor *sensor = s->priv;

	return sensor->pdata->priv_data_set(p);
}

/*
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
 static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct ov3640_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;
	struct omap34xxcam_hw_config hw_config;
	int rval;

	rval = ioctl_g_priv(s, &hw_config);
	if (rval) {
		dev_err(&c->dev, "Unable to get hw params\n");
		return rval;
	}

	rval = sensor->pdata->power_set(on);
	if (rval < 0) {
		dev_err(&c->dev, "Unable to set the power state: "
			OV3640_DRIVER_NAME " sensor\n");
		isp_set_xclk(0, OV3640_USE_XCLKB);
		return rval;
	}

	if (on == V4L2_POWER_ON)
		isp_set_xclk(xclk_current, OV3640_USE_XCLKB);
	else
		isp_set_xclk(0, OV3640_USE_XCLKB);

	if ((on == V4L2_POWER_ON) && (sensor->state == SENSOR_DETECTED))
		ov3640_configure(s);

	if ((on == V4L2_POWER_ON) && (sensor->state == SENSOR_NOT_DETECTED)) {
		rval = ov3640_detect(c);
		if (rval < 0) {
			dev_err(&c->dev, "Unable to detect "
					OV3640_DRIVER_NAME " sensor\n");
			sensor->state = SENSOR_NOT_DETECTED;
			return rval;
		}
		sensor->state = SENSOR_DETECTED;
		sensor->ver = rval;
		pr_info(OV3640_DRIVER_NAME " Chip version 0x%02x detected\n",
								sensor->ver);
	}
	return 0;
}

/*
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call ov3640_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the dev. at slave detach.  The complement of ioctl_dev_init.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.  Returns 0 if
 * ov3640 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 **/
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
					struct v4l2_frmsizeenum *frms)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frms->pixel_format == ov3640_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index >= 2)
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = ov3640_sizes[frms->index].width;
	frms->discrete.height = ov3640_sizes[frms->index].height;

	return 0;
}

const struct v4l2_fract ov3640_frameintervals[] = {
	{ .numerator = 2, .denominator = 15 },
	{ .numerator = 1, .denominator = 15 },
	{ .numerator = 1, .denominator = 30 },
};

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					struct v4l2_frmivalenum *frmi)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frmi->pixel_format == ov3640_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */

	if ((frmi->width == ov3640_sizes[1].width) &&
				(frmi->height == ov3640_sizes[1].height)) {
		/* FIXME: The only frameinterval supported by QXGA capture is
		 * 2/15 fps
		 */
		if (frmi->index != 0)
			return -EINVAL;
	} else {
		if (frmi->index >= 3)
			return -EINVAL;
	}

	frmi->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	frmi->discrete.numerator =
				ov3640_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
				ov3640_frameintervals[frmi->index].denominator;

	return 0;
}

static struct v4l2_int_ioctl_desc ov3640_ioctl_desc[] = {
	{vidioc_int_enum_framesizes_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
	{vidioc_int_dev_init_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_exit},
	{vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_priv_num,
	  (v4l2_int_ioctl_func *)ioctl_g_priv},
	{vidioc_int_init_num,
	  (v4l2_int_ioctl_func *)ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
	{vidioc_int_try_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_try_fmt_cap},
	{vidioc_int_g_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
	{vidioc_int_s_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_s_fmt_cap},
	{vidioc_int_g_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_s_parm},
	{vidioc_int_queryctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_queryctrl},
	{vidioc_int_g_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_s_ctrl},
	  { vidioc_int_g_crop_num,
	  (v4l2_int_ioctl_func *)ioctl_g_crop},
	{vidioc_int_s_crop_num,
	  (v4l2_int_ioctl_func *)ioctl_s_crop},
	  { vidioc_int_cropcap_num,
	  (v4l2_int_ioctl_func *)ioctl_cropcap},
};

static struct v4l2_int_slave ov3640_slave = {
	.ioctls		= ov3640_ioctl_desc,
	.num_ioctls	= ARRAY_SIZE(ov3640_ioctl_desc),
};

static struct v4l2_int_device ov3640_int_device = {
	.module	= THIS_MODULE,
	.name	= OV3640_DRIVER_NAME,
	.priv	= &ov3640,
	.type	= v4l2_int_type_slave,
	.u	= {
		.slave = &ov3640_slave,
	},
};

/*
 * ov3640_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int __init
ov3640_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ov3640_sensor *sensor = &ov3640;
	int err;

	if (i2c_get_clientdata(client))
		return -EBUSY;

	sensor->pdata = client->dev.platform_data;

	if (!sensor->pdata || !sensor->pdata->default_regs) {
		dev_err(&client->dev, "No platform data?\n");
		return -ENODEV;
	}

	sensor->v4l2_int_device = &ov3640_int_device;
	sensor->i2c_client = client;

	i2c_set_clientdata(client, sensor);

	/* Make the default capture format XGA RGB565 */
	sensor->pix.width = ov3640_sizes[XGA].width;
	sensor->pix.height = ov3640_sizes[XGA].height;
	sensor->pix.pixelformat = V4L2_PIX_FMT_RGB565;

	err = v4l2_int_device_register(sensor->v4l2_int_device);
	if (err)
		i2c_set_clientdata(client, NULL);

	return 0;
}

/*
 * ov3640_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device. Complement of ov3640_probe().
 */
static int __exit
ov3640_remove(struct i2c_client *client)
{
	struct ov3640_sensor *sensor = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id ov3640_id[] = {
	{ OV3640_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ov3640_id);

static struct i2c_driver ov3640sensor_i2c_driver = {
	.driver = {
		.name	= OV3640_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe	= ov3640_probe,
	.remove	= __exit_p(ov3640_remove),
	.id_table = ov3640_id,
};

static struct ov3640_sensor ov3640 = {
	.timeperframe = {
		.numerator = 1,
		.denominator = 15,
	},
	.state = SENSOR_NOT_DETECTED,
};

/*
 * ov3640sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init ov3640sensor_init(void)
{
	int err;

	err = i2c_add_driver(&ov3640sensor_i2c_driver);
	if (err) {
		printk(KERN_ERR "Failed to register" OV3640_DRIVER_NAME ".\n");
		return err;
	}
	return 0;
}
late_initcall(ov3640sensor_init);

/*
 * ov3640sensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of ov3640sensor_init.
 */
static void __exit ov3640sensor_cleanup(void)
{
	i2c_del_driver(&ov3640sensor_i2c_driver);
}
module_exit(ov3640sensor_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OV3640 camera sensor driver");


