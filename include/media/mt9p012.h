/*
 * drivers/media/video/mt9p012.h
 *
 * Register definitions for the MT9P012 camera sensor.
 *
 * Author:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Martinez Leonides
 *
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef MT9P012_H
#define MT9P012_H


#define MT9P012_I2C_ADDR		0x10

/* The ID values we are looking for */
#define MT9P012_MOD_ID			0x2800
#define MT9P012_MFR_ID			0x0006

/* MT9P012 has 8/16/32 registers */
#define MT9P012_8BIT			1
#define MT9P012_16BIT			2
#define MT9P012_32BIT			4

/* terminating token for reg list */
#define MT9P012_TOK_TERM 		0xFF

/* delay token for reg list */
#define MT9P012_TOK_DELAY		100

/* Sensor specific GPIO signals */
#define MT9P012_RESET_GPIO  	98
#define MT9P012_STANDBY_GPIO	58

#define VAUX_2_8_V		0x09
#define VAUX_DEV_GRP_P1		0x20
#define VAUX_DEV_GRP_NONE	0x00

#define DEBUG_BASE		0x08000000
#define REG_SDP3430_FPGA_GPIO_2 (0x50)
#define FPGA_SPR_GPIO1_3v3	(0x1 << 14)
#define FPGA_GPIO6_DIR_CTRL	(0x1 << 6)

/* terminating list entry for reg */
#define MT9P012_REG_TERM		0xFF
/* terminating list entry for val */
#define MT9P012_VAL_TERM		0xFF

#define MT9P012_CLKRC			0x11

#define MT9P012_GAIN			0x00

/*
 * The nominal xclk input frequency of the MT9P012 is 12MHz, maximum
 * frequency is 64MHz, and minimum frequency is 2MHz.
 */
#define MT9P012_XCLK_MIN   2000000
#define MT9P012_XCLK_MAX   64000000
#define MT9P012_XCLK_NOM_1 12000000
#define MT9P012_XCLK_NOM_2 24000000

#define MT9P012_USE_XCLKA  	0
#define MT9P012_USE_XCLKB  	1


/* FPS Capabilities */
#define MT9P012_MIN_FPS		11
#define MT9P012_DEF_FPS		15
#define MT9P012_MAX_FPS		30

#define MT9P012_I2C_DELAY   	3
#define I2C_RETRY_COUNT		5

/* Still capture 5 MP */
#define IMAGE_WIDTH_MAX		2592
#define IMAGE_HEIGHT_MAX	1944
/* Still capture 3 MP and down to VGA, using ISP resizer */
#define IMAGE_WIDTH_MIN		2048
#define IMAGE_HEIGHT_MIN	1536


/* Video mode, for D1 NTSC, D1 PAL */
#define VIDEO_WIDTH_2X_BINN	1296
#define VIDEO_HEIGHT_2X_BINN	972

/* Sensor Video mode size for VGA, CIF, QVGA in 4x binning mode */
#define VIDEO_WIDTH_4X_BINN	648
#define VIDEO_HEIGHT_4X_BINN	486
/* To improve image quality in VGA */
#define CIF_PIXELS		(352 * 288)
#define QQVGA_PIXELS		(160 * 120)

/* Video mode, for QCIF, SQCIF */
#define VIDEO_WIDTH_4X_BINN_SCALED      216
#define VIDEO_HEIGHT_4X_BINN_SCALED     162

/* Default coarse integration times to get a good exposure */
#define COARSE_INT_TIME_216	         550
#define COARSE_INT_TIME_648	         550
#define COARSE_INT_TIME_216_30FPS	1350
#define COARSE_INT_TIME_648_30FPS	1350
#define COARSE_INT_TIME_1296		1000
#define COARSE_INT_TIME_3MP		1700
#define COARSE_INT_TIME_5MP		1700
#define COARSE_INT_TIME_INDEX	    	1
#define TST_PAT 			0x0

/* Analog gain values */
#define MIN_GAIN	0x08
#define MAX_GAIN	0x7F
#define DEF_GAIN	0x43
#define GAIN_STEP   	0x1

#define GAIN_INDEX	1

/* Exposure time values */
#define DEF_MIN_EXPOSURE	0x08
#define DEF_MAX_EXPOSURE	0x7F
#define DEF_EXPOSURE	    	0x43
#define EXPOSURE_STEP       	1

#define SENSOR_DETECTED		1
#define SENSOR_NOT_DETECTED	0

/**
 * struct mt9p012_reg - mt9p012 register format
 * @length: length of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for MT9P012 register initialization values
 */
struct mt9p012_reg {
	u16	length;
	u16 	reg;
	u32 	val;
};

enum image_size {
	BIN4XSCALE,
	BIN4X,
	BIN2X,
	THREE_MP,
	FIVE_MP
};

enum pixel_format {
	RAWBAYER10
};

#define NUM_IMAGE_SIZES		5
#define NUM_PIXEL_FORMATS	1
#define NUM_FPS			2	/* 2 ranges */
#define FPS_LOW_RANGE		0
#define FPS_HIGH_RANGE		1

/**
 * struct capture_size - image capture size information
 * @width: image width in pixels
 * @height: image height in pixels
 */
struct capture_size {
	unsigned long width;
	unsigned long height;
};

/**
 * struct mt9p012_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @default_regs: Default registers written after power-on or reset.
 * @ifparm: Interface parameters access function
 * @priv_data_set: device private data (pointer) access function
 */
struct mt9p012_platform_data {
	int (*power_set)(enum v4l2_power power);
	u32 (*set_xclk)(u32 xclkfreq);
	int (*priv_data_set)(void *);
};

/**
 * struct mt9p012_pll_settings - struct for storage of sensor pll values
 * @vt_pix_clk_div: vertical pixel clock divider
 * @vt_sys_clk_div: veritcal system clock divider
 * @pre_pll_div: pre pll divider
 * @fine_int_tm: fine resolution interval time
 * @frame_lines: number of lines in frame
 * @line_len: number of pixels in line
 * @min_pll: minimum pll multiplier
 * @max_pll: maximum pll multiplier
 */
struct mt9p012_pll_settings {
	u16	vt_pix_clk_div;
	u16	vt_sys_clk_div;
	u16	pre_pll_div;

	u16	fine_int_tm;
	u16	frame_lines;
	u16	line_len;

	u16	min_pll;
	u16	max_pll;
};

/*
 * Array of image sizes supported by MT9P012.  These must be ordered from
 * smallest image size to largest.
 */
const static struct capture_size mt9p012_sizes[] = {
	{  216, 162 },	/* 4X BINNING+SCALING */
	{  648, 486 },	/* 4X BINNING */
	{ 1296, 972 },	/* 2X BINNING */
	{ 2048, 1536},	/* 3 MP */
	{ 2592, 1944},	/* 5 MP */
};

/* PLL settings for MT9P012 */
enum mt9p012_pll_type {
  PLL_5MP = 0,
  PLL_3MP,
  PLL_1296_15FPS,
  PLL_1296_30FPS,
  PLL_648_15FPS,
  PLL_648_30FPS,
  PLL_216_15FPS,
  PLL_216_30FPS
};

#endif /* ifndef MT9P012_H */
