/*
 * drivers/media/video/dw9710.h
 *
 * Register defines for Auto Focus device
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Contributors:
 * 	Troy Laramy <t-laramy@ti.com>
 * 	Mohit Jalori <mjalori@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#ifndef CAMAF_DW9710_H
#define CAMAF_DW9710_H

#define DW9710_AF_I2C_ADDR	0x0C
#define DW9710_NAME 		"DW9710"
#define DW9710_I2C_RETRY_COUNT	5
#define MAX_FOCUS_POS	0xFF

#define CAMAF_DW9710_DISABLE		0x1
#define CAMAF_DW9710_ENABLE		0x0
#define CAMAF_DW9710_POWERDN(ARG)	(((ARG) & 0x1) << 15)
#define CAMAF_DW9710_POWERDN_R(ARG)	(((ARG) >> 15) & 0x1)

#define CAMAF_DW9710_DATA(ARG)		(((ARG) & 0xFF) << 6)
#define CAMAF_DW9710_DATA_R(ARG)	(((ARG) >> 6) & 0xFF)
#define CAMAF_FREQUENCY_EQ1(mclk)     	((u16)(mclk/16000))

/* State of lens */
#define LENS_DETECTED 		1
#define LENS_NOT_DETECTED	0

/* Focus control values */
#define DEF_LENS_POSN		0	/* 0x7F */
#define LENS_POSN_STEP		1


/**
 * struct dw9710_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @priv_data_set: device private data (pointer) access function
 */
struct dw9710_platform_data {
	int (*power_set)(enum v4l2_power power);
	int (*priv_data_set)(void *);
};

/*
 * Sets the specified focus value [0(far) - 100(near)]
 */
int dw9710_af_setfocus(u16 posn);
int dw9710_af_getfocus(u16 *value);
#endif /* End of of CAMAF_DW9710_H */
