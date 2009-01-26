/*
 * drivers/media/video/dw9710_priv.h
 *
 * Private defines for Auto Focus device
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Contributors:
 *	Sergio Aguirre <saaguirre@ti.com>
 * 	Troy Laramy
 * 	Mohit Jalori
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#ifndef DW9710_REGS_H
#define DW9710_REGS_H

#define DW9710_I2C_RETRY_COUNT		5
#define DW9710_DISABLE			1
#define DW9710_ENABLE			0
#define DW9710_POWERDN(ARG)		(((ARG) & 0x1) << 15)
#define DW9710_POWERDN_R(ARG)		(((ARG) >> 15) & 0x1)
#define DW9710_DATA(ARG)		(((ARG) & 0xFF) << 6)
#define DW9710_DATA_R(ARG)		(((ARG) >> 6) & 0xFF)

/* State of lens */
#define DW9710_LENS_DETECTED 		1
#define DW9710_LENS_NOT_DETECTED	0

/* Focus control values */
#define DW9710_DEF_LENS_POSN		0	/* 0x7F */
#define DW9710_LENS_POSN_STEP		1
#define DW9710_MAX_FOCUS_POS		0xFF

struct dw9710_device {
	const struct dw9710_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	int opened;
	u16 current_lens_posn;
	u16 saved_lens_posn;
	int state;
	int power_state;
};

/*
 * Sets the specified focus value [0(far) - 100(near)]
 */
int dw9710_af_setfocus(u16 posn);

int dw9710_af_getfocus(u16 *value);

#endif /* End of of DW9710_REGS_H */
