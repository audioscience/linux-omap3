/*
 * tps6507x.h
 *
 * Header file for TI TPS6507x Regulators
 *
 * Copyright (C) 2009 Texas Instrument Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __LINUX_MFD_TPS6507X_H
#define __LINUX_MFD_TPS6507X_H

extern int tps6507x_i2c_read_u8(u8 *val, u8 reg);
extern int tps6507x_i2c_write_u8(u8 val, u8 reg);

#endif
