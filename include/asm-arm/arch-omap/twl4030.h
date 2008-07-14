/*
 * twl4030.h - header for TWL4030 PM and audio CODEC device
 *
 * Copyright (C) 2005-2008 Texas Instruments, Inc.
 *
 * Based on tlv320aic23.c:
 * Copyright (c) by Kai Svahn <kai.svahn@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef __TWL4030_H_
#define __TWL4030_H_

/* USB ID */
#define TWL4030_MODULE_USB		0x00
/* AUD ID */
#define TWL4030_MODULE_AUDIO_VOICE	0x01
#define TWL4030_MODULE_GPIO		0x02
#define TWL4030_MODULE_INTBR		0x03
#define TWL4030_MODULE_PIH		0x04
#define TWL4030_MODULE_TEST		0x05
/* AUX ID */
#define TWL4030_MODULE_KEYPAD		0x06
#define TWL4030_MODULE_MADC		0x07
#define TWL4030_MODULE_INTERRUPTS	0x08
#define TWL4030_MODULE_LED		0x09
#define TWL4030_MODULE_MAIN_CHARGE	0x0A
#define TWL4030_MODULE_PRECHARGE	0x0B
#define TWL4030_MODULE_PWM0		0x0C
#define TWL4030_MODULE_PWM1		0x0D
#define TWL4030_MODULE_PWMA		0x0E
#define TWL4030_MODULE_PWMB		0x0F
/* POWER ID */
#define TWL4030_MODULE_BACKUP		0x10
#define TWL4030_MODULE_INT		0x11
#define TWL4030_MODULE_PM_MASTER	0x12
#define TWL4030_MODULE_PM_RECEIVER	0x13
#define TWL4030_MODULE_RTC		0x14
#define TWL4030_MODULE_SECURED_REG	0x15

/* IRQ information-need base */
#include <asm/arch/irqs.h>
/* TWL4030 interrupts */

#define TWL4030_MODIRQ_GPIO		(IH_TWL4030_BASE + 0)
#define TWL4030_MODIRQ_KEYPAD		(IH_TWL4030_BASE + 1)
#define TWL4030_MODIRQ_BCI		(IH_TWL4030_BASE + 2)
#define TWL4030_MODIRQ_MADC		(IH_TWL4030_BASE + 3)
#define TWL4030_MODIRQ_USB		(IH_TWL4030_BASE + 4)
#define TWL4030_MODIRQ_PWR		(IH_TWL4030_BASE + 5)
/* Rest are unsued currently*/

/* Offsets to Power Registers */
#define TWL4030_VAUX1_DEV_GRP		0x17
#define TWL4030_VAUX1_DEDICATED		0x1A
#define TWL4030_VAUX2_DEV_GRP		0x1B
#define TWL4030_VAUX2_DEDICATED		0x1E
#define TWL4030_VAUX3_DEV_GRP		0x1F
#define TWL4030_VAUX3_DEDICATED		0x22
#define TWL4030_VAUX4_DEV_GRP		0x23
#define TWL4030_VAUX4_DEDICATED		0x26
#define TWL4030_VMMC1_DEV_GRP		0x27
#define TWL4030_VMMC1_DEDICATED		0x2A
#define TWL4030_VMMC2_DEV_GRP		0x2B
#define TWL4030_VMMC2_DEDICATED		0x2E
#define TWL4030_VPLL1_DEV_GRP		0x2F
#define TWL4030_VPLL1_DEDICATED		0x32
#define TWL4030_VPLL2_DEV_GRP		0x33
#define TWL4030_VPLL2_DEDICATED		0x36
#define TWL4030_VSIM_DEV_GRP		0x37
#define TWL4030_VSIM_DEDICATED		0x3A
#define TWL4030_VDAC_DEV_GRP		0x3B
#define TWL4030_VDAC_DEDICATED		0x3E
#define TWL4030_VINTANA1_DEV_GRP	0x3F
#define TWL4030_VINTANA1_DEDICATED 	0x42
#define TWL4030_VINTANA2_DEV_GRP	0x43
#define TWL4030_VINTANA2_DEDICATED 	0x46
#define TWL4030_VINTDIG_DEV_GRP		0x47
#define TWL4030_VINTDIG_DEDICATED  	0x4A
#define TWL4030_VIO_DEV_GRP		0x4B
#define TWL4030_VIO_VSEL		0x54
#define TWL4030_VUSB1V5_DEV_GRP		0x71
#define TWL4030_VUSB1V8_DEV_GRP		0x74
#define TWL4030_VUSB3V1_DEV_GRP		0x77

/* TWL4030 LDO IDs */
#define TWL4030_VAUX1_ID		0x0
#define TWL4030_VAUX2_ID		0x1
#define TWL4030_VAUX3_ID		0x2
#define TWL4030_VAUX4_ID		0x3
#define TWL4030_VMMC1_ID		0x4
#define TWL4030_VMMC2_ID		0x5
#define TWL4030_VPLL1_ID		0x6
#define TWL4030_VPLL2_ID		0x7
#define TWL4030_VSIM_ID			0x8
#define TWL4030_VDAC_ID			0x9
#define TWL4030_VINTANA1_ID		0xA
#define TWL4030_VINTANA2_ID		0xB
#define TWL4030_VINTDIG_ID		0xC
#define TWL4030_VIO_ID			0xD
#define TWL4030_VUSB1V5_ID		0xE
#define TWL4030_VUSB1V8_ID		0xF
#define TWL4030_VUSB3V1_ID		0x10
#define TWL4030_LDO_MAX_ID		0x11

/* TWL4030 Power register values */
#define TWL4030_DEV_GRP_P1		0x20
#define TWL4030_DEV_GRP_NONE		0x0
#define TWL4030_DEDICATED_NONE		0x0

/* TWL4030 GPIO interrupt definitions */

#define TWL4030_GPIO_MIN		0
#define TWL4030_GPIO_MAX		18
#define TWL4030_GPIO_MAX_CD		2
#define TWL4030_GPIO_IRQ_NO(n)		(IH_TWL4030_GPIO_BASE+n)
#define TWL4030_GPIO_IS_INPUT		1
#define TWL4030_GPIO_IS_OUTPUT		0
#define TWL4030_GPIO_IS_ENABLE		1
#define TWL4030_GPIO_IS_DISABLE		0
#define TWL4030_GPIO_PULL_UP		0
#define TWL4030_GPIO_PULL_DOWN		1
#define TWL4030_GPIO_PULL_NONE		2
#define TWL4030_GPIO_EDGE_NONE		0
#define TWL4030_GPIO_EDGE_RISING	1
#define TWL4030_GPIO_EDGE_FALLING	2

/* Functions to read and write from TWL4030 */

/*
 * IMP NOTE:
 * The base address of the module will be added by the triton driver
 * It is the caller's responsibility to ensure sane values
 */
int twl4030_i2c_write_u8(u8 mod_no, u8 val, u8 reg);
int twl4030_i2c_read_u8(u8 mod_no, u8 *val, u8 reg);

 /*
  * i2c_write: IMPORTANT - Allocate value num_bytes+1 and valid data starts at
  *		Offset 1.
  */
int twl4030_i2c_write(u8 mod_no, u8 *value, u8 reg, u8 num_bytes);
int twl4030_i2c_read(u8 mod_no, u8 *value, u8 reg, u8 num_bytes);

/*
 * Exported TWL4030 GPIO APIs
 */
int twl4030_get_gpio_datain(int gpio);
int twl4030_set_gpio_dataout(int gpio, int enable);
int twl4030_request_gpio(int gpio);
int twl4030_set_gpio_direction(int gpio, int is_input);
int twl4030_set_gpio_edge_ctrl(int gpio, int edge);
int twl4030_set_gpio_debounce(int gpio, int enable);
int twl4030_free_gpio(int gpio);

#endif /* End of __TWL4030_H */
