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

/* Register definitions */
#define	TPS6507X_REG_PPATH1				0X01
#define	TPS6507X_REG_INT				0X02
#define	TPS6507X_REG_CHGCONFIG0				0X03
#define	TPS6507X_REG_CHGCONFIG1				0X04
#define	TPS6507X_REG_CHGCONFIG2				0X05
#define	TPS6507X_REG_CHGCONFIG3				0X06
#define	TPS6507X_REG_REG_ADCONFIG			0X07
#define	TPS6507X_REG_TSCMODE				0X08
#define	TPS6507X_REG_ADRESULT_1				0X09
#define	TPS6507X_REG_ADRESULT_2				0X0A
#define	TPS6507X_REG_PGOOD				0X0B
#define	TPS6507X_REG_PGOODMASK				0X0C
#define	TPS6507X_REG_CON_CTRL1				0X0D
#define	TPS6507X_REG_CON_CTRL2				0X0E
#define	TPS6507X_REG_CON_CTRL3				0X0F
#define	TPS6507X_REG_DEFDCDC1				0X10
#define	TPS6507X_REG_DEFDCDC2_LOW			0X11
#define	TPS6507X_REG_DEFDCDC2_HIGH			0X12
#define	TPS6507X_REG_DEFDCDC3_LOW			0X13
#define	TPS6507X_REG_DEFDCDC3_HIGH			0X14
#define	TPS6507X_REG_DEFSLEW				0X15
#define	TPS6507X_REG_LDO_CTRL1				0X16
#define	TPS6507X_REG_DEFLDO2				0X17
#define	TPS6507X_REG_WLED_CTRL1				0X18
#define	TPS6507X_REG_WLED_CTRL2				0X19

/* CON_CTRL1 bitfields */
#define	TPS6507X_CON_CTRL1_DCDC1_ENABLE		BIT(4)
#define	TPS6507X_CON_CTRL1_DCDC2_ENABLE		BIT(3)
#define	TPS6507X_CON_CTRL1_DCDC3_ENABLE		BIT(2)
#define	TPS6507X_CON_CTRL1_LDO1_ENABLE		BIT(1)
#define	TPS6507X_CON_CTRL1_LDO2_ENABLE		BIT(0)

/* DEFDCDC1 bitfields */
#define TPS6507X_DEFDCDC1_DCDC1_EXT_ADJ_EN	BIT(7)
#define TPS6507X_DEFDCDC1_DCDC1_MASK		0X3F

/* DEFDCDC2_LOW bitfields */
#define TPS6507X_DEFDCDC2_LOW_DCDC2_MASK	0X3F

/* DEFDCDC2_HIGH bitfields */
#define TPS6507X_DEFDCDC2_HIGH_DCDC2_MASK	0X3F

/* DEFDCDC3_LOW bitfields */
#define TPS6507X_DEFDCDC3_LOW_DCDC3_MASK	0X3F

/* DEFDCDC3_HIGH bitfields */
#define TPS6507X_DEFDCDC3_HIGH_DCDC3_MASK	0X3F

/* TPS6507X_REG_LDO_CTRL1 bitfields */
#define TPS6507X_REG_LDO_CTRL1_LDO1_MASK	0X0F

/* TPS6507X_REG_DEFLDO2 bitfields */
#define TPS6507X_REG_DEFLDO2_LDO2_MASK		0X3F

/* VDCDC MASK */
#define TPS6507X_DEFDCDCX_DCDC_MASK		0X3F

/* DCDC's */
#define TPS6507X_DCDC_1				0
#define TPS6507X_DCDC_2				1
#define TPS6507X_DCDC_3				2
/* LDOs */
#define TPS6507X_LDO_1				3
#define TPS6507X_LDO_2				4

#define TPS6507X_MAX_REG_ID			TPS6507X_LDO_2

/* Number of step-down converters available */
#define TPS6507X_NUM_DCDC			3
/* Number of LDO voltage regulators  available */
#define TPS6507X_NUM_LDO			2
/* Number of total regulators available */
#define TPS6507X_NUM_REGULATOR			(TPS6507X_NUM_DCDC + TPS6507X_NUM_LDO)

