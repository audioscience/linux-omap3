/*
 * tps6235x-regulator.c -- support regulators in tps6235x family chips
 *
 * Author : Manikandan Pillai<mani.pillai@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
#include <linux/delay.h>

/*
 * These chips are often used in OMAP-based systems.
 *
 * This driver implements software-based resource control for various
 * voltage regulators.  This is usually augmented with state machine
 * based control.
 *
 * For now, all regulator operations apply to VSEL1 (the "ceiling"),
 * instead of VSEL0 (the "floor") which is used for low power modes.
 * Also, this *assumes* only software mode control is used...
*/

#define	TPS6235X_REG_VSEL0	0
#define	TPS6235X_REG_VSEL1	1
#define	TPS6235X_REG_CTRL1	2
#define	TPS6235X_REG_CTRL2	3

/* VSEL bitfields (EN_DCDC is shared) */
#define	TPS6235X_EN_DCDC	BIT(7)
#define	TPS6235X_LIGHTPFM	BIT(6)
#define	TPS6235X_VSM_MSK	(0x3F)

/* CTRL1 bitfields */
#define TPS6235X_EN_SYNC	BIT(5)
#define TPS6235X_HW_nSW		BIT(4)

/* CTRL2 bitfields */
#define TPS6235X_PWR_OK_MSK	BIT(5)
#define TPS6235X_OUT_DIS_MSK	BIT(6)
#define TPS6235X_GO_MSK		BIT(7)

struct tps_info {
	unsigned		min_uV;
	unsigned		max_uV;
	unsigned		mult_uV;
	bool			fixed;
};

struct tps {
	struct regulator_desc	desc;
	struct i2c_client	*client;
	struct regulator_dev	*rdev;
	const struct tps_info	*info;
};

static inline int tps_6235x_read_reg(struct tps *tps, u8 reg, u8 *val)
{
	int status;

	status = i2c_smbus_read_byte_data(tps->client, reg);
	*val = status;
	if (status < 0)
		return status;
	return 0;
}

static inline int tps_6235x_write_reg(struct tps *tps, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(tps->client, reg, val);
}

static int tps6235x_dcdc_is_enabled(struct regulator_dev *dev)
{
	unsigned char vsel1;
	struct tps *tps = rdev_get_drvdata(dev);

	tps_6235x_read_reg(tps, TPS6235X_REG_VSEL1, &vsel1);

	return !(vsel1 & TPS6235X_EN_DCDC);
}

static int tps6235x_dcdc_enable(struct regulator_dev *dev)
{
	unsigned char vsel1;
	int ret;
	struct tps *tps = rdev_get_drvdata(dev);

	ret = tps_6235x_read_reg(tps, TPS6235X_REG_VSEL1, &vsel1);

	if (ret == 0) {
		vsel1 |= TPS6235X_EN_DCDC;
		ret = tps_6235x_write_reg(tps, TPS6235X_REG_VSEL1, vsel1);
	}
	return ret;
}

static int tps6235x_dcdc_disable(struct regulator_dev *dev)
{
	unsigned char vsel1;
	int ret;
	struct tps *tps = rdev_get_drvdata(dev);

	ret = tps_6235x_read_reg(tps, TPS6235X_REG_VSEL1, &vsel1);
	if (ret == 0) {
		vsel1 &= ~(TPS6235X_EN_DCDC);
		ret = tps_6235x_write_reg(tps, TPS6235X_REG_VSEL1, vsel1);
	}
	return ret;
}

static int tps6235x_dcdc_get_voltage(struct regulator_dev *dev)
{
	struct tps *tps = rdev_get_drvdata(dev);
	unsigned char vsel1;
	const struct tps_info *info = tps->info;
	int status;

	status = tps_6235x_read_reg(tps, TPS6235X_REG_VSEL1, &vsel1);
	if (status < 0)
		return status;
	return info->min_uV + ((vsel1 & TPS6235X_VSM_MSK) * info->mult_uV);
}

static int tps6235x_dcdc_set_voltage(struct regulator_dev *dev,
				int min_uV, int max_uV)
{
	struct tps *tps = rdev_get_drvdata(dev);
	const struct tps_info *info = tps->info;
	unsigned char vsel1;
	unsigned step;
	int status;

	/* Output voltage set is = min_op_volt + ( VSM * 12.5mv) */
	/* compute and sanity-check voltage step multiplier */
	step = DIV_ROUND_UP(min_uV - info->min_uV, info->mult_uV);
	if ((info->min_uV + (step * info->mult_uV)) > max_uV)
		return -EINVAL;

	status = tps_6235x_read_reg(tps, TPS6235X_REG_VSEL1, &vsel1);
	if (status < 0)
		return status;

	/* update voltage */
	vsel1 &= ~TPS6235X_VSM_MSK;
	vsel1 |= step;
	return tps_6235x_write_reg(tps, TPS6235X_REG_VSEL1, vsel1);
}

/* tps6345{0,2,4,5} have some parameters hard-wired */
static struct regulator_ops tps6235x_fixed_dcdc_ops = {
	.is_enabled	= tps6235x_dcdc_is_enabled,
	.get_voltage	= tps6235x_dcdc_get_voltage,
	.set_voltage	= tps6235x_dcdc_set_voltage,
};

/* tps6345{1,3,6} are more programmable */
static struct regulator_ops tps6235x_dcdc_ops = {
	.is_enabled	= tps6235x_dcdc_is_enabled,
	.enable		= tps6235x_dcdc_enable,
	.disable	= tps6235x_dcdc_disable,
	.get_voltage	= tps6235x_dcdc_get_voltage,
	.set_voltage	= tps6235x_dcdc_set_voltage,

};

static
int tps_6235x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	static int desc_id;
	const struct tps_info *info = (void *)id->driver_data;
	struct regulator_init_data *init_data;
	struct regulator_dev *rdev;
	struct tps *tps;

	unsigned char reg_val;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	init_data = client->dev.platform_data;
	if (!init_data)
		return -EIO;

	tps = kzalloc(sizeof(*tps), GFP_KERNEL);
	if (!tps)
		return -ENOMEM;

	tps->desc.name = id->name;
	tps->desc.id = desc_id++;
	tps->desc.ops = info->fixed ? &tps6235x_fixed_dcdc_ops :
			&tps6235x_dcdc_ops;
	tps->desc.type = REGULATOR_VOLTAGE;
	tps->desc.owner = THIS_MODULE;

	tps->client = client;
	tps->info = info;

	/* FIXME board init code should provide init_data->driver_data
	 * saying how to configure this regulator:  how big is the
	 * inductor (affects light PFM mode optimization), slew rate,
	 * PLL multiplier, and so forth.
	 */
	tps_6235x_read_reg(tps, TPS6235X_REG_CTRL2, &reg_val);

	reg_val |= (TPS6235X_OUT_DIS_MSK | TPS6235X_GO_MSK);

	tps_6235x_write_reg(tps, TPS6235X_REG_CTRL2, reg_val);
	tps_6235x_read_reg(tps, TPS6235X_REG_CTRL2, &reg_val);

	if (reg_val & TPS6235X_PWR_OK_MSK)
		dev_dbg(&client->dev, "Power is OK  %x\n", reg_val);
	else
		dev_err(&client->dev, "Power not in range \n");

	/* Register the regulators */
	rdev = regulator_register(&tps->desc, &client->dev, tps);

	if (IS_ERR(rdev)) {
		dev_err(&client->dev, "failed to register %s\n", id->name);
			kfree(tps);

		return PTR_ERR(rdev);
	}

	/* Save regulator for cleanup */
	tps->rdev = rdev;
	i2c_set_clientdata(client, tps);

	return 0;
}

/**
 * tps_6235x_remove - TPS6235x driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister  TPS driver as an i2c client device driver
 */
static int __devexit tps_6235x_remove(struct i2c_client *client)
{
	struct tps *tps = i2c_get_clientdata(client);
	regulator_unregister(tps->rdev);
	/* clear the client data in i2c */
	i2c_set_clientdata(client, NULL);
	kfree(tps);
	return 0;
}

/*
 * These regulators have the same register structure, and differ
 * primarily according to supported voltages and default settings.
 */
static const struct tps_info tps62350_info = {
	.min_uV		=  750000,
	.max_uV		= 1537500,
	.mult_uV	=   12500,
	.fixed		=	1,
};
static const struct tps_info tps62351_info = {
	.min_uV		=  900000,
	.max_uV		= 1687500,
	.mult_uV	=   12500,
};
static const struct tps_info tps62352_info = {
	.min_uV		=  750000,
	.max_uV		= 1437500,
	.mult_uV	=   12500,
	.fixed		=	1,
};
static const struct tps_info tps62353_info = {
	.min_uV		=  750000,
	.max_uV		= 1537500,
	.mult_uV	=   12500,
};
static const struct tps_info tps62354_info = {
	.min_uV		=  750000,
	.max_uV		= 1537500,
	.mult_uV	=   12500,
	.fixed		=	1,
};
static const struct tps_info tps62355_info = {
	.min_uV		=  750000,
	.max_uV		= 1537500,
	.mult_uV	=   12500,
	.fixed		=	1,
};
static const struct tps_info tps62356_info = {
	.min_uV		= 1500000,
	.max_uV		= 1975000,
	.mult_uV	=   25000,
};

static const struct i2c_device_id tps_6235x_id[] = {
	{ "tps62350", (unsigned long) &tps62350_info, },
	{ "tps62351", (unsigned long) &tps62351_info, },
	{ "tps62352", (unsigned long) &tps62352_info, },
	{ "tps62353", (unsigned long) &tps62353_info, },
	{ "tps62354", (unsigned long) &tps62354_info, },
	{ "tps62355", (unsigned long) &tps62355_info, },
	{ "tps62356", (unsigned long) &tps62356_info, },
	{},
};

MODULE_DEVICE_TABLE(i2c, tps_6235x_id);

static struct i2c_driver tps_6235x_i2c_driver = {
	.driver = {
		.name	=	"tps_6235x_pwr",
		.owner	=	THIS_MODULE,
	},
	.probe		= tps_6235x_probe,
	.remove		= __devexit_p(tps_6235x_remove),
	.id_table	= tps_6235x_id,
};

/**
 * tps_6235x_init
 *
 * Module init function
 */
static int __init tps_6235x_init(void)
{
	return i2c_add_driver(&tps_6235x_i2c_driver);
}
subsys_initcall(tps_6235x_init);

/**
 * tps_6235x_cleanup
 *
 * Module exit function
 */
static void __exit tps_6235x_cleanup(void)
{
	i2c_del_driver(&tps_6235x_i2c_driver);
}
module_exit(tps_6235x_cleanup);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("TPS6235x voltage regulator driver");
MODULE_LICENSE("GPL");
