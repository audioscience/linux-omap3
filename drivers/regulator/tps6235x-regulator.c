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

extern int tps_6235x_read_reg(struct i2c_client *client, u8 reg, u8 *val);
extern int tps_6235x_write_reg(struct i2c_client *client, u8 reg, u8 val);
extern struct regulator_consumer_supply tps62352_core_consumers;
extern struct regulator_consumer_supply tps62352_mpu_consumers;

/* Minimum and Maximum dc-dc voltage supported by the TPS6235x devices
All voltages given in millivolts */
#define	TPS62352_MIN_CORE_VOLT	750
#define	TPS62352_MAX_CORE_VOLT	1537
#define	TPS62353_MIN_MPU_VOLT	750
#define	TPS62353_MAX_MPU_VOLT	1537

/* Register bit settings */
#define	TPS6235X_EN_DCDC	(0x1 << 0x7)
#define	TPS6235X_VSM_MSK	(0x3F)
#define	TPS6235X_EN_SYN_MSK	(0x1 << 0x5)
#define	TPS6235X_SW_VOLT_MSK	(0x1 << 0x4)
#define	TPS6235X_PWR_OK_MSK	(0x1 << 0x5)
#define	TPS6235X_OUT_DIS_MSK	(0x1 << 0x6)
#define	TPS6235X_GO_MSK		(0x1 << 0x7)

#define	MODULE_NAME		"tps_6235x_pwr"
/*
 * These chips are often used in OMAP-based systems.
 *
 * This driver implements software-based resource control for various
 * voltage regulators.  This is usually augmented with state machine
 * based control.
 */

/* LDO control registers ... offset is from the base of its register bank.
 * The first three registers of all power resource banks help hardware to
 * manage the various resource groups.
 */

#define	TPS6235X_REG_VSEL0	0
#define	TPS6235X_REG_VSEL1	1
#define	TPS6235X_REG_CTRL1	2
#define	TPS6235X_REG_CTRL2	3

/* Device addresses for TPS devices */
#define	TPS_62352_CORE_ADDR	0x4A
#define	TPS_62353_MPU_ADDR	0x48

extern int omap_i2c_match_child(struct device *dev, void *data);

static int tps6235x_dcdc_is_enabled(struct regulator_dev *dev)
{
	unsigned char vsel1;
	int ret;
	struct i2c_client *tps_info	=
			rdev_get_drvdata(dev);
	ret = tps_6235x_read_reg(tps_info, TPS6235X_REG_VSEL1, &vsel1);
	ret &= TPS6235X_EN_DCDC;
	if (ret)
		return 1;
	else
		return 0;
}

static int tps6235x_dcdc_enable(struct regulator_dev *dev)
{
	unsigned char vsel1;
	int ret = -1;
	struct i2c_client *client = rdev_get_drvdata(dev);

	ret = tps_6235x_read_reg(client, TPS6235X_REG_VSEL1, &vsel1);
	if (ret == 0) {
		vsel1 |= TPS6235X_EN_DCDC;
		ret = tps_6235x_write_reg(client, TPS6235X_REG_VSEL1, vsel1);
	}
	return ret;
}

static int tps6235x_dcdc_disable(struct regulator_dev *dev)
{
	unsigned char vsel1;
	int ret = -1;
	struct	i2c_client *client = rdev_get_drvdata(dev);

	ret = tps_6235x_read_reg(client, TPS6235X_REG_VSEL1, &vsel1);
	if (ret == 0) {
		vsel1 &= ~(TPS6235X_EN_DCDC);
		ret = tps_6235x_write_reg(client, TPS6235X_REG_VSEL1, vsel1);
	}
	return ret;
}

static int tps6235x_dcdc_get_voltage(struct regulator_dev *dev)
{
	struct	i2c_client *tps_info = rdev_get_drvdata(dev);
	unsigned char vsel1;
	unsigned int volt;

	/* Read the VSEL1 register to get VSM */
	tps_6235x_read_reg(tps_info, TPS6235X_REG_VSEL1, &vsel1);
	/* Output voltage set is = min_op_volt + ( VSM * 12.5mv) */
	/* To cut out floating point operation we will multiply by 25
	divide by 2 */
	volt = (((vsel1 & TPS6235X_VSM_MSK) * 25) / 2) + TPS62352_MIN_CORE_VOLT;

	return volt * 1000;
}

static int tps6235x_dcdc_set_voltage(struct regulator_dev *dev,
				int min_uV, int max_uV)
{
	unsigned char vsel1;
	unsigned int volt;
	struct	i2c_client *tps_info = rdev_get_drvdata(dev);
	unsigned int millivolts = min_uV / 1000;

	/* check if the millivolts is within range */
	if ((millivolts < TPS62352_MIN_CORE_VOLT) ||
		(millivolts > TPS62352_MAX_CORE_VOLT))
		return -1;

	/* Output voltage set is = min_op_volt + ( VSM * 12.5mv) */
	volt = millivolts - TPS62352_MIN_CORE_VOLT;
	volt /= 25;
	volt *= 2;
	vsel1 = ((TPS6235X_EN_DCDC) | (volt & TPS6235X_VSM_MSK));
	tps_6235x_write_reg(tps_info, TPS6235X_REG_VSEL1, vsel1);
	return 0;
}

static struct regulator_ops tps62352_dcdc_ops = {
	.is_enabled = tps6235x_dcdc_is_enabled,
	.get_voltage = tps6235x_dcdc_get_voltage,
	.set_voltage = tps6235x_dcdc_set_voltage,
};

static struct regulator_ops tps62353_dcdc_ops = {
	.is_enabled = tps6235x_dcdc_is_enabled,
	.enable = tps6235x_dcdc_enable,
	.disable = tps6235x_dcdc_disable,
	.get_voltage = tps6235x_dcdc_get_voltage,
	.set_voltage = tps6235x_dcdc_set_voltage,
};

static struct regulator_desc regulators[] = {
	{
		.name = "tps62352",
		.id = 2,
		.ops = &tps62352_dcdc_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "tps62353",
		.id = 3,
		.ops = &tps62353_dcdc_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static
int tps_6235x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct  regulator_dev           *rdev = NULL;
	unsigned char reg_val;
	struct device *dev_child = NULL;

	tps_6235x_read_reg(client, TPS6235X_REG_CTRL2, &reg_val);
	reg_val |= (TPS6235X_OUT_DIS_MSK | TPS6235X_GO_MSK);
	tps_6235x_write_reg(client, TPS6235X_REG_CTRL2, reg_val);
	tps_6235x_read_reg(client, TPS6235X_REG_CTRL2, &reg_val);

	if (reg_val & TPS6235X_PWR_OK_MSK)
		dev_dbg(&client->dev, "Power is OK  %x\n", reg_val);
	else {
		printk(KERN_ERR "Power not within range = %x\n", reg_val);
		return -2;
	}

	/* Register the regulators */
	if (client->addr == TPS_62352_CORE_ADDR) {
		dev_child = device_find_child(client->adapter->dev.parent,
				"vdd2", omap_i2c_match_child);
		/* dev needs to be inited since this is required to for get() */
		rdev = regulator_register(&regulators[0], dev_child,
					client);
	} else if (client->addr == TPS_62353_MPU_ADDR) {
		/* dev needs to be inited since this is required to for get() */
		dev_child = device_find_child(client->adapter->dev.parent,
				"vdd1", omap_i2c_match_child);
		rdev = regulator_register(&regulators[1], dev_child,
					client);
	}

	if (rdev == NULL) {
		printk(KERN_ERR "ERR in regulator registration\n");
		return -1;
	}

	/* Set the regulator platform data for unregistration later on */
	i2c_set_clientdata(client, rdev);

	return 0;
}

/**
 * tps_6235x_remove - TPS6235x driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister  TPS driver as an i2c client device driver
 */
static int __exit tps_6235x_remove(struct i2c_client *client)
{
	struct  regulator_dev   *rdev = NULL;

	if (!client->adapter)
		return -ENODEV; /* our client isn't attached */

	rdev = (struct  regulator_dev *)i2c_get_clientdata(client);
	regulator_unregister(rdev);
	/* clear the client data in i2c */
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id tps_6235x_id[] = {
	{ "tps62352", 0},
	{ "tps62353", 1},
	{},
};

MODULE_DEVICE_TABLE(i2c, tps_6235x_id);

static struct i2c_driver tps_6235x_i2c_driver = {
	.driver = {
		.name	=	MODULE_NAME,
		.owner	=	THIS_MODULE,
	},
	.probe		= tps_6235x_probe,
	.remove		= __exit_p(tps_6235x_remove),
	.id_table	= tps_6235x_id,
};

/**
 * tps_6235x_init
 *
 * Module init function
 */
static int __init tps_6235x_init(void)
{
	int err;

	err = i2c_add_driver(&tps_6235x_i2c_driver);
	if (err) {
		printk(KERN_ERR "Failed to register " MODULE_NAME ".\n");
		return err;
	}
	return 0;
}


/**
 * tps_6235x_cleanup
 *
 * Module exit function
 */
static void __exit tps_6235x_cleanup(void)
{
	i2c_del_driver(&tps_6235x_i2c_driver);
}

late_initcall(tps_6235x_init);
module_exit(tps_6235x_cleanup);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("TPS6235x based linux driver");
MODULE_LICENSE("GPL");
