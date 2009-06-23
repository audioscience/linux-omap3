/*
 * tps65023-regulator.c
 *
 * Supports TPS65023 Regulator
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/tps65023.h>

struct tps_info {
	const char *name;
	unsigned min_uV;
	unsigned max_uV;
	bool fixed;
	u8 table_len;
	const u16 *table;
};

struct tps_pmic {
	struct regulator_desc desc[TPS65023_NUM_REGULATOR];
	struct i2c_client *client;
	struct regulator_dev *rdev[TPS65023_NUM_REGULATOR];
	const struct tps_info *info[TPS65023_NUM_REGULATOR];
};

static inline int tps_65023_read_reg(struct tps_pmic *tps, u8 reg, u8 *val)
{
	int status;

	status = i2c_smbus_read_byte_data(tps->client, reg);
	*val = status;
	if (status < 0)
		return status;
	return 0;
}

static inline int tps_65023_write_reg(struct tps_pmic *tps, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(tps->client, reg, val);
}

static int tps_65023_set_bits(struct tps_pmic *tps, u8 reg, u8 mask)
{
	u8 data;
	int err;

	err = tps_65023_read_reg(tps, reg, &data);
	if (err) {
			pr_err("Read from reg 0x%x failed\n", reg);
			return err;
	}

	data |= mask;

	return tps_65023_write_reg(tps, reg, data);
}

static int tps_65023_clear_bits(struct tps_pmic *tps, u8 reg, u8 mask)
{
	u8 data;
	int err;

	err = tps_65023_read_reg(tps, reg, &data);
	if (err) {
			pr_err("Read from reg 0x%x failed\n", reg);
			return err;
	}

	data &= ~mask;

	return tps_65023_write_reg(tps, reg, data);
}

static int tps65023_dcdc_is_enabled(struct regulator_dev *dev)
{
	unsigned char reg_ctrl;
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	int ret;
	u8 shift;

	if (dcdc < TPS65023_DCDC_1 || dcdc > TPS65023_DCDC_3)
		return -EINVAL;

	shift = TPS65023_NUM_REGULATOR - dcdc;
	ret = tps_65023_read_reg(tps, TPS65023_REG_REG_CTRL, &reg_ctrl);

	if (ret == 0) {
		reg_ctrl &= (1 << shift);
		return reg_ctrl ? 1 : 0;
	} else
		return ret;
}

static int tps65023_ldo_is_enabled(struct regulator_dev *dev)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	int ldo = rdev_get_id(dev);
	int ret;
	u8 shift;
	unsigned char reg_ctrl;

	if (ldo < TPS65023_LDO_1 || ldo > TPS65023_LDO_2)
		return -EINVAL;

	shift = (ldo == TPS65023_LDO_1 ? 1 : 2);
	ret = tps_65023_read_reg(tps, TPS65023_REG_REG_CTRL, &reg_ctrl);

	if (ret == 0) {
		reg_ctrl &= (1 << shift);
		return reg_ctrl ? 1 : 0;
	} else
		return ret;
}

static int tps65023_dcdc_enable(struct regulator_dev *dev)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	u8 shift;

	if (dcdc < TPS65023_DCDC_1 || dcdc > TPS65023_DCDC_3)
		return -EINVAL;

	shift = TPS65023_NUM_REGULATOR - dcdc;
	return tps_65023_set_bits(tps, TPS65023_REG_REG_CTRL, 1 << shift);
}

static int tps65023_dcdc_disable(struct regulator_dev *dev)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	u8 shift;

	if (dcdc < TPS65023_DCDC_1 || dcdc > TPS65023_DCDC_3)
		return -EINVAL;

	shift = TPS65023_NUM_REGULATOR - dcdc;
	return tps_65023_clear_bits(tps, TPS65023_REG_REG_CTRL, 1 << shift);
}

static int tps65023_ldo_enable(struct regulator_dev *dev)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	int ldo = rdev_get_id(dev);
	u8 shift;

	if (ldo < TPS65023_LDO_1 || ldo > TPS65023_LDO_2)
		return -EINVAL;

	shift = (ldo == TPS65023_LDO_1 ? 1 : 2);
	return tps_65023_set_bits(tps, TPS65023_REG_REG_CTRL, 1 << shift);
}

static int tps65023_ldo_disable(struct regulator_dev *dev)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	int ldo = rdev_get_id(dev);
	u8 shift;

	if (ldo < TPS65023_LDO_1 || ldo > TPS65023_LDO_2)
		return -EINVAL;

	shift = (ldo == TPS65023_LDO_1 ? 1 : 2);
	return tps_65023_clear_bits(tps, TPS65023_REG_REG_CTRL, 1 << shift);
}

static int tps65023_dcdc_get_voltage(struct regulator_dev *dev)
{
	unsigned char reg_ctrl;
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	int ret;

	if (dcdc < TPS65023_DCDC_1 || dcdc > TPS65023_DCDC_3)
		return -EINVAL;

	if (dcdc == TPS65023_DCDC_1) {
		ret = tps_65023_read_reg(tps, TPS65023_REG_DEF_CORE, &reg_ctrl);
		if (ret < 0)
			return ret;
		reg_ctrl &= (tps->info[dcdc]->table_len - 1);
		return tps->info[dcdc]->table[reg_ctrl] * 1000;
	} else
		return tps->info[dcdc]->min_uV;
}

static int tps65023_dcdc_set_voltage(struct regulator_dev *dev,
				int min_uV, int max_uV)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	int vsel;

	if (dcdc != TPS65023_DCDC_1)
		return -EINVAL;

	if (min_uV < tps->info[dcdc]->min_uV || min_uV > tps->info[dcdc]->max_uV)
		return -EINVAL;
	if (max_uV < tps->info[dcdc]->min_uV || max_uV > tps->info[dcdc]->max_uV)
		return -EINVAL;

	for (vsel = 0; vsel < tps->info[dcdc]->table_len; vsel++) {
		int mV = tps->info[dcdc]->table[vsel];
		int uV = mV * 1000;

		/* Break at the first in-range value */
		if (min_uV <= uV && uV <= max_uV)
			break;
	}

	/* write to the register */
	return tps_65023_write_reg(tps, TPS65023_REG_DEF_CORE, vsel);
}

static int tps65023_ldo_get_voltage(struct regulator_dev *dev)
{
	unsigned char reg_ctrl;
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	int ldo = rdev_get_id(dev);
	int ret;

	if (ldo < TPS65023_LDO_1 || ldo > TPS65023_LDO_2)
		return -EINVAL;

	ret = tps_65023_read_reg(tps, TPS65023_REG_LDO_CTRL, &reg_ctrl);
	if (ret < 0)
		return ret;

	reg_ctrl >>= (TPS65023_LDO_CTRL_LDOx_SHIFT(ldo - TPS65023_LDO_1));
	reg_ctrl &= (tps->info[ldo]->table_len - 1);
	return tps->info[ldo]->table[reg_ctrl] * 1000;
}

static int tps65023_ldo_set_voltage(struct regulator_dev *dev,
				int min_uV, int max_uV)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	int ldo = rdev_get_id(dev);
	int vsel;
	int ret;
	unsigned char reg_ctrl;

	if (ldo < TPS65023_LDO_1 || ldo > TPS65023_LDO_2)
		return -EINVAL;

	if (min_uV < tps->info[ldo]->min_uV || min_uV > tps->info[ldo]->max_uV)
		return -EINVAL;
	if (max_uV < tps->info[ldo]->min_uV || max_uV > tps->info[ldo]->max_uV)
		return -EINVAL;

	for (vsel = 0; vsel < tps->info[ldo]->table_len; vsel++) {
		int mV = tps->info[ldo]->table[vsel];
		int uV = mV * 1000;

		/* Break at the first in-range value */
		if (min_uV <= uV && uV <= max_uV)
			break;
	}

	ret = tps_65023_read_reg(tps, TPS65023_REG_LDO_CTRL, &reg_ctrl);

	if (ret < 0)
		return ret;

	reg_ctrl &= TPS65023_LDO_CTRL_LDOx_MASK(ldo - TPS65023_LDO_1);
	reg_ctrl |= (vsel << (TPS65023_LDO_CTRL_LDOx_SHIFT(ldo - TPS65023_LDO_1)));
	return tps_65023_write_reg(tps, TPS65023_REG_LDO_CTRL, reg_ctrl);
}

/* Operations permitted on VDCDCx */
static struct regulator_ops tps65023_dcdc_ops = {
	.is_enabled = tps65023_dcdc_is_enabled,
	.enable = tps65023_dcdc_enable,
	.disable = tps65023_dcdc_disable,
	.get_voltage = tps65023_dcdc_get_voltage,
	.set_voltage = tps65023_dcdc_set_voltage,
};

/* Operations permitted on LDOx */
static struct regulator_ops tps65023_ldo_ops = {
	.is_enabled = tps65023_ldo_is_enabled,
	.enable = tps65023_ldo_enable,
	.disable = tps65023_ldo_disable,
	.get_voltage = tps65023_ldo_get_voltage,
	.set_voltage = tps65023_ldo_set_voltage,
};

static
int tps_65023_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	static int desc_id;
	const struct tps_info *info = (void *)id->driver_data;
	struct regulator_init_data *init_data;
	struct regulator_dev *rdev;
	struct tps_pmic *tps;
	int i;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	/**
	 * init_data points to array of regulator_init structures
	 * coming from the board-evm file.
	 */
	init_data = client->dev.platform_data;

	if (!init_data)
		return -EIO;

	tps = kzalloc(sizeof(*tps), GFP_KERNEL);
	if (!tps)
		return -ENOMEM;

	tps->client = client;

	for (i = 0; i < TPS65023_NUM_REGULATOR; i++, info++, init_data++) {
		/* Store regulator specific information */
		tps->info[i] = info;

		tps->desc[i].name = info->name;
		tps->desc[i].id = desc_id++;
		tps->desc[i].ops = (i > TPS65023_DCDC_3 ?
					&tps65023_ldo_ops : &tps65023_dcdc_ops);
		tps->desc[i].type = REGULATOR_VOLTAGE;
		tps->desc[i].owner = THIS_MODULE;

		/* Register the regulators */
		rdev = regulator_register(&tps->desc[i], &client->dev,
								init_data, tps);
		if (IS_ERR(rdev)) {
			dev_err(&client->dev, "failed to register %s\n",
				id->name);
			kfree(tps);
			return PTR_ERR(rdev);
		}

		/* Save regulator for cleanup */
		tps->rdev[i] = rdev;
	}

	i2c_set_clientdata(client, tps);

	return 0;
}

/**
 * tps_65023_remove - TPS65023 driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister TPS driver as an i2c client device driver
 */
static int __devexit tps_65023_remove(struct i2c_client *client)
{
	struct tps_pmic *tps = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < TPS65023_NUM_REGULATOR; i++)
		regulator_unregister(tps->rdev[i]);

	tps->client = NULL;

	/* clear the client data in i2c */
	i2c_set_clientdata(client, NULL);
	kfree(tps);

	return 0;
}

/* Supported voltage values for regulators */
static const u16 VDCDC1_VSEL_table[] = {
	800, 825, 850, 875,
	900, 925, 950, 975,
	1000, 1025, 1050, 1075,
	1100, 1125, 1150, 1175,
	1200, 1225, 1250, 1275,
	1300, 1325, 1350, 1375,
	1400, 1425, 1450, 1475,
	1500, 1525, 1550, 1600,
};

static const u16 LDO1_VSEL_table[] = {
	1000, 1100, 1300, 1800,
	2200, 2600, 2800, 3150,
};

static const u16 LDO2_VSEL_table[] = {
	1050, 1200, 1300, 1800,
	2500, 2800, 3000, 3300,
};

static const struct tps_info tps65023_regs[] = {
	{
	.name = "VDCDC1",
	.min_uV =  800000,
	.max_uV = 1600000,
	.fixed = 0,
	.table_len = ARRAY_SIZE(VDCDC1_VSEL_table),
	.table = VDCDC1_VSEL_table,
	},
	{
	.name = "VDCDC2",
	.min_uV =  3300000,
	.max_uV = 3300000,
	.fixed = 1,
	.table_len = 0,
	},
	{
	.name = "VDCDC3",
	.min_uV =  1800000,
	.max_uV = 1800000,
	.fixed = 1,
	.table_len = 0,
	},
	{
	.name = "LDO1",
	.min_uV = 1000000,
	.max_uV = 3150000,
	.fixed = 0,
	.table_len = ARRAY_SIZE(LDO1_VSEL_table),
	.table = LDO1_VSEL_table,
	},
	{
	.name = "LDO2",
	.min_uV = 1050000,
	.max_uV = 3300000,
	.fixed = 0,
	.table_len = ARRAY_SIZE(LDO2_VSEL_table),
	.table = LDO2_VSEL_table,
	},
};

static const struct i2c_device_id tps_65023_id = {
	.name = "tps65023",
	.driver_data = (unsigned long) &tps65023_regs[0],
};

MODULE_DEVICE_TABLE(i2c, tps_65023_id);

static struct i2c_driver tps_65023_i2c_driver = {
	.driver = {
		.name = "tps_65023_pwr",
		.owner = THIS_MODULE,
	},
	.probe = tps_65023_probe,
	.remove = __devexit_p(tps_65023_remove),
	.id_table = &tps_65023_id,
};

/**
 * tps_65023_init
 *
 * Module init function
 */
static int __init tps_65023_init(void)
{
	return i2c_add_driver(&tps_65023_i2c_driver);
}
subsys_initcall(tps_65023_init);

/**
 * tps_65023_cleanup
 *
 * Module exit function
 */
static void __exit tps_65023_cleanup(void)
{
	i2c_del_driver(&tps_65023_i2c_driver);
}
module_exit(tps_65023_cleanup);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("TPS65023 voltage regulator driver");
MODULE_LICENSE("GPLv2");
