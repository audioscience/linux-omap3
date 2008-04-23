/*
 * linux/drivers/power/bq27000_battery.c
 *
 * BQ27000 battery driver
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Author: Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include "../w1/w1.h"

#define BQ_BATTERY_TEMP		0x06
#define BQ_BATTERY_TEMP_CK_OFFS	10926
#define BQ_BATTERY_TEMP_FACT_D	40
#define BQ_BATTERY_TEMP_FACT_M	10
#define BQ_BATTERY_VOLT		0x08
#define BQ_BATTERY_RSOC		0x0B /* Relative State-of-Charge */
#define BQ_BATTERY_CUR		0x14
#define HIGH_BYTE(A)		((A) << 8)

extern void w1_bq27000_write(struct device *dev, u8 buf, u8 reg);
extern int w1_bq27000_read(struct device *dev, u8 reg);

struct bq27000_device_info {
	struct device 		*dev;
	struct device		*w1_dev;

	unsigned long		update_time;
	int			voltage_uV;
	int			current_uA;
	int			temp_C;
	int			charge_rsoc;

	struct power_supply	bat;
	struct delayed_work	monitor_work;
};

static enum power_supply_property bq27000_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int bq27000_battery_probe(struct platform_device *dev);
static int bq27000_battery_remove(struct platform_device *dev);
#ifdef CONFIG_PM
static int bq27000_battery_suspend(struct platform_device *dev,
	pm_message_t state);
static int bq27000_battery_resume(struct platform_device *dev);
#endif

static struct platform_driver bq2700_battery_driver = {
	.probe = bq27000_battery_probe,
	.remove = bq27000_battery_remove,
#ifdef CONFIG_PM
	.suspend = bq27000_battery_suspend,
	.resume = bq27000_battery_resume,
#endif
	.driver = {
		.name = "omap-bq2700-battery",
	},
};

static inline int read_bq_val(u8 reg, int *rt_value, int b_single,
	struct bq27000_device_info *di);

/*
 * Return the battery temperature in Celcius degrees
 * Or < 0 if something fails.
 */
static int bqbattery_temperature(struct bq27000_device_info *di)
{
	int ret, temp = 0;

	ret = read_bq_val(BQ_BATTERY_TEMP, &temp, 0, di);
	if (ret) {
		pr_err("BQ27000 battery driver:"
			"Error reading temperature from HDQ device\n");
		return ret;
	}

	return  ((temp * BQ_BATTERY_TEMP_FACT_M) - BQ_BATTERY_TEMP_CK_OFFS)
			/ BQ_BATTERY_TEMP_FACT_D;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bqbattery_voltage(struct bq27000_device_info *di)
{
	int ret, volt = 0;

	ret = read_bq_val(BQ_BATTERY_VOLT, &volt, 0, di);
	if (ret) {
		pr_err("BQ27000 battery driver:"
			"Error reading battery voltage from HDQ device\n");
		return ret;
	}

	return volt;
}

/*
 * Return the battery average current
 * Or < 0 if something fails.
 */
static int bqbattery_current(struct bq27000_device_info *di)
{
	int ret, curr = 0;

	ret = read_bq_val(BQ_BATTERY_CUR, &curr, 0, di);
	if (ret) {
		pr_err("BQ27000 battery driver:"
			"Error reading battery current from HDQ device\n");
		return ret;
	}

	return curr;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bqbattery_rsoc(struct bq27000_device_info *di)
{
	int ret, rsoc = 0;

	ret = read_bq_val(BQ_BATTERY_RSOC, &rsoc, 1, di);
	if (ret) {
		pr_err("BQ27000 battery driver:"
			"Error reading battery Relative"
			"State-of-Charge from HDQ device\n");
		return ret;
	}
	return rsoc;
}

/*
 * Read the content of battery reg through the HDQ interface.
 */
static inline int read_bq_val(u8 reg, int *rt_value, int b_single,
	struct bq27000_device_info *di)
{
	u8 val;

	val = w1_bq27000_read(di->w1_dev, reg);
	*rt_value = val;

	if (!b_single) {
		val = 0;
		val = w1_bq27000_read(di->w1_dev, reg + 1);
		*rt_value +=  HIGH_BYTE((int) val);
	}

	return 0;
}

/*
 * Read the battery temp, voltage, current and relative state of charge
 * through the HDQ interface.
 */
static void bq27000_battery_read_status(struct bq27000_device_info *di)
{
	di->temp_C = bqbattery_temperature(di);
	di->voltage_uV = bqbattery_voltage(di);
	di->current_uA = bqbattery_current(di);
	di->charge_rsoc = bqbattery_rsoc(di);
	return;
}

static void bq27000_battery_work(struct work_struct *work)
{
	struct bq27000_device_info *di = container_of(work,
		struct bq27000_device_info, monitor_work.work);

	bq27000_battery_read_status(di);
	schedule_delayed_work(&di->monitor_work, 100);
	return;
}

#define to_bq27000_device_info(x) container_of((x), \
				struct bq27000_device_info, bat);

static int bq27000_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq27000_device_info *di = to_bq27000_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_uV;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = di->charge_rsoc;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temp_C;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* need to get the correct percentage value per the
		   battery characteristics. Approx values for now.
		 */
		if (di->voltage_uV < 2894)
			val->intval = 5;
		else if (di->voltage_uV < 3451 && di->voltage_uV > 2894)
			val->intval = 20;
		else if (di->voltage_uV < 3902 && di->voltage_uV > 3451)
			val->intval = 50;
		else if (di->voltage_uV < 3949 && di->voltage_uV > 3902)
			val->intval = 75;
		else if (di->voltage_uV > 3949)
			val->intval = 90;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (di->voltage_uV == 0)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq27000_battery_probe(struct  platform_device *pdev)
{
	struct bq27000_device_info *di;
	int retval = 0;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		pr_err("BQ27000 battery driver:"
			"Failed to allocate device info structure\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;
	di->w1_dev = pdev->dev.parent;
	di->bat.name = "bq27000";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27000_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27000_battery_props);
	di->bat.get_property = bq27000_battery_get_property;
	di->bat.external_power_changed = NULL;

	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		pr_err("BQ27000 battery driver: Failed to register battery\n");
		goto batt_failed;
	}

	INIT_DELAYED_WORK(&di->monitor_work, bq27000_battery_work);
	schedule_delayed_work(&di->monitor_work, 0);

	return 0;

batt_failed:
	kfree(di);
	return retval;
}

static int bq27000_battery_remove(struct  platform_device *pdev)
{
	struct bq27000_device_info *di = platform_get_drvdata(pdev);

	flush_scheduled_work();
	power_supply_unregister(&di->bat);
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

#ifdef CONFIG_PM
static int bq27000_battery_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct bq27000_device_info *di = platform_get_drvdata(pdev);

	cancel_delayed_work(&di->monitor_work);
	return 0;
}

static int bq27000_battery_resume(struct platform_device *pdev)
{
	struct bq27000_device_info *di = platform_get_drvdata(pdev);

	schedule_delayed_work(&di->monitor_work, 0);
	return 0;
}
#endif

static int __init bq27000_battery_init(void)
{

	return platform_driver_register(&bq2700_battery_driver);

}

static void __exit bq27000_battery_exit(void)
{
	platform_driver_unregister(&bq2700_battery_driver);
}

module_init(bq27000_battery_init);
module_exit(bq27000_battery_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("BQ27000 battery moniter driver");
MODULE_LICENSE("GPL");
