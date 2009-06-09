/*
 * board-omap3evm-pmic.c
 *
 * Supports run-time detection of different Power Management ICs.
 *
 * Copyright (C) 2009 Texas Instrument Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as  published by the Free
 * Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <mach/common.h>

/*
 * Definitions specific to TWL4030
 */

/*
 * Definitions specific to TPS6235x
 */

/*
 * Definitions specific to TPS65023
 */

/*
 * Definitions specific to TPS65073
 */
#if defined(CONFIG_OMAP3EVM_TPS65073)
/* MPU voltage regulator of DCDC type */
struct regulator_consumer_supply tps65073_mpu_consumers = {
	.supply = "vdd1",
};

/* CORE voltage regulator of DCDC type */
struct regulator_consumer_supply tps65073_core_consumers = {
	.supply = "vdd2",
};

/* SRAM/MEM/WKUP_BG/VDDS voltage regulator of DCDC type */
struct regulator_consumer_supply tps65073_vdds_consumers = {
	.supply = "vdds",
};

/* DPLL voltage regulator of LDO type */
struct regulator_consumer_supply tps65073_dpll_consumers = {
	.supply = "dpll",
};

/* MMC voltage regulator of LDO type */
struct regulator_consumer_supply tps65073_mmc_consumers = {
	.supply = "mmc",
};

struct regulator_init_data tps65073_regulator_data[] = {
	{
		.constraints = {
			.min_uV = 725000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &tps65073_vdds_consumers,
	},
	{
		.constraints = {
			.min_uV = 725000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &tps65073_core_consumers,
	},
	{
		.constraints = {
			.min_uV = 725000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &tps65073_mpu_consumers,
	},
	{
		.constraints = {
			.min_uV = 1000000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &tps65073_dpll_consumers,
	},
	{
		.constraints = {
			.min_uV = 725000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &tps65073_mmc_consumers,
	},
};

static struct i2c_board_info __initdata board_tps65073_instances[] = {
	{
		I2C_BOARD_INFO("tps6507x", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &tps65073_regulator_data[0],
	},
};
#endif

static int flag_pmic_twl4030;
static int flag_pmic_tps6235x;
static int flag_pmic_tps65023;
static int flag_pmic_tps65073;

/*
 * Detect the current PMIC, set one of the flags
 */
static inline int detect_pmic(void)
{
	/* How? Any suggestions?? This is a temporary solution. */
#if defined(CONFIG_TWL4030_CORE)
	flag_pmic_twl4030 = 1;
#endif

#if defined(CONFIG_OMAP3EVM_TPS6235X)
	flag_pmic_tps6235x = 1;
#endif

#if defined(CONFIG_OMAP3EVM_TPS65023)
	flag_pmic_tps65023 = 1;
#endif

#if defined(CONFIG_OMAP3EVM_TPS65073)
	flag_pmic_tps65073 = 1;
#endif

	return 0;
}

/* Functions to detect which PMIC is present */

int pmic_is_twl4030(void)
{
	return flag_pmic_twl4030;
}

int pmic_is_tps6235x(void)
{
	return flag_pmic_tps6235x;
}

int pmic_is_tps65020(void) { return 0; }

int pmic_is_tps65021(void) { return 0; }

int pmic_is_tps65022(void) { return 0; }

int pmic_is_tps65023(void)
{
	return flag_pmic_tps65023;
}

int pmic_is_tps65073(void)
{
	return flag_pmic_tps65073;
}

int pmic_is_tps65950(void)
{
	return flag_pmic_twl4030;
}

/* Detects the PMIC and initializes it accordingly */
int pmic_init(void)
{
#if defined(CONFIG_TWL4030_CORE)
	/* do stuff specific to TWL4030 */
#endif

#if defined(CONFIG_OMAP3EVM_TPS6235X)
	/* do stuff specific to TPS62350 */
#endif

#if defined(CONFIG_OMAP3EVM_TPS65023)
	/* do stuff specific to TPS65023 */
#endif

#if defined(CONFIG_OMAP3EVM_TPS65073)
	/* do stuff specific to TPS65073 */
	omap_register_i2c_bus(1, 400, board_tps65073_instances,
		ARRAY_SIZE(board_tps65073_instances));
#endif

	return 0;
}

