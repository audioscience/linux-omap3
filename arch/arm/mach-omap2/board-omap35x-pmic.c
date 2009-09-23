/*
 * board-omap35x-pmic.c
 *
 * Board specific information for different regulators and platforms.
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

#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <mach/common.h>

/*
 * Definitions specific to TWL4030/TPS65950
 */
#if defined(CONFIG_PMIC_TWL4030)
#if defined(CONFIG_MACH_OMAP3EVM)
#include <linux/platform_device.h>
#include <linux/i2c/twl4030.h>
#include <mach/display.h>

extern struct twl4030_platform_data omap3evm_twldata;
extern struct platform_device omap3_evm_dss_device;
extern struct omap_dss_device omap3_evm_lcd_device;

/* 1.8V VDAC */
static struct regulator_consumer_supply twl4030_vdac_supply[] = {
	{
		.supply	= "vdda_dac",
		.dev = &omap3_evm_dss_device.dev,
	},
};

/* VDAC for DSS driving S-Video */
static struct regulator_init_data omap3evm_vdac = {
	.constraints = {
		.min_uV	= 1800000,
		.max_uV	= 1800000,
		.apply_uV = true,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	= REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(twl4030_vdac_supply),
	.consumer_supplies = twl4030_vdac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_consumer_supply twl4030_vpll2_supply[] = {
	{
		.supply	= "vdvi",
		.dev = &omap3_evm_lcd_device.dev,
	},
	{
		.supply	= "vdds_sdi",
	},
};

static struct regulator_init_data omap3evm_vpll2 = {
	.constraints = {
		/* TBD: Min/Max voltages need to be verified */
		.min_uV	= 1800000,
		.max_uV	= 1800000,
		.apply_uV = true,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(twl4030_vpll2_supply),
	.consumer_supplies = twl4030_vpll2_supply,
};

/* 3.0V VMMC1 */
struct regulator_consumer_supply twl4030_vmmc1_supply[] = {
	{
		/* TBD: Supply name in spec (vdds_mmc1) is different */
		.supply	= "vmmc",
	},
	{
		/*
		 * TBD: Regulator created temporarily so that MMC
		 * driver doesn't fail. Hard-coded supply names
		 * need to be removed from the MMC driver.
		 */
		.supply	= "vmmc_aux",
	},
	{
		.supply = "vdds_sim",
	},
};

static struct regulator_init_data omap3evm_vmmc1 = {
	.constraints = {
		/* TBD: Min/Max voltages need to be verified */
		.min_uV	= 1850000,
		.max_uV	= 3150000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE	| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(twl4030_vmmc1_supply),
	.consumer_supplies = twl4030_vmmc1_supply,
};

static void __init pmic_twl4030_init(void)
{
	/* Initialize the regulator specific fields here */
	omap3evm_twldata.vdac = &omap3evm_vdac;
	omap3evm_twldata.vpll2 = &omap3evm_vpll2;
	omap3evm_twldata.vmmc1 = &omap3evm_vmmc1;
}
#endif	/* OMAP3EVM */
#else
static inline void pmic_twl4030_init(void)
{
}
#endif	/* CONFIG_PMIC_TWL4030 */

/*
 * Definitions specific to TPS65023
 */
#if defined(CONFIG_PMIC_TPS65023)
#if defined(CONFIG_MACH_OMAP3EVM)
/* 1.2V */
static struct regulator_consumer_supply tps65023_dcdc1_consumers[] = {
	{
		.supply = "vdd_mpu_iva",
	},
};

/* 1.15V */
static struct regulator_consumer_supply tps65023_dcdc2_consumers[] = {
	{
		.supply = "vdd_core",
	},
};

/* 1.8v */
static struct regulator_consumer_supply tps65023_dcdc3_consumers[] = {
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_sram",
	},
	{
		.supply = "vdds_mem",
	},
	{
		.supply = "vdds_wkup_bg",
	},
};

/* 1.8V */
static struct regulator_consumer_supply tps65023_ldo1_consumers[] = {
	{
		.supply = "vdds_dpll_dll",
	},
	{
		.supply = "vdds_dpll_per",
	},
};

/* 3.3V */
static struct regulator_consumer_supply tps65023_ldo2_consumers[] = {
	{
		.supply = "vdds_mmc1",
	},
};

static struct regulator_init_data tps65023_regulator_data[] = {
	/* DCDC1 */
	{
		.constraints = {
			.min_uV = 950000,
			.max_uV = 1150000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc1_consumers),
		.consumer_supplies = tps65023_dcdc1_consumers,
	},
	/* DCDC2 */
	{
		.constraints = {
			.min_uV = 950000,
			.max_uV = 1350000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc2_consumers),
		.consumer_supplies = tps65023_dcdc2e_consumers,
	},
	/* DCDC3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc3_consumers),
		.consumer_supplies = tps65023_dcdc3_consumers,
	},
	/* LDO1 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_ldo1_consumers),
		.consumer_supplies = tps65023_ldo1_consumers,
	},
	/* LDO2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_ldo2_consumers),
		.consumer_supplies = tps65023_ldo2_consumers,
	},
};

static struct i2c_board_info __initdata board_tps65023_instances[] = {
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &tps65023_regulator_data[0],
	},
};

static void __init pmic_tps65023_init(void)
{
       omap_register_i2c_bus(1, 400, board_tps65023_instances,
		ARRAY_SIZE(board_tps65023_instances));
}
#endif	/* OMAP3EVM */

#if defined(CONFIG_MACH_OMAP3517EVM)
/* 1.2V */
static struct regulator_consumer_supply tps65023_dcdc1_consumers[] = {
	{
		.supply = "vdd_core",
	},
};

/* 3.3V/1.8V */
static struct regulator_consumer_supply tps65023_dcdc2_consumers[] = {
	{
		.supply = "vddshv",
	},
};

/* 1.8v */
static struct regulator_consumer_supply tps65023_dcdc3_consumers[] = {
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_dsi",		/* check again ??? */
	},
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdds_sram_mpu",
	},
	{
		.supply = "vddsosc",		/* check again ??? */
	},
};

/* 1.8V_USB */
static struct regulator_consumer_supply tps65023_ldo1_consumers[] = {
	{
		.supply = "vdda1p8v_usbphy",
	},
	{
		.supply = "vdda_dac",
	},
};

/* 3.3V */
static struct regulator_consumer_supply tps65023_ldo2_consumers[] = {
	{
		.supply = "vdda3p3v_usbphy",
	},
};

static struct regulator_init_data tps65023_regulator_data[] = {
	/* DCDC1 */
	{
		.constraints = {
			.min_uV = 1200000,
			.max_uV = 1200000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc1_consumers),
		.consumer_supplies = tps65023_dcdc1_consumers,
	},
	/* DCDC2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc2_consumers),
		.consumer_supplies = tps65023_dcdc2_consumers,
	},
	/* DCDC3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc3_consumers),
		.consumer_supplies = tps65023_dcdc3_consumers,
	},
	/* LDO1 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_ldo1_consumers),
		.consumer_supplies = tps65023_ldo1_consumers,
	},
	/* LDO2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_ldo2_consumers),
		.consumer_supplies = tps65023_ldo2_consumers,
	},
};

static struct i2c_board_info __initdata board_tps65023_instances[] = {
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &tps65023_regulator_data[0],
	},
};

static void __init pmic_tps65023_init(void)
{
	omap_register_i2c_bus(1, 400, board_tps65023_instances,
			ARRAY_SIZE(board_tps65023_instances));
}
#endif	/* OMAP3517EVM */
#else
#define board_tps65023_instances NULL
static inline void pmic_tps65023_init(void)
{
}
#endif	/* CONFIG_PMIC_TPS65023 */

/*
 * Definitions specific to TPS65073
 */
#if defined(CONFIG_PMIC_TPS65073)
#if defined(CONFIG_MACH_OMAP3EVM)
/* 1.8V */
static struct regulator_consumer_supply tps65073_dcdc1_consumers[] = {
	{
		.supply = "vdds_wkup_bg",
	},
	{
		.supply = "vdds_mem",
	},
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_sram",
	},
};

/* 1.2V */
static struct regulator_consumer_supply tps65073_dcdc2_consumers[] = {
	{
		.supply = "vddcore",
	},
};

/* 1.2v */
static struct regulator_consumer_supply tps65073_dcdc3_consumers[] = {
	{
		.supply = "vdd_mpu_iva",
	},
};

/* 1.8V */
static struct regulator_consumer_supply tps65073_ldo1_consumers[] = {
	{
		.supply = "vdds_dpll_dll",
	},
	{
		.supply = "vdds_dpll_per",
	},
};

/* 1.8V */
static struct regulator_consumer_supply tps65073_ldo2_consumers[] = {
	{
		.supply = "vdds_mmc1",
	},
};

static struct regulator_init_data tps65073_regulator_data[] = {
	/* TBD, check min/max values for all */
	/* DCDC1 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65073_dcdc1_consumers),
		.consumer_supplies = tps65073_dcdc1_consumers,
	},
	/* DCDC2 */
	{
		.constraints = {
			.min_uV = 1200000,
			.max_uV = 1200000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65073_dcdc2_consumers),
		.consumer_supplies = tps65073_dcdc2_consumers,
	},
	/* DCDC3 */
	{
		.constraints = {
			.min_uV = 1200000,
			.max_uV = 1200000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65073_dcdc3_consumers),
		.consumer_supplies = tps65073_dcdc3_consumers,
	},
	/* LDO1 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65073_ldo1_consumers),
		.consumer_supplies = tps65073_ldo1_consumers,
	},
	/* LDO2 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65073_ldo2_consumers),
		.consumer_supplies = tps65073_ldo2_consumers,
	},
};

static struct i2c_board_info __initdata board_tps65073_instances[] = {
	{
		I2C_BOARD_INFO("tps65073", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &tps65023_regulator_data[0],
	},
};

static void __init pmic_tps65073_init(void)
{
       omap_register_i2c_bus(1, 400, board_tps65073_instances,
		ARRAY_SIZE(board_tps65073_instances));
}
#endif	/* OMAP3EVM */
#else
#define board_tps65073_instances NULL
static inline void pmic_tps65073_init(void)
{
}
#endif	/* CONFIG_PMIC_TPS65073 */

/* Detects the PMIC and initializes it accordingly */
void omap35x_pmic_init(void)
{
	pmic_twl4030_init();
	pmic_tps65023_init();
	pmic_tps65073_init();
}
