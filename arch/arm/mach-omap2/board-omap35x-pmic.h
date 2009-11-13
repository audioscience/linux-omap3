/*
 * board-omap35x-pmic.h
 *
 * Macros to create regulator supplies and regulator init data, along with the
 * default wrappers for various TI PMICs like TWL4030/TPS65950, TPS65023 etc.
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
#include <linux/platform_device.h>
#include <plat/common.h>

/* Create supplies for a specific regulator */
#define REGULATOR_COMSUMER_START(regulator) \
	static struct regulator_consumer_supply regulator##_consumers[]
/* Add/define supplies to the specific regulator */
#define REGULATOR_COMSUMER_DEFINE(s, device) \
		{ \
			.supply = #s, \
			.dev = device, \
		}

/* Define regulator with no supplies attached to it */
#define REGULATOR_CONSUMER_NO_SUPPLY(regulator) \
	REGULATOR_COMSUMER_START(regulator) = {}

/* Define regulator with a single supply attached to it */
#define REGULATOR_CONSUMER_SINGLE_SUPPLY(regulator, s, device) \
	REGULATOR_COMSUMER_START(regulator) = { \
	REGULATOR_COMSUMER_DEFINE(s, device), \
	}

/* Define regulator with multiple supplies attached to it like */
	/*
	REGULATOR_COMSUMER_START(name) = {
		REGULATOR_COMSUMER_DEFINE(supply, device),
		REGULATOR_COMSUMER_DEFINE(supply, device),
		REGULATOR_COMSUMER_DEFINE(supply, device),
	}
	*/

/* Define constraints flags */
#define REGULATOR_CONSTRAINTS_FLAGS(reg_always_on, reg_boot_on, reg_apply_uV) \
			.always_on = reg_always_on, \
			.boot_on = reg_boot_on, \
			.apply_uV = reg_apply_uV,

/* Define regulation constraints */
#define REGULATOR_CONSTRAINTS(n, min, max, modes, ops, reg_on, apply_uv) \
		{ \
			.name = #n, \
			.min_uV	= min, \
			.max_uV	= max, \
			.valid_modes_mask = modes, \
			.valid_ops_mask	= ops, \
			.always_on = reg_on, \
			.apply_uV = apply_uv, \
		},

/* Declare the regulator initialization data */
#define REGULATOR_INIT_DATA_START(regulator) \
	static struct regulator_init_data regulator##_data[]

/* Populate various fields in the regulator initialization data */
#define REGULATOR_INIT_DATA_DEFINE(regulator, n, min, max, modes, ops, \
		reg_on, apply_uv) \
	{ \
		.constraints = REGULATOR_CONSTRAINTS(n, min, max, modes, ops, \
				reg_on, apply_uv) \
		.num_consumer_supplies = ARRAY_SIZE(regulator##_consumers), \
		.consumer_supplies = regulator##_consumers, \
	}

/* Define regulator initialization data */
#define REGULATOR_INIT_DATA(regulator,n,min,max,modes,ops,reg_on,apply_uv) \
	REGULATOR_INIT_DATA_START(regulator) = { \
	REGULATOR_INIT_DATA_DEFINE(regulator, n, min, max, modes, ops, \
			reg_on, apply_uv), \
	}

/*
 * Default wrappers specific to TWL4030/TPS65950 PMIC
 */
#if defined(CONFIG_PMIC_TWL4030) || defined(CONFIG_TWL4030_CORE)
#define TWL_REGULATOR_MODES_DEFAULT	(REGULATOR_MODE_NORMAL | \
					REGULATOR_MODE_STANDBY)
#define TWL_REGULATOR_OPS_DEFAULT	(REGULATOR_CHANGE_MODE | \
					REGULATOR_CHANGE_STATUS)

/* Default supplies for TWL4030 regulators */
#define TWL_VAUX1_SUPPLY	REGULATOR_CONSUMER_SINGLE_SUPPLY(vaux1, \
					vaux1, NULL)
#define TWL_VAUX2_SUPPLY	REGULATOR_CONSUMER_SINGLE_SUPPLY(vaux2, \
					vaux2, NULL)
#define TWL_VAUX3_SUPPLY	REGULATOR_CONSUMER_SINGLE_SUPPLY(vaux3, \
					vaux3, NULL)
#define TWL_VAUX4_SUPPLY	REGULATOR_CONSUMER_SINGLE_SUPPLY(vaux4, \
					vaux4, NULL)

#define TWL_VMMC1_SUPPLY	REGULATOR_CONSUMER_SINGLE_SUPPLY(vmmc1, \
					vmmc, NULL)
#define TWL_VMMC2_SUPPLY	REGULATOR_CONSUMER_SINGLE_SUPPLY(vmmc2, \
					vmmc, NULL)

#define TWL_VPLL1_SUPPLY	REGULATOR_CONSUMER_SINGLE_SUPPLY(vpll1, \
					vpll1, NULL)
#define TWL_VPLL2_SUPPLY	REGULATOR_CONSUMER_SINGLE_SUPPLY(vpll2, \
					vdvi, NULL)

#define TWL_VSIM_SUPPLY		REGULATOR_CONSUMER_SINGLE_SUPPLY(vsim, \
					vmmc_aux, NULL)
#define TWL_VDAC_SUPPLY		REGULATOR_CONSUMER_SINGLE_SUPPLY(vdac, \
					vdac, NULL)

#define TWL_VUSB1V5_SUPPLY	REGULATOR_CONSUMER_SINGLE_SUPPLY(vusb1v5, \
					vusb1v5, NULL)
#define TWL_VUSB1V8_SUPPLY	REGULATOR_CONSUMER_SINGLE_SUPPLY(vusb1v8, \
					vusb1v8, NULL)
#define TWL_VUSB3V1_SUPPLY	REGULATOR_CONSUMER_SINGLE_SUPPLY(vusb3v1, \
					vusb3v1, NULL)

/* Default initialization data for TWL4030 regulators */
/* VAUX1 */
#define TWL_VAUX1_DATA	REGULATOR_INIT_DATA(vaux1, VAUX1, 2800000, 2800000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

/* VAUX2 */
#define TWL_VAUX2_DATA	REGULATOR_INIT_DATA(vaux2, VAUX2, 2800000, 2800000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

/* VAUX3 */
#define TWL_VAUX3_DATA	REGULATOR_INIT_DATA(vaux3, VAUX3, 2800000, 2800000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

/* VAUX4 */
#define TWL_VAUX4_DATA	REGULATOR_INIT_DATA(vaux4, VAUX4, 1800000, 1800000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

/* VMMC1 */
#define TWL_VMMC1_DATA	REGULATOR_INIT_DATA(vmmc1, VMMC1, 1850000, 3150000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

/* VMMC2 */
#define TWL_VMMC2_DATA	REGULATOR_INIT_DATA(vmmc2, VMMC2, 1850000, 1850000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

/* VPLL1 */
#define TWL_VPLL1_DATA	REGULATOR_INIT_DATA(vpll1, VPLL1, 1300000, 1300000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

/* VPLL2 */
#define TWL_VPLL2_DATA	REGULATOR_INIT_DATA(vpll2, VDVI, 1800000, 1800000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

/* VSIM */
#define TWL_VSIM_DATA	REGULATOR_INIT_DATA(vsim, VSIM, 1800000, 3000000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

/* VDAC */
#define TWL_VDAC_DATA	REGULATOR_INIT_DATA(vdac, VDAC, 1800000, 1800000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

/* VUSB1V5 */
#define TWL_VUSB1V5_DATA	REGULATOR_INIT_DATA(vusb1v5, VUSB1V5, \
						1500000, 1500000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

/* VUSB1V8 */
#define TWL_VUSB1V8_DATA	REGULATOR_INIT_DATA(vusb1v8, VUSB1V8, \
						1800000, 1800000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

/* VUSB3V1 */
#define TWL_VUSB3V1_DATA	REGULATOR_INIT_DATA(vusb3v1, VUSB3V1, \
						3100000, 3100000, \
						TWL_REGULATOR_MODES_DEFAULT, \
						TWL_REGULATOR_OPS_DEFAULT, \
						false, true)

#endif	/* CONFIG_PMIC_TWL4030 || CONFIG_TWL4030_CORE */

/*
 * Definitions specific to TPS65023
 */
#if defined(CONFIG_PMIC_TPS65023)
#endif	/* CONFIG_PMIC_TPS65023 */

/*
 * Definitions specific to TPS65073
 */
#if defined(CONFIG_PMIC_TPS65073)
#endif	/* CONFIG_PMIC_TPS65073 */
