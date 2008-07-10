/*
 * linux/arch/arm/mach-omap2/resource_34xx.h
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 */

#ifndef __ARCH_ARM_MACH_OMAP3_RESOURCE_34XX_H
#define __ARCH_ARM_MACH_OMAP3_RESOURCE_34XX_H

#include <asm/arch/resource.h>
#include <asm/arch/prcm.h>
#include <asm/arch/prcm_34xx.h>

/* #define DEBUG_RES_FRWK 1 */
#ifdef DEBUG_RES_FRWK
#define DPRINTK(fmt, args...)\
 printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

/* TODO These need to be moved to an appropriate TWL4030 file */
/* TWL4030 LDO IDs */
#define TWL4030_VAUX1_ID                0x0
#define TWL4030_VAUX2_ID                0x1
#define TWL4030_VAUX3_ID                0x2
#define TWL4030_VAUX4_ID                0x3
#define TWL4030_VMMC1_ID                0x4
#define TWL4030_VMMC2_ID                0x5
#define TWL4030_VPLL1_ID                0x6
#define TWL4030_VPLL2_ID                0x7
#define TWL4030_VSIM_ID                 0x8
#define TWL4030_VDAC_ID                 0x9
#define TWL4030_VINTANA1_ID             0xA
#define TWL4030_VINTANA2_ID             0xB
#define TWL4030_VINTDIG_ID              0xC
#define TWL4030_VIO_ID                  0xD
#define TWL4030_VUSB1V5_ID              0xE
#define TWL4030_VUSB1V8_ID              0xF
#define TWL4030_VUSB3V1_ID              0x10
#define TWL4030_LDO_MAX_ID              0x11

extern unsigned int vdd1_opp_setting(u32 target_opp_no);
extern unsigned int vdd2_opp_setting(u32 target_opp_no);
extern int prcm_set_memory_resource_on_state(unsigned short state);
int set_memory_resource_state(unsigned short);
/* Activation/Validation functions for various shared resources */
int activate_power_res(struct shared_resource *resp,
		       unsigned short current_level,
		       unsigned short target_level);
int activate_logical_res(struct shared_resource *resp,
			 unsigned short current_level,
			 unsigned short target_level);
int activate_memory_res(unsigned long prcm_id, unsigned short current_level,
			unsigned short target_level);
int activate_dpll_res(unsigned long prcm_id, unsigned short current_level,
			unsigned short target_level);
int validate_power_res(struct shared_resource *res,
			unsigned short current_level,
			unsigned short target_level);
int validate_logical_res(struct shared_resource *res,
			unsigned short current_level,
			unsigned short target_level);
int validate_memory_res(struct shared_resource *res,
			unsigned short current_level,
			unsigned short target_level);
int validate_dpll_res(struct shared_resource *res, unsigned short current_level,
			unsigned short target_level);
int activate_constraint(struct shared_resource *resp,
			unsigned short current_value,
			unsigned short target_value);
int activate_pd_constraint(struct shared_resource *resp,
			   unsigned short current_value,
			   unsigned short target_value);
int validate_constraint(struct shared_resource *res,
			unsigned short current_level,
			unsigned short target_level);

int activate_memory_logic(struct shared_resource *resp,
			 unsigned short current_level,
			 unsigned short target_level);
int validate_memory_logic(struct shared_resource *resp,
			 unsigned short current_level,
			 unsigned short target_level);

int activate_autoidle_resource(struct shared_resource *resp,
			unsigned short current_level,
			unsigned short target_level);
int validate_autoidle_resource(struct shared_resource *resp,
			unsigned short current_level,
			unsigned short target_level);

int activate_triton_power_res(struct shared_resource *resp,
			unsigned short current_level,
			unsigned short target_level);
int validate_triton_power_res(struct shared_resource *res,
			unsigned short current_level,
			unsigned short target_level);

static struct shared_resource dss = {
	.name = "dss",
	.prcm_id = DOM_DSS,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource cam = {
	.name = "cam",
	.prcm_id = DOM_CAM,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource iva2 = {
	.name = "iva2",
	.prcm_id = DOM_IVA2,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource sgx = {
	.name = "sgx",
	.prcm_id = DOM_SGX,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource usbhost = {
	.name = "usb",
	.prcm_id = DOM_USBHOST,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource per = {
	.name = "per",
	.prcm_id = DOM_PER,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource neon  = {
	.name = "neon",
	.prcm_id = DOM_NEON,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource core = {
	.name = "core",
	.prcm_id = DOM_CORE1,
	.res_type = RES_LOGICAL,
	.no_of_users = 0,
	.curr_level = LOGICAL_UNUSED,
	.max_levels = LOGICAL_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_logical_res,
	.validate = validate_logical_res,
};

/* Constraint resources */
static struct shared_resource latency = {
	.name = "latency",
	.prcm_id = RES_LATENCY_CO,
	.res_type = RES_LATENCY_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_constraint,
	.validate = validate_constraint,
};

static struct shared_resource arm_freq = {
	.name = "arm_freq",
	.prcm_id = PRCM_ARMFREQ_CONSTRAINT,
	.res_type = RES_FREQ_CO,
	.no_of_users = 0,
	.curr_level = curr_arm_freq,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_constraint,
	.validate = validate_constraint,
};

static struct shared_resource dsp_freq = {
	.name = "dsp_freq",
	.prcm_id = PRCM_DSPFREQ_CONSTRAINT,
	.res_type = RES_FREQ_CO,
	.no_of_users = 0,
	.curr_level = curr_dsp_freq,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_constraint,
	.validate = validate_constraint,
};

static struct shared_resource vdd1_opp = {
	.name = "vdd1_opp",
	.prcm_id = PRCM_VDD1_CONSTRAINT,
	.res_type = RES_OPP_CO,
	.no_of_users = 0,
	.curr_level = curr_vdd1_opp,
	.max_levels = max_vdd1_opp+1,
	.linked_res_num = 0,
	.action = activate_constraint,
	.validate = validate_constraint,
};

static struct shared_resource vdd2_opp = {
	.name = "vdd2_opp",
	.prcm_id = PRCM_VDD2_CONSTRAINT,
	.res_type = RES_OPP_CO,
	.no_of_users = 0,
	.curr_level = curr_vdd2_opp,
	.max_levels = max_vdd2_opp+1,
	.linked_res_num = 0,
	.action = activate_constraint,
	.validate = validate_constraint,
};

static char *lat_dss_linked_res[] = {"dss",};

static struct shared_resource lat_dss = {
	.name = "lat_dss",
	.prcm_id = PRCM_DSS_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_dss_linked_res) /
			  sizeof(lat_dss_linked_res[0]),
	.linked_res_names = lat_dss_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_cam_linked_res[] = {"cam",};

static struct shared_resource lat_cam = {
	.name = "lat_cam",
	.prcm_id = PRCM_CAM_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_cam_linked_res) /
			  sizeof(lat_cam_linked_res[0]),
	.linked_res_names = lat_cam_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_iva2_linked_res[] = {"iva2",};

static struct shared_resource lat_iva2 = {
	.name = "lat_iva2",
	.prcm_id = PRCM_IVA2_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_iva2_linked_res) /
			  sizeof(lat_iva2_linked_res[0]),
	.linked_res_names = lat_iva2_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_3d_linked_res[] = {"sgx",};

static struct shared_resource lat_3d = {
	.name = "lat_3d",
	.prcm_id = PRCM_3D_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_3d_linked_res) /
			  sizeof(lat_3d_linked_res[0]),
	.linked_res_names = lat_3d_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_usbhost_linked_res[] = {"usb",};

static struct shared_resource lat_usbhost = {
	.name = "lat_usbhost",
	.prcm_id = PRCM_USBHOST_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_usbhost_linked_res) /
			  sizeof(lat_usbhost_linked_res[0]),
	.linked_res_names = lat_usbhost_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_per_linked_res[] = {"per",};

static struct shared_resource lat_per = {
	.name = "lat_per",
	.prcm_id = PRCM_PER_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_per_linked_res) /
			  sizeof(lat_per_linked_res[0]),
	.linked_res_names = lat_per_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_neon_linked_res[] = {"neon",};

static struct shared_resource lat_neon = {
	.name = "lat_neon",
	.prcm_id = PRCM_NEON_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_neon_linked_res) /
			  sizeof(lat_neon_linked_res[0]),
	.linked_res_names = lat_neon_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_core1_linked_res[] = {"core",};

static struct shared_resource lat_core1 = {
	.name = "lat_core1",
	.prcm_id = PRCM_CORE1_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_core1_linked_res) /
			  sizeof(lat_core1_linked_res[0]),
	.linked_res_names = lat_core1_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

/* Domain logic and memory resource */
static struct shared_resource mpu_l2cacheon = {
	.name = "mpu_l2cacheon",
	.prcm_id = PRCM_MPU_L2CACHEON,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = MEMORY_OFF,
	.max_levels = MEMORY_MAXLEVEL_DOMAINON,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource mpu_l2cacheret = {
	.name = "mpu_l2cacheret",
	.prcm_id = PRCM_MPU_L2CACHERET,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = MEMORY_OFF,
	.max_levels = MEMORY_MAXLEVEL_DOMAINRET,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource mpu_logicl1cacheret = {
	.name = "mpu_logicl1cacheret",
	.prcm_id = PRCM_MPU_LOGICL1CACHERET,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = LOGIC_OFF,
	.max_levels = LOGIC_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource core_mem2on = {
	.name = "core_mem2on",
	.prcm_id = PRCM_CORE_MEM2ON,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = MEMORY_ON,
	.max_levels = MEMORY_MAXLEVEL_DOMAINON,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource core_mem1on = {
	.name = "core_mem1on",
	.prcm_id = PRCM_CORE_MEM1ON,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = MEMORY_ON,
	.max_levels = MEMORY_MAXLEVEL_DOMAINON,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource core_mem2ret = {
	.name = "core_mem2ret",
	.prcm_id = PRCM_CORE_MEM2RET,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = MEMORY_RET,
	.max_levels = MEMORY_MAXLEVEL_DOMAINRET,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource core_mem1ret = {
	.name = "core_mem1ret",
	.prcm_id = PRCM_CORE_MEM1RET,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = MEMORY_RET,
	.max_levels = MEMORY_MAXLEVEL_DOMAINRET,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource core_logicret = {
	.name = "core_logicret",
	.prcm_id = PRCM_CORE_LOGICRET,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = LOGIC_RET,
	.max_levels = LOGIC_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource per_logicret = {
	.name = "per_logicret",
	.prcm_id = PRCM_PER_LOGICRET,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = LOGIC_OFF,
	.max_levels = LOGIC_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource per_autoidle_res = {
	.name = "per_autoidle_res",
	.prcm_id = DPLL4_PER ,
	.res_type = RES_DPLL,
	.no_of_users = 0,
	.curr_level = DPLL_AUTOIDLE,
	.max_levels = DPLL_RES_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_autoidle_resource,
	.validate = validate_autoidle_resource,
};


static struct shared_resource mpu_autoidle_res = {
	.name = "mpu_autoidle_res",
	.prcm_id = DPLL1_MPU,
	.res_type = RES_DPLL,
	.no_of_users = 0,
	.curr_level = DPLL_AUTOIDLE,
	.max_levels = DPLL_RES_MAXLEVEL ,
	.linked_res_num = 0,
	.action = activate_autoidle_resource,
	.validate = validate_autoidle_resource,
};

static struct shared_resource core_autoidle_res = {
	.name = "core_autoidle_res",
	.prcm_id = DPLL3_CORE,
	.res_type = RES_DPLL,
	.no_of_users = 0,
	.curr_level = DPLL_AUTOIDLE,
	.max_levels = DPLL_RES_CORE_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_autoidle_resource,
	.validate = validate_autoidle_resource,
};

static struct shared_resource iva2_autoidle_res = {
	.name = "iva2_autoidle_res",
	.prcm_id = DPLL2_IVA2,
	.res_type = RES_DPLL,
	.no_of_users = 0,
	.curr_level = DPLL_AUTOIDLE,
	.max_levels = DPLL_RES_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_autoidle_resource,
	.validate = validate_autoidle_resource,
};

static struct shared_resource per2_autoidle_res = {
	.name = "per2_autoidle_res",
	.prcm_id = DPLL5_PER2,
	.res_type = RES_DPLL,
	.no_of_users = 0,
	.curr_level = DPLL_AUTOIDLE,
	.max_levels = DPLL_RES_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_autoidle_resource,
	.validate = validate_autoidle_resource,
};

static struct shared_resource t2_vaux1 = {
	.name = "t2_vaux1",
	.prcm_id = TWL4030_VAUX1_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VAUX1_OFF,
	.max_levels = T2_VAUX1_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vaux2 = {
	.name = "t2_vaux2",
	.prcm_id = TWL4030_VAUX2_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VAUX2_OFF,
	.max_levels = T2_VAUX2_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vaux3 = {
	.name = "t2_vaux3",
	.prcm_id = TWL4030_VAUX3_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VAUX3_OFF,
	.max_levels = T2_VAUX3_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vaux4 = {
	.name = "t2_vaux4",
	.prcm_id = TWL4030_VAUX4_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VAUX4_OFF,
	.max_levels = T2_VAUX4_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vmmc1 = {
	.name = "t2_vmmc1",
	.prcm_id = TWL4030_VMMC1_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VMMC1_OFF,
	.max_levels = T2_VMMC1_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vmmc2 = {
	.name = "t2_vmmc2",
	.prcm_id = TWL4030_VMMC2_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VMMC2_OFF,
	.max_levels = T2_VMMC2_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vpll2 = {
	.name = "t2_vpll2",
	.prcm_id = TWL4030_VPLL2_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VPLL2_OFF,
	.max_levels = T2_VPLL2_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vsim = {
	.name = "t2_vsim",
	.prcm_id = TWL4030_VSIM_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VSIM_OFF,
	.max_levels = T2_VSIM_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vdac = {
	.name = "t2_vdac",
	.prcm_id = TWL4030_VDAC_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VDAC_OFF,
	.max_levels = T2_VDAC_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vintana2 = {
	.name = "t2_vintana2",
	.prcm_id = TWL4030_VINTANA2_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VINTANA2_OFF,
	.max_levels = T2_VINTANA2_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vio = {
	.name = "t2_vio",
	.prcm_id = TWL4030_VIO_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VIO_OFF,
	.max_levels = T2_VIO_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vusb1v5 = {
	.name = "t2_vusb1v5",
	.prcm_id = TWL4030_VUSB1V5_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VUSB1V5_OFF,
	.max_levels = T2_VUSB1V5_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vusb1v8 = {
	.name = "t2_vusb1v8",
	.prcm_id = TWL4030_VUSB1V8_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VUSB1V8_OFF,
	.max_levels = T2_VUSB1V8_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vusb3v1 = {
	.name = "t2_vusb3v1",
	.prcm_id = TWL4030_VUSB3V1_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VUSB3V1_OFF,
	.max_levels = T2_VUSB3V1_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource *res_list[] = {
	/* Power domain resources */
	&dss,
	&cam,
	&iva2,
	&sgx,
	&usbhost,
	&per,
	&neon,
	/* Logical resources */
	&core,
	/* Constraints */
	&latency,
	&arm_freq,
	&dsp_freq,
	&vdd1_opp,
	&vdd2_opp,
	&lat_dss,
	&lat_cam,
	&lat_iva2,
	&lat_3d,
	&lat_usbhost,
	&lat_per,
	&lat_neon,
	&lat_core1,
	/*memory and logic resource */
	&mpu_l2cacheon,
	&mpu_l2cacheret,
	&mpu_logicl1cacheret,
	&core_mem2on,
	&core_mem1on,
	&core_mem2ret,
	&core_mem1ret,
	&core_logicret,
	&per_logicret,
	&per_autoidle_res,
	&mpu_autoidle_res,
	&core_autoidle_res,
	&iva2_autoidle_res,
	&per2_autoidle_res,
	&t2_vaux1,
	&t2_vaux2,
	&t2_vaux3,
	&t2_vaux4,
	&t2_vmmc1,
	&t2_vmmc2,
	&t2_vpll2,
	&t2_vsim,
	&t2_vdac,
	&t2_vintana2,
	&t2_vio,
	&t2_vusb1v5,
	&t2_vusb1v8,
	&t2_vusb3v1,
	NULL
};

#endif /* __ARCH_ARM_MACH_OMAP3_RESOURCE_34XX_H */
