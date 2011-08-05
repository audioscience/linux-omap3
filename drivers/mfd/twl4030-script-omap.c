/*
 * OMAP power script for PMIC TWL4030
 *
 * Author: Lesly A M <leslyam@ti.com>
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Lesly A M <leslyam@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/i2c/twl.h>

/*
 * power management signal connections for OMAP3430 with TWL5030
 *
 *                          TWL5030                             OMAP3430
 *                     ______________________             _____________________
 *                    |                      |           |                     |
 *                    |          (P1) NSLEEP1|<----------|SYS_OFFMODE          |
 *                    |              NRESWARM|<----------|NWARMRESET           |
 *                    |          (P2) NSLEEP2|---|       |                     |
 *                    |                      |  ===      |                     |
 *                    |                      |   -       |                     |
 *                    |                      |           |                     |
 *                    |                 VDD1 |---------->| VDD1                |
 *                    |                 VDD2 |---------->| VDD2                |
 *                    |                  VIO |---------->| VDDS                |
 *  ________          |                VAUX1 |           |                     |
 * |        |         |                 ...  |           |                     |
 * |  ENABLE|<--------|CLKEN           CLKREQ|<----------|SYS_CLKREQ           |
 * |  CLKOUT|-------->|HFCLKIN  (P3) HFCLKOUT|---------->|XTALIN               |
 * |________|         |______________________|           |_____________________|
 *
 *
 *	Signal descriptions:
 *
 * SYS_OFFMODE - OMAP drives this signal low only when the OMAP is in the
 *	OFF idle mode. It is driven high when a wake up event is detected.
 *	This signal should control the P1 device group in the PMIC.
 *
 * SYS_CLKREQ - OMAP should drive this signal low when the OMAP goes into
 *	any idle mode. This signal should control the P3 device group
 *	in the PMIC. It is used to notify PMIC when XTALIN is no longer needed.
 *
 * NSLEEP1(P1) - When this signal goes low the P1 sleep sequence is executed
 *	in the PMIC turning off certain resources. When this signal goes high
 *	the P1 active sequence is executed turning back on certain resources.
 *
 * NSLEEP2(P2) - This signal controls the P2 device group of the PMIC.
 *	It is not used in this setup and should be tied to ground.
 *	This can be used for connecting a different processor or MODEM chip.
 *
 * CLKREQ(P3) - When this signal goes low the P3 sleep sequence is executed
 *	in the PMIC turning off HFCLKOUT. When this signal goes high
 *	the P3 active sequence is executed turning back on HFCLKOUT and other
 *	resources.
 *
 * CLKEN - Enable signal for oscillator. Should only go low when OMAP is
 *	in the OFF idle mode due to long oscillator startup times.
 *
 * HFCLKIN - Oscillator output clock into PMIC.
 *
 * HFCLKOUT - System clock output from PMIC to OMAP.
 *
 * XTALIN - OMAP system clock input(HFCLKOUT).
 */

/*
 * Recommended sleep and active sequences for TWL5030 when connected to OMAP3
 *
 * WARNING: If the board is using NSLEEP2(P2), should modify this script and
 * setuptime values accordingly.
 *
 * Chip Retention/Off (using i2c for scaling voltage):
 *	When OMAP de-assert the SYS_CLKREQ signal, only HFCLKOUT is affected
 *	since it is the only resource assigned to P3 only.
 *
 * Sysoff (using sys_off signal):
 *	When OMAP de-assert the SYS_OFFMODE signal A2S(active to sleep sequence)
 *	on the PMIC is executed. This will put resources of TYPE2=1 and TYPE2=2
 *	into sleep. At this point only resources assigned to P1 only will be
 *	affected (VDD1, VDD2 & VPLL1).
 *
 *	Next the OMAP will lower SYS_CLKREQ which will allow the A2S sequence
 *	in PMIC to execute again. This will put resources of TYPE2=1 and TYPE2=2
 *	into sleep but will affect resources that are assigned to P3(HFCLKOUT)
 *	only or assigned to P1 and P3.
 *
 *	On wakeup event OMAP goes active and pulls the SYS_CLKREQ high,
 *	which will execute the P3 S2A sequence on the PMIC. This will turn on
 *	resources assigned to P3 or assigned to P1 and P3 and of TYPE2=2.
 *
 *	Next the OMAP will wait the PRM_VOLTOFFSET time and then de-assert
 *	the SYS_OFFMODE pin allowing the PMIC to execute the P1 S2A active
 *	sequence. This will turn on resources assigned to P1 or assigned to
 *	P1 and P3 and of TYPE2=1.
 *
 *	Timing diagram for OMAP wakeup from OFFMODE using sys_off signal
 *                 _____________________________________________________________
 * OMAP active  __/
 *                       |<--------------------PRM_CLKSETP-------------------->|
 *                        ______________________________________________________
 * SYS_CLKREQ   _________/
 *                           ___________________________________________________
 * CLKEN        ____________/
 *
 * HFCLKIN      _______________________________________________/////////////////
 *
 * HFCLKOUT     __________________________________________________//////////////
 *                       |<---PRM_VOLTOFFSET-->|
 *                                              ________________________________
 * SYS_OFFMODE  _______________________________/
 *                                             |<--------PRM_VOLTSETUP2------->|
 *                                                                   ___________
 * VPLL1        ____________________________________________________/
 *                                                                            __
 * VDD1         _____________________________________________________________/
 *                                                                            __
 * VDD2         _____________________________________________________________/
 *
 *	Other resources which are not handled by this script should be
 *	controlled by the respective drivers using them (VAUX1, VAUX2, VAUX3,
 *	VAUX4, VMMC1, VMMC2, VPLL2, VSIM, VDAC, VUSB1V5, VUSB1V8 & VUSB3V1).
 *
 * More info:
 *	http://omapedia.org/wiki/TWL4030_power_scripts
 */

/**
 * DOC: Sleep to active sequence for P1/P2
 *
 * Sequence to control the TWL4030 Power resources,
 * when the system wakeup from sleep.
 * Executed upon P1_P2 transition for wakeup
 * (sys_offmode signal de-asserted on OMAP).
 */
static struct twl4030_ins wakeup_p12_seq[] __initdata = {
	/*
	 * Broadcast message to put resources to active
	 *
	 * Since we are not using TYPE, resources which have TYPE2 configured
	 * as 1 will be targeted (VPLL1, VDD1, VDD2, REGEN, NRES_PWRON, SYSEN).
	 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p12_script __initdata = {
	.script	= wakeup_p12_seq,
	.size	= ARRAY_SIZE(wakeup_p12_seq),
	.flags	= TWL4030_WAKEUP12_SCRIPT,
};

/**
 * DOC: Sleep to active sequence for P3
 *
 * Sequence to control the TWL4030 Power resources,
 * when the system wakeup from sleep.
 * Executed upon P3 transition for wakeup
 * (clkreq signal asserted on OMAP).
 */
static struct twl4030_ins wakeup_p3_seq[] __initdata = {
	/*
	 * Broadcast message to put resources to active
	 *
	 * Since we are not using TYPE, resources which have TYPE2 configured
	 * as 2 will be targeted
	 * (VINTANA1, VINTANA2, VINTDIG, VIO, CLKEN, HFCLKOUT).
	 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p3_script __initdata = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

/**
 * DOC: Active to sleep sequence for P1/P2/P3
 *
 * Sequence to control the TWL4030 Power resources,
 * when the system goes into sleep.
 * Executed upon P1_P2/P3 transition for sleep.
 * (sys_offmode signal asserted/clkreq de-asserted on OMAP).
 */
static struct twl4030_ins sleep_on_seq[] __initdata = {
	/* Broadcast message to put res to sleep (TYPE2 = 1, 2) */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_SLEEP), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_SLEEP), 2},
};

static struct twl4030_script sleep_on_script __initdata = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

/**
 * DOC: Warm reset sequence
 *
 * Sequence to reset the TWL4030 Power resources,
 * when the system gets warm reset.
 * Executed upon warm reset signal.
 *
 * First the device is put in reset, then the system clock is requested to
 * the external oscillator, and default ON power reference and power providers
 * are enabled. Next some additional resources which are software controlled
 * are enabled. Finally sequence is ended by the release of TWL5030 reset.
 */
static struct twl4030_ins wrst_seq[] __initdata = {
	/*
	 * As a workaround for OMAP Erratum  (ID: i537 - OMAP HS devices are
	 * not recovering from warm reset while in OFF mode)
	 * NRESPWRON is toggled to force a power on reset condition to OMAP
	 */
	/* Trun OFF NRES_PWRON */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_NRES_PWRON, RES_STATE_OFF), 2},
	/* Reset twl4030 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 2},
	/* Reset MAIN_REF */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_MAIN_REF, RES_STATE_WRST), 2},
	/* Reset All type2_group2 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_WRST), 2},
	/* Reset VUSB_3v1 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VUSB_3V1, RES_STATE_WRST), 2},
	/* Reset All type2_group1 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_WRST), 2},
	/* Reset the Reset & Contorl_signals */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0,
							RES_STATE_WRST), 2},
	/* Re-enable twl4030 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 2},
	/* Trun ON NRES_PWRON */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_NRES_PWRON, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wrst_script __initdata = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

/* TWL4030 script for sleep, wakeup & warm_reset */
static struct twl4030_script *twl4030_scripts[] __initdata = {
	&wakeup_p12_script,
	&wakeup_p3_script,
	&sleep_on_script,
	&wrst_script,
};

/**
 * DOC: TWL4030 resource configuration
 *
 * Resource which are attached to P1 device group alone
 * will go to sleep state, when sys_off signal from OMAP is de-asserted.
 *	(VPLL1, VDD1, VDD2)
 *
 * None of the resources are attached to P2 device group alone.
 * (WARNING: If MODEM or connectivity chip is connected to NSLEEP2 PIN on
 * TWL4030, should modify the resource configuration accordingly).
 *
 * Resource which are attached to P3 device group alone
 * will go to sleep state, when clk_req signal from OMAP is de-asserted.
 *	(HFCLKOUT)
 *
 * Resource which are attached to more than one device group
 * will go to sleep state, when corresponding signals are de-asserted.
 *	(VINTANA1, VINTANA2, VINTDIG, VIO, REGEN, NRESPWRON, CLKEN, SYSEN)
 *
 * REGEN is an output of the device which can be connected to slave power ICs
 * or external LDOs that power on before voltage for the IO interface (VIO).
 *
 * SYSEN is a bidirectional signal of the device that controls slave power ICs.
 * In master mode, the device sets SYSEN high to enable the slave power ICs.
 * In slave mode, when one of the power ICs drives the SYSEN signal low,
 * all devices of the platform stay in the wait-on state.
 *
 * Resource which are attached to none of the device group by default
 * will be in sleep state. These resource should be controlled by
 * the respective drivers using them.
 * Resource which are controlled by drivers are not modified here.
 *	(VAUX1, VAUX2, VAUX3, VAUX4, VMMC1, VMMC2, VPLL2, VSIM, VDAC,
 *	VUSB1V5, VUSB1V8, VUSB3V1)
 *
 * Resource using reset values.
 *	(32KCLKOUT, TRITON_RESET, MAINREF)
 */
static struct twl4030_resconfig twl4030_rconfig[] __initdata = {
	{ .resource = RES_VPLL1, .devgroup = DEV_GRP_P1, .type = 3,
		.type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VINTANA1, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VINTANA2, .devgroup = DEV_GRP_ALL, .type = 0,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VINTDIG, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VIO, .devgroup = DEV_GRP_ALL, .type = 2,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VDD1, .devgroup = DEV_GRP_P1,
		.type = 4, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VDD2, .devgroup = DEV_GRP_P1,
		.type = 3, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_REGEN, .devgroup = DEV_GRP_ALL, .type = 2,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_NRES_PWRON, .devgroup = DEV_GRP_ALL, .type = 0,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_CLKEN, .devgroup = DEV_GRP_ALL, .type = 3,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_SYSEN, .devgroup = DEV_GRP_ALL, .type = 6,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3,
		.type = 0, .type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ 0, 0},
};

/*
 * Sleep and active sequences with changes for TWL5030 Erratum 27 workaround
 *
 * Sysoff (using sys_off signal):
 *	When SYS_CLKREQ goes low during retention no resources will be affected
 *	since no resources are assigned to P3 only.
 *
 *	Since all resources are assigned to P1 and P3 then all resources
 *	will be affected on the falling edge of P3 (SYS_CLKREQ).
 *	When OMAP lower the SYS_CLKREQ signal PMIC will execute the
 *	A2S sequence in which HFCLKOUT is dissabled first and
 *	after 488.32 usec(PRM_VOLTOFFSET) resources assigned to P1 and P3
 *	and of TYPE2=1 are put to sleep
 *	(VDD1, VDD2, VPLL1, REGEN, NRESPWRON & SYSEN).
 *	Again after a 61.04 usec resources assigned to P1 and P3
 *	and of TYPE2=2 are put to sleep
 *	(VINTANA1, VINTANA2, VINTDIG, VIO & CLKEN).
 *
 *	On wakeup event OMAP goes active and pulls the SYS_CLKREQ high,
 *	and will execute the S2A sequence which is same for P1_P2 & P3.
 *	This will turn on all resources of TYPE2=2 to go to the active state.
 *	Three dummy broadcast messages are added to get a delay of ~10 ms
 *	before enabling the HFCLKOUT resource. And after a 30.52 usec
 *	all resources of TYPE2=1 are put to the active state.
 *
 *	This 10ms delay can be reduced if the oscillator is having less
 *	stabilization time. A should be taken care if it needs more time
 *	for stabilization.
 *
 */

/**
 * DOC: Sleep to Active sequence for P1/P2/P3
 *
 * The wakeup sequence is adjusted to do the VDD1/VDD2 voltage ramp-up
 * only after HFCLKIN is stabilized and the HFCLKOUT is enabled.
 */
static struct twl4030_ins wakeup_seq_erratum27[] __initdata = {
	/*
	 * Broadcast message to put res(TYPE2 = 2) to active.
	 * Wait for ~10 mS (ramp-up time for OSC on the board)
	 * after HFCLKIN is enabled
	 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 55},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 55},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 54},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 1},
	/* Singular message to enable HCLKOUT after HFCLKIN is stabilized */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_HFCLKOUT, RES_STATE_ACTIVE), 1},
	/*
	 * Broadcast message to put res(TYPE2 = 1) to active.
	 * VDD1/VDD2 ramp-up after HFCLKIN is stable and HFCLKOUT is enabled.
	 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_script_erratum27 __initdata = {
	.script	= wakeup_seq_erratum27,
	.size	= ARRAY_SIZE(wakeup_seq_erratum27),
	.flags	= TWL4030_WAKEUP12_SCRIPT | TWL4030_WAKEUP3_SCRIPT,
};

/**
 * DOC: Active to Sleep sequence for P1/P2/P3
 *
 * The sleep sequence is adjusted to do the switching of VDD1/VDD2/VIO OSC from
 * HFCLKIN to internal oscillator when the HFCLKIN is stable.
 */
static struct twl4030_ins sleep_on_seq_erratum27[] __initdata = {
	/*
	 * Singular message to disable HCLKOUT.
	 * Wait for ~488.32 uS to do the switching of VDD1/VDD2/VIO OSC from
	 * HFCLKIN to internal oscillator before disabling HFCLKIN.
	 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_HFCLKOUT, RES_STATE_SLEEP), 20},
	/* Broadcast message to put res(TYPE2 = 1) to sleep */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_SLEEP), 2},
	/* Broadcast message to put res(TYPE2 = 2) to sleep, disable HFCLKIN */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_SLEEP), 2},
};

static struct twl4030_script sleep_on_script_erratum27 __initdata = {
	.script	= sleep_on_seq_erratum27,
	.size	= ARRAY_SIZE(sleep_on_seq_erratum27),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

/* TWL4030 script for sleep, wakeup & warm_reset */
static struct twl4030_script *twl4030_scripts_erratum27[] __initdata = {
	&wakeup_script_erratum27,
	&sleep_on_script_erratum27,
	&wrst_script,
};

/**
 * DOC: TWL4030 resource configuration
 *
 * VDD1/VDD2/VPLL are assigned to P1 and P3, to have better control
 * during OFFMODE. HFCLKOUT is assigned to P1 and P3 (*p2) to turn off
 * only during OFFMODE.
 * (*P2 is included if the platform uses it for modem/some other processor)
 */
static struct twl4030_resconfig twl4030_rconfig_erratum27[] __initdata = {
	{ .resource = RES_VPLL1, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
		.type = 3, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VINTANA1, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VINTANA2, .devgroup = DEV_GRP_ALL, .type = 0,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VINTDIG, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VIO, .devgroup = DEV_GRP_ALL, .type = 2,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VDD1, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
		.type = 4, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VDD2, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
		.type = 3, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_REGEN, .devgroup = DEV_GRP_ALL, .type = 2,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_NRES_PWRON, .devgroup = DEV_GRP_ALL, .type = 0,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_CLKEN, .devgroup = DEV_GRP_ALL, .type = 3,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_SYSEN, .devgroup = DEV_GRP_ALL, .type = 6,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
		.type = 0, .type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ 0, 0},
};

/**
 * twl5030_script_erratum27() - API to modify TWL4030 script
 *
 * Updating the TWL4030 script & resource configuration
 */
static void __init twl5030_script_erratum27(void)
{
	twl4030_generic_script.scripts = twl4030_scripts_erratum27;
	twl4030_generic_script.num = ARRAY_SIZE(twl4030_scripts_erratum27);
	twl4030_generic_script.resource_config = twl4030_rconfig_erratum27;
}

struct twl4030_power_data twl4030_generic_script __initdata = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
	.twl5030_erratum27wa_script = twl5030_script_erratum27,
};

static int __init twl4030_script_probe(struct platform_device *pdev)
{
	return twl4030_power_init(&twl4030_generic_script);
}

static int twl4030_script_remove(struct platform_device *pdev)
{
	return twl4030_remove_script(TWL4030_SLEEP_SCRIPT |
			TWL4030_WAKEUP12_SCRIPT | TWL4030_WAKEUP3_SCRIPT |
			TWL4030_WRST_SCRIPT);
}

static struct platform_driver twl4030_script_driver = {
	.remove	= twl4030_script_remove,
	.driver	= {
			.name = "twl4030_script",
			.owner = THIS_MODULE,
		},
};

static int __init twl4030_script_init(void)
{
	/* Register the TWL4030 script driver */
	return platform_driver_probe(&twl4030_script_driver,
					twl4030_script_probe);
}

static void __exit twl4030_script_cleanup(void)
{
	/* Unregister TWL4030 script driver */
	platform_driver_unregister(&twl4030_script_driver);
}

/*
module_init(twl4030_script_init);
module_exit(twl4030_script_cleanup);
*/

MODULE_DESCRIPTION("OMAP TWL4030 script driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments Inc");
