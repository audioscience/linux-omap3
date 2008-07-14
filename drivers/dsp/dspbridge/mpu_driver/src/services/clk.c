/*
 * dspbridge/src/services/clk.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 *  ======== clk.c ========
 *  Purpose:
 *      Clock and Timer services.
 *
 *  Public Functions:
 *      CLK_Exit
 *      CLK_Init
 *	CLK_Enable
 *	CLK_Disable
 *	CLK_GetRate
 *	CLK_Set_32KHz
 *! Revision History:
 *! ================
 *! 08-May-2007 rg: moved all clock functions from sync module.
 *		    And added CLK_Set_32KHz, CLK_Set_SysClk.
 */

/*  ----------------------------------- Host OS */
#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <csl.h>
#include <mem.h>

/*  ----------------------------------- This */
#include <clk.h>
#include <util.h>


/*  ----------------------------------- Defines, Data Structures, Typedefs */

typedef volatile unsigned long  REG_UWORD32;
typedef unsigned long  	     UWORD32;

#define CM_AUTOIDLE1_CORE 0x48004A30
#define CM_AUTOIDLE1_SIZE  0x20
#define CM_AUTOIDLE2_CORE 0x48004A34
#define CM_AUTOIDLE2_SIZE  0x20

#define SYSCONFIG_SIZE 0x1000
#define IVA2_SYSC_BASE 		0x48002000 /*0x01C20000*/
#define IVA2_WUGEN_BSAE 	0x01C21000
#define IVA2_IVLCD_BASE 	0x00080000
#define IVA2_SEQ_BASE 		0x00090000
#define IVA2_VIDEO_BASE 	0x0009C000
#define IVA2_IME_BASE 		0x000A0000
#define IVA2_ILF_BASE 		0x000A1000

#define CM_IDLEST_PER 		0x48005020

#define MAILBOX_Base 0x48094000
#define GPT_Timer1_Base 0x48318000
#define GPT_Timer2_Base 0x49032000
#define GPT_Timer5_Base 0x49038000
#define GPT_Timer6_Base 0x4903A000
#define GPT_Timer7_Base  0x4903C000
#define GPT_Timer8_Base 0x4903E000

#define McBSP1_Base 0x48074000
#define McBSP2_Base 0x49022000
#define McBSP3_Base 0x49024000
#define McBSP4_Base 0x49026000
#define McBSP5_Base 0x48096000

#define WDT3_Base 0x49030000
#define GRPSEL_Base 0x48307000
#define PER_PRM_Base 0x48307000
#define SSI_Base        0x48058000

#define MCBSP1_BASE	       IO_ADDRESS(McBSP1_Base)
#define MCBSP2_BASE	       IO_ADDRESS(McBSP2_Base)
#define MCBSP3_BASE	       IO_ADDRESS(McBSP3_Base)
#define MCBSP4_BASE	       IO_ADDRESS(McBSP4_Base)
#define MCBSP5_BASE	       IO_ADDRESS(McBSP5_Base)
#define GPT1_BASE 			IO_ADDRESS(GPT_Timer1_Base)
#define GPT2_BASE 			IO_ADDRESS(GPT_Timer2_Base)
#define GPT5_BASE 			IO_ADDRESS(GPT_Timer5_Base)
#define GPT6_BASE 			IO_ADDRESS(GPT_Timer6_Base)
#define GPT7_BASE 			IO_ADDRESS(GPT_Timer7_Base)
#define GPT8_BASE 			IO_ADDRESS(GPT_Timer8_Base)
#define WDT3_BASE 			IO_ADDRESS(WDT3_Base)
#define MBX_BASE			IO_ADDRESS(MAILBOX_Base)
#define IVA2_BASE	   		IO_ADDRESS(IVA2_SYSC_BASE)
#define GRPSEL_BASE 		IO_ADDRESS(GRPSEL_Base)
#define PER_PRM_BASE		IO_ADDRESS(PER_PRM_Base)
#define SSI_BASE                     IO_ADDRESS(SSI_Base)

#define GRP_MPU_MASK		0x3effe
#define GRP_IVA_MASK		0x00001


#define LEVEL1  0
#define LEVEL2  1

struct SERVICES_Clk_t {
	struct clk *clk_handle;
	const char *clk_name;
};

DWORD autoIdle1_core;
DWORD autoIdle2_core;

/* The row order of the below array needs to match with the clock enumerations
 * 'SERVICES_ClkId' provided in the header file.. any changes in the
 * enumerations needs to be fixed in the array as well
 */
static struct SERVICES_Clk_t SERVICES_Clks[] = {
	{NULL, "iva2_ck"},
	{NULL, "mailboxes_ick"},
	{NULL, "gpt5_fck"},
	{NULL, "gpt5_ick"},
	{NULL, "gpt6_fck"},
	{NULL, "gpt6_ick"},
	{NULL, "gpt7_fck"},
	{NULL, "gpt7_ick"},
	{NULL, "gpt8_fck"},
	{NULL, "gpt8_ick"},
	{NULL, "wdt_fck"},
	{NULL, "wdt_ick"},
	{NULL, "mcbsp1_fck"},
	{NULL, "mcbsp1_ick"},
	{NULL, "mcbsp2_fck"},
	{NULL, "mcbsp2_ick"},
	{NULL, "mcbsp3_fck"},
	{NULL, "mcbsp3_ick"},
	{NULL, "mcbsp4_fck"},
	{NULL, "mcbsp4_ick"},
	{NULL, "mcbsp5_fck"},
	{NULL, "mcbsp5_ick"},
	{NULL, "ssi_ssr_sst_fck"},
	{NULL, "ssi_ick"},
	{NULL, "omap_32k_fck"},
	{NULL, "sys_ck"},
	{NULL, ""}
};


#ifndef DISABLE_BRIDGE_PM
/*extern struct device dspbridge_device;*/
extern struct platform_device dspbridge_device;
#endif



/* Generic TIMER object: */
struct TIMER_OBJECT {
	/* DWORD	   dwSignature;*/    /* Used for object validation. */
	struct timer_list timer;
};

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask CLK_debugMask = { 0, 0 };	/* GT trace variable */
#endif

struct GPT6_configuration {
	UWORD32 config;
	UWORD32 tier;
	UWORD32 twer;
	UWORD32 tclr;
	UWORD32 tldr;
	UWORD32 ttgr;
};
/* Shadow settings to save the GPT6 timer settings */
struct GPT6_configuration  gpt6_config;

/*
 *  ======== CLK_Exit ========
 *  Purpose:
 *      Cleanup CLK module.
 */
VOID CLK_Exit(void)
{
	int i = 0;

	GT_0trace(CLK_debugMask, GT_5CLASS, "CLK_Exit\n");
	/* Relinquish the clock handles */
	while (i < SERVICESCLK_NOT_DEFINED) {
		if (SERVICES_Clks[i].clk_handle)
			clk_put(SERVICES_Clks[i].clk_handle);

		SERVICES_Clks[i].clk_handle = NULL;
		i++;
	}

}

/*
 *  ======== CLK_Init ========
 *  Purpose:
 *      Initialize CLK module.
 */
BOOL CLK_Init(void)
{
	struct clk *clk_handle;
	int i = 0;
	GT_create(&CLK_debugMask, "CK");	/* CK for CLK */
	GT_0trace(CLK_debugMask, GT_5CLASS, "CLK_Init\n");

	/* Get the clock handles from base port and store locally */
	while (i < SERVICESCLK_NOT_DEFINED) {
		/* get the handle from BP */
#ifndef DISABLE_BRIDGE_PM
		clk_handle = clk_get(&dspbridge_device.dev,
			     SERVICES_Clks[i].clk_name);
#else
		clk_handle = clk_get(NULL, SERVICES_Clks[i].clk_name);
#endif

		if (!clk_handle) {
			GT_1trace(CLK_debugMask, GT_7CLASS,
				  "CLK_Init: failed to get Clk "
				  "handle %s \n", SERVICES_Clks[i].clk_name);
			/* should we fail here?? */
		} else {
			GT_1trace(CLK_debugMask, GT_7CLASS,
				  "CLK_Init: PASS and Clk handle"
				  "%s \n", SERVICES_Clks[i].clk_name);
		}
		SERVICES_Clks[i].clk_handle = clk_handle;
		i++;
	}

	return TRUE;
}

/*
 *  ======== CLK_Enable ========
 *  Purpose:
 *      Enable Clock .
 *
*/
DSP_STATUS CLK_Enable(IN SERVICES_ClkId clk_id)
{
	DSP_STATUS status = DSP_SOK;
	struct clk *pClk;

	DBC_Require(clk_id < SERVICESCLK_NOT_DEFINED);
	GT_1trace(CLK_debugMask, GT_6CLASS,
		  "CLK_Enable: CLK Id = 0x%x \n", clk_id);

	pClk = SERVICES_Clks[clk_id].clk_handle;
	if (pClk) {
		if (clk_enable(pClk) == 0x0) {
			/* Success ? */
		} else {
			GT_1trace(CLK_debugMask, GT_7CLASS,
				  "CLK_Enable: failed to Enable "
				  "CLK %s \n", SERVICES_Clks[clk_id].clk_name);
			status = DSP_EFAIL;
		}
	} else {
		GT_1trace(CLK_debugMask, GT_7CLASS,
			  "CLK_Enable: failed to get "
			  "CLK %s \n", SERVICES_Clks[clk_id].clk_name);
		status = DSP_EFAIL;
	}
	/* The SSI module need to configured not to have the Forced idle for
	 * master interface. If it is set to forced idle, the SSI module is
	 * transitioning to standby thereby causing the client in the DSP hang
	 * waiting for the SSI module to be active after enabling the clocks
	 */
	if (clk_id == SERVICESCLK_ssi_fck)
		SSI_Clk_Prepare(TRUE);

	return status;
}
/*
 *  ======== CLK_Set_32KHz ========
 *  Purpose:
 *      To Set parent of a clock to 32KHz.
 */

DSP_STATUS CLK_Set_32KHz(IN SERVICES_ClkId clk_id)
{
	DSP_STATUS status = DSP_SOK;
	struct clk *pClk;
	struct   clk *pClkParent;
	SERVICES_ClkId sys_32k_id = SERVICESCLK_sys_32k_ck;
	pClkParent =  SERVICES_Clks[sys_32k_id].clk_handle;

	DBC_Require(clk_id < SERVICESCLK_NOT_DEFINED);
	GT_1trace(CLK_debugMask, GT_6CLASS, "CLK_Set_32KHz: CLK Id  = 0x%x is "
		  "setting to 32KHz \n", clk_id);
	pClk = SERVICES_Clks[clk_id].clk_handle;
	if (pClk) {
		if (clk_set_parent(pClk, pClkParent) == 0x0) {
		} else {
			GT_1trace(CLK_debugMask, GT_7CLASS,
				  "CLK_Set_32KHz: Failed to "
				  "set to 32KHz %s \n",
				  SERVICES_Clks[clk_id].clk_name);
			status = DSP_EFAIL;
		}
	}
	return status;
}

/*
 *  ======== CLK_Disable ========
 *  Purpose:
 *      Disable the clock.
 *
*/
DSP_STATUS CLK_Disable(IN SERVICES_ClkId clk_id)
{
	DSP_STATUS status = DSP_SOK;
	struct clk *pClk;
	DBC_Require(clk_id < SERVICESCLK_NOT_DEFINED);
	INT clkUseCnt;

	GT_1trace(CLK_debugMask, GT_6CLASS,
		  "CLK_Disable: CLK Id = 0x%x \n", clk_id);

	pClk = SERVICES_Clks[clk_id].clk_handle;

	clkUseCnt = CLK_Get_UseCnt(clk_id);
	if (clkUseCnt == -1) {
		GT_1trace(CLK_debugMask, GT_7CLASS,
			  "CLK_Disable: failed to get "
			  "CLK Use count for Clk %s \n",
			  SERVICES_Clks[clk_id].clk_name);
	} else if (clkUseCnt == 0) {
		GT_1trace(CLK_debugMask, GT_7CLASS,
			  "CLK_Disable:  Clk %s  is already"
			  "disabled\n", SERVICES_Clks[clk_id].clk_name);
		 return status;
	}
	if (clk_id == SERVICESCLK_ssi_ick)
		SSI_Clk_Prepare(FALSE);

		if (pClk) {
			clk_disable(pClk);
		} else {
			GT_1trace(CLK_debugMask, GT_7CLASS,
				  "CLK_Disable: failed to get "
				  "CLK %s \n", SERVICES_Clks[clk_id].clk_name);
			status = DSP_EFAIL;
		}
	return status;
}

/*
 *  ======== CLK_GetRate ========
 *  Purpose:
 *      GetClock Speed.
 *
 */

DSP_STATUS CLK_GetRate(IN SERVICES_ClkId clk_id, ULONG *speedKhz)
{
	DSP_STATUS status = DSP_SOK;
	struct clk *pClk;
	ULONG clkSpeedHz;

	DBC_Require(clk_id < SERVICESCLK_NOT_DEFINED);
	*speedKhz = 0x0;

	GT_1trace(CLK_debugMask, GT_7CLASS,
		  "CLK_GetRate: CLK Id = 0x%x \n", clk_id);
	pClk = SERVICES_Clks[clk_id].clk_handle;
	if (pClk) {
		clkSpeedHz = clk_get_rate(pClk);
		*speedKhz = clkSpeedHz / 1000;
		GT_2trace(CLK_debugMask, GT_6CLASS,
			  "CLK_GetRate: clkSpeedHz = %d , "
			  "speedinKhz=%d \n", clkSpeedHz, *speedKhz);
	} else {
		GT_1trace(CLK_debugMask, GT_7CLASS,
			  "CLK_GetRate: failed to get CLK %s\n",
			  SERVICES_Clks[clk_id].clk_name);
		status = DSP_EFAIL;
	}
	return status;
}

INT CLK_Get_UseCnt(IN SERVICES_ClkId clk_id)
{
	DSP_STATUS status = DSP_SOK;
	struct clk *pClk;
	INT useCount = -1;
	DBC_Require(clk_id < SERVICESCLK_NOT_DEFINED);

	pClk = SERVICES_Clks[clk_id].clk_handle;

	if (pClk) {
		useCount =  clk_get_usecount(pClk);
	} else {
		GT_1trace(CLK_debugMask, GT_7CLASS,
			  "CLK_GetRate: failed to get "
			  "CLK %s \n", SERVICES_Clks[clk_id].clk_name);
		status = DSP_EFAIL;
	}
	return useCount;
}

void SSI_Clk_Prepare(BOOL FLAG)
{
	UWORD32 ssi_sysconfig;
	ssi_sysconfig = __raw_readl((SSI_BASE) + 0x10);


	if (FLAG) {
		/* Set Autoidle, SIDLEMode to smart idle, and MIDLEmode to
		 * no idle */
		ssi_sysconfig = 0x1011;
	} else {
		/* Set Autoidle, SIDLEMode to forced idle, and MIDLEmode to
		 * forced idle*/
		ssi_sysconfig = 0x1;
	}
	__raw_writel((UWORD32)ssi_sysconfig, SSI_BASE + 0x10);
}


/*

DSP_STATUS CLK_AutoIdleCtrl(IN SERVICES_ClkId clk_id,INT cmd)
{
	DSP_STATUS status =DSP_SOK;
	INT prcm_status =0;
	INT prcm_cmd = 0;
	unsigned long temp;

	autoIdle1_core = (DWORD)ioremap(CM_AUTOIDLE1_CORE, CM_AUTOIDLE1_SIZE);
	autoIdle2_core = (DWORD)ioremap(CM_AUTOIDLE2_CORE, CM_AUTOIDLE2_SIZE);
	temp = (unsigned long) * ((volatile unsigned long  *)
		((unsigned long) (autoIdle1_core)));
    GT_1trace(SYNC_debugMask, GT_7CLASS,"CM_AUTOIDLE1_CORE = 0x%x \n",temp);
	temp = (unsigned long)
		*((volatile unsigned long *)((unsigned long)(autoIdle2_core)));
    GT_1trace(SYNC_debugMask, GT_7CLASS, "CM_AUTOIDLE2_CORE = 0x%x \n",temp);

	if(cmd == 0){
		prcm_cmd = PRCM_DISABLE;
	} else	{
		prcm_cmd = PRCM_ENABLE;
	}

	switch(clk_id) {
		case SERVICESCLK_gpt5_ick:
		prcm_status = interface_clock_autoidle(PRCM_GPT5,prcm_cmd);
		break;
		case SERVICESCLK_gpt6_ick:
		prcm_status = interface_clock_autoidle(PRCM_GPT6,prcm_cmd);
		break;
		case SERVICESCLK_gpt7_ick:
		prcm_status = interface_clock_autoidle(PRCM_GPT7,prcm_cmd);
		break;
		case SERVICESCLK_gpt8_ick:
		prcm_status = interface_clock_autoidle(PRCM_GPT8,prcm_cmd);
		break;
		case SERVICESCLK_wdt4_ick:
		prcm_status = interface_clock_autoidle(PRCM_WDT4,prcm_cmd);
		break;
		case SERVICESCLK_mcbsp1_ick:
		prcm_status = interface_clock_autoidle(PRCM_MCBSP1,prcm_cmd);
		break;
		case SERVICESCLK_mcbsp2_ick:
		prcm_status = interface_clock_autoidle(PRCM_MCBSP2,prcm_cmd);
		break;
		case SERVICESCLK_mcbsp3_ick:
		prcm_status = interface_clock_autoidle(PRCM_MCBSP3,prcm_cmd);
		break;
		case SERVICESCLK_mcbsp4_ick:
		prcm_status = interface_clock_autoidle(PRCM_MCBSP4,prcm_cmd);
		break;
		case SERVICESCLK_mcbsp5_ick:
		prcm_status = interface_clock_autoidle(PRCM_MCBSP5,prcm_cmd);
		break;
		case SERVICESCLK_ssi_ick:
		prcm_status = interface_clock_autoidle(PRCM_SSI,prcm_cmd);
		break;
		default:
		GT_1trace(SYNC_debugMask,GT_7CLASS,
			  "CLK_AutoIdle: failed for CLK %s \n",
			  SERVICES_Clks[clk_id].clk_name);
		status = DSP_EFAIL;
		break;
	}

	if (prcm_status){
		status = DSP_SOK;
	} else	{
		status = DSP_EFAIL;
	}

	temp = (unsigned long)
		*((volatile unsigned long *)((unsigned long)(autoIdle1_core)));
	GT_1trace(SYNC_debugMask, GT_7CLASS,
		  "CM_AUTOIDLE1_CORE = 0x%x \n",temp);
	temp = (unsigned long)
		*((volatile unsigned long *)((unsigned long)(autoIdle2_core)));
	GT_1trace(SYNC_debugMask, GT_7CLASS,
		  "CM_AUTOIDLE2_CORE = 0x%x \n",temp);
	return status;
}

*/
