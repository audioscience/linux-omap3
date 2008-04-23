/*
 * dspbridge/src/osal/linux/clk.c
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
 *	CLK_Set_SysClk
 *	CLK_Set_32KHz
 *	TIMER_OpenTimer
 *	TIMER_CloseTimer
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
#include <dbg_zones.h>
#include <gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <csl.h>
#include <mem.h>

/*  ----------------------------------- This */
#include <clk.h>
#include <util.h>


/*  ----------------------------------- Defines, Data Structures, Typedefs */

typedef volatile unsigned long  REG_UWORD32;
typedef volatile  long  		     REG_WORD32;
typedef unsigned long  	     UWORD32;
typedef long  			     WORD32;


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

#if 0
DWORD gpt5_base, gpt6_base, gpt7_base, gpt8_base;
DWORD mcbsp1_base, mcbsp2_base, mcbsp3_base, mcbsp4_base, mcbsp5_base;
DWORD wdt3_base;
DWORD mailbox_base;
DWORD iva2_sysc_base, iva2_wugen_base, iva2_ivlcd_base, iva2_seq_base,
      iva2_ime_base, iva2_ilf_base, iva2_video_base;
DWORD idlest_per;
#endif

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

#if defined(OMAP_2430) || defined(OMAP_3430)
struct OSAL_Clk_t {
	struct clk *clk_handle;
	const char *clk_name;
};

DWORD autoIdle1_core;
DWORD autoIdle2_core;

/* The row order of the below array needs to match with the clock enumerations
 * 'OSAL_ClkId' provided in the header file.. any changes in the enumerations
 * needs to be fixed in the array as well
 */
static struct OSAL_Clk_t OSAL_Clks[] = {
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
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
extern struct omap_dev bridge_dsp_ldm;
#else
/*extern struct device dspbridge_device;*/
extern struct platform_device dspbridge_device;
#endif


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
VOID CLK_Exit()
{
	struct clk *clk_handle;
	int i = 0;

	GT_0trace(CLK_debugMask, GT_5CLASS, "CLK_Exit\n");
	/* Relinquish the clock handles */
	while (i < OSALCLK_NOT_DEFINED) {
		if (OSAL_Clks[i].clk_handle)
			clk_put(OSAL_Clks[i].clk_handle);

		OSAL_Clks[i].clk_handle = NULL;
		i++;
	}

}

/*
 *  ======== CLK_Init ========
 *  Purpose:
 *      Initialize CLK module.
 */
BOOL CLK_Init()
{
	struct clk *clk_handle;
	int i = 0;
	GT_create(&CLK_debugMask, "CK");	/* CK for CLK */
	GT_0trace(CLK_debugMask, GT_5CLASS, "CLK_Init\n");
#if 0
	UWORD32 iva2_grp_sel;
	UWORD32 mpu_grp_sel;

	mpu_grp_sel = __raw_readl((GRPSEL_BASE) + 0xA4);
	iva2_grp_sel = __raw_readl((GRPSEL_BASE) + 0xA8);

	mpu_grp_sel &=  GRP_MPU_MASK;
	iva2_grp_sel |=  GRP_IVA_MASK;

	__raw_writel((UWORD32)mpu_grp_sel, GRPSEL_BASE + 0xA4);
	__raw_writel((UWORD32)iva2_grp_sel, GRPSEL_BASE + 0xA8);

#endif
	/* Get the clock handles from base port and store locally */
	while (i < OSALCLK_NOT_DEFINED) {
		/* get the handle from BP */
#ifndef DISABLE_BRIDGE_PM
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
		clk_handle = clk_get(&bridge_dsp_ldm.dev,
			     OSAL_Clks[i].clk_name);
#else
		clk_handle = clk_get(&dspbridge_device.dev,
			     OSAL_Clks[i].clk_name);
#endif
#else
		clk_handle = clk_get(NULL, OSAL_Clks[i].clk_name);
#endif

		if (!clk_handle) {
			GT_1trace(CLK_debugMask, GT_7CLASS,
				  "CLK_Init: failed to get Clk "
				  "handle %s \n", OSAL_Clks[i].clk_name);
			/* should we fail here?? */
		} else {
			GT_1trace(CLK_debugMask, GT_7CLASS,
				  "CLK_Init: PASS and Clk handle"
				  "%s \n", OSAL_Clks[i].clk_name);
		}
		OSAL_Clks[i].clk_handle = clk_handle;
		i++;
	}

	return (TRUE);
}

/*
 *  ======== TIMER_OpenTimer ========
 *  Purpose:
 *      Open a new timer object.
 *   TODO -- modify the function to pass the Callback function and other parms
 *
*/
DSP_STATUS TIMER_OpenTimer(OUT struct TIMER_OBJECT **phTimer)
{
	struct TIMER_OBJECT *pTimer = NULL;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(phTimer != NULL);
	GT_1trace(CLK_debugMask, GT_ENTER, "TIMER_OpenTimer: phTimer 0x%x \n",
		  phTimer);
	/* Allocate memory for sync object */
	pTimer = MEM_Calloc(sizeof(struct TIMER_OBJECT), MEM_NONPAGED);
	if (pTimer != NULL) {
		init_timer((void *)pTimer);
	} else {
		status = DSP_EMEMORY;
		GT_0trace(CLK_debugMask, GT_6CLASS,
			  "TIMER_OpenTimer: MEM_AllocObject failed\n");
	}
	*phTimer = pTimer;
	return (status);
}

/*
 *  ======== TIMER_CloseTimer ========
 *  Purpose:
 *      close the timer object.
 *   TODO -- Work in progress
 *
*/
DSP_STATUS TIMER_CloseTimer(IN struct TIMER_OBJECT *hTimer)
{
	DSP_STATUS status = DSP_SOK;
	DBC_Require(hTimer != NULL);
	GT_1trace(CLK_debugMask, GT_ENTER, "TIMER_CloseTimer: hTimer 0x%x \n",
		  hTimer);
	MEM_Free(hTimer);
	return (status);
}


/*
 *  ======== CLK_Enable ========
 *  Purpose:
 *      Enable Clock .
 *
*/
DSP_STATUS CLK_Enable(IN OSAL_ClkId clk_id)
{
	DSP_STATUS status = DSP_SOK;
	struct clk *pClk;
	UWORD32 IdleState;

	DBC_Require(clk_id < OSALCLK_NOT_DEFINED);
	GT_1trace(CLK_debugMask, GT_6CLASS,
		  "CLK_Enable: CLK Id = 0x%x \n", clk_id);

	pClk = OSAL_Clks[clk_id].clk_handle;
	if (pClk) {
		if (clk_enable(pClk) == 0x0) {
			/* Success ? */
		} else {
			GT_1trace(CLK_debugMask, GT_7CLASS,
				  "CLK_Enable: failed to Enable "
				  "CLK %s \n", OSAL_Clks[clk_id].clk_name);
			status = DSP_EFAIL;
		}
	} else {
		GT_1trace(CLK_debugMask, GT_7CLASS,
			  "CLK_Enable: failed to get "
			  "CLK %s \n", OSAL_Clks[clk_id].clk_name);
		status = DSP_EFAIL;
	}
	/* The SSI module need to configured not to have the Forced idle for master
	     interface. If it is set to forced idle, the SSI module is transitioning to
	     standby thereby causing the client in the DSP hang waiting for the SSI
	     module to be active after enabling the clocks
	 */
	if (clk_id == OSALCLK_ssi_fck)
		SSI_Clk_Prepare(TRUE);

#if 0 /* The SYSCONFIG Register settings are taken care from DSP side now*/
		switch (clk_id) {
		case OSALCLK_iva2_ck:
			IVA_SYS_Prepare();
			break;
		case OSALCLK_mailbox_ick:
			MAILBOX_Clk_Prepare();
			break;
		case OSALCLK_gpt5_fck:
			GPT5_Clk_Prepare(LEVEL2);
			break;
		case OSALCLK_gpt6_fck:
			GPT6_Clk_Prepare(LEVEL2);
			break;
		case OSALCLK_gpt7_fck:
			GPT7_Clk_Prepare(LEVEL2);
			break;
		case OSALCLK_gpt8_fck:
			GPT8_Clk_Prepare(LEVEL2);
			break;
#if 0

		case OSALCLK_wdt3_fck:
			WDT3_Clk_Prepare(LEVEL2);
			break;
#endif
		case OSALCLK_mcbsp1_fck:
			McBSP1_Clk_Prepare(LEVEL2);
			break;
		case OSALCLK_mcbsp2_fck:
			McBSP2_Clk_Prepare(LEVEL2);
			break;
		case OSALCLK_mcbsp3_fck:
			McBSP3_Clk_Prepare(LEVEL2);
			break;
		case OSALCLK_mcbsp4_fck:
			McBSP4_Clk_Prepare(LEVEL2);
			break;
		case OSALCLK_mcbsp5_fck:
			McBSP5_Clk_Prepare(LEVEL2);
			break;
		case OSALCLK_ssi_ick:
		case OSALCLK_sys_32k_ck:
		case OSALCLK_sys_ck:
		default:
			break;
		}
#endif
	return (status);
}
/*
 *  ======== CLK_Set_32KHz ========
 *  Purpose:
 *      To Set parent of a clock to 32KHz.
 */

DSP_STATUS CLK_Set_32KHz(IN OSAL_ClkId clk_id)
{
	DSP_STATUS status = DSP_SOK;
	struct clk *pClk;
	struct   clk *pClkParent;
	OSAL_ClkId sys_32k_id = OSALCLK_sys_32k_ck;
	pClkParent =  OSAL_Clks[sys_32k_id].clk_handle;

	DBC_Require(clk_id < OSALCLK_NOT_DEFINED);
	GT_1trace(CLK_debugMask, GT_6CLASS, "CLK_Set_32KHz: CLK Id  = 0x%x is "
		  "setting to 32KHz \n", clk_id);
	pClk = OSAL_Clks[clk_id].clk_handle;
	if (pClk) {
		if (clk_set_parent(pClk, pClkParent) == 0x0) {
		} else {
			GT_1trace(CLK_debugMask, GT_7CLASS,
				  "CLK_Set_32KHz: Failed to "
				  "set to 32KHz %s \n",
				  OSAL_Clks[clk_id].clk_name);
			status = DSP_EFAIL;
		}
	}
	return(status);
}

/*
 *  ======== CLK_Set_SysClk ========
 *  Purpose:
 *      To Set parent of a clock to System Clock.
 */

DSP_STATUS CLK_Set_SysClk(IN OSAL_ClkId clk_id)
{
	DSP_STATUS status = DSP_SOK;
	struct clk *pClk;
	struct   clk *pClkParent;
	OSAL_ClkId sys_ck_id = OSALCLK_sys_ck;
	pClkParent =  OSAL_Clks[sys_ck_id].clk_handle;

	DBC_Require(clk_id < OSALCLK_NOT_DEFINED);
	GT_1trace(CLK_debugMask, GT_6CLASS, "CLK_Set_32KHz: CLK Id  = 0x%x is "
		  "setting to 32KHz \n", clk_id);
	pClk = OSAL_Clks[clk_id].clk_handle;
	if (pClk) {
		if (clk_set_parent(pClk, pClkParent) == 0x0) {
		} else {
			GT_1trace(CLK_debugMask, GT_7CLASS,
				  "CLK_Set_32KHz: Failed to "
				  "set to System Clock %s \n",
				  OSAL_Clks[clk_id].clk_name);
			status = DSP_EFAIL;
		}
	}
	return(status);
}

/*
 *  ======== CLK_Disable ========
 *  Purpose:
 *      Disable the clock.
 *
*/
DSP_STATUS CLK_Disable(IN OSAL_ClkId clk_id)
{
	DSP_STATUS status = DSP_SOK;
	struct clk *pClk;
	DBC_Require(clk_id < OSALCLK_NOT_DEFINED);
	INT clkUseCnt;

	GT_1trace(CLK_debugMask, GT_6CLASS,
		  "CLK_Disable: CLK Id = 0x%x \n", clk_id);

	pClk = OSAL_Clks[clk_id].clk_handle;

	clkUseCnt = CLK_Get_UseCnt(clk_id);
	if (clkUseCnt == -1) {
		GT_1trace(CLK_debugMask, GT_7CLASS,
			  "CLK_Disable: failed to get "
			  "CLK Use count for Clk %s \n",
			  OSAL_Clks[clk_id].clk_name);
	} else if (clkUseCnt == 0) {
		GT_1trace(CLK_debugMask, GT_7CLASS,
			  "CLK_Disable:  Clk %s  is already"
			  "disabled\n", OSAL_Clks[clk_id].clk_name);
		 return status;
	}
	if (clk_id == OSALCLK_ssi_ick)
		SSI_Clk_Prepare(FALSE);

#if 0 /* The SYSCONFIG register settings are taken care from DSP side now*/
		switch (clk_id) {
		case OSALCLK_gpt5_ick:
			GPT5_Clk_Prepare(LEVEL1);
			break;
		case OSALCLK_gpt6_ick:
			GPT6_Clk_Prepare(LEVEL1);
			break;
		case OSALCLK_gpt7_ick:
			GPT7_Clk_Prepare(LEVEL1);
			break;
		case OSALCLK_gpt8_ick:
			GPT8_Clk_Prepare(LEVEL1);
			break;
#if 0
		case OSALCLK_wdt3_ick:
			WDT3_Clk_Prepare(LEVEL1);
			break;
#endif
		case OSALCLK_mcbsp1_ick:
			McBSP1_Clk_Prepare(LEVEL1);
			break;
		case OSALCLK_mcbsp2_ick:
			McBSP2_Clk_Prepare(LEVEL1);
			break;
		case OSALCLK_mcbsp3_ick:
			McBSP3_Clk_Prepare(LEVEL1);
			break;
		case OSALCLK_mcbsp4_ick:
			McBSP4_Clk_Prepare(LEVEL1);
			break;
		case OSALCLK_mcbsp5_ick:
			McBSP5_Clk_Prepare(LEVEL1);
			break;
		case OSALCLK_ssi_ick:
		case OSALCLK_sys_32k_ck:
		case OSALCLK_sys_ck:
		case OSALCLK_iva2_ck:
		case OSALCLK_mailbox_ick:
		default:
			break;
		}
#endif
		if (pClk) {
			clk_disable(pClk);
		} else {
			GT_1trace(CLK_debugMask, GT_7CLASS,
				  "CLK_Disable: failed to get "
				  "CLK %s \n", OSAL_Clks[clk_id].clk_name);
			status = DSP_EFAIL;
		}
	return (status);
}

/*
 *  ======== CLK_GetRate ========
 *  Purpose:
 *      GetClock Speed.
 *
 */

DSP_STATUS CLK_GetRate(IN OSAL_ClkId clk_id, ULONG *speedKhz)
{
	DSP_STATUS status = DSP_SOK;
	struct clk *pClk;
	ULONG clkSpeedHz;

	DBC_Require(clk_id < OSALCLK_NOT_DEFINED);
	*speedKhz = 0x0;

	GT_1trace(CLK_debugMask, GT_7CLASS,
		  "CLK_GetRate: CLK Id = 0x%x \n", clk_id);
	pClk = OSAL_Clks[clk_id].clk_handle;
	if (pClk) {
		clkSpeedHz = clk_get_rate(pClk);
		*speedKhz = clkSpeedHz / 1000;
		GT_2trace(CLK_debugMask, GT_6CLASS,
			  "CLK_GetRate: clkSpeedHz = %d , "
			  "speedinKhz=%d \n", clkSpeedHz, *speedKhz);
	} else {
		GT_1trace(CLK_debugMask, GT_7CLASS,
			  "CLK_GetRate: failed to get CLK %s\n",
			  OSAL_Clks[clk_id].clk_name);
		status = DSP_EFAIL;
	}
	return (status);
}

void IVA_SYS_Prepare()
{
	UWORD32 iva2_offset;

	iva2_offset = __raw_readl((IVA2_BASE) + 0x10);

	/*iva2_offset  = (UWORD32) *((REG_UWORD32 *)
	 * ((UWORD32)(iva2_sysc_bas)+0x08));*/
	iva2_offset  = (iva2_offset & 0xfffffffe);
	iva2_offset |= 0x00000001;
	__raw_writel((UWORD32)iva2_offset, IVA2_BASE+0x10);
	/* *((REG_UWORD32*) ((UWORD32) (iva2_sysc_base ) + 0x08)) =
	 * (UWORD32) iva2_offset; */

#if 0
	iva2_offset  = (UWORD32) * ((REG_UWORD32 *) ((UWORD32) (iva2_wugen_base)
			+ 0x08));
	iva2_offset  &= 0xfffffffe;
	iva2_offset |= 0x00000001;
	*((REG_UWORD32 *) ((UWORD32) (iva2_wugen_base) + 0x08)) =
							(UWORD32) iva2_offset;

	iva2_offset  = (UWORD32) * ((REG_UWORD32 *) ((UWORD32) (iva2_ivlcd_base)
			+ 0x10));
	iva2_offset  &= 0xfffffffe;
	iva2_offset |= 0x00000001;
	*((REG_UWORD32 *) ((UWORD32) (iva2_ivlcd_base) + 0x10)) = (UWORD32)
								iva2_offset;

	iva2_offset  = (UWORD32) * ((REG_UWORD32 *) ((UWORD32) (iva2_seq_base)
			+ 0x08));
	iva2_offset  &= 0xfffffffe;
	iva2_offset |= 0x00000001;
	*((REG_UWORD32 *) ((UWORD32) (iva2_seq_base) + 0x08))
						= (UWORD32) iva2_offset;
	iva2_offset  = (UWORD32) * ((REG_UWORD32 *) ((UWORD32) (iva2_video_base)
			+ 0x08));
	iva2_offset  &= 0xfffffffe;
	iva2_offset |= 0x00000001;
	*((REG_UWORD32 *) ((UWORD32) (iva2_video_base) + 0x08))
						= (UWORD32) iva2_offset;
	iva2_offset  = (UWORD32) * ((REG_UWORD32 *) ((UWORD32) (iva2_ime_base)
			+ 0x10));
	iva2_offset  &= 0xfffffffe;
	iva2_offset |= 0x00000001;
	*((REG_UWORD32 *) ((UWORD32) (iva2_ime_base) + 0x10))
						= (UWORD32) iva2_offset;
	iva2_offset  = (UWORD32) * ((REG_UWORD32 *) ((UWORD32) (iva2_ilf_base)
			+ 0x10));
	iva2_offset  &= 0xfffffffe;
	iva2_offset |= 0x00000001;
	*((REG_UWORD32 *) ((UWORD32) (iva2_ilf_base) + 0x10))
						= (UWORD32) iva2_offset;
#endif
}


void MAILBOX_Clk_Prepare()
{
	UWORD32 mailbox_sys;
	mailbox_sys = __raw_readl((MBX_BASE) + 0x10);
	mailbox_sys = mailbox_sys & (0xFFFFFFEE);
	mailbox_sys = mailbox_sys | (0x00000011);
	__raw_writel((UWORD32)mailbox_sys, MBX_BASE+0x10);
	/* *((REG_UWORD32*) ((UWORD32) (mailbox_base ) + 0x10))  =
	  (UWORD32) mailbox_sys; */
}

void GPT5_Clk_Prepare(BOOL FLAG)
{
	UWORD32 gpt5_sys;
	gpt5_sys = __raw_readl((GPT5_BASE) + 0x10);
	if (FLAG) {
		gpt5_sys &= 0xfffffdea;
		gpt5_sys |= 0x00000215;
	} else {
		gpt5_sys &= 0xffffffea;
		gpt5_sys |= 0x00000015;
	}
	__raw_writel((UWORD32)gpt5_sys, GPT5_BASE + 0x10);
	/* *((REG_UWORD32*)((UWORD32) (gpt5_base ) + 0x10))
	 * = (UWORD32) gpt5_sys; */
}

void GPT6_Clk_Prepare(BOOL FLAG)
{
	UWORD32 gpt6_sys, per_wkup;
	if (FLAG) {
		/* Restore the GPT6 setings  */
		__raw_writel((UWORD32)gpt6_config.config, GPT6_BASE + 0x10);
		__raw_writel((UWORD32)gpt6_config.tier, GPT6_BASE + 0x1c);
		__raw_writel((UWORD32)gpt6_config.twer, GPT6_BASE + 0x20);
		/* Set the setting to start the GPT6 timer */
		gpt6_config.tclr = gpt6_config.tclr | 0x1;
		__raw_writel((UWORD32)gpt6_config.tclr, GPT6_BASE + 0x24);
		__raw_writel((UWORD32)gpt6_config.tldr, GPT6_BASE + 0x2c);
		__raw_writel((UWORD32)gpt6_config.ttgr, GPT6_BASE + 0x30);

	} else {
		/* Save the GPT6 configuration before disabling the timer.
		    We are saving this settings because the disabling of GPT6
		    leads to core off and thus loss of GPT6 settings
		 */
		gpt6_config.config = __raw_readl((GPT6_BASE) + 0x10);
		gpt6_config.tier = __raw_readl((GPT6_BASE) + 0x1c);
		gpt6_config.twer = __raw_readl((GPT6_BASE) + 0x20);
		gpt6_config.tclr = __raw_readl((GPT6_BASE) + 0x24);
		gpt6_config.tldr = __raw_readl((GPT6_BASE) + 0x2c);
		gpt6_config.ttgr = __raw_readl((GPT6_BASE) + 0x30);
		per_wkup = __raw_readl((PER_PRM_BASE)+0xB0);
		per_wkup |= 0x000000080;
		/* Set the sys config to force idle before disabling the clock.
		    This had to done to make sure the peripheral domain
		    transitions to off mode. Without this, some issues were
		    noticed.
		 */
		gpt6_sys = 0x00000105;
		__raw_writel((UWORD32)gpt6_sys, GPT6_BASE + 0x10);
		__raw_writel((UWORD32)per_wkup, PER_PRM_BASE + 0xB0);
	}
}

void GPT7_Clk_Prepare(BOOL FLAG)
{
	UWORD32 gpt7_sys;


	gpt7_sys = __raw_readl((GPT7_BASE) + 0x10);
	if (FLAG) {
		gpt7_sys &= 0xfffffdea;
		gpt7_sys |= 0x00000215;
	} else {
		gpt7_sys &= 0xffffffea;
		gpt7_sys |= 0x00000015;

	}
	__raw_writel((UWORD32)gpt7_sys, GPT7_BASE + 0x10);
	/* *((REG_UWORD32*) ((UWORD32) (gpt7_base ) + 0x10))  =
	 * (UWORD32) gpt7_sys;*/
}

void GPT8_Clk_Prepare(BOOL FLAG)
{
	UWORD32 gpt8_sys;

	gpt8_sys = __raw_readl((GPT8_BASE) + 0x10);
	if (FLAG) {
		gpt8_sys &= 0xfffffdea;
		gpt8_sys |= 0x00000215;
	} else {
		gpt8_sys &= 0xffffffea;
		gpt8_sys |= 0x00000015;
	}
	__raw_writel((UWORD32)gpt8_sys, GPT8_BASE + 0x10);

	/*  *((REG_UWORD32*) ((UWORD32) (gpt8_base ) + 0x10))  =
	 *  (UWORD32) gpt8_sys; */
}

void WDT3_Clk_Prepare(BOOL FLAG)
{
	UWORD32  wdt3_sys;

	wdt3_sys =  __raw_readl((WDT3_BASE) + 0x10);

	if (FLAG) {
		wdt3_sys &= 0xfffffdea;
		wdt3_sys |= 0x00000215;
	} else {
		wdt3_sys &= 0xffffffea;
		wdt3_sys |= 0x00000015;
	}
	__raw_writel((UWORD32)wdt3_sys, WDT3_BASE + 0x10);

	/* *((REG_UWORD32*) ((UWORD32) (wdt3_base ) + 0x10))
	 * = (UWORD32) wdt3_sys; */
}

void McBSP1_Clk_Prepare(BOOL FLAG)
{

	UWORD32 mcbsp1_sys, mcbsp1_idle;

	mcbsp1_sys = __raw_readl((MCBSP1_BASE) + 0x8c);
	mcbsp1_idle = __raw_readl((MCBSP1_BASE) + 0x48);

	if (FLAG) {
		mcbsp1_sys &= 0xfffffdeb;
		mcbsp1_sys |= 0x00000114;
		mcbsp1_idle &= 0xffffbfff;
		mcbsp1_idle |= 0x00004000;
	} else {
		mcbsp1_sys &= 0xffffffeb;
		mcbsp1_sys |= 0x00000014;
		mcbsp1_idle &= 0xffffbfff;
		mcbsp1_idle |= 0x00004000;
	}
	__raw_writel((UWORD32)mcbsp1_sys, MCBSP1_BASE + 0x8c);
	__raw_writel((UWORD32)mcbsp1_idle, MCBSP1_BASE + 0x48);
}

void McBSP2_Clk_Prepare(BOOL FLAG)
{
	UWORD32  mcbsp2_sys, mcbsp2_idle, mcbsp2_st_sys;
	mcbsp2_sys = __raw_readl((MCBSP2_BASE) + 0x8c);
	mcbsp2_idle = __raw_readl((MCBSP2_BASE) + 0x48);
	mcbsp2_st_sys = __raw_readl((MCBSP2_BASE) + 0x10);

	if (FLAG) {
		mcbsp2_sys &= 0xfffffdeb;
		mcbsp2_sys |= 0x00000114;
		mcbsp2_idle &= 0xffffbfff;
		mcbsp2_idle |= 0x00004000;
		mcbsp2_st_sys |= 0x1;
	} else {
		mcbsp2_sys &= 0xffffffeb;
		mcbsp2_sys |= 0x00000014;
		mcbsp2_idle &= 0xffffbfff;
		mcbsp2_idle |= 0x00004000;
		mcbsp2_st_sys |= 0x1;
	}
	__raw_writel((UWORD32)mcbsp2_sys, MCBSP2_BASE + 0x8c);
	__raw_writel((UWORD32)mcbsp2_idle, MCBSP2_BASE + 0x48);
	__raw_writel((UWORD32)mcbsp2_st_sys, MCBSP2_BASE + 0x10);
}


void McBSP3_Clk_Prepare(BOOL FLAG)
{
	UWORD32 mcbsp3_sys, mcbsp3_idle, mcbsp3_st_sys;

	mcbsp3_sys = __raw_readl((MCBSP3_BASE) + 0x8c);
	mcbsp3_idle = __raw_readl((MCBSP3_BASE) + 0x48);
	mcbsp3_st_sys = __raw_readl((MCBSP3_BASE) + 0x10);


	if (FLAG) {
		mcbsp3_sys &= 0xfffffdeb;
		mcbsp3_sys |= 0x00000114;
		mcbsp3_idle &= 0xffffbfff;
		mcbsp3_idle |= 0x00004000;
		mcbsp3_st_sys |= 0x1;
	} else {
		mcbsp3_sys &= 0xffffffeb;
		mcbsp3_sys |= 0x00000014;
		mcbsp3_idle &= 0xffffbfff;
		mcbsp3_idle |= 0x00004000;
		mcbsp3_st_sys |= 0x1;
	}
	__raw_writel((UWORD32)mcbsp3_sys, MCBSP3_BASE + 0x8c);
	__raw_writel((UWORD32)mcbsp3_idle, MCBSP3_BASE + 0x48);
	__raw_writel((UWORD32)mcbsp3_st_sys, MCBSP3_BASE + 0x10);
}

void McBSP4_Clk_Prepare(BOOL FLAG)
{
	UWORD32 mcbsp4_sys, mcbsp4_idle;
	mcbsp4_sys = __raw_readl((MCBSP4_BASE) + 0x8c);
	mcbsp4_idle = __raw_readl((MCBSP4_BASE) + 0x48);


	if (FLAG) {
		mcbsp4_sys &= 0xfffffdeb;
		mcbsp4_sys |= 0x00000114;
		mcbsp4_idle &= 0xffffbfff;
		mcbsp4_idle |= 0x00004000;
	} else {
		mcbsp4_sys &= 0xffffffeb;
		mcbsp4_sys |= 0x00000014;
		mcbsp4_idle &= 0xffffbfff;
		mcbsp4_idle |= 0x00004000;
	}

	__raw_writel((UWORD32)mcbsp4_sys, MCBSP4_BASE + 0x8c);
	__raw_writel((UWORD32)mcbsp4_idle, MCBSP4_BASE + 0x48);
}

void McBSP5_Clk_Prepare(BOOL FLAG)
{
	UWORD32  mcbsp5_sys, mcbsp5_idle;

	mcbsp5_sys = __raw_readl((MCBSP5_BASE) + 0x8c);
	mcbsp5_idle = __raw_readl((MCBSP5_BASE) + 0x48);

	if (FLAG) {
		mcbsp5_sys &= 0xfffffdeb;
		mcbsp5_sys |= 0x00000114;
		mcbsp5_idle &= 0xffffbfff;
		mcbsp5_idle |= 0x00004000;
	} else {
		mcbsp5_sys &= 0xffffffeb;
		mcbsp5_sys |= 0x00000014;
		mcbsp5_idle &= 0xffffbfff;
		mcbsp5_idle |= 0x00004000;
	}

	__raw_writel((UWORD32)mcbsp5_sys, MCBSP5_BASE + 0x8c);
	__raw_writel((UWORD32)mcbsp5_idle, MCBSP5_BASE + 0x48);
}


INT CLK_Get_UseCnt(IN OSAL_ClkId clk_id)
{
	DSP_STATUS status = DSP_SOK;
	struct clk *pClk;
	INT useCount = -1;
	DBC_Require(clk_id < OSALCLK_NOT_DEFINED);

	pClk = OSAL_Clks[clk_id].clk_handle;

	if (pClk) {
		useCount =  clk_get_usecount(pClk);
	} else {
		GT_1trace(CLK_debugMask, GT_7CLASS,
			  "CLK_GetRate: failed to get "
			  "CLK %s \n", OSAL_Clks[clk_id].clk_name);
		status = DSP_EFAIL;
	}
	return(useCount);
}

void SSI_Clk_Prepare(BOOL FLAG)
{
	UWORD32 ssi_sysconfig;
	ssi_sysconfig = __raw_readl((SSI_BASE)+ 0x10);


	if (FLAG) {
		/* Set Autoidle, SIDLEMode to smart idle, and MIDLEmode to no idle */
		ssi_sysconfig = 0x1011;
	} else {
		/* Set Autoidle, SIDLEMode to forced idle, and MIDLEmode to
		     forced idle*/
		ssi_sysconfig = 0x1;
	}
	__raw_writel((UWORD32)ssi_sysconfig, SSI_BASE + 0x10);
}


/*

DSP_STATUS CLK_AutoIdleCtrl(IN OSAL_ClkId clk_id,INT cmd)
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
		case OSALCLK_gpt5_ick:
		prcm_status = interface_clock_autoidle(PRCM_GPT5,prcm_cmd);
		break;
		case OSALCLK_gpt6_ick:
		prcm_status = interface_clock_autoidle(PRCM_GPT6,prcm_cmd);
		break;
		case OSALCLK_gpt7_ick:
		prcm_status = interface_clock_autoidle(PRCM_GPT7,prcm_cmd);
		break;
		case OSALCLK_gpt8_ick:
		prcm_status = interface_clock_autoidle(PRCM_GPT8,prcm_cmd);
		break;
		case OSALCLK_wdt4_ick:
		prcm_status = interface_clock_autoidle(PRCM_WDT4,prcm_cmd);
		break;
		case OSALCLK_mcbsp1_ick:
		prcm_status = interface_clock_autoidle(PRCM_MCBSP1,prcm_cmd);
		break;
		case OSALCLK_mcbsp2_ick:
		prcm_status = interface_clock_autoidle(PRCM_MCBSP2,prcm_cmd);
		break;
		case OSALCLK_mcbsp3_ick:
		prcm_status = interface_clock_autoidle(PRCM_MCBSP3,prcm_cmd);
		break;
		case OSALCLK_mcbsp4_ick:
		prcm_status = interface_clock_autoidle(PRCM_MCBSP4,prcm_cmd);
		break;
		case OSALCLK_mcbsp5_ick:
		prcm_status = interface_clock_autoidle(PRCM_MCBSP5,prcm_cmd);
		break;
		case OSALCLK_ssi_ick:
		prcm_status = interface_clock_autoidle(PRCM_SSI,prcm_cmd);
		break;
		default:
		GT_1trace(SYNC_debugMask,GT_7CLASS,
			  "CLK_AutoIdle: failed for CLK %s \n",
			  OSAL_Clks[clk_id].clk_name);
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
#endif

