/*
 * dspbridge/mpu_driver/inc/clk.h
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
 *  ======== clk.h ========
 *  Purpose: Provides Clock functions.
 *
 *! Revision History:
 *! ================
 *! 08-May-2007 rg: Moved all clock functions from sync module.
 */

#ifndef _CLK_H
#define _CLK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <dspapi.h>

	/* Generic TIMER object: */
	/*typedef struct TIMER_OBJECT *TIMER_HOBJECT;*/
	struct TIMER_OBJECT;
	typedef enum {
		OSALCLK_iva2_ck = 0,
		OSALCLK_mailbox_ick,
		OSALCLK_gpt5_fck,
		OSALCLK_gpt5_ick,
		OSALCLK_gpt6_fck,
		OSALCLK_gpt6_ick,
		OSALCLK_gpt7_fck,
		OSALCLK_gpt7_ick,
		OSALCLK_gpt8_fck,
		OSALCLK_gpt8_ick,
		OSALCLK_wdt3_fck,
		OSALCLK_wdt3_ick,
		OSALCLK_mcbsp1_fck,
		OSALCLK_mcbsp1_ick,
		OSALCLK_mcbsp2_fck,
		OSALCLK_mcbsp2_ick,
		OSALCLK_mcbsp3_fck,
		OSALCLK_mcbsp3_ick,
		OSALCLK_mcbsp4_fck,
		OSALCLK_mcbsp4_ick,
		OSALCLK_mcbsp5_fck,
		OSALCLK_mcbsp5_ick,
		OSALCLK_ssi_fck,
		OSALCLK_ssi_ick,
		OSALCLK_sys_32k_ck,
		OSALCLK_sys_ck,
		OSALCLK_NOT_DEFINED
	} OSAL_ClkId;

/*
 *  ======== CLK_Exit ========
 *  Purpose:
 *      Discontinue usage of module; free resources when reference count
 *      reaches 0.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      CLK initialized.
 *  Ensures:
 *      Resources used by module are freed when cRef reaches zero.
 */
	extern VOID CLK_Exit();

/*
 *  ======== CLK_Init ========
 *  Purpose:
 *      Initializes private state of CLK module.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occured.
 *  Requires:
 *  Ensures:
 *      CLK initialized.
 */
	extern BOOL CLK_Init();


/*
 *  ======== CLK_Enable ========
 *  Purpose:
 *      Enables the clock requested.
 *  Parameters:
 *  Returns:
 *      DSP_SOK:	Success.
 *	DSP_EFAIL:	Error occured while enabling the clock.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS CLK_Enable(IN OSAL_ClkId clk_id);

/*
 *  ======== CLK_Disable ========
 *  Purpose:
 *      Disables the clock requested.
 *  Parameters:
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      Error occured while disabling the clock.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS CLK_Disable(IN OSAL_ClkId clk_id);

/*
 *  ======== CLK_GetRate ========
 *  Purpose:
 *      Get the clock rate of requested clock.
 *  Parameters:
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      Error occured while Getting the clock rate.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS CLK_GetRate(IN OSAL_ClkId clk_id, ULONG *speedMhz);
/*
 *  ======== CLK_Set_32KHz ========
 *  Purpose:
 *      Set the requested clock to 32KHz.
 *  Parameters:
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      Error occured while setting the clock parent to 32KHz.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS CLK_Set_32KHz(IN OSAL_ClkId clk_id);
	extern void SSI_Clk_Prepare(BOOL FLAG);

/*
 *  ======== CLK_Get_RefCnt ========
 *  Purpose:
 *      get the reference count for the clock.
 *  Parameters:
 *  Returns:
 *      INT:        Reference Count for the clock.
 *      DSP_EFAIL:  Error occured while getting the reference count of a clock.
 *  Requires:
 *  Ensures:
 */
	extern INT CLK_Get_UseCnt(IN OSAL_ClkId clk_id);
/*	extern DSP_STATUS CLK_AutoIdleCtrl(IN OSAL_ClkId clk_id,INT cmd); */

#ifdef __cplusplus
}
#endif
#endif				/* _SYNC_H */
