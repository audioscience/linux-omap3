/*
 * dspbridge/src/hal/omap3/prcm/hal_prcm.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
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
 *  ======== hal_prcm.c ========
 *  Description:
 *      API definitions to configure PRCM (Power, Reset & Clocks Manager)
 *
 *! Revision History:
 *! ================
 *! 16 Feb 2003 sb: Initial version
 */

/*
* PROJECT SPECIFIC INCLUDE FILES
*/
#include <GlobalTypes.h>
#include "PRCMRegAcM.h"
#include <hal_defs.h>
#include <hal_prcm.h>

/*
* LOCAL FUNCTIONS PROTOTYPES
*/
static HAL_STATUS HAL_CLK_WriteVal(const UWORD32 baseAddress,
				    HAL_ClkModule_t c, HAL_SetClear_t val);
static HAL_STATUS HAL_CLK_AutoIdleWriteVal(const UWORD32 baseAddress,
					    HAL_ClkModule_t c,
					    HAL_SetClear_t val);
static HAL_STATUS HAL_CLK_SyncWriteVal(const UWORD32 baseAddress,
					HAL_ClkModule_t c, HAL_SetClear_t val);
static HAL_STATUS HAL_RST_WriteVal(const UWORD32 baseAddress,
				    HAL_RstModule_t r, HAL_SetClear_t val);

/*
* EXPORTED FUNCTIONS
*/

HAL_STATUS HAL_RST_Reset(const UWORD32 baseAddress, HAL_RstModule_t r)
{
    return HAL_RST_WriteVal(baseAddress, r, HAL_SET);
}

HAL_STATUS HAL_RST_UnReset(const UWORD32 baseAddress, HAL_RstModule_t r)
{
    return HAL_RST_WriteVal(baseAddress, r, HAL_CLEAR);
}

/*
* LOCAL FUNCTIONS
*/

static HAL_STATUS HAL_CLK_WriteVal(const UWORD32 baseAddress,
				    HAL_ClkModule_t c, HAL_SetClear_t val)
{
    HAL_STATUS status = RET_OK;

	switch (c) {
	case HAL_CLK_DSP_CPU:
		CM_FCLKEN_IVA2EN_DSPWrite32(baseAddress, val);
		break;

	case HAL_CLK_DSP_IPI_MMU:
		PRCMCM_ICLKEN_DSPEN_DSP_IPIWrite32(baseAddress, val);
		break;

	case HAL_CLK_IF_MBOX:
		CM_ICLKEN1_COREEN_MAILBOXESWrite32(baseAddress, val);
		break;

	case HAL_CLK_GPT5:
		CM_FCLKEN_PER_GPT5WriteRegister32(baseAddress, val);
		CM_ICLKEN_PER_GPT5WriteRegister32(baseAddress, val);
		break;

	case HAL_CLK_GPT6:
		CM_FCLKEN_PER_GPT6WriteRegister32(baseAddress, val);
		CM_ICLKEN_PER_GPT6WriteRegister32(baseAddress, val);
		break;

	case HAL_CLK_GPT7:
		PRCMCM_FCLKEN1_COREEN_GPT7Write32(baseAddress, val);
		PRCMCM_ICLKEN1_COREEN_GPT7Write32(baseAddress, val);
		break;
	case HAL_CLK_GPT8:
		PRCMCM_FCLKEN1_COREEN_GPT8Write32(baseAddress, val);
		PRCMCM_ICLKEN1_COREEN_GPT8Write32(baseAddress, val);
		break;

	default:
		status = RET_FAIL;
		break;
    }

    return status;
}

static HAL_STATUS HAL_CLK_AutoIdleWriteVal(const UWORD32 baseAddress,
					    HAL_ClkModule_t c,
					    HAL_SetClear_t val)
{
    HAL_STATUS status = RET_OK;

	switch (c) {
	case HAL_CLK_DSP_IPI_MMU:
	    PRCMCM_AUTOIDLE_DSPAUTO_DSP_IPIWrite32(baseAddress, val);
	    break;

	default:
	    status = RET_FAIL;
	    break;
    }

    return status;
}

static HAL_STATUS HAL_CLK_SyncWriteVal(const UWORD32 baseAddress,
					HAL_ClkModule_t c, HAL_SetClear_t val)
{
    HAL_STATUS status = RET_OK;

	switch (c) {
	case HAL_CLK_DSP_CPU:
	    PRCMCM_CLKSEL_DSPSYNC_DSPWrite32(baseAddress, val);
	    break;

	default:
	    status = RET_FAIL;
	    break;
	}

    return status;
}

static HAL_STATUS HAL_RST_WriteVal(const UWORD32 baseAddress,
				    HAL_RstModule_t r, HAL_SetClear_t val)
{
    HAL_STATUS status = RET_OK;

	switch (r) {
	case HAL_RST1_IVA2:
	    PRM_RSTCTRL_IVA2RST1_DSPWrite32(baseAddress, val);
	    break;
	case HAL_RST2_IVA2:
	    PRM_RSTCTRL_IVA2RST2_DSPWrite32(baseAddress, val);
	    break;
	case HAL_RST3_IVA2:
	    PRM_RSTCTRL_IVA2RST3_DSPWrite32(baseAddress, val);
	    break;
	default:
	    status = RET_FAIL;
	    break;
    }
    return status;
}

HAL_STATUS HAL_PWR_IVA2StateGet(const UWORD32 baseAddress, HAL_PwrModule_t p,
				HAL_PwrState_t *value)
{
	HAL_STATUS status = RET_OK;
	UWORD32 temp;

	switch (p) {
	case HAL_PWR_DOMAIN_DSP:
		/* wait until Transition is complete */
		do {
			/* mdelay(1); */
			temp = PRCMPM_PWSTST_IVA2InTransitionRead32
				(baseAddress);

		} while (temp);

		temp = PRCMPM_PWSTST_IVA2ReadRegister32(baseAddress);
		*value = PRCMPM_PWSTST_IVA2PowerStateStGet32(temp);
		break;

	default:
		status = RET_FAIL;
		break;
	}
	return status;
}

HAL_STATUS HAL_PWRST_IVA2RegGet(const UWORD32 baseAddress, UWORD32 *value)
{
	HAL_STATUS status = RET_OK;

	*value = PRCMPM_PWSTST_IVA2ReadRegister32(baseAddress);

	return status;
}


HAL_STATUS HAL_PWR_IVA2PowerStateSet(const UWORD32 baseAddress,
				     HAL_PwrModule_t p, HAL_PwrState_t value)
{
	HAL_STATUS status = RET_OK;

	switch (p) {
	case HAL_PWR_DOMAIN_DSP:
		switch (value) {
		case HAL_PWR_STATE_ON:
			PRCMPM_PWSTCTRL_IVA2PowerStateWriteON32(baseAddress);
			break;
		case HAL_PWR_STATE_RET:
			PRCMPM_PWSTCTRL_DSPPowerStateWriteRET32(baseAddress);
			break;
		case HAL_PWR_STATE_OFF:
			PRCMPM_PWSTCTRL_IVA2PowerStateWriteOFF32(baseAddress);
			break;
		default:
			status = RET_FAIL;
			break;
		}
		break;

	default:
		status = RET_FAIL;
		break;
	}

	return status;
}

HAL_STATUS HAL_PWR_CLKCTRL_IVA2RegSet(const UWORD32 baseAddress,
				      HAL_TransitionState_t val)
{
       HAL_STATUS status = RET_OK;

	PRCMCM_CLKSTCTRL_IVA2WriteRegister32(baseAddress, val);

	return status;

}

HAL_STATUS HAL_RSTST_RegGet(const UWORD32 baseAddress, HAL_RstModule_t m,
			    UWORD32 *value)
{
	HAL_STATUS status = RET_OK;

	*value = PRCMRM_RSTST_DSPReadRegister32(baseAddress);

	return status;
}

HAL_STATUS HAL_RSTCTRL_RegGet(const UWORD32 baseAddress, HAL_RstModule_t m,
			      UWORD32 *value)
{
	HAL_STATUS status = RET_OK;

	*value = PRCMRM_RSTCTRL_DSPReadRegister32(baseAddress);

	return status;
}
