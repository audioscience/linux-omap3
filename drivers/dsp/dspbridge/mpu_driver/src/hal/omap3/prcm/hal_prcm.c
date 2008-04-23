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
/* ============================================================================
* STANDARD INCLUDE FILES
* =============================================================================
*/

/* ============================================================================
* PROJECT SPECIFIC INCLUDE FILES
* =============================================================================
*/
#include <GlobalTypes.h>
#include "PRCMRegAcM.h"
#include <hal_defs.h>
#include <hal_prcm.h>

/* ============================================================================
* GLOBAL VARIABLES DECLARATIONS
* =============================================================================
*/

/* ============================================================================
* LOCAL TYPES AND DEFINITIONS
* =============================================================================
*/


/* ============================================================================
* LOCAL VARIABLES DECLARATIONS
* =============================================================================
*/

/* ============================================================================
* LOCAL FUNCTIONS PROTOTYPES
* =============================================================================
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

/* ============================================================================
* EXPORTED FUNCTIONS
* =============================================================================
*/

HAL_STATUS HAL_PLLAPLLs_ClkinRead(const UWORD32 baseAddress, UWORD32 *apllClkin)
{
    HAL_STATUS status = RET_OK;

	*apllClkin = PRCMCM_CLKSEL1_PLLAPLLs_ClkinRead32(baseAddress);

    return status;
}

HAL_STATUS HAL_CLK_ConfControl_Validate(const UWORD32 baseAddress)
{
	HAL_STATUS status = RET_OK;
	PRCMPRCM_CLKCFG_CTRLValid_configWriteClk_valid32(baseAddress);
	return status;
}

HAL_STATUS HAL_CLK_SetInputClock(const UWORD32 baseAddress, HAL_GPtimer_t gpt,
				 HAL_Clocktype_t c)
{
    HAL_STATUS status = RET_OK;

	switch (gpt) {
	case HAL_GPT5:
		if (c == HAL_CLK_32KHz) {
			CM_CLKSEL_PER_GPT5Write32k32(baseAddress);
		} else if (c == HAL_CLK_SYS) {
			PRCMCM_CLKSEL2_CORECLKSEL_GPT5WriteSys32(baseAddress);
		} else if (c == HAL_CLK_EXT) {
			PRCMCM_CLKSEL2_CORECLKSEL_GPT5WriteExt32(baseAddress);
		} else {
			status = RET_FAIL;
		}
	    break;

	case HAL_GPT6:
		if (c == HAL_CLK_32KHz) {
			CM_CLKSEL_PER_GPT6Write32k32(baseAddress);
		} else if (c == HAL_CLK_SYS) {
			PRCMCM_CLKSEL2_CORECLKSEL_GPT6WriteSys32(baseAddress);
		} else if (c == HAL_CLK_EXT) {
			PRCMCM_CLKSEL2_CORECLKSEL_GPT6WriteExt32(baseAddress);
		} else {
			status = RET_FAIL;
		}
	    break;

	case HAL_GPT7:
		if (c == HAL_CLK_32KHz) {
			PRCMCM_CLKSEL2_CORECLKSEL_GPT7Write32k32(baseAddress);
		} else if (c == HAL_CLK_SYS) {
			PRCMCM_CLKSEL2_CORECLKSEL_GPT7WriteSys32(baseAddress);
		} else if (c == HAL_CLK_EXT) {
			PRCMCM_CLKSEL2_CORECLKSEL_GPT7WriteExt32(baseAddress);
		} else {
			status = RET_FAIL;
		}
	    break;
	case HAL_GPT8:
		if (c == HAL_CLK_32KHz) {
			PRCMCM_CLKSEL2_CORECLKSEL_GPT8Write32k32(baseAddress);
		} else if (c == HAL_CLK_SYS) {
			PRCMCM_CLKSEL2_CORECLKSEL_GPT8WriteSys32(baseAddress);
		} else if (c == HAL_CLK_EXT) {
			PRCMCM_CLKSEL2_CORECLKSEL_GPT8WriteExt32(baseAddress);
		} else {
			status = RET_FAIL;
		}
	    break;
	default:
	    status = RET_FAIL;
	    break;
    }

    return status;
}

HAL_STATUS HAL_CLK_Enable(const UWORD32 baseAddress, HAL_ClkModule_t c)
{
    return HAL_CLK_WriteVal(baseAddress, c, HAL_SET);
}

HAL_STATUS HAL_CLK_Disable(const UWORD32 baseAddress, HAL_ClkModule_t c)
{
    return HAL_CLK_WriteVal(baseAddress, c, HAL_CLEAR);
}

HAL_STATUS HAL_CLK_AutoIdleEnable(const UWORD32 baseAddress, HAL_ClkModule_t c)
{
    return HAL_CLK_AutoIdleWriteVal(baseAddress, c, HAL_SET);
}

HAL_STATUS HAL_CLK_AutoIdleDisable(const UWORD32 baseAddress, HAL_ClkModule_t c)
{
    return HAL_CLK_AutoIdleWriteVal(baseAddress, c, HAL_CLEAR);
}

HAL_STATUS HAL_CLK_IdleStatus(const UWORD32 baseAddress, HAL_ClkModule_t c,
			      HAL_IdleState_t *idleState)
{
    HAL_STATUS status = RET_OK;

	switch (c) {
	case HAL_CLK_DSP_CPU:
	    *idleState =
		(HAL_IdleState_t)PRM_IDLEST_IVA2ST_IVA2Read32(baseAddress);
	    break;

	case HAL_CLK_DSP_IPI_MMU:
	    *idleState =
		(HAL_IdleState_t)PRCMCM_IDLEST_DSPST_IPIRead32(baseAddress);
	    break;

	default:
	    status = RET_FAIL;
	    break;
	}

    return status;
}

HAL_STATUS HAL_CLK_DivisorSet(const UWORD32 baseAddress, HAL_ClkModule_t c,
			      HAL_ClkDiv_t divisor)
{
    HAL_STATUS status = RET_OK;

	switch (c) {
	case HAL_CLK_DSP_CPU:
	    PRCMCM_CLKSEL_DSPCLKSEL_DSPWrite32(baseAddress, divisor);
	    break;

	case HAL_CLK_DSP_IPI_MMU:
	    PRCMCM_CLKSEL_DSPCLKSEL_DSP_IFWrite32(baseAddress, divisor);
	    break;

	default:
	    status = RET_FAIL;
	    break;
    }

    return status;
}

HAL_STATUS HAL_CLK_SyncEnable(const UWORD32 baseAddress, HAL_ClkModule_t c)
{
    return HAL_CLK_SyncWriteVal(baseAddress, c, HAL_SET);
}

HAL_STATUS HAL_CLK_SyncDisable(const UWORD32 baseAddress, HAL_ClkModule_t c)
{
    return HAL_CLK_SyncWriteVal(baseAddress, c, HAL_CLEAR);
}

HAL_STATUS HAL_CLK_DSPPerClks(const UWORD32 baseAddress,
			      HAL_SetClear_t *idleState)
{
    HAL_STATUS status = RET_OK;
    UWORD32 FclkEnVal, FclkEn1Val = 0;
    UWORD32 IclkEnVal, IclkEn1Val  = 0;

    /* GPT5-8 = Bits 6-9, McBSP2 = Bit 0 */
    UWORD32 ClkMaskPerVal = 0x000003C1;
    UWORD32 ClkMaskCoreVal = 0x00000200;  /* McBSP1 = Bit 9 */

	/* Read the register CM_FCLKEN1_CORE in CM */
	FclkEn1Val = CM_FCLKEN1_COREReadRegister32(baseAddress);
	if ((FclkEn1Val & ClkMaskCoreVal) == 0)	{
		*idleState = (HAL_SetClear_t)HAL_CLEAR ;
	} else {
		*idleState = (HAL_SetClear_t)HAL_SET ;
		return status ;
	}

	/* Read the register CM_FCLKEN_PER in PER */
	FclkEn1Val = CM_FCLKEN_PERReadRegister32(baseAddress) ;
	if ((FclkEn1Val & ClkMaskPerVal) == 0) {
		*idleState = (HAL_SetClear_t)HAL_CLEAR ;
	} else {
		*idleState = (HAL_SetClear_t)HAL_SET ;
		return status ;
	}

	/* Read the register CM_ICLKEN1_CORE in CM */
	IclkEn1Val = CM_ICLKEN1_COREReadRegister32(baseAddress) ;
	if ((IclkEn1Val & ClkMaskCoreVal) == 0) {
		*idleState = (HAL_SetClear_t)HAL_CLEAR ;
	} else {
		*idleState = (HAL_SetClear_t)HAL_SET ;
		return status ;
	}
	/* Read the register CM_ICLKEN_PER in PER */
	IclkEn1Val = CM_ICLKEN_PERReadRegister32(baseAddress) ;
	if ((IclkEn1Val & ClkMaskPerVal) == 0) {
		*idleState = (HAL_SetClear_t)HAL_CLEAR ;
	} else {
		*idleState = (HAL_SetClear_t)HAL_SET ;
		return status ;
	}
	return status ;
}

HAL_STATUS HAL_RST_Reset(const UWORD32 baseAddress, HAL_RstModule_t r)
{
    return HAL_RST_WriteVal(baseAddress, r, HAL_SET);
}

HAL_STATUS HAL_RST_UnReset(const UWORD32 baseAddress, HAL_RstModule_t r)
{
    return HAL_RST_WriteVal(baseAddress, r, HAL_CLEAR);
}

HAL_STATUS HAL_RST_GetCause(const UWORD32 baseAddress, HAL_RstModule_t r)
{
    HAL_STATUS status = RET_OK;

    return status;
}


HAL_STATUS HAL_PWR_EnableWkupEvent(const UWORD32 baseAddress, HAL_PwrModule_t p)
{
    HAL_STATUS status = RET_OK;

    return status;
}

HAL_STATUS HAL_PWR_DisableWkupEvent(const UWORD32 baseAddress,
				    HAL_PwrModule_t p)
{
    HAL_STATUS status = RET_OK;

    return status;
}

HAL_STATUS HAL_PWR_EnableWkupDependency(const UWORD32 baseAddress,
					HAL_PwrModule_t p, HAL_PwrModule_t src)
{
    HAL_STATUS status = RET_OK;

    return status;
}

HAL_STATUS HAL_PWR_DisableWkupDependency(const UWORD32 baseAddress,
					 HAL_PwrModule_t p, HAL_PwrModule_t src)
{
    HAL_STATUS status = RET_OK;

    return status;
}

HAL_STATUS HAL_PWR_EnableAutoState(const UWORD32 baseAddress,
				    HAL_PwrModule_t p)
{
    HAL_STATUS status = RET_OK;

    return status;
}

HAL_STATUS HAL_PWR_DisableAutoState(const UWORD32 baseAddress,
				     HAL_PwrModule_t p)
{
    HAL_STATUS status = RET_OK;

    return status;
}

/*
HAL_STATUS HAL_PWR_SetState (const UWORD32 baseAddress,
				HAL_PwrModule_t p, PwrState s)
{
    HAL_STATUS status = RET_OK;

    return status;
}
*/


HAL_STATUS HAL_PWR_ForceState(const UWORD32 baseAddress, HAL_PwrModule_t p)
{
    HAL_STATUS status = RET_OK;

    return status;
}

HAL_STATUS HAL_PWR_GetState(const UWORD32 baseAddress, HAL_PwrModule_t p)
{
    HAL_STATUS status = RET_OK;

    return status;
}


HAL_STATUS HAL_PWR_IsInTransition(const UWORD32 baseAddress, HAL_PwrModule_t p)
{
    HAL_STATUS status = RET_OK;

    return status;
}

HAL_STATUS HAL_PWR_IsClkActive(const UWORD32 baseAddress, HAL_PwrModule_t p)
{
    HAL_STATUS status = RET_OK;

    return status;
}

/* ============================================================================
* LOCAL FUNCTIONS
* =============================================================================
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

HAL_STATUS HAL_CLK_AutoStateGet(const UWORD32 baseAddress, HAL_ClkSubsys_t c,
				UWORD32 *value)
{
	HAL_STATUS status = RET_OK;

	switch (c) {
	case HAL_CLK_DSPSS:
		*value = PRCMCM_CLKSTCTRL_DSPAutostate_DSPRead32(baseAddress);
		break;
	default:
		status = RET_FAIL;
		break;
	}

	return status;
}

HAL_STATUS HAL_CLK_AutoStateSet(const UWORD32 baseAddress, HAL_ClkSubsys_t c,
				HAL_SetClear_t *value)
{
	HAL_STATUS status = RET_OK;

	switch (c) {
	case HAL_CLK_DSPSS:
		PRCMCM_CLKSTCTRL_DSPAutostate_DSPWrite32(baseAddress, value);
		break;
	default:
		status = RET_FAIL;
		break;
	}

	return status;
}

HAL_STATUS HAL_CLK_IdleStateGet(const UWORD32 baseAddress, UWORD32 *value)
{
	HAL_STATUS status = RET_OK;

	*value = PRCMCM_IDLEST_DSPReadRegister32(baseAddress);

	return status;
}

HAL_STATUS HAL_PWR_StateGet(const UWORD32 baseAddress, HAL_PwrModule_t p,
			    HAL_PwrState_t *value)
{
	HAL_STATUS status = RET_OK;
	UWORD32 temp;

	switch (p) {
	case HAL_PWR_DOMAIN_DSP:
		/* wait until Transition is complete */
		do {
			/* mdelay(1); */
			temp = PRCMPM_PWSTST_DSPInTransitionRead32(baseAddress);

		} while (temp);

		temp = PRCMPM_PWSTST_DSPReadRegister32(baseAddress);
		*value = PRCMPM_PWSTST_DSPPowerStateStGet32(temp);
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



HAL_STATUS HAL_PWRST_RegGet(const UWORD32 baseAddress, UWORD32 *value)
{
	HAL_STATUS status = RET_OK;

	*value = PRCMPM_PWSTST_DSPReadRegister32(baseAddress);

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

HAL_STATUS HAL_PWR_ForceStateSet(const UWORD32 baseAddress, HAL_PwrModule_t p,
				 HAL_ForceState_t value)
{
	HAL_STATUS status = RET_OK;

	switch (p) {
	case HAL_PWR_DOMAIN_DSP:
		PRCMPM_PWSTCTRL_DSPForceStateWrite32(baseAddress, value);
		break;

	default:
		status = RET_FAIL;
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

HAL_STATUS HAL_RSTST_RegClear(const UWORD32 baseAddress, HAL_RstModule_t m)
{
	HAL_STATUS status = RET_OK;
	UWORD32 value;

	value = PRCMRM_RSTST_DSPReadRegister32(baseAddress);
	PRCMRM_RSTST_DSPWriteRegister32(baseAddress, value);

	return status;
}

HAL_STATUS HAL_RSTCTRL_RegGet(const UWORD32 baseAddress, HAL_RstModule_t m,
			      UWORD32 *value)
{
	HAL_STATUS status = RET_OK;

	*value = PRCMRM_RSTCTRL_DSPReadRegister32(baseAddress);

	return status;
}


