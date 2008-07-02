/*
 * dspbridge/src/hal/omap3/inc/hal_prcm.h
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
 *  ======== hal_prcm.h ========
 *  Description:
 *      PRCM types and API declarations
 *
 *! Revision History:
 *! ================
 *! 16 Feb 2003 sb: Initial version
 */

#ifndef __HAL_PRCM_H
#define __HAL_PRCM_H

#ifdef __cplusplus
extern "C"
{
#endif

/*
* TYPE:	 HAL_ClkModule
*
* DESCRIPTION:  Enumerated Type used to specify the clock domain
*/

typedef enum HAL_ClkModule {
/* DSP Domain */
    HAL_CLK_DSP_CPU,
    HAL_CLK_DSP_IPI_MMU,
    HAL_CLK_IVA_ARM,
    HAL_CLK_IVA_COP,	/* IVA Coprocessor */

/* Core Domain */
    HAL_CLK_FN_WDT4,	/* Functional Clock */
    HAL_CLK_FN_WDT3,
    HAL_CLK_FN_UART2,
    HAL_CLK_FN_UART1,
    HAL_CLK_GPT5,
    HAL_CLK_GPT6,
    HAL_CLK_GPT7,
    HAL_CLK_GPT8,

    HAL_CLK_IF_WDT4,	/* Interface Clock */
    HAL_CLK_IF_WDT3,
    HAL_CLK_IF_UART2,
    HAL_CLK_IF_UART1,
    HAL_CLK_IF_MBOX

} HAL_ClkModule_t;

typedef enum HAL_ClkSubsys {
    HAL_CLK_DSPSS,
    HAL_CLK_IVASS
} HAL_ClkSubsys_t;

/*
* TYPE:	 HAL_GPtimers
*
* DESCRIPTION:  General purpose timers
*/
typedef enum HAL_GPtimers {
    HAL_GPT5 = 5,
    HAL_GPT6 = 6,
    HAL_GPT7 = 7,
    HAL_GPT8 = 8
} HAL_GPtimer_t;


/*
* TYPE:	 GP timers Input clock type
*
* DESCRIPTION:  General purpose timers
*/
typedef enum HAL_GPtimers_Input_clock {
    HAL_CLK_32KHz = 0,
    HAL_CLK_SYS   = 1,
    HAL_CLK_EXT   = 2
} HAL_Clocktype_t;

/*
* TYPE:	 HAL_ClkDiv
*
* DESCRIPTION:  Clock divisors
*/
typedef enum HAL_ClkDiv {
    HAL_CLK_DIV_1 = 0x1,
    HAL_CLK_DIV_2 = 0x2,
    HAL_CLK_DIV_3 = 0x3,
    HAL_CLK_DIV_4 = 0x4,
    HAL_CLK_DIV_6 = 0x6,
    HAL_CLK_DIV_8 = 0x8,
    HAL_CLK_DIV_12 = 0xC
} HAL_ClkDiv_t;

/*
* TYPE:	 HAL_RstModule
*
* DESCRIPTION:  Enumerated Type used to specify the module to be reset
*/
typedef enum HAL_RstModule {
    HAL_RST1_IVA2,  /* Reset the DSP */
    HAL_RST2_IVA2,  /* Reset MMU and LEON HWa */
    HAL_RST3_IVA2   /* Reset LEON sequencer */
} HAL_RstModule_t;

/*
* TYPE:	 HAL_RstCause
*
* DESCRIPTION:  Enumerated Type used to specify the cause of the reset
*/
/* TBD */
typedef enum RstCause {
    HAL_RSTCAUSE_DSP1_UMA_DMA,
    HAL_RSTCAUSE_DSP2_IPI_MMU,
    HAL_RSTCAUSE_IVA,
    HAL_RSTCAUSE_UMA,
    HAL_RSTCAUSE_CORE,
    HAL_RSTCAUSE_MPU,
    HAL_RSTCAUSE_GLOBALWARM
} RstCause;

/*
* TYPE:	 HAL_PwrModule
*
* DESCRIPTION:  Enumerated Type used to specify the power domain
*/

typedef enum HAL_PwrModule {
/* Domains */
    HAL_PWR_DOMAIN_CORE,
    HAL_PWR_DOMAIN_MPU,
    HAL_PWR_DOMAIN_WAKEUP,
    HAL_PWR_DOMAIN_DSP,

/* Sub-domains */
    HAL_PWR_DSP_IPI,	/* IPI = Intrusive Port Interface */
    HAL_PWR_IVA_ISP	 /* ISP = Intrusive Slave Port */
} HAL_PwrModule_t;

typedef enum HAL_PwrState {
    HAL_PWR_STATE_OFF,
    HAL_PWR_STATE_RET,
    HAL_PWR_STATE_INACT,
    HAL_PWR_STATE_ON = 3
} HAL_PwrState_t;

typedef enum HAL_ForceState {
    HAL_FORCE_OFF,
    HAL_FORCE_ON
} HAL_ForceState_t;

typedef enum HAL_IdleState {
    HAL_ACTIVE,
    HAL_STANDBY

} HAL_IdleState_t;

typedef enum HAL_PWR_TransState {
    HAL_AUTOTRANS_DIS,
    HAL_SW_SUP_SLEEP,
    HAL_SW_SUP_WAKEUP,
    HAL_AUTOTRANS_EN
} HAL_TransitionState_t;


extern HAL_STATUS HAL_RST_Reset(const UWORD32 baseAddress,
				 HAL_RstModule_t r);

extern HAL_STATUS HAL_RST_UnReset(const UWORD32 baseAddress,
				   HAL_RstModule_t r);

extern HAL_STATUS HAL_RSTCTRL_RegGet(const UWORD32 baseAddress,
					     HAL_RstModule_t p,
					     UWORD32 *value);
extern HAL_STATUS HAL_RSTST_RegGet(const UWORD32 baseAddress,
					   HAL_RstModule_t p, UWORD32 *value);

extern HAL_STATUS HAL_PWR_PowerStateSet(const UWORD32 baseAddress,
						HAL_PwrModule_t p,
						HAL_PwrState_t value);

extern HAL_STATUS HAL_CLK_SetInputClock(const UWORD32 baseAddress,
					HAL_GPtimer_t gpt, HAL_Clocktype_t c) ;

extern HAL_STATUS HAL_PWR_IVA2StateGet(const UWORD32 baseAddress,
					HAL_PwrModule_t p,
					HAL_PwrState_t *value);

extern HAL_STATUS HAL_PWRST_IVA2RegGet(const UWORD32 baseAddress,
					UWORD32 *value);

extern HAL_STATUS HAL_PWR_IVA2PowerStateSet(const UWORD32 baseAddress,
					    HAL_PwrModule_t p,
					    HAL_PwrState_t value);

extern HAL_STATUS HAL_PWR_CLKCTRL_IVA2RegSet(const UWORD32 baseAddress,
					     HAL_TransitionState_t val);

#ifdef __cplusplus
}
#endif
#endif  /* __HAL_PRCM_H */
