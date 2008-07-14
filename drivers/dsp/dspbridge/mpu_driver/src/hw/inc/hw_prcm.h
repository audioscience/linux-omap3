/*
 * dspbridge/src/hw/omap3/inc/hw_prcm.h
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
 *  ======== hw_prcm.h ========
 *  Description:
 *      PRCM types and API declarations
 *
 *! Revision History:
 *! ================
 *! 16 Feb 2003 sb: Initial version
 */

#ifndef __HW_PRCM_H
#define __HW_PRCM_H

/*
* TYPE:	 HW_ClkModule
*
* DESCRIPTION:  Enumerated Type used to specify the clock domain
*/

typedef enum HW_ClkModule {
/* DSP Domain */
    HW_CLK_DSP_CPU,
    HW_CLK_DSP_IPI_MMU,
    HW_CLK_IVA_ARM,
    HW_CLK_IVA_COP,	/* IVA Coprocessor */

/* Core Domain */
    HW_CLK_FN_WDT4,	/* Functional Clock */
    HW_CLK_FN_WDT3,
    HW_CLK_FN_UART2,
    HW_CLK_FN_UART1,
    HW_CLK_GPT5,
    HW_CLK_GPT6,
    HW_CLK_GPT7,
    HW_CLK_GPT8,

    HW_CLK_IF_WDT4,	/* Interface Clock */
    HW_CLK_IF_WDT3,
    HW_CLK_IF_UART2,
    HW_CLK_IF_UART1,
    HW_CLK_IF_MBOX

} HW_ClkModule_t;

typedef enum HW_ClkSubsys {
    HW_CLK_DSPSS,
    HW_CLK_IVASS
} HW_ClkSubsys_t;

/*
* TYPE:	 HW_GPtimers
*
* DESCRIPTION:  General purpose timers
*/
typedef enum HW_GPtimers {
    HW_GPT5 = 5,
    HW_GPT6 = 6,
    HW_GPT7 = 7,
    HW_GPT8 = 8
} HW_GPtimer_t;


/*
* TYPE:	 GP timers Input clock type
*
* DESCRIPTION:  General purpose timers
*/
typedef enum HW_GPtimers_Input_clock {
    HW_CLK_32KHz = 0,
    HW_CLK_SYS   = 1,
    HW_CLK_EXT   = 2
} HW_Clocktype_t;

/*
* TYPE:	 HW_ClkDiv
*
* DESCRIPTION:  Clock divisors
*/
typedef enum HW_ClkDiv {
    HW_CLK_DIV_1 = 0x1,
    HW_CLK_DIV_2 = 0x2,
    HW_CLK_DIV_3 = 0x3,
    HW_CLK_DIV_4 = 0x4,
    HW_CLK_DIV_6 = 0x6,
    HW_CLK_DIV_8 = 0x8,
    HW_CLK_DIV_12 = 0xC
} HW_ClkDiv_t;

/*
* TYPE:	 HW_RstModule
*
* DESCRIPTION:  Enumerated Type used to specify the module to be reset
*/
typedef enum HW_RstModule {
    HW_RST1_IVA2,  /* Reset the DSP */
    HW_RST2_IVA2,  /* Reset MMU and LEON HWa */
    HW_RST3_IVA2   /* Reset LEON sequencer */
} HW_RstModule_t;

/*
* TYPE:	 HW_RstCause
*
* DESCRIPTION:  Enumerated Type used to specify the cause of the reset
*/
/* TBD */
typedef enum RstCause {
    HW_RSTCAUSE_DSP1_UMA_DMA,
    HW_RSTCAUSE_DSP2_IPI_MMU,
    HW_RSTCAUSE_IVA,
    HW_RSTCAUSE_UMA,
    HW_RSTCAUSE_CORE,
    HW_RSTCAUSE_MPU,
    HW_RSTCAUSE_GLOBALWARM
} RstCause;

/*
* TYPE:	 HW_PwrModule
*
* DESCRIPTION:  Enumerated Type used to specify the power domain
*/

typedef enum HW_PwrModule {
/* Domains */
    HW_PWR_DOMAIN_CORE,
    HW_PWR_DOMAIN_MPU,
    HW_PWR_DOMAIN_WAKEUP,
    HW_PWR_DOMAIN_DSP,

/* Sub-domains */
    HW_PWR_DSP_IPI,	/* IPI = Intrusive Port Interface */
    HW_PWR_IVA_ISP	 /* ISP = Intrusive Slave Port */
} HW_PwrModule_t;

typedef enum HW_PwrState {
    HW_PWR_STATE_OFF,
    HW_PWR_STATE_RET,
    HW_PWR_STATE_INACT,
    HW_PWR_STATE_ON = 3
} HW_PwrState_t;

typedef enum HW_ForceState {
    HW_FORCE_OFF,
    HW_FORCE_ON
} HW_ForceState_t;

typedef enum HW_IdleState {
    HW_ACTIVE,
    HW_STANDBY

} HW_IdleState_t;

typedef enum HW_PWR_TransState {
    HW_AUTOTRANS_DIS,
    HW_SW_SUP_SLEEP,
    HW_SW_SUP_WAKEUP,
    HW_AUTOTRANS_EN
} HW_TransitionState_t;


extern HW_STATUS HW_RST_Reset(const UWORD32 baseAddress,
				 HW_RstModule_t r);

extern HW_STATUS HW_RST_UnReset(const UWORD32 baseAddress,
				   HW_RstModule_t r);

extern HW_STATUS HW_RSTCTRL_RegGet(const UWORD32 baseAddress,
					     HW_RstModule_t p,
					     UWORD32 *value);
extern HW_STATUS HW_RSTST_RegGet(const UWORD32 baseAddress,
					   HW_RstModule_t p, UWORD32 *value);

extern HW_STATUS HW_PWR_PowerStateSet(const UWORD32 baseAddress,
						HW_PwrModule_t p,
						HW_PwrState_t value);

extern HW_STATUS HW_CLK_SetInputClock(const UWORD32 baseAddress,
					HW_GPtimer_t gpt, HW_Clocktype_t c) ;

extern HW_STATUS HW_PWR_IVA2StateGet(const UWORD32 baseAddress,
					HW_PwrModule_t p,
					HW_PwrState_t *value);

extern HW_STATUS HW_PWRST_IVA2RegGet(const UWORD32 baseAddress,
					UWORD32 *value);

extern HW_STATUS HW_PWR_IVA2PowerStateSet(const UWORD32 baseAddress,
					    HW_PwrModule_t p,
					    HW_PwrState_t value);

extern HW_STATUS HW_PWR_CLKCTRL_IVA2RegSet(const UWORD32 baseAddress,
					     HW_TransitionState_t val);

#endif  /* __HW_PRCM_H */
