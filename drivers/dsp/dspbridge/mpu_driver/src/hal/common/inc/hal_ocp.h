/*
 * dspbridge/src/hal/common/inc/hal_ocp.h
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
 *  ======== hal_ocp.h ========
 *  Description:
 *      API declarations for generic OCP Socket system registers
 *
 *! Revision History:
 *! ================
 *! 16 Feb 2003 sb: Initial version
 */
#ifndef __HAL_OCP_H
#define __HAL_OCP_H

/*
 * INCLUDE FILES (only if necessary)
 *
 */

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * EXPORTED DEFINITIONS
 *
 */


/*
 * EXPORTED TYPES
 *
 */

/* ----------------------------------------------------------------------------
* TYPE:	 HAL_IdleMode_t
*
* DESCRIPTION:  Enumerated Type for idle modes in OCP SYSCONFIG register
*
* -----------------------------------------------------------------------------
*/
typedef enum HAL_OCPIdleMode {
    HAL_OCP_FORCE_IDLE,
    HAL_OCP_NO_IDLE,
    HAL_OCP_SMART_IDLE

} HAL_OCPIdleMode_t ;

/*
 * EXPORTED VARIABLES
 *
 */


/*
 * EXPORTED FUNCTIONS
 *
 */

extern HAL_STATUS HAL_OCP_SoftReset(const UWORD32 baseAddress);

extern HAL_STATUS HAL_OCP_SoftResetIsDone(const UWORD32 baseAddress,
					   UWORD32 *resetIsDone);

extern HAL_STATUS HAL_OCP_IdleModeSet(const UWORD32 baseAddress,
				       HAL_OCPIdleMode_t idleMode);

extern HAL_STATUS HAL_OCP_IdleModeGet(const UWORD32 baseAddress,
				       HAL_OCPIdleMode_t *idleMode);

extern HAL_STATUS HAL_OCP_AutoIdleSet(const UWORD32 baseAddress,
				       HAL_SetClear_t autoIdle);

extern HAL_STATUS HAL_OCP_AutoIdleGet(const UWORD32 baseAddress,
				       HAL_SetClear_t *autoIdle);


#ifdef __cplusplus
}
#endif
#endif  /* __HAL_OCP_H */
