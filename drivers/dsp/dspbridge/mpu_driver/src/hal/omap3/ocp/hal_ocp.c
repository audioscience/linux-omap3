/*
 * dspbridge/src/hal/common/ocp/hal_ocp.c
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
 *  ======== hal_ocp.c ========
 *  Description:
 *      API definitions to setup OCP Socket system registers
 *
 *! Revision History:
 *! ================
 *! 16 Feb 2003 sb: Initial version
 */

/* =========================================================================
* STANDARD INCLUDE FILES
* =======================================================================*/

/* =========================================================================
* PROJECT SPECIFIC INCLUDE FILES
* ========================================================================*/
#include <GlobalTypes.h>
#include <hal_defs.h>
#include <hal_ocp.h>
#include <MLBRegAcM.h>
/* ========================================================================
* GLOBAL VARIABLES DECLARATIONS
* ========================================================================
*/

/* =========================================================================
* LOCAL TYPES AND DEFINITIONS
* ==========================================================================
*/


/* =========================================================================
* LOCAL VARIABLES DECLARATIONS
* ==========================================================================
*/

/* =========================================================================
* LOCAL FUNCTIONS PROTOTYPES
* ==========================================================================
*/

/* =========================================================================
* EXPORTED FUNCTIONS
* ==========================================================================
*/

/* We use Mailbox beach macros to access the generic OCP registers */

HAL_STATUS HAL_OCP_SoftReset(const UWORD32 baseAddress)
{
    HAL_STATUS status = RET_OK;

    MLBMAILBOX_SYSCONFIGSoftResetWrite32(baseAddress, HAL_SET);

    return status;
}

HAL_STATUS HAL_OCP_SoftResetIsDone(const UWORD32 baseAddress,
				   UWORD32 *resetIsDone)
{
    HAL_STATUS status = RET_OK;

    *resetIsDone = MLBMAILBOX_SYSSTATUSResetDoneRead32(baseAddress);

    return status;
}

HAL_STATUS HAL_OCP_IdleModeSet(const UWORD32 baseAddress,
				HAL_OCPIdleMode_t idleMode)
{
    HAL_STATUS status = RET_OK;

    MLBMAILBOX_SYSCONFIGSIdleModeWrite32(baseAddress, idleMode);

    return status;
}


HAL_STATUS HAL_OCP_IdleModeGet(const UWORD32 baseAddress,
				HAL_OCPIdleMode_t *idleMode)
{
    HAL_STATUS status = RET_OK;

    *idleMode = (HAL_OCPIdleMode_t)MLBMAILBOX_SYSCONFIGSIdleModeRead32
		(baseAddress);

    return status;
}

HAL_STATUS HAL_OCP_AutoIdleSet(const UWORD32 baseAddress,
				HAL_SetClear_t autoIdle)
{
    HAL_STATUS status = RET_OK;

    MLBMAILBOX_SYSCONFIGAutoIdleWrite32(baseAddress, autoIdle);

    return status;
}

HAL_STATUS HAL_OCP_AutoIdleGet(const UWORD32 baseAddress,
				HAL_SetClear_t *autoIdle)
{
    HAL_STATUS status = RET_OK;

    *autoIdle = (HAL_SetClear_t)MLBMAILBOX_SYSCONFIGAutoIdleRead32
		(baseAddress);

    return status;
}

/* =========================================================================
*  LOCAL FUNCTIONS
* ==========================================================================
*/

