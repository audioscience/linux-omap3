/*
 * dspbridge/src/hal/common/inc/hal_defs.h
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
 *  ======== hal_defs.h ========
 *  Description:
 *      Global HAL definitions
 *
 *! Revision History:
 *! ================
 *! 19 Apr 2004 sb: Added generic page size, endianness and element size defns
 *! 16 Feb 2003 sb: Initial version
 */
#ifndef __HAL_DEFS_H
#define __HAL_DEFS_H

/* ============================================================================
* INCLUDE FILES (only if necessary)
* =============================================================================
*/

#include <GlobalTypes.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ============================================================================
* EXPORTED DEFINITIONS
* =============================================================================
*/

/* Page size */
#define HAL_PAGE_SIZE_4KB   0x1000
#define HAL_PAGE_SIZE_64KB  0x10000
#define HAL_PAGE_SIZE_1MB   0x100000
#define HAL_PAGE_SIZE_16MB  0x1000000

/* ============================================================================
* EXPORTED TYPES
* =============================================================================
*/
/* ----------------------------------------------------------------------------
* TYPE:         HAL_STATUS
*
* DESCRIPTION:  return type for HAL API
*
* -----------------------------------------------------------------------------
*/
typedef long HAL_STATUS;

/* ----------------------------------------------------------------------------
* TYPE:         HAL_SetClear_t
*
* DESCRIPTION:  Enumerated Type used to set and clear any bit
*
* -----------------------------------------------------------------------------
*/
typedef enum HAL_SetClear {
    HAL_CLEAR,
    HAL_SET

} HAL_SetClear_t ;

/* ----------------------------------------------------------------------------
* TYPE:         HAL_Endianism_t
*
* DESCRIPTION:  Enumerated Type used to specify the endianism
*               Do NOT change these values. They are used as bit fields.
*
* -----------------------------------------------------------------------------
*/
typedef enum HAL_Endianism {
    HAL_LITTLE_ENDIAN,
    HAL_BIG_ENDIAN

} HAL_Endianism_t;

/* ----------------------------------------------------------------------------
* TYPE:         HAL_ElementSize_t
*
* DESCRIPTION:  Enumerated Type used to specify the element size
*               Do NOT change these values. They are used as bit fields.
*
* -----------------------------------------------------------------------------
*/
typedef enum HAL_ElementSize {
    HAL_ELEM_SIZE_8BIT,
    HAL_ELEM_SIZE_16BIT,
    HAL_ELEM_SIZE_32BIT,
    HAL_ELEM_SIZE_64BIT

} HAL_ElementSize_t;

/* ----------------------------------------------------------------------------
* TYPE:         HAL_IdleMode_t
*
* DESCRIPTION:  Enumerated Type used to specify Idle modes
*
* -----------------------------------------------------------------------------
*/
	typedef enum HAL_IdleMode {
		HAL_FORCE_IDLE,
		HAL_NO_IDLE,
		HAL_SMART_IDLE
	} HAL_IdleMode_t;

/* ============================================================================
* EXPORTED VARIABLES
* =============================================================================
*/

/* ============================================================================
* EXPORTED FUNCTIONS
* =============================================================================
*/

#ifdef __cplusplus
}
#endif
#endif  /* __HAL_DEFS_H */
