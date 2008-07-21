/*
 * dspbridge/src/hw/common/inc/hw_defs.h
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
 *  ======== hw_defs.h ========
 *  Description:
 *      Global HW definitions
 *
 *! Revision History:
 *! ================
 *! 19 Apr 2004 sb: Added generic page size, endianness and element size defns
 *! 16 Feb 2003 sb: Initial version
 */
#ifndef __HW_DEFS_H
#define __HW_DEFS_H

#include <GlobalTypes.h>

/* Page size */
#define HW_PAGE_SIZE_4KB   0x1000
#define HW_PAGE_SIZE_64KB  0x10000
#define HW_PAGE_SIZE_1MB   0x100000
#define HW_PAGE_SIZE_16MB  0x1000000

/* ----------------------------------------------------------------------------
* TYPE:         HW_STATUS
*
* DESCRIPTION:  return type for HW API
*
* -----------------------------------------------------------------------------
*/
typedef long HW_STATUS;

/* ----------------------------------------------------------------------------
* TYPE:         HW_SetClear_t
*
* DESCRIPTION:  Enumerated Type used to set and clear any bit
*
* -----------------------------------------------------------------------------
*/
typedef enum HW_SetClear {
    HW_CLEAR,
    HW_SET

} HW_SetClear_t ;

/* ----------------------------------------------------------------------------
* TYPE:         HW_Endianism_t
*
* DESCRIPTION:  Enumerated Type used to specify the endianism
*               Do NOT change these values. They are used as bit fields.
*
* -----------------------------------------------------------------------------
*/
typedef enum HW_Endianism {
    HW_LITTLE_ENDIAN,
    HW_BIG_ENDIAN

} HW_Endianism_t;

/* ----------------------------------------------------------------------------
* TYPE:         HW_ElementSize_t
*
* DESCRIPTION:  Enumerated Type used to specify the element size
*               Do NOT change these values. They are used as bit fields.
*
* -----------------------------------------------------------------------------
*/
typedef enum HW_ElementSize {
    HW_ELEM_SIZE_8BIT,
    HW_ELEM_SIZE_16BIT,
    HW_ELEM_SIZE_32BIT,
    HW_ELEM_SIZE_64BIT

} HW_ElementSize_t;

/* ----------------------------------------------------------------------------
* TYPE:         HW_IdleMode_t
*
* DESCRIPTION:  Enumerated Type used to specify Idle modes
*
* -----------------------------------------------------------------------------
*/
	typedef enum HW_IdleMode {
		HW_FORCE_IDLE,
		HW_NO_IDLE,
		HW_SMART_IDLE
	} HW_IdleMode_t;

#endif  /* __HW_DEFS_H */
