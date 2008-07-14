/*
 * dspbridge/src/hw/omap3/inc/EasiGlobal.h
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

#ifndef __EASIGLOBAL_H
#define __EASIGLOBAL_H

/* ----------------------------------------------------------------------------
* DEFINE:        READ_ONLY, WRITE_ONLY &  READ_WRITE
*
* DESCRIPTION:  Defines used to describe register types for EASI-checker tests.
*
* NOTE:         Note
*
* -----------------------------------------------------------------------------
*/

#define READ_ONLY    1
#define WRITE_ONLY   2
#define READ_WRITE   3

/*
 * EXPORTED TYPES
 *
 */

/* ----------------------------------------------------------------------------
* TYPE:        regEnum
*
* DESCRIPTION:  regtypes are used to switch to correct test function
*               there are as many regtypes as there are data definitions
*
* NOTE:         None
*
* -----------------------------------------------------------------------------
*/
typedef enum _regEnum {
    ACC_32BIT,
    ACC_16BIT,
    ACC_8BIT
} regEnum;

/* ----------------------------------------------------------------------------
* TYPE:        registerPORStruct
*
* DESCRIPTION:  all required register data held in an array of structure for
*               power on reset tests.
*
* NOTE:         None
*
* -----------------------------------------------------------------------------
*/
struct registerPORStruct {
    UWORD32    ID;
    UWORD32    address;
    UWORD32    powerOnValue;
    UWORD32    powerOnMask;
    regEnum    regType;
} ;

/* ----------------------------------------------------------------------------
* TYPE:        registerIntegrityStruct
*
* DESCRIPTION:  all required register data held in an array of structure for
*               integrity tests.
*
* NOTE:         None
*
* -----------------------------------------------------------------------------
*/
struct registerIntegrityStruct {
    UWORD32    ID;
    UWORD32    address;
    UWORD32    readMask;
    UWORD32    powerOnValue;
    UWORD32    includeMask;
    UWORD8     regType;
} ;

/* ----------------------------------------------------------------------------
* TYPE:        registerROWOStruct
*
* DESCRIPTION:  all required register data held in an array of structure for
*               read only/write only register tests.
*
* NOTE:         None
*
* -----------------------------------------------------------------------------
*/
struct registerROWOStruct {
    UWORD32    ID;
    UWORD32    address;
    UWORD32    readMask;
    UWORD32    powerOnValue;
    UWORD32    includeMask;
    UWORD8     regType;
} ;

/* ----------------------------------------------------------------------------
* TYPE:        registerExclusivityStruct
*
* DESCRIPTION:  all required register data held in an array of structure for
*               exclusivity tests.
*
* NOTE:         None
*
* -----------------------------------------------------------------------------
*/
struct registerExclusivityStruct {
    UWORD32    ID;
    UWORD8     IOstatus;
    UWORD32    address;
    UWORD32    powerOnValue;
    UWORD32    powerOnMask;
    UWORD32    readMask;
    UWORD32    includeMask;
    UWORD8     regType;
} ;

/*
 * EXPORTED VARIABLES
 *
 */

/* ----------------------------------------------------------------------------
* VARIABLE:     EASIDummy
*
* DESCRIPTION:  A dummy variable used to such that macro _DEBUG_LEVEL_1_EASI
*               compiles when used as an expression inside a macro with the
*               comma operator.
*
* NOTE:         None
*
* -----------------------------------------------------------------------------
*/
extern UWORD32 EASIDummy;

/*
 * EXPORTED FUNCTIONS/MACROS
 *
 */

/* ----------------------------------------------------------------------------
* MACRO:        _DEBUG_LEVEL_1_EASI
*
* DESCRIPTION:  A MACRO which can be used to indicate that a particular beach
*               register access function was called.
*
* NOTE:         We currently dont use this functionality.
*
* -----------------------------------------------------------------------------
*/
#define _DEBUG_LEVEL_1_EASI(easiNum)     ((void)0)

#endif	/* __EASIGLOBAL_H */

