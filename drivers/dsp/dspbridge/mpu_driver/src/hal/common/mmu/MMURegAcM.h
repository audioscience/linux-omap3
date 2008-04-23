/*
 * dspbridge/src/hal/common/mmu/MMURegAcM.h
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


#ifndef _MMU_REG_ACM_H
#define _MMU_REG_ACM_H

#if defined(__cplusplus)
extern "C" {
#endif  /* defined(__cplusplus) */


#include <GlobalTypes.h>
/* #include "BaseAddress.h" */

#include <EasiGlobal.h>
#include <EasiBase.h>

#include "MMUAccInt.h"


/**************************************************************************
* EXPORTED DEFINITIONS
**************************************************************************/

#if defined(USE_LEVEL_1_MACROS)

#define MMUMMU_REVISIONReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_REVISIONReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_REVISION_OFFSET))


/********************************************************************/


#define MMUMMU_REVISIONRevRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_REVISIONRevRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_REVISION_OFFSET)))) &\
      MMU_MMU_REVISION_Rev_MASK) >>\
      MMU_MMU_REVISION_Rev_OFFSET))


/********************************************************************/


#define MMUMMU_REVISIONRevGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_REVISIONRevGet32),\
      (UWORD32)(((var) & MMU_MMU_REVISION_Rev_MASK) >>\
      MMU_MMU_REVISION_Rev_OFFSET))


/********************************************************************/


#define MMUMMU_SYSCONFIGReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_SYSCONFIG_OFFSET))


/********************************************************************/


#define MMUMMU_SYSCONFIGWriteRegister32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_SYSCONFIG_OFFSET;\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGWriteRegister32);\
    WR_MEM_32_VOLATILE((baseAddress)+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_SYSCONFIGClockActivityRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGClockActivityRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSCONFIG_OFFSET)))) &\
      MMU_MMU_SYSCONFIG_ClockActivity_MASK) >>\
      MMU_MMU_SYSCONFIG_ClockActivity_OFFSET))


/********************************************************************/


#define MMUMMU_SYSCONFIGClockActivityGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGClockActivityGet32),\
      (UWORD32)(((var) & MMU_MMU_SYSCONFIG_ClockActivity_MASK) >>\
      MMU_MMU_SYSCONFIG_ClockActivity_OFFSET))


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSCONFIG_OFFSET)))) &\
      MMU_MMU_SYSCONFIG_IdleMode_MASK) >>\
      MMU_MMU_SYSCONFIG_IdleMode_OFFSET))


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeReadIsSfIdle32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeReadIsSfIdle32),\
      (MMUMMU_SYSCONFIGIdleModeSfIdle == (MMUMMU_SYSCONFIGIdleModeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSCONFIG_OFFSET)))) &\
      MMU_MMU_SYSCONFIG_IdleMode_MASK) >>\
      MMU_MMU_SYSCONFIG_IdleMode_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeReadIsSnIdle32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeReadIsSnIdle32),\
      (MMUMMU_SYSCONFIGIdleModeSnIdle == (MMUMMU_SYSCONFIGIdleModeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSCONFIG_OFFSET)))) &\
      MMU_MMU_SYSCONFIG_IdleMode_MASK) >>\
      MMU_MMU_SYSCONFIG_IdleMode_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeReadIsSsIdle32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeReadIsSsIdle32),\
      (MMUMMU_SYSCONFIGIdleModeSsIdle == (MMUMMU_SYSCONFIGIdleModeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSCONFIG_OFFSET)))) &\
      MMU_MMU_SYSCONFIG_IdleMode_MASK) >>\
      MMU_MMU_SYSCONFIG_IdleMode_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeReadIsRes32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeReadIsRes32),\
      (MMUMMU_SYSCONFIGIdleModeRes == (MMUMMU_SYSCONFIGIdleModeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSCONFIG_OFFSET)))) &\
      MMU_MMU_SYSCONFIG_IdleMode_MASK) >>\
      MMU_MMU_SYSCONFIG_IdleMode_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeGet32),\
      (UWORD32)(((var) & MMU_MMU_SYSCONFIG_IdleMode_MASK) >>\
      MMU_MMU_SYSCONFIG_IdleMode_OFFSET))


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeIsSfIdle32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeIsSfIdle32),\
      (MMUMMU_SYSCONFIGIdleModeSfIdle  ==  (MMUMMU_SYSCONFIGIdleModeE)\
      (((var) & MMU_MMU_SYSCONFIG_IdleMode_MASK) >>\
      MMU_MMU_SYSCONFIG_IdleMode_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeIsSnIdle32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeIsSnIdle32),\
      (MMUMMU_SYSCONFIGIdleModeSnIdle  ==  (MMUMMU_SYSCONFIGIdleModeE)\
      (((var) & MMU_MMU_SYSCONFIG_IdleMode_MASK) >>\
      MMU_MMU_SYSCONFIG_IdleMode_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeIsSsIdle32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeIsSsIdle32),\
      (MMUMMU_SYSCONFIGIdleModeSsIdle  ==  (MMUMMU_SYSCONFIGIdleModeE)\
      (((var) & MMU_MMU_SYSCONFIG_IdleMode_MASK) >>\
      MMU_MMU_SYSCONFIG_IdleMode_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeIsRes32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeIsRes32),\
      (MMUMMU_SYSCONFIGIdleModeRes  ==  (MMUMMU_SYSCONFIGIdleModeE)\
	(((var) & MMU_MMU_SYSCONFIG_IdleMode_MASK) >>\
      MMU_MMU_SYSCONFIG_IdleMode_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_SYSCONFIG_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeWrite32);\
    data &= ~(MMU_MMU_SYSCONFIG_IdleMode_MASK);\
    newValue <<= MMU_MMU_SYSCONFIG_IdleMode_OFFSET;\
    newValue &= MMU_MMU_SYSCONFIG_IdleMode_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeWriteSfIdle32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_SYSCONFIG_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_SYSCONFIGIdleModeSfIdle <<\
			       MMU_MMU_SYSCONFIG_IdleMode_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeWriteSfIdle32);\
    data &= ~(MMU_MMU_SYSCONFIG_IdleMode_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeWriteSnIdle32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_SYSCONFIG_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_SYSCONFIGIdleModeSnIdle <<\
			       MMU_MMU_SYSCONFIG_IdleMode_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeWriteSnIdle32);\
    data &= ~(MMU_MMU_SYSCONFIG_IdleMode_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeWriteSsIdle32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_SYSCONFIG_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_SYSCONFIGIdleModeSsIdle <<\
			       MMU_MMU_SYSCONFIG_IdleMode_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeWriteSsIdle32);\
    data &= ~(MMU_MMU_SYSCONFIG_IdleMode_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeWriteRes32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_SYSCONFIG_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_SYSCONFIGIdleModeRes <<\
			       MMU_MMU_SYSCONFIG_IdleMode_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeWriteRes32);\
    data &= ~(MMU_MMU_SYSCONFIG_IdleMode_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_SYSCONFIGIdleModeSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGIdleModeSet32),\
      (((var) & ~(MMU_MMU_SYSCONFIG_IdleMode_MASK)) |\
      (((value) << MMU_MMU_SYSCONFIG_IdleMode_OFFSET) &\
      MMU_MMU_SYSCONFIG_IdleMode_MASK)))


/********************************************************************/


#define MMUMMU_SYSCONFIGSoftResetRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGSoftResetRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSCONFIG_OFFSET)))) &\
      MMU_MMU_SYSCONFIG_SoftReset_MASK) >>\
      MMU_MMU_SYSCONFIG_SoftReset_OFFSET))


/********************************************************************/


#define MMUMMU_SYSCONFIGSoftResetReadIsalways_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_SYSCONFIGSoftResetReadIsalways_r32),\
      (MMUMMU_SYSCONFIGSoftResetalways_r == (MMUMMU_SYSCONFIGSoftResetE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSCONFIG_OFFSET)))) &\
      MMU_MMU_SYSCONFIG_SoftReset_MASK) >>\
      MMU_MMU_SYSCONFIG_SoftReset_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGSoftResetReadIsnever_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGSoftResetReadIsnever_r32),\
      (MMUMMU_SYSCONFIGSoftResetnever_r == (MMUMMU_SYSCONFIGSoftResetE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSCONFIG_OFFSET)))) &\
      MMU_MMU_SYSCONFIG_SoftReset_MASK) >>\
      MMU_MMU_SYSCONFIG_SoftReset_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGSoftResetGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGSoftResetGet32),\
      (UWORD32)(((var) & MMU_MMU_SYSCONFIG_SoftReset_MASK) >>\
	MMU_MMU_SYSCONFIG_SoftReset_OFFSET))


/********************************************************************/


#define MMUMMU_SYSCONFIGSoftResetIsalways_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGSoftResetIsalways_r32),\
      (MMUMMU_SYSCONFIGSoftResetalways_r == (MMUMMU_SYSCONFIGSoftResetE)\
      (((var) & MMU_MMU_SYSCONFIG_SoftReset_MASK) >>\
      MMU_MMU_SYSCONFIG_SoftReset_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGSoftResetIsnever_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGSoftResetIsnever_r32),\
      (MMUMMU_SYSCONFIGSoftResetnever_r == (MMUMMU_SYSCONFIGSoftResetE)\
	(((var) & MMU_MMU_SYSCONFIG_SoftReset_MASK) >>\
      MMU_MMU_SYSCONFIG_SoftReset_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGSoftResetWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_SYSCONFIG_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGSoftResetWrite32);\
    data &= ~(MMU_MMU_SYSCONFIG_SoftReset_MASK);\
    newValue <<= MMU_MMU_SYSCONFIG_SoftReset_OFFSET;\
    newValue &= MMU_MMU_SYSCONFIG_SoftReset_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_SYSCONFIGSoftResetWritenofun_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_SYSCONFIG_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_SYSCONFIGSoftResetnofun_w <<\
			       MMU_MMU_SYSCONFIG_SoftReset_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGSoftResetWritenofun_w32);\
    data &= ~(MMU_MMU_SYSCONFIG_SoftReset_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_SYSCONFIGSoftResetWriterstMode_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_SYSCONFIG_OFFSET;\
    const UWORD32 newValue =\
	(UWORD32)MMUMMU_SYSCONFIGSoftResetrstMode_w <<\
	MMU_MMU_SYSCONFIG_SoftReset_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGSoftResetWriterstMode_w32);\
    data &= ~(MMU_MMU_SYSCONFIG_SoftReset_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_SYSCONFIGSoftResetSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGSoftResetSet32),\
      (((var) & ~(MMU_MMU_SYSCONFIG_SoftReset_MASK)) |\
      (((value) << MMU_MMU_SYSCONFIG_SoftReset_OFFSET) &\
      MMU_MMU_SYSCONFIG_SoftReset_MASK)))


/********************************************************************/


#define MMUMMU_SYSCONFIGAutoIdleRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGAutoIdleRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSCONFIG_OFFSET)))) &\
      MMU_MMU_SYSCONFIG_AutoIdle_MASK) >>\
      MMU_MMU_SYSCONFIG_AutoIdle_OFFSET))


/********************************************************************/


#define MMUMMU_SYSCONFIGAutoIdleReadIsclkfree32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGAutoIdleReadIsclkfree32),\
      (MMUMMU_SYSCONFIGAutoIdleclkfree == (MMUMMU_SYSCONFIGAutoIdleE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSCONFIG_OFFSET)))) &\
      MMU_MMU_SYSCONFIG_AutoIdle_MASK) >>\
      MMU_MMU_SYSCONFIG_AutoIdle_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGAutoIdleReadIsautoClkGate32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_SYSCONFIGAutoIdleReadIsautoClkGate32),\
      (MMUMMU_SYSCONFIGAutoIdleautoClkGate == (MMUMMU_SYSCONFIGAutoIdleE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSCONFIG_OFFSET)))) &\
      MMU_MMU_SYSCONFIG_AutoIdle_MASK) >>\
      MMU_MMU_SYSCONFIG_AutoIdle_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGAutoIdleGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGAutoIdleGet32),\
      (UWORD32)(((var) & MMU_MMU_SYSCONFIG_AutoIdle_MASK) >> \
	MMU_MMU_SYSCONFIG_AutoIdle_OFFSET))


/********************************************************************/


#define MMUMMU_SYSCONFIGAutoIdleIsclkfree32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGAutoIdleIsclkfree32),\
      (MMUMMU_SYSCONFIGAutoIdleclkfree == (MMUMMU_SYSCONFIGAutoIdleE)\
      (((var) & MMU_MMU_SYSCONFIG_AutoIdle_MASK) >>\
      MMU_MMU_SYSCONFIG_AutoIdle_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGAutoIdleIsautoClkGate32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGAutoIdleIsautoClkGate32),\
      (MMUMMU_SYSCONFIGAutoIdleautoClkGate == (MMUMMU_SYSCONFIGAutoIdleE)\
	(((var) & MMU_MMU_SYSCONFIG_AutoIdle_MASK) >>\
      MMU_MMU_SYSCONFIG_AutoIdle_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSCONFIGAutoIdleWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_SYSCONFIG_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGAutoIdleWrite32);\
    data &= ~(MMU_MMU_SYSCONFIG_AutoIdle_MASK);\
    newValue <<= MMU_MMU_SYSCONFIG_AutoIdle_OFFSET;\
    newValue &= MMU_MMU_SYSCONFIG_AutoIdle_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_SYSCONFIGAutoIdleWriteclkfree32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_SYSCONFIG_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_SYSCONFIGAutoIdleclkfree <<\
			       MMU_MMU_SYSCONFIG_AutoIdle_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGAutoIdleWriteclkfree32);\
    data &= ~(MMU_MMU_SYSCONFIG_AutoIdle_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_SYSCONFIGAutoIdleWriteautoClkGate32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_SYSCONFIG_OFFSET;\
    const UWORD32 newValue = \
	(UWORD32)MMUMMU_SYSCONFIGAutoIdleautoClkGate <<\
	MMU_MMU_SYSCONFIG_AutoIdle_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_SYSCONFIGAutoIdleWriteautoClkGate32);\
    data &= ~(MMU_MMU_SYSCONFIG_AutoIdle_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_SYSCONFIGAutoIdleSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSCONFIGAutoIdleSet32),\
      (((var) & ~(MMU_MMU_SYSCONFIG_AutoIdle_MASK)) |\
      (((value) << MMU_MMU_SYSCONFIG_AutoIdle_OFFSET) &\
      MMU_MMU_SYSCONFIG_AutoIdle_MASK)))


/********************************************************************/


#define MMUMMU_SYSSTATUSReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSSTATUSReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_SYSSTATUS_OFFSET))


/********************************************************************/


#define MMUMMU_SYSSTATUSResetDoneRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSSTATUSResetDoneRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSSTATUS_OFFSET)))) &\
      MMU_MMU_SYSSTATUS_ResetDone_MASK) >>\
      MMU_MMU_SYSSTATUS_ResetDone_OFFSET))


/********************************************************************/


#define MMUMMU_SYSSTATUSResetDoneReadIsrstongoing32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_SYSSTATUSResetDoneReadIsrstongoing32),\
      (MMUMMU_SYSSTATUSResetDonerstongoing == (MMUMMU_SYSSTATUSResetDoneE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSSTATUS_OFFSET)))) &\
      MMU_MMU_SYSSTATUS_ResetDone_MASK) >>\
      MMU_MMU_SYSSTATUS_ResetDone_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSSTATUSResetDoneReadIsrstcomp32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSSTATUSResetDoneReadIsrstcomp32),\
      (MMUMMU_SYSSTATUSResetDonerstcomp == (MMUMMU_SYSSTATUSResetDoneE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_SYSSTATUS_OFFSET)))) &\
      MMU_MMU_SYSSTATUS_ResetDone_MASK) >>\
      MMU_MMU_SYSSTATUS_ResetDone_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSSTATUSResetDoneGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSSTATUSResetDoneGet32),\
      (UWORD32)(((var) & MMU_MMU_SYSSTATUS_ResetDone_MASK) >>\
	MMU_MMU_SYSSTATUS_ResetDone_OFFSET))


/********************************************************************/


#define MMUMMU_SYSSTATUSResetDoneIsrstongoing32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSSTATUSResetDoneIsrstongoing32),\
      (MMUMMU_SYSSTATUSResetDonerstongoing  ==  (MMUMMU_SYSSTATUSResetDoneE)\
      (((var) & MMU_MMU_SYSSTATUS_ResetDone_MASK) >>\
      MMU_MMU_SYSSTATUS_ResetDone_OFFSET)))


/********************************************************************/


#define MMUMMU_SYSSTATUSResetDoneIsrstcomp32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_SYSSTATUSResetDoneIsrstcomp32),\
      (MMUMMU_SYSSTATUSResetDonerstcomp  ==  (MMUMMU_SYSSTATUSResetDoneE)\
	   (((var) & MMU_MMU_SYSSTATUS_ResetDone_MASK) >>\
      MMU_MMU_SYSSTATUS_ResetDone_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_IRQSTATUS_OFFSET))


/********************************************************************/


#define MMUMMU_IRQSTATUSWriteRegister32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSWriteRegister32);\
    WR_MEM_32_VOLATILE((baseAddress)+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSMultiHitFaultRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSMultiHitFaultRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_MultiHitFault_MASK) >>\
      MMU_MMU_IRQSTATUS_MultiHitFault_OFFSET))


/********************************************************************/


#define MMUMMU_IRQSTATUSMultiHitFaultReadIsnMHF_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSMultiHitFaultReadIsnMHF_r32),\
    (MMUMMU_IRQSTATUSMultiHitFaultnMHF_r == (MMUMMU_IRQSTATUSMultiHitFaultE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_MultiHitFault_MASK) >>\
      MMU_MMU_IRQSTATUS_MultiHitFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSMultiHitFaultReadIsMHF_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSMultiHitFaultReadIsMHF_r32),\
     (MMUMMU_IRQSTATUSMultiHitFaultMHF_r == (MMUMMU_IRQSTATUSMultiHitFaultE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_MultiHitFault_MASK) >>\
      MMU_MMU_IRQSTATUS_MultiHitFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSMultiHitFaultGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSMultiHitFaultGet32),\
      (UWORD32)(((var) & MMU_MMU_IRQSTATUS_MultiHitFault_MASK) >>\
      MMU_MMU_IRQSTATUS_MultiHitFault_OFFSET))


/********************************************************************/


#define MMUMMU_IRQSTATUSMultiHitFaultIsnMHF_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSMultiHitFaultIsnMHF_r32),\
      (MMUMMU_IRQSTATUSMultiHitFaultnMHF_r == \
      (MMUMMU_IRQSTATUSMultiHitFaultE)(((var) & \
      MMU_MMU_IRQSTATUS_MultiHitFault_MASK) >>\
      MMU_MMU_IRQSTATUS_MultiHitFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSMultiHitFaultIsMHF_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSMultiHitFaultIsMHF_r32),\
     (MMUMMU_IRQSTATUSMultiHitFaultMHF_r == (MMUMMU_IRQSTATUSMultiHitFaultE)\
      (((var) & MMU_MMU_IRQSTATUS_MultiHitFault_MASK) >>\
	    MMU_MMU_IRQSTATUS_MultiHitFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSMultiHitFaultWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSMultiHitFaultWrite32);\
    data &= ~(MMU_MMU_IRQSTATUS_MultiHitFault_MASK);\
    newValue <<= MMU_MMU_IRQSTATUS_MultiHitFault_OFFSET;\
    newValue &= MMU_MMU_IRQSTATUS_MultiHitFault_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSMultiHitFaultWriteMHFstat_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    const UWORD32 newValue = \
      (UWORD32)MMUMMU_IRQSTATUSMultiHitFaultMHFstat_w <<\
      MMU_MMU_IRQSTATUS_MultiHitFault_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSMultiHitFaultWriteMHFstat_w32);\
    data &= ~(MMU_MMU_IRQSTATUS_MultiHitFault_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSMultiHitFaultWriterMHFstat_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    const UWORD32 newValue = \
      (UWORD32)MMUMMU_IRQSTATUSMultiHitFaultrMHFstat_w <<\
      MMU_MMU_IRQSTATUS_MultiHitFault_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSMultiHitFaultWriterMHFstat_w32);\
    data &= ~(MMU_MMU_IRQSTATUS_MultiHitFault_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSMultiHitFaultSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSMultiHitFaultSet32),\
      (((var) & ~(MMU_MMU_IRQSTATUS_MultiHitFault_MASK)) |\
      (((value) << MMU_MMU_IRQSTATUS_MultiHitFault_OFFSET) &\
      MMU_MMU_IRQSTATUS_MultiHitFault_MASK)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTableWalkFaultRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTableWalkFaultRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_TableWalkFault_MASK) >>\
      MMU_MMU_IRQSTATUS_TableWalkFault_OFFSET))


/********************************************************************/


#define MMUMMU_IRQSTATUSTableWalkFaultReadIsnTWF_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSTableWalkFaultReadIsnTWF_r32),\
      (MMUMMU_IRQSTATUSTableWalkFaultnTWF_r == \
      (MMUMMU_IRQSTATUSTableWalkFaultE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_TableWalkFault_MASK) >>\
      MMU_MMU_IRQSTATUS_TableWalkFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTableWalkFaultReadIsTWF_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSTableWalkFaultReadIsTWF_r32),\
      (MMUMMU_IRQSTATUSTableWalkFaultTWF_r == \
      (MMUMMU_IRQSTATUSTableWalkFaultE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_TableWalkFault_MASK) >>\
      MMU_MMU_IRQSTATUS_TableWalkFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTableWalkFaultGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTableWalkFaultGet32),\
      (UWORD32)(((var) & MMU_MMU_IRQSTATUS_TableWalkFault_MASK) >>\
      MMU_MMU_IRQSTATUS_TableWalkFault_OFFSET))


/********************************************************************/


#define MMUMMU_IRQSTATUSTableWalkFaultIsnTWF_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTableWalkFaultIsnTWF_r32),\
      (MMUMMU_IRQSTATUSTableWalkFaultnTWF_r == \
      (MMUMMU_IRQSTATUSTableWalkFaultE)(((var) &\
      MMU_MMU_IRQSTATUS_TableWalkFault_MASK) >>\
      MMU_MMU_IRQSTATUS_TableWalkFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTableWalkFaultIsTWF_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTableWalkFaultIsTWF_r32),\
      (MMUMMU_IRQSTATUSTableWalkFaultTWF_r == \
      (MMUMMU_IRQSTATUSTableWalkFaultE)(((var) &\
      MMU_MMU_IRQSTATUS_TableWalkFault_MASK) >>\
      MMU_MMU_IRQSTATUS_TableWalkFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTableWalkFaultWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTableWalkFaultWrite32);\
    data &= ~(MMU_MMU_IRQSTATUS_TableWalkFault_MASK);\
    newValue <<= MMU_MMU_IRQSTATUS_TableWalkFault_OFFSET;\
    newValue &= MMU_MMU_IRQSTATUS_TableWalkFault_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSTableWalkFaultWriteTWFstat_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    const UWORD32 newValue = \
      (UWORD32)MMUMMU_IRQSTATUSTableWalkFaultTWFstat_w <<\
      MMU_MMU_IRQSTATUS_TableWalkFault_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSTableWalkFaultWriteTWFstat_w32);\
    data &= ~(MMU_MMU_IRQSTATUS_TableWalkFault_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSTableWalkFaultWriterTWFstat_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    const UWORD32 newValue = \
      (UWORD32)MMUMMU_IRQSTATUSTableWalkFaultrTWFstat_w <<\
      MMU_MMU_IRQSTATUS_TableWalkFault_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSTableWalkFaultWriterTWFstat_w32);\
    data &= ~(MMU_MMU_IRQSTATUS_TableWalkFault_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSTableWalkFaultSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTableWalkFaultSet32),\
      (((var) & ~(MMU_MMU_IRQSTATUS_TableWalkFault_MASK)) |\
      (((value) << MMU_MMU_IRQSTATUS_TableWalkFault_OFFSET) &\
      MMU_MMU_IRQSTATUS_TableWalkFault_MASK)))


/********************************************************************/


#define MMUMMU_IRQSTATUSEMUMissRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSEMUMissRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_EMUMiss_MASK) >>\
      MMU_MMU_IRQSTATUS_EMUMiss_OFFSET))


/********************************************************************/


#define MMUMMU_IRQSTATUSEMUMissReadIsnEMUM_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSEMUMissReadIsnEMUM_r32),\
      (MMUMMU_IRQSTATUSEMUMissnEMUM_r == (MMUMMU_IRQSTATUSEMUMissE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_EMUMiss_MASK) >>\
      MMU_MMU_IRQSTATUS_EMUMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSEMUMissReadIsEMUM_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSEMUMissReadIsEMUM_r32),\
      (MMUMMU_IRQSTATUSEMUMissEMUM_r == (MMUMMU_IRQSTATUSEMUMissE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_EMUMiss_MASK) >>\
      MMU_MMU_IRQSTATUS_EMUMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSEMUMissGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSEMUMissGet32),\
      (UWORD32)(((var) & MMU_MMU_IRQSTATUS_EMUMiss_MASK) >>\
	MMU_MMU_IRQSTATUS_EMUMiss_OFFSET))


/********************************************************************/


#define MMUMMU_IRQSTATUSEMUMissIsnEMUM_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSEMUMissIsnEMUM_r32),\
      (MMUMMU_IRQSTATUSEMUMissnEMUM_r == (MMUMMU_IRQSTATUSEMUMissE)\
	(((var) & MMU_MMU_IRQSTATUS_EMUMiss_MASK) >>\
      MMU_MMU_IRQSTATUS_EMUMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSEMUMissIsEMUM_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSEMUMissIsEMUM_r32),\
      (MMUMMU_IRQSTATUSEMUMissEMUM_r == (MMUMMU_IRQSTATUSEMUMissE)\
	(((var) & MMU_MMU_IRQSTATUS_EMUMiss_MASK) >>\
      MMU_MMU_IRQSTATUS_EMUMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSEMUMissWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSEMUMissWrite32);\
    data &= ~(MMU_MMU_IRQSTATUS_EMUMiss_MASK);\
    newValue <<= MMU_MMU_IRQSTATUS_EMUMiss_OFFSET;\
    newValue &= MMU_MMU_IRQSTATUS_EMUMiss_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSEMUMissWriteEstat_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_IRQSTATUSEMUMissEstat_w <<\
      MMU_MMU_IRQSTATUS_EMUMiss_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSEMUMissWriteEstat_w32);\
    data &= ~(MMU_MMU_IRQSTATUS_EMUMiss_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSEMUMissWriterEstat_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_IRQSTATUSEMUMissrEstat_w <<\
      MMU_MMU_IRQSTATUS_EMUMiss_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSEMUMissWriterEstat_w32);\
    data &= ~(MMU_MMU_IRQSTATUS_EMUMiss_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSEMUMissSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSEMUMissSet32),\
      (((var) & ~(MMU_MMU_IRQSTATUS_EMUMiss_MASK)) |\
      (((value) << MMU_MMU_IRQSTATUS_EMUMiss_OFFSET) &\
      MMU_MMU_IRQSTATUS_EMUMiss_MASK)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTranslationFaultRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTranslationFaultRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_TranslationFault_MASK) >>\
      MMU_MMU_IRQSTATUS_TranslationFault_OFFSET))


/********************************************************************/


#define MMUMMU_IRQSTATUSTranslationFaultReadIsnFault_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSTranslationFaultReadIsnFault_r32),\
      (MMUMMU_IRQSTATUSTranslationFaultnFault_r  ==  \
      (MMUMMU_IRQSTATUSTranslationFaultE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_TranslationFault_MASK) >>\
      MMU_MMU_IRQSTATUS_TranslationFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTranslationFaultReadIsFault_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(E\
      ASIL1_MMUMMU_IRQSTATUSTranslationFaultReadIsFault_r32),\
      (MMUMMU_IRQSTATUSTranslationFaultFault_r  ==  \
      (MMUMMU_IRQSTATUSTranslationFaultE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_TranslationFault_MASK) >>\
      MMU_MMU_IRQSTATUS_TranslationFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTranslationFaultGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTranslationFaultGet32),\
      (UWORD32)(((var) & MMU_MMU_IRQSTATUS_TranslationFault_MASK) >>\
      MMU_MMU_IRQSTATUS_TranslationFault_OFFSET))


/********************************************************************/


#define MMUMMU_IRQSTATUSTranslationFaultIsnFault_r32(var)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSTranslationFaultIsnFault_r32),\
      (MMUMMU_IRQSTATUSTranslationFaultnFault_r  ==  \
      (MMUMMU_IRQSTATUSTranslationFaultE)(((var) &\
		MMU_MMU_IRQSTATUS_TranslationFault_MASK) >>\
      MMU_MMU_IRQSTATUS_TranslationFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTranslationFaultIsFault_r32(var)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSTranslationFaultIsFault_r32),\
      (MMUMMU_IRQSTATUSTranslationFaultFault_r  ==  \
      (MMUMMU_IRQSTATUSTranslationFaultE)(((var) &\
	MMU_MMU_IRQSTATUS_TranslationFault_MASK) >>\
      MMU_MMU_IRQSTATUS_TranslationFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTranslationFaultWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTranslationFaultWrite32);\
    data &= ~(MMU_MMU_IRQSTATUS_TranslationFault_MASK);\
    newValue <<= MMU_MMU_IRQSTATUS_TranslationFault_OFFSET;\
    newValue &= MMU_MMU_IRQSTATUS_TranslationFault_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSTranslationFaultWriteFstat_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    const UWORD32 newValue = \
      (UWORD32)MMUMMU_IRQSTATUSTranslationFaultFstat_w <<\
      MMU_MMU_IRQSTATUS_TranslationFault_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSTranslationFaultWriteFstat_w32);\
    data &= ~(MMU_MMU_IRQSTATUS_TranslationFault_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSTranslationFaultWriterFstat_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    const UWORD32 newValue = \
      (UWORD32)MMUMMU_IRQSTATUSTranslationFaultrFstat_w <<\
      MMU_MMU_IRQSTATUS_TranslationFault_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQSTATUSTranslationFaultWriterFstat_w32);\
    data &= ~(MMU_MMU_IRQSTATUS_TranslationFault_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSTranslationFaultSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTranslationFaultSet32),\
      (((var) & ~(MMU_MMU_IRQSTATUS_TranslationFault_MASK)) |\
      (((value) << MMU_MMU_IRQSTATUS_TranslationFault_OFFSET) &\
      MMU_MMU_IRQSTATUS_TranslationFault_MASK)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTLBMissRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTLBMissRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_TLBMiss_MASK) >>\
      MMU_MMU_IRQSTATUS_TLBMiss_OFFSET))


/********************************************************************/


#define MMUMMU_IRQSTATUSTLBMissReadIsnTLBM_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTLBMissReadIsnTLBM_r32),\
      (MMUMMU_IRQSTATUSTLBMissnTLBM_r == (MMUMMU_IRQSTATUSTLBMissE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_TLBMiss_MASK) >>\
      MMU_MMU_IRQSTATUS_TLBMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTLBMissReadIsTLBM_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTLBMissReadIsTLBM_r32),\
      (MMUMMU_IRQSTATUSTLBMissTLBM_r == (MMUMMU_IRQSTATUSTLBMissE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQSTATUS_OFFSET)))) &\
      MMU_MMU_IRQSTATUS_TLBMiss_MASK) >>\
      MMU_MMU_IRQSTATUS_TLBMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTLBMissGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTLBMissGet32),\
      (UWORD32)(((var) & MMU_MMU_IRQSTATUS_TLBMiss_MASK) >>\
      MMU_MMU_IRQSTATUS_TLBMiss_OFFSET))


/********************************************************************/


#define MMUMMU_IRQSTATUSTLBMissIsnTLBM_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTLBMissIsnTLBM_r32),\
      (MMUMMU_IRQSTATUSTLBMissnTLBM_r == (MMUMMU_IRQSTATUSTLBMissE)\
	(((var) & MMU_MMU_IRQSTATUS_TLBMiss_MASK) >>\
      MMU_MMU_IRQSTATUS_TLBMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTLBMissIsTLBM_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTLBMissIsTLBM_r32),\
      (MMUMMU_IRQSTATUSTLBMissTLBM_r == (MMUMMU_IRQSTATUSTLBMissE)\
	(((var) & MMU_MMU_IRQSTATUS_TLBMiss_MASK) >>\
      MMU_MMU_IRQSTATUS_TLBMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQSTATUSTLBMissWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTLBMissWrite32);\
    data &= ~(MMU_MMU_IRQSTATUS_TLBMiss_MASK);\
    newValue <<= MMU_MMU_IRQSTATUS_TLBMiss_OFFSET;\
    newValue &= MMU_MMU_IRQSTATUS_TLBMiss_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSTLBMissWriteMstat_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_IRQSTATUSTLBMissMstat_w <<\
      MMU_MMU_IRQSTATUS_TLBMiss_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTLBMissWriteMstat_w32);\
    data &= ~(MMU_MMU_IRQSTATUS_TLBMiss_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSTLBMissWriterMstat_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQSTATUS_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_IRQSTATUSTLBMissrMstat_w <<\
      MMU_MMU_IRQSTATUS_TLBMiss_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTLBMissWriterMstat_w32);\
    data &= ~(MMU_MMU_IRQSTATUS_TLBMiss_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQSTATUSTLBMissSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQSTATUSTLBMissSet32),\
      (((var) & ~(MMU_MMU_IRQSTATUS_TLBMiss_MASK)) |\
      (((value) << MMU_MMU_IRQSTATUS_TLBMiss_OFFSET) &\
      MMU_MMU_IRQSTATUS_TLBMiss_MASK)))


/********************************************************************/


#define MMUMMU_IRQENABLEReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_IRQENABLE_OFFSET))


/********************************************************************/


#define MMUMMU_IRQENABLEWriteRegister32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEWriteRegister32);\
    WR_MEM_32_VOLATILE((baseAddress)+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_IRQENABLEMultiHitFaultRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEMultiHitFaultRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_MultiHitFault_MASK) >>\
      MMU_MMU_IRQENABLE_MultiHitFault_OFFSET))


/********************************************************************/


#define MMUMMU_IRQENABLEMultiHitFaultReadIsMHFltMAsk32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLEMultiHitFaultReadIsMHFltMAsk32),\
      (MMUMMU_IRQENABLEMultiHitFaultMHFltMAsk == \
      (MMUMMU_IRQENABLEMultiHitFaultE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_MultiHitFault_MASK) >>\
      MMU_MMU_IRQENABLE_MultiHitFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLEMultiHitFaultReadIsMHFltGInt32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLEMultiHitFaultReadIsMHFltGInt32),\
      (MMUMMU_IRQENABLEMultiHitFaultMHFltGInt  ==  \
      (MMUMMU_IRQENABLEMultiHitFaultE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_MultiHitFault_MASK) >>\
      MMU_MMU_IRQENABLE_MultiHitFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLEMultiHitFaultGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEMultiHitFaultGet32),\
      (UWORD32)(((var) & MMU_MMU_IRQENABLE_MultiHitFault_MASK) >>\
      MMU_MMU_IRQENABLE_MultiHitFault_OFFSET))


/********************************************************************/


#define MMUMMU_IRQENABLEMultiHitFaultIsMHFltMAsk32(var)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLEMultiHitFaultIsMHFltMAsk32),\
      (MMUMMU_IRQENABLEMultiHitFaultMHFltMAsk  ==  \
      (MMUMMU_IRQENABLEMultiHitFaultE)(((var) &\
      MMU_MMU_IRQENABLE_MultiHitFault_MASK) >>\
      MMU_MMU_IRQENABLE_MultiHitFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLEMultiHitFaultIsMHFltGInt32(var)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLEMultiHitFaultIsMHFltGInt32),\
      (MMUMMU_IRQENABLEMultiHitFaultMHFltGInt  ==  \
      (MMUMMU_IRQENABLEMultiHitFaultE)(((var) &\
      MMU_MMU_IRQENABLE_MultiHitFault_MASK) >>\
      MMU_MMU_IRQENABLE_MultiHitFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLEMultiHitFaultWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEMultiHitFaultWrite32);\
    data &= ~(MMU_MMU_IRQENABLE_MultiHitFault_MASK);\
    newValue <<= MMU_MMU_IRQENABLE_MultiHitFault_OFFSET;\
    newValue &= MMU_MMU_IRQENABLE_MultiHitFault_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_IRQENABLEMultiHitFaultWriteMHFltMAsk32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    const UWORD32 newValue = \
	(UWORD32)MMUMMU_IRQENABLEMultiHitFaultMHFltMAsk <<\
	MMU_MMU_IRQENABLE_MultiHitFault_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLEMultiHitFaultWriteMHFltMAsk32);\
    data &= ~(MMU_MMU_IRQENABLE_MultiHitFault_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQENABLEMultiHitFaultWriteMHFltGInt32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    const UWORD32 newValue = \
      (UWORD32)MMUMMU_IRQENABLEMultiHitFaultMHFltGInt <<\
      MMU_MMU_IRQENABLE_MultiHitFault_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLEMultiHitFaultWriteMHFltGInt32);\
    data &= ~(MMU_MMU_IRQENABLE_MultiHitFault_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQENABLEMultiHitFaultSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEMultiHitFaultSet32),\
      (((var) & ~(MMU_MMU_IRQENABLE_MultiHitFault_MASK)) |\
      (((value) << MMU_MMU_IRQENABLE_MultiHitFault_OFFSET) &\
      MMU_MMU_IRQENABLE_MultiHitFault_MASK)))


/********************************************************************/


#define MMUMMU_IRQENABLETableWalkFaultRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETableWalkFaultRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_TableWalkFault_MASK) >>\
      MMU_MMU_IRQENABLE_TableWalkFault_OFFSET))


/********************************************************************/


#define MMUMMU_IRQENABLETableWalkFaultReadIsTWLFltMAsk32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETableWalkFaultReadIsTWLFltMAsk32),\
      (MMUMMU_IRQENABLETableWalkFaultTWLFltMAsk  ==  \
      (MMUMMU_IRQENABLETableWalkFaultE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_TableWalkFault_MASK) >>\
      MMU_MMU_IRQENABLE_TableWalkFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLETableWalkFaultReadIsTWLFltGInt32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETableWalkFaultReadIsTWLFltGInt32),\
      (MMUMMU_IRQENABLETableWalkFaultTWLFltGInt  ==  \
      (MMUMMU_IRQENABLETableWalkFaultE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_TableWalkFault_MASK) >>\
      MMU_MMU_IRQENABLE_TableWalkFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLETableWalkFaultGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETableWalkFaultGet32),\
      (UWORD32)(((var) & MMU_MMU_IRQENABLE_TableWalkFault_MASK) >>\
      MMU_MMU_IRQENABLE_TableWalkFault_OFFSET))


/********************************************************************/


#define MMUMMU_IRQENABLETableWalkFaultIsTWLFltMAsk32(var)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETableWalkFaultIsTWLFltMAsk32),\
      (MMUMMU_IRQENABLETableWalkFaultTWLFltMAsk  ==  \
	(MMUMMU_IRQENABLETableWalkFaultE)(((var) &\
      MMU_MMU_IRQENABLE_TableWalkFault_MASK) >>\
      MMU_MMU_IRQENABLE_TableWalkFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLETableWalkFaultIsTWLFltGInt32(var)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETableWalkFaultIsTWLFltGInt32),\
      (MMUMMU_IRQENABLETableWalkFaultTWLFltGInt  ==  \
	(MMUMMU_IRQENABLETableWalkFaultE)(((var) &\
      MMU_MMU_IRQENABLE_TableWalkFault_MASK) >>\
      MMU_MMU_IRQENABLE_TableWalkFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLETableWalkFaultWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETableWalkFaultWrite32);\
    data &= ~(MMU_MMU_IRQENABLE_TableWalkFault_MASK);\
    newValue <<= MMU_MMU_IRQENABLE_TableWalkFault_OFFSET;\
    newValue &= MMU_MMU_IRQENABLE_TableWalkFault_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_IRQENABLETableWalkFaultWriteTWLFltMAsk32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    const UWORD32 newValue = \
      (UWORD32)MMUMMU_IRQENABLETableWalkFaultTWLFltMAsk <<\
      MMU_MMU_IRQENABLE_TableWalkFault_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETableWalkFaultWriteTWLFltMAsk32);\
    data &= ~(MMU_MMU_IRQENABLE_TableWalkFault_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQENABLETableWalkFaultWriteTWLFltGInt32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    const UWORD32 newValue = \
      (UWORD32)MMUMMU_IRQENABLETableWalkFaultTWLFltGInt <<\
      MMU_MMU_IRQENABLE_TableWalkFault_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETableWalkFaultWriteTWLFltGInt32);\
    data &= ~(MMU_MMU_IRQENABLE_TableWalkFault_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQENABLETableWalkFaultSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETableWalkFaultSet32),\
      (((var) & ~(MMU_MMU_IRQENABLE_TableWalkFault_MASK)) |\
      (((value) << MMU_MMU_IRQENABLE_TableWalkFault_OFFSET) &\
      MMU_MMU_IRQENABLE_TableWalkFault_MASK)))


/********************************************************************/


#define MMUMMU_IRQENABLEEMUMissRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEEMUMissRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_EMUMiss_MASK) >>\
      MMU_MMU_IRQENABLE_EMUMiss_OFFSET))


/********************************************************************/


#define MMUMMU_IRQENABLEEMUMissReadIsEMUMFltMask32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLEEMUMissReadIsEMUMFltMask32),\
      (MMUMMU_IRQENABLEEMUMissEMUMFltMask == (MMUMMU_IRQENABLEEMUMissE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_EMUMiss_MASK) >>\
      MMU_MMU_IRQENABLE_EMUMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLEEMUMissReadIsEMUMFltGInt32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLEEMUMissReadIsEMUMFltGInt32),\
      (MMUMMU_IRQENABLEEMUMissEMUMFltGInt == (MMUMMU_IRQENABLEEMUMissE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_EMUMiss_MASK) >>\
      MMU_MMU_IRQENABLE_EMUMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLEEMUMissGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEEMUMissGet32),\
      (UWORD32)(((var) & MMU_MMU_IRQENABLE_EMUMiss_MASK) >>\
      MMU_MMU_IRQENABLE_EMUMiss_OFFSET))


/********************************************************************/


#define MMUMMU_IRQENABLEEMUMissIsEMUMFltMask32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEEMUMissIsEMUMFltMask32),\
      (MMUMMU_IRQENABLEEMUMissEMUMFltMask  ==  \
	(MMUMMU_IRQENABLEEMUMissE)(((var) &\
      MMU_MMU_IRQENABLE_EMUMiss_MASK) >>\
      MMU_MMU_IRQENABLE_EMUMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLEEMUMissIsEMUMFltGInt32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEEMUMissIsEMUMFltGInt32),\
      (MMUMMU_IRQENABLEEMUMissEMUMFltGInt  ==  \
	(MMUMMU_IRQENABLEEMUMissE)(((var) &\
      MMU_MMU_IRQENABLE_EMUMiss_MASK) >>\
      MMU_MMU_IRQENABLE_EMUMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLEEMUMissWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEEMUMissWrite32);\
    data &= ~(MMU_MMU_IRQENABLE_EMUMiss_MASK);\
    newValue <<= MMU_MMU_IRQENABLE_EMUMiss_OFFSET;\
    newValue &= MMU_MMU_IRQENABLE_EMUMiss_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_IRQENABLEEMUMissWriteEMUMFltMask32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    const UWORD32 newValue = \
	(UWORD32)MMUMMU_IRQENABLEEMUMissEMUMFltMask <<\
      MMU_MMU_IRQENABLE_EMUMiss_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEEMUMissWriteEMUMFltMask32);\
    data &= ~(MMU_MMU_IRQENABLE_EMUMiss_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQENABLEEMUMissWriteEMUMFltGInt32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    const UWORD32 newValue = \
	(UWORD32)MMUMMU_IRQENABLEEMUMissEMUMFltGInt <<\
      MMU_MMU_IRQENABLE_EMUMiss_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEEMUMissWriteEMUMFltGInt32);\
    data &= ~(MMU_MMU_IRQENABLE_EMUMiss_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQENABLEEMUMissSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLEEMUMissSet32),\
      (((var) & ~(MMU_MMU_IRQENABLE_EMUMiss_MASK)) |\
      (((value) << MMU_MMU_IRQENABLE_EMUMiss_OFFSET) &\
      MMU_MMU_IRQENABLE_EMUMiss_MASK)))


/********************************************************************/


#define MMUMMU_IRQENABLETranslationFaultRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETranslationFaultRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_TranslationFault_MASK) >>\
      MMU_MMU_IRQENABLE_TranslationFault_OFFSET))


/********************************************************************/


#define MMUMMU_IRQENABLETranslationFaultReadIsTranFltMask32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETranslationFaultReadIsTranFltMask32),\
      (MMUMMU_IRQENABLETranslationFaultTranFltMask  ==  \
      (MMUMMU_IRQENABLETranslationFaultE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_TranslationFault_MASK) >>\
      MMU_MMU_IRQENABLE_TranslationFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLETranslationFaultReadIsTranFltGInt32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETranslationFaultReadIsTranFltGInt32),\
      (MMUMMU_IRQENABLETranslationFaultTranFltGInt  ==  \
      (MMUMMU_IRQENABLETranslationFaultE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_TranslationFault_MASK) >>\
      MMU_MMU_IRQENABLE_TranslationFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLETranslationFaultGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETranslationFaultGet32),\
      (UWORD32)(((var) & MMU_MMU_IRQENABLE_TranslationFault_MASK) >>\
      MMU_MMU_IRQENABLE_TranslationFault_OFFSET))


/********************************************************************/


#define MMUMMU_IRQENABLETranslationFaultIsTranFltMask32(var)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETranslationFaultIsTranFltMask32),\
      (MMUMMU_IRQENABLETranslationFaultTranFltMask == \
      (MMUMMU_IRQENABLETranslationFaultE)(((var) \
	 & MMU_MMU_IRQENABLE_TranslationFault_MASK) >>\
      MMU_MMU_IRQENABLE_TranslationFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLETranslationFaultIsTranFltGInt32(var)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETranslationFaultIsTranFltGInt32),\
      (MMUMMU_IRQENABLETranslationFaultTranFltGInt == \
	(MMUMMU_IRQENABLETranslationFaultE)(((var)&\
      MMU_MMU_IRQENABLE_TranslationFault_MASK) >>\
      MMU_MMU_IRQENABLE_TranslationFault_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLETranslationFaultWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETranslationFaultWrite32);\
    data &= ~(MMU_MMU_IRQENABLE_TranslationFault_MASK);\
    newValue <<= MMU_MMU_IRQENABLE_TranslationFault_OFFSET;\
    newValue &= MMU_MMU_IRQENABLE_TranslationFault_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_IRQENABLETranslationFaultWriteTranFltMask32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    const UWORD32 newValue = (UWORD32)\
      MMUMMU_IRQENABLETranslationFaultTranFltMask <<\
      MMU_MMU_IRQENABLE_TranslationFault_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETranslationFaultWriteTranFltMask32);\
    data &= ~(MMU_MMU_IRQENABLE_TranslationFault_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQENABLETranslationFaultWriteTranFltGInt32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    const UWORD32 newValue = (UWORD32)\
      MMUMMU_IRQENABLETranslationFaultTranFltGInt <<\
      MMU_MMU_IRQENABLE_TranslationFault_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETranslationFaultWriteTranFltGInt32);\
    data &= ~(MMU_MMU_IRQENABLE_TranslationFault_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQENABLETranslationFaultSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETranslationFaultSet32),\
      (((var) & ~(MMU_MMU_IRQENABLE_TranslationFault_MASK)) |\
      (((value) << MMU_MMU_IRQENABLE_TranslationFault_OFFSET) &\
      MMU_MMU_IRQENABLE_TranslationFault_MASK)))


/********************************************************************/


#define MMUMMU_IRQENABLETLBMissRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETLBMissRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_TLBMiss_MASK) >>\
      MMU_MMU_IRQENABLE_TLBMiss_OFFSET))


/********************************************************************/


#define MMUMMU_IRQENABLETLBMissReadIsTrMissIntM32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETLBMissReadIsTrMissIntM32),\
      (MMUMMU_IRQENABLETLBMissTrMissIntM == (MMUMMU_IRQENABLETLBMissE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_TLBMiss_MASK) >>\
      MMU_MMU_IRQENABLE_TLBMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLETLBMissReadIsTrMissGInt32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_IRQENABLETLBMissReadIsTrMissGInt32),\
      (MMUMMU_IRQENABLETLBMissTrMissGInt == (MMUMMU_IRQENABLETLBMissE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_IRQENABLE_OFFSET)))) &\
      MMU_MMU_IRQENABLE_TLBMiss_MASK) >>\
      MMU_MMU_IRQENABLE_TLBMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLETLBMissGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETLBMissGet32),\
      (UWORD32)(((var) & MMU_MMU_IRQENABLE_TLBMiss_MASK) >>\
      MMU_MMU_IRQENABLE_TLBMiss_OFFSET))


/********************************************************************/


#define MMUMMU_IRQENABLETLBMissIsTrMissIntM32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETLBMissIsTrMissIntM32),\
      (MMUMMU_IRQENABLETLBMissTrMissIntM == (MMUMMU_IRQENABLETLBMissE)\
	(((var) & MMU_MMU_IRQENABLE_TLBMiss_MASK) >>\
      MMU_MMU_IRQENABLE_TLBMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLETLBMissIsTrMissGInt32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETLBMissIsTrMissGInt32),\
      (MMUMMU_IRQENABLETLBMissTrMissGInt == (MMUMMU_IRQENABLETLBMissE)\
	(((var) & MMU_MMU_IRQENABLE_TLBMiss_MASK) >>\
      MMU_MMU_IRQENABLE_TLBMiss_OFFSET)))


/********************************************************************/


#define MMUMMU_IRQENABLETLBMissWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETLBMissWrite32);\
    data &= ~(MMU_MMU_IRQENABLE_TLBMiss_MASK);\
    newValue <<= MMU_MMU_IRQENABLE_TLBMiss_OFFSET;\
    newValue &= MMU_MMU_IRQENABLE_TLBMiss_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_IRQENABLETLBMissWriteTrMissIntM32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_IRQENABLETLBMissTrMissIntM <<\
      MMU_MMU_IRQENABLE_TLBMiss_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETLBMissWriteTrMissIntM32);\
    data &= ~(MMU_MMU_IRQENABLE_TLBMiss_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQENABLETLBMissWriteTrMissGInt32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_IRQENABLE_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_IRQENABLETLBMissTrMissGInt <<\
      MMU_MMU_IRQENABLE_TLBMiss_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETLBMissWriteTrMissGInt32);\
    data &= ~(MMU_MMU_IRQENABLE_TLBMiss_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_IRQENABLETLBMissSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_IRQENABLETLBMissSet32),\
      (((var) & ~(MMU_MMU_IRQENABLE_TLBMiss_MASK)) |\
      (((value) << MMU_MMU_IRQENABLE_TLBMiss_OFFSET) &\
      MMU_MMU_IRQENABLE_TLBMiss_MASK)))


/********************************************************************/


#define MMUMMU_WALKING_STReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_WALKING_STReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_WALKING_ST_OFFSET))


/********************************************************************/


#define MMUMMU_WALKING_STWriteRegister32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_WALKING_ST_OFFSET;\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_WALKING_STWriteRegister32);\
    WR_MEM_32_VOLATILE((baseAddress)+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_WALKING_STTWLRunningRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_WALKING_STTWLRunningRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_WALKING_ST_OFFSET))))\
      & MMU_MMU_WALKING_ST_TWLRunning_MASK) >>\
      MMU_MMU_WALKING_ST_TWLRunning_OFFSET))


/********************************************************************/


#define MMUMMU_WALKING_STTWLRunningReadIsTWLComp32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_WALKING_STTWLRunningReadIsTWLComp32),\
      (MMUMMU_WALKING_STTWLRunningTWLComp == (MMUMMU_WALKING_STTWLRunningE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_WALKING_ST_OFFSET))))\
      & MMU_MMU_WALKING_ST_TWLRunning_MASK) >>\
      MMU_MMU_WALKING_ST_TWLRunning_OFFSET)))


/********************************************************************/


#define MMUMMU_WALKING_STTWLRunningReadIsTWLRun32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_WALKING_STTWLRunningReadIsTWLRun32),\
      (MMUMMU_WALKING_STTWLRunningTWLRun == (MMUMMU_WALKING_STTWLRunningE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+\
      (MMU_MMU_WALKING_ST_OFFSET)))) &\
      MMU_MMU_WALKING_ST_TWLRunning_MASK) >>\
      MMU_MMU_WALKING_ST_TWLRunning_OFFSET)))


/********************************************************************/


#define MMUMMU_WALKING_STTWLRunningGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_WALKING_STTWLRunningGet32),\
      (UWORD32)(((var) & MMU_MMU_WALKING_ST_TWLRunning_MASK) >>\
	MMU_MMU_WALKING_ST_TWLRunning_OFFSET))


/********************************************************************/


#define MMUMMU_WALKING_STTWLRunningIsTWLComp32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_WALKING_STTWLRunningIsTWLComp32),\
      (MMUMMU_WALKING_STTWLRunningTWLComp == (MMUMMU_WALKING_STTWLRunningE)\
      (((var) & MMU_MMU_WALKING_ST_TWLRunning_MASK) >>\
      MMU_MMU_WALKING_ST_TWLRunning_OFFSET)))


/********************************************************************/


#define MMUMMU_WALKING_STTWLRunningIsTWLRun32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_WALKING_STTWLRunningIsTWLRun32),\
      (MMUMMU_WALKING_STTWLRunningTWLRun == \
	(MMUMMU_WALKING_STTWLRunningE)\
      (((var) & MMU_MMU_WALKING_ST_TWLRunning_MASK) >>\
      MMU_MMU_WALKING_ST_TWLRunning_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_CNTL_OFFSET))


/********************************************************************/


#define MMUMMU_CNTLWriteRegister32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_CNTL_OFFSET;\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLWriteRegister32);\
    WR_MEM_32_VOLATILE((baseAddress)+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_CNTLEmuTLBUpdateRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLEmuTLBUpdateRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CNTL_OFFSET)))) &\
      MMU_MMU_CNTL_EmuTLBUpdate_MASK) >>\
      MMU_MMU_CNTL_EmuTLBUpdate_OFFSET))


/********************************************************************/


#define MMUMMU_CNTLEmuTLBUpdateReadIsEMUdis32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLEmuTLBUpdateReadIsEMUdis32),\
      (MMUMMU_CNTLEmuTLBUpdateEMUdis == (MMUMMU_CNTLEmuTLBUpdateE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CNTL_OFFSET)))) &\
      MMU_MMU_CNTL_EmuTLBUpdate_MASK) >>\
      MMU_MMU_CNTL_EmuTLBUpdate_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLEmuTLBUpdateReadIsEMUen32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLEmuTLBUpdateReadIsEMUen32),\
      (MMUMMU_CNTLEmuTLBUpdateEMUen == (MMUMMU_CNTLEmuTLBUpdateE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CNTL_OFFSET)))) &\
      MMU_MMU_CNTL_EmuTLBUpdate_MASK) >>\
      MMU_MMU_CNTL_EmuTLBUpdate_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLEmuTLBUpdateGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLEmuTLBUpdateGet32),\
      (UWORD32)(((var) & MMU_MMU_CNTL_EmuTLBUpdate_MASK) >>\
      MMU_MMU_CNTL_EmuTLBUpdate_OFFSET))


/********************************************************************/


#define MMUMMU_CNTLEmuTLBUpdateIsEMUdis32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLEmuTLBUpdateIsEMUdis32),\
      (MMUMMU_CNTLEmuTLBUpdateEMUdis == (MMUMMU_CNTLEmuTLBUpdateE)\
	(((var) & MMU_MMU_CNTL_EmuTLBUpdate_MASK) >>\
      MMU_MMU_CNTL_EmuTLBUpdate_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLEmuTLBUpdateIsEMUen32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLEmuTLBUpdateIsEMUen32),\
      (MMUMMU_CNTLEmuTLBUpdateEMUen == (MMUMMU_CNTLEmuTLBUpdateE)\
	(((var) & MMU_MMU_CNTL_EmuTLBUpdate_MASK) >>\
      MMU_MMU_CNTL_EmuTLBUpdate_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLEmuTLBUpdateWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_CNTL_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLEmuTLBUpdateWrite32);\
    data &= ~(MMU_MMU_CNTL_EmuTLBUpdate_MASK);\
    newValue <<= MMU_MMU_CNTL_EmuTLBUpdate_OFFSET;\
    newValue &= MMU_MMU_CNTL_EmuTLBUpdate_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_CNTLEmuTLBUpdateWriteEMUdis32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CNTL_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CNTLEmuTLBUpdateEMUdis <<\
      MMU_MMU_CNTL_EmuTLBUpdate_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLEmuTLBUpdateWriteEMUdis32);\
    data &= ~(MMU_MMU_CNTL_EmuTLBUpdate_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CNTLEmuTLBUpdateWriteEMUen32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CNTL_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CNTLEmuTLBUpdateEMUen <<\
      MMU_MMU_CNTL_EmuTLBUpdate_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLEmuTLBUpdateWriteEMUen32);\
    data &= ~(MMU_MMU_CNTL_EmuTLBUpdate_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CNTLEmuTLBUpdateSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLEmuTLBUpdateSet32),\
      (((var) & ~(MMU_MMU_CNTL_EmuTLBUpdate_MASK)) |\
      (((value) << MMU_MMU_CNTL_EmuTLBUpdate_OFFSET) &\
      MMU_MMU_CNTL_EmuTLBUpdate_MASK)))


/********************************************************************/


#define MMUMMU_CNTLTWLEnableRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLTWLEnableRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CNTL_OFFSET)))) &\
      MMU_MMU_CNTL_TWLEnable_MASK) >>\
      MMU_MMU_CNTL_TWLEnable_OFFSET))


/********************************************************************/


#define MMUMMU_CNTLTWLEnableReadIsTWLdis32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLTWLEnableReadIsTWLdis32),\
      (MMUMMU_CNTLTWLEnableTWLdis == (MMUMMU_CNTLTWLEnableE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CNTL_OFFSET)))) &\
      MMU_MMU_CNTL_TWLEnable_MASK) >>\
      MMU_MMU_CNTL_TWLEnable_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLTWLEnableReadIsTWLen32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLTWLEnableReadIsTWLen32),\
      (MMUMMU_CNTLTWLEnableTWLen == (MMUMMU_CNTLTWLEnableE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CNTL_OFFSET)))) &\
      MMU_MMU_CNTL_TWLEnable_MASK) >>\
      MMU_MMU_CNTL_TWLEnable_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLTWLEnableGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLTWLEnableGet32),\
      (UWORD32)(((var) & MMU_MMU_CNTL_TWLEnable_MASK) >>\
      MMU_MMU_CNTL_TWLEnable_OFFSET))


/********************************************************************/


#define MMUMMU_CNTLTWLEnableIsTWLdis32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLTWLEnableIsTWLdis32),\
      (MMUMMU_CNTLTWLEnableTWLdis == (MMUMMU_CNTLTWLEnableE)\
	(((var) & MMU_MMU_CNTL_TWLEnable_MASK) >>\
      MMU_MMU_CNTL_TWLEnable_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLTWLEnableIsTWLen32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLTWLEnableIsTWLen32),\
      (MMUMMU_CNTLTWLEnableTWLen == (MMUMMU_CNTLTWLEnableE)\
	(((var) & MMU_MMU_CNTL_TWLEnable_MASK) >>\
      MMU_MMU_CNTL_TWLEnable_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLTWLEnableWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_CNTL_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLTWLEnableWrite32);\
    data &= ~(MMU_MMU_CNTL_TWLEnable_MASK);\
    newValue <<= MMU_MMU_CNTL_TWLEnable_OFFSET;\
    newValue &= MMU_MMU_CNTL_TWLEnable_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_CNTLTWLEnableWriteTWLdis32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CNTL_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CNTLTWLEnableTWLdis <<\
      MMU_MMU_CNTL_TWLEnable_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLTWLEnableWriteTWLdis32);\
    data &= ~(MMU_MMU_CNTL_TWLEnable_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CNTLTWLEnableWriteTWLen32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CNTL_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CNTLTWLEnableTWLen <<\
      MMU_MMU_CNTL_TWLEnable_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLTWLEnableWriteTWLen32);\
    data &= ~(MMU_MMU_CNTL_TWLEnable_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CNTLTWLEnableSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLTWLEnableSet32),\
      (((var) & ~(MMU_MMU_CNTL_TWLEnable_MASK)) |\
      (((value) << MMU_MMU_CNTL_TWLEnable_OFFSET) &\
      MMU_MMU_CNTL_TWLEnable_MASK)))


/********************************************************************/


#define MMUMMU_CNTLMMUEnableRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLMMUEnableRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CNTL_OFFSET)))) &\
      MMU_MMU_CNTL_MMUEnable_MASK) >>\
      MMU_MMU_CNTL_MMUEnable_OFFSET))


/********************************************************************/


#define MMUMMU_CNTLMMUEnableReadIsMMUdis32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLMMUEnableReadIsMMUdis32),\
      (MMUMMU_CNTLMMUEnableMMUdis == (MMUMMU_CNTLMMUEnableE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CNTL_OFFSET)))) &\
      MMU_MMU_CNTL_MMUEnable_MASK) >>\
      MMU_MMU_CNTL_MMUEnable_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLMMUEnableReadIsMMUen32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLMMUEnableReadIsMMUen32),\
      (MMUMMU_CNTLMMUEnableMMUen == (MMUMMU_CNTLMMUEnableE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CNTL_OFFSET)))) &\
      MMU_MMU_CNTL_MMUEnable_MASK) >>\
      MMU_MMU_CNTL_MMUEnable_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLMMUEnableGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLMMUEnableGet32),\
      (UWORD32)(((var) & MMU_MMU_CNTL_MMUEnable_MASK) >>\
      MMU_MMU_CNTL_MMUEnable_OFFSET))


/********************************************************************/


#define MMUMMU_CNTLMMUEnableIsMMUdis32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLMMUEnableIsMMUdis32),\
      (MMUMMU_CNTLMMUEnableMMUdis == (MMUMMU_CNTLMMUEnableE)\
      (((var) & MMU_MMU_CNTL_MMUEnable_MASK) >>\
      MMU_MMU_CNTL_MMUEnable_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLMMUEnableIsMMUen32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLMMUEnableIsMMUen32),\
      (MMUMMU_CNTLMMUEnableMMUen == (MMUMMU_CNTLMMUEnableE)\
	(((var) & MMU_MMU_CNTL_MMUEnable_MASK) >>\
      MMU_MMU_CNTL_MMUEnable_OFFSET)))


/********************************************************************/


#define MMUMMU_CNTLMMUEnableWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_CNTL_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLMMUEnableWrite32);\
    data &= ~(MMU_MMU_CNTL_MMUEnable_MASK);\
    newValue <<= MMU_MMU_CNTL_MMUEnable_OFFSET;\
    newValue &= MMU_MMU_CNTL_MMUEnable_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_CNTLMMUEnableWriteMMUdis32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CNTL_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CNTLMMUEnableMMUdis <<\
      MMU_MMU_CNTL_MMUEnable_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLMMUEnableWriteMMUdis32);\
    data &= ~(MMU_MMU_CNTL_MMUEnable_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CNTLMMUEnableWriteMMUen32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CNTL_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CNTLMMUEnableMMUen <<\
      MMU_MMU_CNTL_MMUEnable_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLMMUEnableWriteMMUen32);\
    data &= ~(MMU_MMU_CNTL_MMUEnable_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CNTLMMUEnableSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CNTLMMUEnableSet32),\
      (((var) & ~(MMU_MMU_CNTL_MMUEnable_MASK)) |\
      (((value) << MMU_MMU_CNTL_MMUEnable_OFFSET) &\
      MMU_MMU_CNTL_MMUEnable_MASK)))


/********************************************************************/


#define MMUMMU_FAULT_ADReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_FAULT_ADReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_FAULT_AD_OFFSET))


/********************************************************************/


#define MMUMMU_FAULT_ADFaultAddressRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_FAULT_ADFaultAddressRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_FAULT_AD_OFFSET)))) &\
      MMU_MMU_FAULT_AD_FaultAddress_MASK) >>\
      MMU_MMU_FAULT_AD_FaultAddress_OFFSET))


/********************************************************************/


#define MMUMMU_FAULT_ADFaultAddressGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_FAULT_ADFaultAddressGet32),\
      (UWORD32)(((var) & MMU_MMU_FAULT_AD_FaultAddress_MASK) >>\
	MMU_MMU_FAULT_AD_FaultAddress_OFFSET))


/********************************************************************/


#define MMUMMU_TTBReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_TTBReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_TTB_OFFSET))


/********************************************************************/


#define MMUMMU_TTBWriteRegister32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_TTB_OFFSET;\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_TTBWriteRegister32);\
    WR_MEM_32_VOLATILE((baseAddress)+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_TTBTTBAddressRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_TTBTTBAddressRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_TTB_OFFSET)))) &\
      MMU_MMU_TTB_TTBAddress_MASK) >>\
      MMU_MMU_TTB_TTBAddress_OFFSET))


/********************************************************************/


#define MMUMMU_TTBTTBAddressGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_TTBTTBAddressGet32),\
      (UWORD32)(((var) & MMU_MMU_TTB_TTBAddress_MASK) >>\
      MMU_MMU_TTB_TTBAddress_OFFSET))


/********************************************************************/


#define MMUMMU_TTBTTBAddressWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_TTB_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_TTBTTBAddressWrite32);\
    data &= ~(MMU_MMU_TTB_TTBAddress_MASK);\
    newValue <<= MMU_MMU_TTB_TTBAddress_OFFSET;\
    newValue &= MMU_MMU_TTB_TTBAddress_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_TTBTTBAddressSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_TTBTTBAddressSet32),\
      (((var) & ~(MMU_MMU_TTB_TTBAddress_MASK)) |\
      (((value) << MMU_MMU_TTB_TTBAddress_OFFSET) &\
      MMU_MMU_TTB_TTBAddress_MASK)))


/********************************************************************/


#define MMUMMU_LOCKReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LOCKReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_LOCK_OFFSET))


/********************************************************************/


#define MMUMMU_LOCKWriteRegister32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_LOCK_OFFSET;\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LOCKWriteRegister32);\
    WR_MEM_32_VOLATILE((baseAddress)+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_LOCKBaseValueRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LOCKBaseValueRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_LOCK_OFFSET)))) &\
      MMU_MMU_LOCK_BaseValue_MASK) >>\
      MMU_MMU_LOCK_BaseValue_OFFSET))


/********************************************************************/


#define MMUMMU_LOCKBaseValueGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LOCKBaseValueGet32),\
      (UWORD32)(((var) & MMU_MMU_LOCK_BaseValue_MASK) >>\
      MMU_MMU_LOCK_BaseValue_OFFSET))


/********************************************************************/


#define MMUMMU_LOCKBaseValueWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_LOCK_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LOCKBaseValueWrite32);\
    data &= ~(MMU_MMU_LOCK_BaseValue_MASK);\
    newValue <<= MMU_MMU_LOCK_BaseValue_OFFSET;\
    newValue &= MMU_MMU_LOCK_BaseValue_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_LOCKBaseValueSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LOCKBaseValueSet32),\
      (((var) & ~(MMU_MMU_LOCK_BaseValue_MASK)) |\
      (((value) << MMU_MMU_LOCK_BaseValue_OFFSET) &\
      MMU_MMU_LOCK_BaseValue_MASK)))


/********************************************************************/


#define MMUMMU_LOCKCurrentVictimRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LOCKCurrentVictimRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_LOCK_OFFSET)))) &\
      MMU_MMU_LOCK_CurrentVictim_MASK) >>\
      MMU_MMU_LOCK_CurrentVictim_OFFSET))


/********************************************************************/


#define MMUMMU_LOCKCurrentVictimGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LOCKCurrentVictimGet32),\
      (UWORD32)(((var) & MMU_MMU_LOCK_CurrentVictim_MASK) >>\
	MMU_MMU_LOCK_CurrentVictim_OFFSET))


/********************************************************************/


#define MMUMMU_LOCKCurrentVictimWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_LOCK_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LOCKCurrentVictimWrite32);\
    data &= ~(MMU_MMU_LOCK_CurrentVictim_MASK);\
    newValue <<= MMU_MMU_LOCK_CurrentVictim_OFFSET;\
    newValue &= MMU_MMU_LOCK_CurrentVictim_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_LOCKCurrentVictimSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LOCKCurrentVictimSet32),\
      (((var) & ~(MMU_MMU_LOCK_CurrentVictim_MASK)) |\
      (((value) << MMU_MMU_LOCK_CurrentVictim_OFFSET) &\
      MMU_MMU_LOCK_CurrentVictim_MASK)))


/********************************************************************/


#define MMUMMU_LD_TLBReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LD_TLBReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_LD_TLB_OFFSET))


/********************************************************************/


#define MMUMMU_LD_TLBWriteRegister32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_LD_TLB_OFFSET;\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LD_TLBWriteRegister32);\
    WR_MEM_32_VOLATILE((baseAddress)+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_LD_TLBLdTLBItemRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LD_TLBLdTLBItemRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_LD_TLB_OFFSET)))) &\
      MMU_MMU_LD_TLB_LdTLBItem_MASK) >>\
      MMU_MMU_LD_TLB_LdTLBItem_OFFSET))


/********************************************************************/


#define MMUMMU_LD_TLBLdTLBItemReadIsalways_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LD_TLBLdTLBItemReadIsalways_r32),\
      (MMUMMU_LD_TLBLdTLBItemalways_r == (MMUMMU_LD_TLBLdTLBItemE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_LD_TLB_OFFSET)))) &\
      MMU_MMU_LD_TLB_LdTLBItem_MASK) >>\
      MMU_MMU_LD_TLB_LdTLBItem_OFFSET)))


/********************************************************************/


#define MMUMMU_LD_TLBLdTLBItemReadIsnever_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LD_TLBLdTLBItemReadIsnever_r32),\
      (MMUMMU_LD_TLBLdTLBItemnever_r == (MMUMMU_LD_TLBLdTLBItemE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_LD_TLB_OFFSET)))) &\
      MMU_MMU_LD_TLB_LdTLBItem_MASK) >>\
      MMU_MMU_LD_TLB_LdTLBItem_OFFSET)))


/********************************************************************/


#define MMUMMU_LD_TLBLdTLBItemGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LD_TLBLdTLBItemGet32),\
      (UWORD32)(((var) & MMU_MMU_LD_TLB_LdTLBItem_MASK) >>\
      MMU_MMU_LD_TLB_LdTLBItem_OFFSET))


/********************************************************************/


#define MMUMMU_LD_TLBLdTLBItemIsalways_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LD_TLBLdTLBItemIsalways_r32),\
      (MMUMMU_LD_TLBLdTLBItemalways_r == (MMUMMU_LD_TLBLdTLBItemE)\
	(((var) & MMU_MMU_LD_TLB_LdTLBItem_MASK) >>\
      MMU_MMU_LD_TLB_LdTLBItem_OFFSET)))


/********************************************************************/


#define MMUMMU_LD_TLBLdTLBItemIsnever_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LD_TLBLdTLBItemIsnever_r32),\
      (MMUMMU_LD_TLBLdTLBItemnever_r == (MMUMMU_LD_TLBLdTLBItemE)\
	(((var) & MMU_MMU_LD_TLB_LdTLBItem_MASK) >>\
      MMU_MMU_LD_TLB_LdTLBItem_OFFSET)))


/********************************************************************/


#define MMUMMU_LD_TLBLdTLBItemWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_LD_TLB_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LD_TLBLdTLBItemWrite32);\
    data &= ~(MMU_MMU_LD_TLB_LdTLBItem_MASK);\
    newValue <<= MMU_MMU_LD_TLB_LdTLBItem_OFFSET;\
    newValue &= MMU_MMU_LD_TLB_LdTLBItem_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_LD_TLBLdTLBItemWritenoeffect_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_LD_TLB_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_LD_TLBLdTLBItemnoeffect_w <<\
      MMU_MMU_LD_TLB_LdTLBItem_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LD_TLBLdTLBItemWritenoeffect_w32);\
    data &= ~(MMU_MMU_LD_TLB_LdTLBItem_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_LD_TLBLdTLBItemWriteldTLB_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_LD_TLB_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_LD_TLBLdTLBItemldTLB_w <<\
      MMU_MMU_LD_TLB_LdTLBItem_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LD_TLBLdTLBItemWriteldTLB_w32);\
    data &= ~(MMU_MMU_LD_TLB_LdTLBItem_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_LD_TLBLdTLBItemSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_LD_TLBLdTLBItemSet32),\
      (((var) & ~(MMU_MMU_LD_TLB_LdTLBItem_MASK)) |\
      (((value) << MMU_MMU_LD_TLB_LdTLBItem_OFFSET) &\
      MMU_MMU_LD_TLB_LdTLBItem_MASK)))


/********************************************************************/


#define MMUMMU_CAMReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_CAM_OFFSET))


/********************************************************************/


#define MMUMMU_CAMWriteRegister32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMWriteRegister32);\
    WR_MEM_32_VOLATILE((baseAddress)+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_CAMVATagRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVATagRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CAM_OFFSET)))) &\
      MMU_MMU_CAM_VATag_MASK) >>\
      MMU_MMU_CAM_VATag_OFFSET))


/********************************************************************/


#define MMUMMU_CAMVATagGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVATagGet32),\
      (UWORD32)(((var) & MMU_MMU_CAM_VATag_MASK) >>\
	MMU_MMU_CAM_VATag_OFFSET))


/********************************************************************/


#define MMUMMU_CAMVATagWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVATagWrite32);\
    data &= ~(MMU_MMU_CAM_VATag_MASK);\
    newValue <<= MMU_MMU_CAM_VATag_OFFSET;\
    newValue &= MMU_MMU_CAM_VATag_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_CAMVATagSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVATagSet32),\
      (((var) & ~(MMU_MMU_CAM_VATag_MASK)) |\
      (((value) << MMU_MMU_CAM_VATag_OFFSET) &\
      MMU_MMU_CAM_VATag_MASK)))


/********************************************************************/


#define MMUMMU_CAMPRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CAM_OFFSET)))) &\
      MMU_MMU_CAM_P_MASK) >>\
      MMU_MMU_CAM_P_OFFSET))


/********************************************************************/


#define MMUMMU_CAMPReadIsCanFlush32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPReadIsCanFlush32),\
      (MMUMMU_CAMPCanFlush == (MMUMMU_CAMPE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CAM_OFFSET)))) &\
      MMU_MMU_CAM_P_MASK) >>\
      MMU_MMU_CAM_P_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMPReadIsNoFlush32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPReadIsNoFlush32),\
      (MMUMMU_CAMPNoFlush == (MMUMMU_CAMPE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CAM_OFFSET)))) &\
      MMU_MMU_CAM_P_MASK) >>\
      MMU_MMU_CAM_P_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMPGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPGet32),\
      (UWORD32)(((var) & MMU_MMU_CAM_P_MASK) >> MMU_MMU_CAM_P_OFFSET))


/********************************************************************/


#define MMUMMU_CAMPIsCanFlush32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPIsCanFlush32),\
      (MMUMMU_CAMPCanFlush == (MMUMMU_CAMPE)(((var) & MMU_MMU_CAM_P_MASK) >>\
      MMU_MMU_CAM_P_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMPIsNoFlush32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPIsNoFlush32),\
      (MMUMMU_CAMPNoFlush == (MMUMMU_CAMPE)(((var) & MMU_MMU_CAM_P_MASK) >>\
      MMU_MMU_CAM_P_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMPWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPWrite32);\
    data &= ~(MMU_MMU_CAM_P_MASK);\
    newValue <<= MMU_MMU_CAM_P_OFFSET;\
    newValue &= MMU_MMU_CAM_P_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_CAMPWriteCanFlush32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CAMPCanFlush <<\
      MMU_MMU_CAM_P_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPWriteCanFlush32);\
    data &= ~(MMU_MMU_CAM_P_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CAMPWriteNoFlush32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CAMPNoFlush <<\
      MMU_MMU_CAM_P_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPWriteNoFlush32);\
    data &= ~(MMU_MMU_CAM_P_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CAMPSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPSet32),\
      (((var) & ~(MMU_MMU_CAM_P_MASK)) |\
      (((value) << MMU_MMU_CAM_P_OFFSET) &\
      MMU_MMU_CAM_P_MASK)))


/********************************************************************/


#define MMUMMU_CAMVRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CAM_OFFSET)))) &\
      MMU_MMU_CAM_V_MASK) >>\
      MMU_MMU_CAM_V_OFFSET))


/********************************************************************/


#define MMUMMU_CAMVReadIsInvalid32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVReadIsInvalid32),\
      (MMUMMU_CAMVInvalid == (MMUMMU_CAMVE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CAM_OFFSET)))) &\
      MMU_MMU_CAM_V_MASK) >>\
      MMU_MMU_CAM_V_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMVReadIsValid32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVReadIsValid32),\
      (MMUMMU_CAMVValid == (MMUMMU_CAMVE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CAM_OFFSET)))) &\
      MMU_MMU_CAM_V_MASK) >>\
      MMU_MMU_CAM_V_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMVGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVGet32),\
      (UWORD32)(((var) & MMU_MMU_CAM_V_MASK) >> MMU_MMU_CAM_V_OFFSET))


/********************************************************************/


#define MMUMMU_CAMVIsInvalid32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVIsInvalid32),\
      (MMUMMU_CAMVInvalid == (MMUMMU_CAMVE)(((var) & MMU_MMU_CAM_V_MASK) >>\
      MMU_MMU_CAM_V_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMVIsValid32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVIsValid32),\
      (MMUMMU_CAMVValid == (MMUMMU_CAMVE)(((var) & MMU_MMU_CAM_V_MASK) >>\
      MMU_MMU_CAM_V_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMVWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVWrite32);\
    data &= ~(MMU_MMU_CAM_V_MASK);\
    newValue <<= MMU_MMU_CAM_V_OFFSET;\
    newValue &= MMU_MMU_CAM_V_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_CAMVWriteInvalid32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CAMVInvalid <<\
      MMU_MMU_CAM_V_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVWriteInvalid32);\
    data &= ~(MMU_MMU_CAM_V_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CAMVWriteValid32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CAMVValid <<\
      MMU_MMU_CAM_V_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVWriteValid32);\
    data &= ~(MMU_MMU_CAM_V_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CAMVSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMVSet32),\
      (((var) & ~(MMU_MMU_CAM_V_MASK)) |\
      (((value) << MMU_MMU_CAM_V_OFFSET) &\
      MMU_MMU_CAM_V_MASK)))


/********************************************************************/


#define MMUMMU_CAMPageSizeRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CAM_OFFSET)))) &\
      MMU_MMU_CAM_PageSize_MASK) >>\
      MMU_MMU_CAM_PageSize_OFFSET))


/********************************************************************/


#define MMUMMU_CAMPageSizeReadIsSection32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeReadIsSection32),\
      (MMUMMU_CAMPageSizeSection == (MMUMMU_CAMPageSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CAM_OFFSET)))) &\
      MMU_MMU_CAM_PageSize_MASK) >>\
      MMU_MMU_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMPageSizeReadIsLarge32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeReadIsLarge32),\
      (MMUMMU_CAMPageSizeLarge == (MMUMMU_CAMPageSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CAM_OFFSET)))) &\
      MMU_MMU_CAM_PageSize_MASK) >>\
      MMU_MMU_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMPageSizeReadIsSmall32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeReadIsSmall32),\
      (MMUMMU_CAMPageSizeSmall == (MMUMMU_CAMPageSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CAM_OFFSET)))) &\
      MMU_MMU_CAM_PageSize_MASK) >>\
      MMU_MMU_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMPageSizeReadIsSuper32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeReadIsSuper32),\
      (MMUMMU_CAMPageSizeSuper == (MMUMMU_CAMPageSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_CAM_OFFSET)))) &\
      MMU_MMU_CAM_PageSize_MASK) >>\
      MMU_MMU_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMPageSizeGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeGet32),\
      (UWORD32)(((var) & MMU_MMU_CAM_PageSize_MASK) >>\
      MMU_MMU_CAM_PageSize_OFFSET))


/********************************************************************/


#define MMUMMU_CAMPageSizeIsSection32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeIsSection32),\
      (MMUMMU_CAMPageSizeSection == (MMUMMU_CAMPageSizeE)\
      (((var) & MMU_MMU_CAM_PageSize_MASK) >>\
      MMU_MMU_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMPageSizeIsLarge32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeIsLarge32),\
      (MMUMMU_CAMPageSizeLarge == (MMUMMU_CAMPageSizeE)\
      (((var) & MMU_MMU_CAM_PageSize_MASK) >>\
      MMU_MMU_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMPageSizeIsSmall32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeIsSmall32),\
      (MMUMMU_CAMPageSizeSmall == (MMUMMU_CAMPageSizeE)\
      (((var) & MMU_MMU_CAM_PageSize_MASK) >>\
      MMU_MMU_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMPageSizeIsSuper32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeIsSuper32),\
      (MMUMMU_CAMPageSizeSuper == (MMUMMU_CAMPageSizeE)\
      (((var) & MMU_MMU_CAM_PageSize_MASK) >>\
      MMU_MMU_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_CAMPageSizeWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeWrite32);\
    data &= ~(MMU_MMU_CAM_PageSize_MASK);\
    newValue <<= MMU_MMU_CAM_PageSize_OFFSET;\
    newValue &= MMU_MMU_CAM_PageSize_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_CAMPageSizeWriteSection32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CAMPageSizeSection <<\
      MMU_MMU_CAM_PageSize_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeWriteSection32);\
    data &= ~(MMU_MMU_CAM_PageSize_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CAMPageSizeWriteLarge32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CAMPageSizeLarge <<\
      MMU_MMU_CAM_PageSize_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeWriteLarge32);\
    data &= ~(MMU_MMU_CAM_PageSize_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CAMPageSizeWriteSmall32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CAMPageSizeSmall <<\
      MMU_MMU_CAM_PageSize_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeWriteSmall32);\
    data &= ~(MMU_MMU_CAM_PageSize_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CAMPageSizeWriteSuper32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_CAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_CAMPageSizeSuper <<\
      MMU_MMU_CAM_PageSize_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeWriteSuper32);\
    data &= ~(MMU_MMU_CAM_PageSize_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_CAMPageSizeSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_CAMPageSizeSet32),\
      (((var) & ~(MMU_MMU_CAM_PageSize_MASK)) |\
      (((value) << MMU_MMU_CAM_PageSize_OFFSET) &\
      MMU_MMU_CAM_PageSize_MASK)))


/********************************************************************/


#define MMUMMU_RAMReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_RAM_OFFSET))


/********************************************************************/


#define MMUMMU_RAMWriteRegister32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMWriteRegister32);\
    WR_MEM_32_VOLATILE((baseAddress)+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_RAMPhysicalAddressRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMPhysicalAddressRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_RAM_OFFSET)))) &\
      MMU_MMU_RAM_PhysicalAddress_MASK) >>\
      MMU_MMU_RAM_PhysicalAddress_OFFSET))


/********************************************************************/


#define MMUMMU_RAMPhysicalAddressGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMPhysicalAddressGet32),\
      (UWORD32)(((var) & MMU_MMU_RAM_PhysicalAddress_MASK) >>\
	MMU_MMU_RAM_PhysicalAddress_OFFSET))


/********************************************************************/


#define MMUMMU_RAMPhysicalAddressWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMPhysicalAddressWrite32);\
    data &= ~(MMU_MMU_RAM_PhysicalAddress_MASK);\
    newValue <<= MMU_MMU_RAM_PhysicalAddress_OFFSET;\
    newValue &= MMU_MMU_RAM_PhysicalAddress_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_RAMPhysicalAddressSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMPhysicalAddressSet32),\
      (((var) & ~(MMU_MMU_RAM_PhysicalAddress_MASK)) |\
      (((value) << MMU_MMU_RAM_PhysicalAddress_OFFSET) &\
      MMU_MMU_RAM_PhysicalAddress_MASK)))


/********************************************************************/


#define MMUMMU_RAMEndiannessRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMEndiannessRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_RAM_OFFSET)))) &\
      MMU_MMU_RAM_Endianness_MASK) >>\
      MMU_MMU_RAM_Endianness_OFFSET))


/********************************************************************/


#define MMUMMU_RAMEndiannessReadIsLittle32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMEndiannessReadIsLittle32),\
      (MMUMMU_RAMEndiannessLittle == (MMUMMU_RAMEndiannessE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_RAM_OFFSET)))) &\
      MMU_MMU_RAM_Endianness_MASK) >>\
      MMU_MMU_RAM_Endianness_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMEndiannessReadIsBig32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMEndiannessReadIsBig32),\
      (MMUMMU_RAMEndiannessBig == (MMUMMU_RAMEndiannessE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_RAM_OFFSET)))) &\
      MMU_MMU_RAM_Endianness_MASK) >>\
      MMU_MMU_RAM_Endianness_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMEndiannessGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMEndiannessGet32),\
      (UWORD32)(((var) & MMU_MMU_RAM_Endianness_MASK) >>\
      MMU_MMU_RAM_Endianness_OFFSET))


/********************************************************************/


#define MMUMMU_RAMEndiannessIsLittle32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMEndiannessIsLittle32),\
      (MMUMMU_RAMEndiannessLittle == (MMUMMU_RAMEndiannessE)\
      (((var) & MMU_MMU_RAM_Endianness_MASK) >>\
      MMU_MMU_RAM_Endianness_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMEndiannessIsBig32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMEndiannessIsBig32),\
      (MMUMMU_RAMEndiannessBig == (MMUMMU_RAMEndiannessE)\
      (((var) & MMU_MMU_RAM_Endianness_MASK) >>\
      MMU_MMU_RAM_Endianness_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMEndiannessWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMEndiannessWrite32);\
    data &= ~(MMU_MMU_RAM_Endianness_MASK);\
    newValue <<= MMU_MMU_RAM_Endianness_OFFSET;\
    newValue &= MMU_MMU_RAM_Endianness_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_RAMEndiannessWriteLittle32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_RAMEndiannessLittle <<\
      MMU_MMU_RAM_Endianness_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMEndiannessWriteLittle32);\
    data &= ~(MMU_MMU_RAM_Endianness_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_RAMEndiannessWriteBig32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_RAMEndiannessBig <<\
      MMU_MMU_RAM_Endianness_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMEndiannessWriteBig32);\
    data &= ~(MMU_MMU_RAM_Endianness_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_RAMEndiannessSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMEndiannessSet32),\
      (((var) & ~(MMU_MMU_RAM_Endianness_MASK)) |\
      (((value) << MMU_MMU_RAM_Endianness_OFFSET) &\
      MMU_MMU_RAM_Endianness_MASK)))


/********************************************************************/


#define MMUMMU_RAMElementSizeRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_RAM_OFFSET)))) &\
      MMU_MMU_RAM_ElementSize_MASK) >>\
      MMU_MMU_RAM_ElementSize_OFFSET))


/********************************************************************/


#define MMUMMU_RAMElementSizeReadIsByte32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeReadIsByte32),\
      (MMUMMU_RAMElementSizeByte == (MMUMMU_RAMElementSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_RAM_OFFSET)))) &\
      MMU_MMU_RAM_ElementSize_MASK) >>\
      MMU_MMU_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMElementSizeReadIsShort32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeReadIsShort32),\
      (MMUMMU_RAMElementSizeShort == (MMUMMU_RAMElementSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_RAM_OFFSET)))) &\
      MMU_MMU_RAM_ElementSize_MASK) >>\
      MMU_MMU_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMElementSizeReadIsLong32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeReadIsLong32),\
      (MMUMMU_RAMElementSizeLong == (MMUMMU_RAMElementSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_RAM_OFFSET)))) &\
      MMU_MMU_RAM_ElementSize_MASK) >>\
      MMU_MMU_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMElementSizeReadIsNone32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeReadIsNone32),\
      (MMUMMU_RAMElementSizeNone == (MMUMMU_RAMElementSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_RAM_OFFSET)))) &\
      MMU_MMU_RAM_ElementSize_MASK) >>\
      MMU_MMU_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMElementSizeGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeGet32),\
      (UWORD32)(((var) & MMU_MMU_RAM_ElementSize_MASK) >>\
      MMU_MMU_RAM_ElementSize_OFFSET))


/********************************************************************/


#define MMUMMU_RAMElementSizeIsByte32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeIsByte32),\
      (MMUMMU_RAMElementSizeByte == (MMUMMU_RAMElementSizeE)\
      (((var) & MMU_MMU_RAM_ElementSize_MASK) >>\
      MMU_MMU_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMElementSizeIsShort32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeIsShort32),\
      (MMUMMU_RAMElementSizeShort == (MMUMMU_RAMElementSizeE)\
	(((var) & MMU_MMU_RAM_ElementSize_MASK) >>\
      MMU_MMU_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMElementSizeIsLong32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeIsLong32),\
      (MMUMMU_RAMElementSizeLong == (MMUMMU_RAMElementSizeE)\
      (((var) & MMU_MMU_RAM_ElementSize_MASK) >>\
      MMU_MMU_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMElementSizeIsNone32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeIsNone32),\
      (MMUMMU_RAMElementSizeNone == (MMUMMU_RAMElementSizeE)\
      (((var) & MMU_MMU_RAM_ElementSize_MASK) >>\
      MMU_MMU_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMElementSizeWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeWrite32);\
    data &= ~(MMU_MMU_RAM_ElementSize_MASK);\
    newValue <<= MMU_MMU_RAM_ElementSize_OFFSET;\
    newValue &= MMU_MMU_RAM_ElementSize_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_RAMElementSizeWriteByte32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_RAMElementSizeByte <<\
      MMU_MMU_RAM_ElementSize_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeWriteByte32);\
    data &= ~(MMU_MMU_RAM_ElementSize_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_RAMElementSizeWriteShort32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_RAMElementSizeShort <<\
      MMU_MMU_RAM_ElementSize_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeWriteShort32);\
    data &= ~(MMU_MMU_RAM_ElementSize_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_RAMElementSizeWriteLong32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_RAMElementSizeLong <<\
      MMU_MMU_RAM_ElementSize_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeWriteLong32);\
    data &= ~(MMU_MMU_RAM_ElementSize_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_RAMElementSizeWriteNone32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_RAMElementSizeNone <<\
      MMU_MMU_RAM_ElementSize_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeWriteNone32);\
    data &= ~(MMU_MMU_RAM_ElementSize_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_RAMElementSizeSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMElementSizeSet32),\
      (((var) & ~(MMU_MMU_RAM_ElementSize_MASK)) |\
      (((value) << MMU_MMU_RAM_ElementSize_OFFSET) &\
      MMU_MMU_RAM_ElementSize_MASK)))


/********************************************************************/


#define MMUMMU_RAMMixedRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMMixedRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_RAM_OFFSET)))) &\
      MMU_MMU_RAM_Mixed_MASK) >>\
      MMU_MMU_RAM_Mixed_OFFSET))


/********************************************************************/


#define MMUMMU_RAMMixedReadIsTLBes32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMMixedReadIsTLBes32),\
      (MMUMMU_RAMMixedTLBes == (MMUMMU_RAMMixedE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_RAM_OFFSET)))) &\
      MMU_MMU_RAM_Mixed_MASK) >>\
      MMU_MMU_RAM_Mixed_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMMixedReadIsCPUes32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMMixedReadIsCPUes32),\
      (MMUMMU_RAMMixedCPUes == (MMUMMU_RAMMixedE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_RAM_OFFSET)))) &\
      MMU_MMU_RAM_Mixed_MASK) >>\
      MMU_MMU_RAM_Mixed_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMMixedGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMMixedGet32),\
      (UWORD32)(((var) & MMU_MMU_RAM_Mixed_MASK) >>\
	MMU_MMU_RAM_Mixed_OFFSET))


/********************************************************************/


#define MMUMMU_RAMMixedIsTLBes32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMMixedIsTLBes32),\
      (MMUMMU_RAMMixedTLBes == (MMUMMU_RAMMixedE)\
      (((var) & MMU_MMU_RAM_Mixed_MASK) >>\
      MMU_MMU_RAM_Mixed_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMMixedIsCPUes32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMMixedIsCPUes32),\
      (MMUMMU_RAMMixedCPUes == (MMUMMU_RAMMixedE)\
      (((var) & MMU_MMU_RAM_Mixed_MASK) >>\
      MMU_MMU_RAM_Mixed_OFFSET)))


/********************************************************************/


#define MMUMMU_RAMMixedWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMMixedWrite32);\
    data &= ~(MMU_MMU_RAM_Mixed_MASK);\
    newValue <<= MMU_MMU_RAM_Mixed_OFFSET;\
    newValue &= MMU_MMU_RAM_Mixed_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_RAMMixedWriteTLBes32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_RAMMixedTLBes <<\
      MMU_MMU_RAM_Mixed_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMMixedWriteTLBes32);\
    data &= ~(MMU_MMU_RAM_Mixed_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_RAMMixedWriteCPUes32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_RAM_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_RAMMixedCPUes <<\
      MMU_MMU_RAM_Mixed_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMMixedWriteCPUes32);\
    data &= ~(MMU_MMU_RAM_Mixed_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_RAMMixedSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_RAMMixedSet32),\
      (((var) & ~(MMU_MMU_RAM_Mixed_MASK)) |\
      (((value) << MMU_MMU_RAM_Mixed_OFFSET) &\
      MMU_MMU_RAM_Mixed_MASK)))


/********************************************************************/


#define MMUMMU_GFLUSHReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_GFLUSHReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_GFLUSH_OFFSET))


/********************************************************************/


#define MMUMMU_GFLUSHWriteRegister32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_GFLUSH_OFFSET;\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_GFLUSHWriteRegister32);\
    WR_MEM_32_VOLATILE((baseAddress)+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_GFLUSHGlobalFlushRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_GFLUSHGlobalFlushRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_GFLUSH_OFFSET)))) &\
      MMU_MMU_GFLUSH_GlobalFlush_MASK) >>\
      MMU_MMU_GFLUSH_GlobalFlush_OFFSET))


/********************************************************************/


#define MMUMMU_GFLUSHGlobalFlushReadIsrtn0_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_GFLUSHGlobalFlushReadIsrtn0_r32),\
      (MMUMMU_GFLUSHGlobalFlushrtn0_r == (MMUMMU_GFLUSHGlobalFlushE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_GFLUSH_OFFSET)))) &\
      MMU_MMU_GFLUSH_GlobalFlush_MASK) >>\
      MMU_MMU_GFLUSH_GlobalFlush_OFFSET)))


/********************************************************************/


#define MMUMMU_GFLUSHGlobalFlushReadIsnever_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_GFLUSHGlobalFlushReadIsnever_r32),\
      (MMUMMU_GFLUSHGlobalFlushnever_r == (MMUMMU_GFLUSHGlobalFlushE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_GFLUSH_OFFSET)))) &\
      MMU_MMU_GFLUSH_GlobalFlush_MASK) >>\
      MMU_MMU_GFLUSH_GlobalFlush_OFFSET)))


/********************************************************************/


#define MMUMMU_GFLUSHGlobalFlushGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_GFLUSHGlobalFlushGet32),\
      (UWORD32)(((var) & MMU_MMU_GFLUSH_GlobalFlush_MASK) >>\
      MMU_MMU_GFLUSH_GlobalFlush_OFFSET))


/********************************************************************/


#define MMUMMU_GFLUSHGlobalFlushIsrtn0_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_GFLUSHGlobalFlushIsrtn0_r32),\
      (MMUMMU_GFLUSHGlobalFlushrtn0_r == (MMUMMU_GFLUSHGlobalFlushE)\
	(((var) & MMU_MMU_GFLUSH_GlobalFlush_MASK) >>\
      MMU_MMU_GFLUSH_GlobalFlush_OFFSET)))


/********************************************************************/


#define MMUMMU_GFLUSHGlobalFlushIsnever_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_GFLUSHGlobalFlushIsnever_r32),\
      (MMUMMU_GFLUSHGlobalFlushnever_r == (MMUMMU_GFLUSHGlobalFlushE)\
	(((var) & MMU_MMU_GFLUSH_GlobalFlush_MASK) >>\
      MMU_MMU_GFLUSH_GlobalFlush_OFFSET)))


/********************************************************************/


#define MMUMMU_GFLUSHGlobalFlushWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_GFLUSH_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_GFLUSHGlobalFlushWrite32);\
    data &= ~(MMU_MMU_GFLUSH_GlobalFlush_MASK);\
    newValue <<= MMU_MMU_GFLUSH_GlobalFlush_OFFSET;\
    newValue &= MMU_MMU_GFLUSH_GlobalFlush_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_GFLUSHGlobalFlushWritenft_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_GFLUSH_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_GFLUSHGlobalFlushnft_w <<\
      MMU_MMU_GFLUSH_GlobalFlush_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_GFLUSHGlobalFlushWritenft_w32);\
    data &= ~(MMU_MMU_GFLUSH_GlobalFlush_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_GFLUSHGlobalFlushWriteflush_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_GFLUSH_OFFSET;\
    const UWORD32 newValue = (UWORD32)MMUMMU_GFLUSHGlobalFlushflush_w <<\
      MMU_MMU_GFLUSH_GlobalFlush_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_GFLUSHGlobalFlushWriteflush_w32);\
    data &= ~(MMU_MMU_GFLUSH_GlobalFlush_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_GFLUSHGlobalFlushSet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_GFLUSHGlobalFlushSet32),\
      (((var) & ~(MMU_MMU_GFLUSH_GlobalFlush_MASK)) |\
      (((value) << MMU_MMU_GFLUSH_GlobalFlush_OFFSET) &\
      MMU_MMU_GFLUSH_GlobalFlush_MASK)))


/********************************************************************/


#define MMUMMU_FLUSH_ENTRYReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_FLUSH_ENTRYReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_FLUSH_ENTRY_OFFSET))


/********************************************************************/


#define MMUMMU_FLUSH_ENTRYWriteRegister32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_FLUSH_ENTRY_OFFSET;\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_FLUSH_ENTRYWriteRegister32);\
    WR_MEM_32_VOLATILE((baseAddress)+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_FLUSH_ENTRYFlushEntryRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_FLUSH_ENTRYFlushEntryRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+\
      (MMU_MMU_FLUSH_ENTRY_OFFSET)))) &\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK) >>\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_OFFSET))


/********************************************************************/


#define MMUMMU_FLUSH_ENTRYFlushEntryReadIsalways_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_FLUSH_ENTRYFlushEntryReadIsalways_r32),\
      (MMUMMU_FLUSH_ENTRYFlushEntryalways_r == \
      (MMUMMU_FLUSH_ENTRYFlushEntryE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+\
      (MMU_MMU_FLUSH_ENTRY_OFFSET)))) &\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK) >>\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_OFFSET)))


/********************************************************************/


#define MMUMMU_FLUSH_ENTRYFlushEntryReadIsnever_r32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_FLUSH_ENTRYFlushEntryReadIsnever_r32),\
      (MMUMMU_FLUSH_ENTRYFlushEntrynever_r == \
      (MMUMMU_FLUSH_ENTRYFlushEntryE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+\
      (MMU_MMU_FLUSH_ENTRY_OFFSET)))) &\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK) >>\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_OFFSET)))


/********************************************************************/


#define MMUMMU_FLUSH_ENTRYFlushEntryGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_FLUSH_ENTRYFlushEntryGet32),\
      (UWORD32)(((var) & MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK) >>\
	MMU_MMU_FLUSH_ENTRY_FlushEntry_OFFSET))


/********************************************************************/


#define MMUMMU_FLUSH_ENTRYFlushEntryIsalways_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_FLUSH_ENTRYFlushEntryIsalways_r32),\
      (MMUMMU_FLUSH_ENTRYFlushEntryalways_r == \
	(MMUMMU_FLUSH_ENTRYFlushEntryE)(((var) &\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK) >>\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_OFFSET)))


/********************************************************************/


#define MMUMMU_FLUSH_ENTRYFlushEntryIsnever_r32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_FLUSH_ENTRYFlushEntryIsnever_r32),\
      (MMUMMU_FLUSH_ENTRYFlushEntrynever_r == \
	(MMUMMU_FLUSH_ENTRYFlushEntryE)(((var) &\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK) >>\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_OFFSET)))


/********************************************************************/


#define MMUMMU_FLUSH_ENTRYFlushEntryWrite32(baseAddress, value)\
{\
    const UWORD32 offset = MMU_MMU_FLUSH_ENTRY_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE((baseAddress)+offset);\
    register UWORD32 newValue = (value);\
    _DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_FLUSH_ENTRYFlushEntryWrite32);\
    data &= ~(MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK);\
    newValue <<= MMU_MMU_FLUSH_ENTRY_FlushEntry_OFFSET;\
    newValue &= MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK;\
    newValue |= data;\
    WR_MEM_32_VOLATILE(baseAddress+offset, newValue);\
}


/********************************************************************/


#define MMUMMU_FLUSH_ENTRYFlushEntryWritenofun_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_FLUSH_ENTRY_OFFSET;\
    const UWORD32 newValue = (UWORD32)\
      MMUMMU_FLUSH_ENTRYFlushEntrynofun_w <<\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_FLUSH_ENTRYFlushEntryWritenofun_w32);\
    data &= ~(MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_FLUSH_ENTRYFlushEntryWriteflushTLB_w32(baseAddress)\
{\
    const UWORD32 offset = MMU_MMU_FLUSH_ENTRY_OFFSET;\
    const UWORD32 newValue = (UWORD32)\
      MMUMMU_FLUSH_ENTRYFlushEntryflushTLB_w <<\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_OFFSET;\
    register UWORD32 data = RD_MEM_32_VOLATILE(baseAddress+offset);\
    _DEBUG_LEVEL_1_EASI(\
      EASIL1_MMUMMU_FLUSH_ENTRYFlushEntryWriteflushTLB_w32);\
    data &= ~(MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK);\
    data |= newValue;\
    WR_MEM_32_VOLATILE(baseAddress+offset, data);\
}


/********************************************************************/


#define MMUMMU_FLUSH_ENTRYFlushEntrySet32(var, value)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_FLUSH_ENTRYFlushEntrySet32),\
      (((var) & ~(MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK)) |\
      (((value) << MMU_MMU_FLUSH_ENTRY_FlushEntry_OFFSET) &\
      MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK)))


/********************************************************************/


#define MMUMMU_READ_CAMReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_READ_CAM_OFFSET))


/********************************************************************/


#define MMUMMU_READ_CAMVATagRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMVATagRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_CAM_OFFSET)))) &\
      MMU_MMU_READ_CAM_VATag_MASK) >>\
      MMU_MMU_READ_CAM_VATag_OFFSET))


/********************************************************************/


#define MMUMMU_READ_CAMVATagGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMVATagGet32),\
      (UWORD32)(((var) & MMU_MMU_READ_CAM_VATag_MASK) >>\
      MMU_MMU_READ_CAM_VATag_OFFSET))


/********************************************************************/


#define MMUMMU_READ_CAMPRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_CAM_OFFSET)))) &\
      MMU_MMU_READ_CAM_P_MASK) >>\
      MMU_MMU_READ_CAM_P_OFFSET))


/********************************************************************/


#define MMUMMU_READ_CAMPReadIsCanFlush32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPReadIsCanFlush32),\
      (MMUMMU_READ_CAMPCanFlush == (MMUMMU_READ_CAMPE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_CAM_OFFSET)))) &\
      MMU_MMU_READ_CAM_P_MASK) >>\
      MMU_MMU_READ_CAM_P_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMPReadIsNoFlush32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPReadIsNoFlush32),\
      (MMUMMU_READ_CAMPNoFlush == (MMUMMU_READ_CAMPE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_CAM_OFFSET)))) &\
      MMU_MMU_READ_CAM_P_MASK) >>\
      MMU_MMU_READ_CAM_P_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMPGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPGet32),\
      (UWORD32)(((var) & MMU_MMU_READ_CAM_P_MASK) >>\
	MMU_MMU_READ_CAM_P_OFFSET))


/********************************************************************/


#define MMUMMU_READ_CAMPIsCanFlush32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPIsCanFlush32),\
      (MMUMMU_READ_CAMPCanFlush == (MMUMMU_READ_CAMPE)\
      (((var) & MMU_MMU_READ_CAM_P_MASK) >>\
      MMU_MMU_READ_CAM_P_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMPIsNoFlush32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPIsNoFlush32),\
      (MMUMMU_READ_CAMPNoFlush == (MMUMMU_READ_CAMPE)\
      (((var) & MMU_MMU_READ_CAM_P_MASK) >>\
      MMU_MMU_READ_CAM_P_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMVRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMVRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_CAM_OFFSET)))) &\
      MMU_MMU_READ_CAM_V_MASK) >>\
      MMU_MMU_READ_CAM_V_OFFSET))


/********************************************************************/


#define MMUMMU_READ_CAMVReadIsInvalid32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMVReadIsInvalid32),\
      (MMUMMU_READ_CAMVInvalid == (MMUMMU_READ_CAMVE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_CAM_OFFSET)))) &\
      MMU_MMU_READ_CAM_V_MASK) >>\
      MMU_MMU_READ_CAM_V_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMVReadIsValid32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMVReadIsValid32),\
      (MMUMMU_READ_CAMVValid == (MMUMMU_READ_CAMVE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_CAM_OFFSET)))) &\
      MMU_MMU_READ_CAM_V_MASK) >>\
      MMU_MMU_READ_CAM_V_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMVGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMVGet32),\
      (UWORD32)(((var) & MMU_MMU_READ_CAM_V_MASK) >>\
	MMU_MMU_READ_CAM_V_OFFSET))


/********************************************************************/


#define MMUMMU_READ_CAMVIsInvalid32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMVIsInvalid32),\
      (MMUMMU_READ_CAMVInvalid == (MMUMMU_READ_CAMVE)(((var) &\
      MMU_MMU_READ_CAM_V_MASK) >>\
      MMU_MMU_READ_CAM_V_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMVIsValid32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMVIsValid32),\
      (MMUMMU_READ_CAMVValid == (MMUMMU_READ_CAMVE)(((var) &\
      MMU_MMU_READ_CAM_V_MASK) >>\
      MMU_MMU_READ_CAM_V_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMPageSizeRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPageSizeRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_CAM_OFFSET)))) &\
      MMU_MMU_READ_CAM_PageSize_MASK) >>\
      MMU_MMU_READ_CAM_PageSize_OFFSET))


/********************************************************************/


#define MMUMMU_READ_CAMPageSizeReadIsSection32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPageSizeReadIsSection32),\
      (MMUMMU_READ_CAMPageSizeSection == (MMUMMU_READ_CAMPageSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_CAM_OFFSET)))) &\
      MMU_MMU_READ_CAM_PageSize_MASK) >>\
      MMU_MMU_READ_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMPageSizeReadIsLarge32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPageSizeReadIsLarge32),\
      (MMUMMU_READ_CAMPageSizeLarge == (MMUMMU_READ_CAMPageSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_CAM_OFFSET)))) &\
      MMU_MMU_READ_CAM_PageSize_MASK) >>\
      MMU_MMU_READ_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMPageSizeReadIsSmall32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPageSizeReadIsSmall32),\
      (MMUMMU_READ_CAMPageSizeSmall == (MMUMMU_READ_CAMPageSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_CAM_OFFSET)))) &\
      MMU_MMU_READ_CAM_PageSize_MASK) >>\
      MMU_MMU_READ_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMPageSizeReadIsSuper32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPageSizeReadIsSuper32),\
      (MMUMMU_READ_CAMPageSizeSuper == (MMUMMU_READ_CAMPageSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_CAM_OFFSET)))) &\
      MMU_MMU_READ_CAM_PageSize_MASK) >>\
      MMU_MMU_READ_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMPageSizeGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPageSizeGet32),\
      (UWORD32)(((var) & MMU_MMU_READ_CAM_PageSize_MASK) >>\
      MMU_MMU_READ_CAM_PageSize_OFFSET))


/********************************************************************/


#define MMUMMU_READ_CAMPageSizeIsSection32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPageSizeIsSection32),\
      (MMUMMU_READ_CAMPageSizeSection == (MMUMMU_READ_CAMPageSizeE)\
	(((var) & MMU_MMU_READ_CAM_PageSize_MASK) >>\
      MMU_MMU_READ_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMPageSizeIsLarge32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPageSizeIsLarge32),\
      (MMUMMU_READ_CAMPageSizeLarge == (MMUMMU_READ_CAMPageSizeE)\
	(((var) & MMU_MMU_READ_CAM_PageSize_MASK) >>\
      MMU_MMU_READ_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMPageSizeIsSmall32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPageSizeIsSmall32),\
      (MMUMMU_READ_CAMPageSizeSmall == (MMUMMU_READ_CAMPageSizeE)\
	(((var) & MMU_MMU_READ_CAM_PageSize_MASK) >>\
      MMU_MMU_READ_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_CAMPageSizeIsSuper32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_CAMPageSizeIsSuper32),\
      (MMUMMU_READ_CAMPageSizeSuper == (MMUMMU_READ_CAMPageSizeE)\
	(((var) & MMU_MMU_READ_CAM_PageSize_MASK) >>\
      MMU_MMU_READ_CAM_PageSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_READ_RAM_OFFSET))


/********************************************************************/


#define MMUMMU_READ_RAMPhysicalAddressRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMPhysicalAddressRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_RAM_OFFSET)))) &\
      MMU_MMU_READ_RAM_PhysicalAddress_MASK) >>\
      MMU_MMU_READ_RAM_PhysicalAddress_OFFSET))


/********************************************************************/


#define MMUMMU_READ_RAMPhysicalAddressGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMPhysicalAddressGet32),\
      (UWORD32)(((var) & MMU_MMU_READ_RAM_PhysicalAddress_MASK) >>\
      MMU_MMU_READ_RAM_PhysicalAddress_OFFSET))


/********************************************************************/


#define MMUMMU_READ_RAMEndiannessRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMEndiannessRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_RAM_OFFSET)))) &\
      MMU_MMU_READ_RAM_Endianness_MASK) >>\
      MMU_MMU_READ_RAM_Endianness_OFFSET))


/********************************************************************/


#define MMUMMU_READ_RAMEndiannessReadIsLittle32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMEndiannessReadIsLittle32),\
      (MMUMMU_READ_RAMEndiannessLittle == (MMUMMU_READ_RAMEndiannessE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_RAM_OFFSET)))) &\
      MMU_MMU_READ_RAM_Endianness_MASK) >>\
      MMU_MMU_READ_RAM_Endianness_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMEndiannessReadIsBig32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMEndiannessReadIsBig32),\
      (MMUMMU_READ_RAMEndiannessBig == (MMUMMU_READ_RAMEndiannessE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_RAM_OFFSET)))) &\
      MMU_MMU_READ_RAM_Endianness_MASK) >>\
      MMU_MMU_READ_RAM_Endianness_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMEndiannessGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMEndiannessGet32),\
      (UWORD32)(((var) & MMU_MMU_READ_RAM_Endianness_MASK) >>\
	MMU_MMU_READ_RAM_Endianness_OFFSET))


/********************************************************************/


#define MMUMMU_READ_RAMEndiannessIsLittle32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMEndiannessIsLittle32),\
      (MMUMMU_READ_RAMEndiannessLittle == (MMUMMU_READ_RAMEndiannessE)\
	(((var) & MMU_MMU_READ_RAM_Endianness_MASK) >>\
      MMU_MMU_READ_RAM_Endianness_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMEndiannessIsBig32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMEndiannessIsBig32),\
      (MMUMMU_READ_RAMEndiannessBig == (MMUMMU_READ_RAMEndiannessE)\
      (((var) & MMU_MMU_READ_RAM_Endianness_MASK) >>\
      MMU_MMU_READ_RAM_Endianness_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMElementSizeRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMElementSizeRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_RAM_OFFSET)))) &\
      MMU_MMU_READ_RAM_ElementSize_MASK) >>\
      MMU_MMU_READ_RAM_ElementSize_OFFSET))


/********************************************************************/


#define MMUMMU_READ_RAMElementSizeReadIsByte32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMElementSizeReadIsByte32),\
      (MMUMMU_READ_RAMElementSizeByte == (MMUMMU_READ_RAMElementSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_RAM_OFFSET)))) &\
      MMU_MMU_READ_RAM_ElementSize_MASK) >>\
      MMU_MMU_READ_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMElementSizeReadIsShort32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMElementSizeReadIsShort32),\
      (MMUMMU_READ_RAMElementSizeShort == (MMUMMU_READ_RAMElementSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_RAM_OFFSET)))) &\
      MMU_MMU_READ_RAM_ElementSize_MASK) >>\
      MMU_MMU_READ_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMElementSizeReadIsLong32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMElementSizeReadIsLong32),\
      (MMUMMU_READ_RAMElementSizeLong == (MMUMMU_READ_RAMElementSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_RAM_OFFSET)))) &\
      MMU_MMU_READ_RAM_ElementSize_MASK) >>\
      MMU_MMU_READ_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMElementSizeReadIsNone32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMElementSizeReadIsNone32),\
      (MMUMMU_READ_RAMElementSizeNone == (MMUMMU_READ_RAMElementSizeE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_RAM_OFFSET)))) &\
      MMU_MMU_READ_RAM_ElementSize_MASK) >>\
      MMU_MMU_READ_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMElementSizeGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMElementSizeGet32),\
      (UWORD32)(((var) & MMU_MMU_READ_RAM_ElementSize_MASK) >>\
	MMU_MMU_READ_RAM_ElementSize_OFFSET))


/********************************************************************/


#define MMUMMU_READ_RAMElementSizeIsByte32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMElementSizeIsByte32),\
      (MMUMMU_READ_RAMElementSizeByte == (MMUMMU_READ_RAMElementSizeE)\
      (((var) & MMU_MMU_READ_RAM_ElementSize_MASK) >>\
      MMU_MMU_READ_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMElementSizeIsShort32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMElementSizeIsShort32),\
      (MMUMMU_READ_RAMElementSizeShort == (MMUMMU_READ_RAMElementSizeE)\
      (((var) & MMU_MMU_READ_RAM_ElementSize_MASK) >>\
      MMU_MMU_READ_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMElementSizeIsLong32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMElementSizeIsLong32),\
      (MMUMMU_READ_RAMElementSizeLong == (MMUMMU_READ_RAMElementSizeE)\
      (((var) & MMU_MMU_READ_RAM_ElementSize_MASK) >>\
      MMU_MMU_READ_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMElementSizeIsNone32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMElementSizeIsNone32),\
      (MMUMMU_READ_RAMElementSizeNone == (MMUMMU_READ_RAMElementSizeE)\
      (((var) & MMU_MMU_READ_RAM_ElementSize_MASK) >>\
      MMU_MMU_READ_RAM_ElementSize_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMMixedRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMMixedRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_RAM_OFFSET)))) &\
      MMU_MMU_READ_RAM_Mixed_MASK) >>\
      MMU_MMU_READ_RAM_Mixed_OFFSET))


/********************************************************************/


#define MMUMMU_READ_RAMMixedReadIsTLBes32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMMixedReadIsTLBes32),\
      (MMUMMU_READ_RAMMixedTLBes == (MMUMMU_READ_RAMMixedE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_RAM_OFFSET)))) &\
      MMU_MMU_READ_RAM_Mixed_MASK) >>\
      MMU_MMU_READ_RAM_Mixed_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMMixedReadIsCPUes32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMMixedReadIsCPUes32),\
      (MMUMMU_READ_RAMMixedCPUes == (MMUMMU_READ_RAMMixedE)\
      (((RD_MEM_32_VOLATILE(((baseAddress)+(MMU_MMU_READ_RAM_OFFSET)))) &\
      MMU_MMU_READ_RAM_Mixed_MASK) >>\
      MMU_MMU_READ_RAM_Mixed_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMMixedGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMMixedGet32),\
      (UWORD32)(((var) & MMU_MMU_READ_RAM_Mixed_MASK) >>\
      MMU_MMU_READ_RAM_Mixed_OFFSET))


/********************************************************************/


#define MMUMMU_READ_RAMMixedIsTLBes32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMMixedIsTLBes32),\
      (MMUMMU_READ_RAMMixedTLBes == (MMUMMU_READ_RAMMixedE)(((var) &\
	    MMU_MMU_READ_RAM_Mixed_MASK) >>\
      MMU_MMU_READ_RAM_Mixed_OFFSET)))


/********************************************************************/


#define MMUMMU_READ_RAMMixedIsCPUes32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_READ_RAMMixedIsCPUes32),\
      (MMUMMU_READ_RAMMixedCPUes == (MMUMMU_READ_RAMMixedE)(((var) &\
	    MMU_MMU_READ_RAM_Mixed_MASK) >>\
      MMU_MMU_READ_RAM_Mixed_OFFSET)))


/********************************************************************/


#define MMUMMU_EMU_FAULT_ADReadRegister32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_EMU_FAULT_ADReadRegister32),\
      RD_MEM_32_VOLATILE((baseAddress)+MMU_MMU_EMU_FAULT_AD_OFFSET))


/********************************************************************/


#define MMUMMU_EMU_FAULT_ADEmuFaultAddressRead32(baseAddress)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_EMU_FAULT_ADEmuFaultAddressRead32),\
      (((RD_MEM_32_VOLATILE(((baseAddress)+\
	    (MMU_MMU_EMU_FAULT_AD_OFFSET)))) &\
      MMU_MMU_EMU_FAULT_AD_EmuFaultAddress_MASK) >>\
      MMU_MMU_EMU_FAULT_AD_EmuFaultAddress_OFFSET))


/********************************************************************/


#define MMUMMU_EMU_FAULT_ADEmuFaultAddressGet32(var)\
    (_DEBUG_LEVEL_1_EASI(EASIL1_MMUMMU_EMU_FAULT_ADEmuFaultAddressGet32),\
      (UWORD32)(((var) & MMU_MMU_EMU_FAULT_AD_EmuFaultAddress_MASK)  >>\
      MMU_MMU_EMU_FAULT_AD_EmuFaultAddress_OFFSET))


/********************************************************************/


#endif	/* USE_LEVEL_1_MACROS */


/*****************************************************************************
* EXPORTED TYPES
******************************************************************************
*/

/*****************************************************************************
* EXPORTED VARIABLES
******************************************************************************
*/

/*****************************************************************************
* EXPORTED FUNCTIONS
******************************************************************************
*/


#if defined(__cplusplus)
}	/* End of C++ extern block */
#endif /* defined(__cplusplus) */

#endif /* _MMU_REG_ACM_H */
/* EOF */

