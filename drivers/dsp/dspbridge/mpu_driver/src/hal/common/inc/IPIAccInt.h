/*
 * dspbridge/src/hal/common/regdefs/inc/arm11/IPIAccInt.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef _IPI_ACC_INT_H
#define _IPI_ACC_INT_H

#if defined(__cplusplus)
extern "C" {
#endif				/* defined(__cplusplus) */

/*****************************************************************************
* EXPORTED DEFINITIONS
******************************************************************************
*/

/* Mappings of level 1 EASI function numbers to function names */

#define EASIL1_IPIIPI_REVISIONReadRegister32     IPI_BASE_EASIL1 + 0
#define EASIL1_IPIIPI_REVISIONReadRegister32U    IPI_BASE_EASIL1 + 1
#define EASIL1_IPIIPI_REVISIONReadRegister32L    IPI_BASE_EASIL1 + 2
#define EASIL1_IPIIPI_REVISIONRevRead32         IPI_BASE_EASIL1 + 3
#define EASIL1_IPIIPI_REVISIONRevRead32L        IPI_BASE_EASIL1 + 4
#define EASIL1_IPIIPI_REVISIONRevGet32          IPI_BASE_EASIL1 + 5
#define EASIL1_IPIIPI_REVISIONRevGet32L          IPI_BASE_EASIL1 + 6
#define EASIL1_IPIIPI_SYSCONFIGReadRegister32    IPI_BASE_EASIL1 + 7
#define EASIL1_IPIIPI_SYSCONFIGReadRegister32U  IPI_BASE_EASIL1 + 8
#define EASIL1_IPIIPI_SYSCONFIGReadRegister32L   IPI_BASE_EASIL1 + 9
#define EASIL1_IPIIPI_SYSCONFIGWriteRegister32  IPI_BASE_EASIL1 + 10
#define EASIL1_IPIIPI_SYSCONFIGWriteRegister32U IPI_BASE_EASIL1 + 11
#define EASIL1_IPIIPI_SYSCONFIGWriteRegister32L  IPI_BASE_EASIL1 + 12
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleRead32    IPI_BASE_EASIL1 + 13
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleRead32L   IPI_BASE_EASIL1 + 14
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleReadIsclkfree32 IPI_BASE_EASIL1 + 15
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleReadIsclkfree32L IPI_BASE_EASIL1 + 16
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleReadIsautoclkgate32 \
						IPI_BASE_EASIL1 + 17
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleReadIsautoclkgate32L  \
						IPI_BASE_EASIL1 + 18
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleGet32       IPI_BASE_EASIL1 + 19
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleGet32L      IPI_BASE_EASIL1 + 20
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleIsclkfree32  IPI_BASE_EASIL1 + 21
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleIsclkfree32L    IPI_BASE_EASIL1 + 22
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleIsautoclkgate32  \
						IPI_BASE_EASIL1 + 23
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleIsautoclkgate32L  \
						IPI_BASE_EASIL1 + 24
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleWrite32   IPI_BASE_EASIL1 + 25
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleWrite32L   IPI_BASE_EASIL1 + 26
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleWriteclkfree32  IPI_BASE_EASIL1 + 27
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleWriteclkfree32L  IPI_BASE_EASIL1 + 28
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleWriteautoclkgate32 \
						IPI_BASE_EASIL1 + 29
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleWriteautoclkgate32L \
						IPI_BASE_EASIL1 + 30
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleSet32   IPI_BASE_EASIL1 + 31
#define EASIL1_IPIIPI_SYSCONFIGAutoIdleSet32L  IPI_BASE_EASIL1 + 32
#define EASIL1_IPIIPI_INDEXReadRegister32       IPI_BASE_EASIL1 + 33
#define EASIL1_IPIIPI_INDEXReadRegister32U      IPI_BASE_EASIL1 + 34
#define EASIL1_IPIIPI_INDEXReadRegister32L     IPI_BASE_EASIL1 + 35
#define EASIL1_IPIIPI_INDEXWriteRegister32      IPI_BASE_EASIL1 + 36
#define EASIL1_IPIIPI_INDEXWriteRegister32U    IPI_BASE_EASIL1 + 37
#define EASIL1_IPIIPI_INDEXWriteRegister32L    IPI_BASE_EASIL1 + 38
#define EASIL1_IPIIPI_INDEXPageIndexValueRead32 IPI_BASE_EASIL1 + 39
#define EASIL1_IPIIPI_INDEXPageIndexValueRead32L   IPI_BASE_EASIL1 + 40
#define EASIL1_IPIIPI_INDEXPageIndexValueGet32     IPI_BASE_EASIL1 + 41
#define EASIL1_IPIIPI_INDEXPageIndexValueGet32L    IPI_BASE_EASIL1 + 42
#define EASIL1_IPIIPI_INDEXPageIndexValueWrite32   IPI_BASE_EASIL1 + 43
#define EASIL1_IPIIPI_INDEXPageIndexValueWrite32L  IPI_BASE_EASIL1 + 44
#define EASIL1_IPIIPI_INDEXPageIndexValueSet32     IPI_BASE_EASIL1 + 45
#define EASIL1_IPIIPI_INDEXPageIndexValueSet32L    IPI_BASE_EASIL1 + 46
#define EASIL1_IPIIPI_ENTRYReadRegister32          IPI_BASE_EASIL1 + 47
#define EASIL1_IPIIPI_ENTRYReadRegister32U         IPI_BASE_EASIL1 + 48
#define EASIL1_IPIIPI_ENTRYReadRegister32L         IPI_BASE_EASIL1 + 49
#define EASIL1_IPIIPI_ENTRYWriteRegister32         IPI_BASE_EASIL1 + 50
#define EASIL1_IPIIPI_ENTRYWriteRegister32U        IPI_BASE_EASIL1 + 51
#define EASIL1_IPIIPI_ENTRYWriteRegister32L        IPI_BASE_EASIL1 + 52
#define EASIL1_IPIIPI_ENTRYElemSizeValueRead32     IPI_BASE_EASIL1 + 53
#define EASIL1_IPIIPI_ENTRYElemSizeValueRead32L    IPI_BASE_EASIL1 + 54
#define EASIL1_IPIIPI_ENTRYElemSizeValueReadIsElemSz8b32  \
						IPI_BASE_EASIL1 + 55
#define EASIL1_IPIIPI_ENTRYElemSizeValueReadIsElemSz8b32L   \
						IPI_BASE_EASIL1 + 56
#define EASIL1_IPIIPI_ENTRYElemSizeValueReadIsElemSz16b32    \
						IPI_BASE_EASIL1 + 57
#define EASIL1_IPIIPI_ENTRYElemSizeValueReadIsElemSz16b32L   \
						IPI_BASE_EASIL1 + 58
#define EASIL1_IPIIPI_ENTRYElemSizeValueReadIsElemSz32b32      \
						IPI_BASE_EASIL1 + 59
#define EASIL1_IPIIPI_ENTRYElemSizeValueReadIsElemSz32b32L    \
						IPI_BASE_EASIL1 + 60
#define EASIL1_IPIIPI_ENTRYElemSizeValueReadIsReserved32    \
						IPI_BASE_EASIL1 + 61
#define EASIL1_IPIIPI_ENTRYElemSizeValueReadIsReserved32L  \
						IPI_BASE_EASIL1 + 62
#define EASIL1_IPIIPI_ENTRYElemSizeValueGet32     IPI_BASE_EASIL1 + 63
#define EASIL1_IPIIPI_ENTRYElemSizeValueGet32L    IPI_BASE_EASIL1 + 64
#define EASIL1_IPIIPI_ENTRYElemSizeValueIsElemSz8b32   IPI_BASE_EASIL1 + 65
#define EASIL1_IPIIPI_ENTRYElemSizeValueIsElemSz8b32L IPI_BASE_EASIL1 + 66
#define EASIL1_IPIIPI_ENTRYElemSizeValueIsElemSz16b32  IPI_BASE_EASIL1 + 67
#define EASIL1_IPIIPI_ENTRYElemSizeValueIsElemSz16b32L IPI_BASE_EASIL1 + 68
#define EASIL1_IPIIPI_ENTRYElemSizeValueIsElemSz32b32  IPI_BASE_EASIL1 + 69
#define EASIL1_IPIIPI_ENTRYElemSizeValueIsElemSz32b32L IPI_BASE_EASIL1 + 70
#define EASIL1_IPIIPI_ENTRYElemSizeValueIsReserved32   IPI_BASE_EASIL1 + 71
#define EASIL1_IPIIPI_ENTRYElemSizeValueIsReserved32L  IPI_BASE_EASIL1 + 72
#define EASIL1_IPIIPI_ENTRYElemSizeValueWrite32       IPI_BASE_EASIL1 + 73
#define EASIL1_IPIIPI_ENTRYElemSizeValueWrite32L    IPI_BASE_EASIL1 + 74
#define EASIL1_IPIIPI_ENTRYElemSizeValueWriteElemSz8b32    \
						IPI_BASE_EASIL1 + 75
#define EASIL1_IPIIPI_ENTRYElemSizeValueWriteElemSz8b32L  \
						IPI_BASE_EASIL1 + 76
#define EASIL1_IPIIPI_ENTRYElemSizeValueWriteElemSz16b32     \
						IPI_BASE_EASIL1 + 77
#define EASIL1_IPIIPI_ENTRYElemSizeValueWriteElemSz16b32L    \
						IPI_BASE_EASIL1 + 78
#define EASIL1_IPIIPI_ENTRYElemSizeValueWriteElemSz32b32      \
						IPI_BASE_EASIL1 + 79
#define EASIL1_IPIIPI_ENTRYElemSizeValueWriteElemSz32b32L   \
						IPI_BASE_EASIL1 + 80
#define EASIL1_IPIIPI_ENTRYElemSizeValueWriteReserved32    \
						IPI_BASE_EASIL1 + 81
#define EASIL1_IPIIPI_ENTRYElemSizeValueWriteReserved32L    \
						IPI_BASE_EASIL1 + 82
#define EASIL1_IPIIPI_ENTRYElemSizeValueSet32     IPI_BASE_EASIL1 + 83
#define EASIL1_IPIIPI_ENTRYElemSizeValueSet32L    IPI_BASE_EASIL1 + 84
#define EASIL1_IPIIPI_ENABLEReadRegister32        IPI_BASE_EASIL1 + 85
#define EASIL1_IPIIPI_ENABLEReadRegister32U      IPI_BASE_EASIL1 + 86
#define EASIL1_IPIIPI_ENABLEReadRegister32L       IPI_BASE_EASIL1 + 87
#define EASIL1_IPIIPI_ENABLEWriteRegister32       IPI_BASE_EASIL1 + 88
#define EASIL1_IPIIPI_ENABLEWriteRegister32U     IPI_BASE_EASIL1 + 89
#define EASIL1_IPIIPI_ENABLEWriteRegister32L     IPI_BASE_EASIL1 + 90
#define EASIL1_IPIIPI_ENABLEEnableRead32          IPI_BASE_EASIL1 + 91
#define EASIL1_IPIIPI_ENABLEEnableRead32L         IPI_BASE_EASIL1 + 92
#define EASIL1_IPIIPI_ENABLEEnableGet32           IPI_BASE_EASIL1 + 93
#define EASIL1_IPIIPI_ENABLEEnableGet32L           IPI_BASE_EASIL1 + 94
#define EASIL1_IPIIPI_ENABLEEnableWrite32         IPI_BASE_EASIL1 + 95
#define EASIL1_IPIIPI_ENABLEEnableWrite32L        IPI_BASE_EASIL1 + 96
#define EASIL1_IPIIPI_ENABLEEnableSet32           IPI_BASE_EASIL1 + 97
#define EASIL1_IPIIPI_ENABLEEnableSet32L          IPI_BASE_EASIL1 + 98
#define EASIL1_IPIIPI_IOMAPReadRegister32         IPI_BASE_EASIL1 + 99
#define EASIL1_IPIIPI_IOMAPReadRegister32U        IPI_BASE_EASIL1 + 100
#define EASIL1_IPIIPI_IOMAPReadRegister32L       IPI_BASE_EASIL1 + 101
#define EASIL1_IPIIPI_IOMAPWriteRegister32        IPI_BASE_EASIL1 + 102
#define EASIL1_IPIIPI_IOMAPWriteRegister32U       IPI_BASE_EASIL1 + 103
#define EASIL1_IPIIPI_IOMAPWriteRegister32L       IPI_BASE_EASIL1 + 104
#define EASIL1_IPIIPI_IOMAPIOMapBaseAddrRead32    IPI_BASE_EASIL1 + 105
#define EASIL1_IPIIPI_IOMAPIOMapBaseAddrRead32L   IPI_BASE_EASIL1 + 106
#define EASIL1_IPIIPI_IOMAPIOMapBaseAddrGet32     IPI_BASE_EASIL1 + 107
#define EASIL1_IPIIPI_IOMAPIOMapBaseAddrGet32L    IPI_BASE_EASIL1 + 108
#define EASIL1_IPIIPI_IOMAPIOMapBaseAddrWrite32   IPI_BASE_EASIL1 + 109
#define EASIL1_IPIIPI_IOMAPIOMapBaseAddrWrite32L   IPI_BASE_EASIL1 + 110
#define EASIL1_IPIIPI_IOMAPIOMapBaseAddrSet32      IPI_BASE_EASIL1 + 111
#define EASIL1_IPIIPI_IOMAPIOMapBaseAddrSet32L    IPI_BASE_EASIL1 + 112
#define EASIL1_IPIIPI_DSPBOOTCFGReadRegister32    IPI_BASE_EASIL1 + 113
#define EASIL1_IPIIPI_DSPBOOTCFGReadRegister32U   IPI_BASE_EASIL1 + 114
#define EASIL1_IPIIPI_DSPBOOTCFGReadRegister32L   IPI_BASE_EASIL1 + 115
#define EASIL1_IPIIPI_DSPBOOTCFGWriteRegister32   IPI_BASE_EASIL1 + 116
#define EASIL1_IPIIPI_DSPBOOTCFGWriteRegister32U  IPI_BASE_EASIL1 + 117
#define EASIL1_IPIIPI_DSPBOOTCFGWriteRegister32L  IPI_BASE_EASIL1 + 118
#define EASIL1_IPIIPI_DSPBOOTCFGDSPBootModRead32  IPI_BASE_EASIL1 + 119
#define EASIL1_IPIIPI_DSPBOOTCFGDSPBootModRead32L  IPI_BASE_EASIL1 + 120
#define EASIL1_IPIIPI_DSPBOOTCFGDSPBootModGet32   IPI_BASE_EASIL1 + 121
#define EASIL1_IPIIPI_DSPBOOTCFGDSPBootModGet32L  IPI_BASE_EASIL1 + 122
#define EASIL1_IPIIPI_DSPBOOTCFGDSPBootModWrite32 IPI_BASE_EASIL1 + 123
#define EASIL1_IPIIPI_DSPBOOTCFGDSPBootModWrite32L    \
						IPI_BASE_EASIL1 + 124
#define EASIL1_IPIIPI_DSPBOOTCFGDSPBootModSet32    IPI_BASE_EASIL1 + 125
#define EASIL1_IPIIPI_DSPBOOTCFGDSPBootModSet32L   IPI_BASE_EASIL1 + 126

/* Register offset address definitions */

#define IPI_IPI_REVISION_OFFSET                 0x0
#define IPI_IPI_SYSCONFIG_OFFSET                0x10
#define IPI_IPI_INDEX_OFFSET                   0x40
#define IPI_IPI_ENTRY_OFFSET                  0x44
#define IPI_IPI_ENABLE_OFFSET                 0x48
#define IPI_IPI_IOMAP_OFFSET                  0x4c
#define IPI_IPI_DSPBOOTCFG_OFFSET                0x50

/* Bitfield mask and offset declarations */

#define IPI_IPI_REVISION_Rev_MASK                0xff
#define IPI_IPI_REVISION_Rev_OFFSET              0
#define IPI_IPI_REVISION_Rev_LO_MASK            0xff
#define IPI_IPI_REVISION_Rev_LO_OFFSET           0
#define IPI_IPI_SYSCONFIG_AutoIdle_MASK          0x1
#define IPI_IPI_SYSCONFIG_AutoIdle_OFFSET         0
#define IPI_IPI_SYSCONFIG_AutoIdle_LO_MASK         0x1
#define IPI_IPI_SYSCONFIG_AutoIdle_LO_OFFSET      0
#define IPI_IPI_INDEX_PageIndexValue_MASK         0xffff
#define IPI_IPI_INDEX_PageIndexValue_OFFSET       0
#define IPI_IPI_INDEX_PageIndexValue_LO_MASK      0xffff
#define IPI_IPI_INDEX_PageIndexValue_LO_OFFSET      0
#define IPI_IPI_ENTRY_ElemSizeValue_MASK            0x3
#define IPI_IPI_ENTRY_ElemSizeValue_OFFSET         0
#define IPI_IPI_ENTRY_ElemSizeValue_LO_MASK          0x3
#define IPI_IPI_ENTRY_ElemSizeValue_LO_OFFSET        0
#define IPI_IPI_ENABLE_Enable_MASK                         0x1
#define IPI_IPI_ENABLE_Enable_OFFSET                  0
#define IPI_IPI_ENABLE_Enable_LO_MASK                0x1
#define IPI_IPI_ENABLE_Enable_LO_OFFSET              0
#define IPI_IPI_IOMAP_IOMapBaseAddr_MASK              0x3f
#define IPI_IPI_IOMAP_IOMapBaseAddr_OFFSET         0
#define IPI_IPI_IOMAP_IOMapBaseAddr_LO_MASK           0x3f
#define IPI_IPI_IOMAP_IOMapBaseAddr_LO_OFFSET       0
#define IPI_IPI_DSPBOOTCFG_DSPBootMod_MASK           0xf
#define IPI_IPI_DSPBOOTCFG_DSPBootMod_OFFSET         0
#define IPI_IPI_DSPBOOTCFG_DSPBootMod_LO_MASK       0xf
#define IPI_IPI_DSPBOOTCFG_DSPBootMod_LO_OFFSET      0
#define SYSC_IVA2BOOTMOD_OFFSET                   0x404
#define SYSC_IVA2BOOTMOD_MASK                      0xf
#define SYSC_IVA2BOOTADDR_OFFSET                0x400
#define SYSC_IVA2BOOTADDR_MASK                 0xfffffc00

/**************************************************************************
* EXPORTED TYPES
*********************************************************************** */

/* The following type defs represent the enumerated values for each bitfield */

typedef enum {
	IPIIPI_SYSCONFIGAutoIdleclkfree = 0x0000,
	IPIIPI_SYSCONFIGAutoIdleautoclkgate = 0x0001
} IPIIPI_SYSCONFIGAutoIdleE;

typedef enum {
	IPIIPI_ENTRYElemSizeValueElemSz8b = 0x0000,
	IPIIPI_ENTRYElemSizeValueElemSz16b = 0x0001,
	IPIIPI_ENTRYElemSizeValueElemSz32b = 0x0002,
	IPIIPI_ENTRYElemSizeValueReserved = 0x0003
} IPIIPI_ENTRYElemSizeValueE;

/*****************************************************************************
* EXPORTED VARIABLES
******************************************************************************
*/

/*****************************************************************************
* EXPORTED FUNCTIONS
******************************************************************************
*/

#if defined(__cplusplus)
}				/* End of C++ extern block */
#endif				/* defined(__cplusplus) */
#endif				/* _IPI_ACC_INT_H */
/* EOF */
