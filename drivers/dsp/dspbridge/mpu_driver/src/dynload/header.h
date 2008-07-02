/*
 * dspbridge/src/dynload/header.h
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



#define TRUE 1
#define FALSE 0
#ifndef NULL
#define NULL 0
#endif

#if defined(NO_STD_LIB)
#include "dl_stdlib.h"
#define DL_STRCMP  DL__strcmp
#else
#include <string.h>
#define DL_STRCMP  strcmp
#endif

/* maximum parenthesis nesting in relocation stack expressions */
#define STATIC_EXPR_STK_SIZE 10
#ifndef __KERNEL__
#include <stdint.h>
#else
#include <linux/types.h>
typedef unsigned int            uint_least32_t;
typedef unsigned short int	uint_least16_t;
#endif

#include "doff.h"
#include "dynamic_loader.h"
#include "params.h"
#include "dload_internal.h"
#include "reloc_table.h"

#if LEAD3
#define TI_C55X_REV2 "$TI_capability_requires_rev2"
#define TI_C55X_REV3 "$TI_capability_requires_rev3"
#define TI_C55X_MEM_MODEL "$TI_capability$C5500$MemoryModel"
#endif

/*
 * Plausibility limits
 *
 * These limits are imposed upon the input DOFF file as a check for validity.
 * They are hard limits, in that the load will fail if they are exceeded.
 * The numbers selected are arbitrary, in that the loader implementation does
 * not require these limits.
 */

/* maximum number of bytes in string table */
#define MAX_REASONABLE_STRINGTAB (0x100000)
/* maximum number of code,data,etc. sections */
#define MAX_REASONABLE_SECTIONS (200)
/* maximum number of linker symbols */
#define MAX_REASONABLE_SYMBOLS (100000)

/* shift count to align F_BIG with DLOAD_LITTLE */
#define ALIGN_COFF_ENDIANNESS 7
#define ENDIANNESS_MASK (DF_BYTE_ORDER >> ALIGN_COFF_ENDIANNESS)
