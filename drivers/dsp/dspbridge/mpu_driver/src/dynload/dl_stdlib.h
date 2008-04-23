/*
 * dspbridge/src/dynload/dl_stdlib.h
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



#ifndef __DL_STDLIB__
#define __DL_STDLIB__
/*
 * Standard library functions for DL usage
 *
 * This module is used for systems where the standard library functions
 *  are not accessible from the dynamic loader and system-wide alternatives
 *  have been provided
 */

extern unsigned DL__strlen(const char *string);
#define strlen DL__strlen

extern int DL__strcmp(register const char *string1,
		      register const char *string2);
#define strcmp DL__strcmp

#endif				/* __DL_STDLIB__ */
