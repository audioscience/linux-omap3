/*
 * dspbridge/inc/gp.h
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


/*
 *  ======== gp.h ========
 *! Revision History
 *! ================
 */

#ifndef GP_
#define GP_

#include <stdarg.h>

/*
 *  ======== GP_exit ========
 */
#define GP_exit()

/*
 *  ======== GP_init ========
 */
#define GP_init()

/*
 *  ======== GP_printf ========
 */
extern void GP_printf(char *fmt, ...);

/*
 *  ======== GP_putchar ========
 */
extern void GP_putchar(char c);

/*
 *  ======== GP_snprintf ========
 */
#define GP_snprintf snprintf

/*
 *  ======== GP_vprintf ========
 */
extern void GP_vprintf(char *fmt, va_list va);

/*
 *  ======== GP_vsprintf ========
 */
extern void GP_vsprintf(char *buf, char *fmt, va_list va);

#endif
