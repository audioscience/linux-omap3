/*
 * dspbridge/inc/soc.h
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


#ifdef LTT_SOC

#include <linux/trace.h>

#define SOC_Create(x) \
	     trace_create_event("BRG", x, CUSTOM_EVENT_FORMAT_TYPE_STR, NULL);
#define SOC_Delete(x) trace_destroy_event(x);
#define SOC_Trace(x, ...) trace_std_formatted_event(x, __VA_ARGS__);

#endif
