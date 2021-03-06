/*
 * Texas Instruments 3-Port Ethernet Switch Address Lookup Engine APIs
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __TI_CPTS_H__
#define __TI_CPTS_H__

typedef int (*cpts_extevent_cb)(int index, u64 timestamp);

extern int cpts_systime_write(u64 ns);
extern int cpts_systime_read(u64 *ns);
extern int cpts_ctrl_hwpush(int index, int state);
extern int cpts_set_hwevent_callback(cpts_extevent_cb cb);

#endif
