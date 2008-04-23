/*
 * dspbridge/inc/pwr_sh.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation version 2.1 of the License.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

/*
 *  ======== pwr_sh.h ========
 *
 *  Power Manager shared definitions (used on both GPP and DSP sides).
 *
 *! Revision History
 *! ================
 *! 17-Apr-2002 sg: Initial.
 */

#ifndef PWR_SH_
#define PWR_SH_

#include <mbx_sh.h>

#ifdef __cplusplus
extern "C" {
#endif

/* valid sleep command codes that can be sent by GPP via mailbox: */
#define PWR_DEEPSLEEP           MBX_PM_DSPIDLE
#define PWR_EMERGENCYDEEPSLEEP  MBX_PM_EMERGENCYSLEEP
#define PWR_SLEEPUNTILRESTART   MBX_PM_SLEEPUNTILRESTART
#define PWR_WAKEUP              MBX_PM_DSPWAKEUP
#define PWR_AUTOENABLE          MBX_PM_PWRENABLE
#define PWR_AUTODISABLE         MBX_PM_PWRDISABLE
#define PWR_RETENTION             MBX_PM_DSPRETN

#ifdef __cplusplus
}
#endif
#endif				/* PWR_SH_ */
