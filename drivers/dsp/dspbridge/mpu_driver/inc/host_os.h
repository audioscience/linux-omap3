/*
 * dspbridge/mpu_driver/inc/host_os.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
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
 *  ======== windows.h ========
 *
 *! Revision History
 *! ================
 *! 08-Mar-2004 sb Added cacheflush.h to support Dynamic Memory Mapping feature
 *! 16-Feb-2004 sb Added headers required for consistent_alloc
 */

#ifndef _HOST_OS_H_
#define _HOST_OS_H_

#ifdef __KERNEL__

#include <linux/autoconf.h>
#include <asm/system.h>
#include <asm/atomic.h>
#include <asm/semaphore.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/syscalls.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/vmalloc.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
/* #include <asm/arch/bus.h> */


#if defined(OMAP_2430) || defined(OMAP_3430)
#include <asm/arch/clock.h>
#ifdef OMAP_3430
#include <linux/clk.h>
/*  #include <asm-arm/hardware/clock.h> */
#endif
#endif

#include <linux/pagemap.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>

/*  ----------------------------------- Macros */

#define SEEK_SET        0	/* Seek from beginning of file.  */
#define SEEK_CUR        1	/* Seek from current position.  */
#define SEEK_END        2	/* Seek from end of file.  */


/* TODO -- Remove, once BP defines them */
#ifdef OMAP_3430
#define INT_MAIL_MPU_IRQ        26
#define INT_DSP_MMU_IRQ        28
#endif


#else

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#endif

#include <dbtype.h>

#endif
