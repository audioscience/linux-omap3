/*
 * dspbridge/src/rmgr/linux/common/drv_interface.c
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
 *  ======== linux_driver.c ========
 *  Description:
 *      DSP/BIOS Bridge driver interface.
 *
 *  Public Functions:
 *      driver_init
 *      driver_exit
 *      driver_open
 *      driver_release
 *      driver_ioctl
 *      driver_mmap
 *
 *! Revision History
 *! ================
 *! 21-Apr-2004 map   Deprecated use of MODULE_PARM for kernel versions
 *!		   greater than 2.5, use module_param.
 *! 08-Mar-2004 sb    Added the dsp_debug argument, which keeps the DSP in self
 *!		   loop after image load and waits in a loop for DSP to start
 *! 16-Feb-2004 vp    Deprecated the usage of MOD_INC_USE_COUNT and
 *! 						MOD_DEC_USE_COUNT
 *!		   for kernel versions greater than 2.5
 *! 20-May-2003 vp    Added unregister functions for the DPM.
 *! 24-Mar-2003 sb    Pass pid instead of driverContext to DSP_Close
 *! 24-Mar-2003 vp    Added Power Management support.
 *! 21-Mar-2003 sb    Configure SHM size using insmod argument shm_size
 *! 10-Feb-2003 vp    Updated based on code review comments
 *! 18-Oct-2002 sb    Created initial version
 */

/*  ----------------------------------- Host OS */

#if 0
#if defined(CONFIG_MODVERSIONS)
#define MODVERSIONS
#include <linux/modversions.h>
#endif
#endif


#include <host_os.h>
#include <linux/platform_device.h>
#include <linux/pm.h>


#ifdef MODULE
#include <linux/module.h>
#endif

#include <linux/device.h>
#include <linux/init.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#ifndef DISABLE_BRIDGE_PM
#ifndef DISABLE_BRIDGE_DVFS
#include <asm/arch/resource.h>
#endif
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
#include <asm/arch/prcm_34xx.h>
#else
#include <asm/arch/prcm.h>
#endif
#endif

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <gt.h>
#include <dbc.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <osal.h>
#include <sync.h>
#include <reg.h>
#include <csl.h>
#include <prcs.h>

/*  ----------------------------------- Platform Manager */
#include <wcdioctl.h>
#include <_dcd.h>
#include <dspdrv.h>
#include <dbreg.h>

/*  ----------------------------------- Resource Manager */
#include <pwr.h>

/*  ----------------------------------- This */
#include <drv_interface.h>

#ifdef LTT_SOC
#include <soc.h>
#endif

#ifndef RES_CLEANUP_DISABLE
#include <cfg.h>
#include <resourcecleanup.h>
#include <chnl.h>
#include <proc.h>
#include <cfg.h>
#include <dev.h>
#include <drvdefs.h>
#include <drv.h>
#include <dbreg.h>
#define OMAP24xx_DSP_DEVID 1
#endif

#define BRIDGE_NAME "C6410"
/*  ----------------------------------- Globals */
#define DRIVER_NAME  "DspBridge"
#define DRIVER_MAJOR 0		/* Linux assigns our Major device number */
#define DRIVER_MINOR 0		/* Linux assigns our Major device number */
INT dsp_debug = 0;
INT dsp_inact_time = 5000;
/* This is a test variable used by Bridge to test different sleep states */
INT dsp_test_sleepstate = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
struct bridge_dev {
	struct cdev cdev;
};

struct bridge_dev *bridge_device = NULL;

static struct class *bridge_class = NULL;
#endif

DWORD driverContext;
CHAR *GT_str = NULL;
INT driver_major = DRIVER_MAJOR;
INT driver_minor = DRIVER_MINOR;
CHAR *base_img = NULL;
CHAR *iva_img = NULL;
CHAR *num_procs = "C55=1";
INT shm_size = 0x150000;	/* 1 MB */
INT iva_extmem_size = 0x0;	/* 0 KB */

#ifdef LTT_SOC
ULONG GSC, GSI, GSR, GMW, GMR, GNP, GNG, GNC;
#endif

UINT phys_mempool_base = 0x0;
UINT phys_mempool_size = 0x0;
#if !defined(OMAP_2430) && !defined(OMAP_3430)
BOOL tc_wordswapon = TRUE;	/* Default value is always TRUE */
#else
BOOL tc_wordswapon = FALSE;	/* Default value is always TRUE */
#endif



#ifndef DISABLE_BRIDGE_PM
struct omap24xx_bridge_suspend_data {
	int suspended;
	wait_queue_head_t suspend_wq;
};

static struct omap24xx_bridge_suspend_data bridge_suspend_data;

int omap24xxbridge_suspend_lockout(struct omap24xx_bridge_suspend_data *s,
				  struct file *f)
{
    if ((s)->suspended) {
	if ((f)->f_flags & O_NONBLOCK)
	    return DSP_EDPMSUSPEND;
	wait_event_interruptible((s)->suspend_wq, (s)->suspended == 0);
	}
	return(0);
}

#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
#ifdef GT_TRACE
MODULE_PARM(GT_str, "s");
MODULE_PARM_DESC(GT_str, "GT string, default = NULL");

MODULE_PARM(dsp_debug, "i");
MODULE_PARM_DESC(dsp_debug, "Wait after loading DSP image. default = FALSE");
#else
#ifndef DDSP_DEBUG_PRODUCT
EXPORT_NO_SYMBOLS;
#endif
#endif

MODULE_PARM(driver_major, "i");	/* Driver's major number */
MODULE_PARM_DESC(driver_major, "Major device number, default = 0 (auto)");

MODULE_PARM(driver_minor, "i");	/* Driver's major number */
MODULE_PARM_DESC(driver_minor, "Minor device number, default = 0 (auto)");

MODULE_PARM(dsp_inact_time, "i");
MODULE_PARM_DESC(dsp_inact_time, "DSP Inactivity time value = 5000");

MODULE_PARM(base_img, "s");
MODULE_PARM_DESC(base_img, "DSP base image, default = NULL");
MODULE_PARM(iva_img, "s");
MODULE_PARM_DESC(iva_img, "IVA base image, default = NULL");
MODULE_PARM(num_procs, "s");
MODULE_PARM_DESC(num_procs,
		"Number of DSP processors , default = IVA=1, C55=1");

MODULE_PARM(shm_size, "i");
MODULE_PARM_DESC(shm_size, "SHM size, default = 512 KB, minimum = 64 KB");
MODULE_PARM(iva_extmem_size, "i");
MODULE_PARM_DESC(iva_extmem_size, "IVAEXTMEM size, default = 0 KB");
MODULE_PARM(phys_mempool_base, "i");
MODULE_PARM_DESC(phys_mempool_base,
		 "Physical memory pool base passed to driver");

MODULE_PARM(phys_mempool_size, "i");
MODULE_PARM_DESC(phys_mempool_size,
		 "Physical memory pool size passed to driver");
MODULE_PARM(tc_wordswapon, "i");
MODULE_PARM_DESC(tc_wordswapon, "TC Word Swap Option. default = TRUE");

#else				/* Kernel 2.6.x */

#ifdef DEBUG
module_param(GT_str, charp, 0);
MODULE_PARM_DESC(GT_str, "GT string, default = NULL");

module_param(dsp_debug, int, 0);
MODULE_PARM_DESC(dsp_debug, "Wait after loading DSP image. default = FALSE");
#else
#ifndef DDSP_DEBUG_PRODUCT
EXPORT_NO_SYMBOLS;
#endif
#endif

module_param(driver_major, int, 0);	/* Driver's major number */
MODULE_PARM_DESC(driver_major, "Major device number, default = 0 (auto)");

module_param(driver_minor, int, 0);	/* Driver's major number */
MODULE_PARM_DESC(driver_minor, "Minor device number, default = 0 (auto)");

module_param(dsp_inact_time, int, 0);
MODULE_PARM_DESC(dsp_inact_time, "DSP Inactivity time value = 5000");

module_param(dsp_test_sleepstate, int, 0);
MODULE_PARM_DESC(dsp_test_sleepstate, "DSP Sleep state = 0");

module_param(base_img, charp, 0);
MODULE_PARM_DESC(base_img, "DSP base image, default = NULL");
module_param(iva_img, charp, 0);
MODULE_PARM_DESC(iva_img, "IVA base image, default = NULL");
module_param(num_procs, charp, 0);
MODULE_PARM_DESC(num_procs, "Number of processors , default = IVA=1, C55=1");

module_param(shm_size, int, 0);
MODULE_PARM_DESC(shm_size, "SHM size, default = 512 KB, minimum = 64 KB");
module_param(iva_extmem_size, int, 0);
MODULE_PARM_DESC(iva_extmem_size, "IVAEXTMEM size, default = 0 KB");

module_param(phys_mempool_base, int, 0);
MODULE_PARM_DESC(phys_mempool_base,
		"Physical memory pool base passed to driver");

module_param(phys_mempool_size, int, 0);
MODULE_PARM_DESC(phys_mempool_size,
		"Physical memory pool size passed to driver");
module_param(tc_wordswapon, bool, 0);
MODULE_PARM_DESC(tc_wordswapon, "TC Word Swap Option. default = TRUE");
#endif

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");

char *driver_name = DRIVER_NAME;
struct GT_Mask driverTrace;

struct file_operations bridge_fops = {
      open:bridge_open,
      release:bridge_release,
      ioctl:bridge_ioctl,
      mmap:bridge_mmap
};

#ifndef DISABLE_BRIDGE_PM
DWORD timeOut = 1000;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0)
static int bridge_suspend(struct device *dev, u32 state, u32 level);
static int bridge_resume(struct device *dev, u32 level);
#else
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
static int bridge_suspend(struct device *dev, u32 state);
static int bridge_resume(struct device *dev);
#else
static int bridge_suspend(struct platform_device *pdev, pm_message_t state);
static int bridge_resume(struct platform_device *pdev);
#endif
#endif
#endif

static void bridge_free(struct device *dev);

#if defined(OMAP_2430) || defined(OMAP_3430)
static int
omap24xx_bridge_probe(struct omap_dev *dev)
{
	return 0;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)

struct omap_dev bridge_dsp_ldm = {
	.name = BRIDGE_NAME,
	.devid = OMAP24xx_DSP_DEVID,
	.busid = OMAP_BUS_L3,
	.dev = {
		.release = bridge_free,
		},
	.irq =  {
		INT_MAIL_MPU_IRQ,
		INT_DSP_MMU_IRQ,
		},
};
#else
 struct platform_device omap_dspbridge_dev = {
		.name = BRIDGE_NAME,
		.id = -1,
		.num_resources = 0,
		.dev = {
		.release = bridge_free,
		},
		.resource = NULL,
};

#if 0
struct device_driver dsp_driver = {
	.name = "DspBridge",
};

struct device dsp_device = {
	.driver = &dsp_driver,
};
#endif
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
 #ifndef DISABLE_BRIDGE_PM
 #ifndef DISABLE_BRIDGE_DVFS
/* The number of OPPs supported in the system */
INT dsp_max_opps = CO_VDD1_OPP5;
UINT vdd1_dsp_freq[6][4] = {

	 {0, 0, 0, 0},

	 {0, 90000, 0, 86000},

	{0, 180000, 80000, 170000},

	{0, 360000, 160000, 340000},

	{0, 396000, 325000, 376000},

	{0, 430000, 355000, 430000},
};

/* The handle for setting constraints */
struct constraint_handle *dsp_constraint_handle = NULL;

static int dspbridge_pre_scale(struct notifier_block *op, unsigned long level,
				void *ptr)
{
	PWR_PM_PreScale(PRCM_VDD1, level);
	return 0;
}

static int dspbridge_post_scale(struct notifier_block *op, unsigned long level,
				void *ptr)
{
	PWR_PM_PostScale(PRCM_VDD1, level);
	return 0;
}


static struct notifier_block omap34xxbridge_pre_scale = {
	.notifier_call = dspbridge_pre_scale,
	NULL,
};

static struct notifier_block omap34xxbridge_post_scale = {
	.notifier_call = dspbridge_post_scale,
	NULL,
};
static struct constraint_id cnstr_id_vdd1 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd1_opp",
};
#endif
#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0)

static struct omap_driver bridge_driver_ldm = {
      drv:{
	      name:"DspBridge",
	 },
      devid:OMAP24xx_DSP_DEVID, /*check if this device ID is valid in 2420 BP*/
      busid:OMAP_BUS_L3,
      clocks:0,
      probe:omap24xx_bridge_probe,
 #ifndef DISABLE_BRIDGE_PM
      suspend:bridge_suspend,
      resume:bridge_resume,
 #endif
      remove:NULL,
};
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
static struct omap_driver bridge_driver_ldm = {
      .drv = {
	      .name = "DspBridge",
	 },
      .devid = OMAP24xx_DSP_DEVID, /*check if this device ID is valid in 2420
									BP*/
      .busid = OMAP_BUS_L3,
      .clocks = 0,
      .probe = omap24xx_bridge_probe,
 #ifndef DISABLE_BRIDGE_PM
      .suspend = bridge_suspend,
      .resume = bridge_resume,
#endif
      .remove = NULL,
};
#else
static struct platform_driver bridge_driver_ldm = {
      .driver = {
	      .owner	= THIS_MODULE,
	      .name     = BRIDGE_NAME,
	 },
      .probe = omap24xx_bridge_probe,
 #ifndef DISABLE_BRIDGE_PM
      .suspend = bridge_suspend,
      .resume = bridge_resume,
#endif
      .shutdown = NULL,
      .remove = NULL,

};


struct device dspbridge_device = {
	.driver = &bridge_driver_ldm.driver,
};

#endif

#endif




#if defined(OMAP_2430) || defined(OMAP_3430)
static int bridge_driver_register(void)
{
	int retVal;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)

	extern int omap_driver_register(struct omap_driver *driver);
	retVal = omap_driver_register(&bridge_driver_ldm);
	GT_1trace(driverTrace, GT_1CLASS,
		"omap_driver_register returned with Value 0x%x \n", retVal);
	if (retVal)
		GT_0trace(driverTrace, GT_7CLASS,
			"Failed to register the driver to DPM !!");
#endif
	return retVal;
}

static int bridge_device_register(void)
{
	int retVal;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	extern int omap_device_register(struct omap_dev *device);
	retVal = omap_device_register(&bridge_dsp_ldm);
	GT_1trace(driverTrace, GT_1CLASS,
		"omap_device_register returned with Value 0x%x \n", retVal);
	if (retVal)
		GT_0trace(driverTrace, GT_7CLASS,
			"Failed to register the device to DPM !!");
#endif
	return retVal;
}

static void bridge_driver_unregister(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	extern void omap_driver_unregister(struct omap_driver *driver);
	omap_driver_unregister(&bridge_driver_ldm);
#endif
}

static void bridge_device_unregister(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	extern void omap_device_unregister(struct omap_dev *device);
	omap_device_unregister(&bridge_dsp_ldm);
#endif
}

#else				/* OMAP_2430 */

static struct device bridge_device_ldm = {
      name:"C5510",
      bus_id:" ",
      driver:NULL,
      power_state:DPM_POWER_ON,
};

static void bridge_driver_register(void)
{
	extern void mpu_public_driver_register(struct device_driver *driver);
	mpu_public_driver_register(&bridge_driver_ldm);
}

static void bridge_device_register(void)
{
	extern void mpu_public_device_register(struct device *driver);
	mpu_public_device_register(&bridge_device_ldm);
}

static void bridge_driver_unregister(void)
{
	extern void mpu_public_driver_unregister(struct device_driver *driver);
	mpu_public_driver_unregister(&bridge_driver_ldm);
}

static void bridge_device_unregister(void)
{
	extern void mpu_public_device_unregister(struct device *driver);
	mpu_public_device_unregister(&bridge_device_ldm);
}
#endif				/* OMAP_24xx */
#endif

/*
 * Purpose:
 *     Initialization routine. Executed when the driver is loaded (as a kernel
 *     module),
 *     or when the system is booted (when included as part of the kernel image).
 */
int bridge_init()
{
	int status;
	DWORD initStatus;
	DWORD temp;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
	dev_t   dev = 0 ;
	int     result;

	/* use 2.6 device model */
	if (driver_major) {
		dev = MKDEV(driver_major, driver_minor);
		result = register_chrdev_region(dev, 1, driver_name);
	} else {
		result = alloc_chrdev_region(&dev, driver_minor, 1,
					    driver_name);
		driver_major = MAJOR(dev);
	}

	if (result < 0) {
		printk("bridge_init: Can't get Major %d \n", driver_major);
		return result;
	}

	bridge_device = kmalloc(sizeof(struct bridge_dev), GFP_KERNEL);
	if (!bridge_device) {
		result = -ENOMEM;
		unregister_chrdev_region(dev, 1);
		return result;
	}
	memset(bridge_device, 0, sizeof(struct bridge_dev));
	cdev_init(&bridge_device->cdev, &bridge_fops);
	bridge_device->cdev.owner = THIS_MODULE;
	bridge_device->cdev.ops = &bridge_fops;

	status = cdev_add(&bridge_device->cdev, dev, 1);

	if (status) {
		printk("Failed to add the bridge device \n");
		return status;
	}

	/* udev support */
#ifdef OMAP_3430
	bridge_class = class_create(THIS_MODULE, "ti_bridge");
#else
	bridge_class = class_simple_create(THIS_MODULE, "ti_bridge");
#endif
	if (IS_ERR(bridge_class))
		printk(KERN_ERR "Error creating bridge class \n");

#ifdef OMAP_3430
	class_device_create(bridge_class, NULL, MKDEV(driver_major,
			   driver_minor), NULL, "DspBridge");
#else
	class_simple_device_add(bridge_class, MKDEV(driver_major, driver_minor),
				NULL, "DspBridge", driver_minor);
#endif
       /*printk("Registered driver major = %d , minor = %d \n", driver_major,
					driver_minor);*/
#else
	status = register_chrdev(driver_major, driver_name, &bridge_fops);
	if (status >= 0) {
		driver_major = status;
		status = 0;
	} else {
		printk(KERN_ERR "Failed to register Bridge driver "
			"status = 0x%x \n", status);
		return status;
	}
#endif

#ifdef LTT_SOC
	GSC = SOC_Create("GSC\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\n");
	GSI = SOC_Create("GSI\t0x%x\t0x%x\t0x%x\n");
	GSR = SOC_Create("GSR\t0x%x\t0x%x\t0x%x\t0x%x\n");
	GNG = SOC_Create("GNG\t0x%x\t0x%x\t0x%x\t0x%x\n");
	GNP = SOC_Create("GNP\t0x%x\t0x%x\t0x%x\t0x%x\n");
	GMW = SOC_Create("GMW\t0x%x\n");
	GMR = SOC_Create("GMR\t0x%x\n");
	GNC = SOC_Create("GNC\t0x%x\t0x%x\n");
#endif


	GT_init();
	GT_create(&driverTrace, "LD");

#ifdef DEBUG
	if (GT_str)
		GT_set(GT_str);

#else
#if (defined DDSP_DEBUG_PRODUCT) && GT_TRACE
	GT_set("**=67");
#endif
#endif


	GT_0trace(driverTrace, GT_ENTER, "-> driver_init\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	status = bridge_driver_register();
	if (!status)
		status = bridge_device_register();

#else
	status = platform_driver_register(&bridge_driver_ldm);
	if (!status)
		status = platform_device_register(&omap_dspbridge_dev);

#endif

#ifndef DISABLE_BRIDGE_PM
	/* Initialize the wait queue */
	if (!status) {
		bridge_suspend_data.suspended = 0;
		init_waitqueue_head(&bridge_suspend_data.suspend_wq);
	}
#endif

	OSAL_Init();

	/*  Autostart flag.  This should be set to TRUE if the DSP image should
	 *  be loaded and run during bridge module initialization  */

	if (base_img) {
		temp = TRUE;
		REG_SetValue(NULL, NULL, AUTOSTART, REG_DWORD, (BYTE *)&temp,
			    sizeof(temp));
		REG_SetValue(NULL, NULL, DEFEXEC, REG_SZ, base_img,
			    CSL_Strlen(base_img) + 1);
	} else {
		temp = FALSE;
		REG_SetValue(NULL, NULL, AUTOSTART, REG_DWORD, (BYTE *)&temp,
			    sizeof(temp));
		REG_SetValue(NULL, NULL, DEFEXEC, REG_SZ, "\0", 2);
	}
	REG_SetValue(NULL, NULL, NUMPROCS, REG_SZ, num_procs,
		    CSL_Strlen(num_procs) + 1);

	if (shm_size >= 0x10000) {	/* 64 KB */
		initStatus = REG_SetValue(NULL, NULL, SHMSIZE, REG_DWORD,
					  (BYTE *)&shm_size, sizeof(shm_size));
	} else {
		initStatus = DSP_EINVALIDARG;
		status = -1;
		GT_0trace(driverTrace, GT_7CLASS,
			 "SHM size must be atleast 64 KB\n");
	}
	GT_1trace(driverTrace, GT_7CLASS,
		 "requested shm_size = 0x%x\n", shm_size);

	if (phys_mempool_base > 0x0) {
		initStatus = REG_SetValue(NULL, NULL, PHYSMEMPOOLBASE,
					 REG_DWORD, (BYTE *)&phys_mempool_base,
					 sizeof(phys_mempool_base));
	}
	GT_1trace(driverTrace, GT_7CLASS, "phys_mempool_base = 0x%x \n",
		 phys_mempool_base);

	if (phys_mempool_size > 0x0) {
		initStatus = REG_SetValue(NULL, NULL, PHYSMEMPOOLSIZE,
					 REG_DWORD, (BYTE *)&phys_mempool_size,
					 sizeof(phys_mempool_size));
	}
	GT_1trace(driverTrace, GT_7CLASS, "phys_mempool_size = 0x%x \n",
		 phys_mempool_base);
	if ((phys_mempool_base > 0x0) && (phys_mempool_size > 0x0))
		MEM_ExtPhysPoolInit(phys_mempool_base, phys_mempool_size);
	if (tc_wordswapon) {
		GT_0trace(driverTrace, GT_7CLASS, "TC Word Swap is enabled\n");
		REG_SetValue(NULL, NULL, TCWORDSWAP, REG_DWORD,
			    (BYTE *)&tc_wordswapon, sizeof(tc_wordswapon));
	} else {
		GT_0trace(driverTrace, GT_7CLASS, "TC Word Swap is disabled\n");
		REG_SetValue(NULL, NULL, TCWORDSWAP,
			    REG_DWORD, (BYTE *)&tc_wordswapon,
			    sizeof(tc_wordswapon));
	}
	if (DSP_SUCCEEDED(initStatus)) {
		driverContext = DSP_Init(&initStatus);
		if (DSP_FAILED(initStatus)) {
			status = -1;
			GT_0trace(driverTrace, GT_7CLASS,
				 "DSP/BIOS Bridge initialization Failed\n");
		}
 #ifndef DISABLE_BRIDGE_PM
 #ifndef DISABLE_BRIDGE_DVFS
		/* Register for the constraints */
		dsp_constraint_handle = constraint_get("dspbridge",
						      &cnstr_id_vdd1);
		constraint_register_post_notification(dsp_constraint_handle,
						 &omap34xxbridge_post_scale,
						 CO_VDD1_OPP5 + 1);
#endif
#endif
	}
	if (status != 0)
		bridge_exit();

	DBC_Assert(status == 0);
	DBC_Assert(DSP_SUCCEEDED(initStatus));
	GT_0trace(driverTrace, GT_ENTER, " <- driver_init\n");
	return status;
}

/*
 * Purpose:
 *  This function is invoked during unlinking of the bridge module from the
 *  kernel.  *  Bridge resources are freed in this function.
 */
static void bridge_exit()
{
	dev_t   devno;
	BOOL ret;
	GT_0trace(driverTrace, GT_ENTER, "-> driver_exit\n");
#ifdef LTT_SOC
	SOC_Delete(GSC);
	SOC_Delete(GSI);
	SOC_Delete(GSR);
	SOC_Delete(GMW);
	SOC_Delete(GNG);
	SOC_Delete(GNP);
	SOC_Delete(GMR);
	SOC_Delete(GNC);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	GT_0trace(driverTrace, GT_1CLASS,
		  "Unregister the driver and device from the DPM \n");
	bridge_device_unregister();
	bridge_driver_unregister();
#else
#ifndef DISABLE_BRIDGE_PM
#ifndef DISABLE_BRIDGE_DVFS
	/* remove the constraints */
	if (dsp_constraint_handle != NULL) {
		GT_0trace(driverTrace, GT_7CLASS,
			 "bridge_exit: remove constraints\n");
		constraint_remove(dsp_constraint_handle);
		constraint_unregister_post_notification(dsp_constraint_handle,
						&omap34xxbridge_post_scale,
						CO_VDD1_OPP5 + 1);
		constraint_put(dsp_constraint_handle);
		dsp_constraint_handle = NULL;
	} else {
		GT_0trace(driverTrace, GT_7CLASS,
			 "dsp_constraint_handle is NULL\n");

	}
#endif /*#ifndef DISABLE_BRIDGE_DVFS*/
#endif /*#ifndef DISABLE_BRIDGE_PM*/
	/* unregister bridge driver */
	platform_device_unregister(&omap_dspbridge_dev);
	platform_driver_unregister(&bridge_driver_ldm);

#endif
	if (driverContext) {
		ret = DSP_Deinit(driverContext);
		driverContext = 0;

		DBC_Assert(ret == TRUE);
	}
	OSAL_Exit();
	GT_exit();
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0)
	unregister_chrdev(driver_major, driver_name);
#else
	devno = MKDEV(driver_major, driver_minor);
	if (bridge_device) {
		cdev_del(&bridge_device->cdev);
		kfree(bridge_device);
	}
	unregister_chrdev_region(devno, 1);
	if (bridge_class) {
		/* remove the device from sysfs */
#ifdef OMAP_3430
		class_device_destroy(bridge_class, MKDEV(driver_major,
				    driver_minor));
		class_destroy(bridge_class);
#else
		class_simple_device_remove(MKDEV(driver_major, driver_minor));
		class_simple_destroy(bridge_class);
#endif
	}
#endif
}

/*
 * Purpose: This function is called when an application opens handle to the
 * bridge *  driver.  */

int bridge_open(struct inode *ip, struct file *filp)
{
	int status = 0;
	GT_0trace(driverTrace, GT_ENTER, "-> driver_open\n");
#ifndef RES_CLEANUP_DISABLE
	HANDLE	     hProcess;
	DSP_STATUS dsp_status = DSP_SOK;
	HANDLE	     hDrvObject = NULL;
	struct PROCESS_CONTEXT    *pPctxt = NULL;
	struct PROCESS_CONTEXT	*pTmp = NULL;
	struct PROCESS_CONTEXT    *pCtxtclosed = NULL;
	struct PROCESS_CONTEXT    *pCtxttraverse = NULL;
	struct task_struct *tsk = NULL;
	dsp_status = CFG_GetObject((DWORD *)&hDrvObject, REG_DRV_OBJECT);

	/*Checking weather task structure for all process existing
	 * in the process context list If not removing those processes*/
	if (!DSP_SUCCEEDED(dsp_status))
		goto func_cont;

	DRV_GetProcCtxtList(&pCtxtclosed, hDrvObject);
	while (pCtxtclosed != NULL) {
		tsk = find_task_by_pid(pCtxtclosed->pid);
		/*if ((tsk == NULL) || (tsk->exit_state == 16))*/
		if ((tsk == NULL) || (tsk->exit_state == EXIT_ZOMBIE)) {
			/* || (tsk->exit_state == EXIT_DEAD)*/
			/*printk("***Task structure not existing for */
			/* process***%d\n", pCtxtclosed->pid);*/
			GT_1trace(driverTrace, GT_5CLASS,
				 "***Task structure not existing for "
				 "process***%d\n", pCtxtclosed->pid);
			DRV_RemoveAllResources(pCtxtclosed);
			if (pCtxtclosed->hProcessor != NULL) {
				DRV_GetProcCtxtList(&pCtxttraverse,
						    hDrvObject);
				if (pCtxttraverse->next == NULL) {
					PROC_Detach(pCtxtclosed->hProcessor);
				} else {
					if ((pCtxtclosed->pid ==
					  pCtxttraverse->pid) &&
					  (pCtxttraverse->next != NULL)) {
						pCtxttraverse =
							pCtxttraverse->next;
					}
					while ((pCtxttraverse != NULL) &&
					     (pCtxtclosed->hProcessor
					     != pCtxttraverse->hProcessor)) {
						pCtxttraverse =
							pCtxttraverse->next;
						if ((pCtxttraverse != NULL) &&
						  (pCtxtclosed->pid ==
						  pCtxttraverse->pid)) {
							pCtxttraverse =
							   pCtxttraverse->next;
						}
					}
					if (pCtxttraverse == NULL) {
						PROC_Detach
						     (pCtxtclosed->hProcessor);
					}
				}
			}
			pTmp = pCtxtclosed->next;
			DRV_RemoveProcContext(hDrvObject, pCtxtclosed,
					      pCtxtclosed->pid);
		} else {
			pTmp = pCtxtclosed->next;
		}
		pCtxtclosed = pTmp;
	}
func_cont:
	dsp_status = CFG_GetObject((DWORD *)&hDrvObject, REG_DRV_OBJECT);
	if (DSP_SUCCEEDED(dsp_status))
		dsp_status = DRV_InsertProcContext(hDrvObject, &pPctxt);

	if (pPctxt != NULL) {
		PRCS_GetCurrentHandle(&hProcess);
		DRV_ProcUpdatestate(pPctxt, PROC_RES_ALLOCATED);
		DRV_ProcSetPID(pPctxt, hProcess);
	}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
	MOD_INC_USE_COUNT;
#endif
	GT_0trace(driverTrace, GT_ENTER, " <- driver_open\n");
	return status;
}

/*
 * Purpose:
 *  This function is called when an application closes handle to the bridge
 *  driver.
 */
int bridge_release(struct inode *ip, struct file *filp)
{
	int status;
	HANDLE pid;

	GT_0trace(driverTrace, GT_ENTER, "-> driver_release\n");

	status = PRCS_GetCurrentHandle(&pid);

	if (DSP_SUCCEEDED(status))
		status = DSP_Close((DWORD) pid);


	(status == TRUE) ? (status = 0) : (status = -1);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
	MOD_DEC_USE_COUNT;
#endif

	GT_0trace(driverTrace, GT_ENTER, " <- driver_release\n");

	return status;
}

static void bridge_free(struct device *dev)
{
	/* nothing to Free */
}


/*
 * Purpose:
 *  This function provides IO interface to the bridge driver.
 */
int bridge_ioctl(struct inode *ip, struct file *filp, unsigned int code,
		unsigned long args)
{
	int status;
	DWORD retval = DSP_SOK;
	Trapped_Args pBufIn;

	DBC_Require(filp != 0);
#ifndef DISABLE_BRIDGE_PM
	status = omap24xxbridge_suspend_lockout(&bridge_suspend_data, filp);
	if (status != 0)
		return(status);

#endif

	GT_0trace(driverTrace, GT_ENTER, " -> driver_ioctl\n");

	/* Deduct one for the CMD_BASE. */
	code = (code - 1);

	status = copy_from_user(&pBufIn,
				(Trapped_Args *)args, sizeof(Trapped_Args));

	if (status >= 0) {
		status = WCD_CallDevIOCtl(code, &pBufIn, &retval);

		if (DSP_SUCCEEDED(status)) {
			status = retval;
		} else {
			GT_1trace(driverTrace, GT_7CLASS,
				 "IOCTL Failed, code : 0x%x\n", code);
			status = -1;
		}

	}

	GT_0trace(driverTrace, GT_ENTER, " <- driver_ioctl\n");

	return (status);
}

/*
 * Purpose:
 *  This function maps kernel space memory to user space memory.
 */
int bridge_mmap(struct file *filp, struct vm_area_struct *vma)
{
	ULONG offset = vma->vm_pgoff << PAGE_SHIFT;
	ULONG status;

	DBC_Assert(vma->vm_start < vma->vm_end);

	vma->vm_flags |= VM_RESERVED | VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	GT_6trace(driverTrace, GT_3CLASS,
		 "vm filp %p offset %lx start %lx end %lx"
		 " page_prot %lx flags %lx\n", filp, offset, vma->vm_start,
		 vma->vm_end, vma->vm_page_prot, vma->vm_flags);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 5, 0)
	status = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				vma->vm_end - vma->vm_start, vma->vm_page_prot);
#else
	status = remap_page_range(vma->vm_start, offset,
				vma->vm_end - vma->vm_start, vma->vm_page_prot);
#endif
	if (status != 0)
		status = -EAGAIN;

	return status;
}

#ifndef RES_CLEANUP_DISABLE
/*To remove all process resources before removing the process from the
 * process context list*/
DSP_STATUS DRV_RemoveAllResources(HANDLE hPCtxt)
{
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	if (pCtxt != NULL) {
		/*DRV_RemoveAllDSPHEAPResElements(pCtxt);*/
		DRV_RemoveAllSTRMResElements(pCtxt);
		DRV_RemoveAllNodeResElements(pCtxt);
		DRV_RemoveAllDMMResElements(pCtxt);
		DRV_ProcUpdatestate(pCtxt, PROC_RES_FREED);
	}
}
#endif

#ifndef DISABLE_BRIDGE_PM

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
static int bridge_suspend(struct device *dev, u32 state, u32 level)
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
static int bridge_suspend(struct device *dev, u32 state)
#else
bridge_suspend(struct platform_device *pdev, pm_message_t state)
#endif
#endif
{
	DWORD status = DSP_EFAIL;
	DWORD command = PWR_EMERGENCYDEEPSLEEP;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
	switch (level) {
	case SUSPEND_POWER_DOWN:
		status = PWR_SleepDSP(command, timeOut);
		break;
	}
#else
	status = PWR_SleepDSP(command, timeOut);
#endif
	if (DSP_SUCCEEDED(status)) {
		bridge_suspend_data.suspended = 1;
		return 0;
	} else {
		return -1;
	}
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
static int bridge_resume(struct device *dev, u32 level)
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
static int bridge_resume(struct device *dev)
#else
bridge_resume(struct platform_device *pdev)
#endif
#endif
{
	DWORD status = DSP_EFAIL;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
	switch (level) {
	case RESUME_POWER_ON:
		status = PWR_WakeDSP(timeOut);
		break;
	}
#else
	status = PWR_WakeDSP(timeOut);
#endif
	if (DSP_SUCCEEDED(status)) {
		bridge_suspend_data.suspended = 0;
		wake_up(&bridge_suspend_data.suspend_wq);

		return 0;
	} else {
		return -1;
	}
}

/* Test case hooks */
int test_bridge_suspend(void)
{
	DWORD status;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
	status = bridge_suspend(NULL, 0, SUSPEND_POWER_DOWN);
#else
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	status = bridge_suspend(NULL, 0);
#else
	pm_message_t state;
	state.event = 0;
	status = bridge_suspend(NULL, state);
#endif
#endif
	if (status == 0)
		return DSP_SOK;
	else
		return DSP_EFAIL;

}

int test_bridge_resume(void)
{
	DWORD status;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
	status = bridge_resume(NULL, RESUME_POWER_ON);
#else
	status = bridge_resume(NULL);
#endif
	if (status == 0)
		return DSP_SOK;
	else
		return DSP_EFAIL;

}
#endif
/*
 * Purpose:
 *  Bridge driver initialization and de-initialization functions
 */
module_init(bridge_init);
module_exit(bridge_exit);

