/*
 * drivers/video/backlight/omap3evm_bl.c
 *
 * Backlight driver for OMAP3EVM
 *
 * Copyright (c) 2009, Texas Instruments Incorporated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <mach/omapfb.h>

#if defined(CONFIG_TWL4030_CORE)
#include <linux/i2c/twl4030.h>
#endif


/**
 * Name of the driver
 */
#define OMAPBL_DRVNAME	"omap-backlight"

/**
 * Name of the device
 */
#define OMAPBL_DEVNAME	"omap-backlight"

/**
 * Minimum intensity supported by the panel
 */
#define OMAPBL_MIN_INTENSITY	0
/**
 * Maximum intensity supported by the panel
 */
#define OMAPBL_MAX_INTENSITY	100

/**
 * Default intensity after boot-up
 */
#define OMAPBL_DEF_INTENSITY	70

/**
 * Flag indicating the driver status - suspended / running
 */
#define OMAPBL_SUSPENDED	0x01

/**
 * Flag indicating low battery
 */
#define OMAPBL_BATTLOW		0x02

#define TWL_PWMA_PWMAON		0x00
#define TWL_PWMA_PWMAOFF	0x01

/**
 * Current backlight intensity
 */
static int panel_intensity;

/**
 * Backlight properties
 */
static struct backlight_properties omapbl_props;

/**
 * Generic backlight information
 */
static struct generic_bl_info *omapbl_info;

/**
 * Backlight device
 */
struct backlight_device *omapbl_device;

/**
 * Backlight flags
 */
static unsigned long omapbl_flags;

static int omapbl_set_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;
	u8 c;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (omapbl_flags & OMAPBL_SUSPENDED)
		intensity = 0;
	if (omapbl_flags & OMAPBL_BATTLOW)
		intensity &= omapbl_info->limit_mask;

	c = ((125 * (100 - intensity)) / 100) + 2;

#if defined(CONFIG_TWL4030_CORE)
	twl4030_i2c_write_u8(TWL4030_MODULE_LED, 0x11, TWL4030_LED_EN);
	twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, c, TWL_PWMA_PWMAOFF);
#endif

	panel_intensity = intensity;

	return 0;
}

static int omapbl_get_intensity(struct backlight_device *bd)
{
	return panel_intensity;
}

/**
 * omapbl_limit_intensity - Limit the backlight iuntensity
 * @limit - Value 0 clears the limit. Else used as limit to be set.
 *
 * When the battery is low, this function is called to limit the backlight.
 */
void omapbl_limit_intensity(int limit)
{
	if (limit)
		omapbl_flags |= OMAPBL_BATTLOW;
	else
		omapbl_flags &= ~OMAPBL_BATTLOW;

	backlight_update_status(omapbl_device);
}
EXPORT_SYMBOL(omapbl_limit_intensity);

static struct backlight_ops omapbl_ops = {
	.get_brightness = omapbl_get_intensity,
	.update_status  = omapbl_set_intensity,
};

static int omapbl_probe(struct platform_device *pdev)
{
	omapbl_device = backlight_device_register (OMAPBL_DRVNAME,
							&pdev->dev,
							NULL,
							&omapbl_ops);

	if (IS_ERR (omapbl_device))
		return PTR_ERR (omapbl_device);

	platform_set_drvdata(pdev, omapbl_device);

	omapbl_device->props.power		= FB_BLANK_UNBLANK;
	omapbl_device->props.max_brightness	= OMAPBL_MAX_INTENSITY;
	omapbl_device->props.brightness		= OMAPBL_DEF_INTENSITY;

	omapbl_set_intensity(omapbl_device);

	printk(KERN_INFO "omap-backlight: driver initialized.\n");

	return 0;
}

static int omapbl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	omapbl_props.power = 0;
	omapbl_props.brightness = 0;
	backlight_update_status(bd);

	backlight_device_unregister(bd);

	printk(KERN_INFO "omap-backlight: driver unloaded.\n");

	return 0;
}

#ifdef CONFIG_PM
static int omapbl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	printk(KERN_INFO "omap-backlight: suspending...\n");

	omapbl_flags |= OMAPBL_SUSPENDED;
	backlight_update_status(bd);

	return 0;
}

static int omapbl_resume(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	printk(KERN_INFO "omap-backlight: resuming...\n");

	omapbl_flags &= ~OMAPBL_SUSPENDED;
	backlight_update_status(bd);

	return 0;
}
#else
#define omapbl_suspend	NULL
#define omapbl_resume	NULL
#endif

static struct platform_driver omap_backlight_drv = {
	.probe		= omapbl_probe,
	.remove		= omapbl_remove,
	.suspend	= omapbl_suspend,
	.resume		= omapbl_resume,
	.driver		= {
				.name	= OMAPBL_DRVNAME,
			},
};

static struct platform_device *omap_backlight_dev;

static int __init omapbl_init(void)
{
	int ret = platform_driver_register(&omap_backlight_drv);

	if (!ret) {
		omap_backlight_dev = platform_device_alloc(OMAPBL_DEVNAME, -1);

		if (!omap_backlight_dev)
			return -ENOMEM;

		ret = platform_device_add(omap_backlight_dev);

		if (ret) {
			platform_device_put(omap_backlight_dev);
			platform_driver_unregister(&omap_backlight_drv);
		}
	}

	return ret;
}

static void __exit omapbl_exit(void)
{
	platform_device_unregister(omap_backlight_dev);
	platform_driver_unregister(&omap_backlight_drv);
}

module_init(omapbl_init);
module_exit(omapbl_exit);

MODULE_AUTHOR("Sanjeev Premi <premi@ti.com>");
MODULE_DESCRIPTION("OMAP LCD Backlight driver");
MODULE_LICENSE("GPLv2");

