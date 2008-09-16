/*
 * drivers/video/omap/omap_disp_out.c
 *
 * Driver for LCD and TV output on OMAP SDPs
 *	- Tested on OMAP2420 H4
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * History:
 * 30-Mar-2006	Khasim	Added LCD data lines 18/16 support
 * 15-Apr-2006	Khasim	Modified proc fs to sysfs
 * 20-Apr-2006	Khasim	Modified PM/DPM support for Mobi-Linux
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <linux/delay.h>
#ifdef CONFIG_TRACK_RESOURCES
#include <linux/device.h>
#endif
#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <asm/arch/omap-dss.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include <linux/clk.h>
#if defined(CONFIG_MACH_OMAP_2430SDP) ||  defined(CONFIG_MACH_OMAP3EVM) || defined(CONFIG_MACH_OMAP_3430LABRADOR)
#include <linux/i2c/twl4030.h>
#endif
#include <linux/workqueue.h>
//#include <asm/arch/power_companion.h>
#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif
#include <linux/platform_device.h>
#include "omap_fb.h"

#define DRIVER			"omap_disp_out"
#define	DRIVER_DESC		"OMAP Display output driver"

#define OMAP_LCD_DRIVER	"omap_lcd"
#define OMAP_TV_DRIVER		"omap_tv"

#ifdef CONFIG_MACH_OMAP_H4
#define OMAP_LCD_DEVICE		"h4_lcd"
#define OMAP_TV_DEVICE		"h4_tv"
#endif

#ifdef CONFIG_MACH_OMAP_2430SDP
#define OMAP_LCD_DEVICE		"sdp2430_lcd"
#define OMAP_TV_DEVICE		"sdp2430_tv"
#endif

#if defined(CONFIG_MACH_OMAP_3430SDP) ||  defined(CONFIG_MACH_OMAP3EVM) || defined(CONFIG_MACH_OMAP_3430LABRADOR)
#if 0
#define OMAP_LCD_DEVICE		"sdp3430_lcd"
#define OMAP_TV_DEVICE		"sdp3430_tv"
#else
#define OMAP_LCD_DEVICE		"omap_lcd"
#define OMAP_TV_DEVICE		"omap_tv"
#endif
#endif

#if defined(CONFIG_OMAP3EVM_LCD_VGA) || defined(CONFIG_FB_OMAP_LCD_VGA)
#define H4_LCD_XRES	 	480
#define H4_LCD_YRES 		640
#define H4_LCD_PIXCLOCK_MAX	41700 /* in pico seconds  */
#define H4_LCD_PIXCLOCK_MIN	38000  /* in pico seconds */
#else
#ifdef CONFIG_OMAP3430_ES2
#define H4_LCD_XRES	 	240
#define H4_LCD_YRES 		320
#define H4_LCD_PIXCLOCK_MAX	167000 /* in pico seconds  */
#define H4_LCD_PIXCLOCK_MIN	152000  /* in pico seconds */
#else
#define H4_LCD_XRES	 	240
#define H4_LCD_YRES 		320
#define H4_LCD_PIXCLOCK_MAX	185186 /* freq 5.4 MHz */
#define H4_LCD_PIXCLOCK_MIN	138888 /* freq 7.2 MHz */
#endif
#endif

#define H4_TV_XRES		/*640*/720
#define H4_TV_YRES		482

#ifdef CONFIG_MACH_OMAP_2430SDP
#define LCD_PANEL_ENABLE_GPIO 		154
#define LCD_PANEL_BACKLIGHT_GPIO 	91
#endif

#if defined(CONFIG_ARCH_OMAP3430)
#if defined(CONFIG_MACH_OMAP_3430LABRADOR)
#define LCD_PANEL_ENABLE_GPIO 		15
#define LCD_PANEL_RESET_GPIO		55
#define LCD_PANEL_QVGA_GPIO		56
#define LCD_PANEL_BACKLIGHT_GPIO 	7
#elif defined(CONFIG_ARCH_OMAP3430SDP) && defined(CONFIG_OMAP3430_ES2)
#define LCD_PANEL_ENABLE_GPIO 		5
#define LCD_PANEL_BACKLIGHT_GPIO 	8
#elif defined(CONFIG_MACH_OMAP3EVM)
#define LCD_PANEL_ENABLE_GPIO       153
#define LCD_PANEL_LR                2
#define LCD_PANEL_UD                3
#define LCD_PANEL_INI               152
#define LCD_PANEL_QVGA              154
#define LCD_PANEL_RESB              155
#else
#define LCD_PANEL_ENABLE_GPIO 		28
#define LCD_PANEL_BACKLIGHT_GPIO 	24
#endif
#endif

#ifdef CONFIG_MACH_OMAP3EVM
#define LCD_PANEL_ENABLE_GPIO       153
#define LCD_PANEL_LR                2
#define LCD_PANEL_UD                3
#define LCD_PANEL_INI               152
#define LCD_PANEL_QVGA              154
#define LCD_PANEL_RESB              155
#endif

#define CONFIG_OMAP_LCD
#define ENABLE_VDAC_DEDICATED		0x03
#define ENABLE_VDAC_DEV_GRP             0x20
#define ENABLE_VPLL2_DEDICATED		0x05
#define ENABLE_VPLL2_DEV_GRP            0xE0

#ifdef CONFIG_VIDEO_OMAP_TVOUT
#define CONFIG_OMAP_TV
#define MENELAUS_I2C_ADAP_ID		0
#endif

#ifdef CONFIG_MACH_OMAP3EVM
extern void omap_disp_replication_enable (void);
#endif

#define CONFIG_TWL4030_CORE_T2
#define CONFIG_I2C_TWL4030_CORE

extern int omap_get_dss1_clock(void);
extern ssize_t
fb_out_show(struct device *cdev, struct device_attribute *attr,
		char *buf);

extern ssize_t
fb_out_store(struct device *cdev, struct device_attribute *attr,
		const char *buffer, size_t count);

extern int fb_out_layer;
extern int omapfb_set_output_layer(int layer);

extern int twl4030_request_gpio(int gpio);
extern int twl4030_free_gpio(int gpio);
extern int twl4030_set_gpio_dataout(int gpio, int enable);
extern int twl4030_set_gpio_direction(int gpio, int is_input);

#ifdef CONFIG_ARCH_OMAP34XX
extern int lpr_enabled;
#endif

/*---------------------------------------------------------------------------*/

#ifdef CONFIG_OMAP_LCD

static int lcd_in_use;
static int lcd_backlight_state=1;

static int previous_lcd_backlight_state=1;

#ifdef CONFIG_MACH_OMAP_H4
static int h4_read_gpio_expa(u8 *val);
static int h4_write_gpio_expa(u8 val);
#endif

static void lcd_backlight_off(struct work_struct *work);
static void lcd_backlight_on(struct work_struct *work);

#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT

DECLARE_WORK(lcd_bklight_on, lcd_backlight_on);
DECLARE_WORK(lcd_bklight_off, lcd_backlight_off);

static void lcd_panel_enable(struct work_struct *work);
static void lcd_panel_disable(struct work_struct *work);

DECLARE_WORK(lcd_panel_on, lcd_panel_enable);
DECLARE_WORK(lcd_panel_off, lcd_panel_disable);

static void
power_lcd_backlight(int level)
{
	switch (level) {
		case LCD_OFF:
			if(!in_interrupt())
				lcd_backlight_off(NULL);
			else
				schedule_work(&lcd_bklight_off);
			break;
		default:
			if(!in_interrupt())
				lcd_backlight_on(NULL);
			else
				schedule_work(&lcd_bklight_on);
			break;
	}
}

static void
power_lcd_panel(int level)
{
	switch (level) {
		case LCD_OFF:
			if(!in_interrupt())
				lcd_panel_disable(NULL);
			else
				schedule_work(&lcd_panel_off);
			break;
		default:
			if(!in_interrupt())
				lcd_panel_enable(NULL);
			else
				schedule_work(&lcd_panel_on);
			break;
	}
}

#ifdef CONFIG_MACH_OMAP_H4

/* Write to GPIO EXPA on the board.
 * The GPIO expanders need an independent I2C client driver.
 */
static int
h4_write_gpio_expa(u8 val)
{
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[1];

	adap = i2c_get_adapter(0);
	if (!adap)
		return -ENODEV;
	msg->addr = 0x20;	/* I2C address of GPIO EXPA */
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;
	data[0] = val;
	err = i2c_transfer(adap, msg, 1);
	if (err >= 0)
		return 0;
	return err;
}

/* Read from GPIO EXPA on the board.
 * The GPIO expanders need an independent I2C client driver.
 */
static int
h4_read_gpio_expa(u8 *val)
{
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[1];

	adap = i2c_get_adapter(0);
	if (!adap)
		return -ENODEV;
	msg->addr = 0x20;	/* I2C address of GPIO EXPA */
	msg->flags = I2C_M_RD;
	msg->len = 1;
	msg->buf = data;
	err = i2c_transfer(adap, msg, 1);
	*val = data[0];
	if (err >= 0)
		return 0;
	return err;
}
#endif

static void
lcd_panel_enable(struct work_struct *work)
{
#ifdef CONFIG_MACH_OMAP_H4
	unsigned char expa;
	int err;

	/* read current state of GPIO EXPA outputs */
	if ((err = h4_read_gpio_expa(&expa))) {
		printk(KERN_ERR DRIVER
				"Error reading GPIO EXPA\n");
		return;
	}
	/* Set GPIO EXPA P7 (LCD_ENVDD) to power-up LCD and
	 * set GPIO EXPA P5 (LCD_ENBKL) to turn on backlight
	 */
	if ((err = h4_write_gpio_expa(expa | 0x80))) {
		printk(KERN_ERR DRIVER
				"Error writing to GPIO EXPA\n");
		return;
	}
#endif
#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
	omap_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 1);
	/* power to the RGB lines from T2 is issued separately in
	 * omap_dss-rgb_enable */
#elif defined(CONFIG_MACH_OMAP_3430LABRADOR)
	twl4030_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 1);
#endif

#ifdef CONFIG_MACH_OMAP3EVM
	omap_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 0);
#endif
}

void
omap_dss_rgb_enable(void)
{
#ifdef CONFIG_MACH_OMAP_2430SDP
	twl4030_vaux2_ldo_use();
#else
	if(is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
//		if( 0 != twl4030_vaux3_ldo_use())
			printk(KERN_WARNING "omap_disp: twl4030_vaux3_ldo_use returns error \n");
	}
	else {

		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				ENABLE_VPLL2_DEDICATED,TWL4030_VPLL2_DEDICATED);

		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				ENABLE_VPLL2_DEV_GRP,TWL4030_VPLL2_DEV_GRP);

		mdelay(4);
	}
#endif
}
EXPORT_SYMBOL(omap_dss_rgb_enable);

static void
lcd_panel_disable(struct work_struct *work)
{
#ifdef CONFIG_MACH_OMAP_H4
	unsigned char expa;
	int err;

	/* read current state of GPIO EXPA outputs */
	if ((err = h4_read_gpio_expa(&expa))) {
		printk(KERN_ERR DRIVER
				"Error reading GPIO EXPA\n");
		return;
	}

	/* Clear GPIO EXPA P7 (LCD_ENVDD) to power-uoff LCD and
	 * clear GPIO EXPA P5 (LCD_ENBKL) to turn off backlight
	 */
	if ((err = h4_write_gpio_expa(expa & ~0x80))) {
		printk(KERN_ERR DRIVER
				"Error writing to GPIO EXPA\n");
		return;
	}
#endif
#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
	omap_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 0);
	/* power to the RGB lines is disabled in omap_dss_rgb_disable */
#elif defined(CONFIG_MACH_OMAP_3430LABRADOR)
	twl4030_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 0);
#endif
#ifdef CONFIG_MACH_OMAP3EVM
	omap_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 1);
#endif

}

void
omap_dss_rgb_disable(void)
{
#ifdef CONFIG_MACH_OMAP_2430SDP
	twl4030_vaux2_ldo_unuse();
#else
	if(is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
//		if( 0 != twl4030_vaux3_ldo_unuse())
			printk(KERN_WARNING "omap_disp: twl4030_vaux3_ldo_unuse returns error \n");
	}
	else {
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				0x0,TWL4030_VPLL2_DEDICATED);

		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				0x0,TWL4030_VPLL2_DEV_GRP);
		mdelay(4);
	}
#endif

}
EXPORT_SYMBOL(omap_dss_rgb_disable);

static void
lcd_backlight_on(struct work_struct *work)
{
#ifdef CONFIG_MACH_OMAP_H4
	unsigned char expa;
	int err;

	/* read current state of GPIO EXPA outputs */
	if ((err = h4_read_gpio_expa(&expa))) {
		printk(KERN_ERR DRIVER
				"Error reading GPIO EXPA\n");
		return;
	}

	/* Set GPIO EXPA P7 (LCD_ENVDD) to power-up LCD and
	 * set GPIO EXPA P5 (LCD_ENBKL) to turn on backlight
	 */
	if ((err = h4_write_gpio_expa(expa | 0x20))) {
		printk(KERN_ERR DRIVER
				"Error writing to GPIO EXPA\n");
		return;
	}
	lcd_backlight_state = LCD_ON;
#endif
#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
	omap_set_gpio_dataout(LCD_PANEL_BACKLIGHT_GPIO, 1);
	lcd_backlight_state = LCD_ON;
#elif defined(CONFIG_MACH_OMAP_3430LABRADOR)
	twl4030_set_gpio_dataout(LCD_PANEL_BACKLIGHT_GPIO, 1);
	lcd_backlight_state = LCD_ON;
#endif

#ifdef CONFIG_MACH_OMAP3EVM
	twl4030_i2c_write_u8(TWL4030_MODULE_LED, 0x32, 0);
	lcd_backlight_state = LCD_ON;
#endif
}

static void
lcd_backlight_off(struct work_struct *work)
{
#ifdef CONFIG_MACH_OMAP_H4
	unsigned char expa;
	int err;

	/* read current state of GPIO EXPA outputs */
	if ((err = h4_read_gpio_expa(&expa))) {
		printk(KERN_ERR DRIVER
				"Error reading GPIO EXPA\n");
		return;
	}

	/* Clear GPIO EXPA P7 (LCD_ENVDD) to power-uoff LCD and
	 * clear GPIO EXPA P5 (LCD_ENBKL) to turn off backlight
	 */
	if ((err = h4_write_gpio_expa(expa & ~0x20))) {
		printk(KERN_ERR DRIVER
				"Error writing to GPIO EXPA\n");
		return;
	}
	lcd_backlight_state = LCD_OFF;
#endif
#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
	omap_set_gpio_dataout(LCD_PANEL_BACKLIGHT_GPIO, 0);
	lcd_backlight_state = LCD_OFF;
#elif defined(CONFIG_MACH_OMAP_3430LABRADOR)
	twl4030_set_gpio_dataout(LCD_PANEL_BACKLIGHT_GPIO, 0);
	lcd_backlight_state = LCD_OFF;
#endif
#ifdef CONFIG_MACH_OMAP3EVM
	twl4030_i2c_write_u8(TWL4030_MODULE_LED, 0x31, 0);
	lcd_backlight_state = LCD_OFF;
#endif
}

void enable_backlight(void)
{
	/* If already enabled, return*/
	if (lcd_in_use)
		return;
	if (previous_lcd_backlight_state == LCD_ON) {
		power_lcd_backlight(LCD_ON);
#if 0
		power_lcd_panel(LCD_ON);
#endif
	}
	lcd_in_use = 1;
}
EXPORT_SYMBOL(enable_backlight);

void disable_backlight(void)
{
	/* If LCD is already disabled, return*/
	if (!lcd_in_use)
		return;
	previous_lcd_backlight_state = lcd_backlight_state;
	power_lcd_backlight(LCD_OFF);
#if 0
	power_lcd_panel(LCD_OFF);
#endif
	lcd_in_use = 0;
}
EXPORT_SYMBOL(disable_backlight);

static int gpio_reserved = 0;

#ifndef CONFIG_LCD_IOCTL
static
#endif
int omap_lcd_init(struct omap_lcd_info *info)
{
#if defined(CONFIG_OMAP3EVM_LCD_VGA) || defined(CONFIG_FB_OMAP_LCD_VGA)
	u32	pixclock	= H4_LCD_PIXCLOCK_MAX,/* picoseconds */
		left_margin	= 79,		/* pixclocks */
		right_margin	= 89,		/* pixclocks */
		upper_margin	= 1,		/* line clocks */
		lower_margin	= 0,		/* line clocks */
		hsync_len	= 3,		/* pixclocks */
		vsync_len	= 2,		/* line clocks */
		sync		= 1,		/* hsync & vsync polarity */
		acb		= 0x28,		/* AC-bias pin frequency */
		ipc		= 1,		/* Invert pixel clock */
		onoff		= 1;		/* HSYNC/VSYNC Pixel clk Control*/
#else
#ifdef CONFIG_OMAP3430_ES2
	u32	pixclock	= H4_LCD_PIXCLOCK_MAX,/* picoseconds */
		left_margin	= 39,		/* pixclocks */
		right_margin	= 45,		/* pixclocks */
		upper_margin	= 1,		/* line clocks */
		lower_margin	= 0,		/* line clocks */
		hsync_len	= 3,		/* pixclocks */
		vsync_len	= 2,		/* line clocks */
		sync		= 1,		/* hsync & vsync polarity */
		acb		= 0x28,		/* AC-bias pin frequency */
		ipc		= 1,		/* Invert pixel clock */
		onoff		= 1;		/* HSYNC/VSYNC Pixel clk Control*/
#else
	u32	pixclock	= H4_LCD_PIXCLOCK_MAX,/* picoseconds */
		left_margin	= 40,		/* pixclocks */
		right_margin	= 4,		/* pixclocks */
		upper_margin	= 8,		/* line clocks */
		lower_margin	= 2,		/* line clocks */
		hsync_len	= 4,		/* pixclocks */
		vsync_len	= 2,		/* line clocks */
		sync		= 0,		/* hsync & vsync polarity */
		acb		= 0,		/* AC-bias pin frequency */
		ipc		= 0,		/* Invert pixel clock */
		onoff		= 0;		/* HSYNC/VSYNC Pixel clk Control*/

#endif
#endif
	u32 clkdiv;
#ifdef CONFIG_OMAP_DSI
	u32 * handle;
#endif

#ifdef CONFIG_LCD_IOCTL
	if (info) {
		pixclock     = info->pixclock,
		left_margin  = info->left_margin,
		right_margin = info->right_margin,
		upper_margin = info->upper_margin,
		lower_margin = info->lower_margin,
		hsync_len    = info->hsync_len,
		vsync_len    = info->vsync_len,
		sync         = info->sync,
		acb          = info->acb,
		ipc          = info->ipc,
		onoff        = info->onoff;
	}

	if (gpio_reserved == 1)
		goto bypass_gpio;
#endif

#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
	omap_request_gpio(LCD_PANEL_ENABLE_GPIO);  /* LCD panel */
	omap_request_gpio(LCD_PANEL_BACKLIGHT_GPIO);	 /* LCD backlight */
	omap_set_gpio_direction(LCD_PANEL_ENABLE_GPIO, 0); /* output */
	omap_set_gpio_direction(LCD_PANEL_BACKLIGHT_GPIO, 0); /* output */
#elif defined(CONFIG_MACH_OMAP_3430LABRADOR)
	omap_request_gpio(LCD_PANEL_RESET_GPIO);
	omap_request_gpio(LCD_PANEL_QVGA_GPIO);
	twl4030_request_gpio(LCD_PANEL_ENABLE_GPIO);  /* LCD panel */
	twl4030_request_gpio(LCD_PANEL_BACKLIGHT_GPIO);	 /* LCD backlight */

	omap_set_gpio_direction(LCD_PANEL_QVGA_GPIO, 0);
	omap_set_gpio_direction(LCD_PANEL_RESET_GPIO, 0);
	twl4030_set_gpio_direction(LCD_PANEL_ENABLE_GPIO, 0); /* output */
	twl4030_set_gpio_direction(LCD_PANEL_BACKLIGHT_GPIO, 0); /* output */

#ifdef CONFIG_FB_OMAP_LCD_VGA
	omap_set_gpio_dataout(LCD_PANEL_QVGA_GPIO, 0);
#else
	omap_set_gpio_dataout(LCD_PANEL_QVGA_GPIO, 1);
#endif
	omap_set_gpio_dataout(LCD_PANEL_RESET_GPIO, 1);
#endif

#if defined(CONFIG_MACH_OMAP3EVM)
        omap_request_gpio(LCD_PANEL_LR);     /* LR */
        omap_request_gpio(LCD_PANEL_UD);     /* UD */
        omap_request_gpio(LCD_PANEL_INI);    /* INI */
        omap_request_gpio(LCD_PANEL_RESB);   /* RESB */
        omap_request_gpio(LCD_PANEL_QVGA);   /* QVGA */
        omap_request_gpio(LCD_PANEL_ENABLE_GPIO); /* ENVDD */

        omap_set_gpio_direction(LCD_PANEL_LR, 0); /* output */
        omap_set_gpio_direction(LCD_PANEL_UD, 0); /* output */
        omap_set_gpio_direction(LCD_PANEL_INI, 0); /* output */
        omap_set_gpio_direction(LCD_PANEL_RESB, 0); /* output */
        omap_set_gpio_direction(LCD_PANEL_QVGA, 0);
        omap_set_gpio_direction(LCD_PANEL_ENABLE_GPIO, 0); /* output */

        twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, 0x7F, 0);
        twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, 0x7F, 1);
        twl4030_i2c_write_u8(TWL4030_MODULE_PWMB, 0x7F, 0);
        twl4030_i2c_write_u8(TWL4030_MODULE_PWMB, 0x7F, 1);
#endif

#ifdef CONFIG_LCD_IOCTL
bypass_gpio:
	gpio_reserved = 1;
#endif
	omap_dss_rgb_enable();
	omap_disp_get_dss();

#if defined(CONFIG_MACH_OMAP3EVM)
       omap_set_gpio_dataout(LCD_PANEL_RESB, 1);
       omap_set_gpio_dataout(LCD_PANEL_INI, 1);
#ifdef CONFIG_OMAP3EVM_LCD_VGA
       omap_set_gpio_dataout(LCD_PANEL_QVGA, 0);
#else
       omap_set_gpio_dataout(LCD_PANEL_QVGA, 1);
#endif
       omap_set_gpio_dataout(LCD_PANEL_LR, 1);
       omap_set_gpio_dataout(LCD_PANEL_UD, 1);
#endif

#ifndef CONFIG_OMAP_DSI
	omap_disp_set_panel_size(OMAP_OUTPUT_LCD, H4_LCD_XRES, H4_LCD_YRES);

	/* clkdiv = pixclock / (omap dss1 clock period) */
	clkdiv = pixclock / (1000000000UL/omap_get_dss1_clock());
	clkdiv /= 1000;

	omap_disp_config_lcd(clkdiv,
			      left_margin - 1,	// hbp
			      right_margin - 1,	// hfp
			      hsync_len - 1,	// hsw
			      upper_margin,	// vbp
			      lower_margin,	// vfp
			      vsync_len - 1	// vsw
			      );

	omap_disp_lcdcfg_polfreq(	sync,   // horizontal sync active low
					sync,   // vertical sync active low
					acb,    // ACB
					ipc,    // IPC
					onoff   // ONOFF
				);
#ifdef CONFIG_MACH_OMAP3EVM
        omap_disp_replication_enable();
#endif

	omap_disp_enable_output_dev(OMAP_OUTPUT_LCD);
	udelay(20);
#else

	/* Enable power to DSI, edisco and initialize the edisco */
	if( 0 != twl4030_vaux3_ldo_use())
			printk(KERN_WARNING "omap_disp: twl4030_vaux3_ldo_unuse returns error \n");

	omap_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 1);

	omap_set_gpio_dataout(LCD_PANEL_BACKLIGHT_GPIO, 1);

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					ENABLE_VPLL2_DEDICATED,TWL4030_VPLL2_DEDICATED);

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					ENABLE_VPLL2_DEV_GRP,TWL4030_VPLL2_DEV_GRP);

	printk("Setting pin mux for DSI !!!! \n");
	omap_writel(0x02010201,0x480020dc);
	omap_writel(0x02010201,0x480020e0);
	omap_writel(0x02010201,0x480020e4);
	printk("Calling to initialize edisco !!!! \n");
	edisco_init(handle);
	// edisco_write(handle, 0x12,&ebuf_len,&ebuf);
#endif

	enable_backlight();
	power_lcd_panel(LCD_ON);
#ifndef CONFIG_OMAP_DSI
	omap_disp_put_dss();
#endif

	printk(KERN_DEBUG DRIVER
	       "LCD panel %dx%d\n", H4_LCD_XRES, H4_LCD_YRES);
	return 0;
}

static int
lcd_exit(void)
{
	if (!lcd_in_use)
		return 0;

	omap_disp_get_dss();
	omap_disp_disable_output_dev(OMAP_OUTPUT_LCD);
	omap_disp_put_dss();

#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
	omap_free_gpio(LCD_PANEL_ENABLE_GPIO);  /* LCD panel */
	omap_free_gpio(LCD_PANEL_BACKLIGHT_GPIO);  /* LCD backlight */
#elif defined(CONFIG_MACH_OMAP_3430LABRADOR)
	omap_free_gpio(LCD_PANEL_RESET_GPIO);
	omap_free_gpio(LCD_PANEL_QVGA_GPIO);
	twl4030_free_gpio(LCD_PANEL_ENABLE_GPIO);  /* LCD panel */
	twl4030_free_gpio(LCD_PANEL_BACKLIGHT_GPIO);  /* LCD backlight */
#endif

	lcd_in_use = 0;

	return 0;
}
/* ------------------------------------------------------------------------------ */
/* Power and device Management */

static int __init
lcd_probe(struct platform_device *odev)
{
	return omap_lcd_init(0);
}

#ifdef CONFIG_PM
static int lcd_suspend(struct platform_device *odev, pm_message_t state);
static int lcd_resume(struct platform_device *odev);
#endif

static struct platform_driver omap_lcd_driver = {
	.driver = {
		.name   = OMAP_LCD_DRIVER,
	},
	.probe          = lcd_probe,
#ifdef CONFIG_PM
	.suspend        = lcd_suspend,
	.resume         = lcd_resume,
#endif
};

static struct platform_device lcd_device = {
        .name     = OMAP_LCD_DEVICE,
	.id    = 9,
};

#ifdef CONFIG_PM
static int
lcd_suspend(struct platform_device *odev, pm_message_t state)
{
		if (!lcd_in_use)
		return 0;
	disable_backlight();
	omap_dss_rgb_disable();

	return 0;
}

static int
lcd_resume(struct platform_device *odev)
{
	if (lcd_in_use)
		return 0;
	omap_dss_rgb_enable();
	udelay(20);
	enable_backlight();

	return 0;
}

#endif /* CONFIG_PM */
#endif	/* CONFIG_OMAP_LCD */

/*---------------------------------------------------------------------------*/

#ifdef CONFIG_OMAP_TV

static int tv_in_use;

static void h4_i2c_tvout_off(struct work_struct *work);
static void h4_i2c_tvout_on(struct work_struct *work);

DECLARE_WORK(h4_tvout_on, h4_i2c_tvout_on);
DECLARE_WORK(h4_tvout_off, h4_i2c_tvout_off);

static void
power_tv(int level)
{
	switch(level) {
		case TV_OFF:
			if(!in_interrupt())
				h4_i2c_tvout_off(NULL);
			else
				schedule_work(&h4_tvout_off);
			break;
		default:
			if(!in_interrupt())
				h4_i2c_tvout_on(NULL);
			else
				schedule_work(&h4_tvout_on);
			break;
	}
}

static void
h4_i2c_tvout_off(struct work_struct *work)
{
#if defined (CONFIG_MACH_OMAP_H4) || defined (CONFIG_TWL4030_CORE_M1)
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	/*
	 * Turn OFF TV block (AVDD and VREF) in menelaus chip
	 * MENELAUS_LDO_CTRL8 (0x11 -> 0x03)
	 */
	adap = i2c_get_adapter(MENELAUS_I2C_ADAP_ID);
	if (!adap) {
		printk(KERN_ERR DRIVER
				"Unable to get I2C adapter \n");
	}
	msg->addr = 0x72;/* I2C address of Menelaus Chip */
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = 0x11; /* LD0_CTRL8 */
	data[1] = 0x00; /* Disable bits for the 0.5V reference LDO */
	err = i2c_transfer(adap, msg, 1);
	if (err > 2) {
		printk(KERN_ERR DRIVER
				"Disabling TV block through Menelaus failed %d\n",err);
	}
#endif
#if defined (CONFIG_OMAP_TV)
	omap_disp_set_tvref(TVREF_OFF);
#endif

#if (defined(CONFIG_TWL4030_CORE_T2) && defined(CONFIG_I2C_TWL4030_CORE))  \
	|| defined(CONFIG_MACH_OMAP_3430LABRADOR) || defined(CONFIG_MACH_OMAP3EVM)
	omap_disp_set_tvref(TVREF_OFF);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,0x00,TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,0x00,TWL4030_VDAC_DEV_GRP);
#endif
}

static void
h4_i2c_tvout_on(struct work_struct *work)
{
#if defined (CONFIG_MACH_OMAP_H4) || defined (CONFIG_TWL4030_CORE_M1)
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	/*
	 * Turn ON TV block (AVDD and VREF) in menelaus chip
	 * MENELAUS_LDO_CTRL8 (0x11 -> 0x03)
	 */
	adap = i2c_get_adapter(MENELAUS_I2C_ADAP_ID);
	if (!adap) {
		printk(KERN_ERR DRIVER
				"Unable to get I2C adapter \n");
	}
	msg->addr = 0x72;/* I2C address of Menelaus Chip */
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = 0x11; /* LD0_CTRL8 */
	data[1] = 0x03;	/* Enable bits for the 0.5V reference
			 * and the VADAC LDO
			 */
	err = i2c_transfer(adap, msg, 1);
	if (err > 2) {
		printk(KERN_ERR DRIVER
				"Enabling TV block through Menelaus failed %d\n",err);
	}
#endif
#if defined (CONFIG_OMAP_TV)
	omap_disp_set_tvref(TVREF_ON);
#endif
#if (defined(CONFIG_TWL4030_CORE_T2) && defined(CONFIG_I2C_TWL4030_CORE)) \
	|| defined(CONFIG_MACH_OMAP_3430LABRADOR)
	omap_disp_set_tvref(TVREF_ON);

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEDICATED,TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEV_GRP,TWL4030_VDAC_DEV_GRP);
#endif
}

static int
tv_init(void)
{
	omap_disp_get_all_clks();
	power_tv(TV_ON);
	omap_disp_set_tvstandard(NTSC_M);
	omap_disp_set_panel_size(OMAP_OUTPUT_TV, H4_TV_XRES, H4_TV_YRES);
	omap_disp_enable_output_dev(OMAP_OUTPUT_TV);
	omap_disp_put_all_clks();
	printk(KERN_DEBUG DRIVER
			"TV %dx%d interlaced\n", H4_TV_XRES, H4_TV_YRES);
	tv_in_use = 1;
	return 0;
}

static int
tv_exit(void)
{
	if (!tv_in_use)
		return 0;

	omap_disp_get_all_clks();
	omap_disp_disable_output_dev(OMAP_OUTPUT_TV);
	power_tv(TV_OFF);
	omap_disp_put_all_clks();
	tv_in_use = 0;
	return 0;
}

static int __init tv_probe(struct platform_device *odev);
#ifdef CONFIG_PM
static int tv_suspend(struct platform_device *odev, pm_message_t state);
static int tv_resume(struct platform_device *odev);
#endif

static struct platform_driver omap_tv_driver = {
	.driver = {
		.name   = OMAP_TV_DRIVER,
	},
	.probe          = tv_probe,
#ifdef CONFIG_PM
	.suspend        = tv_suspend,
	.resume         = tv_resume,
#endif
};

static struct platform_device tv_device = {
        .name     = OMAP_TV_DEVICE,
	.id    = 10,
};

static int __init
tv_probe(struct platform_device *odev)
{
	return tv_init();
}

#ifdef CONFIG_PM
static int
tv_suspend(struct platform_device *odev, pm_message_t state)
{
	if (!tv_in_use)
		return 0;

	/* TODO-- need to delink DSS and TV clocks.. For now, TV is put to
	 * off in fb_blank and put_dss */

	tv_in_use = 0;

	return 0;
}

static int
tv_resume(struct platform_device *odev)
{
	if (tv_in_use)
		return 0;

	/* TODO-- need to delink DSS and TV clocks.. For now, TV is put to
	 * on in fb_blank and get_dss */
	tv_in_use = 1;
	return 0;
}

#endif	/* CONFIG_PM */

#endif /* CONFIG_OMAP_TV */

/*---------------------------------------------------------------------------*/
/* Sysfs support */

struct board_properties {
	struct module *owner;
};
static struct board_properties *bd;

struct dispc_device {
	struct board_properties *props;
	struct device class_dev;
};
static struct dispc_device *new_bd;

static void dispc_class_release(struct device *dev)
{
	if(new_bd != NULL) kfree(new_bd);
	if(bd != NULL) kfree(bd);
}

struct class dispc_class = {
	.name = "display_control",
	.dev_release = dispc_class_release,
};

static ssize_t
read_layer_out(char *buf, int layer)
{
	int p = 0;
	int output_dev = omap_disp_get_output_dev(layer);

	switch (output_dev) {
		case OMAP_OUTPUT_LCD :
			p = sprintf(buf, "channel0\n");
			break;
		case OMAP_OUTPUT_TV :
			p = sprintf(buf, "channel1\n");
			break;
	}
	return(p);
}

static ssize_t
write_layer_out(const char *buffer, size_t count,int layer)
{
	int out_dev;
	if (!buffer || (count == 0))
		return 0;

	/* only 'lcd' or 'tv' are valid inputs */
	if (strncmp(buffer, "channel0", 8) == 0)
		out_dev = OMAP_OUTPUT_LCD;
#ifdef CONFIG_OMAP_TV
	else if (strncmp(buffer, "channel1", 8) == 0)
		out_dev = OMAP_OUTPUT_TV;
#endif
	else
		return -EINVAL;

	if(omap_disp_get_output_dev(layer) == out_dev)
		return count;

	if(out_dev == OMAP_OUTPUT_TV)
		omap_disp_get_all_clks();
	else
		omap_disp_get_dss();
	if (fb_out_layer != OMAP_GRAPHICS)
	{
		omap_disp_disable_layer(fb_out_layer);
		omap_disp_release_layer (fb_out_layer);
	}
	mdelay(1);
	omap_disp_set_output_dev(layer, out_dev);
	mdelay(1);

	if(fb_out_layer != OMAP_GRAPHICS)
		omapfb_set_output_layer(fb_out_layer);
	if(out_dev == OMAP_OUTPUT_TV)
		omap_disp_put_all_clks();
	else
		omap_disp_put_dss();
	return count;
}

static ssize_t
graphics_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return(read_layer_out(buf,OMAP_GRAPHICS));
}

static ssize_t
graphics_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	return(write_layer_out(buffer,count,OMAP_GRAPHICS));
}

static ssize_t
video1_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return(read_layer_out(buf,OMAP_VIDEO1));
}

static ssize_t
video1_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	return(write_layer_out(buffer,count,OMAP_VIDEO1));
}

static ssize_t
video2_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return(read_layer_out(buf,OMAP_VIDEO2));
}

static ssize_t
video2_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	return(write_layer_out(buffer,count,OMAP_VIDEO2));
}

static ssize_t
lcdbacklight_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int p =0;

	switch (lcd_backlight_state) {
		case LCD_ON:
			p = sprintf(buf, "on\n");
			break;
		case LCD_OFF:
			p = sprintf(buf, "off\n");
			break;
	}
	return p;
}

static ssize_t
lcdbacklight_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	/*
	 * If the LCD is already in suspend state do not accept any changes to
	 * backlight
	 */

	if (!lcd_in_use)
		return -EINVAL;

	if (!buffer || (count == 0))
		return 0;

	if (strncmp(buffer,"on",2) == 0){
		power_lcd_backlight(LCD_ON);
	}
	else if (strncmp(buffer, "off", 3) == 0){
		power_lcd_backlight(LCD_OFF);
	}
	else{
		return -EINVAL;
	}
	return count;
}

static ssize_t
lcd_data_lines_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int p =0;
	int current_lcddatalines_state;

	current_lcddatalines_state = omap_disp_get_lcddatalines();

	switch (current_lcddatalines_state) {
		case LCD_DATA_LINE_18BIT:
			p = sprintf(buf, "18 bits\n");
			break;
		case LCD_DATA_LINE_16BIT:
			p = sprintf(buf, "16 bits\n");
			break;
	}
	return (p);
}

static ssize_t
lcd_data_lines_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
    int no_of_data_lines;

	if (!buffer || (count == 0))
		return 0;

	if (strncmp(buffer,"18",2) == 0)
		no_of_data_lines = LCD_DATA_LINE_18BIT;
	else if (strncmp(buffer, "16", 2) == 0)
		no_of_data_lines = LCD_DATA_LINE_16BIT;
	else
		return -EINVAL;

	omap_disp_set_lcddatalines(no_of_data_lines);
	return count;
}

static ssize_t
dithering_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int p=0;
	int current_dither_state;

	omap_disp_get_dss();
	current_dither_state = omap_disp_get_dithering();
	switch (current_dither_state) {
		case DITHERING_ON:
			p = sprintf(buf, "on\n");
			break;
		case DITHERING_OFF:
			p = sprintf(buf, "off\n");
			break;
	}
	omap_disp_put_dss();
	return p;
}

static ssize_t
dithering_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	int dither_state;

	if (!buffer || (count == 0)){
		return 0;
	}
	if (strncmp(buffer,"on",2) == 0){
		dither_state = DITHERING_ON;
	}
	else if (strncmp(buffer, "off", 3) == 0){
		dither_state = DITHERING_OFF;
	}
	else
		return -EINVAL;
        omap_disp_get_dss();
	omap_disp_set_dithering(dither_state);
        omap_disp_put_dss();
	return count;
}

#ifdef CONFIG_ARCH_OMAP34XX
static ssize_t
lcd_alphablend_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int p=0;
	int alphablend_state;

	omap_disp_get_dss();
	alphablend_state = omap_disp_get_alphablend(OMAP_OUTPUT_LCD);
	switch (alphablend_state) {
		case 0:
			p = sprintf(buf, "off\n");
			break;
		case 1:
			p = sprintf(buf, "on\n");
			break;
	}
	omap_disp_put_dss();
	return p;
}

static ssize_t
lcd_alphablend_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	int alpha_state;

	if (!buffer || (count == 0)){
		return 0;
	}
	if (strncmp(buffer,"on",2) == 0){
		alpha_state = 1;
	}
	else if (strncmp(buffer, "off", 3) == 0){
		alpha_state = 0;
	}
	else
		return -EINVAL;
        omap_disp_get_dss();
	omap_disp_set_alphablend(OMAP_OUTPUT_LCD,alpha_state);
        omap_disp_put_dss();
	return count;
}

static ssize_t
tv_alphablend_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int p=0;
	int alphablend_state;

	omap_disp_get_dss();
	alphablend_state = omap_disp_get_alphablend(OMAP_OUTPUT_TV);
	switch (alphablend_state) {
		case 0:
			p = sprintf(buf, "off\n");
			break;
		case 1:
			p = sprintf(buf, "on\n");
			break;
	}
	omap_disp_put_dss();
	return p;
}

static ssize_t
tv_alphablend_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	int alpha_state;

	if (!buffer || (count == 0)){
		return 0;
	}
	if (strncmp(buffer,"on",2) == 0){
		alpha_state = 1;
	}
	else if (strncmp(buffer, "off", 3) == 0){
		alpha_state = 0;
	}
	else
		return -EINVAL;
        omap_disp_get_dss();
	omap_disp_set_alphablend(OMAP_OUTPUT_TV,alpha_state);
        omap_disp_put_dss();
	return count;
}

static ssize_t
gfx_global_alpha_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int p=0;
	unsigned char alphablend_value;

        omap_disp_get_dss();
	alphablend_value = omap_disp_get_global_alphablend_value(OMAP_GRAPHICS);
	p= sprintf(buf, "%d \n",alphablend_value);
	omap_disp_put_dss();
	return p;
}

static ssize_t
gfx_global_alpha_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	unsigned char alpha_value;

	if (!buffer || (count == 0)){
		return 0;
	}

	if (sscanf(buffer,"%hhu",&alpha_value) != 1) {
		printk(KERN_ERR "gfx_global_alpha_store: Invalid value \n");
		return -EINVAL;
	}

	omap_disp_get_dss();
	omap_disp_set_global_alphablend_value(OMAP_GRAPHICS,alpha_value);
        omap_disp_put_dss();
	return count;
}

static ssize_t
vid2_global_alpha_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int p=0;
	int alphablend_value;

        omap_disp_get_dss();
	alphablend_value = omap_disp_get_global_alphablend_value(OMAP_VIDEO2);
	p= sprintf(buf, "%d \n",alphablend_value);
	omap_disp_put_dss();
	return p;
}

static ssize_t
vid2_global_alpha_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	int alpha_value;

	if (!buffer || (count == 0)){
		return 0;
	}

	if (sscanf(buffer,"%d",&alpha_value) != 1) {
		printk(KERN_ERR "gfx_global_alpha_store: Invalid value \n");
		return -EINVAL;
	}

	omap_disp_get_dss();
	omap_disp_set_global_alphablend_value(OMAP_VIDEO2,alpha_value);
        omap_disp_put_dss();
	return count;
}

static ssize_t lpr_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int p = 0;

	switch (lpr_enabled) {
		case 0:
			p = sprintf(buf, "disable\n");
			break;
		case 1:
			p = sprintf(buf, "enable\n");
			break;
	}
	return p;
}

static ssize_t lpr_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	int rc;

	if (!buffer || (count == 0))
		return 0;

	if (strncmp(buffer, "enable", 6) == 0) {
		if ((rc = omap_disp_lpr_enable())) {
			printk("can't enable lpr!\n");
			return rc;
		}
	}
	else if (strncmp(buffer, "disable", 7) == 0) {
		if ((rc = omap_disp_lpr_disable())) {
			printk("can't disable lpr!\n");
			return rc;
		}
	}
	else {
		return -EINVAL;
	}

	return count;
}

static ssize_t gfx_fifo_low_threshold_show(struct device *dev, struct
		device_attribute *attr, char *buf)
{
	int thrs;

	omap_disp_get_dss();
	thrs = omap_disp_get_gfx_fifo_low_threshold();
	omap_disp_put_dss();

	return sprintf(buf, "%d\n", thrs);
}

static ssize_t gfx_fifo_low_threshold_store(struct device *dev, struct
		device_attribute *attr, const char *buffer, size_t count)
{
	unsigned int v;

	if (!buffer || (count == 0)){
		return 0;
	}

	if (sscanf(buffer, "%d", &v) != 1) {
		printk(KERN_ERR "gfx_fifo_low_threshold: Invalid value\n");
		return -EINVAL;
	}

	omap_disp_get_dss();
	omap_disp_set_gfx_fifo_low_threshold(v);
	omap_disp_put_dss();

	return count;
}

static ssize_t gfx_fifo_high_threshold_show(struct device *cdev, struct
		device_attribute *attr, char *buf)
{
	int thrs;

	omap_disp_get_dss();
	thrs = omap_disp_get_gfx_fifo_high_threshold();
	omap_disp_put_dss();

	return sprintf(buf, "%d\n", thrs);
}

static ssize_t gfx_fifo_high_threshold_store(struct device *dev, struct
		device_attribute *attr, const char *buffer, size_t count)
{
	unsigned int v;

	if (!buffer || (count == 0)){
		return 0;
	}

	if (sscanf(buffer, "%d", &v) != 1) {
		printk(KERN_ERR "gfx_fifo_high_threshold: Invalid value\n");
		return -EINVAL;
	}

	omap_disp_get_dss();
	omap_disp_set_gfx_fifo_high_threshold(v);
	omap_disp_put_dss();

	return count;
}

#endif

static ssize_t ch0_output_show(struct device *dev, struct
		device_attribute *attr, char *buf)
{
	int p=0;
	omap_disp_get_all_clks();
	p = sprintf(buf, omap_disp_get_output(0));
	omap_disp_put_all_clks();
	return p;
}
static ssize_t ch0_output_store(struct device *dev, struct device_attribute
		*attr, const char *buf, size_t count)
{
	int ret=0;
	char buffer[MAX_CHAR];
	strncpy(buffer, buf, MAX_CHAR);
	buffer[count-1] = 0;
	omap_disp_get_all_clks();
	ret = omap_disp_set_output(0, buffer);
	omap_disp_put_all_clks();
	if(ret < 0)
		return ret;
	else
		return count;
}
static ssize_t ch1_output_show(struct device *dev, struct device_attribute
		*attr, char *buf)
{
	int p=0;
	omap_disp_get_all_clks();
	p = sprintf(buf, omap_disp_get_output(1));
	omap_disp_put_all_clks();
	return p;
}
static ssize_t ch1_output_store(struct device *dev, struct
		device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	char buffer[MAX_CHAR];
	strncpy(buffer, buf, MAX_CHAR);
	buffer[count-1] = 0;
	omap_disp_get_all_clks();
	ret = omap_disp_set_output(1, buffer);
	omap_disp_put_all_clks();
	if(ret < 0)
		return ret;
	else
		return count;
}
static ssize_t ch0_mode_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int p = 0;
	omap_disp_get_all_clks();
	p = sprintf(buf, omap_disp_get_mode(0));
	omap_disp_put_all_clks();
	return p;
}
static ssize_t ch0_mode_store(struct device *dev, struct device_attribute
		*attr, const char *buf, size_t count)
{
	int ret = 0;
	char buffer[MAX_CHAR];
	strncpy(buffer, buf, MAX_CHAR);
	buffer[count-1] = 0;
	omap_disp_get_all_clks();
	ret = omap_disp_set_mode(0, buffer);
	omap_disp_put_all_clks();
	if(ret < 0)
		return ret;
	else
		return count;
}
static ssize_t ch1_mode_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int p = 0;
	omap_disp_get_all_clks();
	p = sprintf(buf, omap_disp_get_mode(1));
	omap_disp_put_all_clks();
	return p;
}
static ssize_t ch1_mode_store(struct device *dev, struct device_attribute
		*attr, const char *buf, size_t count)
{
	int ret = 0;
	char buffer[MAX_CHAR];
	strncpy(buffer, buf, MAX_CHAR);
	buffer[count-1] = 0;
	omap_disp_get_all_clks();
	ret = omap_disp_set_mode(1, buffer);
	omap_disp_put_all_clks();
	if(ret < 0)
		return ret;
	else
		return count;
}

#define DECLARE_ATTR(_name,_mode,_show,_store)                  \
{                                                               \
	.attr   = { .name = __stringify(_name), .mode = _mode, .owner = THIS_MODULE },  \
	.show   = _show,                                        \
	.store  = _store,                                       \
}

static struct device_attribute bl_device_attributes[] = {
	DECLARE_ATTR(dithering,      S_IRWXUGO, dithering_show,    dithering_store),
	DECLARE_ATTR(graphics,       S_IRWXUGO, graphics_show,     graphics_store),
	DECLARE_ATTR(video1,         S_IRWXUGO, video1_show,       video1_store),
	DECLARE_ATTR(video2,         S_IRWXUGO, video2_show,       video2_store),
	DECLARE_ATTR(lcdbacklight,   S_IRWXUGO, lcdbacklight_show, lcdbacklight_store),
	DECLARE_ATTR(lcd_data_lines, S_IRWXUGO, lcd_data_lines_show, lcd_data_lines_store),
	DECLARE_ATTR(frame_buffer,   S_IRWXUGO, fb_out_show, fb_out_store),

#ifdef CONFIG_ARCH_OMAP34XX
	DECLARE_ATTR(lcd_alphablend, S_IRWXUGO, lcd_alphablend_show, lcd_alphablend_store),
	DECLARE_ATTR(tv_alphablend,  S_IRWXUGO, tv_alphablend_show,  tv_alphablend_store),
	DECLARE_ATTR(gfx_global_alpha,  S_IRWXUGO, gfx_global_alpha_show,  gfx_global_alpha_store),
	DECLARE_ATTR(vid2_global_alpha,  S_IRWXUGO, vid2_global_alpha_show,  vid2_global_alpha_store),
	DECLARE_ATTR(lpr, S_IRWXUGO, lpr_show, lpr_store),
	DECLARE_ATTR(gfx_fifo_low_threshold, S_IRWXUGO,
			gfx_fifo_low_threshold_show,
			gfx_fifo_low_threshold_store),
	DECLARE_ATTR(gfx_fifo_high_threshold, S_IRWXUGO,
			gfx_fifo_high_threshold_show,
			gfx_fifo_high_threshold_store),
#endif
	DECLARE_ATTR(ch0_output,   S_IRWXUGO, ch0_output_show, ch0_output_store),
	DECLARE_ATTR(ch1_output,   S_IRWXUGO, ch1_output_show, ch1_output_store),
	DECLARE_ATTR(ch0_mode,   S_IRWXUGO, ch0_mode_show, ch0_mode_store),
	DECLARE_ATTR(ch1_mode,   S_IRWXUGO, ch1_mode_show, ch1_mode_store),
};

static int
create_sysfs_files(void)
{

	int rc=0,i,ret;

	if((ret = class_register(&dispc_class)) != 0 )
		return ret;

	bd = kmalloc(sizeof(struct board_properties), GFP_KERNEL);
	if (unlikely(!bd))
		return -ENOMEM;

	bd->owner = THIS_MODULE;
	new_bd = kmalloc(sizeof(struct dispc_device), GFP_KERNEL);
	if (unlikely(!new_bd))
		return -ENOMEM;

	new_bd->props = bd;
	memset(&new_bd->class_dev, 0, sizeof(new_bd->class_dev));
	new_bd->class_dev.class = &dispc_class;
	strlcpy(new_bd->class_dev.bus_id, "omap_disp_control", KOBJ_NAME_LEN);
	rc = device_register(&new_bd->class_dev);

	if (unlikely(rc)) {
		kfree(new_bd);
		return -EPERM;
	}
	for (i = 0; i < ARRAY_SIZE(bl_device_attributes); i++) {
		rc = device_create_file(&new_bd->class_dev, &bl_device_attributes[i]);
		if (unlikely(rc)) {
			while (--i >= 0){
				device_remove_file(&new_bd->class_dev,
						&bl_device_attributes[i]);
			}
			device_unregister(&new_bd->class_dev);
			/* No need to kfree(new_bd) since release() method does it for us*/
			return -EPERM;
		}
	}
	return 0;
}

static void
remove_sysfs_files(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(bl_device_attributes); i++) {
		device_remove_file(&new_bd->class_dev,
						&bl_device_attributes[i]);
	}
	device_unregister(&new_bd->class_dev);
}

static int __init
omap_dispout_init(void)
{
	if (create_sysfs_files() < 0) {
		printk(KERN_ERR DRIVER
				"Could not create sysfs files for display control\n");
		return -ENODEV;
	}

#ifdef CONFIG_OMAP_LCD
	/* Register the driver with LDM */
	if (platform_driver_register(&omap_lcd_driver)) {
		printk(KERN_ERR DRIVER ": failed to register omap_lcd driver\n");
		return -ENODEV;
	}

	/* Register the device with LDM */
	if (platform_device_register(&lcd_device)) {
		printk(KERN_ERR DRIVER ": failed to register lcd device\n");
		platform_driver_unregister(&omap_lcd_driver);
		return -ENODEV;
	}
#endif

#ifdef CONFIG_OMAP_TV
	/* Register the driver with LDM */
	if (platform_driver_register(&omap_tv_driver)) {
		printk(KERN_ERR DRIVER ": failed to register omap_tv driver\n");
		return -ENODEV;
	}

	/* Register the device with LDM */
	if (platform_device_register(&tv_device)) {
		printk(KERN_ERR DRIVER ": failed to register tv device\n");
		platform_driver_unregister(&omap_tv_driver);
		return -ENODEV;
	}
#endif

	return 0;

}
device_initcall(omap_dispout_init);

static void __exit
omap_dispout_exit(void)
{
	remove_sysfs_files();

#ifdef CONFIG_OMAP_LCD
	lcd_exit();
	platform_device_unregister(&lcd_device);
	platform_driver_unregister(&omap_lcd_driver);
#endif

#ifdef CONFIG_OMAP_TV
	tv_exit();
	platform_device_unregister(&tv_device);
	platform_driver_unregister(&omap_tv_driver);
#endif
}
module_exit(omap_dispout_exit);

/*---------------------------------------------------------------------------*/
/* Framebuffer related */

/* We're intializing the virtual framebuffer dimensions to
 * (H4_XRES, H4_YRES*3) in order to support triple buffering.  The
 * onscreen framebuffer can be flipped via the panning ioctl by specifying
 * offsets of (0, 0), (0, H4_YRES), or (0, 2*H4_YRES).
 */

static struct fb_var_screeninfo h4_lcd_var = {
	.xres		= H4_LCD_XRES,
	.yres		= H4_LCD_YRES,
	.xres_virtual	= H4_LCD_XRES,
	.yres_virtual	= H4_LCD_YRES*2,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 16,
	.grayscale	= 0,
	.red		= {11, 5, 0},
	.green		= { 5, 6, 0},
	.blue		= { 0, 5, 0},
	.transp		= { 0, 0, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= H4_LCD_PIXCLOCK_MAX,/* picoseconds */
	.left_margin	= 75,		/* pixclocks, Adjusted the left and right
					 * margins to get Vertical sync frequency
					 * of 60Hz
					 */
	.right_margin	= 65,		/* pixclocks */
#ifdef CONFIG_MACH_OMAP3EVM
	.upper_margin	= 2,		/* line clocks */
#else
	.upper_margin	= 8,		/* line clocks */
#endif
	.lower_margin 	= 2,
	.hsync_len	= 4,		/* pixclocks */
	.vsync_len	= 2,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_NONINTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo ntsc_tv_var = {
/* NTSC frame size is 720 * 480,
 * but due to overscan, about 640 x 430 is visible
 */
 	.xres = 640,
 	.yres = 430,
 	.xres_virtual	= 720,
 	.yres_virtual	= 480 * 2,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 16,
	.grayscale	= 0,
	.red		= {11, 5, 0},
	.green		= { 5, 6, 0},
	.blue		= { 0, 5, 0},
	.transp		= { 0, 0, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 0,		/* picoseconds */
	.left_margin	= 0,		/* pixclocks */
	.right_margin	= 0,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 0,		/* pixclocks */
	.vsync_len	= 0,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_INTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo pal_tv_var = {
/* PAL frame size is 720 * 546,
 * but due to overscan, about 640 x 520 is visible
 */
 	.xres = 640,
 	.yres = 480,
 	.xres_virtual	= 720,
 	.yres_virtual	= 576 * 2,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 16,
	.grayscale	= 0,
	.red		= {11, 5, 0},
	.green		= { 5, 6, 0},
	.blue		= { 0, 5, 0},
	.transp		= { 0, 0, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 0,		/* picoseconds */
	.left_margin	= 0,		/* pixclocks */
	.right_margin	= 0,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 0,		/* pixclocks */
	.vsync_len	= 0,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_INTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

void
get_panel_default_var(struct fb_var_screeninfo *var, int output_dev)
{
	if (output_dev == OMAP_OUTPUT_LCD) {
		memcpy((struct fb_var_screeninfo *) var,
				&h4_lcd_var, sizeof(*var));
	}
	else if (output_dev == OMAP_OUTPUT_TV) {
		int tv = omap_disp_get_tvstandard();
		if(tv == PAL_BDGHI ||
				tv == PAL_NC    ||
				tv == PAL_N     ||
				tv == PAL_M     ||
				tv == PAL_60){
			memcpy((struct fb_var_screeninfo *) var,
					&pal_tv_var, sizeof(*var));
		}else {
			memcpy((struct fb_var_screeninfo *) var,
					&ntsc_tv_var, sizeof(*var));
		}
	}
}

u32
get_panel_pixclock_max(int output_dev)
{
	if (output_dev == OMAP_OUTPUT_LCD) {
		return H4_LCD_PIXCLOCK_MAX;
	}
	else if (output_dev == OMAP_OUTPUT_TV) {
		return ~0;
	}

	return -EINVAL;
}

u32
get_panel_pixclock_min(int output_dev)
{
	if (output_dev == OMAP_OUTPUT_LCD) {
		return H4_LCD_PIXCLOCK_MIN;
	}
	else if (output_dev == OMAP_OUTPUT_TV) {
		return 0;
	}

	return -EINVAL;
}

EXPORT_SYMBOL(get_panel_default_var);
EXPORT_SYMBOL(get_panel_pixclock_max);
EXPORT_SYMBOL(get_panel_pixclock_min);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
