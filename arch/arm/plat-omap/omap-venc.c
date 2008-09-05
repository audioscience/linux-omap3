/*
 * arch/arm/plat-omap/omap-venc.c
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Leveraged code from the OMAP24xx camera driver
 * Video-for-Linux (Version 2) camera capture driver for
 * the OMAP24xx camera controller.
 *
 * Author:  (@ti.com)
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * History:
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#ifdef CONFIG_TRACK_RESOURCES
#include <linux/device.h>
#endif
#include <asm/arch/omap-dss.h>
#include <asm/arch/omap-venc.h>
#if defined(CONFIG_MACH_OMAP_2430SDP) ||  defined(CONFIG_MACH_OMAP3EVM) \
|| defined(CONFIG_MACH_OMAP_3430LABRADOR)
#include <linux/i2c/twl4030.h>
#endif
#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif
#include <linux/platform_device.h>

#define OMAP_TV_DRIVER			"omap_tv"

#ifdef CONFIG_MACH_OMAP_H4
#define OMAP_TV_DEVICE		"h4_tv"
#endif

#ifdef CONFIG_MACH_OMAP_2430SDP
#define OMAP_TV_DEVICE		"sdp2430_tv"
#endif

#if defined(CONFIG_MACH_OMAP_3430SDP) ||  defined(CONFIG_MACH_OMAP3EVM) \
|| defined(CONFIG_MACH_OMAP_3430LABRADOR)
#define OMAP_TV_DEVICE		"omap_tv"
#endif

#define H4_TV_XRES		720
#define H4_TV_YRES		482

#define MENELAUS_I2C_ADAP_ID		0

#define ENABLE_VDAC_DEDICATED		0x03
#define ENABLE_VDAC_DEV_GRP		0x20
#define ENABLE_VPLL2_DEDICATED		0x05
#define ENABLE_VPLL2_DEV_GRP		0xE0

#define CONFIG_TWL4030_CORE_T2
#define CONFIG_I2C_TWL4030_CORE

#define ENCODER_NAME	"TV_ENCODER"

struct omap_output_info outputs[] = {
	{"SVIDEO", (void *)tv_standards, ARRAY_SIZE(tv_standards), 5,
	 0}
};

#define OMAP_VENC_NO_OUTPUTS 1
static struct omap_enc_output_ops outputs_ops = {
	.count = OMAP_VENC_NO_OUTPUTS,
	.enumoutput = omap_venc_enumoutput,
	.setoutput = omap_venc_setoutput,
	.getoutput = NULL,
};

static struct omap_enc_mode_ops standards_ops = {
	.setmode = omap_venc_setstd,
	.getmode = omap_venc_getstd,
};

struct omap_encoder_device tv_enc = {
	.name = ENCODER_NAME,
	.channel_id = 1,
	.current_output = 0,
	.no_outputs = 1,
	.initialize = tv_initialize,
	.deinitialize = tv_deinitialize,
	.output_ops = &outputs_ops,
	.mode_ops = &standards_ops,
};

/*
 * VENC register I/O Routines
 */
static inline u32 venc_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + VENC_REG_OFFSET + offset);
}

static inline u32 venc_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + VENC_REG_OFFSET + offset);
	return val;
}

static inline u32 venc_reg_merge(u32 offset, u32 val, u32 mask)
{
	u32 addr = DSS_REG_BASE + VENC_REG_OFFSET + offset;
	u32 new_val = (omap_readl(addr) & ~mask) | (val & mask);

	omap_writel(new_val, addr);
	return new_val;
}

int tv_initialize(void *data)
{
	return 0;
}

int tv_deinitialize(void *data)
{
	return 0;
}

static void config_venc(struct tv_standard_config *tvstd)
{
	int i;

	i = 0;
	/*
	 * Write 1 to the 8th bit of the F_Control register to reset the VENC
	 */
	venc_reg_merge(VENC_F_CONTROL, VENC_FCONTROL_RESET,
			VENC_FCONTROL_RESET);
	/* wait for reset to complete */
	while ((venc_reg_in(VENC_F_CONTROL) & VENC_FCONTROL_RESET) ==
			0x00000100) {
		udelay(10);
		if (i++ > 10)
			break;
	}

	if (venc_reg_in(VENC_F_CONTROL) & VENC_FCONTROL_RESET) {
		printk(KERN_WARNING
				"omap_disp: timeout waiting for venc reset\n");
		/* remove the reset */
		venc_reg_merge(VENC_F_CONTROL, (0 << 8),
				VENC_FCONTROL_RESET);
	}

	venc_reg_out(VENC_LLEN, tvstd->venc_llen);
	venc_reg_out(VENC_FLENS, tvstd->venc_flens);
	venc_reg_out(VENC_HFLTR_CTRL, tvstd->venc_hfltr_ctrl);
	venc_reg_out(VENC_CC_CARR_WSS_CARR, tvstd->venc_cc_carr_wss_carr);
	venc_reg_out(VENC_C_PHASE, tvstd->venc_c_phase);
	venc_reg_out(VENC_GAIN_U, tvstd->venc_gain_u);
	venc_reg_out(VENC_GAIN_V, tvstd->venc_gain_v);
	venc_reg_out(VENC_GAIN_Y, tvstd->venc_gain_y);
	venc_reg_out(VENC_BLACK_LEVEL, tvstd->venc_black_level);
	venc_reg_out(VENC_BLANK_LEVEL, tvstd->venc_blank_level);
	venc_reg_out(VENC_X_COLOR, tvstd->venc_x_color);
	venc_reg_out(VENC_M_CONTROL, tvstd->venc_m_control);
	venc_reg_out(VENC_BSTAMP_WSS_DATA, tvstd->venc_bstamp_wss_data);
	venc_reg_out(VENC_S_CARR, tvstd->venc_s_carr);
	venc_reg_out(VENC_LINE21, tvstd->venc_line21);
	venc_reg_out(VENC_LN_SEL, tvstd->venc_ln_sel);
	venc_reg_out(VENC_L21_WC_CTL, tvstd->venc_l21_wc_ctl);
	venc_reg_out(VENC_HTRIGGER_VTRIGGER,
			tvstd->venc_htrigger_vtrigger);
	venc_reg_out(VENC_SAVID_EAVID, tvstd->venc_savid_eavid);
	venc_reg_out(VENC_FLEN_FAL, tvstd->venc_flen_fal);
	venc_reg_out(VENC_LAL_PHASE_RESET, tvstd->venc_lal_phase_reset);
	venc_reg_out(VENC_HS_INT_START_STOP_X,
			tvstd->venc_hs_int_start_stop_x);
	venc_reg_out(VENC_HS_EXT_START_STOP_X,
			tvstd->venc_hs_ext_start_stop_x);
	venc_reg_out(VENC_VS_INT_START_X, tvstd->venc_vs_int_start_x);
	venc_reg_out(VENC_VS_INT_STOP_X_VS_INT_START_Y,
			tvstd->venc_vs_int_stop_x_vs_int_start_y);
	venc_reg_out(VENC_VS_INT_STOP_Y_VS_EXT_START_X,
			tvstd->venc_vs_int_stop_y_vs_ext_start_x);
	venc_reg_out(VENC_VS_EXT_STOP_X_VS_EXT_START_Y,
			tvstd->venc_vs_ext_stop_x_vs_ext_start_y);
	venc_reg_out(VENC_VS_EXT_STOP_Y, tvstd->venc_vs_ext_stop_y);
	venc_reg_out(VENC_AVID_START_STOP_X,
			tvstd->venc_avid_start_stop_x);
	venc_reg_out(VENC_AVID_START_STOP_Y,
			tvstd->venc_avid_start_stop_y);
	venc_reg_out(VENC_FID_INT_START_X_FID_INT_START_Y,
			tvstd->venc_fid_int_start_x_fid_int_start_y);
	venc_reg_out(VENC_FID_INT_OFFSET_Y_FID_EXT_START_X,
			tvstd->venc_fid_int_offset_y_fid_ext_start_x);
	venc_reg_out(VENC_FID_EXT_START_Y_FID_EXT_OFFSET_Y,
			tvstd->venc_fid_ext_start_y_fid_ext_offset_y);
	venc_reg_out(VENC_TVDETGP_INT_START_STOP_X,
			tvstd->venc_tvdetgp_int_start_stop_x);
	venc_reg_out(VENC_TVDETGP_INT_START_STOP_Y,
			tvstd->venc_tvdetgp_int_start_stop_y);
	venc_reg_out(VENC_GEN_CTRL, tvstd->venc_gen_ctrl);
	venc_reg_out(VENC_DAC_TST, tvstd->venc_dac_tst);
	venc_reg_out(VENC_DAC, venc_reg_in(VENC_DAC));
	venc_reg_out(VENC_F_CONTROL, F_CONTROL_GEN);
	venc_reg_out(VENC_SYNC_CONTROL, SYNC_CONTROL_GEN);
}

int omap_venc_setstd(char *mode_name, void *data)
{
	int i;
	struct omap_encoder_device *enc_dev;

	enc_dev = (struct omap_encoder_device *) data;

	if (!mode_name)
		return -1;
	for (i = 0; i < ARRAY_SIZE(tv_standards); i++) {
		if (!(strcmp(tv_standards[i].std_name, mode_name))) {
			outputs[enc_dev->current_output].current_mode = i;
			config_venc(&tv_standards[i]);
			return 0;
		}
	}
	return -1;

}

char *omap_venc_getstd(void *data)
{
	int mode_index;
	struct omap_encoder_device *enc_dev =
			(struct omap_encoder_device *) data;

	mode_index = outputs[enc_dev->current_output].current_mode;
	return tv_standards[mode_index].std_name;
}

int omap_venc_setoutput(int index, char *mode_name, void *data)
{
	struct omap_encoder_device *enc_dev =
			(struct omap_encoder_device *)data;
	u32 current_mode;

	enc_dev->current_output = index;
	current_mode = outputs[enc_dev->current_output].current_mode;
	config_venc(&tv_standards[current_mode]);
	strcpy(mode_name, tv_standards[current_mode].std_name);

	return 0;
}

char *omap_venc_enumoutput(int index, void *data)
{
	return outputs[index].name;
}

static int tv_in_use;

static void tvout_off(struct work_struct *work);
static void tvout_on(struct work_struct *work);

DECLARE_WORK(work_q_tvout_on, tvout_on);
DECLARE_WORK(work_q_tvout_off, tvout_off);

static void power_tv(int level)
{
	switch (level) {
	case TV_OFF:
		if (!in_interrupt())
			tvout_off(NULL);
		else
			schedule_work(&work_q_tvout_off);
		break;
	default:
		if (!in_interrupt())
			tvout_on(NULL);
		else
			schedule_work(&work_q_tvout_on);
		break;
	}
}

static void tvout_off(struct work_struct *work)
{
#if defined(CONFIG_MACH_OMAP_H4) || defined(CONFIG_TWL4030_CORE_M1)
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	/*
	 * Turn OFF TV block (AVDD and VREF) in menelaus chip
	 * MENELAUS_LDO_CTRL8 (0x11 -> 0x03)
	 */
	adap = i2c_get_adapter(MENELAUS_I2C_ADAP_ID);
	if (!adap)
		printk(KERN_ERR DRIVER "Unable to get I2C adapter \n");
	msg->addr = 0x72;	/* I2C address of Menelaus Chip */
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = 0x11;		/* LD0_CTRL8 */
	data[1] = 0x00;		/* Disable bits for the 0.5V reference LDO */
	err = i2c_transfer(adap, msg, 1);
	if (err > 2) {
		printk(KERN_ERR DRIVER
		       "Disabling TV block through Menelaus failed %d\n",
		       err);
	}
#endif
	omap_disp_set_tvref(TVREF_OFF);

#if (defined(CONFIG_TWL4030_CORE_T2) && defined(CONFIG_I2C_TWL4030_CORE))  \
		|| defined(CONFIG_MACH_OMAP_3430LABRADOR) \
		|| defined(CONFIG_MACH_OMAP3EVM)
	omap_disp_set_tvref(TVREF_OFF);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			     TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			     TWL4030_VDAC_DEV_GRP);
#endif
}

static void tvout_on(struct work_struct *work)
{
#if defined(CONFIG_MACH_OMAP_H4) || defined(CONFIG_TWL4030_CORE_M1)
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	/*
	 * Turn ON TV block (AVDD and VREF) in menelaus chip
	 * MENELAUS_LDO_CTRL8 (0x11 -> 0x03)
	 */
	adap = i2c_get_adapter(MENELAUS_I2C_ADAP_ID);
	if (!adap)
		printk(KERN_ERR DRIVER "Unable to get I2C adapter \n");
	msg->addr = 0x72;	/* I2C address of Menelaus Chip */
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = 0x11;		/* LD0_CTRL8 */
	data[1] = 0x03;		/* Enable bits for the 0.5V reference
				 * and the VADAC LDO
				 */
	err = i2c_transfer(adap, msg, 1);
	if (err > 2) {
		printk(KERN_ERR DRIVER
		       "Enabling TV block through Menelaus failed %d\n",
		       err);
	}
#endif
	omap_disp_set_tvref(TVREF_ON);
#if (defined(CONFIG_TWL4030_CORE_T2) && defined(CONFIG_I2C_TWL4030_CORE)) \
		|| defined(CONFIG_MACH_OMAP_3430LABRADOR)
	omap_disp_set_tvref(TVREF_ON);

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			     ENABLE_VDAC_DEDICATED,
			     TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			     ENABLE_VDAC_DEV_GRP, TWL4030_VDAC_DEV_GRP);
#endif
}

static int tv_init(void)
{
	omap_disp_get_all_clks();
	power_tv(TV_ON);
	omap_disp_enable_output_dev(OMAP_OUTPUT_TV);
	omap_disp_put_all_clks();
	printk(KERN_DEBUG "TV %dx%d interlaced\n", H4_TV_XRES, H4_TV_YRES);
	tv_in_use = 1;
	if (omap_register_encoder(&tv_enc))
		return -1;

	return 0;
}

static int tv_exit(void)
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
		   .name = OMAP_TV_DRIVER,
		   },
	.probe = tv_probe,
#ifdef CONFIG_PM
	.suspend = tv_suspend,
	.resume = tv_resume,
#endif
};

static struct platform_device tv_device = {
	.name = OMAP_TV_DEVICE,
	.id = 10,
};

static int __init tv_probe(struct platform_device *odev)
{
	return tv_init();
}

#ifdef CONFIG_PM
static int tv_suspend(struct platform_device *odev, pm_message_t state)
{
	if (!tv_in_use)
		return 0;

	/* TODO-- need to delink DSS and TV clocks.. For now, TV is put to
	 * off in fb_blank and put_dss */

	tv_in_use = 0;

	return 0;
}

static int tv_resume(struct platform_device *odev)
{
	if (tv_in_use)
		return 0;

	/* TODO-- need to delink DSS and TV clocks.. For now, TV is put to
	 * on in fb_blank and get_dss */
	tv_in_use = 1;
	return 0;
}

#endif				/* CONFIG_PM */

static int __init omap_tv_init(void)
{
	/* Register the driver with LDM */
	if (platform_driver_register(&omap_tv_driver)) {
		printk(KERN_ERR ": failed to register omap_tv driver\n");
		return -ENODEV;
	}
	/* Register the device with LDM */
	if (platform_device_register(&tv_device)) {
		printk(KERN_ERR ": failed to register tv device\n");
		platform_driver_unregister(&omap_tv_driver);
		return -ENODEV;
	}
	return 0;

}

device_initcall(omap_tv_init);

static void __exit
omap_tv_exit(void)
{

	tv_exit();
	platform_device_unregister(&tv_device);
	platform_driver_unregister(&omap_tv_driver);
}
module_exit(omap_tv_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
