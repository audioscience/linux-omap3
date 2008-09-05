/*
 * arch/arm/mach-omap2/board-omap3evm-dc.c
 *
 * Driver for OMAP3 EVM Daughter Card
 *
 * Copyright (C) 2008 Texas Instruments Inc
 *
 * Contributors:
 *     Anuj Aggarwal <anuj.aggarwal@ti.com>
 *     Sivaraj R <sivaraj@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>

#include <asm/arch/io.h>
#include <asm/arch/gpmc.h>

#if defined(CONFIG_VIDEO_TVP5146) || defined(CONFIG_VIDEO_TVP5146_MODULE)
#include <linux/videodev2.h>
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/tvp5146.h>

#if defined(CONFIG_VIDEO_OMAP3_CAMERA) \
	|| defined(CONFIG_VIDEO_OMAP3_CAMERA_MODULE)
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#endif				/* #ifdef CONFIG_VIDEO_OMAP3_CAMERA */

#endif				/* #ifdef CONFIG_VIDEO_TVP5146 */

#include "board-omap3evm-dc.h"

#define MODULE_NAME			"omap3evmdc"

#ifdef DEBUG
#define dprintk(fmt, args...) printk(KERN_ERR MODULE_NAME ": " fmt, ## args)
#else
#define dprintk(fmt, args...)
#endif				/* #ifdef DEBUG */

/* Macro Definitions */

/* System control module register offsets */
#define REG_CONTROL_PADCONF_GPMC_NCS3	(0x480020B4u)
#define REG_CONTROL_PADCONF_I2C2_SDA	(0x480021C0u)
#define REG_CONTROL_PADCONF_I2C3_SDA	(0x480021C4u)

#define PADCONF_I2C3_SCL_MASK		(0xFFFF0000u)
#define PADCONF_I2C3_SDA_MASK		(0x0000FFFFu)
#define PADCONF_GPMC_NCS4_MASK		(0xFFFF0000u)

/* mux mode 0 (enable I2C3 SCL), pull-up enable, input enable */
#define PADCONF_I2C3_SCL_DEF		(0x01180000u)
/* mux mode 0 (enable I2C3 SDA), pull-up enable, input enable */
#define PADCONF_I2C3_SDA_DEF		(0x00000118u)
/* mux mode 0 (enable GPMC CS4), pull-up enable */
#define PADCONF_GPMC_NCS4_DEF		(0x00180000u)

/* board internal information (BEGIN) */

/* GPMC chip select used in board */
#define BOARD_GPMC_CS			(4)
/* I2C bus to which all I2C slave devices are attached */
#define BOARD_I2C_BUSNUM		(3)

/* I2C address of chips present in board */
#define TVP5146_I2C_ADDR		(0x5D)

/* Register offsets */
#define REG_MLC_NCS0			(0x00000000u)
#define REG_MLC_NCS1			(0x00000040u)
#define REG_F_ACSN			(0x00000080u)
#define REG_F_SCSN			(0x000000C0u)
#define REG_PATA_NCS			(0x00000100u)
#define REG_BUS_CTRL1			(0x00000180u)
#define REG_BUS_CTRL2			(0x000001C0u)

/* Bit defines for Bus Control 1 register */
#define TVP5146_EN_SHIFT		(0x0000u)
#define TVP5146_EN_MASK			(1u << TVP5146_EN_SHIFT)

#define MCBSP2_EN_SHIFT			(0x0001u)
#define MCBSP2_EN_MASK			(0x0006u)	/* two bits used */
#define MCBSP2_DISCONNECT_BIT		(0x0000u)
#define MCBSP2_AIC23_EN_BIT		(0x0001u)
#define MCBSP2_SPDIF_EN_BIT		(0x0002u)
#define MCBSP2_HDMI_EN_BIT		(0x0003u)

#define BT_MCBSP3_EN_SHIFT		(0x0003u)
#define BT_MCBSP3_EN_MASK		(1u << BT_MCBSP3_EN_SHIFT)

#define JAC_MCSPI2_EN_SHIFT		(0x0004u)
#define JAC_MCSPI2_EN_MASK		(1u << JAC_MCSPI2_EN_SHIFT)

#define BT_UART1_EN_SHIFT		(0x0005u)
#define BT_UART1_EN_MASK		(1u << BT_UART1_EN_SHIFT)

#define MLC_NAND_EN_SHIFT		(0x0006u)
#define MLC_NAND_EN_MASK		(1u << MLC_NAND_EN_SHIFT)

#define JAC_MCBSP1_EN_SHIFT		(0x0007u)
#define JAC_MCBSP1_EN_MASK		(1u << JAC_MCBSP1_EN_SHIFT)

#define IMAGE_SENSOR_EN_SHIFT		(0x0008u)
#define IMAGE_SENSOR_EN_MASK		(1u << IMAGE_SENSOR_EN_SHIFT)

#define JAC_MCBSP3_EN_SHIFT		(0x0009u)
#define JAC_MCBSP3_EN_MASK		(1u << JAC_MCBSP3_EN_SHIFT)

#define SPDIF_MCSPI2_EN_SHIFT		(0x000Au)
#define SPDIF_MCSPI2_EN_MASK		(1u << SPDIF_MCSPI2_EN_SHIFT)

/* default value for bus control registers */
#define BUS_CONTROL1_DEF		(0x0141u)	/* Disable all mux */
#define BUS_CONTROL2_DEF		(0x010Au)	/* Disable all mux */

/* board internal information (END) */

/**
 * struct gpmc_info -  Structure to store GPMC information.
 * @gpmc_phy_base - GPMC device physical base address.
 * @gpmc_vir_base - GPMC device virtual base address.
 * @bus_control1 - Bus control register 1.
 * @bus_control2 - Bus control register 2.
 * @reg_lock - To lock access to GPMC bus control registers.
 */
struct gpmc_info {
	u32 *gpmc_phy_base;
	u32 *gpmc_vir_base;

	u16 bus_control1;
	u16 bus_control2;

	spinlock_t reg_lock;
};
static struct gpmc_info gpmc_handle;

/**
 * @brief write_gpmc_reg - writes to registers through GPMC.
 *
 * @param val - value to write
 * @param reg - register offset
 *
 * @note bus control registers are write only registers. Hence the values are
 *       updated locally in variables and are used when they are read.
 */
static inline void write_gpmc_reg(u16 val, u32 reg)
{
	if (reg == REG_BUS_CTRL1)
		gpmc_handle.bus_control1 = val;
	else if (reg == REG_BUS_CTRL2)
		gpmc_handle.bus_control2 = val;

	__raw_writew(val, ((u32) gpmc_handle.gpmc_vir_base + reg));
}

/**
 * @brief read_gpmc_reg - read register value
 *
 * @param reg - register offset
 * @return register value
 *
 * @note bus control registers are write only registers. Hence the values are
 *       updated locally in variables and are used when they are read.
 *       Read access to other registers are unknown and hence GPMC read
 *       operation is performed as is.
 */
static inline u16 read_gpmc_reg(u32 reg)
{
	if (reg == REG_BUS_CTRL1)
		return gpmc_handle.bus_control1;
	else if (reg == REG_BUS_CTRL2)
		return gpmc_handle.bus_control2;
	else
		return __raw_readw((u32) gpmc_handle.gpmc_vir_base + reg);
}

/**
 * @brief omap3evmdc_set_mux - Sets mux to enable/disable signal routing to
 *                             different peripherals present in board
 * IMPORTANT - This function will take care of writing appropriate values for
 * active low signals as well
 *
 * @param mux_id - enum, mux id to enable/disable
 * @param value - enum, ENABLE_MUX for enabling and DISABLE_MUX for disabling
 *
 * @return result of operation - 0 is success
 */
int omap3evmdc_set_mux(enum omap3evmdc_mux mux_id, enum config_mux value)
{
	int err = 0;
	u16 bus_ctrl;
	unsigned long flags;

	if (unlikely(mux_id >= NUM_MUX)) {
		dprintk("Invalid mux id\n");
		return -EPERM;
	}

	spin_lock_irqsave(&gpmc_handle.reg_lock, flags);

	switch (mux_id) {
	case MUX_TVP5146:
		bus_ctrl = read_gpmc_reg(REG_BUS_CTRL1);
		bus_ctrl &= ~TVP5146_EN_MASK;

		/* active low signal. set 0 to enable, 1 to disable */
		if (DISABLE_MUX == value)
			bus_ctrl |= (1 << TVP5146_EN_SHIFT);

		write_gpmc_reg(bus_ctrl, REG_BUS_CTRL1);
		break;

	case MUX_AIC23_MCBSP2:
		bus_ctrl = read_gpmc_reg(REG_BUS_CTRL1);
		bus_ctrl &= ~MCBSP2_EN_MASK;

		/* active high signal. set 1 to enable, 0 to disable */
		if (ENABLE_MUX == value)
			bus_ctrl |=
			    (MCBSP2_AIC23_EN_BIT << MCBSP2_EN_SHIFT);

		write_gpmc_reg(bus_ctrl, REG_BUS_CTRL1);
		break;

	case MUX_SPDIF_MCBSP2:
		bus_ctrl = read_gpmc_reg(REG_BUS_CTRL1);
		bus_ctrl &= ~MCBSP2_EN_MASK;

		/* active high signal. set 1 to enable, 0 to disable */
		if (ENABLE_MUX == value)
			bus_ctrl |=
			    (MCBSP2_SPDIF_EN_BIT << MCBSP2_EN_SHIFT);

		write_gpmc_reg(bus_ctrl, REG_BUS_CTRL1);
		break;

	case MUX_HDMI_MCBSP2:
		bus_ctrl = read_gpmc_reg(REG_BUS_CTRL1);
		bus_ctrl &= ~MCBSP2_EN_MASK;

		/* active high signal. set 1 to enable, 0 to disable */
		if (ENABLE_MUX == value)
			bus_ctrl |=
			    (MCBSP2_HDMI_EN_BIT << MCBSP2_EN_SHIFT);

		write_gpmc_reg(bus_ctrl, REG_BUS_CTRL1);
		break;

	case MUX_BT_MCBSP3:
		bus_ctrl = read_gpmc_reg(REG_BUS_CTRL1);
		bus_ctrl &= ~BT_MCBSP3_EN_MASK;

		/* active high signal. set 1 to enable, 0 to disable */
		if (ENABLE_MUX == value)
			bus_ctrl |= (1 << BT_MCBSP3_EN_SHIFT);

		write_gpmc_reg(bus_ctrl, REG_BUS_CTRL1);
		break;

	case MUX_JAC_MCSPI2:
		bus_ctrl = read_gpmc_reg(REG_BUS_CTRL1);
		bus_ctrl &= ~JAC_MCSPI2_EN_MASK;

		/* active high signal. set 1 to enable, 0 to disable */
		if (ENABLE_MUX == value)
			bus_ctrl |= (1 << JAC_MCSPI2_EN_SHIFT);

		write_gpmc_reg(bus_ctrl, REG_BUS_CTRL1);
		break;

	case MUX_BT_UART1:
		bus_ctrl = read_gpmc_reg(REG_BUS_CTRL1);
		bus_ctrl &= ~BT_UART1_EN_MASK;

		/* active high signal. set 1 to enable, 0 to disable */
		if (ENABLE_MUX == value)
			bus_ctrl |= (1 << BT_UART1_EN_SHIFT);

		write_gpmc_reg(bus_ctrl, REG_BUS_CTRL1);
		break;

	case MUX_MLC_NAND:
		bus_ctrl = read_gpmc_reg(REG_BUS_CTRL1);
		bus_ctrl &= ~MLC_NAND_EN_MASK;

		/* active low signal. set 0 to enable, 1 to disable */
		if (DISABLE_MUX == value)
			bus_ctrl |= (1 << MLC_NAND_EN_SHIFT);

		write_gpmc_reg(bus_ctrl, REG_BUS_CTRL1);
		break;

	case MUX_JAC_MCBSP1:
		bus_ctrl = read_gpmc_reg(REG_BUS_CTRL1);
		bus_ctrl &= ~JAC_MCBSP1_EN_MASK;

		/* active high signal. set 1 to enable, 0 to disable */
		if (ENABLE_MUX == value)
			bus_ctrl |= (1 << JAC_MCBSP1_EN_SHIFT);

		write_gpmc_reg(bus_ctrl, REG_BUS_CTRL1);
		break;

	case MUX_IMAGE_SENSOR:
		bus_ctrl = read_gpmc_reg(REG_BUS_CTRL1);
		bus_ctrl &= ~IMAGE_SENSOR_EN_MASK;

		/* active low signal. set 0 to enable, 1 to disable */
		if (DISABLE_MUX == value)
			bus_ctrl |= (1 << IMAGE_SENSOR_EN_SHIFT);

		write_gpmc_reg(bus_ctrl, REG_BUS_CTRL1);
		break;

	case MUX_JAC_MCBSP3:
		bus_ctrl = read_gpmc_reg(REG_BUS_CTRL1);
		bus_ctrl &= ~JAC_MCBSP3_EN_MASK;

		/* active high signal. set 1 to enable, 0 to disable */
		if (ENABLE_MUX == value)
			bus_ctrl |= (1 << JAC_MCBSP3_EN_SHIFT);

		write_gpmc_reg(bus_ctrl, REG_BUS_CTRL1);
		break;

	case MUX_SPDIF_MCSPI2:
		bus_ctrl = read_gpmc_reg(REG_BUS_CTRL1);
		bus_ctrl &= ~SPDIF_MCSPI2_EN_MASK;

		/* active high signal. set 1 to enable, 0 to disable */
		if (ENABLE_MUX == value)
			bus_ctrl |= (1 << SPDIF_MCSPI2_EN_SHIFT);

		write_gpmc_reg(bus_ctrl, REG_BUS_CTRL1);
		break;

	case NUM_MUX:
	default:
		dprintk("Invalid mux id\n");
		err = -EPERM;
	}

	spin_unlock_irqrestore(&gpmc_handle.reg_lock, flags);

	return err;
}
EXPORT_SYMBOL(omap3evmdc_set_mux);

#if defined(CONFIG_VIDEO_TVP5146) || defined(CONFIG_VIDEO_TVP5146_MODULE)

/* TVP5146 default register values */
static struct tvp5146_reg tvp5146_reg_list[] = {
	{TOK_WRITE, REG_INPUT_SEL, 0x05},	/* Composite selected */
	{TOK_WRITE, REG_AFE_GAIN_CTRL, 0x0F},
	{TOK_WRITE, REG_VIDEO_STD, 0x00},	/* Auto mode */
	{TOK_WRITE, REG_OPERATION_MODE, 0x00},
	{TOK_SKIP, REG_AUTOSWITCH_MASK, 0x3F},
	{TOK_WRITE, REG_COLOR_KILLER, 0x10},
	{TOK_WRITE, REG_LUMA_CONTROL1, 0x00},
	{TOK_WRITE, REG_LUMA_CONTROL2, 0x00},
	{TOK_WRITE, REG_LUMA_CONTROL3, 0x02},
	{TOK_WRITE, REG_BRIGHTNESS, 0x80},
	{TOK_WRITE, REG_CONTRAST, 0x80},
	{TOK_WRITE, REG_SATURATION, 0x80},
	{TOK_WRITE, REG_HUE, 0x00},
	{TOK_WRITE, REG_CHROMA_CONTROL1, 0x00},
	{TOK_WRITE, REG_CHROMA_CONTROL2, 0x0E},
	{TOK_SKIP, 0x0F, 0x00},	/* Reserved */
	{TOK_WRITE, REG_COMP_PR_SATURATION, 0x80},
	{TOK_WRITE, REG_COMP_Y_CONTRAST, 0x80},
	{TOK_WRITE, REG_COMP_PB_SATURATION, 0x80},
	{TOK_SKIP, 0x13, 0x00},	/* Reserved */
	{TOK_WRITE, REG_COMP_Y_BRIGHTNESS, 0x80},
	{TOK_SKIP, 0x15, 0x00},	/* Reserved */
	{TOK_SKIP, REG_AVID_START_PIXEL_LSB, 0x55},	/* NTSC timing */
	{TOK_SKIP, REG_AVID_START_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_AVID_STOP_PIXEL_LSB, 0x25},
	{TOK_SKIP, REG_AVID_STOP_PIXEL_MSB, 0x03},
	{TOK_SKIP, REG_HSYNC_START_PIXEL_LSB, 0x00},	/* NTSC timing */
	{TOK_SKIP, REG_HSYNC_START_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_HSYNC_STOP_PIXEL_LSB, 0x40},
	{TOK_SKIP, REG_HSYNC_STOP_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_VSYNC_START_LINE_LSB, 0x04},	/* NTSC timing */
	{TOK_SKIP, REG_VSYNC_START_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VSYNC_STOP_LINE_LSB, 0x07},
	{TOK_SKIP, REG_VSYNC_STOP_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VBLK_START_LINE_LSB, 0x01},	/* NTSC timing */
	{TOK_SKIP, REG_VBLK_START_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VBLK_STOP_LINE_LSB, 0x15},
	{TOK_SKIP, REG_VBLK_STOP_LINE_MSB, 0x00},
	{TOK_SKIP, 0x26, 0x00},	/* Reserved */
	{TOK_SKIP, 0x27, 0x00},	/* Reserved */
	{TOK_SKIP, REG_FAST_SWTICH_CONTROL, 0xCC},
	{TOK_SKIP, 0x29, 0x00},	/* Reserved */
	{TOK_SKIP, REG_FAST_SWTICH_SCART_DELAY, 0x00},
	{TOK_SKIP, 0x2B, 0x00},	/* Reserved */
	{TOK_SKIP, REG_SCART_DELAY, 0x00},
	{TOK_SKIP, REG_CTI_DELAY, 0x00},
	{TOK_SKIP, REG_CTI_CONTROL, 0x00},
	{TOK_SKIP, 0x2F, 0x00},	/* Reserved */
	{TOK_SKIP, 0x30, 0x00},	/* Reserved */
	{TOK_SKIP, 0x31, 0x00},	/* Reserved */
	{TOK_WRITE, REG_SYNC_CONTROL, 0x0C},	/* HS, VS active high */
	{TOK_WRITE, REG_OUTPUT_FORMATTER1, 0x00},	/* 10-bit BT.656 */
	{TOK_WRITE, REG_OUTPUT_FORMATTER2, 0x11},	/* Enable clk & data */
	{TOK_WRITE, REG_OUTPUT_FORMATTER3, 0xEE},	/* Enable AVID & FLD */
	{TOK_WRITE, REG_OUTPUT_FORMATTER4, 0xAF},	/* Enable VS & HS */
	{TOK_WRITE, REG_OUTPUT_FORMATTER5, 0xFF},
	{TOK_WRITE, REG_OUTPUT_FORMATTER6, 0xFF},
	{TOK_WRITE, REG_CLEAR_LOST_LOCK, 0x01},	/* Clear status */
	{TOK_TERM, 0, 0}
};

/* Supported inputs - Composite and S-Video */
static const struct tvp5146_input_info tvp5146_input_list[] = {
	{
	 .input_sel = 0x05,	/* Composite input 2_B */
	 .lock_mask = 0x0E,	/* Color subcarrier VS & HS lock */
	 .input = {
		   .index = 0,
		   .name = "Vin (Composite)",
		   .type = V4L2_INPUT_TYPE_CAMERA,
		   .std = V4L2_STD_NTSC | V4L2_STD_PAL,}
	 },
	{
	 .input_sel = 0x46,	/* S-Video input 2_C(Y), 1_C(C) */
	 .lock_mask = 0x06,	/* VS & HS lock */
	 .input = {
		   .index = 1,
		   .name = "Vin (S-Video)",
		   .type = V4L2_INPUT_TYPE_CAMERA,
		   .std = V4L2_STD_NTSC | V4L2_STD_PAL,}
	 }
};

#define TVP5146_NUM_INPUTS		ARRAY_SIZE(tvp5146_input_list)

#if defined(CONFIG_VIDEO_OMAP3_CAMERA) \
	|| defined(CONFIG_VIDEO_OMAP3_CAMERA_MODULE)
static struct omap34xxcam_hw_config decoder_hwc = {
	.dev_index = 0,
	.dev_minor = 0,
	.dev_type = OMAP34XXCAM_SLAVE_SENSOR,
	.u.sensor.xclk = OMAP34XXCAM_XCLK_NONE,
	.u.sensor.sensor_isp = V4L2_IF_CAP_SOC,
};

static struct isp_interface_config tvp5146_if_config = {
	.ccdc_par_ser = ISP_PARLL_YUV_BT,
	.dataline_shift = 0x2,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.vdint0_timing = 0x0,
	.vdint1_timing = 0x0,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.u.par.par_bridge = 0x0,
	.u.par.par_clk_pol = 0x0,
};
#endif

static struct v4l2_ifparm ifparm = {
	.capability = V4L2_IF_CAP_SOC,
	.if_type = V4L2_IF_TYPE_BT656,
	.u = {
	      .bt656 = {
			.frame_start_on_rising_vs = 1,
			.bt_sync_correct = 0,
			.swap = 0,
			.latch_clk_inv = 0,
			.nobt_hs_inv = 0,	/* active high */
			.nobt_vs_inv = 0,	/* active high */
			.mode = V4L2_IF_TYPE_BT656_MODE_BT_8BIT,
			.clock_min = TVP5146_XCLK_BT656,
			.clock_max = TVP5146_XCLK_BT656,
			},
	      },
};

/**
 * @brief tvp5146_ifparm - Returns the TVP5146 decoder interface parameters
 *
 * @param p - pointer to v4l2_ifparm structure
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_ifparm(struct v4l2_ifparm *p)
{
	if (p == NULL)
		return -EINVAL;

	*p = ifparm;
	return 0;
}

/**
 * @brief tvp5146_set_prv_data - Returns tvp5146 omap34xx driver private data
 *
 * @param priv - pointer to omap34xxcam_hw_config structure
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_set_prv_data(void *priv)
{
#if defined(CONFIG_VIDEO_OMAP3_CAMERA) \
	|| defined(CONFIG_VIDEO_OMAP3_CAMERA_MODULE)
	struct omap34xxcam_hw_config *hwc = priv;

	if (priv == NULL)
		return -EINVAL;

	hwc->u.sensor.sensor_isp = decoder_hwc.u.sensor.sensor_isp;
	hwc->u.sensor.xclk = decoder_hwc.u.sensor.xclk;
	hwc->dev_index = decoder_hwc.dev_index;
	hwc->dev_minor = decoder_hwc.dev_minor;
	hwc->dev_type = decoder_hwc.dev_type;
	return 0;
#else
	return -EINVAL;
#endif
}

/**
 * @brief tvp5146_power_set - Power-on or power-off TVP5146 device
 *
 * @param power - enum, Power on/off, resume/standby
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_power_set(enum v4l2_power power)
{
	switch (power) {
	case V4L2_POWER_OFF:
#if defined(CONFIG_VIDEO_OMAP3_CAMERA) \
	|| defined(CONFIG_VIDEO_OMAP3_CAMERA_MODULE)
		if (isp_free_interface(ISP_PARLL_YUV_BT))
			return -ENODEV;
#endif

		/* Disable mux for TVP5146 decoder data path */
		if (omap3evmdc_set_mux(MUX_TVP5146, DISABLE_MUX))
			return -ENODEV;
		break;

	case V4L2_POWER_STANDBY:
		break;

	case V4L2_POWER_ON:
#if defined(CONFIG_VIDEO_OMAP3_CAMERA) \
	|| defined(CONFIG_VIDEO_OMAP3_CAMERA_MODULE)
		if (isp_request_interface(ISP_PARLL_YUV_BT))
			return -ENODEV;

		isp_configure_interface(&tvp5146_if_config);
#endif

		/* Enable mux for TVP5146 decoder data path */
		if (omap3evmdc_set_mux(MUX_TVP5146, ENABLE_MUX)) {
#if defined(CONFIG_VIDEO_OMAP3_CAMERA) \
	|| defined(CONFIG_VIDEO_OMAP3_CAMERA_MODULE)
			isp_free_interface(ISP_PARLL_YUV_BT);
#endif
			return -ENODEV;
		}
		break;

	case V4L2_POWER_RESUME:
		break;

	default:
		return -ENODEV;
		break;
	}
	return 0;
}

static struct tvp5146_platform_data tvp5146_pdata = {
	.power_set = tvp5146_power_set,
	.priv_data_set = tvp5146_set_prv_data,
	.ifparm = tvp5146_ifparm,

	/* TVP5146 regsiter list, contains default values */
	.reg_list = tvp5146_reg_list,

	/* Number of supported inputs */
	.num_inputs = TVP5146_NUM_INPUTS,
	.input_list = tvp5146_input_list,
};

static struct i2c_board_info __initdata tvp5146_i2c_board_info = {
	I2C_BOARD_INFO(TVP5146_MODULE_NAME, TVP5146_I2C_ADDR),
	.platform_data = &tvp5146_pdata,
};

#endif				/* #ifdef CONFIG_VIDEO_TVP5146 */

/*
 * GPMC timing to access mux registers - values in ns.
 * Timings based on non-muxed (limited address) GPMC mode of operation
 */
static struct gpmc_timings board_gpmc_timings = {
	.sync_clk = 20,		/* Don't care for async operation */

	.cs_on = 30,		/* Assertion time */
	.cs_rd_off = 96,	/* Read deassertion time */
	.cs_wr_off = 150,	/* Write deassertion time */

	.adv_on = 30,		/* Assertion time */
	.adv_rd_off = 12,	/* Read deassertion time */
	.adv_wr_off = 150,	/* Write deassertion time */

	.we_on = 60,		/* WE assertion time */
	.we_off = 150,		/* WE deassertion time */

	.oe_on = 30,		/* OE assertion time */
	.oe_off = 150,		/* OE deassertion time */

	.page_burst_access = 6,
	.access = 90,		/* Start-cycle to first data valid delay */

	.rd_cycle = 102,	/* Total read cycle time */
	.wr_cycle = 186,	/* Total write cycle time */
};

/**
 * @brief omap3evmdc_gpmc_init - acquires and configures BOARD_GPMC_CS GPMC
 *                               chip select
 *
 * @return result of operation - 0 is success
 */
static int omap3evmdc_gpmc_init(void)
{
	int err;
	int cs = BOARD_GPMC_CS;
	u32 *gpmc_phy_base, *gpmc_vir_base;
	unsigned long flags;

	err = gpmc_cs_set_timings(cs, &board_gpmc_timings);
	if (err < 0) {
		dprintk("Couldn't set GPMC timing for CS%d \n", cs);
		return err;
	}

	err =
	    gpmc_cs_request(cs, SZ_16M, (unsigned long *) &gpmc_phy_base);
	if (err < 0) {
		dprintk("gpmc_cs_request failed: err = %d\n\n", err);
		return err;
	}

	/* remap the GPMC memory - disable cache since we deal with registers */
	gpmc_vir_base =
	    (u32 *) ioremap_nocache((unsigned long) gpmc_phy_base, SZ_16M);

	printk(KERN_INFO MODULE_NAME
	       ": GPMC cs%d request passed. Physical base: 0x%p "
	       "Virtual base: 0x%p \n", cs, gpmc_phy_base, gpmc_vir_base);

	if (gpmc_vir_base != NULL) {
		spin_lock_init(&gpmc_handle.reg_lock);

		gpmc_handle.gpmc_phy_base = gpmc_phy_base;
		gpmc_handle.gpmc_vir_base = gpmc_vir_base;

		spin_lock_irqsave(&gpmc_handle.reg_lock, flags);

		/* write default values */
		write_gpmc_reg(BUS_CONTROL1_DEF, REG_BUS_CTRL1);
		write_gpmc_reg(BUS_CONTROL2_DEF, REG_BUS_CTRL2);

		spin_unlock_irqrestore(&gpmc_handle.reg_lock, flags);
	}

	return err;
}

/**
 * @brief omap3evmdc_init - module init function. Should be called before any
 *                          client driver init call
 *
 * @return result of operation - 0 is success
 */
static int __init omap3evmdc_init(void)
{
	int err;

	/*
	 * I2C3 SCL pin mux settings - mux mode 0, pull-up enable, input enable
	 * Uses the MSB 16-bit of this register, retain the LSB 16-bit.
	 * This pin is shared with gpio_184 (mux mode 4)
	 */
	omap_writel(((omap_readl(REG_CONTROL_PADCONF_I2C2_SDA) &
		      ~PADCONF_I2C3_SCL_MASK) | PADCONF_I2C3_SCL_DEF),
		    REG_CONTROL_PADCONF_I2C2_SDA);

	/*
	 * I2C3 SDA pin mux settings - mux mode 0, pull-up enable, input enable
	 * Uses the LSB 16-bit of this register, retain the MSB 16-bit.
	 * This pin is shared with gpio_185 (mux mode 4)
	 */
	omap_writel(((omap_readl(REG_CONTROL_PADCONF_I2C3_SDA) &
		      ~PADCONF_I2C3_SDA_MASK) | PADCONF_I2C3_SDA_DEF),
		    REG_CONTROL_PADCONF_I2C3_SDA);

	/*
	 * GPMC CS4 pin mux settings - mux mode 0, pull-up enable
	 * Uses the MSB 16-bit of this register, retain the LSB 16-bit.
	 * This pin is shared with sys_ndmareq1 (mux mode 1), mcbsp4_clkx
	 * (mux mode 2), gpt9_pwm_evt (mux mode 3), gpio_55 (mux mode 4)
	 */
	omap_writel(((omap_readl(REG_CONTROL_PADCONF_GPMC_NCS3) &
		      ~PADCONF_GPMC_NCS4_MASK) | PADCONF_GPMC_NCS4_DEF),
		    REG_CONTROL_PADCONF_GPMC_NCS3);

	err = omap3evmdc_gpmc_init();
	if (err) {
		dprintk("GPMC init failed \n");
		return err;
	}

	/*
	 * Register each of the I2C devices present in the board to the I2C
	 * framework.
	 * If more I2C devices are added, then each device information should
	 * be registered with I2C using i2c_register_board_info().
	 */
#if defined(CONFIG_VIDEO_TVP5146) || defined(CONFIG_VIDEO_TVP5146_MODULE)
	err = i2c_register_board_info(BOARD_I2C_BUSNUM,
					&tvp5146_i2c_board_info, 1);
	if (err) {
		dprintk("TVP5146 I2C Board Registration failed \n");
		gpmc_cs_free(BOARD_GPMC_CS);
		return err;
	}
#endif

	printk(KERN_INFO MODULE_NAME ": Driver registration complete \n");

	return 0;
}

/**
 * @brief omap3evmdc_exit - module exit function.
 *
 * @return result of operation - 0 is success
 */
static void __exit omap3evmdc_exit(void)
{
	gpmc_cs_free(BOARD_GPMC_CS);
}

arch_initcall(omap3evmdc_init);
module_exit(omap3evmdc_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP3 EVM Daughter Card Driver");
MODULE_LICENSE("GPL");
