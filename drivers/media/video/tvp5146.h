/*
 * drivers/media/video/tvp5146.h
 *
 * Copyright (C) 2008 Texas Instruments Inc
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

#ifndef _TVP5146_H
#define _TVP5146_H

/*
 * TVP5146 registers
 */
#define REG_INPUT_SEL			(0x00)
#define REG_AFE_GAIN_CTRL		(0x01)
#define REG_VIDEO_STD			(0x02)
#define REG_OPERATION_MODE		(0x03)
#define REG_AUTOSWITCH_MASK		(0x04)

#define REG_COLOR_KILLER		(0x05)
#define REG_LUMA_CONTROL1		(0x06)
#define REG_LUMA_CONTROL2		(0x07)
#define REG_LUMA_CONTROL3		(0x08)

#define REG_BRIGHTNESS			(0x09)
#define REG_CONTRAST			(0x0A)
#define REG_SATURATION			(0x0B)
#define REG_HUE				(0x0C)

#define REG_CHROMA_CONTROL1		(0x0D)
#define REG_CHROMA_CONTROL2		(0x0E)

/* 0x0F Reserved */

#define REG_COMP_PR_SATURATION		(0x10)
#define REG_COMP_Y_CONTRAST		(0x11)
#define REG_COMP_PB_SATURATION		(0x12)

/* 0x13 Reserved */

#define REG_COMP_Y_BRIGHTNESS		(0x14)

/* 0x15 Reserved */

#define REG_AVID_START_PIXEL_LSB	(0x16)
#define REG_AVID_START_PIXEL_MSB	(0x17)
#define REG_AVID_STOP_PIXEL_LSB		(0x18)
#define REG_AVID_STOP_PIXEL_MSB		(0x19)

#define REG_HSYNC_START_PIXEL_LSB	(0x1A)
#define REG_HSYNC_START_PIXEL_MSB	(0x1B)
#define REG_HSYNC_STOP_PIXEL_LSB	(0x1C)
#define REG_HSYNC_STOP_PIXEL_MSB	(0x1D)

#define REG_VSYNC_START_LINE_LSB	(0x1E)
#define REG_VSYNC_START_LINE_MSB	(0x1F)
#define REG_VSYNC_STOP_LINE_LSB		(0x20)
#define REG_VSYNC_STOP_LINE_MSB		(0x21)

#define REG_VBLK_START_LINE_LSB		(0x22)
#define REG_VBLK_START_LINE_MSB		(0x23)
#define REG_VBLK_STOP_LINE_LSB		(0x24)
#define REG_VBLK_STOP_LINE_MSB		(0x25)

/* 0x26 - 0x27 Reserved */

#define REG_FAST_SWTICH_CONTROL		(0x28)

/* 0x29 Reserved */

#define REG_FAST_SWTICH_SCART_DELAY	(0x2A)

/* 0x2B Reserved */

#define REG_SCART_DELAY			(0x2C)
#define REG_CTI_DELAY			(0x2D)
#define REG_CTI_CONTROL			(0x2E)

/* 0x2F - 0x31 Reserved */

#define REG_SYNC_CONTROL		(0x32)
#define REG_OUTPUT_FORMATTER1		(0x33)
#define REG_OUTPUT_FORMATTER2		(0x34)
#define REG_OUTPUT_FORMATTER3		(0x35)
#define REG_OUTPUT_FORMATTER4		(0x36)
#define REG_OUTPUT_FORMATTER5		(0x37)
#define REG_OUTPUT_FORMATTER6		(0x38)
#define REG_CLEAR_LOST_LOCK		(0x39)

#define REG_STATUS1			(0x3A)
#define REG_STATUS2			(0x3B)

#define REG_AGC_GAIN_STATUS_LSB		(0x3C)
#define REG_AGC_GAIN_STATUS_MSB		(0x3D)

/* 0x3E Reserved */

#define REG_VIDEO_STD_STATUS		(0x3F)
#define REG_GPIO_INPUT1			(0x40)
#define REG_GPIO_INPUT2			(0x41)

/* 0x42 - 0x45 Reserved */

#define REG_AFE_COARSE_GAIN_CH1		(0x46)
#define REG_AFE_COARSE_GAIN_CH2		(0x47)
#define REG_AFE_COARSE_GAIN_CH3		(0x48)
#define REG_AFE_COARSE_GAIN_CH4		(0x49)

#define REG_AFE_FINE_GAIN_PB_B_LSB	(0x4A)
#define REG_AFE_FINE_GAIN_PB_B_MSB	(0x4B)
#define REG_AFE_FINE_GAIN_Y_G_CHROMA_LSB	(0x4C)
#define REG_AFE_FINE_GAIN_Y_G_CHROMA_MSB	(0x4D)
#define REG_AFE_FINE_GAIN_PR_R_LSB	(0x4E)
#define REG_AFE_FINE_GAIN_PR_R_MSB	(0x4F)
#define REG_AFE_FINE_GAIN_CVBS_LUMA_LSB	(0x50)
#define REG_AFE_FINE_GAIN_CVBS_LUMA_MSB	(0x51)

/* 0x52 - 0x68 Reserved */

#define REG_FBIT_VBIT_CONTROL1		(0x69)

/* 0x6A - 0x6B Reserved */

#define REG_BACKEND_AGC_CONTROL		(0x6C)

/* 0x6D - 0x6E Reserved */

#define REG_AGC_DECREMENT_SPEED_CONTROL	(0x6F)
#define REG_ROM_VERSION			(0x70)

/* 0x71 - 0x73 Reserved */

#define REG_AGC_WHITE_PEAK_PROCESSING	(0x74)
#define REG_FBIT_VBIT_CONTROL2		(0x75)
#define REG_VCR_TRICK_MODE_CONTROL	(0x76)
#define REG_HORIZONTAL_SHAKE_INCREMENT	(0x77)
#define REG_AGC_INCREMENT_SPEED		(0x78)
#define REG_AGC_INCREMENT_DELAY		(0x79)

/* 0x7A - 0x7F Reserved */

#define REG_CHIP_ID_MSB			(0x80)
#define REG_CHIP_ID_LSB			(0x81)

/* 0x82 Reserved */

#define REG_CPLL_SPEED_CONTROL		(0x83)

/* 0x84 - 0x96 Reserved */

#define REG_STATUS_REQUEST		(0x97)

/* 0x98 - 0x99 Reserved */

#define REG_VERTICAL_LINE_COUNT_LSB	(0x9A)
#define REG_VERTICAL_LINE_COUNT_MSB	(0x9B)

/* 0x9C - 0x9D Reserved */

#define REG_AGC_DECREMENT_DELAY		(0x9E)

/* 0x9F - 0xB0 Reserved */

#define REG_VDP_TTX_FILTER_1_MASK1	(0xB1)
#define REG_VDP_TTX_FILTER_1_MASK2	(0xB2)
#define REG_VDP_TTX_FILTER_1_MASK3	(0xB3)
#define REG_VDP_TTX_FILTER_1_MASK4	(0xB4)
#define REG_VDP_TTX_FILTER_1_MASK5	(0xB5)
#define REG_VDP_TTX_FILTER_2_MASK1	(0xB6)
#define REG_VDP_TTX_FILTER_2_MASK2	(0xB7)
#define REG_VDP_TTX_FILTER_2_MASK3	(0xB8)
#define REG_VDP_TTX_FILTER_2_MASK4	(0xB9)
#define REG_VDP_TTX_FILTER_2_MASK5	(0xBA)
#define REG_VDP_TTX_FILTER_CONTROL	(0xBB)
#define REG_VDP_FIFO_WORD_COUNT		(0xBC)
#define REG_VDP_FIFO_INTERRUPT_THRLD	(0xBD)

/* 0xBE Reserved */

#define REG_VDP_FIFO_RESET		(0xBF)
#define REG_VDP_FIFO_OUTPUT_CONTROL	(0xC0)
#define REG_VDP_LINE_NUMBER_INTERRUPT	(0xC1)
#define REG_VDP_PIXEL_ALIGNMENT_LSB	(0xC2)
#define REG_VDP_PIXEL_ALIGNMENT_MSB	(0xC3)

/* 0xC4 - 0xD5 Reserved */

#define REG_VDP_LINE_START		(0xD6)
#define REG_VDP_LINE_STOP		(0xD7)
#define REG_VDP_GLOBAL_LINE_MODE	(0xD8)
#define REG_VDP_FULL_FIELD_ENABLE	(0xD9)
#define REG_VDP_FULL_FIELD_MODE		(0xDA)

/* 0xDB - 0xDF Reserved */

#define REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR	(0xE0)
#define REG_VBUS_DATA_ACCESS_VBUS_ADDR_INCR	(0xE1)
#define REG_FIFO_READ_DATA			(0xE2)

/* 0xE3 - 0xE7 Reserved */

#define REG_VBUS_ADDRESS_ACCESS1	(0xE8)
#define REG_VBUS_ADDRESS_ACCESS2	(0xE9)
#define REG_VBUS_ADDRESS_ACCESS3	(0xEA)

/* 0xEB - 0xEF Reserved */

#define REG_INTERRUPT_RAW_STATUS0	(0xF0)
#define REG_INTERRUPT_RAW_STATUS1	(0xF1)
#define REG_INTERRUPT_STATUS0		(0xF2)
#define REG_INTERRUPT_STATUS1		(0xF3)
#define REG_INTERRUPT_MASK0		(0xF4)
#define REG_INTERRUPT_MASK1		(0xF5)
#define REG_INTERRUPT_CLEAR0		(0xF6)
#define REG_INTERRUPT_CLEAR1		(0xF7)

/* 0xF8 - 0xFF Reserved */

/*
 * Mask and bit definitions of TVP5146 registers
 */
/* The ID values we are looking for */
#define TVP5146_CHIP_ID_MSB		(0x51)
#define TVP5146_CHIP_ID_LSB		(0x46)

#define VIDEO_STD_MASK			(0x07)
#define VIDEO_STD_AUTO_SWITCH_BIT	(0x00)
#define VIDEO_STD_NTSC_MJ_BIT		(0x01)
#define VIDEO_STD_PAL_BDGHIN_BIT	(0x02)
#define VIDEO_STD_PAL_M_BIT		(0x03)
#define VIDEO_STD_PAL_COMBINATION_N_BIT	(0x04)
#define VIDEO_STD_NTSC_4_43_BIT		(0x05)
#define VIDEO_STD_SECAM_BIT		(0x06)
#define VIDEO_STD_PAL_60_BIT		(0x07)

/*
 * Other macros
 */
#define TVP5146_MODULE_NAME		"tvp5146"
#define TVP5146_I2C_DELAY		(3)
#define I2C_RETRY_COUNT			(5)
#define LOCK_RETRY_COUNT		(3)
#define LOCK_RETRY_DELAY		(200)

#define TOK_WRITE			(0)	/* token for write operation */
#define TOK_TERM			(1)	/* terminating token */
#define TOK_DELAY			(2)	/* delay token for reg list */
#define TOK_SKIP			(3)	/* token to skip a register */

#define TVP5146_XCLK_BT656		(27000000)

/* Number of pixels and number of lines per frame for different standards */
#define NTSC_NUM_ACTIVE_PIXELS		(720)
#define NTSC_NUM_ACTIVE_LINES		(480)
#define PAL_NUM_ACTIVE_PIXELS		(720)
#define PAL_NUM_ACTIVE_LINES		(576)

/**
 * enum tvp5146_std - enum for supported standards
 */
enum tvp5146_std {
	STD_NTSC_MJ = 0,
	STD_PAL_BDGHIN,
	STD_INVALID
};

/**
 * enum tvp5146_state - enum for different decoder states
 */
enum tvp5146_state {
	STATE_NOT_DETECTED,
	STATE_DETECTED
};

/**
 * struct tvp5146_reg - Structure for TVP5146 register initialization values
 * @token - Token: TOK_WRITE, TOK_TERM etc..
 * @reg - Register offset
 * @val - Register Value for TOK_WRITE or delay in ms for TOK_DELAY
 */
struct tvp5146_reg {
	u8 token;
	u8 reg;
	u32 val;
};

/**
 * struct tvp5146_std_info - Structure to store standard informations
 * @width: Line width in pixels
 * @height:Number of active lines
 * @video_std: Value to write in REG_VIDEO_STD register
 * @standard: v4l2 standard structure information
 */
struct tvp5146_std_info {
	unsigned long width;
	unsigned long height;
	u8 video_std;
	struct v4l2_standard standard;
};

/**
 * struct tvp5146_ctrl_info - Information regarding supported controls
 * @reg_address: Register offset of control register
 * @query_ctrl: v4l2 query control information
 */
struct tvp5146_ctrl_info {
	u8 reg_address;
	struct v4l2_queryctrl query_ctrl;
};

/**
 * struct tvp5146_input_info - Information regarding supported inputs
 * @input_sel: Input select register
 * @lock_mask: lock mask - depends on Svideo/CVBS
 * @input: v4l2 input information
 */
struct tvp5146_input_info {
	u8 input_sel;
	u8 lock_mask;
	struct v4l2_input input;
};

/**
 * struct tvp5146_platform_data - Platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @ifparm: Interface parameters access function
 * @priv_data_set: Device private data (pointer) access function
 * @reg_list: The board dependent driver should fill the default value for
 *            required registers depending on board layout. The TVP5146
 *            driver will update this register list for the registers
 *            whose values should be maintained across open()/close() like
 *            setting brightness as defined in V4L2.
 *            The register list should be in the same order as defined in
 *            TVP5146 datasheet including reserved registers. As of now
 *            the driver expects the size of this list to be a minimum of
 *            57 + 1 (upto regsiter REG_CLEAR_LOST_LOCK).
 *            The last member should be of the list should be
 *            {TOK_TERM, 0, 0} to indicate the end of register list.
 * @num_inputs: Number of input connection in board
 * @input_list: Input information list for num_inputs
 */
struct tvp5146_platform_data {
	int (*power_set) (enum v4l2_power on);
	int (*ifparm) (struct v4l2_ifparm *p);
	int (*priv_data_set) (void *);

	struct tvp5146_reg *reg_list;

	int num_inputs;
	const struct tvp5146_input_info *input_list;
};

/**
 * struct tvp5146_decoded - TVP5146 decoder object
 * @v4l2_int_device: Slave handle
 * @pdata: Board specific
 * @client: I2C client data
 * @ver: Chip version
 * @state: TVP5146 decoder state - detected or not-detected
 * @pix: Current pixel format
 * @num_fmts: Number of formats
 * @fmt_list: Format list
 * @current_std: Current standard
 * @num_stds: Number of standards
 * @std_list: Standards list
 * @num_ctrls: Number of controls
 * @ctrl_list: Control list
 */
struct tvp5146_decoder {
	struct v4l2_int_device *v4l2_int_device;
	const struct tvp5146_platform_data *pdata;
	struct i2c_client *client;

	int ver;
	enum tvp5146_state state;

	struct v4l2_pix_format pix;
	int num_fmts;
	const struct v4l2_fmtdesc *fmt_list;

	enum tvp5146_std current_std;
	int num_stds;
	struct tvp5146_std_info *std_list;

	int num_ctrls;
	const struct tvp5146_ctrl_info *ctrl_list;

	int inputidx;
};

#endif				/* ifndef _TVP5146_H */
