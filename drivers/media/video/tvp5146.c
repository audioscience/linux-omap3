/*
 * drivers/media/video/tvp5146.c
 *
 * TI TVP5146 decoder driver
 *
 * Copyright (C) 2008 Texas Instruments Inc
 *
 * Contributors:
 *     Brijesh R Jadav <brijesh.j@ti.com>
 *     Hardik Shah <hardik.shah@ti.com>
 *     Manjunath Hadli <mrh@ti.com>
 *     Sivaraj R <sivaraj@ti.com>
 *     Vaibhav Hiremath <hvaibhav@ti.com>
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

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <media/v4l2-int-device.h>
#include "tvp5146.h"

#define MODULE_NAME	TVP5146_MODULE_NAME

/* Debug functions */
#ifdef DEBUG

#define dump_reg(client, reg, val)					\
	do {								\
		tvp5146_read_reg(client, reg, &val);			\
		dev_dbg(&(client)->dev, "Reg(0x%.2X): 0x%.2X\n", reg, val); \
	} while (0)

#endif				/* #ifdef DEBUG */

/* list of image formats supported by tvp5146 decoder */
static const struct v4l2_fmtdesc tvp5146_fmt_list[] = {
	{
	 .index = 0,
	 .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	 .flags = 0,
	 .description = "8-bit UYVY 4:2:2 Format",
	 .pixelformat = V4L2_PIX_FMT_UYVY,
	 }
};

#define TVP5146_NUM_FORMATS		ARRAY_SIZE(tvp5146_fmt_list)

/*
 * Supported standards - These must be ordered according to enum tvp5146_std
 * order.
 */
static struct tvp5146_std_info tvp5146_std_list[] = {
	{
	 .width = NTSC_NUM_ACTIVE_PIXELS,
	 .height = NTSC_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_NTSC_MJ_BIT,
	 .standard = {
		      .index = 0,
		      .id = V4L2_STD_NTSC,
		      .name = "NTSC",
		      .frameperiod = {1001, 30000},
		      .framelines = 525}
	 },
	{
	 .width = PAL_NUM_ACTIVE_PIXELS,
	 .height = PAL_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_PAL_BDGHIN_BIT,
	 .standard = {
		      .index = 1,
		      .id = V4L2_STD_PAL,
		      .name = "PAL",
		      .frameperiod = {1, 25},
		      .framelines = 625}
	 }
};

#define TVP5146_NUM_STANDARDS		ARRAY_SIZE(tvp5146_std_list)

/* Supported controls */
static const struct tvp5146_ctrl_info tvp5146_ctrl_list[] = {
	{
	 .reg_address = REG_BRIGHTNESS,
	 .query_ctrl = {
			.id = V4L2_CID_BRIGHTNESS,
			.name = "BRIGHTNESS",
			.type = V4L2_CTRL_TYPE_INTEGER,
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = 128}
	 },
	{
	 .reg_address = REG_CONTRAST,
	 .query_ctrl = {
			.id = V4L2_CID_CONTRAST,
			.name = "CONTRAST",
			.type = V4L2_CTRL_TYPE_INTEGER,
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = 128}
	 },
	{
	 .reg_address = REG_SATURATION,
	 .query_ctrl = {
			.id = V4L2_CID_SATURATION,
			.name = "SATURATION",
			.type = V4L2_CTRL_TYPE_INTEGER,
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = 128}
	 },
	{
	 .reg_address = REG_HUE,
	 .query_ctrl = {
			.id = V4L2_CID_HUE,
			.name = "HUE",
			.type = V4L2_CTRL_TYPE_INTEGER,
			.minimum = -180,
			.maximum = 180,
			.step = 180,
			.default_value = 0}
	 },
	{
	 .reg_address = REG_AFE_GAIN_CTRL,
	 .query_ctrl = {
			.id = V4L2_CID_AUTOGAIN,
			.name = "Automatic Gain Control",
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 1}
	 }
};

#define TVP5146_NUM_CONTROLS		ARRAY_SIZE(tvp5146_ctrl_list)

/*
 * Read a value from a register in an tvp5146 decoder device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int tvp5146_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	u8 data;

	if (!client->adapter)
		return -ENODEV;

	/* [MSG1] fill the register address data */
	data = reg;
	msg[0].addr = client->addr;
	msg[0].len = 1;
	msg[0].flags = 0;
	msg[0].buf = &data;

	/* [MSG2] fill the data rx buffer */
	msg[1].addr = client->addr;
	msg[1].len = 1;		/* only 1 byte */
	msg[1].flags = I2C_M_RD;	/* Read the register values */
	msg[1].buf = val;
	err = i2c_transfer(client->adapter, msg, 2);
	if (err >= 0)
		return 0;

	dev_err(&client->dev,
		"read from device 0x%.2x, offset 0x%.2x error %d\n",
		client->addr, reg, err);

	return err;
}

/*
 * Write a value to a register in an tvp5146 decoder device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int tvp5146_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int err;
	int retry = 0;
	struct i2c_msg msg[1];
	u8 data[2];

	if (!client->adapter)
		return -ENODEV;

again:
	data[0] = reg;		/* Register offset */
	data[1] = val;		/* Register value */
	msg->addr = client->addr;
	msg->len = 2;
	msg->flags = 0;		/* write operation */
	msg->buf = data;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;

	dev_err(&client->dev,
		"wrote 0x%.2x to offset 0x%.2x error %d\n", val, reg, err);
	if (retry <= I2C_RETRY_COUNT) {
		dev_info(&client->dev, "retry ... %d\n", retry);
		retry++;
		schedule_timeout(msecs_to_jiffies(20));
		goto again;
	}
	return err;
}

/*
 * tvp5146_write_regs : Initializes a list of TVP5146 registers
 *		if token is TOK_TERM, then entire write operation terminates
 *		if token is TOK_DELAY, then a delay of 'val' msec is introduced
 *		if token is TOK_SKIP, then the register write is skipped
 *		if token is TOK_WRITE, then the register write is performed
 *
 * reglist - list of registers to be written
 * Returns zero if successful, or non-zero otherwise.
 */
static int tvp5146_write_regs(struct i2c_client *client,
			      const struct tvp5146_reg reglist[])
{
	int err;
	const struct tvp5146_reg *next = reglist;

	for (; next->token != TOK_TERM; next++) {
		if (next->token == TOK_DELAY) {
			schedule_timeout(msecs_to_jiffies(next->val));
			continue;
		}

		if (next->token == TOK_SKIP)
			continue;

		err = tvp5146_write_reg(client, next->reg, (u8) next->val);
		if (err) {
			dev_err(&client->dev, "write failed. Err[%d]\n",
				err);
			return err;
		}
	}
	return 0;
}

/*
 * tvp5146_get_current_std:
 * Returns the current standard detected by TVP5146
 */
static enum tvp5146_std tvp5146_get_current_std(struct tvp5146_decoder
						*decoder)
{
	u8 std, std_status;

	if (tvp5146_read_reg(decoder->client, REG_VIDEO_STD, &std))
		return STD_INVALID;

	if ((std & VIDEO_STD_MASK) == VIDEO_STD_AUTO_SWITCH_BIT) {
		/* use the standard status register */
		if (tvp5146_read_reg(decoder->client, REG_VIDEO_STD_STATUS,
				     &std_status))
			return STD_INVALID;
	} else
		std_status = std;	/* use the standard register itself */

	switch (std_status & VIDEO_STD_MASK) {
	case VIDEO_STD_NTSC_MJ_BIT:
		return STD_NTSC_MJ;
		break;

	case VIDEO_STD_PAL_BDGHIN_BIT:
		return STD_PAL_BDGHIN;
		break;

	default:
		return STD_INVALID;
		break;
	}

	return STD_INVALID;
}

#ifdef DEBUG
/*
 * TVP5146 register dump function
 */
void tvp5146_reg_dump(struct tvp5146_decoder *decoder)
{
	u8 value;

	dump_reg(decoder->client, REG_INPUT_SEL, value);
	dump_reg(decoder->client, REG_AFE_GAIN_CTRL, value);
	dump_reg(decoder->client, REG_VIDEO_STD, value);
	dump_reg(decoder->client, REG_OPERATION_MODE, value);
	dump_reg(decoder->client, REG_COLOR_KILLER, value);
	dump_reg(decoder->client, REG_LUMA_CONTROL1, value);
	dump_reg(decoder->client, REG_LUMA_CONTROL2, value);
	dump_reg(decoder->client, REG_LUMA_CONTROL3, value);
	dump_reg(decoder->client, REG_BRIGHTNESS, value);
	dump_reg(decoder->client, REG_CONTRAST, value);
	dump_reg(decoder->client, REG_SATURATION, value);
	dump_reg(decoder->client, REG_HUE, value);
	dump_reg(decoder->client, REG_CHROMA_CONTROL1, value);
	dump_reg(decoder->client, REG_CHROMA_CONTROL2, value);
	dump_reg(decoder->client, REG_COMP_PR_SATURATION, value);
	dump_reg(decoder->client, REG_COMP_Y_CONTRAST, value);
	dump_reg(decoder->client, REG_COMP_PB_SATURATION, value);
	dump_reg(decoder->client, REG_COMP_Y_BRIGHTNESS, value);
	dump_reg(decoder->client, REG_AVID_START_PIXEL_LSB, value);
	dump_reg(decoder->client, REG_AVID_START_PIXEL_MSB, value);
	dump_reg(decoder->client, REG_AVID_STOP_PIXEL_LSB, value);
	dump_reg(decoder->client, REG_AVID_STOP_PIXEL_MSB, value);
	dump_reg(decoder->client, REG_HSYNC_START_PIXEL_LSB, value);
	dump_reg(decoder->client, REG_HSYNC_START_PIXEL_MSB, value);
	dump_reg(decoder->client, REG_HSYNC_STOP_PIXEL_LSB, value);
	dump_reg(decoder->client, REG_HSYNC_STOP_PIXEL_MSB, value);
	dump_reg(decoder->client, REG_VSYNC_START_LINE_LSB, value);
	dump_reg(decoder->client, REG_VSYNC_START_LINE_MSB, value);
	dump_reg(decoder->client, REG_VSYNC_STOP_LINE_LSB, value);
	dump_reg(decoder->client, REG_VSYNC_STOP_LINE_MSB, value);
	dump_reg(decoder->client, REG_VBLK_START_LINE_LSB, value);
	dump_reg(decoder->client, REG_VBLK_START_LINE_MSB, value);
	dump_reg(decoder->client, REG_VBLK_STOP_LINE_LSB, value);
	dump_reg(decoder->client, REG_VBLK_STOP_LINE_MSB, value);
	dump_reg(decoder->client, REG_SYNC_CONTROL, value);
	dump_reg(decoder->client, REG_OUTPUT_FORMATTER1, value);
	dump_reg(decoder->client, REG_OUTPUT_FORMATTER2, value);
	dump_reg(decoder->client, REG_OUTPUT_FORMATTER3, value);
	dump_reg(decoder->client, REG_OUTPUT_FORMATTER4, value);
	dump_reg(decoder->client, REG_OUTPUT_FORMATTER5, value);
	dump_reg(decoder->client, REG_OUTPUT_FORMATTER6, value);
	dump_reg(decoder->client, REG_CLEAR_LOST_LOCK, value);
}
#endif

/*
 * Configure the tvp5146 with the current register settings
 * Returns zero if successful, or non-zero otherwise.
 */
static int tvp5146_configure(struct tvp5146_decoder *decoder)
{
	int err;

	/* common register initialization */
	err =
	    tvp5146_write_regs(decoder->client, decoder->pdata->reg_list);
	if (err)
		return err;

#ifdef DEBUG
	tvp5146_reg_dump(decoder);
#endif

	return 0;
}

/*
 * Detect if an tvp5146 is present, and if so which revision.
 * A device is considered to be detected if the chip ID (LSB and MSB)
 * registers match the expected values.
 * Any value of the rom version register is accepted.
 * Returns ENODEV error number if no device is detected, or zero
 * if a device is detected.
 */
static int tvp5146_detect(struct tvp5146_decoder *decoder)
{
	u8 chip_id_msb, chip_id_lsb, rom_ver;

	if (tvp5146_read_reg
	    (decoder->client, REG_CHIP_ID_MSB, &chip_id_msb))
		return -ENODEV;
	if (tvp5146_read_reg
	    (decoder->client, REG_CHIP_ID_LSB, &chip_id_lsb))
		return -ENODEV;
	if (tvp5146_read_reg(decoder->client, REG_ROM_VERSION, &rom_ver))
		return -ENODEV;

	dev_info(&decoder->client->dev,
		 "chip id detected msb:0x%x lsb:0x%x rom version:0x%x\n",
		 chip_id_msb, chip_id_lsb, rom_ver);
	if ((chip_id_msb != TVP5146_CHIP_ID_MSB)
		|| (chip_id_lsb != TVP5146_CHIP_ID_LSB)) {
		/* We didn't read the values we expected, so this must not be
		 * an TVP5146.
		 */
		dev_err(&decoder->client->dev,
			"chip id mismatch msb:0x%x lsb:0x%x\n",
			chip_id_msb, chip_id_lsb);
		return -ENODEV;
	}

	decoder->ver = rom_ver;
	decoder->state = STATE_DETECTED;

	return 0;
}

/*
 * following are decoder interface functions implemented by
 * tvp5146 decoder driver.
 */

/**
 * ioctl_querystd - V4L2 decoder interface handler for VIDIOC_QUERYSTD ioctl
 * @s: pointer to standard V4L2 device structure
 * @std_id: standard V4L2 std_id ioctl enum
 *
 * Returns the current standard detected by TVP5146. If no active input is
 * detected, returns -EINVAL
 */
static int ioctl_querystd(struct v4l2_int_device *s, v4l2_std_id *std_id)
{
	struct tvp5146_decoder *decoder = s->priv;
	enum tvp5146_std current_std;
	u8 sync_lock_status, lock_mask;

	if (std_id == NULL)
		return -EINVAL;

	/* get the current standard */
	current_std = tvp5146_get_current_std(decoder);
	if (current_std == STD_INVALID)
		return -EINVAL;

	/* check whether signal is locked */
	if (tvp5146_read_reg
	    (decoder->client, REG_STATUS1, &sync_lock_status))
		return -EINVAL;

	lock_mask =
	    decoder->pdata->input_list[decoder->inputidx].lock_mask;
	if (lock_mask != (sync_lock_status & lock_mask))
		return -EINVAL;	/* No input detected */

	decoder->current_std = current_std;
	*std_id = decoder->std_list[current_std].standard.id;

	return 0;
}

/**
 * ioctl_s_std - V4L2 decoder interface handler for VIDIOC_S_STD ioctl
 * @s: pointer to standard V4L2 device structure
 * @std_id: standard V4L2 v4l2_std_id ioctl enum
 *
 * If std_id is supported, sets the requested standard. Otherwise, returns
 * -EINVAL
 */
static int ioctl_s_std(struct v4l2_int_device *s, v4l2_std_id *std_id)
{
	struct tvp5146_decoder *decoder = s->priv;
	int err, i;

	if (std_id == NULL)
		return -EINVAL;

	for (i = 0; i < decoder->num_stds; i++)
		if (*std_id & decoder->std_list[i].standard.id)
			break;

	if (i == decoder->num_stds)
		return -EINVAL;

	err = tvp5146_write_reg(decoder->client, REG_VIDEO_STD,
				decoder->std_list[i].video_std);
	if (err)
		return err;

	decoder->current_std = i;
	decoder->pdata->reg_list[REG_VIDEO_STD].val =
	    decoder->std_list[i].video_std;

	return 0;
}

/**
 * ioctl_enum_input - V4L2 decoder interface handler for VIDIOC_ENUMINPUT ioctl
 * @s: pointer to standard V4L2 device structure
 * @input: standard V4L2 VIDIOC_ENUMINPUT ioctl structure
 *
 * If index is valid, returns the description of the input. Otherwise, returns
 * -EINVAL if any error occurs
 */
static int
ioctl_enum_input(struct v4l2_int_device *s, struct v4l2_input *input)
{
	struct tvp5146_decoder *decoder = s->priv;
	int index;

	if (input == NULL)
		return -EINVAL;

	index = input->index;
	if ((index >= decoder->pdata->num_inputs) || (index < 0))
		return -EINVAL;	/* Index out of bound */

	memcpy(input, &decoder->pdata->input_list[index].input,
		sizeof(struct v4l2_input));

	return 0;
}

/**
 * ioctl_s_input - V4L2 decoder interface handler for VIDIOC_S_INPUT ioctl
 * @s: pointer to standard V4L2 device structure
 * @index: number of the input
 *
 * If index is valid, selects the requested input. Otherwise, returns -EINVAL if
 * the input is not supported or there is no active signal present in the
 * selected input.
 */
static int ioctl_s_input(struct v4l2_int_device *s, int index)
{
	struct tvp5146_decoder *decoder = s->priv;
	u8 input_sel;
	int err;
	enum tvp5146_std current_std = STD_INVALID;
	u8 sync_lock_status, lock_mask;
	int try_count = LOCK_RETRY_COUNT;

	if ((index >= decoder->pdata->num_inputs) || (index < 0))
		return -EINVAL;	/* Index out of bound */

	/* Get the register value to be written to select the requested input */
	input_sel = decoder->pdata->input_list[index].input_sel;
	err = tvp5146_write_reg(decoder->client, REG_INPUT_SEL, input_sel);
	if (err)
		return err;

	decoder->inputidx = index;
	decoder->pdata->reg_list[REG_INPUT_SEL].val = input_sel;

	/* Clear status */
	msleep(LOCK_RETRY_DELAY);
	err =
	    tvp5146_write_reg(decoder->client, REG_CLEAR_LOST_LOCK, 0x01);
	if (err)
		return err;

	while (try_count-- > 0) {
		/* Allow decoder to sync up with new input */
		msleep(LOCK_RETRY_DELAY);

		/* get the current standard for future reference */
		current_std = tvp5146_get_current_std(decoder);
		if (current_std == STD_INVALID)
			continue;

		if (tvp5146_read_reg(decoder->client, REG_STATUS1,
					&sync_lock_status))
			return -EINVAL;

		lock_mask =
		    decoder->pdata->input_list[decoder->inputidx].
		    lock_mask;
		if (lock_mask == (sync_lock_status & lock_mask))
			break;	/* Input detected */
	}

	if ((current_std == STD_INVALID) || (try_count < 0))
		return -EINVAL;

	decoder->current_std = current_std;

	return 0;
}

/**
 * ioctl_g_input - V4L2 decoder interface handler for VIDIOC_G_INPUT ioctl
 * @s: pointer to standard V4L2 device structure
 * @index: returns the current selected input
 *
 * Returns the current selected input. Returns -EINVAL if any error occurs
 */
static int ioctl_g_input(struct v4l2_int_device *s, int *index)
{
	struct tvp5146_decoder *decoder = s->priv;
	int err = -EINVAL, i, inputidx;

	if (index == NULL)
		return -EINVAL;

	/* Search through the input list for active inputs */
	inputidx = decoder->inputidx;
	for (i = 0; i < decoder->pdata->num_inputs; i++) {
		inputidx++;	/* Move to next input */
		if (inputidx >= decoder->pdata->num_inputs)
			inputidx = 0;	/* fall back to first input */

		err = ioctl_s_input(s, inputidx);
		if (!err) {
			/* Active input found - select it and return success */
			*index = inputidx;
			return 0;
		}
	}

	return err;
}

/**
 * ioctl_queryctrl - V4L2 decoder interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the ctrl_list[] array. Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int
ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qctrl)
{
	struct tvp5146_decoder *decoder = s->priv;
	int id, index;
	const struct tvp5146_ctrl_info *control = NULL;

	if (qctrl == NULL)
		return -EINVAL;

	id = qctrl->id;
	memset(qctrl, 0, sizeof(struct v4l2_queryctrl));
	qctrl->id = id;

	for (index = 0; index < decoder->num_ctrls; index++) {
		control = &decoder->ctrl_list[index];
		if (control->query_ctrl.id == qctrl->id)
			break;	/* Match found */
	}
	if (index == decoder->num_ctrls)
		return -EINVAL;	/* Index out of bound */

	memcpy(qctrl, &control->query_ctrl, sizeof(struct v4l2_queryctrl));

	return 0;
}

/**
 * ioctl_g_ctrl - V4L2 decoder interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the decoder. Otherwise, returns -EINVAL if the control is not
 * supported.
 */
static int
ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *ctrl)
{
	struct tvp5146_decoder *decoder = s->priv;
	int err, index;
	u8 val;
	int value;
	const struct tvp5146_ctrl_info *control = NULL;

	if (ctrl == NULL)
		return -EINVAL;

	for (index = 0; index < decoder->num_ctrls; index++) {
		control = &decoder->ctrl_list[index];
		if (control->query_ctrl.id == ctrl->id)
			break;	/* Match found */
	}
	if (index == decoder->num_ctrls)
		return -EINVAL;	/* Index out of bound */

	err =
	    tvp5146_read_reg(decoder->client, control->reg_address, &val);
	if (err < 0)
		return err;

	/* cross check */
	if (val != decoder->pdata->reg_list[control->reg_address].val)
		return -EINVAL;	/* Driver & TVP5146 setting mismatch */

	value = val;
	if (V4L2_CID_AUTOGAIN == ctrl->id) {
		if ((value & 0x3) == 3)
			value = 1;
		else
			value = 0;
	}

	if (V4L2_CID_HUE == ctrl->id) {
		if (value == 0x7F)
			value = 180;
		else if (value == 0x80)
			value = -180;
		else
			value = 0;
	}

	ctrl->value = value;

	return err;
}

/**
 * ioctl_s_ctrl - V4L2 decoder interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW. Otherwise, returns -EINVAL if the control is not supported.
 */
static int
ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *ctrl)
{
	struct tvp5146_decoder *decoder = s->priv;
	int err, value, index;
	const struct tvp5146_ctrl_info *control = NULL;

	if (ctrl == NULL)
		return -EINVAL;

	value = (__s32) ctrl->value;
	for (index = 0; index < decoder->num_ctrls; index++) {
		control = &decoder->ctrl_list[index];
		if (control->query_ctrl.id == ctrl->id)
			break;	/* Match found */
	}
	if (index == decoder->num_ctrls)
		return -EINVAL;	/* Index out of bound */

	if (V4L2_CID_AUTOGAIN == ctrl->id) {
		if (value == 1)
			value = 0x0F;
		else if (value == 0)
			value = 0x0C;
		else
			return -ERANGE;
	} else if (V4L2_CID_HUE == ctrl->id) {
		if (value == 180)
			value = 0x7F;
		else if (value == -180)
			value = 0x80;
		else if (value == 0)
			value = 0;
		else
			return -ERANGE;
	} else {
		if ((value < control->query_ctrl.minimum)
			|| (value > control->query_ctrl.maximum))
			return -ERANGE;
	}

	err =
	    tvp5146_write_reg(decoder->client, control->reg_address,
				value);
	if (err < 0)
		return err;

	decoder->pdata->reg_list[control->reg_address].val = value;
	return err;
}

/**
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl to enumerate supported formats
 */
static int
ioctl_enum_fmt_cap(struct v4l2_int_device *s, struct v4l2_fmtdesc *fmt)
{
	struct tvp5146_decoder *decoder = s->priv;
	int index;

	if (fmt == NULL)
		return -EINVAL;

	index = fmt->index;
	if ((index >= decoder->num_fmts) || (index < 0))
		return -EINVAL;	/* Index out of bound */

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	memcpy(fmt, &decoder->fmt_list[index],
		sizeof(struct v4l2_fmtdesc));

	return 0;
}

/**
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type. This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int
ioctl_try_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct tvp5146_decoder *decoder = s->priv;
	int ifmt;
	struct v4l2_pix_format *pix;
	enum tvp5146_std current_std;

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	pix = &f->fmt.pix;

	/* Calculate height and width based on current standard */
	current_std = tvp5146_get_current_std(decoder);
	if (current_std == STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;
	pix->width = decoder->std_list[current_std].width;
	pix->height = decoder->std_list[current_std].height;

	for (ifmt = 0; ifmt < decoder->num_fmts; ifmt++) {
		if (pix->pixelformat ==
			decoder->fmt_list[ifmt].pixelformat)
			break;
	}
	if (ifmt == decoder->num_fmts)
		ifmt = 0;	/* None of the format matched, select default */
	pix->pixelformat = decoder->fmt_list[ifmt].pixelformat;

	pix->field = V4L2_FIELD_INTERLACED;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
	pix->priv = 0;

	return 0;
}

/**
 * ioctl_s_fmt_cap - V4L2 decoder interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int
ioctl_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct tvp5146_decoder *decoder = s->priv;
	struct v4l2_pix_format *pix;
	int rval;

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	pix = &f->fmt.pix;
	rval = ioctl_try_fmt_cap(s, f);
	if (rval)
		return rval;
	else
		decoder->pix = *pix;

	return rval;
}

/**
 * ioctl_g_fmt_cap - V4L2 decoder interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the decoder's current pixel format in the v4l2_format
 * parameter.
 */
static int
ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct tvp5146_decoder *decoder = s->priv;

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	f->fmt.pix = decoder->pix;

	return 0;
}

/**
 * ioctl_g_parm - V4L2 decoder interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the decoder's video CAPTURE parameters.
 */
static int
ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct tvp5146_decoder *decoder = s->priv;
	struct v4l2_captureparm *cparm;
	enum tvp5146_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	/* get the current standard */
	current_std = tvp5146_get_current_std(decoder);
	decoder->current_std = current_std;

	cparm = &a->parm.capture;
	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe
	    = decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

/**
 * ioctl_s_parm - V4L2 decoder interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the decoder to use the input parameters, if possible. If
 * not possible, returns the appropriate error code.
 */
static int
ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct tvp5146_decoder *decoder = s->priv;
	struct v4l2_fract *timeperframe;
	enum tvp5146_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	timeperframe = &a->parm.capture.timeperframe;

	/* get the current standard */
	current_std = tvp5146_get_current_std(decoder);
	decoder->current_std = current_std;

	*timeperframe =
	    decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

/**
 * ioctl_g_ifparm - V4L2 decoder interface handler for vidioc_int_g_ifparm_num
 * @s: pointer to standard V4L2 device structure
 * @p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p. This value is returned in the p
 * parameter.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct tvp5146_decoder *decoder = s->priv;
	int rval;

	if (p == NULL)
		return -EINVAL;

	rval = decoder->pdata->ifparm(p);
	if (rval) {
		dev_err(&decoder->client->dev, "error. Err[%d]\n", rval);
		return rval;
	}

	p->u.bt656.clock_curr = TVP5146_XCLK_BT656;

	return 0;
}

/**
 * ioctl_g_priv - V4L2 decoder interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold decoder's private data address
 *
 * Returns device's (decoder's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct tvp5146_decoder *decoder = s->priv;

	return decoder->pdata->priv_data_set(p);
}

/**
 * ioctl_s_power - V4L2 decoder interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct tvp5146_decoder *decoder = s->priv;
	int err = 0;

	switch (on) {
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		err =
		    tvp5146_write_reg(decoder->client, REG_OPERATION_MODE,
					0x01);
		/* Disable mux for TVP5146 decoder data path */
		err |= decoder->pdata->power_set(on);
		break;

	case V4L2_POWER_STANDBY:
		err = decoder->pdata->power_set(on);
		break;

	case V4L2_POWER_ON:
		/* Enable mux for TVP5146 decoder data path */
		err = decoder->pdata->power_set(on);

		/* Power Up Sequence */
		err |=
		    tvp5146_write_reg(decoder->client, REG_OPERATION_MODE,
					0x01);
		err |=
		    tvp5146_write_reg(decoder->client, REG_OPERATION_MODE,
					0x00);

		/* Detect the sensor is not already detected */
		if (decoder->state == STATE_NOT_DETECTED) {
			err |= tvp5146_detect(decoder);
			if (err < 0) {
				dev_err(&decoder->client->dev,
					"Unable to detect decoder\n");
				return err;
			}
			dev_info(&decoder->client->dev,
				 "chip version 0x%.2x detected\n",
				 decoder->ver);
		}
		break;

	case V4L2_POWER_RESUME:
		err = decoder->pdata->power_set(on);
		if (decoder->state == STATE_DETECTED)
			err |= tvp5146_configure(decoder);
		break;

	default:
		return -ENODEV;
		break;
	}

	return err;
}

/**
 * ioctl_init - V4L2 decoder interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the decoder device (calls tvp5146_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	struct tvp5146_decoder *decoder = s->priv;

	/* Set default standard to auto */
	decoder->pdata->reg_list[REG_VIDEO_STD].val =
	    VIDEO_STD_AUTO_SWITCH_BIT;

	return tvp5146_configure(decoder);
}

/**
 * ioctl_dev_exit - V4L2 decoder interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the dev. at slave detach. The complement of ioctl_dev_init.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_init - V4L2 decoder interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master. Returns 0 if
 * tvp5146 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct tvp5146_decoder *decoder = s->priv;
	int err;

	err = tvp5146_detect(decoder);
	if (err < 0) {
		dev_err(&decoder->client->dev,
			"Unable to detect decoder\n");
		return err;
	}

	dev_info(&decoder->client->dev,
		 "chip version 0x%.2x detected\n", decoder->ver);

	return 0;
}

static struct v4l2_int_ioctl_desc tvp5146_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ioctl_dev_init},
	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func*) ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ioctl_s_power},
	{vidioc_int_g_priv_num, (v4l2_int_ioctl_func*) ioctl_g_priv},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
	{vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
	{vidioc_int_try_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_try_fmt_cap},
	{vidioc_int_g_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
	{vidioc_int_s_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_s_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
	{vidioc_int_queryctrl_num,
	 (v4l2_int_ioctl_func *) ioctl_queryctrl},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_querystd_num, (v4l2_int_ioctl_func *) ioctl_querystd},
	{vidioc_int_s_std_num, (v4l2_int_ioctl_func *) ioctl_s_std},
	{vidioc_int_enum_input_num,
	 (v4l2_int_ioctl_func *) ioctl_enum_input},
	{vidioc_int_g_input_num, (v4l2_int_ioctl_func *) ioctl_g_input},
	{vidioc_int_s_input_num, (v4l2_int_ioctl_func *) ioctl_s_input},
};

static struct v4l2_int_slave tvp5146_slave = {
	.ioctls = tvp5146_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(tvp5146_ioctl_desc),
};

static struct tvp5146_decoder tvp5146_dev = {
	.state = STATE_NOT_DETECTED,

	.num_fmts = TVP5146_NUM_FORMATS,
	.fmt_list = tvp5146_fmt_list,

	.pix = {		/* Default to NTSC 8-bit YUV 422 */
		.width = NTSC_NUM_ACTIVE_PIXELS,
		.height = NTSC_NUM_ACTIVE_LINES,
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.field = V4L2_FIELD_INTERLACED,
		.bytesperline = NTSC_NUM_ACTIVE_PIXELS * 2,
		.sizeimage =
		NTSC_NUM_ACTIVE_PIXELS * 2 * NTSC_NUM_ACTIVE_LINES,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		},

	.current_std = STD_NTSC_MJ,
	.num_stds = TVP5146_NUM_STANDARDS,
	.std_list = tvp5146_std_list,

	.num_ctrls = TVP5146_NUM_CONTROLS,
	.ctrl_list = tvp5146_ctrl_list,

	.inputidx = 0,		/* Composite selected */
};

static struct v4l2_int_device tvp5146_int_device = {
	.module = THIS_MODULE,
	.name = MODULE_NAME,
	.priv = &tvp5146_dev,
	.type = v4l2_int_type_slave,
	.u = {
	      .slave = &tvp5146_slave,
	      },
};

/**
 * tvp5146_probe - decoder driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register decoder as an i2c client device and V4L2
 * device.
 */
static int
tvp5146_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tvp5146_decoder *decoder = &tvp5146_dev;
	int err;

	if (i2c_get_clientdata(client))
		return -EBUSY;

	decoder->pdata = client->dev.platform_data;
	if (!decoder->pdata) {
		dev_err(&client->dev, "No platform data\n!!");
		return -ENODEV;
	}

	decoder->v4l2_int_device = &tvp5146_int_device;
	decoder->client = client;
	i2c_set_clientdata(client, decoder);

	/* Register with V4L2 layer as slave device */
	err = v4l2_int_device_register(decoder->v4l2_int_device);
	if (err) {
		i2c_set_clientdata(client, NULL);
		dev_err(&client->dev,
			"Unable to register to v4l2. Err[%d]\n", err);

	} else
		dev_info(&client->dev, "Registered to v4l2 done!!\n");

	return 0;
}

/**
 * tvp5146_remove - decoder driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister decoder as an i2c client device and V4L2
 * device. Complement of tvp5146_probe().
 */
static int __exit tvp5146_remove(struct i2c_client *client)
{
	struct tvp5146_decoder *decoder = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(decoder->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id tvp5146_id[] = {
	{MODULE_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tvp5146_id);

static struct i2c_driver tvp5146_i2c_driver = {
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = tvp5146_probe,
	.remove = __exit_p(tvp5146_remove),
	.id_table = tvp5146_id,
};

/**
 * tvp5146_init
 *
 * Module init function
 */
static int __init tvp5146_init(void)
{
	int err;

	err = i2c_add_driver(&tvp5146_i2c_driver);
	if (err) {
		printk(KERN_ERR "Failed to register " MODULE_NAME ".\n");
		return err;
	}
	return 0;
}

/**
 * tvp5146_cleanup
 *
 * Module exit function
 */
static void __exit tvp5146_cleanup(void)
{
	i2c_del_driver(&tvp5146_i2c_driver);
}

late_initcall(tvp5146_init);
module_exit(tvp5146_cleanup);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("TVP5146 linux decoder driver");
MODULE_LICENSE("GPL");
