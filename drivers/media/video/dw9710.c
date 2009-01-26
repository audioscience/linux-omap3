/*
 * drivers/media/video/dw9710.c
 *
 * DW9710 Coil Motor (LENS) driver
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Contributors:
 * 	Troy Laramy <t-laramy@ti.com>
 * 	Mohit Jalori <mjalori@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/device.h>

#include <mach/gpio.h>

#include <media/v4l2-int-device.h>
#include <media/dw9710.h>

#include "dw9710_priv.h"

static struct dw9710_device dw9710 = {
	.state = DW9710_LENS_NOT_DETECTED,
	.current_lens_posn = DW9710_DEF_LENS_POSN,
};

static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {
	{
		{
			.id = V4L2_CID_FOCUS_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Focus, Absolute",
			.minimum = 0,
			.maximum = DW9710_MAX_FOCUS_POS,
			.step = DW9710_LENS_POSN_STEP,
			.default_value = DW9710_DEF_LENS_POSN,
		},
		.current_value = DW9710_DEF_LENS_POSN,
	}
};

/**
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
static int find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--) {
		if (video_control[i].qc.id == id)
			return i;
	}

	return -EINVAL;
}

/**
 * dw9710_reg_read - Reads a value from a register in DW9710 Coil driver device.
 * @client: Pointer to structure of I2C client.
 * @value: Pointer to u16 for returning value of register to read.
 *
 * Returns zero if successful, or non-zero otherwise.
 **/
static int dw9710_reg_read(struct i2c_client *client, u16 *value)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = I2C_M_RD;
	msg->len = 2;
	msg->buf = data;

	data[0] = 0;
	data[1] = 0;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		err = ((data[0] & 0xFF) << 8) | (data[1]);
		*value = err;
		return 0;
	}
	return err;
}

/**
 * dw9710_reg_write - Writes a value to a register in DW9710 Coil driver device.
 * @client: Pointer to structure of I2C client.
 * @value: Value of register to write.
 *
 * Returns zero if successful, or non-zero otherwise.
 **/
static int dw9710_reg_write(struct i2c_client *client, u16 value)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

again:
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	data[0] = (u8)(value >> 8);
	data[1] = (u8)(value & 0xFF);

	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0)
		return 0;

	if (retry <= DW9710_I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retry ... %d", retry);
		retry++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto again;
	}
	return err;
}

/**
 * dw9710_detect - Detects DW9710 Coil driver device.
 * @client: Pointer to structure of I2C client.
 *
 * Returns 0 if successful, -1 if camera is off or if test register value
 * wasn't stored properly, or returned errors from either dw9710_reg_write or
 * dw9710_reg_read functions.
 **/
static int dw9710_detect(struct i2c_client *client)
{
	int err = 0;
	u16 wposn = 0, rposn = 0;
	u16 posn = 0x05;

	wposn = (DW9710_POWERDN(DW9710_ENABLE) | DW9710_DATA(posn));

	err = dw9710_reg_write(client, wposn);
	if (err) {
		dev_err(&client->dev, "Unable to write DW9710 \n");
		return err;
	}

	err = dw9710_reg_read(client, &rposn);
	if (err) {
		dev_err(&client->dev, "Unable to read DW9710 \n");
		return err;
	}

	if (wposn != rposn) {
		dev_err(&client->dev, "W/R MISMATCH!\n");
		return -1;
	}
	posn = 0;
	wposn = (DW9710_POWERDN(DW9710_ENABLE) | DW9710_DATA(posn));
	err = dw9710_reg_write(client, wposn);

	return err;
}

/**
 * dw9710_af_setfocus - Sets the desired focus.
 * @posn: Desired focus position, 0 (far) - 100 (close).
 *
 * Returns 0 on success, -EINVAL if camera is off or focus value is out of
 * bounds, or returned errors from either dw9710_reg_write or dw9710_reg_read
 * functions.
 **/
int dw9710_af_setfocus(u16 posn)
{
	struct dw9710_device *af_dev = &dw9710;
	struct i2c_client *client = af_dev->i2c_client;
	u16 cur_focus_value = 0;
	int ret = -EINVAL;

	if (posn > DW9710_MAX_FOCUS_POS) {
		dev_err(&client->dev, "Bad posn params 0x%x \n", posn);
		return ret;
	}

	if ((af_dev->power_state == V4L2_POWER_OFF) ||
		(af_dev->power_state == V4L2_POWER_STANDBY)) {
		af_dev->current_lens_posn = posn;
		return 0;
	}

	ret = dw9710_reg_read(client, &cur_focus_value);

	if (ret) {
		dev_err(&client->dev, "Read of current Lens position failed\n");
		return ret;
	}

	if (DW9710_DATA_R(cur_focus_value) == posn) {
		dev_dbg(&client->dev, "Device already in requested focal point\n");
		return ret;
	}

	ret = dw9710_reg_write(client, DW9710_POWERDN(DW9710_ENABLE) |
						DW9710_DATA(posn));

	if (ret)
		dev_err(&client->dev, "Setfocus register write failed\n");
	dw9710.current_lens_posn = posn;
	return ret;
}
EXPORT_SYMBOL(dw9710_af_setfocus);

/**
 * dw9710_af_getfocus - Gets the focus value from device.
 * @value: Pointer to u16 variable which will contain the focus value.
 *
 * Returns 0 if successful, -EINVAL if camera is off, or return value of
 * dw9710_reg_read if fails.
 **/
int dw9710_af_getfocus(u16 *value)
{
	struct dw9710_device *af_dev = &dw9710;
	struct i2c_client *client = af_dev->i2c_client;
	int ret;
	u16 posn = 0;

	if ((af_dev->power_state == V4L2_POWER_OFF) ||
	   (af_dev->power_state == V4L2_POWER_STANDBY))
		return -EINVAL;

	ret = dw9710_reg_read(client, &posn);

	if (ret) {
		dev_err(&client->dev, "Read of current Lens position failed\n");
		return ret;
	}
	*value = DW9710_DATA_R(posn);
	dw9710.current_lens_posn = DW9710_DATA_R(posn);
	return ret;
}
EXPORT_SYMBOL(dw9710_af_getfocus);

/**
 * ioctl_queryctrl - V4L2 lens interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
				struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = video_control[i].qc;
	return 0;
}

/**
 * ioctl_g_ctrl - V4L2 DW9710 lens interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	struct vcontrol *lvc;
	int i;
	u16 curr_posn;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case  V4L2_CID_FOCUS_ABSOLUTE:
		if (dw9710_af_getfocus(&curr_posn))
			return -EFAULT;
		vc->value = curr_posn;
		lvc->current_value = curr_posn;
		break;
	}

	return 0;
}

/**
 * ioctl_s_ctrl - V4L2 DW9710 lens interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	int retval = -EINVAL;
	int i;
	struct vcontrol *lvc;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case V4L2_CID_FOCUS_ABSOLUTE:
		retval = dw9710_af_setfocus(vc->value);
		if (!retval)
			lvc->current_value = vc->value;
		break;
	}

	return retval;
}

/**
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct dw9710_device *lens = s->priv;

	return lens->pdata->priv_data_set(p);

}

/**
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct dw9710_device *lens = s->priv;
	struct i2c_client *c = lens->i2c_client;
	int rval;

	rval = lens->pdata->power_set(on);
	if (rval < 0) {
		dev_err(&c->dev, "Unable to set the power state: " DW9710_NAME
								" lens HW\n");
		return rval;
	}

	if ((on == V4L2_POWER_ON) &&
				(lens->state == DW9710_LENS_NOT_DETECTED)) {
		rval = dw9710_detect(c);
		if (rval < 0) {
			dev_err(&c->dev, "Unable to detect "
				DW9710_NAME " lens HW\n");
			lens->state = DW9710_LENS_NOT_DETECTED;
			return rval;
		}
		lens->state = DW9710_LENS_DETECTED;
		pr_info(DW9710_NAME " lens HW detected\n");
	}

	if ((lens->power_state == V4L2_POWER_STANDBY) && (on == V4L2_POWER_ON)
				&& (lens->state == DW9710_LENS_DETECTED))
		dw9710_af_setfocus(lens->current_lens_posn);

	lens->power_state = on;
	return 0;
}

static struct v4l2_int_ioctl_desc dw9710_ioctl_desc[] = {
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_priv },
	{ .num = vidioc_int_queryctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_queryctrl },
	{ .num = vidioc_int_g_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ .num = vidioc_int_s_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_ctrl },
};

static struct v4l2_int_slave dw9710_slave = {
	.ioctls = dw9710_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(dw9710_ioctl_desc),
};

static struct v4l2_int_device dw9710_int_device = {
	.module = THIS_MODULE,
	.name = DW9710_NAME,
	.priv = &dw9710,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &dw9710_slave,
	},
};

/**
 * dw9710_probe - Probes the driver for valid I2C attachment.
 * @client: Pointer to structure of I2C client.
 *
 * Returns 0 if successful, or -EBUSY if unable to get client attached data.
 **/
static int
dw9710_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct dw9710_device *lens = &dw9710;
	int err;

	dev_info(&client->dev, "dw9710 probe called....\n");

	if (i2c_get_clientdata(client)) {
		dev_err(&client->dev, " DTA BUSY %s\n", client->name);
		return -EBUSY;
	}

	lens->pdata = client->dev.platform_data;

	if (!lens->pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	lens->v4l2_int_device = &dw9710_int_device;

	lens->i2c_client = client;
	i2c_set_clientdata(client, lens);

	err = v4l2_int_device_register(lens->v4l2_int_device);
	if (err) {
		dev_err(&client->dev, "Failed to Register "
			DW9710_NAME " as V4L2 device.\n");
		i2c_set_clientdata(client, NULL);
	} else {
		dev_err(&client->dev, "Registered "
			DW9710_NAME " as V4L2 device.\n");
	}

	return 0;
}

/**
 * dw9710_remove - Routine when device its unregistered from I2C
 * @client: Pointer to structure of I2C client.
 *
 * Returns 0 if successful, or -ENODEV if the client isn't attached.
 **/
static int __exit dw9710_remove(struct i2c_client *client)
{
	if (!client->adapter)
		return -ENODEV;

	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id dw9710_id[] = {
	{ DW9710_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, dw9710_id);

static struct i2c_driver dw9710_i2c_driver = {
	.driver = {
		.name = DW9710_NAME,
		.owner = THIS_MODULE,
	},
	.probe = dw9710_probe,
	.remove = __exit_p(dw9710_remove),
	.id_table = dw9710_id,
};

/**
 * dw9710_init - Module initialisation.
 *
 * Returns 0 if successful, or -EINVAL if device couldn't be initialized, or
 * added as a character device.
 **/
static int __init dw9710_init(void)
{
	int err = -EINVAL;

	err = i2c_add_driver(&dw9710_i2c_driver);
	if (err)
		goto fail;
	return err;
fail:
	printk(KERN_ERR "Failed to register " DW9710_NAME ".\n");
	return err;
}
module_init(dw9710_init);

/**
 * dw9710_cleanup - Module cleanup.
 **/
static void __exit dw9710_cleanup(void)
{
	i2c_del_driver(&dw9710_i2c_driver);
}
module_exit(dw9710_cleanup);

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DW9710 LENS driver");
