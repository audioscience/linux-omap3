/*
 * drivers/media/video/omap34xxcam.c
 *
 * Video-for-Linux (Version 2) Camera capture driver for OMAP34xx ISP.
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/pci.h>		/* needed for videobufs */
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/version.h>
#include <linux/platform_device.h>

#include <media/v4l2-common.h>

#include "omap34xxcam.h"
#include "isp/isp.h"
#include "isp/ispmmu.h"
#include "isp/ispreg.h"
#include "isp/ispccdc.h"
#include "isp/isph3a.h"
#include "isp/isphist.h"
#include "isp/isppreview.h"
#include "isp/ispresizer.h"

#define OMAP34XXCAM_VERSION KERNEL_VERSION(0, 0, 0)

/* global variables */
static struct v4l2_int_device omap34xxcam;

/* module parameters */
static int video_nr = -1;	/* video device minor (-1 ==> auto assign) */
static int capture_mem = 2592 * 1944 * 2 * 2;
static int omap34xxcam_device_register(struct v4l2_int_device *s);
static void omap34xxcam_device_unregister(struct v4l2_int_device *s);
static int omap34xxcam_remove(struct platform_device *pdev);
struct omap34xxcam_fh *camfh_saved;

/*
 * V4L2 handling
 *
 */

/* Our own specific controls */
V4L2_INT_WRAPPER_1(priv_start, int, *);

/*
 *
 * OMAP3 ISP Handling
 *
 */

/**
 * omap34xxcamisp_configure_interface - configures the OMAP ISP interface
 * @p: standard V4L2 interface parameters structure
 * @cam: per-device camera information data structure
 *
 * Configures the ISP interface based on the type of sensor being used
 * (raw or smart/soc).
 */
static int
omap34xxcamisp_configure_interface(struct v4l2_ifparm *p,
				   struct omap34xxcam_device *cam)
{
	struct isp_interface_config config;
	struct omap34xxcam_sensor *sens = &cam->sens;

	/* We still dont support serial interface */
	if (p->if_type == V4L2_IF_TYPE_BT656)
		config.ccdc_par_ser = 0;
	else
		return -EINVAL;

	switch (sens->sensor_isp) {
	case V4L2_IF_CAP_RAW:
		/* Configured for RAW Bayer 10 default */
		if (p->u.bt656.mode == V4L2_IF_TYPE_BT656_MODE_NOBT_10BIT)
			config.dataline_shift = 1;	/*Choose 10 bits */

		if (p->u.bt656.frame_start_on_rising_vs)
			config.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE;
		else
			config.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSFALL;

		config.para_clk_pol = p->u.bt656.latch_clk_inv;
		config.par_bridge = 0;

		break;

	case V4L2_IF_CAP_SOC:
		/* Configured for YUYV/UYVY default */

		config.dataline_shift = 2;

		if (p->u.bt656.frame_start_on_rising_vs)
			config.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE;
		else
			config.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSFALL;

		config.para_clk_pol = p->u.bt656.latch_clk_inv;
		config.par_bridge = 3;

		/* Need to reconfigure in set format for any updates */

		break;

	default:
		dev_err(cam->dev, "SOC type %d not supported\n", p->if_type);
		return -EINVAL;
	}

	config.shutter = 0;
	config.prestrobe = 0;
	config.strobe = 0;
	config.vdint0_timing = 0;
	config.vdint1_timing = 0;
	isp_configure_interface(&config);

	return 0;
}

/*
 *
 * Sensor handling.
 *
 */

/**
 * omap34xxcam_sensor_if_enable - enable/configure the ext. sensor interface
 * @cam: ptr. to per-device camera information data structure
 *
 * Enable the external sensor interface. Try to negotiate interface
 * parameters with the sensor and start using the new ones. The calls
 * to sensor_if_enable and sensor_if_disable need not to be balanced.
 */
static int omap34xxcam_sensor_if_enable(struct omap34xxcam_device *cam)
{
	struct v4l2_ifparm p;
	struct omap34xxcam_hw_config hwc;
	int rval;

	rval = vidioc_int_g_ifparm(cam->sdev, &p);
	if (rval) {
		dev_err(cam->dev, "vidioc_int_g_ifparm failed with %d\n", rval);
		return rval;
	}

	/* Get XCLK Type */
	rval = vidioc_int_g_priv(cam->sdev, (void *)&hwc);
	if (rval) {
		dev_err(cam->dev, "vidioc_int_g_priv failed with %d\n", rval);
		return rval;
	}
	cam->sens.sensor_isp = hwc.sensor_isp;
	cam->sens.xclk = hwc.xclk;

	if (hwc.sensor_isp)
		dev_dbg(cam->dev, "SOC Sensor connected\n");
	else
		dev_dbg(cam->dev, "RAW Sensor connected\n");

	/* Enable the ISP clock. This provides clock to the sensor */
	isp_get();

	/* Enable the ISP interface on OMAP3 */
	rval = omap34xxcamisp_configure_interface(&p, cam);
	if (rval) {
		dev_err(cam->dev, "ISP interface setup failed with %d\n", rval);
		return rval;
	}

	switch (p.if_type) {
	case V4L2_IF_TYPE_BT656:
		/*
		 * The clock rate that the sensor wants has changed.
		 * We have to adjust the xclk from OMAP 2 side to
		 * match the sensor's wish as closely as possible.
		 */
		if (p.u.bt656.clock_curr != cam->if_u.bt656.xclk) {
			u32 xclk = p.u.bt656.clock_curr;

			if (xclk == 0)
				return -EINVAL;

			if (xclk < p.u.bt656.clock_min
			    || xclk > p.u.bt656.clock_max)
				return -EINVAL;

			cam->if_u.bt656.xclk = xclk;
		}

		/* program the agreed new xclk frequency */
		if (hwc.xclk != OMAP34XXCAM_XCLK_NONE)
			cam->if_u.bt656.xclk =
			    isp_set_xclk(cam->if_u.bt656.xclk, hwc.xclk);
		break;

	default:
		/* FIXME: how about other interfaces? */
		dev_err(cam->dev, "interface type %d not supported\n",
			p.if_type);
		return -EINVAL;
	}

	return 0;

}

/**
 * omap34xxcam_sensor_if_disable - disable the external sensor interface
 * @cam: ptr. to per-device camera information data structure
 *
 * Disable the external sensor interface and clock.
 */
static void omap34xxcam_sensor_if_disable(const struct omap34xxcam_device *cam)
{
	struct v4l2_ifparm p;
	struct omap34xxcam_hw_config hwc;

	if (!cam->sdev)
		return;

	BUG_ON(vidioc_int_g_ifparm(cam->sdev, &p) < 0);
	BUG_ON(vidioc_int_g_priv(cam->sdev, (void *)&hwc) < 0);

	switch (p.if_type) {
	case V4L2_IF_TYPE_BT656:
		isp_set_xclk(0, hwc.xclk);
		isp_put();
		break;
	default:
		break;
	}
}

/**
 * omap34xxcam_sensor_init - Initialise the sensor hardware
 * @cam: ptr. to per-device camera information data structure
 *
 * Enable the sensor interface, power up sensor, send internal
 * initialization ioc to sensor driver, then disable sensor
 * interface and power sensor down.  If sensor initialization
 * was successful, function returns 0, otherwise the
 * appropriate error code is returned.
 */
static int omap34xxcam_sensor_init(struct omap34xxcam_device *cam)
{
	int err = 0;
	struct v4l2_int_device *sdev = cam->sdev;

	err = omap34xxcam_sensor_if_enable(cam);
	if (err) {
		dev_err(cam->dev, "sensor interface could not be enabled at "
			"initialisation, %d\n", err);
		cam->sdev = NULL;
		goto out;
	}

	/* power up sensor during sensor initialization */
	vidioc_int_s_power(sdev, V4L2_POWER_ON);

	err = vidioc_int_dev_init(sdev);
	if (err) {
		dev_err(cam->dev, "cannot initialize sensor, error %d\n", err);
		/* Sensor init failed --- it's nonexistent to us! */
		cam->sdev = NULL;
		goto out;
	}

	dev_info(cam->dev, "sensor is %s\n", sdev->name);

out:
	omap34xxcam_sensor_if_disable(cam);

	if (err)
		vidioc_int_s_power(sdev, V4L2_POWER_OFF);
	else
		vidioc_int_s_power(sdev, V4L2_POWER_STANDBY);

	return err;
}

/**
 * omap34xxcam_sensor_exit - Detach the slave sensor device
 * @cam: ptr. to per-device camera information data structure
 *
 * Send vidioc_int_dev_exit_num to sensor (slave) device to
 * inform it that it has been detached.  Complement of
 * omap34xxcam_sensor_init()
 */
static void omap34xxcam_sensor_exit(struct omap34xxcam_device *cam)
{
	if (cam->sdev) {
		vidioc_int_dev_exit(cam->sdev);
		vidioc_int_s_power(cam->sdev, V4L2_POWER_OFF);
	}
}

/**
 * omap34xxcam_sensor_disable - Disable sensor device
 * @cam: ptr. to per-device camera information data structure
 *
 * Disable sensor interface and power off slave sensor device.
 */
static void omap34xxcam_sensor_disable(struct omap34xxcam_device *cam)
{
	omap34xxcam_sensor_if_disable(cam);
	vidioc_int_s_power(cam->sdev, V4L2_POWER_STANDBY);
}


/**
 * omap34xxcam_sensor_enable - Enable sensor device
 * @cam: ptr. to per-device camera information data structure
 *
 * Power-up and configure camera sensor and sensor interface. On
 * successful return (0), it's ready for capturing now.
 */
static int omap34xxcam_sensor_enable(struct omap34xxcam_device *cam)
{
	int rval;

	omap34xxcam_sensor_if_enable(cam);

	rval = vidioc_int_s_power(cam->sdev, V4L2_POWER_RESUME);
	if (rval)
		goto out;

	rval = vidioc_int_init(cam->sdev);
	if (rval)
		goto out;

	return 0;

out:
	omap34xxcam_sensor_disable(cam);

	return rval;
}

/**
 * omap34xxcam_update_vbq - Updates VBQ with completed input buffer
 * @vb: ptr. to standard V4L2 video buffer structure
 *
 * Updates video buffer queue with completed buffer passed as
 * input parameter.  Also updates ISP H3A timestamp and field count
 * statistics.
 */
int omap34xxcam_update_vbq(struct videobuf_buffer *vb)
{
	struct omap34xxcam_fh *fh = camfh_saved;
	struct omap34xxcam_device *cam = fh->cam;
	struct isph3a_aewb_xtrastats xtrastats;
	int rval = 0;

	do_gettimeofday(&vb->ts);
	vb->field_count = atomic_add_return(2, &fh->field_count);
	vb->state = VIDEOBUF_DONE;

	xtrastats.ts = vb->ts;
	xtrastats.field_count = vb->field_count;

	if (cam->streaming)
		rval = 1;

	wake_up(&vb->done);
	isph3a_aewb_setxtrastats(&xtrastats);

	return rval;
}

/**
 * omap34xxcam_vbq_setup - Calcs size and num of buffs allowed in queue
 * @vbq: ptr. to standard V4L2 video buffer queue structure
 * @cnt: ptr to location to hold the count of buffers to be in the queue
 * @size: ptr to location to hold the size of a frame
 *
 * Calculates the number of buffers of current image size that can be
 * supported by the available capture memory.
 */
static int omap34xxcam_vbq_setup(struct videobuf_queue *vbq, unsigned int *cnt,
				 unsigned int *size)
{
	struct omap34xxcam_fh *fh = vbq->priv_data;
	struct v4l2_format format;

	if (*cnt <= 0)
		*cnt = VIDEO_MAX_FRAME;	/* supply a default number of buffers */

	if (*cnt > VIDEO_MAX_FRAME)
		*cnt = VIDEO_MAX_FRAME;

	isp_g_fmt_cap(&format);
	*size = format.fmt.pix.sizeimage;

	/* accessing fh->cam->capture_mem is ok, it's constant */
	while (*size * *cnt > fh->cam->capture_mem)
		(*cnt)--;

	return 0;
}

/**
 * omap34xxcam_vbq_release - Free resources for input VBQ and VB
 * @vbq: ptr. to standard V4L2 video buffer queue structure
 * @vb: ptr to standard V4L2 video buffer structure
 *
 * Unmap and free all memory associated with input VBQ and VB, also
 * unmap the address in ISP MMU.  Reset the VB state.
 */
static void omap34xxcam_vbq_release(struct videobuf_queue *vbq,
				    struct videobuf_buffer *vb)
{
	isp_vbq_release(vbq, vb);

	if (vb->memory != V4L2_MEMORY_MMAP) {
		videobuf_dma_unmap(vbq, videobuf_to_dma(vb));
		videobuf_dma_free(videobuf_to_dma(vb));
	}
	vb->state = VIDEOBUF_NEEDS_INIT;
	return;
}

/**
 * omap34xxcam_vbq_prepare - V4L2 video ops buf_prepare handler
 * @vbq: ptr. to standard V4L2 video buffer queue structure
 * @vb: ptr to standard V4L2 video buffer structure
 * @field: standard V4L2 field enum
 *
 * Verifies there is sufficient locked memory for the requested
 * buffer, or if there is not, allocates, locks and initializes
 * it.
 */
static int omap34xxcam_vbq_prepare(struct videobuf_queue *vbq,
				   struct videobuf_buffer *vb,
				   enum v4l2_field field)
{
	struct v4l2_format format;
	unsigned int size;
	int err = 0;

	isp_g_fmt_cap(&format);
	size = format.fmt.pix.sizeimage;
	/*
	 * Accessing pix here is okay since it's constant while
	 * streaming is on (and we only get called then).
	 */
	if (vb->baddr) {
		/* This is a userspace buffer. */
		if (size > vb->bsize)
			/* The buffer isn't big enough. */
			err = -EINVAL;
		else {
			vb->size = size;
			vb->bsize = vb->size;
		}
	} else {
		if (vb->state != VIDEOBUF_NEEDS_INIT) {
			/*
			 * We have a kernel bounce buffer that has
			 * already been allocated.
			 */
			if (size > vb->size) {
				/*
				 * The image size has been changed to
				 * a larger size since this buffer was
				 * allocated, so we need to free and
				 * reallocate it.
				 */
				omap34xxcam_vbq_release(vbq, vb);
				vb->size = size;
			}
		} else {
			/* We need to allocate a new kernel bounce buffer. */
			vb->size = size;
		}
	}

	if (err)
		return err;

	vb->width = format.fmt.pix.width;
	vb->height = format.fmt.pix.height;
	vb->field = field;

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		err = videobuf_iolock(vbq, vb, NULL);
		if (!err) {
			/* isp_addr will be stored locally inside isp code */
			err = isp_vbq_prepare(vbq, vb, field);
		}
	}

	if (!err)
		vb->state = VIDEOBUF_PREPARED;
	else
		omap34xxcam_vbq_release(vbq, vb);

	return err;

}

/**
 * omap34xxcam_vbq_queue - V4L2 video ops buf_queue handler
 * @vbq: ptr. to standard V4L2 video buffer queue structure
 * @vb: ptr to standard V4L2 video buffer structure
 *
 * Maps the video buffer to sgdma and through the isp, sets
 * the isp buffer done callback and sets the video buffer state
 * to active.
 */
static void omap34xxcam_vbq_queue(struct videobuf_queue *vbq,
				  struct videobuf_buffer *vb)
{
	struct omap34xxcam_fh *fh = vbq->priv_data;
	struct omap34xxcam_device *cam = fh->cam;
	enum videobuf_state state = vb->state;
	isp_vbq_callback_ptr func_ptr;
	int err = 0;
	camfh_saved = fh;

	func_ptr = omap34xxcam_update_vbq;
	vb->state = VIDEOBUF_ACTIVE;

	err = isp_sgdma_queue(videobuf_to_dma(vb),
			      vb, 0, &cam->dma_notify, func_ptr);
	if (err) {
		dev_dbg(cam->dev, "vbq queue failed\n");
		vb->state = state;
	}

}

static struct videobuf_queue_ops omap34xxcam_vbq_ops = {
	.buf_setup = omap34xxcam_vbq_setup,
	.buf_prepare = omap34xxcam_vbq_prepare,
	.buf_queue = omap34xxcam_vbq_queue,
	.buf_release = omap34xxcam_vbq_release,
};

/*
 *
 * IOCTL interface.
 *
 */

/**
 * vidioc_querycap - V4L2 query capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @cap: ptr to standard V4L2 capability structure
 *
 * Fill in the V4L2 capabliity structure for the camera device
 */
static int vidioc_querycap(struct file *file, void *fh,
			   struct v4l2_capability *cap)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;

	strlcpy(cap->driver, CAM_NAME, sizeof(cap->driver));
	strlcpy(cap->card, cam->sdev->name, sizeof(cap->card));
	cap->version = OMAP34XXCAM_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

/**
 * vidioc_enum_fmt_cap - V4L2 enumerate format capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @f: ptr to standard V4L2 format description structure
 *
 * Fills in enumerate format capabilities information for sensor (if SOC
 * sensor attached) or ISP (if raw sensor attached).
 */
static int vidioc_enum_fmt_cap(struct file *file, void *fh,
			       struct v4l2_fmtdesc *f)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	struct omap34xxcam_sensor *sens = &cam->sens;
	int rval;

	if (sens->sensor_isp)
		rval = vidioc_int_enum_fmt_cap(cam->sdev, f);
	else
		rval = isp_enum_fmt_cap(f);

	return rval;
}

/**
 * vidioc_g_fmt_cap - V4L2 get format capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @f: ptr to standard V4L2 format structure
 *
 * Fills in format capabilities for sensor (if SOC sensor attached) or ISP
 * (if raw sensor attached).
 */
static int vidioc_g_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	struct omap34xxcam_sensor *sens = &cam->sens;
	int rval;

	mutex_lock(&cam->mutex);
	if (sens->sensor_isp) {
		rval = vidioc_int_g_fmt_cap(cam->sdev, f);
	} else {
		isp_g_fmt_cap(f);
		rval = 0;
	}
	mutex_unlock(&cam->mutex);

	return rval;
}

/**
 * vidioc_s_fmt_cap - V4L2 set format capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @f: ptr to standard V4L2 format structure
 *
 * Attempts to set input format with the sensor driver (first) and then the
 * ISP.  Returns the return code from vidioc_g_fmt_cap().
 */
static int vidioc_s_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;

	mutex_lock(&cam->mutex);
	if (cam->streaming) {
		rval = -EBUSY;
		goto out;
	}

	ofh->pix.width = f->fmt.pix.width;
	ofh->pix.height = f->fmt.pix.height;
	ofh->pix.pixelformat = f->fmt.pix.pixelformat;
	/* Always negotiate with the sensor first */
	rval = vidioc_int_s_fmt_cap(cam->sdev, f);
	if (rval)
		goto out;

	/* Negotiate with OMAP3 ISP */
	rval = isp_s_fmt_cap(pix, &ofh->pix);
out:
	mutex_unlock(&cam->mutex);

	if (!rval) {
		mutex_lock(&ofh->vbq.vb_lock);
		ofh->pix = f->fmt.pix;
		mutex_unlock(&ofh->vbq.vb_lock);
	}

	memset(f, 0, sizeof(*f));
	rval = vidioc_g_fmt_cap(file, fh, f);
	return rval;
}

/**
 * vidioc_try_fmt_cap - V4L2 try format capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @f: ptr to standard V4L2 format structure
 *
 * Checks if the given format is supported by the sensor driver and
 * by the ISP.
 */
static int vidioc_try_fmt_cap(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_pix_format pix_out;
	int rval;

	mutex_lock(&cam->mutex);
	pix_out.width = f->fmt.pix.width;
	pix_out.height = f->fmt.pix.height;
	pix_out.pixelformat = f->fmt.pix.pixelformat;
	rval = vidioc_int_try_fmt_cap(cam->sdev, f);
	if (rval)
		goto out;

	rval = isp_try_fmt_cap(pix, &pix_out);

out:
	mutex_unlock(&cam->mutex);
	return rval;
}

/**
 * vidioc_reqbufs - V4L2 request buffers IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @b: ptr to standard V4L2 request buffers structure
 *
 * Attempts to get a buffer from the buffer queue associated with the
 * fh through the video buffer library API.
 */
static int vidioc_reqbufs(struct file *file, void *fh,
			  struct v4l2_requestbuffers *b)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	int rval;

	mutex_lock(&cam->mutex);
	if (cam->streaming) {
		mutex_unlock(&cam->mutex);
		return -EBUSY;
	}

	mutex_unlock(&cam->mutex);

	rval = videobuf_reqbufs(&ofh->vbq, b);

	/*
	 * Either videobuf_reqbufs failed or the buffers are not
	 * memory-mapped (which would need special attention).
	 */
	if (rval < 0 || b->memory != V4L2_MEMORY_MMAP)
		goto out;

out:
	return rval;
}

/**
 * vidioc_querybuf - V4L2 query buffer IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @b: ptr to standard V4L2 buffer structure
 *
 * Attempts to fill in the v4l2_buffer structure for the buffer queue
 * associated with the fh through the video buffer library API.
 */
static int vidioc_querybuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct omap34xxcam_fh *ofh = fh;

	return videobuf_querybuf(&ofh->vbq, b);
}

/**
 * vidioc_qbuf - V4L2 queue buffer IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @b: ptr to standard V4L2 buffer structure
 *
 * Attempts to queue the v4l2_buffer on the buffer queue
 * associated with the fh through the video buffer library API.
 */
static int vidioc_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct omap34xxcam_fh *ofh = fh;

	return videobuf_qbuf(&ofh->vbq, b);
}

/**
 * vidioc_dqbuf - V4L2 dequeue buffer IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @b: ptr to standard V4L2 buffer structure
 *
 * Attempts to dequeue the v4l2_buffer from the buffer queue
 * associated with the fh through the video buffer library API.  If the
 * buffer is a user space buffer, then this function will also requeue it,
 * as user does not expect to do this.
 */
static int vidioc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct omap34xxcam_fh *ofh = fh;

	return videobuf_dqbuf(&ofh->vbq, b, file->f_flags & O_NONBLOCK);
}

/**
 * vidioc_streamon - V4L2 streamon IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @i: V4L2 buffer type
 *
 * Attempts to start streaming by enabling the sensor interface and turning
 * on video buffer streaming through the video buffer library API.  Upon
 * success the function returns 0, otherwise an error code is returned.
 */
static int vidioc_streamon(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	int rval;

	mutex_lock(&cam->mutex);
	if (cam->streaming) {
		rval = -EBUSY;
		goto out;
	}

	rval = omap34xxcam_sensor_if_enable(cam);
	if (rval) {
		dev_dbg(cam->dev, "vidioc_int_g_ifparm failed\n");
		goto out;
	}

	cam->dma_notify = 1;
	rval = videobuf_streamon(&ofh->vbq);
	if (!rval)
		cam->streaming = file;

out:
	mutex_unlock(&cam->mutex);

	return rval;
}

/**
 * vidioc_streamoff - V4L2 streamoff IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @i: V4L2 buffer type
 *
 * Attempts to stop streaming by flushing all scheduled work, waiting on
 * any queued buffers to complete and then stopping the ISP and turning
 * off video buffer streaming through the video buffer library API.  Upon
 * success the function returns 0, otherwise an error code is returned.
 */
static int vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	struct videobuf_queue *q = &ofh->vbq;
	int bufcount;
	int rval;

	for (bufcount = 0; bufcount < VIDEO_MAX_FRAME; bufcount++) {
		if (NULL == q->bufs[bufcount])
			continue;
		if (q->bufs[bufcount]->state == VIDEOBUF_QUEUED) {
			rval = videobuf_waiton(q->bufs[bufcount], 0, 0);
			if (rval)
				return rval;
		}
	}

	isp_stop();

	rval = videobuf_streamoff(q);
	if (!rval) {
		mutex_lock(&cam->mutex);
		cam->streaming = NULL;
		mutex_unlock(&cam->mutex);
	}

	return rval;
}

/**
 * vidioc_enum_input - V4L2 enumerate input IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @inp: V4L2 input type information structure
 *
 * Fills in v4l2_input structure.  Returns 0.
 */
static int vidioc_enum_input(struct file *file, void *fh,
			     struct v4l2_input *inp)
{
	if (inp->index > 0)
		return -EINVAL;

	strlcpy(inp->name, "camera", sizeof(inp->name));
	inp->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

/**
 * vidioc_g_input - V4L2 get input IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @i: address to hold index of input supported
 *
 * Sets index to 0.
 */
static int vidioc_g_input(struct file *file, void *fh, unsigned int *i)
{
	*i = 0;

	return 0;
}

/**
 * vidioc_s_input - V4L2 set input IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @i: index of input selected
 *
 * 0 is only index supported.
 */
static int vidioc_s_input(struct file *file, void *fh, unsigned int i)
{
	if (i > 0)
		return -EINVAL;

	return 0;
}

/**
 * vidioc_queryctrl - V4L2 query control IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 query control ioctl structure
 *
 * If the requested control is supported, returns the control information
 * in the v4l2_queryctrl structure.  Otherwise, returns -EINVAL if the
 * control is not supported.  If the sensor being used is a "smart sensor",
 * this request is passed to the sensor driver, otherwise the ISP is
 * queried and if it does not support the requested control, the request
 * is forwarded to the "raw" sensor driver to see if it supports it.
 */
static int vidioc_queryctrl(struct file *file, void *fh,
			    struct v4l2_queryctrl *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	struct omap34xxcam_sensor *sens = &cam->sens;
	int rval;

	mutex_lock(&cam->mutex);
	if (sens->sensor_isp) {
		rval = vidioc_int_queryctrl(cam->sdev, a);
	} else {
		rval = isp_queryctrl(a);
		if (rval) {
			/* ISP does not support, check sensor */
			rval = vidioc_int_queryctrl(cam->sdev, a);
		}
	}
	mutex_unlock(&cam->mutex);

	return rval;
}

/**
 * vidioc_g_ctrl - V4L2 get control IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 control structure
 *
 * If the sensor being used is a "smart sensor",
 * this request is passed to the sensor driver, otherwise the ISP is
 * queried and if it does not support the requested control, the request
 * is forwarded to the "raw" sensor driver to see if it supports it.
 * If one of these supports the control, the current value of the control
 * is returned in the v4l2_control structure.  Otherwise, -EINVAL is
 * returned if the control is not supported.
 */
static int vidioc_g_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	struct omap34xxcam_sensor *sens = &cam->sens;
	int rval;

	mutex_lock(&cam->mutex);
	if (sens->sensor_isp) {
		rval = vidioc_int_g_ctrl(cam->sdev, a);
	} else {
		rval = isp_g_ctrl(a);
		/* If control not supported on ISP, try sensor */
		if (rval)
			rval = vidioc_int_g_ctrl(cam->sdev, a);
	}
	mutex_unlock(&cam->mutex);

	return rval;
}

/**
 * vidioc_s_ctrl - V4L2 set control IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 control structure
 *
 * If the sensor being used is a "smart sensor", this request is passed to
 * the sensor driver.  Otherwise if the control id is the private
 * V4L2_CID_PRIVATE_ISP_AEWB_REQ from the ISP for analog gain or exposure,
 * then this request is forwared directly to the sensor driver.  Otherwise,
 * the ISP is queried and if it does not support the requested control,
 * the request is forwarded to the "raw" sensor driver to see if it supports
 * it.
 * If one of these supports the control, the current value of the control
 * is returned in the v4l2_control structure.  Otherwise, -EINVAL is
 * returned if the control is not supported.
 */
static int vidioc_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	struct omap34xxcam_sensor *sens = &cam->sens;
	int rval;

	mutex_lock(&cam->mutex);
	if (sens->sensor_isp) {
		rval = vidioc_int_s_ctrl(cam->sdev, a);
	} else {
		if (a->id == V4L2_CID_PRIVATE_ISP_AEWB_REQ) {
			/* Need to update sensor first */
			struct isph3a_aewb_data data;
			struct v4l2_control vc;
			if (copy_from_user(&data, (void *)a->value,
					   sizeof(data))) {
				rval = -EFAULT;
				printk(KERN_ERR "Failed copy_from_user\n");
				goto out;
			}
			if (data.update & SET_EXPOSURE) {
				vc.id = V4L2_CID_EXPOSURE;
				vc.value = data.shutter;
				rval = vidioc_int_s_ctrl(cam->sdev, &vc);
				if (rval)
					goto out;
			}
			if (data.update & SET_ANALOG_GAIN) {
				vc.id = V4L2_CID_GAIN;
				vc.value = data.gain;
				rval = vidioc_int_s_ctrl(cam->sdev, &vc);
				if (rval)
					goto out;
			}
		}
		rval = isp_s_ctrl(a);
		/* If control not supported on ISP, try sensor */
		if (rval)
			rval = vidioc_int_s_ctrl(cam->sdev, a);
	}
out:
	mutex_unlock(&cam->mutex);

	return rval;
}

/**
 * vidioc_g_parm - V4L2 get parameters IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 stream parameters structure
 *
 * If request is for video capture buffer type, handles request by
 * forwarding to sensor driver.
 */
static int vidioc_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	int rval;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	mutex_lock(&cam->mutex);
	rval = vidioc_int_g_parm(cam->sdev, a);
	mutex_unlock(&cam->mutex);

	return rval;
}

/**
 * vidioc_s_parm - V4L2 set parameters IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 stream parameters structure
 *
 * If request is for video capture buffer type, handles request by
 * first getting current stream parameters from sensor, then forwarding
 * request to set new parameters to sensor driver.  It then attempts to
 * enable the sensor interface with the new parameters.  If this fails, it
 * reverts back to the previous parameters.
 */
static int vidioc_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	struct v4l2_streamparm old_streamparm;
	int rval;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	mutex_lock(&cam->mutex);
	if (cam->streaming) {
		rval = -EBUSY;
		goto out;
	}

	old_streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	rval = vidioc_int_g_parm(cam->sdev, &old_streamparm);
	if (rval)
		goto out;

	rval = vidioc_int_s_parm(cam->sdev, a);
	if (rval)
		goto out;

	rval = omap34xxcam_sensor_if_enable(cam);
	/*
	 * Revert to old streaming parameters if enabling sensor
	 * interface with the new ones failed.
	 */
	if (rval)
		vidioc_int_s_parm(cam->sdev, &old_streamparm);

out:
	mutex_unlock(&cam->mutex);

	return rval;
}

/**
 * vidioc_cropcap - V4L2 crop capture IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 crop capture structure
 *
 * If using a "smart" sensor, just forwards request to the sensor driver,
 * otherwise fills in the v4l2_cropcap values locally.
 */
static int vidioc_cropcap(struct file *file, void *fh, struct v4l2_cropcap *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	struct omap34xxcam_sensor *sens = &cam->sens;
	struct v4l2_cropcap *cropcap = a;
	int rval;

	if (sens->sensor_isp) {
		rval = vidioc_int_cropcap(cam->sdev, a);
	} else {
		cropcap->bounds.left = cropcap->bounds.top = 0;
		cropcap->bounds.width = ofh->pix.width;
		cropcap->bounds.height = ofh->pix.height;
		cropcap->defrect = cropcap->bounds;
		cropcap->pixelaspect.numerator = 1;
		cropcap->pixelaspect.denominator = 1;
		rval = 0;
	}
	return rval;
}

/**
 * vidioc_g_crop - V4L2 get capture crop IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 crop structure
 *
 * If using a "smart" sensor, just forwards request to the sensor driver,
 * otherwise calls the isp functions to fill in current crop values.
 */
static int vidioc_g_crop(struct file *file, void *fh, struct v4l2_crop *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	struct omap34xxcam_sensor *sens = &cam->sens;
	int rval = 0;

	if (sens->sensor_isp)
		rval = vidioc_int_g_crop(cam->sdev, a);
	else
		rval = isp_g_crop(a);

	return rval;
}

/**
 * vidioc_s_crop - V4L2 set capture crop IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 crop structure
 *
 * If using a "smart" sensor, just forwards request to the sensor driver,
 * otherwise calls the isp functions to set the current crop values.
 */
static int vidioc_s_crop(struct file *file, void *fh, struct v4l2_crop *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_device *cam = ofh->cam;
	struct omap34xxcam_sensor *sens = &cam->sens;
	struct v4l2_pix_format *pix = &ofh->pix;
	int rval = 0;

	if (sens->sensor_isp)
		rval = vidioc_int_s_crop(cam->sdev, a);
	else
		rval = isp_s_crop(a, pix);

	return rval;
}

/*
 *
 * File operations.
 *
 */

/**
 * omap34xxcam_poll - file operations poll handler
 * @file: ptr. to system file structure
 * @wait: system poll table structure
 *
 */
static unsigned int omap34xxcam_poll(struct file *file,
				     struct poll_table_struct *wait)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_device *cam = fh->cam;
	struct videobuf_buffer *vb;

	mutex_lock(&cam->mutex);
	if (cam->streaming != file) {
		mutex_unlock(&cam->mutex);
		return POLLERR;
	}
	mutex_unlock(&cam->mutex);

	mutex_lock(&fh->vbq.vb_lock);
	if (list_empty(&fh->vbq.stream)) {
		mutex_unlock(&fh->vbq.vb_lock);
		return POLLERR;
	}
	vb = list_entry(fh->vbq.stream.next, struct videobuf_buffer, stream);
	mutex_unlock(&fh->vbq.vb_lock);

	poll_wait(file, &vb->done, wait);

	if (vb->state == VIDEOBUF_DONE || vb->state == VIDEOBUF_ERROR)
		return POLLIN | POLLRDNORM;

	return 0;
}

/**
 * omap34xxcam_mmap - file operations mmap handler
 * @file: ptr. to system file structure
 * @vma: system virt. mem. area structure
 *
 * Maps a virtual memory area via the video buffer API
 */
static int omap34xxcam_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct omap34xxcam_fh *fh = file->private_data;
	return videobuf_mmap_mapper(&fh->vbq, vma);
}

/**
 * omap34xxcam_open - file operations open handler
 * @inode: ptr. to system inode structure
 * @file: ptr. to system file structure
 *
 * Allocates and initializes the per-filehandle data (omap34xxcam_fh),
 * enables the sensor, opens/initializes the ISP interface and the
 * video buffer queue.  Note that this function will allow multiple
 * file handles to be open simultaneously, however only the first
 * handle opened will initialize the ISP.  It is the application
 * responsibility to only use one handle for streaming and the others
 * for control only.
 * This function returns 0 upon success and -ENODEV upon error.
 */
static int omap34xxcam_open(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);
	struct omap34xxcam_device *cam = omap34xxcam.priv;
	struct omap34xxcam_sensor *sens = &cam->sens;
	struct omap34xxcam_fh *fh;
	struct v4l2_format format;

	if (!cam || !cam->vfd || (cam->vfd->minor != minor))
		return -ENODEV;

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (fh == NULL)
		return -ENOMEM;

	mutex_lock(&cam->mutex);
	if (cam->sdev == NULL || !try_module_get(cam->sdev->module)) {
		mutex_unlock(&cam->mutex);
		goto out_try_module_get;
	}

	if (atomic_inc_return(&cam->users) == 1) {
		if (omap34xxcam_sensor_enable(cam)) {
			mutex_unlock(&cam->mutex);
			goto out_omap34xxcam_sensor_enable;
		}
		isp_open();
	}

	mutex_unlock(&cam->mutex);
	fh->cam = cam;
	mutex_lock(&cam->mutex);

	if (sens->sensor_isp)
		vidioc_int_g_fmt_cap(cam->sdev, &format);
	else
		isp_g_fmt_cap(&format);

	mutex_unlock(&cam->mutex);
	/* FIXME: how about fh->pix when there are more users? */
	fh->pix = format.fmt.pix;

	file->private_data = fh;

	spin_lock_init(&fh->vbq_lock);

	videobuf_queue_sg_init(&fh->vbq, &omap34xxcam_vbq_ops, NULL,
				&fh->vbq_lock, V4L2_BUF_TYPE_VIDEO_CAPTURE,
				V4L2_FIELD_NONE,
				sizeof(struct videobuf_buffer), fh);

	return 0;

out_omap34xxcam_sensor_enable:
	module_put(cam->sdev->module);

out_try_module_get:
	kfree(fh);

	return -ENODEV;
}

/**
 * omap34xxcam_release - file operations release handler
 * @inode: ptr. to system inode structure
 * @file: ptr. to system file structure
 *
 * Complement of omap34xxcam_open.  This function will flush any scheduled
 * work, disable the sensor, close the ISP interface, stop the
 * video buffer queue from streaming and free the per-filehandle data
 * (omap34xxcam_fh).  Note that because multiple open file handles
 * are allowed, this function will only close the ISP and disable the
 * sensor when the last open file handle (by count) is closed.
 * This function returns 0.
 */
static int omap34xxcam_release(struct inode *inode, struct file *file)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_device *cam = fh->cam;
	bool full_deinit = false;

	mutex_lock(&cam->mutex);
	if (atomic_dec_return(&cam->users) == 0)
		full_deinit = true;
	mutex_unlock(&cam->mutex);

	if (full_deinit)
		isp_close();

	/* stop streaming capture */
	videobuf_streamoff(&fh->vbq);

	if (full_deinit) {
		mutex_lock(&cam->mutex);
		if (cam->streaming == file) {
			cam->streaming = NULL;
			mutex_unlock(&cam->mutex);
		} else {
			mutex_unlock(&cam->mutex);
		}
	}

	if (full_deinit) {
		mutex_lock(&cam->mutex);
		omap34xxcam_sensor_disable(cam);
		mutex_unlock(&cam->mutex);
	}

	file->private_data = NULL;

	module_put(cam->sdev->module);
	kfree(fh);

	return 0;
}

static long omap34xxcam_unlocked_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	return (long)video_ioctl2(file->f_dentry->d_inode, file, cmd, arg);
}

static struct file_operations omap34xxcam_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = omap34xxcam_unlocked_ioctl,
	.poll = omap34xxcam_poll,
	.mmap = omap34xxcam_mmap,
	.open = omap34xxcam_open,
	.release = omap34xxcam_release,
};

/**
 * omap34xxcam_device_unregister - V4L2 detach handler
 * @s: ptr. to standard V4L2 device information structure
 *
 * Detach sensor and unregister and release the video device.
 */
static void omap34xxcam_device_unregister(struct v4l2_int_device *s)
{
	struct omap34xxcam_device *cam = s->u.slave->master->priv;

	omap34xxcam_sensor_exit(cam);

	if (cam->vfd) {
		if (cam->vfd->minor == -1) {
			/*
			 * The device was never registered, so release the
			 * video_device struct directly.
			 */
			video_device_release(cam->vfd);
		} else {
			/*
			 * The unregister function will release the
			 * video_device struct as well as
			 * unregistering it.
			 */
			video_unregister_device(cam->vfd);
		}
		cam->vfd = NULL;
	}

	cam->sdev = NULL;
}

/**
 * omap34xxcam_device_register - V4L2 attach handler
 * @s: ptr. to standard V4L2 device information structure
 *
 * Allocates and initializes the V4L2 video_device structure, initializes
 * the sensor, and finally registers the device with V4L2 based on the
 * video_device structure.
 *
 * Returns 0 on success, otherwise an appropriate error code on
 * failure.
 */
static int omap34xxcam_device_register(struct v4l2_int_device *s)
{
	struct omap34xxcam_device *cam = s->u.slave->master->priv;
	struct video_device *vfd;
	int rval;

	/* We already have a slave. */
	if (cam->sdev)
		return -EBUSY;

	cam->sdev = s;

	/* initialize the video_device struct */
	vfd = cam->vfd = video_device_alloc();
	if (!vfd) {
		dev_err(cam->dev, "could not allocate video device struct\n");
		rval = -ENOMEM;
		goto err;
	}
	vfd->release = video_device_release;

	vfd->dev = cam->dev;

	strlcpy(vfd->name, CAM_NAME "/", sizeof(vfd->name));
	strlcpy(vfd->name + strlen(vfd->name),
		s->name, sizeof(vfd->name) - strlen(vfd->name));
	vfd->type = VID_TYPE_CAPTURE;
	vfd->fops = &omap34xxcam_fops;
	vfd->priv = cam;
	vfd->minor = -1;

	vfd->vidioc_querycap = vidioc_querycap;
	vfd->vidioc_enum_fmt_cap = vidioc_enum_fmt_cap;
	vfd->vidioc_g_fmt_cap = vidioc_g_fmt_cap;
	vfd->vidioc_s_fmt_cap = vidioc_s_fmt_cap;
	vfd->vidioc_try_fmt_cap = vidioc_try_fmt_cap;
	vfd->vidioc_reqbufs = vidioc_reqbufs;
	vfd->vidioc_querybuf = vidioc_querybuf;
	vfd->vidioc_qbuf = vidioc_qbuf;
	vfd->vidioc_dqbuf = vidioc_dqbuf;
	vfd->vidioc_streamon = vidioc_streamon;
	vfd->vidioc_streamoff = vidioc_streamoff;
	vfd->vidioc_enum_input = vidioc_enum_input;
	vfd->vidioc_g_input = vidioc_g_input;
	vfd->vidioc_s_input = vidioc_s_input;
	vfd->vidioc_queryctrl = vidioc_queryctrl;
	vfd->vidioc_g_ctrl = vidioc_g_ctrl;
	vfd->vidioc_s_ctrl = vidioc_s_ctrl;
	vfd->vidioc_g_parm = vidioc_g_parm;
	vfd->vidioc_s_parm = vidioc_s_parm;
	vfd->vidioc_cropcap = vidioc_cropcap;
	vfd->vidioc_g_crop = vidioc_g_crop;
	vfd->vidioc_s_crop = vidioc_s_crop;

	rval = omap34xxcam_sensor_init(cam);
	if (rval)
		goto err;

	if (video_register_device(vfd, VFL_TYPE_GRABBER, video_nr) < 0) {
		dev_err(cam->dev, "could not register V4L device\n");
		vfd->minor = -1;
		rval = -EBUSY;
		goto err;
	}

	dev_info(cam->dev, "registered device video%d\n", vfd->minor);

	return 0;

err:
	omap34xxcam_device_unregister(s);

	return rval;
}

static struct v4l2_int_master omap34xxcam_master = {
	.attach = omap34xxcam_device_register,
	.detach = omap34xxcam_device_unregister,
};

static struct v4l2_int_device omap34xxcam = {
	.module = THIS_MODULE,
	.name = CAM_NAME,
	.type = v4l2_int_type_master,
	.u = {
	      .master = &omap34xxcam_master},
};

/*
 *
 * Driver Suspend/Resume
 *
 */

#ifdef CONFIG_PM
/**
 * omap34xxcam_suspend - platform driver PM suspend handler
 * @pdev: ptr. to platform level device information structure
 * @state: power state
 *
 * If applicable, stop capture and disable sensor.
 *
 * Returns 0 always
 */
static int omap34xxcam_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct omap34xxcam_device *cam = platform_get_drvdata(pdev);

	if (atomic_read(&cam->users) == 0)
		return 0;

	if (cam->streaming)
		isp_stop();

	omap34xxcam_sensor_disable(cam);

	return 0;
}

/**
 * omap34xxcam_resume - platform driver PM resume handler
 * @pdev: ptr. to platform level device information structure
 *
 * If applicable, resume capture and enable sensor.
 *
 * Returns 0 always
 */
static int omap34xxcam_resume(struct platform_device *pdev)
{
	struct omap34xxcam_device *cam = platform_get_drvdata(pdev);

	if (atomic_read(&cam->users) == 0)
		return 0;

	omap34xxcam_sensor_enable(cam);

	if (cam->streaming)
		isp_start();

	return 0;
}
#endif

/*
 *
 * Driver initialisation and deinitialisation.
 *
 */

/**
 * omap34xxcam_probe - platform driver probe handler
 * @pdev: ptr. to platform level device information structure
 *
 * Allocates and initializes camera device information structure
 * (omap34xxcam_device), maps the device registers and gets the
 * device IRQ.  Registers the device as a V4L2 client.
 *
 * Returns 0 on success or -ENODEV on failure.
 */
static int omap34xxcam_probe(struct platform_device *pdev)
{

	struct omap34xxcam_device *cam;
	struct resource *mem;
	int irq;

	cam = kzalloc(sizeof(*cam), GFP_KERNEL);
	if (!cam) {
		dev_err(&pdev->dev, "could not allocate memory\n");
		goto err;
	}

	platform_set_drvdata(pdev, cam);

	cam->dev = &pdev->dev;
	/*
	 * Impose a lower limit on the amount of memory allocated for
	 * capture. We require at least enough memory to double-buffer
	 * QVGA (300KB).
	 */
	if (capture_mem < 320 * 240 * 2 * 2)
		capture_mem = 320 * 240 * 2 * 2;

	cam->capture_mem = capture_mem;

	/* request the mem region for the camera registers */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(cam->dev, "no mem resource?\n");
		goto err;
	}

	if (!request_mem_region(mem->start, (mem->end - mem->start) + 1,
				pdev->name)) {
		dev_err(cam->dev,
			"cannot reserve camera register I/O region\n");
		goto err;

	}
	cam->mmio_base_phys = mem->start;
	cam->mmio_size = (mem->end - mem->start) + 1;

	/* map the region */
	cam->mmio_base = (unsigned long)
			ioremap_nocache(cam->mmio_base_phys, cam->mmio_size);
	if (!cam->mmio_base) {
		dev_err(cam->dev, "cannot map camera register I/O region\n");
		goto err;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(cam->dev, "no irq for camera?\n");
		goto err;
	}

	mutex_init(&cam->mutex);

	omap34xxcam.priv = cam;
	isp_sgdma_init();
	if (v4l2_int_device_register(&omap34xxcam))
		goto err;

	return 0;

err:
	omap34xxcam_remove(pdev);
	return -ENODEV;
}

/**
 * omap34xxcam_remove - platform driver remove handler
 * @pdev: ptr. to platform level device information structure
 *
 * Unregister device with V4L2, unmap camera registers, and
 * free camera device information structure (omap34xxcam_device).
 *
 * Returns 0 always.
 */
static int omap34xxcam_remove(struct platform_device *pdev)
{
	struct omap34xxcam_device *cam = platform_get_drvdata(pdev);

	if (!cam)
		return 0;

	if (omap34xxcam.priv != NULL)
		v4l2_int_device_unregister(&omap34xxcam);

	omap34xxcam.priv = NULL;

	if (cam->mmio_base) {
		iounmap((void *)cam->mmio_base);
		cam->mmio_base = 0;
	}

	if (cam->mmio_base_phys) {
		release_mem_region(cam->mmio_base_phys, cam->mmio_size);
		cam->mmio_base_phys = 0;
	}

	kfree(cam);

	return 0;
}

static struct platform_driver omap34xxcam_driver = {
	.probe = omap34xxcam_probe,
	.remove = omap34xxcam_remove,
#ifdef CONFIG_PM
	.suspend = omap34xxcam_suspend,
	.resume = omap34xxcam_resume,
#endif
	.driver = {
		   .name = CAM_NAME,
		   },
};

/*
 *
 * Module initialisation and deinitialisation
 *
 */

/**
 * omap34xxcam_init - module_init function
 *
 * Calls platfrom driver to register probe, remove,
 * suspend and resume functions.
 *
 */
static int __init omap34xxcam_init(void)
{
	return platform_driver_register(&omap34xxcam_driver);
}

/**
 * omap34xxcam_cleanup - module_exit function
 *
 * Calls platfrom driver to unregister probe, remove,
 * suspend and resume functions.
 *
 */
static void __exit omap34xxcam_cleanup(void)
{
	platform_driver_unregister(&omap34xxcam_driver);
}

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP34xx Video for Linux camera driver");
MODULE_LICENSE("GPL");

module_init(omap34xxcam_init);
module_exit(omap34xxcam_cleanup);
