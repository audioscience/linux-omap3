/*
 * drivers/media/video/omap34xxcam.h
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
 * Video-for-Linux (Version 2) camera capture driver for OMAP34xx ISP.
 * Leverage omap24xx camera driver
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2006 Texas Instruments.
 *
 */

#ifndef OMAP34XXCAM_H
#define OMAP34XXCAM_H

#include <media/v4l2-int-device.h>
#include "isp/isp.h"

#define CAM_NAME "omap34xxcam"

#define OMAP_ISP_AF     	(1 << 4)
#define OMAP_ISP_HIST   	(1 << 5)
#define OMAP34XXCAM_XCLK_NONE	-1
#define OMAP34XXCAM_XCLK_A	0
#define OMAP34XXCAM_XCLK_B	1

struct omap34xxcam_device;

/**
 * struct omap34xxcam_sensor - sensor information structure
 * @xclk: OMAP34XXCAM_XCLK_A or OMAP34XXCAM_XCLK_B
 * @sensor_isp: Is sensor smart/SOC or raw
 */
struct omap34xxcam_sensor {
	int xclk;
	int sensor_isp;
};

/**
 * struct omap34xxcam_hw_config - struct for vidioc_int_g_priv ioctl
 * @xclk: OMAP34XXCAM_XCLK_A or OMAP34XXCAM_XCLK_B
 * @sensor_isp: Is sensor smart/SOC or raw
 * @s_pix_sparm: Access function to set pix and sparm.
 * Pix will override sparm
 */
struct omap34xxcam_hw_config {
	int xclk;
	int sensor_isp;
	int (*s_pix_sparm) (struct omap34xxcam_device *cam,
			    struct v4l2_pix_format *pix,
			    struct v4l2_streamparm *parm);
};

/**
 * struct omap34xxcam_device - per-device data structure
 * @mutex: mutex serialises access to this structure. Also camera
 * opening and releasing is synchronised by this.
 * @users: user (open file handle) count.
 * @core_enable_disable_lock: Lock to serialise core enabling and
 * disabling and access to sgdma_in_queue.
 * @sgdma_in_queue: Number or sgdma requests in scatter-gather queue,
 * protected by the lock above.
 * @sens: Sensor interface parameters
 * @if_u: interface type
 * @cc_ctrl: CC_CTRL register value
 * @sgdma: ISP sgdma subsystem information structure
 * @dma_notify: DMA notify flag
 * @irq: irq number platform HW resource
 * @mmio_base: register map memory base (platform HW resource)
 * @mmio_base_phys: register map memory base physical address
 * @mmio_size: register map memory size
 * @sdev: V4L2 device structure
 * @dev: device structure
 * @vfd: video device file handle
 * @sensor_reset_work: sensor reset work structure
 * @in_reset: flag to indicate in reset operation.  Don't enable core if
 * this is non-zero! This exists to help decisionmaking in a case
 * where videobuf_qbuf is called while we are in the middle of
 * a reset.
 * @reset_disable: flag to disallow resets.  Non-zero if we don't want any
 * resets for now. Used to prevent reset work to run when we're about to
 * stop streaming.
 * @capture_mem: memory reserved for capturing image data.
 * @fck: camera module fck clock information
 * @ick: camera module ick clock information
 * @streaming: file handle, if streaming is on.
 */
struct omap34xxcam_device {
	struct mutex mutex;
	atomic_t users;
	spinlock_t core_enable_disable_lock;
	int sgdma_in_queue;
	struct omap34xxcam_sensor sens;
	union {
		struct parallel {
			u32 xclk;
		} bt656;
	} if_u;
	u32 cc_ctrl;
	struct isp_sgdma sgdma;
	int dma_notify;

	/*** platform HW resource ***/
	unsigned int irq;
	unsigned long mmio_base;
	unsigned long mmio_base_phys;
	unsigned long mmio_size;

	/*** interfaces and device ***/
	struct v4l2_int_device *sdev;
	struct device *dev;
	struct video_device *vfd;

	/*** camera and sensor reset related stuff ***/
	struct work_struct sensor_reset_work;

	atomic_t in_reset;
	atomic_t reset_disable;

	/*** video device parameters ***/
	int capture_mem;

	/*** camera module clocks ***/
	struct clk *fck;
	struct clk *ick;

	/*** capture data ***/
	struct file *streaming;
};

/**
 * struct omap34xxcam_fh - per-filehandle data structure
 * @vbq_lock: spinlock for the videobuf queue
 * @vbq: V4L2 video buffer queue structure
 * @pix: V4L2 pixel format structure (serialise pix by vbq->lock)
 * @field_count: field counter for videobuf_buffer
 * @cam: camera device information structure
 */
struct omap34xxcam_fh {
	spinlock_t vbq_lock;
	struct videobuf_queue vbq;
	struct v4l2_pix_format pix;
	atomic_t field_count;
	/* accessing cam here doesn't need serialisation: it's constant */
	struct omap34xxcam_device *cam;
};

#endif /* ifndef OMAP34XXCAM_H */
