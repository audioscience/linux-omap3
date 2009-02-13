/*
 * drivers/media/video/isp/omap_previewer.c
 *
 * Wrapper for Preview module in TI's OMAP3430 ISP
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Contributors:
 * 	Leonides Martinez <leonides.martinez@ti.com>
 * 	Sergio Aguirre <saaguirre@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <media/v4l2-dev.h>
#include <asm/cacheflush.h>

#include "isp.h"
#include "ispmmu.h"
#include "ispreg.h"
#include "omap_previewer.h"

#define OMAP_PREV_NAME		"omap-previewer"

static int prev_major = -1;
static struct device *prev_dev;
static struct class *prev_class;
static struct prev_device *prevdevice;
static struct platform_driver omap_previewer_driver;

static u32 prev_bufsize;

/**
 * prev_calculate_crop - Calculate crop size according to device parameters
 * @device: Structure containing ISP preview wrapper global information
 * @crop: Structure containing crop size
 *
 * This function is used to calculate frame size reduction depending on
 * the features enabled by the application.
 **/
static void prev_calculate_crop(struct prev_device *device,
						struct prev_cropsize *crop)
{
	dev_dbg(prev_dev, "prev_calculate_crop E\n");

	if (!device || !crop) {
		dev_err(prev_dev, "\nErron in argument");
		return;
	}

	isppreview_try_size(device->params->size_params.hsize,
					device->params->size_params.vsize,
					&crop->hcrop, &crop->vcrop);
	crop->hcrop &= PREV_16PIX_ALIGN_MASK;
	dev_dbg(prev_dev, "prev_calculate_crop L\n");
}

/**
 * prev_get_status - Get status of ISP preview module
 * @status: Structure containing the busy state.
 *
 * Checks if the ISP preview module is busy.
 *
 * Returns 0 if successful, or -EINVAL if the status parameter is invalid.
 **/
static int prev_get_status(struct prev_status *status)
{
	if (!status) {
		dev_err(prev_dev, "get_status: invalid parameter\n");
		return -EINVAL;
	}
	status->hw_busy = (char)isppreview_busy();
	return 0;
}

/**
 * prev_hw_setup - Stores the desired configuration in the proper HW registers
 * @config: Structure containing the desired configuration for ISP preview
 *          module.
 *
 * Reads the structure sent, and modifies the desired registers.
 *
 * Always returns 0.
 **/
static int prev_hw_setup(struct prev_params *config)
{
	dev_dbg(prev_dev, "prev_hw_setup E\n");

	if (config->features & PREV_AVERAGER)
		isppreview_config_averager(config->average);
	else
		isppreview_config_averager(0);

	if (config->features & PREV_INVERSE_ALAW)
		isppreview_enable_invalaw(1);
	else
		isppreview_enable_invalaw(0);

	if (config->features & PREV_HORZ_MEDIAN_FILTER) {
		isppreview_config_hmed(config->hmf_params);
		isppreview_enable_hmed(1);
	} else
		isppreview_enable_hmed(0);

	if (config->features & PREV_DARK_FRAME_SUBTRACT) {
		isppreview_set_darkaddr(config->drkf_params.addr);
		isppreview_config_darklineoffset(config->drkf_params.offset);
		isppreview_enable_drkframe(1);
	} else
		isppreview_enable_drkframe(0);

	if (config->features & PREV_LENS_SHADING) {
		isppreview_config_drkf_shadcomp(config->lens_shading_shift);
		isppreview_enable_shadcomp(1);
	} else
		isppreview_enable_shadcomp(0);

	dev_dbg(prev_dev, "prev_hw_setup L\n");
	return 0;
}

/**
 * prev_validate_params - Validate configuration parameters for Preview Wrapper
 * @params: Structure containing configuration parameters
 *
 * Validate configuration parameters for Preview Wrapper
 *
 * Returns 0 if successful, or -EINVAL if a parameter value is invalid.
 **/
static int prev_validate_params(struct prev_params *params)
{
	if (!params) {
		dev_err(prev_dev, "validate_params: error in argument");
		goto err_einval;
	}

	if ((params->features & PREV_AVERAGER) == PREV_AVERAGER) {
		if ((params->average != NO_AVE)
					&& (params->average != AVE_2_PIX)
					&& (params->average != AVE_4_PIX)
					&& (params->average != AVE_8_PIX)) {
			dev_err(prev_dev, "validate_params: wrong pix "
								"average\n");
			goto err_einval;
		} else if (((params->average == AVE_2_PIX)
					&& (params->size_params.hsize % 2))
					|| ((params->average == AVE_4_PIX)
					&& (params->size_params.hsize % 4))
					|| ((params->average == AVE_8_PIX)
					&& (params->size_params.hsize % 8))) {
			dev_err(prev_dev, "validate_params: "
					"wrong pix average for input size\n");
			goto err_einval;
		}
	}

	if ((params->size_params.pixsize != PREV_INWIDTH_8BIT)
					&& (params->size_params.pixsize
					!= PREV_INWIDTH_10BIT)) {
		dev_err(prev_dev, "validate_params: wrong pixsize\n");
		goto err_einval;
	}

	if (params->size_params.hsize > MAX_IMAGE_WIDTH
					|| params->size_params.hsize < 0) {
		dev_err(prev_dev, "validate_params: wrong hsize\n");
		goto err_einval;
	}

	if ((params->pix_fmt != YCPOS_YCrYCb)
					&& (YCPOS_YCbYCr != params->pix_fmt)
					&& (YCPOS_CbYCrY != params->pix_fmt)
					&& (YCPOS_CrYCbY != params->pix_fmt)) {
		dev_err(prev_dev, "validate_params: wrong pix_fmt");
		goto err_einval;
	}

	if ((params->features & PREV_DARK_FRAME_SUBTRACT)
						&& (params->features
						& PREV_DARK_FRAME_CAPTURE)) {
		dev_err(prev_dev, "validate_params: DARK FRAME CAPTURE and "
						"SUBSTRACT cannot be enabled "
						"at same time\n");
		goto err_einval;
	}

	if (params->features & PREV_DARK_FRAME_SUBTRACT)
		if (!params->drkf_params.addr
					|| (params->drkf_params.offset % 32)) {
			dev_err(prev_dev, "validate_params: dark frame "
								"address\n");
			goto err_einval;
		}

	if (params->features & PREV_LENS_SHADING)
		if ((params->lens_shading_shift > 7)
					|| !params->drkf_params.addr
					|| (params->drkf_params.offset % 32)) {
			dev_err(prev_dev, "validate_params: lens shading "
								"shift\n");
			goto err_einval;
		}

	if ((params->size_params.in_pitch <= 0)
				|| (params->size_params.in_pitch % 32)) {
		params->size_params.in_pitch =
				(params->size_params.hsize * 2) & 0xFFE0;
		dev_err(prev_dev, "\nError in in_pitch; new value = %d",
						params->size_params.in_pitch);
	}

	return 0;
err_einval:
	return -EINVAL;
}

/**
 * preview_isr - Callback from ISP driver for ISP Preview Interrupt
 * @status: ISP IRQ0STATUS register value
 * @arg1: Structure containing ISP preview wrapper global information
 * @arg2: Currently not used
 **/
static void preview_isr(unsigned long status, isp_vbq_callback_ptr arg1,
								void *arg2)
{
	struct prev_device *device = (struct prev_device *)arg1;

	if ((status & PREV_DONE) != PREV_DONE)
		return;

	if (device)
		complete(&device->wfc);
}

/**
 * prev_do_preview - Performs the Preview process
 * @device: Structure containing ISP preview wrapper global information
 * @arg: Currently not used
 *
 * Returns 0 if successful, or -EINVAL if the sent parameters are invalid.
 **/
static int prev_do_preview(struct prev_device *device, int *arg)
{
	int bpp, size;
	int ret = 0;
	u32 out_hsize, out_vsize, out_line_offset;

	dev_dbg(prev_dev, "prev_do_preview E\n");

	if (!device) {
		dev_err(prev_dev, "preview: invalid parameters\n");
		return -EINVAL;
	}

	if (device->params->size_params.pixsize == PREV_INWIDTH_8BIT)
		bpp = 1;
	else
		bpp = 2;

	size = device->params->size_params.hsize *
		device->params->size_params.vsize * bpp;

	ret = isppreview_set_inaddr(device->isp_addr_read);
	if (ret)
		goto out;

	ret = isppreview_set_outaddr(device->isp_addr_read);
	if (ret)
		goto out;

	isppreview_try_size(device->params->size_params.hsize,
					device->params->size_params.vsize,
					&out_hsize, &out_vsize);

	ret = isppreview_config_inlineoffset(device->params->size_params.hsize
						* bpp);
	if (ret)
		goto out;

	out_line_offset = (out_hsize * bpp) & PREV_32BYTES_ALIGN_MASK;

	ret = isppreview_config_outlineoffset(out_line_offset);
	if (ret)
		goto out;

	ret = isppreview_config_size(device->params->size_params.hsize,
					device->params->size_params.vsize,
					out_hsize, out_vsize);
	if (ret)
		goto out;

	isppreview_config_datapath(PRV_RAW_MEM, PREVIEW_MEM);

	ret = isp_set_callback(CBK_PREV_DONE, preview_isr, (void *)device,
								(void *)NULL);
	if (ret) {
		dev_err(prev_dev, "ERROR while setting Previewer callback!\n");
		goto out;
	}
	isppreview_enable(1);

	wait_for_completion_interruptible(&device->wfc);

	if (device->isp_addr_read) {
		ispmmu_unmap(device->isp_addr_read);
		device->isp_addr_read = 0;
	}

	ret = isp_unset_callback(CBK_PREV_DONE);

	dev_dbg(prev_dev, "prev_do_preview L\n");
out:
	return ret;
}

/**
 * previewer_vbq_release - Videobuffer queue release
 * @q: Structure containing the videobuffer queue.
 * @vb: Structure containing the videobuffer used for previewer processing.
 **/
static void previewer_vbq_release(struct videobuf_queue *q,
						struct videobuf_buffer *vb)
{
	struct prev_fh *fh = q->priv_data;
	struct prev_device *device = fh->device;

	ispmmu_unmap(device->isp_addr_read);
	device->isp_addr_read = 0;
	spin_lock(&device->vbq_lock);
	vb->state = VIDEOBUF_NEEDS_INIT;
	spin_unlock(&device->vbq_lock);
	dev_dbg(prev_dev, "previewer_vbq_release\n");
}

/**
 * previewer_vbq_setup - Sets up the videobuffer size and validates count.
 * @q: Structure containing the videobuffer queue.
 * @cnt: Number of buffers requested
 * @size: Size in bytes of the buffer used for previewing
 *
 * Always returns 0.
 **/
static int previewer_vbq_setup(struct videobuf_queue *q,
							unsigned int *cnt,
							unsigned int *size)
{
	struct prev_fh *fh = q->priv_data;
	struct prev_device *device = fh->device;
	u32 bpp = 1;

	spin_lock(&device->vbq_lock);
	if (*cnt <= 0)
		*cnt = VIDEO_MAX_FRAME;

	if (*cnt > VIDEO_MAX_FRAME)
		*cnt = VIDEO_MAX_FRAME;

	if (!device->params->size_params.hsize ||
		!device->params->size_params.vsize) {
		dev_err(prev_dev, "Can't setup buffer size\n");
		spin_unlock(&device->vbq_lock);
		return -EINVAL;
	}

	if (device->params->size_params.pixsize == PREV_INWIDTH_10BIT)
		bpp = 2;
	*size = prev_bufsize = bpp * device->params->size_params.hsize
					* device->params->size_params.vsize;
	spin_unlock(&device->vbq_lock);
	dev_dbg(prev_dev, "previewer_vbq_setup\n");
	return 0;
}

/**
 * previewer_vbq_prepare - Videobuffer is prepared and mmapped.
 * @q: Structure containing the videobuffer queue.
 * @vb: Structure containing the videobuffer used for previewer processing.
 * @field: Type of field to set in videobuffer device.
 *
 * Returns 0 if successful, or -EINVAL if buffer couldn't get allocated, or
 * -EIO if the ISP MMU mapping fails
 **/
static int previewer_vbq_prepare(struct videobuf_queue *q,
						struct videobuf_buffer *vb,
						enum v4l2_field field)
{
	struct prev_fh *fh = q->priv_data;
	struct prev_device *device = fh->device;
	int err = -EINVAL;
	unsigned int isp_addr;
	struct videobuf_dmabuf *dma = videobuf_to_dma(vb);

	dev_dbg(prev_dev, "previewer_vbq_prepare E\n");
	spin_lock(&device->vbq_lock);
	if (vb->baddr) {
		vb->size = prev_bufsize;
		vb->bsize = prev_bufsize;
	} else {
		spin_unlock(&device->vbq_lock);
		dev_err(prev_dev, "No user buffer allocated\n");
		goto out;
	}

	vb->width = device->params->size_params.hsize;
	vb->height = device->params->size_params.vsize;
	vb->field = field;
	spin_unlock(&device->vbq_lock);

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		err = videobuf_iolock(q, vb, NULL);
		if (!err) {
			isp_addr = ispmmu_map_sg(dma->sglist, dma->sglen);
			if (!isp_addr)
				err = -EIO;
			else
				device->isp_addr_read = isp_addr;
		}
	}

	if (!err) {
		vb->state = VIDEOBUF_PREPARED;
		flush_cache_user_range(NULL, vb->baddr,
					(vb->baddr + vb->bsize));
	} else
		previewer_vbq_release(q, vb);

	dev_dbg(prev_dev, "previewer_vbq_prepare L\n");
out:
	return err;
}

static void previewer_vbq_queue(struct videobuf_queue *q,
						struct videobuf_buffer *vb)
{
	return;
}

/**
 * previewer_open - Initializes and opens the Preview Wrapper
 * @inode: Inode structure associated with the Preview Wrapper
 * @filp: File structure associated with the Preview Wrapper
 *
 * Returns 0 if successful, -EACCES if its unable to initialize default config,
 * -EBUSY if its already opened or the ISP module is not available, or -ENOMEM
 * if its unable to allocate the device in kernel space memory.
 **/
static int previewer_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct prev_device *device = prevdevice;
	struct prev_params *config = isppreview_get_config();
	struct prev_fh *fh;

	if (config == NULL) {
		dev_err(prev_dev, "Unable to initialize default config "
			"from isppreviewer\n\n");
		return -EACCES;
	}

	if (device->opened || (filp->f_flags & O_NONBLOCK)) {
		dev_err(prev_dev, "previewer_open: device is already "
								"opened\n");
		return -EBUSY;
	}

	fh = kzalloc(sizeof(struct prev_fh), GFP_KERNEL);
	if (NULL == fh)
		return -ENOMEM;

	isp_get();
	ret = isppreview_request();
	if (ret) {
		isp_put();
		dev_err(prev_dev, "Can't acquire isppreview\n");
		return ret;
	}

	device->params = config;
	device->opened = 1;

	filp->private_data = fh;
	fh->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fh->device = device;

	videobuf_queue_sg_init(&fh->vbq, &device->vbq_ops, NULL,
					&device->vbq_lock, fh->type,
					V4L2_FIELD_NONE,
					sizeof(struct videobuf_buffer), fh);

	init_completion(&device->wfc);
	device->wfc.done = 0;
	mutex_init(&device->prevwrap_mutex);

	return 0;
}

/**
 * previewer_release - Releases Preview Wrapper and frees up allocated memory
 * @inode: Inode structure associated with the Preview Wrapper
 * @filp: File structure associated with the Preview Wrapper
 *
 * Always returns 0.
 **/
static int previewer_release(struct inode *inode, struct file *filp)
{
	struct prev_fh *fh = filp->private_data;
	struct prev_device *device = fh->device;
	struct videobuf_queue *q = &fh->vbq;

	device->opened = 0;
	device->params = NULL;
	isppreview_free();
	videobuf_mmap_free(q);
	isp_put();
	prev_bufsize = 0;
	filp->private_data = NULL;
	kfree(fh);

	dev_dbg(prev_dev, "previewer_release\n");
	return 0;
}

/**
 * previewer_mmap - Memory maps the Preview Wrapper module.
 * @file: File structure associated with the Preview Wrapper
 * @vma: Virtual memory area structure.
 *
 * Returns 0 if successful, or returned value by the videobuf_mmap_mapper()
 * function.
 **/
static int previewer_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct prev_fh *fh = file->private_data;
	dev_dbg(prev_dev, "previewer_mmap\n");

	return videobuf_mmap_mapper(&fh->vbq, vma);
}

/**
 * previewer_ioctl - I/O control function for Preview Wrapper
 * @inode: Inode structure associated with the Preview Wrapper.
 * @file: File structure associated with the Preview Wrapper.
 * @cmd: Type of command to execute.
 * @arg: Argument to send to requested command.
 *
 * Returns 0 if successful, -1 if bad command passed or access is denied,
 * -EFAULT if copy_from_user() or copy_to_user() fails, -EINVAL if parameter
 * validation fails or parameter structure is not present
 **/
static int previewer_ioctl(struct inode *inode, struct file *file,
					unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct prev_params params;
	struct prev_fh *fh = file->private_data;
	struct prev_device *device = fh->device;

	dev_dbg(prev_dev, "Entering previewer_ioctl()\n");

	if ((_IOC_TYPE(cmd) != PREV_IOC_BASE)
					|| (_IOC_NR(cmd) > PREV_IOC_MAXNR)) {
		dev_err(prev_dev, "Bad command Value \n");
		goto err_minusone;
	}

	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	if (ret) {
		dev_err(prev_dev, "access denied\n");
		goto err_minusone;
	}

	switch (cmd) {
	case PREV_REQBUF:
		if (mutex_lock_interruptible(&device->prevwrap_mutex))
			goto err_eintr;
		ret = videobuf_reqbufs(&fh->vbq, (void *)arg);
		mutex_unlock(&device->prevwrap_mutex);
		break;

	case PREV_QUERYBUF:
		if (mutex_lock_interruptible(&device->prevwrap_mutex))
			goto err_eintr;
		ret = videobuf_querybuf(&fh->vbq, (void *)arg);
		mutex_unlock(&device->prevwrap_mutex);
		break;

	case PREV_QUEUEBUF:
		if (mutex_lock_interruptible(&device->prevwrap_mutex))
			goto err_eintr;
		ret = videobuf_qbuf(&fh->vbq, (void *)arg);
		mutex_unlock(&device->prevwrap_mutex);
		break;

	case PREV_SET_PARAM:
		if (mutex_lock_interruptible(&device->prevwrap_mutex))
			goto err_eintr;
		if (copy_from_user(&params, (struct prev_params *)arg,
						sizeof(struct prev_params))) {
			mutex_unlock(&device->prevwrap_mutex);
			return -EFAULT;
		}
		ret = prev_validate_params(&params);
		if (ret < 0) {
			dev_err(prev_dev, "Error validating parameters!\n");
			mutex_unlock(&device->prevwrap_mutex);
			goto out;
		}
		if (device->params)
			memcpy(device->params, &params,
						sizeof(struct prev_params));
		else {
			mutex_unlock(&device->prevwrap_mutex);
			return -EINVAL;
		}

		ret = prev_hw_setup(device->params);
		mutex_unlock(&device->prevwrap_mutex);
		break;

	case PREV_GET_PARAM:
		if (copy_to_user((struct prev_params *)arg, device->params,
						sizeof(struct prev_params)))
			ret = -EFAULT;
		break;

	case PREV_GET_STATUS:
		ret = prev_get_status((struct prev_status *)arg);
		break;

	case PREV_PREVIEW:
		if (mutex_lock_interruptible(&device->prevwrap_mutex))
			goto err_eintr;
		ret = prev_do_preview(device, (int *)arg);
		mutex_unlock(&device->prevwrap_mutex);
		break;

	case PREV_GET_CROPSIZE:
		{
		struct prev_cropsize outputsize;
		prev_calculate_crop(device, &outputsize);
		if (copy_to_user((struct prev_cropsize *)arg, &outputsize,
						sizeof(struct prev_cropsize)))
			ret = -EFAULT;
		}
		break;

	default:
		dev_err(prev_dev, "previewer_ioctl: Invalid Command Value\n");
		ret = -EINVAL;
	}
out:
	return ret;
err_minusone:
	return -1;
err_eintr:
	return -EINTR;
}

/**
 * previewer_platform_release - Acts when Reference count is zero
 * @device: Structure containing ISP preview wrapper global information
 *
 * This is called when the reference count goes to zero
 **/
static void previewer_platform_release(struct device *device)
{
	dev_dbg(prev_dev, "previewer_platform_release()\n");
}

static struct file_operations prev_fops = {
	.owner = THIS_MODULE,
	.open = previewer_open,
	.release = previewer_release,
	.mmap = previewer_mmap,
	.ioctl = previewer_ioctl,
};

static struct platform_device omap_previewer_device = {
	.name = OMAP_PREV_NAME,
	.id = -1,
	.dev = {
		.release = previewer_platform_release,
	}
};

/**
 * previewer_probe - Checks for device presence
 * @pdev: Structure containing details of the current device.
 *
 * Always returns 0
 **/
static int __init previewer_probe(struct platform_device *pdev)
{
	return 0;
}

/**
 * previewer_remove - Handles the removal of the driver
 * @pdev: Structure containing details of the current device.
 *
 * Always returns 0.
 **/
static int previewer_remove(struct platform_device *pdev)
{
	dev_dbg(prev_dev, "previewer_remove()\n");

	platform_device_unregister(&omap_previewer_device);
	platform_driver_unregister(&omap_previewer_driver);
	unregister_chrdev(prev_major, OMAP_PREV_NAME);
	return 0;
}

static struct platform_driver omap_previewer_driver = {
	.probe = previewer_probe,
	.remove = previewer_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = OMAP_PREV_NAME,
	},
};

/**
 * omap_previewer_init - Initialization of Preview Wrapper
 *
 * Returns 0 if successful, -ENOMEM if could not allocate memory, -ENODEV if
 * could not register the wrapper as a character device, or other errors if the
 * device or driver can't register.
 **/
static int __init omap_previewer_init(void)
{
	int ret;
	struct prev_device *device;

	device = kzalloc(sizeof(struct prev_device), GFP_KERNEL);
	if (!device) {
		dev_err(prev_dev, OMAP_PREV_NAME ": could not allocate"
								" memory\n");
		return -ENOMEM;
	}
	prev_major = register_chrdev(0, OMAP_PREV_NAME, &prev_fops);

	if (prev_major < 0) {
		dev_err(prev_dev, OMAP_PREV_NAME ": initialization "
				"failed. could not register character "
				"device\n");
		return -ENODEV;
	}

	ret = platform_driver_register(&omap_previewer_driver);
	if (ret) {
		dev_err(prev_dev, OMAP_PREV_NAME
			": failed to register platform driver!\n");
		goto fail2;
	}
	ret = platform_device_register(&omap_previewer_device);
	if (ret) {
		dev_err(prev_dev, OMAP_PREV_NAME
			": failed to register platform device!\n");
		goto fail3;
	}

	prev_class = class_create(THIS_MODULE, OMAP_PREV_NAME);
	if (!prev_class)
		goto fail4;

	prev_dev = device_create(prev_class, prev_dev,
						(MKDEV(prev_major, 0)), NULL,
						OMAP_PREV_NAME);
	dev_dbg(prev_dev, OMAP_PREV_NAME ": Registered Previewer Wrapper\n");
	device->opened = 0;

	device->vbq_ops.buf_setup = previewer_vbq_setup;
	device->vbq_ops.buf_prepare = previewer_vbq_prepare;
	device->vbq_ops.buf_release = previewer_vbq_release;
	device->vbq_ops.buf_queue = previewer_vbq_queue;
	spin_lock_init(&device->vbq_lock);

	prevdevice = device;
	return 0;

fail4:
	platform_device_unregister(&omap_previewer_device);
fail3:
	platform_driver_unregister(&omap_previewer_driver);
fail2:
	unregister_chrdev(prev_major, OMAP_PREV_NAME);

	return ret;
}

/**
 * omap_previewer_exit - Close of Preview Wrapper
 **/
static void __exit omap_previewer_exit(void)
{
	previewer_remove(&omap_previewer_device);
	kfree(prevdevice);
	prev_major = -1;
}

module_init(omap_previewer_init);
module_exit(omap_previewer_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP ISP Previewer");
MODULE_LICENSE("GPL");
