/*
 * drivers/media/video/omap/isp/omap_resizer.h
 *
 * Include file for Resizer module wrapper in TI's OMAP3430 ISP
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_RESIZER_WRAP_H
#define OMAP_ISP_RESIZER_WRAP_H

/* ioctls definition */
#define RSZ_IOC_BASE			'R'
#define RSZ_IOC_MAXNR			8

/*Ioctl options which are to be passed while calling the ioctl*/
#define RSZ_REQBUF			_IOWR(RSZ_IOC_BASE, 1,\
						struct v4l2_requestbuffers)
#define RSZ_QUERYBUF			_IOWR(RSZ_IOC_BASE, 2,\
						struct v4l2_buffer)
#define RSZ_S_PARAM			_IOWR(RSZ_IOC_BASE, 3,\
						struct rsz_params)
#define RSZ_G_PARAM			_IOWR(RSZ_IOC_BASE, 4,\
						struct rsz_params)
#define RSZ_RESIZE			_IOWR(RSZ_IOC_BASE, 5, int)
#define RSZ_G_STATUS			_IOWR(RSZ_IOC_BASE, 6,\
						struct rsz_status)
#define RSZ_QUEUEBUF			_IOWR(RSZ_IOC_BASE, 7,\
						struct v4l2_buffer)
#define RSZ_GET_CROPSIZE		_IOWR(RSZ_IOC_BASE, 8,\
						struct rsz_cropsize)

/* Defines and Constants*/

#define MAX_CHANNELS			16
#define MAX_IMAGE_WIDTH			2047
#define MAX_IMAGE_WIDTH_HIGH		2047

#define ALIGNMENT			16
#define CHANNEL_BUSY			1
#define CHANNEL_FREE			0
#define PIXEL_EVEN			2
#define RATIO_MULTIPLIER		256

/* Bit position Macro */
/* macro for bit set and clear */
#define BITSET(variable, bit)		(variable) | (1 << bit)
#define BITRESET(variable, bit)		(variable) & ~(0x00000001 << (bit))
#define SET_BIT_INPUTRAM		28
#define SET_BIT_CBLIN			29
#define SET_BIT_INPTYP			27
#define SET_BIT_YCPOS			26
#define INPUT_RAM			1
#define UP_RSZ_RATIO			64
#define DOWN_RSZ_RATIO			512
#define UP_RSZ_RATIO1			513
#define DOWN_RSZ_RATIO1			1024
#define RSZ_IN_SIZE_VERT_SHIFT		16
#define MAX_HORZ_PIXEL_8BIT		31
#define MAX_HORZ_PIXEL_16BIT		15
#define NUM_PHASES			8
#define NUM_TAPS			4
#define NUM_D2PH			4	/* for downsampling
						 * 2+x ~ 4x, number of phases
						 */
#define NUM_D2TAPS			7 	/* for downsampling
						 * 2+x ~ 4x,number of taps
						 */
#define ALIGN32				32
#define MAX_COEF_COUNTER		16
#define COEFF_ADDRESS_OFFSET		0x04

#define RSZ_INTYPE_YCBCR422_16BIT	0
#define RSZ_INTYPE_PLANAR_8BIT		1
#define RSZ_PIX_FMT_UYVY		1	/* cb:y:cr:y */
#define RSZ_PIX_FMT_YUYV		0	/* y:cb:y:cr */

enum config_done {
	STATE_CONFIGURED,			/* Resizer driver configured
						 * by application.
						 */
	STATE_NOT_CONFIGURED			/* Resizer driver not
						 * configured by application.
						 */
};

/* Structure Definitions */

/* used to luma enhancement options */

struct rsz_yenh {
	int type;				/* represents luma enable or
						 * disable.
						 */
	unsigned char gain;			/* represents gain. */
	unsigned char slop;			/* represents slop. */
	unsigned char core;			/* Represents core value. */
};

/* Conatins all the parameters for resizing. This structure
 * is used to configure resiser parameters
 */
struct rsz_params {
	int in_hsize;				/* input frame horizontal
						 * size.
						 */
	int in_vsize;				/* input frame vertical size */
	int in_pitch;				/* offset between two rows of
						 * input frame.
						 */
	int inptyp;				/* for determining 16 bit or
						 * 8 bit data.
						 */
	int vert_starting_pixel;		/* for specifying vertical
						 * starting pixel in input.
						 */
	int horz_starting_pixel;		/* for specyfing horizontal
						 * starting pixel in input.
						 */
	int cbilin;				/* # defined, filter with luma
						 * or bi-linear interpolation.
						 */
	int pix_fmt;				/* # defined, UYVY or YUYV */
	int out_hsize;				/* output frame horizontal
						 * size.
						 */
	int out_vsize;				/* output frame vertical
						 * size.
						 */
	int out_pitch;				/* offset between two rows of
						 * output frame.
						 */
	int hstph;				/* for specifying horizontal
						 * starting phase.
						 */
	int vstph;				/* for specifying vertical
						 * starting phase.
						 */
	u16 tap4filt_coeffs[32];		/* horizontal filter
						 * coefficients.
						 */
	u16 tap7filt_coeffs[32];		/* vertical filter
						 * coefficients.
						 */
	struct rsz_yenh yenh_params;
};

struct rsz_mult {
	int in_hsize;				/* input frame horizontal
						 * size.
						 */
	int in_vsize;				/* input frame vertical size.
						 */
	int out_hsize;				/* output frame horizontal
						 * size.
						 */
	int out_vsize;				/* output frame vertical
						 * size.
						 */
	int in_pitch;				/* offset between two rows of
						 * input frame.
						 */
	int out_pitch;				/* offset between two rows of
						 * output frame.
						 */
	int end_hsize;
	int end_vsize;
	int num_tap;				/* 0 = 7tap; 1 = 4tap */
	int active;
	int inptyp;
	int vrsz;
	int hrsz;
	int hstph;				/* for specifying horizontal
						 * starting phase.
						 */
	int vstph;
	int pix_fmt;				/* # defined, UYVY or YUYV. */
	int cbilin;				/* # defined, filter with luma
						 * or bi-linear.
						 */
	u16 tap4filt_coeffs[32];		/* horizontal filter
						 * coefficients.
						 */
	u16 tap7filt_coeffs[32];		/* vertical filter
						 * coefficients.
						 */
};

/* Contains the status of hardware and channel */
struct rsz_status {
	int chan_busy;				/* 1: channel is busy,
						 * 0: channel is not busy
						 */
	int hw_busy;				/* 1: hardware is busy,
						 * 0: hardware is not busy
						 */
	int src;				/* # defined, can be either
						 * SD-RAM or CCDC/PREVIEWER
						 */
};

/* Passed by application for getting crop size */
struct rsz_cropsize {
	unsigned int hcrop;			/* Number of pixels per line
						 * cropped in output image.
						 */

	unsigned int vcrop;			/* Number of lines cropped
						 * in output image.
						 */
};

/* Register mapped structure which contains the every register
   information */
struct resizer_config {
	u32 rsz_pcr;				/* pcr register mapping
						 * variable.
						 */
	u32 rsz_in_start;			/* in_start register mapping
						 * variable.
						 */
	u32 rsz_in_size;			/* in_size register mapping
						 * variable.
						 */
	u32 rsz_out_size;			/* out_size register mapping
						 * variable.
						 */
	u32 rsz_cnt;				/* rsz_cnt register mapping
						 * variable.
						 */
	u32 rsz_sdr_inadd;			/* sdr_inadd register mapping
						 * variable.
						 */
	u32 rsz_sdr_inoff;			/* sdr_inoff register mapping
						 * variable.
						 */
	u32 rsz_sdr_outadd;			/* sdr_outadd register mapping
						 * variable.
						 */
	u32 rsz_sdr_outoff;			/* sdr_outbuff register
						 * mapping variable.
						 */
	u32 rsz_coeff_horz[16];			/* horizontal coefficients
						 * mapping array.
						 */
	u32 rsz_coeff_vert[16];			/* vertical coefficients
						 * mapping array.
						 */
	u32 rsz_yehn;				/* yehn(luma)register mapping
						 * variable.
						 */
};

/* Channel specific structure contains information regarding
   the every channel */
struct channel_config {
	struct resizer_config register_config;	/* Instance of register set
						 * mapping structure
						 */
	int status;				/* Specifies whether the
						 * channel is busy or not
						 */
	struct mutex chanprotection_mutex;
	enum config_done config_state;

};

/* Global structure which contains information about number of channels
   and protection variables */
struct device_params {

	struct rsz_params *params;
	unsigned char opened;			/* state of the device */
	struct completion compl_isr;		/* Completion for interrupt */
	struct mutex reszwrap_mutex;		/* Semaphore for array */

	struct channel_config *config;		/* Pointer to channel
						 * configuration.
						 */
	spinlock_t vbq_lock;			/* spinlock for videobuf
						 * queues.
						 */
	struct videobuf_queue_ops vbq_ops;	/* videobuf queue operations */
	dma_addr_t isp_addr_read;		/* Input/Output address */
	dma_addr_t isp_addr_write;		/* Input/Output address */
};

/* per-filehandle data structure */
struct rsz_fh {
	enum v4l2_buf_type type;
	struct videobuf_queue vbq;
	struct device_params *device;
};

/* functions definition */
void rsz_hardware_setup(struct channel_config *rsz_conf_chan);
int rsz_set_params(struct rsz_params *, struct channel_config *);
int rsz_get_params(struct rsz_params *, struct channel_config *);
void rsz_copy_data(struct rsz_params *params);
void rsz_isr(unsigned long status, isp_vbq_callback_ptr arg1, void *arg2);
void rsz_calculate_crop(struct channel_config *rsz_conf_chan,
						struct rsz_cropsize *cropsize);
int rsz_set_multipass(struct channel_config *rsz_conf_chan);
int rsz_set_ratio(struct channel_config *rsz_conf_chan);
void rsz_config_ratio(struct channel_config *rsz_conf_chan);

#endif
