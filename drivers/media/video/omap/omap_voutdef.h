/*
 * drivers/media/video/omap/omap_voutdef.h
 *
 * Copyright (C) 2005 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef OMAP_VOUTDEF_H
#define OMAP_VOUTDEF_H

#include <linux/omap_vout.h>

#define YUYV_BPP        2
#define RGB565_BPP      2
#define RGB24_BPP       3
#define RGB32_BPP       4
#define TILE_SIZE       32
#define YUYV_VRFB_BPP   2
#define RGB_VRFB_BPP    1

/*
 * This structure is used to store the DMA transfer parameters
 * for VRFB hidden buffer
 */
struct vid_vrfb_dma {
	int dev_id;
	int dma_ch;
	int req_status;
	int tx_status;
	wait_queue_head_t wait;
};

/* per-device data structure */
struct omap_vout_device {
	struct device dev;
	struct video_device *vfd;
	int vid;
	int opened;

	/* Power management suspend lockout stuff */
	int suspended;
	wait_queue_head_t suspend_wq;

#ifdef DEBUG_ALLOW_WRITE
	unsigned long framebuffer_base;
	unsigned long framebuffer_base_phys;
	unsigned long framebuffer_size;
#endif

	/* we don't allow to change image fmt/size once buffer has been allocated */
	int buffer_allocated;
	/* allow to reuse previosuly allocated buffer which is big enough */
	int buffer_size;
	/* keep buffer info accross opens */
	unsigned long buf_virt_addr[VIDEO_MAX_FRAME], buf_phy_addr[VIDEO_MAX_FRAME];
	unsigned int buf_memory_type;

	/* we don't allow to request new buffer when old buffers are still mmaped */
	int mmap_count;

	spinlock_t vbq_lock;		/* spinlock for videobuf queues */
	unsigned long field_count;	/* field counter for videobuf_buffer */

	/* non-NULL means streaming is in progress. */
	struct omap_vout_fh *streaming;

	struct v4l2_pix_format pix;
	struct v4l2_rect crop;
	struct v4l2_window win;
	struct v4l2_framebuffer fbuf;

	/* Lock to protect the shared data structures in ioctl */
	struct semaphore lock;

	/* rotation variablse goes here */
	unsigned long sms_rot_virt[4]; /* virtual addresss for four angles */
					/* four angles */
	dma_addr_t sms_rot_phy[4][4];

	int mirror;
	int rotation;

	int bpp; /* bytes per pixel */
	int vrfb_bpp; /* bytes per pixel with respect to VRFB */
	unsigned int tile_aligned_psize;

	struct vid_vrfb_dma vrfb_dma_tx;
	unsigned int smsshado_phy_addr[4];
	unsigned int smsshado_virt_addr[4];
	unsigned int vrfb_context[4];
	unsigned int smsshado_size;
	unsigned char pos;

	int flicker_filter;
	int ps, vr_ps, line_length, first_int, field_id;
	enum v4l2_memory memory;
	struct videobuf_buffer *curFrm, *nextFrm;
	struct list_head dma_queue;
	u8 *queued_buf_addr[32];
	u32 cropped_offset;
	s32 tv_field1_offset;

};

/* per-filehandle data structure */
struct omap_vout_fh {
	struct omap_vout_device *vout;
	enum v4l2_buf_type type;
	struct videobuf_queue vbq;
	int io_allowed;
};

#ifdef CONFIG_ARCH_OMAP34XX

/*******************************************************************/
/* auxiliary display buffer type */
struct aux_disp_buf {
	int index;
	void * data;
};

struct aux_disp_queue_hdr {
	int queue_depth;
	int queued;
	int dequeued;
	int processed;
	struct aux_disp_buf *aux_disp_queue;
};

/* auxiliary device data structure */
struct omap3_aux_disp_device {
	struct device dev;
	struct video_device *vfd;
	int opened;

	/* Power management suspend lockout stuff */
	int suspended;
	wait_queue_head_t suspend_wq;

#ifdef DEBUG_ALLOW_WRITE
	unsigned long framebuffer_base;
	unsigned long framebuffer_base_phys;
	unsigned long framebuffer_size;
#endif

	/* we don't allow to change image fmt/size once buffer has been allocated */
	int buffer_allocated;
	/* allow to reuse previosuly allocated buffer which is big enough */
	int buffer_size;
	/* keep buffer info accross opens */
	unsigned long buf_virt_addr[VIDEO_MAX_FRAME], buf_phy_addr[VIDEO_MAX_FRAME];
	unsigned int buf_memory_type;

	/* we don't allow to request new buffer when old buffers are still mmaped */
	int mmap_count;

	spinlock_t vbq_lock;		/* spinlock for videobuf queues */
	unsigned long field_count;	/* field counter for videobuf_buffer */

	/* non-NULL means streaming is in progress. */
	struct omap_vout_fh *streaming;

	struct v4l2_pix_format pix;
	struct v4l2_rect crop;
	struct v4l2_window win;
	struct v4l2_framebuffer fbuf;

	int mirror;
	int rotation;

	int bpp; /* bytes per pixel */
	struct aux_disp_queue_hdr	aux_queue_hdr;
};

/* per-filehandle data structure */
struct omap3_aux_disp_fh {
	struct omap3_aux_disp_device *vout;
	enum v4l2_buf_type type;
	struct videobuf_queue vbq;
};
#endif

#endif	/* ifndef OMAP_VOUTDEF_H */
