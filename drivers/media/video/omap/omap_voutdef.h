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

#include <mach/display.h>

#define YUYV_BPP        2
#define RGB565_BPP      2
#define RGB24_BPP       3
#define RGB32_BPP       4
#define TILE_SIZE       32
#define YUYV_VRFB_BPP   2
#define RGB_VRFB_BPP    1
#define MAX_CID			3

/* Rotation using VRFB */
#define SMS_ROT_VIRT_BASE(context, degree)      (0x70000000             \
						| 0x4000000 * (context) \
						| 0x1000000 * (degree/90))
#define SMS_IMAGEHEIGHT_OFFSET                  16
#define SMS_IMAGEWIDTH_OFFSET                   0
#define SMS_PH_OFFSET                           8
#define SMS_PW_OFFSET                           4
#define SMS_PS_OFFSET                           0

#define SMS_PHYS        OMAP343X_SMS_BASE       /* 0x6C000000 */
#define SMS_VIRT        (SMS_PHYS + IO_OFFSET)  /* 0xFC000000 */
#define SMS_SIZE        SZ_1M

#define SDRC_PHYS       OMAP343X_SDRC_BASE      /* 0x6D000000 */
#define SDRC_VIRT       (SDRC_PHYS + IO_OFFSET) /* 0xFD000000 */
#define SDRC_SIZE       SZ_1M

#define GPMC_PHYS       OMAP34XX_GPMC_BASE      /* 0x6E000000 */
#define GPMC_VIRT       (GPMC_PHYS + IO_OFFSET) /* 0xFE000000 */
#define GPMC_SIZE       SZ_1M

#define OMAP_SMS_BASE   SMS_PHYS

#define SMS_CONTEXT_BASE(context)       (OMAP_SMS_BASE + 0x10 * context)
#define SMS_ROT0_PHYSICAL_BA(virt)      (virt + 0x188)
#define SMS_ROT_CONTROL(virt)           (virt + 0x180)
#define SMS_ROT0_SIZE(virt)             (virt + 0x184)

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

struct omapvideo_info {
	int id;
	int num_overlays;
	struct omap_overlay *overlays[3];
	struct omap2video_device *vid_dev;
};

struct omap2video_device {
	struct device *dev;
	struct mutex  mtx;

	int state;

	int num_videos;
	struct omap_vout_device *vouts[10];

	int num_displays;
	struct omap_display *displays[10];
	int num_overlays;
	struct omap_overlay *overlays[10];
	int num_managers;
	struct omap_overlay_manager *managers[10];
};

/* per-device data structure */
struct omap_vout_device {

	struct omapvideo_info vid_info;
	struct device dev;
	struct video_device *vfd;
	int vid;
	int opened;

	/* Power management suspend lockout stuff */
	int suspended;
	wait_queue_head_t suspend_wq;

	/* we don't allow to change image fmt/size once buffer has
	 * been allocated
	 */
	int buffer_allocated;
	/* allow to reuse previosuly allocated buffer which is big enough */
	int buffer_size;
	/* keep buffer info accross opens */
	unsigned long buf_virt_addr[VIDEO_MAX_FRAME];
	unsigned long buf_phy_addr[VIDEO_MAX_FRAME];
	unsigned int buf_memory_type;

	/* we don't allow to request new buffer when old buffers are
	 * still mmaped
	 */
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

	/* V4L2 control structure for different control id */
	struct v4l2_control control[MAX_CID];
	int rotation;
	int mirror;
	int flicker_filter;
	/* V4L2 control structure for different control id */

	int bpp; /* bytes per pixel */
	int vrfb_bpp; /* bytes per pixel with respect to VRFB */

	struct vid_vrfb_dma vrfb_dma_tx;
	unsigned int smsshado_phy_addr[4];
	unsigned int smsshado_virt_addr[4];
	unsigned int vrfb_context[4];
	bool vrfb_static_allocation;
	unsigned int smsshado_size;
	unsigned char pos;
	unsigned int src_chroma_key_enable;
	unsigned int dst_chroma_key_enable;
	unsigned int src_chroma_key;
	unsigned int dst_chroma_key;

	int ps, vr_ps, line_length, first_int, field_id;
	enum v4l2_memory memory;
	struct videobuf_buffer *curFrm, *nextFrm;
	struct list_head dma_queue;
	u8 *queued_buf_addr[32];
	u32 cropped_offset;
	s32 tv_field1_offset;
	void *isr_handle;

};

/* per-filehandle data structure */
struct omap_vout_fh {
	struct omap_vout_device *vout;
	enum v4l2_buf_type type;
	struct videobuf_queue vbq;
	int io_allowed;
};

#endif	/* ifndef OMAP_VOUTDEF_H */
