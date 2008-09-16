/*
 * drivers/media/video/omap/omap_vout.c
 *
 * Copyright (C) 2005-2006 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Leveraged code from the omap camera driver
 * Video-for-Linux (Version 2) camera capture driver for
 * the OMAP display controller.
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2004 Texas Instruments.
 *
 * History:
 * 20-APR-2006	Khasim		Modified VRFB based Rotation,
 *				The image data is always read from 0 degree
 *				view and written
 *				to the virtual space of desired rotation angle
 * 4-DEC-2006 Jian		Changed to support better memory management
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/smp_lock.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/videodev.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <media/videobuf-dma-sg.h>
#include <linux/input.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-dev.h>

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif
#ifdef CONFIG_DPM
#include <linux/dpm.h>
#endif

#include <asm/arch/omap-dss.h>

#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/irq.h>
#include <asm/semaphore.h>
#include <asm/processor.h>
#include <asm/arch/dma.h>

#include "omap_voutlib.h"

/*
 * Un-comment this to use Debug Write call
 */
/* #define DEBUG_ALLOW_WRITE */

#include "omap_voutdef.h"

unsigned long timeout;

/*
 * Uncomment this if debugging support needs to be enabled
 */

/* #define DEBUG */

#undef DEBUG
#ifdef DEBUG
#define DPRINTK(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DPRINTK( x... )
#endif

/*
 * -1 means rotation support is disabled
 * 0/90/180/270 are initial rotation angles
 */

static int rotation_support = -1;

/* configuration macros */
#define VOUT_NAME		"omap_vout"
#define V1OUT_NAME		"omap_vout1"
#define V2OUT_NAME		"omap_vout2"

#define D1_PAL_WIDTH		720
#define D1_PAL_HEIGHT		576

#define D1_PAL_VSTAT_WIDTH		832
#define D1_PAL_VSTAT_HEIGHT		672

#define D1_NTSC_VSTAT_WIDTH		832
#define D1_NTSC_VSTAT_HEIGHT	560

#define WVGA_WIDTH		854
#define WVGA_HEIGHT		480

#define VGA_WIDTH		640
#define VGA_HEIGHT		480
#define QQVGA_WIDTH		160
#define QQVGA_HEIGHT		120
#define BYTESPERPIXEL		2
#define DMA_CHAN_ALLOTED	1
#define DMA_CHAN_NOT_ALLOTED	0
#define NUM_OF_VIDEO_CHANNELS	2
#define VRF_SIZE		MAX_PIXELS_PER_LINE * MAX_LINES * 4
#define SMS_RGB_PIXSIZE		2
#define SMS_YUYV_PIXSIZE	4
#define VRFB_TX_TIMEOUT		1000

#define VID_MAX_WIDTH		1280/*D1_PAL_WIDTH*/ /* Largest width */
#define VID_MAX_HEIGHT		720/*D1_PAL_HEIGHT*/ /* Largest height */
#define VID_MIN_WIDTH		0
#define VID_MIN_HEIGHT		0
#define OMAP_MAX_BUF_SIZE (VID_MAX_WIDTH*VID_MAX_HEIGHT*4)
#define OMAP_VIDEO1_SMS_START	(0xE0000000)
#define OMAP_VIDEO2_SMS_START	(0xF0000000)

static struct omap_vout_device *saved_v1out, *saved_v2out;

#define STREAMING_IS_ON()	((saved_v1out && saved_v1out->streaming) || \
				(saved_v2out && saved_v2out->streaming))

/*
 * this is the layer being linked to (slave layer). possible values are:
 * OMAP_VIDEO1:  V1 is linked to V2. V1 uses V2's pix and crop.
 * OMAP_VIDEO2:  V2 is linked to V1. V2 uses V1's pix and crop.
 * -1: no link.
 */

static int vout_linked;
static spinlock_t vout_link_lock;

static struct videobuf_queue_ops video_vbq_ops;

static u32 video1_numbuffers = 3;
static u32 video2_numbuffers = 3;
static u32 video1_bufsize = OMAP_MAX_BUF_SIZE;
static u32 video2_bufsize = OMAP_MAX_BUF_SIZE;
module_param(video1_numbuffers, uint, S_IRUGO);
module_param(video2_numbuffers, uint, S_IRUGO);
module_param(video1_bufsize, uint, S_IRUGO);
module_param(video2_bufsize, uint, S_IRUGO);

void omap_vout_isr(void *arg, struct pt_regs *regs, unsigned int irqstatus);
/* module parameters */

/*
 * Maximum amount of memory to use for rendering buffers.
 * Default is enough to four (RGB24) VGA buffers.
 */
#define MAX_ALLOWED_VIDBUFFERS            4
/*static int render_mem = VID_MAX_WIDTH * VID_MAX_HEIGHT * 4 * MAX_ALLOWED_VIDBUFFERS;*/

static struct tvlcd_status_t tvlcd_status;

/* list of image formats supported by OMAP video pipelines */
const static struct v4l2_fmtdesc omap_formats[] = {
	{
		/* Note:  V4L2 defines RGB565 as:
		 *
		 *      Byte 0                    Byte 1
		 *      g2 g1 g0 r4 r3 r2 r1 r0   b4 b3 b2 b1 b0 g5 g4 g3
		 *
		 * We interpret RGB565 as:
		 *
		 *      Byte 0                    Byte 1
		 *      g2 g1 g0 b4 b3 b2 b1 b0   r4 r3 r2 r1 r0 g5 g4 g3
		 */
		.description = "RGB565, le",
		.pixelformat = V4L2_PIX_FMT_RGB565,
	},
	{
		/* Note:  V4L2 defines RGB565X as:
		 *
		 *      Byte 0                    Byte 1
		 *      b4 b3 b2 b1 b0 g5 g4 g3   g2 g1 g0 r4 r3 r2 r1 r0
		 *
		 * We interpret RGB565X as:
		 *
		 *      Byte 0                    Byte 1
		 *      r4 r3 r2 r1 r0 g5 g4 g3   g2 g1 g0 b4 b3 b2 b1 b0
		 */
		.description = "RGB565, be",
		.pixelformat = V4L2_PIX_FMT_RGB565X,
	},
	{
		/* Note:  V4L2 defines RGB32 as: RGB-8-8-8-8  we use
		 *  this for RGB24 unpack mode, the last 8 bits are ignored
		 * */
		.description = "RGB32, le",
		.pixelformat = V4L2_PIX_FMT_RGB32,
	},
	{
		/* Note:  V4L2 defines RGB24 as: RGB-8-8-8  we use
		 *        this for RGB24 packed mode
		 *
		 */
		.description = "RGB24, le",
		.pixelformat = V4L2_PIX_FMT_RGB24,
	},
	{
		.description = "YUYV (YUV 4:2:2), packed",
		.pixelformat = V4L2_PIX_FMT_YUYV,
	},
	{
		.description = "UYVY, packed",
		.pixelformat = V4L2_PIX_FMT_UYVY,
	},
};

#define NUM_OUTPUT_FORMATS (sizeof(omap_formats)/sizeof(omap_formats[0]))

/* CONFIG_PM */
#ifdef CONFIG_PM
#define omap_vout_suspend_lockout(s,f) \
	if ((s)->suspended) {\
		if ((f)->f_flags & O_NONBLOCK)\
			return -EBUSY;\
		wait_event_interruptible((s)->suspend_wq,\
					(s)->suspended == 0);\
	}
#else
#define omap_vout_suspend_lockout(s, f) do {(s)->suspended = 0;} while(0)
#endif

static unsigned long omap_vout_alloc_buffer(u32 buf_size,
		u32 *phys_addr)
{
	unsigned long virt_addr, addr;
	u32 order, size;
	size = PAGE_ALIGN(buf_size);
	order = get_order(size);
	virt_addr = __get_free_pages(GFP_KERNEL | GFP_DMA, order);
	addr = virt_addr;
	if(virt_addr) {
		while (size > 0) {
			SetPageReserved(virt_to_page(addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
	}
	*phys_addr = (u32) virt_to_phys((void*)virt_addr);
	return virt_addr;
}

static void omap_vout_free_buffer(unsigned long virtaddr, u32 phys_addr,
		u32 buf_size)
{
	unsigned long addr = virtaddr;
	u32 order, size;
	size = PAGE_ALIGN(buf_size);
	order = get_order(size);
	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages((unsigned long)virtaddr, order);
}

static int try_format (struct v4l2_pix_format *pix, struct v4l2_pix_format *def_pix)
{
	int ifmt,bpp =0;

	if (pix->width > VID_MAX_WIDTH)
		pix->width = VID_MAX_WIDTH;
	if (pix->height > VID_MAX_HEIGHT)
		pix->height = VID_MAX_HEIGHT;
	if (pix->width <= VID_MIN_WIDTH)
		pix->width = def_pix->width;
	if (pix->height <= VID_MIN_HEIGHT)
		pix->height = def_pix->height;

	for (ifmt = 0; ifmt < NUM_OUTPUT_FORMATS; ifmt++){
		if (pix->pixelformat == omap_formats[ifmt].pixelformat)
			break;
	}

	if (ifmt == NUM_OUTPUT_FORMATS)
		ifmt = 0;

	pix->pixelformat = omap_formats[ifmt].pixelformat;
	pix->field = /*V4L2_FIELD_NONE*/V4L2_FIELD_ANY;
	pix->priv = 0;

	switch (pix->pixelformat){
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_UYVY:
		default:
			pix->colorspace = V4L2_COLORSPACE_JPEG;
			bpp = YUYV_BPP;
			break;
		case V4L2_PIX_FMT_RGB565:
		case V4L2_PIX_FMT_RGB565X:
			pix->colorspace = V4L2_COLORSPACE_SRGB;
			bpp = RGB565_BPP;
			break;
		case V4L2_PIX_FMT_RGB24:
			pix->colorspace = V4L2_COLORSPACE_SRGB;
			bpp = RGB24_BPP;
			break;
		case V4L2_PIX_FMT_RGB32:
		case V4L2_PIX_FMT_BGR32:
			pix->colorspace = V4L2_COLORSPACE_SRGB;
			bpp = RGB32_BPP;
			break;
	}
	pix->bytesperline = pix->width * bpp;
	pix->sizeimage = pix->bytesperline * pix->height;
	return(bpp);
}

/*
 * omap_vout_uservirt_to_phys: This inline function is used to convert user
 * space virtual address to physical address.
 */
static inline u32 omap_vout_uservirt_to_phys(u32 virtp)
{
	unsigned long physp = 0;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;

	/* For kernel direct-mapped memory, take the easy way */
	if (virtp >= PAGE_OFFSET) {
		physp = virt_to_phys((void *)virtp);
	} else if ((vma = find_vma(mm, virtp)) && (vma->vm_flags & VM_IO)
			&& (vma->vm_pgoff)) {
		/* this will catch, kernel-allocated,
		   mmaped-to-usermode addresses */
		physp = (vma->vm_pgoff << PAGE_SHIFT) + (virtp - vma->vm_start);
	} else {
		/* otherwise, use get_user_pages() for general userland pages */
		int res, nr_pages = 1;
		struct page *pages;
		down_read(&current->mm->mmap_sem);

		res = get_user_pages(current, current->mm,
				virtp, nr_pages, 1, 0, &pages, NULL);
		up_read(&current->mm->mmap_sem);

		if (res == nr_pages) {
			physp = __pa(page_address(&pages[0]) +
					(virtp & ~PAGE_MASK));
		} else {
			printk("omap_vout_uservirt_to_phys:\
					get_user_pages failed\n");
			return 0;
		}
	}

	return physp;
}

static void vrfb_dma_tx_callback (int lch, u16 ch_status, void *data)
{
	struct vid_vrfb_dma *t = (struct vid_vrfb_dma *) data;
	t->tx_status = 1;
	wake_up_interruptible (&t->wait);
}

static void omap_vout_sync (struct omap_vout_device *dest,
		   struct omap_vout_device *src)
{
	int rotation = -1;

	if(dest->rotation >= 0)
		rotation = dest->rotation;
	/*
	 * once linked, dest shares src's framebuffer, pix and crop
	 */

	dest->pix = src->pix;
	dest->crop = src->crop;

	if (src->streaming)
		omap_disp_config_vlayer (dest->vid, &dest->pix, &dest->crop,
				&dest->win, rotation, dest->mirror);
}

/* Buffer setup function is called by videobuf layer when REQBUF ioctl is
 * called. This is used to setup buffers and return size and count of
 * buffers allocated. After the call to this buffer, videobuf layer will
 * setup buffer queue depending on the size and count of buffers */
static int omap_vout_buffer_setup(struct videobuf_queue *q, unsigned int *count,
		unsigned int *size)
{
	struct omap_vout_fh *fh = (struct omap_vout_fh *)q->priv_data;
	struct omap_vout_device *vout = fh->vout;
	int startindex = 0, i, j;
	u32 phy_addr = 0, virt_addr = 0;

	if(!vout) {
		return -EINVAL;
	}

	if(V4L2_BUF_TYPE_VIDEO_OUTPUT != q->type)
		return -EINVAL;

        startindex = (vout->vid == OMAP_VIDEO1) ? video1_numbuffers :
                video2_numbuffers;
	if(V4L2_MEMORY_MMAP == vout->memory &&
			*count < startindex)
		*count = startindex;

	if(vout->rotation != -1 && *count > 4)
		*count = 4;

	/* If rotation is enabled, allocate memory for VRFB space also */
	if(vout->rotation >= 0) {
		for(i = 0; i < *count ; i ++) {
			if (!vout->smsshado_virt_addr[i]) {
				vout->smsshado_virt_addr[i] =
					omap_vout_alloc_buffer(
						vout->smsshado_size,
						&vout->smsshado_phy_addr[i]);
			}

			if (!vout->smsshado_virt_addr[i]) {
				if(V4L2_MEMORY_MMAP == vout->memory &&
						i >= startindex)
					break;
				for(j = 0 ; j < i ; j ++) {
					omap_vout_free_buffer(
						vout->smsshado_virt_addr[j],
						vout->smsshado_phy_addr[j],
						vout->smsshado_size);
					vout->smsshado_virt_addr[j]=0;
					vout->smsshado_phy_addr[j]=0;
				}
				*count = 0;
				return -ENOMEM;
			}

			memset((void *) vout->smsshado_virt_addr[i], 0,
					vout->smsshado_size);

			if(vout->rotation == 90 || vout->rotation == 270) {
				omap_disp_set_vrfb (vout->vrfb_context[i],
					vout->smsshado_phy_addr[i],
					vout->pix.height, vout->pix.width,
					vout->bpp * vout->vrfb_bpp);
			} else {
				omap_disp_set_vrfb (vout->vrfb_context[i],
					vout->smsshado_phy_addr[i],
					vout->pix.width, vout->pix.height,
					vout->bpp * vout->vrfb_bpp);
			}
		}
	}

	if(V4L2_MEMORY_MMAP != vout->memory)
		return 0;

	*size = vout->buffer_size;
	startindex = (vout->vid == OMAP_VIDEO1) ? video1_numbuffers :
		video2_numbuffers;
	for(i = startindex ; i < *count ; i ++) {
		vout->buffer_size = *size;

		virt_addr = omap_vout_alloc_buffer(vout->buffer_size,
				&phy_addr);
		if(!virt_addr) {
			if(vout->rotation < 0)
				break;
			for(j = i ; j < *count ; j ++) {
				omap_vout_free_buffer(
					vout->smsshado_virt_addr[j],
					vout->smsshado_phy_addr[j],
					vout->smsshado_size);
				vout->smsshado_virt_addr[j]=0;
				vout->smsshado_phy_addr[j]=0;
			}
			break;
		}

		vout->buf_virt_addr[i] = virt_addr;
		vout->buf_phy_addr[i] = phy_addr;
	}

	*count = vout->buffer_allocated = i;
	return 0;
}

/* This function will be called when VIDIOC_QBUF ioctl is called.
 * It prepare buffers before give out for the display. This function
 * user space virtual address into physical address if userptr memory
 * exchange mechanism is used. If rotation is enabled, it copies entire
 * buffer into VRFB memory space before giving it to the DSS. */
static int omap_vout_buffer_prepare(struct videobuf_queue *q,
		struct videobuf_buffer *vb, enum v4l2_field field)
{
	struct omap_vout_fh *fh = (struct omap_vout_fh *)q->priv_data;
	struct omap_vout_device *vout = fh->vout;
	u32 dest_frame_index = 0, src_element_index = 0;
	u32 dest_element_index = 0, src_frame_index = 0;
	u32 elem_count = 0, frame_count = 0, pixsize = 2,mir_rot_deg = 0;
	struct videobuf_dmabuf *dmabuf=NULL;

	if (VIDEOBUF_NEEDS_INIT == vb->state) {
		vb->width = vout->pix.width;
		vb->height = vout->pix.height;
		vb->size = vb->width * vb->height * vout->bpp;
		vb->field = field;
	}
	vb->state = VIDEOBUF_PREPARED;
	/* if user pointer memory mechanism is used, get the physical
	 * address of the buffer
	 */
	if (V4L2_MEMORY_USERPTR == vb->memory) {
		if (0 == vb->baddr) {
			return -EINVAL;
		}
		/* Virtual address */
		/* priv points to struct videobuf_pci_sg_memory. But we went
		 * pointer to videobuf_dmabuf, which is member of
		 * videobuf_pci_sg_memory */
		dmabuf = videobuf_to_dma(q->bufs[vb->i]);
		dmabuf->vmalloc = (void*)vb->baddr;

		/* Physical address */
		dmabuf->bus_addr = (dma_addr_t)omap_vout_uservirt_to_phys(
				vb->baddr);
	}

	if (vout->rotation >= 0) {
		dmabuf = videobuf_to_dma(q->bufs[vb->i]);

		/* If rotation is enabled, copy input buffer into VRFB
		 * memory space using DMA. We are copying input buffer
		 * into VRFB memory space of desired angle and DSS will
		 * read image VRFB memory for 0 degree angle */
		pixsize = vout->bpp * vout->vrfb_bpp;
		/*
		 * DMA transfer in double index mode
		 */

		/* Frame index */
		dest_frame_index = ((MAX_PIXELS_PER_LINE * pixsize) -
				(vout->pix.width * vout->bpp)) + 1;

		/* Source and destination parameters */
		src_element_index = 0;
		src_frame_index = 0;
		dest_element_index = 1;

		/* Number of elements per frame */
		elem_count = vout->pix.width * vout->bpp;
		frame_count = vout->pix.height;
		vout->vrfb_dma_tx.tx_status = 0;
		omap_set_dma_transfer_params (vout->vrfb_dma_tx.dma_ch,
			OMAP_DMA_DATA_TYPE_S32, (elem_count / 4), frame_count,
			OMAP_DMA_SYNC_ELEMENT,vout->vrfb_dma_tx.dev_id, 0x0);

		omap_set_dma_src_params (vout->vrfb_dma_tx.dma_ch,
			0,	/* src_port required only for OMAP1 */
			OMAP_DMA_AMODE_POST_INC,
			dmabuf->bus_addr,
			src_element_index, src_frame_index);

		/*set dma source burst mode for VRFB*/
		omap_set_dma_src_burst_mode(vout->vrfb_dma_tx.dma_ch,
			OMAP_DMA_DATA_BURST_16);

		if (vout->mirror == 1) {
			/* Following is used to select appropriate VRFB
			 * memory space for rotation with mirroring */
			mir_rot_deg = (vout->rotation == 90) ? (270 / 90):
				(vout->rotation == 270) ? (90 / 90):
				(vout->rotation == 180) ? (0 / 90) : (180 / 90);

			omap_set_dma_dest_params (vout->vrfb_dma_tx.dma_ch,
				0,	/* dest_port required only for OMAP1 */
				OMAP_DMA_AMODE_DOUBLE_IDX,
				vout->sms_rot_phy[vb->i][mir_rot_deg],
				dest_element_index, dest_frame_index);
		} else { /* No Mirroring */
			omap_set_dma_dest_params (vout->vrfb_dma_tx.dma_ch,
				0,	/* dest_port required only for OMAP1 */
				OMAP_DMA_AMODE_DOUBLE_IDX,
				vout->sms_rot_phy[vb->i][vout->rotation/90],
				dest_element_index, dest_frame_index);
		}

		/*set dma dest burst mode for VRFB*/
		omap_set_dma_dest_burst_mode(vout->vrfb_dma_tx.dma_ch,
				OMAP_DMA_DATA_BURST_16);
		omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0x20, 0);

		omap_start_dma (vout->vrfb_dma_tx.dma_ch);
		interruptible_sleep_on_timeout (&vout->vrfb_dma_tx.wait,
				VRFB_TX_TIMEOUT);

		if (vout->vrfb_dma_tx.tx_status == 0) {
			omap_stop_dma (vout->vrfb_dma_tx.dma_ch);
			return -EINVAL;
		}
		/* Store buffers physical address into an array. Addresses
		 * from this array will be used to configure DSS */
		vout->queued_buf_addr[vb->i] = (u8*)vout->sms_rot_phy[vb->i][0];
	} else {
		dmabuf = videobuf_to_dma(q->bufs[vb->i]);

		vout->queued_buf_addr[vb->i] = (u8*)dmabuf->bus_addr;
	}
	return 0;
}

/* Buffer queue funtion will be called from the videobuf layer when _QBUF
 * ioctl is called. It is used to enqueue buffer, which is ready to be
 * displayed. */
static void omap_vout_buffer_queue(struct videobuf_queue *q,
					struct videobuf_buffer *vb)
{
	struct omap_vout_fh *fh = (struct omap_vout_fh *)q->priv_data;
	struct omap_vout_device *vout = fh->vout;

	/* Driver is also maintainig a queue. So enqueue buffer in the driver
	 * queue */
	list_add_tail(&vb->queue, &vout->dma_queue);

	vb->state = VIDEOBUF_PREPARED;
}

/* Buffer release function is called from videobuf layer to release buffer
 * which are already allocated */
static void omap_vout_buffer_release(struct videobuf_queue *q,
					struct videobuf_buffer *vb)
{
	struct omap_vout_fh *fh = (struct omap_vout_fh *)q->priv_data;
	struct omap_vout_device *vout = fh->vout;

	vb->state = VIDEOBUF_NEEDS_INIT;

	if(V4L2_MEMORY_MMAP != vout->memory)
		return;
}

static int omap_vout_calculate_offset(struct omap_vout_device *vout)
{
	struct v4l2_pix_format *pix = &(vout->pix);
	struct v4l2_rect *crop = &(vout->crop);
	struct v4l2_window *win = &(vout->win);
	int rotation_deg;
	int mirroring = vout->mirror;
	int vr_ps = 1, ps = 2, temp_ps = 2;
	int offset=0, ctop=0, cleft=0, line_length=0;
	int *cropped_offset = &(vout->cropped_offset);

	if ((omap_disp_get_output_dev(vout->vid) == OMAP_OUTPUT_TV) &&
			((win->w.width == crop->width) &&
			 (win->w.height == crop->height)))
		vout->flicker_filter = 1;
	else
		vout->flicker_filter = 0;

	if(1 == vout->mirror && vout->rotation >= 0) {
		rotation_deg = (vout->rotation == 90)? 270:
			(vout->rotation == 270)? 90:
			(vout->rotation == 180)? 0:180;

	} else if(vout->rotation >= 0){
		rotation_deg = vout->rotation;
	} else {
		rotation_deg = -1;
	}

	if (V4L2_PIX_FMT_YUYV == pix->pixelformat ||
			V4L2_PIX_FMT_UYVY == pix->pixelformat) {
		if (rotation_deg >= 0 || mirroring == 1){
			/*
			 * ps    - Actual pixel size for YUYV/UYVY for
			 *              VRFB/Mirroring is 4 bytes
			 * vr_ps - Virtually pixel size for YUYV/UYVY is
			 *              2 bytes
			 */
			ps    = 4;
			vr_ps = 2;
		} else {
			ps = 2; /* otherwise the pixel size is 2 byte */
		}
	} else if(V4L2_PIX_FMT_RGB32 == pix->pixelformat) {
		ps = 4;
	} else if(V4L2_PIX_FMT_RGB24 == pix->pixelformat) {
		ps = 3;
	}
	vout->ps = ps;
	vout->vr_ps = vr_ps;
	if (rotation_deg >= 0) {
		line_length = MAX_PIXELS_PER_LINE;
		ctop  = (pix->height - crop->height) - crop->top;
		cleft = (pix->width - crop->width) - crop->left;
	} else {
		line_length = pix->width;
	}
	vout->line_length = line_length;
	switch (rotation_deg) {
		case 90:
			offset = (omap_disp_get_vrfb_offset(pix->width, ps,
					SIDE_H) - (pix->width/vr_ps)) * ps *
				line_length;
			temp_ps = ps/vr_ps;
			if (mirroring == 0) {
				*cropped_offset = offset + line_length *
					temp_ps * cleft + crop->top * temp_ps;
			} else {
				*cropped_offset = offset + line_length *
					temp_ps * cleft + crop->top *
					temp_ps + (line_length *
					( (crop->width/(vr_ps)) - 1) * ps);
			}
			break;

		case 180:
			offset = (omap_disp_get_vrfb_offset(pix->height, ps,
					SIDE_H) - pix->height) * ps *
				line_length +
				(omap_disp_get_vrfb_offset(pix->width, ps,
				    SIDE_W) - (pix->width/vr_ps)) * ps;
			if (mirroring == 0) {
				*cropped_offset = offset + (line_length * ps *
					ctop) + (cleft/vr_ps) * ps;
			} else {
				*cropped_offset = offset + (line_length * ps *
					ctop) + (cleft/vr_ps) * ps +
					(line_length * (crop->height -
					1) * ps);
			}
			break;

		case 270:
			offset = (omap_disp_get_vrfb_offset(pix->height, ps,
					SIDE_W) - pix->height) * ps;
			temp_ps = ps/vr_ps;
			if (mirroring == 0) {
				*cropped_offset = offset + line_length *
					temp_ps * crop->left + ctop * ps;
			} else {
				*cropped_offset = offset + line_length *
					temp_ps * crop->left + ctop * ps +
					(line_length * ((crop->width/vr_ps) -
							1) * ps);
			}
			break;
		case 0:
			if (mirroring == 0) {
				*cropped_offset = (line_length * ps) *
					crop->top + (crop->left/vr_ps) * ps;
			} else {
				*cropped_offset = (line_length * ps) *
					crop->top + (crop->left/vr_ps) * ps +
					(line_length * (crop->height - 1) * ps);
			}
			break;
		default:
			if (mirroring == 0) {
				*cropped_offset = line_length * ps *
					crop->top + crop->left * ps;
			} else {
				*cropped_offset = (line_length * ps *
						crop->top) / vr_ps +
					(crop->left * ps) / vr_ps +
					((crop->width / vr_ps) - 1) * ps;
			}
			break;
	}

	if(vout->flicker_filter == 1)
		vout->tv_field1_offset = 0;
	else if(vout->rotation >= 0) {
		if(vout->mirror == 1)
			vout->tv_field1_offset = -vout->line_length * vout->ps;
		else
			vout->tv_field1_offset = vout->line_length * vout->ps;
	} else {
		if(vout->mirror == 1)
			vout->tv_field1_offset = vout->line_length *
				vout->ps/vout->vr_ps;
		else
			vout->tv_field1_offset = vout->line_length * vout->ps;
	}
	return 0;
}

static int
omap_vout_do_ioctl (struct inode *inode, struct file *file,
		       unsigned int cmd, void *arg)
{
	struct omap_vout_fh *fh =
		(struct omap_vout_fh *)file->private_data;
	struct omap_vout_device *vout = fh->vout;
	int err;

	switch (cmd){
	case VIDIOC_ENUMOUTPUT:
	{
		struct v4l2_output *output = (struct v4l2_output *) arg;
		int index = output->index;

		if (index > 0)
			return -EINVAL;

		memset (output, 0, sizeof (*output));
		output->index = index;

		strncpy (output->name, "video out", sizeof (
						output->name));
		output->type = V4L2_OUTPUT_TYPE_MODULATOR;
		return 0;
	}

	case VIDIOC_G_OUTPUT:
	{
		unsigned int *output = arg;
		*output = 0;
		return 0;
	}

	case VIDIOC_S_OUTPUT:
	{
		unsigned int *output = arg;
		if (*output > 0)
			return -EINVAL;
		return 0;
	}

	case VIDIOC_QUERYCAP:
	{
		struct v4l2_capability *cap =
			(struct v4l2_capability *) arg;
		memset (cap, 0, sizeof (*cap));
		strncpy (cap->driver, VOUT_NAME, sizeof (cap->driver));
		strncpy (cap->card, vout->vfd->name,
				sizeof (cap->card));
		cap->bus_info[0] = '\0';
		cap->capabilities =
			V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;
#ifdef DEBUG_ALLOW_WRITE
		cap->capabilities |= V4L2_CAP_READWRITE;
#endif
		return 0;
	}

	case VIDIOC_ENUM_FMT:
	{
		struct v4l2_fmtdesc *fmt = arg;
		int index = fmt->index;
		enum v4l2_buf_type type = fmt->type;
		memset (fmt, 0, sizeof (*fmt));
		fmt->index = index;
		fmt->type = type;

		switch (fmt->type){
			case V4L2_BUF_TYPE_VIDEO_OUTPUT:
			case V4L2_BUF_TYPE_VIDEO_OVERLAY:
				if (index >= NUM_OUTPUT_FORMATS)
					return -EINVAL;
				break;
			default:
				return -EINVAL;
		}

		fmt->flags = omap_formats[index].flags;
		strncpy (fmt->description,
				omap_formats[index].description,
				sizeof (fmt->description));
		fmt->pixelformat = omap_formats[index].pixelformat;

		return 0;
	}

	case VIDIOC_G_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *) arg;

		switch (f->type){
			case V4L2_BUF_TYPE_VIDEO_OUTPUT:
			{
				struct v4l2_pix_format *pix =
					&f->fmt.pix;
				memset (pix, 0, sizeof (*pix));
				*pix = vout->pix;
				return 0;
			}

			case V4L2_BUF_TYPE_VIDEO_OVERLAY:
			{
				struct v4l2_window *win = &f->fmt.win;
				memset (win, 0, sizeof (*win));

				/*
				 * The API has a bit of a problem here.
				 * We're returning a v4l2_window
				 * structure, but that structure
				 * contains pointers to variable-sized
				 * objects for clipping rectangles and
				 * clipping bitmaps.  We will just
				 * return NULLs for those pointers.
				 */

				win->w = vout->win.w;
				win->field = vout->win.field;
				win->chromakey = vout->win.chromakey;
				return 0;
			}

			default:
				return -EINVAL;
		}
	}

	case VIDIOC_TRY_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *) arg;

		if (vout->streaming)
			return -EBUSY;

		/* We dont support RGB24-packed mode if vrfb rotation
		 * is enabled*/
		if(vout->rotation != -1 && f->fmt.pix.pixelformat ==
				V4L2_PIX_FMT_RGB24)
			return -EINVAL;

		switch (f->type) {
			case V4L2_BUF_TYPE_VIDEO_OVERLAY:
			{
				struct v4l2_window *win = &f->fmt.win;
				err = omap_vout_try_window (
						&vout->fbuf, win);
				return err;
			}

			case V4L2_BUF_TYPE_VIDEO_OUTPUT:
			{
				/* don't allow to change img for the
				 * linked layer */
				if (vout->vid == vout_linked)
					return -EINVAL;
			omap_disp_get_dss();
			/* get the framebuffer parameters */
			if (vout->rotation == 90 || vout->rotation == 270) {
				omap_disp_get_panel_size (
					omap_disp_get_output_dev (vout->vid),
					&(vout->fbuf.fmt.height),
					&(vout->fbuf.fmt.width));
			} else {
			omap_disp_get_panel_size (
				omap_disp_get_output_dev (vout->vid),
					&(vout->fbuf.fmt.width),
					&(vout->fbuf.fmt.height));
			}
			omap_disp_put_dss();
			try_format (&f->fmt.pix, &vout->fbuf.fmt);
				return 0;
			}

			default:
				return -EINVAL;
		}
	}

	case VIDIOC_S_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *) arg;

		if (vout->streaming)
			return -EBUSY;

		if (down_interruptible(&vout->lock))
			return -EBUSY;
		/* We dont support RGB24-packed mode if vrfb rotation
		 * is enabled*/
		if(vout->rotation != -1 && f->fmt.pix.pixelformat ==
				V4L2_PIX_FMT_RGB24 )
		{
			up(&vout->lock);
			return -EINVAL;
		}

		omap_disp_get_dss();

		/* get the framebuffer parameters */
		if(vout->rotation == 90 || vout->rotation == 270){
			omap_disp_get_panel_size (
					omap_disp_get_output_dev (vout->vid),
					&(vout->fbuf.fmt.height),
					&(vout->fbuf.fmt.width));
		} else {
			omap_disp_get_panel_size (
					omap_disp_get_output_dev (vout->vid),
					&(vout->fbuf.fmt.width),
					&(vout->fbuf.fmt.height));
		}

		omap_disp_put_dss();

		switch (f->type) {
		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		{
			struct v4l2_window *win = &f->fmt.win;
			err = omap_vout_new_window (
					&vout->crop, &vout->win, &vout->fbuf,
					win);
			up(&vout->lock);
			return err;
		}

		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		{
			int bpp;
			/*
			 * don't allow to change img for
			 * the linked layer */
			if (vout->vid == vout_linked) {
				up(&vout->lock);
				return -EINVAL;
			}
			/* change to samller size is OK */
			bpp = try_format (&f->fmt.pix, &vout->fbuf.fmt);
			f->fmt.pix.sizeimage = f->fmt.pix.width *
				f->fmt.pix.height *bpp;

			/* try & set the new output format */
			vout->bpp = bpp;
			vout->pix = f->fmt.pix;
			vout->vrfb_bpp = 1;
			/* If YUYV then vrfb bpp is 2, for
			 * others its 1*/
			if (V4L2_PIX_FMT_YUYV == vout->pix.pixelformat
					|| V4L2_PIX_FMT_UYVY ==
					vout->pix.pixelformat)
				vout->vrfb_bpp = 2;

			/* set default crop and win */
			omap_vout_new_format (&vout->pix, &vout->fbuf,
					&vout->crop, &vout->win);
			up(&vout->lock);
			return 0;
		}

		default:
			up(&vout->lock);
			return -EINVAL;
		}
	}

	case VIDIOC_CROPCAP:
	{
		struct v4l2_cropcap *cropcap =
			(struct v4l2_cropcap *) arg;
		enum v4l2_buf_type type = cropcap->type;

		memset (cropcap, 0, sizeof (*cropcap));
		cropcap->type = type;
		switch (type) {
		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		{
			struct v4l2_pix_format *pix = &vout->pix;

			/* Width and height are always even */
			cropcap->bounds.width = pix->width & ~1;
			cropcap->bounds.height = pix->height & ~1;

			omap_vout_default_crop (&vout->pix,
				&vout->fbuf, &cropcap->defrect);
			cropcap->pixelaspect.numerator = 1;
			cropcap->pixelaspect.denominator = 1;
			return 0;
		}

		default:
			return -EINVAL;
		}
	}

	case VIDIOC_G_CROP:
	{
		struct v4l2_crop *crop = (struct v4l2_crop *) arg;

		switch (crop->type) {
			case V4L2_BUF_TYPE_VIDEO_OUTPUT:
			{
				crop->c = vout->crop;
				return 0;
			}
			default:
				return -EINVAL;
		}
	}

	case VIDIOC_S_CROP:
	{
		struct v4l2_crop *crop = (struct v4l2_crop *) arg;
		if (vout->streaming)
			return -EBUSY;

		if (down_interruptible(&vout->lock))
			return -EBUSY;
		omap_disp_get_dss();

		/* get the framebuffer parameters */
		if(vout->rotation == 90 || vout->rotation == 270){
			omap_disp_get_panel_size (
				omap_disp_get_output_dev (vout->vid),
					&(vout->fbuf.fmt.height),
					&(vout->fbuf.fmt.width));
		} else {
			omap_disp_get_panel_size (
				omap_disp_get_output_dev (vout->vid),
					&(vout->fbuf.fmt.width),
					&(vout->fbuf.fmt.height));

		}
		omap_disp_put_dss();

		switch (crop->type) {
			case V4L2_BUF_TYPE_VIDEO_OUTPUT:
			{
				err = omap_vout_new_crop (&vout->pix,
					&vout->crop, &vout->win, &vout->fbuf,
					&crop->c);
				up(&vout->lock);
				return err;
			}
			default:
				up(&vout->lock);
				return -EINVAL;
		}
	}

	case VIDIOC_REQBUFS:
	{
		struct v4l2_requestbuffers *req =
			(struct v4l2_requestbuffers *) arg;
		struct videobuf_queue *q = &fh->vbq;
		unsigned int i, num_buffers=0;
		int ret = 0;
		struct videobuf_dmabuf *dmabuf=NULL;

		if (down_interruptible(&vout->lock))
			return -EBUSY;
		/* don't allow to buffer request for the linked layer */
		if (vout->vid == vout_linked) {
			up(&vout->lock);
			return -EINVAL;
		}

		if ((req->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) ||
				(req->count < 0)) {
			up(&vout->lock);
			return -EINVAL;
		}
		/* if memory is not mmp or userptr
		   return error */
		if ((V4L2_MEMORY_MMAP != req->memory) &&
				(V4L2_MEMORY_USERPTR != req->memory)) {
			up(&vout->lock);
			return -EINVAL;
		}

		/* Cannot be requested when streaming is on */
		if (vout->streaming) {
			up(&vout->lock);
			return -EBUSY;
		}

		/* If buffers are already allocated free them */
		if (q->bufs[0] && (V4L2_MEMORY_MMAP == q->bufs[0]->memory)) {
			if (vout->mmap_count) {
				up(&vout->lock);
				return -EBUSY;
			}
			num_buffers = (vout->vid == OMAP_VIDEO1) ?
				video1_numbuffers : video2_numbuffers;
			for(i = num_buffers;i < vout->buffer_allocated;i++) {
				dmabuf = videobuf_to_dma(q->bufs[i]);
				omap_vout_free_buffer((u32)dmabuf->vmalloc,
					dmabuf->bus_addr,
					vout->buffer_size);
				vout->buf_virt_addr[i] = 0;
				vout->buf_phy_addr[i] = 0;
			}
			vout->buffer_allocated = num_buffers;
			videobuf_mmap_free(q);
		} else if(q->bufs[0] && (V4L2_MEMORY_USERPTR ==
					q->bufs[0]->memory)) {
			if(vout->buffer_allocated) {
				videobuf_mmap_free(q);
				for(i = 0 ; i < vout->buffer_allocated ; i ++){
					if(q->bufs[i])
						kfree(q->bufs[i]);
					q->bufs[i] = NULL;
				}
				vout->buffer_allocated = 0;
			}
		}
		fh->io_allowed = 1;

		/*store the memory type in data structure*/
		vout->memory = req->memory;

		INIT_LIST_HEAD(&vout->dma_queue);

		/* call videobuf_reqbufs api */
		ret = videobuf_reqbufs(q, req);
		if(ret < 0) {
			up(&vout->lock);
			return ret;
		}

		vout->buffer_allocated = req->count;
		for(i = 0 ; i < req->count ; i ++) {
			dmabuf = videobuf_to_dma(q->bufs[i]);
			dmabuf->vmalloc =
				(void *)vout->buf_virt_addr[i];
			dmabuf->bus_addr =
				(dma_addr_t)vout->buf_phy_addr[i];

			dmabuf->sglen = 1;
		}
		up(&vout->lock);
		return 0;
	}

	case VIDIOC_QUERYBUF:
		return videobuf_querybuf (&fh->vbq, arg);

	case VIDIOC_QBUF:
	{
		struct v4l2_buffer *buffer = (struct v4l2_buffer *) arg;
		struct videobuf_queue *q = &fh->vbq;
		struct omap_vout_device *dest;
		int output_dev = omap_disp_get_output_dev(vout->vid);
		int streaming_on = STREAMING_IS_ON(), ret=0, rotation=-1;

		if(!fh->io_allowed) {
			return -EINVAL;
		}

		timeout = HZ / 5;
		timeout += jiffies;

		omap_disp_get_tvlcd(&tvlcd_status);

		if (!streaming_on)
			omap_disp_get_dss();

		if (tvlcd_status.status == TVLCD_STOP) {
			if (tvlcd_status.ltype == vout->vid) {
				omap_disp_disable_layer (vout->vid);
				vout->streaming = NULL;
			} else if (vout_linked != -1 &&
					vout_linked != vout->vid){
				omap_disp_disable_layer (
					(vout->vid == OMAP_VIDEO1)
					? OMAP_VIDEO2 : OMAP_VIDEO1);
			}

			while(omap_disp_reg_sync_bit(output_dev ) &&
					time_before(jiffies, timeout)) {
				if ((!in_interrupt()) && (!irqs_disabled())) {
					set_current_state(TASK_INTERRUPTIBLE);
					schedule_timeout(1);
				} else {
					udelay(10);
				}
			}
			if(vout->rotation == 90 || vout->rotation == 270){
				omap_disp_get_panel_size(
					omap_disp_get_output_dev (vout->vid),
						&(vout->fbuf.fmt.height),
						&(vout->fbuf.fmt.width));
			} else {
				omap_disp_get_panel_size(
					omap_disp_get_output_dev (vout->vid),
						&(vout->fbuf.fmt.width),
						&(vout->fbuf.fmt.height));
			}
			/* set default crop and win */
			omap_vout_new_format (&vout->pix, &vout->fbuf,
					&vout->crop, &vout->win);
			omap_disp_set_output_dev(tvlcd_status.ltype,
					tvlcd_status.output_dev);
			omap_disp_set_tvlcd(TVLCD_CONTINUE);
			if (!streaming_on)
				omap_disp_put_dss();

			omap_disp_get_tvlcd(&tvlcd_status);
		}

		if (tvlcd_status.status == TVLCD_CONTINUE) {
			if (tvlcd_status.ltype == vout->vid) {
				rotation = (vout->rotation >= 0) ?
					vout->rotation:-1;
				vout->streaming = fh;
				omap_disp_config_vlayer(vout->vid,
					&vout->pix, &vout->crop, &vout->win,
					rotation, vout->mirror);
			} else if (vout_linked != -1 && vout_linked !=
						vout->vid) {
				dest = (vout_linked == OMAP_VIDEO1) ?
					saved_v1out : saved_v2out;
				rotation = (dest->rotation >= 0) ?
					dest->rotation:-1;
				omap_disp_config_vlayer (dest->vid,
					&dest->pix, &dest-> crop, &dest->win,
					rotation, dest->mirror);
			}

			omap_disp_set_tvlcd(0);
		}
		/* don't allow to queue buffer for the linked layer */
		if (vout->vid == vout_linked) {
			if (!streaming_on)
				omap_disp_put_dss();
			return -EINVAL;
		}

		if ((V4L2_BUF_TYPE_VIDEO_OUTPUT != buffer->type) ||
				(buffer->index >= vout->buffer_allocated) ||
			(q->bufs[buffer->index]->memory != buffer->memory)) {
			if (!streaming_on)
				omap_disp_put_dss();
			return -EINVAL;
		}
		if(V4L2_MEMORY_USERPTR == buffer->memory) {
			if((buffer->length < vout->pix.sizeimage) ||
					(0 == buffer->m.userptr)) {
				if (!streaming_on)
						omap_disp_put_dss();
				return -EINVAL;
			}
		}

		if (vout->rotation >= 0 && vout->vrfb_dma_tx.req_status ==
				DMA_CHAN_NOT_ALLOTED) {
			if (!streaming_on)
				omap_disp_put_dss();
			return -EINVAL;
		}

		ret = videobuf_qbuf(q, buffer);
		if (!streaming_on)
			omap_disp_put_dss();
		return ret;
	}

	case VIDIOC_DQBUF:
	{
		struct videobuf_queue *q = &fh->vbq;
		int ret = 0;
		/* don't allow to dequeue buffer for the linked layer */
		if (vout->vid == vout_linked)
			return -EINVAL;

		if(!vout->streaming || !fh->io_allowed)
			return -EINVAL;

		if (file->f_flags & O_NONBLOCK)
			/* Call videobuf_dqbuf for non
			   blocking mode */
			ret = videobuf_dqbuf(q, (struct v4l2_buffer *)arg, 1);
		else
			/* Call videobuf_dqbuf for
			   blocking mode */
			ret = videobuf_dqbuf(q, (struct v4l2_buffer *)arg, 0);
		return ret;
	}

	case VIDIOC_STREAMON:
	{
		struct videobuf_queue *q = &fh->vbq;
		struct omap_vout_device *dest=NULL;
		struct videobuf_dmabuf *dmabuf=NULL;
		int ret=0, rotation=-1, mask=0;
		u32 addr=0;

		if (down_interruptible(&vout->lock))
			return -EBUSY;
		if(!fh->io_allowed) {
			up(&vout->lock);
			return -EINVAL;
		}

		if(vout->streaming) {
			up(&vout->lock);
			return -EBUSY;
		}

		ret = videobuf_streamon(q);
		if(ret < 0) {
			up(&vout->lock);
			return ret;
		}

		if(list_empty(&vout->dma_queue)) {
			up(&vout->lock);
			return -EIO;
		}
		/* Get the next frame from the buffer queue */
		vout->nextFrm = vout->curFrm =
			list_entry(vout->dma_queue.next,
					struct videobuf_buffer, queue);
		/* Remove buffer from the buffer queue */
		list_del(&vout->curFrm->queue);
		/* Mark state of the current frame to active */
		vout->curFrm->state = VIDEOBUF_ACTIVE;
		/* Initialize field_id and started member */
		vout->field_id = 0;

		/* set flag here. Next QBUF will start DMA */
		vout->streaming = fh;

		vout->first_int = 1;

		omap_disp_get_dss();

		rotation = (vout->rotation >= 0)?vout->rotation:-1;

		omap_disp_config_vlayer (vout->vid, &vout->pix, &vout->crop,
				&vout->win, rotation, vout->mirror);

		omap_vout_calculate_offset(vout);

		addr = (unsigned long)vout->queued_buf_addr[vout->curFrm->i] +
			vout->cropped_offset;
		omap_disp_set_addr(vout->vid, addr, addr,
				addr+vout->tv_field1_offset);

		/* Configure also linked layer */
		if (vout_linked != -1 && vout_linked != vout->vid){
			dest = (vout_linked == OMAP_VIDEO1) ? saved_v1out :
				saved_v2out;
			rotation = (dest->rotation >= 0)?dest->rotation:-1;
			if(rotation > -1) {
				if(!(vout->rotation > -1)){
					printk("Rotation should be enabled \
							in linking layer\n");
					up(&vout->lock);
					return -EINVAL;
				}
				if(rotation != vout->rotation){
					printk("Rotation must be same \
							in both the layers\n");
					up(&vout->lock);
					return -EINVAL;
				}
			}
			omap_vout_calculate_offset(dest);

			if(rotation >= 0) {
				addr = (unsigned long)
				vout->queued_buf_addr[vout->curFrm->i] +
				dest->cropped_offset;
			} else {
				dmabuf = videobuf_to_dma(vout->curFrm);
				addr = (unsigned long)dmabuf->bus_addr +
					dest->cropped_offset;
			}
			omap_disp_config_vlayer (dest->vid, &dest->pix,
					&dest->crop, &dest->win,
					rotation, dest->mirror);
			omap_disp_set_addr(dest->vid, addr, addr,
					addr+dest->tv_field1_offset);
		}
		/* Register ISR handler */
		mask = (DISPC_IRQSTATUS_EVSYNC_ODD | DISPC_IRQSTATUS_EVSYNC_EVEN
			| DISPC_IRQSTATUS_VSYNC);
		ret = omap_disp_register_isr(omap_vout_isr, vout, mask);
		if(ret < 0)  {
			up(&vout->lock);
			return -EIO;
		}

		omap_disp_start_video_layer(vout->vid);
		if (vout_linked != -1 && vout_linked != vout->vid){
			omap_disp_start_video_layer(dest->vid);
		}
		up(&vout->lock);
		return 0;
	}

	case VIDIOC_STREAMOFF:
	{
		struct videobuf_queue *q = &fh->vbq;
		int ret = 0;
		if(!fh->io_allowed) {
			return -EINVAL;
		}
		if(!vout->streaming) {
			return -EINVAL;
		}

		if (vout->streaming == fh){
			omap_disp_disable_layer (vout->vid);
			vout->streaming = NULL;

			/* stop the slave layer */
			if (vout_linked != -1 && vout_linked != vout->vid){
				omap_disp_disable_layer (
						(vout->vid == OMAP_VIDEO1)
						? OMAP_VIDEO2 : OMAP_VIDEO1);
			}
			omap_disp_put_dss();

			ret = videobuf_streamoff(q);
			omap_disp_unregister_isr(omap_vout_isr);
			return 0;
		}

		return -EINVAL;
	}

	case VIDIOC_S_OMAP_LINK:
	{
		int *link = arg;

		spin_lock (&vout_link_lock);
		if ((*link == 0) && (vout_linked == vout->vid))
			vout_linked = -1;

		omap_disp_get_dss();
		if ((*link == 1) && (vout_linked == -1 ||
					vout_linked == vout->vid)){
			vout_linked = vout->vid;

			if (vout_linked == OMAP_VIDEO2){
				/* sync V2 to V1 for img and crop */
				omap_vout_sync (saved_v2out, saved_v1out);
			} else {
				/* sync V1 to V2 */
				omap_vout_sync (saved_v1out, saved_v2out);
			}
		}

		omap_disp_put_dss();

		spin_unlock (&vout_link_lock);
		return 0;
	}

	case VIDIOC_G_OMAP_LINK:
	{
		int *link = arg;

		spin_lock (&vout_link_lock);
		if (vout_linked == vout->vid)
			*link = 1;
		else
			*link = 0;
		spin_unlock (&vout_link_lock);
		return 0;
	}

	case VIDIOC_S_OMAP_MIRROR:
	{
		int *mirror = arg;

		if (down_interruptible(&vout->lock))
			return -EBUSY;
		if ((*mirror == 0) && (vout->mirror == 1)) {
			vout->mirror = 0;
			up(&vout->lock);
			return 0;
		} else if ((*mirror == 1) && (vout->mirror == 0)) {
			vout->mirror = 1;
			up(&vout->lock);
			return 0;
		}
		up(&vout->lock);
		return -EINVAL;
	}

	case VIDIOC_G_OMAP_MIRROR:
	{
		int *mirror = arg;
		*mirror = vout->mirror;
		return 0;
	}

	case VIDIOC_S_OMAP_ROTATION:
	{
		int *rotation = arg;

		if (down_interruptible(&vout->lock))
			return -EBUSY;
		if ((*rotation == 0) || (*rotation == 90) ||
				(*rotation == 180) || (*rotation == 270) ||
				(*rotation == -1)) {
			vout->rotation = (*rotation == 90) ? 270 :
				(*rotation == 270) ? 90 : *rotation;
			up(&vout->lock);
			rotation_support = vout->rotation;
			return 0;
		} else {
			up(&vout->lock);
			return -EINVAL;
		}
	}

	case VIDIOC_G_OMAP_ROTATION:
	{
		int *rotation = arg;
		*rotation = (vout->rotation == 90) ? 270 :
			(vout->rotation == 270) ? 90 : vout->rotation;
		return 0;
	}

	case VIDIOC_S_OMAP_COLORKEY:
	{
		struct omap_vout_colorkey *colorkey =
			(struct omap_vout_colorkey *) arg;

		if (down_interruptible(&vout->lock))
			return -EBUSY;
		if ((colorkey->output_dev != OMAP_OUTPUT_LCD &&
				colorkey->output_dev != OMAP_OUTPUT_TV) ||
				(colorkey->key_type != OMAP_GFX_DESTINATION
				&& colorkey->key_type != OMAP_VIDEO_SOURCE)) {
			up(&vout->lock);
			return -EINVAL;
		}
		omap_disp_get_dss();

		omap_disp_set_colorkey (colorkey->output_dev,
				colorkey->key_type, colorkey->key_val);

		omap_disp_put_dss();
		up(&vout->lock);

		return 0;
	}

	case VIDIOC_G_OMAP_COLORKEY:
	{
		struct omap_vout_colorkey *colorkey =
			(struct omap_vout_colorkey *) arg;

		if (colorkey->output_dev != OMAP_OUTPUT_LCD
				&& colorkey->output_dev != OMAP_OUTPUT_TV)
			return -EINVAL;

		omap_disp_get_dss();

		omap_disp_get_colorkey (colorkey->output_dev,
				&colorkey->key_type,
				&colorkey->key_val);

		omap_disp_put_dss();

		return 0;

	}

	case VIDIOC_S_OMAP_BGCOLOR:
	{
		struct omap_vout_bgcolor *bgcolor =
			(struct omap_vout_bgcolor *) arg;

		if (down_interruptible(&vout->lock))
			return -EBUSY;
		if (bgcolor->output_dev != OMAP_OUTPUT_LCD
				&& bgcolor->output_dev != OMAP_OUTPUT_TV) {
			up(&vout->lock);
			return -EINVAL;
		}
		omap_disp_get_dss();

		omap_disp_set_bg_color (bgcolor->output_dev, bgcolor->color);

		omap_disp_put_dss();

		up(&vout->lock);
		return 0;
	}

	case VIDIOC_G_OMAP_BGCOLOR:
	{
		struct omap_vout_bgcolor *bgcolor =
			(struct omap_vout_bgcolor *) arg;

		if (bgcolor->output_dev != OMAP_OUTPUT_LCD
				&& bgcolor->output_dev != OMAP_OUTPUT_TV)
			return -EINVAL;

		omap_disp_get_dss();

		omap_disp_get_bg_color (bgcolor->output_dev, &bgcolor->color);

		omap_disp_put_dss();

		return 0;
	}

	case VIDIOC_OMAP_COLORKEY_ENABLE:
	{
		int *output_dev = arg;

		if (*output_dev != OMAP_OUTPUT_LCD && *output_dev !=
				OMAP_OUTPUT_TV)
			return -EINVAL;

		omap_disp_get_dss();

		omap_disp_enable_colorkey (*output_dev);

		omap_disp_put_dss();

		return 0;
	}

	case VIDIOC_OMAP_COLORKEY_DISABLE:
	{
		int *output_dev = arg;

		if (*output_dev != OMAP_OUTPUT_LCD && *output_dev !=
				OMAP_OUTPUT_TV)
			return -EINVAL;

		omap_disp_get_dss();

		omap_disp_disable_colorkey (*output_dev);

		omap_disp_put_dss();

		return 0;
	}

	case VIDIOC_S_OMAP_COLORCONV:
	{
		int v;
		struct omap_vout_colconv *ccmtx =
			(struct omap_vout_colconv *) arg;

		if (down_interruptible(&vout->lock))
			return -EBUSY;
		if(vout->vid == OMAP_VIDEO1) v =0;
		else v =1;

		current_colorconv_values[v][0][0]=ccmtx->RY;
		current_colorconv_values[v][0][1]=ccmtx->RCr;
		current_colorconv_values[v][0][2]=ccmtx->RCb;
		current_colorconv_values[v][1][0]=ccmtx->GY;
		current_colorconv_values[v][1][1]=ccmtx->GCr;
		current_colorconv_values[v][1][2]=ccmtx->GCb;
		current_colorconv_values[v][2][0]=ccmtx->BY;
		current_colorconv_values[v][2][1]=ccmtx->BCr;
		current_colorconv_values[v][2][2]=ccmtx->BCb;
		omap_disp_get_dss();
		omap_disp_set_colorconv(vout->vid, &vout->pix);
		omap_disp_put_dss();
		up(&vout->lock);
		return 0;
	}

	case VIDIOC_G_OMAP_COLORCONV:
	{
		int v;
		struct omap_vout_colconv *ccmtx =
			(struct omap_vout_colconv *) arg;

		if(vout->vid == OMAP_VIDEO1) v =0;
		else v =1;

		ccmtx->RY = current_colorconv_values[v][0][0];
		ccmtx->RCr= current_colorconv_values[v][0][1];
		ccmtx->RCb= current_colorconv_values[v][0][2];
		ccmtx->GY = current_colorconv_values[v][1][0];
		ccmtx->GCr= current_colorconv_values[v][1][1];
		ccmtx->GCb= current_colorconv_values[v][1][2];
		ccmtx->BY = current_colorconv_values[v][2][0];
		ccmtx->BCr= current_colorconv_values[v][2][1];
		ccmtx->BCb= current_colorconv_values[v][2][2];

		return 0;
	}

	case VIDIOC_S_OMAP_DEFCOLORCONV:
	{
		if (down_interruptible(&vout->lock))
			return -EBUSY;
		omap_disp_get_dss();
		omap_disp_set_default_colorconv(vout->vid, &vout->pix);

		omap_disp_put_dss();
		up(&vout->lock);
		return 0;
	}

	default:
		/* unrecognized ioctl */
		return -ENOIOCTLCMD;

	} /* End of switch(cmd) */

	return 0;
}

/*
 *  file operations
 */

#ifdef DEBUG_ALLOW_WRITE

static ssize_t omap_vout_write (struct file *file, const char *data,
		size_t count, loff_t * ppos)
{
	struct omap_vout_fh *fh = file->private_data;
	struct omap_vout_device *vout = fh->vout;

	omap_disp_get_dss();

	omap_vout_suspend_lockout (vout, file);
	if ((*ppos) >= vout->pix.sizeimage) {
		omap_disp_put_dss();
		return 0;
	}
	if (count + (*ppos) > vout->pix.sizeimage)
		count = vout->pix.sizeimage - (*ppos);
	if (copy_from_user((void *) (vout->framebuffer_base + (*ppos)), data,
				count)){
		printk ("error in copying data");
	}
	*ppos += count;

#if 1
	if (*ppos == vout->pix.sizeimage)
		mdelay (5000);
#endif

	omap_disp_put_dss();

	return count;
}

#endif

static void omap_vout_vm_open (struct vm_area_struct *vma)
{
	struct omap_vout_device *vout = vma->vm_private_data;
	DPRINTK ("vm_open [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->mmap_count++;
}

static void omap_vout_vm_close (struct vm_area_struct *vma)
{
	struct omap_vout_device *vout = vma->vm_private_data;
	DPRINTK ("vm_close [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->mmap_count--;
}

static struct vm_operations_struct omap_vout_vm_ops = {
	.open = omap_vout_vm_open,
	.close = omap_vout_vm_close,
};

static int omap_vout_mmap (struct file *file, struct vm_area_struct *vma)
{
	struct omap_vout_fh *fh = file->private_data;
	struct omap_vout_device *vout = fh->vout;
	struct videobuf_queue *q = &fh->vbq;
	unsigned long size = (vma->vm_end - vma->vm_start);
	unsigned long start = vma->vm_start;
	int i;
	void *pos;
	struct videobuf_dmabuf *dmabuf=NULL;

	DPRINTK ("pgoff=0x%x, start=0x%x, end=0x%x\n", vma->vm_pgoff,
			vma->vm_start,  vma->vm_end);

	/* look for the buffer to map */
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (NULL == q->bufs[i])	continue;
		if (V4L2_MEMORY_MMAP != q->bufs[i]->memory)
			continue;
		if (q->bufs[i]->boff == (vma->vm_pgoff << PAGE_SHIFT))
			break;
	}

	if (VIDEO_MAX_FRAME == i) {
		DPRINTK ("offset invalid [offset=0x%lx]\n",
				(vma->vm_pgoff << PAGE_SHIFT));
		return -EINVAL;
	}
	q->bufs[i]->baddr = vma->vm_start;

	vma->vm_flags |= VM_RESERVED;
	vma->vm_page_prot = pgprot_writecombine (vma->vm_page_prot);
	vma->vm_ops = &omap_vout_vm_ops;
	vma->vm_private_data = (void *) vout;
	dmabuf = videobuf_to_dma(q->bufs[i]);
	pos = dmabuf->vmalloc;
	vma->vm_pgoff = virt_to_phys((void *)pos) >> PAGE_SHIFT;
	while (size > 0) {
		unsigned long pfn;
		pfn = virt_to_phys((void *)pos) >> PAGE_SHIFT;
		if (remap_pfn_range(vma, start, pfn, PAGE_SIZE, PAGE_SHARED))
			return -EAGAIN;
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	vout->mmap_count++;
	return 0;
}

static int omap_vout_ioctl (struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct omap_vout_fh *fh = file->private_data;
	struct omap_vout_device *vout = fh->vout;

	omap_vout_suspend_lockout (vout, file);
	return video_usercopy (inode, file, cmd, arg, omap_vout_do_ioctl);
}

static void omap_vout_free_allbuffers(struct omap_vout_device *vout)
{
	int num_buffers = 0, i;
	num_buffers = (vout->vid == OMAP_VIDEO1) ?
		video1_numbuffers : video2_numbuffers;
	for(i = num_buffers ; i < vout->buffer_allocated ; i ++) {
		if(vout->buf_virt_addr[i]) {
			omap_vout_free_buffer(vout->buf_virt_addr[i],
					vout->buf_phy_addr[i],
					vout->buffer_size);
		}
		vout->buf_virt_addr[i] = 0;
		vout->buf_phy_addr[i] = 0;
	}
	for(i = 0 ; i < 4 ; i ++) {
		if(vout->smsshado_virt_addr[i]) {
			omap_vout_free_buffer(vout->smsshado_virt_addr[i],
					vout->smsshado_phy_addr[i],
					vout->smsshado_size);
			vout->smsshado_virt_addr[i]=0;
			vout->smsshado_phy_addr[i]=0;
		}
	}
	vout->buffer_allocated = num_buffers;
}

static int omap_vout_release (struct inode *inode, struct file *file)
{
	struct omap_vout_fh *fh = file->private_data;
	struct omap_vout_device *vout;
	struct videobuf_queue *q;

	DPRINTK ("entering\n");

	if (fh == 0)
		return 0;
	if ((vout = fh->vout) == 0)
		return 0;
	q = &fh->vbq;

	omap_disp_get_dss();
	omap_vout_suspend_lockout (vout, file);

	/*
	 * Check if the hidden buffer transfer is happening with DMA
	 * if yes then stop it
	 */

	if (vout->rotation >= 0) {
		if (vout->vrfb_dma_tx.tx_status == 0) {
			/*
			 * DMA will be stopped once here and again after
			 * wakeup to avoid race conditions due to time
			 * taken to wakeup the sleeping process
			 */

			omap_stop_dma (vout->vrfb_dma_tx.dma_ch);
			wake_up_interruptible(&vout->vrfb_dma_tx.wait);
		}
	}

	omap_disp_disable_layer (vout->vid);

	if ((vout_linked != -1) && (vout->vid != vout_linked))
		omap_disp_disable_layer ((vout->vid == OMAP_VIDEO1) ?
				OMAP_VIDEO2 : OMAP_VIDEO1);

	if(fh->io_allowed) {
		videobuf_streamoff(q);
		videobuf_queue_cancel(q);
		/* Free all buffers */
		omap_vout_free_allbuffers(vout);
		videobuf_mmap_free(q);
	}

#ifdef DEBUG_ALLOW_WRITE
	if (vout->framebuffer_base){

		dma_free_coherent(NULL, vout->framebuffer_size, (void *)
				vout->framebuffer_base, vout->
				framebuffer_base_phys);
		vout->framebuffer_base = 0;
	}
	vout->framebuffer_base_phys = 0;
#endif

	if (vout->streaming == fh) {
		omap_disp_unregister_isr(omap_vout_isr);
		vout->streaming = NULL;
	}

	if (vout->mmap_count != 0){
		vout->mmap_count = 0;
		printk("mmap count is not zero!\n");
	}

	omap_disp_release_layer (vout->vid);
	omap_disp_put_dss ();
	vout->opened -= 1;
	file->private_data = NULL;

	if (vout->buffer_allocated)
		videobuf_mmap_free(q);

	kfree (fh);

	/* need to remove the link when the either slave or master is gone */
	spin_lock (&vout_link_lock);
	if (vout_linked != -1) {
		vout_linked = -1;
	}
	spin_unlock (&vout_link_lock);

	return 0;
}

static int omap_vout_open (struct inode *inode, struct file *file)
{
	int minor = MINOR (file->f_dentry->d_inode->i_rdev);
	struct omap_vout_device *vout = NULL;
	struct omap_vout_fh *fh;
	struct videobuf_queue *q;
#ifdef DEBUG_ALLOW_WRITE
	int rotation=-1;
#endif
	/*int i;*/

	DPRINTK ("entering\n");

	if (saved_v1out && saved_v1out->vfd && (saved_v1out->vfd->minor ==
				minor)) {
		vout = saved_v1out;
	}

	if (vout == NULL) {
		if (saved_v2out && saved_v2out->vfd
				&& (saved_v2out->vfd->minor == minor)) {
			vout = saved_v2out;
		}
	}

	if (vout == NULL)
		return -ENODEV;

	/* for now, we only support single open */
	if (vout->opened)
		return -EBUSY;

	vout->opened += 1;
	if (!omap_disp_request_layer (vout->vid)) {
		vout->opened -= 1;
		return -ENODEV;
	}

	omap_disp_get_dss ();
	omap_vout_suspend_lockout (vout, file);
	/* allocate per-filehandle data */
	fh = kmalloc (sizeof (*fh), GFP_KERNEL);
	if (NULL == fh) {
		omap_disp_release_layer (vout->vid);
		omap_disp_put_dss ();
		vout->opened -= 1;
		return -ENOMEM;
	}
	memset(fh, 0, sizeof(*fh));

	file->private_data = fh;
	fh->vout = vout;
	fh->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

#ifdef DEBUG_ALLOW_WRITE
	vout->framebuffer_size = VID_MAX_HEIGHT * VID_MAX_HEIGHT * 2;
	vout->framebuffer_base = dma_alloc_coherent(NULL,vout->framebuffer_size,
			(dma_addr_t *) & vout->	framebuffer_base_phys,
			GFP_KERNEL | GFP_DMA);

	if (!vout->framebuffer_base){
		kfree (fh);
		omap_disp_release_layer (vout->vid);
		omap_disp_put_dss ();
		vout->opened -= 1;
		return -ENOMEM;
	}
	rotation = (vout->rotation >= 0)?vout->rotation:-1;
	memset ((void *) vout->framebuffer_base, 0, vout->framebuffer_size);
	omap_disp_config_vlayer (vout->vid, &vout->pix, &vout->crop,
			&vout->win, rotation, vout->mirror);
	omap_disp_start_vlayer (vout->vid, &vout->pix, &vout->crop,
			vout->framebuffer_base_phys, rotation, vout->mirror);
#endif

	q = &fh->vbq;
	video_vbq_ops.buf_setup = omap_vout_buffer_setup;
	video_vbq_ops.buf_prepare = omap_vout_buffer_prepare;
	video_vbq_ops.buf_release = omap_vout_buffer_release;
	video_vbq_ops.buf_queue = omap_vout_buffer_queue;
	spin_lock_init(&vout->vbq_lock);

	videobuf_queue_sg_init (q, &video_vbq_ops, NULL, &vout->vbq_lock,
			fh->type, V4L2_FIELD_NONE, sizeof
			(struct videobuf_buffer), fh);

	omap_disp_put_dss();

	return 0;
}

static struct file_operations omap_vout_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
#ifdef DEBUG_ALLOW_WRITE
	.write = omap_vout_write,
#endif
	.ioctl = omap_vout_ioctl,
	.mmap = omap_vout_mmap,
	.open = omap_vout_open,
	.release = omap_vout_release,
};

#ifdef CONFIG_PM
static int omap_vout_suspend (struct platform_device *dev,
		pm_message_t state)
{
	struct omap_vout_device *vout = platform_get_drvdata (dev);

	/* lock-out applications during suspend */
	if(vout->suspended == 1)
		return 0;
	if (vout->opened) {
		/* stall vid DMA */
		if (vout->streaming) {
			omap_disp_disable_layer (vout->vid);
			/*
			 * Check if the hidden buffer transfer is happening
			 * with DMA if yes then stop it
			 */

			if (vout->rotation >= 0) {
				if (vout->vrfb_dma_tx.tx_status == 0) {
					/*
					 * DMA will be stopped once here
					 * and again after wakeup to
					 * avoid race conditions due to time
					 * taken to wakeup the sleeping
					 * process */
					omap_stop_dma (
						vout->vrfb_dma_tx.dma_ch);
					wake_up_interruptible (
						&vout->vrfb_dma_tx.wait);
				}
			}
		}
		vout->suspended = 1;
		omap_disp_put_dss ();
	}

	return 0;
}

static int omap_vout_resume (struct platform_device *dev)
{
	struct omap_vout_device *vout = platform_get_drvdata (dev);
	if(vout->suspended == 0)
		return 0;
	if (vout->opened) {
		omap_disp_get_dss ();

		/* resume vid DMA */
		if (vout->streaming)
			omap_disp_enable_layer (vout->vid);

		/* wake up applications waiting on suspend queue */
		vout->suspended = 0;
		wake_up (&vout->suspend_wq);
	}
	return 0;
}

#ifdef CONFIG_DPM

static struct constraints omap_vout_constraints = {
	.count = 2,
	.param = {
		{DPM_MD_V, OMAP_V_MIN, OMAP_V_MAX},
		{DPM_MD_SLEEP_MODE, PM_SUSPEND_STANDBY, PM_SUSPEND_MEM},
	},
};

static int omap_vout_scale (int vid, struct notifier_block *op,
		unsigned long level)
{
	struct omap_vout_device *vout;
	vout = (vid == OMAP_VIDEO1) ? saved_v1out : saved_v2out;
	if (!vout->opened)
		return 0;
	switch (level) {
	case SCALE_PRECHANGE:
		if (vout->streaming) {
			omap_disp_disable_layer (vout->vid);
			if (vout->rotation >= 0) {
				if (vout->vrfb_dma_tx.tx_status == 0){
					omap_stop_dma (
						vout->vrfb_dma_tx.dma_ch);
				}
			}
		}
		break;
	case SCALE_POSTCHANGE:
		if (vout->streaming) {
			omap_disp_enable_layer (vout->vid);
			if (vout->rotation >= 0) {
				if (vout->vrfb_dma_tx.tx_status == 0) {
					omap_start_dma (
						vout->vrfb_dma_tx.dma_ch);
				}
			}
		}
		break;
	}
	return 0;
}

static int omap_v1out_scale (struct notifier_block *op,
		unsigned long level, void *ptr)
{
	return omap_vout_scale (OMAP_VIDEO1, op, level);
}

static int omap_v2out_scale (struct notifier_block *op,
		unsigned long level, void *ptr)
{
	return omap_vout_scale (OMAP_VIDEO2, op, level);
}

static struct notifier_block omap_v1out_pre_scale = {
	.notifier_call = omap_v1out_scale,
};

static struct notifier_block omap_v1out_post_scale = {
	.notifier_call = omap_v1out_scale,
};

static struct notifier_block omap_v2out_pre_scale = {
	.notifier_call = omap_v2out_scale,
};

static struct notifier_block omap_v2out_post_scale = {
	.notifier_call = omap_v2out_scale,
};

#endif
#endif /* PM */

static int
omap_vout_probe (struct platform_device *dev)
{
	return 0;
}
static void omap_vout_platform_release(struct device
				  *device)
{
	/* This is called when the reference count goes to zero */
}

static struct platform_device omap_v1out_dev = {
	.name = V1OUT_NAME,
	.id = 11,
	//.devid = OMAP_V1OUT_DEVID,
	// .busid = OMAP_BUS_L3,
	.dev = {
#ifdef CONFIG_DPM
		.constraints = &omap_vout_constraints,
#endif
		.release = omap_vout_platform_release,
	},
};

static struct platform_device omap_v2out_dev = {
	.name = V2OUT_NAME,
	.id = 12,
	//.devid = OMAP_V2OUT_DEVID,
	//.busid = OMAP_BUS_L3,
	.dev = {
#ifdef CONFIG_DPM
		.constraints = &omap_vout_constraints,
#endif
		.release = omap_vout_platform_release,
	},
};

static struct platform_driver omap_v1out_driver = {
	.driver = {
		.name = V1OUT_NAME,
	},
	//.devid   = OMAP_V1OUT_DEVID,
	// .id   = 11,
	//.busid   = OMAP_BUS_L3,
	// .clocks  = 0,
	.probe   = omap_vout_probe,
#ifdef CONFIG_PM
	.suspend = omap_vout_suspend,
	.resume  = omap_vout_resume,
#endif
};

static struct platform_driver omap_v2out_driver = {
	.driver = {
		.name = V2OUT_NAME,
	},
	//.id   = 12,
	//.devid   = OMAP_V2OUT_DEVID,
	//.busid   = OMAP_BUS_L3,
	//.clocks  = 0,
	.probe   = omap_vout_probe,
#ifdef CONFIG_PM
	.suspend = omap_vout_suspend,
	.resume  = omap_vout_resume,
#endif
};

void omap_vout_isr(void *arg, struct pt_regs *regs, unsigned int irqstatus)
{
	struct timeval timevalue;
	unsigned int out_dev;
	struct omap_vout_device *vout=(struct omap_vout_device *)arg,
				   *dest=NULL;
	struct videobuf_dmabuf *dmabuf=NULL;
	u32 addr, fid;

	if(!vout->streaming)
		return;

	spin_lock(&vout->vbq_lock);
	do_gettimeofday(&timevalue);
	out_dev = omap_disp_get_output_dev(vout->vid);
	if(out_dev == OMAP_OUTPUT_LCD) {
		if(!(irqstatus & DISPC_IRQSTATUS_VSYNC))
			return;
		if(!vout->first_int &&
				(vout->curFrm != vout->nextFrm)){
			vout->curFrm->ts = timevalue;
			vout->curFrm->state = VIDEOBUF_DONE;
			wake_up_interruptible(&vout->curFrm->done);
			vout->curFrm = vout->nextFrm;
		}
		vout->first_int = 0;
		if(list_empty(&vout->dma_queue)) {
			spin_unlock(&vout->vbq_lock);
			return;
		}
		vout->nextFrm = list_entry(vout->dma_queue.next,
				struct videobuf_buffer,
				queue);
		list_del(&vout->nextFrm->queue);

		vout->nextFrm->state = VIDEOBUF_ACTIVE;

		addr = (unsigned long)
			vout->queued_buf_addr[vout->nextFrm->i] +
			vout->cropped_offset;
		omap_disp_set_addr(vout->vid, addr, addr,
				addr + vout->tv_field1_offset);
		if (vout_linked != -1 && vout_linked != vout->vid){
			dmabuf = videobuf_to_dma(vout->nextFrm);
			dest = (vout_linked == OMAP_VIDEO1) ? saved_v1out :
				saved_v2out;
			if(dest->rotation > -1)
				addr = (unsigned long)
					vout->queued_buf_addr
					[vout->nextFrm->i] +
					dest->cropped_offset;
			else
				addr = dmabuf->bus_addr +
					dest->cropped_offset;
			omap_disp_set_addr(dest->vid, addr, addr,
					addr + dest->tv_field1_offset);
		}
	} else {
		if(vout->first_int) {
			vout->first_int = 0;
			spin_unlock(&vout->vbq_lock);
			return ;
		}
		if(irqstatus & DISPC_IRQSTATUS_EVSYNC_ODD)
			fid = 1;
		else if(irqstatus & DISPC_IRQSTATUS_EVSYNC_EVEN)
			fid = 0;
		else {
			spin_unlock(&vout->vbq_lock);
			return;
		}
		vout->field_id ^= 1;
		if(fid != vout->field_id) {
			if(0 == fid)
				vout->field_id = fid;

			spin_unlock(&vout->vbq_lock);
			return;
		}
		if(0 == fid) {
			if(vout->curFrm == vout->nextFrm) {
				spin_unlock(&vout->vbq_lock);
				return;
			}
			vout->curFrm->ts = timevalue;
			vout->curFrm->state = VIDEOBUF_DONE;
			wake_up_interruptible(&vout->curFrm->done);
			vout->curFrm = vout->nextFrm;
		} else if(1 == fid){
			if(list_empty(&vout->dma_queue) ||
					(vout->curFrm != vout->nextFrm)) {
				spin_unlock(&vout->vbq_lock);
				return ;
			}

			vout->nextFrm = list_entry(vout->dma_queue.next,
					struct videobuf_buffer, queue);

			list_del(&vout->nextFrm->queue);

			vout->nextFrm->state = VIDEOBUF_ACTIVE;

			addr = (unsigned long)
				vout->queued_buf_addr[vout->nextFrm->i] +
				vout->cropped_offset;
			omap_disp_set_addr(vout->vid, addr, addr,
					addr + vout->tv_field1_offset);
			if (vout_linked != -1 && vout_linked != vout->vid) {
				dmabuf = videobuf_to_dma(vout->nextFrm);
				dest = (vout_linked == OMAP_VIDEO1) ?
					saved_v1out : saved_v2out;
				if(dest->rotation > -1)
					addr = (unsigned long)
						vout->queued_buf_addr
						[vout->nextFrm->i] +
						dest->cropped_offset;
				else
					addr = dmabuf->bus_addr +
						dest->cropped_offset;
				omap_disp_set_addr(dest->vid, addr, addr,
						addr + dest->tv_field1_offset);
			}
		}
	}
	spin_unlock(&vout->vbq_lock);
}

static void cleanup_vout_device (int vid)
{
	struct video_device *vfd;
	struct omap_vout_device *vout;
	int i, j, numbuffers;

	vout = (vid == OMAP_VIDEO1) ? saved_v1out : saved_v2out;
	if (!vout)
		return;
	vfd = vout->vfd;

	if (vfd) {
		if (vfd->minor == -1) {
			/*
			 * The device was never registered, so release the
			 * video_device struct directly.
			 */
			video_device_release (vfd);
		} else {
			/*
			 * The unregister function will release the video_device
			 * struct as well as unregistering it.
			 */
			video_unregister_device (vfd);
		}
	}

	for(i = 0 ; i < 4 ; i ++) {
		for( j = 0 ; j < 4 ; j ++) {
			release_mem_region (vout->sms_rot_phy[i][j], VRF_SIZE);
		}
	}

	/* Allocate memory for the buffes */
	numbuffers = (vid == OMAP_VIDEO1) ? video1_numbuffers :
		video2_numbuffers;
	vout->buffer_size = (vid == OMAP_VIDEO1) ? video1_bufsize :
		video2_bufsize;
	for(i = 0 ; i < numbuffers ; i ++) {
		omap_vout_free_buffer(vout->buf_virt_addr[i],
				vout->buf_phy_addr[i],
				vout->buffer_size);
		vout->buf_phy_addr[i]=0;
		vout->buf_virt_addr[i]=0;
	}

	if (vout->vrfb_dma_tx.req_status == DMA_CHAN_ALLOTED){
		vout->vrfb_dma_tx.req_status = DMA_CHAN_NOT_ALLOTED;
		omap_free_dma (vout->vrfb_dma_tx.dma_ch);
	}

	platform_device_unregister ((vid == OMAP_VIDEO1) ?
			&omap_v1out_dev : &omap_v2out_dev);
	platform_driver_unregister ((vid == OMAP_VIDEO1) ?
			&omap_v1out_driver : &omap_v2out_driver);

#ifdef CONFIG_DPM
	if (vid == OMAP_VIDEO1){
		dpm_unregister_scale(&omap_v1out_pre_scale,
				SCALE_PRECHANGE);
		dpm_unregister_scale(&omap_v1out_post_scale,
				SCALE_POSTCHANGE);
	} else {
		dpm_unregister_scale(&omap_v2out_pre_scale,SCALE_PRECHANGE);
		dpm_unregister_scale(&omap_v2out_pre_scale,SCALE_POSTCHANGE);
	}
#endif

	kfree (vout);

	if (vid == OMAP_VIDEO1)
		saved_v1out = NULL;
	else
		saved_v2out = NULL;
}

static struct omap_vout_device * init_vout_device (int vid)
{
	int r, i, j;
	struct omap_vout_device *vout;
	struct video_device *vfd;
	struct v4l2_pix_format *pix;
	struct platform_driver *this_driver;
	struct platform_device *this_dev;
	u32 sms_start_addr, numbuffers;
	int index_i, index_j, image_width, image_height;

	vout = kmalloc (sizeof (struct omap_vout_device), GFP_KERNEL);
	if (!vout){
		printk (KERN_ERR VOUT_NAME ": could not allocate memory\n");
		return NULL;
	}

	memset (vout, 0, sizeof (struct omap_vout_device));
	vout->vid = vid;
	vout->rotation = rotation_support;

	/* set the default pix */
	pix = &vout->pix;
	pix->width = QQVGA_WIDTH;
	pix->height = QQVGA_HEIGHT;

	pix->pixelformat = V4L2_PIX_FMT_RGB565;
	pix->field = V4L2_FIELD_ANY;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	pix->colorspace = V4L2_COLORSPACE_JPEG;

	vout->bpp = RGB565_BPP;
	vout->vrfb_bpp = 1;

	/* get the screen parameters */
	omap_disp_get_panel_size (omap_disp_get_output_dev (vout->vid),
			&(vout->fbuf.fmt.width),&(vout->fbuf.fmt.height));

	/* set default crop and win */
	omap_vout_new_format (pix, &vout->fbuf, &vout->crop, &vout->win);

	/* initialize the video_device struct */
	vfd = vout->vfd = video_device_alloc ();
	if (!vfd){
		printk (KERN_ERR VOUT_NAME": could not allocate video \
				device struct\n");
		kfree (vout);
		return NULL;
	}
	vfd->release = video_device_release;

	strncpy (vfd->name, VOUT_NAME, sizeof (vfd->name));
	vfd->type = VID_TYPE_OVERLAY | VID_TYPE_CHROMAKEY;
	/* need to register for a VID_HARDWARE_* ID in videodev.h */
	vfd->fops = &omap_vout_fops;
	video_set_drvdata (vfd, vout);
	vfd->minor = -1;

	sms_start_addr = (vid == OMAP_VIDEO1)?OMAP_VIDEO1_SMS_START :
			OMAP_VIDEO2_SMS_START;
	for(i = 0 ; i < 4 ; i ++) {
		for(j = 0 ; j < 4 ; j ++) {
			vout->sms_rot_phy[i][j] = sms_start_addr;
			if (!request_mem_region (vout->sms_rot_phy[i][j],
						VRF_SIZE, vfd->name)) {
				printk("Cannot reserve smsm IO %x\n",
						vout->sms_rot_phy[i][j]);
				index_i = i;
				index_j = j;
				goto rotation_free;
			}
			sms_start_addr += 0x1000000;
		}
	}
	index_i = 4;
	index_j = 0;

	/* Allocate memory for the buffes */
	numbuffers = (vid == OMAP_VIDEO1) ? video1_numbuffers :
		video2_numbuffers;
	vout->buffer_size = (vid == OMAP_VIDEO1) ? video1_bufsize :
		video2_bufsize;
	printk("Buffer Size = %d\n",vout->buffer_size);
	for(i = 0 ; i < numbuffers ; i ++) {
		vout->buf_virt_addr[i] = omap_vout_alloc_buffer(
				vout->buffer_size,
				(u32*)&vout->buf_phy_addr[i]);
		if(!vout->buf_virt_addr[i]) {
			numbuffers = i;
			goto free_buffer_memory;
		}
	}

	vout->suspended = 0;
	init_waitqueue_head (&vout->suspend_wq);
	init_MUTEX(&vout->lock);

	if (video_register_device (vfd, VFL_TYPE_GRABBER, vid) < 0){
		printk (KERN_ERR VOUT_NAME": could not register Video for \
				Linux device\n");
		vfd->minor = -1;
		goto free_buffer_memory;
	}

	this_driver = (vid == OMAP_VIDEO1) ?
		&omap_v1out_driver : &omap_v2out_driver;
	this_dev = (vid == OMAP_VIDEO1) ?
		&omap_v1out_dev : &omap_v2out_dev;
	if (platform_driver_register (this_driver) != 0) {
		printk (KERN_ERR VOUT_NAME ": could not register \
				Video driver\n");
		cleanup_vout_device (vid);
		return NULL;
	}
	if (platform_device_register (this_dev) != 0) {
		printk (KERN_ERR VOUT_NAME ": could not register \
				Video device\n");
		cleanup_vout_device (vid);
		return NULL;
	}
	/* set driver specific data to use in power mgmt functions */

	platform_set_drvdata (this_dev, vout);

	if(vid == OMAP_VIDEO1) {
		vout->vrfb_context[0] = 4;
		vout->vrfb_context[1] = 5;
		vout->vrfb_context[2] = 6;
		vout->vrfb_context[3] = 7;
	} else {
		vout->vrfb_context[0] = 8;
		vout->vrfb_context[1] = 9;
		vout->vrfb_context[2] = 10;
		vout->vrfb_context[3] = 11;
	}
	vout->cropped_offset = 0;

#ifdef CONFIG_DPM
	/* Scaling is enabled only when DPM is enabled */
	if(vid == OMAP_VIDEO1) {
		dpm_register_scale(&omap_v1out_pre_scale,SCALE_PRECHANGE);
		dpm_register_scale(&omap_v1out_post_scale,SCALE_POSTCHANGE);
	} else {
		dpm_register_scale(&omap_v2out_pre_scale,SCALE_PRECHANGE);
		dpm_register_scale(&omap_v2out_post_scale,SCALE_POSTCHANGE);
	}
#endif
	/* Calculate VRFB memory size */
	/* allocate for worst case size */
	image_width = VID_MAX_WIDTH / TILE_SIZE;
	if (VID_MAX_WIDTH % TILE_SIZE)
		image_width++;

	image_width = image_width * TILE_SIZE;
	image_height = VID_MAX_HEIGHT / TILE_SIZE;

	if (VID_MAX_HEIGHT % TILE_SIZE)
		image_height++;

	image_height = image_height * TILE_SIZE;
	vout->smsshado_size = PAGE_ALIGN(image_width * image_height * 2 * 2);

	/*
	 * Request and Initialize DMA, for DMA based VRFB transfer
	 */
	vout->vrfb_dma_tx.dev_id = OMAP_DMA_NO_DEVICE;
	vout->vrfb_dma_tx.dma_ch = -1;
	vout->vrfb_dma_tx.req_status = DMA_CHAN_ALLOTED;
	r = omap_request_dma (vout->vrfb_dma_tx.dev_id,
			"VRFB DMA TX", vrfb_dma_tx_callback,
			(void *) &vout->vrfb_dma_tx,
			&vout->vrfb_dma_tx.dma_ch);
	if (r < 0)
		vout->vrfb_dma_tx.req_status = DMA_CHAN_NOT_ALLOTED;

	init_waitqueue_head (&vout->vrfb_dma_tx.wait);

	/*if rotation support */
	printk (KERN_INFO VOUT_NAME ": registered device video%d [v4l2]\n",
			vfd->minor);
	return vout;

free_buffer_memory:
	for(i = 0 ; i < numbuffers ; i ++){
		omap_vout_free_buffer(vout->buf_virt_addr[i],
				vout->buf_phy_addr[i], vout->buffer_size);
		vout->buf_virt_addr[i] = 0;
		vout->buf_phy_addr[i] = 0;
	}

rotation_free:
	for(i = 0 ; i < index_i ; i ++){
		for( j = 0 ;j < 4 ; j ++) {
			release_mem_region (vout->sms_rot_phy[i][j],
					VRF_SIZE);
		}
	}
	for(j = 0 ; j < index_j ; j ++){
		release_mem_region (vout->sms_rot_phy[index_i][j], VRF_SIZE);
	}
	video_device_release (vfd);
	kfree (vout);
	return NULL;
}

static int __init omap_vout_init (void)
{
	omap_disp_get_dss();
	saved_v1out = init_vout_device (OMAP_VIDEO1);
	if (saved_v1out == NULL) {
		omap_disp_put_dss();
		return -ENODEV;
	}
	omap_disp_save_initstate(OMAP_DSS_DISPC_GENERIC);
	omap_disp_save_initstate(OMAP_VIDEO1);

	saved_v2out = init_vout_device (OMAP_VIDEO2);
	if (saved_v2out == NULL){
		cleanup_vout_device (OMAP_VIDEO1);
		omap_disp_put_dss();
		return -ENODEV;
	}
	omap_disp_save_initstate(OMAP_DSS_DISPC_GENERIC);
	omap_disp_save_initstate(OMAP_VIDEO2);
	omap_disp_put_dss();

	vout_linked = -1;
	spin_lock_init (&vout_link_lock);
	return 0;
}

static void omap_vout_cleanup (void)
{
	omap_disp_get_dss();
	cleanup_vout_device (OMAP_VIDEO1);
	cleanup_vout_device (OMAP_VIDEO2);
	omap_disp_put_dss();
}

#ifndef MODULE
/*
 *	omap_vout_setup - process command line options
 *	@options: string of options
 *
 *	NOTE: This function is a __setup and __init function.
 *
 *	Returns zero.
 */
int __init omap_vout_setup (char *options)
{
	char *this_opt;
	int i;

	if (!options || !*options)return 0;

	DPRINTK ("Options \"%s\"\n", options);
	i = strlen (VOUT_NAME);
	if (!strncmp (options, VOUT_NAME, i) && options[i] == ':'){
		this_opt = options + i + 1;
		if (!this_opt || !*this_opt)
			return 0;

		if (!strncmp (this_opt, "rotation=", 9)){
			int deg = simple_strtoul (this_opt + 9, NULL, 0);
			switch (deg){
				case 0:
				case 90:
				case 180:
				case 270:
					rotation_support = (deg == 90) ?
						270 : (deg == 270) ? 90 : deg;
					break;
				default:
					rotation_support = -1;
					break;
			}
			printk (KERN_INFO VOUT_NAME ": Rotation %s\n",
					(rotation_support == -1) ?
					"none (supported: \"rotation=\
				[-1|0|90|180|270]\")" :
					this_opt + 9);
		} else
			printk (KERN_INFO VOUT_NAME ": Invalid parameter \
					\"%s\" "
					"(supported: \
				\"rotation=[-1|0|90|180|270]\")\n",
					this_opt);
		return 0;
	}

	/*
	 * If we get here no fb was specified.
	 * We consider the argument to be a global video mode option.
	 */
	/* TODO - remove when FB is configured */

	return 0;
}

__setup ("videoout=", omap_vout_setup);
#endif

MODULE_AUTHOR ("Texas Instruments.");
MODULE_DESCRIPTION ("OMAP Video for Linux Video out driver");
MODULE_LICENSE ("GPL");
/* TODO -- Enabling it results in build erros, why?? */

late_initcall (omap_vout_init);
module_exit (omap_vout_cleanup);
