/*
 * arch/arm/plat-omap/dsi_drv.c
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied
 *
 * The purpose of this file is to provide DSI DAL driver on the
 * CSST Target. This file implements the DAL layer API functions
 * for the DSI (Display Serial Interface) module of OMAP
 *
 *
 */

/*==== INCLUDES ============================================================*/
#include "dsi_drv.h"
#include "dsi_dis.h"
#include "dsi.h"
#include "brd_dsi.h"

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/arch/hardware.h>
#include <asm/arch/display.h>
#include <asm/arch/clock.h>

/*=============================== GLOBALS =================================*/

/*====================================FUNCTIONS =============================*/

/*-----------------------------------------------------------------------------
| Function    : S32 dsi_init(const void *initstr, U32 *device_handle)
+------------------------------------------------------------------------------
| Description : Implements the DAL interface init API for DSI module.
|	        This function need to be called to initialize the DSI driver.
|
| Parameters  : initstr - pointer to the T_DSI_INIT structure
|               device_handle-handle/reference to the driver structure/instance
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 dsi_init(const void *init_str, U32 *device_handle)
{
	T_DSI_INIT	*dsi_init_str;
	T_DSI_DIS	*dsi_dis;
	S32 ret	= OMAP_DAL_SUCCESS;

	dsi_init_str = (T_DSI_INIT *)init_str;
	dsi_dis = (T_DSI_DIS *)kmalloc(sizeof(T_DSI_DIS), GFP_KERNEL);
	if (dsi_dis == NULL) {

		printk(KERN_INFO "dsi_drv.c : dsi_init allocation failed \n");

		return OMAP_DAL_ERROR;
	}

	dsi_dis->init = *dsi_init_str;

	*device_handle = (U32)dsi_dis;

	/* Fill the DSI D-PHY configuration. This need to be implemented by the
	 * platform dependent low level driver */
	get_dphy_config(&dsi_dis->dphy_config,
				dsi_dis->init.num_data_lanes,
				dsi_dis->init.output_intf_config.pixel_clk,
				dsi_dis->init.output_intf_config.pixel_format);

	/* Initialize the DSI platform specific configuration */
	ret = dsi_platform_init(dsi_dis);
	if (ret != OMAP_DAL_SUCCESS) {

		kfree(dsi_dis);
		return ret;
	}

	/* Configure the DSI interface of OMAP */
	ret = config_dsi_interface(dsi_dis);
	if (ret != OMAP_DAL_SUCCESS) {


		dsi_platform_deinit(dsi_dis);
		kfree(dsi_dis);
	}

	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 dsi_read(U32 dis_addr, U32 tag,  U32 *len, U8* buf)
+------------------------------------------------------------------------------
| Description : This function implements the DAL read API. Implements the
|		READ tags supported by DSI DAL layer.
|
| Parameters  : device_handle - device_handle returned by the init API.
|	        read_tag - Read TAG
|		buf_len	 - Length of the buffer
|               buf	 - Pointer to REQ_RESP_BUF
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 dsi_read(U32 device_handle, U32 read_tag, U32 *buf_len, U8 *buf)
{
	T_DSI_DIS *dsi_dis;
	T_REQ_RESP_BUF *in_out_buf;
	S32 ret	= OMAP_DAL_SUCCESS;
	dsi_dis = (T_DSI_DIS *)device_handle;
	in_out_buf = (T_REQ_RESP_BUF *)buf;

	/*TODO: parameter validation */
	switch (read_tag) {

	case SEND_DCS_READ_CMD:

		ret = send_dsi_packet(DCS_READ_CMD,
				dsi_dis->init.virtual_channel_id,
				READ_CMD,
				in_out_buf->req_buf.len,
				in_out_buf->req_buf.buf,
				in_out_buf->mode,
				dsi_dis->init.ecc_flag,
				dsi_dis->init.checksum_flag,
				in_out_buf->resp_buf.buf,
				in_out_buf->resp_buf.len);
		break;

	default:
		return OMAP_DAL_ERROR;
	}
	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 dsi_write(U32 device_handle,U32 write_tag,U32 *len,U8* buf)
+------------------------------------------------------------------------------
| Description : This function implements the DAL write API. Implements the
|               WRITE Tags supported by the DSI DAL driver.
|
| Parameters  : device_handle - device_handle returned by the init API.
|	        write_tag - Write TAG
|		buf_len	 - Length of the buffer
|              buf	 - Pointer to REQ_RESP_BUF
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 dsi_write(U32 device_handle, U32 write_tag, U32 *len, U8 *buf)
{
	T_DSI_DIS *dsi_dis;
	T_REQ_RESP_BUF *in_out_buf;
	U8 data_type;
	S32 ret	= OMAP_DAL_SUCCESS;
	dsi_dis = (T_DSI_DIS *)device_handle;
	in_out_buf = (T_REQ_RESP_BUF *)buf;


	/*TODO: parameter validation */
	switch (write_tag) {

	case SEND_DCS_WRITE_CMD:


		if (in_out_buf->req_buf.len == 1)
			data_type = DCS_WRITE_SHORT_NO_PARAM_CMD;

		else if (in_out_buf->req_buf.len == 2)
			data_type = DCS_WRITE_SHORT_CMD;

		else if (in_out_buf->req_buf.len > 2)
			data_type = DCS_WRITE_LONG_CMD;

		else
			return OMAP_DAL_ERROR;

		ret = send_dsi_packet(data_type,
				dsi_dis->init.virtual_channel_id,
				WRITE_CMD,
				in_out_buf->req_buf.len,
				in_out_buf->req_buf.buf,
				in_out_buf->mode,
				dsi_dis->init.ecc_flag,
				dsi_dis->init.checksum_flag,
				in_out_buf->resp_buf.buf,
				in_out_buf->resp_buf.len);
		break;

	case DSI_DISPLAY_OUT:

		if (in_out_buf->type == ENABLE_VIDEO_TX) {

			printk(KERN_INFO "Enable Video TX \n\r");
			printk(KERN_INFO "dsi_write:dsi_vid_mode:\
				channel id=0x%x, hact=0x%x, pixel_format=\
				0x%x \n",\
				dsi_dis->init.virtual_channel_id,\
				dsi_dis->init.output_intf_config.hact,\
				dsi_dis->init.output_intf_config.pixel_format);

			dsi_video_mode(ENABLE_DSI_OUTPUT,
				dsi_dis->init.virtual_channel_id,
				dsi_dis->init.output_intf_config.hact,
				dsi_dis->init.output_intf_config.\
							pixel_format);

		} else if (in_out_buf->type == DISABLE_VIDEO_TX) {


			dsi_video_mode(DISABLE_DSI_OUTPUT,
				dsi_dis->init.virtual_channel_id,
				dsi_dis->init.output_intf_config.hact,
				dsi_dis->init.output_intf_config.pixel_format);


		}
		break;

	default:
		return OMAP_DAL_ERROR;

	}
	return ret;

}

/*-----------------------------------------------------------------------------
| Function    : S32 dsi_deinit(U32 device_handle)
+------------------------------------------------------------------------------
| Description : This function implements the DSI DAL deinit API. Implements the
|		deinitalization of the DSI module
|
| Parameters  : device_handle - device_handle returned by the init API.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 dsi_deinit(U32 device_handle)
{
	T_DSI_DIS *dsi_dis;
	S32 ret	= OMAP_DAL_SUCCESS;
	dsi_dis = (T_DSI_DIS *)device_handle;

	/* disable the DSI interface of OMAP */
	/* disable_dsi_interface(dsi_dis); */
	disable_dsi_interface();

	/* deinitialize the DSI platform specific configuration */
	ret = dsi_platform_deinit(dsi_dis);
	kfree(dsi_dis);
	printk(KERN_INFO "DSI driver deinit is successful \n\r");
	return ret;
}

/*TODO: support multi virtual channels simultaneously */
