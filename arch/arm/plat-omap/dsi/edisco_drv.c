/*
 * arch/arm/plat-omap/edisco_drv.c
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied
 *
 * The purpose of this file is to provide eDISCO DAL driver on the
 * CSST Target. This file implements the DAL layer API functions
 * for the eDISCO (enhanced Display Controller from Toshiba) on OMAP*
 *
 */

/*==== INCLUDES ============================================================*/
#include "edisco_drv.h"
#include "edisco_dis.h"
#include "dsi_dis.h"
#include "dsi_drv.h"
#include "dsi.h"
#include "types.h"
#include "edisco.h"

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/arch/hardware.h>
#include <asm/arch/display.h>

/*=============================== GLOBALS =================================*/

U8 write_cmd_mode = DSI_LPS_MODE_TX; /* DSI_HS_MODE_TX; */
U8 read_cmd_mode = DSI_LPS_MODE_TX; /* DSI_HS_MODE_TX; */

#define VIDEO_MODE_TX
/* #define CONFIG_READ_ENABLE */
U8 image_buffer[500];

T_EDISCO_DIS	*edisco_dis;
/*====================================FUNCTIONS =============================*/

/*-----------------------------------------------------------------------------
| Function    : S32 edisco_init(const void *initstr, U32 *device_handle)
+------------------------------------------------------------------------------
| Description : This function implements the DAL interface init API for
|		eDSICO driver. This function need to be called to initialize
|		the eDISCO DAL driver.
|
| Parameters  : initstr - pointer to the T_EDISCO_INIT structure
|               device_handle-handle/reference to the driver structure/instance
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
void edisco_init(U32 *device_handle)
{
/*	T_EDISCO_DIS	*edisco_dis; */
	S32 ret	= OMAP_DAL_SUCCESS;

	edisco_dis = (T_EDISCO_DIS *)kmalloc(sizeof(T_EDISCO_DIS), GFP_KERNEL);
	if (edisco_dis == NULL)
	    return;

	/* Initialize the edisco data strucs */
	edisco_dis->init.pixel_format = RGB565;
#ifdef CONFIG_FB_OMAP_LCD_VGA
	edisco_dis->init.image_width = 480;
	edisco_dis->init.image_height = 640;
	edisco_dis->init.display_size = VGA;
#else
	edisco_dis->init.image_width = 240;
	edisco_dis->init.image_height = 320;
	edisco_dis->init.display_size = QVGA;
#endif
	edisco_dis->init.pid  = 0;
	edisco_dis->init.sid  = 0;
	edisco_dis->init.yuv_to_rgb_conv = 2; /* YUV_TO_RGB_CONV_DISABLE; */

	*device_handle = (U32)edisco_dis;

		get_edisco_platform_config(edisco_dis);

	/* TODO -- hardcode the config vlaues here */
#ifdef CONFIG_FB_OMAP_LCD_VGA
	edisco_dis->output_intf_config.hact = 480;
	edisco_dis->output_intf_config.vact = 640;
#else
	edisco_dis->output_intf_config.hact = 240;
	edisco_dis->output_intf_config.vact = 320;
#endif
	edisco_dis->output_intf_config.hsa = 2;
	edisco_dis->output_intf_config.hbp = 38;
	edisco_dis->output_intf_config.hfp = 44;
	edisco_dis->output_intf_config.vsa = 1;
	edisco_dis->output_intf_config.vbp = 1;
	edisco_dis->output_intf_config.vfp = 1;
#ifdef CONFIG_FB_OMAP_LCD_VGA
	edisco_dis->output_intf_config.display_size = VGA;
	edisco_dis->output_intf_config.pixel_clk = 25;
#else
	edisco_dis->output_intf_config.display_size = QVGA;
	edisco_dis->output_intf_config.pixel_clk = 6;
#endif
	edisco_dis->output_intf_config.hsync_pol = ACTIVE_LOW;
	edisco_dis->output_intf_config.vsync_pol = ACTIVE_LOW;
	edisco_dis->output_intf_config.dataen_pol = ACTIVE_HIGH;
	edisco_dis->output_intf_config.clk_pol = RISING_EDGE;
	edisco_dis->output_intf_config.interface_width = 0x5;/*TODO: confirm?*/
	edisco_dis->output_intf_config.pixel_format = RGB565;

	get_edisco_vm_timing(&edisco_dis->video_mode_timing, \
				edisco_dis->init.display_size);
	ret = init_dsi_driver(edisco_dis);
	if (ret != OMAP_DAL_SUCCESS) {

		kfree(edisco_dis);
		return ;
	}

	ret = init_edisco(edisco_dis);
	if (ret != OMAP_DAL_SUCCESS) {

		disable_edisco_power();
		dsi_deinit(edisco_dis->dsi_handle);
		kfree(edisco_dis);
		return ;
	}
	return ;
}
EXPORT_SYMBOL(edisco_init);

void edisco_get_handle(U32 *handle)
{
	*handle = (U32) edisco_dis;
}
EXPORT_SYMBOL(edisco_get_handle);

/*-----------------------------------------------------------------------------
| Function    : S32 edisco_read(U32 device_handle, U32 tag, U32 *len, U8 *buf)
+------------------------------------------------------------------------------
| Description : This function implements the DAL read API. Implements the
|		READ tags supported by eDISCO DAL layer.
|
| Parameters  : device_handle - device_handle returned by the init API.
|	        read_tag - Read TAG
|		len - Length of the buffer
|              buf - Pointer to buffer that needs to be used for return data
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 edisco_read(U32 device_handle, U32 read_tag, U32 *buf_len, U8 *buf)
{
	T_EDISCO_DIS *edisco_dis;
	T_REQ_RESP_BUF *in_out_buf;
	S32 ret	= OMAP_DAL_SUCCESS;
	U32 len;
	edisco_dis = (T_EDISCO_DIS *)device_handle;

	/*:TODO: Need to validate the parameters */

	switch (read_tag) {

	case READ_DEVICE_ID:

		if ((*buf_len != 3) || (buf == 0))
			return OMAP_DAL_ERROR;

		ret = read_edisco_id(edisco_dis->dsi_handle, buf);
		if (ret != OMAP_DAL_SUCCESS)
			return OMAP_DAL_ERROR;

		break;

	case EDISCO_DCS_READ_CMD:

		in_out_buf = (T_REQ_RESP_BUF *)buf;
		len = sizeof(T_REQ_RESP_BUF);
		ret = dsi_read(edisco_dis->dsi_handle,\
			SEND_DCS_READ_CMD, &len, (U8 *)in_out_buf);
		if (ret != OMAP_DAL_SUCCESS)
			return OMAP_DAL_ERROR;

		break;

	default:
		return OMAP_DAL_ERROR;
	}
	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 edisco_write(U32 device_handle, U32 write_tag,
|		 U32 *len, U8* buf)
+------------------------------------------------------------------------------
| Description : This function implements the DAL write API. Implements the
|		WRITE Tags supported by the eDISCO DAL driver.
|
| Parameters  : device_handle - device_handle returned by the init API.
|	        write_tag - write TAG
|		buf_len	  - Length of the buffer
|               buf	  - Pointer to request buffer
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
/* S32 edisco_write(U32 device_handle, U32 write_tag, U32 *buf_len, U8 *buf)*/
void edisco_write(U32 device_handle, U32 write_tag, U32 *buf_len, U8 *buf)
{
	T_EDISCO_DIS *edisco_dis;
	T_REQ_RESP_BUF *in_out_buf;
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;
	edisco_dis = (T_EDISCO_DIS *)device_handle;

	/*:TODO: Need to validate the parameters */

	switch (write_tag) {

	case DISPLAY_OUT:
		if (*buf == DISPLAY_OUT_ENABLE) {
			ret = start_image_display(edisco_dis->dsi_handle);
			if (ret != OMAP_DAL_SUCCESS)
				return;

		} else if (*buf == DISPLAY_OUT_DISABLE) {
			ret = stop_image_display(edisco_dis->dsi_handle);
			if (ret != OMAP_DAL_SUCCESS)
				return;

		}
		break;

	case COLORBAR_DISPLAY:
		/*TODO: Received mail from Toshiba saying colorbar output is
		* 640x480 and cannot be changed, means, we will not be able
		* to use this case since our LCD is 480x640 */
		if (*buf == DISPLAY_OUT_ENABLE) {
			ret = start_colorbar_display(edisco_dis->dsi_handle);
			if (ret != OMAP_DAL_SUCCESS)
				return;

		} else if (*buf == DISPLAY_OUT_DISABLE) {
			ret = stop_colorbar_display(edisco_dis->dsi_handle);
			if (ret != OMAP_DAL_SUCCESS)
				return;

		}
		break;

	case EDISCO_DCS_WRITE_CMD:

		in_out_buf = (T_REQ_RESP_BUF *)buf;
		len = sizeof(T_REQ_RESP_BUF);
		ret = dsi_write(edisco_dis->dsi_handle, \
			SEND_DCS_WRITE_CMD, &len, (U8 *)in_out_buf);
		if (ret != OMAP_DAL_SUCCESS)
			return;

		break;

	default:
		return;
	}
	return;
}
EXPORT_SYMBOL(edisco_write);

/*-----------------------------------------------------------------------------
| Function    : S32 dsi_deinit(U32 device_handle)
+------------------------------------------------------------------------------
| Description : This function implements the eDISCO DAL deinit API. Implements
|		the deinitalization of the eDISCO module
|
| Parameters  : device_handle - device_handle returned by the init API.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
void edisco_deinit(U32 device_handle)
{
	S32 ret	= OMAP_DAL_SUCCESS;
	ret = disable_edisco_power();
	ret = dsi_deinit(edisco_dis->dsi_handle);
	kfree(edisco_dis);
	return ;
}
EXPORT_SYMBOL(edisco_deinit);

/*-----------------------------------------------------------------------------
| Function    : S32 init_dsi_driver(T_EDISCO_DIS *edisco_dis)
+------------------------------------------------------------------------------
| Description : This function initialize the DSI driver. The handle return by
|		the DSI DAL layer will be used for further communication to
|               DSI module
|
| Parameters  : edisco_dis - pointer to the EDISCO DIS.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 init_dsi_driver(T_EDISCO_DIS *edisco_dis)
{
	S32 ret;
	T_DSI_INIT dsi_init_str;

	dsi_init_str.pid = 0;
	dsi_init_str.sid = 0;

	dsi_init_str.video_mode_timing = edisco_dis->video_mode_timing;
	dsi_init_str.output_intf_config = edisco_dis->output_intf_config;
	dsi_init_str.image_width = edisco_dis->init.image_width;
	dsi_init_str.image_height = edisco_dis->init.image_height;
	dsi_init_str.pixel_format = edisco_dis->init.pixel_format;
	dsi_init_str.num_data_lanes = edisco_dis->num_data_lanes;
	dsi_init_str.virtual_channel_id = edisco_dis->virtual_channel_id;
	dsi_init_str.data_lane_pol = edisco_dis->data_lane_pol;
	dsi_init_str.clk_lane_pol = edisco_dis->clk_lane_pol;
	dsi_init_str.ecc_flag = edisco_dis->ecc_flag;
	dsi_init_str.checksum_flag = edisco_dis->checksum_flag;
	dsi_init_str.max_read_resp_size = edisco_dis->max_resp_size;
	dsi_init_str.display_mode = edisco_dis->display_mode;
	dsi_init_str.yuv_to_rgb_conv = edisco_dis->init.yuv_to_rgb_conv;
	dsi_init_str.frame_buffer = edisco_dis->init.frame_buffer;

	ret = dsi_init(&dsi_init_str, &edisco_dis->dsi_handle);

	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 init_edisco(T_EDISCO_DIS *edisco_dis)
+------------------------------------------------------------------------------
| Description : This function initialize the eDISCO chip. This functions sends
|		DCS commands to the eDISCO device to initalize it for MIPI DPI
|               output mode
|
| Parameters  : edisco_dis - pointer to the EDISCO DIS.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 init_edisco(T_EDISCO_DIS *edisco_dis)
{
	S32 ret	= OMAP_DAL_SUCCESS;

	ret = enable_edisco_power();
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	/* Added delay for eDISCO to be stabilize */
	mdelay(100); /* dl_lazy_delay(ONE_SECOND); */
	/*TODO: need to fine tune the below sequence based on
	 * the input from SolDel  */

	ret = exit_sleep_mode(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	ret = enable_special_cmd_set(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	ret = disable_eot_reception(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



#ifdef CONFIG_READ_ENABLE
	ret = get_diag_result(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


#endif

	ret = set_pixel_format(&edisco_dis->output_intf_config, \
				edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


#ifdef CONFIG_READ_ENABLE
	ret = get_pixel_format(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


#endif

	ret = set_output_pixel_clk(&edisco_dis->output_intf_config, \
			edisco_dis->dsi_handle, edisco_dis->input_clk_fre);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


#ifdef CONFIG_READ_ENABLE
	ret = get_buffer_io_line(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = get_output_pclk_freq(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

#endif

	ret = set_output_vertical_timing(&edisco_dis->output_intf_config,\
						edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



#ifdef CONFIG_READ_ENABLE
	ret = get_output_vertical_front_porch(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = get_output_vertical_sync_pulse(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = get_output_vertical_back_porch(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = get_output_vertical_act_lines_msb(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = get_output_vertical_act_lines_csb(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = get_output_vertical_act_lines_lsb(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


#endif

	ret = set_output_horizontal_timing(&edisco_dis->output_intf_config, \
		edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


#ifdef CONFIG_READ_ENABLE
	ret = get_output_horizontal_front_porch(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = get_output_horizontal_sync_pulse(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = get_output_horizontal_back_porch(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = get_output_horizontal_act_lines_msb(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = get_output_horizontal_act_lines_csb(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = get_output_horizontal_act_lines_lsb(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


#endif


	ret = set_output_interface_signal_polarity(&edisco_dis->\
				output_intf_config, edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	ret = set_output_interface_pixel_format(\
		&edisco_dis->output_intf_config, edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	/* Added to change the special command set to disable */
	ret = enable_special_cmd_set(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	ret = set_image_size(edisco_dis->init.image_width,\
			edisco_dis->init.image_height, edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



#ifdef CONFIG_READ_ENABLE
	ret = get_power_mode(edisco_dis->dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


#endif

	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 read_edisco_id(U32 dis_handle, U8 *buf)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to get the
|		device ID of eDISCO
|
| Parameters  : dis_handle - Handle to the DSI driver instance.
|		buf	   - return buffer
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 read_edisco_id(U32 dsi_handle, U8 *buf)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd_buf[1], dcs_cmd_resp[4];
	S32 ret	= OMAP_DAL_SUCCESS;
	U32 len;

	dcs_cmd_buf[0] = DCS_GET_ID1;
	dcs_buf.req_buf.buf = dcs_cmd_buf;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.req_buf.len = 1;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 start_image_display(U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to set the
|		display out to ON and enable the DSI interface of the host(OMAP)
|
| Parameters  : dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 start_image_display(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf, param_buf;
	U8 dcs_cmd[1];
	U32 len;
	/* U32 countj, counti = 19; */
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_SET_DISPLAY_ON;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

#ifdef VIDEO_MODE_TX
	param_buf.type = ENABLE_VIDEO_TX;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, DSI_DISPLAY_OUT, &len, (U8 *)&param_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;
#else
	while (counti--) {
		ret = start_write_to_edisco_buffer(dsi_handle);
		if (ret != OMAP_DAL_SUCCESS)
			return ret;

		for (countj = 0; countj < (640*2-1) ; countj++) {
			ret = continue_write_to_edisco_buffer(dsi_handle);
			if (ret != OMAP_DAL_SUCCESS)
				return ret;
		}
	}
#endif


	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 stop_image_display(U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to set the
|		display out to OFF and disable the DSI interface of
|		the host (OMAP)
| Parameters  : dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 stop_image_display(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf, param_buf;
	U8 dcs_cmd[1];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

#ifdef VIDEO_MODE_TX
	param_buf.type = DISABLE_VIDEO_TX;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, DSI_DISPLAY_OUT, &len, (U8 *)&param_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;
#endif

	dcs_cmd[0] = DCS_SET_DISPLAY_OFF;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}
/*-----------------------------------------------------------------------------
| Function    : S32 start_colorbar_display(U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to display
|		internal generated colorbar from eDISCO device.
|
| Parameters  : dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 start_colorbar_display(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[5];
	U32 len;

	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_SET_DISPLAY_ON;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	dcs_cmd[0] = DCS_WRITE_EDISCO_REGISTER;
	dcs_cmd[1] = 0x20;
	dcs_cmd[2] = 0x00;
	dcs_cmd[3] = 0x02; /* enable colorbar */
	/*TODO:need to check the output inf and select the pixel format */
	dcs_cmd[4] = 0x44; /* RGB888 + continuous output */
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 5;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = enable_special_cmd_set(dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 stop_colorbar_display(U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to stop
|		display colorbar generation
|
| Parameters  : dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 stop_colorbar_display(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[5];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_SET_DISPLAY_OFF;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = write_cmd_mode;
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	ret = enable_special_cmd_set(dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	dcs_cmd[0] = DCS_WRITE_EDISCO_REGISTER;
	dcs_cmd[1] = 0x20;
	dcs_cmd[2] = 0x00;
	dcs_cmd[3] = 0x00; /* disable colorbar */
	/*TODO:need to check the output inf and select the pixel format */
	dcs_cmd[4] = 0x04; /* RGB888 + continuous output */
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 5;
	dcs_buf.mode = write_cmd_mode;
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	ret = enable_special_cmd_set(dsi_handle);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 reset_edisco(U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : Sends DCS commnads to eDISCO device to reset edisco
|
| Parameters  : dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 reset_edisco(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	/*TODO: probably this can be removed if hard reset is enough */
	dcs_cmd[0] = DCS_SOFT_RESET;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = DSI_LPS_MODE_TX;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	/*TODO: need to check whether this delay is enough */
	mdelay(5); /* dl_lazy_delay(500); */

	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 exit_sleep_mode(U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to set it out
|		of the Sleep Mode
|
| Parameters  : dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 exit_sleep_mode(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_EXIT_SLEEP_MODE;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = DSI_LPS_MODE_TX;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	/*TODO: need to check whether this delay is enough */
	mdelay(500); /* dl_lazy_delay(500000); */
	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 set_pixel_format(T_DPI_OUT_CONFIG *lcd_config,
|		U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to set the
|               pixel format of the video data transfered over DSI interface
|
| Parameters  : lcd_config - LCD device configuration.
|		dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 set_pixel_format(T_DPI_OUT_CONFIG *lcd_config, U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[2];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_SET_PIXEL_FORMAT;

	if (lcd_config->pixel_format == RGB565)
		dcs_cmd[1] = 0x05;

	else if (lcd_config->pixel_format == RGB666)
		dcs_cmd[1] = 0x06;


	else if (lcd_config->pixel_format == RGB888)
		dcs_cmd[1] = 0x07;


	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 2;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 set_output_pixel_clk(T_DPI_OUT_CONFIG *lcd_config,
|		U32 dsi_handle, U8 edisco_refclk)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to
|		set the pixel clock frequency of the LCD device connected
|		to MIPI DPI output interface
|
| Parameters  : lcd_config - LCD device configuration.
|		dis_handle - Handle to the DSI driver instance.
|		edisco_refclk - Reference clk given to eDISCO
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 set_output_pixel_clk(T_DPI_OUT_CONFIG *lcd_config,\
				U32 dsi_handle, U8 edisco_refclk)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[2], num_channels = 0, ocf_value = 0;
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	if ((lcd_config->pixel_clk == 25) && (edisco_refclk == REFCLK_32KHZ)) {
		ocf_value = 0xD3; /* 0x13;  for ES2.0 */
		num_channels = 2;

	} else if ((lcd_config->pixel_clk == 6) &&\
			(edisco_refclk == REFCLK_32KHZ)) {
		ocf_value = 0xA6; /* 0x66; for ES2.0 */
		num_channels = 1;

	} else if ((lcd_config->pixel_clk == 25) &&\
			(edisco_refclk == REFCLK_19P2MHZ)) {
		ocf_value = 0x13;
		num_channels = 2;
	} else if ((lcd_config->pixel_clk == 6) &&\
			(edisco_refclk == REFCLK_19P2MHZ)) {

		ocf_value = 0x62;
		num_channels = 1;
	}

	/* configure the number of channels, need to select 2 channels
	 * to get the PCLK upto 30MHz need to select only 1 channel
	 * to get PCLK less than 8MHz */
	dcs_cmd[0] = DCS_SET_DISPLAY_BUFFER_IO_CONTROL;
	dcs_cmd[1] = (num_channels == 2) ? (1) : (0);
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 2;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;




	/* TODO: need to support all the pixel_clk freq supported by eDISCO */
	dcs_cmd[0] = DCS_SET_OUTPUT_PIXEL_CLK_FREQ;
	dcs_cmd[1] = ocf_value;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 2;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 set_output_vertical_timing(T_DPI_OUT_CONFIG *lcd_config,
|		U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to set the
|		vertical timing parameters of the MIPI DPI output interface
|
| Parameters  : lcd_config - LCD device configuration.
|		dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 set_output_vertical_timing(T_DPI_OUT_CONFIG *lcd_config, U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[7];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_SET_OUTPUT_VERTICAL_TIMING;
	dcs_cmd[1] = lcd_config->vfp;
	dcs_cmd[2] = lcd_config->vsa;
	dcs_cmd[3] = lcd_config->vbp;
	dcs_cmd[4] = 0;
	dcs_cmd[5] = (lcd_config->vact >> 8);
	dcs_cmd[6] = (lcd_config->vact);

	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 7;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}


/*-----------------------------------------------------------------------------
| Function    : S32 set_output_horizontal_timing(T_DPI_OUT_CONFIG *lcd_config,
|		U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to set the
|		horizontal timing parameters of the MIPI DPI output interface
|
| Parameters  : lcd_config - LCD device configuration.
|		dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 set_output_horizontal_timing(T_DPI_OUT_CONFIG *lcd_config, U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[7];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_SET_OUTPUT_HORIZONTAL_TIMING;
	dcs_cmd[1] = lcd_config->hfp;
	dcs_cmd[2] = lcd_config->hsa;
	dcs_cmd[3] = lcd_config->hbp;
	dcs_cmd[4] = 0;
	dcs_cmd[5] = (lcd_config->hact >> 8);
	dcs_cmd[6] = (lcd_config->hact);

	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 7;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 enable_special_cmd_set(U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to enable the
|		special command set supported by eDISCO. Calling this function
|		twice will disable the special command set of eDISCO
|
| Parameters  : dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 enable_special_cmd_set(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[5];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_SET_SPECIAL_COMMAND_SET;
	dcs_cmd[1] = 0x03;
	dcs_cmd[2] = 0x7F;
	dcs_cmd[3] = 0x5C;
	dcs_cmd[4] = 0x33;

	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 5;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 set_output_interface_signal_polarity(T_DPI_OUT_CONFIG
|		*lcd_config, U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to set the
|		polarity of the HSync, VSync, and DE signals of MIPI DPI
|		interface. This is an extended DCS command, will have effect
|		when send after enabling the special command set of eDISCO
|
| Parameters  : lcd_config - LCD device configuration.
|		dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 set_output_interface_signal_polarity(T_DPI_OUT_CONFIG *lcd_config,\
						U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[5];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_WRITE_EDISCO_REGISTER;
	dcs_cmd[1] = 0x24;
	dcs_cmd[2] = 0x00;
	dcs_cmd[3] = 0x00;
	dcs_cmd[4] = 0x00;

	dcs_cmd[4] = (lcd_config->hsync_pol == ACTIVE_HIGH) ? (dcs_cmd[4] |\
			(1 << 1)) : (dcs_cmd[4]);
	dcs_cmd[4] = (lcd_config->dataen_pol == ACTIVE_HIGH) ? (dcs_cmd[4]) :\
			(dcs_cmd[4] | (1 << 2));
	dcs_cmd[4] = (lcd_config->vsync_pol == ACTIVE_HIGH) ? (dcs_cmd[4] |\
			(1 << 3)) : (dcs_cmd[4]);

	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 5;
	dcs_buf.mode = write_cmd_mode;
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;
	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 set_output_interface_pixel_format(T_DPI_OUT_CONFIG
|		*lcd_config, U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to set the
|		pixel format of the data send over MIPI DPI output interface.
|		This is an extended DCS command, will have effect when send
|		after enabling the special command set of eDISCO.
|
| Parameters  : lcd_config - LCD device configuration.
|		dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 set_output_interface_pixel_format(T_DPI_OUT_CONFIG *lcd_config,\
						U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[5];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_WRITE_EDISCO_REGISTER;
	dcs_cmd[1] = 0x20;
	dcs_cmd[2] = 0x00;
	dcs_cmd[3] = 0x00;

	if (lcd_config->pixel_format == RGB565)
		dcs_cmd[4] = 0x44;

	else if (lcd_config->pixel_format == RGB666)
		dcs_cmd[4] = 0x04;


	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 5;
	dcs_buf.mode = write_cmd_mode;
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

S32 disable_eot_reception(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[5];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_WRITE_EDISCO_REGISTER;
	dcs_cmd[1] = 0xC8;
	dcs_cmd[2] = 0x00;
	dcs_cmd[3] = 0xC1;
	dcs_cmd[4] = 0x40;

	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 5;
	dcs_buf.mode = DSI_LPS_MODE_TX; /* write_cmd_mode; */
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}


/*-----------------------------------------------------------------------------
| Function    : S32 set_image_size(U16 image_width, U16 image_height,
|		U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : This function sends DCS commnads to eDISCO device to set
|		size of the image transfered to the eDISCO's frame buffer.
|		DCS commands set_column_address and set_page_address will be
|		send to eDISCO to do this
|
| Parameters  :image_width - Width of the image to be transfered to eDISCO
| 		image_height - Height of the image to be transfered to eDISCO
|		dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 set_image_size(U16 image_width, U16 image_height, U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[7];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_SET_COLUMN_ADDRESS;
	dcs_cmd[1] = 0x00;
	dcs_cmd[2] = 0x00;
	dcs_cmd[3] = 0x00;
	dcs_cmd[4] = 0x00;
	dcs_cmd[5] = image_width >> 8;
	dcs_cmd[6] = image_width;

	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 7;
	dcs_buf.mode = write_cmd_mode;
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	dcs_cmd[0] = DCS_SET_PAGE_ADDRESS;
	dcs_cmd[1] = 0x00;
	dcs_cmd[2] = 0x00;
	dcs_cmd[3] = 0x00;
	dcs_cmd[4] = 0x00;
	dcs_cmd[5] = image_height >> 8;
	dcs_cmd[6] = image_height;

	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 7;
	dcs_buf.mode = write_cmd_mode;
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 get_diag_result(U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : Sends DCS commnads Get Diag Results to eDISCO device
|
| Parameters  : dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 get_diag_result(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_DIAG_RESULT;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


/*	dl_lazy_delay(ONE_SECOND); */
	return ret;
}


/*-----------------------------------------------------------------------------
| Function    : S32 set_display_on(U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : Sends DCS commnads Set Display ON command to eDISCO device
|
| Parameters  : dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 set_display_on(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_SET_DISPLAY_ON;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);

	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}


/*-----------------------------------------------------------------------------
| Function    : S32 get_diag_result(U32 dsi_handle)
+------------------------------------------------------------------------------
| Description : Sends DCS commnads Get Diag Results to eDISCO device
|
| Parameters  : dis_handle - Handle to the DSI driver instance.
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 get_pixel_format(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_PIXEL_FORMAT;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}

S32 get_scan_line(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_SCAN_LINE;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}

S32 get_buffer_io_line(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_DISPLAY_BUF_IO_CTRL;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}

S32 get_output_vertical_front_porch(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_VERTICAL_FRONT_PORCH;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

}


S32 get_output_vertical_sync_pulse(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_VERTICAL_SYNC_PULSE;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}

S32 get_output_vertical_back_porch(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_VERTICAL_BACK_PORCH;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}


S32 get_output_vertical_act_lines_msb(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_VERTICAL_ACT_LINES_MSB;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

S32 get_output_vertical_act_lines_csb(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_VERTICAL_ACT_LINES_CSB;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

S32 get_output_vertical_act_lines_lsb(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_VERTICAL_ACT_LINES_LSB;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

S32 get_output_horizontal_front_porch(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_HORIZONTAL_FRONT_PORCH;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}


S32 get_output_horizontal_sync_pulse(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_HORIZONTAL_SYNC_PULSE;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

S32 get_output_horizontal_back_porch(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_HORIZONTAL_BACK_PORCH;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}


S32 get_output_horizontal_act_lines_msb(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_HORIZONTAL_ACT_LINES_MSB;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

S32 get_output_horizontal_act_lines_csb(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_HORIZONTAL_ACT_LINES_CSB;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

S32 get_output_horizontal_act_lines_lsb(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_HORIZONTAL_ACT_LINES_LSB;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

S32 get_output_pclk_freq(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_OUTPUT_PCLK_FREQ;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

S32 read_eDISCO_spec_reg(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[2], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_WRITE_INDEX_REGISTER;
	dcs_cmd[1] = 0x23; /* Register address */
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 2;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);

	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;



	dcs_cmd[0] = DCS_READ_EDISCO_REGISTER;

	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

S32 enable_reg_settings(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[5];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_WRITE_EDISCO_REGISTER;
	dcs_cmd[1] = 0x28;
	dcs_cmd[2] = 0x00;
	dcs_cmd[3] = 0x00;
	dcs_cmd[4] = 0x01;

	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 5;
	dcs_buf.mode = write_cmd_mode;
	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}


S32 get_power_mode(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_POWER_MODE;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}


S32 get_address_mode(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_ADDRESS_MODE;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

S32 get_signal_mode(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], dcs_cmd_resp[4];
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;

	dcs_cmd[0] = DCS_GET_SIGNAL_MODE;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 4;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	return ret;
}

S32 start_write_to_edisco_buffer(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 *dcs_cmd = image_buffer;
	U32 len, buf_len, packet_len;
	S32 ret	= OMAP_DAL_SUCCESS;
	static U32 flag;

	packet_len = (240 * 2) + 1;

	dcs_cmd[0] = DCS_WRITE_MEMORY_START;

	for (buf_len = 0; buf_len < (packet_len - 1); buf_len += 2) {

		if ((flag % 3) == 0) {

			dcs_cmd[buf_len+1] = 0xF8;
			dcs_cmd[buf_len+2] = 0x00;

		} else if ((flag % 3) == 1) {

			dcs_cmd[buf_len+1] = 0x00;
			dcs_cmd[buf_len+2] = 0x1F;

		} else if ((flag % 3) == 2) {

			dcs_cmd[buf_len+1] = 0x07;
			dcs_cmd[buf_len+2] = 0xE0;
		}
	}
	flag++;

	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = packet_len;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);

	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS) {

		kfree(dcs_cmd);
		return ret;
	}
	kfree(dcs_cmd);
	return ret;
}

S32 continue_write_to_edisco_buffer(U32 dsi_handle)
{
	T_REQ_RESP_BUF dcs_buf;
	U8 *dcs_cmd = image_buffer;
	U32 len, packet_len;
	S32 ret	= OMAP_DAL_SUCCESS;

	packet_len = (240 * 2) + 1;

	dcs_cmd[0] = DCS_WRITE_MEMORY_CONTINUE;

	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = packet_len;
	dcs_buf.mode = write_cmd_mode;
	len = sizeof(T_REQ_RESP_BUF);

	ret = dsi_write(dsi_handle, SEND_DCS_WRITE_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS) {

		kfree(dcs_cmd);
		return ret;
	}
	kfree(dcs_cmd);
	return ret;
}

S32 start_read_from_edisco_buffer(U32 dsi_handle)
{

	T_REQ_RESP_BUF dcs_buf;
	U8 dcs_cmd[1], *dcs_cmd_resp;
	U32 len;
	S32 ret	= OMAP_DAL_SUCCESS;


	dcs_cmd_resp = kmalloc(100, GFP_KERNEL);
	if (dcs_cmd_resp == NULL)
		return OMAP_DAL_SUCCESS;



	dcs_cmd[0] = DCS_READ_MEMORY_START;
	dcs_buf.req_buf.buf = dcs_cmd;
	dcs_buf.req_buf.len = 1;
	dcs_buf.mode = read_cmd_mode;
	dcs_buf.resp_buf.buf = dcs_cmd_resp;
	dcs_buf.resp_buf.len = 20;
	len = sizeof(T_REQ_RESP_BUF);
	ret = dsi_read(dsi_handle, SEND_DCS_READ_CMD, &len, (U8 *)&dcs_buf);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;


	kfree(dcs_cmd_resp);

	return ret;
}
