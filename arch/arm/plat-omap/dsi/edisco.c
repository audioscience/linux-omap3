/*
 * arch/arm/plat-omap/edisco.h
 *
 * This file contains the functions which provides the eDSICO settings
 * specific to the SDP3430 platform
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

/*========== INCLUDES======================================================*/
#include "types.h"
#include "dsi_dis.h"
#include "edisco_dis.h"
#include "edisco_drv.h"
#include "edisco.h"
#include "dsi.h"

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

/*============ FUNCTIONS ====================================================*/

/*-----------------------------------------------------------------------------
| Function    : get_edisco_platform_config(T_EDISCO_DIS *edisco_dis)
+------------------------------------------------------------------------------
| Description : Return the eDISCO settings specific to the SDP3430 platform
|
| Parameters  : edisco_dis - pointer to the edisco device information structure
|
| Returns     : None
+----------------------------------------------------------------------------*/
void get_edisco_platform_config(T_EDISCO_DIS *edisco_dis)
{
	edisco_dis->input_interface = MIPI_DSI;
	edisco_dis->output_interface = MIPI_DPI;
	/*TODO: need to be filled with the data from edisco datasheet */
	edisco_dis->max_data_rate = 0;
	edisco_dis->virtual_channel_id = 0;
#ifndef NB_DATALANES_1
	edisco_dis->num_data_lanes = 2; /* changed on 6th Sept */
#else
	edisco_dis->num_data_lanes = 1; /* changed on 6th Sept */
#endif
	edisco_dis->data_lane_pol = LANE_POL_XP_YN;
	edisco_dis->clk_lane_pol = LANE_POL_XP_YN;
	edisco_dis->ecc_flag = ECC_ENABLE;
	edisco_dis->checksum_flag = CHECKSUM_ENABLE;
	/*TODO: need to be filled with the data from edisco datasheet */
	edisco_dis->max_resp_size = 0;
	edisco_dis->display_mode = DSI_VIDEO_MODE;
	edisco_dis->input_clk_fre = REFCLK_32KHZ;
}

/*-----------------------------------------------------------------------------
| Function    : void get_edisco_vm_timing(T_VM_TIMING
|					*video_mode_timing, U16 image_size)
+------------------------------------------------------------------------------
| Description : Return the eDISCO Video Mode timing parameters
|
| Parameters  : video_mode_timing - pointer to the Video Mode timing structure
|		image_size - size of the image to be displayed on DPI interface
|
| Returns     : None
+----------------------------------------------------------------------------*/
void get_edisco_vm_timing(T_VM_TIMING *video_mode_timing, U16 image_size)
{
	if (image_size == VGA) {

		video_mode_timing->hact = 480;
		video_mode_timing->vact = 640;
	} else if (image_size == QVGA) {

		video_mode_timing->hact = 240;
		video_mode_timing->vact = 320;
	}
	/*TODO: this need to be checked, how to fill */
	video_mode_timing->bit_rate = 300;
	video_mode_timing->time_hbp = 0;
	video_mode_timing->time_hact = 0;
	video_mode_timing->time_hfp = 0;
	/*TODO: this need to be checked, how to fill */
	video_mode_timing->time_hsa = 0;
	video_mode_timing->vsa = 1;
	video_mode_timing->vbp = 0;
	video_mode_timing->vfp = 0;
	video_mode_timing->line_time = 2 * (video_mode_timing->time_hbp +
					    video_mode_timing->hact +
					    video_mode_timing->time_hfp +
					    video_mode_timing->time_hsa);
}

/*-----------------------------------------------------------------------------
| Function    : S32 enable_edisco_power()
+------------------------------------------------------------------------------
| Description : Enable the power to the eDISCO chip
|
| Parameters  : None
|
| Returns     : Status (OMAP_DAL_SUCCESS or Error)
+----------------------------------------------------------------------------*/
S32 enable_edisco_power(void)
{
	S32 ret = OMAP_DAL_SUCCESS;

	return ret;
}


/*-----------------------------------------------------------------------------
| Function    : S32 disable_edisco_power()
+------------------------------------------------------------------------------
| Description : Disable the power to eDISCO chip
|
| Parameters  : None
|
| Returns     : Status (OMAP_DAL_SUCCESS or Error)
+----------------------------------------------------------------------------*/
S32 disable_edisco_power(void)
{
	S32 ret = OMAP_DAL_SUCCESS;

	/* TODO -- Disbale edisco power */
	return ret;
}

