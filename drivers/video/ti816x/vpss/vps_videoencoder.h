/*
 *
 * external video encoders/HDMI transmitter header for TI 816X
 *
 * Copyright (C) 2009 TI
 * Author: Yihe Hu <yihehu@ti.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA	02111-1307, USA.
 */

#ifndef __DRIVERS_VIDEO_TI816X_VPSS_VPS_VIDEOENCODER_H_
#define __DRIVERS_VIDEO_TI816X_VPSS_VPS_VIDEOENCODER_H_

/**
  * \brief Get Chip ID
  *
  * This IOCTL can be used to get video  encoder chip information
  * like chip number, revision, firmware/patch revision
  *
  *
  * \param cmdArgs       [IN/OUT]  Vps_VideoEncoderChipId *
  * \param cmdArgsStatus [OUT] NULL
  *
  * \return FVID_SOK on success, else failure
  *
*/
#define IOCTL_VPS_VIDEO_ENCODER_GET_CHIP_ID       \
	(VPS_VID_ENC_IOCTL_BASE + 0x00)

/**
  * \brief Configure HDMI
  *
  * This IOCTL can be used to configure HDMI for mode.
  *
  *
  * \param cmdArgs       [IN]  Vps_VideoEncoderConfigParams *
  * \param cmdArgsStatus [OUT] NULL
  *
  * \return FVID_SOK on success, else failure
  *
*/
#define IOCTL_VPS_VIDEO_ENCODER_SET_MODE       \
	(VPS_VID_ENC_IOCTL_BASE + 0x01)


/**
 * \brief Enum defining ID of the standard Modes.
 *
 *  Standard timinig parameters
 *  will be used if the standard mode id is used for configuring mode
 *  in the hdmi.
 */
enum vps_videoencoderoutputmodeid {
	VPS_VIDEO_ENCODER_MODE_NTSC = 0,
	/**< Mode Id for NTSC */
	VPS_VIDEO_ENCODER_MODE_PAL,
	/**< Mode Id for PAL */
	VPS_VIDEO_ENCODER_MODE_1080P_60,
	/**< Mode Id for 1080p at 60fps mode */
	VPS_VIDEO_ENCODER_MODE_720P_60,
	/**< Mode Id for 720p at 60fps mode */
	VPS_VIDEO_ENCODER_MODE_1080I_60,
	/**< Mode Id for 1080I at 60fps mode */
	VPS_VIDEO_ENCODER_MODE_1080P_30,
	/**< Mode Id for 1080P at 30fps mode */
	VPS_VIDEO_ENCODER_MAX_MODE
	/**< This should be the last mode id */
};

/* \brief Enum defining external or embedded sync mode. */

enum vps_videoencodersyncmode {
	VPS_VIDEO_ENCODER_EXTERNAL_SYNC,
	/**< HDMI in external sync mode i.e. H-sync and V-sync are external */
	VPS_VIDEO_ENCODER_EMBEDDED_SYNC,
	/**< Embedded sync mode */
	VPS_VIDEO_ENCODER_MAX_SYNC
	/**< This should be the last mode id */
} ;

/* \brief Structure for getting HDMI chip identification Id */
struct vps_videoencoderchipid {
	u32                  chipid;
	/**< Chip ID, value is device specific */
	u32                  chiprevision;
	/**< Chip revision, value is device specific  */
	u32                  firmwareversion;
	/**< Chip internal patch/firmware revision, value is device specific */
};


/* \brief Arguments for FVID2_create() */
struct vps_videoencodercreateparams {

	u32  devicei2cinstId;
	/**< I2C device instance ID to use 0 or 1 */
	u32  devicei2cAddr;
	/**< I2C device address for each device */
	u32  inpclk;
	/**< input clock*/
	u32  hdmihotplughpiointrline;
	/**< HDMI hot plug GPIO interrupt line no */
} ;


/* \brief Status of FVID2_create() */
struct vps_videoencodercreatestatus {
	int   retval;
	/**< FVID2_SOK on success, else failure */
} ;



/* \brief configuration paramters for HDMI */
struct vps_videoencoderconfigparams {
	enum vps_videoencoderoutputmodeid  ouputmode;
	/**< output mode of hdmi */
	enum vps_videoencodersyncmode      syncmode;
	/**< Select either embedded or external sync */
} ;

#endif

