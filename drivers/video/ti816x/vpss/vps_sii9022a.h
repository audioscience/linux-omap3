/*
 *
 * SII9022A header file for TI 816X
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


#ifndef __DRIVERS_VIDEO_TI816X_VPSS_VPS_SII_9022A_H_
#define __DRIVERS_VIDEO_TI816X_VPSS_VPS_SII_9022A_H_

#include "vps_videoencoder.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
  * \brief Query hot plug detect(HDP) for HDMI cable connect or disconnect.
  *
  *
  * \param cmdArgs       [IN/OUT]  Vps_HdmiChipId*
  * \param cmdArgsStatus [OUT]  NULL
  *
  * \return FVID_SOK on success, else failure. Status= TRUE means cable
  * connected else not connected
  *
*/
#define IOCTL_VPS_SII9022A_GET_DETAILED_CHIP_ID      \
	(VPS_VID_ENC_IOCTL_BASE + 0x1000)


/**
  * \brief Get detailed Chip Id of HDMI with HDCP details.
  *
  *
  * \param cmdArgs       [IN/OUT]   Pointer to u32 i.e. u32* Status
  * \param cmdArgsStatus [OUT]  NULL
  *
  * \return FVID_SOK on success, else failure.
  *
*/
#define IOCTL_VPS_SII9022A_QUERY_HPD      \
	(VPS_VID_ENC_IOCTL_BASE + 0x1001)

/**
  * \brief enum defining SiI9022A modes
  */
enum vps_sii9022amodes {
	VPS_SII9022A_MODE_720P_60 = 0,
	/**< 720P@60 */
	VPS_SII9022A_MODE_1080P_30,
	/**< 1080P@30 */
	VPS_SII9022A_MODE_1080P_60,
	/**< 1080P@60 */
	VPS_SII9022A_MODE_1080I_60,
	/**< 1080I@60 */
	VPS_SII9022A_MAX_MODES
	/**< This should be last enum */
};


/* \brief Structure for getting HDMI chip identification Id. */
struct vps_hdmichipid {
	u32                  deviceId;
	/**< Device Id TPI */
	u32                  deviceProdRevId;
	/**< Device Production Revision Id TPI  */
	u32                  tpiRevId;
	/**< TPI Revision Id TPI */
	u32                  hdcpRevTpi;
	/**< HDCP revision TPI */
};

/**
 * \brief Mode Information structure.
 */
struct vps_sii9022amodeparams {
	enum vps_sii9022amodes modeid;
	/**< Id of the mode. This structure is used in
	 setting mode in SiI9022A using
	 IOCTL_VPS_VIDEO_ENCODER_SET_MODE IOCTL */
};

/**
 * \brief Structure for hot plug detection parameters.
 *
 * It is used to get
 * the hpd parameters using IOCTL_VPS_SII9022A_QUERY_HPD ioctl.
 */
struct vps_sii9022ahpdprms {
	u32 hpdEvtPending;
	/**< Hot Plug Connection Event Pending */
	u32 busError;
	/**< Receiver Sense Event Pending or CTRL Bus Error */
	u32 hpdStatus;
	/**< Hot Plug Pin Current Status */
} ;

#endif
