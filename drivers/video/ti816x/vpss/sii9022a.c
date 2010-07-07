/*
 *
 * SII9022A driver for TI 816X
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

#define VPSS_SUBMODULE_NAME   "SI9022"

#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/dma-mapping.h>
#include <linux/fvid2.h>
#include <linux/vps.h>
#include <linux/vps_displayctrl.h>

#include "core.h"
#include "vps_videoencoder.h"
#include "vps_sii9022a.h"


/** \brief HDMI SII9022a encoder driver ID used at the time of FVID2 create. */
#define FVID2_VPS_VID_ENC_SII9022A_DRV  (VPS_VID_ENC_DRV_BASE + 0x0000u)

struct vps_sii9022a_ctrl {
	u32           *handle;
	bool          isstarted;
	struct vps_videoencodercreateparams  *crprms;
	u32          crprms_phy;
	struct vps_videoencodercreatestatus  *crstatus;
	u32          crstatus_phy;
	struct vps_hdmichipid                *chipid;
	u32         chipid_phy;
	struct vps_sii9022ahpdprms           *hpdprms;
	u32         hpdrpsm_phy;
	struct vps_sii9022amodeparams        *modeprms;
	u32        modeprms_phy;
};


static struct vps_payload_info   *sii9022a_payload_info;
static struct vps_sii9022a_ctrl  *sii9022a_ctrl;



int sii9022a_setmode(u32 mode)
{
	int r;
	if ((sii9022a_ctrl == NULL) || (sii9022a_ctrl->handle == NULL))
		return -EINVAL;

	switch (mode) {
	case VPS_DC_MODE_720P_60:
		sii9022a_ctrl->modeprms->modeid = VPS_SII9022A_MODE_720P_60;
		break;
	case VPS_DC_MODE_1080P_60:
		sii9022a_ctrl->modeprms->modeid = VPS_SII9022A_MODE_1080P_60;
		break;
	case VPS_DC_MODE_1080P_30:
		sii9022a_ctrl->modeprms->modeid = VPS_SII9022A_MODE_1080P_30;
		break;
	case VPS_DC_MODE_1080I_60:
		sii9022a_ctrl->modeprms->modeid = VPS_SII9022A_MODE_1080I_60;
		break;
	default:
		sii9022a_ctrl->modeprms->modeid = VPS_SII9022A_MAX_MODES;
		break;
	}

	if (sii9022a_ctrl->modeprms->modeid == VPS_SII9022A_MAX_MODES) {
		VPSSERR("wrong mode\n");
		return -EINVAL;
	}

	r = vps_fvid2_control(sii9022a_ctrl->handle,
			      IOCTL_VPS_VIDEO_ENCODER_SET_MODE,
			      (void *)sii9022a_ctrl->modeprms_phy,
			      NULL);

	if (r) {
		VPSSERR("failed to set mode\n");
		sii9022a_ctrl->modeprms->modeid = VPS_SII9022A_MAX_MODES;
	} else
		VPSSERR("set mode to %d\n", sii9022a_ctrl->modeprms->modeid);

	return r;

}

int sii9022a_start(void)
{
	int r = 0;
	if ((sii9022a_ctrl == NULL) || (sii9022a_ctrl->handle == NULL))
		return -EINVAL;

	if (sii9022a_ctrl->modeprms->modeid == VPS_SII9022A_MAX_MODES)
		return -EINVAL;

	if (sii9022a_ctrl->isstarted == false)
		r = vps_fvid2_start(sii9022a_ctrl->handle, NULL);

	if (r)
		VPSSERR("failed to start\n");
	else
		sii9022a_ctrl->isstarted = true;
	return r;

}

int sii9022a_stop(void)
{
	int r = 0;
	if ((sii9022a_ctrl == NULL) || (sii9022a_ctrl->handle == NULL))
		return -EINVAL;

	if (sii9022a_ctrl->isstarted)
		r = vps_fvid2_stop(sii9022a_ctrl->handle, NULL);

	if (r)
		VPSSERR("failed to stop\n");
	else
		sii9022a_ctrl->isstarted = false;
	return r;

}

static int sii9022a_open(void)
{

	sii9022a_ctrl->crprms->devicei2cinstId = 1;
	sii9022a_ctrl->crprms->devicei2cAddr = 0x39;
	sii9022a_ctrl->crprms->inpclk = 0;
	sii9022a_ctrl->crprms->hdmihotplughpiointrline = 0;

	sii9022a_ctrl->handle = vps_fvid2_create(
				FVID2_VPS_VID_ENC_SII9022A_DRV,
				0u,
				(void *)sii9022a_ctrl->crprms_phy,
				(void *)sii9022a_ctrl->crstatus_phy,
				NULL);

	if (sii9022a_ctrl->handle == NULL) {
		VPSSERR("failed to create");
		return -EINVAL;
	}

	return 0;

}

static int sii9022a_close(void)
{
	int r = 0;

	if ((sii9022a_ctrl == NULL) || (sii9022a_ctrl->handle == NULL))
		return 0;

	/*stop if it is on */
	if (sii9022a_ctrl->isstarted) {
		r = vps_fvid2_stop(sii9022a_ctrl->handle,
				   NULL);
		if (r)
			VPSSERR("failed to stop");
		else
			sii9022a_ctrl->isstarted = false;

	}
	/*delete the handle*/
	if (r == 0) {
		r = vps_fvid2_delete(sii9022a_ctrl->handle,
				     NULL);
		if (r)
			VPSSERR("failed to delete");
		else
			sii9022a_ctrl->handle = NULL;
	}

	return r;

}
static int sii9022a_getid(void)
{
	int r;

	if ((sii9022a_ctrl == NULL) || (sii9022a_ctrl->handle == NULL))
		return -EINVAL;

	r = vps_fvid2_control(sii9022a_ctrl->handle,
			      IOCTL_VPS_SII9022A_GET_DETAILED_CHIP_ID,
			      (void *)sii9022a_ctrl->chipid_phy,
			      NULL);

	if (r)
		VPSSERR("failed to get id");
	else
		VPSSDBG("DevId %d, Prod RevID %d,"
			"TPI RevID %d and HDCP RevId %d\n",
			sii9022a_ctrl->chipid->deviceId,
			sii9022a_ctrl->chipid->deviceProdRevId,
			sii9022a_ctrl->chipid->tpiRevId,
			sii9022a_ctrl->chipid->hdcpRevTpi);

	return r;

}

static int sii9022a_get_hpd(void)
{
	int r;

	if ((sii9022a_ctrl == NULL) || (sii9022a_ctrl->handle == NULL))
		return -EINVAL;

	r = vps_fvid2_control(sii9022a_ctrl->handle,
			      IOCTL_VPS_SII9022A_QUERY_HPD,
			      (void *)sii9022a_ctrl->hpdrpsm_phy,
			      NULL);
	if (r)
		VPSSERR("failed to get HPD\n");
	else
		VPSSDBG("HPD: pending event %d, busErr: %d, satatus %d",
			sii9022a_ctrl->hpdprms->hpdEvtPending,
			sii9022a_ctrl->hpdprms->busError,
			sii9022a_ctrl->hpdprms->hpdStatus);

	return r;
}



static inline int get_payload_size(void)
{
	int size = 0;
	size += sizeof(struct vps_videoencodercreateparams);
	size += sizeof(struct vps_videoencodercreatestatus);
	size += sizeof(struct vps_hdmichipid);
	size += sizeof(struct vps_sii9022ahpdprms);
	size += sizeof(struct vps_sii9022amodeparams);

	return size;
}

static inline void assign_payload_addr(struct vps_sii9022a_ctrl *si_ctrl,
				 struct vps_payload_info *pinfo,
				 u32 *buf_offset)
{
	int offset = *buf_offset;
	/*create parameters*/
	si_ctrl->crprms = (struct vps_videoencodercreateparams *)
				((u32)pinfo->vaddr + offset);
	si_ctrl->crprms_phy = pinfo->paddr + offset;
	offset += sizeof(struct vps_videoencodercreateparams);
	/*create status*/
	si_ctrl->crstatus = (struct vps_videoencodercreatestatus *)
				((u32)pinfo->vaddr + offset);
	si_ctrl->crstatus_phy = pinfo->paddr + offset;
	offset += sizeof(struct vps_videoencodercreatestatus);

	/*hdmi chip id*/
	si_ctrl->chipid = (struct vps_hdmichipid *)
			    ((u32)pinfo->vaddr + offset);
	si_ctrl->chipid_phy = pinfo->paddr + offset;
	/*hot pluge detection */
	si_ctrl->hpdprms = (struct vps_sii9022ahpdprms *)
				((u32)pinfo->vaddr + offset);
	si_ctrl->hpdrpsm_phy = pinfo->paddr + offset;
	offset += sizeof(struct vps_sii9022ahpdprms);
	/*mode parameters */
	si_ctrl->modeprms = (struct vps_sii9022amodeparams *)
				((u32)pinfo->vaddr + offset);
	si_ctrl->modeprms_phy = pinfo->paddr + offset;
	offset += sizeof(struct vps_sii9022amodeparams);

	*buf_offset = offset;

}

int __init sii9022a_init(struct platform_device *pdev)
{
	int size;
	int r = 0;
	u32 offset = 0;

	struct vps_payload_info  *pinfo;

	VPSSDBG("enter sii9022a init\n");
	/*allocate payload info*/
	sii9022a_payload_info = kzalloc(sizeof(struct vps_payload_info),
				       GFP_KERNEL);

	if (!sii9022a_payload_info) {
		VPSSERR("failed to allocate payload structure\n");
		return -ENOMEM;
	}
	pinfo = sii9022a_payload_info;

	/*allocate si9022a control*/
	sii9022a_ctrl = kzalloc(sizeof(struct vps_sii9022a_ctrl), GFP_KERNEL);
	if (sii9022a_ctrl == NULL) {
		VPSSERR("failed to allocate control\n");
		r = -ENOMEM;
		goto exit;


	}
	/*allocate shared payload buffer*/
	size = get_payload_size();
	pinfo->vaddr = vps_sbuf_alloc(size, &pinfo->paddr);
	if (pinfo->vaddr == NULL) {
		VPSSERR("failed to allocate payload\n");
		 r = -EINVAL;
		goto exit;
	}

	pinfo->size = PAGE_ALIGN(size);
	assign_payload_addr(sii9022a_ctrl, pinfo, &offset);
	memset(pinfo->vaddr, 0, pinfo->size);

	/*set the mode to invalid*/
	sii9022a_ctrl->modeprms->modeid = VPS_SII9022A_MAX_MODES;

	r = sii9022a_open();
	if (r)
		goto exit;

	r = sii9022a_getid();
	if (r)
		goto exit;

	r = sii9022a_get_hpd();
	if (r)
		goto exit;

	return 0;
exit:
	sii9022a_deinit(pdev);
	return r;

}


int __exit sii9022a_deinit(struct platform_device *pdev)
{
	int r = 0;

	VPSSDBG("sii9022a deinit\n");
	r = sii9022a_close();

	if (sii9022a_payload_info) {
		/*free payload buffer*/
		if (sii9022a_payload_info->vaddr) {
			vps_sbuf_free(sii9022a_payload_info->paddr,
				      sii9022a_payload_info->vaddr,
				      sii9022a_payload_info->size);
		}

		/*free payload info*/
		kfree(sii9022a_payload_info);
		sii9022a_payload_info = NULL;
	}
	/*free si9022a control*/
	kfree(sii9022a_ctrl);
	if (sii9022a_ctrl)
		sii9022a_ctrl = NULL;

	return r;
}
