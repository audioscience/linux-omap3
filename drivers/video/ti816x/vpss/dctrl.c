/*
 * linux/drivers/video/ti816x/vpss/dctrl.c
 *
 * VPSS display controller driver for TI 816X
 *
 * Copyright (C) 2009 TI
 * Author: Yihe Hu <yihehu@ti.com>
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
#define VPSS_SUBMODULE_NAME   "DCTRL"

#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include <linux/vps_proxyserver.h>
#include <linux/fvid2.h>
#include <linux/vps.h>
#include <linux/vps_displayctrl.h>
#include <linux/dc.h>

#include "core.h"

static struct vps_dispctrl *disp_ctrl;
static void *dc_handle;
static struct vps_payload_info  *dc_payload_info;

/*store the current VENC setting*/
static struct vps_dcvencinfo venc_info = {
	{
		{VPS_DC_VENC_HDMI, 0, VPS_DC_MODE_1080P_30, 1920, \
		1080, FVID2_SF_PROGRESSIVE, NULL, 0},

		{VPS_DC_VENC_HDCOMP, 0, VPS_DC_MODE_1080P_30,	\
		1920, 1080, FVID2_SF_INTERLACED, NULL, 0},

		{VPS_DC_VENC_DVO2, 0, VPS_DC_MODE_1080P_30,    \
		1920, 1080, FVID2_SF_PROGRESSIVE, NULL, 0},

		{VPS_DC_VENC_SD, 0, VPS_DC_MODE_NTSC,	  \
		720, 480, FVID2_SF_INTERLACED, NULL, 0},
	},
	0,
	4,
};

/*store the current mode info*/
static struct venc_modeinfo modeinfo[VPS_DC_MAX_MODE] = {
	{"ntsc", 720, 480, 0, VPS_DC_MODE_NTSC},
	{"pal", 720, 576, 0, VPS_DC_MODE_PAL},
	{"1080p-60", 1920, 1080, 1, VPS_DC_MODE_1080P_60},
	{"720p-60", 1280, 720, 1, VPS_DC_MODE_720P_60},
	{"1080i-60", 1920, 1080, 0, VPS_DC_MODE_1080I_60},
	{"1080p-30", 1920, 1080, 1, VPS_DC_MODE_1080P_30},
};

static struct venc_name_id v_nameid[VPS_DC_MAX_VENC] = {
	{"hdmi", VPS_DC_VENC_HDMI, VPS_DC_HDMI_BLEND, HDMI},
	{"hdcomp", VPS_DC_VENC_HDCOMP, VPS_DC_HDCOMP_BLEND, HDCOMP},
	{"dvo2", VPS_DC_VENC_DVO2, VPS_DC_DVO2_BLEND, DVO2},
	{"sdvenc", VPS_DC_VENC_SD, VPS_DC_SDVENC_BLEND, SDVENC}
};

static struct dcnode_info dcnode[] = {
	{"main", VPS_DC_MAIN_INPUT_PATH},			/*0*/
	{"vcomp_mux", VPS_DC_VCOMP_MUX},			/*1*/
	{"hdcomp_mux", VPS_DC_HDCOMP_MUX},			/*2*/
	{"sdvenc_mux", VPS_DC_SDVENC_MUX },			/*3*/
	{"aux", VPS_DC_AUX_INPUT_PATH},				/*4*/
	{"bp0", VPS_DC_BP0_INPUT_PATH},				/*5*/
	{"bp1", VPS_DC_BP1_INPUT_PATH},				/*6*/
	{"dummy", VPS_DC_MAX_NODE_NUM},				/*7*/
	{"dummy", VPS_DC_MAX_NODE_NUM},				/*8*/
	{"dummy", VPS_DC_MAX_NODE_NUM},				/*9*/
	{"sd", VPS_DC_SEC1_INPUT_PATH},				/*10*/
	{"dummy", VPS_DC_MAX_NODE_NUM},				/*11*/
	{"dummy", VPS_DC_MAX_NODE_NUM},				/*12*/
	{"dummy", VPS_DC_MAX_NODE_NUM},				/*13*/
	{"vcomp", VPS_DC_VCOMP},				/*14*/
	{"cigcons", VPS_DC_CIG_CONSTRAINED_OUTPUT},		/*15*/
	{"cigin", VPS_DC_CIG_PIP_INPUT},			/*16*/
	{"cigncons", VPS_DC_CIG_NON_CONSTRAINED_OUTPUT},	/*17*/
	{"cigout", VPS_DC_CIG_PIP_OUTPUT},			/*18*/
	{"grpx0", VPS_DC_GRPX0_INPUT_PATH},			/*19*/
	{"grpx1", VPS_DC_GRPX1_INPUT_PATH},			/*20*/
	{"grpx2", VPS_DC_GRPX2_INPUT_PATH},			/*21*/
	{"hdmi",  VPS_DC_HDMI_BLEND},				/*22*/
	{"hdcomp", VPS_DC_HDCOMP_BLEND},			/*23*/
	{"dvo2", VPS_DC_DVO2_BLEND},				/*24*/
	{"sdvenc", VPS_DC_SDVENC_BLEND},			/*25*/
};

/*S***************************private funtions*******************/

/*get the venc information from M3*/
static int  dc_get_vencinfo(struct vps_dcvencinfo *vinfo)
{
	int r = 0;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	memcpy(disp_ctrl->vinfo, vinfo, sizeof(struct vps_dcvencinfo));
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_GET_VENC_MODE,
			      (void *)disp_ctrl->vinfo_phy,
			      NULL);

	if (r)
		VPSSERR("failed to get venc info.\n");
	else
		memcpy(vinfo,
		       disp_ctrl->vinfo,
		       sizeof(struct vps_dcvencinfo));

	return r;

}

/*get the current format based on the mode id*/
static int get_format_from_mid(int mid, u32 *width, u32 *height, u8 *scformat)
{
	int i;
	for (i = 0; i < VPS_DC_MAX_MODE; i++) {
		/*FIX me add customer mode in the future*/
		if (mid == modeinfo[i].mid) {
			*width = modeinfo[i].width;
			*height = modeinfo[i].height;
			*scformat = modeinfo[i].scformat;
			return 0;
		}
	}

	return -EINVAL;
}
/*get the format based on the venc id*/
static int get_format_from_vid(int vid, u32 *width, u32 *height, u8 *scformat)
{
	int r = 0;
	struct vps_dcvencinfo vinfo;

	vinfo.numvencs = 1;
	vinfo.modeinfo[0].vencid = vid;

	r = dc_get_vencinfo(&vinfo);
	if (r)
		return -EINVAL;

	if (vinfo.modeinfo[0].isvencrunning == 0) {
		VPSSERR("Please enable VENC first\n");
		return -EINVAL;
	}

	if (vinfo.modeinfo[0].iscustommode) {
		*width = vinfo.modeinfo[0].framewidth;
		*height = vinfo.modeinfo[0].frameheight;
		*scformat = vinfo.modeinfo[0].scanformat;
	} else
		r = get_format_from_mid(vinfo.modeinfo[0].modeid,
					width,
					height,
					scformat);

	return r;
}

/*get the format based on the blender id*/
static int get_format_from_bid(int bid, u32 *width, u32 *height, u8 *scformat)
{
	int i;
	int r = -EINVAL;
	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		if (bid == v_nameid[i].blendid) {
			r = get_format_from_vid(v_nameid[i].vid,
						width,
						height,
						scformat);
			break;
		}
	}

	return r;
}

/*get the index of the desired venc id in the database*/
static int get_idx_from_vid(int vid, int *idx)
{
	int i;
	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		if (vid == v_nameid[i].vid) {
			*idx = v_nameid[i].idx;
			return 0;
		}
	}

	return -EINVAL;
}

/*get the venc id based on the name*/
static int dc_get_vencid(char *vname, int *vid)
{

	int i;
	struct venc_name_id *vnid;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	VPSSDBG("enter get venc id\n");
	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		vnid = &v_nameid[i];
		if (!strcmp(vname, vnid->name)) {
			*vid = vnid->vid;
			return 0;
		}
	}
	return -1;
}

/*get the mode id based on the mode name*/
static int dc_get_modeid(char *mname, int *mid)
{
	int i;
	struct venc_modeinfo *vinfo;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	VPSSDBG("enter get mode id\n");
	for (i = 0; i < VPS_DC_MAX_MODE; i++) {
		vinfo = &modeinfo[i];
		if (!strcmp(mname, vinfo->name)) {
			*mid = vinfo->mid;
			return 0;
		}
	}
	return -1;
}

/*get the node id based on the name*/
static int dc_get_nodeid(char *name, int *nid)
{
	int i;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	if (strcmp(name, "dummy") == 0)
		return -EINVAL;

	for (i = 0; i < VPS_DC_MAX_NODE_NUM; i++) {
		struct dcnode_info *ninfo = &dcnode[i];
		if (strcmp(name, ninfo->name) == 0) {
			*nid =  ninfo->id;
			return 0;
		}
	}
	return -EINVAL;
}

/*disable the desired vencs*/
static int dc_venc_disable(int vid)
{
	int i = 0;
	int r = 0;
	struct vps_dcvencinfo vinfo;
	int venc_ids = vid;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	if (vid == 0)
		return 0;

	if (vid & (~VPS_DC_VENC_MASK)) {
		VPSSERR("wrong venc id.\n");
		return -EINVAL;
	}


	VPSSDBG("enter venc disable\n");

	vinfo.numvencs = 0;
	/*get the id of each venc to be disabled*/
	while (venc_ids >> i) {
		if ((venc_ids >> i++) & 1)
			vinfo.modeinfo[vinfo.numvencs++].vencid =
							1 << (i - 1);
	}

	r = dc_get_vencinfo(&vinfo);

	if (r) {
		VPSSERR("faild to get venc info.\n");
		return r;
	}

	venc_ids = vid;
	for (i = 0; i < vinfo.numvencs; i++) {
		if (vinfo.modeinfo[i].isvencrunning == 0) {
			VPSSERR("venc %d already stop\n",
				vinfo.modeinfo[i].vencid);
			venc_ids &= ~vinfo.modeinfo[i].vencid;
		}
	}

#ifdef CONFIG_TI816X_VPSS_SII9022A
	/*disable the off-chip HDMI first if DVO2 venc is assigned*/
	if (venc_ids & VPS_DC_VENC_DVO2) {
		r = sii9022a_stop();
		if (r)
			VPSSERR("failed to stop sii9022a\n");
	}
#endif
	if (venc_ids && !r) {
		*disp_ctrl->dis_vencs = venc_ids;
		r = vps_fvid2_control(disp_ctrl->fvid2_handle,
				      IOCTL_VPS_DCTRL_DISABLE_VENC,
				      (void *)disp_ctrl->dis_vencsphy,
				      NULL);

		if (r == 0) {
			disp_ctrl->enabled_venc_ids &= ~venc_ids;
			if (disp_ctrl->tiedvenc) {
				disp_ctrl->tiedvenc &= ~venc_ids;
				venc_ids = 0;
				i = 0;
				/*calculate how vencs left in tied list*/
				while (disp_ctrl->tiedvenc >> i) {
					if ((disp_ctrl->tiedvenc >> i++) & 1)
						venc_ids++;

				}
				/*if one venc left,set tiedvenc to zero*/
				if (venc_ids == 1)
					disp_ctrl->tiedvenc = 0;
			}
		} else
			VPSSERR("failed to disable the venc.\n");

	}

	return r;
}

/*set the mode for desired vencs*/
static int dc_set_vencmode(struct vps_dcvencinfo *vinfo)
{
	int i, r = 0;
	int vencs = 0;
	struct vps_dcvencinfo vi;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;


	/*get the current setting based on the app inputs*/
	for (i = 0; i < vinfo->numvencs; i++)
		vi.modeinfo[i].vencid = vinfo->modeinfo[i].vencid;

	vi.numvencs = vinfo->numvencs;

	r = dc_get_vencinfo(&vi);

	if (r) {
		VPSSERR("failed to get venc info.\n");
		goto exit;
	}

	/*make sure current venc status is matching */
	disp_ctrl->vinfo->numvencs = 0;
	disp_ctrl->vinfo->tiedvencs = 0;
	for (i = 0; i < vinfo->numvencs; i++) {
		if (vi.modeinfo[i].isvencrunning) {
			if (vi.modeinfo[i].modeid !=
			    vinfo->modeinfo[i].modeid) {
				r = -EINVAL;
				VPSSERR("venc %d already running with \
						different mode\n",
						vi.modeinfo[i].vencid);
				goto exit;
			} else
				VPSSDBG("venc %d already running\n",
					vi.modeinfo[i].vencid);

		} else {
			memcpy(&disp_ctrl->vinfo->modeinfo \
					[disp_ctrl->vinfo->numvencs++],
			       &vinfo->modeinfo[i],
			       sizeof(struct vps_dcmodeinfo));
			vencs |= vinfo->modeinfo[i].vencid;
		}
	}
	if (vinfo->tiedvencs) {
		if ((vencs & vinfo->tiedvencs) != vinfo->tiedvencs) {
			r = -EINVAL;
			VPSSERR("can not tied venc\n");
			goto exit;
		} else
			disp_ctrl->vinfo->tiedvencs = vinfo->tiedvencs;
	}

	if (disp_ctrl->vinfo->numvencs) {

		/*set the VENC Mode*/
		r = vps_fvid2_control(disp_ctrl->fvid2_handle,
				IOCTL_VPS_DCTRL_SET_VENC_MODE,
				(void *)disp_ctrl->vinfo_phy,
				NULL);
		if (r) {
			VPSSERR("failed to set venc mdoe.\n");
			goto exit;
		}
		disp_ctrl->enabled_venc_ids |= vencs;
	}
#ifdef CONFIG_TI816X_VPSS_SII9022A
	/*config the sii9022a*/
	if (!r) {
		for (i = 0; i < vinfo->numvencs; i++) {
			if (vinfo->modeinfo[i].vencid == VPS_DC_VENC_DVO2) {
				struct vps_dcmodeinfo *minfo =
							&vinfo->modeinfo[i];
				r = sii9022a_setmode(minfo->modeid);
				if (!r) {
					r = sii9022a_start();
					if (r)
						VPSSERR(
						   "start sii9022a failed\n");
				} else
					VPSSERR("set sii9022a mode failed\n");
			}

		}

	}
#endif

exit:
	return r;

}

/*E******************************** private functions *********************/

/*S*******************************  public functions  *********************/

/*get the id(venc,blender,mode) based on the name*/
int vps_dc_get_id(char *name, int *id, enum dc_idtype type)
{

	int r = -EINVAL;
	switch (type) {
	case DC_BLEND_ID:
	case DC_NODE_ID:
		r = dc_get_nodeid(name, id);
		break;
	case DC_VENC_ID:
		r = dc_get_vencid(name, id);
		break;
	case DC_MODE_ID:
		r = dc_get_modeid(name, id);
		break;
	}

	return r;
}

/*get the tied venc information*/
int vps_dc_get_tiedvenc(u8 *tiedvenc)
{
	*tiedvenc = disp_ctrl->tiedvenc;
	return 0;
}
/*set the streaming on the blender, not used*/
void vps_dc_set_actnodes(u8 setflag, u8 bid)
{
	struct dc_blenderinfo *binfo = &disp_ctrl->blenders[bid];

	if (setflag)
		binfo->actnodes++;
	else
		if (binfo->actnodes != 0)
			binfo->actnodes--;

}
/*get the venc infor for the desired vencs*/
int vps_dc_get_vencinfo(struct vps_dcvencinfo *vinfo)
{
	int r;
	dc_lock(disp_ctrl);
	r = dc_get_vencinfo(vinfo);
	dc_unlock(disp_ctrl);

	return r;
}

/*get the node name based on the id*/
int vps_dc_get_node_name(int id, char *name)
{
	int i;
	for (i = 0; i < VPS_DC_MAX_NODE_NUM; i++) {
		struct dcnode_info *ninfo = &dcnode[i];
		if (id == ninfo->id) {
			strcpy(name, ninfo->name);
			return 0;
		}
	}
	return -EINVAL;

}

/*get the clk source*/
int vps_dc_get_clksrc(enum vps_dcdvo2clksrc *dvo2,
			enum vps_dchdcompclksrc *hdcomp)
{
	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	VPSSDBG("enter get clksrc\n");
	dc_lock(disp_ctrl);
	*dvo2 = disp_ctrl->dvo2clksrc;
	*hdcomp = disp_ctrl->hdcompclksrc;
	dc_unlock(disp_ctrl);
	return 0;
}

/*set dc config not used now*/
int vps_dc_set_config(struct vps_dcconfig *usercfg, int setflag)
{
	int r = 0;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	if (usercfg->vencinfo.numvencs > VPS_DC_MAX_VENC) {
		VPSSERR("num vens (%d) over max\n",
			usercfg->vencinfo.numvencs);
		return -EINVAL;
	}
	if (usercfg->vencinfo.tiedvencs & (~disp_ctrl->tiedvenc)) {
		VPSSERR("tied venc not match.\n");
		return -EINVAL;
	}
	VPSSDBG("enter set config\n");
	dc_lock(disp_ctrl);

	memcpy(disp_ctrl->dccfg, usercfg, sizeof(struct vps_dcconfig));


	if (setflag) {
		r = vps_fvid2_control(disp_ctrl->fvid2_handle,
				      IOCTL_VPS_DCTRL_SET_CONFIG,
				      (void *)disp_ctrl->dccfg_phy,
				      NULL);
		if (r)
			VPSSDBG("faield to set the DC config.\n");
	} else {
		r = vps_fvid2_control(disp_ctrl->fvid2_handle,
				      IOCTL_VPS_DCTRL_CLEAR_CONFIG,
				      (void *)disp_ctrl->dccfg_phy,
				      NULL);

		if (r)
			VPSSDBG("faield to clear the DC config.\n");

	}

	dc_unlock(disp_ctrl);

	return r;
}

/*get current venc output format*/
int vps_dc_get_outpfmt(int id, u32 *width,
		       u32 *height,
		       u8 *scformat,
		       enum dc_idtype type)
{
	int r;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	VPSSDBG("enter get output format\n");

	dc_lock(disp_ctrl);
	if (type == DC_VENC_ID)
		r = get_format_from_vid(id, width, height, scformat);
	 else if (type == DC_BLEND_ID)
		r = get_format_from_bid(id, width, height, scformat);
	 else if (type == DC_MODE_ID)
		r = get_format_from_mid(id, width, height, scformat);
	 else
		r = -EINVAL;

	dc_unlock(disp_ctrl);
	return r;
}

/* set/clear the node path/edge */
int vps_dc_set_node(u8 nodeid, u8 inputid, u8 enable)
{

	int r = 0;
	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	VPSSDBG("enter set node\n");
	dc_lock(disp_ctrl);

	disp_ctrl->nodeinfo->nodeid = nodeid;
	disp_ctrl->nodeinfo->inputid = inputid;

	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_GET_NODE_INPUT_STATUS,
			      (void *)disp_ctrl->ninfo_phy,
			      NULL);

	if (r) {
		VPSSERR("failed to get node input status\n");
		goto exit;
	}
	if (disp_ctrl->nodeinfo->isenable == enable) {
		if (enable)
			VPSSDBG("node already connected\n");
		else
			VPSSDBG("node already disconnected\n");

		goto exit;
	}
	/*call ioctl to set/clear the node */
	disp_ctrl->nodeinfo->isenable = enable;
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_NODE_INPUT,
			      (void *)disp_ctrl->ninfo_phy,
			      NULL);
	if (r)
		VPSSERR("failed to enable node.\n");

exit:
	dc_unlock(disp_ctrl);
	return r;
}
/*E********************************* public functions *****************/

/*sysfs function for blender starting from here*/
static ssize_t blender_mode_show(struct dc_blenderinfo *binfo, char *buf)
{
	int i;
	u32 idx = binfo->idx;
	int l = 0;
	for (i = 0; i < VPS_DC_MAX_MODE; i++) {
		if (modeinfo[i].mid == venc_info.modeinfo[idx].modeid) {
			l = snprintf(buf, PAGE_SIZE, "%s\n",
				modeinfo[i].name);
			break;
		}
	}
	return l;
}

static ssize_t blender_mode_store(struct dc_blenderinfo *binfo,
		const char *buf, size_t size)
{
	int r;
	u32 idx = binfo->idx;
	u32 mid;
	char *input = (char *)buf;
	struct vps_dcvencinfo vinfo;

	dc_lock(binfo->dcctrl);

	/*venc should be stop before changes*/
	vinfo.numvencs = 1;
	vinfo.modeinfo[0].vencid = venc_info.modeinfo[idx].vencid;
	r = dc_get_vencinfo(&vinfo);
	if (r) {
		r = -EINVAL;
		goto exit;
	}
	if (vinfo.modeinfo[0].isvencrunning) {
		VPSSERR("stop venc before changing mode\n");
		r = -EINVAL;
		goto exit;
	}

	input  = strsep(&input, "\n");
	r = dc_get_modeid(input, &mid);
	if (r) {
		VPSSERR("failed to get the mode %s.\n", input);
		r = -EINVAL;
		goto exit;
	}

	venc_info.modeinfo[idx].modeid = mid;
	r = size;
exit:
	dc_unlock(binfo->dcctrl);
	return r;
}

static ssize_t blender_timings_show(struct dc_blenderinfo *binfo, char *buf)
{
	int r;
	struct vps_dctiminginfo t;
	struct vps_dcvencinfo vinfo;

	dc_lock(binfo->dcctrl);

	memset(&t, 0, sizeof(struct vps_dctiminginfo));
	dc_get_vencid(binfo->name, &vinfo.modeinfo[0].vencid);

	vinfo.numvencs = 1;

	r = dc_get_vencinfo(&vinfo);

	if (r) {
		VPSSERR(" Failed to get venc infor\n");
		r = -EINVAL;
		goto exit;

	}
	if (vinfo.modeinfo[0].isvencrunning == 0) {
		VPSSERR("please enable venc first\n");
		r = -EINVAL;
		goto exit;
	}

	t.mode = vinfo.modeinfo[0].modeid;
	if (vinfo.modeinfo[0].iscustommode) {
		t.width = vinfo.modeinfo[0].framewidth;
		t.height = vinfo.modeinfo[0].frameheight;
		t.scanformat = vinfo.modeinfo[0].scanformat;
	} else {
		get_format_from_mid(t.mode,
				    &t.width,
				    &t.height,
				    (u8 *)&t.scanformat);
	}


	r = snprintf(buf,
			PAGE_SIZE,
			"%u,%u/%u/%u/%u,%u/%u/%u/%u/%u/%u/%u,%u/%u\n",
			t.pixelclock,
			t.width, t.hfrontporch, t.hbackporch, t.hsynclen,
			t.height, t.vfrontporch[0], t.vfrontporch[1],
			t.vbackporch[0], t.vbackporch[1],
			t.vsynclen[0], t.vsynclen[1],
			t.scanformat, t.mode);

exit:
	dc_unlock(binfo->dcctrl);
	return r;

}

static ssize_t blender_timings_store(struct dc_blenderinfo *binfo,
		const char *buf, size_t size)
{
	return size;
}

static ssize_t blender_enabled_show(struct dc_blenderinfo *binfo, char *buf)
{
	int r;
	struct vps_dcvencinfo vinfo;

	dc_lock(binfo->dcctrl);
	dc_get_vencid(binfo->name, &vinfo.modeinfo[0].vencid);

	vinfo.numvencs = 1;

	r = dc_get_vencinfo(&vinfo);

	if (r) {
		VPSSERR(" Failed to get venc infor\n");
		r = -EINVAL;
		goto exit;
	}

	r = snprintf(buf, PAGE_SIZE, "%d\n", vinfo.modeinfo[0].isvencrunning);

exit:
	dc_unlock(binfo->dcctrl);
	return r;
}

static ssize_t blender_enabled_store(struct dc_blenderinfo *binfo,
				     const char *buf,
				     size_t size)
{
	int enabled;
	int vid;
	int r = 0;
	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	enabled = simple_strtoul(buf, NULL, 10);

	dc_lock(disp_ctrl);
	/*get vid id*/
	dc_get_vencid(binfo->name, &vid);

	if (enabled == 0) {
		r = dc_venc_disable(vid);
		if (r) {
			VPSSERR("failed to disable %s venc\n",
				binfo->name);
			r = -EINVAL;
			goto exit;
		}
	} else {
		int idx;
		struct vps_dcvencinfo vinfo;
		get_idx_from_vid(vid, &idx);
		memcpy(&vinfo.modeinfo[0],
		   &venc_info.modeinfo[idx],
		   sizeof(struct vps_dcvencinfo));
		vinfo.numvencs = 1;
		vinfo.tiedvencs = 0;
		r = dc_set_vencmode(&vinfo);
		if (r) {
			VPSSERR("failed to enable venc %s\n",
				binfo->name);
			r = -EINVAL;
			goto exit;
		}
	}

	r = size;
exit:
	dc_unlock(disp_ctrl);
	return r;
}

static ssize_t blender_clksrc_show(struct dc_blenderinfo *binfo, char *buf)
{

	if (binfo->idx == HDCOMP) {
		if (binfo->dcctrl->hdcompclksrc == VPS_DC_HDCOMPCLKSRC_HDMI)
			return snprintf(buf, PAGE_SIZE, "hdmi\n");
		else
			return snprintf(buf, PAGE_SIZE, "hdcomp\n");
	} else if (binfo->idx == DVO2) {
		if (binfo->dcctrl->dvo2clksrc == VPS_DC_DVO2CLKSRC_HDMI)
			return snprintf(buf, PAGE_SIZE, "hdmi\n");
		else
			return snprintf(buf, PAGE_SIZE, "hdcomp\n");
	} else
		return 0;

}

static ssize_t blender_clksrc_store(struct dc_blenderinfo *binfo,
				     const char *buf,
				     size_t size)
{
	int r = 0;
	char *input = (char *)buf;
	struct vps_dcvencinfo vinfo;

	if ((binfo->idx == HDMI) || (binfo->idx == SDVENC))
		return size;
	input  = strsep(&input, "\n");

	dc_lock(binfo->dcctrl);

	vinfo.numvencs = 1;
	vinfo.modeinfo[0].vencid = venc_info.modeinfo[binfo->idx].vencid;
	r = dc_get_vencinfo(&vinfo);
	if (r)
		goto exit;

	if (vinfo.modeinfo[0].isvencrunning) {
		VPSSERR("stop venc before changing clk");
		r = -EINVAL;
		goto exit;
	}

	if (strcmp(input, "hdmi") == 0) {
		if (binfo->idx == HDCOMP)
			binfo->dcctrl->hdcompclksrc =
				VPS_DC_HDCOMPCLKSRC_HDMI;
		else
			binfo->dcctrl->dvo2clksrc =
				VPS_DC_DVO2CLKSRC_HDMI;

	} else if (strcmp(input, "hdcomp") == 0) {
		if (binfo->idx == HDCOMP)
			binfo->dcctrl->hdcompclksrc =
				VPS_DC_HDCOMPCLKSRC_HDCOMP;
		else
			binfo->dcctrl->dvo2clksrc =
				VPS_DC_DVO2CLKSRC_HDCOMP;

	} else {
		VPSSERR("clock source(%s) not supported.\n", input);
		r = -EINVAL;
		goto exit;
	}

	r = size;
exit:
	dc_unlock(binfo->dcctrl);
	return r;
}
struct blender_attribute {
	struct attribute attr;
	ssize_t (*show)(struct dc_blenderinfo *, char *);
	ssize_t (*store)(struct dc_blenderinfo *, const char *, size_t);
};


#define BLENDER_ATTR(_name, _mode, _show, _store) \
	struct blender_attribute blender_attr_##_name = \
	__ATTR(_name, _mode, _show, _store)

static BLENDER_ATTR(mode, S_IRUGO | S_IWUSR,
				blender_mode_show, blender_mode_store);
static BLENDER_ATTR(timing, S_IRUGO | S_IWUSR,
				blender_timings_show, blender_timings_store);
static BLENDER_ATTR(enabled, S_IRUGO | S_IWUSR,
				blender_enabled_show, blender_enabled_store);
static BLENDER_ATTR(clksrc, S_IRUGO | S_IWUSR,
				blender_clksrc_show, blender_clksrc_store);

static struct attribute *blender_sysfs_attrs[] = {
	&blender_attr_mode.attr,
	&blender_attr_timing.attr,
	&blender_attr_enabled.attr,
	&blender_attr_clksrc.attr,
	NULL
};

static ssize_t blender_attr_show(struct kobject *kobj,
				  struct attribute *attr,
				  char *buf)
{
	struct dc_blenderinfo *binfo = NULL;
	struct blender_attribute *blend_attr = NULL;

	binfo = container_of(kobj, struct dc_blenderinfo, kobj);

	blend_attr = container_of(attr, struct blender_attribute, attr);
	if (!blend_attr->show)
		return -ENOENT;

	return blend_attr->show(binfo, buf);
}

static ssize_t blender_attr_store(struct kobject *kobj,
				   struct attribute *attr,
				   const char *buf,
				   size_t size)
{
	struct dc_blenderinfo *blend;
	struct blender_attribute *blend_attr;

	blend = container_of(kobj, struct dc_blenderinfo, kobj);
	blend_attr = container_of(attr, struct blender_attribute, attr);

	if (!blend_attr->store)
		return -ENOENT;

	return blend_attr->store(blend, buf, size);
}

static const struct sysfs_ops blender_sysfs_ops = {
	.show = blender_attr_show,
	.store = blender_attr_store,
};

static struct kobj_type blender_ktype = {
	.sysfs_ops = &blender_sysfs_ops,
	.default_attrs = blender_sysfs_attrs,
};



/*sysfs for the display controller*/
static ssize_t dctrl_tiedvencs_show(struct vps_dispctrl *dctrl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", disp_ctrl->tiedvenc);
}

static ssize_t dctrl_tiedvencs_store(struct vps_dispctrl *dctrl,
				     const char *buf,
				     size_t size)
{
	int r = 0;
	int vencs = 0;
	int i = 0;
	struct vps_dcvencinfo vinfo;
	dc_lock(disp_ctrl);
	vencs = simple_strtoul(buf, NULL, 10);
	if (vencs & ~VPS_DC_VENC_MASK) {
		r = -EINVAL;
		VPSSERR("vencs %d over limit\n", vencs);
		goto exit;
	}

	if ((vencs == 0) || (disp_ctrl->tiedvenc == vencs)) {
		r = size;
		goto exit;
	}

	vinfo.numvencs = 0;
	vinfo.tiedvencs = vencs;

	/*assemble the structure based on the venc id*/
	while (vencs >> i) {
		/*get id of each venc to be tied*/
		if ((vencs >> i++) & 1) {
			int idx;
			int vid = 1 << (i - 1);
			get_idx_from_vid(vid, &idx);
			memcpy(&vinfo.modeinfo[vinfo.numvencs++],
			       &venc_info.modeinfo[idx],
			       sizeof(struct vps_dcmodeinfo));

		}
	}
	if (vinfo.numvencs < 2) {
		VPSSERR("at least 2 vencs to tied.\n");
		r = -EINVAL;
		goto exit;
	}

	/*set the tied venc mode*/
	r = dc_set_vencmode(&vinfo);
	if (r) {
		VPSSERR("failed to set tied venc\n");
		r = -EINVAL;
		goto exit;
	}
	disp_ctrl->tiedvenc = vinfo.tiedvencs;
	r = size;
exit:
	dc_unlock(disp_ctrl);
	return r;
}

struct dctrl_attribute {
	struct attribute attr;
	ssize_t (*show)(struct vps_dispctrl *, char *);
	ssize_t (*store)(struct vps_dispctrl *, const char *, size_t);
};

#define DCTRL_ATTR(_name, _mode, _show, _store) \
	struct dctrl_attribute dctrl_attr_##_name = \
	__ATTR(_name, _mode, _show, _store)

static DCTRL_ATTR(tiedvencs, S_IRUGO | S_IWUSR,
		dctrl_tiedvencs_show, dctrl_tiedvencs_store);

static struct attribute *dctrl_sysfs_attrs[] = {
	&dctrl_attr_tiedvencs.attr,
	NULL
};

static ssize_t dctrl_attr_show(struct kobject *kobj,
				struct attribute *attr,
				char *buf)
{
	struct vps_dispctrl *dctrl = NULL;
	struct dctrl_attribute *dctrl_attr = NULL;

	dctrl = container_of(kobj, struct vps_dispctrl, kobj);

	dctrl_attr = container_of(attr, struct dctrl_attribute, attr);
	if (!dctrl_attr->show)
		return -ENOENT;

	return dctrl_attr->show(dctrl, buf);
}

static ssize_t dctrl_attr_store(struct kobject *kobj,
				struct attribute *attr,
				const char *buf,
				size_t size)
{
	struct vps_dispctrl *dctrl;
	struct dctrl_attribute *dctrl_attr;

	dctrl = container_of(kobj, struct vps_dispctrl, kobj);
	dctrl_attr = container_of(attr, struct dctrl_attribute, attr);

	if (!dctrl_attr->store)
		return -ENOENT;

	return dctrl_attr->store(dctrl, buf, size);
}


static const struct sysfs_ops dctrl_sysfs_ops = {
	.show = dctrl_attr_show,
	.store = dctrl_attr_store,
};

static struct kobj_type dctrl_ktype = {
	.sysfs_ops = &dctrl_sysfs_ops,
	.default_attrs = dctrl_sysfs_attrs,
};

/*end of sysfs function for display controller*/


static int parse_def_modes(char *mode)

{
	char *str, *options, *this_opt;
	int r = 0;
	struct vps_dcvencinfo *vinfo = &venc_info;
	if (mode == NULL)
		return 0;

	str = kmalloc(strlen(mode) + 1, GFP_KERNEL);
	strcpy(str, mode);
	options = str;
	VPSSDBG("mode %s\n", mode);

	while (!r && (this_opt = strsep(&options, ",")) != NULL) {
		char *p, *display_str, *mode_str;
		int vid, mid;
		int idx;
		p = strchr(this_opt, ':');
		if (!p) {
			r = -EINVAL;
			break;
		}

		*p = 0;
		display_str = this_opt;
		mode_str = p + 1;
		if (dc_get_vencid(display_str, &vid)) {
			VPSSDBG("venc name(%s) not existing.\n",
				display_str);
			continue;
		}
		if (dc_get_modeid(mode_str, &mid)) {
			VPSSDBG("venc mode(%s) is not supported.\n",
				mode_str);
			continue;
		}

		get_idx_from_vid(vid, &idx);
		vinfo->modeinfo[idx].vencid = vid;
		vinfo->modeinfo[idx].modeid = mid;
		vinfo->modeinfo[idx].iscustommode = 0;
		get_format_from_mid(mid,
				    &vinfo->modeinfo[idx].framewidth,
				    &vinfo->modeinfo[idx].frameheight,
				    (u8 *)&vinfo->modeinfo[idx].scanformat);

	   if (options == NULL)
			break;
	}

	kfree(str);

	return r;

}

static inline int get_payload_size(void)
{
	int size = 0;
	size  = sizeof(struct vps_dcconfig);
	size += sizeof(struct vps_dcvencinfo);
	size += sizeof(struct vps_dcnodeinput);
	size += sizeof(struct vps_dcmodeinfo);
	size += sizeof(u32);  /*this is for the disable venc command*/
	/*FIXME add more here*/

	return size;
}

static inline void assign_payload_addr(struct vps_dispctrl *dctrl,
				       struct vps_payload_info *dminfo,
				       u32 *buf_offset)
{
	int offset = *buf_offset;

	/*dc config */
	dctrl->dccfg = (struct vps_dcconfig *)
				((u32)dminfo->vaddr + offset);
	dctrl->dccfg_phy = dminfo->paddr + offset;
	offset += sizeof(struct vps_dcconfig);

	/* venc info*/
	dctrl->vinfo = (struct vps_dcvencinfo *)
				((u32)dminfo->vaddr + offset);
	dctrl->vinfo_phy = dminfo->paddr + offset;
	offset += sizeof(struct vps_dcvencinfo);

	/*node input*/
	dctrl->nodeinfo = (struct vps_dcnodeinput *)
				((u32)dminfo->vaddr + offset);
	dctrl->ninfo_phy = dminfo->paddr + offset;
	offset += sizeof(struct vps_dcnodeinput);

	/*venc disable*/
	dctrl->dis_vencs = (u32 *)((u32)dminfo->vaddr + offset);
	dctrl->dis_vencsphy = dminfo->paddr + offset;
	offset += sizeof(u32);

	*buf_offset = offset;
}

int __init vps_dc_init(struct platform_device *pdev, char *mode, int tied_vencs)
{
	int r = 0;
	int i;
	int size = 0, offset = 0;
	VPSSDBG("dctrl init\n");

	dc_payload_info = kzalloc(sizeof(struct vps_payload_info),
				  GFP_KERNEL);

	if (!dc_payload_info) {
		VPSSERR("allocated payload info failed.\n");
		return -ENOMEM;
	}

	/*allocate non-cacheable memory*/
	size = get_payload_size();
	dc_payload_info->vaddr = vps_sbuf_alloc(size, &dc_payload_info->paddr);
	if (dc_payload_info->vaddr == NULL) {
		VPSSERR("alloc dctrl dma buffer failed\n");
		dc_payload_info->paddr = 0u;
		r = -ENOMEM;
		goto cleanup;
	}
	dc_payload_info->size = PAGE_ALIGN(size);
	memset(dc_payload_info->vaddr, 0, dc_payload_info->size);

	/*get dc handle*/
	dc_handle = vps_fvid2_create(FVID2_VPS_DCTRL_DRV,
				     VPS_DCTRL_INST_0,
				     NULL,
				     (void *)dc_payload_info->paddr,
				     NULL);

	if (dc_handle == NULL) {
		VPSSDBG("Create FVID2 DC handle status 0x%08x.\n",
			*(u32 *)dc_payload_info->vaddr);
		r = -EINVAL;
		goto cleanup;
	}


	/*FIXME setup HDMI other devices*/
	disp_ctrl = kzalloc(sizeof(struct vps_dispctrl), GFP_KERNEL);
	if (disp_ctrl == NULL) {
		r = -ENOMEM;
		goto cleanup;
	}

	disp_ctrl->fvid2_handle = dc_handle;
	assign_payload_addr(disp_ctrl, dc_payload_info, &offset);

	disp_ctrl->dvo2clksrc = VPS_DC_DVO2CLKSRC_HDMI;
	disp_ctrl->hdcompclksrc = VPS_DC_HDCOMPCLKSRC_HDCOMP;


	disp_ctrl->blenders[0].idx = HDMI;
	disp_ctrl->blenders[1].idx = HDCOMP;
	disp_ctrl->blenders[2].idx = DVO2;
	disp_ctrl->blenders[3].idx = SDVENC;
	mutex_init(&disp_ctrl->dcmutex);

	r = kobject_init_and_add(
			&disp_ctrl->kobj,
			&dctrl_ktype,
			&pdev->dev.kobj,
			"dctrl");
	if (r)
		VPSSERR("failed to create dctrl sysfs file.\n");

	/*create sysfs*/
	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		struct dc_blenderinfo *blend = &disp_ctrl->blenders[i];;

		blend->actnodes = 0;
		blend->name = v_nameid[i].name;
		blend->dcctrl = disp_ctrl;
		blend->tinfo = NULL;
		r = kobject_init_and_add(
			&blend->kobj, &blender_ktype,
			&pdev->dev.kobj, v_nameid[i].name);

		if (r) {
			VPSSERR("failed to create blender \
				%d sysfs file.\n", i);
			continue;
		}
	}

	disp_ctrl->tiedvenc = tied_vencs;
	venc_info.tiedvencs = disp_ctrl->tiedvenc;


	/*parse the mode*/
	r = parse_def_modes(mode);
	if (r) {
		VPSSERR("failed to parse mode.\n");
		goto cleanup;
	}

	r = dc_set_vencmode(&venc_info);
	if (r) {
		VPSSERR("Failed to set venc mode.\n");
		goto cleanup;
	}
	return 0;
cleanup:
	vps_dc_deinit(pdev);
	return r;
}


int __exit vps_dc_deinit(struct platform_device *pdev)
{
	int r = 0;
	int i;
	VPSSDBG("dctrl deinit\n");

	if (disp_ctrl) {
		/*disable vencs*/
		if (disp_ctrl->enabled_venc_ids != 0) {
			r = dc_venc_disable(VPS_DC_VENC_MASK);
			if (r) {
				VPSSERR("Failed to disable vencs.\n");
				return r;
			}
		}


		kobject_del(&disp_ctrl->kobj);
		kobject_put(&disp_ctrl->kobj);

		for (i = 0; i < VPS_DC_MAX_VENC; i++) {
			kobject_del(&disp_ctrl->blenders[i].kobj);
			kobject_put(&disp_ctrl->blenders[i].kobj);
		}

		kfree(disp_ctrl);
		disp_ctrl = NULL;
	}

	if (dc_payload_info) {

		/*free memory*/
		if (dc_payload_info->vaddr)
			vps_sbuf_free(dc_payload_info->paddr,
				      dc_payload_info->vaddr,
				      dc_payload_info->size);

		kfree(dc_payload_info);
	}

	if (dc_handle) {
		r = vps_fvid2_delete(dc_handle, NULL);
		if (r) {
			VPSSERR("failed to delete DC fvid2 handle.\n");
			return r;
		}
		dc_handle = NULL;
	}


	return r;
}


