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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/vps_proxyserver.h>
#include <linux/fvid2.h>
#include <linux/vps.h>
#include <linux/vps_displayctrl.h>
#include <linux/dc.h>
#include <linux/grpx.h>

#include "core.h"

#ifdef DEBUG
#define DCDBG(format, ...) \
	if (vpss_debug) \
		printk(KERN_INFO"VPSS_DC: " \
			format, ## __VA_ARGS__)

#define DCERR(format, ...) \
	if (vpss_debug)  \
		printk(KERN_ERR"VPSS_DC: " \
			format, ## __VA_ARGS__)

#else
#define DCDBG(format, ...)
#define DCERR(format, ...)
#endif

static struct vps_dispctrl *disp_ctrl;

/*store the current VENC setting*/
static struct vps_dcvencinfo venc_info = {
	{
		{VPS_DC_VENC_HDMI, 0, VPS_DC_MODE_1080P_60, 1920, \
		1080, FVID2_SF_PROGRESSIVE, NULL},

		{VPS_DC_VENC_HDCOMP, 0, VPS_DC_MODE_720P_60,	\
		1280, 720, FVID2_SF_PROGRESSIVE, NULL},

		{VPS_DC_VENC_DVO2, 0, VPS_DC_MODE_1080P_60,    \
		1920, 1080, FVID2_SF_PROGRESSIVE, NULL},

		{VPS_DC_VENC_SD, 0, VPS_DC_MODE_NTSC,	  \
		720, 480, FVID2_SF_INTERLACED, NULL},
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
	{"grpx22", VPS_DC_GRPX2_INPUT_PATH},			/*21*/
	{"hdmi",  VPS_DC_HDMI_BLEND},				/*22*/
	{"hdcomp", VPS_DC_HDCOMP_BLEND},			/*23*/
	{"dvo2", VPS_DC_DVO2_BLEND},				/*24*/
	{"sdvenc", VPS_DC_SDVENC_BLEND},			/*25*/
};

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

static int get_format_from_vid(int vid, u32 *width, u32 *height, u8 *scformat)
{
	int i;
	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		if (vid == venc_info.modeinfo[i].vencid) {
			*width = modeinfo[venc_info.modeinfo[i].modeid].width;
			*height = modeinfo[venc_info.modeinfo[i].
					modeid].height;
			*scformat =
				modeinfo[venc_info.modeinfo[i].
					modeid].scformat;
			return 0;
		}
	}
	return -EINVAL;
}

static int get_format_from_bid(int bid, u32 *width, u32 *height, u8 *scformat)
{
	int i;
	int r = -EINVAL;
	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		if (bid == v_nameid[i].blendid) {
			r = get_format_from_vid(v_nameid[i].vid,
					width, height, scformat);
			break;
		}
	}

	return r;
}

static inline void set_actnodes(u8 setflag, u8 id)
{
	struct dc_blenderinfo *binfo = &disp_ctrl->blenders[id];

	if (setflag)
		binfo->actnodes++;
	else
		binfo->actnodes--;

}
int vps_dc_get_node_id(int *id, char *name)
{
	int i;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	if (strcmp(name, "dummy") == 0)
		return -EINVAL;

	for (i = 0; i < VPS_DC_MAX_NODE_NUM; i++) {
		struct dcnode_info *ninfo = &dcnode[i];
		if (strcmp(name, ninfo->name) == 0) {
			*id =  ninfo->id;
			return 0;
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL(vps_dc_get_node_id);

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
EXPORT_SYMBOL(vps_dc_get_node_name);

int vps_dc_get_clksrc(enum vps_dcdvo2clksrc *dvo2,
			enum vps_dchdcompclksrc *hdcomp)
{
	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	DCDBG("enter get clksrc\n");
	dc_lock(disp_ctrl);
	*dvo2 = disp_ctrl->dvo2clksrc;
	*hdcomp = disp_ctrl->hdcompclksrc;
	dc_unlock(disp_ctrl);
	return 0;
}
EXPORT_SYMBOL(vps_dc_get_clksrc);

int vps_dc_set_config(struct vps_dcconfig *usercfg, int setflag)
{
	int r = 0;
	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	if (usercfg->vencinfo.numvencs > VPS_DC_MAX_VENC) {
		DCERR("num vens (%d) over max\n",
			usercfg->vencinfo.numvencs);
		return -EINVAL;
	}

	DCDBG("enter set config\n");
	dc_lock(disp_ctrl);
	memcpy(disp_ctrl->dccfg, usercfg, sizeof(struct vps_dcconfig));
	if (setflag) {
		r = vps_fvid2_control(disp_ctrl->fvid2_handle,
				      IOCTL_VPS_DCTRL_SET_CONFIG,
				      (void *)disp_ctrl->dccfg_phy,
				      NULL);
		if (r)
			DCDBG("faield to set the DC config.\n");
	} else {
		r = vps_fvid2_control(disp_ctrl->fvid2_handle,
				      IOCTL_VPS_DCTRL_CLEAR_CONFIG,
				      (void *)disp_ctrl->dccfg_phy,
				      NULL);

		if (r)
			DCDBG("faield to clear the DC config.\n");

	}

	/*based on the end node id to determine whether
	  there is a new edge connected/disconnected to the blend*/

	if (r == 0) {
		int i;
		enum vps_dcusecase ucase = disp_ctrl->dccfg->usecase;
		switch (ucase) {
		case VPS_DC_TRIDISPLAY:
			set_actnodes(setflag, HDMI);
			set_actnodes(setflag, DVO2);
			set_actnodes(setflag, SDVENC);
			break;
		case VPS_DC_DUALHDDISPLAY:
			set_actnodes(setflag, HDMI);
			set_actnodes(setflag, DVO2);
			break;
		case VPS_DC_DUALHDSDDISPLAY:
			set_actnodes(setflag, HDMI);
			set_actnodes(setflag, SDVENC);
			break;
		case VPS_DC_USERSETTINGS:
			for (i = 0; i < disp_ctrl->dccfg->numedges; i++) {
				struct vps_dcedgeinfo *einfo =
					&disp_ctrl->dccfg->edgeinfo[i];
				switch (einfo->endnode) {
				case VPS_DC_HDMI_BLEND:
					set_actnodes(setflag, HDMI);
					break;
				case VPS_DC_HDCOMP_BLEND:
					set_actnodes(setflag, HDCOMP);
					break;
				case VPS_DC_DVO2_BLEND:
					set_actnodes(setflag, DVO2);
					break;
				case VPS_DC_SDVENC_BLEND:
					set_actnodes(setflag, SDVENC);
					break;
				}

			}
			break;
		default:
			DCDBG("wrong usercaes.\n");
			break;
		}
	}

	dc_unlock(disp_ctrl);

	return r;
}
EXPORT_SYMBOL(vps_dc_set_config);

int vps_dc_venc_disable(int *vid, int numvenc)
{
	int i, j, r = 0;
	struct vps_dcvencinfo *vinfo = disp_ctrl->vinfo;
	int venc_ids = 0;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	if (numvenc > VPS_DC_MAX_VENC)
		return -EINVAL;

	DCDBG("enter venc disable\n");
	dc_lock(disp_ctrl);
	for (i = 0; i < numvenc; i++) {
		for (j = 0; j < VPS_DC_MAX_VENC; j++) {
			if (vid[i] == vinfo->modeinfo[j].vencid) {
				venc_ids = vid[i];
				break;
			}
		}

		/*FIX ME check what kind of edge is connected to this venc
		if the edge is not disable, then need disable the edge first
		before the venc*/


		/* the venc is already disable, do not need do it again*/
		if ((disp_ctrl->enabled_venc_ids & vid[i]) == 0)
			venc_ids &= ~vid[i];

		if (j == VPS_DC_MAX_VENC) {
			dc_unlock(disp_ctrl);
			return -EINVAL;
		}
	}
	/*call FIVD2 to disable the assigend venc*/
	if (venc_ids) {
		r = vps_fvid2_control(disp_ctrl->fvid2_handle,
				      IOCTL_VPS_DCTRL_DISABLE_VENC,
				      (void *)virt_to_phys(&venc_ids),
				      NULL);

		if (r == 0)
			disp_ctrl->enabled_venc_ids &= ~venc_ids;
		else
			DCERR("failed to disable the venc.\n");

	}

	dc_unlock(disp_ctrl);
	return r;
}



int vps_dc_get_vencmode(int id,
			struct vps_dcmodeinfo *modeinfo,
			enum dc_idtype type)
{
	int i = 0;
	int vid;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	if (type == DC_MODE_ID)
		return -EINVAL;

	if (type == DC_BLEND_ID) {
		for (i = 0; i < VPS_DC_MAX_VENC; i++) {
			if (id == v_nameid[i].blendid) {
				vid = v_nameid[i].vid;
				break;
			}
		}
	} else
		vid = id;

	if (i == VPS_DC_MAX_VENC)
		return -EINVAL;

	DCDBG("enter get vencmode\n");
	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		if (vid == venc_info.modeinfo[i].vencid) {
			*modeinfo = venc_info.modeinfo[i];
			return 0;
		}
	}

	return -EINVAL;
}
EXPORT_SYMBOL(vps_dc_get_vencmode);

int vps_dc_set_vencmode(int *vid, int *mid,
			void *modeinfo,
			int numvenc,
			int tiedvenc)
{
	int i, j, r = 0;
	struct vps_dcvencinfo *vinfo = disp_ctrl->vinfo;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	if (numvenc > VPS_DC_MAX_VENC) {
		DCDBG("num %d vencs to set is bigger than supported.\n",
			numvenc);
		return -EINVAL;
	}

	DCDBG("enter set venc mode\n");
	dc_lock(disp_ctrl);
	for (i = 0; i < numvenc; i++) {
		/* bit maps of 4 venc is 0xF*/
		if ((vid[i] & 0xF) == 0) {
			DCERR(" venc id %d is nonexist.\n", vid[i]);
			r = -EINVAL;
			goto exit;
		}

		if (mid[i] >= VPS_DC_MAX_MODE) {
			DCERR(" mode %d is nonexist.\n", mid[i]);
			r = -EINVAL;
			goto exit;
		}

		/*check whether the venc is already running, it yes, ask
			app to disable it before update*/
		if (disp_ctrl->enabled_venc_ids & vid[i]) {
			DCERR("venc id %d is already running, \
				disable it first before change.\n",
				vid[i]);
			r = -EINVAL;
			goto exit;
		}
	}

	vinfo->numvencs = numvenc;
	vinfo->tiedvencs = tiedvenc & 0xF;
	/*prepare the IOCTL parameters*/
	for (i = 0; i < numvenc; i++) {
		vinfo->modeinfo[i].vencid = vid[i];
		vinfo->modeinfo[i].iscustommode = 0;
		vinfo->modeinfo[i].modeid = mid[i];

		get_format_from_mid(mid[i],
				    &vinfo->modeinfo[i].framewidth,
				    &vinfo->modeinfo[i].frameheight,
				    (u8 *)&vinfo->modeinfo[i].scanformat);

	}

	/*do not enable the ennc */
#if 0
	/*set the VENC Mode*/
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			IOCTL_VPS_DCTRL_SET_VENC_MODE,
			(void *)disp_ctrl->vinfo_phy,
			NULL);
	if (r) {
		DCERR("failed to set venc mdoe.\n");
		goto exit;
	}

#endif
	if (r == 0) {
		for (i = 0; i < numvenc; i++) {
			/*disp_ctrl->enabled_venc_ids |= vid[i];*/

			for (j = 0; j < VPS_DC_MAX_VENC; j++)
				if (vid[i] == venc_info.modeinfo[j].vencid)
					memcpy(&venc_info.modeinfo[j],
					       &vinfo->modeinfo[i],
					       sizeof(struct vps_dcmodeinfo));

		}
	}
exit:
	dc_unlock(disp_ctrl);
	return r;

}
EXPORT_SYMBOL(vps_dc_set_vencmode);

int vps_dc_get_outpfmt(int id, u32 *width,
		       u32 *height,
		       u8 *scformat,
		       enum dc_idtype type)
{
	int r;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	DCDBG("enter get output format\n");

	if (type == DC_VENC_ID)
		r = get_format_from_vid(id, width, height, scformat);
	 else if (type == DC_BLEND_ID)
		r = get_format_from_bid(id, width, height, scformat);
	 else if (type == DC_MODE_ID)
		r = get_format_from_mid(id, width, height, scformat);
	 else
		r = -EINVAL;

	return r;
}
EXPORT_SYMBOL(vps_dc_get_outpfmt);

int vps_dc_set_node(u8 nodeid, u8 inputid, u8 enable)
{

	int r = 0;
	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	DCDBG("enter set node\n");
	dc_lock(disp_ctrl);
	disp_ctrl->nodeinfo->nodeid = nodeid;
	disp_ctrl->nodeinfo->inputid = inputid;
	disp_ctrl->nodeinfo->isenable = enable;
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_NODE_INPUT,
			      (void *)disp_ctrl->ninfo_phy,
			      NULL);
	if (r)
		DCERR("failed to enable node.\n");

	dc_unlock(disp_ctrl);
	return r;
}
EXPORT_SYMBOL(vps_dc_set_node);


int vps_dc_get_vencid(char *vname, int *vid)
{

	int i;
	struct venc_name_id *vnid;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	DCDBG("enter get venc id\n");
	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		vnid = &v_nameid[i];
		if (!strcmp(vname, vnid->name)) {
			*vid = vnid->vid;
			return 0;
		}
	}
   return -1;
}
EXPORT_SYMBOL(vps_dc_get_vencid);

int vps_dc_get_modeid(char *mname, int *mid)
{
	int i;
	struct venc_modeinfo *vinfo;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	DCDBG("enter get mode id\n");
	for (i = 0; i < VPS_DC_MAX_MODE; i++) {
		vinfo = &modeinfo[i];
		if (!strcmp(mname, vinfo->name)) {
			*mid = vinfo->mid;
			return 0;
		}
	}
	return -1;
}
EXPORT_SYMBOL(vps_dc_get_modeid);


/*SYSFS Function start from here*/
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

	if (binfo->actnodes) {
		DCERR("disable the nodes before changing the mode.\n");
		return -EINVAL;
	}

	dc_lock(binfo->dcctrl);
	input  = strsep(&input, "\n");
	r = vps_dc_get_modeid(input, &mid);
	if (r) {
		dc_unlock(binfo->dcctrl);
		return r;
	}

	venc_info.modeinfo[idx].modeid = mid;
	dc_unlock(binfo->dcctrl);
	return size;
}

static ssize_t blender_timings_show(struct dc_blenderinfo *binfo, char *buf)
{

	return snprintf(buf, PAGE_SIZE , "blender timing\n");

}

static ssize_t blender_timings_store(struct dc_blenderinfo *binfo,
		const char *buf, size_t size)
{
	return size;
}

static ssize_t blender_enabled_show(struct dc_blenderinfo *binfo, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "TBD\n");
}

static ssize_t blender_enabled_store(struct dc_blenderinfo *binfo,
				     const char *buf,
				     size_t size)
{
	DCDBG("TBD\n");
	return size;
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

static struct attribute *blender_sysfs_attrs[] = {
	&blender_attr_mode.attr,
	&blender_attr_timing.attr,
	&blender_attr_enabled.attr,
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


int vps_dc_init(struct platform_device *pdev)
{
	int status = EINVAL;
	int r = 0;
	int i;
	DCDBG("DC INIT\n");

	disp_ctrl = kzalloc(sizeof(struct vps_dispctrl), GFP_KERNEL);

	disp_ctrl->dccfg = kzalloc(sizeof(struct vps_dcconfig), GFP_KERNEL);
	BUG_ON(disp_ctrl->dccfg == NULL);
	disp_ctrl->dccfg_phy = virt_to_phys(disp_ctrl->dccfg);

	disp_ctrl->dvo2clksrc = VPS_DC_DVO2CLKSRC_HDMI;
	disp_ctrl->hdcompclksrc = VPS_DC_HDCOMPCLKSRC_HDCOMP;

	disp_ctrl->vinfo = kzalloc(sizeof(struct vps_dcvencinfo), GFP_KERNEL);
	BUG_ON(disp_ctrl->vinfo == NULL);
	disp_ctrl->vinfo_phy = virt_to_phys(disp_ctrl->vinfo);

	disp_ctrl->nodeinfo = kzalloc(sizeof(struct vps_dcnodeinput),
		GFP_KERNEL);
	BUG_ON(disp_ctrl->nodeinfo == NULL);
	disp_ctrl->ninfo_phy = virt_to_phys(disp_ctrl->nodeinfo);

	disp_ctrl->blenders[0].idx = HDMI;
	disp_ctrl->blenders[1].idx = HDCOMP;
	disp_ctrl->blenders[2].idx = DVO2;
	disp_ctrl->blenders[3].idx = SDVENC;
	mutex_init(&disp_ctrl->dcmutex);

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
			DCERR("failed to create blend \
				%d sysfs file.\n", i);
			continue;
		}
	}

	disp_ctrl->fvid2_handle =
		vps_fvid2_create(FVID2_VPS_DCTRL_DRV,
				 VPS_DCTRL_INST_0,
				 NULL,
				 (void *)virt_to_phys(&status),
				 NULL);

	if (disp_ctrl->fvid2_handle == NULL) {
		DCDBG("Create FVID2 DC handle status 0x%08x.\n", status);
		r = -EINVAL;
	} else
		r = 0;

   return r;
}


int vps_dc_exit(struct platform_device *pdev)
{
	int r = 0;
	int i;
	DCDBG("DC EXIT\n");

	if (disp_ctrl) {
		kfree(disp_ctrl->nodeinfo);
		disp_ctrl->nodeinfo = NULL;
		disp_ctrl->ninfo_phy = 0;

		kfree(disp_ctrl->vinfo);
		disp_ctrl->vinfo = NULL;
		disp_ctrl->vinfo_phy = 0;;

		kfree(disp_ctrl->dccfg);
		disp_ctrl->dccfg = NULL;
		disp_ctrl->dccfg_phy = 0;
		/*This is not used currently*/
		if (disp_ctrl->enabled_venc_ids != 0)
			r = vps_fvid2_control(disp_ctrl->fvid2_handle,
					      IOCTL_VPS_DCTRL_DISABLE_VENC,
					      (void *)virt_to_phys
						(&disp_ctrl->enabled_venc_ids),
					      NULL);

		if (disp_ctrl->fvid2_handle) {
			r = vps_fvid2_delete(disp_ctrl->fvid2_handle, NULL);
			if (r) {
				DCERR("failed to delete DC fvid2 handle.\n");
				return r;
			}
		}
		disp_ctrl->fvid2_handle = NULL;

		for (i = 0; i < VPS_DC_MAX_VENC; i++) {
			kobject_del(&disp_ctrl->blenders[i].kobj);
			kobject_put(&disp_ctrl->blenders[i].kobj);
		}
		kfree(disp_ctrl);
		disp_ctrl = NULL;
	}
	return r;
}


