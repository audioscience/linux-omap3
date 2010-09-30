/*
 * linux/drivers/video/ti81xx/vpss/dctrl.c
 *
 * VPSS display controller driver for TI 81XX
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
#include <mach/board-ti816x.h>

#include "core.h"
#include "system.h"
#include "dc.h"


static struct vps_dispctrl *disp_ctrl;
static void *dc_handle;
static struct vps_payload_info  *dc_payload_info;

/*store the current VENC setting*/
static struct vps_dcvencinfo venc_info = {
	{
		{VPS_DC_VENC_HDMI, 0, FVID2_STD_1080P_60, 1920, \
		1080, FVID2_SF_PROGRESSIVE, NULL, 0},
#ifdef CONFIG_ARCH_TI816X
		{VPS_DC_VENC_HDCOMP, 0, FVID2_STD_1080I_60,	\
		1920, 1080, FVID2_SF_INTERLACED, NULL, 0},
#endif
		{VPS_DC_VENC_DVO2, 0, FVID2_STD_1080P_60,    \
		1920, 1080, FVID2_SF_PROGRESSIVE, NULL, 0},

		{VPS_DC_VENC_SD, 0, FVID2_STD_NTSC,	  \
		720, 480, FVID2_SF_INTERLACED, NULL, 0},
	},
	0,
	VPS_DC_MAX_VENC,
};
/**********************************************************
  The name in the followig arrays are the value used in the sysfs.
**********************************************************/

/*store the current mode info*/
static struct dc_vencmode_info mode_name[] = {
	{"ntsc", 720, 480, 0, FVID2_STD_NTSC},
	{"pal", 720, 576, 0, FVID2_STD_PAL},
	{"1080p-60", 1920, 1080, 1, FVID2_STD_1080P_60},
	{"720p-60", 1280, 720, 1, FVID2_STD_720P_60},
	{"1080i-60", 1920, 1080, 0, FVID2_STD_1080I_60},
	{"1080p-30", 1920, 1080, 1, FVID2_STD_1080P_30},
};

static struct dc_vencname_info venc_name[VPS_DC_MAX_VENC] = {
	{"hdmi", VPS_DC_VENC_HDMI, VPS_DC_HDMI_BLEND, HDMI},
#ifdef CONFIG_ARCH_TI816X
	{"hdcomp", VPS_DC_VENC_HDCOMP, VPS_DC_HDCOMP_BLEND, HDCOMP},
#endif
	{"dvo2", VPS_DC_VENC_DVO2, VPS_DC_DVO2_BLEND, DVO2},
	{"sdvenc", VPS_DC_VENC_SD, VPS_DC_SDVENC_BLEND, SDVENC}
};

static struct vps_sname_info pllvenc_name[] = {
	 {"rfclk", VPS_SYSTEM_VPLL_OUTPUT_VENC_RF},
	 {"dclk", VPS_SYSTEM_VPLL_OUTPUT_VENC_D},
	 {"aclk", VPS_SYSTEM_VPLL_OUTPUT_VENC_A}
};

static struct vps_sname_info pllclk_name[VPS_SYSTEM_VPLL_MAX_FREQ] = {
	{"54", VPS_SYSTEM_VPLL_FREQ_54MHZ},
	{"74.25", VPS_SYSTEM_VPLL_FREQ_74_25MHZ},
	{"148.5", VPS_SYSTEM_VPLL_FREQ_148_5MHZ},
	{"297", VPS_SYSTEM_VPLL_FREQ_297MHZ}
};

static struct vps_sname_info vclksrc_name[] =  {
	{"dclk", VPS_DC_CLKSRC_VENCD},
	{"dclkdiv2", VPS_DC_CLKSRC_VENCD_DIV2},
	{"dclkdiff", VPS_DC_CLKSRC_VENCD_DIV2_DIFF},
	{"aclk", VPS_DC_CLKSRC_VENCA},
	{"aclkdiv2", VPS_DC_CLKSRC_VENCA_DIV2},
	{"aclkdiff", VPS_DC_CLKSRC_VENCA_DIV2_DIFF}
};

static struct vps_sname_info dfmt_name[VPS_DC_DVOFMT_MAX] = {
	{"single", VPS_DC_DVOFMT_SINGLECHAN},
	{"double", VPS_DC_DVOFMT_DOUBLECHAN},
	{"triple", VPS_DC_DVOFMT_TRIPLECHAN_EMBSYNC},
	{"triplediscrete", VPS_DC_DVOFMT_TRIPLECHAN_DISCSYNC},
	{"doublediscrete", VPS_DC_DVOFMT_DOUBLECHAN_DISCSYNC},

};

static struct vps_sname_info afmt_name[VPS_DC_A_OUTPUT_MAX] = {
	{"composite", VPS_DC_A_OUTPUT_COMPOSITE},
	{"svideo", VPS_DC_A_OUTPUT_SVIDEO},
	{"component", VPS_DC_A_OUTPUT_COMPONENT},
};

static struct vps_sname_info datafmt_name[] = {
	{"rgb888", FVID2_DF_RGB24_888},
	{"yuv444p", FVID2_DF_YUV444P},
	{"yuv422spuv", FVID2_DF_YUV422SP_UV},
};

static struct vps_sname_info dc_nodes[] = {
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
#ifdef CONFIG_ARCH_TI816X
	{"hdcomp", VPS_DC_HDCOMP_BLEND},			/*23*/
	{"dvo2", VPS_DC_DVO2_BLEND},				/*24*/
	{"sdvenc", VPS_DC_SDVENC_BLEND},			/*25*/
#else
	{"dvo2", VPS_DC_DVO2_BLEND},				/*23*/
	{"sdvenc", VPS_DC_SDVENC_BLEND},			/*24*/
#endif
};

/*S***************************private funtions*******************/

static inline void dc_lock(struct vps_dispctrl *dctrl)
{
  mutex_lock(&dctrl->dcmutex);
}

static inline void dc_unlock(struct vps_dispctrl *dctrl)
{
    mutex_unlock(&dctrl->dcmutex);
}

static inline bool isdigitalvenc(int vid)
{
	if ((vid == VPS_DC_VENC_HDMI) ||
		(vid == VPS_DC_VENC_DVO2))
		return true;

	return false;
}

static inline bool isdigitalclk(enum vps_dcvencclksrcsel clk)
{
	if ((clk == VPS_DC_CLKSRC_VENCA) ||
		(clk == VPS_DC_CLKSRC_VENCA_DIV2) ||
		(clk == VPS_DC_CLKSRC_VENCA_DIV2_DIFF))
		return false;

	return true;
}
static inline bool isvalidclksrc(int vid, enum vps_dcvencclksrcsel clk)
{
	if ((vid == VPS_DC_VENC_HDMI) && (!isdigitalclk(clk)))
		return false;

	return true;
}

static inline bool isvalidmode(int vid, int mid)
{
	switch (vid) {
	case VPS_DC_VENC_HDMI:
#ifdef CONFIG_ARCH_TI816X
	case VPS_DC_VENC_HDCOMP:
#endif
	case VPS_DC_VENC_DVO2:
		if ((mid == FVID2_STD_NTSC) || (mid == FVID2_STD_PAL))
			return false;
		break;
	case VPS_DC_VENC_SD:
		if (!((mid == FVID2_STD_NTSC) || (mid == FVID2_STD_PAL)))
			return false;
		break;
	}

	return true;
}

/*get the clock venc*/
static inline u32 get_plloutputvenc(int bid)
{
	struct vps_dcvencclksrc clksrc = disp_ctrl->blenders[bid].clksrc;

	if (bid == SDVENC)
		return VPS_SYSTEM_VPLL_OUTPUT_VENC_RF;

	if (isdigitalclk(clksrc.clksrc))
		return VPS_SYSTEM_VPLL_OUTPUT_VENC_D;
	else
		return VPS_SYSTEM_VPLL_OUTPUT_VENC_A;
}

/*get the current format based on the mode id*/
static int dc_get_format_from_mid(int mid,
					    u32 *width,
					    u32 *height,
					    u8 *scformat)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(mode_name); i++) {
		/*FIX me add customer mode in the future*/
		if (mid == mode_name[i].mid) {
			*width = mode_name[i].width;
			*height = mode_name[i].height;
			*scformat = mode_name[i].scformat;
			return 0;
		}
	}

	return -EINVAL;
}


/*get the index of the desired venc id in the database*/
static int get_idx_from_vid(int vid, int *idx)
{
	int i;

	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		if (vid == venc_name[i].vid) {
			*idx = venc_name[i].idx;
			return 0;
		}
	}

	return -EINVAL;
}

/*get the venc id based on the name*/
static int dc_get_vencid(char *vname, int *vid)
{

	int i;
	struct dc_vencname_info *vnid;

	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		vnid = &venc_name[i];
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
	struct dc_vencmode_info *vinfo;

	for (i = 0; i < ARRAY_SIZE(mode_name); i++) {
		vinfo = &mode_name[i];
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


	if (strcmp(name, "dummy") == 0)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(dc_nodes); i++) {
		struct vps_sname_info *ninfo = &dc_nodes[i];
		if (strcmp(name, ninfo->name) == 0) {
			*nid =  ninfo->value;
			return 0;
		}
	}
	return -EINVAL;
}

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

/*get the pll clock based on the mode*/
static enum vps_vpllclkfreq dc_get_pllfreq(int vid, int mid)
{
	enum vps_dcvencclksrcsel  clk;
	int idx;

	get_idx_from_vid(vid, &idx);

	clk = disp_ctrl->blenders[idx].clksrc.clksrc;
	switch (mid) {
	case FVID2_STD_1080P_60:
		if (vid == VPS_DC_VENC_HDMI)
			return VPS_SYSTEM_VPLL_FREQ_297MHZ;
		else if (vid == VPS_DC_VENC_DVO2) {
			if (isdigitalclk(clk))
				return VPS_SYSTEM_VPLL_FREQ_297MHZ;
			else
				return VPS_SYSTEM_VPLL_FREQ_148_5MHZ;
		}
#ifdef CONFIG_ARCH_TI816X
		else if (vid == VPS_DC_VENC_HDCOMP) {
			if (isdigitalclk(clk))
				return VPS_SYSTEM_VPLL_FREQ_297MHZ;
			else
				return VPS_SYSTEM_VPLL_FREQ_148_5MHZ;
		}
#endif
	case FVID2_STD_1080P_30:
	case FVID2_STD_1080I_60:
	case FVID2_STD_720P_60:
		if (vid == VPS_DC_VENC_HDMI)
			return VPS_SYSTEM_VPLL_FREQ_148_5MHZ;
		else if (vid == VPS_DC_VENC_DVO2) {
			if (isdigitalclk(clk))
				return VPS_SYSTEM_VPLL_FREQ_148_5MHZ;
			else
				return VPS_SYSTEM_VPLL_FREQ_74_25MHZ;
		}
#ifdef CONFIG_ARCH_TI816X
		else if (vid == VPS_DC_VENC_HDCOMP) {
			if (isdigitalclk(clk))
				return VPS_SYSTEM_VPLL_FREQ_148_5MHZ;
			else
				return VPS_SYSTEM_VPLL_FREQ_74_25MHZ;
		}
#endif
	case FVID2_STD_NTSC:
	case FVID2_STD_PAL:
		return VPS_SYSTEM_VPLL_FREQ_54MHZ;
	default:
		return VPS_SYSTEM_VPLL_FREQ_148_5MHZ;
	}
}

/*is venc running*/
static int dc_isvencrunning(int vid)
{
	struct vps_dcvencinfo vinfo;
	int r = 0;
	vinfo.numvencs = 1;
	vinfo.modeinfo[0].vencid = vid;
	r = dc_get_vencinfo(&vinfo);

	if (!r)
		return vinfo.modeinfo[0].isvencrunning;

	return r;
}

/*Get the current VENC output info*/
static int dc_get_output(struct vps_dcoutputinfo *oinfo)
{
	int r = 0;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	/*get the venc output info*/
	disp_ctrl->opinfo->vencnodenum = oinfo->vencnodenum;
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_GET_VENC_OUTPUT,
			      (void *)disp_ctrl->opinfo_phy,
			      NULL);

	if (r)
		VPSSERR("failed to get venc output info\n");
	else
		memcpy(oinfo,
		       disp_ctrl->opinfo,
		       sizeof(struct vps_dcoutputinfo));
	return r;
}

/*Set the VENC outputs*/
static int dc_set_output(struct vps_dcoutputinfo *oinfo)
{
	int r;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;


	memcpy(disp_ctrl->opinfo, oinfo, sizeof(struct vps_dcoutputinfo));
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
		IOCTL_VPS_DCTRL_SET_VENC_OUTPUT,
		(void *)disp_ctrl->opinfo_phy,
		NULL);
	if (r)
		VPSSERR("failed to set venc output\n");

	return r;

}

/*set up the pll clock*/
static int dc_set_pll(int bid, int mid)
{

	struct vps_systemvpllclk pll;
	int r = 0;
	enum vps_vpllclkfreq  freq;
	bool    update = false;


	freq =  dc_get_pllfreq(venc_name[bid].vid, mid);
	pll.outputvenc = (enum vps_vplloutputclk)get_plloutputvenc(bid);
	r = vps_system_getpll(&pll);
	if (r)
		return r;
	/*only if it is a new clock*/
	if (pll.outputclk != freq) {
		update = true;
		pll.outputclk = freq;
	}

	if (update == true)
		r = vps_system_setpll(&pll);

	return r;
}

/*get the clock source*/
static int dc_get_clksrc(struct vps_dcvencclksrc *clksrc)
{
	int r = 0;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	disp_ctrl->clksrc->venc = clksrc->venc;
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_GET_VENC_CLK_SRC,
			      (void *)disp_ctrl->clksrc_phy,
			      NULL);

	if (r)
		VPSSERR("get clock source failed\n");
	else
		clksrc->clksrc = disp_ctrl->clksrc->clksrc;

	return r;
}

/*set the clock source*/
static int dc_set_clksrc(struct vps_dcvencclksrc *clksrc)
{
	int r = 0;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	if (!isvalidclksrc(clksrc->venc, clksrc->clksrc)) {
		VPSSERR("invalid clock source\n");
		return -EINVAL;
	}

	disp_ctrl->clksrc->venc = clksrc->venc;
	disp_ctrl->clksrc->clksrc = clksrc->clksrc;

	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_SET_VENC_CLK_SRC,
			      (void *)disp_ctrl->clksrc_phy,
			      NULL);
	if (r)
		VPSSERR("set clock source failed\n");

	return r;
}
/*get the format based on the venc id*/
static int dc_get_format_from_vid(int vid,
					   u32 *width,
					   u32 *height,
					   u8 *scformat)
{
	int r = 0;
	struct vps_dcvencinfo vinfo;

	vinfo.numvencs = 1;
	vinfo.modeinfo[0].vencid = vid;

	r = dc_get_vencinfo(&vinfo);
	if (r)
		return -EINVAL;

	if (vinfo.modeinfo[0].iscustommode) {
		*width = vinfo.modeinfo[0].framewidth;
		*height = vinfo.modeinfo[0].frameheight;
		*scformat = vinfo.modeinfo[0].scanformat;
	} else
		r = dc_get_format_from_mid(vinfo.modeinfo[0].standard,
					   width,
					   height,
					   scformat);

	return r;
}

/*get the format based on the blender id*/
static int dc_get_format_from_bid(int bid,
					   u32 *width,
					   u32 *height,
					   u8 *scformat)
{
	int i;
	int r = -EINVAL;
	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		if (bid == venc_name[i].blendid) {
			r = dc_get_format_from_vid(venc_name[i].vid,
						   width,
						   height,
						   scformat);
			break;
		}
	}

	return r;
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

#ifdef CONFIG_TI81XX_VPSS_SII9022A
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
			if (vi.modeinfo[i].standard !=
			    vinfo->modeinfo[i].standard) {
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
#ifdef CONFIG_TI81XX_VPSS_SII9022A
	/*config the sii9022a*/
	if (!r) {
		for (i = 0; i < vinfo->numvencs; i++) {
			if (vinfo->modeinfo[i].vencid == VPS_DC_VENC_DVO2) {
				struct vps_dcmodeinfo *minfo =
							&vinfo->modeinfo[i];
				r = sii9022a_setmode(minfo->standard);
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
	struct dc_blender_info *binfo = &disp_ctrl->blenders[bid];

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
	for (i = 0; i < ARRAY_SIZE(dc_nodes); i++) {
		struct vps_sname_info *ninfo = &dc_nodes[i];
		if (id == ninfo->value) {
			strcpy(name, ninfo->name);
			return 0;
		}
	}
	return -EINVAL;

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
		r = dc_get_format_from_vid(id, width, height, scformat);
	 else if (type == DC_BLEND_ID)
		r = dc_get_format_from_bid(id, width, height, scformat);
	 else if (type == DC_MODE_ID)
		r = dc_get_format_from_mid(id, width, height, scformat);
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
static ssize_t blender_mode_show(struct dc_blender_info *binfo, char *buf)
{
	int i;
	u32 idx = binfo->idx;
	int l = 0;
	for (i = 0; i < ARRAY_SIZE(mode_name); i++) {
		if (mode_name[i].mid == venc_info.modeinfo[idx].standard) {
			l = snprintf(buf, PAGE_SIZE, "%s\n",
				mode_name[i].name);
			break;
		}
	}
	return l;
}

static ssize_t blender_mode_store(struct dc_blender_info *binfo,
		const char *buf, size_t size)
{
	int r = 0;
	u32 idx = binfo->idx;
	u32 mid;
	char *input = (char *)buf;

	dc_lock(binfo->dctrl);

	/*venc should be stop before changes*/

	if (dc_isvencrunning(venc_info.modeinfo[idx].vencid)) {
		VPSSERR("stop venc before changing mode\n");
		r = -EINVAL;
		goto exit;
	}

	input  = strsep(&input, "\n");
	if (dc_get_modeid(input, &mid)) {
		VPSSERR("failed to get the mode %s.\n", input);
		r = -EINVAL;
		goto exit;
	}
	/*make sure the mode is supported by the venc*/
	if (!isvalidmode(venc_info.modeinfo[idx].vencid, mid))
		goto exit;

	venc_info.modeinfo[idx].standard = mid;
	/*only set the PLL if it is auto mode*/
	if (binfo->dctrl->automode) {
		r = dc_set_pll(binfo->idx, mid);
		if (r)
			goto exit;
	}
#ifdef CONFIG_ARCH_TI816X
	if (binfo->idx == HDCOMP) {
		if (mid == FVID2_STD_1080P_60)
			r = pcf8575_ths7360_hd_enable(
				TI816X_THS7360_SF_TRUE_HD_MODE);
		else
			r = pcf8575_ths7360_hd_enable(
				TI816X_THS7360_SF_HD_MODE);
		if (r) {
			VPSSERR("failed to set THS filter\n");
			goto exit;
		}

	}
#endif
	r = size;
exit:
	dc_unlock(binfo->dctrl);
	return r;
}

static ssize_t blender_timings_show(struct dc_blender_info *binfo, char *buf)
{
	int r;
	struct vps_dctiminginfo t;
	struct vps_dcvencinfo vinfo;

	dc_lock(binfo->dctrl);

	memset(&t, 0, sizeof(struct vps_dctiminginfo));
	vinfo.modeinfo[0].vencid = venc_name[binfo->idx].vid;
	vinfo.numvencs = 1;

	r = dc_get_vencinfo(&vinfo);
	dc_unlock(binfo->dctrl);

	if (r) {
		VPSSERR(" Failed to get venc infor\n");
		return -EINVAL;
	}

	t.mode = vinfo.modeinfo[0].standard;
	if (vinfo.modeinfo[0].iscustommode) {
		t.width = vinfo.modeinfo[0].framewidth;
		t.height = vinfo.modeinfo[0].frameheight;
		t.scanformat = vinfo.modeinfo[0].scanformat;
	} else {
		dc_get_format_from_mid(t.mode,
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

	return r;

}

static ssize_t blender_timings_store(struct dc_blender_info *binfo,
		const char *buf, size_t size)
{
	return size;
}

static ssize_t blender_enabled_show(struct dc_blender_info *binfo, char *buf)
{
	int r;
	struct vps_dcvencinfo vinfo;

	dc_lock(binfo->dctrl);

	vinfo.numvencs = 1;
	vinfo.modeinfo[0].vencid = venc_name[binfo->idx].vid;
	r = dc_get_vencinfo(&vinfo);

	if (r) {
		VPSSERR(" Failed to get venc infor\n");
		r = -EINVAL;
		goto exit;
	}

	r = snprintf(buf, PAGE_SIZE, "%d\n", vinfo.modeinfo[0].isvencrunning);

exit:
	dc_unlock(binfo->dctrl);
	return r;
}

static ssize_t blender_enabled_store(struct dc_blender_info *binfo,
				     const char *buf,
				     size_t size)
{
	int enabled;
	int vid;
	int r = 0;

	enabled = simple_strtoul(buf, NULL, 10);

	dc_lock(disp_ctrl);
	/*get vid id*/
	vid = venc_name[binfo->idx].vid;

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

static ssize_t blender_clksrc_show(struct dc_blender_info *binfo, char *buf)
{
	int r = 0;
	struct vps_dcvencclksrc clksrc;

	dc_lock(binfo->dctrl);
	clksrc.venc = venc_name[binfo->idx].vid;
	r = dc_get_clksrc(&clksrc);
	dc_unlock(binfo->dctrl);

	return snprintf(buf, PAGE_SIZE, "%s\n",
			vclksrc_name[clksrc.clksrc].name);

}

static ssize_t blender_clksrc_store(struct dc_blender_info *binfo,
				     const char *buf,
				     size_t size)
{
	int r = 0, i;
	struct vps_dcvencclksrc clksrc;
	char *input = (char *)buf;
	bool found = false;

	input = strsep(&input, "\n");
	dc_lock(binfo->dctrl);
	clksrc.venc = venc_name[binfo->idx].vid;

	if (dc_isvencrunning(clksrc.venc)) {
		VPSSERR("please stop venc before changing clock source\n");
		r = -EINVAL;
		goto exit;
	}
	/*found the matching clock source*/
	for (i = 0; i < ARRAY_SIZE(vclksrc_name); i++) {
		if (!strcmp(input, vclksrc_name[i].name)) {
			clksrc.clksrc = vclksrc_name[i].value;
			found = true;
			break;
		}
	}
	/*set the clock source*/
	if (found == true) {
		r = dc_set_clksrc(&clksrc);
		if (!r) {
			r = size;
			/*store back*/
			binfo->clksrc.clksrc = clksrc.clksrc;
		}
	} else {
		r = -EINVAL;
		VPSSERR("invalid clock source input\n");
	}

exit:
	dc_unlock(binfo->dctrl);
	return r;
}

static ssize_t blender_output_show(struct dc_blender_info *binfo, char *buf)
{
	struct vps_dcoutputinfo oinfo;
	int r = 0;
	int l = 0, i;

	oinfo.vencnodenum = venc_name[binfo->idx].vid;
	dc_lock(binfo->dctrl);
	r = dc_get_output(&oinfo);
	dc_unlock(binfo->dctrl);
	if (r)
		return -EINVAL;

	if (isdigitalvenc(oinfo.vencnodenum))
		l += snprintf(buf + l,
			      PAGE_SIZE - l, "%s",
			      dfmt_name[oinfo.dvofmt].name);
	else
		l += snprintf(buf + l,
			      PAGE_SIZE - l, "%s",
			      afmt_name[oinfo.afmt].name);

	for (i = 0 ; i < ARRAY_SIZE(datafmt_name); i++) {
		if (datafmt_name[i].value == oinfo.dataformat)
			l += snprintf(buf + l,
				      PAGE_SIZE - l, ",%s\n",
				      datafmt_name[i].name);
	}

	return l;

}

static ssize_t blender_output_store(struct dc_blender_info *binfo,
				     const char *buf,
				     size_t size)
{
	struct vps_dcoutputinfo oinfo;
	int r = 0;
	char *input = (char *)buf;
	char *ptr;
	enum vps_dcdigitalfmt dfmt = VPS_DC_DVOFMT_MAX;
	enum vps_dcanalogfmt afmt = VPS_DC_A_OUTPUT_MAX;
	enum fvid2_dataformat fmt = FVID2_DF_MAX;

	oinfo.vencnodenum = venc_name[binfo->idx].vid;

	dc_lock(binfo->dctrl);

	/*venc should be off before changed output*/
	if (dc_isvencrunning(oinfo.vencnodenum)) {
		VPSSERR("please disable VENC before changing output\n");
		r = -EINVAL;
		goto exit;
	}

	dc_get_output(&oinfo);


	/*remove the return*/
	input = strsep(&input, "\n");
	/*process the input buf*/
	while ((ptr = strsep(&input, ",")) != NULL) {
		int i;
		bool found = false;
		/*check data format first*/
		for (i = 0; i < ARRAY_SIZE(datafmt_name); i++) {
			if (!strcmp(ptr, datafmt_name[i].name))
				fmt = datafmt_name[i].value;
		}
		/*check digital format or analog format based on current venc*/
		if (fmt == FVID2_DF_MAX) {
			if (isdigitalvenc(oinfo.vencnodenum)) {
				for (i = 0; i < VPS_DC_DVOFMT_MAX; i++)
					if (!strcmp(ptr, dfmt_name[i].name)) {
						dfmt = dfmt_name[i].value;
						found = true;
						break;
					}
			} else {
				for (i = 0; i < VPS_DC_A_OUTPUT_MAX; i++)
					if (!strcmp(ptr, afmt_name[i].name)) {
						afmt = afmt_name[i].value;
						found = true;
						break;
					}
			}

			if (found == false) {
				VPSSERR("invalid output value %s\n", ptr);
				r = -EINVAL;
				goto exit;
			}

		}

		if (input == NULL)
			break;
	}

	/*make sure the input is right  before send out to M3*/
	if (isdigitalvenc(oinfo.vencnodenum)) {
		if ((dfmt == VPS_DC_DVOFMT_MAX) && (fmt == FVID2_DF_MAX)) {
			VPSSERR("no valid digital output settings\n");
			r = -EINVAL;
			goto exit;
		}
		if (dfmt != VPS_DC_DVOFMT_MAX)
			oinfo.dvofmt = dfmt;

	} else {
		if ((afmt == VPS_DC_A_OUTPUT_MAX) && (fmt == FVID2_DF_MAX)) {
			VPSSERR("no valid analog output settings\n");
			r = -EINVAL;
			goto exit;

		}
		if ((binfo->idx == SDVENC) &&
		     (afmt == VPS_DC_A_OUTPUT_COMPONENT)) {
			VPSSERR("component out not supported on sdvenc\n");
			r = -EINVAL;
			goto exit;

		}
		if (afmt != VPS_DC_A_OUTPUT_MAX)
			oinfo.afmt = afmt;
	}

	if (fmt != FVID2_DF_MAX)
		oinfo.dataformat = fmt;

	r = dc_set_output(&oinfo);
	if (!r)
		r = size;
exit:
	dc_unlock(binfo->dctrl);
	return r;

}
struct blender_attribute {
	struct attribute attr;
	ssize_t (*show)(struct dc_blender_info *, char *);
	ssize_t (*store)(struct dc_blender_info *, const char *, size_t);
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
static BLENDER_ATTR(output, S_IRUGO | S_IWUSR,
				blender_output_show, blender_output_store);
static BLENDER_ATTR(clksrc, S_IRUGO | S_IWUSR,
		blender_clksrc_show, blender_clksrc_store);

static struct attribute *blender_sysfs_attrs[] = {
	&blender_attr_mode.attr,
	&blender_attr_timing.attr,
	&blender_attr_enabled.attr,
	&blender_attr_output.attr,
	&blender_attr_clksrc.attr,
	NULL
};

static ssize_t blender_attr_show(struct kobject *kobj,
				  struct attribute *attr,
				  char *buf)
{
	struct dc_blender_info *binfo = NULL;
	struct blender_attribute *blend_attr = NULL;

	binfo = container_of(kobj, struct dc_blender_info, kobj);

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
	struct dc_blender_info *blend;
	struct blender_attribute *blend_attr;

	blend = container_of(kobj, struct dc_blender_info, kobj);
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

static ssize_t dctrl_pllclks_show(struct vps_dispctrl *dctrl, char *buf)
{
	int r = 0, l = 0, i = 0;
	struct vps_systemvpllclk  pllclk;

	for (i = 0; i < VPS_SYSTEM_VPLL_OUTPUT_MAX_VENC; i++) {
		pllclk.outputvenc = (enum vps_vplloutputclk)i;
		r = vps_system_getpll(&pllclk);
		if (r)
			return -EINVAL;
		l += snprintf(buf + l,
			      PAGE_SIZE - l,
			      "%s:%s",
			      pllvenc_name[i].name,
			      pllclk_name[pllclk.outputclk].name);
		if (i != VPS_SYSTEM_VPLL_OUTPUT_MAX_VENC - 1)
			l += snprintf(buf + l,
				      PAGE_SIZE - l,
				      ",");
	}
	l += snprintf(buf + l,
		      PAGE_SIZE - l,
		      "\n");

	return l;
}

static ssize_t dctrl_pllclks_store(struct vps_dispctrl *dctrl,
				   const char *buf,
				   size_t size)
{
	struct vps_systemvpllclk pllclk;
	char *input = (char *)buf, *this_opt;
	int r = 0;
	if (dctrl->automode) {
		VPSSERR("please turn off automode first\n");
		return -EINVAL;
	}

	input = strsep(&input, "\n");
	dc_lock(dctrl);
	while (!r && (this_opt = strsep(&input, ",")) != NULL) {
		char *p, *venc_str, *clk_str;
		int i;
		p = strchr(this_opt, ':');
		if (!p)
			break;

		*p = 0;
		venc_str = this_opt;
		clk_str = p + 1;
		r = -EINVAL;
		pllclk.outputvenc = VPS_SYSTEM_VPLL_OUTPUT_MAX_VENC;
		pllclk.outputclk = VPS_SYSTEM_VPLL_MAX_FREQ;

		/*get the output venc*/
		for (i = 0; i < VPS_SYSTEM_VPLL_OUTPUT_MAX_VENC; i++) {
			if (!strcmp(venc_str, pllvenc_name[i].name))  {
				pllclk.outputvenc = pllvenc_name[i].value;
				r = 0;
				break;
			}
		}
		/*get the pll clk*/
		if (!r) {
			for (i = 0; i < VPS_SYSTEM_VPLL_MAX_FREQ; i++) {
				if (!strcmp(clk_str, pllclk_name[i].name)) {
					pllclk.outputclk =
						pllclk_name[i].value;
					break;
				}
			}
		}

		r = vps_system_setpll(&pllclk);
		if (r)
			VPSSERR("set freq %s for %s failed\n",
				clk_str, venc_str);

		if (input == NULL)
			break;
	}
	if (!r)
		r = size;

	dc_unlock(dctrl);
	return r;
}

static ssize_t dctrl_automode_show(struct vps_dispctrl *dctrl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", dctrl->automode);
}

static ssize_t dctrl_automode_store(struct vps_dispctrl *dctrl,
				    const char *buf,
				    size_t size)
{
	int enabled;
	enabled = simple_strtoul(buf, NULL, 10);

	dctrl->automode = (bool)enabled;
	return size;
}

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
static DCTRL_ATTR(pllclks, S_IRUGO | S_IWUSR,
		dctrl_pllclks_show, dctrl_pllclks_store);
static DCTRL_ATTR(automode, S_IRUGO | S_IWUSR,
		dctrl_automode_show, dctrl_automode_store);

static struct attribute *dctrl_sysfs_attrs[] = {
	&dctrl_attr_tiedvencs.attr,
	&dctrl_attr_pllclks.attr,
	&dctrl_attr_automode.attr,
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
static int parse_def_clksrc(const char *clksrc)
{
	int r = 0, i;
	char *str, *options, *this_opt;

	if (clksrc == NULL)
		return 0;

	str = kmalloc(strlen(clksrc) + 1, GFP_KERNEL);
	strcpy(str, clksrc);
	options = str;
	VPSSDBG("clksrc %s\n", clksrc);
	while (!r && (this_opt = strsep(&options, ",")) != NULL) {
		char *p, *venc, *csrc;
		int vid, idx;
		struct dc_blender_info *binfo;

		p = strchr(this_opt, ':');
		if (!p)
			break;

		*p = 0;
		venc = this_opt;
		csrc = p + 1;
		/*parse the clock source for each possible venc input*/
		for (i = 0; i < ARRAY_SIZE(vclksrc_name); i++) {
			if (!strcmp(csrc, vclksrc_name[i].name)) {
				if (dc_get_vencid(venc, &vid)) {
					VPSSERR("wrong venc\n");
					break;
				}
				/* no clock for SD VENC*/
				if (vid == VPS_DC_VENC_SD)
					break;

				get_idx_from_vid(vid, &idx);
				binfo = &disp_ctrl->blenders[idx];
				/*is valid clock source*/
				if (isvalidclksrc(vid,
				      vclksrc_name[i].value)) {
					binfo->clksrc.clksrc =
							vclksrc_name[i].value;

				} else
					VPSSERR("wrong clock source\n");

				break;

			}
		}
		if (i == ARRAY_SIZE(vclksrc_name))
			VPSSERR("wrong clock source\n");

		if (options == NULL)
			break;


	}
	kfree(str);
	return r;
}

static int parse_def_modes(const char *mode)

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
			VPSSERR("venc name(%s) not existing.\n",
				display_str);
			continue;
		}
		if (dc_get_modeid(mode_str, &mid)) {
			VPSSERR("venc mode(%s) is not supported.\n",
				mode_str);
			continue;
		}

		if (!isvalidmode(vid, mid))
			continue;

		get_idx_from_vid(vid, &idx);
		vinfo->modeinfo[idx].vencid = vid;
		vinfo->modeinfo[idx].standard = mid;
		vinfo->modeinfo[idx].iscustommode = 0;
		dc_get_format_from_mid(mid,
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
	size += sizeof(struct vps_dcoutputinfo);
	size += sizeof(struct vps_dcvencclksrc);
	size += sizeof(u32);  /*this is for the disable venc command*/
	/*FIXME add more here*/

	return size;
}

static inline void assign_payload_addr(struct vps_dispctrl *dctrl,
				       struct vps_payload_info *pinfo,
				       u32 *buf_offset)
{
	int offset = *buf_offset;

	/*dc config */
	dctrl->dccfg = (struct vps_dcconfig *)setaddr(pinfo,
					      &offset,
					      &dctrl->dccfg_phy,
					      sizeof(struct vps_dcconfig));

	/* venc info*/
	dctrl->vinfo = (struct vps_dcvencinfo *)setaddr(pinfo,
						&offset,
						&dctrl->vinfo_phy,
						sizeof(struct vps_dcvencinfo));

	/*node input*/
	dctrl->nodeinfo = (struct vps_dcnodeinput *)setaddr(
					pinfo,
					&offset,
					&dctrl->ninfo_phy,
					sizeof(struct vps_dcnodeinput));
	/*venc disable*/
	dctrl->dis_vencs = (u32 *)setaddr(pinfo,
				  &offset,
				  &dctrl->dis_vencsphy,
				  sizeof(u32));

	/*venc output infor*/
	dctrl->opinfo = (struct vps_dcoutputinfo *)setaddr(
					pinfo,
					&offset,
					&dctrl->opinfo_phy,
					sizeof(struct vps_dcoutputinfo));

	/*venc clock source*/
	dctrl->clksrc = (struct vps_dcvencclksrc *)setaddr(
					pinfo,
					&offset,
					&dctrl->clksrc_phy,
					sizeof(struct vps_dcvencclksrc));

	*buf_offset = offset;
}

int __init vps_dc_init(struct platform_device *pdev,
			  const char *mode,
			  int tied_vencs,
			  const char *clksrc)
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


	/*allocate display_control memory*/
	disp_ctrl = kzalloc(sizeof(struct vps_dispctrl), GFP_KERNEL);
	if (disp_ctrl == NULL) {
		r = -ENOMEM;
		goto cleanup;
	}
	disp_ctrl->fvid2_handle = dc_handle;
	disp_ctrl->automode = true;
	assign_payload_addr(disp_ctrl, dc_payload_info, &offset);

	mutex_init(&disp_ctrl->dcmutex);

	r = kobject_init_and_add(
			&disp_ctrl->kobj,
			&dctrl_ktype,
			&pdev->dev.kobj,
			"system");
	if (r)
		VPSSERR("failed to create dctrl sysfs file.\n");

	/*create sysfs*/
	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		struct dc_blender_info *blend = &disp_ctrl->blenders[i];;

		blend->idx = i;
		blend->actnodes = 0;
		blend->name = venc_name[i].name;
		blend->dctrl = disp_ctrl;
		blend->tinfo = NULL;
		if (i != SDVENC) {
			blend->clksrc.venc = venc_name[i].vid;
			dc_get_clksrc(&blend->clksrc);
		}
		r = kobject_init_and_add(
			&blend->kobj, &blender_ktype,
			&pdev->dev.kobj, venc_name[i].name);

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

	r = parse_def_clksrc(clksrc);
	if (r) {
		VPSSERR("failed to parse clock source\n");
		goto cleanup;
	}

	/*set the clock source*/
	if (clksrc) {
		for (i = 0; i < VPS_DC_MAX_VENC - 1; i++)
			r = dc_set_clksrc(&disp_ctrl->blenders[i].clksrc);

		if (r) {
			VPSSERR("failed to set clock resource");
			goto cleanup;
		}
	}
	/*config the PLL*/
	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		r = dc_set_pll(i, venc_info.modeinfo[i].standard);
		if (r) {
			VPSSERR("failed to set pll");
			goto cleanup;
		}

	}

	/*set the venc output*/
	for (i = 0; i < VPS_DC_MAX_VENC; i++) {
		switch (i) {
		case HDMI:
			disp_ctrl->opinfo->vencnodenum = VPS_DC_VENC_HDMI;
			disp_ctrl->opinfo->dvofmt =
				VPS_DC_DVOFMT_TRIPLECHAN_DISCSYNC;
			disp_ctrl->opinfo->dataformat = FVID2_DF_RGB24_888;
			break;
#ifdef CONFIG_ARCH_TI816X
		case HDCOMP:
			disp_ctrl->opinfo->vencnodenum = VPS_DC_VENC_HDCOMP;
			disp_ctrl->opinfo->afmt = VPS_DC_A_OUTPUT_COMPONENT;
			disp_ctrl->opinfo->dataformat = FVID2_DF_YUV422SP_UV;
			break;
#endif
		case DVO2:
			disp_ctrl->opinfo->vencnodenum = VPS_DC_VENC_DVO2;
			disp_ctrl->opinfo->dvofmt = VPS_DC_DVOFMT_DOUBLECHAN;
			disp_ctrl->opinfo->dataformat = FVID2_DF_YUV422SP_UV;

			break;
		case SDVENC:
			disp_ctrl->opinfo->vencnodenum = VPS_DC_VENC_SD;
			disp_ctrl->opinfo->afmt = VPS_DC_A_OUTPUT_COMPOSITE;
			disp_ctrl->opinfo->dataformat = FVID2_DF_YUV422SP_UV;
			break;

		}
		r = dc_set_output(disp_ctrl->opinfo);
		if (r) {
			VPSSERR("failed to set venc output\n");
			goto cleanup;
		}
	}
	/*set the venc mode*/
	r = dc_set_vencmode(&venc_info);
	if (r) {
		VPSSERR("Failed to set venc mode.\n");
		goto cleanup;
	}
	/*set the the THS filter*/
#ifdef CONFIG_ARCH_TI816X
	r = pcf8575_ths7375_enable(TI816X_THSFILTER_ENABLE_MODULE);
	if (venc_info.modeinfo[HDCOMP].standard == FVID2_STD_1080P_60)
		r |= pcf8575_ths7360_hd_enable(
			TI816X_THS7360_SF_TRUE_HD_MODE);
	else
		r |= pcf8575_ths7360_hd_enable(TI816X_THS7360_SF_HD_MODE);
#endif

	r |= pcf8575_ths7360_sd_enable(TI816X_THSFILTER_ENABLE_MODULE);

	if (r < 0) {
		VPSSERR("failed to setup THS filter.\n");
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

