/*
 *
 * Graphics internal header file for TI 816X VPSS
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

#ifndef __LINUX_GRPX_H__
#define __LINUX_GRPX_H__

#ifdef __KERNEL__

/* 2 extra lines output from scaler*/
#define GRPX_SCALED_REGION_EXTRA_LINES      0x02
/* 2 extra pixel output from scaler*/
#define GRPX_SCALED_REGION_EXTRA_PIXES      0x02
/* one line gap for the up scaled except first region*/
#define GRPX_REGION_UP_SCALED_GAP           0x01
/* two lines gap for the downscaled except first region*/
#define GRPX_REGION_DOWN_SCALED_GAP         0x02


struct vps_grpx_state {
	bool                            isfmtset;
	bool                            isparamset;
	bool                            isstarted;
	bool                            isdcConfig;
	bool                            clutSet;
	bool                            scset;
	bool                            stenset;
	bool                            regset;
	bool                            varset;
};

/*vsync call back for the application*/
typedef void (*vsync_callback_t)(void *arg);

struct vps_grpx_ctrl{
	struct kobject                  kobj;
	u32                             grpx_num;
	struct mutex                    gmutex;
	struct list_head                list;
	/*fvid2 control handle*/
	void                            *handle;
	struct vps_grpx_state           gstate;
	/*create params*/
	struct vps_grpxcreateparams     *gcparam;
	u32                             gcp_phy;
	/*create status*/
	struct vps_grpxcreatestatus     *gcstatus;
	u32                             gcs_phy;
	/* scaling parameters*/
	struct vps_grpxscparams         *gscparams;
	u32                             gscp_phy;
	struct vps_grpxsccoeff          *gsccoeff;
	u32                             gscoff_phy;
	/*runtime change parameters*/
	struct vps_grpxrtparams         *grtconfig;
	u32                             grtc_phy;
	/*default params*/
	struct vps_grpxrtparams         *gparams;
	u32                             gp_phy;
	/* runtime undate list*/
	struct vps_grpxrtlist           *grtlist;
	u32				grtlist_phy;
	/*parameter list*/
	struct vps_grpxparamlist        *glist;
	u32                             glist_phy;
	/*last buffer address*/
	u32                             buffer_addr;
	/* The followings  are the FVID2 varables*/
	/*fvid2 create params*/
	struct fvid2_cbparams           *cbparams;
	u32                             cbp_phy;
	/* set format params*/
	struct fvid2_format             *inputf;
	u32                             inputf_phy;
	/*frame list*/
	struct fvid2_framelist          *framelist;
	u32                             frmls_phy;
	/*frames*/
	struct fvid2_frame              *frames;
	u32                             frm_phy;

	vsync_callback_t                vsync_cb;
	void                            *vcb_arg;
	/*output format*/
	u32                             framewidth;
	u32                             frameheight;
	u8                              scformat;

	/*display controller settings*/
	struct vps_dcconfig             dccfg;
	int                             numends;
	int                             snode;
	int                             enodes[VPS_DC_MAX_VENC];

	/*function pointer*/
	int (*apply_changes)(struct vps_grpx_ctrl *gctrl);
	int (*set_input)(struct vps_grpx_ctrl *gctrl,
		u32 width, u32 height, u8 scfmt);
	int (*set_buffer)(struct vps_grpx_ctrl *gctrl,
		u32 buffer_paddr);
	int (*get_resolution)(struct vps_grpx_ctrl *gctrl,
		u32 *fwidth, u32 *fheight, u8 *scfmt);
	int (*set_format)(struct vps_grpx_ctrl *gctrl,
			u8 bpp, u32 df, u32 pitch);
	int (*check_params)(struct vps_grpx_ctrl *gctrl,
		      struct vps_grpxregionparams *regp, u16 ridx);
	int (*set_clutptr)(struct vps_grpx_ctrl *gctrl, u32 ptr);

	int (*set_scparams)(struct vps_grpx_ctrl *gctrl,
			  struct vps_grpxscparams *sci);
	int (*get_scparams)(struct vps_grpx_ctrl *gctrl,
			  struct vps_grpxscparams *sci);
	int (*set_regparams)(struct vps_grpx_ctrl *gctrl,
			   struct vps_grpxregionparams *gparams);
	int (*get_regparams)(struct vps_grpx_ctrl *gctrl,
			   struct vps_grpxregionparams *gparams);

	int (*set_stenparams)(struct vps_grpx_ctrl *gctrl,
			 u32 stenptr, u32 pitch);

	int (*get_stenparams)(struct vps_grpx_ctrl *gctrl,
			 u32 *ptr, u32 *pitch);
	int (*create)(struct vps_grpx_ctrl *gctrl);
	int (*delete)(struct vps_grpx_ctrl *gctrl);

	int (*register_vsync_cb)(struct vps_grpx_ctrl *gctrl,
			       vsync_callback_t vcb, void *arg);
	int (*unregister_vsync_cb)(struct vps_grpx_ctrl *gctrl);

	int (*start)(struct vps_grpx_ctrl *gctrl);
	int (*stop)(struct vps_grpx_ctrl *gctrl);

	int (*create_dcconfig)(struct vps_grpx_ctrl *gctrl);
	int (*set_dcconfig)(struct vps_grpx_ctrl *gctrl, u8 setflag);
};

static inline void grpx_lock(struct vps_grpx_ctrl *gctrl)
{
	mutex_lock(&gctrl->gmutex);
}

static inline void grpx_unlock(struct vps_grpx_ctrl *gctrl)
{
	mutex_unlock(&gctrl->gmutex);
}

int vps_grpx_get_num_grpx(void);
struct vps_grpx_ctrl *vps_grpx_get_ctrl(int num);

#endif
#endif
