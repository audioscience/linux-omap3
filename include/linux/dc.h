/*
 *
 * Display Controller Internal Header file for TI 816X VPSS
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

#ifndef __LINUX_DC_H__
#define __LINUX_DC_H__

#ifdef __KERNEL__


#define VPS_DC_MAX_NODE_NUM   26
#define HDMI 0
#define HDCOMP 1
#define DVO2 2
#define SDVENC 3
enum dc_idtype {
	DC_BLEND_ID = 0,
	DC_VENC_ID,
	DC_MODE_ID
};

struct dcnode_info {
	char *name;
	int id;
 };

struct venc_modeinfo {
	char *name;
	u16 width;
	u16 height;
	u8 scformat;
	enum vps_dcmodeid  mid;
};

struct venc_name_id {
	char *name;
	int  vid;
	int  blendid;
	/*0:hdmi, 1: HDCOMP, 2: DVO2, 3:SD*/
	int  idx;
};


struct dc_blenderinfo {
	char *name;
	u32   idx;
	bool  enabled;
	struct kobject  kobj;
	u32          actnodes;
	struct vps_dctiminginfo  *tinfo;
	struct vps_dispctrl *dcctrl;
};

struct vps_dispctrl {
	struct mutex  dcmutex;
	struct dc_blenderinfo  blenders[VPS_DC_MAX_VENC];
	enum vps_dcdvo2clksrc dvo2clksrc;
	enum vps_dchdcompclksrc hdcompclksrc;
	int tiedvenc;
	struct vps_dcconfig *dccfg;
	u32 dccfg_phy;
	struct vps_dcvencinfo *vinfo;
	u32 vinfo_phy;
	struct vps_dcnodeinput *nodeinfo;
	u32 ninfo_phy;
	int enabled_venc_ids;
	void   *fvid2_handle;
};

static inline void dc_lock(struct vps_dispctrl *dctrl)
{
  mutex_lock(&dctrl->dcmutex);
}

static inline void dc_unlock(struct vps_dispctrl *dctrl)
{
    mutex_unlock(&dctrl->dcmutex);
}

int vps_dc_init(struct platform_device *pdev);

int vps_dc_exit(struct platform_device *pdev);

int vps_dc_get_node_id(int *id, char *name);
int vps_dc_get_node_name(int id, char *name);

int vps_dc_get_clksrc(enum vps_dcdvo2clksrc *dvo2,
		      enum vps_dchdcompclksrc *hdcomp);


int vps_dc_get_vencmode(int id, struct vps_dcmodeinfo *modeinfo,
			enum dc_idtype type);
int vps_dc_set_vencmode(int *vid, int *mid,
    void *modeinfo, int numvenc, int tiedvenc);
int vps_dc_set_config(struct vps_dcconfig *usercfg, int setflag);
int vps_dc_get_outpfmt(int id, u32 *width, u32 *height,
		      u8 *scformat, enum dc_idtype type);

int vps_dc_set_node(u8 nodeid, u8 inputid, u8 enable);

int vps_dc_get_vencid(char *vname, int *vid);

int vps_dc_get_modeid(char *mname, int *mid);


#endif
#endif
