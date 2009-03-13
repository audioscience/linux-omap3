/*
 * linux/arch/arm/plat-omap/dss/display.c
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DSS_SUBSYS_NAME "DISPLAY"

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/clk.h>

#include <mach/display.h>
#include <mach/clock.h>
#include "dss.h"

#define DSS_MAX_DISPLAYS                8

static int num_displays;
static struct omap_display displays[DSS_MAX_DISPLAYS];

static ssize_t show_clk(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t l, size = PAGE_SIZE;

	l = 0;

	l += dss_print_clocks(buf + l, size - l);

	l += dispc_print_clocks(buf + l, size - l);
#ifdef CONFIG_OMAP2_DSS_DSI
	l += dsi_print_clocks(buf + l, size - l);
#endif
	return l;
}

static DEVICE_ATTR(clk, S_IRUGO, show_clk, NULL);

int initialize_sysfs(struct device *dev)
{
	int r;

	r = device_create_file(dev, &dev_attr_clk);
	if (r)
		DSSERR("failed to create sysfs clk file\n");

	return r;
}

void uninitialize_sysfs(struct device *dev)
{
	device_remove_file(dev, &dev_attr_clk);
}

static void default_get_resolution(struct omap_display *display,
			int *xres, int *yres)
{
	*xres = display->panel->timings.x_res;
	*yres = display->panel->timings.y_res;
}

void initialize_displays(struct omap_dss_platform_data *pdata)
{
	int i;

	num_displays = 0;

	BUG_ON(pdata->num_displays > DSS_MAX_DISPLAYS);

	for (i = 0; i < pdata->num_displays; ++i) {
		struct omap_display *display = &displays[i];

		/*atomic_set(&display->ref_count, 0);*/
		display->ref_count = 0;

		display->hw_config = *pdata->displays[i];
		display->type = pdata->displays[i]->type;
		display->name = pdata->displays[i]->name;

		display->get_resolution = default_get_resolution;

		switch (display->type) {

		case OMAP_DISPLAY_TYPE_DPI:
			dpi_init_display(display);
			break;
#ifdef CONFIG_OMAP2_DSS_RFBI
		case OMAP_DISPLAY_TYPE_DBI:
			rfbi_init_display(display);
			break;
#endif
#ifdef CONFIG_OMAP2_DSS_VENC
		case OMAP_DISPLAY_TYPE_VENC:
			venc_init_display(display);
			break;
#endif
#ifdef CONFIG_OMAP2_DSS_SDI
		case OMAP_DISPLAY_TYPE_SDI:
			sdi_init_display(display);
			break;
#endif
#ifdef CONFIG_OMAP2_DSS_DSI
		case OMAP_DISPLAY_TYPE_DSI:
			dsi_init_display(display);
			break;
#endif

		default:
			DSSERR("Support for display '%s' not compiled in.\n",
					display->name);
			continue;
		}

		num_displays++;
	}
}

static int check_overlay(struct omap_overlay *ovl,
		struct omap_display *display)
{
	struct omap_overlay_info *info;
	int outw, outh;
	int dw, dh;

	if (!display)
		return 0;

	if (!ovl->info.enabled)
		return 0;

	info = &ovl->info;

	if ((ovl->caps & OMAP_DSS_OVL_CAP_SCALE) == 0) {
		outw = info->width;
		outh = info->height;
	} else {
		if (info->out_width == 0)
			outw = info->width;
		else
			outw = info->out_width;

		if (info->out_height == 0)
			outh = info->height;
		else
			outh = info->out_height;
	}

	display->get_resolution(display, &dw, &dh);

	if (dw < info->pos_x + outw) {
		DSSDBG("check_overlay failed 1\n");
		return -EINVAL;
	}

	if (dh < info->pos_y + outh) {
		DSSDBG("check_overlay failed 2\n");
		return -EINVAL;
	}

	return 0;
}

static int omap_dss_set_manager(struct omap_overlay *ovl,
		struct omap_overlay_manager *mgr)
{
	int r;

	if (ovl->manager) {
		DSSERR("overlay '%s' already has a manager '%s'\n",
				ovl->name, ovl->manager->name);
	}

	r = check_overlay(ovl, mgr->display);
	if (r)
		return r;

	ovl->manager = mgr;

	return 0;
}

static int omap_dss_unset_manager(struct omap_overlay *ovl)
{
	if (!ovl->manager) {
		DSSERR("failed to detach overlay: manager not set\n");
		return -EINVAL;
	}

	ovl->manager = NULL;

	return 0;
}

static int omap_dss_set_display(struct omap_overlay_manager *mgr,
		struct omap_display *display)
{
	int i;
	int r;

	if (display->manager) {
		DSSERR("display '%s' already has a manager '%s'\n",
			       display->name, display->manager->name);
		return -EINVAL;
	}

	if ((mgr->supported_displays & display->type) == 0) {
		DSSERR("display '%s' does not support manager '%s'\n",
			       display->name, mgr->name);
		return -EINVAL;
	}

	for (i = 0; i < mgr->num_overlays; i++) {
		struct omap_overlay *ovl = &mgr->overlays[i];

		if (ovl->manager != mgr || !ovl->info.enabled)
			continue;

		r = check_overlay(ovl, display);
		if (r)
			return r;
	}

	display->manager = mgr;
	mgr->display = display;

	return 0;
}

static int omap_dss_unset_display(struct omap_overlay_manager *mgr)
{
	if (!mgr->display) {
		DSSERR("failed to unset display, display not set.\n");
		return -EINVAL;
	}

	mgr->display->manager = NULL;
	mgr->display = NULL;

	return 0;
}

static int omap_dss_setup_overlay_input(struct omap_overlay *ovl,
		u32 paddr, void *vaddr, int tv_field1_offset, int screen_width,
		int width, int height,
		enum omap_color_mode color_mode, int rotation, int mirror,
		int global_alpha)
{
	int r;
	struct omap_overlay_info old_info;

	if ((ovl->supported_modes & color_mode) == 0) {
		DSSERR("overlay doesn't support mode %d\n", color_mode);
		return -EINVAL;
	}

	old_info = ovl->info;

	ovl->info.paddr = paddr;
	ovl->info.vaddr = vaddr;
	ovl->info.screen_width = screen_width;
	ovl->info.rotation = rotation;
	ovl->info.mirror = mirror;
	ovl->info.tv_field1_offset = tv_field1_offset;
	ovl->info.width = width;
	ovl->info.height = height;
	ovl->info.color_mode = color_mode;
	ovl->info.global_alpha = global_alpha;

	if (ovl->manager) {
		r = check_overlay(ovl, ovl->manager->display);
		if (r) {
			ovl->info = old_info;
			return r;
		}
	}

	return 0;
}

static int omap_dss_setup_overlay_output(struct omap_overlay *ovl,
		int pos_x, int pos_y,
		int out_width, int out_height)
{
	int r;
	struct omap_overlay_info old_info;

	old_info = ovl->info;

	ovl->info.pos_x = pos_x;
	ovl->info.pos_y = pos_y;
	ovl->info.out_width = out_width;
	ovl->info.out_height = out_height;

	if (ovl->manager) {
		r = check_overlay(ovl, ovl->manager->display);
		if (r) {
			ovl->info = old_info;
			return r;
		}
	}

	return 0;
}

static int omap_dss_enable_overlay(struct omap_overlay *ovl, int enable)
{
	struct omap_overlay_info old_info;
	int r;

	old_info = ovl->info;

	ovl->info.enabled = enable ? 1 : 0;

	if (ovl->manager) {
		r = check_overlay(ovl, ovl->manager->display);
		if (r) {
			ovl->info = old_info;
			return r;
		}
	}

	return 0;
}

static int omap_dss_mgr_apply(struct omap_overlay_manager *mgr)
{
	int i;
	int r = 0;

	DSSDBG("omap_dss_mgr_apply(%s)\n", mgr->name);

	if (!mgr->display) {
		DSSDBG("no display, aborting apply\n");
		return 0;
	}

	/* on a manual update display update() handles configuring
	 * planes */
	if (mgr->display->get_update_mode) {
		enum omap_dss_update_mode mode;
		mode = mgr->display->get_update_mode(mgr->display);
		if (mode == OMAP_DSS_UPDATE_MANUAL)
			return 0;
	}

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	for (i = 0; i < mgr->num_overlays; i++) {
		int ilace = 0;
		int outw, outh;

		struct omap_overlay *ovl = &mgr->overlays[i];

		if (!ovl->manager) {
			dispc_enable_plane(ovl->id, 0);
			continue;
		}

		if (ovl->manager != mgr)
			continue;

		if (!ovl->info.enabled) {
			dispc_enable_plane(ovl->id, 0);
			continue;
		}

		if (mgr->display->type == OMAP_DISPLAY_TYPE_VENC)
			ilace = 1;

		if (ovl->info.out_width == 0)
			outw = ovl->info.width;
		else
			outw = ovl->info.out_width;

		if (ovl->info.out_height == 0)
			outh = ovl->info.height;
		else
			outh = ovl->info.out_height;

		r = dispc_setup_plane(ovl->id, ovl->manager->id,
				ovl->info.paddr,
				ovl->info.tv_field1_offset,
				ovl->info.screen_width,
				ovl->info.pos_x,
				ovl->info.pos_y,
				ovl->info.width,
				ovl->info.height,
				outw,
				outh,
				ovl->info.color_mode,
				ilace,
				ovl->info.rotation,
				ovl->info.mirror,
				ovl->info.global_alpha);

		if (r) {
			DSSERR("dispc_setup_plane failed\n");
			goto exit;
		}
		dispc_enable_plane(ovl->id, 1);
	}

	dispc_go(mgr->id);

exit:
	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);

	return r;
}

static struct omap_overlay dispc_overlays[] = {
	{
		.name = "gfx",
		.id = OMAP_DSS_GFX,
		.set_manager = &omap_dss_set_manager,
		.unset_manager = &omap_dss_unset_manager,
		.setup_input = &omap_dss_setup_overlay_input,
		.setup_output = &omap_dss_setup_overlay_output,
		.enable = &omap_dss_enable_overlay,
		.supported_modes = OMAP_DSS_COLOR_GFX_OMAP3,
	},
	{
		.name = "vid1",
		.id = OMAP_DSS_VIDEO1,
		.set_manager = &omap_dss_set_manager,
		.unset_manager = &omap_dss_unset_manager,
		.setup_input = &omap_dss_setup_overlay_input,
		.setup_output = &omap_dss_setup_overlay_output,
		.enable = &omap_dss_enable_overlay,
		.supported_modes = OMAP_DSS_COLOR_VID_OMAP3,
		.caps = OMAP_DSS_OVL_CAP_SCALE,
	},
	{
		.name = "vid2",
		.id = OMAP_DSS_VIDEO2,
		.set_manager = &omap_dss_set_manager,
		.unset_manager = &omap_dss_unset_manager,
		.setup_input = &omap_dss_setup_overlay_input,
		.setup_output = &omap_dss_setup_overlay_output,
		.enable = &omap_dss_enable_overlay,
		.supported_modes = OMAP_DSS_COLOR_VID_OMAP3,
		.caps = OMAP_DSS_OVL_CAP_SCALE,
	},
};

static struct omap_overlay_manager dispc_overlay_managers[] =
{
	[OMAP_DSS_OVL_MGR_LCD] = {
		.name = "lcd",
		.id = OMAP_DSS_CHANNEL_LCD,
		.num_overlays = 3,
		.overlays = dispc_overlays,
		.set_display = &omap_dss_set_display,
		.unset_display = &omap_dss_unset_display,
		.apply = &omap_dss_mgr_apply,
		.caps = OMAP_DSS_OVL_MGR_CAP_DISPC,
		.supported_displays =
			OMAP_DISPLAY_TYPE_DPI | OMAP_DISPLAY_TYPE_DBI |
			OMAP_DISPLAY_TYPE_SDI | OMAP_DISPLAY_TYPE_DSI,
	},
	[OMAP_DSS_OVL_MGR_TV] = {
		.name = "tv",
		.id = OMAP_DSS_CHANNEL_DIGIT,
		.num_overlays = 3,
		.overlays = dispc_overlays,
		.set_display = &omap_dss_set_display,
		.unset_display = &omap_dss_unset_display,
		.apply = &omap_dss_mgr_apply,
		.caps = OMAP_DSS_OVL_MGR_CAP_DISPC,
		.supported_displays = OMAP_DISPLAY_TYPE_VENC,
	},
};

static int num_overlays = 3;

static struct omap_overlay *omap_dss_overlays[10] = {
	&dispc_overlays[0],
	&dispc_overlays[1],
	&dispc_overlays[2],
};

static int num_overlay_managers = 2;

static struct omap_overlay_manager *omap_dss_overlay_managers[10] = {
	&dispc_overlay_managers[0],
	&dispc_overlay_managers[1],
};

static void omap_dss_add_overlay(struct omap_overlay *overlay)
{
	int i = num_overlays++;

	omap_dss_overlays[i] = overlay;
}

static void omap_dss_add_overlay_manager(struct omap_overlay_manager *manager)
{
	int i = num_overlay_managers++;
	omap_dss_overlay_managers[i] = manager;
}

int omap_dss_get_num_overlays(void)
{
	return num_overlays;
}
EXPORT_SYMBOL(omap_dss_get_num_overlays);

struct omap_overlay *omap_dss_get_overlay(int num)
{
	BUG_ON(num >= num_overlays);
	return omap_dss_overlays[num];
}
EXPORT_SYMBOL(omap_dss_get_overlay);

int omap_dss_get_num_overlay_managers(void)
{
	return num_overlay_managers;
}
EXPORT_SYMBOL(omap_dss_get_num_overlay_managers);

struct omap_overlay_manager *omap_dss_get_overlay_manager(int num)
{
	BUG_ON(num >= num_overlay_managers);
	return omap_dss_overlay_managers[num];
}
EXPORT_SYMBOL(omap_dss_get_overlay_manager);

static int ovl_mgr_apply_l4(struct omap_overlay_manager *mgr)
{
	DSSDBG("omap_dss_mgr_apply_l4(%s)\n", mgr->name);

	return 0;
}

void initialize_overlays(const char *def_disp_name)
{
	int i;
	struct omap_overlay_manager *lcd_mgr;
	struct omap_overlay_manager *tv_mgr;
	struct omap_overlay_manager *def_mgr = NULL;
	struct omap_overlay *ovl;

	lcd_mgr = omap_dss_get_overlay_manager(OMAP_DSS_OVL_MGR_LCD);
	tv_mgr = omap_dss_get_overlay_manager(OMAP_DSS_OVL_MGR_TV);

	if (def_disp_name) {
		for (i = 0; i < num_displays; i++) {
			struct omap_display *display = &displays[i];

			if (strcmp(display->name, def_disp_name) == 0) {
				if (display->type != OMAP_DISPLAY_TYPE_VENC) {
					omap_dss_set_display(lcd_mgr, display);
					def_mgr = lcd_mgr;
				} else {
					omap_dss_set_display(tv_mgr, display);
					def_mgr = tv_mgr;
				}

				break;
			}
		}

		if (!def_mgr)
			DSSWARN("default display %s not found\n",
					def_disp_name);
	}

	if (def_mgr != lcd_mgr) {
		/* connect lcd manager to first non-VENC display found */
		for (i = 0; i < num_displays; i++) {
			struct omap_display *display = &displays[i];
			if (display->type != OMAP_DISPLAY_TYPE_VENC) {
				omap_dss_set_display(lcd_mgr, display);

				if (!def_mgr)
					def_mgr = lcd_mgr;

				break;
			}
		}
	}

	if (def_mgr != tv_mgr) {
		/* connect tv manager to first VENC display found */
		for (i = 0; i < num_displays; i++) {
			struct omap_display *display = &displays[i];
			if (display->type == OMAP_DISPLAY_TYPE_VENC) {
				omap_dss_set_display(tv_mgr, display);

				if (!def_mgr)
					def_mgr = tv_mgr;

				break;
			}
		}
	}

	/* connect all dispc overlays to def_mgr */
	if (def_mgr) {
		for (i = 0; i < 3; i++) {
			ovl = omap_dss_get_overlay(i);
			omap_dss_set_manager(ovl, def_mgr);
		}
	}

	/* Set the TV MGR as the default mgr for video based on config
	   option */
#ifdef CONFIG_VID1_TV_MANAGER
	ovl = omap_dss_get_overlay(1);
	omap_dss_unset_manager(ovl);
	omap_dss_set_manager(ovl, tv_mgr);
#endif
#ifdef CONFIG_VID2_TV_MANAGER
	ovl = omap_dss_get_overlay(2);
	omap_dss_unset_manager(ovl);
	omap_dss_set_manager(ovl, tv_mgr);
#endif
	/* setup L4 overlay as an example */
	{
		static struct omap_overlay ovl = {
			.name = "l4-ovl",
			.supported_modes = OMAP_DSS_COLOR_RGB24U,
			.set_manager = &omap_dss_set_manager,
			.unset_manager = &omap_dss_unset_manager,
			.setup_input = &omap_dss_setup_overlay_input,
			.setup_output = &omap_dss_setup_overlay_output,
			.enable = &omap_dss_enable_overlay,
		};

		static struct omap_overlay_manager mgr = {
			.name = "l4",
			.num_overlays = 1,
			.overlays = &ovl,
			.set_display = &omap_dss_set_display,
			.unset_display = &omap_dss_unset_display,
			.apply = &ovl_mgr_apply_l4,
			.supported_displays =
				OMAP_DISPLAY_TYPE_DBI | OMAP_DISPLAY_TYPE_DSI,
		};

		omap_dss_add_overlay(&ovl);
		omap_dss_add_overlay_manager(&mgr);
		omap_dss_set_manager(&ovl, &mgr);
	}

}

int omap_dss_suspend_all_displays(void)
{
	int i = 0, r;

	for (i = 0; i < num_displays; ++i) {
		struct omap_display *display = &displays[i];

		if (!display)
			continue;

		if (display->state != OMAP_DSS_DISPLAY_ACTIVE) {
			continue;
		}

		if (!display->suspend) {
			DSSERR("display '%s' doesn't implement suspend\n",
					display->name);
			r = -ENOSYS;
			goto err;
		}

		r = display->suspend(display);

		if (r)
			goto err;
	}

	return 0;
err:
	/* resume all displays that were suspended */
	omap_dss_resume_all_displays();
	return r;
}

int omap_dss_resume_all_displays(void)
{
	int i = 0, r;

	for (i = 0; i < num_displays; ++i) {
		struct omap_display *display = &displays[i];
		if (display && display->resume) {
			if (display->state != OMAP_DSS_DISPLAY_SUSPENDED)
				continue;

			r = display->resume(display);
			if (r)
				return r;
		}
	}

	return 0;
}

int omap_dss_get_num_displays(void)
{
	return num_displays;
}
EXPORT_SYMBOL(omap_dss_get_num_displays);

struct omap_display *omap_dss_get_display(int no)
{
	struct omap_display *display;

	if (no >= num_displays)
		return NULL;

	display = &displays[no];

	switch (display->type) {
	case OMAP_DISPLAY_TYPE_VENC:
		break;

	case OMAP_DISPLAY_TYPE_DPI:
	case OMAP_DISPLAY_TYPE_SDI:
		if (display->panel == NULL)
			return NULL;
		break;

	case OMAP_DISPLAY_TYPE_DBI:
	case OMAP_DISPLAY_TYPE_DSI:
		if (display->panel == NULL || display->ctrl == NULL)
			return NULL;
		break;

	default:
		return NULL;
	}

	if (display->panel) {
		if (!try_module_get(display->panel->owner))
			goto err0;

		if (display->panel->init)
			if (display->panel->init(display) != 0)
				goto err1;
	}

	if (display->ctrl) {
		if (!try_module_get(display->ctrl->owner))
			goto err2;

		if (display->ctrl->init)
			if (display->ctrl->init(display) != 0)
				goto err3;
	}

	return display;
err3:
	if (display->ctrl)
		module_put(display->ctrl->owner);
err2:
	if (display->panel && display->panel->init)
		display->panel->cleanup(display);
err1:
	if (display->panel)
		module_put(display->panel->owner);
err0:
	return NULL;
}
EXPORT_SYMBOL(omap_dss_get_display);

void omap_dss_put_display(struct omap_display *display)
{
	/*
	 * Please make sure that you call display->disable
	 * before calling this function.
	 */
	if (display->ref_count != 0)
		return;

	if (display->ctrl) {
		if (display->ctrl->cleanup)
			display->ctrl->cleanup(display);
		module_put(display->ctrl->owner);
	}

	if (display->panel) {
		if (display->panel->cleanup)
			display->panel->cleanup(display);
		module_put(display->panel->owner);
	}
}
EXPORT_SYMBOL(omap_dss_put_display);

void omap_dss_register_ctrl(struct omap_ctrl *ctrl)
{
	int i;

	for (i = 0; i < num_displays; i++) {
		struct omap_display *display = &displays[i];
		if (display->hw_config.ctrl_name &&
		    strcmp(display->hw_config.ctrl_name, ctrl->name) == 0) {
			display->ctrl = ctrl;
			DSSDBG("ctrl '%s' registered\n", ctrl->name);
		}
	}
}
EXPORT_SYMBOL(omap_dss_register_ctrl);

void omap_dss_register_panel(struct omap_panel *panel)
{
	int i;

	for (i = 0; i < num_displays; i++) {
		struct omap_display *display = &displays[i];
		if (display->hw_config.panel_name &&
		    strcmp(display->hw_config.panel_name, panel->name) == 0) {
			display->panel = panel;
			DSSDBG("panel '%s' registered\n", panel->name);
		}
	}
}
EXPORT_SYMBOL(omap_dss_register_panel);

void omap_dss_unregister_ctrl(struct omap_ctrl *ctrl)
{
	int i;

	for (i = 0; i < num_displays; i++) {
		struct omap_display *display = &displays[i];
		if (display->hw_config.ctrl_name &&
		    strcmp(display->hw_config.ctrl_name, ctrl->name) == 0)
			display->ctrl = NULL;
	}
}
EXPORT_SYMBOL(omap_dss_unregister_ctrl);

void omap_dss_unregister_panel(struct omap_panel *panel)
{
	int i;

	for (i = 0; i < num_displays; i++) {
		struct omap_display *display = &displays[i];
		if (display->hw_config.panel_name &&
		    strcmp(display->hw_config.panel_name, panel->name) == 0)
			display->panel = NULL;
	}
}
EXPORT_SYMBOL(omap_dss_unregister_panel);
