/*
 * linux/arch/arm/plat-omap/dss/sdi.c
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
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

#define DSS_SUBSYS_NAME "SDI"

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <mach/board.h>
#include <mach/display.h>
#include "dss.h"


static struct {
	int update_enabled;
} sdi;

static int sdi_display_enable(struct omap_display *display)
{
	struct dispc_clock_info cinfo;
	int lck_div, pck_div;
	unsigned long fck;
	struct omap_panel *panel = display->panel;
	unsigned high, low, burst;
	unsigned long pck;

	if (display->state != OMAP_DSS_DISPLAY_DISABLED) {
		DSSERR("display already enabled\n");
		return -EINVAL;
	}

	panel->enable(display);

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	dispc_set_parallel_interface_mode(OMAP_DSS_PARALLELMODE_BYPASS);

	dispc_set_burst_size(OMAP_DSS_GFX, OMAP_DSS_BURST_16x32);
	dispc_set_burst_size(OMAP_DSS_VIDEO1, OMAP_DSS_BURST_16x32);
	dispc_set_burst_size(OMAP_DSS_VIDEO2, OMAP_DSS_BURST_16x32);

	burst = 16 * 32 / 8;

	high = dispc_get_plane_fifo_size(OMAP_DSS_GFX) - burst;
	low = dispc_get_plane_fifo_size(OMAP_DSS_GFX) / 4 * 3;
	dispc_setup_plane_fifo(OMAP_DSS_GFX, low, high);

	high = dispc_get_plane_fifo_size(OMAP_DSS_VIDEO1) - burst;
	low = dispc_get_plane_fifo_size(OMAP_DSS_VIDEO1) / 4 * 3;
	dispc_setup_plane_fifo(OMAP_DSS_VIDEO1, low, high);

	high = dispc_get_plane_fifo_size(OMAP_DSS_VIDEO2) - burst;
	low = dispc_get_plane_fifo_size(OMAP_DSS_VIDEO2) / 4 * 3;
	dispc_setup_plane_fifo(OMAP_DSS_VIDEO2, low, high);

	/* 15.5.9.1.2 */
	panel->config |= OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF;

	dispc_set_pol_freq(panel);

	dispc_calc_clock_div(1, panel->timings.pixel_clock * 1000,
			&cinfo);

	if (dispc_set_clock_div(&cinfo)) {
		DSSERR("Failed to set DSS clocks\n");
		return -EINVAL;
	}

	fck = cinfo.fck;
	lck_div = cinfo.lck_div;
	pck_div = cinfo.pck_div;

	pck = fck / lck_div / pck_div / 1000;

	if (pck != panel->timings.pixel_clock) {
		DSSWARN("Could not find exact pixel clock. Requested %d kHz, "
				"got %lu kHz\n",
				panel->timings.pixel_clock, pck);

		panel->timings.pixel_clock = pck;
	}

	dispc_set_lcd_timings(&panel->timings);

	dispc_set_lcd_display_type(OMAP_DSS_LCD_DISPLAY_TFT);
	dispc_set_tft_data_lines(24);
	dispc_lcd_enable_signal_polarity(1);
	dispc_pck_free_enable(1);

	dss_sdi_init(display->hw_config.u.sdi.datapairs);

	mdelay(2);

	dispc_enable_lcd_out(1);

	display->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void sdi_display_disable(struct omap_display *display)
{
	if (display->state == OMAP_DSS_DISPLAY_DISABLED)
		return;

	display->panel->disable(display);
	dispc_enable_lcd_out(0);

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);

	display->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int sdi_display_set_update_mode(struct omap_display *display,
		enum omap_dss_update_mode mode)
{
	if (mode == OMAP_DSS_UPDATE_MANUAL)
		return -EINVAL;

	if (mode == OMAP_DSS_UPDATE_DISABLED) {
		dispc_enable_lcd_out(0);
		sdi.update_enabled = 0;
	} else {
		dispc_enable_lcd_out(1);
		sdi.update_enabled = 1;
	}

	return 0;
}

static enum omap_dss_update_mode sdi_display_get_update_mode(
		struct omap_display *display)
{
	return sdi.update_enabled ? OMAP_DSS_UPDATE_AUTO :
		OMAP_DSS_UPDATE_DISABLED;
}


void sdi_init_display(struct omap_display *display)
{
	DSSDBG("SDI init\n");

	display->enable = sdi_display_enable;
	display->disable = sdi_display_disable;
	display->set_update_mode = sdi_display_set_update_mode;
	display->get_update_mode = sdi_display_get_update_mode;
}

int sdi_init(void)
{
	return 0;
}

void sdi_exit(void)
{
}
