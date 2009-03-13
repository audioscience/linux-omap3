/*
 * linux/arch/arm/plat-omap/dss/dpi.c
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

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/completion.h>
#include <linux/jiffies.h>

#include <mach/board.h>
#include <mach/display.h>
#include "dss.h"

/* TODO find the exact value of the fifo threshold.
 * With previously configured threshold under run
 * errors were coming
 */
#define OMAP_DSS_FIFO_LOW_THRESHOLD	0x03bc
#define OMAP_DSS_FIFO_HIGH_THRESHOLD	0x03fc

static struct {
	int update_enabled;
} dpi;

static void dpi_set_mode(struct omap_display *display)
{
	struct omap_panel *panel = display->panel;
	int lck_div, pck_div;
	unsigned long fck;
	unsigned long pck;
	int is_tft;

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	dispc_set_pol_freq(panel);

	is_tft = (display->panel->config & OMAP_DSS_LCD_TFT) != 0;

#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL
	{
		struct dsi_clock_info cinfo;
		dsi_pll_calc_pck(is_tft,
				display->panel->timings.pixel_clock * 1000,
				&cinfo);

		dsi_pll_program(&cinfo);

		dss_select_clk_source(0, 1);

		dispc_set_lcd_divisor(cinfo.lck_div, cinfo.pck_div);

		fck = cinfo.dispc_fck;
		lck_div = cinfo.lck_div;
		pck_div = cinfo.pck_div;
	}
#else
	{
		struct dispc_clock_info cinfo;
		dispc_calc_clock_div(is_tft, panel->timings.pixel_clock * 1000,
				&cinfo);

		if (dispc_set_clock_div(&cinfo)) {
			DSSERR("Failed to set DSS clocks\n");
			dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
			return;
		}

		fck = cinfo.fck;
		lck_div = cinfo.lck_div;
		pck_div = cinfo.pck_div;
	}
#endif

	pck = fck / lck_div / pck_div / 1000;

	if (pck != panel->timings.pixel_clock) {
		DSSWARN("Could not find exact pixel clock. "
				"Requested %d kHz, got %lu kHz\n",
				panel->timings.pixel_clock, pck);

		panel->timings.pixel_clock = pck;
	}

	dispc_set_lcd_timings(&panel->timings);

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
}

static int dpi_display_enable(struct omap_display *display)
{
	struct omap_panel *panel = display->panel;
	int r;
	int is_tft;

	if (display->ref_count > 0) {
		display->ref_count++;
		return -EINVAL;
	}
	if (display->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;
	r = panel->enable(display);
	if (r)
		return r;

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL
	dss_clk_enable(DSS_CLK_FCK2);
	r = dsi_pll_init(0, 1);
	if (r)
		return r;
#endif
	is_tft = (display->panel->config & OMAP_DSS_LCD_TFT) != 0;

	dispc_set_parallel_interface_mode(OMAP_DSS_PARALLELMODE_BYPASS);
	dispc_set_lcd_display_type(is_tft ? OMAP_DSS_LCD_DISPLAY_TFT :
			OMAP_DSS_LCD_DISPLAY_STN);
	dispc_set_tft_data_lines(display->hw_config.u.dpi.data_lines);

	dispc_set_burst_size(OMAP_DSS_GFX, OMAP_DSS_BURST_16x32);
	dispc_set_burst_size(OMAP_DSS_VIDEO1, OMAP_DSS_BURST_16x32);
	dispc_set_burst_size(OMAP_DSS_VIDEO2, OMAP_DSS_BURST_16x32);

	dispc_setup_plane_fifo(OMAP_DSS_GFX, OMAP_DSS_FIFO_LOW_THRESHOLD,
			OMAP_DSS_FIFO_HIGH_THRESHOLD);
	dispc_setup_plane_fifo(OMAP_DSS_VIDEO1, OMAP_DSS_FIFO_LOW_THRESHOLD,
			OMAP_DSS_FIFO_HIGH_THRESHOLD);
	dispc_setup_plane_fifo(OMAP_DSS_VIDEO1, OMAP_DSS_FIFO_LOW_THRESHOLD,
			OMAP_DSS_FIFO_HIGH_THRESHOLD);

	dpi_set_mode(display);

	mdelay(2);

	dispc_enable_lcd_out(1);

	display->state = OMAP_DSS_DISPLAY_ACTIVE;
	display->ref_count++;

	return 0;
}

static int dpi_display_resume(struct omap_display *display);

static void dpi_display_disable(struct omap_display *display)
{
	if (display->ref_count > 1) {
		display->ref_count--;
		return;
	}
	if (display->state == OMAP_DSS_DISPLAY_DISABLED)
		return;

	if (display->state == OMAP_DSS_DISPLAY_SUSPENDED)
		dpi_display_resume(display);

	display->panel->disable(display);
	dispc_enable_lcd_out(0);

#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL
	dss_select_clk_source(0, 0);
	dsi_pll_uninit();
	dss_clk_disable(DSS_CLK_FCK2);
#endif

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);

	display->state = OMAP_DSS_DISPLAY_DISABLED;
	display->ref_count--;
}
/*
 * Interrupt Service Routine for frame done interrupt.
 */
static void dpi_display_isr(void *arg, unsigned int irqstatus)
{
	struct omap_display *display = (struct omap_display *)arg;

	complete(&display->frame_done);
}

static int dpi_display_suspend(struct omap_display *display)
{
	void *handle = NULL;

	if (display->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	if (display->panel->suspend)
		display->panel->suspend(display);

	/*
	 * Wait for frame done interrupt
	 */
	handle = omap_dispc_register_isr(dpi_display_isr, display,
			DISPC_IRQ_FRAMEDONE);
	if (!handle)
		return -EINVAL;

	init_completion(&display->frame_done);

	dispc_enable_lcd_out(0);
	if (!wait_for_completion_timeout(&display->frame_done,
				msecs_to_jiffies(500))) {
		DSSERR("Timeout waiting for FRAME DONE\n");
	}

	if (omap_dispc_unregister_isr(handle) < 0) {
		DSSERR("Failed to unregister the ISR\n");
		return -EINVAL;
	}
	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);

	display->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}

static int dpi_display_resume(struct omap_display *display)
{
	if (display->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	dispc_enable_lcd_out(1);

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	if (display->panel->resume)
		display->panel->resume(display);

	display->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void dpi_set_timings(struct omap_display *display,
			struct omap_video_timings *timings)
{
	DSSDBG("dpi_set_timings\n");
	display->panel->timings = *timings;
	if (display->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dpi_set_mode(display);
		dispc_go(OMAP_DSS_CHANNEL_LCD);
	}
}

static void dpi_display_set_bg_color(struct omap_display *display,
			unsigned int color)
{
	omap_dispc_set_default_color(OMAP_DSS_CHANNEL_LCD, color);
	dispc_go(OMAP_DSS_CHANNEL_LCD);
}

static int dpi_display_get_bg_color(struct omap_display *display)
{
	return omap_dispc_get_default_color(OMAP_DSS_CHANNEL_LCD);
}

static int dpi_check_timings(struct omap_display *display,
			struct omap_video_timings *timings)
{
	int is_tft;
	int r;
	int lck_div, pck_div;
	unsigned long fck;
	unsigned long pck;

	if (timings->hsw < 1 || timings->hsw > 64 ||
		timings->hfp < 1 || timings->hfp > 256 ||
		timings->hbp < 1 || timings->hbp > 256) {
		return -EINVAL;
	}

	if (timings->vsw < 1 || timings->vsw > 64 ||
		timings->vfp > 256 || timings->vbp > 256) {
		return -EINVAL;
	}

	if (timings->pixel_clock == 0)
		return -EINVAL;

	is_tft = (display->panel->config & OMAP_DSS_LCD_TFT) != 0;

#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL
	{
		struct dsi_clock_info cinfo;
		r = dsi_pll_calc_pck(is_tft, timings->pixel_clock * 1000,
				&cinfo);

		if (r)
			return r;

		fck = cinfo.dispc_fck;
		lck_div = cinfo.lck_div;
		pck_div = cinfo.pck_div;
	}
#else
	{
		struct dispc_clock_info cinfo;
		r = dispc_calc_clock_div(is_tft, timings->pixel_clock * 1000,
				&cinfo);

		if (r)
			return r;

		fck = cinfo.fck;
		lck_div = cinfo.lck_div;
		pck_div = cinfo.pck_div;
	}
#endif

	pck = fck / lck_div / pck_div / 1000;

	timings->pixel_clock = pck;

	return 0;
}

static void dpi_get_timings(struct omap_display *display,
			struct omap_video_timings *timings)
{
	*timings = display->panel->timings;
}

static int dpi_display_set_update_mode(struct omap_display *display,
		enum omap_dss_update_mode mode)
{
	if (mode == OMAP_DSS_UPDATE_MANUAL)
		return -EINVAL;

	if (mode == OMAP_DSS_UPDATE_DISABLED) {
		dispc_enable_lcd_out(0);
		dpi.update_enabled = 0;
	} else {
		dispc_enable_lcd_out(1);
		dpi.update_enabled = 1;
	}

	return 0;
}

static enum omap_dss_update_mode dpi_display_get_update_mode(
		struct omap_display *display)
{
	return dpi.update_enabled ? OMAP_DSS_UPDATE_AUTO :
		OMAP_DSS_UPDATE_DISABLED;
}

static void dpi_display_set_color_keying(struct omap_display *display,
	struct omap_color_key *key)
{
	omap_dispc_set_trans_key(OMAP_DSS_CHANNEL_LCD, key->type, key->color);
	omap_dispc_enable_trans_key(OMAP_DSS_CHANNEL_LCD, key->enable);
	dispc_go(OMAP_DSS_CHANNEL_LCD);
}

static void dpi_enable_alpha_blending(struct omap_display *display,
	unsigned int enable)
{
	dispc_enable_alpha_blending(OMAP_DSS_CHANNEL_LCD, enable);
	dispc_go(OMAP_DSS_CHANNEL_LCD);
}
static int dpi_get_alpha_blending(struct omap_display *display)
{
	return dispc_get_alpha_blending(OMAP_DSS_CHANNEL_LCD);
}
static void dpi_display_get_color_keying(struct omap_display *display,
	struct omap_color_key *key)
{
	dispc_get_color_keying(OMAP_DSS_CHANNEL_LCD, key);
}

void dpi_init_display(struct omap_display *display)
{
	DSSDBG("DPI init_display\n");

	display->enable = dpi_display_enable;
	display->disable = dpi_display_disable;
	display->suspend = dpi_display_suspend;
	display->resume = dpi_display_resume;
	display->set_timings = dpi_set_timings;
	display->check_timings = dpi_check_timings;
	display->get_timings = dpi_get_timings;
	display->set_update_mode = dpi_display_set_update_mode;
	display->get_update_mode = dpi_display_get_update_mode;
	display->set_bg_color = dpi_display_set_bg_color;
	display->get_bg_color = dpi_display_get_bg_color;
	display->set_color_keying = dpi_display_set_color_keying;
	display->get_color_keying = dpi_display_get_color_keying;
	display->enable_alpha_blending = dpi_enable_alpha_blending;
	display->get_alpha_blending = dpi_get_alpha_blending;
}

int dpi_init(void)
{
	return 0;
}

void dpi_exit(void)
{
}

