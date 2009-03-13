/*
 * linux/arch/arm/plat-omap/dss/dss.h
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

#ifndef __OMAP2_DSS_H
#define __OMAP2_DSS_H

#ifdef CONFIG_OMAP2_DSS_DEBUG_SUPPORT
#define DEBUG
#endif

#ifdef DEBUG
extern unsigned int dss_debug;
#ifdef DSS_SUBSYS_NAME
#define DSSDBG(format, ...) \
	if (dss_debug) \
		printk(KERN_DEBUG "omap-dss " DSS_SUBSYS_NAME ": " format, \
		## __VA_ARGS__)
#else
#define DSSDBG(format, ...) \
	if (dss_debug) \
		printk(KERN_DEBUG "omap-dss: " format, ## __VA_ARGS__)
#endif
#else
#define DSSDBG(format, ...)
#endif

#ifdef DSS_SUBSYS_NAME
#define DSSERR(format, ...) \
	printk(KERN_ERR "omap-dss " DSS_SUBSYS_NAME " error: " format, \
	## __VA_ARGS__)
#else
#define DSSERR(format, ...) \
	printk(KERN_ERR "omap-dss error: " format, ## __VA_ARGS__)
#endif

#ifdef DSS_SUBSYS_NAME
#define DSSINFO(format, ...) \
	printk(KERN_INFO "omap-dss " DSS_SUBSYS_NAME ": " format, \
	## __VA_ARGS__)
#else
#define DSSINFO(format, ...) \
	printk(KERN_INFO "omap-dss: " format, ## __VA_ARGS__)
#endif

#ifdef DSS_SUBSYS_NAME
#define DSSWARN(format, ...) \
	printk(KERN_WARNING "omap-dss " DSS_SUBSYS_NAME ": " format, \
	## __VA_ARGS__)
#else
#define DSSWARN(format, ...) \
	printk(KERN_WARNING "omap-dss: " format, ## __VA_ARGS__)
#endif

/* OMAP TRM gives bitfields as start:end, where start is the higher bit
   number. For example 7:0 */
#define FLD_MASK(start, end)	(((1 << (start - end + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << end) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

#define DISPC_MAX_FCK 173000000

enum omap_burst_size {
	OMAP_DSS_BURST_4x32 = 0,
	OMAP_DSS_BURST_8x32 = 1,
	OMAP_DSS_BURST_16x32 = 2,
};

enum omap_parallel_interface_mode {
	OMAP_DSS_PARALLELMODE_BYPASS,		/* MIPI DPI */
	OMAP_DSS_PARALLELMODE_RFBI,		/* MIPI DBI */
	OMAP_DSS_PARALLELMODE_DSI,
};

enum dss_clock {
	DSS_CLK_ICK	= 1 << 0,
	DSS_CLK_FCK1	= 1 << 1,
	DSS_CLK_FCK2	= 1 << 2,
	DSS_CLK_54M	= 1 << 3,
	DSS_CLK_96M	= 1 << 4,
};

struct dispc_clock_info {
	/* rates that we get with dividers below */
	unsigned long fck;
	unsigned long lck;
	unsigned long pck;

	/* dividers */
	int fck_div;
	int lck_div;
	int pck_div;
};

struct dsi_clock_info {
	/* rates that we get with dividers below */
	unsigned long fint;
	unsigned long dsiphy;
	unsigned long clkin;	/* input clk for DSI PLL */
	unsigned long dispc_fck;	/* output clk, DSI1_PLL_FCLK */
	unsigned long dsi_fck;	/* output clk, DSI2_PLL_FCLK */
	unsigned long lck;
	unsigned long pck;

	/* dividers */
	int regn;
	int regm;
	int regm3;
	int regm4;

	int lck_div;
	int pck_div;

	int highfreq;
	int use_dss2_fck;
};

int initialize_sysfs(struct device *dev);
void uninitialize_sysfs(struct device *dev);
void initialize_displays(struct omap_dss_platform_data *pdata);
void initialize_overlays(const char *def_disp_name);
int omap_dss_suspend_all_displays(void);
int omap_dss_resume_all_displays(void);

/* DSS */
int dss_init(void);
void dss_exit(void);

void dss_clk_enable(enum dss_clock clks);
void dss_clk_disable(enum dss_clock clks);

void dss_sdi_init(int datapairs);
void dss_select_clk_source(int dsi, int dispc);
int dss_get_dsi_clk_source(void);
int dss_get_dispc_clk_source(void);
void dss_set_venc_output(enum omap_dss_venc_type type);
void dss_set_dac_pwrdn_bgz(int enable);
unsigned long dss_clk_get_rate(enum dss_clock clk);
ssize_t dss_print_clocks(char *buf, ssize_t size);

/* SDI */
int sdi_init(void);
void sdi_exit(void);
void sdi_init_display(struct omap_display *display);

/* DSI */
int dsi_init(void);
void dsi_exit(void);

void dsi_save_context(void);
void dsi_restore_context(void);

void dsi_init_display(struct omap_display *display);
void dsi_irq_handler(void);
unsigned long dsi_get_dsi1_pll_rate(void);
unsigned long dsi_get_dsi2_pll_rate(void);
int dsi_pll_calc_pck(int is_tft, unsigned long req_pck,
		struct dsi_clock_info *cinfo);
int dsi_pll_program(struct dsi_clock_info *cinfo);
int dsi_pll_init(int enable_hsclk, int enable_hsdiv);
void dsi_pll_uninit(void);
ssize_t dsi_print_clocks(char *buf, ssize_t size);

/* DPI */
int dpi_init(void);
void dpi_exit(void);
void dpi_init_display(struct omap_display *display);

/* DISPC */
int dispc_init(void);
void dispc_exit(void);
void dispc_irq_handler(void);
void dispc_fake_vsync_irq(void);

void dispc_save_context(void);
void dispc_restore_context(void);

void dispc_lcd_enable_signal_polarity(int act_high);
void dispc_lcd_enable_signal(int enable);
void dispc_pck_free_enable(int enable);
void dispc_enable_fifohandcheck(int enable);

void dispc_set_lcd_size(int width, int height);
void dispc_set_digit_size(int width, int height);
u32 dispc_get_plane_fifo_size(enum omap_plane plane);
void dispc_setup_plane_fifo(enum omap_plane plane, u32 low, u32 high);
void dispc_set_burst_size(enum omap_plane plane,
		enum omap_burst_size burst_size);

void dispc_set_plane_ba0(enum omap_plane plane, u32 paddr);
void dispc_set_plane_ba1(enum omap_plane plane, u32 paddr);
void dispc_set_plane_pos(enum omap_plane plane, int x, int y);
void dispc_set_plane_size(enum omap_plane plane, int width, int height);
void dispc_set_row_inc(enum omap_plane plane, int inc);

int dispc_setup_plane(enum omap_plane plane, enum omap_channel channel_out,
		      u32 paddr, int tv_field1_offset, int screen_width,
		      int pos_x, int pos_y,
		      int width, int height,
		      int out_width, int out_height,
		      enum omap_color_mode color_mode,
		      int ilace, int rotation, int mirror, int global_alpha);

void dispc_go(enum omap_channel channel);
void dispc_enable_lcd_out(int enable);
void dispc_enable_digit_out(int enable);
int dispc_enable_plane(enum omap_plane plane, int enable);

void dispc_set_parallel_interface_mode(enum omap_parallel_interface_mode mode);
void dispc_set_tft_data_lines(int data_lines);
void dispc_set_lcd_display_type(enum omap_lcd_display_type type);
void dispc_set_loadmode(enum omap_dss_load_mode mode);

void omap_dispc_set_trans_key(enum omap_channel ch,
		enum omap_dss_color_key_type type,
		u32 trans_key);
void omap_dispc_enable_trans_key(enum omap_channel ch, int enable);
void dispc_enable_alpha_blending(enum omap_channel ch, int enable);
void dispc_get_color_keying(enum omap_channel ch, struct omap_color_key *key);

void dispc_set_lcd_timings(struct omap_video_timings *timings);
unsigned long dispc_fclk_rate(void);
unsigned long dispc_pclk_rate(void);
void dispc_set_pol_freq(struct omap_panel *panel);
void find_lck_pck_divs(int is_tft, unsigned long req_pck, unsigned long fck,
		int *lck_div, int *pck_div);
int dispc_calc_clock_div(int is_tft, unsigned long req_pck,
		struct dispc_clock_info *cinfo);
int dispc_set_clock_div(struct dispc_clock_info *cinfo);
void dispc_set_lcd_divisor(int lck_div, int pck_div);

void dispc_setup_partial_planes(struct omap_display *display,
				int *x, int *y, int *w, int *h);
void dispc_draw_partial_planes(struct omap_display *display);

ssize_t dispc_print_clocks(char *buf, ssize_t size);
void omap_dispc_set_default_color(enum omap_channel channel, u32 color);
int dispc_get_alpha_blending(enum omap_channel ch);
int omap_dispc_get_default_color(enum omap_channel channel);

/* VENC */
int venc_init(void);
void venc_exit(void);
void venc_init_display(struct omap_display *display);

/* RFBI */
int rfbi_init(void);
void rfbi_exit(void);

int rfbi_configure(int rfbi_module, int bpp, int lines);
void rfbi_enable_rfbi(int enable);
void rfbi_transfer_area(int width, int height,
			     void (callback)(void *data), void *data);
void rfbi_set_timings(int rfbi_module, struct rfbi_timings *t);
unsigned long rfbi_get_max_tx_rate(void);
void rfbi_init_display(struct omap_display *display);

#endif
