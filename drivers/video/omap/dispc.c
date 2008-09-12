/*
 * OMAP2 display controller support
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Imre Deak <imre.deak@nokia.com>
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
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <asm/arch/sram.h>
#include <asm/arch/omapfb.h>
#include <asm/arch/board.h>
#include <asm/arch/omap-dss.h>
#include "dispc.h"

#define MODULE_NAME			"dispc"

#define DISPC_BASE			(DSS_REG_BASE + DISPC_REG_OFFSET)
#define DISPC_IRQ_MASK_ALL		0x7fff

#define DISPC_IRQ_MASK_ERROR		(DISPC_IRQSTATUS_GFXFIFOUNDERFLOW |\
					 DISPC_IRQSTATUS_VID2FIFOUNDERFLOW | \
					 DISPC_IRQSTATUS_VID2FIFOUNDERFLOW | \
					 DISPC_IRQSTATUS_SYNCLOST)

#define MAX_PALETTE_SIZE		(256 * 16)

#define FLD_MASK(pos, len)	(((1 << len) - 1) << pos)

#define MOD_REG_FLD(reg, mask, val) \
	dispc_write_reg((reg), (dispc_read_reg(reg) & ~(mask)) | (val));

#define OMAP2_SRAM_START		0x40200000
/* Maximum size, in reality this is smaller if SRAM is partially locked. */
#define OMAP2_SRAM_SIZE			0xa0000		/* 640k */

/* We support the SDRAM / SRAM types. See OMAPFB_PLANE_MEMTYPE_* in omapfb.h */
#define DISPC_MEMTYPE_NUM		2

#define RESMAP_SIZE(_page_cnt)						\
	((_page_cnt + (sizeof(unsigned long) * 8) - 1) / 8)
#define RESMAP_PTR(_res_map, _page_nr)					\
	(((_res_map)->map) + (_page_nr) / (sizeof(unsigned long) * 8))
#define RESMAP_MASK(_page_nr)						\
	(1 << ((_page_nr) & (sizeof(unsigned long) * 8 - 1)))

struct resmap {
	unsigned long	start;
	unsigned	page_cnt;
	unsigned long	*map;
};

static struct {
	u32		base;

	struct omapfb_mem_desc	mem_desc;
	struct resmap		*res_map[DISPC_MEMTYPE_NUM];
	atomic_t		map_count[OMAPFB_PLANE_NUM];

	dma_addr_t	palette_paddr;
	void		*palette_vaddr;

	int		ext_mode;

	unsigned long	enabled_irqs;
	void		(*irq_callback)(void *);
	void		*irq_callback_data;
	struct completion	frame_done;

	int		fir_hinc[OMAPFB_PLANE_NUM];
	int		fir_vinc[OMAPFB_PLANE_NUM];

	struct clk	*dss_ick, *dss1_fck;
	struct clk	*dss_54m_fck;

	enum omapfb_update_mode	update_mode;
	struct omapfb_device	*fbdev;

	struct omapfb_color_key	color_key;
} dispc;

static void inline dispc_write_reg(int idx, u32 val)
{
	__raw_writel(val, dispc.base + idx);
}

static u32 inline dispc_read_reg(int idx)
{
	u32 l = __raw_readl(dispc.base + idx);
	return l;
}

/* Select RFBI or bypass mode */
static void enable_rfbi_mode(int enable)
{
	u32 l;

	l = dispc_read_reg(DISPC_CONTROL);
	/* Enable RFBI, GPIO0/1 */
	l &= ~((1 << 11) | (1 << 15) | (1 << 16));
	l |= enable ? (1 << 11) : 0;
	/* RFBI En: GPIO0/1=10  RFBI Dis: GPIO0/1=11 */
	l |= 1 << 15;
	l |= enable ? 0 : (1 << 16);
	dispc_write_reg(DISPC_CONTROL, l);

	/* Set bypass mode in RFBI module */
	l = __raw_readl(io_p2v(DSS_REG_BASE + DSS_CONTROL));
	l |= enable ? 0 : (1 << 1);
	__raw_writel(l, io_p2v(DSS_REG_BASE + DSS_CONTROL));
}

static void set_lcd_data_lines(int data_lines)
{
	u32 l;
	int code = 0;

	switch (data_lines) {
	case 12:
		code = 0;
		break;
	case 16:
		code = 1;
		break;
	case 18:
		code = 2;
		break;
	case 24:
		code = 3;
		break;
	default:
		BUG();
	}

	l = dispc_read_reg(DISPC_CONTROL);
	l &= ~(0x03 << 8);
	l |= code << 8;
	dispc_write_reg(DISPC_CONTROL, l);
}

static void set_load_mode(int mode)
{
	BUG_ON(mode & ~(DISPC_LOAD_CLUT_ONLY | DISPC_LOAD_FRAME_ONLY |
			DISPC_LOAD_CLUT_ONCE_FRAME));
	MOD_REG_FLD(DISPC_CONFIG, 0x03 << 1, mode << 1);
}

void omap_dispc_set_lcd_size(int x, int y)
{
	BUG_ON((x > (1 << 11)) || (y > (1 << 11)));
	omap_disp_get_dss();
	MOD_REG_FLD(DISPC_SIZE_LCD, FLD_MASK(16, 11) | FLD_MASK(0, 11),
			((y - 1) << 16) | (x - 1));
	omap_disp_put_dss();
}
EXPORT_SYMBOL(omap_dispc_set_lcd_size);

void omap_dispc_set_digit_size(int x, int y)
{
	BUG_ON((x > (1 << 11)) || (y > (1 << 11)));
	omap_disp_get_dss();
	MOD_REG_FLD(DISPC_SIZE_DIG, FLD_MASK(16, 11) | FLD_MASK(0, 11),
			((y - 1) << 16) | (x - 1));
	omap_disp_put_dss();
}
EXPORT_SYMBOL(omap_dispc_set_digit_size);

static void setup_plane_fifo(int plane, int ext_mode)
{
	const u32 ftrs_reg[] = { DISPC_GFX_FIFO_THRESHOLD,
				DISPC_VID_FIFO_THRESHOLD(0),
				DISPC_VID_FIFO_THRESHOLD(1) };
	const u32 fsz_reg[] = { DISPC_GFX_FIFO_SIZE,
				DISPC_VID_FIFO_SIZE(0),
				DISPC_VID_FIFO_SIZE(1) };
	int low, high;
	u32 l;

	BUG_ON(plane > 2);

	l = dispc_read_reg(fsz_reg[plane]);
	l &= FLD_MASK(0, 9);
	if (ext_mode) {
		low = l * 3 / 4;
		high = l;
	} else {
		low = l / 4;
		high = l * 3 / 4;
	}
	MOD_REG_FLD(ftrs_reg[plane], FLD_MASK(16, 9) | FLD_MASK(0, 9),
			(high << 16) | low);
}

void omap_dispc_enable_lcd_out(int enable)
{
	omap_disp_get_dss();
	MOD_REG_FLD(DISPC_CONTROL, 1, enable ? 1 : 0);
	omap_disp_put_dss();
}
EXPORT_SYMBOL(omap_dispc_enable_lcd_out);

void omap_dispc_enable_digit_out(int enable)
{
	omap_disp_get_dss();
	MOD_REG_FLD(DISPC_CONTROL, 1 << 1, enable ? 1 << 1 : 0);
	omap_disp_put_dss();
}
EXPORT_SYMBOL(omap_dispc_enable_digit_out);

static inline int _setup_plane(int plane, int channel_out,
				  u32 paddr, int screen_width,
				  int pos_x, int pos_y, int width, int height,
				  int color_mode)
{
	const u32 at_reg[] = { DISPC_GFX_ATTRIBUTES,
				DISPC_VID_ATTRIBUTES(0),
				DISPC_VID_ATTRIBUTES(1) };
	const u32 ba_reg[] = { DISPC_GFX_BA0, DISPC_VID_BA0(0),
				DISPC_VID_BA0(1) };
	const u32 ps_reg[] = { DISPC_GFX_POSITION,
				DISPC_VID_POSITION(0),
				DISPC_VID_POSITION(1) };
	const u32 sz_reg[] = { DISPC_GFX_SIZE,
				DISPC_VID_PICTURE_SIZE(0),
				DISPC_VID_PICTURE_SIZE(1) };
	const u32 ri_reg[] = { DISPC_GFX_ROW_INC,
				DISPC_VID_ROW_INC(0),
				DISPC_VID_ROW_INC(1) };
	const u32 vs_reg[] = { 0, DISPC_VID_SIZE(0),
				DISPC_VID_SIZE(1) };

	int chout_shift, burst_shift;
	int chout_val;
	int color_code;
	int bpp;
	int cconv_en;
	int set_vsize;
	u32 l;

#ifdef VERBOSE
	dev_dbg(dispc.fbdev->dev, "plane %d channel %d paddr %#08x scr_width %d"
		    " pos_x %d pos_y %d width %d height %d color_mode %d\n",
		    plane, channel_out, paddr, screen_width, pos_x, pos_y,
		    width, height, color_mode);
#endif

	set_vsize = 0;
	switch (plane) {
	case OMAPFB_PLANE_GFX:
		burst_shift = 6;
		chout_shift = 8;
		break;
	case OMAPFB_PLANE_VID1:
	case OMAPFB_PLANE_VID2:
		burst_shift = 14;
		chout_shift = 16;
		set_vsize = 1;
		break;
	default:
		return -EINVAL;
	}

	switch (channel_out) {
	case OMAPFB_CHANNEL_OUT_LCD:
		chout_val = 0;
		break;
	case OMAPFB_CHANNEL_OUT_DIGIT:
		chout_val = 1;
		break;
	default:
		return -EINVAL;
	}

	cconv_en = 0;
	switch (color_mode) {
	case OMAPFB_COLOR_RGB565:
		color_code = DISPC_RGB_16_BPP;
		bpp = 16;
		break;
	case OMAPFB_COLOR_YUV422:
		if (plane == 0)
			return -EINVAL;
		color_code = DISPC_UYVY_422;
		cconv_en = 1;
		bpp = 16;
		break;
	case OMAPFB_COLOR_YUY422:
		if (plane == 0)
			return -EINVAL;
		color_code = DISPC_YUV2_422;
		cconv_en = 1;
		bpp = 16;
		break;
	default:
		return -EINVAL;
	}

	l = dispc_read_reg(at_reg[plane]);

	l &= ~(0x0f << 1);
	l |= color_code << 1;
	l &= ~(1 << 9);
	l |= cconv_en << 9;

	l &= ~(0x03 << burst_shift);
	l |= DISPC_BURST_8x32 << burst_shift;

	l &= ~(1 << chout_shift);
	l |= chout_val << chout_shift;

	dispc_write_reg(at_reg[plane], l);

	dispc_write_reg(ba_reg[plane], paddr);
	MOD_REG_FLD(ps_reg[plane],
		    FLD_MASK(16, 11) | FLD_MASK(0, 11), (pos_y << 16) | pos_x);

	MOD_REG_FLD(sz_reg[plane], FLD_MASK(16, 11) | FLD_MASK(0, 11),
			((height - 1) << 16) | (width - 1));

	if (set_vsize) {
		/* Set video size if set_scale hasn't set it */
		if (!dispc.fir_vinc[plane])
			MOD_REG_FLD(vs_reg[plane],
				FLD_MASK(16, 11), (height - 1) << 16);
		if (!dispc.fir_hinc[plane])
			MOD_REG_FLD(vs_reg[plane],
				FLD_MASK(0, 11), width - 1);
	}

	dispc_write_reg(ri_reg[plane], (screen_width - width) * bpp / 8 + 1);

	return height * screen_width * bpp / 8;
}

static int omap_dispc_setup_plane(int plane, int channel_out,
				  unsigned long offset,
				  int screen_width,
				  int pos_x, int pos_y, int width, int height,
				  int color_mode)
{
	u32 paddr;
	int r;

	if ((unsigned)plane > dispc.mem_desc.region_cnt)
		return -EINVAL;
	paddr = dispc.mem_desc.region[plane].paddr + offset;
	omap_disp_get_dss();
	r = _setup_plane(plane, channel_out, paddr,
			screen_width,
			pos_x, pos_y, width, height, color_mode);
	omap_disp_put_dss();
	return r;
}

static void write_firh_reg(int plane, int reg, u32 value)
{
	u32 base;

	if (plane == 1)
		base = DISPC_VID_FIR_COEF_H(0, reg);
	else
		base = DISPC_VID_FIR_COEF_H(1, reg);
	dispc_write_reg(base, value);
}

static void write_firhv_reg(int plane, int reg, u32 value)
{
	u32 base;

	if (plane == 1)
		base = DISPC_VID_FIR_COEF_HV(0, reg);
	else
		base = DISPC_VID_FIR_COEF_HV(1, reg);
	dispc_write_reg(base, value);
}

static void set_upsampling_coef_table(int plane)
{
	const u32 coef[][2] = {
		{ 0x00800000, 0x00800000 },
		{ 0x0D7CF800, 0x037B02FF },
		{ 0x1E70F5FF, 0x0C6F05FE },
		{ 0x335FF5FE, 0x205907FB },
		{ 0xF74949F7, 0x00404000 },
		{ 0xF55F33FB, 0x075920FE },
		{ 0xF5701EFE, 0x056F0CFF },
		{ 0xF87C0DFF, 0x027B0300 },
	};
	int i;

	for (i = 0; i < 8; i++) {
		write_firh_reg(plane, i, coef[i][0]);
		write_firhv_reg(plane, i, coef[i][1]);
	}
}

static int omap_dispc_set_scale(int plane,
				int orig_width, int orig_height,
				int out_width, int out_height)
{
	const u32 at_reg[]  = { 0, DISPC_VID_ATTRIBUTES(0),
				DISPC_VID_ATTRIBUTES(1) };
	const u32 vs_reg[]  = { 0, DISPC_VID_SIZE(0),
				DISPC_VID_SIZE(1) };
	const u32 fir_reg[] = { 0, DISPC_VID_FIR(0),
				DISPC_VID_FIR(1) };

	u32 l;
	int fir_hinc;
	int fir_vinc;

	if ((unsigned)plane > OMAPFB_PLANE_NUM)
		return -ENODEV;

	if (plane == OMAPFB_PLANE_GFX &&
	    (out_width != orig_width || out_height != orig_height))
		return -EINVAL;

	omap_disp_get_dss();
	if (orig_width < out_width) {
		/*
		 * Upsampling.
		 * Currently you can only scale both dimensions in one way.
		 */
		if (orig_height > out_height ||
		    orig_width * 8 < out_width ||
		    orig_height * 8 < out_height) {
			omap_disp_put_dss();
			return -EINVAL;
		}
		set_upsampling_coef_table(plane);
	} else if (orig_width > out_width) {
		/* Downsampling not yet supported
		*/

		omap_disp_put_dss();
		return -EINVAL;
	}
	if (!orig_width || orig_width == out_width)
		fir_hinc = 0;
	else
		fir_hinc = 1024 * orig_width / out_width;
	if (!orig_height || orig_height == out_height)
		fir_vinc = 0;
	else
		fir_vinc = 1024 * orig_height / out_height;
	dispc.fir_hinc[plane] = fir_hinc;
	dispc.fir_vinc[plane] = fir_vinc;

	MOD_REG_FLD(fir_reg[plane],
		    FLD_MASK(16, 12) | FLD_MASK(0, 12),
		    ((fir_vinc & 4095) << 16) |
		    (fir_hinc & 4095));

	dev_dbg(dispc.fbdev->dev, "out_width %d out_height %d orig_width %d "
		"orig_height %d fir_hinc  %d fir_vinc %d\n",
		out_width, out_height, orig_width, orig_height,
		fir_hinc, fir_vinc);

	MOD_REG_FLD(vs_reg[plane],
		    FLD_MASK(16, 11) | FLD_MASK(0, 11),
		    ((out_height - 1) << 16) | (out_width - 1));

	l = dispc_read_reg(at_reg[plane]);
	l &= ~(0x03 << 5);
	l |= fir_hinc ? (1 << 5) : 0;
	l |= fir_vinc ? (1 << 6) : 0;
	dispc_write_reg(at_reg[plane], l);

	omap_disp_put_dss();
	return 0;
}

static int omap_dispc_enable_plane(int plane, int enable)
{
	const u32 at_reg[] = { DISPC_GFX_ATTRIBUTES,
				DISPC_VID_ATTRIBUTES(0),
				DISPC_VID_ATTRIBUTES(1) };
	if ((unsigned int)plane > dispc.mem_desc.region_cnt)
		return -EINVAL;

	omap_disp_get_dss();
	MOD_REG_FLD(at_reg[plane], 1, enable ? 1 : 0);
	omap_disp_put_dss();

	return 0;
}

static int omap_dispc_set_color_key(struct omapfb_color_key *ck)
{
	u32 df_reg, tr_reg;
	int shift, val;

	switch (ck->channel_out) {
	case OMAPFB_CHANNEL_OUT_LCD:
		df_reg = DISPC_DEFAULT_COLOR0;
		tr_reg = DISPC_TRANS_COLOR0;
		shift = 10;
		break;
	case OMAPFB_CHANNEL_OUT_DIGIT:
		df_reg = DISPC_DEFAULT_COLOR1;
		tr_reg = DISPC_TRANS_COLOR1;
		shift = 12;
		break;
	default:
		return -EINVAL;
	}
	switch (ck->key_type) {
	case OMAPFB_COLOR_KEY_DISABLED:
		val = 0;
		break;
	case OMAPFB_COLOR_KEY_GFX_DST:
		val = 1;
		break;
	case OMAPFB_COLOR_KEY_VID_SRC:
		val = 3;
		break;
	default:
		return -EINVAL;
	}
	omap_disp_get_dss();
	MOD_REG_FLD(DISPC_CONFIG, FLD_MASK(shift, 2), val << shift);

	if (val != 0)
		dispc_write_reg(tr_reg, ck->trans_key);
	dispc_write_reg(df_reg, ck->background);
	omap_disp_put_dss();

	dispc.color_key = *ck;

	return 0;
}

static int omap_dispc_get_color_key(struct omapfb_color_key *ck)
{
	*ck = dispc.color_key;
	return 0;
}

static void load_palette(void)
{
}

static int omap_dispc_set_update_mode(enum omapfb_update_mode mode)
{
	int r = 0;

	if (mode != dispc.update_mode) {
		switch (mode) {
		case OMAPFB_AUTO_UPDATE:
		case OMAPFB_MANUAL_UPDATE:
			omap_disp_get_dss();
			omap_dispc_enable_lcd_out(1);
			dispc.update_mode = mode;
			break;
		case OMAPFB_UPDATE_DISABLED:
			init_completion(&dispc.frame_done);
			omap_dispc_enable_lcd_out(0);
			if (!wait_for_completion_timeout(&dispc.frame_done,
					msecs_to_jiffies(500))) {
				dev_err(dispc.fbdev->dev,
					 "timeout waiting for FRAME DONE\n");
			}
			dispc.update_mode = mode;
			omap_disp_put_dss();
			break;
		default:
			r = -EINVAL;
		}
	}

	return r;
}

static void omap_dispc_get_caps(int plane, struct omapfb_caps *caps)
{
	caps->ctrl |= OMAPFB_CAPS_PLANE_RELOCATE_MEM;
	if (plane > 0)
		caps->ctrl |= OMAPFB_CAPS_PLANE_SCALE;
	caps->plane_color |= (1 << OMAPFB_COLOR_RGB565) |
			     (1 << OMAPFB_COLOR_YUV422) |
			     (1 << OMAPFB_COLOR_YUY422);
	if (plane == 0)
		caps->plane_color |= (1 << OMAPFB_COLOR_CLUT_8BPP) |
				     (1 << OMAPFB_COLOR_CLUT_4BPP) |
				     (1 << OMAPFB_COLOR_CLUT_2BPP) |
				     (1 << OMAPFB_COLOR_CLUT_1BPP) |
				     (1 << OMAPFB_COLOR_RGB444);
}

static enum omapfb_update_mode omap_dispc_get_update_mode(void)
{
	return dispc.update_mode;
}

static void setup_color_conv_coef(void)
{
	u32 mask = FLD_MASK(16, 11) | FLD_MASK(0, 11);
	int cf1_reg = DISPC_VID_CONV_COEF0(0);
	int cf2_reg = DISPC_VID_CONV_COEF0(1);
	int at1_reg = DISPC_VID_ATTRIBUTES(0);
	int at2_reg = DISPC_VID_ATTRIBUTES(1);
	const struct color_conv_coef {
		int  ry,  rcr,  rcb,   gy,  gcr,  gcb,   by,  bcr,  bcb;
		int  full_range;
	}  ctbl_bt601_5 = {
		    298,  409,    0,  298, -208, -100,  298,    0,  517, 0,
	};
	const struct color_conv_coef *ct;
#define CVAL(x, y)	(((x & 2047) << 16) | (y & 2047))

	ct = &ctbl_bt601_5;

	MOD_REG_FLD(cf1_reg,		mask,	CVAL(ct->rcr, ct->ry));
	MOD_REG_FLD(cf1_reg + 4,	mask,	CVAL(ct->gy,  ct->rcb));
	MOD_REG_FLD(cf1_reg + 8,	mask,	CVAL(ct->gcb, ct->gcr));
	MOD_REG_FLD(cf1_reg + 12,	mask,	CVAL(ct->bcr, ct->by));
	MOD_REG_FLD(cf1_reg + 16,	mask,	CVAL(0,	      ct->bcb));

	MOD_REG_FLD(cf2_reg,		mask,	CVAL(ct->rcr, ct->ry));
	MOD_REG_FLD(cf2_reg + 4,	mask,	CVAL(ct->gy,  ct->rcb));
	MOD_REG_FLD(cf2_reg + 8,	mask,	CVAL(ct->gcb, ct->gcr));
	MOD_REG_FLD(cf2_reg + 12,	mask,	CVAL(ct->bcr, ct->by));
	MOD_REG_FLD(cf2_reg + 16,	mask,	CVAL(0,	      ct->bcb));
#undef CVAL

	MOD_REG_FLD(at1_reg, (1 << 11), ct->full_range);
	MOD_REG_FLD(at2_reg, (1 << 11), ct->full_range);
}

static void calc_ck_div(int is_tft, int pck, int *lck_div, int *pck_div)
{
	unsigned long fck, lck;

	*lck_div = 1;
	pck = max(1, pck);
	fck = clk_get_rate(dispc.dss1_fck);
	lck = fck;
	*pck_div = (lck + pck - 1) / pck;
	if (is_tft)
		*pck_div = max(2, *pck_div);
	else
		*pck_div = max(3, *pck_div);
	if (*pck_div > 255) {
		*pck_div = 255;
		lck = pck * *pck_div;
		*lck_div = fck / lck;
		BUG_ON(*lck_div < 1);
		if (*lck_div > 255) {
			*lck_div = 255;
			dev_warn(dispc.fbdev->dev, "pixclock %d kHz too low.\n",
				 pck / 1000);
		}
	}
}

static void set_lcd_tft_mode(int enable)
{
	u32 mask;

	mask = 1 << 3;
	MOD_REG_FLD(DISPC_CONTROL, mask, enable ? mask : 0);
}

static void set_lcd_timings(void)
{
	u32 l;
	int lck_div, pck_div;
	struct lcd_panel *panel = dispc.fbdev->panel;
	int is_tft = panel->config & OMAP_LCDC_PANEL_TFT;
	unsigned long fck;

	l = dispc_read_reg(DISPC_TIMING_H);
	l &= ~(FLD_MASK(0, 6) | FLD_MASK(8, 8) | FLD_MASK(20, 8));
	l |= ( max(1, (min(64,  panel->hsw))) - 1 ) << 0;
	l |= ( max(1, (min(256, panel->hfp))) - 1 ) << 8;
	l |= ( max(1, (min(256, panel->hbp))) - 1 ) << 20;
	dispc_write_reg(DISPC_TIMING_H, l);

	l = dispc_read_reg(DISPC_TIMING_V);
	l &= ~(FLD_MASK(0, 6) | FLD_MASK(8, 8) | FLD_MASK(20, 8));
	l |= ( max(1, (min(64,  panel->vsw))) - 1 ) << 0;
	l |= ( max(0, (min(255, panel->vfp))) - 0 ) << 8;
	l |= ( max(0, (min(255, panel->vbp))) - 0 ) << 20;
	dispc_write_reg(DISPC_TIMING_V, l);

	l = dispc_read_reg(DISPC_POL_FREQ);
	l &= ~FLD_MASK(12, 6);
	l |= (panel->config & OMAP_LCDC_SIGNAL_MASK) << 12;
	l |= panel->acb & 0xff;
	dispc_write_reg(DISPC_POL_FREQ, l);

	calc_ck_div(is_tft, panel->pixel_clock * 1000, &lck_div, &pck_div);

	l = dispc_read_reg(DISPC_DIVISOR);
	l &= ~(FLD_MASK(16, 8) | FLD_MASK(0, 8));
	l |= (lck_div << 16) | (pck_div << 0);
	dispc_write_reg(DISPC_DIVISOR, l);

	/* update panel info with the exact clock */
	fck = clk_get_rate(dispc.dss1_fck);
	panel->pixel_clock = fck / lck_div / pck_div / 1000;
}

int omap_dispc_request_irq(void (*callback)(void *data), void *data)
{
	int r = 0;

	BUG_ON(callback == NULL);

	if (dispc.irq_callback)
		r = -EBUSY;
	else {
		dispc.irq_callback = callback;
		dispc.irq_callback_data = data;
	}

	return r;
}
EXPORT_SYMBOL(omap_dispc_request_irq);

void omap_dispc_enable_irqs(int irq_mask)
{
	omap_disp_get_dss();
	dispc.enabled_irqs = irq_mask;
	irq_mask |= DISPC_IRQ_MASK_ERROR;
	MOD_REG_FLD(DISPC_IRQENABLE, 0x7fff, irq_mask);
	omap_disp_put_dss();
}
EXPORT_SYMBOL(omap_dispc_enable_irqs);

void omap_dispc_disable_irqs(int irq_mask)
{
	omap_disp_get_dss();
	dispc.enabled_irqs &= ~irq_mask;
	irq_mask &= ~DISPC_IRQ_MASK_ERROR;
	MOD_REG_FLD(DISPC_IRQENABLE, 0x7fff, irq_mask);
	omap_disp_put_dss();
}
EXPORT_SYMBOL(omap_dispc_disable_irqs);

void omap_dispc_free_irq(void)
{
	omap_disp_get_dss();
	omap_dispc_disable_irqs(DISPC_IRQ_MASK_ALL);
	dispc.irq_callback = NULL;
	dispc.irq_callback_data = NULL;
	omap_disp_put_dss();
}
EXPORT_SYMBOL(omap_dispc_free_irq);

static irqreturn_t omap_dispc_irq_handler(int irq, void *dev)
{
	u32 stat = dispc_read_reg(DISPC_IRQSTATUS);

	if (stat & DISPC_IRQSTATUS_FRAMEDONE)
		complete(&dispc.frame_done);

	if (stat & DISPC_IRQ_MASK_ERROR) {
		if (printk_ratelimit()) {
			dev_err(dispc.fbdev->dev, "irq error status %04x\n",
				stat & 0x7fff);
		}
	}

	if ((stat & dispc.enabled_irqs) && dispc.irq_callback)
		dispc.irq_callback(dispc.irq_callback_data);

	dispc_write_reg(DISPC_IRQSTATUS, stat);

	return IRQ_HANDLED;
}

static void omap_dispc_suspend(void)
{
	if (dispc.update_mode == OMAPFB_AUTO_UPDATE) {
		init_completion(&dispc.frame_done);
		omap_dispc_enable_lcd_out(0);
		if (!wait_for_completion_timeout(&dispc.frame_done,
				msecs_to_jiffies(500))) {
			dev_err(dispc.fbdev->dev,
				"timeout waiting for FRAME DONE\n");
		}
		omap_disp_put_dss();
	}
}

static void omap_dispc_resume(void)
{
	if (dispc.update_mode == OMAPFB_AUTO_UPDATE) {
		omap_disp_get_dss();
		if (!dispc.ext_mode) {
			set_lcd_timings();
			load_palette();
		}
		omap_dispc_enable_lcd_out(1);
	}
}

static int omap_dispc_update_window(struct fb_info *fbi,
				 struct omapfb_update_window *win,
				 void (*complete_callback)(void *arg),
				 void *complete_callback_data)
{
	return dispc.update_mode == OMAPFB_UPDATE_DISABLED ? -ENODEV : 0;
}

static int mmap_kern(struct omapfb_mem_region *region)
{
	struct vm_struct	*kvma;
	struct vm_area_struct	vma;
	pgprot_t		pgprot;
	unsigned long		vaddr;

	kvma = get_vm_area(region->size, VM_IOREMAP);
	if (kvma == NULL) {
		dev_err(dispc.fbdev->dev, "can't get kernel vm area\n");
		return -ENOMEM;
	}
	vma.vm_mm = &init_mm;

	vaddr = (unsigned long)kvma->addr;

	pgprot = pgprot_writecombine(pgprot_kernel);
	vma.vm_start = vaddr;
	vma.vm_end = vaddr + region->size;
	if (io_remap_pfn_range(&vma, vaddr, region->paddr >> PAGE_SHIFT,
			   region->size, pgprot) < 0) {
		dev_err(dispc.fbdev->dev, "kernel mmap for FBMEM failed\n");
		return -EAGAIN;
	}
	region->vaddr = (void *)vaddr;

	return 0;
}

static void mmap_user_open(struct vm_area_struct *vma)
{
	int plane = (int)vma->vm_private_data;

	atomic_inc(&dispc.map_count[plane]);
}

static void mmap_user_close(struct vm_area_struct *vma)
{
	int plane = (int)vma->vm_private_data;

	atomic_dec(&dispc.map_count[plane]);
}

static struct vm_operations_struct mmap_user_ops = {
	.open = mmap_user_open,
	.close = mmap_user_close,
};

static int omap_dispc_mmap_user(struct fb_info *info,
				struct vm_area_struct *vma)
{
	struct omapfb_plane_struct *plane = info->par;
	unsigned long off;
	unsigned long start;
	u32 len;

	if (vma->vm_end - vma->vm_start == 0)
		return 0;
	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;
	off = vma->vm_pgoff << PAGE_SHIFT;

	start = info->fix.smem_start;
	len = info->fix.smem_len;
	if (off >= len)
		return -EINVAL;
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_ops = &mmap_user_ops;
	vma->vm_private_data = (void *)plane->idx;
	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			     vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;
	/* vm_ops.open won't be called for mmap itself. */
	atomic_inc(&dispc.map_count[plane->idx]);
	return 0;
}

static void unmap_kern(struct omapfb_mem_region *region)
{
	vunmap(region->vaddr);
}

static int alloc_palette_ram(void)
{
	dispc.palette_vaddr = dma_alloc_writecombine(dispc.fbdev->dev,
		MAX_PALETTE_SIZE, &dispc.palette_paddr, GFP_KERNEL);
	if (dispc.palette_vaddr == NULL) {
		dev_err(dispc.fbdev->dev, "failed to alloc palette memory\n");
		return -ENOMEM;
	}

	return 0;
}

static void free_palette_ram(void)
{
	dma_free_writecombine(dispc.fbdev->dev, MAX_PALETTE_SIZE,
			dispc.palette_vaddr, dispc.palette_paddr);
}

static int alloc_fbmem(struct omapfb_mem_region *region)
{
	region->vaddr = dma_alloc_writecombine(dispc.fbdev->dev,
			region->size, &region->paddr, GFP_KERNEL);

	if (region->vaddr == NULL) {
		dev_err(dispc.fbdev->dev, "unable to allocate FB DMA memory\n");
		return -ENOMEM;
	}

	return 0;
}

static void free_fbmem(struct omapfb_mem_region *region)
{
	dma_free_writecombine(dispc.fbdev->dev, region->size,
			      region->vaddr, region->paddr);
}

static struct resmap *init_resmap(unsigned long start, size_t size)
{
	unsigned page_cnt;
	struct resmap *res_map;

	page_cnt = PAGE_ALIGN(size) / PAGE_SIZE;
	res_map =
	    kzalloc(sizeof(struct resmap) + RESMAP_SIZE(page_cnt), GFP_KERNEL);
	if (res_map == NULL)
		return NULL;
	res_map->start = start;
	res_map->page_cnt = page_cnt;
	res_map->map = (unsigned long *)(res_map + 1);
	return res_map;
}

static void cleanup_resmap(struct resmap *res_map)
{
	kfree(res_map);
}

static inline int resmap_mem_type(unsigned long start)
{
	if (start >= OMAP2_SRAM_START &&
	    start < OMAP2_SRAM_START + OMAP2_SRAM_SIZE)
		return OMAPFB_MEMTYPE_SRAM;
	else
		return OMAPFB_MEMTYPE_SDRAM;
}

static inline int resmap_page_reserved(struct resmap *res_map, unsigned page_nr)
{
	return *RESMAP_PTR(res_map, page_nr) & RESMAP_MASK(page_nr) ? 1 : 0;
}

static inline void resmap_reserve_page(struct resmap *res_map, unsigned page_nr)
{
	BUG_ON(resmap_page_reserved(res_map, page_nr));
	*RESMAP_PTR(res_map, page_nr) |= RESMAP_MASK(page_nr);
}

static inline void resmap_free_page(struct resmap *res_map, unsigned page_nr)
{
	BUG_ON(!resmap_page_reserved(res_map, page_nr));
	*RESMAP_PTR(res_map, page_nr) &= ~RESMAP_MASK(page_nr);
}

static void resmap_reserve_region(unsigned long start, size_t size)
{

	struct resmap	*res_map;
	unsigned	start_page;
	unsigned	end_page;
	int		mtype;
	unsigned	i;

	mtype = resmap_mem_type(start);
	res_map = dispc.res_map[mtype];
	dev_dbg(dispc.fbdev->dev, "reserve mem type %d start %08lx size %d\n",
		mtype, start, size);
	start_page = (start - res_map->start) / PAGE_SIZE;
	end_page = start_page + PAGE_ALIGN(size) / PAGE_SIZE;
	for (i = start_page; i < end_page; i++)
		resmap_reserve_page(res_map, i);
}

static void resmap_free_region(unsigned long start, size_t size)
{
	struct resmap	*res_map;
	unsigned	start_page;
	unsigned	end_page;
	unsigned	i;
	int		mtype;

	mtype = resmap_mem_type(start);
	res_map = dispc.res_map[mtype];
	dev_dbg(dispc.fbdev->dev, "free mem type %d start %08lx size %d\n",
		mtype, start, size);
	start_page = (start - res_map->start) / PAGE_SIZE;
	end_page = start_page + PAGE_ALIGN(size) / PAGE_SIZE;
	for (i = start_page; i < end_page; i++)
		resmap_free_page(res_map, i);
}

static unsigned long resmap_alloc_region(int mtype, size_t size)
{
	unsigned i;
	unsigned total;
	unsigned start_page;
	unsigned long start;
	struct resmap *res_map = dispc.res_map[mtype];

	BUG_ON(mtype >= DISPC_MEMTYPE_NUM || res_map == NULL || !size);

	size = PAGE_ALIGN(size) / PAGE_SIZE;
	start_page = 0;
	total = 0;
	for (i = 0; i < res_map->page_cnt; i++) {
		if (resmap_page_reserved(res_map, i)) {
			start_page = i + 1;
			total = 0;
		} else if (++total == size)
			break;
	}
	if (total < size)
		return 0;

	start = res_map->start + start_page * PAGE_SIZE;
	resmap_reserve_region(start, size * PAGE_SIZE);

	return start;
}

/* Note that this will only work for user mappings, we don't deal with
 * kernel mappings here, so fbcon will keep using the old region.
 */
static int omap_dispc_setup_mem(int plane, size_t size, int mem_type,
				unsigned long *paddr)
{
	struct omapfb_mem_region *rg;
	unsigned long new_addr = 0;

	if ((unsigned)plane > dispc.mem_desc.region_cnt)
		return -EINVAL;
	if (mem_type >= DISPC_MEMTYPE_NUM)
		return -EINVAL;
	if (dispc.res_map[mem_type] == NULL)
		return -ENOMEM;
	rg = &dispc.mem_desc.region[plane];
	if (size == rg->size && mem_type == rg->type)
		return 0;
	if (atomic_read(&dispc.map_count[plane]))
		return -EBUSY;
	if (rg->size != 0)
		resmap_free_region(rg->paddr, rg->size);
	if (size != 0) {
		new_addr = resmap_alloc_region(mem_type, size);
		if (!new_addr) {
			/* Reallocate old region. */
			resmap_reserve_region(rg->paddr, rg->size);
			return -ENOMEM;
		}
	}
	rg->paddr = new_addr;
	rg->size = size;
	rg->type = mem_type;

	*paddr = new_addr;

	return 0;
}

static int setup_fbmem(struct omapfb_mem_desc *req_md)
{
	struct omapfb_mem_region	*rg;
	int i;
	int r;
	unsigned long			mem_start[DISPC_MEMTYPE_NUM];
	unsigned long			mem_end[DISPC_MEMTYPE_NUM];

	if (!req_md->region_cnt) {
		dev_err(dispc.fbdev->dev, "no memory regions defined\n");
		return -ENOENT;
	}

	rg = &req_md->region[0];
	memset(mem_start, 0xff, sizeof(mem_start));
	memset(mem_end, 0, sizeof(mem_end));

	for (i = 0; i < req_md->region_cnt; i++, rg++) {
		int mtype;
		if (rg->paddr) {
			rg->alloc = 0;
			if (rg->vaddr == NULL) {
				rg->map = 1;
				if ((r = mmap_kern(rg)) < 0)
					return r;
			}
		} else {
			if (rg->type != OMAPFB_MEMTYPE_SDRAM) {
				dev_err(dispc.fbdev->dev,
					"unsupported memory type\n");
				return -EINVAL;
			}
			rg->alloc = rg->map = 1;
			if ((r = alloc_fbmem(rg)) < 0)
				return r;
		}
		mtype = rg->type;

		if (rg->paddr < mem_start[mtype])
			mem_start[mtype] = rg->paddr;
		if (rg->paddr + rg->size > mem_end[mtype])
			mem_end[mtype] = rg->paddr + rg->size;
	}

	for (i = 0; i < DISPC_MEMTYPE_NUM; i++) {
		unsigned long start;
		size_t size;
		if (mem_end[i] == 0)
			continue;
		start = mem_start[i];
		size = mem_end[i] - start;
		dispc.res_map[i] = init_resmap(start, size);
		r = -ENOMEM;
		if (dispc.res_map[i] == NULL)
			goto fail;
		/* Initial state is that everything is reserved. This
		 * includes possible holes as well, which will never be
		 * freed.
		 */
		resmap_reserve_region(start, size);
	}

	dispc.mem_desc = *req_md;

	return 0;
fail:
	for (i = 0; i < DISPC_MEMTYPE_NUM; i++) {
		if (dispc.res_map[i] != NULL)
			cleanup_resmap(dispc.res_map[i]);
	}
	return r;
}

static void cleanup_fbmem(void)
{
	struct omapfb_mem_region *rg;
	int i;

	for (i = 0; i < DISPC_MEMTYPE_NUM; i++) {
		if (dispc.res_map[i] != NULL)
			cleanup_resmap(dispc.res_map[i]);
	}
	rg = &dispc.mem_desc.region[0];
	for (i = 0; i < dispc.mem_desc.region_cnt; i++, rg++) {
		if (rg->alloc)
			free_fbmem(rg);
		else {
			if (rg->map)
				unmap_kern(rg);
		}
	}
}

static int omap_dispc_init(struct omapfb_device *fbdev, int ext_mode,
			   struct omapfb_mem_desc *req_vram)
{
	int r = 0;
	u32 l;
	struct lcd_panel *panel = fbdev->panel;
	int tmo = 10000;
	int skip_init = 0;
	int i;

	memset(&dispc, 0, sizeof(dispc));

	dispc.base = io_p2v(DISPC_BASE);
	dispc.fbdev = fbdev;
	dispc.ext_mode = ext_mode;

	init_completion(&dispc.frame_done);
	omap_disp_get_dss();
	dispc.dss_ick = clk_get(fbdev->dev, "dss_ick");
	dispc.dss1_fck = clk_get(fbdev->dev, cpu_is_omap34xx() ?
					"dss1_alwon_fck" : "dss1_fck");
	dispc.dss_54m_fck = clk_get(fbdev->dev, cpu_is_omap34xx() ?
					"dss_tv_fck" : "dss_54m_fck");

	l = dispc_read_reg(DISPC_CONTROL);
	/* LCD enabled ? */
	if ((l & 1) || (l & 0x2)) {
		pr_info("omapfb: skipping hardware initialization\n");
		skip_init = 1;
	}

	if (!skip_init) {
		/* Reset monitoring works only w/ the 54M clk */
		/* Soft reset */
		MOD_REG_FLD(DISPC_SYSCONFIG, 1 << 1, 1 << 1);

		while (!(dispc_read_reg(DISPC_SYSSTATUS) & 1)) {
			if (!--tmo) {
				dev_err(dispc.fbdev->dev, "soft reset failed\n");
				r = -ENODEV;
				goto fail1;
			}
		}
	}

	/* Enable smart idle and autoidle */
	l = dispc_read_reg(DISPC_SYSCONFIG);
	l &= ~((3 << 12) | (3 << 3));
	l |= (2 << 12) | (2 << 3) | (1 << 2) | (1 << 0);
	dispc_write_reg(DISPC_SYSCONFIG, l);

	/* Set functional clock autogating */
	l = dispc_read_reg(DISPC_CONFIG);
	l |= 1 << 9;
	dispc_write_reg(DISPC_CONFIG, l);

	l = dispc_read_reg(DISPC_IRQSTATUS);
	dispc_write_reg(DISPC_IRQSTATUS, l);

	/* Enable those that we handle always */
	if (omap_disp_register_isr((omap_disp_isr_t)omap_dispc_irq_handler,
					fbdev, DISPC_IRQENABLE_FRAMEDONE |
					DISPC_IRQSTATUS_GFXFIFOUNDERFLOW)) {
		dev_err(dispc.fbdev->dev, "can't get DSS IRQ\n");
		goto fail1;
	}

	/* L3 firewall setting: enable access to OCM RAM */
	__raw_writel(0x402000b0, io_p2v(0x680050a0));

	if ((r = alloc_palette_ram()) < 0)
		goto fail2;

	if ((r = setup_fbmem(req_vram)) < 0)
		goto fail3;

		for (i = 0; i < dispc.mem_desc.region_cnt; i++) {
			memset(dispc.mem_desc.region[i].vaddr, 0,
				dispc.mem_desc.region[i].size);
		}

		/* Set logic clock to fck, pixel clock to fck/2 for now */
		MOD_REG_FLD(DISPC_DIVISOR, FLD_MASK(16, 8), 1 << 16);
		MOD_REG_FLD(DISPC_DIVISOR, FLD_MASK(0, 8), 2 << 0);

		setup_plane_fifo(0, ext_mode);
		set_lcd_tft_mode(panel->config & OMAP_LCDC_PANEL_TFT);
		set_load_mode(DISPC_LOAD_FRAME_ONLY);

		if (!ext_mode) {
			set_lcd_data_lines(panel->data_lines);
			omap_dispc_set_lcd_size(panel->x_res, panel->y_res);
			set_lcd_timings();
		} else
			set_lcd_data_lines(panel->bpp);
		enable_rfbi_mode(ext_mode);

	l = dispc_read_reg(DISPC_REVISION);
	pr_info("omapfb: DISPC version %d.%d initialized\n",
		 l >> 4 & 0x0f, l & 0x0f);

	return r;
fail3:
	free_palette_ram();
fail2:
	free_irq(INT_24XX_DSS_IRQ, fbdev);
fail1:
	omap_disp_put_dss();
	return r;
}

static void omap_dispc_cleanup(void)
{
	int i;

	omap_dispc_set_update_mode(OMAPFB_UPDATE_DISABLED);
	/* This will also disable clocks that are on */
	for (i = 0; i < dispc.mem_desc.region_cnt; i++)
		omap_dispc_enable_plane(i, 0);
	cleanup_fbmem();
	free_palette_ram();
	free_irq(INT_24XX_DSS_IRQ, dispc.fbdev);
	omap_disp_put_dss();
}

const struct lcd_ctrl omap2_int_ctrl = {
	.name			= "internal",
	.init			= omap_dispc_init,
	.cleanup		= omap_dispc_cleanup,
	.get_caps		= omap_dispc_get_caps,
	.set_update_mode	= omap_dispc_set_update_mode,
	.get_update_mode	= omap_dispc_get_update_mode,
	.update_window		= omap_dispc_update_window,
	.suspend		= omap_dispc_suspend,
	.resume			= omap_dispc_resume,
	.setup_plane		= omap_dispc_setup_plane,
	.setup_mem		= omap_dispc_setup_mem,
	.set_scale		= omap_dispc_set_scale,
	.enable_plane		= omap_dispc_enable_plane,
	.set_color_key		= omap_dispc_set_color_key,
	.get_color_key		= omap_dispc_get_color_key,
	.mmap			= omap_dispc_mmap_user,
};
