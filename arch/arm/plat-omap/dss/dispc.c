/*
 * linux/arch/arm/plat-omap/dss/dispc.c
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

#define DSS_SUBSYS_NAME "DISPC"

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/jiffies.h>

#include <mach/sram.h>
#include <mach/board.h>
#include <mach/clock.h>

#include <mach/display.h>

#include "dss.h"

/* DISPC */
#define DISPC_BASE			0x48050400

#define DISPC_SZ_REGS			SZ_1K

struct dispc_reg { u16 idx; };

#define DISPC_REG(idx)			((const struct dispc_reg) { idx })

/* DISPC common */
#define DISPC_REVISION			DISPC_REG(0x0000)
#define DISPC_SYSCONFIG			DISPC_REG(0x0010)
#define DISPC_SYSSTATUS			DISPC_REG(0x0014)
#define DISPC_IRQSTATUS			DISPC_REG(0x0018)
#define DISPC_IRQENABLE			DISPC_REG(0x001C)
#define DISPC_CONTROL			DISPC_REG(0x0040)
#define DISPC_CONFIG			DISPC_REG(0x0044)
#define DISPC_CAPABLE			DISPC_REG(0x0048)
#define DISPC_DEFAULT_COLOR0		DISPC_REG(0x004C)
#define DISPC_DEFAULT_COLOR1		DISPC_REG(0x0050)
#define DISPC_TRANS_COLOR0		DISPC_REG(0x0054)
#define DISPC_TRANS_COLOR1		DISPC_REG(0x0058)
#define DISPC_LINE_STATUS		DISPC_REG(0x005C)
#define DISPC_LINE_NUMBER		DISPC_REG(0x0060)
#define DISPC_TIMING_H			DISPC_REG(0x0064)
#define DISPC_TIMING_V			DISPC_REG(0x0068)
#define DISPC_POL_FREQ			DISPC_REG(0x006C)
#define DISPC_DIVISOR			DISPC_REG(0x0070)
#define DISPC_GLOBAL_ALPHA		DISPC_REG(0x0074)
#define DISPC_SIZE_DIG			DISPC_REG(0x0078)
#define DISPC_SIZE_LCD			DISPC_REG(0x007C)

/* DISPC GFX plane */
#define DISPC_GFX_BA0			DISPC_REG(0x0080)
#define DISPC_GFX_BA1			DISPC_REG(0x0084)
#define DISPC_GFX_POSITION		DISPC_REG(0x0088)
#define DISPC_GFX_SIZE			DISPC_REG(0x008C)
#define DISPC_GFX_ATTRIBUTES		DISPC_REG(0x00A0)
#define DISPC_GFX_FIFO_THRESHOLD	DISPC_REG(0x00A4)
#define DISPC_GFX_FIFO_SIZE_STATUS	DISPC_REG(0x00A8)
#define DISPC_GFX_ROW_INC		DISPC_REG(0x00AC)
#define DISPC_GFX_PIXEL_INC		DISPC_REG(0x00B0)
#define DISPC_GFX_WINDOW_SKIP		DISPC_REG(0x00B4)
#define DISPC_GFX_TABLE_BA		DISPC_REG(0x00B8)

#define DISPC_DATA_CYCLE1		DISPC_REG(0x01D4)
#define DISPC_DATA_CYCLE2		DISPC_REG(0x01D8)
#define DISPC_DATA_CYCLE3		DISPC_REG(0x01DC)

#define DISPC_CPR_COEF_R		DISPC_REG(0x0220)
#define DISPC_CPR_COEF_G		DISPC_REG(0x0224)
#define DISPC_CPR_COEF_B		DISPC_REG(0x0228)

#define DISPC_GFX_PRELOAD		DISPC_REG(0x022C)

/* DISPC Video plane, n = 0 for VID1 and n = 1 for VID2 */
#define DISPC_VID_REG(n, idx)		DISPC_REG(0x00BC + (n)*0x90 + idx)

#define DISPC_VID_BA0(n)		DISPC_VID_REG(n, 0x0000)
#define DISPC_VID_BA1(n)		DISPC_VID_REG(n, 0x0004)
#define DISPC_VID_POSITION(n)		DISPC_VID_REG(n, 0x0008)
#define DISPC_VID_SIZE(n)		DISPC_VID_REG(n, 0x000C)
#define DISPC_VID_ATTRIBUTES(n)		DISPC_VID_REG(n, 0x0010)
#define DISPC_VID_FIFO_THRESHOLD(n)	DISPC_VID_REG(n, 0x0014)
#define DISPC_VID_FIFO_SIZE_STATUS(n)	DISPC_VID_REG(n, 0x0018)
#define DISPC_VID_ROW_INC(n)		DISPC_VID_REG(n, 0x001C)
#define DISPC_VID_PIXEL_INC(n)		DISPC_VID_REG(n, 0x0020)
#define DISPC_VID_FIR(n)		DISPC_VID_REG(n, 0x0024)
#define DISPC_VID_PICTURE_SIZE(n)	DISPC_VID_REG(n, 0x0028)
#define DISPC_VID_ACCU0(n)		DISPC_VID_REG(n, 0x002C)
#define DISPC_VID_ACCU1(n)		DISPC_VID_REG(n, 0x0030)

/* coef index i = {0, 1, 2, 3, 4, 5, 6, 7} */
#define DISPC_VID_FIR_COEF_H(n, i)	DISPC_REG(0x00F0 + (n)*0x90 + (i)*0x8)
/* coef index i = {0, 1, 2, 3, 4, 5, 6, 7} */
#define DISPC_VID_FIR_COEF_HV(n, i)	DISPC_REG(0x00F4 + (n)*0x90 + (i)*0x8)
/* coef index i = {0, 1, 2, 3, 4} */
#define DISPC_VID_CONV_COEF(n, i)	DISPC_REG(0x0130 + (n)*0x90 + (i)*0x4)
/* coef index i = {0, 1, 2, 3, 4, 5, 6, 7} */
#define DISPC_VID_FIR_COEF_V(n, i)	DISPC_REG(0x01E0 + (n)*0x20 + (i)*0x4)

#define DISPC_VID_PRELOAD(n)		DISPC_REG(0x230 + (n)*0x04)

#define DISPC_IRQ_MASK_ERROR            (DISPC_IRQ_GFX_FIFO_UNDERFLOW | \
					 DISPC_IRQ_OCP_ERR | \
					 DISPC_IRQ_VID1_FIFO_UNDERFLOW | \
					 DISPC_IRQ_VID2_FIFO_UNDERFLOW | \
					 DISPC_IRQ_SYNC_LOST | \
					 DISPC_IRQ_SYNC_LOST_DIGIT)

#define DISPC_MAX_NR_ISRS		8

static struct {
	omap_dispc_isr_t	isr;
	void			*arg;
	u32			mask;
} registered_isr[DISPC_MAX_NR_ISRS];

#define REG_GET(idx, start, end) \
	FLD_GET(dispc_read_reg(idx), start, end)

#define REG_FLD_MOD(idx, val, start, end)				\
	dispc_write_reg(idx, FLD_MOD(dispc_read_reg(idx), val, start, end))

static const struct dispc_reg dispc_reg_att[] = { DISPC_GFX_ATTRIBUTES,
	DISPC_VID_ATTRIBUTES(0),
	DISPC_VID_ATTRIBUTES(1) };

static struct {
	void __iomem    *base;

	struct clk	*dpll4_m4_ck;

	spinlock_t	irq_lock;

	unsigned long	cache_req_pck;
	unsigned long	cache_prate;
	struct dispc_clock_info cache_cinfo;

	u32		ctx[DISPC_SZ_REGS / sizeof(u32)];
} dispc;

static inline void dispc_write_reg(const struct dispc_reg idx, u32 val)
{
	__raw_writel(val, dispc.base + idx.idx);
}

static inline u32 dispc_read_reg(const struct dispc_reg idx)
{
	return __raw_readl(dispc.base + idx.idx);
}

#define SR(reg) \
	dispc.ctx[(DISPC_##reg).idx / sizeof(u32)] = dispc_read_reg(DISPC_##reg)
#define RR(reg) \
	dispc_write_reg(DISPC_##reg, dispc.ctx[(DISPC_##reg).idx / sizeof(u32)])

void dispc_save_context(void)
{
	if (cpu_is_omap24xx())
		return;

	SR(SYSCONFIG);
	SR(IRQENABLE);
	SR(CONTROL);
	SR(CONFIG);
	SR(DEFAULT_COLOR0);
	SR(DEFAULT_COLOR1);
	SR(TRANS_COLOR0);
	SR(TRANS_COLOR1);
	SR(LINE_NUMBER);
	SR(TIMING_H);
	SR(TIMING_V);
	SR(POL_FREQ);
	SR(DIVISOR);
	SR(GLOBAL_ALPHA);
	SR(SIZE_DIG);
	SR(SIZE_LCD);

	SR(GFX_BA0);
	SR(GFX_BA1);
	SR(GFX_POSITION);
	SR(GFX_SIZE);
	SR(GFX_ATTRIBUTES);
	SR(GFX_FIFO_THRESHOLD);
	SR(GFX_ROW_INC);
	SR(GFX_PIXEL_INC);
	SR(GFX_WINDOW_SKIP);
	SR(GFX_TABLE_BA);

	SR(DATA_CYCLE1);
	SR(DATA_CYCLE2);
	SR(DATA_CYCLE3);

	SR(CPR_COEF_R);
	SR(CPR_COEF_G);
	SR(CPR_COEF_B);

	SR(GFX_PRELOAD);

	/* VID1 */
	SR(VID_BA0(0));
	SR(VID_BA1(0));
	SR(VID_POSITION(0));
	SR(VID_SIZE(0));
	SR(VID_ATTRIBUTES(0));
	SR(VID_FIFO_THRESHOLD(0));
	SR(VID_ROW_INC(0));
	SR(VID_PIXEL_INC(0));
	SR(VID_FIR(0));
	SR(VID_PICTURE_SIZE(0));
	SR(VID_ACCU0(0));
	SR(VID_ACCU1(0));

	SR(VID_FIR_COEF_H(0, 0));
	SR(VID_FIR_COEF_H(0, 1));
	SR(VID_FIR_COEF_H(0, 2));
	SR(VID_FIR_COEF_H(0, 3));
	SR(VID_FIR_COEF_H(0, 4));
	SR(VID_FIR_COEF_H(0, 5));
	SR(VID_FIR_COEF_H(0, 6));
	SR(VID_FIR_COEF_H(0, 7));

	SR(VID_FIR_COEF_HV(0, 0));
	SR(VID_FIR_COEF_HV(0, 1));
	SR(VID_FIR_COEF_HV(0, 2));
	SR(VID_FIR_COEF_HV(0, 3));
	SR(VID_FIR_COEF_HV(0, 4));
	SR(VID_FIR_COEF_HV(0, 5));
	SR(VID_FIR_COEF_HV(0, 6));
	SR(VID_FIR_COEF_HV(0, 7));

	SR(VID_CONV_COEF(0, 0));
	SR(VID_CONV_COEF(0, 1));
	SR(VID_CONV_COEF(0, 2));
	SR(VID_CONV_COEF(0, 3));
	SR(VID_CONV_COEF(0, 4));

	SR(VID_FIR_COEF_V(0, 0));
	SR(VID_FIR_COEF_V(0, 1));
	SR(VID_FIR_COEF_V(0, 2));
	SR(VID_FIR_COEF_V(0, 3));
	SR(VID_FIR_COEF_V(0, 4));
	SR(VID_FIR_COEF_V(0, 5));
	SR(VID_FIR_COEF_V(0, 6));
	SR(VID_FIR_COEF_V(0, 7));

	SR(VID_PRELOAD(0));

	/* VID2 */
	SR(VID_BA0(1));
	SR(VID_BA1(1));
	SR(VID_POSITION(1));
	SR(VID_SIZE(1));
	SR(VID_ATTRIBUTES(1));
	SR(VID_FIFO_THRESHOLD(1));
	SR(VID_ROW_INC(1));
	SR(VID_PIXEL_INC(1));
	SR(VID_FIR(1));
	SR(VID_PICTURE_SIZE(1));
	SR(VID_ACCU0(1));
	SR(VID_ACCU1(1));

	SR(VID_FIR_COEF_H(1, 0));
	SR(VID_FIR_COEF_H(1, 1));
	SR(VID_FIR_COEF_H(1, 2));
	SR(VID_FIR_COEF_H(1, 3));
	SR(VID_FIR_COEF_H(1, 4));
	SR(VID_FIR_COEF_H(1, 5));
	SR(VID_FIR_COEF_H(1, 6));
	SR(VID_FIR_COEF_H(1, 7));

	SR(VID_FIR_COEF_HV(1, 0));
	SR(VID_FIR_COEF_HV(1, 1));
	SR(VID_FIR_COEF_HV(1, 2));
	SR(VID_FIR_COEF_HV(1, 3));
	SR(VID_FIR_COEF_HV(1, 4));
	SR(VID_FIR_COEF_HV(1, 5));
	SR(VID_FIR_COEF_HV(1, 6));
	SR(VID_FIR_COEF_HV(1, 7));

	SR(VID_CONV_COEF(1, 0));
	SR(VID_CONV_COEF(1, 1));
	SR(VID_CONV_COEF(1, 2));
	SR(VID_CONV_COEF(1, 3));
	SR(VID_CONV_COEF(1, 4));

	SR(VID_FIR_COEF_V(1, 0));
	SR(VID_FIR_COEF_V(1, 1));
	SR(VID_FIR_COEF_V(1, 2));
	SR(VID_FIR_COEF_V(1, 3));
	SR(VID_FIR_COEF_V(1, 4));
	SR(VID_FIR_COEF_V(1, 5));
	SR(VID_FIR_COEF_V(1, 6));
	SR(VID_FIR_COEF_V(1, 7));

	SR(VID_PRELOAD(1));
}

void dispc_restore_context(void)
{
	RR(SYSCONFIG);
	RR(IRQENABLE);
	/*RR(CONTROL);*/
	RR(CONFIG);
	RR(DEFAULT_COLOR0);
	RR(DEFAULT_COLOR1);
	RR(TRANS_COLOR0);
	RR(TRANS_COLOR1);
	RR(LINE_NUMBER);
	RR(TIMING_H);
	RR(TIMING_V);
	RR(POL_FREQ);
	RR(DIVISOR);
	RR(GLOBAL_ALPHA);
	RR(SIZE_DIG);
	RR(SIZE_LCD);

	RR(GFX_BA0);
	RR(GFX_BA1);
	RR(GFX_POSITION);
	RR(GFX_SIZE);
	RR(GFX_ATTRIBUTES);
	RR(GFX_FIFO_THRESHOLD);
	RR(GFX_ROW_INC);
	RR(GFX_PIXEL_INC);
	RR(GFX_WINDOW_SKIP);
	RR(GFX_TABLE_BA);

	RR(DATA_CYCLE1);
	RR(DATA_CYCLE2);
	RR(DATA_CYCLE3);

	RR(CPR_COEF_R);
	RR(CPR_COEF_G);
	RR(CPR_COEF_B);

	RR(GFX_PRELOAD);

	/* VID1 */
	RR(VID_BA0(0));
	RR(VID_BA1(0));
	RR(VID_POSITION(0));
	RR(VID_SIZE(0));
	RR(VID_ATTRIBUTES(0));
	RR(VID_FIFO_THRESHOLD(0));
	RR(VID_ROW_INC(0));
	RR(VID_PIXEL_INC(0));
	RR(VID_FIR(0));
	RR(VID_PICTURE_SIZE(0));
	RR(VID_ACCU0(0));
	RR(VID_ACCU1(0));

	RR(VID_FIR_COEF_H(0, 0));
	RR(VID_FIR_COEF_H(0, 1));
	RR(VID_FIR_COEF_H(0, 2));
	RR(VID_FIR_COEF_H(0, 3));
	RR(VID_FIR_COEF_H(0, 4));
	RR(VID_FIR_COEF_H(0, 5));
	RR(VID_FIR_COEF_H(0, 6));
	RR(VID_FIR_COEF_H(0, 7));

	RR(VID_FIR_COEF_HV(0, 0));
	RR(VID_FIR_COEF_HV(0, 1));
	RR(VID_FIR_COEF_HV(0, 2));
	RR(VID_FIR_COEF_HV(0, 3));
	RR(VID_FIR_COEF_HV(0, 4));
	RR(VID_FIR_COEF_HV(0, 5));
	RR(VID_FIR_COEF_HV(0, 6));
	RR(VID_FIR_COEF_HV(0, 7));

	RR(VID_CONV_COEF(0, 0));
	RR(VID_CONV_COEF(0, 1));
	RR(VID_CONV_COEF(0, 2));
	RR(VID_CONV_COEF(0, 3));
	RR(VID_CONV_COEF(0, 4));

	RR(VID_FIR_COEF_V(0, 0));
	RR(VID_FIR_COEF_V(0, 1));
	RR(VID_FIR_COEF_V(0, 2));
	RR(VID_FIR_COEF_V(0, 3));
	RR(VID_FIR_COEF_V(0, 4));
	RR(VID_FIR_COEF_V(0, 5));
	RR(VID_FIR_COEF_V(0, 6));
	RR(VID_FIR_COEF_V(0, 7));

	RR(VID_PRELOAD(0));

	/* VID2 */
	RR(VID_BA0(1));
	RR(VID_BA1(1));
	RR(VID_POSITION(1));
	RR(VID_SIZE(1));
	RR(VID_ATTRIBUTES(1));
	RR(VID_FIFO_THRESHOLD(1));
	RR(VID_ROW_INC(1));
	RR(VID_PIXEL_INC(1));
	RR(VID_FIR(1));
	RR(VID_PICTURE_SIZE(1));
	RR(VID_ACCU0(1));
	RR(VID_ACCU1(1));

	RR(VID_FIR_COEF_H(1, 0));
	RR(VID_FIR_COEF_H(1, 1));
	RR(VID_FIR_COEF_H(1, 2));
	RR(VID_FIR_COEF_H(1, 3));
	RR(VID_FIR_COEF_H(1, 4));
	RR(VID_FIR_COEF_H(1, 5));
	RR(VID_FIR_COEF_H(1, 6));
	RR(VID_FIR_COEF_H(1, 7));

	RR(VID_FIR_COEF_HV(1, 0));
	RR(VID_FIR_COEF_HV(1, 1));
	RR(VID_FIR_COEF_HV(1, 2));
	RR(VID_FIR_COEF_HV(1, 3));
	RR(VID_FIR_COEF_HV(1, 4));
	RR(VID_FIR_COEF_HV(1, 5));
	RR(VID_FIR_COEF_HV(1, 6));
	RR(VID_FIR_COEF_HV(1, 7));

	RR(VID_CONV_COEF(1, 0));
	RR(VID_CONV_COEF(1, 1));
	RR(VID_CONV_COEF(1, 2));
	RR(VID_CONV_COEF(1, 3));
	RR(VID_CONV_COEF(1, 4));

	RR(VID_FIR_COEF_V(1, 0));
	RR(VID_FIR_COEF_V(1, 1));
	RR(VID_FIR_COEF_V(1, 2));
	RR(VID_FIR_COEF_V(1, 3));
	RR(VID_FIR_COEF_V(1, 4));
	RR(VID_FIR_COEF_V(1, 5));
	RR(VID_FIR_COEF_V(1, 6));
	RR(VID_FIR_COEF_V(1, 7));

	RR(VID_PRELOAD(1));

	/* enable last, because LCD & DIGIT enable are here */
	RR(CONTROL);
}

#undef SR
#undef RR

static inline void enable_clocks(int enable)
{
	if (enable)
		dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);
	else
		dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
}

void dispc_go(enum omap_channel channel)
{
	int bit;
	unsigned long tmo;

	enable_clocks(1);

	if (channel == OMAP_DSS_CHANNEL_LCD)
		bit = 0; /* LCDENABLE */
	else
		bit = 1; /* DIGITALENABLE */

	/* if the channel is not enabled, we don't need GO */
	if (REG_GET(DISPC_CONTROL, bit, bit) == 0)
		goto end;

	if (channel == OMAP_DSS_CHANNEL_LCD)
		bit = 5; /* GOLCD */
	else
		bit = 6; /* GODIGIT */

	tmo = jiffies + msecs_to_jiffies(200);
	while (REG_GET(DISPC_CONTROL, bit, bit) == 1) {
		if (time_after(jiffies, tmo)) {
			DSSERR("timeout waiting GO flag\n");
			goto end;
		}
		cpu_relax();
	}

	DSSDBG("GO %s\n", channel == OMAP_DSS_CHANNEL_LCD ? "LCD" : "DIGIT");

	REG_FLD_MOD(DISPC_CONTROL, 1, bit, bit);
end:
	enable_clocks(0);
}

static void _dispc_write_firh_reg(enum omap_plane plane, int reg, u32 value)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	dispc_write_reg(DISPC_VID_FIR_COEF_H(plane-1, reg), value);
}

static void _dispc_write_firhv_reg(enum omap_plane plane, int reg, u32 value)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	dispc_write_reg(DISPC_VID_FIR_COEF_HV(plane-1, reg), value);
}

static void _dispc_set_scale_coef(enum omap_plane plane, int hscaleup,
		int vscaleup)
{
	/* Coefficients for horizontal up-sampling */
	const u32 coef_hup[8] = {
		0x00800000,
		0x0D7CF800,
		0x1E70F5FF,
		0x335FF5FE,
		0xF74949F7,
		0xF55F33FB,
		0xF5701EFE,
		0xF87C0DFF,
	};

	/* Coefficients for horizontal down-sampling */
	const u32 coef_hdown[8] = {
		0x24382400,
		0x28371FFE,
		0x2C361BFB,
		0x303516F9,
		0x11343311,
		0x1635300C,
		0x1B362C08,
		0x1F372804,
	};

	/* Coefficients for horizontal and vertical up-sampling */
	const u32 coef_hvup[8] = {
		0x00800000,
		0x037B02FF,
		0x0C6F05FE,
		0x205907FB,
		0x00404000,
		0x075920FE,
		0x056F0CFF,
		0x027B0300,
	};

	/* Coefficients for horizontal and vertical down-sampling */
	const u32 coef_hvdown[8] = {
		0x24382400,
		0x28391F04,
		0x2D381B08,
		0x3237170C,
		0x123737F7,
		0x173732F9,
		0x1B382DFB,
		0x1F3928FE,
	};

	const u32 *h_coef;
	const u32 *hv_coef;
	const u32 *hv_coef_mod;
	int i;

	if (hscaleup)
		h_coef = coef_hup;
	else
		h_coef = coef_hdown;

	if (vscaleup) {
		hv_coef = coef_hvup;

		if (hscaleup)
			hv_coef_mod = NULL;
		else
			hv_coef_mod = coef_hvdown;
	} else {
		hv_coef = coef_hvdown;

		if (hscaleup)
			hv_coef_mod = coef_hvup;
		else
			hv_coef_mod = NULL;
	}

	for (i = 0; i < 8; i++) {
		u32 h, hv;

		h = h_coef[i];

		hv = hv_coef[i];

		if (hv_coef_mod) {
			hv &= 0xffffff00;
			hv |= (hv_coef_mod[i] & 0xff);
		}

		_dispc_write_firh_reg(plane, i, h);
		_dispc_write_firhv_reg(plane, i, hv);
	}
}

static void _dispc_setup_color_conv_coef(void)
{
	const struct color_conv_coef {
		int  ry,  rcr,  rcb,   gy,  gcr,  gcb,   by,  bcr,  bcb;
		int  full_range;
	}  ctbl_bt601_5 = {
		298,  409,    0,  298, -208, -100,  298,    0,  517, 0,
	};

	const struct color_conv_coef *ct;

#define CVAL(x, y) (FLD_VAL(x, 26, 16) | FLD_VAL(y, 10, 0))

	ct = &ctbl_bt601_5;

	dispc_write_reg(DISPC_VID_CONV_COEF(0, 0), CVAL(ct->rcr, ct->ry));
	dispc_write_reg(DISPC_VID_CONV_COEF(0, 1), CVAL(ct->gy,	 ct->rcb));
	dispc_write_reg(DISPC_VID_CONV_COEF(0, 2), CVAL(ct->gcb, ct->gcr));
	dispc_write_reg(DISPC_VID_CONV_COEF(0, 3), CVAL(ct->bcr, ct->by));
	dispc_write_reg(DISPC_VID_CONV_COEF(0, 4), CVAL(0,       ct->bcb));

	dispc_write_reg(DISPC_VID_CONV_COEF(1, 0), CVAL(ct->rcr, ct->ry));
	dispc_write_reg(DISPC_VID_CONV_COEF(1, 1), CVAL(ct->gy,	 ct->rcb));
	dispc_write_reg(DISPC_VID_CONV_COEF(1, 2), CVAL(ct->gcb, ct->gcr));
	dispc_write_reg(DISPC_VID_CONV_COEF(1, 3), CVAL(ct->bcr, ct->by));
	dispc_write_reg(DISPC_VID_CONV_COEF(1, 4), CVAL(0,       ct->bcb));

#undef CVAL

	REG_FLD_MOD(DISPC_VID_ATTRIBUTES(0), ct->full_range, 11, 11);
	REG_FLD_MOD(DISPC_VID_ATTRIBUTES(1), ct->full_range, 11, 11);
}

static void _dispc_set_plane_ba0(enum omap_plane plane, u32 paddr)
{
	const struct dispc_reg ba0_reg[] = { DISPC_GFX_BA0,
		DISPC_VID_BA0(0),
		DISPC_VID_BA0(1) };

	dispc_write_reg(ba0_reg[plane], paddr);
}

static void _dispc_set_plane_ba1(enum omap_plane plane, u32 paddr)
{
	const struct dispc_reg ba1_reg[] = { DISPC_GFX_BA1,
				      DISPC_VID_BA1(0),
				      DISPC_VID_BA1(1) };

	dispc_write_reg(ba1_reg[plane], paddr);
}

static void _dispc_set_plane_pos(enum omap_plane plane, int x, int y)
{
	const struct dispc_reg pos_reg[] = { DISPC_GFX_POSITION,
				      DISPC_VID_POSITION(0),
				      DISPC_VID_POSITION(1) };

	u32 val = FLD_VAL(y, 26, 16) | FLD_VAL(x, 10, 0);
	dispc_write_reg(pos_reg[plane], val);
}

static void _dispc_set_pic_size(enum omap_plane plane, int width, int height)
{
	const struct dispc_reg siz_reg[] = { DISPC_GFX_SIZE,
				      DISPC_VID_PICTURE_SIZE(0),
				      DISPC_VID_PICTURE_SIZE(1) };
	u32 val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);
	dispc_write_reg(siz_reg[plane], val);
}

static void _dispc_set_vid_size(enum omap_plane plane, int width, int height)
{
	u32 val;
	const struct dispc_reg vsi_reg[] = { DISPC_VID_SIZE(0),
				      DISPC_VID_SIZE(1) };

	BUG_ON(plane == OMAP_DSS_GFX);

	val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);
	dispc_write_reg(vsi_reg[plane-1], val);
}

static void _dispc_set_row_inc(enum omap_plane plane, int inc)
{
	const struct dispc_reg ri_reg[] = { DISPC_GFX_ROW_INC,
				     DISPC_VID_ROW_INC(0),
				     DISPC_VID_ROW_INC(1) };

	dispc_write_reg(ri_reg[plane], inc);
}

static void _dispc_set_color_mode(enum omap_plane plane,
		enum omap_color_mode color_mode)
{
	u32 m = 0;

	switch (color_mode) {
	case OMAP_DSS_COLOR_CLUT1:
		m = 0x0; break;
	case OMAP_DSS_COLOR_CLUT2:
		m = 0x1; break;
	case OMAP_DSS_COLOR_CLUT4:
		m = 0x2; break;
	case OMAP_DSS_COLOR_CLUT8:
		m = 0x3; break;
	case OMAP_DSS_COLOR_RGB12U:
		m = 0x4; break;
	case OMAP_DSS_COLOR_ARGB16:
		m = 0x5; break;
	case OMAP_DSS_COLOR_RGB16:
		m = 0x6; break;
	case OMAP_DSS_COLOR_RGB24U:
		m = 0x8; break;
	case OMAP_DSS_COLOR_RGB24P:
		m = 0x9; break;
	case OMAP_DSS_COLOR_YUV2:
		m = 0xa; break;
	case OMAP_DSS_COLOR_UYVY:
		m = 0xb; break;
	case OMAP_DSS_COLOR_ARGB32:
		m = 0xc; break;
	case OMAP_DSS_COLOR_RGBA32:
		m = 0xd; break;
	case OMAP_DSS_COLOR_RGBX32:
		m = 0xe; break;
	default:
		BUG(); break;
	}

	REG_FLD_MOD(dispc_reg_att[plane], m, 4, 1);
}

static void _dispc_set_channel_out(enum omap_plane plane,
		enum omap_channel channel)
{
	int shift;
	u32 val;

	switch (plane) {
	case OMAP_DSS_GFX:
		shift = 8;
		break;
	case OMAP_DSS_VIDEO1:
	case OMAP_DSS_VIDEO2:
		shift = 16;
		break;
	default:
		BUG();
		return;
	}

	val = dispc_read_reg(dispc_reg_att[plane]);
	val = FLD_MOD(val, channel, shift, shift);
	dispc_write_reg(dispc_reg_att[plane], val);
}

void dispc_set_burst_size(enum omap_plane plane,
		enum omap_burst_size burst_size)
{
	int shift;
	u32 val;

	enable_clocks(1);

	switch (plane) {
	case OMAP_DSS_GFX:
		shift = 6;
		break;
	case OMAP_DSS_VIDEO1:
	case OMAP_DSS_VIDEO2:
		shift = 14;
		break;
	default:
		BUG();
		return;
	}

	val = dispc_read_reg(dispc_reg_att[plane]);
	val = FLD_MOD(val, burst_size, shift+1, shift);
	dispc_write_reg(dispc_reg_att[plane], val);

	enable_clocks(0);
}

static void _dispc_set_vid_color_conv(enum omap_plane plane, int enable)
{
	u32 val;

	BUG_ON(plane == OMAP_DSS_GFX);

	val = dispc_read_reg(dispc_reg_att[plane]);
	val = FLD_MOD(val, enable, 9, 9);
	dispc_write_reg(dispc_reg_att[plane], val);
}

void dispc_set_lcd_size(int width, int height)
{
	u32 val;
	BUG_ON((width > (1 << 11)) || (height > (1 << 11)));
	val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);
	enable_clocks(1);
	dispc_write_reg(DISPC_SIZE_LCD, val);
	enable_clocks(0);
}

void dispc_set_digit_size(int width, int height)
{
	u32 val;
	BUG_ON((width > (1 << 11)) || (height > (1 << 11)));
	val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);
	enable_clocks(1);
	dispc_write_reg(DISPC_SIZE_DIG, val);
	enable_clocks(0);
}

int dispc_get_alpha_blending(enum omap_channel ch)
{
	u32 config;

	enable_clocks(1);
	config = dispc_read_reg(DISPC_CONFIG);
	enable_clocks(0);

	if (ch == OMAP_DSS_CHANNEL_LCD) {
		if (config & 0x40000)
			return 1;
		else
			return 0;
	} else if (ch == OMAP_DSS_CHANNEL_DIGIT) {
		if (config & 0x80000)
			return 1;
		else
			return 0;
	}
	return 0;

}

void dispc_get_color_keying(enum omap_channel ch, struct omap_color_key *key)
{
	u32 config;
	const struct dispc_reg tr_reg[] = {
		DISPC_TRANS_COLOR0, DISPC_TRANS_COLOR1 };

	enable_clocks(1);
	config = dispc_read_reg(DISPC_CONFIG);
	enable_clocks(0);

	key->type = 0;

	if (ch == OMAP_DSS_CHANNEL_LCD) {
		/* check color keying is enabled or not */
		if (config & 0x400) {
			/* Check whether it is src color keying
			   or dst color keying */
			if (config & 0x800)
				key->type = OMAP_DSS_COLOR_KEY_VID_SRC;
			else
				key->type = OMAP_DSS_COLOR_KEY_GFX_DST;

			key->enable = 1;
		} else
			key->enable = 0;
		enable_clocks(1);
		key->color = dispc_read_reg(tr_reg[ch]);
		enable_clocks(0);
	} else if (ch == OMAP_DSS_CHANNEL_DIGIT) {
		/* check color keying is enabled or not */
		if (config & 0x1000) {
			/* Check whether it is src color keying
			   or dst color keying */
			if (config & 0x2000)
				key->type = OMAP_DSS_COLOR_KEY_VID_SRC;
			else
				key->type = OMAP_DSS_COLOR_KEY_GFX_DST;

			key->enable = 1;
		} else
			key->enable = 0;

		enable_clocks(1);
		key->color = dispc_read_reg(tr_reg[ch]);
		enable_clocks(0);
	}

}

u32 dispc_get_plane_fifo_size(enum omap_plane plane)
{
	const struct dispc_reg fsz_reg[] = { DISPC_GFX_FIFO_SIZE_STATUS,
				      DISPC_VID_FIFO_SIZE_STATUS(0),
				      DISPC_VID_FIFO_SIZE_STATUS(1) };
	u32 size;

	enable_clocks(1);

	if (cpu_is_omap24xx())
		size = FLD_GET(dispc_read_reg(fsz_reg[plane]), 8, 0);
	else if (cpu_is_omap34xx())
		size = FLD_GET(dispc_read_reg(fsz_reg[plane]), 10, 0);
	else
		BUG();

	enable_clocks(0);

	return size;
}

void dispc_setup_plane_fifo(enum omap_plane plane, u32 low, u32 high)
{
	const struct dispc_reg ftrs_reg[] = { DISPC_GFX_FIFO_THRESHOLD,
				       DISPC_VID_FIFO_THRESHOLD(0),
				       DISPC_VID_FIFO_THRESHOLD(1) };
	const struct dispc_reg fsz_reg[] = { DISPC_GFX_FIFO_SIZE_STATUS,
				      DISPC_VID_FIFO_SIZE_STATUS(0),
				      DISPC_VID_FIFO_SIZE_STATUS(1) };
	u32 size;

	enable_clocks(1);

	if (cpu_is_omap24xx())
		size = FLD_GET(dispc_read_reg(fsz_reg[plane]), 8, 0);
	else if (cpu_is_omap34xx())
		size = FLD_GET(dispc_read_reg(fsz_reg[plane]), 10, 0);
	else
		BUG();

	BUG_ON(low > size || high > size);

	DSSDBG("fifo(%d) size %d, low/high old %u/%u, new %u/%u\n",
			plane, size,
			REG_GET(ftrs_reg[plane], 11, 0),
			REG_GET(ftrs_reg[plane], 27, 16),
			low, high);

	if (cpu_is_omap24xx())
		dispc_write_reg(ftrs_reg[plane],
				FLD_VAL(high, 24, 16) | FLD_VAL(low, 8, 0));
	else
		dispc_write_reg(ftrs_reg[plane],
				FLD_VAL(high, 27, 16) | FLD_VAL(low, 11, 0));

	enable_clocks(0);
}

static void _dispc_set_fir(enum omap_plane plane, int hinc, int vinc)
{
	u32 val;
	const struct dispc_reg fir_reg[] = { DISPC_VID_FIR(0),
				      DISPC_VID_FIR(1) };

	BUG_ON(plane == OMAP_DSS_GFX);

	val = FLD_VAL(vinc, 27, 16) | FLD_VAL(hinc, 11, 0);
	dispc_write_reg(fir_reg[plane-1], val);
}

static void _dispc_set_vid_accu0(enum omap_plane plane, int haccu, int vaccu)
{
	u32 val;
	const struct dispc_reg ac0_reg[] = { DISPC_VID_ACCU0(0),
				      DISPC_VID_ACCU0(1) };

	BUG_ON(plane == OMAP_DSS_GFX);

	val = FLD_VAL(vaccu, 25, 16) | FLD_VAL(haccu, 9, 0);
	dispc_write_reg(ac0_reg[plane-1], val);
}

static void _dispc_set_vid_accu1(enum omap_plane plane, int haccu, int vaccu)
{
	u32 val;
	const struct dispc_reg ac1_reg[] = { DISPC_VID_ACCU1(0),
				      DISPC_VID_ACCU1(1) };

	BUG_ON(plane == OMAP_DSS_GFX);

	val = FLD_VAL(vaccu, 25, 16) | FLD_VAL(haccu, 9, 0);
	dispc_write_reg(ac1_reg[plane-1], val);
}

static void _dispc_set_scaling(enum omap_plane plane,
			       int orig_width, int orig_height,
			       int out_width, int out_height,
			       int ilace)
{
	int fir_hinc;
	int fir_vinc;
	int hscaleup, vscaleup;
	int fieldmode = 0;
	int accu0 = 0;
	int accu1 = 0;
	u32 l;

	BUG_ON(plane == OMAP_DSS_GFX);

	hscaleup = orig_width <= out_width;
	vscaleup = orig_height <= out_height;

	_dispc_set_scale_coef(plane, hscaleup, vscaleup);

	if (!orig_width || orig_width == out_width)
		fir_hinc = 0;
	else
		fir_hinc = 1024 * orig_width / out_width;

	if (!orig_height || orig_height == out_height)
		fir_vinc = 0;
	else
		fir_vinc = 1024 * orig_height / out_height;
	_dispc_set_fir(plane, fir_hinc, fir_vinc);

	l = dispc_read_reg(dispc_reg_att[plane]);
	l &= ~(0x0f << 5);

	l |= fir_hinc ? (1 << 5) : 0;
	l |= fir_vinc ? (1 << 6) : 0;

	l |= hscaleup ? 0 : (1 << 7);
	l |= vscaleup ? 0 : (1 << 8);

	dispc_write_reg(dispc_reg_att[plane], l);

	if (ilace) {
		if (fieldmode) {
			accu0 = fir_vinc / 2;
			accu1 = 0;
		} else {
			accu0 = 0;
			accu1 = fir_vinc / 2;
			if (accu1 >= 1024/2) {
				accu0 = 1024/2;
				accu1 -= accu0;
			}
		}
	}

	_dispc_set_vid_accu0(plane, 0, accu0);
	_dispc_set_vid_accu1(plane, 0, accu1);
}

static int _dispc_set_rotation_mirroring(enum omap_plane plane,
	int rotation, int mirroring, enum omap_color_mode color_mode)
{
	if (color_mode == OMAP_DSS_COLOR_YUV2 ||
			color_mode == OMAP_DSS_COLOR_UYVY) {
		if (mirroring == 1) {
			if (rotation == 90)
				REG_FLD_MOD(dispc_reg_att[plane], 0x3, 13, 12);
			else if (rotation == 270)
				REG_FLD_MOD(dispc_reg_att[plane], 0x1, 13, 12);
			else if (rotation == 0)
				REG_FLD_MOD(dispc_reg_att[plane], 0x2, 13, 12);
			else if (rotation == 180)
				REG_FLD_MOD(dispc_reg_att[plane], 0x0, 13, 12);
		} else {
			if (rotation == 90)
				REG_FLD_MOD(dispc_reg_att[plane], 0x3, 13, 12);
			else if (rotation == 270)
				REG_FLD_MOD(dispc_reg_att[plane], 0x01, 13, 12);
			else if (rotation == 180)
				REG_FLD_MOD(dispc_reg_att[plane], 0x2, 13, 12);
			else if (rotation == 0 || rotation == -1)
				REG_FLD_MOD(dispc_reg_att[plane], 0x0, 13, 12);
		}
	}
	if (rotation == 90 || rotation == 270)
		REG_FLD_MOD(dispc_reg_att[plane], 0x1, 18, 18);
	else
		REG_FLD_MOD(dispc_reg_att[plane], 0x0, 18, 18);

	return 0;
}

#define MAX_PIXELS_PER_LINE     2048
#define MAX_LINES               2048

static void _dispc_calc_and_set_row_inc(enum omap_plane plane,
	int screen_width, int cropwidth,
	int cleft, enum omap_color_mode color_mode, int rotation, int mirror,
	int fieldmode)
{
	int ps = 2, vr_ps = 1;
	int row_inc_value = 0;
	int temp;

	switch (color_mode) {
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		if (mirror == 1 || rotation >= 0) {
			/*
			 * ps      - In VRFB space the pixel size for YUYV/UYVY
			 * is 4 bytes
			 * vr_ps - Actual pixel size for YUYV/UYVY  is 2 bytes
			 */
			ps = 4;
			vr_ps = 2;
		}
		break;
	case OMAP_DSS_COLOR_RGB24P:
		ps = 3;		/* pixel size is 3 bytes */
		break;

	case OMAP_DSS_COLOR_RGB24U:
	case OMAP_DSS_COLOR_ARGB32:
	case OMAP_DSS_COLOR_RGBA32:
		ps = 4;		/* pixel size is 4 bytes */
		break;

	case OMAP_DSS_COLOR_RGB16:
	default:
		ps = 2;		/* pixel size is 2 bytes */
		break;
	}
	if (rotation == 90 || rotation == 270) {
		row_inc_value = 1 + (MAX_PIXELS_PER_LINE - screen_width +
			 (screen_width - cropwidth - cleft) + cleft) * ps;

	} else if (rotation == 180 || rotation == 0) {
		if (color_mode == OMAP_DSS_COLOR_YUV2
			|| color_mode == OMAP_DSS_COLOR_UYVY) {
			temp = MAX_PIXELS_PER_LINE - (screen_width / vr_ps);
			temp += (screen_width - cropwidth - cleft) / vr_ps;
			row_inc_value = ((temp + (cleft/vr_ps)) * ps);
			row_inc_value++;

		} else
			row_inc_value = 1 + (MAX_PIXELS_PER_LINE -
				screen_width + (screen_width - cropwidth -
				cleft) + cleft) * ps;
	} else
		row_inc_value = 1 + (screen_width * ps) - cropwidth * ps;

	if (fieldmode) {
		if (rotation >= 0)
			row_inc_value = row_inc_value +
				MAX_PIXELS_PER_LINE * ps;
		else
			row_inc_value = row_inc_value + screen_width * ps;
	}
	_dispc_set_row_inc(plane, row_inc_value);

}

static int _dispc_setup_plane(enum omap_plane plane,
		enum omap_channel channel_out,
		u32 paddr, int tv_field1_offset, int screen_width,
		int pos_x, int pos_y,
		int width, int height,
		int out_width, int out_height,
		enum omap_color_mode color_mode,
		int ilace, int rotation, int mirror, int global_alpha)
{
	int fieldmode = 0;
	int bpp;
	int cconv;
	int scaling = 0;

	if (plane == OMAP_DSS_GFX) {
		if (width != out_width || height != out_height)
			return -EINVAL;
	} else {
		/* video plane */
		if (width != out_width || height != out_height)
			scaling = 1;

		if (out_width < width/2 ||
		   out_width > width*8)
			return -EINVAL;

		if (out_height < height/2 ||
		   out_height > height*8)
			return -EINVAL;
	}
	switch (color_mode) {
	case OMAP_DSS_COLOR_RGB16:
		bpp = 16;
		cconv = 0;
		break;

	case OMAP_DSS_COLOR_RGB24P:
		bpp = 24;
		cconv = 0;
		break;

	case OMAP_DSS_COLOR_RGB24U:
	case OMAP_DSS_COLOR_ARGB32:
	case OMAP_DSS_COLOR_RGBA32:
		bpp = 32;
		cconv = 0;
		break;

	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		BUG_ON(plane == OMAP_DSS_GFX);
		bpp = 16;
		cconv = 1;
		break;

	default:
		BUG();
		return 1;
	}
	if (ilace) {
		if (height == out_height || height > out_height)
			fieldmode = 1;
	}

	if (fieldmode)
		height /= 2;

	if (ilace)
		out_height /= 2;

	if (plane != OMAP_DSS_GFX)
		_dispc_set_scaling(plane, width, height,
				   out_width, out_height,
				   ilace);
	/* attributes */
	_dispc_set_channel_out(plane, channel_out);

	/* Set rotation and mirroring attributes */
	_dispc_set_rotation_mirroring(plane, rotation,
			mirror, color_mode);

	_dispc_set_color_mode(plane, color_mode);
	if (plane != OMAP_DSS_GFX)
		_dispc_set_vid_color_conv(plane, cconv);

	/* */
	_dispc_set_plane_ba0(plane, paddr);

	if (fieldmode)
		_dispc_set_plane_ba1(plane, paddr + tv_field1_offset);

	else
		_dispc_set_plane_ba1(plane, paddr);

	_dispc_set_plane_pos(plane, pos_x, pos_y);

	_dispc_set_pic_size(plane, width, height);

	if (plane != OMAP_DSS_GFX)
		_dispc_set_vid_size(plane, out_width, out_height);

	_dispc_calc_and_set_row_inc(plane, screen_width, width, 0,
			color_mode, rotation, mirror, fieldmode);

	if (plane == OMAP_DSS_GFX)
		REG_FLD_MOD(DISPC_GLOBAL_ALPHA, global_alpha, 7, 0);
	else if (plane == OMAP_DSS_VIDEO2)
		REG_FLD_MOD(DISPC_GLOBAL_ALPHA, global_alpha, 23, 16);

	return 0;
}

static void _dispc_enable_plane(enum omap_plane plane, int enable)
{
	REG_FLD_MOD(dispc_reg_att[plane], enable ? 1 : 0, 0, 0);
}

void dispc_enable_lcd_out(int enable)
{
	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 0, 0);
	enable_clocks(0);
}

void dispc_enable_digit_out(int enable)
{
	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 1, 1);
	enable_clocks(0);
}

void dispc_lcd_enable_signal_polarity(int act_high)
{
	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONTROL, act_high ? 1 : 0, 29, 29);
	enable_clocks(0);
}

void dispc_lcd_enable_signal(int enable)
{
	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 28, 28);
	enable_clocks(0);
}

void dispc_pck_free_enable(int enable)
{
	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 27, 27);
	enable_clocks(0);
}

void dispc_enable_fifohandcheck(int enable)
{
	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONFIG, enable ? 1 : 0, 16, 16);
	enable_clocks(0);
}

void dispc_set_lcd_display_type(enum omap_lcd_display_type type)
{
	int mode;

	switch (type) {
	case OMAP_DSS_LCD_DISPLAY_STN:
		mode = 0;
		break;

	case OMAP_DSS_LCD_DISPLAY_TFT:
		mode = 1;
		break;

	default:
		BUG();
		return;
	}

	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONTROL, mode, 3, 3);
	enable_clocks(0);
}

void dispc_set_loadmode(enum omap_dss_load_mode mode)
{
	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONFIG, mode, 2, 1);
	enable_clocks(0);
}

void omap_dispc_set_default_color(enum omap_channel channel, u32 color)
{
	const struct dispc_reg def_reg[] = { DISPC_DEFAULT_COLOR0,
				DISPC_DEFAULT_COLOR1 };

	enable_clocks(1);
	dispc_write_reg(def_reg[channel], color);
	enable_clocks(0);
}

int omap_dispc_get_default_color(enum omap_channel channel)
{
	const struct dispc_reg def_reg[] = { DISPC_DEFAULT_COLOR0,
				DISPC_DEFAULT_COLOR1 };

	unsigned int color;
	enable_clocks(1);
	color = dispc_read_reg(def_reg[channel]);
	enable_clocks(0);
	return color;
}

void omap_dispc_set_trans_key(enum omap_channel ch,
		enum omap_dss_color_key_type type,
		u32 trans_key)
{
	const struct dispc_reg tr_reg[] = {
		DISPC_TRANS_COLOR0, DISPC_TRANS_COLOR1 };

	enable_clocks(1);
	if (ch == OMAP_DSS_CHANNEL_LCD)
		REG_FLD_MOD(DISPC_CONFIG, type, 11, 11);
	else /* OMAP_DSS_CHANNEL_DIGIT */
		REG_FLD_MOD(DISPC_CONFIG, type, 13, 13);

	dispc_write_reg(tr_reg[ch], trans_key);
	enable_clocks(0);
}

void dispc_enable_alpha_blending(enum omap_channel ch, int enable)
{
	enable_clocks(1);
	if (ch == OMAP_DSS_CHANNEL_LCD)
		REG_FLD_MOD(DISPC_CONFIG, enable, 18, 18);
	else /* OMAP_DSS_CHANNEL_DIGIT */
		REG_FLD_MOD(DISPC_CONFIG, enable, 19, 19);
	enable_clocks(0);
}

void omap_dispc_enable_trans_key(enum omap_channel ch, int enable)
{
	enable_clocks(1);
	if (ch == OMAP_DSS_CHANNEL_LCD)
		REG_FLD_MOD(DISPC_CONFIG, enable, 10, 10);
	else /* OMAP_DSS_CHANNEL_DIGIT */
		REG_FLD_MOD(DISPC_CONFIG, enable, 12, 12);
	enable_clocks(0);
}

void dispc_set_tft_data_lines(int data_lines)
{
	int code;

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
		return;
	}

	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONTROL, code, 9, 8);
	enable_clocks(0);
}

void dispc_set_parallel_interface_mode(enum omap_parallel_interface_mode mode)
{
	u32 l;
	int stallmode;
	int gpout0 = 1;
	int gpout1;

	switch (mode) {
	case OMAP_DSS_PARALLELMODE_BYPASS:
		stallmode = 0;
		gpout1 = 1;
		break;

	case OMAP_DSS_PARALLELMODE_RFBI:
		stallmode = 1;
		gpout1 = 0;
		break;

	case OMAP_DSS_PARALLELMODE_DSI:
		stallmode = 1;
		gpout1 = 1;
		break;

	default:
		BUG();
		return;
	}

	enable_clocks(1);

	l = dispc_read_reg(DISPC_CONTROL);

	l = FLD_MOD(l, stallmode, 11, 11);
	l = FLD_MOD(l, gpout0, 15, 15);
	l = FLD_MOD(l, gpout1, 16, 16);

	dispc_write_reg(DISPC_CONTROL, l);

	enable_clocks(0);
}

static void _dispc_set_lcd_timings(int hsw, int hfp, int hbp,
				   int vsw, int vfp, int vbp)
{
	u32 timing_h, timing_v;

	BUG_ON(hsw < 1 || hsw > 64);
	BUG_ON(hfp < 1 || hfp > 256);
	BUG_ON(hbp < 1 || hbp > 256);

	BUG_ON(vsw < 1 || vsw > 64);
	BUG_ON(vfp < 0 || vfp > 255);
	BUG_ON(vbp < 0 || vbp > 255);

	timing_h = FLD_VAL(hsw-1, 5, 0) | FLD_VAL(hfp-1, 15, 8) |
		FLD_VAL(hbp-1, 27, 20);

	timing_v = FLD_VAL(vsw-1, 5, 0) | FLD_VAL(vfp, 15, 8) |
		FLD_VAL(vbp, 27, 20);

	enable_clocks(1);
	dispc_write_reg(DISPC_TIMING_H, timing_h);
	dispc_write_reg(DISPC_TIMING_V, timing_v);
	enable_clocks(0);
}

/* change name to mode? */
void dispc_set_lcd_timings(struct omap_video_timings *timings)
{
	unsigned xtot, ytot;
	unsigned long ht, vt;

	_dispc_set_lcd_timings(timings->hsw, timings->hfp, timings->hbp,
			timings->vsw, timings->vfp, timings->vbp);

	dispc_set_lcd_size(timings->x_res, timings->y_res);

	xtot = timings->x_res + timings->hfp + timings->hsw + timings->hbp;
	ytot = timings->y_res + timings->vfp + timings->vsw + timings->vbp;

	ht = (timings->pixel_clock * 1000) / xtot;
	vt = (timings->pixel_clock * 1000) / xtot / ytot;

	DSSDBG("xres %u yres %u\n", timings->x_res, timings->y_res);
	DSSDBG("pck %u\n", timings->pixel_clock);
	DSSDBG("hsw %d hfp %d hbp %d vsw %d vfp %d vbp %d\n",
			timings->hsw, timings->hfp, timings->hbp,
			timings->vsw, timings->vfp, timings->vbp);

	DSSDBG("hsync %luHz, vsync %luHz\n", ht, vt);
}

void dispc_set_lcd_divisor(int lck_div, int pck_div)
{
	BUG_ON(lck_div < 1);
	BUG_ON(pck_div < 2);

	enable_clocks(1);
	dispc_write_reg(DISPC_DIVISOR,
			FLD_VAL(lck_div, 23, 16) | FLD_VAL(pck_div, 7, 0));
	enable_clocks(0);
}

static void dispc_get_lcd_divisor(int *lck_div, int *pck_div)
{
	u32 l;
	l = dispc_read_reg(DISPC_DIVISOR);
	*lck_div = FLD_GET(l, 23, 16);
	*pck_div = FLD_GET(l, 7, 0);
}

unsigned long dispc_fclk_rate(void)
{
	unsigned long r = 0;

	if (dss_get_dispc_clk_source() == 0)
		r = dss_clk_get_rate(DSS_CLK_FCK1);
	else
#ifdef CONFIG_OMAP2_DSS_DSI
		r = dsi_get_dsi1_pll_rate();
#else
	BUG();
#endif
	return r;
}

unsigned long dispc_pclk_rate(void)
{
	int lcd, pcd;
	unsigned long r;
	u32 l;

	l = dispc_read_reg(DISPC_DIVISOR);

	lcd = FLD_GET(l, 23, 16);
	pcd = FLD_GET(l, 7, 0);

	r = dispc_fclk_rate();

	return r / lcd / pcd;
}

ssize_t dispc_print_clocks(char *buf, ssize_t size)
{
	ssize_t l = 0;
	int lcd, pcd;

	enable_clocks(1);

	dispc_get_lcd_divisor(&lcd, &pcd);

	l += snprintf(buf + l, size - l, "- dispc -\n");

	l += snprintf(buf + l, size - l, "dispc fclk source = %s\n",
			dss_get_dispc_clk_source() == 0 ?
			"dss1_alwon_fclk" : "dsi1_pll_fclk");

	l += snprintf(buf + l, size - l,
			"pixel clk = %lu / %d / %d = %lu\n",
			dispc_fclk_rate(),
			lcd, pcd,
			dispc_pclk_rate());

	enable_clocks(0);

	return l;
}

static void _dispc_set_pol_freq(int onoff, int rf, int ieo, int ipc,
				int ihs, int ivs, int acbi, int acb)
{
	u32 l = 0;

	DSSDBG("onoff %d rf %d ieo %d ipc %d ihs %d ivs %d acbi %d acb %d\n",
			onoff, rf, ieo, ipc, ihs, ivs, acbi, acb);

	l |= FLD_VAL(onoff, 17, 17);
	l |= FLD_VAL(rf, 16, 16);
	l |= FLD_VAL(ieo, 15, 15);
	l |= FLD_VAL(ipc, 14, 14);
	l |= FLD_VAL(ihs, 13, 13);
	l |= FLD_VAL(ivs, 12, 12);
	l |= FLD_VAL(acbi, 11, 8);
	l |= FLD_VAL(acb, 7, 0);

	enable_clocks(1);
	dispc_write_reg(DISPC_POL_FREQ, l);
	enable_clocks(0);
}

void dispc_set_pol_freq(struct omap_panel *panel)
{
	_dispc_set_pol_freq((panel->config & OMAP_DSS_LCD_ONOFF) != 0,
				 (panel->config & OMAP_DSS_LCD_RF) != 0,
				 (panel->config & OMAP_DSS_LCD_IEO) != 0,
				 (panel->config & OMAP_DSS_LCD_IPC) != 0,
				 (panel->config & OMAP_DSS_LCD_IHS) != 0,
				 (panel->config & OMAP_DSS_LCD_IVS) != 0,
				 panel->acbi, panel->acb);
}

void find_lck_pck_divs(int is_tft, unsigned long req_pck, unsigned long fck,
		int *lck_div, int *pck_div)
{
	int pcd_min = is_tft ? 2 : 3;
	unsigned long best_pck;
	int best_ld, cur_ld;
	int best_pd, cur_pd;

	best_pck = 0;
	best_ld = 0;
	best_pd = 0;

	for (cur_ld = 1; cur_ld <= 255; ++cur_ld) {
		unsigned long lck = fck / cur_ld;

		for (cur_pd = pcd_min; cur_pd <= 255; ++cur_pd) {
			unsigned long pck = lck / cur_pd;
			long old_delta = abs(best_pck - req_pck);
			long new_delta = abs(pck - req_pck);

			if (best_pck == 0 || new_delta < old_delta) {
				best_pck = pck;
				best_ld = cur_ld;
				best_pd = cur_pd;

				if (pck == req_pck)
					goto found;
			}

			if (pck < req_pck)
				break;
		}

		if (lck / pcd_min < req_pck)
			break;
	}

found:
	*lck_div = best_ld;
	*pck_div = best_pd;
}

int dispc_calc_clock_div(int is_tft, unsigned long req_pck,
		struct dispc_clock_info *cinfo)
{
	unsigned long prate;
	struct dispc_clock_info cur, best;
	int match = 0;
	int min_fck_per_pck;

	if (cpu_is_omap34xx())
		prate = clk_get_rate(clk_get_parent(dispc.dpll4_m4_ck));
	else
		prate = 0;

	if (req_pck == dispc.cache_req_pck &&
			((cpu_is_omap34xx() && prate == dispc.cache_prate) ||
			 dispc.cache_cinfo.fck == dss_clk_get_rate(DSS_CLK_FCK1))) {
		DSSDBG("dispc clock info found from cache.\n");
		*cinfo = dispc.cache_cinfo;
		return 0;
	}

	min_fck_per_pck = CONFIG_OMAP2_DSS_MIN_FCK_PER_PCK;

	if (min_fck_per_pck &&
		req_pck * min_fck_per_pck > DISPC_MAX_FCK) {
		DSSERR("Requested pixel clock not possible with the current "
				"OMAP2_DSS_MIN_FCK_PER_PCK setting. Turning "
				"the constraint off.\n");
		min_fck_per_pck = 0;
	}

retry:
	memset(&cur, 0, sizeof(cur));
	memset(&best, 0, sizeof(best));

	if (cpu_is_omap24xx()) {
		/* XXX can we change the clock on omap2? */
		cur.fck = dss_clk_get_rate(DSS_CLK_FCK1);
		cur.fck_div = 1;

		match = 1;

		find_lck_pck_divs(is_tft, req_pck, cur.fck,
				&cur.lck_div, &cur.pck_div);

		cur.lck = cur.fck / cur.lck_div;
		cur.pck = cur.lck / cur.pck_div;

		best = cur;

		goto found;
	} else if (cpu_is_omap34xx()) {
		for (cur.fck_div = 16; cur.fck_div > 0; --cur.fck_div) {
			cur.fck = prate / cur.fck_div * 2;

			if (cur.fck > DISPC_MAX_FCK)
				continue;

			if (min_fck_per_pck &&
					cur.fck < req_pck * min_fck_per_pck)
				continue;

			match = 1;

			find_lck_pck_divs(is_tft, req_pck, cur.fck,
					&cur.lck_div, &cur.pck_div);

			cur.lck = cur.fck / cur.lck_div;
			cur.pck = cur.lck / cur.pck_div;

			if (abs(cur.pck - req_pck) < abs(best.pck - req_pck)) {
				best = cur;

				if (cur.pck == req_pck)
					goto found;
			}
		}
	} else {
		BUG();
	}

found:
	if (!match) {
		if (min_fck_per_pck) {
			DSSERR("Could not find suitable clock settings.\n"
					"Turning FCK/PCK constraint off and"
					"trying again.\n");
			min_fck_per_pck = 0;
			goto retry;
		}

		DSSERR("Could not find suitable clock settings.\n");

		return -EINVAL;
	}

	if (cinfo)
		*cinfo = best;

	dispc.cache_req_pck = req_pck;
	dispc.cache_prate = prate;
	dispc.cache_cinfo = best;

	return 0;
}

int dispc_set_clock_div(struct dispc_clock_info *cinfo)
{
	unsigned long prate;
	int r;

	if (cpu_is_omap34xx()) {
		prate = clk_get_rate(clk_get_parent(dispc.dpll4_m4_ck));
		DSSDBG("dpll4_m4 = %ld\n", prate);
	}

	DSSDBG("fck = %ld (%d)\n", cinfo->fck, cinfo->fck_div);
	DSSDBG("lck = %ld (%d)\n", cinfo->lck, cinfo->lck_div);
	DSSDBG("pck = %ld (%d)\n", cinfo->pck, cinfo->pck_div);

	if (cpu_is_omap34xx()) {
		r = clk_set_rate(dispc.dpll4_m4_ck, prate / cinfo->fck_div);
		if (r)
			return r;
	}

	dispc_set_lcd_divisor(cinfo->lck_div, cinfo->pck_div);

	return 0;
}

void *omap_dispc_register_isr(omap_dispc_isr_t isr, void *arg, u32 mask)
{
	int i;
	int ret = -EBUSY;
	unsigned long flags;
	u32 new_mask = 0;

	if (isr == NULL)
		return NULL;

	spin_lock_irqsave(&dispc.irq_lock, flags);

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		if (registered_isr[i].isr != NULL)
			continue;

		registered_isr[i].isr = isr;
		registered_isr[i].arg = arg;
		registered_isr[i].mask = mask;

		enable_clocks(1);
		new_mask = dispc_read_reg(DISPC_IRQENABLE);
		new_mask |= mask;
		dispc_write_reg(DISPC_IRQSTATUS, mask);
		dispc_write_reg(DISPC_IRQENABLE, new_mask);
		enable_clocks(0);

		ret = 0;
		break;
	}

	if (!ret)
		return (void *)&registered_isr[i];
	else
		return (void *)NULL;

	spin_unlock_irqrestore(&dispc.irq_lock, flags);
}
EXPORT_SYMBOL(omap_dispc_register_isr);

int omap_dispc_unregister_isr(void *handle)
{
	int i, j;
	unsigned long flags;
	u32 new_mask = DISPC_IRQ_MASK_ERROR;
	int ret = -EINVAL;
	struct registered_isr *isr;

	spin_lock_irqsave(&dispc.irq_lock, flags);

	isr = (struct registered_isr *)handle;

	if (isr == NULL)
		return -EINVAL;

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		if (isr == (struct registered_isr *)&(registered_isr[i].isr)) {

			registered_isr[i].isr = NULL;
			registered_isr[i].arg = NULL;
			registered_isr[i].mask = 0;

			for (j = 0; j < DISPC_MAX_NR_ISRS; j++)
				new_mask |= registered_isr[j].mask;

			enable_clocks(1);
			dispc_write_reg(DISPC_IRQENABLE, new_mask);
			enable_clocks(0);
			ret = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&dispc.irq_lock, flags);
	return ret;
}
EXPORT_SYMBOL(omap_dispc_unregister_isr);

#ifdef DEBUG
static void print_irq_status(u32 status)
{
	if ((status & DISPC_IRQ_MASK_ERROR) == 0)
		return;

	printk(KERN_DEBUG "DISPC IRQ: 0x%x: ", status);

#define PIS(x) \
	if (status & DISPC_IRQ_##x) \
		printk(#x " ");
	PIS(GFX_FIFO_UNDERFLOW);
	PIS(OCP_ERR);
	PIS(VID1_FIFO_UNDERFLOW);
	PIS(VID2_FIFO_UNDERFLOW);
	PIS(SYNC_LOST);
	PIS(SYNC_LOST_DIGIT);
#undef PIS

	printk("\n");
}
#endif

/* Called from dss.c. Note that we don't touch clocks here,
 * but we presume they are on because we got an IRQ. However,
 * an irq handler may turn the clocks off, so we may not have
 * clock later in the function. */
void dispc_irq_handler(void)
{
	int i;
	u32 irqstatus = dispc_read_reg(DISPC_IRQSTATUS);
	static int errors;
	u32 handledirqs = 0;

#ifdef DEBUG
	if (dss_debug)
		print_irq_status(irqstatus);
#endif
	/* Ack the interrupt. Do it here before clocks are possibly turned
	 * off */
	dispc_write_reg(DISPC_IRQSTATUS, irqstatus);

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		if (!registered_isr[i].isr)
			continue;
		if (registered_isr[i].mask & irqstatus) {
			registered_isr[i].isr(registered_isr[i].arg,
					      irqstatus);

			handledirqs |= registered_isr[i].mask;
		}
	}

	if (irqstatus & ~handledirqs & DISPC_IRQ_MASK_ERROR) {
		if (printk_ratelimit()) {
			/* Suppressing the Digital sync lost error
			   condition as it comes first time everytime
			   venc comes out of reset and venc will come
			   out of reset everytime when power management
			   is enabled and system will go to higher C
			   states */
			if (!(irqstatus & DISPC_IRQ_SYNC_LOST_DIGIT))
				DSSERR("dispc irq error status %04x\n",
						irqstatus);
#ifdef DEBUG
			else
				DSSERR("dispc irq error status %04x\n",
						irqstatus);
#endif

		}
		if (errors++ > 100) {
			DSSERR("Excessive DISPC errors\n"
					"Turning off lcd and digit\n");
			dispc_enable_lcd_out(0);
			dispc_enable_digit_out(0);
		}
	}

}

#ifdef CONFIG_OMAP2_DSS_FAKE_VSYNC
void dispc_fake_vsync_irq(void)
{
	u32 irqstatus = DISPC_IRQ_VSYNC;
	int i;

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		if (!registered_isr[i].isr)
			continue;
		if (registered_isr[i].mask & irqstatus)
			registered_isr[i].isr(registered_isr[i].arg,
					      irqstatus);
	}
}
#endif

static void _omap_dispc_initialize_irq(void)
{
	memset(registered_isr, 0, sizeof(registered_isr));

	/* there's SYNC_LOST_DIGIT waiting after enabling the DSS,
	 * so clear it */
	dispc_write_reg(DISPC_IRQSTATUS,
			dispc_read_reg(DISPC_IRQSTATUS));

	/* We'll handle these always */
	dispc_write_reg(DISPC_IRQENABLE, DISPC_IRQ_MASK_ERROR);
}

static void _omap_dispc_initial_config(void)
{
	u32 l;

	l = dispc_read_reg(DISPC_SYSCONFIG);
	l = FLD_MOD(l, 2, 13, 12);	/* MIDLEMODE: smart standby */
	l = FLD_MOD(l, 2, 4, 3);	/* SIDLEMODE: smart idle */
	l = FLD_MOD(l, 1, 2, 2);	/* ENWAKEUP */
	l = FLD_MOD(l, 1, 0, 0);	/* AUTOIDLE */
	dispc_write_reg(DISPC_SYSCONFIG, l);

	/* FUNCGATED */
	REG_FLD_MOD(DISPC_CONFIG, 1, 9, 9);

	/* L3 firewall setting: enable access to OCM RAM */
	__raw_writel(0x402000b0, IO_ADDRESS(0x680050a0));

	_dispc_setup_color_conv_coef();

	dispc_set_loadmode(OMAP_DSS_LOAD_FRAME_ONLY);

	/* Set logic clock to fck, pixel clock to fck/2 for now */
	dispc_set_lcd_divisor(1, 2);
}

int dispc_init(void)
{
	u32 rev;

	spin_lock_init(&dispc.irq_lock);

	dispc.base = ioremap(DISPC_BASE, DISPC_SZ_REGS);
	if (!dispc.base) {
		DSSERR("can't ioremap DISPC\n");
		return -ENOMEM;
	}

	if (cpu_is_omap34xx()) {
		dispc.dpll4_m4_ck = clk_get(NULL, "dpll4_m4_ck");
		if (IS_ERR(dispc.dpll4_m4_ck)) {
			DSSERR("Failed to get dpll4_m4_ck\n");
			return -ENODEV;
		}
	}

	enable_clocks(1);

	_omap_dispc_initial_config();

	_omap_dispc_initialize_irq();

	dispc_save_context();

	rev = dispc_read_reg(DISPC_REVISION);
	printk(KERN_INFO "OMAP DISPC rev %d.%d\n",
	       FLD_GET(rev, 7, 4), FLD_GET(rev, 3, 0));

	enable_clocks(0);

	return 0;
}

void dispc_exit(void)
{
	if (cpu_is_omap34xx())
		clk_put(dispc.dpll4_m4_ck);
	iounmap(dispc.base);
}

int dispc_enable_plane(enum omap_plane plane, int enable)
{
	DSSDBG("dispc_enable_plane %d, %d\n", plane, enable);

	enable_clocks(1);
	_dispc_enable_plane(plane, enable);
	enable_clocks(0);

	return 0;
}

int dispc_setup_plane(enum omap_plane plane, enum omap_channel channel_out,
		       u32 paddr, int tv_field1_offset, int screen_width,
		       int pos_x, int pos_y,
		       int width, int height,
		       int out_width, int out_height,
		       enum omap_color_mode color_mode,
		       int ilace, int rotation, int mirror, int global_alpha)
{
	int r = 0;

	DSSDBG("dispc_setup_plane %d, %x, sw %d, %d,%d, %dx%d -> "
	       "%dx%d, (ilace %d)\n",
	       plane, paddr, screen_width, pos_x, pos_y,
	       width, height,
	       out_width, out_height,
	       ilace);
	enable_clocks(1);

	r = _dispc_setup_plane(plane, channel_out,
			   paddr, tv_field1_offset, screen_width,
			   pos_x, pos_y,
			   width, height,
			   out_width, out_height,
			   color_mode, ilace, rotation, mirror, global_alpha);

	enable_clocks(0);

	return r;
}

static int dispc_is_intersecting(int x1, int y1, int w1, int h1,
				 int x2, int y2, int w2, int h2)
{
	if (x1 >= (x2+w2))
		return 0;

	if ((x1+w1) <= x2)
		return 0;

	if (y1 >= (y2+h2))
		return 0;

	if ((y1+h1) <= y2)
		return 0;

	return 1;
}

static int dispc_is_overlay_scaled(struct omap_overlay_info *pi)
{
	if (pi->width != pi->out_width)
		return 1;

	if (pi->height != pi->out_height)
		return 1;

	return 0;
}

/* returns the area that needs updating */
void dispc_setup_partial_planes(struct omap_display *display,
				    int *xi, int *yi, int *wi, int *hi)
{
	struct omap_overlay_manager *mgr;
	int i;

	int x, y, w, h;

	x = *xi;
	y = *yi;
	w = *wi;
	h = *hi;

	DSSDBG("dispc_setup_partial_planes %d,%d %dx%d\n",
		*xi, *yi, *wi, *hi);

	mgr = display->manager;

	if (!mgr) {
		DSSDBG("no manager\n");
		return;
	}

	for (i = 0; i < mgr->num_overlays; i++) {
		struct omap_overlay *ovl;
		struct omap_overlay_info *pi;
		ovl = &mgr->overlays[i];

		if (ovl->manager != mgr)
			continue;

		if ((ovl->caps & OMAP_DSS_OVL_CAP_SCALE) == 0)
			continue;

		pi = &ovl->info;

		if (!pi->enabled)
			continue;
		/*
		 * If the plane is intersecting and scaled, we
		 * enlarge the update region to accomodate the
		 * whole area
		 */

		if (dispc_is_intersecting(x, y, w, h,
					  pi->pos_x, pi->pos_y,
					  pi->out_width, pi->out_height)) {
			if (dispc_is_overlay_scaled(pi)) {

				int x1, y1, x2, y2;

				if (x > pi->pos_x)
					x1 = pi->pos_x;
				else
					x1 = x;

				if (y > pi->pos_y)
					y1 = pi->pos_y;
				else
					y1 = y;

				if ((x + w) < (pi->pos_x + pi->out_width))
					x2 = pi->pos_x + pi->out_width;
				else
					x2 = x + w;

				if ((y + h) < (pi->pos_y + pi->out_height))
					y2 = pi->pos_y + pi->out_height;
				else
					y2 = y + h;

				x = x1;
				y = y1;
				w = x2 - x1;
				h = y2 - y1;

				DSSDBG("Update area after enlarge due to "
					"scaling %d, %d %dx%d\n",
					x, y, w, h);
			}
		}
	}

	for (i = 0; i < mgr->num_overlays; i++) {
		struct omap_overlay *ovl = &mgr->overlays[i];
		struct omap_overlay_info *pi = &ovl->info;

		int px = pi->pos_x;
		int py = pi->pos_y;
		int pw = pi->width;
		int ph = pi->height;
		int pow = pi->out_width;
		int poh = pi->out_height;
		u32 pa = pi->paddr;
		int psw = pi->screen_width;
		int bpp;

		if (ovl->manager != mgr)
			continue;

		/*
		 * If plane is not enabled or the update region
		 * does not intersect with the plane in question,
		 * we really disable the plane from hardware
		 */

		if (!pi->enabled ||
		    !dispc_is_intersecting(x, y, w, h,
					   px, py, pow, poh)) {
			dispc_enable_plane(ovl->id, 0);
			continue;
		}

		switch (pi->color_mode) {
		case OMAP_DSS_COLOR_RGB16:
			bpp = 16;
			break;

		case OMAP_DSS_COLOR_RGB24P:
			bpp = 24;
			break;

		case OMAP_DSS_COLOR_RGB24U:
			bpp = 32;
			break;

		case OMAP_DSS_COLOR_YUV2:
		case OMAP_DSS_COLOR_UYVY:
			bpp = 16;
			break;

		default:
			BUG();
			return;
		}

		if (x > pi->pos_x) {
			px = 0;
			pw -= (x - pi->pos_x);
			pa += (x - pi->pos_x) * bpp / 8;
		} else {
			px = pi->pos_x - x;
		}

		if (y > pi->pos_y) {
			py = 0;
			ph -= (y - pi->pos_y);
			pa += (y - pi->pos_y) * psw * bpp / 8;
		} else {
			py = pi->pos_y - y;
		}

		if (w < (px+pw))
			pw -= (px+pw) - (w);

		if (h < (py+ph))
			ph -= (py+ph) - (h);

		/* Can't scale the GFX plane */
		if ((ovl->caps & OMAP_DSS_OVL_CAP_SCALE) == 0 ||
				dispc_is_overlay_scaled(pi) == 0) {
			pow = pw;
			poh = ph;
		}

		DSSDBG("calc  plane %d, %x, sw %d, %d,%d, %dx%d -> %dx%d\n",
				ovl->id, pa, psw, px, py, pw, ph, pow, poh);

		dispc_setup_plane(ovl->id, mgr->id,
				pa, pi->tv_field1_offset, psw,
				px, py,
				pw, ph,
				pow, poh,
				pi->color_mode, 0, 0, 0, 0);

		dispc_enable_plane(ovl->id, 1);
	}

	*xi = x;
	*yi = y;
	*wi = w;
	*hi = h;

}

