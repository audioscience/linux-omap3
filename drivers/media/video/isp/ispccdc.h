/*
 * drivers/media/video/isp/ispccdc.h
 *
 * Driver include file for CCDC module in TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_CCDC_H
#define OMAP_ISP_CCDC_H

/* Abstraction layer CCDC configurations */
#define ISP_ABS_CCDC_ALAW		(1 << 0)
#define ISP_ABS_CCDC_LPF 		(1 << 1)
#define ISP_ABS_CCDC_BLCLAMP		(1 << 2)
#define ISP_ABS_CCDC_BCOMP		(1 << 3)
#define ISP_ABS_CCDC_FPC		(1 << 4)
#define ISP_ABS_CCDC_CULL		(1 << 5)
#define ISP_ABS_CCDC_COLPTN		(1 << 6)
#define ISP_ABS_CCDC_CONFIG_LSC		(1 << 7)

#define ISP_ABS_TBL_LSC			(1 << 0)

#ifndef CONFIG_ARCH_OMAP3410
# define cpu_is_omap3410()		0
# define is_isplsc_activated()		1
# include "isppreview.h"
# define USE_ISP_LSC
#else
# define cpu_is_omap3410()		1
# define is_isplsc_activated()		0
# undef USE_ISP_LSC
#endif

#ifdef OMAP_ISPCCDC_DEBUG
# define is_ispccdc_debug_enabled()		1
#else
# define is_ispccdc_debug_enabled()		0
#endif

/* Enumeration constants for CCDC input output format */
enum ccdc_input {
	CCDC_RAW,
	CCDC_YUV_SYNC,
	CCDC_YUV_BT,
	CCDC_OTHERS
};

enum ccdc_output {
	CCDC_YUV_RSZ,
	CCDC_YUV_MEM_RSZ,
	CCDC_OTHERS_VP,
	CCDC_OTHERS_MEM,
	CCDC_OTHERS_VP_MEM
};

/* Enumeration constants for the sync interface parameters */
enum inpmode {
	RAW,
	YUV16,
	YUV8
};
enum datasize {
	DAT8,
	DAT10,
	DAT11,
	DAT12
};

/* Enumeration constants for Video Port */
enum vpin {
	BIT12_3 = 3,
	BIT11_2 = 4,
	BIT10_1 = 5,
	BIT9_0 = 6
};

enum vpif_freq {
	PIXCLKBY2,
	PIXCLKBY3_5,
	PIXCLKBY4_5,
	PIXCLKBY5_5,
	PIXCLKBY6_5
};

/* Enumeration constants for Alaw input width */
enum alaw_ipwidth {
	ALAW_BIT12_3 = 0x3,
	ALAW_BIT11_2 = 0x4,
	ALAW_BIT10_1 = 0x5,
	ALAW_BIT9_0 = 0x6
};
/**
 * struct ispccdc_syncif - Structure for Sync Interface between sensor and CCDC
 * @ccdc_mastermode: Master mode. 1 - Master, 0 - Slave.
 * @fldstat: Field state. 0 - Odd Field, 1 - Even Field.
 * @ipmod: Input mode.
 * @datsz: Data size.
 * @fldmode: 0 - Progressive, 1 - Interlaced.
 * @datapol: 0 - Positive, 1 - Negative.
 * @fldpol: 0 - Positive, 1 - Negative.
 * @hdpol: 0 - Positive, 1 - Negative.
 * @vdpol: 0 - Positive, 1 - Negative.
 * @fldout: 0 - Input, 1 - Output.
 * @hs_width: Width of the Horizontal Sync pulse, used for HS/VS Output.
 * @vs_width: Width of the Vertical Sync pulse, used for HS/VS Output.
 * @ppln: Number of pixels per line, used for HS/VS Output.
 * @hlprf: Number of half lines per frame, used for HS/VS Output.
 * @bt_r656_en: 1 - Enable ITU-R BT656 mode, 0 - Sync mode.
 */
struct ispccdc_syncif {
	u8 ccdc_mastermode;
	u8 fldstat;
	enum inpmode ipmod;
	enum datasize datsz;
	u8 fldmode;
	u8 datapol;
	u8 fldpol;
	u8 hdpol;
	u8 vdpol;
	u8 fldout;
	u8 hs_width;
	u8 vs_width;
	u8 ppln;
	u8 hlprf;
	u8 bt_r656_en;
};

/**
 * struct ispccdc_lsc_config - Structure for LSC configuration.
 * @offset: Table Offset of the gain table.
 * @gain_mode_n: Vertical dimension of a paxel in LSC configuration.
 * @gain_mode_m: Horizontal dimension of a paxel in LSC configuration.
 * @gain_format: Gain table format.
 * @fmtsph: Start pixel horizontal from start of the HS sync pulse.
 * @fmtlnh: Number of pixels in horizontal direction to use for the data
 *          reformatter.
 * @fmtslv: Start line from start of VS sync pulse for the data reformatter.
 * @fmtlnv: Number of lines in vertical direction for the data reformatter.
 * @initial_x: X position, in pixels, of the first active pixel in reference
 *             to the first active paxel. Must be an even number.
 * @initial_y: Y position, in pixels, of the first active pixel in reference
 *             to the first active paxel. Must be an even number.
 * @size: Size of LSC gain table. Filled when loaded from userspace.
 */
struct ispccdc_lsc_config {
	u8 offset;
	u8 gain_mode_n;
	u8 gain_mode_m;
	u8 gain_format;
	u16 fmtsph;
	u16 fmtlnh;
	u16 fmtslv;
	u16 fmtlnv;
	u8 initial_x;
	u8 initial_y;
	u32 size;
};

/**
 * struct ispccdc_bclamp - Structure for Optical & Digital black clamp subtract
 * @obgain: Optical black average gain.
 * @obstpixel: Start Pixel w.r.t. HS pulse in Optical black sample.
 * @oblines: Optical Black Sample lines.
 * @oblen: Optical Black Sample Length.
 * @dcsubval: Digital Black Clamp subtract value.
 */
struct ispccdc_bclamp {
	u8 obgain;
	u8 obstpixel;
	u8 oblines;
	u8 oblen;
	u16 dcsubval;
};

/**
 * ispccdc_fpc - Structure for FPC
 * @fpnum: Number of faulty pixels to be corrected in the frame.
 * @fpcaddr: Memory address of the FPC Table
 */
struct ispccdc_fpc {
	u16 fpnum;
	u32 fpcaddr;
};

/**
 * ispccdc_blcomp - Structure for Black Level Compensation parameters.
 * @b_mg: B/Mg pixels. 2's complement. -128 to +127.
 * @gb_g: Gb/G pixels. 2's complement. -128 to +127.
 * @gr_cy: Gr/Cy pixels. 2's complement. -128 to +127.
 * @r_ye: R/Ye pixels. 2's complement. -128 to +127.
 */
struct ispccdc_blcomp {
	u8 b_mg;
	u8 gb_g;
	u8 gr_cy;
	u8 r_ye;
};

/**
 * struct ispccdc_vp - Structure for Video Port parameters
 * @bitshift_sel: Video port input select. 3 - bits 12-3, 4 - bits 11-2,
 *                5 - bits 10-1, 6 - bits 9-0.
 * @freq_sel: Video port data ready frequency. 1 - 1/3.5, 2 - 1/4.5,
 *            3 - 1/5.5, 4 - 1/6.5.
 */
struct ispccdc_vp {
	enum vpin bitshift_sel;
	enum vpif_freq freq_sel;
};

/**
 * ispccdc_refmt - Structure for Reformatter parameters
 * @lnalt: Line alternating mode enable. 0 - Enable, 1 - Disable.
 * @lnum: Number of output lines from 1 input line. 1 to 4 lines.
 * @plen_even: Number of program entries in even line minus 1.
 * @plen_odd: Number of program entries in odd line minus 1.
 * @prgeven0: Program entries 0-7 for even lines register
 * @prgeven1: Program entries 8-15 for even lines register
 * @prgodd0: Program entries 0-7 for odd lines register
 * @prgodd1: Program entries 8-15 for odd lines register
 * @fmtaddr0: Output line in which the original pixel is to be placed
 * @fmtaddr1: Output line in which the original pixel is to be placed
 * @fmtaddr2: Output line in which the original pixel is to be placed
 * @fmtaddr3: Output line in which the original pixel is to be placed
 * @fmtaddr4: Output line in which the original pixel is to be placed
 * @fmtaddr5: Output line in which the original pixel is to be placed
 * @fmtaddr6: Output line in which the original pixel is to be placed
 * @fmtaddr7: Output line in which the original pixel is to be placed
 */
struct ispccdc_refmt {
	u8 lnalt;
	u8 lnum;
	u8 plen_even;
	u8 plen_odd;
	u32 prgeven0;
	u32 prgeven1;
	u32 prgodd0;
	u32 prgodd1;
	u32 fmtaddr0;
	u32 fmtaddr1;
	u32 fmtaddr2;
	u32 fmtaddr3;
	u32 fmtaddr4;
	u32 fmtaddr5;
	u32 fmtaddr6;
	u32 fmtaddr7;
};

/**
 * ispccdc_culling - Structure for Culling parameters.
 * @v_pattern: Vertical culling pattern.
 * @h_odd: Horizontal Culling pattern for odd lines.
 * @h_even: Horizontal Culling pattern for even lines.
 */
struct ispccdc_culling {
	u8 v_pattern;
	u16 h_odd;
	u16 h_even;
};

/**
 * ispccdc_update_config - Structure for CCDC configuration.
 * @update: Specifies which CCDC registers should be updated.
 * @flag: Specifies which CCDC functions should be enabled.
 * @alawip: Enable/Disable A-Law compression.
 * @bclamp: Black clamp control register.
 * @blcomp: Black level compensation value for RGrGbB Pixels. 2's complement.
 * @fpc: Number of faulty pixels corrected in the frame, address of FPC table.
 * @cull: Cull control register.
 * @colptn: Color pattern of the sensor.
 */
struct ispccdc_update_config {
	u16 update;
	u16 flag;
	enum alaw_ipwidth alawip;
	struct ispccdc_bclamp *bclamp;
	struct ispccdc_blcomp *blcomp;
	struct ispccdc_fpc *fpc;
	struct ispccdc_lsc_config *lsc_cfg;
	struct ispccdc_culling *cull;
	u32 colptn;
};

int ispccdc_request(void);

int ispccdc_free(void);

int ispccdc_config_datapath(enum ccdc_input input, enum ccdc_output output);

void ispccdc_config_crop(u32 left, u32 top, u32 height, u32 width);

void ispccdc_config_sync_if(struct ispccdc_syncif syncif);

int ispccdc_config_black_clamp(struct ispccdc_bclamp bclamp);

void ispccdc_enable_black_clamp(u8 enable);

int ispccdc_config_fpc(struct ispccdc_fpc fpc);

void ispccdc_enable_fpc(u8 enable);

void ispccdc_config_black_comp(struct ispccdc_blcomp blcomp);

void ispccdc_config_vp(struct ispccdc_vp vp);

void ispccdc_enable_vp(u8 enable);

void ispccdc_config_reformatter(struct ispccdc_refmt refmt);

void ispccdc_enable_reformatter(u8 enable);

void ispccdc_config_culling(struct ispccdc_culling culling);

void ispccdc_enable_lpf(u8 enable);

void ispccdc_config_alaw(enum alaw_ipwidth ipwidth);

void ispccdc_enable_alaw(u8 enable);

int ispccdc_load_lsc(u32 table_size);

void ispccdc_config_lsc(struct ispccdc_lsc_config *lsc_cfg);

void ispccdc_enable_lsc(u8 enable);

void ispccdc_config_imgattr(u32 colptn);

void ispccdc_config_shadow_registers(void);

int ispccdc_try_size(u32 input_w, u32 input_h, u32 *output_w, u32 *output_h);

int ispccdc_config_size(u32 input_w, u32 input_h, u32 output_w, u32 output_h);

int ispccdc_config_outlineoffset(u32 offset, u8 oddeven, u8 numlines);

int ispccdc_set_outaddr(u32 addr);

void ispccdc_enable(u8 enable);

int ispccdc_busy(void);

void ispccdc_save_context(void);

void ispccdc_restore_context(void);

void ispccdc_print_status(void);

int omap34xx_isp_ccdc_config(void *userspace_add);

int omap34xx_isp_lsc_update(void *userspace_add);

#endif		/* OMAP_ISP_CCDC_H */
