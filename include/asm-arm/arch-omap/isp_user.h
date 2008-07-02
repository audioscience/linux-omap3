/*
 * include/asm-arm/arch-omap/isp_user.h
 *
 * Include file for OMAP ISP module in TI's OMAP3430.
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

#ifndef OMAP_ISP_USER_H
#define OMAP_ISP_USER_H

/* AE/AWB related structures and flags*/

/* Flags for update field */
#define REQUEST_STATISTICS	(1 << 0)
#define SET_COLOR_GAINS		(1 << 1)
#define SET_DIGITAL_GAIN	(1 << 2)
#define SET_EXPOSURE		(1 << 3)
#define SET_ANALOG_GAIN		(1 << 4)

#define MAX_FRAME_COUNT		0x0FFF
#define MAX_FUTURE_FRAMES	10

/**
 * struct isph3a_aewb_config - AE AWB configuration reset values.
 * saturation_limit: Saturation limit.
 * @win_height: Window Height. Range 2 - 256, even values only.
 * @win_width: Window Width. Range 6 - 256, even values only.
 * @ver_win_count: Vertical Window Count. Range 1 - 128.
 * @hor_win_count: Horizontal Window Count. Range 1 - 36.
 * @ver_win_start: Vertical Window Start. Range 0 - 4095.
 * @hor_win_start: Horizontal Window Start. Range 0 - 4095.
 * @blk_ver_win_start: Black Vertical Windows Start. Range 0 - 4095.
 * @blk_win_height: Black Window Height. Range 2 - 256, even values only.
 * @subsample_ver_inc: Subsample Vertical points increment Range 2 - 32, even
 *                     values only.
 * @subsample_hor_inc: Subsample Horizontal points increment Range 2 - 32, even
 *                     values only.
 * @alaw_enable: AEW ALAW EN flag.
 * @aewb_enable: AE AWB stats generation EN flag.
 */
struct isph3a_aewb_config {
	u16 saturation_limit;
	u16 win_height;
	u16 win_width;
	u16 ver_win_count;
	u16 hor_win_count;
	u16 ver_win_start;
	u16 hor_win_start;
	u16 blk_ver_win_start;
	u16 blk_win_height;
	u16 subsample_ver_inc;
	u16 subsample_hor_inc;
	u8 alaw_enable;
	u8 aewb_enable;
};

/**
 * struct isph3a_aewb_data - Structure of data sent to or received from user
 * @h3a_aewb_statistics_buf: Pointer to pass to user.
 * @shutter: Shutter speed.
 * @gain: Sensor analog Gain.
 * @shutter_cap: Shutter speed for capture.
 * @gain_cap: Sensor Gain for capture.
 * @dgain: White balance digital gain.
 * @wb_gain_b: White balance color gain blue.
 * @wb_gain_r: White balance color gain red.
 * @wb_gain_gb: White balance color gain green blue.
 * @wb_gain_gr: White balance color gain green red.
 * @frame_number: Frame number of requested stats.
 * @curr_frame: Current frame number being processed.
 * @update: Bitwise flags to update parameters.
 * @ts: Timestamp of returned framestats.
 * @field_count: Sequence number of returned framestats.
 */
struct isph3a_aewb_data {
	void *h3a_aewb_statistics_buf;
	u32 shutter;
	u16 gain;
	u32 shutter_cap;
	u16 gain_cap;
	u16 dgain;
	u16 wb_gain_b;
	u16 wb_gain_r;
	u16 wb_gain_gb;
	u16 wb_gain_gr;
	u16 frame_number;
	u16 curr_frame;
	u8 update;
	struct timeval ts;
	unsigned long field_count;
};


/* Histogram related structs */
/* Flags for number of bins */
#define BINS_32			0x0
#define BINS_64			0x1
#define BINS_128		0x2
#define BINS_256		0x3

struct isp_hist_config {
	u8 hist_source;		/* CCDC or Memory */
	u8 input_bit_width;	/* Needed o know the size per pixel */
	u8 hist_frames;		/* Num of frames to be processed and
				 * accumulated
				 */
	u8 hist_h_v_info;	/* frame-input width and height if source is
				 * memory
				 */
	u16 hist_radd;		/* frame-input address in memory */
	u16 hist_radd_off;	/* line-offset for frame-input */
	u16 hist_bins;		/* number of bins: 32, 64, 128, or 256 */
	u16 wb_gain_R;		/* White Balance Field-to-Pattern Assignments */
	u16 wb_gain_RG;		/* White Balance Field-to-Pattern Assignments */
	u16 wb_gain_B;		/* White Balance Field-to-Pattern Assignments */
	u16 wb_gain_BG;		/* White Balance Field-to-Pattern Assignments */
	u8 num_regions;		/* number of regions to be configured */
	u16 reg0_hor;		/* Region 0 size and position */
	u16 reg0_ver;		/* Region 0 size and position */
	u16 reg1_hor;		/* Region 1 size and position */
	u16 reg1_ver;		/* Region 1 size and position */
	u16 reg2_hor;		/* Region 2 size and position */
	u16 reg2_ver;		/* Region 2 size and position */
	u16 reg3_hor;		/* Region 3 size and position */
	u16 reg3_ver;		/* Region 3 size and position */
};

struct isp_hist_data {

	u32 *hist_statistics_buf;	/* Pointer to pass to user */

};


/* ISP CCDC structs */

/* Abstraction layer CCDC configurations */
#define ISP_ABS_CCDC_ALAW		(1 << 0)
#define ISP_ABS_CCDC_LPF 		(1 << 1)
#define ISP_ABS_CCDC_BLCLAMP		(1 << 2)
#define ISP_ABS_CCDC_BCOMP		(1 << 3)
#define ISP_ABS_CCDC_FPC		(1 << 4)
#define ISP_ABS_CCDC_CULL		(1 << 5)
#define ISP_ABS_CCDC_COLPTN		(1 << 6)
#define ISP_ABS_CCDC_CONFIG_LSC		(1 << 7)
#define ISP_ABS_TBL_LSC			(1 << 8)

#define RGB_MAX				3

/* Enumeration constants for Alaw input width */
enum alaw_ipwidth {
	ALAW_BIT12_3 = 0x3,
	ALAW_BIT11_2 = 0x4,
	ALAW_BIT10_1 = 0x5,
	ALAW_BIT9_0 = 0x6
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
 * @lsc: Pointer to LSC gain table.
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
	u8 *lsc;
};

/* Preivew configuration */

/*Abstraction layer preview configurations*/
#define ISP_ABS_PREV_LUMAENH		(1 << 0)
#define ISP_ABS_PREV_INVALAW		(1 << 1)
#define ISP_ABS_PREV_HRZ_MED		(1 << 2)
#define ISP_ABS_PREV_CFA		(1 << 3)
#define ISP_ABS_PREV_CHROMA_SUPP	(1 << 4)
#define ISP_ABS_PREV_WB			(1 << 5)
#define ISP_ABS_PREV_BLKADJ		(1 << 6)
#define ISP_ABS_PREV_RGB2RGB		(1 << 7)
#define ISP_ABS_PREV_COLOR_CONV		(1 << 8)
#define ISP_ABS_PREV_YC_LIMIT		(1 << 9)
#define ISP_ABS_PREV_DEFECT_COR		(1 << 10)
#define ISP_ABS_PREV_GAMMABYPASS	(1 << 11)
#define ISP_ABS_TBL_NF 			(1 << 12)
#define ISP_ABS_TBL_REDGAMMA		(1 << 13)
#define ISP_ABS_TBL_GREENGAMMA		(1 << 14)
#define ISP_ABS_TBL_BLUEGAMMA		(1 << 15)

/**
 * struct ispprev_hmed - Structure for Horizontal Median Filter.
 * @odddist: Distance between consecutive pixels of same color in the odd line.
 * @evendist: Distance between consecutive pixels of same color in the even
 *            line.
 * @thres: Horizontal median filter threshold.
 */
struct ispprev_hmed {
	u8 odddist;
	u8 evendist;
	u8 thres;
};

/*
 * Enumeration for CFA Formats supported by preview
 */
enum cfa_fmt {
	CFAFMT_BAYER, CFAFMT_SONYVGA, CFAFMT_RGBFOVEON,
	CFAFMT_DNSPL, CFAFMT_HONEYCOMB, CFAFMT_RRGGBBFOVEON
};

/**
 * struct ispprev_cfa - Structure for CFA Inpterpolation.
 * @cfafmt: CFA Format Enum value supported by preview.
 * @cfa_gradthrs_vert: CFA Gradient Threshold - Vertical.
 * @cfa_gradthrs_horz: CFA Gradient Threshold - Horizontal.
 * @cfa_table: Pointer to the CFA table.
 */
struct ispprev_cfa {
	enum cfa_fmt cfafmt;
	u8 cfa_gradthrs_vert;
	u8 cfa_gradthrs_horz;
	u32 *cfa_table;
};

/**
 * struct ispprev_csup - Structure for Chrominance Suppression.
 * @gain: Gain.
 * @thres: Threshold.
 * @hypf_en: Flag to enable/disable the High Pass Filter.
 */
struct ispprev_csup {
	u8 gain;
	u8 thres;
	u8 hypf_en;
};

/**
 * struct ispprev_wbal - Structure for White Balance.
 * @dgain: Digital gain (U10Q8).
 * @coef3: White balance gain - COEF 3 (U8Q5).
 * @coef2: White balance gain - COEF 2 (U8Q5).
 * @coef1: White balance gain - COEF 1 (U8Q5).
 * @coef0: White balance gain - COEF 0 (U8Q5).
 */
struct ispprev_wbal {
	u16 dgain;
	u8 coef3;
	u8 coef2;
	u8 coef1;
	u8 coef0;
};

/**
 * struct ispprev_blkadj - Structure for Black Adjustment.
 * @red: Black level offset adjustment for Red in 2's complement format
 * @green: Black level offset adjustment for Green in 2's complement format
 * @blue: Black level offset adjustment for Blue in 2's complement format
 */
struct ispprev_blkadj {
	/*Black level offset adjustment for Red in 2's complement format */
	u8 red;
	/*Black level offset adjustment for Green in 2's complement format */
	u8 green;
	/* Black level offset adjustment for Blue in 2's complement format */
	u8 blue;
};

/**
 * struct ispprev_rgbtorgb - Structure for RGB to RGB Blending.
 * @matrix: Blending values(S12Q8 format)
 *              [RR] [GR] [BR]
 *              [RG] [GG] [BG]
 *              [RB] [GB] [BB]
 * @offset: Blending offset value for R,G,B in 2's complement integer format.
 */
struct ispprev_rgbtorgb {
	u16 matrix[3][3];
	u16 offset[3];
};

/**
 * struct ispprev_csc - Structure for Color Space Conversion from RGB-YCbYCr
 * @matrix: Color space conversion coefficients(S10Q8)
 *              [CSCRY]  [CSCGY]  [CSCBY]
 *              [CSCRCB] [CSCGCB] [CSCBCB]
 *              [CSCRCR] [CSCGCR] [CSCBCR]
 * @offset: CSC offset values for Y offset, CB offset and CR offset respectively
 */
struct ispprev_csc {
	u16 matrix[RGB_MAX][RGB_MAX];
	s16 offset[RGB_MAX];
};

/**
 * struct ispprev_yclimit - Structure for Y, C Value Limit.
 * @minC: Minimum C value
 * @maxC: Maximum C value
 * @minY: Minimum Y value
 * @maxY: Maximum Y value
 */
struct ispprev_yclimit {
	u8 minC;
	u8 maxC;
	u8 minY;
	u8 maxY;
};

/**
 * struct ispprev_dcor - Structure for Defect correction.
 * @couplet_mode_en: Flag to enable or disable the couplet dc Correction in NF
 * @detect_correct: Thresholds for correction bit 0:10 detect 16:25 correct
 */
struct ispprev_dcor {
	u8 couplet_mode_en;
	u32 detect_correct[4];
};

/**
 * struct ispprev_nf - Structure for Noise Filter
 * @spread: Spread value to be used in Noise Filter
 * @table: Pointer to the Noise Filter table
 */
struct ispprev_nf {
	u8 spread;
	u32 table[64];
};

/**
 * struct ispprv_update_config - Structure for Preview Configuration (user).
 * @update: Specifies which ISP Preview registers should be updated.
 * @flag: Specifies which ISP Preview functions should be enabled.
 * @yen: Pointer to luma enhancement table.
 * @shading_shift: 3bit value of shift used in shading compensation.
 * @prev_hmed: Pointer to structure containing the odd and even distance.
 *             between the pixels in the image along with the filter threshold.
 * @prev_cfa: Pointer to structure containing the CFA interpolation table, CFA.
 *            format in the image, vertical and horizontal gradient threshold.
 * @csup: Pointer to Structure for Chrominance Suppression coefficients.
 * @prev_wbal: Pointer to structure for White Balance.
 * @prev_blkadj: Pointer to structure for Black Adjustment.
 * @rgb2rgb: Pointer to structure for RGB to RGB Blending.
 * @prev_csc: Pointer to structure for Color Space Conversion from RGB-YCbYCr.
 * @yclimit: Pointer to structure for Y, C Value Limit.
 * @prev_dcor: Pointer to structure for defect correction.
 * @prev_nf: Pointer to structure for Noise Filter
 * @red_gamma: Pointer to red gamma correction table.
 * @green_gamma: Pointer to green gamma correction table.
 * @blue_gamma: Pointer to blue gamma correction table.
 */
struct ispprv_update_config {
	u16 update;
	u16 flag;
	void *yen;
	u32 shading_shift;
	struct ispprev_hmed *prev_hmed;
	struct ispprev_cfa *prev_cfa;
	struct ispprev_csup *csup;
	struct ispprev_wbal *prev_wbal;
	struct ispprev_blkadj *prev_blkadj;
	struct ispprev_rgbtorgb *rgb2rgb;
	struct ispprev_csc *prev_csc;
	struct ispprev_yclimit *yclimit;
	struct ispprev_dcor *prev_dcor;
	struct ispprev_nf *prev_nf;
	u32 *red_gamma;
	u32 *green_gamma;
	u32 *blue_gamma;
};

/**
 * struct isptables_update - Structure for Table Configuration.
 * @update: Specifies which tables should be updated.
 * @flag: Specifies which tables should be enabled.
 * @prev_nf: Pointer to structure for Noise Filter
 * @lsc: Pointer to LSC gain table. (currently not used)
 * @red_gamma: Pointer to red gamma correction table.
 * @green_gamma: Pointer to green gamma correction table.
 * @blue_gamma: Pointer to blue gamma correction table.
 */
struct isptables_update {
	u16 update;
	u16 flag;
	struct ispprev_nf *prev_nf;
	u32 *lsc;
	u32 *red_gamma;
	u32 *green_gamma;
	u32 *blue_gamma;
};

#endif
