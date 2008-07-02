/*
 * drivers/media/video/isp/isppreview.h
 *
 * Driver include file for Preview module in TI's OMAP3430 Camera ISP
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

#ifndef OMAP_ISP_PREVIEW_H
#define OMAP_ISP_PREVIEW_H

/* Isp query control structure */

#define ISPPRV_BRIGHT_STEP		0x1
#define ISPPRV_BRIGHT_DEF		0x1
#define ISPPRV_BRIGHT_LOW		0x0
#define ISPPRV_BRIGHT_HIGH		0xF
#define ISPPRV_BRIGHT_UNITS		0x7

#define ISPPRV_CONTRAST_STEP		0x1
#define ISPPRV_CONTRAST_DEF		0x2
#define ISPPRV_CONTRAST_LOW		0x0
#define ISPPRV_CONTRAST_HIGH		0xF
#define ISPPRV_CONTRAST_UNITS		0x5

#define NO_AVE				0x0
#define AVE_2_PIX			0x1
#define AVE_4_PIX			0x2
#define AVE_8_PIX			0x3
#define AVE_ODD_PIXEL_DIST		(1 << 4) /* For Bayer Sensors */
#define AVE_EVEN_PIXEL_DIST		(1 << 2)

#define WB_GAIN_MAX			4
#define RGB_MAX				3

/* Features list */
#define PREV_AVERAGER			(1 << 0)
#define PREV_INVERSE_ALAW 		(1 << 1)
#define PREV_HORZ_MEDIAN_FILTER		(1 << 2)
#define PREV_NOISE_FILTER 		(1 << 3)
#define PREV_CFA			(1 << 4)
#define PREV_GAMMA_BYPASS		(1 << 5)
#define PREV_LUMA_ENHANCE		(1 << 6)
#define PREV_CHROMA_SUPPRESS		(1 << 7)
#define PREV_DARK_FRAME_SUBTRACT	(1 << 8)
#define PREV_LENS_SHADING		(1 << 9)
#define PREV_DARK_FRAME_CAPTURE		(1 << 10)
#define PREV_DEFECT_COR			(1 << 11)

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

#define ISP_NF_TABLE_SIZE 		(1 << 10)

#define ISP_GAMMA_TABLE_SIZE 		(1 << 10)

/*
 *Enumeration Constants for input and output format
 */
enum preview_input {
	PRV_RAW_CCDC,
	PRV_RAW_MEM,
	PRV_RGBBAYERCFA,
	PRV_COMPCFA,
	PRV_CCDC_DRKF,
	PRV_OTHERS
};
enum preview_output {
	PREVIEW_RSZ,
	PREVIEW_MEM
};
/*
 * Configure byte layout of YUV image
 */
enum preview_ycpos_mode {
	YCPOS_YCrYCb = 0,
	YCPOS_YCbYCr = 1,
	YCPOS_CbYCrY = 2,
	YCPOS_CrYCbY = 3
};

enum preview_color_effect {
	PREV_DEFAULT_COLOR = 0,
	PREV_BW_COLOR = 1,
	PREV_SEPIA_COLOR = 2
};

/*
 * Enumeration for CFA Formats supported by preview
 */
enum cfa_fmt {
	CFAFMT_BAYER, CFAFMT_SONYVGA, CFAFMT_RGBFOVEON,
	CFAFMT_DNSPL, CFAFMT_HONEYCOMB, CFAFMT_RRGGBBFOVEON
};

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
 * struct ispprev_dcor - Structure for Defect correction.
 * @couplet_mode_en: Flag to enable or disable the couplet dc Correction in NF
 * @detect_correct: Thresholds for correction bit 0:10 detect 16:25 correct
 */
struct ispprev_dcor {
	u8 couplet_mode_en;
	u32 detect_correct[4];
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
 * struct ispprev_gtable - Structure for Gamma Correction.
 * @redtable: Pointer to the red gamma table.
 * @greentable: Pointer to the green gamma table.
 * @bluetable: Pointer to the blue gamma table.
 */
struct ispprev_gtable {
	u32 *redtable;
	u32 *greentable;
	u32 *bluetable;
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
 * struct prev_white_balance - Structure for White Balance 2.
 * @wb_dgain: White balance common gain.
 * @wb_gain: Individual color gains.
 * @wb_coefmatrix: Coefficient matrix
 */
struct prev_white_balance {
	u16 wb_dgain; /* white balance common gain */
	u8 wb_gain[WB_GAIN_MAX]; /* individual color gains */
	u8 wb_coefmatrix[WB_GAIN_MAX][WB_GAIN_MAX];
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
 * struct prev_size_params - Structure for size parameters.
 * @hstart: Starting pixel.
 * @vstart: Starting line.
 * @hsize: Width of input image.
 * @vsize: Height of input image.
 * @pixsize: Pixel size of the image in terms of bits.
 * @in_pitch: Line offset of input image.
 * @out_pitch: Line offset of output image.
 */
struct prev_size_params {
	unsigned int hstart;
	unsigned int vstart;
	unsigned int hsize;
	unsigned int vsize;
	unsigned char pixsize;
	unsigned short in_pitch;
	unsigned short out_pitch;
};

/**
 * struct prev_rgb2ycbcr_coeffs - Structure RGB2YCbCr parameters.
 * @coeff: Color conversion gains in 3x3 matrix.
 * @offset: Color conversion offsets.
 */
struct prev_rgb2ycbcr_coeffs {
	short coeff[RGB_MAX][RGB_MAX];
	short offset[RGB_MAX];
};

/**
 * struct prev_darkfrm_params - Structure for Dark frame suppression.
 * @addr: Memory start address.
 * @offset: Line offset.
 */
struct prev_darkfrm_params {
	u32 addr;
	u32 offset;
};

/**
 * struct prev_params - Structure for all configuration
 * @features: Set of features enabled.
 * @pix_fmt: Output pixel format.
 * @cfa: CFA coefficients.
 * @csup: Chroma suppression coefficients.
 * @ytable: Pointer to Luma enhancement coefficients.
 * @nf: Noise filter coefficients.
 * @dcor: Noise filter coefficients.
 * @gtable: Gamma coefficients.
 * @wbal: White Balance parameters.
 * @blk_adj: Black adjustment parameters.
 * @rgb2rgb: RGB blending parameters.
 * @rgb2ycbcr: RGB to ycbcr parameters.
 * @hmf_params: Horizontal median filter.
 * @size_params: Size parameters.
 * @drkf_params: Darkframe parameters.
 * @lens_shading_shift:
 * @average: Downsampling rate for averager.
 * @contrast: Contrast.
 * @brightness: Brightness.
 */
struct prev_params {
	u16 features;
	enum preview_ycpos_mode pix_fmt;
	struct ispprev_cfa cfa;
	struct ispprev_csup csup;
	u32 *ytable;
	struct ispprev_nf nf;
	struct ispprev_dcor dcor;
	struct ispprev_gtable gtable;
	struct ispprev_wbal wbal;
	struct ispprev_blkadj blk_adj;
	struct ispprev_rgbtorgb rgb2rgb;
	struct ispprev_csc rgb2ycbcr;
	struct ispprev_hmed hmf_params;
	struct prev_size_params size_params;
	struct prev_darkfrm_params drkf_params;
	u8 lens_shading_shift;
	u8 average;
	u8 contrast;
	u8 brightness;
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
 * @red_gamma: Pointer to red gamma correction table.
 * @green_gamma: Pointer to green gamma correction table.
 * @blue_gamma: Pointer to blue gamma correction table.
 */
struct isptables_update {
	u16 update;
	u16 flag;
	struct ispprev_nf *prev_nf;
	u32 *red_gamma;
	u32 *green_gamma;
	u32 *blue_gamma;
};

void isppreview_config_shadow_registers(void);

int isppreview_request(void);

int isppreview_free(void);

int isppreview_config_datapath(enum preview_input input,
					enum preview_output output);

void isppreview_config_ycpos(enum preview_ycpos_mode mode);

void isppreview_config_averager(u8 average);

void isppreview_enable_invalaw(u8 enable);

void isppreview_enable_drkframe(u8 enable);

void isppreview_enable_shadcomp(u8 enable);

void isppreview_config_drkf_shadcomp(u8 scomp_shtval);

void isppreview_enable_gammabypass(u8 enable);

void isppreview_enable_hmed(u8 enable);

void isppreview_config_hmed(struct ispprev_hmed);

void isppreview_enable_noisefilter(u8 enable);

void isppreview_config_noisefilter(struct ispprev_nf prev_nf);

void isppreview_enable_dcor(u8 enable);

void isppreview_config_dcor(struct ispprev_dcor prev_dcor);


void isppreview_config_cfa(struct ispprev_cfa);

void isppreview_config_gammacorrn(struct ispprev_gtable);

void isppreview_config_chroma_suppression(struct ispprev_csup csup);

void isppreview_enable_cfa(u8 enable);

void isppreview_config_luma_enhancement(u32 *ytable);

void isppreview_enable_luma_enhancement(u8 enable);

void isppreview_enable_chroma_suppression(u8 enable);

void isppreview_config_whitebalance(struct ispprev_wbal);

void isppreview_config_blkadj(struct ispprev_blkadj);

void isppreview_config_rgb_blending(struct ispprev_rgbtorgb);

void isppreview_config_rgb_to_ycbcr(struct ispprev_csc);

void isppreview_update_contrast(u8 *contrast);

void isppreview_query_contrast(u8 *contrast);

void isppreview_config_contrast(u8 contrast);

void isppreview_get_contrast_range(u8 *min_contrast, u8 *max_contrast);

void isppreview_update_brightness(u8 *brightness);

void isppreview_config_brightness(u8 brightness);

void isppreview_get_brightness_range(u8 *min_brightness, u8 *max_brightness);

void isppreview_set_color(u8 *mode);

void isppreview_get_color(u8 *mode);

void isppreview_query_brightness(u8 *brightness);

void isppreview_config_yc_range(struct ispprev_yclimit yclimit);

int isppreview_try_size(u32 input_w, u32 input_h, u32 *output_w,
				u32 *output_h);

int isppreview_config_size(u32 input_w, u32 input_h, u32 output_w,
			u32 output_h);

int isppreview_config_inlineoffset(u32 offset);

int isppreview_set_inaddr(u32 addr);

int isppreview_config_outlineoffset(u32 offset);

int isppreview_set_outaddr(u32 addr);

int isppreview_config_darklineoffset(u32 offset);

int isppreview_set_darkaddr(u32 addr);

void isppreview_enable(u8 enable);

int isppreview_busy(void);

struct prev_params *isppreview_get_config(void);

void isppreview_print_status(void);

#ifndef CONFIG_ARCH_OMAP3410
void isppreview_save_context(void);
#else
static inline void isppreview_save_context(void) {}
#endif

#ifndef CONFIG_ARCH_OMAP3410
void isppreview_restore_context(void);
#else
static inline void isppreview_restore_context(void) {}
#endif

int omap34xx_isp_preview_config(void *userspace_add);

int omap34xx_isp_tables_update(struct isptables_update *isptables_struct);

#endif/* OMAP_ISP_PREVIEW_H */
