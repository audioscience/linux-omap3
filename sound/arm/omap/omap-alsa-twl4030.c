/*
 * sound/arm/omap/omap-audio-twl4030.c
 *
 * Codec driver for TWL4030 for OMAP processors
 *
 * Copyright (C) 2004-2007 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contributors:
 *	Nishant Menon
 * 	Jian Zhang
 *	Hari Nagalla
 *	Misael Lopez Cruz
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/control.h>

#include <asm/arch/omap-alsa.h>
#include <linux/i2c/twl4030.h>
#if defined(CONFIG_ARCH_OMAP243X) || defined(CONFIG_ARCH_OMAP3430)
#include <asm/arch/mcbsp.h>
#include <asm/arch/clock.h>
#else
#error "Unsupported configuration"
#endif

#include "omap-alsa-twl4030.h"
#include "omap-alsa-dma.h"

/******************************** Debug Macros ********************************/
/* To generate a rather shrill tone -test the entire path */
#undef TONE_GEN
/* To dump the twl registers for debug */
#undef TWL_DUMP_REGISTERS
#undef TWL_DUMP_REGISTERS_MCBSP
#undef DEBUG

#ifdef TWL_DUMP_REGISTERS
static void twl4030_dump_registers(void);
#endif
#ifdef TONE_GEN
static void generate_tone(void);
#endif

static char twl4030_configured;		/* Configured count */
static int mixer_dev_id;

/*******************************************************************************
 *
 * Module data structures
 *
 ******************************************************************************/

static unsigned int twl4030_rates[] = {
	8000, 11025, 12000,
	16000, 22050, 24000,
	32000, 44100, 48000
};

static struct snd_pcm_hw_constraint_list twl4030_pcm_hw_constraint_list = {
	.count = ARRAY_SIZE(twl4030_rates),
	.list = twl4030_rates,
	.mask = 0,
};

static struct snd_pcm_hardware twl4030_pcm_hardware_playback = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
		  SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 16,
	.periods_max = 255,
	.fifo_size = 0,
};

static struct snd_pcm_hardware twl4030_pcm_hardware_capture = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
		  SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 16,
	.periods_max = 255,
	.fifo_size = 0,
};

/* Hint - optimization wise move the most used values up the list */
static const struct sample_rate_info_t valid_sample_rates[] = {
	{.rate = 8000, .apll = AUDIO_MODE_RATE_08_000},
	{.rate = 16000, .apll = AUDIO_MODE_RATE_16_000},
	{.rate = 44100, .apll = AUDIO_MODE_RATE_44_100},
	{.rate = 11025, .apll = AUDIO_MODE_RATE_11_025},
	{.rate = 12000, .apll = AUDIO_MODE_RATE_12_000},
	{.rate = 22050, .apll = AUDIO_MODE_RATE_22_050},
	{.rate = 24000, .apll = AUDIO_MODE_RATE_24_000},
	{.rate = 32000, .apll = AUDIO_MODE_RATE_32_000},
	{.rate = 48000, .apll = AUDIO_MODE_RATE_48_000},
	/* Dont support 96Khz -requires HSCLK >26Mhz
	   { .rate = 96000, .apll = AUDIO_MODE_RATE_96_000 }, */
};

struct codec_local_info twl4030_local = {
	.play_volume =  WRITE_LR_VOLUME(DEFAULT_OUTPUT_VOLUME),
	.rec_volume = WRITE_LR_VOLUME(DEFAULT_INPUT_VOLUME),
	.line = WRITE_LR_VOLUME(DEFAULT_INPUT_LINE_VOLUME),
	.mic = WRITE_LR_VOLUME(DEFAULT_INPUT_MIC_VOLUME),
	.aux = WRITE_LR_VOLUME(DEFAULT_INPUT_LINE_VOLUME),
	.hset = WRITE_LR_VOLUME(DEFAULT_OUTPUT_HSET_VOLUME),
	.classd = WRITE_LR_VOLUME(DEFAULT_OUTPUT_SPK_VOLUME),
	.ear = WRITE_LEFT_VOLUME(DEFAULT_OUTPUT_EAR_VOLUME),
	.sidetone = WRITE_LEFT_VOLUME(DEFAULT_SIDETONE_VOLUME),
	.carkit_out = WRITE_LR_VOLUME(DEFAULT_OUTPUT_CARKIT_VOLUME),
	.carkit_in = WRITE_LEFT_VOLUME(DEFAULT_INPUT_CARKIT_VOLUME),
	.handsfree_en = 1,
	.hsmic_en = 1,
	.main_mic_en = 1,
	.sub_mic_en = 1,
	.current_input = DEFAULT_INPUT_TWL_DEVICE,
	.current_output = DEFAULT_OUTPUT_TWL_DEVICE,
	.audio_samplerate = AUDIO_RATE_DEFAULT,
	.current_bitspersample = AUDIO_SAMPLE_DATA_WIDTH_16,
	.current_stereomode = STEREO_MODE,
	.recsrc = DEFAULT_INPUT_LNX_DEVICE,
	.outsrc = DEFAULT_OUTPUT_LNX_DEVICE,
	.mod_cnt = 0,
};

/*
 * The TWL4030 will always use stereo I2S protocol to communicate
 *
 * McBSP Configuration Required:
 * Stereo 16 bit:(default)
 * -------------
 * Single phase, FSYNC=Rising, words=1 DMA->Normal,32bit DXR
 *
 * Stereo 24 bit:
 * -------------
 * Single phase, FSYNC=Falling, words=2 DMA->Normal,32bit DXR
 *
 * Mono 16 bit:
 * ------------
 * Single phase, FSYNC=Rising, words=1 DMA->Normal,16 bit DXR+2
 * OR
 * Single phase, FSYNC=Falling, words=1 DMA->Normal,32bit DXR
 *
 * Mono 24 bit:
 * ------------
 * Single phase, FSYNC=Falling, words=2 DMA-> ei=1,fi=-1,32bit DXR
 *
 */

struct codec_mcbsp_settings twl4030_mcbsp_settings = {
	.audio_mcbsp_tx_transfer_params = {
		.skip_alt = OMAP_MCBSP_SKIP_NONE,
		.auto_reset = OMAP_MCBSP_AUTO_XRST,
		.callback = twl4030_mcbsp_dma_cb,
		.word_length1 = OMAP_MCBSP_WORD_32
	},
	.audio_mcbsp_rx_transfer_params = {
		.skip_alt = OMAP_MCBSP_SKIP_NONE,
		.auto_reset = OMAP_MCBSP_AUTO_RRST,
		.callback = twl4030_mcbsp_dma_cb,
		.word_length1 = OMAP_MCBSP_WORD_32
	},
	.audio_mcbsp_tx_cfg_param = {
#ifdef TWL_MASTER
		.fsync_src = OMAP_MCBSP_TXFSYNC_EXTERNAL,
		.clk_mode = OMAP_MCBSP_CLKTXSRC_EXTERNAL,
#else
		.fsync_src = OMAP_MCBSP_TXFSYNC_INTERNAL,
		.clk_mode = OMAP_MCBSP_CLKTXSRC_INTERNAL,
#endif
		.fs_polarity = OMAP_MCBSP_CLKX_POLARITY_RISING,
		.clk_polarity = OMAP_MCBSP_FS_ACTIVE_LOW,
		.frame_length1 = OMAP_MCBSP_FRAMELEN_N(1),
		.word_length1 = OMAP_MCBSP_WORD_32,
		.justification = OMAP_MCBSP_RJUST_ZEROMSB,
		.reverse_compand = OMAP_MCBSP_MSBFIRST,
		.phase = OMAP_MCBSP_FRAME_SINGLEPHASE,
		.data_delay = OMAP_MCBSP_DATADELAY1
	},
	.audio_mcbsp_rx_cfg_param = {
#ifdef TWL_MASTER
		.fsync_src = OMAP_MCBSP_RXFSYNC_EXTERNAL,
		.clk_mode = OMAP_MCBSP_CLKRXSRC_EXTERNAL,
#else
		.fsync_src = OMAP_MCBSP_RXFSYNC_INTERNAL,
		.clk_mode = OMAP_MCBSP_CLKRXSRC_INTERNAL,
#endif
		.fs_polarity = OMAP_MCBSP_CLKR_POLARITY_RISING,
		.clk_polarity = OMAP_MCBSP_FS_ACTIVE_LOW,
		.frame_length1 = OMAP_MCBSP_FRAMELEN_N(1),
		.word_length1 = OMAP_MCBSP_WORD_32,
		.justification = OMAP_MCBSP_RJUST_ZEROMSB,
		.reverse_compand = OMAP_MCBSP_MSBFIRST,
		.phase = OMAP_MCBSP_FRAME_SINGLEPHASE,
		.data_delay = OMAP_MCBSP_DATADELAY1
	},
	.audio_mcbsp_srg_fsg_cfg = {
		.period = 0,
		.pulse_width = 0,
		.fsgm = 0,
		.sample_rate = 0,
		.bits_per_sample = 0,
		.srg_src = OMAP_MCBSP_SRGCLKSRC_CLKX,
#ifdef TWL_MASTER
		.sync_mode = OMAP_MCBSP_SRG_FREERUNNING,
#else
		.sync_mode = OMAP_MCBSP_SRG_RUNNING,
#endif
		.polarity = OMAP_MCBSP_CLKX_POLARITY_FALLING,
		.dlb = 0
	}
};

/******************************************************************************
 *
 * Common APIs
 *
 *****************************************************************************/

inline int audio_twl4030_write(u8 address, u8 data)
{
	int ret = 0;

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE, data, address);
	if (ret >= 0) {
		ret = 0;
	} else {
		printk(KERN_ERR "TWL4030:Audio:Write[0x%x] Error %d\n",
			address, ret);
	}

	return ret;
}

inline int audio_twl4030_read(u8 address)
{
	u8 data;
	int ret = 0;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &data, address);
	if (ret >= 0) {
		ret = data;
	} else {
		printk(KERN_ERR "TWL4030:Audio:Read[0x%x] Error %d\n",
			address, ret);
	}
	return ret;
}

/*******************************************************************************
 *
 * Codec util APIs
 *
 ******************************************************************************/

/*
 * Configure GPIO for data out
 */
inline int twl4030_ext_mut_conf(void)
{
	int ret;
	u8 data;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &data, GPIO_DATA_DIR);
	if (ret)
		return ret;
	data |= 0x1 << T2_AUD_EXT_MUT_GPIO;
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, data, GPIO_DATA_DIR);

	return ret;
}

/*
 * Disable mute also handle time of wait
 */
inline int twl4030_ext_mut_off(void)
{
	int ret;
	u8 data;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &data, GPIO_CLR);
	if (ret)
		return ret;
	/* Wait for ramp duration, settling time for signal */
	udelay(1);
	data |= 0x1 << T2_AUD_EXT_MUT_GPIO;
	/* Clear mute */
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, data, GPIO_CLR);
	return ret;
}

/*
 * Enable mute
 */
inline int twl4030_ext_mut_on(void)
{
	int ret;
	u8 data;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &data, GPIO_SET);
	if (ret)
		return ret;
	data |= 0x1 << T2_AUD_EXT_MUT_GPIO;
	/* Set mute */
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, data, GPIO_SET);

	return ret;
}
/*
 * twl4030_codec_on
 */
static inline int twl4030_codec_on(void)
{
	int data = audio_twl4030_read(REG_CODEC_MODE);

	if (unlikely(data < 0)) {
		printk(KERN_ERR "Reg read failed\n");
		return data;
	}
	if (unlikely(data & BIT_CODEC_MODE_CODECPDZ_M))
		return 0;

	data |= BIT_CODEC_MODE_CODECPDZ_M;

	return audio_twl4030_write(REG_CODEC_MODE, (u8)data);
}

/*
 * Switch off the codec
 */
static inline int twl4030_codec_off(void)
{
	int data = audio_twl4030_read(REG_CODEC_MODE);

	if (unlikely(data < 0)) {
		printk(KERN_ERR "Reg read failed\n");
		return data;
	}
	/* Expected to be already off at bootup - but
	 * we do not know the status of the device
	 * hence would like to force a shut down
	 */
	if (unlikely(!(data & BIT_CODEC_MODE_CODECPDZ_M)))
		return 0;
	data &= ~BIT_CODEC_MODE_CODECPDZ_M;

	return audio_twl4030_write(REG_CODEC_MODE, (u8)data);
}

/*
 * Set the power to on after toggle to off and then on
 */
int twl4030_codec_tog_on(void)
{
	int ret = 0;
	int data = audio_twl4030_read(REG_CODEC_MODE);

	if (unlikely(data < 0)) {
		printk(KERN_ERR "Reg read failed\n");
		return data;
	}
	data &= ~BIT_CODEC_MODE_CODECPDZ_M;
	ret =  audio_twl4030_write(REG_CODEC_MODE, (u8)data);
	if (ret) {
		printk(KERN_ERR "Codec write failed ! %d\n", ret);
		return ret;
	}
	udelay(10); /* 10 ms delay for power settling */
	data |= BIT_CODEC_MODE_CODECPDZ_M;
	ret =  audio_twl4030_write(REG_CODEC_MODE, (u8) data);
	udelay(10); /* 10 ms delay for power settling */

	return ret;
}

/*
 * Enable the output path
 *  * NOTE * Codec power must be shut down during before this call
 *  * NOTE * This does not take care of gain settings for the specified output
 */
static int twl4030_enable_output(void)
{
	u8 ear_ctl = 0;
	u8 hs_ctl = 0;
	u8 hs_pop = 0;
	u8 hf_ctll = 0;
	u8 hf_ctlr = 0;
	u8 dac_ctl = 0;
	u8 ck_ctll = 0;
	u8 ck_ctlr = 0;
	u8 opt = 0;
	u32 line = 0;
	int ret = 0;

	opt = audio_twl4030_read(REG_OPTION) & ~(BIT_OPTION_ARXR2_EN_M |
					       BIT_OPTION_ARXL2_EN_M);

	/* AR2 and AL2 are active for I2S */
	if ((twl4030_local.current_output & OUTPUT_STEREO_HEADSET) ==
			OUTPUT_STEREO_HEADSET) {
		hs_ctl |= BIT_HS_SEL_HSOR_AR2_EN_M | BIT_HS_SEL_HSOL_AL2_EN_M;
		/* POP control - VMID? */
		hs_pop = BIT_HS_POPN_SET_VMID_EN_M;
		dac_ctl = BIT_AVDAC_CTL_ADACL2_EN_M | BIT_AVDAC_CTL_ADACR2_EN_M;
		opt |= BIT_OPTION_ARXR2_EN_M | BIT_OPTION_ARXL2_EN_M;
	}
	if ((twl4030_local.current_output & OUTPUT_HANDS_FREE_CLASSD) ==
			OUTPUT_HANDS_FREE_CLASSD) {
		if (twl4030_local.handsfree_en) {
			hf_ctll |=
			    HANDS_FREEL_AL2 << BIT_HFL_CTL_HFL_INPUT_SEL |
			    BIT_HFL_CTL_HFL_REF_EN_M;
			hf_ctlr |=
			    HANDS_FREER_AR2 << BIT_HFR_CTL_HFR_INPUT_SEL |
			    BIT_HFR_CTL_HFR_REF_EN_M;
		}
		dac_ctl = BIT_AVDAC_CTL_ADACL2_EN_M | BIT_AVDAC_CTL_ADACR2_EN_M;
		opt |= BIT_OPTION_ARXR2_EN_M | BIT_OPTION_ARXL2_EN_M;
	}
	if ((twl4030_local.current_output & OUTPUT_MONO_EARPIECE) ==
			OUTPUT_MONO_EARPIECE) {
		/* only AL2 comes in case of i2s */
		ear_ctl |= BIT_EAR_CTL_EAR_AL2_EN_M;
		dac_ctl = BIT_AVDAC_CTL_ADACL2_EN_M;
		opt |= BIT_OPTION_ARXL2_EN_M;
	}
	if ((twl4030_local.current_output & OUTPUT_CARKIT) ==
			OUTPUT_CARKIT) {
		ck_ctll |= BIT_PRECKL_CTL_PRECKL_AL2_EN_M
					| BIT_PRECKL_CTL_PRECKL_EN_M;
		ck_ctlr |= BIT_PRECKR_CTL_PRECKR_AR2_EN_M
					| BIT_PRECKR_CTL_PRECKR_EN_M;
		dac_ctl = BIT_AVDAC_CTL_ADACL2_EN_M
					| BIT_AVDAC_CTL_ADACR2_EN_M;
		opt |= BIT_OPTION_ARXL2_EN_M | BIT_OPTION_ARXR2_EN_M;
	}
	if (opt) {
		ret = audio_twl4030_write(REG_OPTION, opt);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	if (ear_ctl) {
		u8 temp;
		temp = audio_twl4030_read(REG_EAR_CTL);
		ear_ctl |= temp;
		ret = audio_twl4030_write(REG_EAR_CTL, ear_ctl);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	if (hs_ctl) {
		ret = audio_twl4030_write(REG_HS_SEL, hs_ctl);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	if (hs_pop) {
		/* IMPORTANT: The following sequence is *required*
		 * for starting the headset- esp for
		 * ensuring the existance of the negative phase of
		 * analog signal
		 */
		ret = audio_twl4030_write(REG_HS_POPN_SET, hs_pop);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hs_pop |= BIT_HS_POPN_SET_RAMP_EN_M;
		udelay(1); /* Require a short delay before enabling ramp */
		ret = audio_twl4030_write(REG_HS_POPN_SET, hs_pop);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	/* IMPORTANT: The following sequence is *required*
	 * for starting the speakers!
	 */
	if (hf_ctll) {
		ret = audio_twl4030_write(REG_HFL_CTL, hf_ctll);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hf_ctll |= BIT_HFL_CTL_HFL_RAMP_EN_M;
		ret = audio_twl4030_write(REG_HFL_CTL, hf_ctll);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hf_ctll |= BIT_HFL_CTL_HFL_LOOP_EN_M;
		ret = audio_twl4030_write(REG_HFL_CTL, hf_ctll);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hf_ctll |= BIT_HFL_CTL_HFL_HB_EN_M;
		ret = audio_twl4030_write(REG_HFL_CTL, hf_ctll);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	if (hf_ctlr) {
		ret = audio_twl4030_write(REG_HFR_CTL, hf_ctlr);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hf_ctlr |= BIT_HFR_CTL_HFR_RAMP_EN_M;
		ret = audio_twl4030_write(REG_HFR_CTL, hf_ctlr);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hf_ctlr |= BIT_HFR_CTL_HFR_LOOP_EN_M;
		ret = audio_twl4030_write(REG_HFR_CTL, hf_ctlr);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hf_ctlr |= BIT_HFR_CTL_HFR_HB_EN_M;
		ret = audio_twl4030_write(REG_HFR_CTL, hf_ctlr);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	if (dac_ctl) {
		/* I2S should go thru DACR2, DACL2- unless we are on mono */
		if (twl4030_local.current_stereomode == MONO_MODE)
			dac_ctl &= ~BIT_AVDAC_CTL_ADACR2_EN_M;
		ret = audio_twl4030_write(REG_AVDAC_CTL, dac_ctl);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	if (ck_ctll) {
		ret = audio_twl4030_write(REG_PRECKL_CTL, ck_ctll);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	if (ck_ctlr)
		ret = audio_twl4030_write(REG_PRECKR_CTL, ck_ctlr);
enable_op_exit:
	if (ret)
		printk(KERN_ERR "Error in Enable output[%d] in Line %d\n",
			ret, line);
	return ret;
}

/*
 * Remove the output path
 *  * NOTE * Shut down the codec before attempting this
 */
static int twl4030_disable_output(void)
{
	int ret = 0;
	int line = 0;
	u8 read_reg;

	read_reg = audio_twl4030_read(REG_PRECKR_CTL);
	/* To preserve gain settings */
	read_reg &= BIT_PRECKR_CTL_PRECKR_GAIN_M;
	ret = audio_twl4030_write(REG_PRECKR_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	read_reg = audio_twl4030_read(REG_PRECKL_CTL);
	/* To preserve gain settings */
	read_reg &= BIT_PRECKL_CTL_PRECKL_GAIN_M;
	ret = audio_twl4030_write(REG_PRECKL_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	ret = audio_twl4030_write(REG_PREDR_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	ret = audio_twl4030_write(REG_PREDL_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	read_reg = audio_twl4030_read(REG_EAR_CTL);
	/* To preserve gain settings */
	read_reg &= BIT_EAR_CTL_EAR_GAIN_M;
	ret = audio_twl4030_write(REG_EAR_CTL, read_reg);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	ret = audio_twl4030_write(REG_HS_SEL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	ret = audio_twl4030_write(REG_HFL_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	ret = audio_twl4030_write(REG_HFR_CTL, 0x0);
disable_op_exit:
	if (ret)
		printk(KERN_ERR "Disable Output Error [%d] in Line %d\n", ret,
		       line);
	return ret;
}

/*
 * Enable the input to the correct device.
 * Enable all the ADCs and path settings
 *  * NOTE * This will not set the gain
 * Reason being that the gain setting can be done with CODEC power down
 *  * NOTE * This should be called with codec power down
 */
static int twl4030_enable_input(void)
{
	u8 micbias_ctl = 0;
	u8 mic_en1 = 0;
	u8 mic_en2 = 0;
	u8 adc = 0;
	int ret = 0;
	u8 opt_ip = 0;
	int line = 0;

	ret = audio_twl4030_read(REG_OPTION);
	if (ret < 0)
		goto enable_ip_exit;

	/* Dont use path1/path2 left and right.. we will enable it later.. */
	opt_ip =
	    (ret &
	     ~(BIT_OPTION_ATXL1_EN_M | BIT_OPTION_ATXR1_EN_M |
	       BIT_OPTION_ATXL2_VTXL_EN_M | BIT_OPTION_ATXR2_VTXR_EN_M));

	/* HS MIC */
	if ((twl4030_local.current_input & INPUT_HEADSET_MIC) ==
			INPUT_HEADSET_MIC) {
		/* Mono Path */
		if (twl4030_local.hsmic_en) {
			micbias_ctl |= BIT_MICBIAS_CTL_HSMICBIAS_EN_M;
			mic_en1 |= BIT_ANAMICL_HSMIC_EN_M
					| BIT_ANAMICL_MICAMPL_EN_M;
			adc |= BIT_AVADC_CTL_ADCL_EN_M;
			opt_ip |= BIT_OPTION_ATXL1_EN_M;
		}
	}
	/* Main Mic */
	if ((twl4030_local.current_input & INPUT_MAIN_MIC) ==
			INPUT_MAIN_MIC) {
		if (twl4030_local.main_mic_en) {
			micbias_ctl |= BIT_MICBIAS_CTL_MICBIAS1_EN_M;
			mic_en1 |= BIT_ANAMICL_MAINMIC_EN_M
					 | BIT_ANAMICL_MICAMPL_EN_M;
			adc |= BIT_AVADC_CTL_ADCL_EN_M;
			opt_ip |= BIT_OPTION_ATXL1_EN_M;
		}
	}
	/* Sub Mic */
	if ((twl4030_local.current_input & INPUT_SUB_MIC) ==
			INPUT_SUB_MIC) {
		if (twl4030_local.sub_mic_en) {
			micbias_ctl |= BIT_MICBIAS_CTL_MICBIAS2_EN_M;
			mic_en2 |= BIT_ANAMICR_SUBMIC_EN_M
					 | BIT_ANAMICR_MICAMPR_EN_M;
			adc |= BIT_AVADC_CTL_ADCR_EN_M;
			opt_ip |= BIT_OPTION_ATXR1_EN_M;
		}
	}
	/* Aux */
	if ((twl4030_local.current_input & INPUT_AUX) ==
			INPUT_AUX) {
		mic_en1 |= BIT_ANAMICL_AUXL_EN_M | BIT_ANAMICL_MICAMPL_EN_M;
		mic_en2 |= BIT_ANAMICR_AUXR_EN_M | BIT_ANAMICR_MICAMPR_EN_M;
		adc |= BIT_AVADC_CTL_ADCL_EN_M | BIT_AVADC_CTL_ADCR_EN_M;
		opt_ip |= BIT_OPTION_ATXL1_EN_M | BIT_OPTION_ATXR1_EN_M;
	}
	/* Carkit */
	if ((twl4030_local.current_input & INPUT_CARKIT) ==
			INPUT_CARKIT) {
		mic_en1 |= BIT_ANAMICL_CKMIC_EN_M | BIT_ANAMICL_MICAMPL_EN_M;
		adc |= BIT_AVADC_CTL_ADCL_EN_M;
		opt_ip |= BIT_OPTION_ATXL1_EN_M;
	}
	ret = audio_twl4030_write(REG_OPTION, opt_ip);
	if (ret) {
		line = __LINE__;
		goto enable_ip_exit;
	}
	ret = audio_twl4030_write(REG_MICBIAS_CTL, micbias_ctl);
	if (ret) {
		line = __LINE__;
		goto enable_ip_exit;
	}
	ret = audio_twl4030_write(REG_ANAMICL, mic_en1);
	if (ret) {
		line = __LINE__;
		goto enable_ip_exit;
	}
	ret = audio_twl4030_write(REG_ANAMICR, mic_en2);
	if (ret) {
		line = __LINE__;
		goto enable_ip_exit;
	}
	ret = audio_twl4030_write(REG_AVADC_CTL, adc);
	if (ret) {
		line = __LINE__;
		goto enable_ip_exit;
	}
	/* Use ADC TX2 routed to digi mic - we dont use tx2 path
	 * - route it to digi mic1
	 */
	ret = audio_twl4030_write(REG_ADCMICSEL, 0x0);
	if (ret) {
		line = __LINE__;
		goto enable_ip_exit;
	}
	ret = audio_twl4030_write(REG_DIGMIXING, 0x0);	/* No Karaoke */
enable_ip_exit:
	if (ret)
		printk(KERN_ERR "Error In Enable input[%d] in Line %d\n",
			ret, line);
	return ret;
}

/*
 * Reset all the inputs
 *  * NOTE * This should be called with codec power down
 */
static int twl4030_disable_input(void)
{
	int ret = 0;
	int line = 0;

	/* Disable all devices */
	ret = audio_twl4030_write(REG_MICBIAS_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_ip_exit;
	}
	ret = audio_twl4030_write(REG_ANAMICL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_ip_exit;
	}
	ret = audio_twl4030_write(REG_ANAMICR, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_ip_exit;
	}
	ret = audio_twl4030_write(REG_AVADC_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_ip_exit;
	}
	ret = audio_twl4030_write(REG_ADCMICSEL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_ip_exit;
	}
	ret = audio_twl4030_write(REG_DIGMIXING, 0x0);
disable_ip_exit:
	if (ret)
		printk(KERN_ERR "Error in disable of input [%d] in Line %d\n",
		       ret, line);
	return ret;
}

/*
 * Set up proper source. Call me with the codec powered down
 */
int twl4030_select_source(int flag, int val)
{
	int ret = 0;
	int temp = 0;

	switch (flag) {
	case DIR_OUT:
		/*
		 * If more than one play device selected,
		 * disable the device that is currently in use.
		 */
		if (hweight32(val) > 1)
			val &= ~twl4030_local.outsrc;
		/* Can select multiple output */
		if ((val & SOUND_MASK_LINE1) == SOUND_MASK_LINE1)
			temp |= OUTPUT_STEREO_HEADSET;
		if ((val & SOUND_MASK_SPEAKER) == SOUND_MASK_SPEAKER)
			temp |= OUTPUT_HANDS_FREE_CLASSD;
		if ((val & SOUND_MASK_PHONEOUT) == SOUND_MASK_PHONEOUT)
			temp |= OUTPUT_MONO_EARPIECE;
		if ((val & SOUND_MASK_CD) == SOUND_MASK_CD)
			temp |= OUTPUT_CARKIT;
		twl4030_local.current_output = temp;
		/* Toggle the source */
		ret = twl4030_disable_output();
		if (!ret)
			ret = twl4030_enable_output();
		if (!ret)
			twl4030_local.outsrc = val;
		break;
	case DIR_IN:
		/* If more than one device requested, reject the request */
		if (hweight32(val) > 1)
			return -EINVAL;
		/* Select multiple inputs */
		if ((val & SOUND_MASK_LINE) == SOUND_MASK_LINE)
			temp |= INPUT_HEADSET_MIC;
		if ((val & SOUND_MASK_MIC) == SOUND_MASK_MIC)
			temp |= INPUT_MAIN_MIC | INPUT_SUB_MIC;
		if ((val & SOUND_MASK_RADIO) == SOUND_MASK_RADIO)
			temp |= INPUT_AUX;
		if ((val & SOUND_MASK_CD) == SOUND_MASK_CD)
			temp |= INPUT_CARKIT;
		twl4030_local.current_input = temp;
		/* Toggle the source */
		ret = twl4030_disable_input();
		if (!ret)
			ret = twl4030_enable_input();
		if (!ret)
			twl4030_local.recsrc = val;
		break;
	default:
		printk(KERN_WARNING PLATFORM_NAME "-" CODEC_NAME
		       ": Wrong twl4030_selectsource flag specified\n");
		ret = -EPERM;
		break;

	}
	if (!ret)
		twl4030_local.mod_cnt++;
	else
		printk(KERN_ERR "Error selsrc Flag=%d,err=%d\n", flag, ret);

	return ret;
}

/*
 * Set the gain of the requested device
 */
int twl4030_setvolume(int flag, u8 gain_l, u8 gain_r)
{
	int ret = 0;

	if ((gain_l > AUDIO_MAX_OUTPUT_VOLUME)
	    || (gain_r > AUDIO_MAX_OUTPUT_VOLUME)) {
		printk(KERN_ERR "Invalid gain value %d %d\n", gain_l, gain_r);
		return -EPERM;
	}
	switch (flag) {
	case OUTPUT_VOLUME:
		{
			/* Normal volume control */
			u8 fine_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    AUDIO_OUTPUT_INCREMENT);
			u8 fine_val_r =
			    (unsigned char)((gain_r * COMPUTE_PRECISION) /
					    AUDIO_OUTPUT_INCREMENT);
			/* Inverted power control big value is small volume */
			u8 ana_val_r =
			    (unsigned char)(((AUDIO_MAX_OUTPUT_VOLUME -
			     gain_r) * COMPUTE_PRECISION) / ARX_APGA_INCR);
			u8 ana_val_l =
			    (unsigned char)(((AUDIO_MAX_OUTPUT_VOLUME -
			     gain_l) * COMPUTE_PRECISION) / ARX_APGA_INCR);
			/* Default value at this time... make it ioctl ?? */
			u8 coarse_val = AUDIO_DEF_COARSE_VOLUME_LEVEL;

			/* I2S - SDRL2 and SDRR2 */
			/* Digital boost */
			ret = audio_twl4030_write(REG_ARXL2PGA,
					coarse_val <<
					BIT_ARXL2PGA_ARXL2PGA_CGAIN |
					fine_val_l <<
					BIT_ARXL2PGA_ARXL2PGA_FGAIN);
			if (!ret)
				ret = audio_twl4030_write(REG_ARXR2PGA,
					coarse_val <<
					BIT_ARXL2PGA_ARXL2PGA_CGAIN
					| fine_val_r <<
					BIT_ARXR2PGA_ARXR2PGA_FGAIN);
			/* Analog boost */
			if (!ret)
				ret = audio_twl4030_write(REG_ARXL2_APGA_CTL,
					BIT_ARXL2_APGA_CTL_ARXL2_PDZ_M
					| BIT_ARXL2_APGA_CTL_ARXL2_DA_EN_M
					| ana_val_l <<
					BIT_ARXL2_APGA_CTL_ARXL2_GAIN_SET);
			if (!ret)
				ret = audio_twl4030_write(REG_ARXR2_APGA_CTL,
					BIT_ARXR2_APGA_CTL_ARXR2_PDZ_M
					| BIT_ARXR2_APGA_CTL_ARXR2_DA_EN_M
					| ana_val_r <<
					BIT_ARXR2_APGA_CTL_ARXR2_GAIN_SET);
			if (!ret)
				twl4030_local.play_volume =
					    WRITE_LEFT_VOLUME(gain_l) |
					    WRITE_RIGHT_VOLUME(gain_r);
		}
		break;
	case OUTPUT_STEREO_HEADSET:
		/* Only if current output device is stereo headset */
		if ((twl4030_local.current_output & OUTPUT_STEREO_HEADSET) ==
				OUTPUT_STEREO_HEADSET) {
			/* Normal volume control */
			u8 fine_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    NON_LIN_INCREMENT);
			u8 fine_val_r =
			    (unsigned char)((gain_r * COMPUTE_PRECISION) /
					    NON_LIN_INCREMENT);
			u8 value_set[NON_LIN_GAIN_MAX] = NON_LIN_VALS;
			/* Handle Mute request */
			fine_val_l = (gain_l == 0) ? 0 : value_set[fine_val_l];
			fine_val_r = (gain_r == 0) ? 0 : value_set[fine_val_r];
			ret = audio_twl4030_write(REG_HS_GAIN_SET,
					  (fine_val_l <<
					   BIT_HS_GAIN_SET_HSL_GAIN) |
					  (fine_val_r <<
					   BIT_HS_GAIN_SET_HSR_GAIN));
			if (!ret)
				twl4030_local.hset =
				    WRITE_LEFT_VOLUME(gain_l) |
				    WRITE_RIGHT_VOLUME(gain_r);
		}
		break;
	case OUTPUT_HANDS_FREE_CLASSD:
		if ((twl4030_local.current_output & OUTPUT_HANDS_FREE_CLASSD) ==
				OUTPUT_HANDS_FREE_CLASSD) {
			/* NOTE: CLASSD no special gain */
			twl4030_local.classd =
			    WRITE_LEFT_VOLUME(gain_l) |
			    WRITE_RIGHT_VOLUME(gain_r);
		}
		break;
	case OUTPUT_MONO_EARPIECE:
		if ((twl4030_local.current_output & OUTPUT_MONO_EARPIECE) ==
				OUTPUT_MONO_EARPIECE) {
			/* Normal volume control */
			u8 curr_val;
			u8 fine_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    NON_LIN_INCREMENT);
			u8 value_set[NON_LIN_GAIN_MAX] = NON_LIN_VALS;
			fine_val_l = (gain_l == 0) ? 0 : value_set[fine_val_l];
			/*EAR_CTL */
			curr_val = audio_twl4030_read(REG_EAR_CTL);
			curr_val &= ~BIT_EAR_CTL_EAR_GAIN_M;
			ret = audio_twl4030_write(REG_EAR_CTL, curr_val |
					  (fine_val_l <<
					  BIT_EAR_CTL_EAR_GAIN));
			if (!ret)
				twl4030_local.ear = WRITE_LEFT_VOLUME(gain_l);
		}
		break;
	case OUTPUT_SIDETONE:
		/* Sidetone Gain Control */
		if (twl4030_local.current_output) {
			ret = audio_twl4030_write(REG_VSTPGA, gain_l);
			if (!ret)
				twl4030_local.sidetone =
						WRITE_LEFT_VOLUME(gain_l);
		}
		break;
	case OUTPUT_CARKIT:
		if ((twl4030_local.current_output & OUTPUT_CARKIT) ==
				OUTPUT_CARKIT) {
			u8 curr_val;
			u8 fine_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    NON_LIN_INCREMENT);
			u8 fine_val_r =
			    (unsigned char)((gain_r * COMPUTE_PRECISION) /
					    NON_LIN_INCREMENT);
			u8 value_set[NON_LIN_GAIN_MAX] = NON_LIN_VALS;
			fine_val_l = (gain_l == 0) ? 0 : value_set[fine_val_l];
			fine_val_r = (gain_r == 0) ? 0 : value_set[fine_val_r];
			/* Left gain */
			curr_val = audio_twl4030_read(REG_PRECKL_CTL);
			curr_val &= ~BIT_PRECKL_CTL_PRECKL_GAIN_M;
			ret = audio_twl4030_write(REG_PRECKL_CTL, curr_val |
					  (fine_val_l <<
					  BIT_PRECKL_CTL_PRECKL_GAIN));
			if (!ret)
				twl4030_local.carkit_out =
						WRITE_LEFT_VOLUME(gain_l);
			/* Right gain */
			curr_val = audio_twl4030_read(REG_PRECKR_CTL);
			curr_val &= ~BIT_PRECKR_CTL_PRECKR_GAIN_M;
			ret = audio_twl4030_write(REG_PRECKR_CTL, curr_val |
					  (fine_val_r <<
					  BIT_PRECKR_CTL_PRECKR_GAIN));
			if (!ret)
				twl4030_local.carkit_out =
						WRITE_RIGHT_VOLUME(gain_r);
		}
		break;
	case INPUT_VOLUME:
		/* Set input volume */
		{
			u8 set_val_l = (unsigned char)
					((gain_l * COMPUTE_PRECISION) /
					AUDIO_INPUT_INCREMENT);
			u8 set_val_r = (unsigned char)
					((gain_r * COMPUTE_PRECISION) /
					AUDIO_INPUT_INCREMENT);
			/* NOTE: ANAMIC gain settings is handled by a
			* default value
			*/
			/* I2S - TXL1 and TXR1 only */
			ret = audio_twl4030_write(REG_ATXL1PGA,
					set_val_l <<
					BIT_ATXL1PGA_ATXL1PGA_GAIN);
			if (!ret)
				ret = audio_twl4030_write(REG_ATXR1PGA,
					set_val_r <<
					BIT_ATXR1PGA_ATXR1PGA_GAIN);
			if (!ret)
				twl4030_local.rec_volume =
					    WRITE_LEFT_VOLUME(gain_l) |
					    WRITE_RIGHT_VOLUME(gain_r);
		}
		break;
	case INPUT_HEADSET_MIC:
		if ((twl4030_local.current_input & INPUT_HEADSET_MIC) ==
				INPUT_HEADSET_MIC) {
			u8 set_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    MIC_AMP_INCR);
			/* ANAMIC_GAIN */
			ret =
			    audio_twl4030_write(REG_ANAMIC_GAIN,
					set_val_l <<
					BIT_ANAMIC_GAIN_MICAMPL_GAIN);
			if (!ret)
				twl4030_local.line =
				    WRITE_LEFT_VOLUME(gain_l) |
				    WRITE_RIGHT_VOLUME(gain_l);
		}
		break;
	case INPUT_MAIN_MIC:
		/* We do not use ALC Use ANAMIC_GAIN */
		/* left volume for main mic */
		if ((twl4030_local.current_input & INPUT_MAIN_MIC) ==
				INPUT_MAIN_MIC) {
			u8 set_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    MIC_AMP_INCR);
			int read_val = audio_twl4030_read(REG_ANAMIC_GAIN);
			if (read_val >= 0) {
				/* Clear the left vol entry */
				read_val &= ~(BIT_ANAMIC_GAIN_MICAMPL_GAIN_M);
				read_val |=
				    set_val_l << BIT_ANAMIC_GAIN_MICAMPL_GAIN;
				ret =
				    audio_twl4030_write(REG_ANAMIC_GAIN,
							(u8) read_val);
				if (!ret)
					twl4030_local.mic =
					    WRITE_LEFT_VOLUME(gain_l) |
					    WRITE_RIGHT_VOLUME(twl4030_local.
								mic);
			}
		}
		break;
	case INPUT_SUB_MIC:
		/* We do not use ALC */
		/* right volume for submic */
		if ((twl4030_local.current_input & INPUT_SUB_MIC) ==
				INPUT_SUB_MIC) {
			u8 set_val_r =
			    (unsigned char)((gain_r * COMPUTE_PRECISION) /
					    MIC_AMP_INCR);
			int read_val = audio_twl4030_read(REG_ANAMIC_GAIN);
			if (read_val >= 0) {
				/* Clear the right vol entry */
				read_val &= ~(BIT_ANAMIC_GAIN_MICAMPR_GAIN_M);
				read_val |=
				    set_val_r << BIT_ANAMIC_GAIN_MICAMPR_GAIN;
				ret =
				    audio_twl4030_write(REG_ANAMIC_GAIN,
							(u8) read_val);
				if (!ret)
					twl4030_local.mic =
					    WRITE_LEFT_VOLUME(twl4030_local.mic)
					    | WRITE_RIGHT_VOLUME(gain_r);
			}
		}
		break;
	case INPUT_AUX:
		if ((twl4030_local.current_input & INPUT_AUX) ==
				INPUT_AUX) {
			u8 set_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    MIC_AMP_INCR);
			u8 set_val_r =
			    (unsigned char)((gain_r * COMPUTE_PRECISION) /
					    MIC_AMP_INCR);
			/* ANAMIC_GAIN */
			ret =
			    audio_twl4030_write(REG_ANAMIC_GAIN,
					(set_val_l <<
					BIT_ANAMIC_GAIN_MICAMPL_GAIN)
					| (set_val_r <<
					BIT_ANAMIC_GAIN_MICAMPR_GAIN));
			if (!ret)
				twl4030_local.aux =
				    WRITE_LEFT_VOLUME(gain_l) |
				    WRITE_RIGHT_VOLUME(gain_r);
		}
		break;
	case INPUT_CARKIT:
		if ((twl4030_local.current_input & INPUT_CARKIT) ==
				INPUT_CARKIT) {
			u8 set_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    MIC_AMP_INCR);
			/* ANAMIC_GAIN */
			ret =
			    audio_twl4030_write(REG_ANAMIC_GAIN,
					set_val_l <<
					BIT_ANAMIC_GAIN_MICAMPL_GAIN);
			if (!ret)
				twl4030_local.carkit_in =
				    WRITE_LEFT_VOLUME(gain_l) |
				    WRITE_RIGHT_VOLUME(gain_l);
		}
		break;
	default:
		printk(KERN_WARNING PLATFORM_NAME "-" CODEC_NAME
		       ": Wrong twl4030_setvolume flag specified\n");
		ret = -EPERM;
		break;
	}
	if (!ret)
		twl4030_local.mod_cnt++;

	return ret;
}

/*
 * Configure the codec's data path
 */
static int twl4030_codec_conf_data_path(void)
{
	u8 codec_data_width = 0;
	u8 codec_mode = 0;

	/* Check sample width */
	if (twl4030_local.current_bitspersample ==
				AUDIO_SAMPLE_DATA_WIDTH_16) {
		/* Data width 16-bits */
		codec_data_width = AUDIO_DATA_WIDTH_16SAMPLE_16DATA;
	} else if (twl4030_local.current_bitspersample ==
				AUDIO_SAMPLE_DATA_WIDTH_24) {
		/* Data width 24-bits */
		codec_data_width = AUDIO_DATA_WIDTH_32SAMPLE_24DATA;
	} else {
		printk(KERN_ERR "Unknown sample width %d\n",
			twl4030_local.current_bitspersample);
		return -EPERM;
	}

	/* No need to set BIT_AUDIO_IF_CLK256FS_EN_M -not using it as CLKS!! */
	/* configure the audio IF of codec- Application Mode */
	codec_mode =
#ifndef TWL_MASTER
	    BIT_AUDIO_IF_AIF_SLAVE_EN_M |
#endif
	    (codec_data_width << BIT_AUDIO_IF_DATA_WIDTH) |
	    (AUDIO_DATA_FORMAT_I2S << BIT_AUDIO_IF_AIF_FORMAT) |
	    BIT_AUDIO_IF_AIF_EN_M;

	return audio_twl4030_write(REG_AUDIO_IF, codec_mode);
}

/*
 * Set the sample rate of the codec and communication media (mcbsp)
 *  * NOTE * Shut down the codec to change sample rate
 *           Cannot reprogram the Codec APLL while codec is powered
 */
int twl4030_set_samplerate(long sample_rate)
{
	int ret = 0;
	int count = 0;
	u8 codec_mode = 0;

	/* Validate if rate is proper */
	for (; count < NUMBER_OF_RATES_SUPPORTED; count++)
		if (valid_sample_rates[count].rate == sample_rate)
			break;

	if (count >= NUMBER_OF_RATES_SUPPORTED) {
		printk(KERN_ERR "[%d] Unsupported sample rate!!\n",
		       (u32) sample_rate);
		return -EPERM;
	}

	ret =
	    audio_twl4030_write(REG_APLL_CTL,
				AUDIO_APLL_DEFAULT << BIT_APLL_CTL_APLL_INFREQ);
	if (ret < 0) {
		printk(KERN_ERR "unable to set the INFREQ %d\n", ret);
		return ret;
	}
	/* Configure the codec -rate */
	ret = audio_twl4030_read(REG_CODEC_MODE);
	if (ret < 0) {
		printk(KERN_ERR "unable to read codec_mode %d\n", ret);
		return ret;
	}
	codec_mode = (u8) ret;

	/* Clear unnecessary bits */
	codec_mode &= ~(BIT_CODEC_MODE_APLL_RATE_M);

	codec_mode |=
	    (valid_sample_rates[count].apll << BIT_CODEC_MODE_APLL_RATE);
	ret = audio_twl4030_write(REG_CODEC_MODE, codec_mode);

	/* Program the apll */
	if (!ret)
		ret = audio_twl4030_write(REG_APLL_CTL,
					AUDIO_APLL_DEFAULT <<
					BIT_APLL_CTL_APLL_INFREQ |
					BIT_APLL_CTL_APLL_EN_M);

	/* Change the sample rate if we are successful */
	if (!ret)
		twl4030_local.audio_samplerate = sample_rate;

	return ret;
}

/*
 * twl4030_unconfigure
 */
void twl4030_unconfigure(void)
{
	if (twl4030_configured == 1) {
		twl4030_codec_off();
		twl4030_disable_output();
		twl4030_disable_input();
	}
	if (twl4030_configured > 0)
		twl4030_configured--;
}

/*
 * Clean up the register settings
 */
static int twl4030_cleanup(void)
{
	int ret = 0;
	int line = 0;
	ret = audio_twl4030_write(REG_VRXPGA, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_VDL_APGA_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_BTPGA, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_DTMF_FREQSEL, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_DTMF_TONOFF, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_DTMF_PGA_CTL2, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_DTMF_PGA_CTL1, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_DTMF_WANONOFF, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_HS_POPN_SET, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	/* Shut down the voice and other paths completely */
	ret = audio_twl4030_write(REG_ARXR1PGA, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXL1PGA, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXL1_APGA_CTL, 0x00);	/* Path1 */
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXR1_APGA_CTL, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXL2_APGA_CTL, 0x00);	/* Path1 */
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXR2_APGA_CTL, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ALC_CTL, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXL1PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXR1PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXL2PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXR2PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ATXL1PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ATXR1PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_AVTXL2PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_AVTXR2PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_AVDAC_CTL, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_OPTION, 0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
cleanup_exit:
	if (ret)
		printk(KERN_ERR "Cleanup Error[%d] Error in line %d\n",
		       ret, line);
	return (ret);
}

/*
 * twl4030_configure
 *  * NOTE * Should be called with codec off
 */
int twl4030_configure(void)
{
	int ret = 0;
	int line = 0;

	if (twl4030_configured == 0) {
		int data = 0;
		ret = twl4030_ext_mut_conf();
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_ext_mut_on();
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}

		ret = twl4030_cleanup();
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}

		/* Set up the codec_mode - use option 1
		 * - assume no voice path
		 */
		data = audio_twl4030_read(REG_CODEC_MODE);
		if (data < 0) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = audio_twl4030_write(REG_CODEC_MODE,
					  ((u8)data) | CODEC_OPTION_1 <<
					  BIT_CODEC_MODE_OPT_MODE);
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_codec_conf_data_path();
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		/* Select Rx path
		 * SDRL1->RxL1
		 * SDRR1->RxR1
		 * SDRL2->RxL2 (mono SDRM2)
		 * SDRL2->RxL2 (mono SDRM2)
		 */
		ret =
		    audio_twl4030_write(REG_RX_PATH_SEL,
					((twl4030_local.current_stereomode ==
					  STEREO_MODE) ? 0x00 :
					 BIT_RX_PATH_SEL_RXL2_SEL_M |
					 BIT_RX_PATH_SEL_RXR2_SEL_M));
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		/* Set the gains - we do not know the defaults
		 * attempt to set the volume of all the devices
		 * only those enabled get set.
		 */
		ret = twl4030_setvolume(OUTPUT_VOLUME,
					READ_LEFT_VOLUME
					(twl4030_local.play_volume),
					READ_RIGHT_VOLUME
					(twl4030_local.play_volume));
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_setvolume(OUTPUT_STEREO_HEADSET,
					READ_LEFT_VOLUME(twl4030_local.hset),
					READ_RIGHT_VOLUME(twl4030_local.hset));
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_setvolume(OUTPUT_HANDS_FREE_CLASSD,
				      READ_LEFT_VOLUME(twl4030_local.classd),
				      READ_RIGHT_VOLUME(twl4030_local.classd));
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_setvolume(OUTPUT_MONO_EARPIECE,
					READ_LEFT_VOLUME(twl4030_local.ear), 0);
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_setvolume(OUTPUT_SIDETONE,
					READ_LEFT_VOLUME
					(twl4030_local.sidetone), 0);
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_setvolume(OUTPUT_CARKIT,
					READ_LEFT_VOLUME
					(twl4030_local.carkit_out),
					READ_RIGHT_VOLUME
				      (twl4030_local.carkit_out));
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_setvolume(INPUT_VOLUME,
					READ_LEFT_VOLUME
					(twl4030_local.rec_volume),
					READ_RIGHT_VOLUME
					(twl4030_local.rec_volume));
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_setvolume(INPUT_HEADSET_MIC,
					READ_LEFT_VOLUME(twl4030_local.line),
					READ_RIGHT_VOLUME(twl4030_local.line));
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_setvolume(INPUT_MAIN_MIC,
					READ_LEFT_VOLUME(twl4030_local.mic), 0);
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_setvolume(INPUT_SUB_MIC, 0,
					READ_RIGHT_VOLUME(twl4030_local.mic));
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_setvolume(INPUT_AUX,
					READ_LEFT_VOLUME(twl4030_local.aux),
					READ_RIGHT_VOLUME(twl4030_local.aux));
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_setvolume(INPUT_CARKIT,
					READ_LEFT_VOLUME
					(twl4030_local.carkit_in),
					0);
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		/* Do not bypass the high pass filters */
		ret = audio_twl4030_write(REG_MISC_SET_2, 0x0);
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		/* Switch on the input required */
		if (!(twl4030_disable_output())) {
			ret = twl4030_enable_output();
			if (ret) {
				line = __LINE__;
				goto configure_exit;
			}
		}
		if (!(twl4030_disable_input())) {
			ret = twl4030_enable_input();
			if (ret) {
				line = __LINE__;
				goto configure_exit;
			}
		}
	}
	twl4030_configured++;
configure_exit:
	if (ret)
		printk(KERN_ERR "Configuration Error[%d] Error in line %d\n",
				ret, line);
	return ret;
}

/*
 * McBSP callback
 */
void twl4030_mcbsp_dma_cb(u32 ch_status, void *arg)
{
	if (ch_status) {
		printk(KERN_ERR "Error happend[%d 0x%x]!!\n", ch_status,
		       ch_status);
		return;

	}
	callback_omap_alsa_sound_dma(arg);
}

/*******************************************************************************
 *
 * Codec APIs
 *
 ******************************************************************************/

static int twl4030_default_samplerate(void)
{
	return AUDIO_RATE_DEFAULT;
}

/*
 * omap_twl4030_initialize
 */
static int omap_twl4030_initialize(void)
{
	int ret = 0;

	omap_mcbsp_set_io_type(AUDIO_MCBSP, 0);
	ret = omap_mcbsp_request(AUDIO_MCBSP);
	if (unlikely(ret)) {
		printk(KERN_ERR " Request for MCBSP Failed[%d]\n", ret);
		goto initialize_exit_path1;
	}

#ifdef TWL_DUMP_REGISTERS
	printk(KERN_INFO "pre\n");
	twl4030_dump_registers();
#endif
	ret = twl4030_ext_mut_conf();
	if (ret) {
		printk(KERN_ERR "a twl4030_ext_mut_conf failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	ret = twl4030_ext_mut_on();
	if (ret) {
		printk(KERN_ERR "a twl4030_ext_mut_on failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	/* Toggle the codec power mode */
	ret = twl4030_codec_tog_on();
	if (unlikely(ret)) {
		printk(KERN_ERR "a1 twl4030_codec_tog failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	/* Sample rate configuration - set APLLs to setup regs */
	ret = twl4030_set_samplerate(twl4030_local.audio_samplerate);
	if (ret) {
		printk(KERN_ERR "Sample rate setting failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	ret = twl4030_configure();
	if (unlikely(ret)) {
		printk(KERN_ERR " twl4030_configure_device failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	/* Switch off the codec so that when mcbsp starts.. we are waiting */
	if (unlikely(twl4030_codec_off())) {
		printk(KERN_ERR "a2 twl4030_codec_off failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	ret = twl4030_conf_data_interface();
	if (ret) {
		printk(KERN_ERR "Codec Data init failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	if (ret) {
		printk(KERN_ERR "register of ISR failed [%d]\n", ret);
		goto initialize_exit_path3;
	}
	ret = twl4030_codec_on();
	if (unlikely(ret)) {
		printk(KERN_ERR "a2 twl4030_codec_on failed [%d]\n", ret);
		goto initialize_exit_path3;
	}
	ret = twl4030_ext_mut_off();
	if (unlikely(ret)) {
		printk(KERN_ERR "twl4030_ext_mut_off failed [%d]\n", ret);
		goto initialize_exit_path3;
	}
	/* Codec is operational */
#ifdef TWL_DUMP_REGISTERS
	printk(KERN_INFO "post\n");
	twl4030_dump_registers();
#endif
#ifdef TWL_DUMP_REGISTERS_MCBSP
	printk(KERN_INFO "CONFIG");
	omap_mcbsp_dump_reg(AUDIO_MCBSP);
#endif

#ifdef DEBUG
#ifdef TONE_GEN
	generate_tone();
#endif
#endif				/* DEBUG */
	return 0;
initialize_exit_path3:
	twl4030_unconfigure();
	(void)omap2_mcbsp_reset(AUDIO_MCBSP);
initialize_exit_path2:
	/* Don't care about result */
	(void)omap_mcbsp_free(AUDIO_MCBSP);
initialize_exit_path1:
	return ret;
}

/*
 * omap_twl4030_shutdown
 */
static int omap_twl4030_shutdown(void)
{
	(void)omap2_mcbsp_reset(AUDIO_MCBSP);
	omap_mcbsp_free(AUDIO_MCBSP);
	twl4030_unconfigure();
	return 0;
}

int twl4030_stereomode_set(int mode, int dsp)
{
	int ret = 0;
	u8 dac_ctl = 0;

	if (twl4030_local.current_stereomode == mode) {
		/* Nothing to do at all */
		return 0;
	}
	/* If the data streams are active. It is a very very bad idea to change
	 * data transfer modes
	 */

	ret = twl4030_ext_mut_on();
	if (unlikely(ret)) {
		printk(KERN_ERR "twl4030_ext_mut_on failed [%d]\n", ret);
		return ret;
	}
	/* Toggle power of codec */
	ret = twl4030_codec_tog_on();
	if (ret < 0) {
		printk(KERN_ERR "MONO/STEREO Codec set failed\n");
		goto set_stereo_mode_exit;
	}
	ret = audio_twl4030_read(REG_AVDAC_CTL);
	if (ret < 0) {
		printk(KERN_ERR "did not get dac ctrl reg\n");
		goto set_stereo_mode_exit;
	}
	dac_ctl = ret;
	twl4030_local.current_stereomode = mode;
	if (twl4030_local.current_stereomode == MONO_MODE)
		dac_ctl &= ~BIT_AVDAC_CTL_ADACR2_EN_M;
	else
		dac_ctl |= BIT_AVDAC_CTL_ADACR2_EN_M;
	ret = audio_twl4030_write(REG_AVDAC_CTL, dac_ctl);
	if (ret < 0) {
		printk(KERN_ERR "did not set dac ctrl reg\n");
		goto set_stereo_mode_exit;
	}

	/* Power off codec */
	ret = twl4030_codec_off();
	if (ret) {
		printk(KERN_ERR "Unable to switch off the codec \n");
		goto set_stereo_mode_exit;
	}
	if (dsp) {
		ret = twl4030_conf_data_interface();
		if (ret) {
			printk(KERN_ERR "Configure data interface failed\n");
			goto set_stereo_mode_exit;
		}
		ret = omap2_mcbsp_dma_recv_params(AUDIO_MCBSP,
				&(twl4030_mcbsp_settings.
				audio_mcbsp_rx_transfer_params));
		if (ret < 0) {
			printk(KERN_ERR "MONO/STEREO RX params failed");
			goto set_stereo_mode_exit;
		}
		ret = omap2_mcbsp_dma_trans_params(AUDIO_MCBSP,
				&(twl4030_mcbsp_settings.
				audio_mcbsp_tx_transfer_params));
		if (ret < 0) {
			printk(KERN_ERR "MONO/STEREO TX params failed");
			goto set_stereo_mode_exit;
		}
	}
	/* Set the Mixing bit off if stereo, else set it to on */
#ifdef MONO_MODE_SOUNDS_STEREO
	ret =
	    audio_twl4030_write(REG_RX_PATH_SEL,
				((mode ==
				  STEREO_MODE) ? 0x00 :
				 BIT_RX_PATH_SEL_RXL2_SEL_M |
				 BIT_RX_PATH_SEL_RXR2_SEL_M));
#else
	ret = audio_twl4030_write(REG_RX_PATH_SEL, 0x00);
#endif
	ret = (!twl4030_codec_on()) && (twl4030_ext_mut_off());
	if (unlikely(ret))
		printk(KERN_ERR "twl4030_ext_mut_off failed [%d]\n", ret);
set_stereo_mode_exit:
	if (ret)
		printk(KERN_ERR "Setting Stereo mode failed[0x%x]\n", mode);
	return ret;
}

/*
 * Check if the device is in real present or not.
 * If present then register the mixer device. else return failure
 */
int __devinit omap_twl4030_probe(struct platform_device *pdev)
{
	struct omap_alsa_codec_config *codec_cfg;
	int ret = 0;

	codec_cfg = pdev->dev.platform_data;
	if (codec_cfg != NULL) {
		codec_cfg->name = "TWL4030";
		codec_cfg->hw_constraints_rates =
					&twl4030_pcm_hw_constraint_list;
		codec_cfg->snd_omap_alsa_playback =
					&twl4030_pcm_hardware_playback;
		codec_cfg->snd_omap_alsa_capture =
					&twl4030_pcm_hardware_capture;
		codec_cfg->codec_configure_dev = twl4030_configure;
		codec_cfg->codec_set_samplerate = twl4030_set_samplerate;
		codec_cfg->codec_set_stereomode = twl4030_stereomode_set;
		codec_cfg->codec_clock_on = omap_twl4030_initialize;
		codec_cfg->codec_clock_off = omap_twl4030_shutdown;
		codec_cfg->get_default_samplerate = twl4030_default_samplerate;
		codec_cfg->get_default_samplerate = twl4030_default_samplerate;
		ret = snd_omap_alsa_post_probe(pdev, codec_cfg);
	} else {
		return -ENODEV;
	}

	/* Check if T2 device is actually present - Read IDCODE reg */
	printk(KERN_INFO PLATFORM_NAME " " CODEC_NAME " Audio Support: ");
	if (twl4030_i2c_read_u8(TWL4030_MODULE_INTBR, (u8 *)&ret, 0x00)) {
		ret = -ENODEV;
		printk(KERN_INFO "Chip Not detected\n");
		goto twl4030_probe_out;
	}
	if (mixer_dev_id >= 0) {
		/* Announcement Time */
		printk(KERN_INFO "Chip Rev[0x%02x] Initialized\n", ret);
		ret = 0;
	} else {
		printk(KERN_INFO "Mixer Not Initialized\n");
		ret = mixer_dev_id;
	}
twl4030_probe_out:
	return ret;
}

/*******************************************************************************
 *
 * Module APIs
 *
 ******************************************************************************/

static void twl4030_audio_release(struct device *dev)
{
	/* Nothing to release */
}

static struct platform_driver omap_audio_driver = {
	.probe = omap_twl4030_probe,
	.remove = snd_omap_alsa_remove,
#ifdef CONFIG_PM
	.suspend = snd_omap_alsa_suspend,
	.resume = snd_omap_alsa_resume,
#endif
	.driver = {
		.name = OMAP_AUDIO_NAME
	}
};

static struct platform_device omap_audio_device = {
	.name = OMAP_AUDIO_NAME,
	.id = 7,
	.dev = {
		.release = twl4030_audio_release,
	},
};

/*
 * twl4030_init
 */
static int __init twl4030_init(void)
{

	int err = 0;
	struct omap_alsa_codec_config *codec_cfg;

	/* Initialize TWL4030 configured counter */
	twl4030_configured = 0;

	/* Register the codec with the audio driver */
	err = platform_driver_register(&omap_audio_driver);
	if (err)
		printk(KERN_ERR "Failed to register TWL driver"
		       " with Audio ALSA Driver\n");

	codec_cfg = kmalloc(sizeof(struct omap_alsa_codec_config), GFP_KERNEL);
	if (!codec_cfg)
		return -ENOMEM;
	memset(codec_cfg, 0, sizeof(struct omap_alsa_codec_config));
	omap_audio_device.dev.platform_data = codec_cfg;

	err = platform_device_register(&omap_audio_device);
	if (err) {
		printk(KERN_ERR "OMAP Audio Device Register failed =%d\n", err);
		kfree(codec_cfg);
		codec_cfg = NULL;
		return err;
	}
	return err;
}

/*
 * twl4030_exit
 *
 */
static void __exit twl4030_exit(void)
{
	platform_device_unregister(&omap_audio_device);
	platform_driver_unregister(&omap_audio_driver);

	return;
}

/*******************************************************************************
 *
 * Debug APIs
 *
 ******************************************************************************/

/*******************************************************************************
 * TONEGEN:
 * This is a test to generate a rather unpleasant sound..
 * verifies if the mcbsp is active
 ******************************************************************************/
#ifdef TONE_GEN
/* Generates a shrill tone */
u16 tone[] = {
	0x0ce4, 0x0ce4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000,
	0x0CE4, 0x0CE4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000,
	0x0CE4, 0x0CE4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000
};

void generate_tone(void)
{
	int count = 0;
	int ret = 0;

	printk(KERN_INFO "TONE GEN TEST:");

	for (count = 0; count < 5000; count++) {
		int bytes;
		for (bytes = 0; bytes < sizeof(tone) / 2; bytes++) {
			ret = omap_mcbsp_pollwrite(AUDIO_MCBSP, tone[bytes]);
			if (ret == -1) {
				/* Retry */
				bytes--;
			} else if (ret == -2) {
				printk(KERN_INFO "ERROR:bytes=%d\n", bytes);
				return;
			}
		}
	}
	printk(KERN_INFO "SUCCESS\n");
}
#endif	/* End of TONE_GEN */

/*******************************************************************************
 * TWL_DUMP_REGISTERS:
 * This will dump the entire register set of Page 2 twl4030.
 * Useful for major goof ups
 ******************************************************************************/
#ifdef TWL_DUMP_REGISTERS
/*
 * twl4030_dump registers
 */
void twl4030_dump_registers(void)
{
	int i = 0;
	u16 data = 0;

	printk(KERN_INFO "TWL 4030 Register dump for Audio Module\n");
	for (i = REG_CODEC_MODE; i <= REG_MISC_SET_2; i++) {
		data = audio_twl4030_read(i);
		printk(KERN_INFO "Register[0x%02x]=0x%04x\n", i, data);

	}
}
#endif	/* End of #ifdef TWL_DUMP_REGISTERS */

module_init(twl4030_init);
module_exit(twl4030_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Codec audio driver for the TI TWL4030 codec.");
MODULE_LICENSE("GPL");
