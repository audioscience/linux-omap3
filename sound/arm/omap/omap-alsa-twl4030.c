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

#include <asm/arch/hardware.h>
#include <asm/mach-types.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/control.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
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
/* To dump the twl registers for debug */
#undef TWL_DUMP_REGISTERS

#ifdef TWL_DUMP_REGISTERS
static void twl4030_dump_registers(void);
#endif

static int mixer_dev_id;
static int gpio_ext_mut_acquired;

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
static const struct sample_rate_info_t audio_sample_rates[] = {
	{.rate = 8000, .apll = AUDIO_MODE_RATE_08_000},
	{.rate = 16000, .apll = AUDIO_MODE_RATE_16_000},
	{.rate = 44100, .apll = AUDIO_MODE_RATE_44_100},
	{.rate = 11025, .apll = AUDIO_MODE_RATE_11_025},
	{.rate = 12000, .apll = AUDIO_MODE_RATE_12_000},
	{.rate = 22050, .apll = AUDIO_MODE_RATE_22_050},
	{.rate = 24000, .apll = AUDIO_MODE_RATE_24_000},
	{.rate = 32000, .apll = AUDIO_MODE_RATE_32_000},
	{.rate = 48000, .apll = AUDIO_MODE_RATE_48_000},
	/* Dont support 96Khz -requires HSCLK > 26 MHz
	   { .rate = 96000, .apll = AUDIO_MODE_RATE_96_000 }, */
};

struct codec_local_info twl4030_local = {
	.codec_mode = VOICE_MODE,
	.master_rec_vol = WRITE_LR_VOLUME(DEFAULT_INPUT_VOLUME),
	.headset_mic_vol = WRITE_LR_VOLUME(DEFAULT_INPUT_HEADSET_VOLUME),
	.main_mic_vol = WRITE_LR_VOLUME(DEFAULT_INPUT_MIC_VOLUME),
	.sub_mic_vol = WRITE_LR_VOLUME(DEFAULT_INPUT_MIC_VOLUME),
	.aux_vol = WRITE_LR_VOLUME(DEFAULT_INPUT_AUX_VOLUME),
	.carkit_mic_vol = WRITE_LEFT_VOLUME(DEFAULT_INPUT_CARKIT_VOLUME),
	.master_play_vol = WRITE_LR_VOLUME(DEFAULT_OUTPUT_VOLUME),
	.headset_vol = WRITE_LR_VOLUME(DEFAULT_OUTPUT_HEADSET_VOLUME),
	.classd_vol = WRITE_LR_VOLUME(DEFAULT_OUTPUT_HANDSFREE_VOLUME),
	.ear_vol = WRITE_LEFT_VOLUME(DEFAULT_OUTPUT_EARPIECE_VOLUME),
	.carkit_vol = WRITE_LR_VOLUME(DEFAULT_OUTPUT_CARKIT_VOLUME),
	.sidetone_vol = WRITE_LEFT_VOLUME(DEFAULT_SIDETONE_VOLUME),
	.downlink_vol = WRITE_LEFT_VOLUME(DEFAULT_DOWNLINK_VOLUME),
	.handsfree_en = 1,
	.headset_mic_en = 1,
	.main_mic_en = 1,
	.sub_mic_en = 1,
	.input_src = DEFAULT_INPUT_TWL_DEVICE,
	.output_src = DEFAULT_OUTPUT_TWL_DEVICE,
	.audio_samplerate = AUDIO_RATE_DEFAULT,
	.voice_samplerate = VOICE_RATE_DEFAULT,
	.bitspersample = AUDIO_SAMPLE_DATA_WIDTH_16,
	.channels = STEREO_MODE,
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
	int i = 0;
	int ret = 0;

	for (i = 0; i < AUDIO_I2C_RETRY; i++) {
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,
						data, address);
		if (ret < 0) {
			printk(KERN_ERR "TWL4030 write to reg 0x%x %d\n",
				address, ret);
		} else {
			ret = 0;
			break;
		}
	}

	return ret;
}

inline int audio_twl4030_read(u8 address, u8 *data)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < AUDIO_I2C_RETRY; i++) {
		ret = twl4030_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE,
						data, address);
		if (ret < 0) {
			printk(KERN_ERR "TWL4030 read from reg 0x%x %d\n",
				address, ret);
		} else {
			ret = 0;
			break;
		}
	}

	return ret;
}

/*******************************************************************************
 *
 * Codec util APIs
 *
 ******************************************************************************/

/* Modem GPIO conf/setting */
int modem_gpio_conf(void)
{
	int ret = 0;

	ret = omap_request_gpio(MODEM_GPIO);
	if (ret) {
		printk(KERN_ERR "Error requesting modem gpio %d\n", ret);
		return ret;
	}
	omap_set_gpio_direction(MODEM_GPIO, 0);
	omap_set_gpio_dataout(MODEM_GPIO, 1);

	return 0;
}

/*
 * Configure GPIO for data out
 */
inline int twl4030_ext_mut_conf(void)
{
	int ret;

	/* External mute gpio already acquired */
	if (gpio_ext_mut_acquired)
		return 0;

	ret = twl4030_request_gpio(TWL4030_AUDIO_EXT_MUT);
        if (ret)
 		return ret;

	gpio_ext_mut_acquired = 1;
	ret = twl4030_set_gpio_direction(TWL4030_AUDIO_EXT_MUT, 0);

	return ret;
}

/*
 * Unconfigure GPIO used for external mute
 */
static inline int twl4030_ext_mut_unconf(void)
{
	int ret;

	ret = twl4030_free_gpio(TWL4030_AUDIO_EXT_MUT);
	if (!ret)
		gpio_ext_mut_acquired = 0;

	return ret;
}
/*
 * Disable mute also handle time of wait
 */
inline int twl4030_ext_mut_off(void)
{
	/* Wait for ramp duration, settling time for signal */
	udelay(1);

	/* Clear mute */
        return twl4030_set_gpio_dataout(TWL4030_AUDIO_EXT_MUT, 0);
}

/*
 * Enable mute
 */
inline int twl4030_ext_mut_on(void)
{
	return twl4030_set_gpio_dataout(TWL4030_AUDIO_EXT_MUT, 1);
}

/*
 * twl4030_codec_on - Switch codec on
 */
inline int twl4030_codec_on(void)
{
	u8 data = 0;
	int ret = 0;

	ret = audio_twl4030_read(REG_CODEC_MODE, &data);
	if (ret) {
		printk(KERN_ERR "Reg read failed\n");
		return ret;
	}

	if (data & BIT_CODEC_MODE_CODECPDZ_M)
		return 0;

	data |= BIT_CODEC_MODE_CODECPDZ_M;

	return audio_twl4030_write(REG_CODEC_MODE, data);
}

/*
 * twl4030_codec_off - Switch codec off
 */
inline int twl4030_codec_off(void)
{
	u8 data = 0;
	int ret = 0;

	ret = audio_twl4030_read(REG_CODEC_MODE, &data);
	if (ret) {
		printk(KERN_ERR "Reg read failed\n");
		return ret;
	}
	/* Expected to be already off at bootup - but
	 * we do not know the status of the device
	 * hence would like to force a shut down
	 */
	if (!(data & BIT_CODEC_MODE_CODECPDZ_M))
		return 0;

	data &= ~BIT_CODEC_MODE_CODECPDZ_M;

	return audio_twl4030_write(REG_CODEC_MODE, data);
}

/*
 * twl4030_codec_tog_on - Set the power to on after toggle to off and then on
 */
int twl4030_codec_tog_on(void)
{
	u8 data = 0;
	int ret = 0;

	ret = audio_twl4030_read(REG_CODEC_MODE, &data);
	if (ret) {
		printk(KERN_ERR "Reg read failed %d\n", ret);
		return ret;
	}
	data &= ~BIT_CODEC_MODE_CODECPDZ_M;
	ret =  audio_twl4030_write(REG_CODEC_MODE, data);
	if (ret) {
		printk(KERN_ERR "Reg write failed %d\n", ret);
		return ret;
	}

	udelay(10); /* 10 ms delay for power settling */
	data |= BIT_CODEC_MODE_CODECPDZ_M;
	ret =  audio_twl4030_write(REG_CODEC_MODE, data);
	if (ret) {
		printk(KERN_ERR "Reg write failed %d\n", ret);
		return ret;
	}
	udelay(10); /* 10 ms delay for power settling */

	return ret;
}

/*
 * twl4030_set_codec_mode - Set code operational mode: audio, voice
 */
int twl4030_set_codec_mode(int codec_mode)
{
	u8 data = 0;
	int ret = 0;

	ret = audio_twl4030_read(REG_CODEC_MODE, &data);
	if (ret) {
		printk(KERN_ERR "Failed to read codec mode %d\n", ret);
		return ret;
	}

	/* Option 1: Audio mode */
	if (codec_mode == AUDIO_MODE) {
		data |= BIT_CODEC_MODE_OPT_MODE_M;
	/* Option 2: Voice/audio mode */
	} else if (codec_mode == VOICE_MODE) {
		data &= ~BIT_CODEC_MODE_OPT_MODE_M;
	} else {
		printk(KERN_ERR "Invalid codec mode\n");
		return -EPERM;
	}

	ret = audio_twl4030_write(REG_CODEC_MODE, data);
	if (ret)
		printk(KERN_ERR "Failed to write codec mode %d\n", ret);

	return ret;
}


/*
 * twl4030_enable_output - Enable the output path
 */
static int twl4030_enable_output(int codec_mode, int source)
{
	u8 option = 0;
	u8 ear_ctl = 0;
	u8 hs_sel = 0;
	u8 hs_pop = 0;
	u8 hfl_ctl = 0;
	u8 hfr_ctl = 0;
	u8 preckl_ctl = 0;
	u8 preckr_ctl = 0;
	u8 dac = 0;
	u8 data = 0;
	int ret = 0;

	ret = audio_twl4030_read(REG_OPTION, &option);
	if (ret)
		goto enable_op_exit;

	ret = audio_twl4030_read(REG_AVDAC_CTL, &dac);
	if (ret)
		goto enable_op_exit;

	option &= ~(BIT_OPTION_ARXR2_EN_M | BIT_OPTION_ARXL2_EN_M);
	dac &= ~(BIT_AVDAC_CTL_VDAC_EN_M | BIT_AVDAC_CTL_ADACL2_EN_M
		| BIT_AVDAC_CTL_ADACR2_EN_M);

	switch (source) {
	/* Headset */
	case OUTPUT_STEREO_HEADSET:
		if (codec_mode == AUDIO_MODE) {
			option |= BIT_OPTION_ARXR2_EN_M
					| BIT_OPTION_ARXL2_EN_M;
			hs_sel = BIT_HS_SEL_HSOR_AR2_EN_M
					| BIT_HS_SEL_HSOL_AL2_EN_M;
			dac |= BIT_AVDAC_CTL_ADACL2_EN_M
					| BIT_AVDAC_CTL_ADACR2_EN_M;
		} else if (codec_mode == VOICE_MODE) {
			option |= BIT_OPTION_ARXL1_VRX_EN_M;
			hs_sel = BIT_HS_SEL_HSOL_VOICE_EN_M;
			dac |= BIT_AVDAC_CTL_VDAC_EN_M;
		}
		hs_pop = BIT_HS_POPN_SET_VMID_EN_M;
		break;
	/* Hands free */
	case OUTPUT_HANDS_FREE_CLASSD:
		if (twl4030_local.handsfree_en) {
			if (codec_mode == AUDIO_MODE) {
				option |= BIT_OPTION_ARXR2_EN_M
						| BIT_OPTION_ARXL2_EN_M;
				hfl_ctl = (HANDS_FREEL_AL2
						<< BIT_HFL_CTL_HFL_INPUT_SEL)
						| BIT_HFL_CTL_HFL_REF_EN_M;
				hfr_ctl = (HANDS_FREER_AR2
						<< BIT_HFR_CTL_HFR_INPUT_SEL)
						| BIT_HFR_CTL_HFR_REF_EN_M;
				dac |= BIT_AVDAC_CTL_ADACL2_EN_M
						| BIT_AVDAC_CTL_ADACR2_EN_M;
			} else if (codec_mode == VOICE_MODE) {
				option |= BIT_OPTION_ARXL1_VRX_EN_M;
				hfl_ctl = (HANDS_FREEL_VOICE
						<< BIT_HFL_CTL_HFL_INPUT_SEL)
						| BIT_HFL_CTL_HFL_REF_EN_M;
				dac |= BIT_AVDAC_CTL_VDAC_EN_M;
			}
		}
		break;
	/* Earpiece */
	case OUTPUT_MONO_EARPIECE:
		if (codec_mode == AUDIO_MODE) {
			option |= BIT_OPTION_ARXL2_EN_M;
			ear_ctl = BIT_EAR_CTL_EAR_AL2_EN_M;
			dac |= BIT_AVDAC_CTL_ADACL2_EN_M;
		} else if (codec_mode == VOICE_MODE) {
			option |= BIT_OPTION_ARXL1_VRX_EN_M;
			ear_ctl = BIT_EAR_CTL_EAR_VOICE_EN_M;
			dac |= BIT_AVDAC_CTL_VDAC_EN_M;
		}
		break;
	/* Carkit */
	case OUTPUT_CARKIT:
		if (codec_mode == AUDIO_MODE) {
			option |= BIT_OPTION_ARXL2_EN_M
					| BIT_OPTION_ARXR2_EN_M;
			preckl_ctl = BIT_PRECKL_CTL_PRECKL_AL2_EN_M
					| BIT_PRECKL_CTL_PRECKL_EN_M;
			preckr_ctl = BIT_PRECKR_CTL_PRECKR_AR2_EN_M
					| BIT_PRECKR_CTL_PRECKR_EN_M;
			dac |= BIT_AVDAC_CTL_ADACL2_EN_M
					| BIT_AVDAC_CTL_ADACR2_EN_M;
		} else if (codec_mode == VOICE_MODE) {
			option |= BIT_OPTION_ARXL1_VRX_EN_M;
			preckl_ctl = BIT_PRECKL_CTL_PRECKL_VOICE_EN_M
					| BIT_PRECKL_CTL_PRECKL_EN_M;
			dac |= BIT_AVDAC_CTL_VDAC_EN_M;
		}
		break;
	}

	ret = audio_twl4030_write(REG_OPTION, option);
	if (ret)
		goto enable_op_exit;

	if (ear_ctl) {
		ret = audio_twl4030_read(REG_EAR_CTL, &data);
		if (ret)
			goto enable_op_exit;

		data &= ~(BIT_EAR_CTL_EAR_AR1_EN_M
				| BIT_EAR_CTL_EAR_AL2_EN_M
				| BIT_EAR_CTL_EAR_AL1_EN_M
				| BIT_EAR_CTL_EAR_VOICE_EN_M);
		data |= ear_ctl;
		ret = audio_twl4030_write(REG_EAR_CTL, data);
		if (ret)
			goto enable_op_exit;
	}

	if (hs_sel) {
		ret = audio_twl4030_write(REG_HS_SEL, hs_sel);
		if (ret)
			goto enable_op_exit;
	}

	if (hs_pop) {
		ret = audio_twl4030_write(REG_HS_POPN_SET, hs_pop);
		if (ret)
			goto enable_op_exit;

		hs_pop |= BIT_HS_POPN_SET_RAMP_EN_M;
		udelay(1); /* Require a short delay before enabling ramp */
		ret = audio_twl4030_write(REG_HS_POPN_SET, hs_pop);
		if (ret)
			goto enable_op_exit;
	}

	if (hfl_ctl) {
		ret = audio_twl4030_write(REG_HFL_CTL, hfl_ctl);
		if (ret)
			goto enable_op_exit;

		hfl_ctl |= BIT_HFL_CTL_HFL_RAMP_EN_M;
		ret = audio_twl4030_write(REG_HFL_CTL, hfl_ctl);
		if (ret)
			goto enable_op_exit;

		hfl_ctl |= BIT_HFL_CTL_HFL_LOOP_EN_M;
		ret = audio_twl4030_write(REG_HFL_CTL, hfl_ctl);
		if (ret)
			goto enable_op_exit;

		hfl_ctl |= BIT_HFL_CTL_HFL_HB_EN_M;
		ret = audio_twl4030_write(REG_HFL_CTL, hfl_ctl);
		if (ret)
			goto enable_op_exit;
	}

	if (hfr_ctl) {
		ret = audio_twl4030_write(REG_HFR_CTL, hfr_ctl);
		if (ret)
			goto enable_op_exit;

		hfr_ctl |= BIT_HFR_CTL_HFR_RAMP_EN_M;
		ret = audio_twl4030_write(REG_HFR_CTL, hfr_ctl);
		if (ret)
			goto enable_op_exit;

		hfr_ctl |= BIT_HFR_CTL_HFR_LOOP_EN_M;
		ret = audio_twl4030_write(REG_HFR_CTL, hfr_ctl);
		if (ret)
			goto enable_op_exit;

		hfr_ctl |= BIT_HFR_CTL_HFR_HB_EN_M;
		ret = audio_twl4030_write(REG_HFR_CTL, hfr_ctl);
		if (ret)
			goto enable_op_exit;
	}

	if (preckl_ctl) {
		ret = audio_twl4030_write(REG_PRECKL_CTL, preckl_ctl);
		if (ret)
			goto enable_op_exit;
	}

	if (preckr_ctl) {
		ret = audio_twl4030_write(REG_PRECKR_CTL, preckr_ctl);
		if (ret)
			goto enable_op_exit;
	}

	ret = audio_twl4030_write(REG_AVDAC_CTL, dac);
enable_op_exit:
	if (ret)
		printk(KERN_ERR "Error enabling output %d\n", ret);

	return ret;
}

/*
 * twl4030_disable_outputs - Remove the output path
 */
static int twl4030_disable_outputs(void)
{
	int ret = 0;
	u8 opt = 0;

	/* Turn RX filters off */
	ret = audio_twl4030_read(REG_OPTION, &opt);
	if (ret)
		goto disable_op_exit;

	opt &= ~(BIT_OPTION_ARXL1_VRX_EN_M | BIT_OPTION_ARXR1_EN_M
		| BIT_OPTION_ARXL2_EN_M | BIT_OPTION_ARXR2_EN_M);
	ret = audio_twl4030_write(REG_OPTION, opt);
	if (ret)
		goto disable_op_exit;

	/* Turn DACs off */
	ret = audio_twl4030_write(REG_AVDAC_CTL, 0x00);
	if (ret)
		goto disable_op_exit;

	/* Headset */
	ret = audio_twl4030_write(REG_HS_SEL, 0x00);
	if (ret)
		goto disable_op_exit;

	/* Hands free */
	ret = audio_twl4030_write(REG_HFL_CTL, 0x00);
	if (ret)
		goto disable_op_exit;

	ret = audio_twl4030_write(REG_HFR_CTL, 0x00);
	if (ret)
		goto disable_op_exit;

	/* Earpiece */
	ret = audio_twl4030_write(REG_EAR_CTL, 0x00);
	if (ret)
		goto disable_op_exit;

	/* Carkit */
	ret = audio_twl4030_write(REG_PRECKL_CTL, 0x00);
	if (ret)
		goto disable_op_exit;

	ret = audio_twl4030_write(REG_PRECKR_CTL, 0x00);
disable_op_exit:
	if (ret)
		printk(KERN_ERR "Error disabling output %d\n", ret);

	return ret;
}

/*
 * twl4030_enable_input - Enable the input to the correct device.
 */
static int twl4030_enable_input(int codec_mode, int source)
{
	u8 micbias = 0;
	u8 mic_l = 0;
	u8 mic_r = 0;
	u8 adc = 0;
	u8 option = 0;
	int ret = 0;

	ret = audio_twl4030_read(REG_OPTION, &option);
	if (ret)
		goto enable_ip_exit;

	ret = audio_twl4030_read(REG_AVADC_CTL, &adc);
	if (ret)
		goto enable_ip_exit;

	option &= ~(BIT_OPTION_ATXL1_EN_M
			| BIT_OPTION_ATXR1_EN_M
			| BIT_OPTION_ATXL2_VTXL_EN_M
			| BIT_OPTION_ATXR2_VTXR_EN_M);
	adc &= ~(BIT_AVADC_CTL_ADCL_EN_M | BIT_AVADC_CTL_ADCR_EN_M);

	switch (source) {
	/* Headset mic */
	case INPUT_HEADSET_MIC:
		if (twl4030_local.headset_mic_en) {
			micbias = BIT_MICBIAS_CTL_HSMICBIAS_EN_M;
			mic_l = BIT_ANAMICL_HSMIC_EN_M
					| BIT_ANAMICL_MICAMPL_EN_M;
			adc |= BIT_AVADC_CTL_ADCL_EN_M;
			if (codec_mode == AUDIO_MODE) {
				option |= BIT_OPTION_ATXL1_EN_M;
			} else if (codec_mode == VOICE_MODE) {
				option |= BIT_OPTION_ATXL2_VTXL_EN_M;
				mic_l |= (OFFSET_CNCL_VRX
					<< BIT_ANAMICL_OFFSET_CNCL_SEL);
			}
		}
		break;
	/* Main mic */
	case INPUT_MAIN_MIC:
		if (twl4030_local.main_mic_en) {
			mic_l = BIT_ANAMICL_MAINMIC_EN_M
					 | BIT_ANAMICL_MICAMPL_EN_M;
			micbias = BIT_MICBIAS_CTL_MICBIAS1_EN_M;
			adc |= BIT_AVADC_CTL_ADCL_EN_M;
			if (codec_mode == AUDIO_MODE) {
				option |= BIT_OPTION_ATXL1_EN_M;
			} else if (codec_mode == VOICE_MODE) {
				option |= BIT_OPTION_ATXL2_VTXL_EN_M;
				mic_l |= (OFFSET_CNCL_VRX
					<< BIT_ANAMICL_OFFSET_CNCL_SEL);
			}
		}
		break;
	/* Sub mic */
	case INPUT_SUB_MIC:
		if (twl4030_local.sub_mic_en) {
			micbias = BIT_MICBIAS_CTL_MICBIAS2_EN_M;
			mic_r = BIT_ANAMICR_SUBMIC_EN_M
					 | BIT_ANAMICR_MICAMPR_EN_M;
			adc |= BIT_AVADC_CTL_ADCR_EN_M;
			if (codec_mode == AUDIO_MODE)
				option |= BIT_OPTION_ATXR1_EN_M;
			else if (codec_mode == VOICE_MODE)
				option |= BIT_OPTION_ATXR2_VTXR_EN_M;
		}
		break;
	/* Auxiliar input */
	case INPUT_AUX:
		mic_l = BIT_ANAMICL_AUXL_EN_M | BIT_ANAMICL_MICAMPL_EN_M;
		mic_r = BIT_ANAMICR_AUXR_EN_M | BIT_ANAMICR_MICAMPR_EN_M;
		adc |= BIT_AVADC_CTL_ADCL_EN_M | BIT_AVADC_CTL_ADCR_EN_M;
		if (codec_mode == AUDIO_MODE) {
			option |= BIT_OPTION_ATXL1_EN_M
					| BIT_OPTION_ATXR1_EN_M;
		} else if (codec_mode == VOICE_MODE) {
			option |= BIT_OPTION_ATXL2_VTXL_EN_M
					| BIT_OPTION_ATXR2_VTXR_EN_M;
			mic_l |= (OFFSET_CNCL_VRX
					<< BIT_ANAMICL_OFFSET_CNCL_SEL);
		}
		break;
	/* Carkit */
	case INPUT_CARKIT:
		mic_l = BIT_ANAMICL_CKMIC_EN_M | BIT_ANAMICL_MICAMPL_EN_M;
		adc |= BIT_AVADC_CTL_ADCL_EN_M;
		if (codec_mode == AUDIO_MODE) {
			option |= BIT_OPTION_ATXL1_EN_M;
		} else if (codec_mode == VOICE_MODE) {
			option |= BIT_OPTION_ATXL2_VTXL_EN_M;
			mic_l |= (OFFSET_CNCL_VRX <<
					BIT_ANAMICL_OFFSET_CNCL_SEL);
		}
		break;
	}

	ret = audio_twl4030_write(REG_OPTION, option);
	if (ret)
		goto enable_ip_exit;

	ret = audio_twl4030_write(REG_MICBIAS_CTL, micbias);
	if (ret)
		goto enable_ip_exit;

	ret = audio_twl4030_write(REG_ANAMICL, mic_l);
	if (ret)
		goto enable_ip_exit;

	ret = audio_twl4030_write(REG_ANAMICR, mic_r);
	if (ret)
		goto enable_ip_exit;

	ret = audio_twl4030_write(REG_AVADC_CTL, adc);
	if (ret)
		goto enable_ip_exit;

	/* Use ADC TX2 routed to digi mic - we dont use tx2 path
	 * - route it to digi mic1
	 */
	ret = audio_twl4030_write(REG_ADCMICSEL, 0x00);
	if (ret)
		goto enable_ip_exit;

	ret = audio_twl4030_write(REG_DIGMIXING, 0x00);
enable_ip_exit:
	if (ret)
		printk(KERN_ERR "Error enabling input %d\n", ret);

	return ret;
}

/*
 * twl4030_disable_inputs - Reset all the inputs
 */
static int twl4030_disable_inputs(void)
{
	u8 opt = 0;
	int ret = 0;

	/* Turn TX filters off */
	ret = audio_twl4030_read(REG_OPTION, &opt);
	if (ret)
		goto disable_ip_exit;

	opt &= ~(BIT_OPTION_ATXL1_EN_M | BIT_OPTION_ATXR1_EN_M
		| BIT_OPTION_ATXL2_VTXL_EN_M | BIT_OPTION_ATXR2_VTXR_EN_M);
	ret = audio_twl4030_write(REG_OPTION, opt);
	if (ret)
		goto disable_ip_exit;

	/* Turn DACs off */
	ret = audio_twl4030_write(REG_AVDAC_CTL, 0x00);
	if (ret)
		goto disable_ip_exit;

	/* Turn mics bias off */
	ret = audio_twl4030_write(REG_MICBIAS_CTL, 0x00);
	if (ret)
		goto disable_ip_exit;

	ret = audio_twl4030_write(REG_ANAMICL, 0x00);
	if (ret)
		goto disable_ip_exit;

	ret = audio_twl4030_write(REG_ANAMICR, 0x00);
	if (ret)
		goto disable_ip_exit;

	ret = audio_twl4030_write(REG_ADCMICSEL, 0x00);
	if (ret)
		goto disable_ip_exit;

	ret = audio_twl4030_write(REG_DIGMIXING, 0x00);

disable_ip_exit:
	if (ret)
		printk(KERN_ERR "Error disabling input %d\n", ret);
	return ret;
}

/*
 * twl4030_select_source - Setup specified source
 */
int twl4030_select_source(int source)
{
	int ret = 0;

	if (is_input(source))
		ret = twl4030_disable_inputs();
	else
		ret = twl4030_disable_outputs();
	if (ret)
		goto select_src_exit;

	switch (source) {
	case HEADSET_MIC_SOURCE:
		ret = twl4030_enable_input(twl4030_local.codec_mode,
					INPUT_HEADSET_MIC);
		break;
	case MAIN_SUB_MIC_SOURCE:
		ret = twl4030_enable_input(twl4030_local.codec_mode,
					INPUT_SUB_MIC);
		if (ret)
			goto select_src_exit;
		ret = twl4030_enable_input(twl4030_local.codec_mode,
					INPUT_MAIN_MIC);
		break;
	case AUX_SOURCE:
		ret = twl4030_enable_input(twl4030_local.codec_mode,
					INPUT_AUX);
		break;
	case CARKIT_MIC_SOURCE:
		ret = twl4030_enable_input(twl4030_local.codec_mode,
					INPUT_CARKIT);
		break;
	case HEADSET_SOURCE:
		ret = twl4030_enable_output(twl4030_local.codec_mode,
					OUTPUT_STEREO_HEADSET);
		break;
	case HANDS_FREE_SOURCE:
		ret = twl4030_enable_output(twl4030_local.codec_mode,
					OUTPUT_HANDS_FREE_CLASSD);
		break;
	case EARPIECE_SOURCE:
		ret = twl4030_enable_output(twl4030_local.codec_mode,
					OUTPUT_MONO_EARPIECE);
		break;
	case CARKIT_SOURCE:
		ret = twl4030_enable_output(twl4030_local.codec_mode,
					OUTPUT_CARKIT);
		break;
	}

select_src_exit:
	if (ret)
		printk(KERN_ERR "Error setting source %d\n", ret);

	return ret;
}

/*
 * compute_gain - Calculate gain for a volume in range [0, 100]
 */
u8 compute_gain(int volume, int min_gain, int max_gain)
{
	int gain = 0;

	gain = ((max_gain - min_gain) * volume + AUDIO_MAX_VOLUME * min_gain) /
		AUDIO_MAX_VOLUME;

	return (u8) gain;
}

/*
 * twl4030_set_digital_volume - Set digital volume for input/output sources
 */
int twl4030_set_digital_volume(int source, int volume)
{
	u8 reg = 0;
	u8 data = 0;
	u8 coarse_gain = 0;
	u8 fine_gain = 0;
	int ret = 0;

	if (volume > AUDIO_MAX_VOLUME) {
		printk(KERN_ERR "Invalid volume: %d\n", volume);
		return -EPERM;
	}

	if (is_input(source)) {
		fine_gain = compute_gain(volume,
				INPUT_MIN_GAIN, INPUT_MAX_GAIN);
	} else if (is_output(source)) {
		fine_gain = compute_gain(volume,
				OUTPUT_MIN_GAIN, OUTPUT_MAX_GAIN);
		coarse_gain = AUDIO_DEF_COARSE_VOLUME_LEVEL;
	} else if (is_voice(source)) {
		fine_gain = compute_gain(volume,
				VOICE_MIN_GAIN, VOICE_MAX_GAIN);
	} else if (is_sidetone(source)) {
		fine_gain = compute_gain(volume,
				SIDETONE_MIN_GAIN, SIDETONE_MAX_GAIN);
	}

	switch (source) {
	case TXL1:
		reg = REG_ATXL1PGA;
		data = fine_gain << BIT_ATXL1PGA_ATXL1PGA_GAIN;
		break;
	case TXR1:
		reg = REG_ATXR1PGA;
		data = fine_gain << BIT_ATXR1PGA_ATXR1PGA_GAIN;
		break;
	case TXL2:
		reg = REG_AVTXL2PGA;
		data = fine_gain << BIT_AVTXL2PGA_AVTXL2PGA_GAIN;
		break;
	case TXR2:
		reg = REG_AVTXR2PGA;
		data = fine_gain << BIT_AVTXR2PGA_AVTXR2PGA_GAIN;
		break;
	case RXL1:
		reg = REG_ARXL1PGA;
		data = coarse_gain << BIT_ARXL1PGA_ARXL1PGA_CGAIN
			| fine_gain << BIT_ARXL1PGA_ARXL1PGA_FGAIN;
		break;
	case RXR1:
		reg = REG_ARXR1PGA;
		data = coarse_gain << BIT_ARXR1PGA_ARXR1PGA_CGAIN
			| fine_gain << BIT_ARXR1PGA_ARXR1PGA_FGAIN;
		break;
	case RXL2:
		reg = REG_ARXL2PGA;
		data = coarse_gain << BIT_ARXL2PGA_ARXL2PGA_CGAIN
			| fine_gain << BIT_ARXL2PGA_ARXL2PGA_FGAIN;
		break;
	case RXR2:
		reg = REG_ARXR2PGA;
		data = coarse_gain << BIT_ARXR2PGA_ARXR2PGA_CGAIN
			| fine_gain << BIT_ARXR2PGA_ARXR2PGA_FGAIN;
		break;
	case VDL:
		reg = REG_VRXPGA;
		data = fine_gain << BIT_VRXPGA_VRXPGA_GAIN;
		break;
	case VST:
		reg = REG_VSTPGA;
		data = fine_gain << BIT_VSTPGA_VSTPGA_GAIN;
		break;
	default:
		printk(KERN_ERR "Invalid source type\n");
		return -EPERM;
	}

	ret = audio_twl4030_write(reg, data);
	if (ret)
		printk(KERN_ERR "Error setting digital volume %d\n", ret);

	return ret;
}

/*
 * twl4030_set_analog_volume - Set analog volume for input/output sources
 */
static int twl4030_set_analog_volume(int source, int volume)
{
	u8 gain = 0;
	u8 data = 0;
	u8 reg = 0;
	int ret = 0;

	if (volume > AUDIO_MAX_VOLUME) {
		printk(KERN_ERR "Invalid volume: %d\n", volume);
		return -EPERM;
	}

	gain = compute_gain(volume, OUTPUT_ANALOG_GAIN_MIN,
				OUTPUT_ANALOG_GAIN_MAX);
	switch (source) {
	case RXL1:
		reg = REG_ARXL1_APGA_CTL;
		data = gain << BIT_ARXL1_APGA_CTL_ARXL1_GAIN_SET
			| BIT_ARXL1_APGA_CTL_ARXL1_DA_EN_M
			| BIT_ARXL1_APGA_CTL_ARXL1_PDZ_M;
		break;
	case RXR1:
		reg = REG_ARXR1_APGA_CTL;
		data = gain << BIT_ARXR1_APGA_CTL_ARXR1_GAIN_SET
			| BIT_ARXR1_APGA_CTL_ARXR1_DA_EN_M
			| BIT_ARXR1_APGA_CTL_ARXR1_PDZ_M;
		break;
	case RXL2:
		reg = REG_ARXL2_APGA_CTL;
		data = gain << BIT_ARXL2_APGA_CTL_ARXL2_GAIN_SET
			| BIT_ARXL2_APGA_CTL_ARXL2_DA_EN_M
			| BIT_ARXL2_APGA_CTL_ARXL2_PDZ_M;
		break;
	case RXR2:
		reg = REG_ARXR2_APGA_CTL;
		data = gain << BIT_ARXR2_APGA_CTL_ARXR2_GAIN_SET
			| BIT_ARXR2_APGA_CTL_ARXR2_DA_EN_M
			| BIT_ARXR2_APGA_CTL_ARXR2_PDZ_M;
		break;
	case VDL:
		reg = REG_VDL_APGA_CTL;
		data = gain << BIT_VDL_APGA_CTL_VDL_GAIN_SET
			| BIT_VDL_APGA_CTL_VDL_DA_EN_M
			| BIT_VDL_APGA_CTL_VDL_PDZ_M;
		break;
	default:
		printk(KERN_ERR "Invalid source type\n");
		return -EPERM;
	}

	ret = audio_twl4030_write(reg, data);
	if (ret)
		printk(KERN_ERR "Error setting analog volume %d\n", ret);

	return ret;
}

/*
 * twl4030_set_volume -Set the gain of the requested device
 */
int twl4030_setvolume(int source, int volume_l, int volume_r)
{
	u8 gain_l = 0;
	u8 gain_r = 0;
	u8 data = 0;
	int ret = 0;

	if ((volume_l > AUDIO_MAX_VOLUME) || (volume_r > AUDIO_MAX_VOLUME)) {
		printk(KERN_ERR "Invalid volume values: %d, %d\n",
			volume_l, volume_r);
		return -EPERM;
	}

	switch (source) {
	case OUTPUT_VOLUME:
		ret = twl4030_set_digital_volume(RXL2, volume_l);
		if (ret) {
			printk(KERN_ERR "RXL2 digital volume error %d\n", ret);
			return ret;
		}
		ret = twl4030_set_digital_volume(RXR2, volume_r);
		if (ret) {
			printk(KERN_ERR "RXR2 digital volume error %d\n", ret);
			return ret;
		}
		ret = twl4030_set_analog_volume(RXL2, volume_l);
		if (ret) {
			printk(KERN_ERR "RXL2 analog volume %d\n", ret);
			return ret;
		}
		ret = twl4030_set_analog_volume(RXR2, volume_r);
		if (ret) {
			printk(KERN_ERR "RXR2 analog volume %d\n", ret);
			return ret;
		}
		twl4030_local.master_play_vol = WRITE_LEFT_VOLUME(volume_l) |
						WRITE_RIGHT_VOLUME(volume_r);
		break;
	case OUTPUT_STEREO_HEADSET:
		gain_l = compute_gain(volume_l, HEADSET_GAIN_MIN,
					HEADSET_GAIN_MAX);
		gain_r = compute_gain(volume_r, HEADSET_GAIN_MIN,
					HEADSET_GAIN_MAX);
		/* Handle mute request */
		gain_l = (volume_l == 0) ? 0 : gain_l;
		gain_r = (volume_r == 0) ? 0 : gain_r;
		ret = audio_twl4030_write(REG_HS_GAIN_SET,
				  (gain_l << BIT_HS_GAIN_SET_HSL_GAIN) |
				  (gain_r << BIT_HS_GAIN_SET_HSR_GAIN));
		if (ret) {
			printk(KERN_ERR "Headset volume error %d\n", ret);
			return ret;
		}
		twl4030_local.headset_vol = WRITE_LEFT_VOLUME(volume_l) |
					WRITE_RIGHT_VOLUME(volume_r);
		break;
	case OUTPUT_HANDS_FREE_CLASSD:
		/* ClassD no special gain */
		twl4030_local.classd_vol = WRITE_LEFT_VOLUME(volume_l) |
					WRITE_RIGHT_VOLUME(volume_r);
		break;
	case OUTPUT_MONO_EARPIECE:
		gain_l = compute_gain(volume_l, EARPHONE_GAIN_MIN,
					EARPHONE_GAIN_MAX);
		gain_l = (volume_l == 0) ? 0 : gain_l;
		ret = audio_twl4030_read(REG_EAR_CTL, &data);
		if (ret) {
			printk(KERN_ERR "Earpiece volume error %d\n", ret);
			return ret;
		}
		data &= ~BIT_EAR_CTL_EAR_GAIN_M;
		data |= gain_l << BIT_EAR_CTL_EAR_GAIN;
		ret = audio_twl4030_write(REG_EAR_CTL, data);
		if (ret) {
			printk(KERN_ERR "Earpiece volume error %d\n", ret);
			return ret;
		}
		twl4030_local.ear_vol = WRITE_LEFT_VOLUME(volume_l);
		break;
	case OUTPUT_SIDETONE:
		ret = twl4030_set_digital_volume(VST, volume_l);
		if (ret) {
			printk(KERN_ERR "Sidetone volume error %d\n", ret);
			return ret;
		}
		twl4030_local.sidetone_vol = WRITE_LEFT_VOLUME(volume_l);
		break;
	case OUTPUT_CARKIT:
		gain_l = compute_gain(volume_l, CARKIT_MIN_GAIN,
					CARKIT_MAX_GAIN);
		gain_r = compute_gain(volume_r, CARKIT_MIN_GAIN,
					CARKIT_MAX_GAIN);
		gain_l = (volume_l == 0) ? 0 : gain_l;
		gain_r = (volume_r == 0) ? 0 : gain_r;
		/* Left gain */
		ret = audio_twl4030_read(REG_PRECKL_CTL, &data);
		if (ret) {
			printk(KERN_ERR "Carkit output volume error %d\n", ret);
			return ret;
		}
		data &= ~BIT_PRECKL_CTL_PRECKL_GAIN_M;
		data |=  gain_l << BIT_PRECKL_CTL_PRECKL_GAIN;
		ret = audio_twl4030_write(REG_PRECKL_CTL, data);
		if (ret) {
			printk(KERN_ERR "Carkit output volume error %d\n", ret);
			return ret;
		}
		twl4030_local.carkit_vol = WRITE_LEFT_VOLUME(volume_l);
		/* Right gain */
		ret = audio_twl4030_read(REG_PRECKR_CTL, &data);
		if (ret) {
			printk(KERN_ERR "Carkit output volume error %d\n", ret);
			return ret;
		}
		data &= ~BIT_PRECKR_CTL_PRECKR_GAIN_M;
		data |= gain_r << BIT_PRECKR_CTL_PRECKR_GAIN;
		ret = audio_twl4030_write(REG_PRECKR_CTL, data);
		if (ret) {
			printk(KERN_ERR "Carkit output volume error %d\n", ret);
			return ret;
		}
		twl4030_local.carkit_vol |= WRITE_RIGHT_VOLUME(volume_r);
		break;
	case OUTPUT_DOWNLINK:
		ret = twl4030_set_digital_volume(VDL, volume_l);
		if (ret) {
			printk(KERN_ERR "Downlink volume error %d\n", ret);
			return ret;
		}
		ret = twl4030_set_analog_volume(VDL, volume_l);
		if (ret) {
			printk(KERN_ERR "Downlink volume error %d\n", ret);
			return ret;
		}
		twl4030_local.downlink_vol = WRITE_LEFT_VOLUME(volume_l);
		break;
	case INPUT_VOLUME:
		ret = twl4030_set_digital_volume(TXL1, volume_l);
		if (ret) {
			printk(KERN_ERR "TXL1 digital volume error %d\n", ret);
			return ret;
		}
		ret = twl4030_set_digital_volume(TXR1, volume_r);
		if (ret) {
			printk(KERN_ERR "TXR1 digital volume error %d\n", ret);
			return ret;
		}
		ret = twl4030_set_digital_volume(TXL2, volume_l);
		if (ret) {
			printk(KERN_ERR "TXL2 digital volume error %d\n", ret);
			return ret;
		}
		ret = twl4030_set_digital_volume(TXR2, volume_r);
		if (ret) {
			printk(KERN_ERR "TXR2 digital volume error %d\n", ret);
			return ret;
		}
		twl4030_local.master_rec_vol = WRITE_LEFT_VOLUME(volume_l) |
						WRITE_RIGHT_VOLUME(volume_r);
		break;
	case INPUT_HEADSET_MIC:
		gain_l = compute_gain(volume_l, MICROPHONE_MIN_GAIN,
					MICROPHONE_MAX_GAIN);
		ret = audio_twl4030_read(REG_ANAMIC_GAIN, &data);
		if (ret) {
			printk(KERN_ERR "Headset mic volume error %d\n", ret);
			return ret;
		}
		data &= ~BIT_ANAMIC_GAIN_MICAMPL_GAIN_M;
		data |= gain_l << BIT_ANAMIC_GAIN_MICAMPL_GAIN;
		ret = audio_twl4030_write(REG_ANAMIC_GAIN, data);
		if (ret) {
			printk(KERN_ERR "Headset mic volume error %d\n", ret);
			return ret;
		}
		twl4030_local.headset_mic_vol = WRITE_LEFT_VOLUME(volume_l);
		break;
	case INPUT_MAIN_MIC:
		gain_l = compute_gain(volume_l, MICROPHONE_MIN_GAIN,
					MICROPHONE_MAX_GAIN);
		ret = audio_twl4030_read(REG_ANAMIC_GAIN, &data);
		if (ret) {
			printk(KERN_ERR "Main mic volume error %d\n", ret);
			return ret;
		}
		data &= ~BIT_ANAMIC_GAIN_MICAMPL_GAIN_M;
		data |= gain_l << BIT_ANAMIC_GAIN_MICAMPL_GAIN;
		ret = audio_twl4030_write(REG_ANAMIC_GAIN, data);
		if (ret) {
			printk(KERN_ERR "Main mic volume error %d\n", ret);
			return ret;
		}
		twl4030_local.main_mic_vol = WRITE_LEFT_VOLUME(volume_l);
		break;
	case INPUT_SUB_MIC:
		gain_r = compute_gain(volume_r, MICROPHONE_MIN_GAIN,
					MICROPHONE_MAX_GAIN);
		ret = audio_twl4030_read(REG_ANAMIC_GAIN, &data);
		if (ret) {
			printk(KERN_ERR "Sub mic volume error %d\n", ret);
			return ret;
		}
		data &= ~BIT_ANAMIC_GAIN_MICAMPR_GAIN_M;
		data |= gain_r << BIT_ANAMIC_GAIN_MICAMPR_GAIN;
		ret = audio_twl4030_write(REG_ANAMIC_GAIN, data);
		if (ret) {
			printk(KERN_ERR "Sub mic volume error %d\n", ret);
			return ret;
		}
		twl4030_local.sub_mic_vol = WRITE_RIGHT_VOLUME(volume_r);
		break;
	case INPUT_AUX:
		gain_l = compute_gain(volume_l, MICROPHONE_MIN_GAIN,
					MICROPHONE_MAX_GAIN);
		gain_r = compute_gain(volume_r, MICROPHONE_MIN_GAIN,
					MICROPHONE_MAX_GAIN);
		ret = audio_twl4030_write(REG_ANAMIC_GAIN,
				gain_l << BIT_ANAMIC_GAIN_MICAMPL_GAIN
				| gain_r << BIT_ANAMIC_GAIN_MICAMPR_GAIN);
		if (ret) {
			printk(KERN_ERR "Aux input volume error %d\n", ret);
			return ret;
		}
		twl4030_local.aux_vol = WRITE_LEFT_VOLUME(volume_l) |
					WRITE_RIGHT_VOLUME(volume_r);
		break;
	case INPUT_CARKIT:
		gain_l = compute_gain(volume_l, MICROPHONE_MIN_GAIN,
					MICROPHONE_MAX_GAIN);
		gain_r = compute_gain(volume_r, MICROPHONE_MIN_GAIN,
					MICROPHONE_MAX_GAIN);
		ret = audio_twl4030_write(REG_ANAMIC_GAIN,
				gain_l << BIT_ANAMIC_GAIN_MICAMPL_GAIN
				| gain_r << BIT_ANAMIC_GAIN_MICAMPR_GAIN);
		if (ret) {
			printk(KERN_ERR "Carkit input volume error %d\n", ret);
			return ret;
		}
		twl4030_local.carkit_mic_vol = WRITE_LEFT_VOLUME(volume_l) |
						WRITE_RIGHT_VOLUME(volume_r);
		break;
	default:
		printk(KERN_WARNING "Invalid source specified\n");
		ret = -EPERM;
		break;
	}

	return ret;
}


/*
 * twl4030_set_all_volumes - Set stored volume for all sources
 */
static int twl4030_set_all_volumes(void)
{
	int ret = 0;

	/* Input sources */
	ret = twl4030_setvolume(INPUT_VOLUME,
			READ_LEFT_VOLUME(twl4030_local.master_rec_vol),
			READ_RIGHT_VOLUME(twl4030_local.master_rec_vol));
	if (ret)
		goto set_volumes_exit;

	ret = twl4030_setvolume(INPUT_HEADSET_MIC,
			READ_LEFT_VOLUME(twl4030_local.headset_mic_vol),
			READ_RIGHT_VOLUME(twl4030_local.headset_mic_vol));
	if (ret)
		goto set_volumes_exit;

	ret = twl4030_setvolume(INPUT_MAIN_MIC,
			READ_LEFT_VOLUME(twl4030_local.main_mic_vol), 0);
	if (ret)
		goto set_volumes_exit;

	ret = twl4030_setvolume(INPUT_SUB_MIC, 0,
			READ_RIGHT_VOLUME(twl4030_local.sub_mic_vol));
	if (ret)
		goto set_volumes_exit;

	ret = twl4030_setvolume(INPUT_AUX,
			READ_LEFT_VOLUME(twl4030_local.aux_vol),
			READ_RIGHT_VOLUME(twl4030_local.aux_vol));
	if (ret)
		goto set_volumes_exit;

	ret = twl4030_setvolume(INPUT_CARKIT,
			READ_LEFT_VOLUME(twl4030_local.carkit_mic_vol), 0);
	if (ret)
		goto set_volumes_exit;

	/* Output sources */
	ret = twl4030_setvolume(OUTPUT_VOLUME,
			READ_LEFT_VOLUME(twl4030_local.master_play_vol),
			READ_RIGHT_VOLUME(twl4030_local.master_play_vol));
	if (ret)
		goto set_volumes_exit;

	ret = twl4030_setvolume(OUTPUT_STEREO_HEADSET,
			READ_LEFT_VOLUME(twl4030_local.headset_vol),
			READ_RIGHT_VOLUME(twl4030_local.headset_vol));
	if (ret)
		goto set_volumes_exit;

	ret = twl4030_setvolume(OUTPUT_HANDS_FREE_CLASSD,
		      READ_LEFT_VOLUME(twl4030_local.classd_vol),
		      READ_RIGHT_VOLUME(twl4030_local.classd_vol));
	if (ret)
		goto set_volumes_exit;

	ret = twl4030_setvolume(OUTPUT_MONO_EARPIECE,
			READ_LEFT_VOLUME(twl4030_local.ear_vol), 0);
	if (ret)
		goto set_volumes_exit;

	ret = twl4030_setvolume(OUTPUT_CARKIT,
			READ_LEFT_VOLUME(twl4030_local.carkit_vol),
			READ_RIGHT_VOLUME(twl4030_local.carkit_vol));
	if (ret)
		goto set_volumes_exit;

	ret = twl4030_setvolume(OUTPUT_SIDETONE,
			READ_LEFT_VOLUME(twl4030_local.sidetone_vol), 0);
	if (ret)
		goto set_volumes_exit;

	ret = twl4030_setvolume(OUTPUT_DOWNLINK,
			READ_LEFT_VOLUME(twl4030_local.downlink_vol), 0);
set_volumes_exit:
	if (ret)
		printk(KERN_ERR "Failed setting all volumes %d\n", ret);

	return ret;
}

/*
 * twl4030_configure_codec_interface - Configure the codec's data path
 */
static int twl4030_configure_codec_interface(int codec_mode)
{
	int reg = 0;
	u8 data = 0;
	u8 data_width = 0;
	int ret = 0;

	/* Check sample width */
	if (twl4030_local.bitspersample ==
				AUDIO_SAMPLE_DATA_WIDTH_16) {
		/* Data width 16-bits */
		data_width = AUDIO_DATA_WIDTH_16SAMPLE_16DATA;
	} else if (twl4030_local.bitspersample ==
				AUDIO_SAMPLE_DATA_WIDTH_24) {
		/* Data width 24-bits */
		data_width = AUDIO_DATA_WIDTH_32SAMPLE_24DATA;
	} else {
		printk(KERN_ERR "Unknown sample width %d\n",
			twl4030_local.bitspersample);
		return -EPERM;
	}

	/* No need to set BIT_AUDIO_IF_CLK256FS_EN_M -not using it as CLKS!! */
	/* configure the audio IF of codec- Application Mode */
	if (codec_mode == AUDIO_MODE) {
		reg = REG_AUDIO_IF;
		data = data_width << BIT_AUDIO_IF_DATA_WIDTH
			| AUDIO_DATA_FORMAT_I2S << BIT_AUDIO_IF_AIF_FORMAT
			| BIT_AUDIO_IF_AIF_EN_M;
	} else if (codec_mode == VOICE_MODE) {
		reg = REG_VOICE_IF;
		data = BIT_VOICE_IF_VIF_SLAVE_EN_M
			| BIT_VOICE_IF_VIF_DIN_EN_M
			| BIT_VOICE_IF_VIF_DOUT_EN_M
			| BIT_VOICE_IF_VIF_SUB_EN_M
			| BIT_VOICE_IF_VIF_EN_M;
	} else {
		printk(KERN_ERR "Invalid codec mode\n");
		return -EPERM;
	}

	ret = audio_twl4030_write(reg, data);
	if (ret)
		printk(KERN_ERR "Failed configuring codec interface %d\n", ret);

	return ret;
}

/*
 * twl4030_default_samplerate - Set default audio sample rate
 */
static int twl4030_default_samplerate(void)
{
	return AUDIO_RATE_DEFAULT;
}

/*
 * twl4030_set_audio_samplerate - Set sample rate for audio mode
 */
int twl4030_set_audio_samplerate(long sample_rate)
{
	u8 data = 0;
	int count = 0;
	int ret = 0;

	for (count = 0; count < NUMBER_OF_RATES_SUPPORTED; count++) {
		if (audio_sample_rates[count].rate == sample_rate)
			break;
	}

	if (count >= NUMBER_OF_RATES_SUPPORTED) {
		printk(KERN_ERR "Invalid sample rate: %ld\n", sample_rate);
		return -EPERM;
	}

	ret = audio_twl4030_read(REG_CODEC_MODE, &data);
	if (ret) {
		printk(KERN_ERR "Failed to read codec mode %d\n", ret);
		return ret;
	}

	data &= ~(BIT_CODEC_MODE_APLL_RATE_M);
	data |= (audio_sample_rates[count].apll << BIT_CODEC_MODE_APLL_RATE);
	ret = audio_twl4030_write(REG_CODEC_MODE, data);
	if (ret) {
		printk(KERN_ERR "Failed to write codec mode %d\n", ret);
		return ret;
	}

	/* Set system master clock */
	ret = audio_twl4030_write(REG_APLL_CTL, BIT_APLL_CTL_APLL_EN_M |
				AUDIO_APLL_DEFAULT << BIT_APLL_CTL_APLL_INFREQ);
	if (ret) {
		printk(KERN_ERR "Unable to set APLL input frequency %d\n", ret);
		return ret;
	}

	twl4030_local.audio_samplerate = sample_rate;

	return ret;
}

/*
 * twl4030_set_voice_samplerate - Set sample rate for voice mode
 */
int twl4030_set_voice_samplerate(long sample_rate)
{
	u8 data = 0;
	int ret = 0;

	ret = audio_twl4030_read(REG_CODEC_MODE, &data);
	if (ret) {
		printk(KERN_ERR "Failed to read codec mode %d\n", ret);
		return ret;
	}

	if (sample_rate == NARROWBAND_FREQ) {
		data &= ~BIT_CODEC_MODE_SEL_16K_M;
	} else if (sample_rate == WIDEBAND_FREQ) {
		data |= BIT_CODEC_MODE_SEL_16K_M;
	} else  {
		printk(KERN_ERR "Invalid sample rate: %ld", sample_rate);
		return -EPERM;
	}

	ret = audio_twl4030_write(REG_CODEC_MODE, data);
	if (ret) {
		printk(KERN_ERR "Failed to write codec mode %d\n", ret);
		return ret;
	}

	/* Set system master clock */
	ret = audio_twl4030_write(REG_APLL_CTL, BIT_APLL_CTL_APLL_EN_M |
				AUDIO_APLL_DEFAULT << BIT_APLL_CTL_APLL_INFREQ);
	if (ret) {
		printk(KERN_ERR "Unable to set APLL input frequency %d\n", ret);
		return ret;
	}

	twl4030_local.voice_samplerate = sample_rate;

	return ret;
}

int twl4030_set_stereomode(int channels)
{
	int ret = 0;
	u8 dac = 0;

	ret = twl4030_ext_mut_on();
	if (ret)
		goto set_stereo_mode_exit;

	ret = twl4030_codec_tog_on();
	if (ret)
		goto set_stereo_mode_exit;

	/* Enabling DAC left/right based on channels*/
	ret = audio_twl4030_read(REG_AVDAC_CTL, &dac);
	if (ret)
		goto set_stereo_mode_exit;

	twl4030_local.channels = channels;
	if (twl4030_local.channels == MONO_MODE) {
		dac &= ~BIT_AVDAC_CTL_ADACR2_EN_M;
		dac |= BIT_AVDAC_CTL_ADACL2_EN_M;
	} else {
		dac |= (BIT_AVDAC_CTL_ADACL2_EN_M | BIT_AVDAC_CTL_ADACR2_EN_M);
	}
	ret = audio_twl4030_write(REG_AVDAC_CTL, dac);
	if (ret)
		goto set_stereo_mode_exit;

	/* Power off codec */
	ret = twl4030_codec_off();
	if (ret)
		goto set_stereo_mode_exit;

	ret = twl4030_conf_data_interface();
	if (ret)
		goto set_stereo_mode_exit;

	ret = omap2_mcbsp_dma_recv_params(AUDIO_MCBSP,
		&(twl4030_mcbsp_settings.audio_mcbsp_rx_transfer_params));
	if (ret < 0)
		goto set_stereo_mode_exit;

	ret = omap2_mcbsp_dma_trans_params(AUDIO_MCBSP,
		&(twl4030_mcbsp_settings.audio_mcbsp_tx_transfer_params));
	if (ret < 0)
		goto set_stereo_mode_exit;

	/* Set the mixing bit off if stereo, else set it to on */
#ifdef MONO_MODE_SOUNDS_STEREO
	ret = audio_twl4030_write(REG_RX_PATH_SEL,
				((mode == STEREO_MODE) ? 0x00 :
				 BIT_RX_PATH_SEL_RXL2_SEL_M |
				 BIT_RX_PATH_SEL_RXR2_SEL_M));
#else
	ret = audio_twl4030_write(REG_RX_PATH_SEL, 0x00);
#endif
	ret = twl4030_codec_on();
	if (ret)
		goto set_stereo_mode_exit;

	ret = twl4030_ext_mut_off();
set_stereo_mode_exit:
	if (ret)
		printk(KERN_ERR "Setting stereo mode failed %d\n", ret);

	return ret;
}


/*
 * twl4030_cleanup - Clean up the register settings
 */
static int twl4030_cleanup(void)
{
	int ret = 0;

	ret = audio_twl4030_write(REG_OPTION, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_ATXL1PGA, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_ATXR1PGA, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_AVTXL2PGA, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_AVTXR2PGA, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_ARXL1PGA, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_ARXR1PGA, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_ARXL2PGA, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_ARXR2PGA, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_AVDAC_CTL, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_ARXL1_APGA_CTL, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_ARXR1_APGA_CTL, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_ARXL2_APGA_CTL, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_ARXR2_APGA_CTL, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_ALC_CTL, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_BTPGA, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_DTMF_FREQSEL, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_DTMF_TONOFF, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_DTMF_PGA_CTL1, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_DTMF_PGA_CTL2, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_DTMF_WANONOFF, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_VRXPGA, 0x00);
	if (ret)
		goto cleanup_exit;

	ret = audio_twl4030_write(REG_VDL_APGA_CTL, 0x00);
cleanup_exit:
	if (ret)
		printk(KERN_ERR "Cleanup error %d\n", ret);

	return ret;
}

/*
 * twl4030_configure - Configure codec according to specified mode
 */
int twl4030_configure(int codec_mode)
{
	int ret = 0;

	ret = twl4030_cleanup();
	if (ret)
		goto configure_exit;

	ret = twl4030_set_codec_mode(codec_mode);
	if (ret)
		goto configure_exit;

	ret = twl4030_configure_codec_interface(codec_mode);
	if (ret)
		goto configure_exit;

	if (codec_mode == AUDIO_MODE)
		ret = twl4030_set_audio_samplerate(twl4030_local.
				audio_samplerate);
	else if (codec_mode == VOICE_MODE)
		ret = twl4030_set_voice_samplerate(twl4030_local.
				voice_samplerate);
	if (ret)
		goto configure_exit;

	/* Set the gains - we do not know the defaults
	 * attempt to set the volume of all the devices
	 * only those enabled get set.
	 */
	ret = twl4030_set_all_volumes();
	if (ret)
		goto configure_exit;

	/* Do not bypass the high pass filters */
	ret = audio_twl4030_write(REG_MISC_SET_2, 0x00);
	if (ret)
		goto configure_exit;

	/* Switch on the input/output required */
	ret = twl4030_select_source(twl4030_local.input_src);
	if (ret)
		goto configure_exit;

	ret = twl4030_select_source(twl4030_local.output_src);
configure_exit:
	if (ret)
		printk(KERN_ERR "Codec configuration failed %d\n", ret);

	return ret;
}

/*
 * twl4030_unconfigure - Unconfigure codec, turn off
 */
void twl4030_unconfigure(void)
{
	twl4030_disable_outputs();
	twl4030_disable_inputs();
	twl4030_ext_mut_unconf();
}

/*
 * McBSP callback
 */
void twl4030_mcbsp_dma_cb(u32 ch_status, void *arg)
{
	if (ch_status) {
		printk(KERN_ERR "Codec data transfer error %d\n", ch_status);

	}
	callback_omap_alsa_sound_dma(arg);
}

/*******************************************************************************
 *
 * Codec APIs
 *
 ******************************************************************************/


/*
 * codec_configure_device - Codec is configure to audio mode when streaming
 */
void codec_configure_device(void)
{
	twl4030_local.codec_mode = AUDIO_MODE;
	twl4030_ext_mut_conf();
	twl4030_configure(twl4030_local.codec_mode);
}

/*
 * codec_clock_setup - No special clock setup need, only to fulfill
 * requirements
 */
void codec_clock_setup(void)
{
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
		codec_cfg->codec_configure_dev = codec_configure_device;
		codec_cfg->codec_unconfigure_dev = twl4030_unconfigure;
		codec_cfg->codec_set_samplerate = twl4030_set_audio_samplerate;
		codec_cfg->codec_set_stereomode = twl4030_set_stereomode;
		codec_cfg->codec_clock_setup = codec_clock_setup;
		codec_cfg->codec_clock_on = twl4030_codec_on;
		codec_cfg->codec_clock_off = twl4030_codec_off;
		codec_cfg->get_default_samplerate = twl4030_default_samplerate;
		ret = snd_omap_alsa_post_probe(pdev, codec_cfg);
	} else {
		return -ENODEV;
	}

	/* Disconnect McBSP3 for modem connection in LDP */
	if (machine_is_omap_ldp()) {
		omap_cfg_reg(AF6_3430_MCBSP3_DX);
		omap_cfg_reg(AE6_3430_MCBSP3_DR);
		omap_cfg_reg(AF5_3430_MCBSP3_CLX);
		omap_cfg_reg(AE5_3430_MCBSP3_FSX);
		omap_cfg_reg(R27_3430_GPIO_128);
		modem_gpio_conf();
		/* Configure for voice mode by default */
		twl4030_local.codec_mode = VOICE_MODE;
		ret = twl4030_codec_on();
		if (ret) {
			printk(KERN_ERR "Failed turning codec on %d\n", ret);
			goto twl4030_probe_out;
		}
		ret = twl4030_configure(twl4030_local.codec_mode);
		if (ret) {
			printk(KERN_ERR "Failed to initialize codec %d\n", ret);
			goto twl4030_probe_out;
		}
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
 * TWL_DUMP_REGISTERS:
 * This will dump the entire register set of Page 2 twl4030.
 ******************************************************************************/
#ifdef TWL_DUMP_REGISTERS
/*
 * twl4030_dump registers
 */
void twl4030_dump_registers(void)
{
	int i = 0;
	int ret = 0;
	u8 data = 0;

	printk(KERN_INFO "TWL4030 Register dump for Audio Module\n");
	for (i = REG_CODEC_MODE; i <= REG_MISC_SET_2; i++) {
		ret = audio_twl4030_read(i, &data);
		if (ret)
			printk(KERN_ERR "Failed reading reg %02x\n", i);

		printk(KERN_INFO "Register[0x%02x] = 0x%02x\n", i, data);

	}
}
#endif	/* End of #ifdef TWL_DUMP_REGISTERS */

module_init(twl4030_init);
module_exit(twl4030_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Codec audio driver for the TI TWL4030 codec.");
MODULE_LICENSE("GPL");
