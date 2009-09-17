/*
 * omap3517evm.c  -- ALSA SoC support for OMAP3517 EVM
 *
 * Author: Anuj Aggarwal <anuj.aggarwal@ti.com>
 *
 * Based on sound/soc/omap/beagle.c by Steve Sakoman
 *
 * Copyright (C) 2008 Texas Instruments, Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

#include "../codecs/tlv320aic23.h"

#define CODEC_CLOCK 	12000000

static int omap3517evm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_DSP_B |
				  SND_SOC_DAIFMT_NB_IF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_DSP_B |
				  SND_SOC_DAIFMT_NB_IF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
			CODEC_CLOCK, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_CLKR_SRC_CLKX, 0,
				SND_SOC_CLOCK_IN);
	snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_FSR_SRC_FSX, 0,
				SND_SOC_CLOCK_IN);

	return 0;
}

static struct snd_soc_ops omap3517evm_ops = {
	.hw_params = omap3517evm_hw_params,
};

/* omap3517evm machine dapm widgets */
static const struct snd_soc_dapm_widget tlv320aic23_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Line Out", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_MIC("Mic In", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "LOUT"},
	{"Line Out", NULL, "ROUT"},

	{"LLINEIN", NULL, "Line In"},
	{"RLINEIN", NULL, "Line In"},

	{"MICIN", NULL, "Mic In"},
};

static int omap3517evm_aic23_init(struct snd_soc_codec *codec)
{
	/* Add omap3517-evm specific widgets */
	snd_soc_dapm_new_controls(codec, tlv320aic23_dapm_widgets,
				  ARRAY_SIZE(tlv320aic23_dapm_widgets));

	/* Set up davinci-evm specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* always connected */
	snd_soc_dapm_enable_pin(codec, "Line Out");
	snd_soc_dapm_enable_pin(codec, "Line In");
	snd_soc_dapm_enable_pin(codec, "Mic In");

	snd_soc_dapm_sync(codec);

	return 0;
}

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link omap3517evm_dai = {
	.name = "TLV320AIC23",
	.stream_name = "AIC23",
	.cpu_dai = &omap_mcbsp_dai[0],
	.codec_dai = &tlv320aic23_dai,
	.init = omap3517evm_aic23_init,
	.ops = &omap3517evm_ops,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_omap3517evm = {
	.name = "omap3517evm",
	.platform = &omap_soc_platform,
	.dai_link = &omap3517evm_dai,
	.num_links = 1,
};

/* Audio subsystem */
static struct snd_soc_device omap3517evm_snd_devdata = {
	.card = &snd_soc_omap3517evm,
	.codec_dev = &soc_codec_dev_tlv320aic23,
};

static struct platform_device *omap3517evm_snd_device;

static int __init omap3517evm_soc_init(void)
{
	int ret;

	if (!machine_is_omap3517evm()) {
		pr_err("Not OMAP3517 EVM!\n");
		return -ENODEV;
	}
	pr_info("OMAP3 EVM SoC init\n");

	omap3517evm_snd_device = platform_device_alloc("soc-audio", -1);
	if (!omap3517evm_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(omap3517evm_snd_device, &omap3517evm_snd_devdata);
	omap3517evm_snd_devdata.dev = &omap3517evm_snd_device->dev;
	*(unsigned int *)omap3517evm_dai.cpu_dai->private_data = 0; /* McBSP1 */

	ret = platform_device_add(omap3517evm_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(omap3517evm_snd_device);

	return ret;
}

static void __exit omap3517evm_soc_exit(void)
{
	platform_device_unregister(omap3517evm_snd_device);
}

module_init(omap3517evm_soc_init);
module_exit(omap3517evm_soc_exit);

MODULE_AUTHOR("Anuj Aggarwal <anuj.aggarwal@ti.com>");
MODULE_DESCRIPTION("ALSA SoC OMAP3517 EVM");
MODULE_LICENSE("GPL v2");
