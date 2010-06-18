/*
 * linux/arch/arm/mach-omap2/devices.c
 *
 * OMAP2 platform device setup/initialization
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/ahci_platform.h>
#include <linux/davinci_emac.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>
#include <asm/hardware/edma.h>

#include <plat/control.h>
#include <plat/tc.h>
#include <plat/board.h>
#include <plat/mux.h>
#include <mach/gpio.h>
#include <plat/mmc.h>
#include <plat/asp.h>

#include "mux.h"
#include "pcie-ti816x.h"

#define TI816X_RTC_BASE           0x480C0000

#if defined(CONFIG_RTC_DRV_OMAP) || defined(CONFIG_RTC_DRV_OMAP_MODULE)

static struct resource rtc_resources[] = {
        {
                .start          = TI816X_RTC_BASE,
                .end            = TI816X_RTC_BASE + 0x5f,
                .flags          = IORESOURCE_MEM,
        },
        {
                .start          = TI816X_IRQ_RTC,
                .flags          = IORESOURCE_IRQ,
        },
        {
                .start          = TI816X_IRQ_RTC_ALARM,
                .flags          = IORESOURCE_IRQ,
        },
};

static struct platform_device ti816x_rtc_device = {
        .name           = "omap_rtc",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(rtc_resources),
        .resource       = rtc_resources,
};

static int ti816x_init_rtc(void)
{
        return(platform_device_register(&ti816x_rtc_device));
}
#else
static inline void ti816x_init_rtc(void) {}
#endif


#if defined(CONFIG_VIDEO_OMAP2) || defined(CONFIG_VIDEO_OMAP2_MODULE)

static struct resource cam_resources[] = {
	{
		.start		= OMAP24XX_CAMERA_BASE,
		.end		= OMAP24XX_CAMERA_BASE + 0xfff,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_CAM_IRQ,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct platform_device omap_cam_device = {
	.name		= "omap24xxcam",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(cam_resources),
	.resource	= cam_resources,
};

static inline void omap_init_camera(void)
{
	platform_device_register(&omap_cam_device);
}

#elif defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)

static struct resource omap3isp_resources[] = {
	{
		.start		= OMAP3430_ISP_BASE,
		.end		= OMAP3430_ISP_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CBUFF_BASE,
		.end		= OMAP3430_ISP_CBUFF_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CCP2_BASE,
		.end		= OMAP3430_ISP_CCP2_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CCDC_BASE,
		.end		= OMAP3430_ISP_CCDC_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_HIST_BASE,
		.end		= OMAP3430_ISP_HIST_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_H3A_BASE,
		.end		= OMAP3430_ISP_H3A_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_PREV_BASE,
		.end		= OMAP3430_ISP_PREV_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_RESZ_BASE,
		.end		= OMAP3430_ISP_RESZ_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_SBL_BASE,
		.end		= OMAP3430_ISP_SBL_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CSI2A_BASE,
		.end		= OMAP3430_ISP_CSI2A_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CSI2PHY_BASE,
		.end		= OMAP3430_ISP_CSI2PHY_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_34XX_CAM_IRQ,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct platform_device omap3isp_device = {
	.name		= "omap3isp",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap3isp_resources),
	.resource	= omap3isp_resources,
};

static inline void omap_init_camera(void)
{
	platform_device_register(&omap3isp_device);
}
#else
static inline void omap_init_camera(void)
{
}
#endif

#if defined(CONFIG_OMAP_MBOX_FWK) || defined(CONFIG_OMAP_MBOX_FWK_MODULE)

#define MBOX_REG_SIZE   0x120

#ifdef CONFIG_ARCH_OMAP2
static struct resource omap2_mbox_resources[] = {
	{
		.start		= OMAP24XX_MAILBOX_BASE,
		.end		= OMAP24XX_MAILBOX_BASE + MBOX_REG_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_MAIL_U0_MPU,
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start		= INT_24XX_MAIL_U3_MPU,
		.flags		= IORESOURCE_IRQ,
	},
};
static int omap2_mbox_resources_sz = ARRAY_SIZE(omap2_mbox_resources);
#else
#define omap2_mbox_resources		NULL
#define omap2_mbox_resources_sz		0
#endif

#ifdef CONFIG_ARCH_OMAP3
static struct resource omap3_mbox_resources[] = {
	{
		.start		= OMAP34XX_MAILBOX_BASE,
		.end		= OMAP34XX_MAILBOX_BASE + MBOX_REG_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_MAIL_U0_MPU,
		.flags		= IORESOURCE_IRQ,
	},
};
static int omap3_mbox_resources_sz = ARRAY_SIZE(omap3_mbox_resources);
#else
#define omap3_mbox_resources		NULL
#define omap3_mbox_resources_sz		0
#endif

#ifdef CONFIG_ARCH_OMAP4

#define OMAP4_MBOX_REG_SIZE	0x130
static struct resource omap4_mbox_resources[] = {
	{
		.start          = OMAP44XX_MAILBOX_BASE,
		.end            = OMAP44XX_MAILBOX_BASE +
					OMAP4_MBOX_REG_SIZE - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = OMAP44XX_IRQ_MAIL_U0,
		.flags          = IORESOURCE_IRQ,
	},
};
static int omap4_mbox_resources_sz = ARRAY_SIZE(omap4_mbox_resources);
#else
#define omap4_mbox_resources		NULL
#define omap4_mbox_resources_sz		0
#endif

static struct platform_device mbox_device = {
	.name		= "omap2-mailbox",
	.id		= -1,
};

static inline void omap_init_mbox(void)
{
	if (cpu_is_omap24xx()) {
		mbox_device.resource = omap2_mbox_resources;
		mbox_device.num_resources = omap2_mbox_resources_sz;
	} else if (cpu_is_omap34xx()) {
		mbox_device.resource = omap3_mbox_resources;
		mbox_device.num_resources = omap3_mbox_resources_sz;
	} else if (cpu_is_omap44xx()) {
		mbox_device.resource = omap4_mbox_resources;
		mbox_device.num_resources = omap4_mbox_resources_sz;
	} else {
		pr_err("%s: platform not supported\n", __func__);
		return;
	}
	platform_device_register(&mbox_device);
}
#else
static inline void omap_init_mbox(void) { }
#endif /* CONFIG_OMAP_MBOX_FWK */

#if defined(CONFIG_OMAP_STI)

#if defined(CONFIG_ARCH_OMAP2)

#define OMAP2_STI_BASE		0x48068000
#define OMAP2_STI_CHANNEL_BASE	0x54000000
#define OMAP2_STI_IRQ		4

static struct resource sti_resources[] = {
	{
		.start		= OMAP2_STI_BASE,
		.end		= OMAP2_STI_BASE + 0x7ff,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP2_STI_CHANNEL_BASE,
		.end		= OMAP2_STI_CHANNEL_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP2_STI_IRQ,
		.flags		= IORESOURCE_IRQ,
	}
};
#elif defined(CONFIG_ARCH_OMAP3)

#define OMAP3_SDTI_BASE		0x54500000
#define OMAP3_SDTI_CHANNEL_BASE	0x54600000

static struct resource sti_resources[] = {
	{
		.start		= OMAP3_SDTI_BASE,
		.end		= OMAP3_SDTI_BASE + 0xFFF,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3_SDTI_CHANNEL_BASE,
		.end		= OMAP3_SDTI_CHANNEL_BASE + SZ_1M - 1,
		.flags		= IORESOURCE_MEM,
	}
};

#endif

static struct platform_device sti_device = {
	.name		= "sti",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sti_resources),
	.resource	= sti_resources,
};

static inline void omap_init_sti(void)
{
	platform_device_register(&sti_device);
}
#else
static inline void omap_init_sti(void) {}
#endif

#if defined(CONFIG_SPI_OMAP24XX) || defined(CONFIG_SPI_OMAP24XX_MODULE)

#include <plat/mcspi.h>

#define OMAP2_MCSPI1_BASE		0x48098000
#define OMAP2_MCSPI2_BASE		0x4809a000
#define OMAP2_MCSPI3_BASE		0x480b8000
#define OMAP2_MCSPI4_BASE		0x480ba000

#define OMAP4_MCSPI1_BASE		0x48098100
#define OMAP4_MCSPI2_BASE		0x4809a100
#define OMAP4_MCSPI3_BASE		0x480b8100
#define OMAP4_MCSPI4_BASE		0x480ba100

#define TI816X_MCSPI1_BASE		0x48031100

static struct omap2_mcspi_platform_config omap2_mcspi1_config = {
	.num_cs		= 4,
};

#ifdef CONFIG_ARCH_TI816X
static struct resource omap2_mcspi1_resources[] = {
	{
		.start		= TI816X_MCSPI1_BASE,
		.end		= TI816X_MCSPI1_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},
};
#else
static struct resource omap2_mcspi1_resources[] = {
	{
		.start		= OMAP2_MCSPI1_BASE,
		.end		= OMAP2_MCSPI1_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},
};
#endif

static struct platform_device omap2_mcspi1 = {
	.name		= "omap2_mcspi",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(omap2_mcspi1_resources),
	.resource	= omap2_mcspi1_resources,
	.dev		= {
		.platform_data = &omap2_mcspi1_config,
	},
};

#ifndef CONFIG_ARCH_TI816X
static struct omap2_mcspi_platform_config omap2_mcspi2_config = {
	.num_cs		= 2,
};

static struct resource omap2_mcspi2_resources[] = {
	{
		.start		= OMAP2_MCSPI2_BASE,
		.end		= OMAP2_MCSPI2_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device omap2_mcspi2 = {
	.name		= "omap2_mcspi",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(omap2_mcspi2_resources),
	.resource	= omap2_mcspi2_resources,
	.dev		= {
		.platform_data = &omap2_mcspi2_config,
	},
};
#endif


#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3) || \
	defined(CONFIG_ARCH_OMAP4)
static struct omap2_mcspi_platform_config omap2_mcspi3_config = {
	.num_cs		= 2,
};

static struct resource omap2_mcspi3_resources[] = {
	{
	.start		= OMAP2_MCSPI3_BASE,
	.end		= OMAP2_MCSPI3_BASE + 0xff,
	.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device omap2_mcspi3 = {
	.name		= "omap2_mcspi",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(omap2_mcspi3_resources),
	.resource	= omap2_mcspi3_resources,
	.dev		= {
		.platform_data = &omap2_mcspi3_config,
	},
};
#endif

#if defined(CONFIG_ARCH_OMAP3) || defined(CONFIG_ARCH_OMAP4)
static struct omap2_mcspi_platform_config omap2_mcspi4_config = {
	.num_cs		= 1,
};

static struct resource omap2_mcspi4_resources[] = {
	{
		.start		= OMAP2_MCSPI4_BASE,
		.end		= OMAP2_MCSPI4_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device omap2_mcspi4 = {
	.name		= "omap2_mcspi",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(omap2_mcspi4_resources),
	.resource	= omap2_mcspi4_resources,
	.dev		= {
		.platform_data = &omap2_mcspi4_config,
	},
};
#endif

#ifdef CONFIG_ARCH_TI816X
static inline void ti816x_mcspi_fixup(void)
{
	omap2_mcspi1_resources[0].start	= TI816X_MCSPI1_BASE;
	omap2_mcspi1_resources[0].end	= TI816X_MCSPI1_BASE + 0xff;
}
#else
static inline void ti816x_mcspi_fixup(void)
{
}
#endif



#ifdef CONFIG_ARCH_OMAP4
static inline void omap4_mcspi_fixup(void)
{
	omap2_mcspi1_resources[0].start	= OMAP4_MCSPI1_BASE;
	omap2_mcspi1_resources[0].end	= OMAP4_MCSPI1_BASE + 0xff;
	omap2_mcspi2_resources[0].start	= OMAP4_MCSPI2_BASE;
	omap2_mcspi2_resources[0].end	= OMAP4_MCSPI2_BASE + 0xff;
	omap2_mcspi3_resources[0].start	= OMAP4_MCSPI3_BASE;
	omap2_mcspi3_resources[0].end	= OMAP4_MCSPI3_BASE + 0xff;
	omap2_mcspi4_resources[0].start	= OMAP4_MCSPI4_BASE;
	omap2_mcspi4_resources[0].end	= OMAP4_MCSPI4_BASE + 0xff;
}
#else
static inline void omap4_mcspi_fixup(void)
{
}
#endif

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3) || \
	defined(CONFIG_ARCH_OMAP4)
static inline void omap2_mcspi3_init(void)
{
	platform_device_register(&omap2_mcspi3);
}
#else
static inline void omap2_mcspi3_init(void)
{
}
#endif

#if defined(CONFIG_ARCH_OMAP3) || defined(CONFIG_ARCH_OMAP4)
static inline void omap2_mcspi4_init(void)
{
	platform_device_register(&omap2_mcspi4);
}
#else
static inline void omap2_mcspi4_init(void)
{
}
#endif

static void omap_init_mcspi(void)
{
	if (cpu_is_omap44xx())
		omap4_mcspi_fixup();

	if (cpu_is_ti816x())
		ti816x_mcspi_fixup();

	platform_device_register(&omap2_mcspi1);

#if !defined(CONFIG_ARCH_TI816X)
	platform_device_register(&omap2_mcspi2);
#endif

	if (cpu_is_omap2430() || cpu_is_omap343x() || cpu_is_omap44xx())
		omap2_mcspi3_init();

	if (cpu_is_omap343x() || cpu_is_omap44xx())
		omap2_mcspi4_init();
}

#else
static inline void omap_init_mcspi(void) {}
#endif

#ifdef CONFIG_OMAP_SHA1_MD5
static struct resource sha1_md5_resources[] = {
	{
		.start	= OMAP24XX_SEC_SHA1MD5_BASE,
		.end	= OMAP24XX_SEC_SHA1MD5_BASE + 0x64,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_24XX_SHA1MD5,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device sha1_md5_device = {
	.name		= "OMAP SHA1/MD5",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sha1_md5_resources),
	.resource	= sha1_md5_resources,
};

static void omap_init_sha1_md5(void)
{
	platform_device_register(&sha1_md5_device);
}
#else
static inline void omap_init_sha1_md5(void) { }
#endif

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_ARCH_OMAP3) || defined(CONFIG_ARCH_OMAP4)

#define MMCHS_SYSCONFIG			0x0010
#define MMCHS_SYSCONFIG_SWRESET		(1 << 1)
#define MMCHS_SYSSTATUS			0x0014
#define MMCHS_SYSSTATUS_RESETDONE	(1 << 0)

static struct platform_device dummy_pdev = {
	.dev = {
		.bus = &platform_bus_type,
	},
};

/**
 * omap_hsmmc_reset() - Full reset of each HS-MMC controller
 *
 * Ensure that each MMC controller is fully reset.  Controllers
 * left in an unknown state (by bootloader) may prevent retention
 * or OFF-mode.  This is especially important in cases where the
 * MMC driver is not enabled, _or_ built as a module.
 *
 * In order for reset to work, interface, functional and debounce
 * clocks must be enabled.  The debounce clock comes from func_32k_clk
 * and is not under SW control, so we only enable i- and f-clocks.
 **/
static void __init omap_hsmmc_reset(void)
{
	u32 i, nr_controllers;

	if (cpu_is_omap242x())
		return;

	nr_controllers = cpu_is_omap44xx() ? OMAP44XX_NR_MMC :
		(cpu_is_omap34xx() ? OMAP34XX_NR_MMC : OMAP24XX_NR_MMC);

	for (i = 0; i < nr_controllers; i++) {
		u32 v, base = 0;
		struct clk *iclk, *fclk;
		struct device *dev = &dummy_pdev.dev;

		switch (i) {
		case 0:
			base = OMAP2_MMC1_BASE;
			break;
		case 1:
			base = OMAP2_MMC2_BASE;
			break;
		case 2:
			base = OMAP3_MMC3_BASE;
			break;
		case 3:
			if (!cpu_is_omap44xx())
				return;
			base = OMAP4_MMC4_BASE;
			break;
		case 4:
			if (!cpu_is_omap44xx())
				return;
			base = OMAP4_MMC5_BASE;
			break;
		}

		if (cpu_is_omap44xx())
			base += OMAP4_MMC_REG_OFFSET;

		dummy_pdev.id = i;
		dev_set_name(&dummy_pdev.dev, "mmci-omap-hs.%d", i);
		iclk = clk_get(dev, "ick");
		if (iclk && clk_enable(iclk))
			iclk = NULL;

		fclk = clk_get(dev, "fck");
		if (fclk && clk_enable(fclk))
			fclk = NULL;

		if (!iclk || !fclk) {
			printk(KERN_WARNING
			       "%s: Unable to enable clocks for MMC%d, "
			       "cannot reset.\n",  __func__, i);
			break;
		}

		omap_writel(MMCHS_SYSCONFIG_SWRESET, base + MMCHS_SYSCONFIG);
		v = omap_readl(base + MMCHS_SYSSTATUS);
		while (!(omap_readl(base + MMCHS_SYSSTATUS) &
			 MMCHS_SYSSTATUS_RESETDONE))
			cpu_relax();

		if (fclk) {
			clk_disable(fclk);
			clk_put(fclk);
		}
		if (iclk) {
			clk_disable(iclk);
			clk_put(iclk);
		}
	}
}
#else
static inline void omap_hsmmc_reset(void) {}
#endif

#if defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE) || \
	defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)

static inline void omap2_mmc_mux(struct omap_mmc_platform_data *mmc_controller,
			int controller_nr)
{
	if ((mmc_controller->slots[0].switch_pin > 0) && \
		(mmc_controller->slots[0].switch_pin < OMAP_MAX_GPIO_LINES))
		omap_mux_init_gpio(mmc_controller->slots[0].switch_pin,
					OMAP_PIN_INPUT_PULLUP);
	if ((mmc_controller->slots[0].gpio_wp > 0) && \
		(mmc_controller->slots[0].gpio_wp < OMAP_MAX_GPIO_LINES))
		omap_mux_init_gpio(mmc_controller->slots[0].gpio_wp,
					OMAP_PIN_INPUT_PULLUP);

	if (cpu_is_omap2420() && controller_nr == 0) {
		omap_cfg_reg(H18_24XX_MMC_CMD);
		omap_cfg_reg(H15_24XX_MMC_CLKI);
		omap_cfg_reg(G19_24XX_MMC_CLKO);
		omap_cfg_reg(F20_24XX_MMC_DAT0);
		omap_cfg_reg(F19_24XX_MMC_DAT_DIR0);
		omap_cfg_reg(G18_24XX_MMC_CMD_DIR);
		if (mmc_controller->slots[0].wires == 4) {
			omap_cfg_reg(H14_24XX_MMC_DAT1);
			omap_cfg_reg(E19_24XX_MMC_DAT2);
			omap_cfg_reg(D19_24XX_MMC_DAT3);
			omap_cfg_reg(E20_24XX_MMC_DAT_DIR1);
			omap_cfg_reg(F18_24XX_MMC_DAT_DIR2);
			omap_cfg_reg(E18_24XX_MMC_DAT_DIR3);
		}

		/*
		 * Use internal loop-back in MMC/SDIO Module Input Clock
		 * selection
		 */
		if (mmc_controller->slots[0].internal_clock) {
			u32 v = omap_ctrl_readl(OMAP2_CONTROL_DEVCONF0);
			v |= (1 << 24);
			omap_ctrl_writel(v, OMAP2_CONTROL_DEVCONF0);
		}
	}

	if (cpu_is_omap34xx()) {
		if (controller_nr == 0) {
			omap_mux_init_signal("sdmmc1_clk",
				OMAP_PIN_INPUT_PULLUP);
			omap_mux_init_signal("sdmmc1_cmd",
				OMAP_PIN_INPUT_PULLUP);
			omap_mux_init_signal("sdmmc1_dat0",
				OMAP_PIN_INPUT_PULLUP);
			if (mmc_controller->slots[0].wires == 4 ||
				mmc_controller->slots[0].wires == 8) {
				omap_mux_init_signal("sdmmc1_dat1",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat2",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat3",
					OMAP_PIN_INPUT_PULLUP);
			}
			if (mmc_controller->slots[0].wires == 8) {
				omap_mux_init_signal("sdmmc1_dat4",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat5",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat6",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat7",
					OMAP_PIN_INPUT_PULLUP);
			}
		}
		if (controller_nr == 1) {
			/* MMC2 */
			omap_mux_init_signal("sdmmc2_clk",
				OMAP_PIN_INPUT_PULLUP);
			omap_mux_init_signal("sdmmc2_cmd",
				OMAP_PIN_INPUT_PULLUP);
			omap_mux_init_signal("sdmmc2_dat0",
				OMAP_PIN_INPUT_PULLUP);

			/*
			 * For 8 wire configurations, Lines DAT4, 5, 6 and 7 need to be muxed
			 * in the board-*.c files
			 */
			if (mmc_controller->slots[0].wires == 4 ||
				mmc_controller->slots[0].wires == 8) {
				omap_mux_init_signal("sdmmc2_dat1",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat2",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat3",
					OMAP_PIN_INPUT_PULLUP);
			}
			if (mmc_controller->slots[0].wires == 8) {
				omap_mux_init_signal("sdmmc2_dat4.sdmmc2_dat4",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat5.sdmmc2_dat5",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat6.sdmmc2_dat6",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat7.sdmmc2_dat7",
					OMAP_PIN_INPUT_PULLUP);
			}
		}

		/*
		 * For MMC3 the pins need to be muxed in the board-*.c files
		 */
	}
}

void __init omap2_init_mmc(struct omap_mmc_platform_data **mmc_data,
			int nr_controllers)
{
	int i;
	char *name;

	for (i = 0; i < nr_controllers; i++) {
		unsigned long base, size;
		unsigned int irq = 0;

		if (!mmc_data[i])
			continue;

		omap2_mmc_mux(mmc_data[i], i);

		switch (i) {
		case 0:
			if (!cpu_is_ti816x()) {
				base = OMAP2_MMC1_BASE;
				irq = INT_24XX_MMC_IRQ;
			} else {
				base = TI816X_MMC1_BASE;
				irq = TI816X_IRQ_SD;
			}
			break;
		case 1:
			base = OMAP2_MMC2_BASE;
			irq = INT_24XX_MMC2_IRQ;
			break;
		case 2:
			if (!cpu_is_omap44xx() && !cpu_is_omap34xx())
				return;
			base = OMAP3_MMC3_BASE;
			irq = INT_34XX_MMC3_IRQ;
			break;
		case 3:
			if (!cpu_is_omap44xx())
				return;
			base = OMAP4_MMC4_BASE + OMAP4_MMC_REG_OFFSET;
			irq = OMAP44XX_IRQ_MMC4;
			break;
		case 4:
			if (!cpu_is_omap44xx())
				return;
			base = OMAP4_MMC5_BASE + OMAP4_MMC_REG_OFFSET;
			irq = OMAP44XX_IRQ_MMC5;
			break;
		default:
			continue;
		}

		if (cpu_is_omap2420()) {
			size = OMAP2420_MMC_SIZE;
			name = "mmci-omap";
		} else if (cpu_is_omap44xx()) {
			if (i < 3) {
				base += OMAP4_MMC_REG_OFFSET;
				irq += OMAP44XX_IRQ_GIC_START;
			}
			size = OMAP4_HSMMC_SIZE;
			name = "mmci-omap-hs";
		} else if (cpu_is_ti816x()) {
			size = TI816X_HSMMC_SIZE;
			name = "mmci-omap-hs";
		} else {
			size = OMAP3_HSMMC_SIZE;
			name = "mmci-omap-hs";
		}
		omap_mmc_add(name, i, base, size, irq, mmc_data[i]);
	};
}

#endif

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_HDQ_MASTER_OMAP) || defined(CONFIG_HDQ_MASTER_OMAP_MODULE)
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
#define OMAP_HDQ_BASE	0x480B2000
#endif
static struct resource omap_hdq_resources[] = {
	{
		.start		= OMAP_HDQ_BASE,
		.end		= OMAP_HDQ_BASE + 0x1C,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_HDQ_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};
static struct platform_device omap_hdq_dev = {
	.name = "omap_hdq",
	.id = 0,
	.dev = {
		.platform_data = NULL,
	},
	.num_resources	= ARRAY_SIZE(omap_hdq_resources),
	.resource	= omap_hdq_resources,
};
static inline void omap_hdq_init(void)
{
	(void) platform_device_register(&omap_hdq_dev);
}
#else
static inline void omap_hdq_init(void) {}
#endif

/*---------------------------------------------------------------------------*/

#if defined(CONFIG_VIDEO_OMAP2_VOUT) || \
	defined(CONFIG_VIDEO_OMAP2_VOUT_MODULE)
#if defined(CONFIG_FB_OMAP2) || defined(CONFIG_FB_OMAP2_MODULE)
static struct resource omap_vout_resource[3 - CONFIG_FB_OMAP2_NUM_FBS] = {
};
#else
static struct resource omap_vout_resource[2] = {
};
#endif

static struct platform_device omap_vout_device = {
	.name		= "omap_vout",
	.num_resources	= ARRAY_SIZE(omap_vout_resource),
	.resource 	= &omap_vout_resource[0],
	.id		= -1,
};
static void omap_init_vout(void)
{
	if (platform_device_register(&omap_vout_device) < 0)
		printk(KERN_ERR "Unable to register OMAP-VOUT device\n");
}
#else
static inline void omap_init_vout(void) {}
#endif

#define P0PHYCR		0x178  /* SATA PHY0 Control Register offset
                                * from AHCI base)
                                */
#define P1PHYCR		0x1F8  /* SATA PHY0 Control Register offset
                                * from AHCI base)
                                */

#define PHY_ENPLL	1 /* bit0        1 */
#define PHY_MPY		8 /* bits4:1     4 Clock Sources at 100MHz */
#define PHY_LB		0 /* bits6:5     2 */
#define PHY_CLKBYP	0 /* bits8:7     2 */
#define PHY_RXINVPAIR	0 /* bit9        1 */
#define PHY_LBK		0 /* bits11:10   2 */
#define PHY_RXLOS	1 /* bit12	 1 */
#define PHY_RXCDR	4 /* bits15:13   3 */
#define PHY_RXEQ	1 /* bits19:16   4 */
#define PHY_RxENOC	1 /* bit20       1 */
#define PHY_TXINVPAIR	0 /* bit21	 1 */
#define PHY_TXCM	0 /* bit22       1 */
#define PHY_TXSWING	3 /* bits26:23   4 */
#define PHY_TXDE	0 /* bits31:27   5 */
u8	ti_num_ahci_inst = 1;
struct clk *sata_clk;

static int ahci_plat_init(struct device *dev)
{
	u32     	phy_val = 0;
	void __iomem	*base;

	sata_clk = clk_get(NULL, "sata_ick");
	if (IS_ERR(sata_clk)) {
		pr_err("ahci : Failed to get AHCI clock\n");
		return -1;
	}

	if (clk_enable(sata_clk)) {
		pr_err("ahci : Clock Enable Failed\n");
		return -1;
	}

	phy_val = PHY_ENPLL << 0 | PHY_MPY << 1 | PHY_LB << 5 |
			PHY_CLKBYP << 7 | PHY_RXINVPAIR << 9 |
			PHY_LBK  << 10 | PHY_RXLOS << 12 |
			PHY_RXCDR << 13 | PHY_RXEQ << 16 |
			PHY_RxENOC << 20 | PHY_TXINVPAIR << 21 |
			PHY_TXCM << 22 | PHY_TXSWING  << 23 | PHY_TXDE << 27;

	base = ioremap(TI816X_SATA_BASE, 0x10ffff);
	if (!base) {
		printk(KERN_WARNING
				"%s: Unable to map SATA, "
				"cannot turn on PHY.\n",  __func__);
		return -1;
	}

	/* Initialize the SATA PHY */
	writel(phy_val,	base + P0PHYCR);

	/* ti816x platform has 2 SATA PHY's Initialize the second instance */
	if (cpu_is_ti816x() && (ti_num_ahci_inst > 1))
		writel(phy_val, base + P1PHYCR);

	iounmap(base);

	return 0;
}

static void ahci_plat_exit(struct device *dev)
{
}

static struct resource ahci_resources[] = {
	{
		.start	=	TI816X_SATA_BASE,
		.end	=	TI816X_SATA_BASE + 0x10fff,
		.flags	=	IORESOURCE_MEM,
	},
	{
		.start	=	TI816X_IRQ_SATA,
		.flags	=	IORESOURCE_IRQ,
	}
};

struct ahci_platform_data ahci_pdata = {
		.init 	=	ahci_plat_init,
		.exit	=	ahci_plat_exit,
	};

static struct platform_device ti_ahci_device = {
	.name	=	"ahci",
	.id	=	-1,
	.dev	=	{
				.platform_data = &ahci_pdata,
				.coherent_dma_mask = 0xffffffff,
			},
	.num_resources = ARRAY_SIZE(ahci_resources),
	.resource	= ahci_resources,
};

int __init ti_ahci_register(u8 num_inst)
{
	num_inst = num_inst ? num_inst : 1;
	ti_num_ahci_inst = num_inst;
	ahci_pdata.force_port_map = (1 << num_inst) - 1;
	ahci_pdata.mask_port_map = 0;
	return platform_device_register(& ti_ahci_device);
}

#if defined(CONFIG_ARCH_TI816X)

static struct resource ti816x_mcasp_resource[] = {
	{
		.name = "mcasp",
		.start = TI816X_ASP2_BASE,
		.end = TI816X_ASP2_BASE + (SZ_1K * 12) - 1,
		.flags = IORESOURCE_MEM,
	},
	/* TX event */
	{
		.start = TI816X_ASP2_RX_INT,
		.end = TI816X_ASP2_RX_INT,
		.flags = IORESOURCE_DMA,
	},
	/* RX event */
	{
		.start = TI816X_ASP2_TX_INT,
		.end = TI816X_ASP2_TX_INT,
		.flags = IORESOURCE_DMA,
	},
};

static struct platform_device ti816x_mcasp_device = {
	.name = "davinci-mcasp",
	.id = 2,
	.num_resources = ARRAY_SIZE(ti816x_mcasp_resource),
	.resource = ti816x_mcasp_resource,
};

void __init ti816x_register_mcasp(int id, struct snd_platform_data *pdata)
{
	ti816x_mcasp_device.dev.platform_data = pdata;
	platform_device_register(&ti816x_mcasp_device);
}
#endif

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_ARCH_TI816X) && defined(CONFIG_PCI)
static struct ti816x_pcie_data ti816x_pcie_data = {
	        .msi_irq_base	= TI816X_MSI_IRQ_BASE,
		.msi_irq_num	= TI816X_MSI_NR_IRQS,
};

static struct resource ti816x_pcie_resources[] = {
	{
		/* Register space */
		.name		= "regs",
		.start		= TI816X_PCIE_REG_BASE,
		.end		= TI816X_PCIE_REG_BASE + SZ_16M - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		/* Non-prefetch memory */
		.name		= "nonprefetch",
		.start		= TI816X_PCIE_MEM_BASE,
		.end		= TI816X_PCIE_MEM_BASE + SZ_256M - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		/* IO window */
		.name		= "io",
		.start		= TI816X_PCIE_IO_BASE,
		.end		= TI816X_PCIE_IO_BASE + SZ_2M + SZ_1M - 1,
		.flags		= IORESOURCE_IO,
	},
	{
		/* Inbound memory window */
		.name		= "inbound0",
		.start		= PHYS_OFFSET,
		.end		= PHYS_OFFSET + SZ_256M - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		/* Legacy Interrupt */
		.name		= "legacy_int",
		.start		= TI816X_IRQ_PCIINT0,
		.end		= TI816X_IRQ_PCIINT0,
		.flags		= IORESOURCE_IRQ,
	},
	{
		/* MSI Interrupt Line */
		.name		= "msi_int",
		.start		= TI816X_IRQ_PCIINT1,
		.end		= TI816X_IRQ_PCIINT1,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device ti816x_pcie_device = {
	.name		= "ti816x_pcie",
	.id		= 0,
	.dev		= {
		.platform_data = &ti816x_pcie_data,
	},
	.num_resources	= ARRAY_SIZE(ti816x_pcie_resources),
	.resource	= ti816x_pcie_resources,
};

static inline void ti816x_init_pcie(void)
{
	if (cpu_is_ti816x()) {
		u32 v = omap_ctrl_readl(TI816X_CONTROL_PCIE_CFG);
		v = (v & ~TI816X_PCIE_DEVTYPE_MASK) | TI816X_PCIE_DEVTYPE_RC;
		omap_ctrl_writel(v, TI816X_CONTROL_PCIE_CFG);
		platform_device_register(&ti816x_pcie_device);
	}
}
#else
static inline void ti816x_init_pcie(void) {}
#endif

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_ARCH_TI816X)

#define TI816X_TPCC_BASE		0x49000000
#define TI816X_TPTC0_BASE		0x49800000
#define TI816X_TPTC1_BASE		0x49900000
#define TI816X_TPTC2_BASE		0x49a00000
#define TI816X_TPTC3_BASE		0x49b00000

static const s16 ti816x_dma_rsv_chans[][2] = {
	/* (offset, number) */
	{ 0,  4},	/* !@@@ TODO replace as appropriate - Sundaram*/
	{13,  3},
	{24,  4},
	{30,  2},
	{54,  3},
	{-1, -1}
};

static const s16 ti816x_dma_rsv_slots[][2] = {
	/* (offset, number) */
	{ 0,  4},	/* !@@@ TODO replace as appropriate - Sundaram*/
	{13,  3},
	{24,  4},
	{30,  2},
	{54,  3},
	{128, 384},
	{-1, -1}
};

/* Four Transfer Controllers on TI816X */
static const s8 ti816x_queue_tc_mapping[][2] = {
	/* {event queue no, TC no} */
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3},
	{-1, -1}
};
static const s8 ti816x_queue_priority_mapping[][2] = {
	/* {event queue no, Priority} */
	{0, 4},	/* !@@@ TODO replace as appropriate - Sundaram*/
	{1, 0},
	{2, 5},
	{3, 1},
	{-1, -1}
};
static struct edma_soc_info ti816x_edma_info[] = {
	{
		.n_channel		= 64,
		.n_region		= 5,	/* 0-2, 4-5 */
		.n_slot			= 512,
		.n_tc			= 4,
		.n_cc			= 1,
		.rsv_chans		= ti816x_dma_rsv_chans,
		.rsv_slots		= ti816x_dma_rsv_slots,
		.queue_tc_mapping	= ti816x_queue_tc_mapping,
		.queue_priority_mapping	= ti816x_queue_priority_mapping,
	},
};
static struct resource ti816x_edma_resources[] = {
	{
		.name	= "edma_cc0",
		.start	= TI816X_TPCC_BASE,
		.end	= TI816X_TPCC_BASE + SZ_32K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_tc0",
		.start	= TI816X_TPTC0_BASE,
		.end	= TI816X_TPTC0_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_tc1",
		.start	= TI816X_TPTC1_BASE,
		.end	= TI816X_TPTC1_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_tc2",
		.start	= TI816X_TPTC2_BASE,
		.end	= TI816X_TPTC2_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_tc3",
		.start	= TI816X_TPTC3_BASE,
		.end	= TI816X_TPTC3_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma0",
		.start	= TI816X_IRQ_EDMA_COMP,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "edma0_err",
		.start	= TI816X_IRQ_EDMA_ERR,
		.flags	= IORESOURCE_IRQ,
	},
};
static struct platform_device ti816x_edma_device = {
	.name		= "edma",
	.id		= -1,	/* !@@@ TODO replace as appropriate - Sundaram*/
	.dev = {
		.platform_data = ti816x_edma_info,
	},
	.num_resources	= ARRAY_SIZE(ti816x_edma_resources),
	.resource	= ti816x_edma_resources,
};
int __init ti816x_register_edma(void)
{
	struct platform_device *pdev;
	static struct clk *edma_clk;

	if (cpu_is_ti816x())
		pdev = &ti816x_edma_device;
	else {
		pr_err("%s: platform not supported\n", __func__);
		return -ENODEV;
	}

        edma_clk = clk_get(NULL, "tpcc_ick");
         if (IS_ERR(edma_clk)) {
                 printk(KERN_ERR "EDMA: Failed to get clock\n");
                 return -EBUSY;
        }
        clk_enable(edma_clk);
        edma_clk = clk_get(NULL, "tptc0_ick");
         if (IS_ERR(edma_clk)) {
                 printk(KERN_ERR "EDMA: Failed to get clock\n");
                 return -EBUSY;
        }
        clk_enable(edma_clk);
        edma_clk = clk_get(NULL, "tptc1_ick");
         if (IS_ERR(edma_clk)) {
                 printk(KERN_ERR "EDMA: Failed to get clock\n");
                 return -EBUSY;
        }
        clk_enable(edma_clk);
        edma_clk = clk_get(NULL, "tptc2_ick");
         if (IS_ERR(edma_clk)) {
                 printk(KERN_ERR "EDMA: Failed to get clock\n");
                 return -EBUSY;
        }
        clk_enable(edma_clk);
        edma_clk = clk_get(NULL, "tptc3_ick");
         if (IS_ERR(edma_clk)) {
                 printk(KERN_ERR "EDMA: Failed to get clock\n");
                 return -EBUSY;
        }
        clk_enable(edma_clk);


	return platform_device_register(pdev);
}

#else
static inline void ti816x_register_edma(void) {}
#endif

#ifdef CONFIG_ARCH_TI816X

#define TI816X_EMAC1_BASE		(0x4A100000)
#define TI816X_EMAC2_BASE		(0x4A120000)
#define TI816X_EMAC_CNTRL_OFFSET	(0x0)
#define TI816X_EMAC_CNTRL_MOD_OFFSET	(0x900)
#define TI816X_EMAC_CNTRL_RAM_OFFSET	(0x2000)
#define TI816X_EMAC_MDIO_OFFSET		(0x800)
#define TI816X_EMAC_CNTRL_RAM_SIZE	(0x2000)
#define TI816X_EMAC1_HW_RAM_ADDR	(0x4A102000)
#define TI816X_EMAC2_HW_RAM_ADDR	(0x4A122000)

#define TI816X_EMAC_PHY_MASK		(0xF)
#define TI816X_EMAC_MDIO_FREQ		(1000000)

static struct emac_platform_data ti816x_emac1_pdata = {
	.phy_mask	=	TI816X_EMAC_PHY_MASK,
	.mdio_max_freq	=	TI816X_EMAC_MDIO_FREQ,
	.rmii_en	=	0,
};

static struct emac_platform_data ti816x_emac2_pdata = {
	.phy_mask	=	TI816X_EMAC_PHY_MASK,
	.mdio_max_freq	=	TI816X_EMAC_MDIO_FREQ,
	.rmii_en	=	0,
};

static struct resource ti816x_emac1_resources[] = {
	{
		.start	=	TI816X_EMAC1_BASE,
		.end	=	TI816X_EMAC1_BASE + 0x3FFF,
		.flags	=	IORESOURCE_MEM,
	},
	{
		.start	=	TI816X_IRQ_MACRXTHR0,
		.end	=	TI816X_IRQ_MACRXTHR0,
		.flags	=	IORESOURCE_IRQ,
	},
	{
		.start	=	TI816X_IRQ_MACRXINT0,
		.end	=	TI816X_IRQ_MACRXINT0,
		.flags	=	IORESOURCE_IRQ,
	},
	{
		.start	=	TI816X_IRQ_MACTXINT0,
		.end	=	TI816X_IRQ_MACTXINT0,
		.flags	=	IORESOURCE_IRQ,
	},
	{
		.start	=	TI816X_IRQ_MACMISC0,
		.end	=	TI816X_IRQ_MACMISC0,
		.flags	=	IORESOURCE_IRQ,
	},
};

static struct resource ti816x_emac2_resources[] = {
	{
		.start	=	TI816X_EMAC2_BASE,
		.end	=	TI816X_EMAC2_BASE + 0x3FFF,
		.flags	=	IORESOURCE_MEM,
	},
	{
		.start	=	TI816X_IRQ_MACRXTHR1,
		.end	=	TI816X_IRQ_MACRXTHR1,
		.flags	=	IORESOURCE_IRQ,
	},
	{
		.start	=	TI816X_IRQ_MACRXINT1,
		.end	=	TI816X_IRQ_MACRXINT1,
		.flags	=	IORESOURCE_IRQ,
	},
	{
		.start	=	TI816X_IRQ_MACTXINT1,
		.end	=	TI816X_IRQ_MACTXINT1,
		.flags	=	IORESOURCE_IRQ,
	},
	{
		.start	=	TI816X_IRQ_MACMISC1,
		.end	=	TI816X_IRQ_MACMISC1,
		.flags	=	IORESOURCE_IRQ,
	},
};
static struct platform_device ti816x_emac1_device = {
	.name	=	"davinci_emac",
	.id	=	0,
	.num_resources	=	ARRAY_SIZE(ti816x_emac1_resources),
	.resource	=	ti816x_emac1_resources,
};

static struct platform_device ti816x_emac2_device = {
	.name	=	"davinci_emac",
	.id	=	1,
	.num_resources	=	ARRAY_SIZE(ti816x_emac2_resources),
	.resource	=	ti816x_emac2_resources,
};

void ti816x_ethernet_init(void)
{
	u32 mac_lo, mac_hi;

	mac_lo = omap_ctrl_readl(TI816X_CONTROL_MAC_ID0_LO);
	mac_hi = omap_ctrl_readl(TI816X_CONTROL_MAC_ID0_HI);
	ti816x_emac1_pdata.mac_addr[0] = mac_hi & 0xFF;
	ti816x_emac1_pdata.mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	ti816x_emac1_pdata.mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	ti816x_emac1_pdata.mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	ti816x_emac1_pdata.mac_addr[4] = mac_lo & 0xFF;
	ti816x_emac1_pdata.mac_addr[5] = (mac_lo & 0xFF) >> 8;

	ti816x_emac1_pdata.ctrl_reg_offset = TI816X_EMAC_CNTRL_OFFSET;
	ti816x_emac1_pdata.ctrl_mod_reg_offset = TI816X_EMAC_CNTRL_MOD_OFFSET;
	ti816x_emac1_pdata.ctrl_ram_offset = TI816X_EMAC_CNTRL_RAM_OFFSET;
	ti816x_emac1_pdata.mdio_reg_offset = TI816X_EMAC_MDIO_OFFSET;
	ti816x_emac1_pdata.ctrl_ram_size = TI816X_EMAC_CNTRL_RAM_SIZE;
	ti816x_emac1_pdata.version = EMAC_VERSION_2;
	ti816x_emac1_pdata.hw_ram_addr = TI816X_EMAC1_HW_RAM_ADDR;
	ti816x_emac1_pdata.interrupt_enable = NULL;
	ti816x_emac1_pdata.interrupt_disable = NULL;
	ti816x_emac1_device.dev.platform_data = &ti816x_emac1_pdata;
	platform_device_register(&ti816x_emac1_device);

	mac_lo = omap_ctrl_readl(TI816X_CONTROL_MAC_ID1_LO);
	mac_hi = omap_ctrl_readl(TI816X_CONTROL_MAC_ID1_HI);
	ti816x_emac2_pdata.mac_addr[0] = mac_hi & 0xFF;
	ti816x_emac2_pdata.mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	ti816x_emac2_pdata.mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	ti816x_emac2_pdata.mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	ti816x_emac2_pdata.mac_addr[4] = mac_lo & 0xFF;
	ti816x_emac2_pdata.mac_addr[5] = (mac_lo & 0xFF) >> 8;

	ti816x_emac2_pdata.ctrl_reg_offset = TI816X_EMAC_CNTRL_OFFSET;
	ti816x_emac2_pdata.ctrl_mod_reg_offset = TI816X_EMAC_CNTRL_MOD_OFFSET;
	ti816x_emac2_pdata.ctrl_ram_offset = TI816X_EMAC_CNTRL_RAM_OFFSET;
	ti816x_emac2_pdata.mdio_reg_offset = TI816X_EMAC_MDIO_OFFSET;
	ti816x_emac2_pdata.ctrl_ram_size = TI816X_EMAC_CNTRL_RAM_SIZE;
	ti816x_emac2_pdata.version = EMAC_VERSION_2;
	ti816x_emac2_pdata.hw_ram_addr = TI816X_EMAC2_HW_RAM_ADDR;
	ti816x_emac2_pdata.interrupt_enable = NULL;
	ti816x_emac2_pdata.interrupt_disable = NULL;
	ti816x_emac2_device.dev.platform_data = &ti816x_emac2_pdata;
#if 0
	platform_device_register(&ti816x_emac2_device);
	/* TODO: enable Pin Mux for EMAC instance 2 */
#endif
}
#else
static inline void ti816x_ethernet_init(void) {}
#endif
/*-------------------------------------------------------------------------*/

static int __init omap2_init_devices(void)
{
	/* please keep these calls, and their implementations above,
	 * in alphabetical order so they're easier to sort through.
	 */
	omap_hsmmc_reset();
	omap_init_camera();
	omap_init_mbox();
	omap_init_mcspi();
	omap_hdq_init();
	ti816x_init_pcie();
	omap_init_sti();
	omap_init_sha1_md5();
	omap_init_vout();
	ti816x_register_edma();
	ti816x_ethernet_init();

	return 0;
}
arch_initcall(omap2_init_devices);
