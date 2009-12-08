/*
 * linux/arch/arm/mach-omap2/board-omap3evm.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/leds.h>
#include <linux/interrupt.h>

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c/twl4030.h>
#include <linux/regulator/machine.h>
#include <linux/usb/otg.h>
#include <linux/smsc911x.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>


#include <linux/regulator/machine.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/mux.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/mcspi.h>
#include <plat/clock.h>
#include <plat/omap-pm.h>
#include <plat/display.h>

#include "sdram-micron-mt46h32m32lf-6.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "prm-regbits-34xx.h"
#include "omap3-opp.h"
#include "board-omap3evm-camera.h"

#include "board-omap35x-pmic.h"

#define GPMC_CS0_BASE	0x60
#define GPMC_CS_SIZE	0x30

#define NAND_BLOCK_SIZE	SZ_128K

#define OMAP3_EVM_TS_GPIO	175
#define OMAP3_EVM_EHCI_VBUS	22
#define OMAP3_EVM_EHCI_SELECT	61

#define OMAP3EVM_ETHR_START	0x2c000000
#define OMAP3EVM_ETHR_SIZE	1024
#define OMAP3EVM_ETHR_ID_REV	0x50
#define OMAP3EVM_ETHR_GPIO_IRQ	176
#define OMAP3EVM_SMC911X_CS	5


static struct mtd_partition omap3evm_nand_partitions[] = {
       /* All the partition sizes are listed in terms of NAND block size */
       {
               .name           = "xloader-nand",
               .offset         = 0,
               .size           = 4*(SZ_128K),
               .mask_flags     = MTD_WRITEABLE
       },
       {
               .name           = "uboot-nand",
               .offset         = MTDPART_OFS_APPEND,
               .size           = 14*(SZ_128K),
               .mask_flags     = MTD_WRITEABLE
       },
       {
               .name           = "params-nand",
               .offset         = MTDPART_OFS_APPEND,
               .size           = 2*(SZ_128K)
       },
       {
               .name           = "linux-nand",
               .offset         = MTDPART_OFS_APPEND,
               .size           = 40*(SZ_128K)
       },
       {
               .name           = "jffs2-nand",
               .size           = MTDPART_SIZ_FULL,
               .offset         = MTDPART_OFS_APPEND,
       },
};

static struct omap_nand_platform_data omap3evm_nand_data = {
       .parts          = omap3evm_nand_partitions,
       .nr_parts       = ARRAY_SIZE(omap3evm_nand_partitions),
       .nand_setup     = NULL,
       .dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
       .dev_ready      = NULL,
};

static struct resource omap3evm_nand_resource = {
       .flags          = IORESOURCE_MEM,
};

static struct platform_device omap3evm_nand_device = {
       .name           = "omap2-nand",
       .id             = 0,
       .dev            = {
               .platform_data  = &omap3evm_nand_data,
       },
       .num_resources  = 1,
       .resource       = &omap3evm_nand_resource,
};


void __init omap3evm_flash_init(void)
{
       u8 cs = 0;
       u8 nandcs = GPMC_CS_NUM + 1;
       u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

       while (cs < GPMC_CS_NUM) {
               u32 ret = 0;
               ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

               if ((ret & 0xC00) == 0x800) {
                       /* Found it!! */
                       if (nandcs > GPMC_CS_NUM)
                               nandcs = cs;
               }
               cs++;
       }
       if (nandcs > GPMC_CS_NUM) {
               printk(KERN_INFO "NAND: Unable to find configuration "
                       " in GPMC\n ");
               return;
       }

       if (nandcs < GPMC_CS_NUM) {
               omap3evm_nand_data.cs   = nandcs;
               omap3evm_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add +
                                       GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);
               omap3evm_nand_data.gpmc_baseaddr   = (void *) (gpmc_base_add);

               if (platform_device_register(&omap3evm_nand_device) < 0) {
                       printk(KERN_ERR "Unable to register NAND device\n");
                       }
       }
}

static u8 omap3_evm_version;

u8 get_omap3_evm_rev(void)
{
	return omap3_evm_version;
}
EXPORT_SYMBOL(get_omap3_evm_rev);

static void __init omap3_evm_get_revision(void)
{
	void __iomem *ioaddr;
	unsigned int smsc_id;

	/* Ethernet PHY ID is stored at ID_REV register */
	ioaddr = ioremap_nocache(OMAP3EVM_ETHR_START, SZ_1K);
	smsc_id = readl(ioaddr + OMAP3EVM_ETHR_ID_REV) & 0xFFFF0000;
	iounmap(ioaddr);

	switch (smsc_id) {
	/*SMSC9115 chipset*/
	case 0x01150000:
		omap3_evm_version = OMAP3EVM_BOARD_GEN_1;
		break;
	/*SMSC 9220 chipset*/
	case 0x92200000:
	default:
		omap3_evm_version = OMAP3EVM_BOARD_GEN_2;
	}
}

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct resource omap3evm_smsc911x_resources[] = {
	[0] =	{
		.start	= OMAP3EVM_ETHR_START,
		.end	= (OMAP3EVM_ETHR_START + OMAP3EVM_ETHR_SIZE - 1),
		.flags	= IORESOURCE_MEM,
	},
	[1] =	{
		.start	= OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
		.end	= OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
		.flags	= (IORESOURCE_IRQ | IRQF_TRIGGER_LOW),
	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface  = PHY_INTERFACE_MODE_MII,
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags          = (SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS),
};

static struct platform_device omap3evm_smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap3evm_smsc911x_resources),
	.resource	= &omap3evm_smsc911x_resources[0],
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static inline void __init omap3evm_init_smsc911x(void)
{
	int eth_cs;
	struct clk *l3ck;
	unsigned int rate;

	eth_cs = OMAP3EVM_SMC911X_CS;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (gpio_request(OMAP3EVM_ETHR_GPIO_IRQ, "SMC911x irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc911x IRQ\n",
			OMAP3EVM_ETHR_GPIO_IRQ);
		return;
	}

	gpio_direction_input(OMAP3EVM_ETHR_GPIO_IRQ);

	platform_device_register(&omap3evm_smsc911x_device);
}

#else
static inline void __init omap3evm_init_smsc911x(void) { return; }
#endif

/*
 * OMAP3EVM LCD Panel control signals
 */
#define OMAP3EVM_LCD_PANEL_LR		2
#define OMAP3EVM_LCD_PANEL_UD		3
#define OMAP3EVM_LCD_PANEL_INI		152
#define OMAP3EVM_LCD_PANEL_ENVDD	153
#define OMAP3EVM_LCD_PANEL_QVGA		154
#define OMAP3EVM_LCD_PANEL_RESB		155
#define OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO	210
#define OMAP3EVM_DVI_PANEL_EN_GPIO	199

#define ENABLE_VPLL2_DEV_GRP	0xE0

static int lcd_enabled;
static int dvi_enabled;

static void __init omap3_evm_display_init(void)
{
	int r;

	r = gpio_request(OMAP3EVM_LCD_PANEL_RESB, "lcd_panel_resb");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_resb\n");
		return;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_RESB, 1);

	r = gpio_request(OMAP3EVM_LCD_PANEL_INI, "lcd_panel_ini");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_ini\n");
		goto err_1;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_INI, 1);

	r = gpio_request(OMAP3EVM_LCD_PANEL_QVGA, "lcd_panel_qvga");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_qvga\n");
		goto err_2;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_QVGA, 0);

	r = gpio_request(OMAP3EVM_LCD_PANEL_LR, "lcd_panel_lr");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_lr\n");
		goto err_3;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_LR, 1);

	r = gpio_request(OMAP3EVM_LCD_PANEL_UD, "lcd_panel_ud");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_ud\n");
		goto err_4;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_UD, 1);

	r = gpio_request(OMAP3EVM_LCD_PANEL_ENVDD, "lcd_panel_envdd");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_envdd\n");
		goto err_5;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_ENVDD, 0);

	return;

err_5:
	gpio_free(OMAP3EVM_LCD_PANEL_UD);
err_4:
	gpio_free(OMAP3EVM_LCD_PANEL_LR);
err_3:
	gpio_free(OMAP3EVM_LCD_PANEL_QVGA);
err_2:
	gpio_free(OMAP3EVM_LCD_PANEL_INI);
err_1:
	gpio_free(OMAP3EVM_LCD_PANEL_RESB);

}

static int omap3_evm_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}
	gpio_set_value(OMAP3EVM_LCD_PANEL_ENVDD, 0);

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2)
		gpio_set_value(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 0);
	else
		gpio_set_value(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 1);

	lcd_enabled = 1;

	return 0;
}

static void omap3_evm_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(OMAP3EVM_LCD_PANEL_ENVDD, 1);

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2)
		gpio_set_value(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 1);
	else
		gpio_set_value(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 0);

	lcd_enabled = 0;
}

static struct omap_dss_device omap3_evm_lcd_device = {
	.name			= "lcd",
	.driver_name		= "sharp_ls_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 18,
	.platform_enable	= omap3_evm_enable_lcd,
	.platform_disable	= omap3_evm_disable_lcd,
};

static int omap3_evm_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void omap3_evm_disable_tv(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device omap3_evm_tv_device = {
	.name			= "tv",
	.driver_name		= "venc",
	.type			= OMAP_DISPLAY_TYPE_VENC,
#if defined(CONFIG_OMAP2_VENC_OUT_TYPE_SVIDEO)
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
#elif defined(CONFIG_OMAP2_VENC_OUT_TYPE_COMPOSITE)
	.u.venc.type		= OMAP_DSS_VENC_TYPE_COMPOSITE,
#endif
	.platform_enable	= omap3_evm_enable_tv,
	.platform_disable	= omap3_evm_disable_tv,
};

static int omap3_evm_enable_dvi(struct omap_dss_device *dssdev)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, ENABLE_VPLL2_DEV_GRP,
			TWL4030_VPLL2_DEV_GRP);
	gpio_set_value(OMAP3EVM_DVI_PANEL_EN_GPIO, 1);

	dvi_enabled = 1;
	return 0;
}

static void omap3_evm_disable_dvi(struct omap_dss_device *dssdev)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x0,
			TWL4030_VPLL2_DEV_GRP);
	gpio_set_value(OMAP3EVM_DVI_PANEL_EN_GPIO, 0);

	dvi_enabled = 0;
}

static struct omap_dss_device omap3_evm_dvi_device = {
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.platform_enable	= omap3_evm_enable_dvi,
	.platform_disable	= omap3_evm_disable_dvi,
};

static struct omap_dss_device *omap3_evm_dss_devices[] = {
	&omap3_evm_lcd_device,
	&omap3_evm_tv_device,
	&omap3_evm_dvi_device,
};

static struct omap_dss_board_info omap3_evm_dss_data = {
	.num_devices	= ARRAY_SIZE(omap3_evm_dss_devices),
	.devices	= omap3_evm_dss_devices,
	.default_device	= &omap3_evm_lcd_device,
};

static struct platform_device omap3_evm_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &omap3_evm_dss_data,
	},
};

/* Create a single supply for VMMC1 */
REGULATOR_CONSUMER_SINGLE_SUPPLY(vmmc1, vmmc, NULL);

/* Create a single supply for VSIM */
REGULATOR_CONSUMER_SINGLE_SUPPLY(vsim, vmmc_aux, NULL);

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
REGULATOR_INIT_DATA(vmmc1, VMMC1, 1850000, 3150000,
		TWL_REGULATOR_MODES_DEFAULT,
		TWL_REGULATOR_OPS_DEFAULT | REGULATOR_CHANGE_VOLTAGE,
		false, false);

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
/* Create init data for VMMC1 */
REGULATOR_INIT_DATA(vsim, VSIM, 1800000, 3000000,
		TWL_REGULATOR_MODES_DEFAULT,
		TWL_REGULATOR_OPS_DEFAULT | REGULATOR_CHANGE_VOLTAGE,
		false, false);

static struct platform_device omap3evm_camkit_device = {
	.name		= "omap3evm_camkit",
	.id		= -1,
};

/* Create supplies for VAUX2 */
REGULATOR_CONSUMER_SINGLE_SUPPLY(vaux2, hsusb1, NULL);
/* VAUX2 for EHCI module on OMAP3EVM Rev >= E */
TWL_VAUX2_DATA;

/* Create supplies for VUSB1V5 */
REGULATOR_CONSUMER_SINGLE_SUPPLY(vusb1v5, hsusb1-aux, NULL);
/* VUSB1V5 for EHCI module on OMAP3EVM Rev < E*/
TWL_VUSB1V5_DATA;

/* Create supplies for VUSB1V8 */
REGULATOR_CONSUMER_SINGLE_SUPPLY(vusb1v8, hsusb1, NULL);
/* VUSB1V8 for EHCI module */
TWL_VUSB1V8_DATA;

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= 63,
	},
	{}	/* Terminator */
};

static struct gpio_led gpio_leds[] = {
	{
		.name			= "omap3evm::ledb",
		/* normally not visible (board underside) */
		.default_trigger	= "default-on",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

#ifdef CONFIG_PM
/*
 * Save the state of keypad
 *
 * TODO: This definition should ideally be in a header file, but
 *       matrix_keypad.h is not the right one. Also, plat/keypad.h
 *       is no longer used.
 */
struct omap_keypad_pm_state {
	void __iomem *wk_st;
	void __iomem *wk_en;
	u32 wk_mask;
	u32 padconf;
};

/*
 * Board specific hook for keypad suspend
 */
void omap3_evm_kp_suspend(void *ptr)
{
	struct omap_keypad_pm_state *pstate =
			(struct omap_keypad_pm_state *)ptr;

	if (pstate) {
		/*
		 * Set wake-enable bit
		 */
		if (pstate->wk_en && pstate->wk_mask) {
			u32 v = __raw_readl(pstate->wk_en);
			v |= pstate->wk_mask;
			__raw_writel(v, pstate->wk_en);
		}
		/*
		 * Set corresponding IOPAD wakeup-enable
		 */
		if (cpu_is_omap34xx() && pstate->padconf) {
			u16 v = omap_ctrl_readw(pstate->padconf);
			v |= OMAP3_PADCONF_WAKEUPENABLE0;
			omap_ctrl_writew(v, pstate->padconf);
		}
	}
}

/*
 * Board specific hook for keypad resume
 */
void omap3_evm_kp_resume(void *ptr)
{
	struct omap_keypad_pm_state *pstate =
			(struct omap_keypad_pm_state *)ptr;

	if (pstate) {
		/*
		 * Clear wake-enable bit
		 */
		if (pstate->wk_en && pstate->wk_mask) {
			u32 v = __raw_readl(pstate->wk_en);
			v &= ~pstate->wk_mask;
			__raw_writel(v, pstate->wk_en);
		}
		/*
		 * Clear corresponding IOPAD wakeup-enable
		 */
		if (cpu_is_omap34xx() && pstate->padconf) {
			u16 v = omap_ctrl_readw(pstate->padconf);
			v &= ~OMAP3_PADCONF_WAKEUPENABLE0;
			omap_ctrl_writew(v, pstate->padconf);
		}
	}
}

static struct omap_keypad_pm_state omap3evm_kp_pm_state = {
	.wk_st		= OMAP34XX_PRM_REGADDR(WKUP_MOD, PM_WKEN1),
	.wk_en		= OMAP34XX_PRM_REGADDR(WKUP_MOD, PM_WKST1),
	.wk_mask	= OMAP3430_EN_GPIO1,
	.padconf	= 0x1e0,
};
#endif	/* CONFIG_PM */

static int omap3evm_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	omap_cfg_reg(L8_34XX_GPIO63);
	mmc[0].gpio_cd = gpio + 0;
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters */
	vmmc1_consumers[0].dev = mmc[0].dev;
	vsim_consumers[0].dev = mmc[0].dev;

	/*
	 * Most GPIOs are for USB OTG.  Some are mostly sent to
	 * the P2 connector; notably LEDA for the LCD backlight.
	 */

	/* TWL4030_GPIO_MAX + 0 == ledA, LCD Backlight control */
	gpio_request(gpio + TWL4030_GPIO_MAX, "EN_LCD_BKL");
	gpio_direction_output(gpio + TWL4030_GPIO_MAX, 0);

	/* gpio + 7 == DVI Enable */
	gpio_request(gpio + 7, "EN_DVI");
	gpio_direction_output(gpio + 7, 0);

	/* TWL4030_GPIO_MAX + 1 == ledB (out, active low LED) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

	platform_device_register(&leds_gpio);

	return 0;
}

static struct twl4030_gpio_platform_data omap3evm_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.setup		= omap3evm_twl_gpio_setup,
};

static struct twl4030_usb_data omap3evm_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static int board_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_DOWN),
	KEY(0, 2, KEY_ENTER),
	KEY(0, 3, KEY_M),

	KEY(1, 0, KEY_RIGHT),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_I),
	KEY(1, 3, KEY_N),

	KEY(2, 0, KEY_A),
	KEY(2, 1, KEY_E),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_K),

	KEY(3, 0, KEY_B),
	KEY(3, 1, KEY_F),
	KEY(3, 2, KEY_O),
	KEY(3, 3, KEY_P)
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data omap3evm_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 4,
	.cols		= 4,
	.rep		= 1,
#ifdef CONFIG_PM
	.pm_state	= (void *)&omap3evm_kp_pm_state,
	.on_suspend	= omap3_evm_kp_suspend,
	.on_resume	= omap3_evm_kp_resume,
#endif	/* CONFIG_PM */
};

static struct twl4030_madc_platform_data omap3evm_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_ins __initdata sleep_on_seq[] = {
	/* Turn off HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_OFF), 2},
	/* Turn OFF VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_OFF), 2},
	/* Turn OFF VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_OFF), 2},
	/* Turn OFF VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_OFF), 2},
};

static struct twl4030_script sleep_on_script __initdata = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

static struct twl4030_ins wakeup_p12_seq[] __initdata = {
	/* Turn on HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	/* Turn ON VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_ACTIVE), 2},
	/* Turn ON VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_ACTIVE), 2},
	/* Turn ON VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p12_script __initdata = {
	.script	= wakeup_p12_seq,
	.size	= ARRAY_SIZE(wakeup_p12_seq),
	.flags	= TWL4030_WAKEUP12_SCRIPT,
};

static struct twl4030_ins wakeup_p3_seq[] __initdata = {
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p3_script __initdata = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset VDD1 regulator.
 * Reset VDD2 regulator.
 * Reset VPLL1 regulator.
 * Enable sysclk output.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_WRST), 0x60},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_ACTIVE), 2},
};
static struct twl4030_script wrst_script __initdata = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] __initdata = {
	&sleep_on_script,
	&wakeup_p12_script,
	&wakeup_p3_script,
	&wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] = {
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3, .type = -1,
		.type2 = -1 },
	{ .resource = RES_VDD1, .devgroup = DEV_GRP_P1, .type = -1,
		.type2 = -1 },
	{ .resource = RES_VDD2, .devgroup = DEV_GRP_P1, .type = -1,
		.type2 = -1 },
	{ 0, 0},
};

static struct twl4030_power_data omap3evm_t2scripts_data __initdata = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

REGULATOR_CONSUMER_SINGLE_SUPPLY(vdac, vdda_dac, &omap3_evm_dss_device.dev);

/* VDAC for DSS driving S-Video */
TWL_VDAC_DATA;

/* VPLL2 for digital video outputs */
REGULATOR_CONSUMER_SINGLE_SUPPLY(vpll2, vdvi, &omap3_evm_lcd_device.dev);

REGULATOR_INIT_DATA(vpll2, VDVI, 1800000, 1800000,
		TWL_REGULATOR_MODES_DEFAULT,
		TWL_REGULATOR_OPS_DEFAULT,
		false, true);

static struct twl4030_platform_data omap3evm_twldata __initdata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.keypad		= &omap3evm_kp_data,
	.madc		= &omap3evm_madc_data,
	.power		= &omap3evm_t2scripts_data,
	.usb		= &omap3evm_usb_data,
	.gpio		= &omap3evm_gpio_data,
	.vmmc1		= &vmmc1_data[0],
	.vdac		= &vdac_data[0],
	.vpll2		= &vpll2_data[0],
	.vsim		= &vsim_data[0],
};

static struct i2c_board_info __initdata omap3evm_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &omap3evm_twldata,
	},
};

static int __init omap3_evm_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, omap3evm_i2c_boardinfo,
			ARRAY_SIZE(omap3evm_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static void ads7846_dev_init(void)
{
	if (gpio_request(OMAP3_EVM_TS_GPIO, "ADS7846 pendown") < 0) {
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");
		return;
	}

	omap_cfg_reg(AC3_34XX_GPIO175);

	gpio_direction_input(OMAP3_EVM_TS_GPIO);

	omap_set_gpio_debounce(OMAP3_EVM_TS_GPIO, 1);
	omap_set_gpio_debounce_time(OMAP3_EVM_TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(OMAP3_EVM_TS_GPIO);
}

struct ads7846_platform_data ads7846_config = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 10,
	.debounce_tol		= 3,
	.debounce_rep		= 1,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.settle_delay_usecs	= 150,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

struct spi_board_info omap3evm_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(OMAP3_EVM_TS_GPIO),
		.platform_data		= &ads7846_config,
	},
};

static struct omap_board_config_kernel omap3_evm_config[] __initdata = {
};

static void __init omap3_evm_init_irq(void)
{
	omap_board_config = omap3_evm_config;
	omap_board_config_size = ARRAY_SIZE(omap3_evm_config);
	omap2_init_common_hw(mt46h32m32lf6_sdrc_params, NULL, omap3_mpu_rate_table,
	                     omap3_dsp_rate_table, omap3_l3_rate_table);
	omap_init_irq();
	omap_gpio_init();
}

static struct platform_device *omap3_evm_devices[] __initdata = {
	&omap3_evm_dss_device,
	&omap3evm_camkit_device,
};

static struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	/* PHY reset GPIO will be runtime programmed based on EVM version */
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL,

	.aux[0]	= 0,
	.aux[1]	= 0,
	.aux[2]	= 0,
};

/*
 * Set wakeup sources for the board
 */
static void __init omap3_evm_wakeup_sources(void)
{
	pr_info("omap3evm: Adding wakeup sources");

	omap_cfg_reg(AF26_34XX_SYS_NIRQ);
}

static void __init omap3_evm_init(void)
{
	omap3_evm_get_revision();

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2) {
		omap3evm_twldata.vaux2 = &vaux2_data[0];
	} else {
		omap3evm_twldata.vusb1v5 = &vusb1v5_data[0];
		omap3evm_twldata.vusb1v8 = &vusb1v8_data[0];
	}

	omap3_evm_i2c_init();

	regulator_has_full_constraints();

	platform_add_devices(omap3_evm_devices, ARRAY_SIZE(omap3_evm_devices));

	spi_register_board_info(omap3evm_spi_board_info,
				ARRAY_SIZE(omap3evm_spi_board_info));

	omap_serial_init();
#ifdef CONFIG_NOP_USB_XCEIV
	/* OMAP3EVM uses ISP1504 phy and so register nop transceiver */
	usb_nop_xceiv_register();
#endif
	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2) {
		/* enable EHCI VBUS using GPIO22 */
		omap_cfg_reg(AF9_34XX_GPIO22);
		gpio_request(OMAP3_EVM_EHCI_VBUS, "enable EHCI VBUS");
		gpio_direction_output(OMAP3_EVM_EHCI_VBUS, 0);
		gpio_set_value(OMAP3_EVM_EHCI_VBUS, 1);

		/* Select EHCI port on main board */
		omap_cfg_reg(U3_34XX_GPIO61);
		gpio_request(OMAP3_EVM_EHCI_SELECT, "select EHCI port");
		gpio_direction_output(OMAP3_EVM_EHCI_SELECT, 0);
		gpio_set_value(OMAP3_EVM_EHCI_SELECT, 0);

		/* setup EHCI phy reset config */
		omap_cfg_reg(AH14_34XX_GPIO21);
		ehci_pdata.reset_gpio_port[1] = 21;

	} else {
		/* setup EHCI phy reset on MDC */
		omap_cfg_reg(AF4_34XX_GPIO135_OUT);
		ehci_pdata.reset_gpio_port[1] = 135;

		/* MDC also need VUSB1V5 regulator */
		ehci_pdata.aux[1] = 1;
	}
	usb_musb_init();
	omap3evm_flash_init();
	usb_ehci_init(&ehci_pdata);
	ads7846_dev_init();
	omap3evm_init_smsc911x();
	omap3_evm_display_init();

	omap3_evm_wakeup_sources();
}

static void __init omap3_evm_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP3EVM, "OMAP3 EVM")
	/* Maintainer: Syed Mohammed Khasim - Texas Instruments */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_evm_map_io,
	.init_irq	= omap3_evm_init_irq,
	.init_machine	= omap3_evm_init,
	.timer		= &omap_timer,
MACHINE_END
