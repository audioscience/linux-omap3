/*
 * drivers/usb/otg/nop-usb-xceiv.c
 *
 * NOP USB transceiver for all USB transceiver which are either built-in
 * into USB IP or which are mostly autonomous.
 *
 * Copyright (C) 2009 Texas Instruments Inc
 * Author: Ajay Kumar Gupta <ajay.gupta@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Current status:
 * 	this is to add "nop" transceiver for all those phy which is
 * 	autonomous such as isp1504 etc.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/usb/otg.h>

#ifdef CONFIG_TWL4030_CORE

#include <linux/i2c/twl4030.h>

/* In module TWL4030_MODULE_PM_MASTER */
#define PROTECT_KEY                     0x0E

/* In module TWL4030_MODULE_PM_RECEIVER */
#define VUSB_DEDICATED1                 0x7D
#define VUSB_DEDICATED2                 0x7E
#define VUSB1V5_DEV_GRP                 0x71
#define VUSB1V5_TYPE                    0x72
#define VUSB1V5_REMAP                   0x73
#define VUSB1V8_DEV_GRP                 0x74
#define VUSB1V8_TYPE                    0x75
#define VUSB1V8_REMAP                   0x76
#define VUSB3V1_DEV_GRP                 0x77
#define VUSB3V1_TYPE                    0x78
#define VUSB3V1_REMAP                   0x79

#endif

struct nop_usb_xceiv {
	struct otg_transceiver	otg;
	struct device		*dev;
};

static u64 nop_xceiv_dmamask = DMA_32BIT_MASK;

static struct platform_device nop_xceiv_device = {
	.name           = "nop_usb_xceiv",
	.id             = -1,
	.dev = {
		.dma_mask               = &nop_xceiv_dmamask,
		.coherent_dma_mask      = DMA_32BIT_MASK,
		.platform_data          = NULL,
	},
};

void nop_xceiv_register(void)
{
	if (platform_device_register(&nop_xceiv_device) < 0) {
		printk(KERN_ERR "Unable to register NOP-XCEIV device\n");
		return;
	}
}

void nop_xceiv_unregister(void)
{
	platform_device_unregister(&nop_xceiv_device);
}

static inline struct nop_usb_xceiv *xceiv_to_nop(struct otg_transceiver *x)
{
	return container_of(x, struct nop_usb_xceiv, otg);
}

#ifdef CONFIG_TWL4030_CORE
static void twl4030_usb_ldo_init(void)
{
	/* Enable writing to power configuration registers */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0xC0, PROTECT_KEY);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x0C, PROTECT_KEY);

	/* put VUSB3V1 LDO in active state */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB_DEDICATED2);

	/* input to VUSB3V1 LDO is from VBAT, not VBUS */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x14, VUSB_DEDICATED1);

	/* turn on 3.1V regulator */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x20, VUSB3V1_DEV_GRP);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB3V1_TYPE);

	/* turn on 1.5V regulator */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x20, VUSB1V5_DEV_GRP);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V5_TYPE);

	/* turn on 1.8V regulator */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x20, VUSB1V8_DEV_GRP);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V8_TYPE);

	/* disable access to power configuration registers */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0, PROTECT_KEY);
}
#endif

static int nop_set_suspend(struct otg_transceiver *x, int suspend)
{
	return 0;
}

static int nop_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct nop_usb_xceiv *nop;

	if (!x)
		return -ENODEV;

	nop = xceiv_to_nop(x);

	if (!gadget) {
		nop->otg.gadget = NULL;
		return -ENODEV;
	}

	nop->otg.gadget = gadget;
	nop->otg.state = OTG_STATE_B_IDLE;
	return 0;
}

static int nop_set_host(struct otg_transceiver *x, struct usb_bus *host)
{
	struct nop_usb_xceiv *nop;

	if (!x)
		return -ENODEV;

	nop = xceiv_to_nop(x);

	if (!host) {
		nop->otg.host = NULL;
		return -ENODEV;
	}

	nop->otg.host = host;
	return 0;
}

static int __devinit nop_usb_xceiv_probe(struct platform_device *pdev)
{
	struct nop_usb_xceiv	*nop;
	int err;

	nop = kzalloc(sizeof *nop, GFP_KERNEL);
	if (!nop)
		return -ENOMEM;

	nop->dev		= &pdev->dev;
	nop->otg.dev		= nop->dev;
	nop->otg.label		= "nop-xceiv";
	nop->otg.state		= OTG_STATE_UNDEFINED;
	nop->otg.set_host	= nop_set_host;
	nop->otg.set_peripheral	= nop_set_peripheral;
	nop->otg.set_suspend	= nop_set_suspend;

	err = otg_set_transceiver(&nop->otg);
	if (err) {
		dev_err(&pdev->dev, "can't register transceiver, err: %d\n",
			err);
		goto exit;
	}

	platform_set_drvdata(pdev, nop);

#ifdef CONFIG_TWL4030_CORE
	/* twl4030 power configuration */
	twl4030_usb_ldo_init();
#endif
	return 0;
exit:
	kfree(nop);
	return err;
}

static int __devexit nop_usb_xceiv_remove(struct platform_device *pdev)
{
	struct nop_usb_xceiv *nop = platform_get_drvdata(pdev);

	otg_set_transceiver(NULL);

	platform_set_drvdata(pdev, NULL);
	kfree(nop);

	return 0;
}

static struct platform_driver nop_usb_xceiv_driver = {
	.probe		= nop_usb_xceiv_probe,
	.remove		= __devexit_p(nop_usb_xceiv_remove),
	.driver		= {
		.name	= "nop_usb_xceiv",
		.owner	= THIS_MODULE,
	},
};

static int __init nop_usb_xceiv_init(void)
{
	return platform_driver_register(&nop_usb_xceiv_driver);
}
subsys_initcall(nop_usb_xceiv_init);

static void __exit nop_usb_xceiv_exit(void)
{
	platform_driver_unregister(&nop_usb_xceiv_driver);
}
module_exit(nop_usb_xceiv_exit);

MODULE_ALIAS("platform:nop_usb_xceiv");
MODULE_AUTHOR("Texas Instruments Inc");
MODULE_DESCRIPTION("NOP USB Transceiver driver");
MODULE_LICENSE("GPL");
