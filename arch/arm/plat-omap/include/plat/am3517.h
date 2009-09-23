/*:
 * Address mappings and base address for AM3517 specific interconnects
 * and peripherals.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Sriramakrishnan <srk@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_ARCH_AM3517_H
#define __ASM_ARCH_AM3517_H

/*
 * ----------------------------------------------------------------------------
 * Base addresses
 * ----------------------------------------------------------------------------
 */

#define AM3517_IPSS_EMAC_BASE		0x5C000000
#define AM3517_IPSS_USBOTGSS_BASE	0x5C040000
#define AM3517_IPSS_HECC_BASE		0x5C050000
#define AM3517_IPSS_VPFE_BASE		0x5C060000

/*AM3517 CONTROL_LVL_INTR_CLEAR bits*/
#define AM3517_CPGMAC_MISC_PULSE_CLR	BIT(0)
#define AM3517_CPGMAC_RX_PULSE_CLR	BIT(1)
#define AM3517_CPGMAC_RX_THRESH_CLR	BIT(2)
#define AM3517_CPGMAC_TX_PULSE_CLR	BIT(3)
#define AM3517_USBOTG_INT_CLR		BIT(4)
#define AM3517_VPFE_VD0_INT_CLR		BIT(5)
#define AM3517_VPFE_VD1_INT_CLR		BIT(6)
#define AM3517_VPFE_VD2_INT_CLR		BIT(7)

/*AM3517 CONTROL_IP_SW_RESET bits*/
#define AM3517_USBOTG_SW_RST		BIT(0)
#define AM3517_CPGMAC_SW_RST		BIT(1)
#define AM3517_VPFE_VBUSP_SW_RST	BIT(2)
#define AM3517_HECC_SW_RST		BIT(3)
#define AM3517_VPFE_PCLK_SW_RST		BIT(4)

#define AM3517_EMAC_CNTRL_OFFSET	(0x10000)
#define AM3517_EMAC_CNTRL_MOD_OFFSET	(0x0)
#define AM3517_EMAC_CNTRL_RAM_OFFSET	(0x20000)
#define AM3517_EMAC_MDIO_OFFSET		(0x30000)
#define AM3517_EMAC_CNTRL_RAM_SIZE	(0x2000)
#define AM3517_EMAC_RAM_ADDR		(AM3517_EMAC_BASE + \
						AM3517_EMAC_CNTRL_RAM_OFFSET)
#define AM3517_EMAC_HW_RAM_ADDR		(0x01E20000)

#define AM3517_HECC_SCC_HECC_OFFSET   (0x0)
#define AM3517_HECC_SCC_RAM_OFFSET    (0x3000)
#define AM3517_HECC_RAM_OFFSET        (0x3000)
#define AM3517_HECC_MBOX_OFFSET       (0x2000)
#define AM3517_HECC_INT_LINE          (0x0)
#define AM3517_HECC_VERSION           (0x1)


#endif /*  __ASM_ARCH_AM3517_H */
