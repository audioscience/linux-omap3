/*
 * linux/arch/arm/mach-omap2/control3517.h
 * 
 * Definitions for OMAP3505/3517 SoC Control Module (SCM)
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_CONTROL3517_H
#define __ASM_ARCH_CONTROL3517_H

#include <linux/list.h> /* should have been taken care by clock.h */
#include "clock.h"

/* OMAP3505/3517 specific SCM register offsets (from SCM base) */

#define OMAP3517_CONTROL_MSUSPENDMUX_6  0x02A8
#define OMAP3517_CONF0                  0x0580
#define OMAP3517_CONF1                  0x0584
#define OMAP3517_CONF2                  0x0588
#define OMAP3517_CBA_PRIORITY           0x0590
#define OMAP3517_LVL_INT_CLEAR          0x0594
#define OMAP3517_IP_SW_RESET            0x0598
#define OMAP3517_IP_CLK_CTRL            0x059C

/* OMAP3505/3517 IP clock control register bit definitions */
#define USBOTG_VBUSP_CLK_EN          (1 << 0)
#define USBOTG_VBUSP_CLK_EN_SHIFT    0
#define CPGMAC_VBUSP_CLK_EN          (1 << 1)
#define CPGMAC_VBUSP_CLK_EN_SHIFT    1
#define VPFE_VBUSP_CLK_EN            (1 << 2)
#define VPFE_VBUSP_CLK_EN_SHIFT      2
#define HECC_VBUSP_CLK_EN            (1 << 3)
#define HECC_VBUSP_CLK_EN_SHIFT      3
#define USBOTG_VBUSP_CLK_EN_ACK      (1 << 4)
#define CPGMAC_VBUSP_CLK_EN_ACK      (1 << 5)
#define VPFE_VBUSP_CLK_EN_ACK        (1 << 6)
#define HECC_VBUSP_CLK_EN_ACK        (1 << 7)
#define USBOTG_FCLK_EN               (1 << 8)
#define USBOTG_FCLK_EN_SHIFT         8
#define CPGMAC_FCLK_EN               (1 << 9)
#define CPGMAC_FCLK_EN_SHIFT         9
#define VPFE_FCLK_EN                 (1 << 10)
#define VPFE_FCLK_EN_SHIFT           10
	
#define CLK_ENABLE					 0x1
#define CLK_DISABLE                  0x0
	
/* OMAP3505/3517 IP reset control register bit definitions */
#define USBOTG_SW_RST                (1 << 0)
#define CPGMAC_SW_RST                (1 << 1)
#define VPFE_VBUSP_SW_RST            (1 << 2)
#define HECC_SW_RST                  (1 << 3)
#define VPFE_PCLK_SW_RST             (1 << 4)
	
#define IN_RESET					 0x1
#define OUT_OF_RESET                 0x0

/* Interrupt clear register bit definitions */
#define CPGMAC_MISC_INT              (1 << 0)
#define CPGMAC_RX_INT                (1 << 1)
#define CPGMAC_RX_THRESH_INT         (1 << 2)
#define CPGMAC_TX_INT                (1 << 3)
#define USBOTG_USB_INT               (1 << 4)
#define VPFE_VD0_INT                 (1 << 5)
#define VPFE_VD1_INT                 (1 << 6)
#define VPFE_VD2_INT                 (1 << 7)

extern void omap3517_ipss_reset_ctrl(u16 module_bit, char reset_ctrl);
extern void omap3517_ipss_clk_ctrl(u32 reg_offset, u8 clk_shift, char clk_ctrl);
extern int omap3517_ipss_clk_enable(struct clk* clk);
extern void omap3517_ipss_clk_disable(struct clk* clk);
extern void omap3517_ipss_int_status_clr(u16 int_clr_bit);

#endif
