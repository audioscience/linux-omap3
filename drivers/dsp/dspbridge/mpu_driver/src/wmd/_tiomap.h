/*
 * dspbridge/src/wmd/linux/omap/common/_tiomap.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 *  ======== _tiomap.h ========
 *  Description:
 *      Definitions and types private to this WMD.
 *
 *! Revision History
 *! ================
 *! 12-Apr-2004 hp:  Removed the L4 entry of Dsp DMA , not needed
 *! 08-Mar-2004 sb:  Added the Dynamic Memory Mapping feature - PgTableAttrs
 *! 21-Mar-2003 sb:  Added numTLBEntries to struct WMD_DEV_CONTEXT
 *! 20-Feb-2003 vp:  Ported to Linux platform.
 *! 30-Jul-2002 rr:  Modified TBC_ID_VALUE to 0xb47002f.
 *! 10-May-2002 sg:  Added ARM_SYSST_OFFSET and IDLE_DSP_MASK.
 *! 14-Mar-2002 rr:  Added Boot vector field to pDevContext.
 *!                  Added TBC Register defines and value for OMAP1510.
 *! 12-Jan-2002 ag:  wIntrVal2Dsp Added.
 *! 05-Nov-2001 kc:  Added MMU fault related definitions.
 *! 03-Aug-2001 ag   Added TLB table for OEM cofig of DSP-MMU.
 *! 23-Aug-2001 rr:  Added API_SIZE define.
 *! 16-Aug-2001 rr:  Added dwDspExtBaseAddr to access the Ext memory.
 *! 24-Jul-2001 ag:  Added Internal SRAM MMU table entry.
 *! 27-Jun-2001 rr:  Name changed to _tihelen.h.
 *! 27-Jun-2001 ag:  Added dwIntAddr in WMD_DEV_CONTEXT used for MB INTRs msk.
 *! 07-May-2001 ag:  Added DSP Clock Module CLKM2.
 *!                  Added TIHEL_SDRAMPHYSBASE used for DSP MMU init.
 *!                  Added ClearBit() MACRO.
 *! 18-Jan-2001 rr:  Created
 */

#ifndef _TIOMAP_
#define _TIOMAP_

#include <devdefs.h>
#include <hw_defs.h>
#include <hw_mbox.h>
#include <wmdioctl.h>		/* for WMDIOCTL_EXTPROC defn */
#include "sync.h"
#include "clk.h"

struct MAP_L4PERIPHERAL {
	DWORD physAddr;
	DWORD dspVirtAddr;
} ;

#define ARM_MAILBOX_START               0xfffcf000
#define ARM_MAILBOX_LENGTH              0x800

/* New Registers in OMAP3.1 */

#define TESTBLOCK_ID_START              0xfffed400
#define TESTBLOCK_ID_LENGTH             0xff

/* ID Returned by OMAP1510 */
#define TBC_ID_VALUE                    0xB47002F

#define SPACE_LENGTH                    0x2000
#define API_CLKM_DPLL_DMA               0xfffec000
#define ARM_INTERRUPT_OFFSET            0xb00

#define BIOS_24XX

#define L4_PERIPHERAL_NULL          0x0
#define DSPVA_PERIPHERAL_NULL       0x0

#define MAX_LOCK_TLB_ENTRIES 15

#define L4_PERIPHERAL_PRM        0x48306000  /*PRM L4 Peripheral */
#define DSPVA_PERIPHERAL_PRM     0x1181e000
#define L4_PERIPHERAL_SCM        0x48002000  /*SCM L4 Peripheral */
#define DSPVA_PERIPHERAL_SCM     0x1181f000
#define L4_PERIPHERAL_MMU        0x5D000000  /*MMU L4 Peripheral */
#define DSPVA_PERIPHERAL_MMU     0x11820000
#define L4_PERIPHERAL_CM        0x48004000       /* Core L4, Clock Management */
#define DSPVA_PERIPHERAL_CM     0x1181c000
#define L4_PERIPHERAL_PER        0x48005000       /*  PER */
#define DSPVA_PERIPHERAL_PER     0x1181d000

#define L4_PERIPHERAL_GPIO1       0x48310000
#define DSPVA_PERIPHERAL_GPIO1    0x11809000
#define L4_PERIPHERAL_GPIO2       0x49050000
#define DSPVA_PERIPHERAL_GPIO2    0x1180a000
#define L4_PERIPHERAL_GPIO3       0x49052000
#define DSPVA_PERIPHERAL_GPIO3    0x1180b000
#define L4_PERIPHERAL_GPIO4       0x49054000
#define DSPVA_PERIPHERAL_GPIO4    0x1180c000
#define L4_PERIPHERAL_GPIO5       0x49056000
#define DSPVA_PERIPHERAL_GPIO5    0x1180d000

#define L4_PERIPHERAL_IVA2WDT      0x49030000
#define DSPVA_PERIPHERAL_IVA2WDT   0x1180e000

#define L4_PERIPHERAL_DISPLAY     0x48050000
#define DSPVA_PERIPHERAL_DISPLAY  0x1180f000

#define L4_PERIPHERAL_SSI         0x48058000
#define DSPVA_PERIPHERAL_SSI      0x11804000
#define L4_PERIPHERAL_GDD         0x48059000
#define DSPVA_PERIPHERAL_GDD      0x11805000
#define L4_PERIPHERAL_SS1         0x4805a000
#define DSPVA_PERIPHERAL_SS1      0x11806000
#define L4_PERIPHERAL_SS2         0x4805b000
#define DSPVA_PERIPHERAL_SS2      0x11807000

#define L4_PERIPHERAL_CAMERA      0x480BC000
#define DSPVA_PERIPHERAL_CAMERA   0x11819000

#define L4_PERIPHERAL_SDMA        0x48056000
#define DSPVA_PERIPHERAL_SDMA     0x11810000 /*0x1181d000 conflicts with PER */

#define L4_PERIPHERAL_UART1             0x4806a000
#define DSPVA_PERIPHERAL_UART1          0x11811000
#define L4_PERIPHERAL_UART2             0x4806c000
#define DSPVA_PERIPHERAL_UART2          0x11812000
#define L4_PERIPHERAL_UART3             0x49020000
#define DSPVA_PERIPHERAL_UART3    0x11813000

#define L4_PERIPHERAL_MCBSP1      0x48074000
#define DSPVA_PERIPHERAL_MCBSP1   0x11814000
#define L4_PERIPHERAL_MCBSP2      0x49022000
#define DSPVA_PERIPHERAL_MCBSP2   0x11815000
#define L4_PERIPHERAL_MCBSP3      0x49024000
#define DSPVA_PERIPHERAL_MCBSP3   0x11816000
#define L4_PERIPHERAL_MCBSP4      0x49026000
#define DSPVA_PERIPHERAL_MCBSP4   0x11817000
#define L4_PERIPHERAL_MCBSP5      0x48096000
#define DSPVA_PERIPHERAL_MCBSP5   0x11818000

#define L4_PERIPHERAL_GPTIMER5    0x49038000
#define DSPVA_PERIPHERAL_GPTIMER5 0x11800000
#define L4_PERIPHERAL_GPTIMER6    0x4903a000
#define DSPVA_PERIPHERAL_GPTIMER6 0x11801000
#define L4_PERIPHERAL_GPTIMER7    0x4903c000
#define DSPVA_PERIPHERAL_GPTIMER7 0x11802000
#define L4_PERIPHERAL_GPTIMER8    0x4903e000
#define DSPVA_PERIPHERAL_GPTIMER8 0x11803000

#define L4_PERIPHERAL_SPI1      0x48098000
#define DSPVA_PERIPHERAL_SPI1   0x1181a000
#define L4_PERIPHERAL_SPI2      0x4809a000
#define DSPVA_PERIPHERAL_SPI2   0x1181b000

#define L4_PERIPHERAL_MBOX        0x48094000
#define DSPVA_PERIPHERAL_MBOX     0x11808000

#define PM_GRPSEL_BASE 			0x48307000
#define DSPVA_GRPSEL_BASE 		0x11821000

/* define a static array with L4 mappings */
static const struct MAP_L4PERIPHERAL L4PeripheralTable[] = {
	{L4_PERIPHERAL_MBOX, DSPVA_PERIPHERAL_MBOX},
	{L4_PERIPHERAL_SCM, DSPVA_PERIPHERAL_SCM},
	{L4_PERIPHERAL_MMU, DSPVA_PERIPHERAL_MMU},
	{L4_PERIPHERAL_GPTIMER5, DSPVA_PERIPHERAL_GPTIMER5},
	{L4_PERIPHERAL_GPTIMER6, DSPVA_PERIPHERAL_GPTIMER6},
	{L4_PERIPHERAL_GPTIMER7, DSPVA_PERIPHERAL_GPTIMER7},
	{L4_PERIPHERAL_GPTIMER8, DSPVA_PERIPHERAL_GPTIMER8},
	{L4_PERIPHERAL_GPIO1, DSPVA_PERIPHERAL_GPIO1},
	{L4_PERIPHERAL_GPIO2, DSPVA_PERIPHERAL_GPIO2},
	{L4_PERIPHERAL_GPIO3, DSPVA_PERIPHERAL_GPIO3},
	{L4_PERIPHERAL_GPIO4, DSPVA_PERIPHERAL_GPIO4},
	{L4_PERIPHERAL_GPIO5, DSPVA_PERIPHERAL_GPIO5},
	{L4_PERIPHERAL_IVA2WDT, DSPVA_PERIPHERAL_IVA2WDT},
	{L4_PERIPHERAL_DISPLAY, DSPVA_PERIPHERAL_DISPLAY},
	{L4_PERIPHERAL_SSI, DSPVA_PERIPHERAL_SSI},
	{L4_PERIPHERAL_GDD, DSPVA_PERIPHERAL_GDD},
	{L4_PERIPHERAL_SS1, DSPVA_PERIPHERAL_SS1},
	{L4_PERIPHERAL_SS2, DSPVA_PERIPHERAL_SS2},
	{L4_PERIPHERAL_UART1, DSPVA_PERIPHERAL_UART1},
	{L4_PERIPHERAL_UART2, DSPVA_PERIPHERAL_UART2},
	{L4_PERIPHERAL_UART3, DSPVA_PERIPHERAL_UART3},
	{L4_PERIPHERAL_MCBSP1, DSPVA_PERIPHERAL_MCBSP1},
	{L4_PERIPHERAL_MCBSP2, DSPVA_PERIPHERAL_MCBSP2},
	{L4_PERIPHERAL_MCBSP3, DSPVA_PERIPHERAL_MCBSP3},
	{L4_PERIPHERAL_MCBSP4, DSPVA_PERIPHERAL_MCBSP4},
	{L4_PERIPHERAL_MCBSP5, DSPVA_PERIPHERAL_MCBSP5},
	{L4_PERIPHERAL_CAMERA, DSPVA_PERIPHERAL_CAMERA},
	{L4_PERIPHERAL_SPI1, DSPVA_PERIPHERAL_SPI1},
	{L4_PERIPHERAL_SPI2, DSPVA_PERIPHERAL_SPI2},
	{L4_PERIPHERAL_PRM, DSPVA_PERIPHERAL_PRM},
	{L4_PERIPHERAL_CM, DSPVA_PERIPHERAL_CM},
	{L4_PERIPHERAL_PER, DSPVA_PERIPHERAL_PER},
/*	{L4_PERIPHERAL_SDMA, DSPVA_PERIPHERAL_SDMA},*/
	{PM_GRPSEL_BASE, DSPVA_GRPSEL_BASE},
	{L4_PERIPHERAL_NULL, DSPVA_PERIPHERAL_NULL}
};

/* ============================================================================

   15         10                  0
   ---------------------------------
   |0|0|1|0|0|0|c|c|c|i|i|i|i|i|i|i|
   ---------------------------------
   |  (class)  | (module specific) |

   where  c -> Externel Clock Command: Clk & Autoidle Disable/Enable
   i -> External Clock ID Timers 5,6,7,8, McBSP1,2 and WDT3

   ========================================================================== */

/* ============================================================================
   name    MBX_PM_CLK_IDMASK

  desc    DSP External clock id mask.
 ===========================================================================*/
#define MBX_PM_CLK_IDMASK   0x7F

/* ============================================================================
 name    MBX_PM_CLK_CMDSHIFT

 desc    DSP External clock command shift.
 =========================================================================== */
#define MBX_PM_CLK_CMDSHIFT 7

/* ============================================================================
 name    MBX_PM_CLK_CMDMASK

 desc    DSP External clock command mask.
 =========================================================================== */
#define MBX_PM_CLK_CMDMASK 7

/* ============================================================================
  name    MBX_PM_MAX_RESOURCES

  desc    CORE 1 Clock resources.
 =========================================================================== */
#define MBX_CORE1_RESOURCES 7

/* ============================================================================
 name    MBX_PM_MAX_RESOURCES

 desc    CORE 2 Clock Resources.
 =========================================================================== */
#define MBX_CORE2_RESOURCES 1

/* ============================================================================
 name    MBX_PM_MAX_RESOURCES

  desc    TOTAL Clock Reosurces.
 =========================================================================== */
#define MBX_PM_MAX_RESOURCES 11

/*  Power Management Commands*/
typedef enum {
	BPWR_DisableClock = 0,
	BPWR_EnableClock,
	BPWR_DisableAutoIdle,
	BPWR_EnableAutoIdle
} BPWR_ExtClockCmd;

/* OMAP242x specific resources*/
typedef enum {
	BPWR_GPTimer5 = 0x10,
	BPWR_GPTimer6,
	BPWR_GPTimer7,
	BPWR_GPTimer8,
	BPWR_WDTimer3,
	BPWR_MCBSP1,
	BPWR_MCBSP2,
	BPWR_MCBSP3,
	BPWR_MCBSP4,
	BPWR_MCBSP5,
	BPWR_SSI = 0x20
} BPWR_ExtClockId;

static const DWORD BPWR_CLKID[] = {
	(DWORD) BPWR_GPTimer5,
	(DWORD) BPWR_GPTimer6,
	(DWORD) BPWR_GPTimer7,
	(DWORD) BPWR_GPTimer8,
	(DWORD) BPWR_WDTimer3,
	(DWORD) BPWR_MCBSP1,
	(DWORD) BPWR_MCBSP2,
	(DWORD) BPWR_MCBSP3,
	(DWORD) BPWR_MCBSP4,
	(DWORD) BPWR_MCBSP5,
	(DWORD) BPWR_SSI
};

struct BPWR_Clk_t {
	DWORD clkId;
	SERVICES_ClkId funClk;
	SERVICES_ClkId intClk;
} ;

static const struct BPWR_Clk_t BPWR_Clks[] = {
	{(DWORD) BPWR_GPTimer5, SERVICESCLK_gpt5_fck, SERVICESCLK_gpt5_ick},
	{(DWORD) BPWR_GPTimer6, SERVICESCLK_gpt6_fck, SERVICESCLK_gpt6_ick},
	{(DWORD) BPWR_GPTimer7, SERVICESCLK_gpt7_fck, SERVICESCLK_gpt7_ick},
	{(DWORD) BPWR_GPTimer8, SERVICESCLK_gpt8_fck, SERVICESCLK_gpt8_ick},
	{(DWORD) BPWR_WDTimer3, SERVICESCLK_wdt3_fck, SERVICESCLK_wdt3_ick},
	{(DWORD) BPWR_MCBSP1, SERVICESCLK_mcbsp1_fck, SERVICESCLK_mcbsp1_ick},
	{(DWORD) BPWR_MCBSP2, SERVICESCLK_mcbsp2_fck, SERVICESCLK_mcbsp2_ick},
	{(DWORD) BPWR_MCBSP3, SERVICESCLK_mcbsp3_fck, SERVICESCLK_mcbsp3_ick},
	{(DWORD) BPWR_MCBSP4, SERVICESCLK_mcbsp4_fck, SERVICESCLK_mcbsp4_ick},
	{(DWORD) BPWR_MCBSP5, SERVICESCLK_mcbsp5_fck, SERVICESCLK_mcbsp5_ick},
	{(DWORD) BPWR_SSI, SERVICESCLK_ssi_fck, SERVICESCLK_ssi_ick}
};

/* Interrupt Register Offsets */
#define INTH_IT_REG_OFFSET              0x00	/* Interrupt register offset  */
#define INTH_MASK_IT_REG_OFFSET         0x04	/* Mask Interrupt reg offset  */

#define   DSP_MAILBOX1_INT              10

/*  ----------------------------------------------------
 *  INTH_InterruptKind_t                               -
 *  Identify the kind of interrupt: either FIQ/IRQ   -
 *  ----------------------------------------------------
 */
typedef enum {
	INTH_IRQ = 0,
	INTH_FIQ = 1
} INTH_InterruptKind_t;

typedef enum {
	FALLING_EDGE_SENSITIVE = 0,
	LOW_LEVEL_SENSITIVE = 1
} INTH_SensitiveEdge_t;

/*  ------------------------------------------------------
 *  Bit definition of  Interrupt  Level  Registers       -
 *  ------------------------------------------------------
 */

/* Mail Box defines */
#define MB_ARM2DSP1_REG_OFFSET          0x00

#define MB_ARM2DSP1B_REG_OFFSET         0x04

#define MB_DSP2ARM1B_REG_OFFSET         0x0C

#define MB_ARM2DSP1_FLAG_REG_OFFSET     0x18

#define MB_ARM2DSP_FLAG                 0x0001

#define MBOX_ARM2DSP HW_MBOX_ID_0
#define MBOX_DSP2ARM HW_MBOX_ID_1
#define MBOX_ARM HW_MBOX_U0_ARM
#define MBOX_DSP HW_MBOX_U1_DSP1

/* Moved to dbtypes.h */
/*typedef volatile unsigned short REG_UWORD16 ;*/

#define ENABLE                          TRUE
#define DISABLE                         FALSE

#define HIGH_LEVEL                      TRUE
#define LOW_LEVEL                       FALSE

/* Macro's */
#define REG16(A)    (*(REG_UWORD16 *)(A))

#define ClearBit(reg, mask)             (reg &= ~mask)
#define SetBit(reg, mask)               (reg |= mask)

#define SetGroupBits16(reg, position, width, value) \
    reg &= ~((0xFFFF >> (16 - (width))) << (position)) ; \
    reg |= ((value & (0xFFFF >> (16 - (width)))) \
    << (position))

#define ClearBitIndex(reg, index)   (reg &= ~(1 << (index)))

/* Attributes used to manage the DSP MMU page tables */
/*typedef struct PgTableAttrs *pPgTableAttrs;*/

/* This mini driver's device context: */
struct WMD_DEV_CONTEXT {
	struct DEV_OBJECT *hDevObject;	/* Handle to WCD device object. */
	DWORD dwDspBaseAddr;	/* Arm's API to DSP virtual base addr */
	/*
	 * DSP External memory prog address as seen virtually by the OS on
	 * the host side.
	 */
	DWORD dwDspExtBaseAddr;	/* See the comment above        */
	DWORD dwAPIRegBase;	/* API memory mapped registers  */
	DWORD dwDSPMmuBase;	/* DSP MMU Mapped registers     */
	DWORD dwMailBoxBase;	/* Mail box mapped registers    */
	DWORD dwAPIClkBase;	/* CLK Registers                */
	DWORD dwDSPClkM2Base;	/* DSP Clock Module m2          */
	DWORD dwPublicRhea;	/* Pub Rhea                     */
	DWORD dwIntAddr;	/* MB INTR reg                  */
	DWORD dwTCEndianism;	/* TC Endianism register        */
	DWORD dwTestBase;	/* DSP MMU Mapped registers     */
	DWORD dwSelfLoop;	/* Pointer to the selfloop      */
	DWORD dwDSPStartAdd;	/* API Boot vector              */
	DWORD dwInternalSize;	/* Internal memory size         */

	/*
	 * Processor specific info is set when prog loaded and read from DCD.
	 * [See WMD_BRD_Ctrl()]  PROC info contains DSP-MMU TLB entries.
	 */
	/* DMMU TLB entries */
	struct WMDIOCTL_EXTPROC aTLBEntry[WMDIOCTL_NUMOFMMUTLB];
	DWORD dwBrdState;	/* Last known board state.      */
	ULONG ulIntMask;	/* int mask                     */
	USHORT ioBase;	/* Board I/O base               */
	WORD wIntrVal2Dsp;	/* MBX value to DSP. See mbx_sh.h */
	ULONG numTLBEntries;	/* DSP MMU TLB entry counter    */
	ULONG fixedTLBEntries;	/* Fixed DSPMMU TLB entry count */

	/* TC Settings */
	BOOL tcWordSwapOn;	/* Traffic Controller Word Swap */
	struct PgTableAttrs *pPtAttrs;
	UWORD32 uDspPerClks;
} ;

	/*
	 * ======== WMD_TLB_DspVAToMpuPA ========
	 * Purpose:
	 *     Given a DSP virtual address, traverse the page table and return
	 *     a corresponding MPU physical address and size.
	 */
extern DSP_STATUS WMD_TLB_DspVAToMpuPA(struct WMD_DEV_CONTEXT *pDevContext,
				       IN ULONG ulVirtAddr,
				       OUT ULONG *ulPhysAddr,
				       OUT ULONG *sizeTlb);

#endif				/* _TIOMAP_ */

