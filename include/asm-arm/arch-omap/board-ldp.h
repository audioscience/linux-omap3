/*
 * linux/include/asm-arm/arch-omap/board-ldp.h
 *
 * Hardware definitions for TI OMAP3430 LDP board.
 *
 * Initial creation by Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_ARCH_OMAP_3430LDP_H
#define __ASM_ARCH_OMAP_3430LDP_H

extern void sdp_mmc_init(void);
extern void ldp_flash_init(void);

#define DEBUG_BASE			0x08000000  /* debug board */

#ifdef CONFIG_TWL4030_CORE

#define OMAP34XX_ETHR_START		DEBUG_BASE
#define LDP_SMC911X_CS		1
#define LDP_SMC911X_GPIO		152

#define TWL4030_IRQNUM INT_34XX_SYS_NIRQ
/* TWL4030 Primary Interrupt Handler (PIH) interrupts */
#define	IH_TWL4030_BASE		IH_BOARD_BASE
#define	IH_TWL4030_END		(IH_TWL4030_BASE+8)

#define IH_TWL4030_PWRBASE     (IH_TWL4030_END)
#define IH_TWL4030_PWRBASE_END (IH_TWL4030_PWRBASE+8)

#ifdef CONFIG_TWL4030_GPIO
/* TWL4030 GPIO Interrupts */
#define IH_TWL4030_GPIO_BASE	(IH_TWL4030_END)
#define IH_TWL4030_GPIO_END	(IH_TWL4030_BASE+18)
#define NR_IRQS			(IH_TWL4030_GPIO_END)
#else
#define NR_IRQS			(IH_TWL4030_END)
#endif

#endif

#endif /*  __ASM_ARCH_OMAP_3430LDP_H */

