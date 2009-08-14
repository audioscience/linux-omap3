/* arch/arm/plat-omap/include/mach/omap3517.h
 *
 * Hardware definitions for TI OMAP3517 processor.
 *
 * Cleanup for Linux-2.6 by Dirk Behme <dirk.behme@de.bosch.com>
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
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_ARCH_OMAP3517_H
#define __ASM_ARCH_OMAP3517_H

/*
 * ----------------------------------------------------------------------------
 * Base addresses
 * ----------------------------------------------------------------------------
 */

#define OMAP3517_CONF0			0x48002580
#define OMAP3517_CONF1			0x48002584
#define OMAP3517_CONF2			0x48002588
#define OMAP3517_CBA_PRIORITY		0x48002590
#define OMAP3517_LVL_INTR_CLEAR         0x48002594
#define OMAP3517_IP_SW_RESET		0x48002598
#define OMAP3517_IPSS_CLK_CTRL		0x4800259C


#define OMAP3517_CPGMAC_TX_PULSE_CLR    ( 1 << 3)
#define OMAP3517_CPGMAC_RX_THRESH_CLR   ( 1 << 2)
#define OMAP3517_CPGMAC_RX_PULSE_CLR    ( 1 << 1)
#define OMAP3517_CPGMAC_MISC_PULSE_CLR  ( 1 << 0)

#define OMAP3517_EMAC_HW_RAM_ADDR	(0x01E20000)

#endif /*  __ASM_ARCH_OMAP3517_H */

