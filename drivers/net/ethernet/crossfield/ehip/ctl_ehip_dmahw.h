/*
 * Crossfield Ethernet Hard IP DMA Driver for Intel FPGA SoCs
 * Copyright (C) 2020 Crossfield Technology LLC. All rights reserved.
 *
 * Contributors:
 *   Brett McMillian
 *
 * This driver is based on the Altera TSE mSGDMA driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __CTL_EHIP_DMAHW_H__
#define __CTL_EHIP_DMAHW_H__

/* eHIP DMA extended descriptor format */
struct ehip_dma_descriptor {
	u32 addr32;	/* data buffer source address low bits */
	u32 addr64;	/* data buffer destination address low bits */
	u32 length;		/* the number of bytes to transfer
				 * per descriptor
				 */
	u32 control;		/* characteristics of the transfer */
};

/* eHIP DMA descriptor control field bit definitions 
#define EHIP_DMA_DESC_CTL_SET_CH(x)	((x) & 0xff)
#define EHIP_DMA_DESC_CTL_GEN_SOP		BIT(8)
#define EHIP_DMA_DESC_CTL_GEN_EOP		BIT(9)
#define EHIP_DMA_DESC_CTL_PARK_READS	BIT(10)
#define EHIP_DMA_DESC_CTL_PARK_WRITES	BIT(11)
#define EHIP_DMA_DESC_CTL_END_ON_EOP	BIT(12)
#define EHIP_DMA_DESC_CTL_END_ON_LEN	BIT(13)
#define EHIP_DMA_DESC_CTL_TR_COMP_IRQ	BIT(14)
#define EHIP_DMA_DESC_CTL_EARLY_IRQ		BIT(15)
#define EHIP_DMA_DESC_CTL_TR_ERR_IRQ	(0xff << 16)
#define EHIP_DMA_DESC_CTL_EARLY_DONE	BIT(24)
#define EHIP_DMA_DESC_CTL_WAIT_WRESP	BIT(25)*/

/* Writing ‘1’ to the ‘go’ bit commits the entire descriptor into the
 * descriptor FIFO(s)
 */
#define EHIP_DMA_DESC_CTL_GO		BIT(0)

/* Tx buffer control flags 
#define EHIP_DMA_DESC_CTL_TX_FIRST	(EHIP_DMA_DESC_CTL_GEN_SOP |	\
					 EHIP_DMA_DESC_CTL_GO)

#define EHIP_DMA_DESC_CTL_TX_MIDDLE	(EHIP_DMA_DESC_CTL_GO)

#define EHIP_DMA_DESC_CTL_TX_LAST		(EHIP_DMA_DESC_CTL_GEN_EOP |	\
					 EHIP_DMA_DESC_CTL_TR_COMP_IRQ |	\
					 EHIP_DMA_DESC_CTL_GO)

#define EHIP_DMA_DESC_CTL_TX_SINGLE	(EHIP_DMA_DESC_CTL_GEN_SOP |	\
					 EHIP_DMA_DESC_CTL_GEN_EOP |	\
					 EHIP_DMA_DESC_CTL_TR_COMP_IRQ |	\
					 EHIP_DMA_DESC_CTL_GO)

#define EHIP_DMA_DESC_CTL_RX_SINGLE	(EHIP_DMA_DESC_CTL_END_ON_EOP |	\
					 EHIP_DMA_DESC_CTL_END_ON_LEN |	\
					 EHIP_DMA_DESC_CTL_TR_COMP_IRQ |	\
					 EHIP_DMA_DESC_CTL_EARLY_IRQ |	\
					 EHIP_DMA_DESC_CTL_TR_ERR_IRQ |	\
					 EHIP_DMA_DESC_CTL_GO)
*/

/* eHIP DMA response register map */
struct ehip_dma_response {
	u32 bytes_transferred;
	u32 status;	/* 15:8	- Early Termination
				 * 7:0	- Error
				 */
};

#define EHIP_DMA_SW_START_WATCHDOG_CNTR 10000

#define ehip_dma_respoffs(a) (offsetof(struct ehip_dma_response, a))
//#define ehip_dma_csroffs(a) (offsetof(struct ehip_dma_csr, a))
#define ehip_dma_descroffs(a) (offsetof(struct ehip_dma_descriptor, a))

/* eHIP DMA response register bit definitions */
#define EHIP_DMA_RESP_EARLY_TERM	BIT(8)
#define EHIP_DMA_RESP_ERR_MASK	0xFF

#endif /* __CTL_EHIP_DMAHW_H__*/
