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

/* eHIP DMA descriptor control field bit definitions */
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
#define EHIP_DMA_DESC_CTL_WAIT_WRESP	BIT(25)

/* Writing ‘1’ to the ‘go’ bit commits the entire descriptor into the
 * descriptor FIFO(s)
 */
#define EHIP_DMA_DESC_CTL_GO		BIT(31)

/* Tx buffer control flags */
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

/* eHIP DMA extended descriptor stride definitions */
#define EHIP_DMA_DESC_TX_STRIDE		(0x00010001)
#define EHIP_DMA_DESC_RX_STRIDE		(0x00010001)

/* eHIP DMA dispatcher control and status register map */
struct ehip_dma_csr {
	u32 status;		/* Read/Clear */
	u32 control;		/* Read/Write */
	u32 rw_fill_level;	/* bit 31:16 - write fill level
				 * bit 15:0  - read fill level
				 */
	u32 resp_fill_level;	/* bit 15:0 */
	u32 rw_seq_num;		/* bit 31:16 - write sequence number
				 * bit 15:0  - read sequence number
				 */
	u32 comp_cfg1;		/* Component Configuration 1 */
	u32 comp_cfg2;		/* Component Configuration 2 */
	u32 comp_type_ver;	/* bit 15:8 - Component Type 
				 * 		Fixed: 0xDA
				 * bit 7:0  - Component Version
				 * 		Fixed: 0x01
				 */
};

/* eHIP DMA CSR status register bit definitions */
#define EHIP_DMA_CSR_STAT_BUSY			BIT(0)
#define EHIP_DMA_CSR_STAT_DESC_BUF_EMPTY		BIT(1)
#define EHIP_DMA_CSR_STAT_DESC_BUF_FULL		BIT(2)
#define EHIP_DMA_CSR_STAT_RESP_BUF_EMPTY		BIT(3)
#define EHIP_DMA_CSR_STAT_RESP_BUF_FULL		BIT(4)
#define EHIP_DMA_CSR_STAT_STOPPED			BIT(5)
#define EHIP_DMA_CSR_STAT_RESETTING		BIT(6)
#define EHIP_DMA_CSR_STAT_STOPPED_ON_ERR		BIT(7)
#define EHIP_DMA_CSR_STAT_STOPPED_ON_EARLY	BIT(8)
#define EHIP_DMA_CSR_STAT_IRQ			BIT(9)
#define EHIP_DMA_CSR_STAT_MASK			0x3FF
#define EHIP_DMA_CSR_STAT_MASK_WITHOUT_IRQ	0x1FF

#define EHIP_DMA_CSR_STAT_BUSY_GET(v)		GET_BIT_VALUE(v, 0)
#define EHIP_DMA_CSR_STAT_DESC_BUF_EMPTY_GET(v)	GET_BIT_VALUE(v, 1)
#define EHIP_DMA_CSR_STAT_DESC_BUF_FULL_GET(v)	GET_BIT_VALUE(v, 2)
#define EHIP_DMA_CSR_STAT_RESP_BUF_EMPTY_GET(v)	GET_BIT_VALUE(v, 3)
#define EHIP_DMA_CSR_STAT_RESP_BUF_FULL_GET(v)	GET_BIT_VALUE(v, 4)
#define EHIP_DMA_CSR_STAT_STOPPED_GET(v)		GET_BIT_VALUE(v, 5)
#define EHIP_DMA_CSR_STAT_RESETTING_GET(v)	GET_BIT_VALUE(v, 6)
#define EHIP_DMA_CSR_STAT_STOPPED_ON_ERR_GET(v)	GET_BIT_VALUE(v, 7)
#define EHIP_DMA_CSR_STAT_STOPPED_ON_EARLY_GET(v)	GET_BIT_VALUE(v, 8)
#define EHIP_DMA_CSR_STAT_IRQ_GET(v)		GET_BIT_VALUE(v, 9)

/* eHIP DMA CSR control register bit definitions */
#define EHIP_DMA_CSR_CTL_STOP		BIT(0)
#define EHIP_DMA_CSR_CTL_RESET		BIT(1)
#define EHIP_DMA_CSR_CTL_STOP_ON_ERR	BIT(2)
#define EHIP_DMA_CSR_CTL_STOP_ON_EARLY	BIT(3)
#define EHIP_DMA_CSR_CTL_GLOBAL_INTR	BIT(4)
#define EHIP_DMA_CSR_CTL_STOP_DESCS	BIT(5)

/* eHIP DMA CSR fill level bits */
#define EHIP_DMA_CSR_WR_FILL_LEVEL_GET(v)		(((v) & 0xffff0000) >> 16)
#define EHIP_DMA_CSR_RD_FILL_LEVEL_GET(v)		((v) & 0x0000ffff)
#define EHIP_DMA_CSR_RESP_FILL_LEVEL_GET(v)	((v) & 0x0000ffff)

/* eHIP DMA response register map */
struct ehip_dma_response {
	u32 bytes_transferred;
	u32 status;	/* 15:8	- Early Termination
				 * 7:0	- Error
				 */
};

#define ehip_dma_respoffs(a) (offsetof(struct ehip_dma_response, a))
#define ehip_dma_csroffs(a) (offsetof(struct ehip_dma_csr, a))
#define ehip_dma_descroffs(a) (offsetof(struct ehip_dma_descriptor, a))

/* eHIP DMA response register bit definitions */
#define EHIP_DMA_RESP_EARLY_TERM	BIT(8)
#define EHIP_DMA_RESP_ERR_MASK	0xFF

/* eHIP DMA CSR Component Configuration 1 */
#define EHIP_DMA_CSR_COMP_CFG1_BURST_ENABLE		BIT(0)
#define EHIP_DMA_CSR_COMP_CFG1_BURST_WRAPPPING		BIT(1)
#define EHIP_DMA_CSR_COMP_CFG1_CHANNEL_ENABLE		BIT(2)
#define EHIP_DMA_CSR_COMP_CFG1_CHANNEL_WIDTH(x)		(((x) >> 3) & 0x7)
#define EHIP_DMA_CSR_COMP_CFG1_DATA_FIFO_DEPTH(x)		(((x) >> 6) & 0xf)
#define EHIP_DMA_CSR_COMP_CFG1_DATA_WIDTH(x)		(((x) >> 10) & 0x7)
#define EHIP_DMA_CSR_COMP_CFG1_DESC_FIFO_DEPTH(x)		(((x) >> 13) & 0x7)
#define EHIP_DMA_CSR_COMP_CFG1_DMA_MODE(x)		(((x) >> 16) & 0x3)
#define EHIP_DMA_CSR_COMP_CFG1_ENHANCED_FEAT		BIT(18)
#define EHIP_DMA_CSR_COMP_CFG1_ERROR_ENABLE		BIT(19)
#define EHIP_DMA_CSR_COMP_CFG1_ERROR_WIDTH(x)		(((x) >> 20) & 0x7)
#define EHIP_DMA_CSR_COMP_CFG1_MAX_BURST_COUNT(x)		(((x) >> 23) & 0xf)
#define EHIP_DMA_CSR_COMP_CFG1_MAX_BYTE(x)		(((x) >> 27) & 0x1f)

/* eHIP DMA CSR Component Configuration 2 */
#define EHIP_DMA_CSR_COMP_CFG2_STRIDE_ENABLE		BIT(0)
#define EHIP_DMA_CSR_COMP_CFG2_MAX_STRIDE(x)		(((x) >> 1) & 0x00007fff)
#define EHIP_DMA_CSR_COMP_CFG2_PACKET_ENABLE		BIT(16)
#define EHIP_DMA_CSR_COMP_CFG2_PREFETCHER_ENABLE		BIT(17)
#define EHIP_DMA_CSR_COMP_CFG2_PROG_BURST_ENABLE		BIT(18)
#define EHIP_DMA_CSR_COMP_CFG2_RESPONSE_PORT(x)		(((x) >> 19) & 0x3)
#define EHIP_DMA_CSR_COMP_CFG2_TRANSFER_TYPE(x)		(((x) >> 21) & 0x3)


#endif /* __CTL_EHIP_DMAHW_H__*/
