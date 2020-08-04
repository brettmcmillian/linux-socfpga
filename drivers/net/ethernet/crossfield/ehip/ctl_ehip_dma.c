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

#include <linux/netdevice.h>
#include "ctl_ehip_utils.h"
#include "ctl_ehip.h"
#include "rx_ehip_dma_csr.h"
#include "tx_ehip_dma_csr.h"
#include "ctl_ehip_dma.h"
#include "ctl_ehip_dmahw.h"

/* No initialization work to do for eHIP DMA */
int ctl_ehip_dma_initialize(struct ctl_ehip_private *priv)
{
	return 0;
}

void ctl_ehip_dma_uninitialize(struct ctl_ehip_private *priv)
{
}

void ctl_ehip_dma_start_rxdma(struct ctl_ehip_private *priv)
{
	ctl_ehip_clear_bit(&priv->rx_dma_csr->start,
			RX_EHIP_DMA_CSR_START_MASK);
}

void ctl_ehip_dma_start_txdma(struct ctl_ehip_private *priv)
{
	ctl_ehip_clear_bit(&priv->tx_dma_csr->start,
			TX_EHIP_DMA_CSR_START_MASK);
}

void ctl_ehip_dma_reset(struct ctl_ehip_private *priv)
{
	//No software reset for HLS core
}

void ctl_ehip_dma_disable_rxirq(struct ctl_ehip_private *priv)
{
	ctl_ehip_clear_bit(&priv->rx_dma_csr->interrupt_enable,
		      RX_EHIP_DMA_CSR_INTERRUPT_ENABLE_MASK);
}

void ctl_ehip_dma_enable_rxirq(struct ctl_ehip_private *priv)
{
	ctl_ehip_set_bit(&priv->rx_dma_csr->interrupt_enable,
		    RX_EHIP_DMA_CSR_INTERRUPT_ENABLE_MASK);
}

void ctl_ehip_dma_disable_txirq(struct ctl_ehip_private *priv)
{
	ctl_ehip_clear_bit(&priv->tx_dma_csr->interrupt_enable,
		      TX_EHIP_DMA_CSR_INTERRUPT_ENABLE_MASK);
}

void ctl_ehip_dma_enable_txirq(struct ctl_ehip_private *priv)
{
	ctl_ehip_set_bit(&priv->tx_dma_csr->interrupt_enable,
		    TX_EHIP_DMA_CSR_INTERRUPT_ENABLE_MASK);
}

void ctl_ehip_dma_clear_rxirq(struct ctl_ehip_private *priv)
{
	writel(RX_EHIP_DMA_CSR_INTERRUPT_STATUS_MASK, &priv->rx_dma_csr->status);
}

void ctl_ehip_dma_clear_txirq(struct ctl_ehip_private *priv)
{
	writel(RX_EHIP_DMA_CSR_INTERRUPT_STATUS_MASK, &priv->tx_dma_csr->status);
}

/* return 0 to indicate transmit is pending */
int ctl_ehip_dma_tx_buffer(struct ctl_ehip_private *priv, struct ctl_ehip_buffer *buffer)
{
	writel(lower_32_bits(buffer->dma_addr), &priv->tx_dma_desc->addr32);
	writel(upper_32_bits(buffer->dma_addr), &priv->tx_dma_desc->addr64);
	writel(buffer->len, &priv->tx_dma_desc->length);
	writel(EHIP_DMA_DESC_CTL_GO, &priv->tx_dma_desc->control);
	return 0;
}

u32 ctl_ehip_dma_tx_completions(struct ctl_ehip_private *priv)
{
	u32 ready = 0;
	//u32 inuse;
	//u32 status;

	/* Get number of sent descriptors */
	//inuse = readl(&priv->tx_dma_csr->rw_fill_level) & 0xffff;

	//if (inuse) { /* Tx FIFO is not empty */
	//	ready = max_t(int,
	//		      priv->tx_prod - priv->tx_cons - inuse - 1, 0);
	//} else {
		/* Check for buffered last packet */
	//	status = readl(&priv->tx_dma_csr->status);
	//	if (status & EHIP_DMA_CSR_STAT_BUSY)
	//		ready = priv->tx_prod - priv->tx_cons - 1;
	//	else
	//		ready = priv->tx_prod - priv->tx_cons;
	//}
	while (readl(&priv->tx_dma_csr->busy) != TX_EHIP_DMA_CSR_BUSY_MASK) {
		ready++;
		ctl_ehip_dma_start_txdma(priv);
	}

	return ready;
}

/* Put buffer to the eHIP DMA RX FIFO
 */
void ctl_ehip_dma_add_rx_desc(struct ctl_ehip_private *priv,
			struct ctl_ehip_buffer *rxbuffer)
{
	u32 len = priv->rx_dma_buf_sz;
	dma_addr_t dma_addr = rxbuffer->dma_addr;

	writel(lower_32_bits(dma_addr), &priv->rx_dma_desc->addr32);
	writel(upper_32_bits(dma_addr), &priv->rx_dma_desc->addr64);
	writel(len, &priv->rx_dma_desc->length);
	writel(EHIP_DMA_DESC_CTL_GO, &priv->rx_dma_desc->control);
}

/* status is returned on upper 16 bits,
 * length is returned in lower 16 bits
 */
u32 ctl_ehip_dma_rx_status(struct ctl_ehip_private *priv)
{
	u32 rxstatus = 0;
	u32 pktlength;
	u32 pktstatus;

	if (readl(&priv->rx_dma_csr->busy) != RX_EHIP_DMA_CSR_BUSY_MASK) {
		pktlength = readl(&priv->rx_dma_resp->bytes_transferred);
		pktstatus = readl(&priv->rx_dma_resp->status);
		rxstatus = pktstatus;
		rxstatus = rxstatus << 16;
		rxstatus |= (pktlength & 0xffff);
	}
	return rxstatus;
}
