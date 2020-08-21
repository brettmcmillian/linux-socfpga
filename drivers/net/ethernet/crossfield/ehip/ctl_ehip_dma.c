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
#include "rx_dispatcher_csr.h"
#include "tx_dispatcher_csr.h"
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
	if (readl(&priv->rx_dma_csr->busy) != RX_EHIP_DMA_CSR_BUSY_MASK)
		writel(RX_EHIP_DMA_CSR_START_MASK, &priv->rx_dma_csr->start);
}

void ctl_ehip_dma_start_txdma(struct ctl_ehip_private *priv)
{
	if (readl(&priv->tx_dma_csr->busy) != TX_EHIP_DMA_CSR_BUSY_MASK)
		writel(TX_EHIP_DMA_CSR_START_MASK, &priv->tx_dma_csr->start);
}

void ctl_ehip_dma_start_rxdisp(struct ctl_ehip_private *priv)
{
	if (readl(&priv->rx_dma_desc->busy) != RX_DISPATCHER_CSR_BUSY_MASK) {
		writel(0, &priv->rx_dma_desc->length);
		writel(RX_DISPATCHER_CSR_START_MASK, &priv->rx_dma_desc->start);
	}
}

void ctl_ehip_dma_start_txdisp(struct ctl_ehip_private *priv)
{
	if (readl(&priv->tx_dma_desc->busy) != TX_DISPATCHER_CSR_BUSY_MASK) {
		writel(0, &priv->tx_dma_desc->length);
		writel(TX_DISPATCHER_CSR_START_MASK, &priv->tx_dma_desc->start);
	}
}

void ctl_ehip_dma_reset(struct ctl_ehip_private *priv)
{
	//No software reset for HLS core
}

void ctl_ehip_dma_disable_rxirq(struct ctl_ehip_private *priv)
{
	writel(0, &priv->rx_dma_csr->interrupt_enable);
}

void ctl_ehip_dma_enable_rxirq(struct ctl_ehip_private *priv)
{
	writel(RX_EHIP_DMA_CSR_INTERRUPT_ENABLE_MASK,
			&priv->rx_dma_csr->interrupt_enable);
}

void ctl_ehip_dma_disable_txirq(struct ctl_ehip_private *priv)
{
	writel(0, &priv->tx_dma_csr->interrupt_enable);
}

void ctl_ehip_dma_enable_txirq(struct ctl_ehip_private *priv)
{
	writel(TX_EHIP_DMA_CSR_INTERRUPT_ENABLE_MASK,
			&priv->tx_dma_csr->interrupt_enable);
}

void ctl_ehip_dma_clear_rxirq(struct ctl_ehip_private *priv)
{
	writel(RX_EHIP_DMA_CSR_INTERRUPT_STATUS_MASK, &priv->rx_dma_csr->status);
}

void ctl_ehip_dma_clear_txirq(struct ctl_ehip_private *priv)
{
	writel(TX_EHIP_DMA_CSR_INTERRUPT_STATUS_MASK, &priv->tx_dma_csr->status);
}

int ctl_ehip_dma_rxirq_status(struct ctl_ehip_private *priv)
{
	int status = 0;
	status = readl(&priv->rx_dma_csr->status);

	if (status & RX_EHIP_DMA_CSR_DONE_MASK)
		return 1;

	return 0;
}

int ctl_ehip_dma_txirq_status(struct ctl_ehip_private *priv)
{
	int status = 0;
	status = readl(&priv->tx_dma_csr->status);

	if (status & RX_EHIP_DMA_CSR_DONE_MASK)
		return 1;

	return 0;
}

/* return 0 to indicate transmit is pending */
int ctl_ehip_dma_tx_buffer(struct ctl_ehip_private *priv, struct ctl_ehip_buffer *buffer)
{
	writel(lower_32_bits(buffer->dma_addr), &priv->tx_dma_desc->addr32);
	writel(upper_32_bits(buffer->dma_addr), &priv->tx_dma_desc->addr64);
	writel(buffer->len, &priv->tx_dma_desc->length);
	if (readl(&priv->tx_dma_desc->busy) != TX_DISPATCHER_CSR_BUSY_MASK)
		writel(TX_DISPATCHER_CSR_START_MASK, &priv->tx_dma_desc->start);
	return 0;
}

u32 ctl_ehip_dma_tx_completions(struct ctl_ehip_private *priv)
{
	u32 ready = 0;
	u32 inuse = 0;
	u32 status;

	/* Get number of sent descriptors */
	inuse = readl(&priv->tx_dma_desc->fill_level);

	if (inuse) /* Tx FIFO is not empty */
		ready = max_t(int, priv->tx_prod - priv->tx_cons - inuse - 1, 0);
	else 
		ready = priv->tx_prod - priv->tx_cons;
	
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
	if (readl(&priv->rx_dma_desc->busy) != RX_DISPATCHER_CSR_BUSY_MASK)
		writel(RX_DISPATCHER_CSR_START_MASK, &priv->rx_dma_desc->start);
}

/* status is returned on upper 16 bits,
 * length is returned in lower 16 bits
 */
u32 ctl_ehip_dma_rx_status(struct ctl_ehip_private *priv)
{
	u32 fill_level;

	fill_level = readl(&priv->rx_dma_desc->fill_level);

	ctl_ehip_dma_start_rxdisp(priv);
	
	//printk("RX fill level = %d", fill_level);
	return fill_level;
}
