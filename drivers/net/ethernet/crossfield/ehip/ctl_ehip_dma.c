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
#include "ctl_ehip_dmahw.h"
#include "ctl_ehip_dma.h"

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
}

void ctl_ehip_dma_check(struct ctl_ehip_private *priv)
{
	u32 reg;

	printk("crossfield_ehip_dma: Checking status of eHIP DMA cores:\n");

	reg = readl(&priv->rx_dma_csr->status);
	printk("crossfield_ehip_dma: RX Status: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->control);
	printk("crossfield_ehip_dma: RX Control: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->rw_fill_level);
	printk("crossfield_ehip_dma: RX Write/Read Fill Level: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->resp_fill_level);
	printk("crossfield_ehip_dma: RX Response Fill Level: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->rw_seq_num);
	printk("crossfield_ehip_dma: RX Write/Read Sequence Number: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->comp_cfg1);
	printk("crossfield_ehip_dma: RX Component Config 1: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->comp_cfg2);
	printk("crossfield_ehip_dma: RX Component Config 2: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->comp_type_ver);
	printk("crossfield_ehip_dma: RX Component Type & Version: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->status);
	printk("crossfield_ehip_dma: TX Status: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->control);
	printk("crossfield_ehip_dma: TX Control: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->rw_fill_level);
	printk("crossfield_ehip_dma: TX Write/Read Fill Level: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->resp_fill_level);
	printk("crossfield_ehip_dma: TX Response Fill Level: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->rw_seq_num);
	printk("crossfield_ehip_dma: TX Write/Read Sequence Number: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->comp_cfg1);
	printk("crossfield_ehip_dma: TX Component Config 1: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->comp_cfg2);
	printk("crossfield_ehip_dma: TX Component Config 2: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->comp_type_ver);
	printk("crossfield_ehip_dma: TX Component Type & Version: 0x%08x\n", reg);
}


void ctl_ehip_dma_reset(struct ctl_ehip_private *priv)
{
	int counter;

	/* Reset Rx eHIP DMA */
	writel(EHIP_DMA_CSR_STAT_MASK, &priv->rx_dma_csr->status);
	writel(EHIP_DMA_CSR_CTL_RESET, &priv->rx_dma_csr->control);

	counter = 0;
	while (counter++ < ALTERA_S10_100GHIP_SW_RESET_WATCHDOG_CNTR) {
		if (ctl_ehip_bit_is_clear(&priv->rx_dma_csr->status,
				     EHIP_DMA_CSR_STAT_RESETTING))
			break;
		udelay(1);
	}

	if (counter >= ALTERA_S10_100GHIP_SW_RESET_WATCHDOG_CNTR)
		netif_warn(priv, drv, priv->dev,
			   "Crossfield eHIP DMA RX resetting bit never cleared!\n");

	/* clear all status bits */
	writel(EHIP_DMA_CSR_STAT_MASK, &priv->rx_dma_csr->status);

	/* Reset Tx eHIP DMA */
	writel(EHIP_DMA_CSR_STAT_MASK, &priv->tx_dma_csr->status);

	writel(EHIP_DMA_CSR_CTL_RESET, &priv->tx_dma_csr->control);

	counter = 0;
	while (counter++ < ALTERA_S10_100GHIP_SW_RESET_WATCHDOG_CNTR) {
		if (ctl_ehip_bit_is_clear(&priv->tx_dma_csr->status,
				     EHIP_DMA_CSR_STAT_RESETTING))
			break;
		udelay(1);
	}

	if (counter >= ALTERA_S10_100GHIP_SW_RESET_WATCHDOG_CNTR)
		netif_warn(priv, drv, priv->dev,
			   "Crossfield eHIP DMA TX resetting bit never cleared!\n");

	/* clear all status bits */
	writel(EHIP_DMA_CSR_STAT_MASK, &priv->tx_dma_csr->status);
}

void ctl_ehip_dma_disable_rxirq(struct ctl_ehip_private *priv)
{
	ctl_ehip_clear_bit(&priv->rx_dma_csr->control,
		      EHIP_DMA_CSR_CTL_GLOBAL_INTR);
}

void ctl_ehip_dma_enable_rxirq(struct ctl_ehip_private *priv)
{
	ctl_ehip_set_bit(&priv->rx_dma_csr->control,
		    EHIP_DMA_CSR_CTL_GLOBAL_INTR);
}

void ctl_ehip_dma_disable_txirq(struct ctl_ehip_private *priv)
{
	ctl_ehip_clear_bit(&priv->tx_dma_csr->control,
		      EHIP_DMA_CSR_CTL_GLOBAL_INTR);
}

void ctl_ehip_dma_enable_txirq(struct ctl_ehip_private *priv)
{
	ctl_ehip_set_bit(&priv->tx_dma_csr->control,
		    EHIP_DMA_CSR_CTL_GLOBAL_INTR);
}

void ctl_ehip_dma_clear_rxirq(struct ctl_ehip_private *priv)
{
	writel(EHIP_DMA_CSR_STAT_IRQ, &priv->rx_dma_csr->status);
}

void ctl_ehip_dma_clear_txirq(struct ctl_ehip_private *priv)
{
	writel(EHIP_DMA_CSR_STAT_IRQ, &priv->tx_dma_csr->status);
}

/* return 0 to indicate transmit is pending */
int ctl_ehip_dma_tx_buffer(struct ctl_ehip_private *priv, struct ctl_ehip_buffer *buffer)
{
	writel(lower_32_bits(buffer->dma_addr), &priv->tx_dma_desc->read_addr_lo);
	writel(upper_32_bits(buffer->dma_addr), &priv->tx_dma_desc->read_addr_hi);
	writel(0, &priv->tx_dma_desc->write_addr_lo);
	writel(0, &priv->tx_dma_desc->write_addr_lo);
	writel(buffer->len, &priv->tx_dma_desc->len);
	writel(0, &priv->tx_dma_desc->burst_seq_num);
	writel(EHIP_DMA_DESC_TX_STRIDE, &priv->tx_dma_desc->stride);
	writel(EHIP_DMA_DESC_CTL_TX_SINGLE, &priv->tx_dma_desc->control);
	return 0;
}

u32 ctl_ehip_dma_tx_completions(struct ctl_ehip_private *priv)
{
	u32 ready = 0;
	u32 inuse;
	u32 status;

	/* Get number of sent descriptors */
	inuse = readl(&priv->tx_dma_csr->rw_fill_level) & 0xffff;

	if (inuse) { /* Tx FIFO is not empty */
		ready = max_t(int,
			      priv->tx_prod - priv->tx_cons - inuse - 1, 0);
	} else {
		/* Check for buffered last packet */
		status = readl(&priv->tx_dma_csr->status);
		if (status & EHIP_DMA_CSR_STAT_BUSY)
			ready = priv->tx_prod - priv->tx_cons - 1;
		else
			ready = priv->tx_prod - priv->tx_cons;
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
	u32 control = (EHIP_DMA_DESC_CTL_END_ON_EOP
			| EHIP_DMA_DESC_CTL_END_ON_LEN
			| EHIP_DMA_DESC_CTL_TR_COMP_IRQ
			| EHIP_DMA_DESC_CTL_EARLY_IRQ
			| EHIP_DMA_DESC_CTL_TR_ERR_IRQ
			| EHIP_DMA_DESC_CTL_GO);

	writel(0, &priv->rx_dma_desc->read_addr_lo);
	writel(0, &priv->rx_dma_desc->read_addr_hi);
	writel(lower_32_bits(dma_addr), &priv->rx_dma_desc->write_addr_lo);
	writel(upper_32_bits(dma_addr), &priv->rx_dma_desc->write_addr_hi);
	writel(len, &priv->rx_dma_desc->len);
	writel(0, &priv->rx_dma_desc->burst_seq_num);
	writel(EHIP_DMA_DESC_TX_STRIDE, &priv->rx_dma_desc->stride);
	writel(control, &priv->rx_dma_desc->control);
}

/* status is returned on upper 16 bits,
 * length is returned in lower 16 bits
 */
u32 ctl_ehip_dma_rx_status(struct ctl_ehip_private *priv)
{
	u32 rxstatus = 0;
	u32 pktlength;
	u32 pktstatus;

	if (readl(&priv->rx_dma_csr->resp_fill_level) & 0xffff) {
		pktlength = readl(&priv->rx_dma_resp->bytes_transferred);
		pktstatus = readl(&priv->rx_dma_resp->status);
		rxstatus = pktstatus;
		rxstatus = rxstatus << 16;
		rxstatus |= (pktlength & 0xffff);
	}
	return rxstatus;
}
