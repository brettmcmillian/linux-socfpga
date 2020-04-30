/*
 * Intel Stratix 10 mSGDMA Driver
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
#include "altera_s10_utils.h"
#include "altera_s10_100ghip.h"
#include "altera_s10_msgdmahw.h"
#include "altera_s10_msgdma.h"

/* No initialization work to do for MSGDMA */
int s10_msgdma_initialize(struct altera_s10_100ghip_private *priv)
{
	return 0;
}

void s10_msgdma_uninitialize(struct altera_s10_100ghip_private *priv)
{
}

void s10_msgdma_start_rxdma(struct altera_s10_100ghip_private *priv)
{
}

void s10_msgdma_check(struct altera_s10_100ghip_private *priv)
{
	u32 reg;

	printk("altera_s10_100ghip: Checking status of mSGDMA cores:\n");

	reg = readl(&priv->rx_dma_csr->status);
	printk("altera_s10_100ghip: RX Status: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->control);
	printk("altera_s10_100ghip: RX Control: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->rw_fill_level);
	printk("altera_s10_100ghip: RX Write/Read Fill Level: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->resp_fill_level);
	printk("altera_s10_100ghip: RX Response Fill Level: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->rw_seq_num);
	printk("altera_s10_100ghip: RX Write/Read Sequence Number: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->comp_cfg1);
	printk("altera_s10_100ghip: RX Component Config 1: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->comp_cfg2);
	printk("altera_s10_100ghip: RX Component Config 2: 0x%08x\n", reg);

	reg = readl(&priv->rx_dma_csr->comp_type_ver);
	printk("altera_s10_100ghip: RX Component Type & Version: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->status);
	printk("altera_s10_100ghip: TX Status: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->control);
	printk("altera_s10_100ghip: TX Control: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->rw_fill_level);
	printk("altera_s10_100ghip: TX Write/Read Fill Level: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->resp_fill_level);
	printk("altera_s10_100ghip: TX Response Fill Level: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->rw_seq_num);
	printk("altera_s10_100ghip: TX Write/Read Sequence Number: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->comp_cfg1);
	printk("altera_s10_100ghip: TX Component Config 1: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->comp_cfg2);
	printk("altera_s10_100ghip: TX Component Config 2: 0x%08x\n", reg);

	reg = readl(&priv->tx_dma_csr->comp_type_ver);
	printk("altera_s10_100ghip: TX Component Type & Version: 0x%08x\n", reg);
}


void s10_msgdma_reset(struct altera_s10_100ghip_private *priv)
{
	int counter;

	/* Reset Rx mSGDMA */
	writel(MSGDMA_CSR_STAT_MASK, &priv->rx_dma_csr->status);
	writel(MSGDMA_CSR_CTL_RESET, &priv->rx_dma_csr->control);

	counter = 0;
	while (counter++ < ALTERA_S10_100GHIP_SW_RESET_WATCHDOG_CNTR) {
		if (s10_bit_is_clear(&priv->rx_dma_csr->status,
				     MSGDMA_CSR_STAT_RESETTING))
			break;
		udelay(1);
	}

	if (counter >= ALTERA_S10_100GHIP_SW_RESET_WATCHDOG_CNTR)
		netif_warn(priv, drv, priv->dev,
			   "S10 100G HIP Rx mSGDMA resetting bit never cleared!\n");

	/* clear all status bits */
	writel(MSGDMA_CSR_STAT_MASK, &priv->rx_dma_csr->status));

	/* Reset Tx mSGDMA */
	writel(MSGDMA_CSR_STAT_MASK, &priv->tx_dma_csr->status);

	writel(MSGDMA_CSR_CTL_RESET, &priv->tx_dma_csr->control);

	counter = 0;
	while (counter++ < ALTERA_S10_100GHIP_SW_RESET_WATCHDOG_CNTR) {
		if (s10_bit_is_clear(&priv->tx_dma_csr->status,
				     MSGDMA_CSR_STAT_RESETTING))
			break;
		udelay(1);
	}

	if (counter >= ALTERA_S10_100GHIP_SW_RESET_WATCHDOG_CNTR)
		netif_warn(priv, drv, priv->dev,
			   "S10 100G HIP Tx mSGDMA resetting bit never cleared!\n");

	/* clear all status bits */
	writel(MSGDMA_CSR_STAT_MASK, &priv->tx_dma_csr->status);
}

void s10_msgdma_disable_rxirq(struct altera_s10_100ghip_private *priv)
{
	s10_clear_bit(&priv->rx_dma_csr->control,
		      MSGDMA_CSR_CTL_GLOBAL_INTR);
}

void s10_msgdma_enable_rxirq(struct altera_s10_100ghip_private *priv)
{
	s10_set_bit(&priv->rx_dma_csr->control,
		    MSGDMA_CSR_CTL_GLOBAL_INTR);
}

void s10_msgdma_disable_txirq(struct altera_s10_100ghip_private *priv)
{
	s10_clear_bit(&priv->tx_dma_csr->control,
		      MSGDMA_CSR_CTL_GLOBAL_INTR);
}

void s10_msgdma_enable_txirq(struct altera_s10_100ghip_private *priv)
{
	s10_set_bit(&priv->tx_dma_csr->control,
		    MSGDMA_CSR_CTL_GLOBAL_INTR);
}

void s10_msgdma_clear_rxirq(struct altera_s10_100ghip_private *priv)
{
	writel(MSGDMA_CSR_STAT_IRQ, &priv->rx_dma_csr->status);
}

void s10_msgdma_clear_txirq(struct altera_s10_100ghip_private *priv)
{
	writel(MSGDMA_CSR_STAT_IRQ, &priv->tx_dma_csr->status);
}

/* return 0 to indicate transmit is pending */
int s10_msgdma_tx_buffer(struct altera_s10_100ghip_private *priv, struct s10_100ghip_buffer *buffer)
{
	writel(lower_32_bits(buffer->dma_addr), &priv->tx_dma_desc->read_addr_lo);
	writel(upper_32_bits(buffer->dma_addr), &priv->tx_dma_desc->read_addr_hi);
	writel(0, &priv->tx_dma_desc->write_addr_lo);
	writel(0, &priv->tx_dma_desc->write_addr_lo);
	writel(buffer->len, &priv->tx_dma_desc->len);
	writel(0, &priv->tx_dma_desc->burst_seq_num);
	writel(MSGDMA_DESC_TX_STRIDE, &priv->tx_dma_desc->stride);
	writel(MSGDMA_DESC_CTL_TX_SINGLE, &priv->tx_dma_desc->control);
	return 0;
}

u32 s10_msgdma_tx_completions(struct altera_s10_100ghip_private *priv)
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
		if (status & MSGDMA_CSR_STAT_BUSY)
			ready = priv->tx_prod - priv->tx_cons - 1;
		else
			ready = priv->tx_prod - priv->tx_cons;
	}
	return ready;
}

/* Put buffer to the mSGDMA RX FIFO
 */
void s10_msgdma_add_rx_desc(struct altera_s10_100ghip_private *priv,
			struct s10_100ghip_buffer *rxbuffer)
{
	u32 len = priv->rx_dma_buf_sz;
	dma_addr_t dma_addr = rxbuffer->dma_addr;
	u32 control = (MSGDMA_DESC_CTL_END_ON_EOP
			| MSGDMA_DESC_CTL_END_ON_LEN
			| MSGDMA_DESC_CTL_TR_COMP_IRQ
			| MSGDMA_DESC_CTL_EARLY_IRQ
			| MSGDMA_DESC_CTL_TR_ERR_IRQ
			| MSGDMA_DESC_CTL_GO);

	writel(0, &priv->rx_dma_desc->read_addr_lo);
	writel(0, &priv->rx_dma_desc->read_addr_hi);
	writel(lower_32_bits(dma_addr), &priv->rx_dma_desc->write_addr_lo);
	writel(upper_32_bits(dma_addr), &priv->rx_dma_desc->write_addr_hi);
	writel(len, &priv->rx_dma_desc->len);
	writel(0, &priv->rx_dma_desc->burst_seq_num);
	writel(MSGDMA_DESC_TX_STRIDE, &priv->rx_dma_desc->stride);
	writel(control, &priv->rx_dma_desc->control);
}

/* status is returned on upper 16 bits,
 * length is returned in lower 16 bits
 */
u32 s10_msgdma_rx_status(struct altera_s10_100ghip_private *priv)
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
