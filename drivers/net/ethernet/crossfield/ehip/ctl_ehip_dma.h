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

#ifndef __CTL_EHIP_DMA_H__
#define __CTL_EHIP_DMA_H__

void ctl_ehip_dma_reset(struct ctl_ehip_private *);
void ctl_ehip_dma_enable_txirq(struct ctl_ehip_private *);
void ctl_ehip_dma_enable_rxirq(struct ctl_ehip_private *);
void ctl_ehip_dma_disable_rxirq(struct ctl_ehip_private *);
void ctl_ehip_dma_disable_txirq(struct ctl_ehip_private *);
void ctl_ehip_dma_clear_rxirq(struct ctl_ehip_private *);
void ctl_ehip_dma_clear_txirq(struct ctl_ehip_private *);
int ctl_ehip_dma_rxirq_status(struct ctl_ehip_private *priv);
int ctl_ehip_dma_txirq_status(struct ctl_ehip_private *priv);
u32 ctl_ehip_dma_tx_completions(struct ctl_ehip_private *);
void ctl_ehip_dma_add_rx_desc(struct ctl_ehip_private *, struct ctl_ehip_buffer *);
int ctl_ehip_dma_tx_buffer(struct ctl_ehip_private *, struct ctl_ehip_buffer *);
u32 ctl_ehip_dma_rx_status(struct ctl_ehip_private *);
int ctl_ehip_dma_initialize(struct ctl_ehip_private *);
void ctl_ehip_dma_uninitialize(struct ctl_ehip_private *);
void ctl_ehip_dma_start_rxdma(struct ctl_ehip_private *);
void ctl_ehip_dma_start_txdma(struct ctl_ehip_private *);
void ctl_ehip_dma_start_rxdisp(struct ctl_ehip_private *priv);
void ctl_ehip_dma_start_txdisp(struct ctl_ehip_private *priv);

#endif /*  __CTL_EHIP_DMA_H__ */
