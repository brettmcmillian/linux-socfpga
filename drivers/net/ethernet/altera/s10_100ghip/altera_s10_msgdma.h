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

#ifndef __ALTERA_S10_MSGDMA_H__
#define __ALTERA_S10_MSGDMA_H__

void msgdma_reset(struct altera_s10_100ghip_private *);
void msgdma_enable_txirq(struct altera_s10_100ghip_private *);
void msgdma_enable_rxirq(struct altera_s10_100ghip_private *);
void msgdma_disable_rxirq(struct altera_s10_100ghip_private *);
void msgdma_disable_txirq(struct altera_s10_100ghip_private *);
void msgdma_clear_rxirq(struct altera_s10_100ghip_private *);
void msgdma_clear_txirq(struct altera_s10_100ghip_private *);
u32 msgdma_tx_completions(struct altera_s10_100ghip_private *);
void msgdma_add_rx_desc(struct altera_s10_100ghip_private *, struct s10_100ghip_buffer *);
int msgdma_tx_buffer(struct altera_s10_100ghip_private *, struct s10_100ghip_buffer *);
u32 msgdma_rx_status(struct altera_s10_100ghip_private *);
int msgdma_initialize(struct altera_s10_100ghip_private *);
void msgdma_uninitialize(struct altera_s10_100ghip_private *);
void msgdma_start_rxdma(struct altera_s10_100ghip_private *);

#endif /*  __ALTERA_S10_MSGDMA_H__ */
