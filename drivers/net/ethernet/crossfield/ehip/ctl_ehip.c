/*
 * Crossfield Ethernet Hard IP Driver for Intel Stratix 10 100G eHIP
 * Copyright (C) 2020 Crossfield Technology LLC. All rights reserved.
 *
 * Contributors:
 *   Brett McMillian
 *
 * This driver is based on the Altera TSE driver and must be used
 * with the ctl_ehip_dma driver.
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

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <asm/cacheflush.h>

#include "ctl_ehip.h"
#include "ctl_ehip_dma.h"



static atomic_t instance_count = ATOMIC_INIT(~0);
/* Module parameters */
static int debug = -1;
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Message Level (-1: default, 0: no output, 16: all)");

static const u32 default_msg_level = (NETIF_MSG_DRV | NETIF_MSG_PROBE |
					NETIF_MSG_LINK | NETIF_MSG_IFUP |
					NETIF_MSG_IFDOWN);

#define RX_DESCRIPTORS 256
static int dma_rx_num = RX_DESCRIPTORS;
module_param(dma_rx_num, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dma_rx_num, "Number of descriptors in the RX list");

#define TX_DESCRIPTORS 256
static int dma_tx_num = TX_DESCRIPTORS;
module_param(dma_tx_num, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dma_tx_num, "Number of descriptors in the TX list");


#define POLL_PHY (-1)

/* Make sure DMA buffer size is larger than the max frame size
 * plus some alignment offset and a VLAN header. If the max frame size is
 * 1518, a VLAN header would be additional 4 bytes and additional
 * headroom for alignment is 2 bytes, 2048 is just fine.
 */
#define EHIP_RXDMABUFFER_SIZE	2048
#define MM_TRANSFER_SIZE		16
#define ALIGNMENT_SIZE			64

/* Allow network stack to resume queueing packets after we've
 * finished transmitting at least 1/4 of the packets in the queue.
 */
#define CTL_EHIP_TX_THRESH(x)	(x->tx_ring_size / 4)

#define TXQUEUESTOP_THRESHHOLD	2

static const struct of_device_id ctl_ehip_ids[];



static inline u32 ctl_ehip_tx_avail(struct ctl_ehip_private *priv)
{
	return priv->tx_cons + priv->tx_ring_size - priv->tx_prod - 1;
}

/*
 * PCS Register read/write functions
 */
static u16 ctl_ehip_pcs_read(struct ctl_ehip_private *priv, int regnum)
{
	return 0x0;
}

static void ctl_ehip_pcs_write(struct ctl_ehip_private *priv, int regnum,
				u16 value)
{

}

/* Check PCS scratch memory */
static int ctl_ehip_pcs_scratch_test(struct ctl_ehip_private *priv, u32 value)
{
	writel(value, &priv->eth_reconfig->phy_scratch);
	return (readl(&priv->eth_reconfig->phy_scratch) == value);
}

static int ctl_ehip_init_rx_buffer(struct ctl_ehip_private *priv,
			      struct ctl_ehip_buffer *rxbuffer, int len)
{
	rxbuffer->skb = netdev_alloc_skb(priv->dev, len);
	if (!rxbuffer->skb)
		return -ENOMEM;

	rxbuffer->dma_addr = dma_map_single(priv->device, rxbuffer->skb->data,
						len,
						DMA_FROM_DEVICE);

	if (dma_mapping_error(priv->device, rxbuffer->dma_addr)) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		dev_kfree_skb_any(rxbuffer->skb);
		return -EINVAL;
	}
	//rxbuffer->dma_addr &= (dma_addr_t)~3;
	rxbuffer->len = len;
	return 0;
}

static void ctl_ehip_free_rx_buffer(struct ctl_ehip_private *priv,
			       struct ctl_ehip_buffer *rxbuffer)
{
	struct sk_buff *skb = rxbuffer->skb;
	dma_addr_t dma_addr = rxbuffer->dma_addr;

	if (skb != NULL) {
		if (dma_addr)
			dma_unmap_single(priv->device, dma_addr,
					 rxbuffer->len,
					 DMA_FROM_DEVICE);
		dev_kfree_skb_any(skb);
		rxbuffer->skb = NULL;
		rxbuffer->dma_addr = 0;
	}
}

/*
 * Unmap and free Tx buffer resources
 */
static void ctl_ehip_free_tx_buffer(struct ctl_ehip_private *priv,
			       struct ctl_ehip_buffer *buffer)
{
	if (buffer->dma_addr) {
		if (buffer->mapped_as_page)
			dma_unmap_page(priv->device, buffer->dma_addr,
				       buffer->len, DMA_TO_DEVICE);
		else
			dma_unmap_single(priv->device, buffer->dma_addr,
					 buffer->len, DMA_TO_DEVICE);
		buffer->dma_addr = 0;
	}
	if (buffer->skb) {
		dev_kfree_skb_any(buffer->skb);
		buffer->skb = NULL;
	}
}

static int alloc_init_skbufs(struct ctl_ehip_private *priv)
{
	unsigned int rx_descs = priv->rx_ring_size;
	unsigned int tx_descs = priv->tx_ring_size;
	int ret = -ENOMEM;
	int i;

	/* Create Rx ring buffer */
	priv->rx_ring = kcalloc(rx_descs, sizeof(struct ctl_ehip_buffer),
				GFP_KERNEL);
	if (!priv->rx_ring)
		goto err_rx_ring;

	/* Create Tx ring buffer */
	priv->tx_ring = kcalloc(tx_descs, sizeof(struct ctl_ehip_buffer),
				GFP_KERNEL);
	if (!priv->tx_ring)
		goto err_tx_ring;

	priv->tx_cons = 0;
	priv->tx_prod = 0;

	/* Init Rx ring */
	for (i = 0; i < rx_descs; i++) {
		ret = ctl_ehip_init_rx_buffer(priv, &priv->rx_ring[i],
					 priv->rx_dma_buf_sz);
		if (ret)
			goto err_init_rx_buffers;
	}

	priv->rx_cons = 0;
	priv->rx_prod = 0;

	return 0;
err_init_rx_buffers:
	while (--i >= 0)
		ctl_ehip_free_rx_buffer(priv, &priv->rx_ring[i]);
	kfree(priv->tx_ring);
err_tx_ring:
	kfree(priv->rx_ring);
err_rx_ring:
	return ret;
}

static void free_skbufs(struct net_device *dev)
{
	struct ctl_ehip_private *priv = netdev_priv(dev);
	unsigned int rx_descs = priv->rx_ring_size;
	unsigned int tx_descs = priv->tx_ring_size;
	int i;

	/* Release the DMA TX/RX socket buffers */
	for (i = 0; i < rx_descs; i++)
		ctl_ehip_free_rx_buffer(priv, &priv->rx_ring[i]);
	for (i = 0; i < tx_descs; i++)
		ctl_ehip_free_tx_buffer(priv, &priv->tx_ring[i]);


	kfree(priv->tx_ring);
}

/* Reallocate the skb for the reception process
 */
static inline void ctl_ehip_rx_refill(struct ctl_ehip_private *priv)
{
	unsigned int rxsize = priv->rx_ring_size;
	unsigned int entry;
	int ret;

	for (; priv->rx_cons - priv->rx_prod > 0;
			priv->rx_prod++) {
		entry = priv->rx_prod % rxsize;
		if (likely(priv->rx_ring[entry].skb == NULL)) {
			ret = ctl_ehip_init_rx_buffer(priv, &priv->rx_ring[entry],
				priv->rx_dma_buf_sz);
			if (unlikely(ret != 0))
				break;
			priv->dmaops->add_rx_desc(priv, &priv->rx_ring[entry]);
		}
	}
}

/* Pull out the VLAN tag and fix up the packet
 */
static inline void ctl_ehip_rx_vlan(struct net_device *dev, struct sk_buff *skb)
{
	struct ethhdr *eth_hdr;
	u16 vid;
	if ((dev->features & NETIF_F_HW_VLAN_CTAG_RX) &&
	    !__vlan_get_tag(skb, &vid)) {
		eth_hdr = (struct ethhdr *)skb->data;
		memmove(skb->data + VLAN_HLEN, eth_hdr, ETH_ALEN * 2);
		skb_pull(skb, VLAN_HLEN);
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), vid);
	}
}

/* Receive a packet: retrieve and pass over to upper levels
 */
static int ctl_ehip_rx(struct ctl_ehip_private *priv, int limit)
{
	unsigned int count = 0;
	unsigned int next_entry;
	struct sk_buff *skb;
	unsigned int entry = priv->rx_cons % priv->rx_ring_size;
	u32 pktlength;
	unsigned char * data;
	int length_offset = EHIP_RXDMABUFFER_SIZE - MM_TRANSFER_SIZE;

	/* Check for count < limit first as get_rx_status is changing
	* the response-fifo so we must process the next packet
	* after calling get_rx_status if a response is pending.
	* (reading the last byte of the response pops the value from the fifo.)
	*/
	while ((count < limit) &&
	    (priv->dmaops->get_rx_status(priv) < RX_DESCRIPTORS)) {

		count++;
		next_entry = (++priv->rx_cons) % priv->rx_ring_size;

		skb = priv->rx_ring[entry].skb;
		if (unlikely(!skb)) {
			netdev_err(priv->dev,
				   "%s: Inconsistent Rx descriptor chain\n",
				   __func__);
			priv->dev->stats.rx_dropped++;
			break;
		}
		priv->rx_ring[entry].skb = NULL;

		data = (unsigned char *)skb->data;

		pktlength = ((u32)(data[length_offset]) & 0xFF) + (((u32)(data[length_offset+1]) << 8) & 0xFF00);

		if (pktlength > priv->dev->mtu) {
			netdev_err(priv->dev,
				   "Received packet greater than MTU of length %08X\n",
				   pktlength);
			break;
		}

		skb_put(skb, pktlength);

		dma_unmap_single(priv->device, priv->rx_ring[entry].dma_addr,
				 priv->rx_ring[entry].len, DMA_FROM_DEVICE);

		if (netif_msg_pktdata(priv)) {
			netdev_info(priv->dev, "Frame received %d bytes\n",
				    pktlength);
			print_hex_dump(KERN_ERR, "RX Data: ", DUMP_PREFIX_OFFSET,
				       16, 1, skb->data, pktlength, true);
		}

		ctl_ehip_rx_vlan(priv->dev, skb);

		skb->protocol = eth_type_trans(skb, priv->dev);
		skb_checksum_none_assert(skb);

		napi_gro_receive(&priv->rx_napi, skb);

		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += pktlength;

		entry = next_entry;

		ctl_ehip_rx_refill(priv);
	}

	return count;
}

/* Reclaim resources after transmission completes
 */
static int ctl_ehip_tx_complete(struct ctl_ehip_private *priv)
{
	unsigned int txsize = priv->tx_ring_size;
	u32 ready;
	unsigned int entry;
	struct ctl_ehip_buffer *tx_buff;
	int txcomplete = 0;

	spin_lock(&priv->tx_lock);

	ready = priv->dmaops->tx_completions(priv);

	/* Free sent buffers */
	while (ready && (priv->tx_cons != priv->tx_prod)) {
		entry = priv->tx_cons % txsize;
		tx_buff = &priv->tx_ring[entry];

		if (netif_msg_tx_done(priv))
			netdev_dbg(priv->dev, "%s: curr %d, dirty %d\n",
				   __func__, priv->tx_prod, priv->tx_cons);

		if (likely(tx_buff->skb))
			priv->dev->stats.tx_packets++;

		ctl_ehip_free_tx_buffer(priv, tx_buff);
		priv->tx_cons++;

		txcomplete++;
		priv->dmaops->start_txdma(priv);
		ready = priv->dmaops->tx_completions(priv);
	}

	if (unlikely(netif_queue_stopped(priv->dev) &&
		     ctl_ehip_tx_avail(priv) > CTL_EHIP_TX_THRESH(priv))) {
		if (netif_queue_stopped(priv->dev) &&
		    ctl_ehip_tx_avail(priv) > CTL_EHIP_TX_THRESH(priv)) {
			if (netif_msg_tx_done(priv))
				netdev_dbg(priv->dev, "%s: restart transmit\n",
					   __func__);
			netif_wake_queue(priv->dev);
		}
	}

	spin_unlock(&priv->tx_lock);

	//printk("TX Complete\n");

	return txcomplete;
}

/*
 * NAPI polling function
 */
static int ctl_ehip_rx_poll(struct napi_struct *napi, int budget)
{
	struct ctl_ehip_private *priv =
			container_of(napi, struct ctl_ehip_private, rx_napi);
	int rxcomplete = 0;
	unsigned long int flags;

	rxcomplete = ctl_ehip_rx(priv, budget);

	if (rxcomplete < budget) {
		//printk("Completed receiving packets\n");

		napi_complete_done(napi, rxcomplete);

		netdev_dbg(priv->dev,
			   "NAPI Complete, did %d packets with budget %d\n",
			   rxcomplete, budget);

		spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
		priv->dmaops->clear_rxirq(priv);
		priv->dmaops->enable_rxirq(priv);
		spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

		//Update the RX fill level
		priv->dmaops->start_rxdisp(priv);
		priv->dmaops->start_rxdma(priv);
	}

	//printk("rxcomplete = %d", rxcomplete);
	return rxcomplete;
}

/*
 * DMA TX & RX FIFO interrupt routing
 */
static irqreturn_t crossfield_rx_isr(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct ctl_ehip_private *priv;

	//printk("RX IRQ\n");

	if (unlikely(!dev)) {
		pr_err("%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}
	priv = netdev_priv(dev);

	priv->dmaops->start_rxdisp(priv);

	spin_lock(&priv->rxdma_irq_lock);
	/* Clear IRQ Status Registers */
	priv->dmaops->clear_rxirq(priv);
	spin_unlock(&priv->rxdma_irq_lock);

	if (likely(napi_schedule_prep(&priv->rx_napi))) {
		spin_lock(&priv->rxdma_irq_lock);
		priv->dmaops->disable_rxirq(priv);
		spin_unlock(&priv->rxdma_irq_lock);
		priv->dmaops->start_rxdma(priv);
		__napi_schedule(&priv->rx_napi);
	}

	return IRQ_HANDLED;
}

/*
 * NAPI polling function
 */
static int ctl_ehip_tx_poll(struct napi_struct *napi, int budget)
{
	struct ctl_ehip_private *priv =
			container_of(napi, struct ctl_ehip_private, tx_napi);
	int txcomplete = 0;
	unsigned long int flags;

	txcomplete = ctl_ehip_tx_complete(priv);

	if (txcomplete < budget) {

		napi_complete_done(napi, txcomplete);

		netdev_dbg(priv->dev,
			   "NAPI Complete, did %d packets with budget %d\n",
			   txcomplete, budget);

		spin_lock_irqsave(&priv->txdma_irq_lock, flags);
		priv->dmaops->clear_txirq(priv);
		priv->dmaops->enable_txirq(priv);
		spin_unlock_irqrestore(&priv->txdma_irq_lock, flags);

		priv->dmaops->start_txdma(priv);
	}

	return txcomplete;
}

/*
 * DMA TX & RX FIFO interrupt routing
 */
static irqreturn_t crossfield_tx_isr(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct ctl_ehip_private *priv;

	//printk("TX IRQ\n");

	if (unlikely(!dev)) {
		pr_err("%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}
	priv = netdev_priv(dev);

	spin_lock(&priv->txdma_irq_lock);
	/* Clear IRQ Status Registers */
	priv->dmaops->clear_txirq(priv);
	spin_unlock(&priv->txdma_irq_lock);

	if (likely(napi_schedule_prep(&priv->tx_napi))) {
		spin_lock(&priv->txdma_irq_lock);
		priv->dmaops->disable_txirq(priv);
		spin_unlock(&priv->txdma_irq_lock);
		priv->dmaops->start_txdma(priv);
		__napi_schedule(&priv->tx_napi);
	}

	return IRQ_HANDLED;
}

/* Transmit a packet (called by the kernel) using
 * the Crossfield DMA engine. Implies an assumption
 * that there's only one physically contiguous fragment
 * starting at skb->data, for length of skb_headlen(skb).
 */
static int ctl_ehip_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ctl_ehip_private *priv = netdev_priv(dev);
	unsigned int txsize = priv->tx_ring_size;
	unsigned int entry;
	struct ctl_ehip_buffer *buffer = NULL;
	int nfrags = skb_shinfo(skb)->nr_frags;
	unsigned int len = skb_headlen(skb);
	unsigned int headroom = skb_headroom(skb);
	unsigned int padded_len = ALIGNMENT_SIZE * (len/ALIGNMENT_SIZE + 1);

	enum netdev_tx ret = NETDEV_TX_OK;
	dma_addr_t dma_addr;

	spin_lock_bh(&priv->tx_lock);

	if (unlikely(ctl_ehip_tx_avail(priv) < nfrags + 1)) {
		if (!netif_queue_stopped(dev)) {
			netif_stop_queue(dev);
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx list full when queue awake\n",
				   __func__);
		}
		ret = NETDEV_TX_BUSY;
		goto out;
	}

	/* Map the first skb fragment */
	entry = priv->tx_prod % txsize;
	buffer = &priv->tx_ring[entry];

	memmove(skb->head, skb->data, len);

	skb_push(skb, headroom);

	skb_put_padto(skb, padded_len);

	if (netif_msg_pktdata(priv)) {
		netdev_info(priv->dev, "Transmitting frame of %d bytes. Padded length = %d bytes.\n",
					len, padded_len);
		print_hex_dump(KERN_ERR, "TX Data: ", DUMP_PREFIX_OFFSET,
					16, 1, skb->data, len, true);
	}


	dma_addr = dma_map_single(priv->device, skb->data, padded_len,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(priv->device, dma_addr)) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		ret = NETDEV_TX_OK;
		goto out;
	}

	buffer->skb = skb;
	buffer->dma_addr = dma_addr;
	buffer->len = len;

	priv->dmaops->tx_buffer(priv, buffer);

	skb_tx_timestamp(skb);

	priv->tx_prod++;
	dev->stats.tx_bytes += len;

	if (unlikely(ctl_ehip_tx_avail(priv) <= TXQUEUESTOP_THRESHHOLD)) {
		if (netif_msg_hw(priv))
			netdev_dbg(priv->dev, "%s: stop transmitted packets\n",
				   __func__);
		netif_stop_queue(dev);
	}

out:
	spin_unlock_bh(&priv->tx_lock);

	return ret;
}

/* Called every time the controller might need to be made
 * aware of new link state.  The PHY code conveys this
 * information through variables in the phydev structure, and this
 * function converts those variables into the appropriate
 * register values, and can bring down the device if needed.
 */
/*
static void ctl_ehip_adjust_link(struct net_device *dev)
{
	struct ctl_ehip_private *priv = netdev_priv(dev);
	struct phy_device *phydev = dev->phydev;

	if (netif_msg_link(priv))
		phy_print_status(phydev);
}


static struct phy_device *connect_local_phy(struct net_device *dev)
{

}


static int ctl_ehip_phy_get_addr_mdio_create(struct net_device *dev)
{

}
*/

/* Setup PHYLINK to control the PHY through the MAC */
 

static void ctl_ehip_validate(struct net_device *dev, unsigned long *supported,
										struct phylink_link_state *state)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };
	phylink_set(mask, FIBRE);
	phylink_set(mask, 100000baseSR4_Full);
}

static void ctl_ehip_mac_config(struct net_device *dev, unsigned int mode,
										 const struct phylink_link_state *state)
{

}

static int ctl_ehip_link_state(struct net_device *dev, struct phylink_link_state *state)
{
	struct ctl_ehip_private *priv = netdev_priv(dev);
	u32 reg;

	reg = readl(&priv->eth_reconfig->phy_tx_datapath_ready);
	reg &= PHY_TX_PCS_READY;
	if (reg == 0x1)
		netif_carrier_on(dev);
	else
		netif_carrier_off(dev);

	return 0;
}

static void ctl_ehip_mac_link_down(struct net_device *dev, unsigned int mode)
{
	netif_carrier_off(dev);
}

static void ctl_ehip_mac_link_up(struct net_device *dev, unsigned int mode,
											 struct phy_device *phy_dev)
{
		struct ctl_ehip_private *priv = netdev_priv(dev);
	u32 reg;

	reg = readl(&priv->eth_reconfig->phy_rx_pcs_status_for_anlt);
	reg &= PHY_RX_ALIGNED;
	if (reg == 0x1)
		netif_carrier_on(dev);
	else
		printk("ctl_ehip: RX PCS is not aligned\n");
}

static const struct phylink_mac_ops ctl_ehip_phylink_ops = {
	.validate		= ctl_ehip_validate,
	.mac_config		= ctl_ehip_mac_config,
	.mac_link_state	= ctl_ehip_link_state,
	.mac_link_up	= ctl_ehip_mac_link_up,
	.mac_link_down	= ctl_ehip_mac_link_down,
};

static int init_phy(struct net_device *dev)
{
	struct ctl_ehip_private *priv = netdev_priv(dev);
	struct phylink *phylink;
	u32 reg;
	int retries;
	int tx_ready = 0, rx_ready = 0;

/*	ctl_ehip_xcvr_cal_check(priv); */

	for (retries=0; retries < 5; retries++) {
		/* First check for tx_datapath_ready */
		reg = readl(&priv->eth_reconfig->phy_tx_datapath_ready);
		reg &= PHY_TX_PCS_READY;
		if (reg == 0x1)
			tx_ready = 1;

		if (tx_ready != 1) {
			printk("Crossfield eHIP Driver: Resetting the 100G HIP core.\n");
			if (retries == 4) {
				printk("Crossfield eHIP Driver: Failed to bring up the interace.\n");
				return -1;
			}

			/* Issue an Ethernet system reset to the HIP core */
			writel(PHY_EIO_SYS_RST, &priv->eth_reconfig->phy_config);
			udelay(1);
			writel(0x0, &priv->eth_reconfig->phy_config);
			mdelay(10);
		} else
			break;
	}

	for (retries=0; retries < 5; retries++) {
		/* Then check for rx_pcs_ready */
		reg = readl(&priv->eth_reconfig->phy_rx_pcs_status_for_anlt);
		reg &= PHY_RX_ALIGNED;
		if (reg == 0x1)
			rx_ready = 1;

		if (rx_ready != 1) {
			printk("Crossfield eHIP Driver: Resetting the 100G HIP receiver.\n");
			if (retries == 4) {
				printk("Crossfield eHIP Driver: Failed to align the receiver.\n");
				return -2;
			}

			/* Issue an Ethernet system reset to the HIP core */
			writel(PHY_SOFT_RX_RST, &priv->eth_reconfig->phy_config);
			udelay(1);
			writel(0x0, &priv->eth_reconfig->phy_config);
			mdelay(20);
		} else
			break;
	}

	printk("Crossfield eHIP Driver: Interace is ready for link up.\n");

	reg = readl(&priv->eth_reconfig->anlt_sequencer_config);
    reg &= ~ANLT_SEQ_AN_TIMEOUT;
	writel(reg, &priv->eth_reconfig->anlt_sequencer_config);

	writel(0x1, &priv->eth_reconfig->phy_clear_frame_errors);

	priv->phy_iface = PHY_INTERFACE_MODE_NA;
	priv->phy_name = "internal PHY";
	priv->oldlink = 0;
	priv->oldspeed = 0;
	priv->oldduplex = -1;

	phylink = phylink_create(dev, priv->device->of_node, priv->phy_iface, &ctl_ehip_phylink_ops);

	if (phylink == NULL) {
		netdev_err(dev, "could not create phylink.\n");
		return 0;
	}

	priv->phy_link = phylink;

	netdev_dbg(dev, "attached to 100G HIP PHY.\n");

	return 0;
}

static void ctl_ehip_update_mac_addr(struct ctl_ehip_private *priv, u8 *addr)
{
	u32 msb;
	u32 lsb;
	u32 dat;

	msb = (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | addr[5];
	lsb = ((addr[0] << 8) | addr[1]) & 0xffff;

	/* Set primary MAC address */
	writel(msb, &priv->eth_reconfig->txmac_src_address_low);
	writel(lsb, &priv->eth_reconfig->txmac_src_address_high);
}

/* MAC software reset.
 * When reset is triggered, the MAC function completes the current
 * transmission or reception, and subsequently disables the transmit and
 * receive logic, flushes the receive FIFO buffer, and resets the statistics
 * counters.
 */
static int reset_mac(struct ctl_ehip_private *priv)
{
/*	For now don't issue resets.
*/

	return 0;
}

/* Initialize MAC core registers
*/
static int init_mac(struct ctl_ehip_private *priv)
{
/*	Let the MAC come up in the default state and update the MAC address.
*/

	ctl_ehip_update_mac_addr(priv, priv->dev->dev_addr);

	return 0;
}

/* Start/stop MAC transmission logic
 */
static void ctl_ehip_set_mac(struct ctl_ehip_private *priv, bool enable)
{
	u32 reg;
	reg = readl(&priv->eth_reconfig->txmac_config);

	if (enable)
		reg &= ~TX_MAC_DISABLE_TX_MAC;
	else
		reg |= TX_MAC_DISABLE_TX_MAC;

	writel(reg, &priv->eth_reconfig->txmac_config);
}

/* Change the MTU
 */
static int ctl_ehip_change_mtu(struct net_device *dev, int new_mtu)
{
	if (netif_running(dev)) {
		netdev_err(dev, "must be stopped to change its MTU\n");
		return -EBUSY;
	}

	dev->mtu = new_mtu;
	netdev_update_features(dev);

	return 0;
}

/* Set or clear the multicast filter for this adaptor
 */
static void ctl_ehip_set_rx_mode(struct net_device *dev)
{

}

/* Initialize (if necessary) the 100G HIP PCS
 */
static int init_100ghip_pcs(struct net_device *dev)
{
	//For now, just let the PCS come up in the default state.
	struct ctl_ehip_private *priv = netdev_priv(dev);

	if (ctl_ehip_pcs_scratch_test(priv, 0x00000000) &&
		ctl_ehip_pcs_scratch_test(priv, 0xFFFFFFFF) &&
		ctl_ehip_pcs_scratch_test(priv, 0xa5a5a5a5)) {
		netdev_info(dev, "PHY scratch memory test succeeded.\n");
	} else {
		netdev_err(dev, "PHY scratch memory test failed.\n");
		return -ENOMEM;
	}

	return 0;
}

static int ctl_ehip_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
/*	struct ctl_ehip_private *priv = netdev_priv(dev);

	switch (cmd) {
		case SIOREGDUMP:
			ctl_ehip_regdump(priv);
			return 0;
		default:
			return -EOPNOTSUPP;
	}
*/
	return 0;
}

/* Open and initialize the interface
 */
static int ctl_ehip_open(struct net_device *dev)
{
	struct ctl_ehip_private *priv = netdev_priv(dev);
	int ret = 0;
	int i;
	unsigned long int flags;

	/* Reset and configure 100G HIP MAC and probe associated PHY */
	ret = priv->dmaops->init_dma(priv);
	if (ret != 0) {
		netdev_err(dev, "Cannot initialize DMA\n");
		goto phy_error;
	}

	if (netif_msg_ifup(priv))
		netdev_warn(dev, "device MAC address %pM\n",
			    dev->dev_addr);

	if ((priv->revision < 0xd00) || (priv->revision > 0xe00))
		netdev_warn(dev, "100G HIP revision %x\n", priv->revision);

	spin_lock(&priv->mac_cfg_lock);
	/* no-op if MAC not operating in SGMII mode*/
	ret = init_100ghip_pcs(dev);
	if (ret) {
		netdev_err(dev,
			   "Cannot init the 100G HIP PCS (error: %d)\n", ret);
		spin_unlock(&priv->mac_cfg_lock);
		goto phy_error;
	}

	ret = reset_mac(priv);
	/* Note that reset_mac will fail if the clocks are gated by the PHY
	 * due to the PHY being put into isolation or power down mode.
	 * This is not an error if reset fails due to no clock.
	 */
	if (ret)
		netdev_dbg(dev, "Cannot reset MAC core (error: %d)\n", ret);

	ret = init_mac(priv);
	spin_unlock(&priv->mac_cfg_lock);
	if (ret) {
		netdev_err(dev, "Cannot init MAC core (error: %d)\n", ret);
		goto alloc_skbuf_error;
	}

	//priv->dmaops->reset_dma(priv);

	/* Create and initialize the TX/RX descriptors chains. */
	priv->rx_ring_size = dma_rx_num;
	priv->tx_ring_size = dma_tx_num;
	ret = alloc_init_skbufs(priv);
	if (ret) {
		netdev_err(dev, "DMA descriptors initialization failed\n");
		goto alloc_skbuf_error;
	}


	/* Register RX interrupt */
	ret = request_irq(priv->rx_irq, crossfield_rx_isr, 0,
			  dev->name, dev);
	if (ret) {
		netdev_err(dev, "Unable to register RX interrupt %d\n",
			   priv->rx_irq);
		goto init_error;
	}

	/* Register TX interrupt */
	ret = request_irq(priv->tx_irq, crossfield_tx_isr, 0,
			  dev->name, dev);
	if (ret) {
		netdev_err(dev, "Unable to register TX interrupt %d\n",
			   priv->tx_irq);
		goto tx_request_irq_error;
	}

	/* Enable DMA interrupts */
	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
	priv->dmaops->clear_rxirq(priv);
	priv->dmaops->enable_rxirq(priv);

	/* Setup RX descriptor chain */
	for (i = 0; i < priv->rx_ring_size; i++)
		priv->dmaops->add_rx_desc(priv, &priv->rx_ring[i]);

	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	spin_lock_irqsave(&priv->txdma_irq_lock, flags);
	priv->dmaops->clear_txirq(priv);
	priv->dmaops->enable_txirq(priv);
	spin_unlock_irqrestore(&priv->txdma_irq_lock, flags);

	if (priv->phy_link)
		phylink_start(priv->phy_link);

	napi_enable(&priv->rx_napi);
	napi_enable(&priv->tx_napi);
	netif_start_queue(dev);

	priv->dmaops->start_rxdma(priv);
	priv->dmaops->start_txdma(priv);

	/* Start MAC Rx/Tx */
	spin_lock(&priv->mac_cfg_lock);
	ctl_ehip_set_mac(priv, true);
	spin_unlock(&priv->mac_cfg_lock);

	return 0;

tx_request_irq_error:
	free_irq(priv->rx_irq, dev);
	free_irq(priv->tx_irq, dev);
init_error:
	free_skbufs(dev);
alloc_skbuf_error:
phy_error:
	return ret;
}

/* Stop 100G HIP MAC interface and put the device in an inactive state
 */
static int ctl_ehip_shutdown(struct net_device *dev)
{
	struct ctl_ehip_private *priv = netdev_priv(dev);
	int ret;
	unsigned long int flags;

	netif_stop_queue(dev);
	napi_disable(&priv->rx_napi);
	napi_disable(&priv->tx_napi);

	if (priv->phy_link)
		phylink_stop(priv->phy_link);

	/* Disable DMA interrupts */
	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
	priv->dmaops->disable_rxirq(priv);
	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	spin_lock_irqsave(&priv->txdma_irq_lock, flags);
	priv->dmaops->disable_txirq(priv);
	spin_unlock_irqrestore(&priv->txdma_irq_lock, flags);

	/* Free the IRQ lines */
	free_irq(priv->rx_irq, dev);
	free_irq(priv->tx_irq, dev);

	/* disable and reset the MAC, empties fifo */
	spin_lock(&priv->mac_cfg_lock);
	spin_lock(&priv->tx_lock);

	ret = reset_mac(priv);
	/* Note that reset_mac will fail if the clocks are gated by the PHY
	 * due to the PHY being put into isolation or power down mode.
	 * This is not an error if reset fails due to no clock.
	 */
	if (ret)
		netdev_dbg(dev, "Cannot reset MAC core (error: %d)\n", ret);
	priv->dmaops->reset_dma(priv);
	free_skbufs(dev);

	spin_unlock(&priv->tx_lock);
	spin_unlock(&priv->mac_cfg_lock);

	priv->dmaops->uninit_dma(priv);

	return 0;
}

static struct net_device_ops ctl_ehip_netdev_ops = {
	.ndo_open			= ctl_ehip_open,
	.ndo_stop			= ctl_ehip_shutdown,
	.ndo_start_xmit		= ctl_ehip_start_xmit,
	.ndo_set_rx_mode	= ctl_ehip_set_rx_mode,
	.ndo_change_mtu		= ctl_ehip_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl		= ctl_ehip_do_ioctl,
};

/* Map a memory region based on resource name
 */
static int request_and_map(struct platform_device *pdev, const char *name,
			   struct resource **res, void __iomem **ptr)
{
	struct resource *region;
	struct device *device = &pdev->dev;

	*res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (*res == NULL) {
		dev_err(device, "resource %s not defined\n", name);
		return -ENODEV;
	}

	region = devm_request_mem_region(device, (*res)->start,
					 resource_size(*res), dev_name(device));
	if (region == NULL) {
		dev_err(device, "unable to request %s\n", name);
		return -EBUSY;
	}

	*ptr = devm_ioremap_nocache(device, region->start,
				    resource_size(region));
	if (*ptr == NULL) {
		dev_err(device, "ioremap_nocache of %s failed!", name);
		return -ENOMEM;
	}

	return 0;
}

/* Platform Driver functions */

/* Probe Intel 100G HIP MAC device
 */
static int ctl_ehip_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	int ret = -ENODEV;
	struct resource *eth_reconfig;
	struct resource *xcvr_reconfig0, *xcvr_reconfig1, *xcvr_reconfig2, *xcvr_reconfig3;
	struct resource *sysid;
	struct resource *dma_res;
	struct ctl_ehip_private *priv;
	const unsigned char *macaddr;
	const struct of_device_id *of_id = NULL;

	ndev = alloc_etherdev(sizeof(struct ctl_ehip_private));
	if (!ndev) {
		dev_err(&pdev->dev, "Could not allocate network device\n");
		return -ENODEV;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);

	priv = netdev_priv(ndev);
	priv->device = &pdev->dev;
	priv->dev = ndev;
	priv->msg_enable = netif_msg_init(debug, default_msg_level);

	of_id = of_match_device(ctl_ehip_ids, &pdev->dev);

	if (of_id)
		priv->dmaops = (struct crossfield_dmaops *)of_id->data;

	if (priv->dmaops &&
		   priv->dmaops->crossfield_dtype == CROSSFIELD_DTYPE_EHIP_DMA) {
/*		ret = request_and_map(pdev, "rx_resp", &dma_res,
				      (void __iomem **)&priv->rx_dma_resp);
		if (ret)
			goto err_free_netdev;
*/
		ret = request_and_map(pdev, "tx_desc", &dma_res,
				      (void __iomem **)&priv->tx_dma_desc);
		if (ret)
			goto err_free_netdev;

		priv->txdescmem = resource_size(dma_res);
		priv->txdescmem_busaddr = dma_res->start;

		ret = request_and_map(pdev, "rx_desc", &dma_res,
				      (void __iomem **)&priv->rx_dma_desc);
		if (ret)
			goto err_free_netdev;

		priv->rxdescmem = resource_size(dma_res);
		priv->rxdescmem_busaddr = dma_res->start;

	} else {
		goto err_free_netdev;
	}

	if (dma_set_mask(priv->device, DMA_BIT_MASK(priv->dmaops->dmamask))) {
		netdev_warn(ndev, ": %d-bit DMA addressing is not supported");
		goto err_free_netdev;
	}

	/*
	if (!dma_set_mask(priv->device, DMA_BIT_MASK(priv->dmaops->dmamask)))
		dma_set_coherent_mask(priv->device,
				      DMA_BIT_MASK(priv->dmaops->dmamask));
	else if (!dma_set_mask(priv->device, DMA_BIT_MASK(32)))
		dma_set_coherent_mask(priv->device, DMA_BIT_MASK(32));
	else
		goto err_free_netdev;
	*/

	/* Reconfiguration address space */
	ret = request_and_map(pdev, "eth_reconfig", &eth_reconfig,
			      (void __iomem **)&priv->eth_reconfig);
	if (ret)
		goto err_free_netdev;

	ret = request_and_map(pdev, "xcvr_reconfig0", &xcvr_reconfig0,
			      (void __iomem **)&priv->xcvr_reconfig0);
	if (ret)
		goto err_free_netdev;

	ret = request_and_map(pdev, "xcvr_reconfig1", &xcvr_reconfig1,
			      (void __iomem **)&priv->xcvr_reconfig1);
	if (ret)
		goto err_free_netdev;

	ret = request_and_map(pdev, "xcvr_reconfig2", &xcvr_reconfig2,
			      (void __iomem **)&priv->xcvr_reconfig2);
	if (ret)
		goto err_free_netdev;

	ret = request_and_map(pdev, "xcvr_reconfig3", &xcvr_reconfig3,
			      (void __iomem **)&priv->xcvr_reconfig3);
	if (ret)
		goto err_free_netdev;

	ret = request_and_map(pdev, "sysid", &sysid,
				(void __iomem **)&priv->sysid);
	if (ret)
		goto err_free_netdev;


	/* xSGDMA Rx Dispatcher address space */
	ret = request_and_map(pdev, "rx_csr", &dma_res,
			      (void __iomem **)&priv->rx_dma_csr);
	if (ret)
		goto err_free_netdev;


	/* xSGDMA Tx Dispatcher address space */
	ret = request_and_map(pdev, "tx_csr", &dma_res,
			      (void __iomem **)&priv->tx_dma_csr);
	if (ret)
		goto err_free_netdev;


	/* Rx IRQ */
	priv->rx_irq = platform_get_irq_byname(pdev, "rx_irq");
	if (priv->rx_irq == -ENXIO) {
		dev_err(&pdev->dev, "cannot obtain Rx IRQ\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	/* Tx IRQ */
	priv->tx_irq = platform_get_irq_byname(pdev, "tx_irq");
	if (priv->tx_irq == -ENXIO) {
		dev_err(&pdev->dev, "cannot obtain Tx IRQ\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	/* get FIFO depths from device tree */
	if (of_property_read_u32(pdev->dev.of_node, "rx-fifo-depth",
				 &priv->rx_fifo_depth)) {
		dev_err(&pdev->dev, "cannot obtain rx-fifo-depth\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	if (of_property_read_u32(pdev->dev.of_node, "tx-fifo-depth",
				 &priv->tx_fifo_depth)) {
		dev_err(&pdev->dev, "cannot obtain tx-fifo-depth\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	priv->hash_filter = 0;

	priv->added_unicast =
		of_property_read_bool(pdev->dev.of_node,
				      "altr,has-supplementary-unicast");

	priv->dev->min_mtu = ETH_ZLEN + ETH_FCS_LEN;
	/* Max MTU is 1500, ETH_DATA_LEN */
	priv->dev->max_mtu = ETH_DATA_LEN;

	/* Get the max mtu from the device tree. Note that the
	 * "max-frame-size" parameter is actually max mtu. Definition
	 * in the ePAPR v1.1 spec and usage differ, so go with usage.
	 */
	of_property_read_u32(pdev->dev.of_node, "max-frame-size",
			     &priv->dev->max_mtu);

	/* The DMA buffer size already accounts for an alignment bias
	 * to avoid unaligned access exceptions for the NIOS processor,
	 */
	priv->rx_dma_buf_sz = EHIP_RXDMABUFFER_SIZE;

	/* get default MAC address from device tree */
	macaddr = of_get_mac_address(pdev->dev.of_node);
	if (macaddr)
		ether_addr_copy(ndev->dev_addr, macaddr);
	else
		eth_hw_addr_random(ndev);

	/* initialize netdev */
	ndev->mem_start = eth_reconfig->start;
	ndev->mem_end = eth_reconfig->end;
	ndev->netdev_ops = &ctl_ehip_netdev_ops;
	
	ctl_ehip_set_ethtool_ops(ndev);

	ctl_ehip_netdev_ops.ndo_set_rx_mode = ctl_ehip_set_rx_mode;

	/* Scatter/gather IO is not supported,
	 * so it is turned off
	 */
	ndev->hw_features &= ~NETIF_F_SG;
	ndev->features |= ndev->hw_features | NETIF_F_HIGHDMA;

	/* VLAN offloading of tagging, stripping and filtering is not
	 * supported by hardware, but driver will accommodate the
	 * extra 4-byte VLAN tag for processing by upper layers
	 */
	ndev->features |= NETIF_F_HW_VLAN_CTAG_RX;

	/* setup NAPI interface */
	netif_napi_add(ndev, &priv->rx_napi, ctl_ehip_rx_poll, NAPI_POLL_WEIGHT);
	netif_napi_add(ndev, &priv->tx_napi, ctl_ehip_tx_poll, NAPI_POLL_WEIGHT);

	spin_lock_init(&priv->mac_cfg_lock);
	spin_lock_init(&priv->tx_lock);
	spin_lock_init(&priv->rxdma_irq_lock);
	spin_lock_init(&priv->txdma_irq_lock);

	netif_carrier_off(ndev);
	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register 100G HIP net device\n");
		goto err_register_netdev;
	}

	platform_set_drvdata(pdev, ndev);

	priv->revision = readl(&priv->eth_reconfig->phy_revision_id);

	if (netif_msg_probe(priv))
		dev_info(&pdev->dev, "Intel 100G MAC+PCS version %d.%d at 0x%08lx irq 0x%08x/0x%08x\n",
			 (priv->revision >> 8) & 0xff,
			 priv->revision & 0xff,
			 (unsigned long) eth_reconfig->start, priv->rx_irq,
			 priv->tx_irq);

	ret = init_phy(ndev);
	if (ret != 0) {
		netdev_err(ndev, "Cannot attach to the PHY (error: %d)\n", ret);
		goto err_init_phy;
	}

	/* Check to make sure the core is ready */
    ctl_ehip_regdump(priv);

	return 0;

err_init_phy:
	unregister_netdev(ndev);
err_register_netdev:
	netif_napi_del(&priv->rx_napi);
	netif_napi_del(&priv->tx_napi);
err_free_netdev:
	free_netdev(ndev);
	return ret;
}

/* Remove the device driver
 */
static int ctl_ehip_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct ctl_ehip_private *priv = netdev_priv(ndev);

	if (ndev->phydev) {
		phy_disconnect(ndev->phydev);

		if (of_phy_is_fixed_link(priv->device->of_node))
			of_phy_deregister_fixed_link(priv->device->of_node);
	}

	platform_set_drvdata(pdev, NULL);
	unregister_netdev(ndev);
	free_netdev(ndev);

	return 0;
}



static const struct crossfield_dmaops ctl_dtype_ehip_dma = {
	.crossfield_dtype = CROSSFIELD_DTYPE_EHIP_DMA,
	.dmamask = 32,
	.reset_dma = ctl_ehip_dma_reset,
	.enable_txirq = ctl_ehip_dma_enable_txirq,
	.enable_rxirq = ctl_ehip_dma_enable_rxirq,
	.disable_txirq = ctl_ehip_dma_disable_txirq,
	.disable_rxirq = ctl_ehip_dma_disable_rxirq,
	.clear_txirq = ctl_ehip_dma_clear_txirq,
	.clear_rxirq = ctl_ehip_dma_clear_rxirq,
	.rxirq_status = ctl_ehip_dma_rxirq_status,
	.txirq_status = ctl_ehip_dma_txirq_status,
	.tx_buffer = ctl_ehip_dma_tx_buffer,
	.tx_completions = ctl_ehip_dma_tx_completions,
	.add_rx_desc = ctl_ehip_dma_add_rx_desc,
	.get_rx_status = ctl_ehip_dma_rx_status,
	.init_dma = ctl_ehip_dma_initialize,
	.uninit_dma = ctl_ehip_dma_uninitialize,
	.start_rxdma = ctl_ehip_dma_start_rxdma,
	.start_txdma = ctl_ehip_dma_start_txdma,
	.start_rxdisp = ctl_ehip_dma_start_rxdisp,
	.start_txdisp = ctl_ehip_dma_start_txdisp,
};

static const struct of_device_id ctl_ehip_ids[] = {
	{ .compatible = "crossfield,ehip-dma-1.0", .data = &ctl_dtype_ehip_dma, },
	{},
};
MODULE_DEVICE_TABLE(of, ctl_ehip_ids);

static struct platform_driver ctl_ehip_driver = {
	.probe		= ctl_ehip_probe,
	.remove		= ctl_ehip_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= CTL_EHIP_RESOURCE_NAME,
		.of_match_table = ctl_ehip_ids,
	},
};

module_platform_driver(ctl_ehip_driver);

MODULE_AUTHOR("Crossfield Technology");
MODULE_DESCRIPTION("Crossfield Ethernet Hard IP Driver for Intel FPGA SoCs");
MODULE_LICENSE("GPL v2");
