/*
 * Intel Stratix 10 100G Ethernet Hard IP Driver
 * Copyright (C) 2020 Crossfield Technology LLC. All rights reserved.
 *
 * Contributors:
 *   Brett McMillian
 *
 * This driver is based on the Altera TSE driver and must be used
 * with the altera_s10_msgdma driver.
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
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <asm/cacheflush.h>

#include "altera_s10_100ghip.h"
#include "altera_s10_msgdma.h"



static atomic_t instance_count = ATOMIC_INIT(~0);
/* Module parameters */
static int debug = -1;
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Message Level (-1: default, 0: no output, 16: all)");

static const u32 default_msg_level = (NETIF_MSG_DRV | NETIF_MSG_PROBE |
					NETIF_MSG_LINK | NETIF_MSG_IFUP |
					NETIF_MSG_IFDOWN);

#define RX_DESCRIPTORS 64
static int dma_rx_num = RX_DESCRIPTORS;
module_param(dma_rx_num, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dma_rx_num, "Number of descriptors in the RX list");

#define TX_DESCRIPTORS 64
static int dma_tx_num = TX_DESCRIPTORS;
module_param(dma_tx_num, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dma_tx_num, "Number of descriptors in the TX list");


#define POLL_PHY (-1)

/* Make sure DMA buffer size is larger than the max frame size
 * plus some alignment offset and a VLAN header. If the max frame size is
 * 1518, a VLAN header would be additional 4 bytes and additional
 * headroom for alignment is 2 bytes, 2048 is just fine.
 */
#define ALTERA_RXDMABUFFER_SIZE	2048

/* Allow network stack to resume queueing packets after we've
 * finished transmitting at least 1/4 of the packets in the queue.
 */
#define S10_100GHIP_TX_THRESH(x)	(x->tx_ring_size / 4)

#define TXQUEUESTOP_THRESHHOLD	2

static const struct of_device_id altera_s10_100ghip_ids[];



static inline u32 s10_100ghip_tx_avail(struct altera_s10_100ghip_private *priv)
{
	return priv->tx_cons + priv->tx_ring_size - priv->tx_prod - 1;
}

/*
 * PCS Register read/write functions
 */
static u16 s10_100ghip_pcs_read(struct altera_s10_100ghip_private *priv, int regnum)
{
	return 0x0;
}

static void s10_100ghip_pcs_write(struct altera_s10_100ghip_private *priv, int regnum,
				u16 value)
{

}

/* Check PCS scratch memory */
static int s10_100ghip_pcs_scratch_test(struct altera_s10_100ghip_private *priv, u32 value)
{
	writel(&priv->eth_reconfig->phy_scratch, value);
	return (readl(&priv->eth_reconfig->phy_scratch) == value);
}

static int s10_100ghip_init_rx_buffer(struct altera_s10_100ghip_private *priv,
			      struct s10_100ghip_buffer *rxbuffer, int len)
{
	rxbuffer->skb = netdev_alloc_skb_ip_align(priv->dev, len);
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
	rxbuffer->dma_addr &= (dma_addr_t)~3;
	rxbuffer->len = len;
	return 0;
}

static void s10_100ghip_free_rx_buffer(struct altera_s10_100ghip_private *priv,
			       struct s10_100ghip_buffer *rxbuffer)
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
static void s10_100ghip_free_tx_buffer(struct altera_s10_100ghip_private *priv,
			       struct s10_100ghip_buffer *buffer)
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

static int alloc_init_skbufs(struct altera_s10_100ghip_private *priv)
{
	unsigned int rx_descs = priv->rx_ring_size;
	unsigned int tx_descs = priv->tx_ring_size;
	int ret = -ENOMEM;
	int i;

	/* Create Rx ring buffer */
	priv->rx_ring = kcalloc(rx_descs, sizeof(struct s10_100ghip_buffer),
				GFP_KERNEL);
	if (!priv->rx_ring)
		goto err_rx_ring;

	/* Create Tx ring buffer */
	priv->tx_ring = kcalloc(tx_descs, sizeof(struct s10_100ghip_buffer),
				GFP_KERNEL);
	if (!priv->tx_ring)
		goto err_tx_ring;

	priv->tx_cons = 0;
	priv->tx_prod = 0;

	/* Init Rx ring */
	for (i = 0; i < rx_descs; i++) {
		ret = s10_100ghip_init_rx_buffer(priv, &priv->rx_ring[i],
					 priv->rx_dma_buf_sz);
		if (ret)
			goto err_init_rx_buffers;
	}

	priv->rx_cons = 0;
	priv->rx_prod = 0;

	return 0;
err_init_rx_buffers:
	while (--i >= 0)
		s10_100ghip_free_rx_buffer(priv, &priv->rx_ring[i]);
	kfree(priv->tx_ring);
err_tx_ring:
	kfree(priv->rx_ring);
err_rx_ring:
	return ret;
}

static void free_skbufs(struct net_device *dev)
{
	struct altera_s10_100ghip_private *priv = netdev_priv(dev);
	unsigned int rx_descs = priv->rx_ring_size;
	unsigned int tx_descs = priv->tx_ring_size;
	int i;

	/* Release the DMA TX/RX socket buffers */
	for (i = 0; i < rx_descs; i++)
		s10_100ghip_free_rx_buffer(priv, &priv->rx_ring[i]);
	for (i = 0; i < tx_descs; i++)
		s10_100ghip_free_tx_buffer(priv, &priv->tx_ring[i]);


	kfree(priv->tx_ring);
}

/* Reallocate the skb for the reception process
 */
static inline void s10_100ghip_rx_refill(struct altera_s10_100ghip_private *priv)
{
	unsigned int rxsize = priv->rx_ring_size;
	unsigned int entry;
	int ret;

	for (; priv->rx_cons - priv->rx_prod > 0;
			priv->rx_prod++) {
		entry = priv->rx_prod % rxsize;
		if (likely(priv->rx_ring[entry].skb == NULL)) {
			ret = s10_100ghip_init_rx_buffer(priv, &priv->rx_ring[entry],
				priv->rx_dma_buf_sz);
			if (unlikely(ret != 0))
				break;
			priv->dmaops->add_rx_desc(priv, &priv->rx_ring[entry]);
		}
	}
}

/* Pull out the VLAN tag and fix up the packet
 */
static inline void s10_100ghip_rx_vlan(struct net_device *dev, struct sk_buff *skb)
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
static int s10_100ghip_rx(struct altera_s10_100ghip_private *priv, int limit)
{
	unsigned int count = 0;
	unsigned int next_entry;
	struct sk_buff *skb;
	unsigned int entry = priv->rx_cons % priv->rx_ring_size;
	u32 rxstatus;
	u16 pktlength;
	u16 pktstatus;

	printk("altera_s10_100ghip: Receive a packet.\n");

	/* Check for count < limit first as get_rx_status is changing
	* the response-fifo so we must process the next packet
	* after calling get_rx_status if a response is pending.
	* (reading the last byte of the response pops the value from the fifo.)
	*/
	while ((count < limit) &&
	       ((rxstatus = priv->dmaops->get_rx_status(priv)) != 0)) {
		pktstatus = rxstatus >> 16;
		pktlength = rxstatus & 0xffff;

		if ((pktstatus & 0xFF) || (pktlength == 0))
			netdev_err(priv->dev,
				   "RCV pktstatus %08X pktlength %08X\n",
				   pktstatus, pktlength);

		/* DMA transfer from 100G HIP starts with 2 aditional bytes for
		 * IP payload alignment. Status returned by get_rx_status()
		 * contains DMA transfer length. Packet is 2 bytes shorter.
		 */
		pktlength -= 2;

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

		skb_put(skb, pktlength);

		dma_unmap_single(priv->device, priv->rx_ring[entry].dma_addr,
				 priv->rx_ring[entry].len, DMA_FROM_DEVICE);

		if (netif_msg_pktdata(priv)) {
			netdev_info(priv->dev, "frame received %d bytes\n",
				    pktlength);
			print_hex_dump(KERN_ERR, "data: ", DUMP_PREFIX_OFFSET,
				       16, 1, skb->data, pktlength, true);
		}

		s10_100ghip_rx_vlan(priv->dev, skb);

		skb->protocol = eth_type_trans(skb, priv->dev);
		skb_checksum_none_assert(skb);

		napi_gro_receive(&priv->napi, skb);

		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += pktlength;

		entry = next_entry;

		s10_100ghip_rx_refill(priv);
	}

	return count;
}

/* Reclaim resources after transmission completes
 */
static int s10_100ghip_tx_complete(struct altera_s10_100ghip_private *priv)
{
	unsigned int txsize = priv->tx_ring_size;
	u32 ready;
	unsigned int entry;
	struct s10_100ghip_buffer *tx_buff;
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

		s10_100ghip_free_tx_buffer(priv, tx_buff);
		priv->tx_cons++;

		txcomplete++;
		ready--;
	}

	if (unlikely(netif_queue_stopped(priv->dev) &&
		     s10_100ghip_tx_avail(priv) > S10_100GHIP_TX_THRESH(priv))) {
		if (netif_queue_stopped(priv->dev) &&
		    s10_100ghip_tx_avail(priv) > S10_100GHIP_TX_THRESH(priv)) {
			if (netif_msg_tx_done(priv))
				netdev_dbg(priv->dev, "%s: restart transmit\n",
					   __func__);
			netif_wake_queue(priv->dev);
		}
	}

	spin_unlock(&priv->tx_lock);
	return txcomplete;
}

/*
 * NAPI polling function
 */
static int s10_100ghip_poll(struct napi_struct *napi, int budget)
{
	struct altera_s10_100ghip_private *priv =
			container_of(napi, struct altera_s10_100ghip_private, napi);
	int rxcomplete = 0;
	unsigned long int flags;

	s10_100ghip_tx_complete(priv);

	rxcomplete = s10_100ghip_rx(priv, budget);

	if (rxcomplete < budget) {

		napi_complete_done(napi, rxcomplete);

		netdev_dbg(priv->dev,
			   "NAPI Complete, did %d packets with budget %d\n",
			   rxcomplete, budget);

		spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
		priv->dmaops->enable_rxirq(priv);
		priv->dmaops->enable_txirq(priv);
		spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);
	}
	return rxcomplete;
}

/*
 * DMA TX & RX FIFO interrupt routing
 */
static irqreturn_t altera_isr(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct altera_s10_100ghip_private *priv;

	if (unlikely(!dev)) {
		pr_err("%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}
	priv = netdev_priv(dev);

	spin_lock(&priv->rxdma_irq_lock);
	/* reset IRQs */
	priv->dmaops->clear_rxirq(priv);
	priv->dmaops->clear_txirq(priv);
	spin_unlock(&priv->rxdma_irq_lock);

	if (likely(napi_schedule_prep(&priv->napi))) {
		spin_lock(&priv->rxdma_irq_lock);
		priv->dmaops->disable_rxirq(priv);
		priv->dmaops->disable_txirq(priv);
		spin_unlock(&priv->rxdma_irq_lock);
		__napi_schedule(&priv->napi);
	}


	return IRQ_HANDLED;
}

/* Transmit a packet (called by the kernel). Dispatches
 * the MSGDMA method, assumes no scatter/gather support,
 * implying an assumption that there's only one
 * physically contiguous fragment starting at
 * skb->data, for length of skb_headlen(skb).
 */
static int s10_100ghip_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct altera_s10_100ghip_private *priv = netdev_priv(dev);
	unsigned int txsize = priv->tx_ring_size;
	unsigned int entry;
	struct s10_100ghip_buffer *buffer = NULL;
	int nfrags = skb_shinfo(skb)->nr_frags;
	unsigned int nopaged_len = skb_headlen(skb);
	enum netdev_tx ret = NETDEV_TX_OK;
	dma_addr_t dma_addr;

	spin_lock_bh(&priv->tx_lock);

	if (unlikely(s10_100ghip_tx_avail(priv) < nfrags + 1)) {
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

	dma_addr = dma_map_single(priv->device, skb->data, nopaged_len,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(priv->device, dma_addr)) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		ret = NETDEV_TX_OK;
		goto out;
	}

	buffer->skb = skb;
	buffer->dma_addr = dma_addr;
	buffer->len = nopaged_len;

	priv->dmaops->tx_buffer(priv, buffer);

	skb_tx_timestamp(skb);

	priv->tx_prod++;
	dev->stats.tx_bytes += skb->len;

	if (unlikely(s10_100ghip_tx_avail(priv) <= TXQUEUESTOP_THRESHHOLD)) {
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
static void altera_s10_100ghip_adjust_link(struct net_device *dev)
{
	struct altera_s10_100ghip_private *priv = netdev_priv(dev);
	struct phy_device *phydev = dev->phydev;

	if (netif_msg_link(priv))
		phy_print_status(phydev);
}


static struct phy_device *connect_local_phy(struct net_device *dev)
{

}


static int altera_s10_100ghip_phy_get_addr_mdio_create(struct net_device *dev)
{

}
*/

/* Initialize driver's PHY state, and attach to the PHY */
 
static int init_phy(struct net_device *dev)
{
	/*
	struct altera_s10_100ghip_private *priv = netdev_priv(dev);
	struct phy_device *phydev == NULL;
	int ret;

	priv->phy_iface = PHY_INTERFACE_MODE_NA;
	priv->oldlink = 0;
	priv->oldspeed = 0;
	priv->oldduplex = -1;

	phydev = phy_connect(dev, NULL, &altera_s10_100ghip_adjust_link, priv->phy_iface);

	ret = phy_connect_direct(dev, phydev, &altera_s10_100ghip_adjust_link, priv->phy_iface);

	if (ret != 0) {
		netdev_err(dev, "Could not attach to PHY\n");
		phydev = NULL;
		return 0;
	}
	
	phydev->advertising &= SUPPORTED_100000baseSR4_Full;

	netdev_dbg(dev, "attached to 100G HIP PHY.\n");
*/
	return 0;
}


static void s10_100ghip_update_mac_addr(struct altera_s10_100ghip_private *priv, u8 *addr)
{
	u32 msb;
	u32 lsb;
	u32 dat;

	dat = csrrd32(priv->eth_reconfig, s10_100ghip_ethreconfigoffs(txmac_config));
	dat |= TX_MAC_EN_SADDR_INSERT;

	msb = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
	lsb = ((addr[5] << 8) | addr[4]) & 0xffff;

	/* Set primary MAC address */
	csrwr32(msb, priv->eth_reconfig, s10_100ghip_ethreconfigoffs(txmac_src_address_low));
	csrwr32(lsb, priv->eth_reconfig, s10_100ghip_ethreconfigoffs(txmac_src_address_high));

	csrwr32(dat, priv->eth_reconfig, s10_100ghip_ethreconfigoffs(txmac_config));
}

/* MAC software reset.
 * When reset is triggered, the MAC function completes the current
 * transmission or reception, and subsequently disables the transmit and
 * receive logic, flushes the receive FIFO buffer, and resets the statistics
 * counters.
 */
static int reset_mac(struct altera_s10_100ghip_private *priv)
{
/*	For now don't issue resets.

	int counter;
	u32 dat;

	dat = csrrd32(priv->phy_dev, s10_100ghip_phycsroffs(config);
	dat &= 
*/

	return 0;
}

/* Initialize MAC core registers
*/
static int init_mac(struct altera_s10_100ghip_private *priv)
{
/*	Let the MAC come up in the default state and update the MAC address.
*/

	s10_100ghip_update_mac_addr(priv, priv->dev->dev_addr);

	return 0;
}

/* Start/stop MAC transmission logic
 */
static void s10_100ghip_set_mac(struct altera_s10_100ghip_private *priv, bool enable)
{
	u32 value = csrrd32(priv->eth_reconfig, s10_100ghip_ethreconfigoffs(txmac_config));

	if (enable)
		value &= ~TX_MAC_DISABLE_TX_MAC;
	else
		value |= TX_MAC_DISABLE_TX_MAC;

	csrwr32(value, priv->eth_reconfig, s10_100ghip_ethreconfigoffs(txmac_config));
}

/* Change the MTU
 */
static int s10_100ghip_change_mtu(struct net_device *dev, int new_mtu)
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
static void s10_100ghip_set_rx_mode(struct net_device *dev)
{

}

/* Initialize (if necessary) the 100G HIP PCS
 */
static int init_100ghip_pcs(struct net_device *dev)
{
	//For now, just let the PCS come up in the default state.
/*	struct altera_s10_100ghip_private *priv = netdev_priv(dev);

	if (s10_100ghip_pcs_scratch_test(priv, 0x00000000) &&
		s10_100ghip_pcs_scratch_test(priv, 0xFFFFFFFF) &&
		s10_100ghip_pcs_scratch_test(priv, 0xa5a5a5a5)) {
		netdev_info(dev, "PHY Revision ID: 0x%08x\n",
				readl(&priv->eth_reconfig->phy_revision_id));
	} else {
		netdev_err(dev, "PHY scratch memory test failed.\n");
		return -ENOMEM;
	}
*/
	return 0;
}

/* Open and initialize the interface
 */
static int s10_100ghip_open(struct net_device *dev)
{
	struct altera_s10_100ghip_private *priv = netdev_priv(dev);
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

	priv->dmaops->reset_dma(priv);

	/* Create and initialize the TX/RX descriptors chains. */
	priv->rx_ring_size = dma_rx_num;
	priv->tx_ring_size = dma_tx_num;
	ret = alloc_init_skbufs(priv);
	if (ret) {
		netdev_err(dev, "DMA descriptors initialization failed\n");
		goto alloc_skbuf_error;
	}


	/* Register RX interrupt */
	ret = request_irq(priv->rx_irq, altera_isr, IRQF_SHARED,
			  dev->name, dev);
	if (ret) {
		netdev_err(dev, "Unable to register RX interrupt %d\n",
			   priv->rx_irq);
		goto init_error;
	}

	/* Register TX interrupt */
	ret = request_irq(priv->tx_irq, altera_isr, IRQF_SHARED,
			  dev->name, dev);
	if (ret) {
		netdev_err(dev, "Unable to register TX interrupt %d\n",
			   priv->tx_irq);
		goto tx_request_irq_error;
	}

	/* Enable DMA interrupts */
	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
	priv->dmaops->enable_rxirq(priv);
	priv->dmaops->enable_txirq(priv);

	/* Setup RX descriptor chain */
	for (i = 0; i < priv->rx_ring_size; i++)
		priv->dmaops->add_rx_desc(priv, &priv->rx_ring[i]);

	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	if (dev->phydev)
		phy_start(dev->phydev);

	napi_enable(&priv->napi);
	netif_start_queue(dev);

	priv->dmaops->start_rxdma(priv);

	/* Start MAC Rx/Tx */
	spin_lock(&priv->mac_cfg_lock);
	s10_100ghip_set_mac(priv, true);
	spin_unlock(&priv->mac_cfg_lock);

	return 0;

tx_request_irq_error:
	free_irq(priv->rx_irq, dev);
init_error:
	free_skbufs(dev);
alloc_skbuf_error:
phy_error:
	return ret;
}

/* Stop 100G HIP MAC interface and put the device in an inactive state
 */
static int s10_100ghip_shutdown(struct net_device *dev)
{
	struct altera_s10_100ghip_private *priv = netdev_priv(dev);
	int ret;
	unsigned long int flags;

	netif_stop_queue(dev);
	napi_disable(&priv->napi);

	/* Disable DMA interrupts */
	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
	priv->dmaops->disable_rxirq(priv);
	priv->dmaops->disable_txirq(priv);
	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

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

static struct net_device_ops altera_s10_100ghip_netdev_ops = {
	.ndo_open		= s10_100ghip_open,
	.ndo_stop		= s10_100ghip_shutdown,
	.ndo_start_xmit		= s10_100ghip_start_xmit,
	.ndo_set_rx_mode	= s10_100ghip_set_rx_mode,
	.ndo_change_mtu		= s10_100ghip_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
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

static int altera_s10_100ghip_check(struct altera_s10_100ghip_private *priv)
{
	int i;
	u32 reg;

	reg = readl(priv->sysid);
	printk("altera_s10_100ghip: sysid = 0x%08x.\n", reg);

	printk("altera_s10_100ghip: Checking status of 100G HIP.\n");
struct altera_s10_100ghip_private *priv = netdev_priv(dev);
	struct phy_device *phydev == NULL;
	int ret;

	priv->phy_iface = PHY_INTERFACE_MODE_NA;
	priv->oldlink = 0;
	priv->oldspeed = 0;
	priv->oldduplex = -1;

	phydev = phy_connect(dev, NULL, &altera_s10_100ghip_adjust_link, priv->phy_iface);

	ret = phy_connect_direct(dev, phydev, &altera_s10_100ghip_adjust_link, priv->phy_iface);

	if (ret != 0) {
		netdev_err(dev, "Could not attach to PHY\n");
		phydev = NULL;
		return 0;
	}
	
	phydev->advertising &= SUPPORTED_100000baseSR4_Full;

	netdev_dbg(dev, "attached to 100G HIP PHY.\n");
	reg = readl(&priv->eth_reconfig->phy_revision_id);
	printk("altera_s10_100ghip: PHY Revision ID = 0x%08x\n", reg);

	return 0;
}


/* Platform Driver functions */

/* Probe Intel 100G HIP MAC device
 */
static int altera_s10_100ghip_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	int ret = -ENODEV;
	struct resource *eth_reconfig;
	struct resource *xcvr_reconfig0, *xcvr_reconfig1, *xcvr_reconfig2, *xcvr_reconfig3;
	struct resource *sysid;
	struct resource *dma_res;
	struct altera_s10_100ghip_private *priv;
	const unsigned char *macaddr;
	const struct of_device_id *of_id = NULL;

	ndev = alloc_etherdev(sizeof(struct altera_s10_100ghip_private));
	if (!ndev) {
		dev_err(&pdev->dev, "Could not allocate network device\n");
		return -ENODEV;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);

	priv = netdev_priv(ndev);
	priv->device = &pdev->dev;
	priv->dev = ndev;
	priv->msg_enable = netif_msg_init(debug, default_msg_level);

	of_id = of_match_device(altera_s10_100ghip_ids, &pdev->dev);

	if (of_id)
		priv->dmaops = (struct altera_dmaops *)of_id->data;

	if (priv->dmaops &&
		   priv->dmaops->altera_dtype == ALTERA_DTYPE_S10_MSGDMA) {
		ret = request_and_map(pdev, "rx_resp", &dma_res,
				      &priv->rx_dma_resp);
		if (ret)
			goto err_free_netdev;

		ret = request_and_map(pdev, "tx_desc", &dma_res,
				      &priv->tx_dma_desc);
		if (ret)
			goto err_free_netdev;

		priv->txdescmem = resource_size(dma_res);
		priv->txdescmem_busaddr = dma_res->start;

		ret = request_and_map(pdev, "rx_desc", &dma_res,
				      &priv->rx_dma_desc);
		if (ret)
			goto err_free_netdev;

		priv->rxdescmem = resource_size(dma_res);
		priv->rxdescmem_busaddr = dma_res->start;

	} else {
		goto err_free_netdev;
	}

	if (!dma_set_mask(priv->device, DMA_BIT_MASK(priv->dmaops->dmamask)))
		dma_set_coherent_mask(priv->device,
				      DMA_BIT_MASK(priv->dmaops->dmamask));
	else if (!dma_set_mask(priv->device, DMA_BIT_MASK(32)))
		dma_set_coherent_mask(priv->device, DMA_BIT_MASK(32));
	else
		goto err_free_netdev;

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
			      &priv->rx_dma_csr);
	if (ret)
		goto err_free_netdev;


	/* xSGDMA Tx Dispatcher address space */
	ret = request_and_map(pdev, "tx_csr", &dma_res,
			      &priv->tx_dma_csr);
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
	priv->rx_dma_buf_sz = ALTERA_RXDMABUFFER_SIZE;

	/* get default MAC address from device tree */
	macaddr = of_get_mac_address(pdev->dev.of_node);
	if (macaddr)
		ether_addr_copy(ndev->dev_addr, macaddr);
	else
		eth_hw_addr_random(ndev);

	/* Check the mSGDMA Component Configuration Registers */
	s10_msgdma_check(priv);

	/* Check to make sure the core is ready */
        ret = altera_s10_100ghip_check(priv);
	if (ret) {
		dev_err(&pdev->dev, "The 100G HIP core is not ready. Exiting.\n");
		goto err_free_netdev;
	}

	/* initialize netdev */
	ndev->mem_start = eth_reconfig->start;
	ndev->mem_end = eth_reconfig->end;
	ndev->netdev_ops = &altera_s10_100ghip_netdev_ops;
	
	altera_s10_100ghip_set_ethtool_ops(ndev);

	altera_s10_100ghip_netdev_ops.ndo_set_rx_mode = s10_100ghip_set_rx_mode;

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
	netif_napi_add(ndev, &priv->napi, s10_100ghip_poll, NAPI_POLL_WEIGHT);

	spin_lock_init(&priv->mac_cfg_lock);
	spin_lock_init(&priv->tx_lock);
	spin_lock_init(&priv->rxdma_irq_lock);

	netif_carrier_off(ndev);
	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register 100G HIP net device\n");
		goto err_register_netdev;
	}

	platform_set_drvdata(pdev, ndev);

	priv->revision = readl(&priv->eth_reconfig->phy_revision_id);

	if (netif_msg_probe(priv))
		dev_info(&pdev->dev, "Intel 100G MAC+PCS version %d.%d at 0x%08lx irq %d/%d\n",
			 (priv->revision >> 8) & 0xff,
			 priv->revision & 0xff,
			 (unsigned long) eth_reconfig->start, priv->rx_irq,
			 priv->tx_irq);

	ret = init_phy(ndev);
	if (ret != 0) {
		netdev_err(ndev, "Cannot attach to PHY (error: %d)\n", ret);
	}

	return 0;

err_register_netdev:
	netif_napi_del(&priv->napi);
err_free_netdev:
	free_netdev(ndev);
	return ret;
}

/* Remove Altera 100G HIP MAC device
 */
static int altera_s10_100ghip_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct altera_s10_100ghip_private *priv = netdev_priv(ndev);

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



static const struct altera_dmaops altera_dtype_s10_msgdma = {
	.altera_dtype = ALTERA_DTYPE_S10_MSGDMA,
	.dmamask = 64,
	.reset_dma = s10_msgdma_reset,
	.enable_txirq = s10_msgdma_enable_txirq,
	.enable_rxirq = s10_msgdma_enable_rxirq,
	.disable_txirq = s10_msgdma_disable_txirq,
	.disable_rxirq = s10_msgdma_disable_rxirq,
	.clear_txirq = s10_msgdma_clear_txirq,
	.clear_rxirq = s10_msgdma_clear_rxirq,
	.tx_buffer = s10_msgdma_tx_buffer,
	.tx_completions = s10_msgdma_tx_completions,
	.add_rx_desc = s10_msgdma_add_rx_desc,
	.get_rx_status = s10_msgdma_rx_status,
	.init_dma = s10_msgdma_initialize,
	.uninit_dma = s10_msgdma_uninitialize,
	.start_rxdma = s10_msgdma_start_rxdma,
};

static const struct of_device_id altera_s10_100ghip_ids[] = {
	{ .compatible = "altr,s10-100ghip-msgdma-1.0", .data = &altera_dtype_s10_msgdma, },
	{},
};
MODULE_DEVICE_TABLE(of, altera_s10_100ghip_ids);

static struct platform_driver altera_s10_100ghip_driver = {
	.probe		= altera_s10_100ghip_probe,
	.remove		= altera_s10_100ghip_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= ALTERA_S10_100GHIP_RESOURCE_NAME,
		.of_match_table = altera_s10_100ghip_ids,
	},
};

module_platform_driver(altera_s10_100ghip_driver);

MODULE_AUTHOR("Crossfield Technology");
MODULE_DESCRIPTION("Intel Stratix 10 100G Ethernet HIP Driver");
MODULE_LICENSE("GPL v2");
