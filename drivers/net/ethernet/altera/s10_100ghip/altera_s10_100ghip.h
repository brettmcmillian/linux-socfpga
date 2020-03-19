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

#ifndef __ALTERA_S10_100GHIP_H__
#define __ALTERA_S10_100GHIP_H__

#define ALTERA_S10_100GHIP_RESOURCE_NAME	"altera_s10_100ghip"

#include <linux/bitops.h>
#include <linux/if_vlan.h>
#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/phy.h>

#define ANLT_CSR_OFFSET = 0x2C0
#define PHY_CSR_OFFSET = 0xC00 /* (0x300 word offset) */
#define TX_MAC_CSR_OFFSET = 0x1000 /* (0x400 word offset) */
#define RX_MAC_CSR_OFFSET = 0x1400 /* (0x500 word offset) */

#define ALTERA_S10_100GHIP_SW_RESET_WATCHDOG_CNTR	10000
#define ALTERA_S10_100GHIP_MAC_FIFO_WIDTH		4	/* TX/RX FIFO width in
							 * bytes
							 */
/* Rx FIFO default settings */
#define ALTERA_S10_100GHIP_RX_SECTION_EMPTY	16
#define ALTERA_S10_100GHIP_RX_SECTION_FULL	0
#define ALTERA_S10_100GHIP_RX_ALMOST_EMPTY	8
#define ALTERA_S10_100GHIP_RX_ALMOST_FULL	8

/* Tx FIFO default settings */
#define ALTERA_S10_100GHIP_TX_SECTION_EMPTY	16
#define ALTERA_S10_100GHIP_TX_SECTION_FULL	0
#define ALTERA_S10_100GHIP_TX_ALMOST_EMPTY	8
#define ALTERA_S10_100GHIP_TX_ALMOST_FULL	3

/* MAC function configuration default settings */
#define ALTERA_S10_100GHIP_TX_IPG_LENGTH	12

#define ALTERA_S10_100GHIP_PAUSE_QUANTA		0xffff

#define GET_BIT_VALUE(v, bit)		(((v) >> (bit)) & 0x1)

/* MAC Command_Config Register Bit Definitions

#define MAC_CMDCFG_TX_ENA					BIT(0)
#define MAC_CMDCFG_RX_ENA					BIT(1)
#define MAC_CMDCFG_XON_GEN					BIT(2)
#define MAC_CMDCFG_ETH_SPEED				BIT(3)
#define MAC_CMDCFG_PROMIS_EN				BIT(4)
#define MAC_CMDCFG_PAD_EN					BIT(5)
#define MAC_CMDCFG_CRC_FWD					BIT(6)
#define MAC_CMDCFG_PAUSE_FWD				BIT(7)
#define MAC_CMDCFG_PAUSE_IGNORE				BIT(8)
#define MAC_CMDCFG_TX_ADDR_INS				BIT(9)
#define MAC_CMDCFG_HD_ENA					BIT(10)
#define MAC_CMDCFG_EXCESS_COL				BIT(11)
#define MAC_CMDCFG_LATE_COL					BIT(12)
#define MAC_CMDCFG_SW_RESET					BIT(13)
#define MAC_CMDCFG_MHASH_SEL				BIT(14)
#define MAC_CMDCFG_LOOP_ENA					BIT(15)
#define MAC_CMDCFG_TX_ADDR_SEL(v)			(((v) & 0x7) << 16)
#define MAC_CMDCFG_MAGIC_ENA				BIT(19)
#define MAC_CMDCFG_SLEEP					BIT(20)
#define MAC_CMDCFG_WAKEUP					BIT(21)
#define MAC_CMDCFG_XOFF_GEN					BIT(22)
#define MAC_CMDCFG_CNTL_FRM_ENA				BIT(23)
#define MAC_CMDCFG_NO_LGTH_CHECK			BIT(24)
#define MAC_CMDCFG_ENA_10					BIT(25)
#define MAC_CMDCFG_RX_ERR_DISC				BIT(26)
#define MAC_CMDCFG_DISABLE_READ_TIMEOUT		BIT(27)
#define MAC_CMDCFG_CNT_RESET				BIT(31)

#define MAC_CMDCFG_TX_ENA_GET(v)				GET_BIT_VALUE(v, 0)
#define MAC_CMDCFG_RX_ENA_GET(v)				GET_BIT_VALUE(v, 1)
#define MAC_CMDCFG_XON_GEN_GET(v)				GET_BIT_VALUE(v, 2)
#define MAC_CMDCFG_ETH_SPEED_GET(v)				GET_BIT_VALUE(v, 3)
#define MAC_CMDCFG_PROMIS_EN_GET(v)				GET_BIT_VALUE(v, 4)
#define MAC_CMDCFG_PAD_EN_GET(v)				GET_BIT_VALUE(v, 5)
#define MAC_CMDCFG_CRC_FWD_GET(v)				GET_BIT_VALUE(v, 6)
#define MAC_CMDCFG_PAUSE_FWD_GET(v)				GET_BIT_VALUE(v, 7)
#define MAC_CMDCFG_PAUSE_IGNORE_GET(v)			GET_BIT_VALUE(v, 8)
#define MAC_CMDCFG_TX_ADDR_INS_GET(v)			GET_BIT_VALUE(v, 9)
#define MAC_CMDCFG_HD_ENA_GET(v)				GET_BIT_VALUE(v, 10)
#define MAC_CMDCFG_EXCESS_COL_GET(v)			GET_BIT_VALUE(v, 11)
#define MAC_CMDCFG_LATE_COL_GET(v)				GET_BIT_VALUE(v, 12)
#define MAC_CMDCFG_SW_RESET_GET(v)				GET_BIT_VALUE(v, 13)
#define MAC_CMDCFG_MHASH_SEL_GET(v)				GET_BIT_VALUE(v, 14)
#define MAC_CMDCFG_LOOP_ENA_GET(v)				GET_BIT_VALUE(v, 15)
#define MAC_CMDCFG_TX_ADDR_SEL_GET(v)			(((v) >> 16) & 0x7)
#define MAC_CMDCFG_MAGIC_ENA_GET(v)				GET_BIT_VALUE(v, 19)
#define MAC_CMDCFG_SLEEP_GET(v)					GET_BIT_VALUE(v, 20)
#define MAC_CMDCFG_WAKEUP_GET(v)				GET_BIT_VALUE(v, 21)
#define MAC_CMDCFG_XOFF_GEN_GET(v)				GET_BIT_VALUE(v, 22)
#define MAC_CMDCFG_CNTL_FRM_ENA_GET(v)			GET_BIT_VALUE(v, 23)
#define MAC_CMDCFG_NO_LGTH_CHECK_GET(v)			GET_BIT_VALUE(v, 24)
#define MAC_CMDCFG_ENA_10_GET(v)				GET_BIT_VALUE(v, 25)
#define MAC_CMDCFG_RX_ERR_DISC_GET(v)			GET_BIT_VALUE(v, 26)
#define MAC_CMDCFG_DISABLE_READ_TIMEOUT_GET(v)	GET_BIT_VALUE(v, 27)
#define MAC_CMDCFG_CNT_RESET_GET(v)				GET_BIT_VALUE(v, 31)
*/
/* SGMII PCS register addresses

#define SGMII_PCS_SCRATCH			0x10
#define SGMII_PCS_REV				0x11
#define SGMII_PCS_LINK_TIMER_0		0x12
#define SGMII_PCS_LINK_TIMER_1		0x13
#define SGMII_PCS_IF_MODE			0x14
#define SGMII_PCS_DIS_READ_TO		0x15
#define SGMII_PCS_READ_TO			0x16
#define SGMII_PCS_SW_RESET_TIMEOUT 	100 
*/

#define TX_MAC_EN_SADDR_INSERT	BIT(3)
#define TX_MAC_DISABLE_TX_MAC	BIT(2)
#define TX_MAC_DISABLE_TXVLAN	BIT(1)

struct altera_s10_100ghip_ethreconfig {

	/* Auto-negotiation and link training registers are currently disabled */
	u32 anlt_padding[768];

	/* PHY registers begin at offset 0x300 double words */
	u32 phy_revision_id;
	u32 phy_scratch;
	u32 phy_unused1[14];
	u32 phy_config;
	u32 phy_unused2[2];
	u32 phy_pma_serial_loopback;
	u32 phy_unused3[12];
	u32 phy_tx_pll_locked;
	u32 phy_rx_cdr_pll_locked;
	u32 phy_tx_datapath_ready;
	u32 phy_frame_errors_detected;
	u32 phy_clear_frame_errors;
	u32 phy_reset;
	u32 phy_rx_pcs_status_for_anlt;
	u32 phy_pcs_error_injection;
	u32 phy_alignment_marker_lock;
	u32 phy_unused4;
	u32 phy_ber_count;
	u32 phy_unused5[5];
	u32 phy_pcs_virtual_lane0;
	u32 phy_pcs_virtual_lane1;
	u32 phy_pcs_virtual_lane2;
	u32 phy_pcs_virtual_lane3;
	u32 phy_unused6[12];
	u32 phy_recovered_clock_freq;
	u32 phy_tx_clock_freq;
	u32 phy_unused7[52];
	u32 phy_prog_alignment_marker0;
	u32 phy_prog_alignment_marker1;
	u32 phy_prog_alignment_marker2;
	u32 phy_prog_alignment_marker3;

	u32 phy_padding[134];

	/* TX MAC registers begin at offset 0x400 double words */
	u32 txmac_revision_id;
	u32 txmac_scratch;
	u32 txmac_reserved[3];
	u32 txmac_link_fault_config;
	u32 txmac_ipg_words;
	u32 txmac_max_tx_frame_size;
	u32 txmac_unused[2];
	u32 txmac_config;
	u32 txmac_ehip_feature_config;
	u32 txmac_src_address_low;
	u32 txmac_src_address_high;

	u32 txmac_padding[242];

	/* RX MAC registers begin at offset 0x500 double words */
	u32 rxmac_revision_id;
	u32 rxmac_scratch;
	u32 rxmac_reserved[3];
	u32 rxmac_unused1;
	u32 rxmac_max_rx_frame_size;
	u32 rxmac_rx_crc_forwarding;
	u32 rxmac_link_fault_status;
	u32 rxmac_unused2;
	u32 rxmac_config;
	u32 rxmac_ehip_feature_config;

	u32 rxmac_padding[244];

	/* Pause & Priority-Based Flow Control registers begin at offset 0x600 double words */
	u32 fc_txsfc_module_revision_id;
	u32 fc_txsfc_scratch;
	u32 fc_reserved1[3];
	u32 fc_enable_tx_pause_ports;
	u32 fc_tx_pause_request;
	u32 fc_enable_auto_tx_pause_retran;
	u32 fc_retransmit_holdoff_quanta;
	u32 fc_retransmit_pause_quanta;
	u32 fc_enable_tx_xoff;
	u32 fc_enable_uniform_holdoff;
	u32 fc_set_uniform_holdoff;
	u32 fc_dest_addr_low;
	u32 fc_dest_addr_high;
	u32 fc_src_addr_low;
	u32 fc_src_addr_high;
	u32 fc_tx_fc_feature_config;
	u32 fc_unused1[13];
	u32 fc_pause_quanta0;
	u32 fc_pause_quanta1;
	u32 fc_pause_quanta2;
	u32 fc_pause_quanta3;
	u32 fc_pause_quanta4;
	u32 fc_pause_quanta5;
	u32 fc_pause_quanta6;
	u32 fc_pause_quanta7;
	u32 fc_holdoff_quanta0;
	u32 fc_holdoff_quanta1;
	u32 fc_holdoff_quanta2;
	u32 fc_holdoff_quanta3;
	u32 fc_holdoff_quanta4;
	u32 fc_holdoff_quanta5;
	u32 fc_holdoff_quanta6;
	u32 fc_holdoff_quanta7;

	u32 fc_unused2[209];

	u32 fc_rxsfc_module_revision_id;
	u32 fc_rxsfc_scratch;
	u32 fc_reserved2[3];
	u32 fc_enable_rx_pause_frame_proc;
	u32 fc_forward_frames;
	u32 fc_rx_pause_dest_addr_low;
	u32 fc_rx_pause_dest_addr_high;

	u32 fc_padding[247];

	/* TX Statistics Counter registers begin at offset 0x800 double words */
	u32 txstat_frames_lessthan_64B_w_crcerr_low;
	u32 txstat_frames_lessthan_64B_w_crcerr_high;
	u32 txstat_oversized_frames_w_crcerr_low;
	u32 txstat_oversized_frames_w_crcerr_high;
	u32 txstat_frames_w_crcerr_low;
	u32 txstat_frames_w_crcerr_high;
	u32 txstat_frames_w_crcerr_on_ok_low;
	u32 txstat_frames_w_crcerr_on_ok_high;
	u32 txstat_multicast_data_frames_w_crcerr_low;
	u32 txstat_multicast_data_frames_w_crcerr_high;
	u32 txstat_broadcast_data_frames_w_crcerr_low;
	u32 txstat_broadcast_data_frames_w_crcerr_high;
	u32 txstat_unicast_data_frames_w_crcerr_low;
	u32 txstat_unicast_data_frames_w_crcerr_high;
	u32 txstat_multicast_control_frames_w_crcerr_low;
	u32 txstat_multicast_control_frames_w_crcerr_high;
	u32 txstat_broadcast_control_frames_w_crcerr_low;
    u32 txstat_broadcast_control_frames_w_crcerr_high;
    u32 txstat_unicast_control_frames_w_crcerr_low;
    u32 txstat_unicast_control_frames_w_crcerr_high;
	u32 txstat_pause_frame_w_crcerr_low;
	u32 txstat_pause_frame_w_crcerr_high;
	u32 txstat_64B_frames_low;
	u32 txstat_64B_frames_high;
	u32 txstat_65B_127B_frames_low;
	u32 txstat_65B_127B_frames_high;
	u32 txstat_128B_255B_frames_low;
	u32 txstat_128B_255B_frames_high;
	u32 txstat_256B_511B_frames_low;
	u32 txstat_256B_511B_frames_high;
	u32 txstat_512B_1023B_frames_low;
	u32 txstat_512B_1023B_frames_high;
	u32 txstat_1024B_1518B_frames_low;
	u32 txstat_1024B_1518B_frames_high;
	u32 txstat_1519B_max_frames_low;
	u32 txstat_1519B_max_frames_high;
	u32 txstat_oversize_frames_low;
	u32 txstat_oversize_frames_high;
	u32 txstat_multicast_data_frames_wo_err_low;
	u32 txstat_multicast_data_frames_wo_err_high;
	u32 txstat_broadcast_data_frames_wo_err_low;
	u32 txstat_broadcast_data_frames_wo_err_high;
	u32 txstat_unicast_data_frames_wo_err_low;
	u32 txstat_unicast_data_frames_wo_err_high;
	u32 txstat_multicast_control_frames_wo_err_low;
	u32 txstat_multicast_control_frames_wo_err_high;
	u32 txstat_broadcast_control_frames_wo_err_low;
	u32 txstat_broadcast_control_frames_wo_err_high;
	u32 txstat_unicast_control_frames_wo_err_low;
	u32 txstat_unicast_control_frames_wo_err_high;
	u32 txstat_pause_frame_wo_err_low;
	u32 txstat_pause_frame_wo_err_high;
    u32 txstat_frames_lessthan_64B_and_crcerr_low;
	u32 txstat_frames_lessthan_64B_and_crcerr_high;
	u32 txstat_num_frame_starts_low;
	u32 txstat_num_frame_starts_high;
	u32 txstat_num_length_errors_low;
	u32 txstat_num_length_errors_high;
	u32 txstat_pfc_frames_w_crcerr_low;
	u32 txstat_pfc_frames_w_crcerr_high;
	u32 txstat_pfc_frames_wo_err_low;
	u32 txstat_pfc_frames_wo_err_high;
	u32 txstat_unused1[2];
	u32 txstat_module_revision_id;
	u32 txstat_scratch;
	u32 txstat_reserved;
	u32 txstat_unused2[2];
	u32 txstat_config;
	u32 txstat_status;
	u32 txstat_unused3[25];
	u32 txstat_payload_wo_err_low;
	u32 txstat_payload_wo_err_high;
	u32 txstat_frame_bytes_wo_err_low;
	u32 txstat_frame_bytes_wo_err_high;
	u32 txstat_malformed_frames_low;
	u32 txstat_malformed_frames_high;
	u32 txstat_packets_dropped_w_err_low;
	u32 txstat_packets_dropped_w_err_high;
	u32 txstat_frames_w_bad_length_type_low;
	u32 txstat_frames_w_bad_length_type_high;
	

	/* RX Statistics Counter registers begin at offset 0x900 double words */
	u32 rxstat_frames_lessthan_64B_w_crcerr_low;
	u32 rxstat_frames_lessthan_64B_w_crcerr_high;
	u32 rxstat_oversized_frames_w_crcerr_low;
	u32 rxstat_oversized_frames_w_crcerr_high;
	u32 rxstat_frames_w_crcerr_low;
	u32 rxstat_frames_w_crcerr_high;
	u32 rxstat_frames_w_crcerr_on_ok_low;
	u32 rxstat_frames_w_crcerr_on_ok_high;
	u32 rxstat_multicast_data_frames_w_crcerr_low;
	u32 rxstat_multicast_data_frames_w_crcerr_high;
	u32 rxstat_broadcast_data_frames_w_crcerr_low;
	u32 rxstat_broadcast_data_frames_w_crcerr_high;
	u32 rxstat_unicast_data_frames_w_crcerr_low;
	u32 rxstat_unicast_data_frames_w_crcerr_high;
	u32 rxstat_multicast_control_frames_w_crcerr_low;
	u32 rxstat_multicast_control_frames_w_crcerr_high;
	u32 rxstat_broadcast_control_frames_w_crcerr_low;
	u32 rxstat_broadcast_control_frames_w_crcerr_high;
	u32 rxstat_unicast_control_frames_w_crcerr_low;
	u32 rxstat_unicast_control_frames_w_crcerr_high;
	u32 rxstat_pause_frame_w_crcerr_low;
	u32 rxstat_pause_frame_w_crcerr_high;
	u32 rxstat_64B_frames_low;
	u32 rxstat_64B_frames_high;
	u32 rxstat_65B_127B_frames_low;
	u32 rxstat_65B_127B_frames_high;
	u32 rxstat_128B_255B_frames_low;
	u32 rxstat_128B_255B_frames_high;
	u32 rxstat_256B_511B_frames_low;
	u32 rxstat_256B_511B_frames_high;
	u32 rxstat_512B_1023B_frames_low;
	u32 rxstat_512B_1023B_frames_high;
	u32 rxstat_1024B_1518B_frames_low;
	u32 rxstat_1024B_1518B_frames_high;
	u32 rxstat_1519B_max_frames_low;
	u32 rxstat_1519B_max_frames_high;
	u32 rxstat_oversize_frames_low;
	u32 rxstat_oversize_frames_high;
	u32 rxstat_multicast_data_frames_wo_err_low;
	u32 rxstat_multicast_data_frames_wo_err_high;
	u32 rxstat_broadcast_data_frames_wo_err_low;
	u32 rxstat_broadcast_data_frames_wo_err_high;
	u32 rxstat_unicast_data_frames_wo_err_low;
	u32 rxstat_unicast_data_frames_wo_err_high;
	u32 rxstat_multicast_control_frames_wo_err_low;
	u32 rxstat_multicast_control_frames_wo_err_high;
	u32 rxstat_broadcast_control_frames_wo_err_low;
	u32 rxstat_broadcast_control_frames_wo_err_high;
	u32 rxstat_unicast_control_frames_wo_err_low;
	u32 rxstat_unicast_control_frames_wo_err_high;
	u32 rxstat_pause_frame_wo_err_low;
	u32 rxstat_pause_frame_wo_err_high;
	u32 rxstat_frames_lessthan_64B_and_crcerr_low;
	u32 rxstat_frames_lessthan_64B_and_crcerr_high;
	u32 rxstat_num_frame_starts_low;
	u32 rxstat_num_frame_starts_high;
	u32 rxstat_num_length_errors_low;
	u32 rxstat_num_length_errors_high;
	u32 rxstat_pfc_frames_w_crcerr_low;
	u32 rxstat_pfc_frames_w_crcerr_high;
	u32 rxstat_pfc_frames_wo_err_low;
	u32 rxstat_pfc_frames_wo_err_high;
	u32 rxstat_unused1[2];
	u32 rxstat_module_revision_id;
	u32 rxstat_scratch;
	u32 rxstat_reserved[3];
	u32 rxstat_config;
	u32 rxstat_status;
	u32 rxstat_unused3[25];
	u32 rxstat_payload_wo_err_low;
	u32 rxstat_payload_wo_err_high;
	u32 rxstat_frame_bytes_wo_err_low;
	u32 rxstat_frame_bytes_wo_err_high;
	u32 rxstat_malformed_frames_low;
	u32 rxstat_malformed_frames_high;
	u32 rxstat_packets_dropped_w_err_low;
	u32 rxstat_packets_dropped_w_err_high;
	u32 rxstat_frames_w_bad_length_type_low;
	u32 rxstat_frames_w_bad_length_type_high;
};

struct altera_s10_100ghip_xcvrreconfig {

};


#define s10_100ghip_ethreconfigoffs(a) (offsetof(struct altera_s10_100ghip_ethreconfig, a))
#define s10_100ghip_xcvrreconfigoffs(a) (offsetof(struct altera_s10_100ghip_xcvrreconfig, a))

/* Transmit and Receive Command Registers Bit Definitions
 */
#define ALTERA_S10_100GHIP_TX_CMD_STAT_OMIT_CRC		BIT(17)
#define ALTERA_S10_100GHIP_TX_CMD_STAT_TX_SHIFT16	BIT(18)
#define ALTERA_S10_100GHIP_RX_CMD_STAT_RX_SHIFT16	BIT(25)

#define S10_100GHIP_TX_PLL_NOT_LOCKED -1
#define S10_100GHIP_RX_CDR_PLL_NOT_LOCKED -2
#define S10_100GHIP_TX_DATAPATH_NOT_READY -3

/* Wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer
 */
struct s10_100ghip_buffer {
	struct list_head lh;
	struct sk_buff *skb;
	dma_addr_t dma_addr;
	u32 len;
	int mapped_as_page;
};

struct altera_s10_100ghip_private;

#define ALTERA_DTYPE_S10_SGDMA 1
#define ALTERA_DTYPE_S10_MSGDMA 2

/* standard DMA interface for MSGDMA */
struct altera_dmaops {
	int altera_dtype;
	int dmamask;
	void (*reset_dma)(struct altera_s10_100ghip_private *);
	void (*enable_txirq)(struct altera_s10_100ghip_private *);
	void (*enable_rxirq)(struct altera_s10_100ghip_private *);
	void (*disable_txirq)(struct altera_s10_100ghip_private *);
	void (*disable_rxirq)(struct altera_s10_100ghip_private *);
	void (*clear_txirq)(struct altera_s10_100ghip_private *);
	void (*clear_rxirq)(struct altera_s10_100ghip_private *);
	int (*tx_buffer)(struct altera_s10_100ghip_private *, struct s10_100ghip_buffer *);
	u32 (*tx_completions)(struct altera_s10_100ghip_private *);
	void (*add_rx_desc)(struct altera_s10_100ghip_private *, struct s10_100ghip_buffer *);
	u32 (*get_rx_status)(struct altera_s10_100ghip_private *);
	int (*init_dma)(struct altera_s10_100ghip_private *);
	void (*uninit_dma)(struct altera_s10_100ghip_private *);
	void (*start_rxdma)(struct altera_s10_100ghip_private *);
};

/* This structure is private to each device.
 */
struct altera_s10_100ghip_private {
	struct net_device *dev;
	struct device *device;
	struct napi_struct napi;

	/* Ethernet Reconfiguration Address Space */
	struct altera_s10_100ghip_ethreconfig __iomem *eth_reconfig;

	/* XCVR Reconfiguration Address Spaces */
	struct altera_s10_100ghip_xcvrreconfig __iomem *xcvr_reconfig0;
	struct altera_s10_100ghip_xcvrreconfig __iomem *xcvr_reconfig1;
	struct altera_s10_100ghip_xcvrreconfig __iomem *xcvr_reconfig2;
	struct altera_s10_100ghip_xcvrreconfig __iomem *xcvr_reconfig3;

	/* System ID */
	u32 __iomem *sysid;

	/* TSE Revision */
	u32	revision;

	/* mSGDMA Rx Dispatcher address space */
	void __iomem *rx_dma_csr;
	void __iomem *rx_dma_desc;
	void __iomem *rx_dma_resp;

	/* mSGDMA Tx Dispatcher address space */
	void __iomem *tx_dma_csr;
	void __iomem *tx_dma_desc;

	/* Rx buffers queue */
	struct s10_100ghip_buffer *rx_ring;
	u32 rx_cons;
	u32 rx_prod;
	u32 rx_ring_size;
	u32 rx_dma_buf_sz;

	/* Tx ring buffer */
	struct s10_100ghip_buffer *tx_ring;
	u32 tx_prod;
	u32 tx_cons;
	u32 tx_ring_size;

	/* Interrupts */
	u32 tx_irq;
	u32 rx_irq;

	/* RX/TX MAC FIFO configs */
	u32 tx_fifo_depth;
	u32 rx_fifo_depth;

	/* Hash filter settings */
	u32 hash_filter;
	u32 added_unicast;

	/* Descriptor memory info for managing SGDMA */
	u32 txdescmem;
	u32 rxdescmem;
	dma_addr_t rxdescmem_busaddr;
	dma_addr_t txdescmem_busaddr;
	u32 txctrlreg;
	u32 rxctrlreg;
	dma_addr_t rxdescphys;
	dma_addr_t txdescphys;

	struct list_head txlisthd;
	struct list_head rxlisthd;

	/* MAC command_config register protection */
	spinlock_t mac_cfg_lock;
	/* Tx path protection */
	spinlock_t tx_lock;
	/* Rx DMA & interrupt control protection */
	spinlock_t rxdma_irq_lock;

	/* PHY */
	int phy_addr;		/* PHY's MDIO address, -1 for autodetection */
	phy_interface_t phy_iface;
	struct mii_bus *mdio;
	int oldspeed;
	int oldduplex;
	int oldlink;

	/* ethtool msglvl option */
	u32 msg_enable;

	struct altera_dmaops *dmaops;
};

/* Function prototypes
 */
void altera_s10_100ghip_set_ethtool_ops(struct net_device *);

static inline
u32 csrrd32(void __iomem *mac, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)mac + offs);
	return readl(paddr);
}

static inline
u16 csrrd16(void __iomem *mac, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)mac + offs);
	return readw(paddr);
}

static inline
u8 csrrd8(void __iomem *mac, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)mac + offs);
	return readb(paddr);
}

static inline
void csrwr32(u32 val, void __iomem *mac, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)mac + offs);

	writel(val, paddr);
}

static inline
void csrwr16(u16 val, void __iomem *mac, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)mac + offs);

	writew(val, paddr);
}

static inline
void csrwr8(u8 val, void __iomem *mac, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)mac + offs);

	writeb(val, paddr);
}

#endif /* __ALTERA_S10_100GHIP_H__ */
