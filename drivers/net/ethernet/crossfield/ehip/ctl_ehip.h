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

#ifndef __CTL_EHIP_H__
#define __CTL_EHIP_H__

#define CTL_EHIP_RESOURCE_NAME	"ctl_ehip"

#include <linux/bitops.h>
#include <linux/if_vlan.h>
#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/phy.h>

#define ANLT_CSR_OFFSET = 0x2C0
#define PHY_CSR_OFFSET = 0xC00 /* (0x300 word offset) */
#define TX_MAC_CSR_OFFSET = 0x1000 /* (0x400 word offset) */
#define RX_MAC_CSR_OFFSET = 0x1400 /* (0x500 word offset) */

#define CTL_EHIP_SW_RESET_WATCHDOG_CNTR	10000
#define CTL_EHIP_MAC_FIFO_WIDTH		4	/* TX/RX FIFO width in
							 * bytes
							 */
/* Rx FIFO default settings */
#define CTL_EHIP_RX_SECTION_EMPTY	16
#define CTL_EHIP_RX_SECTION_FULL	0
#define CTL_EHIP_RX_ALMOST_EMPTY	8
#define CTL_EHIP_RX_ALMOST_FULL	8

/* Tx FIFO default settings */
#define CTL_EHIP_TX_SECTION_EMPTY	16
#define CTL_EHIP_TX_SECTION_FULL	0
#define CTL_EHIP_TX_ALMOST_EMPTY	8
#define CTL_EHIP_TX_ALMOST_FULL	3

/* MAC function configuration default settings */
#define CTL_EHIP_TX_IPG_LENGTH	12

#define CTL_EHIP_PAUSE_QUANTA		0xffff

#define SUPPORTED_100000baseSR4_Full		1ul << 37;

#define GET_BIT_VALUE(v, bit)		(((v) >> (bit)) & 0x1)

#define ANTL_SEQ_LINK_READY			BIT(0)
#define ANLT_SEQ_AN_TIMEOUT			BIT(1)
#define ANLT_SEQ_LT_TIMEOUT			BIT(2)
#define ANLT_SEQ_RECONFIG_MODE_AN	BIT(8)
#define ANLT_SEQ_RECONFIG_MODE_LT	BIT(9)
#define ANLT_SEQ_RECONFIG_MODE_50G	BIT(12)
#define ANLT_SEQ_RECONFIG_MODE_100G	BIT(13)

#define ANLT_ANSTAT_PAGE_RECV			BIT(1)
#define ANLT_ANSTAT_COMPLETE			BIT(2)
#define ANLT_ANSTAT_ADV_REMOTE_FAULT	BIT(3)
#define ANLT_ANSTAT_RXSM_IDLE			BIT(4)
#define ANLT_ANSTAT_ABILITY				BIT(5)
#define ANLT_ANSTAT_STATUS				BIT(6)
#define ANLT_ANSTAT_LP_FAILURE			BIT(7)
#define ANLT_ANSTAT_FAILURE				BIT(9)
#define ANLT_ANSTAT_NEXT_PG_RECV		BIT(10)
#define ANLT_ANSTAT_RESOLVE_PHY_FAIL	BIT(11)
#define ANLT_ANSTAT_PORT_TYPE_UNKNOWN1	BIT(12)
#define ANLT_ANSTAT_PORT_TYPE_UNKNOWN2	BIT(13)
#define ANLT_ANSTAT_PORT_TYPE_UNKNOWN3	BIT(14)
#define ANLT_ANSTAT_PORT_TYPE_UNKNOWN4	BIT(15)
#define ANLT_ANSTAT_PORT_TYPE_UNKNOWN5	BIT(16)
#define ANLT_ANSTAT_PORT_TYPE_UNKNOWN6	BIT(17)
#define ANLT_ANSTAT_PORT_TYPE_KR4		BIT(18)
#define ANLT_ANSTAT_PORT_TYPE_CR4		BIT(19)
#define ANLT_ANSTAT_PORT_TYPE_UNKNOWN7	BIT(20)
#define ANLT_ANSTAT_PORT_TYPE_UNKNOWN8	BIT(21)
#define ANLT_ANSTAT_PORT_TYPE_UNKNOWN9	BIT(22)

#define ANLT_LTSTAT1_TRAINED_LN0		BIT(0)
#define ANLT_LTSTAT1_FRAME_LOCK_LN0		BIT(1)
#define ANLT_LTSTAT1_STARTUP_LN0		BIT(2)
#define ANLT_LTSTAT1_FAILURE_LN0		BIT(3)
#define ANLT_LTSTAT1_TRAINED_LN1		BIT(8)
#define ANLT_LTSTAT1_FRAME_LOCK_LN1		BIT(9)
#define ANLT_LTSTAT1_STARTUP_LN1		BIT(10)
#define ANLT_LTSTAT1_FAILURE_LN1		BIT(11)
#define ANLT_LTSTAT1_TRAINED_LN2		BIT(16)
#define ANLT_LTSTAT1_FRAME_LOCK_LN2		BIT(17)
#define ANLT_LTSTAT1_STARTUP_LN2		BIT(18)
#define ANLT_LTSTAT1_FAILURE_LN2		BIT(19)
#define ANLT_LTSTAT1_TRAINED_LN3		BIT(24)
#define ANLT_LTSTAT1_FRAME_LOCK_LN3		BIT(25)
#define ANLT_LTSTAT1_STARTUP_LN3		BIT(26)
#define ANLT_LTSTAT1_FAILURE_LN3		BIT(27)

/* Reset registers */
#define PHY_EIO_SYS_RST			0x00000001
#define PHY_SOFT_TX_RST			0x00000002
#define PHY_SOFT_RX_RST			0x00000004

/* TX DATAPATH READY */
#define PHY_TX_PCS_READY		BIT(0)

/* RX PCS Status for AN/LT */
#define PHY_RX_ALIGNED			BIT(0)
#define PHY_HI_BER				BIT(1)

/* PCS Alignment Marker Lock */
#define PHY_AM_LOCK				BIT(0)

/* Transmit and Receive Command Registers Bit Definitions
 */
#define CTL_EHIP_TX_CMD_STAT_OMIT_CRC		BIT(17)
#define CTL_EHIP_TX_CMD_STAT_TX_SHIFT16	BIT(18)
#define CTL_EHIP_RX_CMD_STAT_RX_SHIFT16	BIT(25)

#define CTL_EHIP_TX_PCS_READY				GET_BIT_VALUE(v, 0)

#define S10_100GHIP_TX_PLL_NOT_LOCKED -1
#define S10_100GHIP_RX_CDR_PLL_NOT_LOCKED -2
#define S10_100GHIP_TX_DATAPATH_NOT_READY -3

#define TX_MAC_EN_SADDR_INSERT	BIT(3)
#define TX_MAC_DISABLE_TX_MAC	BIT(2)
#define TX_MAC_DISABLE_TXVLAN	BIT(1)

struct ctl_ehip_ethreconfig {

	u32 padding[176];

	/* Auto-negotiation and link training registers are currently disabled */
	u32 anlt_sequencer_config;
	u32 anlt_sequencer_status;
	u32 anlt_unused1[14];
	u32 anlt_an_config1;
	u32 anlt_an_config2;
	u32 anlt_an_status;
	u32 anlt_an_config3;
	u32 anlt_an_config4;
	u32 anlt_an_config5;
	u32 anlt_an_config6;
	u32 anlt_an_status1;
	u32 anlt_an_status2;
	u32 anlt_an_status3;
	u32 anlt_an_status4;
	u32 anlt_an_status5;
	u32 anlt_an_unused2;
	u32 anlt_an_next_page_override;
	u32 anlt_an_next_page_lp_status;
	u32 anlt_unused3;
	u32 anlt_lt_config1;
	u32 anlt_lt_config2;
	u32 anlt_lt_status1;
	u32 anlt_lt_config_lane0;
	u32 anlt_lt_frame_contents_lane0;
	u32 anlt_xcvr_tx_eq1_lane0;
	u32 anlt_xcvr_tx_eq2_lane0;
	u32 anlt_lt_param;
	u32 anlt_unused[8];
	u32 anlt_lt_config_lane1;
	u32 anlt_lt_frame_contents_lane1;
	u32 anlt_xcvr_tx_eq1_lane1;
	u32 anlt_xcvr_tx_eq2_lane1;
	u32 anlt_lt_config_lane2;
	u32 anlt_lt_frame_contents_lane2;
	u32 anlt_xcvr_tx_eq1_lane2;
	u32 anlt_xcvr_tx_eq2_lane2;
	u32 anlt_lt_config_lane3;
	u32 anlt_lt_frame_contents_lane3;
	u32 anlt_xcvr_tx_eq1_lane3;
	u32 anlt_xcvr_tx_eq2_lane3;

	u32 anlt_padding[532];

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
	
	u32 txstat_padding[150];

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

#define EN_BACKGROUND_CAL		BIT(0)

struct ctl_ehip_xcvrreconfig {
	u32 padding[104];

	/* Transmitter PMA Logical Register Map */
	u32 tx_pma_preemphasis1;
	u32 tx_pma_unused1;
	u32 tx_pma_preemphasis2;
	u32 tx_pma_unused2;
	u32 tx_pma_vod_compensation;
	u32 tx_pma_unused3[2];
	u32 tx_pma_slew_rate;
	u32 tx_pma_padding[13];

	/* Receiver PMA Logical Register Map */
	u32 rx_pma_map[99];

	u32 cal_padding[963];
	u32 background_cal;
};

#define RX_PMA_REG_OFFSET 125

#define ctl_ehip_ethreconfigoffs(a) (offsetof(struct ctl_ehip_ethreconfig, a))
#define ctl_ehip_xcvrreconfigoffs(a) (offsetof(struct ctl_ehip_xcvrreconfig, a))

/* Wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer
 */
struct ctl_ehip_buffer {
	struct list_head lh;
	struct sk_buff *skb;
	dma_addr_t dma_addr;
	u32 len;
	int mapped_as_page;
};

struct ctl_ehip_private;

#define CROSSFIELD_DTYPE_EHIP_DMA 1

/* Standard DMA interface for eHIP DMA */
struct crossfield_dmaops {
	int crossfield_dtype;
	int dmamask;
	void (*reset_dma)(struct ctl_ehip_private *);
	void (*enable_txirq)(struct ctl_ehip_private *);
	void (*enable_rxirq)(struct ctl_ehip_private *);
	void (*disable_txirq)(struct ctl_ehip_private *);
	void (*disable_rxirq)(struct ctl_ehip_private *);
	void (*clear_txirq)(struct ctl_ehip_private *);
	void (*clear_rxirq)(struct ctl_ehip_private *);
	int (*tx_buffer)(struct ctl_ehip_private *, struct ctl_ehip_buffer *);
	u32 (*tx_completions)(struct ctl_ehip_private *);
	void (*add_rx_desc)(struct ctl_ehip_private *, struct ctl_ehip_buffer *);
	u32 (*get_rx_status)(struct ctl_ehip_private *);
	int (*init_dma)(struct ctl_ehip_private *);
	void (*uninit_dma)(struct ctl_ehip_private *);
	void (*start_rxdma)(struct ctl_ehip_private *);
};

/* This structure is private to each device.
 */
struct ctl_ehip_private {
	struct net_device *dev;
	struct device *device;
	struct napi_struct napi;

	/* Ethernet Reconfiguration Address Space */
	struct ctl_ehip_ethreconfig __iomem *eth_reconfig;

	/* XCVR Reconfiguration Address Spaces */
	struct ctl_ehip_xcvrreconfig __iomem *xcvr_reconfig0;
	struct ctl_ehip_xcvrreconfig __iomem *xcvr_reconfig1;
	struct ctl_ehip_xcvrreconfig __iomem *xcvr_reconfig2;
	struct ctl_ehip_xcvrreconfig __iomem *xcvr_reconfig3;

	/* System ID */
	u32 __iomem *sysid;

	/* TSE Revision */
	u32	revision;

	/* eHIP DMA Rx Dispatcher address space */
	struct ehip_dma_csr __iomem *rx_dma_csr;
	struct ehip_dma_desc __iomem *rx_dma_desc;
	struct ehip_dma_response __iomem *rx_dma_resp;

	/* eHIP DMA Tx Dispatcher address space */
	struct ehip_dma_csr __iomem *tx_dma_csr;
	struct ehip_dma_desc __iomem *tx_dma_desc;

	/* Rx buffers queue */
	struct ctl_ehip_buffer *rx_ring;
	u32 rx_cons;
	u32 rx_prod;
	u32 rx_ring_size;
	u32 rx_dma_buf_sz;

	/* Tx ring buffer */
	struct ctl_ehip_buffer *tx_ring;
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
	struct phylink *phy_link;
	phy_interface_t phy_iface;
	const char * phy_name;
	int oldspeed;
	int oldduplex;
	int oldlink;

	/* ethtool msglvl option */
	u32 msg_enable;

	struct crossfield_dmaops *dmaops;
};

/* Function prototypes
 */
void ctl_ehip_set_ethtool_ops(struct net_device *);

void ctl_ehip_regdump(struct ctl_ehip_private *priv);

void ctl_ehip_xcvr_cal_check(struct ctl_ehip_private *priv);

#endif /* __CTL_EHIP_H__ */
