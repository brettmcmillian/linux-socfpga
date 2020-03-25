/*
 * Ethtool support for theIntel Stratix 10 100G Ethernet Hard IP Driver
 * Copyright (C) 2020 Crossfield Technology LLC. All rights reserved.
 *
 * Contributors:
 *   Brett McMillian
 *
 * This driver is based on the Altera TSE driver and must be used
 * with the altera_s10_100ghip driver.
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

#include <linux/ethtool.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/phylink.h>

#include "altera_s10_100ghip.h"

#define S10_100GHIP_STATS_LEN	72
#define S10_100GHIP_NUM_REGS	124

static char const stat_gstrings[][ETH_GSTRING_LEN] = {
	"tx_frames_lessthan_64B_w_crcerr",
	"tx_oversized_frames_w_crcerr",
	"tx_frames_w_crcerr",
	"tx_frames_w_crcerr_on_ok",
	"tx_multicast_data_frames_w_crcerr",
	"tx_broadcast_data_frames_w_crcerr",
	"tx_unicast_data_frames_w_crcerr",
	"tx_multicast_control_frames_w_crcerr",
	"tx_broadcast_control_frames_w_crcerr",
	"tx_unicast_control_frames_w_crcerr",
	"tx_pause_frame_w_crcerr",
	"tx_64B_frames",
	"tx_65B_127B_frames",
	"tx_128B_255B_frames",
	"tx_256B_511B_frames",
	"tx_512B_1023B_frames",
	"tx_1024B_1518B_frames",
	"tx_1519B_max_frames",
	"tx_oversize_frames",
	"tx_multicast_data_frames_wo_err",
	"tx_broadcast_data_frames_wo_err",
	"tx_unicast_data_frames_wo_err",
	"tx_multicast_control_frames_wo_err",
	"tx_broadcast_control_frames_wo_err",
	"tx_unicast_control_frames_wo_err",
	"tx_pause_frame_wo_err",
    "tx_frames_lessthan_64B_and_crcerr",
	"tx_num_frame_starts",
	"tx_num_length_errors",
	"tx_pfc_frames_w_crcerr",
	"tx_pfc_frames_wo_err",
	"tx_payload_wo_err",
	"tx_frame_bytes_wo_err",
	"tx_malformed_frames",
	"tx_packets_dropped_w_err",
	"tx_frames_w_bad_length_type",
	"rx_frames_lessthan_64B_w_crcerr",
	"rx_oversized_frames_w_crcerr",
	"rx_frames_w_crcerr",
	"rx_frames_w_crcerr_on_ok",
	"rx_multicast_data_frames_w_crcerr",
	"rx_broadcast_data_frames_w_crcerr",
	"rx_unicast_data_frames_w_crcerr",
	"rx_multicast_control_frames_w_crcerr",
	"rx_broadcast_control_frames_w_crcerr",
	"rx_unicast_control_frames_w_crcerr",
	"rx_pause_frame_w_crcerr",
	"rx_64B_frames",
	"rx_65B_127B_frames",
	"rx_128B_255B_frames",
	"rx_256B_511B_frames",
	"rx_512B_1023B_frames",
	"rx_1024B_1518B_frames",
	"rx_1519B_max_frames",
	"rx_oversize_frames",
	"rx_multicast_data_frames_wo_err",
	"rx_broadcast_data_frames_wo_err",
	"rx_unicast_data_frames_wo_err",
	"rx_multicast_control_frames_wo_err",
	"rx_broadcast_control_frames_wo_err",
	"rx_unicast_control_frames_wo_err",
	"rx_pause_frame_wo_err",
	"rx_frames_lessthan_64B_and_crcerr",
	"rx_num_frame_starts",
	"rx_num_length_errors",
	"rx_pfc_frames_w_crcerr",
	"rx_pfc_frames_wo_err",
	"rx_payload_wo_err",
	"rx_frame_bytes_wo_err",
	"rx_malformed_frames",
	"rx_packets_dropped_w_err",
	"rx_frames_w_bad_length_type",
};

static void s10_100ghip_get_drvinfo(struct net_device *dev,
			    struct ethtool_drvinfo *info)
{
	struct altera_s10_100ghip_private *priv = netdev_priv(dev);
	u32 rev = readl(&priv->eth_reconfig->phy_revision_id);

	strcpy(info->driver, "altera_s10_100ghip");
	strcpy(info->version, "v1.0");
	snprintf(info->fw_version, ETHTOOL_FWVERS_LEN, "v%d.%d",
		 rev & 0xFFFF, (rev & 0xFFFF0000) >> 16);
	sprintf(info->bus_info, "platform");
}

/* Fill in a buffer with the strings which correspond to the
 * stats
 */
static void s10_100ghip_gstrings(struct net_device *dev, u32 stringset, u8 *buf)
{
	memcpy(buf, stat_gstrings, S10_100GHIP_STATS_LEN * ETH_GSTRING_LEN);
}

static void s10_100ghip_fill_stats(struct net_device *dev, struct ethtool_stats *dummy,
			   u64 *buf)
{
	struct altera_s10_100ghip_private *priv = netdev_priv(dev);
	u64 ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_frames_lessthan_64B_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_frames_lessthan_64B_w_crcerr_low));
	buf[0] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_oversized_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_oversized_frames_w_crcerr_low));
	buf[1] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_frames_w_crcerr_low));
	buf[2] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_frames_w_crcerr_on_ok_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_frames_w_crcerr_on_ok_low));
	buf[3] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_multicast_data_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_multicast_data_frames_w_crcerr_low));
	buf[4] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_broadcast_data_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_broadcast_data_frames_w_crcerr_low));
	buf[5] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_unicast_data_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_unicast_data_frames_w_crcerr_low));
	buf[6] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_multicast_control_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_multicast_control_frames_w_crcerr_low));
	buf[7] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_broadcast_control_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_broadcast_control_frames_w_crcerr_low));
	buf[8] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_unicast_control_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_unicast_control_frames_w_crcerr_low));
	buf[9] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_pause_frame_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_pause_frame_w_crcerr_low));
	buf[10] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_64B_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_64B_frames_low));
	buf[11] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_65B_127B_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_65B_127B_frames_low));
	buf[12] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_128B_255B_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_128B_255B_frames_low));
	buf[13] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_256B_511B_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_256B_511B_frames_low));
	buf[14] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_512B_1023B_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_512B_1023B_frames_low));
	buf[15] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_1024B_1518B_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_1024B_1518B_frames_low));
	buf[16] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_1519B_max_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_1519B_max_frames_low));
	buf[17] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_oversize_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_oversize_frames_low));
	buf[18] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_multicast_data_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_multicast_data_frames_wo_err_low));
	buf[19] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_broadcast_data_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_broadcast_data_frames_wo_err_low));
	buf[20] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_unicast_data_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_unicast_data_frames_wo_err_low));
	buf[21] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_multicast_control_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_multicast_control_frames_wo_err_low));
	buf[22] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_broadcast_control_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_broadcast_control_frames_wo_err_low));
	buf[23] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_unicast_control_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_unicast_control_frames_wo_err_low));
	buf[24] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_pause_frame_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_pause_frame_wo_err_low));
	buf[25] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_frames_lessthan_64B_and_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_frames_lessthan_64B_and_crcerr_low));
	buf[26] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_num_frame_starts_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_num_frame_starts_low));
	buf[27] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_num_length_errors_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_num_length_errors_low));
	buf[28] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_pfc_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_pfc_frames_w_crcerr_low));
	buf[29] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_pfc_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_pfc_frames_wo_err_low));
	buf[30] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_payload_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_payload_wo_err_low));
	buf[31] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_frame_bytes_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_frame_bytes_wo_err_low));
	buf[32] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_malformed_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_malformed_frames_low));
	buf[33] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_packets_dropped_w_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_packets_dropped_w_err_low));
	buf[34] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_frames_w_bad_length_type_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(txstat_frames_w_bad_length_type_low));
	buf[35] = ext;

		ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_frames_lessthan_64B_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_frames_lessthan_64B_w_crcerr_low));
	buf[36] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_oversized_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_oversized_frames_w_crcerr_low));
	buf[37] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_frames_w_crcerr_low));
	buf[38] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_frames_w_crcerr_on_ok_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_frames_w_crcerr_on_ok_low));
	buf[39] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_multicast_data_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_multicast_data_frames_w_crcerr_low));
	buf[40] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_broadcast_data_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_broadcast_data_frames_w_crcerr_low));
	buf[41] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_unicast_data_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_unicast_data_frames_w_crcerr_low));
	buf[42] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_multicast_control_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_multicast_control_frames_w_crcerr_low));
	buf[43] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_broadcast_control_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_broadcast_control_frames_w_crcerr_low));
	buf[44] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_unicast_control_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_unicast_control_frames_w_crcerr_low));
	buf[45] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_pause_frame_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_pause_frame_w_crcerr_low));
	buf[46] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_64B_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_64B_frames_low));
	buf[47] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_65B_127B_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_65B_127B_frames_low));
	buf[48] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_128B_255B_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_128B_255B_frames_low));
	buf[49] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_256B_511B_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_256B_511B_frames_low));
	buf[50] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_512B_1023B_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_512B_1023B_frames_low));
	buf[51] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_1024B_1518B_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_1024B_1518B_frames_low));
	buf[52] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_1519B_max_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_1519B_max_frames_low));
	buf[53] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_oversize_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_oversize_frames_low));
	buf[54] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_multicast_data_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_multicast_data_frames_wo_err_low));
	buf[55] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_broadcast_data_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_broadcast_data_frames_wo_err_low));
	buf[56] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_unicast_data_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_unicast_data_frames_wo_err_low));
	buf[57] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_multicast_control_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_multicast_control_frames_wo_err_low));
	buf[58] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_broadcast_control_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_broadcast_control_frames_wo_err_low));
	buf[59] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_unicast_control_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_unicast_control_frames_wo_err_low));
	buf[60] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_pause_frame_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_pause_frame_wo_err_low));
	buf[61] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_frames_lessthan_64B_and_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_frames_lessthan_64B_and_crcerr_low));
	buf[62] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_num_frame_starts_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_num_frame_starts_low));
	buf[63] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_num_length_errors_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_num_length_errors_low));
	buf[64] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_pfc_frames_w_crcerr_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_pfc_frames_w_crcerr_low));
	buf[65] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_pfc_frames_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_pfc_frames_wo_err_low));
	buf[66] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_payload_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_payload_wo_err_low));
	buf[67] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_frame_bytes_wo_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_frame_bytes_wo_err_low));
	buf[68] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_malformed_frames_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_malformed_frames_low));
	buf[69] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_packets_dropped_w_err_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_packets_dropped_w_err_low));
	buf[70] = ext;

	ext = (u64) csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_frames_w_bad_length_type_high)) << 32;
	ext |= csrrd32(priv->eth_reconfig,
			 s10_100ghip_ethreconfigoffs(rxstat_frames_w_bad_length_type_low));
	buf[71] = ext;
	
}

static int s10_100ghip_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return S10_100GHIP_STATS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

static u32 s10_100ghip_get_msglevel(struct net_device *dev)
{
	struct altera_s10_100ghip_private *priv = netdev_priv(dev);
	return priv->msg_enable;
}

static void s10_100ghip_set_msglevel(struct net_device *dev, uint32_t data)
{
	struct altera_s10_100ghip_private *priv = netdev_priv(dev);
	priv->msg_enable = data;
}

static int s10_100ghip_reglen(struct net_device *dev)
{
	return S10_100GHIP_NUM_REGS * sizeof(u32);
}

static void s10_100ghip_get_regs(struct net_device *dev, struct ethtool_regs *regs,
			 void *regbuf)
{
	int i;
	struct altera_s10_100ghip_private *priv = netdev_priv(dev);
	u32 *buf = regbuf;

	/* Set version to a known value, so ethtool knows
	 * how to do any special formatting of this data.
	 * This version number will need to change if and
	 * when this register table is changed.
	 *
	 * version[31:0] = 1: Dump the first 62 TX and RX Registers
	 *
	 */

	regs->version = 1;

	for (i = 0; i < S10_100GHIP_NUM_REGS/2; i++)
		buf[i] = csrrd32(priv->eth_reconfig, 0x800 + i * 4);

	for (i = S10_100GHIP_NUM_REGS/2; i < S10_100GHIP_NUM_REGS; i++)
		buf[i] = csrrd32(priv->eth_reconfig, 0x900 + i * 4);
}

static u32 s10_100ghip_get_link(struct net_device *dev) {
/*	struct altera_s10_100ghip_private *priv = netdev_priv(dev); */
	return 1;

}

static int s10_100ghip_get_link_ksettings(struct net_device *dev,
										  struct ethtool_link_ksettings *link_ksettings)
{
	ethtool_link_ksettings_zero_link_mode(link_ksettings, supported);
	ethtool_link_ksettings_add_link_mode(link_ksettings, supported, FIBRE);
	ethtool_link_ksettings_add_link_mode(link_ksettings, supported, 100000baseSR4_Full);
	ethtool_link_ksettings_add_link_mode(link_ksettings, advertising, FIBRE);
	ethtool_link_ksettings_add_link_mode(link_ksettings, advertising, 100000baseSR4_Full);

	link_ksettings->base.port 			= PORT_FIBRE;
	link_ksettings->base.transceiver	= XCVR_INTERNAL;
	link_ksettings->base.speed			= SPEED_100000;
	link_ksettings->base.duplex			= DUPLEX_FULL;
	link_ksettings->base.autoneg		= AUTONEG_DISABLE;

	return 0;

/*	struct altera_s10_100ghip_private *priv = netdev_priv(dev);

	return phylink_ethtool_ksettings_get(priv->phy_link, link_ksettings);*/
}

static int s10_100ghip_set_link_ksettings(struct net_device *dev,
										  const struct ethtool_link_ksettings *link_ksettings)
{
/*	link_ksettings->base.speed = SPEED_100000;
	link_ksettings->base.duplex = DUPLEX_FULL;
	link_ksettings->base.autoneg = AUTONEG_DISABLE;*/
	
	return 0;

/*
	struct altera_s10_100ghip_private *priv = netdev_priv(dev);

	return phylink_ethtool_ksettings_set(priv->phy_link, link_ksettings);
*/
	
}

static const struct ethtool_ops s10_100ghip_ethtool_ops = {
	.get_drvinfo 		= s10_100ghip_get_drvinfo,
	.get_regs_len 		= s10_100ghip_reglen,
	.get_regs 			= s10_100ghip_get_regs,
	.get_link 			= s10_100ghip_get_link,
	.get_strings 		= s10_100ghip_gstrings,
	.get_sset_count 	= s10_100ghip_sset_count,
	.get_ethtool_stats 	= s10_100ghip_fill_stats,
	.get_msglevel 		= s10_100ghip_get_msglevel,
	.set_msglevel 		= s10_100ghip_set_msglevel,
	.get_link_ksettings = s10_100ghip_get_link_ksettings,
	.set_link_ksettings = s10_100ghip_set_link_ksettings,
};

void altera_s10_100ghip_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &s10_100ghip_ethtool_ops;
}
