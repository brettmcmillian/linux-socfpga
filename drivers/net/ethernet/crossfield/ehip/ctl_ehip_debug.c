/*
 * Crossfield Ethernet Hard IP Driver for Intel FPGA SoCs
 * Copyright (C) 2020 Crossfield Technology LLC. All rights reserved.
 *
 * Contributors:
 *   Brett McMillian
 *
 * These functions provide debug information for the Altera 100G HIP driver.
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

static void ctl_ehip_anlt_regdump(struct ctl_ehip_private *priv)
{
	u32 reg;

	/* Read the ANLT Sequencer Status Register */
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANTL_SEQ_LINK_READY;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT Sequencer is ready.\n");
	else
		printk("Crossfield eHIP Driver: ANLT Sequencer is not ready.\n");
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANLT_SEQ_AN_TIMEOUT;
	if (reg== 0x1)
		printk("Crossfield eHIP Driver: ANLT Sequencer AN timed out.\n");
	else
		printk("Crossfield eHIP Driver: ANLT Sequencer AN did not time out.\n");
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANLT_SEQ_LT_TIMEOUT;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT Sequencer LT timed out.\n");
	else
		printk("Crossfield eHIP Driver: ANLT Sequencer LT did not time out.\n");
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANLT_SEQ_RECONFIG_MODE_AN;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT Sequencer AN mode.\n");
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANLT_SEQ_RECONFIG_MODE_LT;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT Sequencer LT mode.\n");
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANLT_SEQ_RECONFIG_MODE_50G;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT Sequencer 50G data mode.\n");
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANLT_SEQ_RECONFIG_MODE_100G;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT Sequencer 100G data mode.\n");

	/* Read the ANLT Autonegotiation Status Register */
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PAGE_RECV;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT AN page received.\n");
	else
		printk("Crossfield eHIP Driver: ANLT AN page not received.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_COMPLETE;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT AN complet.\n");
	else
		printk("Crossfield eHIP Driver: ANLT AN not complete.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_ADV_REMOTE_FAULT;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT AN fault sent to link partner.\n");
	else
		printk("Crossfield eHIP Driver: ANLT AN no fault sent to link partner.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_RXSM_IDLE;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT AN RXSM Idle State.\n");
	else
		printk("Crossfield eHIP Driver: ANLT AN RXSM normal.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_ABILITY;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT PHY able to perform AN.\n");
	else
		printk("Crossfield eHIP Driver: ANLT PHY not able to perform AN.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_STATUS;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT AN link is up.\n");
	else
		printk("Crossfield eHIP Driver: ANLT AN link is down.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_LP_FAILURE;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link partner able to AN.\n");
	else
		printk("Crossfield eHIP Driver: ANLT link partner unable to AN.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_FAILURE;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT AN failure detected.\n");
	else
		printk("Crossfield eHIP Driver: ANLT AN no failure detected.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_NEXT_PG_RECV;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT consortium next page received.\n");
	else
		printk("Crossfield eHIP Driver: ANLT consortium next page not received.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_RESOLVE_PHY_FAIL;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT AN complete, but unable to resolve PHY.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN1;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT port type unknown1.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN2;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT port type unknown2.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN3;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT port type unknown3.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN4;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT port type unknown4.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN5;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT port type unknown5.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN6;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT port type unknown6.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_KR4;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT port type 100GBASE-KR4.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_CR4;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT port type 100GBASE-CR4.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN7;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT port type unknown7.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN8;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT port type unknown8.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN9;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT port type unknown9.\n");

	/* Read the ANLT Link Training Status Register 1 */
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_TRAINED_LN0;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training successful on lane 0.\n");
	else
		printk("Crossfield eHIP Driver: ANLT link training unsuccessful on lane 0.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FRAME_LOCK_LN0;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training frame delineation detected on lane 0.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_STARTUP_LN0;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training start-up protocol in progress on lane 0.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FAILURE_LN0;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training failed on lane 0.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_TRAINED_LN1;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training successful on lane 1.\n");
	else
		printk("Crossfield eHIP Driver: ANLT link training unsuccessful on lane 1.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FRAME_LOCK_LN1;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training frame delineation detected on lane 1.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_STARTUP_LN1;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training start-up protocol in progress on lane 1.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FAILURE_LN1;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training failed on lane 1.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_TRAINED_LN2;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training successful on lane 2.\n");
	else
		printk("Crossfield eHIP Driver: ANLT link training unsuccessful on lane 2.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FRAME_LOCK_LN2;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training frame delineation detected on lane 2.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_STARTUP_LN2;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training start-up protocol in progress on lane 2.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FAILURE_LN2;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training failed on lane 2.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_TRAINED_LN3;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training successful on lane 3.\n");
	else
		printk("Crossfield eHIP Driver: ANLT link training unsuccessful on lane 3.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FRAME_LOCK_LN3;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training frame delineation detected on lane 3.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_STARTUP_LN3;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training start-up protocol in progress on lane 3.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FAILURE_LN3;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: ANLT link training failed on lane 3.\n");
}

static void ctl_ehip_phy_regdump(struct ctl_ehip_private *priv)
{
	u32 reg;

	/* Offset:0x300 */
    reg = readl(&priv->eth_reconfig->phy_revision_id);
	printk("Crossfield eHIP Driver: PHY Revision ID = %08x\n", reg);

	/* Offset:0x310 */
    reg = readl(&priv->eth_reconfig->phy_config);
	printk("Crossfield eHIP Driver: PHY Configuration = %08x\n", reg);

	/* Offset:0x313 */
    reg = readl(&priv->eth_reconfig->phy_pma_serial_loopback);
	printk("Crossfield eHIP Driver: PHY PMA Serial Loopback = %08x\n", reg);

	/* Offset:0x320 */
    reg = readl(&priv->eth_reconfig->phy_tx_pll_locked);
	printk("Crossfield eHIP Driver: PHY TX PLL Locked = %08x\n", reg);

	/* Offset:0x321 */
    reg = readl(&priv->eth_reconfig->phy_rx_cdr_pll_locked);
	printk("Crossfield eHIP Driver: PHY RX CDR PLL Locked = %08x\n", reg);

	/* Offset:0x322 */
	reg = readl(&priv->eth_reconfig->phy_tx_datapath_ready);
	reg &= PHY_TX_PCS_READY;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: TX Datapath is ready\n");
	else
		printk("Crossfield eHIP Driver: TX Datapath is not ready\n");

	/* Offset:0x323 */
    reg = readl(&priv->eth_reconfig->phy_frame_errors_detected);
	printk("Crossfield eHIP Driver: PHY Frame Errors Detected = %08x\n", reg);

	/* Offset:0x325 */
    reg = readl(&priv->eth_reconfig->phy_reset);
	printk("Crossfield eHIP Driver: PHY Reset = %08x\n", reg);

	/* Offset:0x326 */
    reg = readl(&priv->eth_reconfig->phy_rx_pcs_status_for_anlt);
	reg &= PHY_HI_BER;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: RX PCS in Hi-BER state\n");
	reg = readl(&priv->eth_reconfig->phy_rx_pcs_status_for_anlt);
	reg &= PHY_RX_ALIGNED;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: RX PCS is fully aligned\n");
	else
		printk("Crossfield eHIP Driver: RX PCS is NOT fully aligned\n");

	/* Offset:0x327 */
    reg = readl(&priv->eth_reconfig->phy_pcs_error_injection);
	printk("Crossfield eHIP Driver: PHY PCS Error Injection = %08x\n", reg);

	/* Offset:0x328 */
    reg = readl(&priv->eth_reconfig->phy_alignment_marker_lock);
	reg &= PHY_AM_LOCK;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: PHY RX PCS alignment marker locked\n");
	else
		printk("Crossfield eHIP Driver: PHY RX PCS alignment marker NOT locked\n");

	/* Offset:0x32A */
    reg = readl(&priv->eth_reconfig->phy_ber_count);
	printk("Crossfield eHIP Driver: PHY BER Count = %08x\n", reg);

	/* Offset:0x333 */
    reg = readl(&priv->eth_reconfig->phy_recovered_clock_freq);
	printk("Crossfield eHIP Driver: PHY RX recovered clock frequency = %08x KHz (/100)\n", reg);
	
	/* Offset:0x341 */
    reg = readl(&priv->eth_reconfig->phy_tx_clock_freq);
	printk("Crossfield eHIP Driver: PHY TX clock frequency = %08x KHz (/100)\n", reg);
}

static void ctl_ehip_txmac_regdump(struct ctl_ehip_private *priv)
{
	u32 reg;

	reg = readl(&priv->eth_reconfig->txmac_link_fault_config);
	printk("Crossfield eHIP Driver: TX MAC Link Fault Config = 0x%08x\n", reg);

	reg = readl(&priv->eth_reconfig->txmac_ipg_words);
	printk("Crossfield eHIP Driver: TX MAC IPG Words to remove = 0x%08x\n", reg);

	reg = readl(&priv->eth_reconfig->txmac_max_tx_frame_size);
	printk("Crossfield eHIP Driver: TX MAC Max Frame Size = 0x%08x\n", reg);

	reg = readl(&priv->eth_reconfig->txmac_config);
	printk("Crossfield eHIP Driver: TX MAC Configuration = 0x%08x\n", reg);

	reg = readl(&priv->eth_reconfig->txmac_ehip_feature_config);
	printk("Crossfield eHIP Driver: TX MAC eHIP Feature Configuration = 0x%08x\n", reg);

	reg = readl(&priv->eth_reconfig->txmac_src_address_low);
	printk("Crossfield eHIP Driver: TX MAC Address Low = 0x%08x\n", reg);

	reg = readl(&priv->eth_reconfig->txmac_src_address_high);
	printk("Crossfield eHIP Driver: TX MAC Address High = 0x%08x\n", reg);
}

static void ctl_ehip_rxmac_regdump(struct ctl_ehip_private *priv)
{
	u32 reg;

	reg = readl(&priv->eth_reconfig->rxmac_max_rx_frame_size);
	printk("Crossfield eHIP Driver: RX MAC Max Frame Size = 0x%08x\n", reg);

	reg = readl(&priv->eth_reconfig->rxmac_rx_crc_forwarding);
	printk("Crossfield eHIP Driver: RX MAC CRC Forwarding = 0x%08x\n", reg);

	reg = readl(&priv->eth_reconfig->rxmac_link_fault_status);
	printk("Crossfield eHIP Driver: RX MAC Link Fault Status = 0x%08x\n", reg);

	reg = readl(&priv->eth_reconfig->rxmac_config);
	printk("Crossfield eHIP Driver: RX MAC Configuration = 0x%08x\n", reg);

	reg = readl(&priv->eth_reconfig->rxmac_ehip_feature_config);
	printk("Crossfield eHIP Driver: RX MAC eHIP Feature Configuration = 0x%08x\n", reg);
}

void ctl_ehip_xcvr_cal_check(struct ctl_ehip_private *priv)
{
	u32 reg;

	reg = readl(&priv->xcvr_reconfig0->background_cal);
	reg &= EN_BACKGROUND_CAL;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: Transceiver 0 background calibration is enabled\n");
	else
		printk("Crossfield eHIP Driver: Transceiver 0 background calibration is disabled\n");

	reg = readl(&priv->xcvr_reconfig1->background_cal);
	reg &= EN_BACKGROUND_CAL;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: Transceiver 1 background calibration is enabled\n");
	else
		printk("Crossfield eHIP Driver: Transceiver 1 background calibration is disabled\n");

	reg = readl(&priv->xcvr_reconfig2->background_cal);
	reg &= EN_BACKGROUND_CAL;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: Transceiver 2 background calibration is enabled\n");
	else
		printk("Crossfield eHIP Driver: Transceiver 2 background calibration is disabled\n");

	reg = readl(&priv->xcvr_reconfig3->background_cal);
	reg &= EN_BACKGROUND_CAL;
	if (reg == 0x1)
		printk("Crossfield eHIP Driver: Transceiver 3 background calibration is enabled\n");
	else
		printk("Crossfield eHIP Driver: Transceiver 3 background calibration is disabled\n");
}

static void ctl_ehip_xcvr_regdump(struct ctl_ehip_private *priv)
{
	u32 reg;

	/*reg = readl(&priv->xcvr_reconfig0->)*/
}

void ctl_ehip_regdump(struct ctl_ehip_private *priv)
{
	u32 reg;

	reg = readl(priv->sysid);
	printk("Crossfield eHIP Driver: sysid = 0x%08x.\n", reg);

    /* Enable the first line only when ANLT is turned on in the HIP */
    /* ANLT only supports backplane Ethernet, so we disable it. */
    ctl_ehip_anlt_regdump(priv);
    ctl_ehip_phy_regdump(priv);
    ctl_ehip_txmac_regdump(priv);
    ctl_ehip_rxmac_regdump(priv);
}