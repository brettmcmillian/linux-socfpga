/*
 * Intel Stratix 10 100G Ethernet Hard IP Driver
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

#include "altera_s10_100ghip.h"

static void altera_s10_100ghip_anlt_regdump(struct altera_s10_100ghip_private *priv)
{
	u32 reg;

	/* Read the ANLT Sequencer Status Register */
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANTL_SEQ_LINK_READY;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT Sequencer is ready.\n");
	else
		printk("altera_s10_100ghip: ANLT Sequencer is not ready.\n");
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANLT_SEQ_AN_TIMEOUT;
	if (reg== 0x1)
		printk("altera_s10_100ghip: ANLT Sequencer AN timed out.\n");
	else
		printk("altera_s10_100ghip: ANLT Sequencer AN did not time out.\n");
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANLT_SEQ_LT_TIMEOUT;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT Sequencer LT timed out.\n");
	else
		printk("altera_s10_100ghip: ANLT Sequencer LT did not time out.\n");
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANLT_SEQ_RECONFIG_MODE_AN;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT Sequencer AN mode.\n");
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANLT_SEQ_RECONFIG_MODE_LT;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT Sequencer LT mode.\n");
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANLT_SEQ_RECONFIG_MODE_50G;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT Sequencer 50G data mode.\n");
	reg = readl(&priv->eth_reconfig->anlt_sequencer_status);
	reg &= ANLT_SEQ_RECONFIG_MODE_100G;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT Sequencer 100G data mode.\n");

	/* Read the ANLT Autonegotiation Status Register */
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PAGE_RECV;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT AN page received.\n");
	else
		printk("altera_s10_100ghip: ANLT AN page not received.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_COMPLETE;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT AN complet.\n");
	else
		printk("altera_s10_100ghip: ANLT AN not complete.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_ADV_REMOTE_FAULT;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT AN fault sent to link partner.\n");
	else
		printk("altera_s10_100ghip: ANLT AN no fault sent to link partner.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_RXSM_IDLE;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT AN RXSM Idle State.\n");
	else
		printk("altera_s10_100ghip: ANLT AN RXSM normal.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_ABILITY;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT PHY able to perform AN.\n");
	else
		printk("altera_s10_100ghip: ANLT PHY not able to perform AN.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_STATUS;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT AN link is up.\n");
	else
		printk("altera_s10_100ghip: ANLT AN link is down.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_LP_FAILURE;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link partner able to AN.\n");
	else
		printk("altera_s10_100ghip: ANLT link partner unable to AN.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_FAILURE;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT AN failure detected.\n");
	else
		printk("altera_s10_100ghip: ANLT AN no failure detected.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_NEXT_PG_RECV;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT consortium next page received.\n");
	else
		printk("altera_s10_100ghip: ANLT consortium next page not received.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_RESOLVE_PHY_FAIL;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT AN complete, but unable to resolve PHY.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN1;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT port type unknown1.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN2;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT port type unknown2.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN3;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT port type unknown3.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN4;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT port type unknown4.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN5;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT port type unknown5.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN6;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT port type unknown6.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_KR4;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT port type 100GBASE-KR4.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_CR4;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT port type 100GBASE-CR4.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN7;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT port type unknown7.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN8;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT port type unknown8.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_ANSTAT_PORT_TYPE_UNKNOWN9;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT port type unknown9.\n");

	/* Read the ANLT Link Training Status Register 1 */
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_TRAINED_LN0;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training successful on lane 0.\n");
	else
		printk("altera_s10_100ghip: ANLT link training unsuccessful on lane 0.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FRAME_LOCK_LN0;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training frame delineation detected on lane 0.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_STARTUP_LN0;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training start-up protocol in progress on lane 0.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FAILURE_LN0;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training failed on lane 0.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_TRAINED_LN1;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training successful on lane 1.\n");
	else
		printk("altera_s10_100ghip: ANLT link training unsuccessful on lane 1.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FRAME_LOCK_LN1;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training frame delineation detected on lane 1.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_STARTUP_LN1;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training start-up protocol in progress on lane 1.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FAILURE_LN1;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training failed on lane 1.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_TRAINED_LN2;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training successful on lane 2.\n");
	else
		printk("altera_s10_100ghip: ANLT link training unsuccessful on lane 2.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FRAME_LOCK_LN2;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training frame delineation detected on lane 2.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_STARTUP_LN2;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training start-up protocol in progress on lane 2.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FAILURE_LN2;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training failed on lane 2.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_TRAINED_LN3;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training successful on lane 3.\n");
	else
		printk("altera_s10_100ghip: ANLT link training unsuccessful on lane 3.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FRAME_LOCK_LN3;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training frame delineation detected on lane 3.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_STARTUP_LN3;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training start-up protocol in progress on lane 3.\n");
	reg = readl(&priv->eth_reconfig->anlt_an_status);
	reg &= ANLT_LTSTAT1_FAILURE_LN3;
	if (reg == 0x1)
		printk("altera_s10_100ghip: ANLT link training failed on lane 3.\n");
}

static void altera_s10_100ghip_phy_regdump(struct altera_s10_100ghip_private *priv)
{
	u32 reg;

	/* Offset:0x300 */
    reg = readl(&priv->eth_reconfig->phy_revision_id);
	printk("altera_s10_100ghip: PHY Revision ID = %08x\n", reg);

	/* Offset:0x310 */
    reg = readl(&priv->eth_reconfig->phy_config);
	printk("altera_s10_100ghip: PHY Configuration = %08x\n", reg);

	/* Offset:0x313 */
    reg = readl(&priv->eth_reconfig->phy_pma_serial_loopback);
	printk("altera_s10_100ghip: PHY PMA Serial Loopback = %08x\n", reg);

	/* Offset:0x320 */
    reg = readl(&priv->eth_reconfig->phy_tx_pll_locked);
	printk("altera_s10_100ghip: PHY TX PLL Locked = %08x\n", reg);

	/* Offset:0x321 */
    reg = readl(&priv->eth_reconfig->phy_rx_cdr_pll_locked);
	printk("altera_s10_100ghip: PHY RX CDR PLL Locked = %08x\n", reg);

	/* Offset:0x322 */
	reg = readl(&priv->eth_reconfig->phy_tx_datapath_ready);
	reg &= PHY_TX_PCS_READY;
	if (reg == 0x1)
		printk("altera_s10_100ghip: TX Datapath is ready\n");
	else
		printk("altera_s10_100ghip: TX Datapath is not ready\n");

	/* Offset:0x323 */
    reg = readl(&priv->eth_reconfig->phy_frame_errors_detected);
	printk("altera_s10_100ghip: PHY Frame Errors Detected = %08x\n", reg);

	/* Offset:0x325 */
    reg = readl(&priv->eth_reconfig->phy_reset);
	printk("altera_s10_100ghip: PHY Reset = %08x\n", reg);

	/* Offset:0x326 */
    reg = readl(&priv->eth_reconfig->phy_rx_pcs_status_for_anlt);
	reg &= PHY_HI_BER;
	if (reg == 0x1)
		printk("altera_s10_100ghip: RX PCS in Hi-BER state\n");
	reg = readl(&priv->eth_reconfig->phy_rx_pcs_status_for_anlt);
	reg &= PHY_RX_ALIGNED;
	if (reg == 0x1)
		printk("altera_s10_100ghip: RX PCS is fully aligned\n");
	else
		printk("altera_s10_100ghip: RX PCS is NOT fully aligned\n");

	/* Offset:0x327 */
    reg = readl(&priv->eth_reconfig->phy_pcs_error_injection);
	printk("altera_s10_100ghip: PHY PCS Error Injection = %08x\n", reg);

	/* Offset:0x328 */
    reg = readl(&priv->eth_reconfig->phy_alignment_marker_lock);
	reg &= PHY_AM_LOCK;
	if (reg == 0x1)
		printk("altera_s10_100ghip: PHY RX PCS alignment marker locked\n");
	else
		printk("altera_s10_100ghip: PHY RX PCS alignment marker NOT locked\n");

	/* Offset:0x32A */
    reg = readl(&priv->eth_reconfig->phy_ber_count);
	printk("altera_s10_100ghip: PHY BER Count = %08x\n", reg);

	/* Offset:0x333 */
    reg = readl(&priv->eth_reconfig->phy_recovered_clock_freq);
	printk("altera_s10_100ghip: PHY RX recovered clock frequency = %08x KHz (/100)\n", reg);
	
	/* Offset:0x341 */
    reg = readl(&priv->eth_reconfig->phy_tx_clock_freq);
	printk("altera_s10_100ghip: PHY TX clock frequency = %08x KHz (/100)\n", reg);
}

static void altera_s10_100ghip_txmac_regdump(struct altera_s10_100ghip_private *priv)
{
	u32 reg;
}

static void altera_s10_100ghip_rxmac_regdump(struct altera_s10_100ghip_private *priv)
{
	u32 reg;

}

void altera_s10_100ghip_regdump(struct altera_s10_100ghip_private *priv)
{
	u32 reg;

	reg = readl(priv->sysid);
	printk("altera_s10_100ghip: sysid = 0x%08x.\n", reg);

    /* Enable the first line only when ANLT is turned on in the HIP */
    /* ANLT only supports backplane Ethernet, so we disable it. */
    /* altera_s10_100ghip_anlt_regdump(priv); */
    altera_s10_100ghip_phy_regdump(priv);
    altera_s10_100ghip_txmac_regdump(priv);
    altera_s10_100ghip_rxmac_regdump(priv);
}

static int altera_s10_100ghip_xcvr_regdump(struct altera_s10_100ghip_private *priv)
{
	u32 reg;

	reg = readl(priv->sysid);
	printk("altera_s10_100ghip: sysid = 0x%08x.\n", reg);

	return 0;
}