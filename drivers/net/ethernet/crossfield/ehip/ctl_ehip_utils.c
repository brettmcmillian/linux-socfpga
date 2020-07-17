/* Crossfield eHIP DMA Driver for Intel FPGA SoCs
 * Copyright (C) 2014 Altera Corporation. All rights reserved
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

#include "ctl_ehip.h"
#include "ctl_ehip_utils.h"

void ctl_ehip_set_bit(void __iomem *ioaddr, u32 bit_mask)
{
	u32 value = readl(ioaddr);
	value |= bit_mask;
	writel(value, ioaddr);
}

void ctl_ehip_clear_bit(void __iomem *ioaddr, u32 bit_mask)
{
	u32 value = readl(ioaddr);
	value &= ~bit_mask;
	writel(value, ioaddr);
}

int ctl_ehip_bit_is_set(void __iomem *ioaddr, u32 bit_mask)
{
	u32 value = readl(ioaddr);
	return (value & bit_mask) ? 1 : 0;
}

int ctl_ehip_bit_is_clear(void __iomem *ioaddr, u32 bit_mask)
{
	u32 value = readl(ioaddr);
	return (value & bit_mask) ? 0 : 1;
}
