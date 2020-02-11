/* Altera TSE SGDMA and MSGDMA Linux driver
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

#include "altera_s10_100ghip.h"
#include "altera_s10_utils.h"

void s10_set_bit(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = csrrd32(ioaddr, offs);
	value |= bit_mask;
	csrwr32(value, ioaddr, offs);
}

void s10_clear_bit(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = csrrd32(ioaddr, offs);
	value &= ~bit_mask;
	csrwr32(value, ioaddr, offs);
}

int s10_bit_is_set(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = csrrd32(ioaddr, offs);
	return (value & bit_mask) ? 1 : 0;
}

int s10_bit_is_clear(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = csrrd32(ioaddr, offs);
	return (value & bit_mask) ? 0 : 1;
}
