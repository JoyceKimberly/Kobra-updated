/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#ifdef TARGET_HC32F46x

#include "../../inc/MarlinConfig.h"

bool SDIO_Init()
{
    return (steup_sdio());
}

/**
 * @brief Read a block
 * @details Read a block from media with SDIO
 *
 * @param block The block index
 * @param src The block buffer
 *
 * @return true on success
 */
bool SDIO_ReadBlock(uint32_t blockAddress, uint8_t *data) {
	uint32_t retries = 3;
	while (retries--) if (SDIO_ReadBlock_DMA(blockAddress, data)) return true;
	return false;
}

/**
 * @brief Write a block
 * @details Write a block to media with SDIO
 *
 * @param block The block index
 * @param src The block data
 *
 * @return true on success
 */
bool SDIO_WriteBlock(uint32_t blockAddress, const uint8_t *data) {
	return SDIO_WriteBlockDMA(blockAddress,data);
}

#endif // TARGET_HC32F46x
