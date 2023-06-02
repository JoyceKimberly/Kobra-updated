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
#pragma once

#include "../inc/MarlinConfig.h"

#include "SdInfo.h"

bool SDIO_Init();
bool SDIO_ReadBlock(uint32_t block, uint8_t *dst);
bool SDIO_WriteBlock(uint32_t block, const uint8_t *src);
bool SDIO_IsReady();
uint32_t SDIO_GetCardSize();

class DiskIODriver {
  public:
    bool init(const uint8_t sckRateID=0, const pin_t chipSelectPin=0) { return SDIO_Init(); }

    bool readCSD(csd_t *csd)                              { return false; }

    bool readStart(const uint32_t block)                  { curBlock = block; return true; }
    bool readData(uint8_t *dst)                           { return readBlock(curBlock++, dst); }
    bool readStop()                                       { curBlock = -1; return true; }

    bool writeStart(const uint32_t block, const uint32_t) { curBlock = block; return true; }
    bool writeData(const uint8_t *src)                    { return writeBlock(curBlock++, src); }
    bool writeStop()                                      { curBlock = -1; return true; }

    bool readBlock(uint32_t block, uint8_t *dst)          { return SDIO_ReadBlock(block, dst); }
    bool writeBlock(uint32_t block, const uint8_t *src)   { return SDIO_WriteBlock(block, src); }

    uint32_t cardSize()                                   { return SDIO_GetCardSize(); }

    bool isReady()                                        { return SDIO_IsReady(); }

    void idle()                                           {}
  private:
    uint32_t curBlock;
};
