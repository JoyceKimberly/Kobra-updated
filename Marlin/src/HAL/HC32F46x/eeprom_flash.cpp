/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2016 Victor Perez victor_pv@hotmail.com
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

/**
 * persistent_store_flash.cpp
 * HAL for stm32duino and compatible (HC32F46x based on STM32F1)
 * Implementation of EEPROM settings in SDCard
 */

#ifdef TARGET_HC32F46x

#include "../../inc/MarlinConfig.h"

#if ENABLED(FLASH_EEPROM_EMULATION)

#include "../shared/eeprom_api.h"

static bool eeprom_data_written = false;

#ifndef MARLIN_EEPROM_SIZE
  #define MARLIN_EEPROM_SIZE size_t(E2END + 1)
#endif
size_t PersistentStore::capacity() { return MARLIN_EEPROM_SIZE; }

bool PersistentStore::access_start() {
    Intflash::eeprom_buffer_fill();

  return true;
}

bool PersistentStore::access_finish() {

  if (eeprom_data_written) {
      TERN_(HAS_PAUSE_SERVO_OUTPUT, PAUSE_SERVO_OUTPUT());
      DISABLE_ISRS();
      Intflash::eeprom_buffer_flush();
      ENABLE_ISRS();
      TERN_(HAS_PAUSE_SERVO_OUTPUT, RESUME_SERVO_OUTPUT());

      eeprom_data_written = false;
  }

  return true;
}

bool PersistentStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {
  while (size--) {
    uint8_t v = *value;
      if (v != Intflash::eeprom_buffered_read_byte(pos)) {
        Intflash::eeprom_buffered_write_byte(pos, v);
        eeprom_data_written = true;
      }
    crc16(crc, &v, 1);
    pos++;
    value++;
  }
  return false;
}

bool PersistentStore::read_data(int &pos, uint8_t *value, size_t size, uint16_t *crc, const bool writing/*=true*/) {
  do {
    const uint8_t c = TERN(FLASH_EEPROM_LEVELING, ram_eeprom[pos], Intflash::eeprom_buffered_read_byte(pos));
    if (writing) *value = c;
    crc16(crc, &c, 1);
    pos++;
    value++;
  } while (--size);
  return false;
}

uint32_t PersistentStore::FLASH_If_Erase(uint32_t addr_start, uint32_t addr_end) {
    Intflash::FlashErasePage(addr_start);
    return(0);
}

uint32_t PersistentStore::FLASH_If_Write(uint32_t destination, const void *p_source, uint32_t length) {
  return (Intflash::Flash_Updata(destination,p_source,length));
}

#endif // FLASH_EEPROM_EMULATION
#endif // TARGET_HC32F46x
