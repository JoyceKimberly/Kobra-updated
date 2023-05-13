/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2017 Victor Perez
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

#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"
#include "HAL.h"
#include "rmu.h"

// ------------------------
// Public Variables
// ------------------------

uint16_t HAL_adc_result;
uint16_t MarlinHAL::adc_result;

// ------------------------
// Public functions
// ------------------------

#if ENABLED(POSTMORTEM_DEBUGGING)
  extern void install_min_serial();
#endif

// HAL initialization task
void MarlinHAL::init() {
  // Ensure F_CPU is a constant expression.
  // If the compiler breaks here, it means that delay code that should compute at compile time will not work.
  // So better safe than sorry here.
  constexpr int cpuFreq = F_CPU;
  UNUSED(cpuFreq);

  NVIC_SetPriorityGrouping(0x3);

  #if HAS_MEDIA && DISABLED(ONBOARD_SDIO) && (defined(SDSS) && SDSS != -1)
    OUT_WRITE(SDSS, HIGH); // Try to set SDSS inactive before any other SPI users start up
  #endif

  #if PIN_EXISTS(LED)
    OUT_WRITE(LED_PIN, LOW);
  #endif

  #if PIN_EXISTS(AUTO_LEVEL_TX)
    OUT_WRITE(AUTO_LEVEL_TX_PIN, HIGH);
    delay(10);
    OUT_WRITE(AUTO_LEVEL_TX_PIN, LOW);
    delay(300);
    OUT_WRITE(AUTO_LEVEL_TX_PIN, HIGH);
  #endif

  #if ENABLED(SRAM_EEPROM_EMULATION)
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();           // Enable access to backup SRAM
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    LL_PWR_EnableBkUpRegulator();         // Enable backup regulator
    while (!LL_PWR_IsActiveFlag_BRR());   // Wait until backup regulator is initialized
  #endif

  //SetTimerInterruptPriorities();

  #if ENABLED(EMERGENCY_PARSER) && (USBD_USE_CDC || USBD_USE_CDC_MSC)
    USB_Hook_init();
  #endif

  TERN_(POSTMORTEM_DEBUGGING, install_min_serial());    // Install the min serial handler

}

void _delay_ms(const int delay_ms) { delay(delay_ms); }

uint32_t AD_DMA[3];

// Init the AD in continuous capture mode
void MarlinHAL::adc_init() {}
//TODO: Make sure this doesn't cause any delay
void HAL_adc_start_conversion(const uint8_t adc_pin) {
        if(adc_pin>BOARD_NR_GPIO_PINS)return;
        uint8_t channel = PIN_MAP[adc_pin].adc_channel;
        DDL_ASSERT(channel!=ADC_PIN_INVALID);
        HAL_adc_result = adc_read(ADC1,channel);
        switch(adc_pin)
        {
            case TEMP_BED_PIN: AD_DMA[0] = HAL_adc_result;break;
            case TEMP_0_PIN: AD_DMA[1] = HAL_adc_result;break;
            case POWER_MONITOR_VOLTAGE_PIN: AD_DMA[2] = HAL_adc_result;break;
            default:break;
        }
}
uint16_t HAL_adc_get_result() { return 1000; } // { return HAL_adc_result; }

void MarlinHAL::reboot() { NVIC_SystemReset(); }

uint8_t MarlinHAL::get_reset_source() {
    uint8_t res;
    res = rmu_get_reset_cause();
    return res;
}

void MarlinHAL::clear_reset_source() { rmu_clear_reset_cause(); }

// ------------------------
// Watchdog Timer
// ------------------------

#if ENABLED(USE_WATCHDOG)

  #include <iwdg.h>

  void watchdogSetup() {
    // do whatever. don't remove this function.
  }

  /**
   *  The watchdog clock is 40Khz. So for a 4s or 8s interval use a /256 preescaler and 625 or 1250 reload value (counts down to 0).
   */
  #define STM32F1_WD_RELOAD TERN(WATCHDOG_DURATION_8S, 1250, 625) // 4 or 8 second timeout

  /**
   * @brief  Initialize the independent hardware watchdog.
   *
   * @return No return
   *
   * @details The watchdog clock is 40Khz. So for a 4s or 8s interval use a /256 preescaler and 625 or 1250 reload value (counts down to 0).
   */
  void MarlinHAL::watchdog_init() {
    #if DISABLED(DISABLE_WATCHDOG_INIT)
      iwdg_init();
    #endif
  }

  // Reset watchdog. MUST be called every 4 or 8 seconds after the
  // first watchdog_init or the STM32F1 will reset.
  void MarlinHAL::watchdog_refresh() {
    #if DISABLED(PINS_DEBUGGING) && PIN_EXISTS(LED)
      TOGGLE(LED_PIN);  // heartbeat indicator
    #endif
    iwdg_feed();
  }

#endif

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

// Reset the system to initiate a firmware flash
void flashFirmware(const int16_t) { hal.reboot(); }

// Maple Compatibility
volatile uint32_t systick_uptime_millis = 0;
systickCallback_t systick_user_callback;
void systick_attach_callback(systickCallback_t cb) { systick_user_callback = cb; }
void HAL_SYSTICK_Callback() {
  systick_uptime_millis++;
  if (systick_user_callback) systick_user_callback();
}

