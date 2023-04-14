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
#pragma once

/**
 * HAL for stm32duino.com based on Libmaple and compatible (HC32F46x based on STM32F1)
 */

#define CPU_32_BIT

#include "../../core/macros.h"
#include "../shared/Marduino.h"
#include "../shared/math_32bit.h"
#include "../shared/HAL_SPI.h"

#include "fastio.h"
#include "timers.h"

#include <stdint.h>

#include "../../inc/MarlinConfigPre.h"
#include "../inc/MarlinConfig.h"

//#include "MarlinSerial.h"

//
// Default graphical display delays
//
#define CPU_ST7920_DELAY_1 300
#define CPU_ST7920_DELAY_2  40
#define CPU_ST7920_DELAY_3 340

#define _MSERIAL(X) Serial##X
#define MSERIAL(X) _MSERIAL(X)
#define NUM_UARTS 4

#if WITHIN(SERIAL_PORT, 1, 6)
  #define MYSERIAL1 MSERIAL(SERIAL_PORT)
#else
  #error "SERIAL_PORT must be from 1 to 6, or -1 for Native USB."
#endif


#ifdef LCD_SERIAL_PORT
  #if WITHIN(LCD_SERIAL_PORT, 1, 6)
    #define LCD_SERIAL MSERIAL(LCD_SERIAL_PORT)
  #else
    #error "LCD_SERIAL_PORT must be from 1 to 6, or -1 for Native USB."
  #endif
  #if HAS_DGUS_LCD
    #define SERIAL_GET_TX_BUFFER_FREE() LCD_SERIAL.availableForWrite()
  #endif
#endif

/**
 * TODO: review this to return 1 for pins that are not analog input
 */
#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) (p)
#endif

// On AVR this is in math.h?
#define square(x) ((x)*(x))

#ifndef strncpy_P
  #define strncpy_P(dest, src, num) strncpy((dest), (src), (num))
#endif

// Fix bug in pgm_read_ptr
#undef pgm_read_ptr
#define pgm_read_ptr(addr) (*(addr))

// ------------------------
// Types
// ------------------------

typedef double isr_float_t;   // FPU ops are used for single-precision, so use double for ISRs.
typedef int16_t pin_t;

// ------------------------
// Interrupts
// ------------------------

#define CRITICAL_SECTION_START()  NOOP//uint32_t primask = __get_PRIMASK(); __disable_irq()
#define CRITICAL_SECTION_END()    NOOP//if (!primask) __enable_irq()
#define cli() __disable_irq()
#define sei() __enable_irq()

// ------------------------
// ADC
// ------------------------

#ifdef ADC_RESOLUTION
  #define HAL_ADC_RESOLUTION ADC_RESOLUTION
#else
  #define HAL_ADC_RESOLUTION 12
#endif

#define HAL_ADC_VREF         3.3

//
// Pin Mapping for M42, M43, M226
//
#define GET_PIN_MAP_PIN(index) index
#define GET_PIN_MAP_INDEX(pin) pin
#define PARSED_PIN_INDEX(code, dval) parser.intval(code, dval)

#define JTAG_DISABLE()    // afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY)
#define JTAGSWD_DISABLE() // afio_cfg_debug_ports(AFIO_DEBUG_NONE)

#define PLATFORM_M997_SUPPORT
void flashFirmware(const int16_t);

// Maple Compatibility
typedef void (*systickCallback_t)(void);
void systick_attach_callback(systickCallback_t cb);
void HAL_SYSTICK_Callback();

extern volatile uint32_t systick_uptime_millis;

#define HAL_CAN_SET_PWM_FREQ   // This HAL supports PWM Frequency adjustment

// ------------------------
// Class Utilities
// ------------------------

// Memory related
#define __bss_end __bss_end__

void _delay_ms(const int delay);

extern "C" char* _sbrk(int incr);

#pragma GCC diagnostic push
#if GCC_VERSION <= 50000
  #pragma GCC diagnostic ignored "-Wunused-function"
#endif

static inline int freeMemory() {
  volatile char top;
  return top;
}

#pragma GCC diagnostic pop

// ------------------------
// MarlinHAL Class
// ------------------------

class MarlinHAL {
public:

  // Earliest possible init, before setup()
  MarlinHAL() {}

  // Watchdog
  static void watchdog_init()    IF_DISABLED(USE_WATCHDOG, {});
  static void watchdog_refresh() IF_DISABLED(USE_WATCHDOG, {});

  static void init();          // Called early in setup()
  static void init_board() {}  // Called less early in setup()
  static void reboot();        // Restart the firmware from 0x0

  // Interrupts
  static bool isr_state() { return true; }
  static void isr_on()  { (sei()); }
  static void isr_off() { (cli()); }

  static uint16_t adc_result;

  // Free SRAM
  static int freeMemory() { return ::freeMemory(); }
};

  void HAL_init();
  inline void HAL_reboot() {}  // reboot the board or restart the bootloader

  // Interrupts
  #define ISRS_ENABLED() NOOP//(!__get_PRIMASK())
  #define ENABLE_ISRS()  __enable_irq()
  #define DISABLE_ISRS() __disable_irq()

  // Reset
  uint8_t HAL_get_reset_source();
  void HAL_clear_reset_source();

  //
  // ADC Methods
  //

  // result of last ADC conversion
  extern uint16_t HAL_adc_result;

  extern uint16_t g_adc_value[3];
  extern uint8_t g_adc_idx;

  uint16_t HAL_adc_get_result();

  // Called by Temperature::init once at startup
  void HAL_adc_init();

  // Called by Temperature::init for each sensor at startup
  void HAL_adc_start_conversion(const uint8_t adc_pin);

  // Begin ADC sampling on the given pin. Called from Temperature::isr!
  #define HAL_START_ADC(pin)  HAL_adc_start(pin)
  inline static void HAL_adc_start(uint32_t pin)
  {
      if       (pin == TEMP_BED_PIN) {
          g_adc_idx = 0;
      } else if(pin == TEMP_0_PIN) {
          g_adc_idx = 1;
      } else if(pin == POWER_MONITOR_VOLTAGE_PIN) {
          g_adc_idx = 2;
      } else {
          g_adc_idx = 0x0;
      }
  }

  // Is the ADC ready for reading?
  #define HAL_ADC_READY() true

  // The current value of the ADC register
  #define HAL_READ_ADC()      HAL_adc_read()
  inline static uint32_t HAL_adc_read()
  {
    return g_adc_value[g_adc_idx];
  }

  /**
   * Set the PWM duty cycle for the pin to the given value.
   * Optionally invert the duty cycle [default = false]
   * Optionally change the maximum size of the provided value to enable finer PWM duty control [default = 255]
   * The timer must be pre-configured with set_pwm_frequency() if the default frequency is not desired.
   */
  void set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t v_size=255, const bool invert=false);

  /**
   * Set the frequency of the timer for the given pin.
   * All Timer PWM pins run at the same frequency.
   */
  void set_pwm_frequency(const pin_t pin, int f_desired);

