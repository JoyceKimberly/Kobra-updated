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
#include "../../inc/MarlinConfig.h"
//#include "MarlinSerial.h"

//
// Default graphical display delays
//
#define CPU_ST7920_DELAY_1 300
#define CPU_ST7920_DELAY_2  40
#define CPU_ST7920_DELAY_3 340

//
// Serial Ports
//
#define _MSERIAL(X) MSerial##X
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

//
// Emergency Parser
//
#if ENABLED(EMERGENCY_PARSER)
extern "C" void usart_rx_irq_hook(uint8_t ch, uint8_t usart);
#endif

//
// Misc. Defines
//
#define STM32_FLASH_SIZE 256
#define square(x) ((x) * (x))

#ifndef strncpy_P
#define strncpy_P(dest, src, num) strncpy((dest), (src), (num))
#endif

//
// Misc. Functions
//
#ifndef analogInputToDigitalPin
#define analogInputToDigitalPin(p) (p)
#endif

#define HAL_CAN_SET_PWM_FREQ   // This HAL supports PWM Frequency adjustment

#define CRITICAL_SECTION_START()        \
  const bool irqon = !__get_PRIMASK(); \
  __disable_irq()

#define CRITICAL_SECTION_END() \
  if (irqon)              \
  __enable_irq()

// Disable interrupts
#define cli() __disable_irq()

// Enable interrupts
#define sei() __enable_irq()

// bss_end alias
#define __bss_end __bss_end__

// fix bug in pgm_read_ptr
#undef pgm_read_ptr
#define pgm_read_ptr(addr) (*(addr))

//
// ADC
//
#define HAL_ADC_VREF 3.3
#define HAL_ADC_RESOLUTION 12

#define GET_PIN_MAP_PIN(index) index
#define GET_PIN_MAP_INDEX(pin) pin
#define PARSED_PIN_INDEX(code, dval) parser.intval(code, dval)

#define JTAG_DISABLE()    // afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY)
#define JTAGSWD_DISABLE() // afio_cfg_debug_ports(AFIO_DEBUG_NONE)

//
// MarlinHAL implementation
//
#include "MarlinHAL.h"
