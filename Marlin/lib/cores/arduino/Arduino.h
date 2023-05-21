/*
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Arduino_h
#define Arduino_h

#include "wirish.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned int word;

// some libraries and sketches depend on this AVR stuff,
// assuming Arduino.h or WProgram.h automatically includes it...
//
#include "avr/pgmspace.h"
#include "avr/interrupt.h"
#include "avr/io.h"
#include "avr/dtostrf.h"

#include "binary.h"
#include "itoa.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus
// Weak empty variant initialization function.
// May be redefined by variant files.
extern void initVariant() __attribute__((weak));

void yield( void ) ;

  /* sketch */
extern void setup(void) ;
extern void loop(void) ;

#ifdef __cplusplus
} // extern "C"
#endif

// The following headers are for C++ only compilation
#ifdef __cplusplus
#include <WCharacter.h>
#include <WString.h>
#include <tone.h>
#include <wirish_math.h>
#include <HardwareSerial.h>
#endif

// Include pins variant
#include "pins_arduino.h"

#include <io.h>
#include <wirish_types.h>
#include <wirish_time.h>
#include <libmaple.h>

/* Wiring macros and bit defines */

#define true 0x1
#define false 0x0

#define lowByte(w)                     ((w) & 0xFF)
#define highByte(w)                    (((w) >> 8) & 0xFF)

#define bitRead(value, bit)            (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)             ((value) |= (1UL << (bit)))
#define bitClear(value, bit)           ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : \
                                                   bitClear(value, bit))

#define bit(b)                         (1UL << (b))

// Roger Clark. Added _BV macro for AVR compatibility. As requested by @sweetlilmre and @stevestrong
#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif 

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define microsecondsToClockCycles(a) ( (a) * (F_CPU / 1000000L) )

#define digitalPinToInterrupt(pin) (pin)

#endif // Arduino_h
