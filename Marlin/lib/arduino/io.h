/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file wirish/include/wirish/io.h
 * @brief Wiring-style pin I/O interface.
 */

#ifndef _WIRISH_IO_H_
#define _WIRISH_IO_H_

#include <libmaple/libmaple_types.h>
#include "../board/startup.h"

typedef enum WiringPinMode {
    OUTPUT, 
    OUTPUT_OPEN_DRAIN, 
    INPUT, 
    INPUT_ANALOG, 
    INPUT_PULLUP,
    INPUT_PULLDOWN, 
    INPUT_FLOATING, 
    PWM, 
    PWM_OPEN_DRAIN, 
} WiringPinMode;

void pinMode(uint8 pin, WiringPinMode mode);

#define HIGH 0x1
#define LOW  0x0

void digitalWrite(uint8 pin, uint8 val);
uint32 digitalRead(uint8 pin);

void pwmWrite(uint8 pin, uint16 duty_cycle16);
void analogWrite(uint8 pin, int duty_cycle8);
uint16 analogRead(uint8 pin);
void gpio_set_mode(uint8 pin, WiringPinMode mode);
WiringPinMode gpio_get_mode(uint8 PinNum) ;

#endif
