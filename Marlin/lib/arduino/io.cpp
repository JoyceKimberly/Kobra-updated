/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2012 LeafLabs, LLC.
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

#include "io.h"
#include "gpio.h"


void gpio_set_mode(uint8 pin, WiringPinMode mode)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    switch(mode) {

        case OUTPUT:
            stcPortInit.enPinMode = Pin_Mode_Out;
            stcPortInit.enPullUp  = Disable;
        break;

        case INPUT:
        case INPUT_PULLDOWN:
            stcPortInit.enPinMode = Pin_Mode_In;
            stcPortInit.enPullUp  = Disable;
        break;

        case INPUT_PULLUP:
            stcPortInit.enPinMode = Pin_Mode_In;
            stcPortInit.enPullUp  = Enable;
        break;

        default:
        break;
    }

    PORT_InitGPIO(pin, &stcPortInit);
}

void pinMode(uint8 pin, WiringPinMode mode)
{
    gpio_set_mode(pin, mode);
}

