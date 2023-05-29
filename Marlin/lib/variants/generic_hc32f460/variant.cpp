/*
 Copyright (c) 2015 Arduino LLC.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"
#include "WVariant.h"

extern const pin_info_t PIN_MAP[BOARD_NR_GPIO_PINS] = {
	// PAx
    {0,  PortA, Pin00, &adc1,  ADC1_IN0,         Func_Gpio},
    {1,  PortA, Pin01, &adc1,  ADC1_IN1,         Func_Gpio},
    {2,  PortA, Pin02, &adc1,  ADC1_IN2,         Func_Usart2_Tx},
    {3,  PortA, Pin03, &adc1,  ADC1_IN3,         Func_Usart2_Rx},
    {4,  PortA, Pin04, &adc1,  ADC12_IN4,        Func_Gpio},
    {5,  PortA, Pin05, &adc1,  ADC12_IN5,        Func_Gpio},
    {6,  PortA, Pin06, &adc1,  ADC12_IN6,        Func_Gpio},
    {7,  PortA, Pin07, &adc1,  ADC12_IN7,        Func_Gpio},
    {8,  PortA, Pin08, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {9,  PortA, Pin09, NULL,   ADC_PIN_INVALID,  Func_Usart1_Tx},
    {10, PortA, Pin10, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {11, PortA, Pin11, NULL,   ADC_PIN_INVALID,  Func_I2c1_Sda},
    {12, PortA, Pin12, NULL,   ADC_PIN_INVALID,  Func_I2c1_Scl},
    {13, PortA, Pin13, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {14, PortA, Pin14, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {15, PortA, Pin15, NULL,   ADC_PIN_INVALID,  Func_Usart1_Rx},

	// PBx
    {0,  PortB, Pin00, &adc1,  ADC12_IN8,        Func_Gpio},
    {1,  PortB, Pin01, &adc1,  ADC12_IN9,        Func_Gpio},
    {2,  PortB, Pin02, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {3,  PortB, Pin03, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {4,  PortB, Pin04, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {5,  PortB, Pin05, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {6,  PortB, Pin06, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {7,  PortB, Pin07, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {8,  PortB, Pin08, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {9,  PortB, Pin09, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {10, PortB, Pin10, NULL,   ADC_PIN_INVALID,  Func_Usart3_Tx},
    {11, PortB, Pin11, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {12, PortB, Pin12, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {13, PortB, Pin13, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {14, PortB, Pin14, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {15, PortB, Pin15, NULL,   ADC_PIN_INVALID,  Func_Gpio},

	// PCx
    {0,  PortC, Pin00, &adc1,  ADC12_IN10,       Func_Gpio},
    {1,  PortC, Pin01, &adc1,  ADC12_IN11,       Func_Gpio},
    {2,  PortC, Pin02, &adc1,  ADC1_IN12,        Func_Gpio},
    {3,  PortC, Pin03, &adc1,  ADC1_IN13,        Func_Gpio},
    {4,  PortC, Pin04, &adc1,  ADC1_IN14,        Func_Gpio},
    {5,  PortC, Pin05, &adc1,  ADC1_IN15,        Func_Gpio},
    {6,  PortC, Pin06, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {7,  PortC, Pin07, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {8,  PortC, Pin08, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {9,  PortC, Pin09, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {10, PortC, Pin10, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {11, PortC, Pin11, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {12, PortC, Pin12, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {13, PortC, Pin13, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {14, PortC, Pin14, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {15, PortC, Pin15, NULL,   ADC_PIN_INVALID,  Func_Gpio},

	// PDx
    {0,  PortD, Pin00, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {1,  PortD, Pin01, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {2,  PortD, Pin02, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {3,  PortD, Pin03, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {4,  PortD, Pin04, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {5,  PortD, Pin05, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {6,  PortD, Pin06, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {7,  PortD, Pin07, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {8,  PortD, Pin08, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {9,  PortD, Pin09, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {10, PortD, Pin10, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {11, PortD, Pin11, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {12, PortD, Pin12, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {13, PortD, Pin13, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {14, PortD, Pin14, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {15, PortD, Pin15, NULL,   ADC_PIN_INVALID,  Func_Gpio},

	// PEx
    {0,  PortE, Pin00, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {1,  PortE, Pin01, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {2,  PortE, Pin02, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {3,  PortE, Pin03, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {4,  PortE, Pin04, NULL,   ADC_PIN_INVALID,  Func_Usart3_Rx},
    {5,  PortE, Pin05, NULL,   ADC_PIN_INVALID,  Func_Usart3_Tx},
    {6,  PortE, Pin06, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {7,  PortE, Pin07, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {8,  PortE, Pin08, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {9,  PortE, Pin09, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {10, PortE, Pin10, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {11, PortE, Pin11, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {12, PortE, Pin12, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {13, PortE, Pin13, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {14, PortE, Pin14, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {15, PortE, Pin15, NULL,   ADC_PIN_INVALID,  Func_Gpio},

	// PHx
    {0,  PortH, Pin00, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {1,  PortH, Pin01, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {2,  PortH, Pin02, NULL,   ADC_PIN_INVALID,  Func_Usart3_Rx},
};
