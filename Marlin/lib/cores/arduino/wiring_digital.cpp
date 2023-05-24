#include "wiring_digital.h"
#include "drivers/gpio/gpio.h"
#include "wiring_constants.h"

void pinMode(uint32_t dwPin, uint32_t dwMode)
{
    if (dwPin >= BOARD_NR_GPIO_PINS)
    {
        return;
    }

    // build pin configuration
    stc_port_init_t pinConf;
    MEM_ZERO_STRUCT(pinConf);
    switch (dwMode)
    {
    case INPUT:
        pinConf.enPinMode = Pin_Mode_In;
        break;
    case INPUT_PULLUP:
        pinConf.enPinMode = Pin_Mode_In;
        pinConf.enPullUp = Enable;
        break;
    case INPUT_ANALOG:
        pinConf.enPinMode = Pin_Mode_Ana;
        break;
    case OUTPUT:
        pinConf.enPinMode = Pin_Mode_Out;
        break;
    default:
        return;
    }

    // set pind config
    PORT_InitGPIO(dwPin, &pinConf);
}

void digitalWrite(uint32_t dwPin, uint32_t dwVal)
{
    if (dwPin >= BOARD_NR_GPIO_PINS)
    {
        return;
    }

    if (dwVal == LOW)
    {
        PORT_ResetBitsGPIO(dwPin);
    }
    else
    {
        PORT_SetBitsGPIO(dwPin);
    }
}

int digitalRead(uint32_t ulPin)
{
    if (ulPin >= BOARD_NR_GPIO_PINS)
    {
        return LOW;
    }

    return PORT_GetBitGPIO(ulPin) ? HIGH : LOW;
}
