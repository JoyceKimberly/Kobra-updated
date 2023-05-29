#pragma once
#include <hc32_ddl.h>
#include <addon_gpio.h>
#include "../adc/adc.h"
#include "../../WVariant.h"
#include "gpio_pindefs.h"
#include "HardwareSerial.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern HardwareSerial MSerial1;
extern HardwareSerial MSerial2;
extern HardwareSerial MSerial3;
extern HardwareSerial MSerial4;

//
// GPIO wrappers for PORT_* functions
//
#define ASSERT_GPIO_PIN(gpio_pin) \
    if (!IS_GPIO_PIN(gpio_pin))   \
    {                             \
        return Error;             \
    }
#define PIN_ARG(gpio_pin) PIN_MAP[gpio_pin].gpio_port, PIN_MAP[gpio_pin].gpio_pin

    /**
     * @brief GPIO wrapper for PORT_Init
     */
	extern inline en_result_t GPIO_Init(uint8_t gpio_pin, const stc_port_init_t *pstcPortInit)
	{
		if (gpio_pin > BOARD_NR_GPIO_PINS)
		{
			return Error;
		}

		return PORT_Init(PIN_MAP[gpio_pin].gpio_port, PIN_MAP[gpio_pin].gpio_pin, pstcPortInit);
	}

    /**
     * @brief GPIO wrapper for PORT_GetConfig
     */
	extern inline en_result_t GPIO_GetConfig(uint8_t gpio_pin, stc_port_init_t *pstcPortInit)
	{
		if (gpio_pin > BOARD_NR_GPIO_PINS)
		{
			return Error;
		}

		return PORT_GetConfig(PIN_MAP[gpio_pin].gpio_port, PIN_MAP[gpio_pin].gpio_pin, pstcPortInit);
	}

    /**
     * @brief GPIO wrapper for PORT_GetBit
     */
	extern inline uint8_t GPIO_GetBit(uint8_t gpio_pin)
	{
		if (gpio_pin > BOARD_NR_GPIO_PINS)
		{
			return Error;
		}

		return (PORT_GetBit(PIN_MAP[gpio_pin].gpio_port, PIN_MAP[gpio_pin].gpio_pin) == Reset) ? false : true;
	}

    /**
     * @brief GPIO wrapper for PORT_SetBits
     */
	extern inline en_result_t GPIO_SetBits(uint8_t gpio_pin)
	{
		if (gpio_pin > BOARD_NR_GPIO_PINS)
		{
			return Error;
		}

		return PORT_SetBits(PIN_MAP[gpio_pin].gpio_port, PIN_MAP[gpio_pin].gpio_pin);
	}

    /**
     * @brief GPIO wrapper for PORT_ResetBits
     */
	extern inline en_result_t GPIO_ResetBits(uint8_t gpio_pin)
	{
		if (gpio_pin > BOARD_NR_GPIO_PINS)
		{
			return Error;
		}

		return PORT_ResetBits(PIN_MAP[gpio_pin].gpio_port, PIN_MAP[gpio_pin].gpio_pin);
	}

    /**
     * @brief GPIO wrapper for PORT_Toggle
     */
	extern inline en_result_t GPIO_Toggle(uint8_t gpio_pin)
	{
		if (gpio_pin > BOARD_NR_GPIO_PINS)
		{
			return Error;
		}

		return PORT_Toggle(PIN_MAP[gpio_pin].gpio_port, PIN_MAP[gpio_pin].gpio_pin);
	}

    /**
     * @brief GPIO wrapper for PORT_SetFunc
     * @note function select is chosen in PIN_MAP
     */
	extern inline en_result_t GPIO_SetFunc(uint8_t gpio_pin, en_functional_state_t subFunction)
	{
		if (gpio_pin > BOARD_NR_GPIO_PINS)
		{
			return Error;
		}

		return PORT_SetFunc(PIN_MAP[gpio_pin].gpio_port, PIN_MAP[gpio_pin].gpio_pin, PIN_MAP[gpio_pin].function, subFunction);
	}

extern void setup_gpio(void);

#ifdef __cplusplus
}
#endif
