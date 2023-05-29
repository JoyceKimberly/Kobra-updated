#pragma once
#include <hc32_ddl.h>
#include <addon_gpio.h>
#include "../adc/adc.h"
#include "gpio_pindefs.h"
#include "HardwareSerial.h"

#ifdef __cplusplus
extern "C"
{
#endif

	//
	// GPIO defines
	//

#define BOARD_NR_GPIO_PINS      83
#define BOARD_NR_ADC_PINS       16

#define BOARD_NR_SPI            3
#define BOARD_SPI1_NSS_PIN      PA4
#define BOARD_SPI1_SCK_PIN      PA5
#define BOARD_SPI1_MISO_PIN     PA6
#define BOARD_SPI1_MOSI_PIN     PA7

#define BOARD_SPI2_NSS_PIN      PB12
#define BOARD_SPI2_SCK_PIN      PB13
#define BOARD_SPI2_MISO_PIN     PB14
#define BOARD_SPI2_MOSI_PIN     PB15

#define BOARD_SPI3_NSS_PIN      PA15
#define BOARD_SPI3_SCK_PIN      PB3
#define BOARD_SPI3_MISO_PIN     PB4
#define BOARD_SPI3_MOSI_PIN     PB5

	//
	// GPIO pin map
	//
	typedef struct hdsc_pin_info
	{
		uint8_t gpio_bit;			 /**< Pin's GPIO port bit. */
		__IO en_port_t gpio_port;
		__IO en_pin_t gpio_pin;
		adc_dev *adc_device;
		__IO uint8_t adc_channel;
		__IO en_port_func_t function;
	} pin_info_t;

	extern const pin_info_t PIN_MAP[];
	extern const uint8_t ADC_PINS[BOARD_NR_ADC_PINS];

/**
 * @brief Feature test: nonzero iff the board has SerialUSB.
 */
 //Roger Clark. Change so that BOARD_HAVE_SERIALUSB is always true, so that it can be controller by -DSERIAL_USB
#define BOARD_HAVE_SERIALUSB 1

/*(defined(BOARD_USB_DISC_DEV) && defined(BOARD_USB_DISC_BIT))*/

extern HardwareSerial MSerial1;
extern HardwareSerial MSerial2;
extern HardwareSerial MSerial3;
extern HardwareSerial MSerial4;

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
