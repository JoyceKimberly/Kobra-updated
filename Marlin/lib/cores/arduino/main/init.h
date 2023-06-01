#pragma once
#include <stdint.h>

#define LD_FLASH_START 0x8000u
// get flash start address from ddl framework
#ifndef LD_FLASH_START
#warning "LD_FLASH_START not defined, fallback to 0x0"
#define LD_FLASH_START 0x0
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#define LED0_PORT                       (PortA)
#define LED0_PIN                        (Pin01)

#define LED_ON()                        (PORT_SetBits(LED0_PORT, LED0_PIN))
#define LED_OFF()                       (PORT_ResetBits(LED0_PORT, LED0_PIN))
#define LED_TOGGLE()                    (PORT_Toggle(LED0_PORT, LED0_PIN))

#define LED0_ON()                       (PORT_SetBits(LED0_PORT, LED0_PIN))
#define LED0_OFF()                      (PORT_ResetBits(LED0_PORT, LED0_PIN))
#define LED0_TOGGLE()                   (PORT_Toggle(LED0_PORT, LED0_PIN))

void get_all_clock(void);
void led_pin_init(void);
void endstop_pin_init(void);
void stepper_pin_init(void);
void heater_pin_init(void);
void fan_pin_init(void);

#ifdef __cplusplus
}
#endif

/**
 * @brief initialize the HC32F460 SoC 
 */
void core_init();
