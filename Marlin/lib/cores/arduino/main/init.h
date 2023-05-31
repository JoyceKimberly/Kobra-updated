#pragma once
#include <stdint.h>

#include "hc32_ddl.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define LED0_PORT                       (PortA)
#define LED0_PIN                        (Pin01)

#define LED0_ON()                       (PORT_SetBits(LED0_PORT, LED0_PIN))
#define LED0_OFF()                      (PORT_ResetBits(LED0_PORT, LED0_PIN))
#define LED0_TOGGLE()                   (PORT_Toggle(LED0_PORT, LED0_PIN))

#define LED0_ON()                       (PORT_SetBits(LED0_PORT, LED0_PIN))
#define LED0_OFF()                      (PORT_ResetBits(LED0_PORT, LED0_PIN))
#define LED0_TOGGLE()                   (PORT_Toggle(LED0_PORT, LED0_PIN))

void get_all_clock(void);
void f_cpu_init(uint32_t clock);
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
