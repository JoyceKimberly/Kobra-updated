#ifndef _BSP_H
#define  _BSP_H

#include "string.h"
//#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <inttypes.h>

//#define ENABLE_TYPE_007

#include "hc32_ddl.h"
#include "../core/macros.h"
#include "../core/boards.h"

#include "systick.h"
#include "board_cfg.h" 
#include "gpio.h" 
#include "adc.h"
#include "../pins/pins.h"
#include "board_sdio.h"
#include "usart.h"
#include "HardwareSerial.h"

#include "libmaple_types.h"
#include "wirish_types.h"
#include "flash.h"

#ifdef __cplusplus
extern "C"{
#endif

extern uint32_t F_CPU;
#define CYCLES_PER_MICROSECOND  (F_CPU / 1000000UL)

typedef enum ExtIntTriggerMode {
    RISING, /**< To trigger an interrupt when the pin transitions LOW
                 to HIGH */
    FALLING, /**< To trigger an interrupt when the pin transitions
                  HIGH to LOW */
    CHANGE /**< To trigger an interrupt when the pin transitions from
                LOW to HIGH or HIGH to LOW (i.e., when the pin
                changes). */
} ExtIntTriggerMode;

static inline void nvic_globalirq_enable() {
    asm volatile("cpsie i");
}
static inline void nvic_globalirq_disable() {
    asm volatile("cpsid i");
}
static inline void interrupts() {
    nvic_globalirq_enable();
}
static inline void noInterrupts() {
    nvic_globalirq_disable();
}
void init(void);
void f_cpu_init(uint32_t clock);
extern void setup_Extinterrupt(void);
extern void attachInterrupt(uint8_t pin, voidFuncPtr handler, uint8_t irqNum,ExtIntTriggerMode mode);
extern void ExtInt_X_MIN_Callback(void);
extern void ExtInt_Y_MIN_Callback(void);
extern void ExtInt_Z_MIN_Callback(void);
extern void ExtInt_Z2_MIN_Callback(void);
extern void ExtInt_Z_MIN_PROBE_Callback(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
