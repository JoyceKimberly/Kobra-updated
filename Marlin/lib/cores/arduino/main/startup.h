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
#include "gpio.h" 
#include "adc.h"
#include "../pins/pins.h"
#include "board_sdio.h"
#include "usart.h"
#include "HardwareSerial.h"

#include "Arduino.h"
#include "flash.h"

#ifdef __cplusplus
extern "C"{
#endif

void init(void);
void f_cpu_init(uint32_t clock);
extern void setup_Extinterrupt(void);
extern void ExtInt_X_MIN_Callback(void);
extern void ExtInt_Y_MIN_Callback(void);
extern void ExtInt_Z_MIN_Callback(void);
extern void ExtInt_Z2_MIN_Callback(void);
extern void ExtInt_Z_MIN_PROBE_Callback(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
