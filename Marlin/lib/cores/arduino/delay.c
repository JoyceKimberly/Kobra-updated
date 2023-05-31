#include "delay.h"
#include <hc32_ddl.h>
#include "systick.h"
#include "bsp_timer.h"
#include "timers.h"

void delay(uint32_t dwMs)
{
    SysTick_Delay(dwMs);
}

void delayMicroseconds(uint32_t dwUs)
{
    for(uint32_t i=0; i<dwUs; i++) {
        for(uint32_t j=0; j<8; j++) {
            __NOP();
        }
    }
}
