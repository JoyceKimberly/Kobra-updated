#include "delay.h"
#include <hc32_ddl.h>
#include "systick.h"
#include "bsp_timer.h"
#include "timers.h"

void delay(uint32_t ms)
{
    SysTick_Delay(ms);
}

void delayMicroseconds(uint32_t us)
{
    for(uint32_t i=0; i<us; i++) {
        for(uint32_t j=0; j<8; j++) {
            __NOP();
        }
    }
}
