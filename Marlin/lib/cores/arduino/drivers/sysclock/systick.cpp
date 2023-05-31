#include "systick.h"
#include "bsp_timer.h"
#include "timers.h"

volatile uint32_t uptime = 0;

uint32_t millis()
{
    return uptime;
}

uint32_t micros(void)
{
  uint32_t m0 = millis();
  __IO uint32_t u0 = SysTick->VAL;
  uint32_t m1 = millis();
  __IO uint32_t u1 = SysTick->VAL;
  const uint32_t tms = SysTick->LOAD + 1;

  if (m1 != m0) {
    return (m1 * 1000 + ((tms - u1) * 1000) / tms);
  } else {
    return (m0 * 1000 + ((tms - u0) * 1000) / tms);
  }
}

extern "C" void SysTick_IrqHandler(void)
{
    SysTick_IncTick();
}

void systick_init()
{
    stc_clk_freq_t clkFreq;
    CLK_GetClockFreq(&clkFreq);
}

void setup_time2A(const uint32_t frequency)
{
}

void setup_step_tim(const uint32_t frequency)
{
    timer42_set_frequency(frequency);
}

void setup_temp_tim(const uint32_t frequency)
{
}

void timer_enable_irq(const uint8_t timer_num,en_functional_state_t state)
{
    switch (timer_num) {
        case STEP_TIMER_NUM:
            timer42_irq_ctrl(state);
            break;
        case TEMP_TIMER_NUM:

            break;
    }
}

bool timer_irq_enabled(M4_TMR0_TypeDef* pstcTim0Reg, const uint8_t timer_num)
{
    bool state;
    switch (timer_num) {
        case STEP_TIMER_NUM:
            state = timer42_irq_get();
            break;

        case TEMP_TIMER_NUM:

        break;
    }

    return state;
}

en_result_t timer_set_compare(const uint8_t timer_num,const uint16_t compare)
{
    switch (timer_num) {
        case STEP_TIMER_NUM:
            timer42_set_compare(compare);
            break;

        case TEMP_TIMER_NUM:

            break;
    }
}

uint16_t timer_get_count(const uint8_t timer_num)
{
    uint16_t count = 0;

    switch (timer_num) {
        case STEP_TIMER_NUM:
            count = timer42_get_count();
            break;

        case TEMP_TIMER_NUM:

        break;
    }

    return count;
}
