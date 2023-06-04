#include "timers.h"
#include <core_debug.h>
#include <hc32_ddl.h>

/**
 * HAL only uses Timer0 Unit 2 for Temperature and Step interrupts
 */

void HAL_timer_start(const timer_channel_t timer_num, const uint32_t frequency)
{
  switch(timer_num)
  {
    case TEMP_TIMER_NUM:
      CORE_DEBUG_PRINTF("HAL_timer_start: temp timer, f=%ld\n", long(frequency));
      setup_temp_tim(frequency);
      break;
    case STEP_TIMER_NUM:
      CORE_DEBUG_PRINTF("HAL_timer_start: step timer, f=%ld\n", long(frequency));
      setup_step_tim(frequency);
      break;
  }
}

void HAL_timer_enable_interrupt(const timer_channel_t timer_num)
{
  switch (timer_num)
  {
  case STEP_TIMER_NUM:
    timer42_irq_ctrl(Enable);
    break;
  case TEMP_TIMER_NUM:
    break;
  }
}

void HAL_timer_disable_interrupt(const timer_channel_t timer_num)
{
  switch (timer_num)
  {
  case STEP_TIMER_NUM:
    timer42_irq_ctrl(Disable);
    break;
  case TEMP_TIMER_NUM:
    break;
  }
}

bool HAL_timer_interrupt_enabled(const timer_channel_t timer_num)
{
  bool state;
  switch (timer_num)
  {
  case STEP_TIMER_NUM:
    state = timer42_irq_get();
    break;
  case TEMP_TIMER_NUM:
    break;
  }
  return state;
}

void HAL_timer_set_compare(const timer_channel_t timer_num, const hal_timer_t compare)
{
    switch (timer_num) {
        case STEP_TIMER_NUM:
            timer_set_compare(timer_num, compare);
            break;

        case TEMP_TIMER_NUM:

            break;
    }
}

hal_timer_t HAL_timer_get_count(const timer_channel_t timer_num)
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

void HAL_timer_isr_prologue(const timer_channel_t timer_num)
{
  TIMER0_ClearFlag(M4_TMR02, (en_tim0_channel_t)timer_num);
}

void HAL_timer_isr_epilogue(const timer_channel_t timer_num)
{
    if(timer_num == TEMP_TIMER_NUM) {
        TIMER4_CNT_ClearIrqFlag(M4_TMR41, Timer4CntZeroMatchInt);
    } else if(timer_num == STEP_TIMER_NUM) {
        TIMER4_CNT_ClearIrqFlag(M4_TMR42, Timer4CntZeroMatchInt);
    }
}
