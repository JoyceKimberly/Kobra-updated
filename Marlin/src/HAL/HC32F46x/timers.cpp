/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#include "../platforms.h"

#ifdef TARGET_HC32F46x

#include "../../inc/MarlinConfig.h"
#include "HAL.h"
#include "timers.h"
#include "hc32f460_clk.h"
#include "hc32f460_pwc.h"
#include "hc32f460_timer0.h"
#include "hc32f460_interrupts.h"

#define TMR_UNIT M4_TMR02

//
// HAL
//

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency)
{
  switch (timer_num)
  {
  case MF_TIMER_STEP:
    setup_step_tim(frequency);
    break;
  case MF_TIMER_TEMP:
    setup_temp_tim(frequency);
    break;
  }
}

void HAL_timer_enable_interrupt(const uint8_t timer_num)
{
  switch (timer_num)
  {
  case MF_TIMER_STEP:
    ENABLE_STEPPER_DRIVER_INTERRUPT();
    break;
  case MF_TIMER_TEMP:
    ENABLE_TEMPERATURE_INTERRUPT();
    break;
  }
}

void HAL_timer_disable_interrupt(const uint8_t timer_num)
{
  switch (timer_num)
  {
  case MF_TIMER_STEP:
    DISABLE_STEPPER_DRIVER_INTERRUPT();
    break;
  case MF_TIMER_TEMP:
    DISABLE_TEMPERATURE_INTERRUPT();
    break;
  }
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num)
{
  return timer_irq_enabled(TMR_UNIT,timer_num);
}

static hal_timer_t compare_hal = 0x0;

void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t compare)
{
//    if(compare_hal != compare){
//        compare_hal = compare;
//        en_functional_state_t state_t = (timer_num==STEP_TIMER_NUM)?Disable:Enable;
//        timer_set_compare(timer_num, compare, state_t);
//    }

    switch (timer_num) {
        case STEP_TIMER_NUM:
            timer_set_compare(timer_num, compare);
            break;

        case TEMP_TIMER_NUM:

            break;
    }
}

#endif // TARGET_HC32F46x
