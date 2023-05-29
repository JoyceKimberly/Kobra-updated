/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2017 Victor Perez
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
#pragma once
#include <stdint.h>
#include <startup.h>
#include "../../core/boards.h"
#include "../../inc/MarlinConfig.h"
#include "HAL.h"
#include "bsp_timer.h"
#include "hc32f460_timer0.h"

// ------------------------
// Defines
// ------------------------

/**
 * TODO: Check and confirm what timer we will use for each Temps and stepper driving.
 * We should probable drive temps with PWM.
 */
#define FORCE_INLINE __attribute__((always_inline)) inline

//
// Misc.
//
typedef en_tim0_channel_t timer_channel_t;
// STM32 timers may be 16 or 32 bit. Limiting HAL_TIMER_TYPE_MAX to 16 bits
// avoids issues with STM32F0 MCUs, which seem to pause timers if UINT32_MAX
// is written to the register. STM32F4 timers do not manifest this issue,
// even when writing to 16 bit timers.
//
// The range of the timer can be queried at runtime using IS_TIM_32B_COUNTER_INSTANCE.
// This is a more expensive check than a simple compile-time constant, so its
// implementation is deferred until the desire for a 32-bit range outweighs the cost
// of adding a run-time check and HAL_TIMER_TYPE_MAX is refactored to allow unique
// values for each timer.
#define hal_timer_t uint32_t
#define HAL_TIMER_TYPE_MAX UINT16_MAX

// frequency of the timer peripheral
#define HAL_TIMER_RATE uint32_t(F_CPU)

//
// Timer Channels and Configuration
//
#define MF_TIMER_STEP       Tim0_ChannelB
#define MF_TIMER_TEMP       Tim0_ChannelA
#define MF_TIMER_PULSE      MF_TIMER_STEP

// channel aliases
#define STEP_TIMER_NUM MF_TIMER_STEP
#define TEMP_TIMER_NUM MF_TIMER_TEMP
#define PULSE_TIMER_NUM MF_TIMER_PULSE

#define TEMP_TIMER_FREQUENCY 1000
#define TEMP_TIMER_PRESCALE 16ul

#define STEPPER_TIMER_PRESCALE 16ul

#define STEPPER_TIMER_RATE (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000)

#define PULSE_TIMER_PRESCALE STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US STEPPER_TIMER_TICKS_PER_US

//
// HAL functions
//
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_enabled(const uint8_t timer_num);
void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t compare);
#define HAL_timer_isr_prologue(TIMER_NUM)

static inline void HAL_timer_isr_epilogue(uint8_t timer_num)
{
    if(timer_num == TEMP_TIMER_NUM) {
        TIMER4_CNT_ClearIrqFlag(M4_TMR41, Timer4CntZeroMatchInt);
    } else if(timer_num == STEP_TIMER_NUM) {
        TIMER4_CNT_ClearIrqFlag(M4_TMR42, Timer4CntZeroMatchInt);
    }
}

#define TIMER_OC_NO_PRELOAD 0 // Need to disable preload also on compare registers.

//
// HAL function aliases
//
#define ENABLE_STEPPER_DRIVER_INTERRUPT() timer_enable_irq(MF_TIMER_STEP,Enable)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() timer_enable_irq(MF_TIMER_STEP,Disable)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(MF_TIMER_STEP)

#define ENABLE_TEMPERATURE_INTERRUPT() timer_enable_irq(MF_TIMER_TEMP,Enable)
#define DISABLE_TEMPERATURE_INTERRUPT() timer_enable_irq(MF_TIMER_TEMP,Disable)

#define HAL_timer_get_count(timer_num) timer_get_count(timer_num)

#define HAL_STEP_TIMER_ISR()      void timer42_zero_match_irq_cb(void)
#define HAL_TEMP_TIMER_ISR()      void timer41_zero_match_irq_cb(void)
#define HAL_TONE_TIMER_ISR()      void Timer01B_CallBack(void)
