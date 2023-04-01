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

/**
 * HAL for stm32duino.com based on Libmaple and compatible (STM32F1)
 */

#include <stdint.h>
#include "../board/startup.h"
#include "../../core/boards.h"
#include "../../inc/MarlinConfig.h"
#include "HAL.h"

#include "bsp_timer.h"
#include "hc32f46x_timer0.h"

// ------------------------
// Defines
// ------------------------

/**
 * TODO: Check and confirm what timer we will use for each Temps and stepper driving.
 * We should probable drive temps with PWM.
 */
#define FORCE_INLINE __attribute__((always_inline)) inline

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

#define HAL_TIMER_RATE uint32_t(F_CPU)  // frequency of timers peripherals

/**
 * Note: Timers may be used by platforms and libraries
 *
 * FAN PWMs:
 *   With FAN_SOFT_PWM disabled the Temperature class uses
 *   FANx_PIN timers to generate FAN PWM signals.
 *
 * Speaker:
 *   When SPEAKER is enabled, one timer is allocated by maple/tone.cpp.
 *   - If BEEPER_PIN has a timer channel (and USE_PIN_TIMER is
 *     defined in tone.cpp) it uses the pin's own timer.
 *   - Otherwise it uses Timer 8 on boards with STM32_HIGH_DENSITY
 *     or Timer 4 on other boards.
 */
#define MF_TIMER_STEP       Tim0_ChannelB
#define MF_TIMER_TEMP       Tim0_ChannelA
#define MF_TIMER_PULSE      MF_TIMER_STEP

#define TEMP_TIMER_FREQUENCY    1000 // temperature interrupt frequency
#define TEMP_TIMER_PRESCALE     16ul // prescaler for setting Temp timer, 72Khz

#define STEPPER_TIMER_PRESCALE      16ul                                        // prescaler for setting stepper timer
#define STEPPER_TIMER_RATE          (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)   // frequency of stepper timer
#define STEPPER_TIMER_TICKS_PER_US  ((STEPPER_TIMER_RATE) / 1000000)            // stepper timer ticks per µs

#define PULSE_TIMER_PRESCALE        STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US    STEPPER_TIMER_TICKS_PER_US

#define ENABLE_STEPPER_DRIVER_INTERRUPT() timer_enable_irq(MF_TIMER_STEP,Enable)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() timer_enable_irq(MF_TIMER_STEP,Disable)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(MF_TIMER_STEP)

#define ENABLE_TEMPERATURE_INTERRUPT() timer_enable_irq(MF_TIMER_TEMP,Enable)
#define DISABLE_TEMPERATURE_INTERRUPT() timer_enable_irq(MF_TIMER_TEMP,Disable)

#define HAL_timer_get_count(timer_num) timer_get_count(timer_num)

#define HAL_STEP_TIMER_ISR()      void timer42_zero_match_irq_cb(void)
#define HAL_TEMP_TIMER_ISR()      void timer41_zero_match_irq_cb(void)
#define HAL_TONE_TIMER_ISR()      void Timer01B_CallBack(void)

#define STEP_TIMER_NUM MF_TIMER_STEP
#define TEMP_TIMER_NUM MF_TIMER_TEMP
#define PULSE_TIMER_NUM MF_TIMER_PULSE

// ------------------------
// Public Variables
// ------------------------

//static HardwareTimer StepperTimer(MF_TIMER_STEP);
//static HardwareTimer TempTimer(MF_TIMER_TEMP);

// ------------------------
// Public functions
// ------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_enabled(const uint8_t timer_num);

/**
 * NOTE: By default libmaple sets ARPE = 1, which means the Auto reload register is preloaded (will only update with an update event)
 * Thus we have to pause the timer, update the value, refresh, resume the timer.
 * That seems like a big waste of time and may be better to change the timer config to ARPE = 0, so ARR can be updated any time.
 * We are using a Channel in each timer in Capture/Compare mode. We could also instead use the Time Update Event Interrupt, but need to disable ARPE
 * so we can change the ARR value on the fly (without calling refresh), and not get an interrupt right there because we caused an UEV.
 * This mode pretty much makes 2 timers unusable for PWM since they have their counts updated all the time on ISRs.
 * The way Marlin manages timer interrupts doesn't make for an efficient usage in STM32F1
 * Todo: Look at that possibility later.
 */

void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t compare);


#define HAL_timer_isr_prologue(TIMER_NUM)
//#define HAL_timer_isr_epilogue(TIMER_NUM)   TIMER4_CNT_ClearIrqFlag(M4_TMR42, Timer4CntZeroMatchInt)

static inline void HAL_timer_isr_epilogue(uint8_t timer_num)
{
    if(timer_num == TEMP_TIMER_NUM) {
        TIMER4_CNT_ClearIrqFlag(M4_TMR41, Timer4CntZeroMatchInt);
    } else if(timer_num == STEP_TIMER_NUM) {
        TIMER4_CNT_ClearIrqFlag(M4_TMR42, Timer4CntZeroMatchInt);
    }
}

#define TIMER_OC_NO_PRELOAD 0 // Need to disable preload also on compare registers.

