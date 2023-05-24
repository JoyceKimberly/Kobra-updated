/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
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

#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"
#include "HAL.h"
#include "hc32_ddl.h"

extern "C" char *_sbrk(int incr);

uint16_t MarlinHAL::adc_result;

MarlinHAL::MarlinHAL() {}

// pclk = system_clock/div4 = 50M
// max cycle = 65536
// max feed interval = 65536 / (50000000/8192) = 10.7s
void MarlinHAL::watchdog_init()
{
#if ENABLED(USE_WATCHDOG)
    stc_wdt_init_t wdtConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(wdtConf);

    wdtConf.enCountCycle = WdtCountCycle65536;
    wdtConf.enClkDiv = WdtPclk3Div8192;
    wdtConf.enRefreshRange = WdtRefresh100Pct;
    wdtConf.enSleepModeCountEn = Disable;
    wdtConf.enRequestType = WdtTriggerResetRequest;
    WDT_Init(&wdtConf);
    WDT_RefreshCounter();
#endif
}

void MarlinHAL::watchdog_refresh()
{
#if ENABLED(USE_WATCHDOG)
    #if DISABLED(PINS_DEBUGGING) && PIN_EXISTS(LED)
      TOGGLE(LED_PIN);  // heartbeat indicator
    #endif

    en_result_t enRet = Error;
    enRet = WDT_RefreshCounter();

    if(enRet != Ok) {
        printf("Failed at function: %s, line: %d\n", __FUNCTION__, __LINE__);
    }
#endif
}

void MarlinHAL::init()
{
    // Ensure F_CPU is a constant expression.
    // If the compiler breaks here, it means that delay code that should compute at compile time will not work.
    // So better safe than sorry here.
    constexpr int cpuFreq = F_CPU;
    UNUSED(cpuFreq);

    NVIC_SetPriorityGrouping(0x3);

    #if HAS_MEDIA && DISABLED(ONBOARD_SDIO) && (defined(SDSS) && SDSS != -1)
      OUT_WRITE(SDSS, HIGH); // Try to set SDSS inactive before any other SPI users start up
    #endif

    #if PIN_EXISTS(LED)
      OUT_WRITE(LED_PIN, LOW);
    #endif

    #if PIN_EXISTS(AUTO_LEVEL_TX)
      OUT_WRITE(AUTO_LEVEL_TX_PIN, HIGH);
      delay(10);
      OUT_WRITE(AUTO_LEVEL_TX_PIN, LOW);
      delay(300);
      OUT_WRITE(AUTO_LEVEL_TX_PIN, HIGH);
    #endif

    #if ENABLED(SRAM_EEPROM_EMULATION)
      __HAL_RCC_PWR_CLK_ENABLE();
      HAL_PWR_EnableBkUpAccess();           // Enable access to backup SRAM
      __HAL_RCC_BKPSRAM_CLK_ENABLE();
      LL_PWR_EnableBkUpRegulator();         // Enable backup regulator
      while (!LL_PWR_IsActiveFlag_BRR());   // Wait until backup regulator is initialized
    #endif
}

void MarlinHAL::init_board() {}

void MarlinHAL::reboot()
{
    NVIC_SystemReset();
}

bool MarlinHAL::isr_state()
{
    return !__get_PRIMASK();
}

void MarlinHAL::isr_on()
{
    __enable_irq();
}

void MarlinHAL::isr_off()
{
    __disable_irq();
}

void MarlinHAL::delay_ms(const int ms)
{
    delay(ms);
}

void MarlinHAL::idletask()
{
    MarlinHAL::watchdog_refresh();
}

uint8_t MarlinHAL::get_reset_source()
{
    // query reset cause
    stc_rmu_rstcause_t rstCause;
    MEM_ZERO_STRUCT(rstCause);
    RMU_GetResetCause(&rstCause);

    // map reset causes to those expected by Marlin
    uint8_t cause;

    typedef enum {
        RST_CAU_POWER_ON    = 0x01,
        RST_CAU_EXTERNAL    = 0x02,
        RST_CAU_BROWN_OUT   = 0x04,
        RST_CAU_WATCHDOG    = 0x08,
        RST_CAU_JTAG        = 0x10,
        RST_CAU_SOFTWARE    = 0x20,
        RST_CAU_BACKUP      = 0x40,
    } rst_cause_t;

    if(Set == rstCause.enSoftware) {
        cause = RST_CAU_SOFTWARE;
    } else if(Set == rstCause.enWdt) {
        cause = RST_CAU_WATCHDOG;
    } else if(Set == rstCause.enRstPin) {
        cause = RST_CAU_EXTERNAL;
    }

    return cause;
}

void MarlinHAL::clear_reset_source()
{
    RMU_ClrResetFlag();
}

int MarlinHAL::freeMemory()
{
    volatile char top;
    return top;
}

void MarlinHAL::adc_init() {}

void MarlinHAL::adc_enable(const pin_t pin)
{
    pinMode(pin, INPUT);
}

void MarlinHAL::adc_start(const pin_t pin)
{
    if       (pin == TEMP_BED_PIN) {
        g_adc_idx = 0;
    } else if(pin == TEMP_0_PIN) {
        g_adc_idx = 1;
    } else if(pin == POWER_MONITOR_VOLTAGE_PIN) {
        g_adc_idx = 2;
    } else {
        g_adc_idx = 0x0;
    }

    MarlinHAL::adc_result = g_adc_value[g_adc_idx];
}

bool MarlinHAL::adc_ready()
{
    return true;
}

uint16_t MarlinHAL::adc_value()
{
    return g_adc_value[g_adc_idx];
}

void MarlinHAL::set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t a, const bool b)
{
    // TODO stub
}

void MarlinHAL::set_pwm_frequency(const pin_t pin, const uint16_t f_desired)
{
    // TODO stub
}

void flashFirmware(const int16_t) { MarlinHAL::reboot(); }

