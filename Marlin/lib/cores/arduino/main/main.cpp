#include "hc32_ddl.h"
#include "../Arduino.h"
#include "init.h"
#include "../core_debug.h"
#include "../core_hooks.h"
#include "usart.h"
#include "adc.h"
#include "bsp_pwm.h"
#include "bsp_sdio.h"
#include "bsp_timer.h"
#include "ots.h"

#include "diskio.h"

#include "marlincore.h"


#define APP_START_ADDRESS   0x8000u


void soft_delay_ms(uint32_t ms)
{
    for(uint32_t i=0; i<ms; i++) {
        for(uint32_t j=0; j<8192; j++) {

#if   defined ( __CC_ARM )
            __nop();
#elif defined ( __GNUC__ )
        	asm("NOP");
#endif

        }
    }
}

void core_init(void)
{
// bootloader vector startup addr
    SCB->VTOR = ((uint32_t) APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);
}

int main(void)
{
	// initialize SoC, then CORE_DEBUG
	core_init();
	CORE_DEBUG_INIT();

	// call setup()
	core_hook_pre_setup();
	CORE_DEBUG_PRINTF("core entering setup\n");

    PWC_HS2HP();

    flash_init();

    uart1_init();
    uart2_init();
    uart4_init();

    H32OTS::init();

    get_all_clock();

    led_pin_init();

    adc_init();

    endstop_pin_init();

    stepper_pin_init();

    heater_pin_init();

// 0x1C swd on ; 0x1F swd off
    PORT_DebugPortSetting(0x1F, Disable);

    fan_pwm_init();
    beep_pwm_init();

    hal_sdio_init();

//    disk_initialize(0);

    timer02A_init();     // 1k Hz, millis()
    timer02B_init();     // soft serial
    timer41_init();      // 1k Hz, used for temperature tick
    timer42_init();      // step motor
    timer01B_init();     // used for beep duration timer

//SysTick configuration
    SysTick_Init(1000u);

	setup();
	core_hook_post_setup();
	
	// call loop() forever
	CORE_DEBUG_PRINTF("core entering main loop\n");
	while (1)
	{
		core_hook_loop();
		loop();
	}
	return 0;
}
