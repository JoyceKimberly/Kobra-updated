#include "../Arduino.h"
#include "init.h"
#include "../core_debug.h"
#include "../core_hooks.h"
#include "hc32_ddl.h"
#include "bsp_pwm.h"
#include "bsp_sdio.h"
#include "bsp_timer.h"
#include "delay.h"
#include "ots.h"
#include "diskio.h"
#include "marlincore.h"

int main(void)
{
	// initialize SoC, then CORE_DEBUG
	core_init();
	CORE_DEBUG_INIT();

	// call setup()
	core_hook_pre_setup();
	CORE_DEBUG_PRINTF("core entering setup\n");

  H32OTS::init();
  get_all_clock();
  endstop_pin_init();
  stepper_pin_init();
  heater_pin_init();

  // 0x1C swd on ; 0x1F swd off
  PORT_DebugPortSetting(0x1F, Disable);

  fan_pwm_init();
  beep_pwm_init();
  hal_sdio_init();

  // disk_initialize(0);

  timer01B_init();     // used for beep duration timer
  timer02A_init();     // 1k Hz, millis()
  timer02B_init();     // soft serial
  timer41_init();      // 1k Hz, used for temperature tick
  timer42_init();      // step motor

  // SysTick configuration
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
