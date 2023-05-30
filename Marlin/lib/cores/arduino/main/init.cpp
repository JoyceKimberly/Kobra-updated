#include "init.h"
#include "../drivers/sysclock/sysclock.h"
#include "../drivers/sysclock/systick.h"
#include "../drivers/adc/adc.h"
#include "../WInterrupts.h"
#include "startup.h"
#include <hc32_ddl.h>

/**
 * set flash latency and cache
 */
inline void flash_init()
{
    stc_clk_xtal_cfg_t   stcXtalCfg;
    stc_clk_mpll_cfg_t   stcMpllCfg;
    en_clk_sys_source_t  enSysClkSrc;
    stc_clk_sysclk_cfg_t stcSysClkCfg;

    MEM_ZERO_STRUCT(enSysClkSrc);
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv  = ClkSysclkDiv1;
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv4;
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv16;
    CLK_SysClkConfig(&stcSysClkCfg);

    CLK_SetPeriClkSource(ClkPeriSrcPclk);

    /* Switch system clock source to MPLL. */
    /* Use Xtal as MPLL source. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_4);
    EFM_Lock();

    /* Switch driver ability */
    PWC_HS2HP();

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 1u; /* XTAL 8M / 1 */
    stcMpllCfg.plln = 42u;   /* 8M*42 = 336M */
    stcMpllCfg.PllpDiv = 2u; /* MLLP = 168M */
    stcMpllCfg.PllqDiv = 2u; /* MLLQ = 168M */
    stcMpllCfg.PllrDiv = 2u; /* MLLR = 168M */
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while (Set != CLK_GetFlagStatus(ClkFlagMPLLRdy))
    {
    }

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}

#define LD_FLASH_START 0x8000u

void core_init()
{
    // setup VTO register
    SCB->VTOR = (uint32_t(LD_FLASH_START) & SCB_VTOR_TBLOFF_Msk);

    // setup the SoC and initialize drivers
    flash_init();
    update_system_clock_frequencies();
    adc_init();
}

uint32_t F_CPU;

void f_cpu_init(uint32_t clock)
{
    F_CPU = clock;
}

void get_all_clock(void)
{
    stc_clk_freq_t   stcClkFreq;
    stc_pll_clk_freq_t stcPllClkFreq;

  	MEM_ZERO_STRUCT(stcClkFreq);
    MEM_ZERO_STRUCT(stcPllClkFreq);

	  CLK_GetClockFreq(&stcClkFreq);
	  CLK_GetPllClockFreq(&stcPllClkFreq);

	  f_cpu_init(stcClkFreq.pclk1Freq);   // used for stepper timer
}

void led_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPullUp  = Disable;

    PORT_Init(PortA, Pin01, &stcPortInit);
    PORT_Init(PortA, Pin04, &stcPortInit);

    LED0_OFF();
}

void endstop_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_In;
    stcPortInit.enPullUp  = Disable;
}

void stepper_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPullUp  = Enable;
}

void heater_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPullUp  = Disable;
}

void fan_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPullUp  = Disable;

// 0x1C swd on ; 0x1F swd off
    PORT_DebugPortSetting(0x1F, Disable);
}
