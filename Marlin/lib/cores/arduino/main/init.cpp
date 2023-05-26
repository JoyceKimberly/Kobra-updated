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
void flash_init(void)
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

void get_all_clock(void)
{
    stc_clk_freq_t   stcClkFreq;
    stc_pll_clk_freq_t stcPllClkFreq;

	MEM_ZERO_STRUCT(stcClkFreq);
    MEM_ZERO_STRUCT(stcPllClkFreq);

	CLK_GetClockFreq(&stcClkFreq);
	CLK_GetPllClockFreq(&stcPllClkFreq);

	f_cpu_init(stcClkFreq.pclk1Freq);   // used for stepper timer

#if 0
    printf("sysclkFreq: %d\n", stcClkFreq.sysclkFreq);
    printf("hclkFreq:   %d\n", stcClkFreq.hclkFreq);
    printf("exckFreq:   %d\n", stcClkFreq.exckFreq);
    printf("pclk0Freq:  %d\n", stcClkFreq.pclk0Freq);
    printf("pclk1Freq:  %d\n", stcClkFreq.pclk1Freq);
    printf("pclk2Freq:  %d\n", stcClkFreq.pclk2Freq);
    printf("pclk3Freq:  %d\n", stcClkFreq.pclk3Freq);
    printf("pclk4Freq:  %d\n", stcClkFreq.pclk4Freq);

    printf("mpllp:      %d\n", stcPllClkFreq.mpllp);
    printf("mpllq:      %d\n", stcPllClkFreq.mpllq);
    printf("mpllr:      %d\n", stcPllClkFreq.mpllr);
    printf("upllp:      %d\n", stcPllClkFreq.upllp);
    printf("upllq:      %d\n", stcPllClkFreq.upllq);
    printf("upllr:      %d\n", stcPllClkFreq.upllr);
#endif
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

#if 0
    PORT_Init(X_MIN_PORT,   X_MIN_PIN,   &stcPortInit);
    PORT_Init(Y_MIN_PORT,   Y_MIN_PIN,   &stcPortInit);
    PORT_Init(Z_MIN_PORT,   Z_MIN_PIN,   &stcPortInit);
    PORT_Init(E0_MIN_PORT,  E0_MIN_PIN,  &stcPortInit);
    PORT_Init(Z_PROBE_PORT, Z_PROBE_PIN, &stcPortInit);
#endif
}

void stepper_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPullUp  = Enable;

#if 0
    PORT_Init(X_ENABLE_PORT,   X_ENABLE_PIN,  &stcPortInit);
    PORT_SetBits(X_ENABLE_PORT, X_ENABLE_PIN);

    stcPortInit.enPullUp  = Disable;

    PORT_Init(X_STEP_PORT,     X_STEP_PIN,    &stcPortInit);
    PORT_Init(X_DIR_PORT,      X_DIR_PIN,     &stcPortInit);

    PORT_Init(Y_ENABLE_PORT,   Y_ENABLE_PIN,  &stcPortInit);
    PORT_Init(Y_STEP_PORT,     Y_STEP_PIN,    &stcPortInit);
    PORT_Init(Y_DIR_PORT,      Y_DIR_PIN,     &stcPortInit);

    PORT_Init(Z_ENABLE_PORT,   Z_ENABLE_PIN,  &stcPortInit);
    PORT_Init(Z_STEP_PORT,     Z_STEP_PIN,    &stcPortInit);
    PORT_Init(Z_DIR_PORT,      Z_DIR_PIN,     &stcPortInit);

    PORT_Init(E0_ENABLE_PORT,  E0_ENABLE_PIN,  &stcPortInit);
    PORT_Init(E0_STEP_PORT,    E0_STEP_PIN,    &stcPortInit);
    PORT_Init(E0_DIR_PORT,     E0_DIR_PIN,     &stcPortInit);
#endif
}

void heater_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPullUp  = Disable;

#if 0
    PORT_Init(HEATER_BED_PORT, HEATER_BED_PIN, &stcPortInit);
    PORT_Init(HEATER_0_PORT,   HEATER_0_PIN,   &stcPortInit);

    PORT_ResetBits(HEATER_BED_PORT, HEATER_BED_PIN);
    PORT_ResetBits(HEATER_0_PORT,   HEATER_0_PIN);
#endif
}

void fan_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPullUp  = Disable;

// 0x1C swd on ; 0x1F swd off
    PORT_DebugPortSetting(0x1F, Disable);

#if 0
    PORT_SetFunc(FAN_0_PORT, FAN_0_PIN, Func_Gpio, Disable);
    PORT_SetFunc(FAN_1_PORT, FAN_1_PIN, Func_Gpio, Disable);
    PORT_SetFunc(FAN_2_PORT, FAN_2_PIN, Func_Gpio, Disable);

    PORT_Init(FAN_0_PORT, FAN_0_PIN, &stcPortInit);
    PORT_Init(FAN_1_PORT, FAN_1_PIN, &stcPortInit);
    PORT_Init(FAN_2_PORT, FAN_2_PIN, &stcPortInit);

    FAN_0_OFF();
    FAN_1_OFF();
    FAN_2_OFF();
#endif
}
