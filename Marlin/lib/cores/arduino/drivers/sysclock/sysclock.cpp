#include "sysclock.h"
#include <hc32_ddl.h>

system_clock_frequencies_t SYSTEM_CLOCK_FREQUENCIES = {0};

//__attribute__((weak))
void sysclock_init(void)
{
    en_clk_sys_source_t  enSysClkSrc;
    MEM_ZERO_STRUCT(enSysClkSrc);

    // setup divisors for the different bus clocks
    stc_clk_sysclk_cfg_t sysClkConf;
    MEM_ZERO_STRUCT(sysClkConf);
    sysClkConf.enHclkDiv  = ClkSysclkDiv1;
    sysClkConf.enExclkDiv = ClkSysclkDiv4;
    sysClkConf.enPclk0Div = ClkSysclkDiv1;
    sysClkConf.enPclk1Div = ClkSysclkDiv2;
    sysClkConf.enPclk2Div = ClkSysclkDiv4;
    sysClkConf.enPclk3Div = ClkSysclkDiv4;
    sysClkConf.enPclk4Div = ClkSysclkDiv16;
    CLK_SysClkConfig(&sysClkConf);

    CLK_SetPeriClkSource(ClkPeriSrcPclk);

    // configure and enable XTAL clock source
    stc_clk_xtal_cfg_t   xtalConf;
    MEM_ZERO_STRUCT(xtalConf);
    xtalConf.enFastStartup = Enable;
    xtalConf.enMode = ClkXtalModeOsc;
    xtalConf.enDrv = ClkXtalLowDrv;
    CLK_XtalConfig(&xtalConf);
    CLK_XtalCmd(Enable);

    // configure PLL using XTAL clock as source
    stc_clk_mpll_cfg_t   pllConf;
    MEM_ZERO_STRUCT(pllConf);
    pllConf.PllpDiv = 2u; /* MLLP = 168M */
    pllConf.PllqDiv = 2u; /* MLLQ = 168M */
    pllConf.PllrDiv = 2u; /* MLLR = 168M */
    pllConf.plln = 42u;   /* 8M*42 = 336M */
    pllConf.pllmDiv = 1u; /* XTAL 8M / 1 */
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&pllConf);

    // enable MPLL and wait until ready
    CLK_MpllCmd(Enable);
    while (Set != CLK_GetFlagStatus(ClkFlagMPLLRdy))
    {
    }

    // switch the system clock to MPLL
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}

void update_system_clock_frequencies()
{
    stc_clk_freq_t clkFreq;
    CLK_GetClockFreq(&clkFreq);
    SYSTEM_CLOCK_FREQUENCIES.system = clkFreq.sysclkFreq;
    SYSTEM_CLOCK_FREQUENCIES.hclk = clkFreq.hclkFreq;
    SYSTEM_CLOCK_FREQUENCIES.pclk0 = clkFreq.pclk0Freq;
    SYSTEM_CLOCK_FREQUENCIES.pclk1 = clkFreq.pclk1Freq;
    SYSTEM_CLOCK_FREQUENCIES.pclk2 = clkFreq.pclk2Freq;
    SYSTEM_CLOCK_FREQUENCIES.pclk3 = clkFreq.pclk3Freq;
    SYSTEM_CLOCK_FREQUENCIES.pclk4 = clkFreq.pclk4Freq;
    SYSTEM_CLOCK_FREQUENCIES.exclk = clkFreq.exckFreq;
}
