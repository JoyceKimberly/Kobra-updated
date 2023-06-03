#include "adc.h"
#include "../../yield.h"
#include "../../core_debug.h"
#include "../gpio/gpio.h"
#include "../irqn/irqn.h"
#include "usart.h"

uint16_t g_adc_value[3];
uint8_t g_adc_idx;

// ADC irq flag bit mask
#define ADC1_SA_DMA_IRQ_BIT (1ul << 0u)

/**
 * @brief assert that channel id is valid
 * @param device ADC device configuration
 * @param channel ADC channel id to check
 */
#define ASSERT_CHANNEL_ID(device, channel) \
    CORE_ASSERT(channel >= 0 && channel < device->adc.channel_count, "invalid channel id")

/**
 * @brief assert that adc device is initialized
 */
#define ASSERT_INITIALIZED(device, function_name) \
    CORE_ASSERT(device->state.initialized, "ADC device not initialized (calling " function_name ")")

/**
 * @brief adc register to debug name
 */
#define ADC_REG_TO_NAME(reg)  \
    reg == M4_ADC1   ? "ADC1" \
    : reg == M4_ADC2 ? "ADC2" \
                     : "N/A"

/**
 * @brief debug printf for ADC
 */
#define ADC_DEBUG_PRINTF(device, fmt, ...) \
    CORE_DEBUG_PRINTF("[%s] " fmt, ADC_REG_TO_NAME(device->adc.register_base), ##__VA_ARGS__)

//
// ADC init
//

/**
 * @brief ADC peripheral init
 */
inline void adc_adc_init(const adc_device_t *device)
{
    // enable ADC peripheral clock
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);

    // initialize ADC peripheral
    stc_adc_init_t init_device;
    MEM_ZERO_STRUCT(init_device);
    init_device.enResolution = AdcResolution_12Bit;
    init_device.enDataAlign  = AdcDataAlign_Right;
    init_device.enAutoClear  = AdcClren_Enable;
    init_device.enScanMode   = AdcMode_SAContinuous;
    init_device.enRschsel    = AdcRschsel_Restart;
    /* 2. Initialize ADC1. */
    ADC_Init(M4_ADC1, &init_device);

}

/**
 * @brief ADC DMA transfer init
 */
inline void adc_dma_init(const adc_device_t *device)
{
    // prepare DMA transfer deviceuration to
    // transfer ADCx->DR0-DRn to state.conversion_results
    stc_dma_config_t dma_device;
    MEM_ZERO_STRUCT(dma_device);
    dma_device.u16BlockSize   = 3;
    dma_device.u16TransferCnt = 0u;
    dma_device.u32SrcAddr     = (uint32_t)(&M4_ADC1->DR10);
    dma_device.u32DesAddr     = (uint32_t)(&g_adc_value[0]);
    dma_device.u16SrcRptSize  = 3;
    dma_device.u16DesRptSize  = 3;
    dma_device.u32DmaLlp      = 0u;
    dma_device.stcSrcNseqCfg.u32Offset = 0u;
    dma_device.stcSrcNseqCfg.u16Cnt    = 0u;
    dma_device.stcDesNseqCfg.u32Offset = 0u;
    dma_device.stcDesNseqCfg.u16Cnt    = 0u;
    dma_device.stcDmaChCfg.enSrcInc    = AddressIncrease;
    dma_device.stcDmaChCfg.enDesInc    = AddressIncrease;
    dma_device.stcDmaChCfg.enSrcRptEn  = Enable;
    dma_device.stcDmaChCfg.enDesRptEn  = Enable;
    dma_device.stcDmaChCfg.enSrcNseqEn = Disable;
    dma_device.stcDmaChCfg.enDesNseqEn = Disable;
    dma_device.stcDmaChCfg.enTrnWidth  = Dma16Bit;
    dma_device.stcDmaChCfg.enLlpEn     = Disable;
    dma_device.stcDmaChCfg.enIntEn     = Disable;

    // enable DMA peripheral clock
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2, Enable);

    // initialize DMA channel and enable
    DMA_InitChannel(M4_DMA2, DmaCh3, &dma_device);
    DMA_Cmd(M4_DMA2, Enable);
    DMA_ChannelCmd(M4_DMA2, DmaCh3, Enable);

    // clear DMA transfer complete flag
    DMA_ClearIrqFlag(M4_DMA2, DmaCh3, TrnCpltIrq);

    // AOS is required to trigger DMA transfer, enable AOS peripheral clock
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    // automatically start DMA transfer when ADC conversion is complete
    DMA_SetTriggerSrc(M4_DMA2, DmaCh3, EVT_ADC1_EOCA);
}




//
// ADC Channel API
//

/**
 *******************************************************************************
 ** \brief  Config the pin which is mapping the channel to analog or digit mode.
 **
 ******************************************************************************/
void adc_setChannelPinMode(const M4_ADC_TypeDef *ADCx, uint32_t channel, en_pin_mode_t mode)
{
    // get channel offset and mask
    uint8_t channelOffset = 0u;
    if (M4_ADC1 == ADCx)
    {
        channel &= ADC1_PIN_MASK_ALL;
    }
    else
    {
        channel &= ADC2_PIN_MASK_ALL;
        channelOffset = 4u;
    }

    // set pin mode of all pins in the channel
    for (uint8_t i = 0u; channel != 0u; i++)
    {
        if (channel & 0x1ul)
        {
            adc_setPinMode((channelOffset + i), mode);
        }

        channel >>= 1u;
    }
}

void adc_channelConfig(adc_device_t *device, en_pin_mode_t mode)
{
    uint8_t samplingTimes[3] = { 0x60, 0x60, 0x60 };

    stc_port_init_t portConf;
    MEM_ZERO_STRUCT(portConf);

    portConf.enPullUp  = Disable;
    portConf.enPinMode = Pin_Mode_Ana;

    PORT_Init(BOARD_ADC_CH0_PORT, BOARD_ADC_CH0_PIN, &portConf);
    PORT_Init(BOARD_ADC_CH1_PORT, BOARD_ADC_CH1_PIN, &portConf);
    PORT_Init(BOARD_ADC_CH2_PORT, BOARD_ADC_CH2_PIN, &portConf);

    // init adc channel
    stc_adc_ch_cfg_t channel_config;
    MEM_ZERO_STRUCT(channel_config);
    channel_config.u32Channel  = ADC1_CH10 | ADC1_CH11 | ADC1_CH12;
    channel_config.u8Sequence  = ADC_SEQ_A;
    channel_config.pu8SampTime = samplingTimes;
    ADC_AddAdcChannel(M4_ADC1, &channel_config);
//    ADC_ConfigAvg(M4_ADC1, AdcAvcnt_64);
//    ADC_AddAvgChannel(M4_ADC1, ADC1_CH10 | ADC1_CH11 | ADC1_CH12);
}

uint16_t adc_read_sync(adc_device_t *device, uint8_t channel)
{
    // wait for adc result
    while (true)
    {
        if (device->HAL_AdcDmaIrqFlag & ADC1_SA_DMA_IRQ_BIT)
        {
            break;
        }

        uint8_t c = 0;
        if (++c >= 100)
        {
            c = 0;
            return 0;
        }

        WDT_RefreshCounter();
    }

    // read result and clear irq flag
    device->HAL_AdcDmaIrqFlag &= ~ADC1_SA_DMA_IRQ_BIT;
    return device->HAL_adc_results[channel];
}

/**
 *******************************************************************************
 ** \brief  ADC trigger source configuration.
 **
 ******************************************************************************/
void adc_triggerConfig(adc_device_t *device, uint32_t fcg0Periph)
{
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    stc_adc_trg_cfg_t stcTrgCfg;
    MEM_ZERO_STRUCT(stcTrgCfg);

    /*
    // select EVT_TMR02_GCMA as ADC1 trigger source
    stc_adc_trg_cfg_t triggerConf = {
        .u8Sequence = ADC_SEQ_A,
        .enTrgSel = AdcTrgsel_TRGX0,
        .enInTrg0 = EVT_TMR02_GCMA,
    };
    ADC_ConfigTriggerSrc(device->regs, &triggerConf);
    ADC_TriggerSrcCmd(device->regs, ADC_SEQ_A, Enable);
    */

    // ADC1 is always triggered by software
    ADC_TriggerSrcCmd(M4_ADC1, ADC_SEQ_A, Disable);
}

/**
 *******************************************************************************
 ** \brief  ADC clock configuration.
 **
 ** \note   1) ADCLK max frequency is 60MHz.
 **         2) If PCLK2 and PCLK4 are selected as the ADC clock,
 **            the following conditions must be met:
 **            a. ADCLK(PCLK2) max 60MHz;
 **            b. PCLK4 : ADCLK = 1:1, 2:1, 4:1, 8:1, 1:2, 1:4
 **
 ******************************************************************************/
void adc_setCLK()
{
#if (ADC_CLK == ADC_CLK_PCLK)
    stc_clk_sysclk_cfg_t sysClkConf;

    /* Set bus clock division, depends on the system clock frequency. */
    sysClkConf.enHclkDiv  = ClkSysclkDiv1;  // 168MHz
    sysClkConf.enExclkDiv = ClkSysclkDiv2;  // 84MHz
    sysClkConf.enPclk0Div = ClkSysclkDiv1;  // 168MHz
    sysClkConf.enPclk1Div = ClkSysclkDiv2;  // 84MHz
    sysClkConf.enPclk2Div = ClkSysclkDiv4;  // 42MHz
    sysClkConf.enPclk3Div = ClkSysclkDiv4;  // 42MHz
    sysClkConf.enPclk4Div = ClkSysclkDiv1;  // 84MHz.
    CLK_SysClkConfig(&sysClkConf);
    CLK_SetPeriClkSource(ClkPeriSrcPclk);
#elif (ADC_CLK == ADC_CLK_MPLLQ)
    stc_clk_xtal_cfg_t xtalConf;
    stc_clk_mpll_cfg_t pllConf;

    if (CLKSysSrcMPLL == CLK_GetSysClkSource())
    {
        /*
         * Configure MPLLQ(same as MPLLP and MPLLR) when you
         * configure MPLL as the system clock.
         */
    }
    else
    {
        /* Use XTAL as MPLL source. */
        xtalConf.enFastStartup = Enable;
        xtalConf.mode = ClkXtalModeOsc;
        xtalConf.enDrv  = ClkXtalLowDrv;
        CLK_XtalConfig(&xtalConf);
        CLK_XtalCmd(Enable);

        /* Set MPLL out 240MHz. */
        pllConf.pllmDiv = 1u;
        /* mpll = 8M / pllmDiv * plln */
        pllConf.PllpDiv = 16u;
        pllConf.PllqDiv = 16u;
        pllConf.PllrDiv = 16u;
        pllConf.plln    = 30u;

        CLK_SetPllSource(ClkPllSrcXTAL);
        CLK_MpllConfig(&pllConf);

        CLK_MpllCmd(Enable);
    }
    CLK_SetPeriClkSource(ClkPeriSrcMpllp);

#elif (ADC_CLK == ADC_CLK_UPLLR)
    stc_clk_xtal_cfg_t xtalConf;
    stc_clk_upll_cfg_t upllConf;

    MEM_ZERO_STRUCT(xtalConf);
    MEM_ZERO_STRUCT(upllConf);

    // Use XTAL as UPLL source
    xtalConf.enFastStartup = Enable;
    xtalConf.mode = ClkXtalModeOsc;
    xtalConf.enDrv  = ClkXtalLowDrv;
    CLK_XtalConfig(&xtalConf);
    CLK_XtalCmd(Enable);

        // upll = 8M(XTAL) / pllmDiv * plln
    upllConf.PllpDiv = 16u;
    upllConf.PllqDiv = 16u;
    upllConf.PllrDiv = 16u;
    upllConf.plln    = 60u;

        // Set UPLL out 240MHz
    upllConf.pllmDiv = 2u;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_UpllConfig(&upllConf);
    CLK_UpllCmd(Enable);
    CLK_SetPeriClkSource(ClkPeriSrcUpllr);
#endif
}

/**
 *******************************************************************************
 ** \brief  Set an ADC pin as analog input mode or digit mode.
 **
 ******************************************************************************/
void adc_setPinMode(uint8_t adcPin, en_pin_mode_t mode)
{
    // translate adc input to pin
    uint8_t pin;
    switch (adcPin)
    {
    case ADC1_IN0:
        pin = PA0;
        break;
    case ADC1_IN1:
        pin = PA1;
        break;
    case ADC1_IN2:
        pin = PA2;
        break;
    case ADC1_IN3:
        pin = PA3;
        break;
    case ADC12_IN4:
        pin = PA4;
        break;
    case ADC12_IN5:
        pin = PA5;
        break;
    case ADC12_IN6:
        pin = PA6;
        break;
    case ADC12_IN7:
        pin = PA7;
        break;
    case ADC12_IN8:
        pin = PB0;
        break;
    case ADC12_IN9:
        pin = PB1;
        break;
    case ADC12_IN10:
        pin = PC0;
        break;
    case ADC12_IN11:
        pin = PC1;
        break;
    case ADC1_IN12:
        pin = PC2;
        break;
    case ADC1_IN13:
        pin = PC3;
        break;
    case ADC1_IN14:
        pin = PC4;
        break;
    case ADC1_IN15:
        pin = PC5;
        break;
    default:
        return;
    }

    // set pin mode
    stc_port_init_t portConf = {
        .enPinMode = mode,
        .enPullUp = Disable,
    };
    GPIO_Init(pin, &portConf);
}
void adc_setDefaultConfig(adc_device_t *device)
{
    // init and config adc and channels
    adc_adc_init(device);
    adc_channelConfig(device, Pin_Mode_Ana);
    adc_triggerConfig(device, PWC_FCG0_PERIPH_AOS);

    // init and config DMA
    adc_dma_init(device);

    ADC_StartConvert(device->regs);
}

void adc_init(void)
{
    // set ADC clock (default is MRC @ 8MHz)
    adc_setCLK();

    // configure ADC
    adc_setDefaultConfig(ADC1);
}
