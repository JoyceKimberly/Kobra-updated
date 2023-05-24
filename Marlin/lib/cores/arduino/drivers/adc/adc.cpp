#include "adc.h"
#include "usart.h"

#define ADC_CH_COUNT	3

uint16_t g_adc_value[3];
uint8_t g_adc_idx;

uint16_t adc_read(adc_dev *dev, uint8_t channel)
{


}

/**
 *******************************************************************************
 ** \brief  ADC clock configuration.
 **
 ** \note   1) ADCLK max frequency is 60MHz.
 **		 2) If PCLK2 and PCLK4 are selected as the ADC clock,
 **			the following conditions must be met:
 **			a. ADCLK(PCLK2) max 60MHz;
 **			b. PCLK4 : ADCLK = 1:1, 2:1, 4:1, 8:1, 1:2, 1:4
 **
 ******************************************************************************/
static void adc_setCLK(void)
{
#if (ADC_CLK == ADC_CLK_PCLK)
	stc_clk_sysclk_cfg_t stcSysclkCfg;

	/* Set bus clock division, depends on the system clock frequency. */
	stcSysclkCfg.enHclkDiv  = ClkSysclkDiv1;  // 168MHz
	stcSysclkCfg.enExclkDiv = ClkSysclkDiv2;  // 84MHz
	stcSysclkCfg.enPclk0Div = ClkSysclkDiv1;  // 168MHz
	stcSysclkCfg.enPclk1Div = ClkSysclkDiv2;  // 84MHz
	stcSysclkCfg.enPclk2Div = ClkSysclkDiv4;  // 42MHz
	stcSysclkCfg.enPclk3Div = ClkSysclkDiv4;  // 42MHz
	stcSysclkCfg.enPclk4Div = ClkSysclkDiv1;  // 84MHz.
	CLK_SysClkConfig(&stcSysclkCfg);
	CLK_SetPeriClkSource(ClkPeriSrcPclk);

#elif (ADC_CLK == ADC_CLK_MPLLQ)
	stc_clk_xtal_cfg_t xtalConf;
	stc_clk_mpll_cfg_t mpllConf;

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
		mpllConf.pllmDiv = 1u;
		/* mpll = 8M / pllmDiv * plln */
		mpllConf.plln	= 30u;
		mpllConf.PllpDiv = 16u;
		mpllConf.PllqDiv = 16u;
		mpllConf.PllrDiv = 16u;
		CLK_SetPllSource(ClkPllSrcXTAL);
		CLK_MpllConfig(&mpllConf);
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
	upllConf.plln	= 60u;

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
 ** \brief  ADC initial configuration.
 **
 ******************************************************************************/
static void adc_initConfig(void)
{
	// enable adc1
	PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);

	// initialize adc1
	stc_adc_init_t adcConf;
	MEM_ZERO_STRUCT(adcConf);
	adcConf.enResolution = AdcResolution_12Bit;
	adcConf.enDataAlign  = AdcDataAlign_Right;
	adcConf.enAutoClear  = AdcClren_Enable;
	adcConf.enScanMode   = AdcMode_SAContinuous;
	adcConf.enRschsel	= AdcRschsel_Restart;
	/* 2. Initialize ADC1. */
	ADC_Init(M4_ADC1, &adcConf);
}

/**
 *******************************************************************************
 ** \brief  Set an ADC pin as analog input mode or digit mode.
 **
 ******************************************************************************/
static void adc_setPinMode(uint8_t adcPin, en_pin_mode_t mode)
{
	// translate adc input to pin
	en_port_t enPort = PortA;
	en_pin_t enPin   = Pin00;
	bool bFlag	   = true;
	switch (adcPin)
	{
	case ADC1_IN0:
		enPort = PortA;
		enPin  = Pin00;
		break;
	case ADC1_IN1:
		enPort = PortA;
		enPin  = Pin01;
		break;
	case ADC1_IN2:
		enPort = PortA;
		enPin  = Pin02;
		break;
	case ADC1_IN3:
		enPort = PortA;
		enPin  = Pin03;
		break;
	case ADC12_IN4:
		enPort = PortA;
		enPin  = Pin04;
		break;
	case ADC12_IN5:
		enPort = PortA;
		enPin  = Pin05;
		break;
	case ADC12_IN6:
		enPort = PortA;
		enPin  = Pin06;
		break;
	case ADC12_IN7:
		enPort = PortA;
		enPin  = Pin07;
		break;
	case ADC12_IN8:
		enPort = PortB;
		enPin  = Pin00;
		break;
	case ADC12_IN9:
		enPort = PortB;
		enPin  = Pin01;
		break;
	case ADC12_IN10:
		enPort = PortC;
		enPin  = Pin00;
		break;
	case ADC12_IN11:
		enPort = PortC;
		enPin  = Pin01;
		break;
	case ADC1_IN12:
		enPort = PortC;
		enPin  = Pin02;
		break;
	case ADC1_IN13:
		enPort = PortC;
		enPin  = Pin03;
		break;
	case ADC1_IN14:
		enPort = PortC;
		enPin  = Pin04;
		break;
	case ADC1_IN15:
		enPort = PortC;
		enPin  = Pin05;
		break;
	default:
		bFlag = false;
		break;
	}

	// set pin mode
	stc_port_init_t portConf;
	MEM_ZERO_STRUCT(portConf);
	portConf.enPinMode = mode;
	portConf.enPullUp  = Disable;
	if (true == bFlag)
	{
		PORT_Init(enPort, enPin, &portConf);
	}
}

/**
 *******************************************************************************
 ** \brief  Config the pin which is mapping the channel to analog or digit mode.
 **
 ******************************************************************************/
static void adc_setChannelPinMode(const M4_ADC_TypeDef *ADCx, uint32_t channel, en_pin_mode_t mode)
{
	// get channel offset and mask
#if (ADC_CH_REMAP)
	uint8_t adcPin;
#else
	uint8_t channelOffset = 0u;
#endif
	if (M4_ADC1 == ADCx)
	{
		channel &= ADC1_PIN_MASK_ALL;
	}
	else
	{
		channel &= ADC2_PIN_MASK_ALL;
#if (!ADC_CH_REMAP)
		channelOffset = 4u;
#endif
	}

	// set pin mode of all pins in the channel
	for (uint8_t i = 0u; channel != 0u; i++)
	{
		if (channel & 0x1ul)
		{
#if (ADC_CH_REMAP)
			adcPin = ADC_GetChannelPinNum(ADCx, i);
			adc_setPinMode(adcPin, mode);
#else
			adc_setPinMode((channelOffset + i), mode);
#endif
		}

		channel >>= 1u;
	}
}

/**
 *******************************************************************************
 ** \brief  ADC channel configuration.
 **
 ******************************************************************************/
static void adc_channelConfig(void)
{
	uint8_t samplingTimes[3] = { 0x60, 0x60, 0x60 };

	// init adc channel
	stc_adc_ch_cfg_t adcChannelConf;
	MEM_ZERO_STRUCT(adcChannelConf);
	adcChannelConf.u32Channel  = BOARD_ADC_CH0_CH | BOARD_ADC_CH1_CH | BOARD_ADC_CH2_CH;
	adcChannelConf.u8Sequence  = ADC_SEQ_A;
	adcChannelConf.pu8SampTime = samplingTimes;
	ADC_AddAdcChannel(M4_ADC1, &adcChannelConf);
//	ADC_ConfigAvg(M4_ADC1, AdcAvcnt_64);
//	ADC_AddAvgChannel(M4_ADC1, BOARD_ADC_CH0_CH | BOARD_ADC_CH1_CH | BOARD_ADC_CH2_CH);
}

/**
 *******************************************************************************
 ** \brief  ADC trigger source configuration.
 **
 ******************************************************************************/
static void adc_triggerConfig(void)
{
	PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

	stc_adc_trg_cfg_t stcTrgCfg;
	MEM_ZERO_STRUCT(stcTrgCfg);

	/*
	 * If select an event(@ref en_event_src_t) to trigger ADC,
	 * AOS must be enabled first.
	 */

	// ADC1 is always triggered by software
	ADC_TriggerSrcCmd(M4_ADC1, ADC_SEQ_A, Disable);
}

// DMA2 CH0 ~ CH2
void adc_dmaInitConfig(void)
{
	// setup dma config
	stc_dma_config_t dmaConf;
	MEM_ZERO_STRUCT(dmaConf);
	dmaConf.u16BlockSize   = ADC_CH_COUNT;
	dmaConf.u16TransferCnt = 0u;
	dmaConf.u32SrcAddr	 = (uint32_t)(&M4_ADC1->DR10);
	dmaConf.u32DesAddr	 = (uint32_t)(&g_adc_value[0]);
	dmaConf.u16SrcRptSize  = ADC_CH_COUNT;
	dmaConf.u16DesRptSize  = ADC_CH_COUNT;
	dmaConf.u32DmaLlp	  = 0u;
	dmaConf.stcSrcNseqCfg.u32Offset = 0u;
	dmaConf.stcSrcNseqCfg.u16Cnt	= 0u;
	dmaConf.stcDesNseqCfg.u32Offset = 0u;
	dmaConf.stcDesNseqCfg.u16Cnt	= 0u;
	dmaConf.stcDmaChCfg.enSrcInc	= AddressIncrease;
	dmaConf.stcDmaChCfg.enDesInc	= AddressIncrease;
	dmaConf.stcDmaChCfg.enSrcRptEn  = Enable;
	dmaConf.stcDmaChCfg.enDesRptEn  = Enable;
	dmaConf.stcDmaChCfg.enSrcNseqEn = Disable;
	dmaConf.stcDmaChCfg.enDesNseqEn = Disable;
	dmaConf.stcDmaChCfg.enTrnWidth  = Dma16Bit;
	dmaConf.stcDmaChCfg.enLlpEn	 = Disable;
	dmaConf.stcDmaChCfg.enIntEn	 = Disable;


	PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2, Enable);
	DMA_InitChannel(M4_DMA2, DmaCh3, &dmaConf);

	DMA_Cmd(M4_DMA2, Enable);
	DMA_ChannelCmd(M4_DMA2, DmaCh3, Enable);
	DMA_ClearIrqFlag(M4_DMA2, DmaCh3, TrnCpltIrq);

	// AOS must be enabled to use DMA
	// AOS enabled at first
	// If you have enabled AOS before, then the following statement is not needed
	PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);
	DMA_SetTriggerSrc(M4_DMA2, DmaCh3, EVT_ADC1_EOCA);
}

void setup_adcs(void)
{
}

void adc_main(void)
{
}

static void adc_pin_init(void)
{
	stc_port_init_t portConf;

	MEM_ZERO_STRUCT(portConf);

	portConf.enPullUp  = Disable;
	portConf.enPinMode = Pin_Mode_Ana;

	PORT_Init(BOARD_ADC_CH0_PORT, BOARD_ADC_CH0_PIN, &portConf);
	PORT_Init(BOARD_ADC_CH1_PORT, BOARD_ADC_CH1_PIN, &portConf);
	PORT_Init(BOARD_ADC_CH2_PORT, BOARD_ADC_CH2_PIN, &portConf);
}

void adc_setDefaultConfig(void)
{
	// init and config adc and channels
	adc_initConfig();
	adc_pin_init();
	adc_channelConfig();
	adc_triggerConfig();

	// init and config DMA
	adc_dmaInitConfig();

	ADC_StartConvert(M4_ADC1);
}

void adc_init(void)
{
	// configure ADC
	adc_setDefaultConfig();
}
