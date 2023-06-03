#include "adc.h"
#include "../gpio/gpio.h"
#include "../irqn/irqn.h"
#include "usart.h"

uint16_t g_adc_value[3];
uint8_t g_adc_idx;

//
// ADC1 device
//
adc_device_t ADC1_device = {
	.HAL_AdcDmaIrqFlag = 0,
	.HAL_adc_results = {0},

	.regs = M4_ADC1,
	.PeriphClock = PWC_FCG3_PERIPH_ADC1,
	.Channel = ADC1_SA_CHANNEL,

	.DMARegs = M4_DMA2,
	.DMAPeriphClock = PWC_FCG0_PERIPH_DMA2,
	.DMAChannel = DmaCh3,
	.DMAenSrc = EVT_ADC1_EOCA,
};
adc_device_t *ADC1 = &ADC1_device;

uint16_t adc_read_sync(adc_device_t *dev, uint8_t channel)
{
	// wait for adc result
	while (true)
	{
		if (dev->HAL_AdcDmaIrqFlag & ADC1_SA_DMA_IRQ_BIT)
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
	dev->HAL_AdcDmaIrqFlag &= ~ADC1_SA_DMA_IRQ_BIT;
	return dev->HAL_adc_results[channel];
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
		pllConf.plln	= 30u;

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
void adc_initConfig(adc_device_t *dev)
{
	// enable ADC1_device
	PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);

	// initialize ADC1_device
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

/**
 *******************************************************************************
 ** \brief  ADC channel configuration.
 **
 ******************************************************************************/
void adc_channelConfig(adc_device_t *dev, en_pin_mode_t mode)
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
	stc_adc_ch_cfg_t adcChannelConf;
	MEM_ZERO_STRUCT(adcChannelConf);
	adcChannelConf.u32Channel  = ADC1_CH10 | ADC1_CH11 | ADC1_CH12;
	adcChannelConf.u8Sequence  = ADC_SEQ_A;
	adcChannelConf.pu8SampTime = samplingTimes;
	ADC_AddAdcChannel(M4_ADC1, &adcChannelConf);
//	ADC_ConfigAvg(M4_ADC1, AdcAvcnt_64);
//	ADC_AddAvgChannel(M4_ADC1, ADC1_CH10 | ADC1_CH11 | ADC1_CH12);
}

/**
 *******************************************************************************
 ** \brief  ADC trigger source configuration.
 **
 ******************************************************************************/
void adc_triggerConfig(adc_device_t *dev, uint32_t fcg0Periph)
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
	ADC_ConfigTriggerSrc(dev->regs, &triggerConf);
	ADC_TriggerSrcCmd(dev->regs, ADC_SEQ_A, Enable);
	*/

	// ADC1 is always triggered by software
	ADC_TriggerSrcCmd(M4_ADC1, ADC_SEQ_A, Disable);
}

// DMA2 CH0 ~ CH2
void adc_dmaInitConfig(adc_device_t *dev)
{
	// setup dma config
	stc_dma_config_t dmaConf;
	MEM_ZERO_STRUCT(dmaConf);
	dmaConf.u16BlockSize   = 3;
	dmaConf.u16TransferCnt = 0u;
	dmaConf.u32SrcAddr	 = (uint32_t)(&M4_ADC1->DR10);
	dmaConf.u32DesAddr	 = (uint32_t)(&g_adc_value[0]);
	dmaConf.u16SrcRptSize  = 3;
	dmaConf.u16DesRptSize  = 3;
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

void adc_dmaRegisterIRQ(stc_irq_regi_conf_t *pstcCfg, uint32_t priority)
{
	int16_t irqNum = pstcCfg->enIRQn;
	if (((irqNum >= Int000_IRQn) && (irqNum <= Int031_IRQn)) ||
		((irqNum >= Int038_IRQn) && (irqNum <= Int043_IRQn)))
	{
		if (enIrqRegistration(pstcCfg) != Ok)
		{
			return;
		}
	}
	else if (irqNum == Int129_IRQn)
	{
		enShareIrqEnable(pstcCfg->enIntSrc);
	}
	else
	{
		return;
	}

	NVIC_ClearPendingIRQ(pstcCfg->enIRQn);
	NVIC_SetPriority(pstcCfg->enIRQn, priority);
	NVIC_EnableIRQ(pstcCfg->enIRQn);
}

/**
 * ADC DMA IRQ handler
 */
void Dma1Btc3_IrqHandler(void)
{
	// DMA_ClearIrqFlag(ADC1->DMARegs, ADC1->DMAChannel, BlkTrnCpltIrq);
	ADC1->HAL_AdcDmaIrqFlag |= ADC1_SA_DMA_IRQ_BIT;
}

void adc_dmaIRQConfig(void)
{
	// get auto-assigned IRQn
	IRQn_Type irqn;
	irqn_aa_get(irqn, "adc dma irq");

	// register IRQ
	stc_irq_regi_conf_t stcAdcIrqCfg = {
		.enIntSrc = INT_DMA1_BTC3,
		.enIRQn = irqn,
		.pfnCallback = &Dma1Btc3_IrqHandler,
	};
	adc_dmaRegisterIRQ(&stcAdcIrqCfg, DDL_IRQ_PRIORITY_DEFAULT);
}

void adc_setDefaultConfig(adc_device_t *dev)
{
	// init and config adc and channels
	adc_initConfig(dev);
	adc_channelConfig(dev, Pin_Mode_Ana);
	adc_triggerConfig(dev, PWC_FCG0_PERIPH_AOS);

	// init and config DMA
	adc_dmaInitConfig(dev);
	adc_dmaIRQConfig();
	ADC_StartConvert(dev->regs);
}

void adc_init(void)
{
	// set ADC clock (default is MRC @ 8MHz)
	adc_setCLK();

	// configure ADC
	adc_setDefaultConfig(ADC1);
}
