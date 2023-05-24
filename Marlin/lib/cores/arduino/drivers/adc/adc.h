#ifndef _ADC_H
#define _ADC_H

#include <hc32_ddl.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ADC_CH 3

	//
	// ADC device definition
	//
	typedef struct adc_dev
	{
		__IO uint32_t HAL_AdcDmaIrqFlag;
		__IO uint16_t HAL_adc_results[ADC_CH];

		M4_ADC_TypeDef *regs;           /**< Register map */
		__IO uint32_t PeriphClock;      /**< clock information */
		__IO uint32_t Channel;

		M4_DMA_TypeDef *DMARegs;
		__IO uint32_t DMAPeriphClock;
		__IO uint8_t DMAChannel;
		__IO en_event_src_t DMAenSrc;
	} adc_dev;

	extern adc_dev adc1;
	extern struct adc_dev *ADC1;

/********************************************************/
/********************************************************/
// ADC1 channel definition ch0~ch3.
#define ADC1_SA_CHANNEL                (ADC1_CH10|ADC1_CH11|ADC1_CH12)
#define ADC1_SA_CHANNEL_COUNT       (ADC_CHANNEL_COUNT)
//ADC1 channel sampling time.      ADC1_CH0  ADC1_CH1 ADC1_CH2  
/********************************************************/
/********************************************************/
//#define ADC1_SB_CHANNEL             (ADC1_CH4|ADC1_CH5|ADC1_CH6)
//#define ADC1_SB_CHANNEL_COUNT       (3u)
//#define ADC1_SB_CHANNEL_SAMPLE_TIME { 0x50,0x60,0x45 }
/********************************************************/
/********************************************************/

/* ADC interrupt flag bit mask definition. */
#define ADC1_SA_DMA_IRQ_BIT         (1ul << 0u)

	extern void setup_adcs(void);
	extern void adc_main(void);
	uint16_t adc_read(adc_dev *dev, uint8_t channel);


#define BOARD_ADC_CH0_PORT       (PortC)
#define BOARD_ADC_CH0_PIN        (Pin00)
#define BOARD_ADC_CH0_CH         (ADC1_CH10)

#define BOARD_ADC_CH1_PORT       (PortC)
#define BOARD_ADC_CH1_PIN        (Pin01)
#define BOARD_ADC_CH1_CH         (ADC1_CH11)

#define BOARD_ADC_CH2_PORT       (PortC)
#define BOARD_ADC_CH2_PIN        (Pin02)
#define BOARD_ADC_CH2_CH         (ADC1_CH12)

/* Timer definition for this example. */
#define TMR_UNIT                 (M4_TMR02)

extern uint16_t  AdcCH0SampleBuf[256];
extern uint16_t  AdcCH1SampleBuf[256];
extern uint16_t  AdcCH2SampleBuf[256];

extern uint32_t  AdcCH0Value;
extern uint32_t  AdcCH1Value;
extern uint32_t  AdcCH2Value;

extern uint16_t g_adc_value[3];
extern uint8_t g_adc_idx;

void adc_init(void);

static void adc_pin_init(void);

static void adc_setCLK(void);

static void adc_initConfig(void);

static void adc_channelConfig(void);

static void adc_triggerConfig(void);


static void adc_setChannelPinMode(const M4_ADC_TypeDef *ADCx,
                                 uint32_t u32Channel,
                                 en_pin_mode_t mode);

static void adc_setPinMode(uint8_t adcPin, en_pin_mode_t mode);

void adc_dmaInitConfig(void);


void BSP_DMA2CH0_TcIrqHander(void);

void BSP_DMA2CH1_TcIrqHander(void);

void BSP_DMA2CH2_TcIrqHander(void);


void AdcConfig(void);

#ifdef __cplusplus
}
#endif

#endif
