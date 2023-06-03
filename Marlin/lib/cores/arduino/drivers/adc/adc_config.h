#pragma once
#include <hc32_ddl.h>

/********************************************************/
/********************************************************/
// ADC1 channel definition ch0~ch3.
#define ADC_CHANNEL_COUNT     3u
#define ADC1_SA_CHANNEL       (ADC1_CH10 | ADC1_CH11 | ADC1_CH12)
#define ADC1_SA_CHANNEL_COUNT (ADC_CHANNEL_COUNT)
//ADC1 channel sampling time.      ADC1_CH0  ADC1_CH1 ADC1_CH2  
/********************************************************/
/********************************************************/
//#define ADC1_SB_CHANNEL             (ADC1_CH4|ADC1_CH5|ADC1_CH6)
//#define ADC1_SB_CHANNEL_COUNT       (3u)
//#define ADC1_SB_CHANNEL_SAMPLE_TIME { 0x50,0x60,0x45 }
/********************************************************/
/********************************************************/

// ADC irq flag bit mask
#define ADC1_SA_DMA_IRQ_BIT (1ul << 0u)

/**
 * @brief ADC device configuration
 */
typedef struct adc_device_t
{
    __IO uint32_t HAL_AdcDmaIrqFlag;
    __IO uint16_t HAL_adc_results[3];

    M4_ADC_TypeDef *regs;           /**< Register map */
    __IO uint32_t PeriphClock;      /**< clock information */
    __IO uint32_t Channel;

    M4_DMA_TypeDef *DMARegs;
    __IO uint32_t DMAPeriphClock;
    __IO uint8_t DMAChannel;
    __IO en_event_src_t DMAenSrc;
} adc_device_t;

//
// ADC devices
//
extern adc_device_t ADC1_device;
extern struct adc_device_t *ADC1;
