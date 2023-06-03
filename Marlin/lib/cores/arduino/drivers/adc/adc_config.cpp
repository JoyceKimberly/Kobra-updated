#include "adc_config.h"

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

//
// ADC devices
//
adc_device_t ADC1_device = {
    .adc = {
        .register_base = M4_ADC1,
        .clock_id = PWC_FCG3_PERIPH_ADC1,
        .sequence = ADC_SEQ_A,
        .channel_count = 3,
    },
    .init_params = {

        //TODO: adc resolution should be configurable at compile time
        .resolution = AdcResolution_12Bit,
        .data_alignment = AdcDataAlign_Right,
        .scan_mode = AdcMode_SAContinuous, // only sequence A
    },
    .dma = {
        .register_base = M4_DMA2,
        .clock_id = PWC_FCG0_PERIPH_DMA2,
        .channel = DmaCh3,
        .event_source = EVT_ADC1_EOCA,
    },
    .state = {
        .conversion_results = new uint16_t[ADC1_CH_COUNT],
    },
};
adc_device_t *ADC1 = &ADC1_device;
