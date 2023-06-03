#include "adc_config.h"

//
// ADC devices
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
