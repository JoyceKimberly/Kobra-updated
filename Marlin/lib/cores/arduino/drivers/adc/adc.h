#pragma once
#include <hc32_ddl.h>
#include "adc_config.h"

#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief ADC peripheral init
     * @param device ADC device configuration
     * @note if the device is initialized, this function will do nothing
     */
    void adc_init();







    /**
     * @brief start adc conversion and wait for result synchronously
     * @param device ADC device configuration
     * @param adc_channel ADC channel to read
     * @return conversion result
     * @note requires adc_device_init() to be called first
     */
    uint16_t adc_read_sync(adc_device_t *device, uint8_t channel);

    #define BOARD_ADC_CH0_PORT       (PortC)
    #define BOARD_ADC_CH0_PIN        (Pin00)

    #define BOARD_ADC_CH1_PORT       (PortC)
    #define BOARD_ADC_CH1_PIN        (Pin01)

    #define BOARD_ADC_CH2_PORT       (PortC)
    #define BOARD_ADC_CH2_PIN        (Pin02)

    extern uint16_t  AdcCH0SampleBuf[256];
    extern uint16_t  AdcCH1SampleBuf[256];
    extern uint16_t  AdcCH2SampleBuf[256];

    extern uint32_t  AdcCH0Value;
    extern uint32_t  AdcCH1Value;
    extern uint32_t  AdcCH2Value;

    extern uint16_t g_adc_value[3];
    extern uint8_t g_adc_idx;

    void BSP_DMA2CH0_TcIrqHander(void);
    void BSP_DMA2CH1_TcIrqHander(void);
    void BSP_DMA2CH2_TcIrqHander(void);

#ifdef __cplusplus
}
#endif
