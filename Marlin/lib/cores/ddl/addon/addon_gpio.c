#include "addon_gpio.h"
#include "hc32f460_utility.h"

#if (DDL_GPIO_ENABLE == DDL_ON)

// copied from 'hc32f46x_gpio.c'
#define IS_VALID_PORT(x) \
    (((x) == PortA) ||   \
     ((x) == PortB) ||   \
     ((x) == PortC) ||   \
     ((x) == PortD) ||   \
     ((x) == PortE) ||   \
     ((x) == PortH))

/**
 * get pin configuration, inverse to PORT_Init
 */
en_result_t PORT_GetConfig(en_port_t port, uint16_t pin, stc_port_init_t *portConf)
{
    DDL_ASSERT(IS_VALID_PORT(port));
    PORT_Unlock();
    for (uint8_t pinPos = 0u; pinPos < 16u; pinPos++)
    {
        if (pin & (1ul << pinPos))
        {
            stc_port_pcr_field_t *PCRx = (stc_port_pcr_field_t *)((uint32_t)(&M4_PORT->PCRA0) +
                                                                  port * 0x40ul + pinPos * 0x04ul);
            stc_port_pfsr_field_t *PFSRx = (stc_port_pfsr_field_t *)((uint32_t)(&M4_PORT->PFSRA0) +
                                                                     port * 0x40ul + pinPos * 0x04ul);

            // input latch setting
            portConf->enLatch = (en_functional_state_t)PCRx->LTE;

            // external interrupt input enable
            portConf->enExInt = (en_functional_state_t)PCRx->INTE;

            // In_Out invert
            portConf->enInvert = (en_functional_state_t)PCRx->INVE;

            // pin pull-up enable
            portConf->enPullUp = (en_functional_state_t)PCRx->PUU;

            // CMOS/OD output toggle
            portConf->enPinOType = (en_pin_o_type_t)PCRx->NOD;

            // pin drive mode
            portConf->enPinDrv = (en_pin_drv_t)PCRx->DRV;

            // pin mode
            if (PCRx->DDIS == 1u)
            {
                portConf->enPinMode = Pin_Mode_Ana;
            }
            else
            {
                portConf->enPinMode = (PCRx->POUTE == 1u) ? Pin_Mode_Out : Pin_Mode_In;
            }

            // sub-function enable
            portConf->enPinSubFunc = (en_functional_state_t)PFSRx->BFE;
        }
    }

    PORT_Lock();
    return Ok;
}
#endif
