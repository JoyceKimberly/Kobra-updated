#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"
#include "HAL.h"
#include "hc32_ddl.h"
#include <core_hooks.h>

//
// Emergency Parser
//
#if ENABLED(EMERGENCY_PARSER)
extern "C" void core_hook_usart_rx_irq(uint8_t ch, uint8_t usart)
{
    // only handle receive on host serial ports
    if (false
#ifdef SERIAL_PORT
        || usart != SERIAL_PORT
#endif
#ifdef SERIAL_PORT_2
        || usart != SERIAL_PORT_2
#endif
#ifdef SERIAL_PORT_3
        || usart != SERIAL_PORT_3
#endif
    )
    {
        return;
    }
}
#endif
