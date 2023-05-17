#pragma once
#include "hc32_common.h"
#include "ddl_config.h"
#if (DDL_GPIO_ENABLE == DDL_ON)

#include "hc32f460_gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

    en_result_t PORT_GetConfig(en_port_t port, uint16_t pin, stc_port_init_t *portConf);

#ifdef __cplusplus
}
#endif
#endif
