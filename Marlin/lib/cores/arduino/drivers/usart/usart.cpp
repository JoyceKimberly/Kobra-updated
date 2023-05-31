#include <hc32_ddl.h>
#include "usart.h"
#include "core_hooks.h"
#include "core_debug.h"
#include "yield.h"
#include "../gpio/gpio.h"
#include "HardwareSerial.h"

//
// global instances
//
#ifndef DISABLE_SERIAL_GLOBALS
HardwareSerial MSerial1(M4_USART1);
HardwareSerial MSerial2(M4_USART2);
HardwareSerial MSerial3(M4_USART3);
HardwareSerial MSerial4(M4_USART4);
#endif

//
// IRQ register / unregister helper
//
inline void usart_irq_register(usart_interrupt_config_t irq)
{
    // create irq registration struct
    stc_irq_regi_conf_t irqConf = {
        .enIntSrc = irq.interrupt_source,
        .enIRQn = irq.interrupt_number,
        .pfnCallback = irq.interrupt_handler,
    };

    // register and enable irq
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, irq.interrupt_priority);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);
}

inline void usart_irq_resign(usart_interrupt_config_t irq)
{
    NVIC_DisableIRQ(irq.interrupt_number);
    NVIC_ClearPendingIRQ(irq.interrupt_number);
    enIrqResign(irq.interrupt_number);
}

//
// debug print helpers
//
#define USART_REG_TO_X(reg) \
    reg == M4_USART1   ? 1  \
    : reg == M4_USART2 ? 2  \
    : reg == M4_USART3 ? 3  \
    : reg == M4_USART4 ? 4  \
                       : 0
#define USART_DEBUG_PRINTF(fmt, ...) \
    CORE_DEBUG_PRINTF("[USART%d] " fmt, USART_REG_TO_X(this->config->peripheral.register_base), ##__VA_ARGS__)

//
// Usart class implementation
//

//
// USART1 callbacks
//
void Usart1RxIrqCallback(void)
{
    uint8_t c = USART_RecData(M4_USART1);
    MSerial1._rx_complete_callback(c);
//  RingBuf_Write(&Usart1RingBuf,(uint8_t)tmp);
}

void Usart1ErrIrqCallback(void)
{
	if (USART_GetStatus(M4_USART1, UsartFrameErr) == Set)
	{
		USART_ClearStatus(M4_USART1, UsartFrameErr);
	}

	if (USART_GetStatus(M4_USART1, UsartParityErr) == Set)
	{
		USART_ClearStatus(M4_USART1, UsartParityErr);
	}

	if (USART_GetStatus(M4_USART1, UsartOverrunErr) == Set)
	{
		USART_ClearStatus(M4_USART1, UsartOverrunErr);
	}
}

void Usart1TxIrqCallback(void)
{
	usart_tx_irq(USART1);
}

void Usart1TxCmpltIrqCallback(void)
{
	USART_FuncCmd(M4_USART1, UsartTxCmpltInt, Disable);
	USART_FuncCmd(M4_USART1, UsartTx, Disable);
}

//
// USART2 callbacks
//
void Usart2RxIrqCallback(void)
{
    uint8_t c = USART_RecData(M4_USART2);
    MSerial2._rx_complete_callback(c);

//  RingBuf_Write(&Usart2RingBuf,(uint8_t)tmp);
}

void Usart2ErrIrqCallback(void)
{
	if (USART_GetStatus(M4_USART2, UsartFrameErr) == Set)
	{
		USART_ClearStatus(M4_USART2, UsartFrameErr);
	}

	if (USART_GetStatus(M4_USART2, UsartParityErr) == Set)
	{
		USART_ClearStatus(M4_USART2, UsartParityErr);
	}

	if (USART_GetStatus(M4_USART2, UsartOverrunErr) == Set)
	{
		USART_ClearStatus(M4_USART2, UsartOverrunErr);
	}
}

void Usart2TxIrqCallback(void)
{
	usart_tx_irq(USART2);
}

void Usart2TxCmpltIrqCallback(void)
{
	USART_FuncCmd(M4_USART2, UsartTxCmpltInt, Disable);
	USART_FuncCmd(M4_USART2, UsartTx, Disable);
}

//
// USART3 callbacks
//
void Usart3RxIrqCallback(void)
{
    uint8_t c = USART_RecData(M4_USART3);
    MSerial3._rx_complete_callback(c);
//  RingBuf_Write(&Usart3RingBuf,(uint8_t)tmp);
}

void Usart3ErrIrqCallback(void)
{
	if (USART_GetStatus(M4_USART3, UsartFrameErr) == Set)
	{
		USART_ClearStatus(M4_USART3, UsartFrameErr);
	}

	if (USART_GetStatus(M4_USART3, UsartParityErr) == Set)
	{
		USART_ClearStatus(M4_USART3, UsartParityErr);
	}

	if (USART_GetStatus(M4_USART3, UsartOverrunErr) == Set)
	{
		USART_ClearStatus(M4_USART3, UsartOverrunErr);
	}
}

void Usart3TxIrqCallback(void)
{
	usart_tx_irq(USART3);
}

void Usart3TxCmpltIrqCallback(void)
{
	USART_FuncCmd(M4_USART3, UsartTxCmpltInt, Disable);
	USART_FuncCmd(M4_USART3, UsartTx, Disable);
}

//
// USART4 callbacks
//
void Usart4RxIrqCallback(void)
{
    uint8_t c = USART_RecData(M4_USART4);
    MSerial4._rx_complete_callback(c);
//  RingBuf_Write(&Usart3RingBuf,(uint8_t)tmp);
}

void Usart4ErrIrqCallback(void)
{
	if (USART_GetStatus(M4_USART4, UsartFrameErr) == Set)
	{
		USART_ClearStatus(M4_USART4, UsartFrameErr);
	}

	if (USART_GetStatus(M4_USART4, UsartParityErr) == Set)
	{
		USART_ClearStatus(M4_USART4, UsartParityErr);
	}

	if (USART_GetStatus(M4_USART4, UsartOverrunErr) == Set)
	{
		USART_ClearStatus(M4_USART4, UsartOverrunErr);
	}
}

void Usart4TxIrqCallback(void)
{
	usart_tx_irq(USART4);
}

void Usart4TxCmpltIrqCallback(void)
{
	USART_FuncCmd(M4_USART4, UsartTxCmpltInt, Disable);
	USART_FuncCmd(M4_USART4, UsartTx, Disable);
}
