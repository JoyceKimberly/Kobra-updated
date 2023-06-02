#include <hc32_ddl.h>
#include "usart.h"
#include "core_hooks.h"
#include "core_debug.h"
#include "yield.h"
#include "../gpio/gpio.h"
#include "../irqn/irqn.h"
#include "HardwareSerial.h"

//
// global instances
//
#ifndef DISABLE_SERIAL_GLOBALS
HardwareSerial MSerial1(&USART1_config, VARIANT_USART1_TX_PIN, VARIANT_USART1_RX_PIN);
HardwareSerial MSerial2(&USART2_config, VARIANT_USART2_TX_PIN, VARIANT_USART2_RX_PIN);
HardwareSerial MSerial3(&USART3_config, VARIANT_USART3_TX_PIN, VARIANT_USART3_RX_PIN);
HardwareSerial MSerial4(&USART4_config, VARIANT_USART4_TX_PIN, VARIANT_USART4_RX_PIN);
#endif

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
}

void Usart1TxCmpltIrqCallback(void)
{
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
}

void Usart2TxCmpltIrqCallback(void)
{
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
}

void Usart3TxCmpltIrqCallback(void)
{
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
}

void Usart4TxCmpltIrqCallback(void)
{
}
