#include "usart.h"
#include "HardwareSerial.h"

uint8_t g_uart2_rx_buf[128];
uint8_t g_uart2_rx_index;

extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
extern HardwareSerial Serial4;

//
// USART1 callbacks
//
void Usart1RxIrqCallback(void)
{
    uint8_t c = USART_RecData(M4_USART1);
    Serial1._rx_complete_callback(c);
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
    Serial2._rx_complete_callback(c);

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
  Serial3._rx_complete_callback(c);
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
  Serial4._rx_complete_callback(c);
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

//
// public api
//
