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
void BSP_USART1_RIIrqHander(void)
{
    uint8_t c = USART_RecData(M4_USART1);
    Serial1._rx_complete_callback(c);
//  RingBuf_Write(&Usart1RingBuf,(uint8_t)tmp);
}

void BSP_USART1_EIIrqHander(void)
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

void BSP_USART1_TIrqHander(void)
{
}

void BSP_USART1_TCIIrqHander(void)
{
}

//
// USART2 callbacks
//
void BSP_USART2_RIIrqHander(void)
{
    uint8_t c = USART_RecData(M4_USART2);
    Serial2._rx_complete_callback(c);

//  RingBuf_Write(&Usart2RingBuf,(uint8_t)tmp);
}

void BSP_USART2_EIIrqHander(void)
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

void BSP_USART2_TIrqHander(void)
{
}

void BSP_USART2_TCIIrqHander(void)
{
}

//
// USART3 callbacks
//
void BSP_USART3_RIIrqHander(void)
{
  uint8_t c = USART_RecData(M4_USART3);
  Serial3._rx_complete_callback(c);
//  RingBuf_Write(&Usart3RingBuf,(uint8_t)tmp);
}

void BSP_USART3_EIIrqHander(void)
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

void BSP_USART3_TIrqHander(void)
{
}

void BSP_USART3_TCIIrqHander(void)
{
}

//
// USART4 callbacks
//
void BSP_USART4_RIIrqHander(void)
{
  uint8_t c = USART_RecData(M4_USART4);
  Serial4._rx_complete_callback(c);
//  RingBuf_Write(&Usart3RingBuf,(uint8_t)tmp);
}

void BSP_USART4_EIIrqHander(void)
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

void BSP_USART4_TIrqHander(void)
{
}

void BSP_USART4_TCIIrqHander(void)
{
}

//
// public api
//
