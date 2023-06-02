/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file wirish/HardwareSerial.cpp
 * @brief Wirish serial port implementation.
 */

#include "HardwareSerial.h"
#include "drivers/gpio/gpio.h"
#include <hc32_ddl.h>
#include "Usart.h"
#include "core_hooks.h"
#include "core_debug.h"
#include "yield.h"
#include "../gpio/gpio.h"
#include "../irqn/irqn.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

//
// IRQ register / unregister helper
//
inline void usart_irq_register(usart_interrupt_config_t &irq, const char *name)
{
    // get auto-assigned irqn and set in irq struct
    IRQn_Type irqn;
    irqn_aa_get(irqn, name);
    irq.interrupt_number = irqn;

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

inline void usart_irq_resign(usart_interrupt_config_t &irq, const char *name)
{
    // disable interrupt and clear pending
    NVIC_DisableIRQ(irq.interrupt_number);
    NVIC_ClearPendingIRQ(irq.interrupt_number);
    enIrqResign(irq.interrupt_number);

    // resign auto-assigned irqn
    irqn_aa_resign(irq.interrupt_number, name);
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

HardwareSerial::HardwareSerial(struct usart_config_t *config, uint16_t tx_pin, uint16_t rx_pin) :
    _rx_buffer_head(0), _rx_buffer_tail(0)
{
    this->config = config;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;

    // unpack rx and tx buffers from usart config
    this->rxBuffer = config->state.rx_buffer;
    this->txBuffer = config->state.tx_buffer;
}

size_t HardwareSerial::begin(uint32_t baud)
{
    return baud;
}

void HardwareSerial::begin(uint32_t baud, uint16_t config)
{
    // create full configuration from input config
    stc_usart_uart_init_t usartConfig = {
        .enClkMode = UsartIntClkCkNoOutput,
        .enClkDiv = UsartClkDiv_16,
        .enDataLength = UsartDataBits8,
        .enDirection = UsartDataLsbFirst,
        .enStopBit = UsartOneStopBit,
        .enParity = UsartParityNone,
        .enSampleMode = UsartSampleBit8,
        .enDetectMode = UsartStartBitFallEdge,
        .enHwFlow = UsartRtsEnable,
    };

    // stop bits
    switch (config & HARDSER_STOP_BIT_MASK)
    {
    default:
    case HARDSER_STOP_BIT_1:
        usartConfig.enStopBit = UsartOneStopBit;
        break;
    case HARDSER_STOP_BIT_2:
        usartConfig.enStopBit = UsartTwoStopBit;
        break;
    }

    // parity
    switch (config & HARDSER_PARITY_MASK)
    {
    default:
    case HARDSER_PARITY_NONE:
        usartConfig.enParity = UsartParityNone;
        break;
    case HARDSER_PARITY_EVEN:
        usartConfig.enParity = UsartParityEven;
        break;
    case HARDSER_PARITY_ODD:
        usartConfig.enParity = UsartParityOdd;
        break;
    }

    // data bits
    switch (config & HARDSER_DATA_MASK)
    {
    default:
    case HARDSER_DATA_8:
        usartConfig.enDataLength = UsartDataBits8;
        break;
    }

    // call begin with full config
    begin(baud, &usartConfig);
}

void HardwareSerial::begin(uint32_t baud, const stc_usart_uart_init_t *config)
{
    // clear rx and tx buffers
    this->rxBuffer->clear();
    // this->txBuffer->clear();

    // set IO pin functions
    GPIO_SetFunc(this->tx_pin, this->config->peripheral.tx_pin_function);
    GPIO_SetFunc(this->rx_pin, this->config->peripheral.rx_pin_function);

    // enable peripheral clock
    PWC_Fcg1PeriphClockCmd(this->config->peripheral.clock_id, Enable);

    // initialize usart peripheral and set baud rate
    USART_UART_Init(this->config->peripheral.register_base, config);
    USART_SetBaudrate(this->config->peripheral.register_base, baud);

    // setup usart interrupts
    usart_irq_register(this->config->interrupts.rx_data_available, "usart rx data available");
    usart_irq_register(this->config->interrupts.rx_error, "usart rx error");
    usart_irq_register(this->config->interrupts.tx_buffer_empty, "usart tx buffer empty");
    usart_irq_register(this->config->interrupts.tx_complete, "usart tx complete");

    // enable usart RX + interrupts
    // (tx is enabled on-demand when data is available to send)
    USART_FuncCmd(this->config->peripheral.register_base, UsartRx, Enable);
    USART_FuncCmd(this->config->peripheral.register_base, UsartRxInt, Enable);

    // write debug message AFTER init (this UART may be used for the debug message)
    USART_DEBUG_PRINTF("begin completed\n");
    this->initialized = true;
}

void HardwareSerial::end()
{
    // write debug message BEFORE deinit (this UART may be used for the debug message)
    USART_DEBUG_PRINTF("end()\n");

    // wait for tx buffer to empty
    flush();

    // disable uart peripheral
    USART_FuncCmd(this->config->peripheral.register_base, UsartTx, Disable);
    USART_FuncCmd(this->config->peripheral.register_base, UsartRx, Disable);

    // resign usart interrupts
    usart_irq_resign(this->config->interrupts.rx_data_available, "usart rx data available");
    usart_irq_resign(this->config->interrupts.rx_error, "usart rx error");
    usart_irq_resign(this->config->interrupts.tx_buffer_empty, "usart tx buffer empty");
    usart_irq_resign(this->config->interrupts.tx_complete, "usart tx complete");

    // deinit uart
    USART_DeInit(this->config->peripheral.register_base);

    // clear rx and tx buffers
    this->rxBuffer->clear();
    this->txBuffer->clear();

    this->initialized = false;
}

int HardwareSerial::available(void)
{
  return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail)) % SERIAL_RX_BUFFER_SIZE;
}

int HardwareSerial::availableForWrite(void)
{
}

int HardwareSerial::peek(void)
{
}

int HardwareSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  } else {
    unsigned char c = HardwareSerial::_rx_buffer[_rx_buffer_tail];
    _rx_buffer_tail = (rx_buffer_index_t)(_rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE;
    return c;
  }
}

void HardwareSerial::flush(void)
{
    // ignore if not initialized
    if (!this->initialized)
    {
        return;
    }
}

size_t HardwareSerial::write(uint8_t ch)
{
    USART_SendData(this->config->peripheral.register_base, ch);

    // wrote one byte
    return 1;
}

const usart_receive_error_t Usart::getReceiveError()
{
    auto rxError = this->config->state.rx_error;
    this->config->state.rx_error = usart_receive_error_t::None;
    return rxError;
}

// Actual interrupt handlers //////////////////////////////////////////////////////////////

void HardwareSerial::_rx_complete_callback(unsigned char c)
{
    rx_buffer_index_t i = (unsigned int)(_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != _rx_buffer_tail) {
      _rx_buffer[_rx_buffer_head] = c;
      _rx_buffer_head = i;

    }
}
void HardwareSerial::set_buffer_head(rx_buffer_index_t index)
{
	if (index != _rx_buffer_tail) {
	_rx_buffer_head = index;
	}
}
