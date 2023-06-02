/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <hc32_ddl.h>
#include <stdint.h>
#include "Print.h"
#include "Stream.h"
#include "hc32f460_usart.h"
#include "RingBuffer.h"
#include "usart_config.h"
#include "../../core_hooks.h"
#include <stddef.h>

#define HARDSER_PARITY_EVEN (0x1ul)
#define HARDSER_PARITY_ODD (0x2ul)
#define HARDSER_PARITY_NONE (0x3ul)
#define HARDSER_PARITY_MASK (0xFul)

#define HARDSER_STOP_BIT_1 (0x10ul)
// #define HARDSER_STOP_BIT_1_5 (0x20ul)
#define HARDSER_STOP_BIT_2 (0x30ul)
#define HARDSER_STOP_BIT_MASK (0xF0ul)

// #define HARDSER_DATA_5 (0x100ul)
// #define HARDSER_DATA_6 (0x200ul)
// #define HARDSER_DATA_7 (0x300ul)
#define HARDSER_DATA_8 (0x400ul)
#define HARDSER_DATA_MASK (0xF00ul)

#define SERIAL_5N1 (HARDSER_STOP_BIT_1 | HARDSER_PARITY_NONE | HARDSER_DATA_5)
#define SERIAL_6N1 (HARDSER_STOP_BIT_1 | HARDSER_PARITY_NONE | HARDSER_DATA_6)
#define SERIAL_7N1 (HARDSER_STOP_BIT_1 | HARDSER_PARITY_NONE | HARDSER_DATA_7)
#define SERIAL_8N1 (HARDSER_STOP_BIT_1 | HARDSER_PARITY_NONE | HARDSER_DATA_8)
#define SERIAL_5N2 (HARDSER_STOP_BIT_2 | HARDSER_PARITY_NONE | HARDSER_DATA_5)
#define SERIAL_6N2 (HARDSER_STOP_BIT_2 | HARDSER_PARITY_NONE | HARDSER_DATA_6)
#define SERIAL_7N2 (HARDSER_STOP_BIT_2 | HARDSER_PARITY_NONE | HARDSER_DATA_7)
#define SERIAL_8N2 (HARDSER_STOP_BIT_2 | HARDSER_PARITY_NONE | HARDSER_DATA_8)
#define SERIAL_5E1 (HARDSER_STOP_BIT_1 | HARDSER_PARITY_EVEN | HARDSER_DATA_5)
#define SERIAL_6E1 (HARDSER_STOP_BIT_1 | HARDSER_PARITY_EVEN | HARDSER_DATA_6)
#define SERIAL_7E1 (HARDSER_STOP_BIT_1 | HARDSER_PARITY_EVEN | HARDSER_DATA_7)
#define SERIAL_8E1 (HARDSER_STOP_BIT_1 | HARDSER_PARITY_EVEN | HARDSER_DATA_8)
#define SERIAL_5E2 (HARDSER_STOP_BIT_2 | HARDSER_PARITY_EVEN | HARDSER_DATA_5)
#define SERIAL_6E2 (HARDSER_STOP_BIT_2 | HARDSER_PARITY_EVEN | HARDSER_DATA_6)
#define SERIAL_7E2 (HARDSER_STOP_BIT_2 | HARDSER_PARITY_EVEN | HARDSER_DATA_7)
#define SERIAL_8E2 (HARDSER_STOP_BIT_2 | HARDSER_PARITY_EVEN | HARDSER_DATA_8)
#define SERIAL_5O1 (HARDSER_STOP_BIT_1 | HARDSER_PARITY_ODD | HARDSER_DATA_5)
#define SERIAL_6O1 (HARDSER_STOP_BIT_1 | HARDSER_PARITY_ODD | HARDSER_DATA_6)
#define SERIAL_7O1 (HARDSER_STOP_BIT_1 | HARDSER_PARITY_ODD | HARDSER_DATA_7)
#define SERIAL_8O1 (HARDSER_STOP_BIT_1 | HARDSER_PARITY_ODD | HARDSER_DATA_8)
#define SERIAL_5O2 (HARDSER_STOP_BIT_2 | HARDSER_PARITY_ODD | HARDSER_DATA_5)
#define SERIAL_6O2 (HARDSER_STOP_BIT_2 | HARDSER_PARITY_ODD | HARDSER_DATA_6)
#define SERIAL_7O2 (HARDSER_STOP_BIT_2 | HARDSER_PARITY_ODD | HARDSER_DATA_7)
#define SERIAL_8O2 (HARDSER_STOP_BIT_2 | HARDSER_PARITY_ODD | HARDSER_DATA_8)

#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 64

#if (SERIAL_TX_BUFFER_SIZE>256)
  typedef uint16_t tx_buffer_index_t;
#else
  typedef uint8_t tx_buffer_index_t;
#endif
#if  (SERIAL_RX_BUFFER_SIZE>256)
  typedef uint16_t rx_buffer_index_t;
#else
  typedef uint8_t rx_buffer_index_t;
#endif

class HardwareSerial : public Stream
{
public:
  unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
  unsigned char _tx_buffer[SERIAL_TX_BUFFER_SIZE];
  /**
   * @brief construct a new Usart object
   * @param config pointer to the usart configuration struct
   * @param tx_pin gpio pin number for tx function
   * @param rx_pin gpio pin number for rx function
   */
  HardwareSerial(struct usart_config_t *config, uint16_t tx_pin, uint16_t rx_pin);
  size_t begin(uint32_t baud);
  void begin(uint32_t baud, uint16_t config);
  void begin(uint32_t baud, const stc_usart_uart_init_t *config);
  void end();
  int available();
  int availableForWrite();
  int peek();
  int read();
  void flush();
  size_t write(uint8_t ch);
  using Print::write; // pull in write(str) and write(buf, size) from Print
  operator bool() { return true; }

  bool connected() {};
  void flushTX() { flush(); };
  void msgDone() {};

  /**
   * @brief access the base usart config struct
   */
  const usart_config_t *c_dev(void) { return this->config; }

  /**
   * @brief get the last receive error
   * @note calling this function clears the error
   */
  const usart_receive_error_t getReceiveError(void);

  // Interrupt handlers - Not intended to be called externally
  void _rx_complete_callback(unsigned char c);
  void set_buffer_head(rx_buffer_index_t index);

private:
  // usart configuration struct
  usart_config_t *config;

  // tx / rx pin numbers
  uint16_t tx_pin;
  uint16_t rx_pin;

  // rx / tx buffers (unboxed from config)
  RingBuffer *rxBuffer;
  RingBuffer *txBuffer;

  // is initialized? (begin() called)
  bool initialized = false;

protected:
  volatile rx_buffer_index_t _rx_buffer_head;
  volatile rx_buffer_index_t _rx_buffer_tail;
};

#endif
