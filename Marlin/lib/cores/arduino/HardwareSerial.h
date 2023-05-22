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

#include <inttypes.h>

#include "Stream.h"
#include "hc32f460_usart.h"

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.
// NOTE: a "power of 2" buffer size is recommended to dramatically
//       optimize all the modulo operations for ring buffers.
// WARNING: When buffer sizes are increased to > 256, the buffer index
// variables are automatically increased in size, but the extra
// atomicity guards needed for that are not implemented. This will
// often work, but occasionally a race condition can occur that makes
// Serial behave erratically. See https://github.com/arduino/Arduino/issues/2405
#if !defined(SERIAL_TX_BUFFER_SIZE)
  #define SERIAL_TX_BUFFER_SIZE 64
#endif
#if !defined(SERIAL_RX_BUFFER_SIZE)
  #define SERIAL_RX_BUFFER_SIZE 64
#endif
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
 
struct usart_dev;

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

/* Roger Clark 
 * Moved macros from hardwareSerial.cpp
 */
 
#define DEFINE_HWSERIAL(name, n)                                   \
	HardwareSerial name(USART##n,                                  \
						BOARD_USART##n##_TX_PIN,                   \
						BOARD_USART##n##_RX_PIN)

#define DEFINE_HWSERIAL_UART(name, n)                             \
	HardwareSerial name(UART##n,                                  \
						BOARD_USART##n##_TX_PIN,                   \
						BOARD_USART##n##_RX_PIN)				


/* Roger clark. Changed class inheritance from Print to Stream.
 * Also added new functions for peek() and availableForWrite()
 * Note. AvailableForWrite is only a stub function in the cpp
 */

class HardwareSerial : public Stream
{
public:
  HardwareSerial(struct usart_dev *usart_device,
                 uint8_t tx_pin,
                 uint8_t rx_pin);

    HardwareSerial(M4_USART_TypeDef *base);
    unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
    unsigned char _tx_buffer[SERIAL_TX_BUFFER_SIZE];
    unsigned char *g_rx_buffer;

  /* Set up/tear down */
  inline size_t begin(uint32_t baudrate) { return baudrate; }
  void end();
  virtual int available(void);
  int availableForWrite(void);
  virtual int peek(void);
  virtual int read(void);
  virtual void flush(void);
  virtual size_t write(uint8_t);
  using Print::write; // pull in write(str) and write(buf, size) from Print
  operator bool() { return true; };
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }

    /* Pin accessors */
    int txPin(void) { return this->tx_pin; }
    int rxPin(void) { return this->rx_pin; }

  // escape hatch to underlying usart_dev
  struct usart_dev *c_dev(void) { return this->usart_device; }

    bool connected() {};
    void flushTX() { flush(); };
    void msgDone() {};

    // Interrupt handlers - Not intended to be called externally
    void _rx_complete_callback(unsigned char c);
    void set_buffer_head(rx_buffer_index_t index);

private:
  struct usart_dev *usart_device;
  uint8_t tx_pin;
  uint8_t rx_pin;

  protected:
	M4_USART_TypeDef *uart_base;
    // Has any byte been written to the UART since begin()
    bool _written;

    volatile rx_buffer_index_t _rx_buffer_head;
    volatile rx_buffer_index_t _rx_buffer_tail;
#if 0  
    // Don't put any members after these buffers, since only the first
    // 32 bytes of this struct can be accessed quickly using the ldd
    // instruction.
    unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
    unsigned char _tx_buffer[SERIAL_TX_BUFFER_SIZE];	
#endif
};

#endif
