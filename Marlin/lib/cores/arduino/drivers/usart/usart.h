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

#pragma once
#include <startup.h>
#include <hc32_ddl.h>
#include <stdint.h>
#include "HardwareSerial.h"
#include "RingBuffer.h"
#include "usart_config.h"
#include "../../core_hooks.h"
#include <stddef.h>

class Usart : public HardwareSerial
{
public:
  Usart(struct usart_config_t *config);
  void begin(uint32_t baud);
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
  void IrqHandler();

  /**
   * @brief access the base usart config struct
   */
  const usart_config_t *c_dev(void) { return this->config; }

  /**
   * @brief get the last receive error
   * @note calling this function clears the error
   */
  const usart_receive_error_t getReceiveError(void);

private:
  // usart configuration struct
  usart_config_t *config;

  // rx / tx buffers (unboxed from config)
  RingBuffer *rxBuffer;
  RingBuffer *txBuffer;
};

//
// global instances
//
#ifndef DISABLE_SERIAL_GLOBALS
extern HardwareSerial MSerial1;
extern HardwareSerial MSerial2;
extern HardwareSerial MSerial3;
extern HardwareSerial MSerial4;

#define Serial MSerial1
#endif

#ifdef __cplusplus
extern "C"
{
#endif

void uart1_init(void);
void Usart1RxIrqCallback(void);
void Usart1ErrIrqCallback(void);
void Usart1TxIrqCallback(void);
void Usart1TxCmpltIrqCallback(void);

void uart2_init(void);
void Usart2RxIrqCallback(void);
void Usart2ErrIrqCallback(void);
void Usart2TxIrqCallback(void);
void Usart2TxCmpltIrqCallback(void);

void uart3_init(void);
void Usart3RxIrqCallback(void);
void Usart3ErrIrqCallback(void);
void Usart3TxIrqCallback(void);
void Usart3TxCmpltIrqCallback(void);

void uart4_init(void);
void Usart4RxIrqCallback(void);
void Usart4ErrIrqCallback(void);
void Usart4TxIrqCallback(void);
void Usart4TxCmpltIrqCallback(void);

/*
 * Devices
 */
#define USART_RX_BUF_SIZE               128
#define USART_TX_BUF_SIZE               128

    /** USART device type */
    typedef struct usart_dev
    {
        M4_USART_TypeDef *regs;             /**< Register map */
        RingBuffer *rb;                 /**< RX ring buffer */
        RingBuffer *wb;                 /**< TX ring buffer */
        uint32_t max_baud;                 /**< @brief Deprecated.
                                        * Maximum baud rate. */
        uint8_t rx_buf[USART_RX_BUF_SIZE]; /**< @brief Deprecated.
                                        * Actual RX buffer used by rb.
                                        * This field will be removed in
                                        * a future release. */
        uint8_t tx_buf[USART_TX_BUF_SIZE]; /**< Actual TX buffer used by wb */
        uint32_t clk_id;
        stc_usart_uart_init_t *pstcInitCfg;
        IRQn_Type RX_IRQ;
        IRQn_Type TX_IRQ;
        IRQn_Type RX_error_IRQ;
        IRQn_Type TX_complete_IRQ;
        uint32_t IRQ_priority;
    } usart_dev;

    // usart device variables
#define USART_DEV_VARS(nr)      \
    extern usart_dev usart##nr; \
    extern struct usart_dev *USART##nr;

    USART_DEV_VARS(1)
    USART_DEV_VARS(2)
    USART_DEV_VARS(3)
    USART_DEV_VARS(4)

    // public api
    void usart_init(usart_dev *dev);
    void usart_set_baud_rate(usart_dev *dev, uint32_t baud);
    void usart_enable(usart_dev *dev);
    void usart_disable(usart_dev *dev);
    void usart_foreach(void (*fn)(usart_dev *dev));
    uint32_t usart_tx(usart_dev *dev, const uint8_t *buf, uint32_t len);
    uint32_t usart_rx(usart_dev *dev, uint8_t *buf, uint32_t len);
    void usart_putudec(usart_dev *dev, uint32_t val);

    /**
     * @brief Disable all serial ports.
     */
    static inline void usart_disable_all(void)
    {
        usart_foreach(usart_disable);
    }

    /**
     * @brief Transmit one character on a serial port.
     *
     * This function blocks until the character has been queued
     * for transmission.
     *
     * @param dev Serial port to send on.
     * @param byte Byte to transmit.
     */
    static inline void usart_putc(usart_dev *dev, uint8_t byte)
    {
        while (!usart_tx(dev, &byte, 1));
    }

    /**
     * @brief Transmit a character string on a serial port.
     *
     * This function blocks until str is completely transmitted.
     *
     * @param dev Serial port to send on
     * @param str String to send
     */
    static inline void usart_putstr(usart_dev *dev, const char *str)
    {
        uint32_t i = 0;
        while (str[i] != '\0')
        {
            usart_putc(dev, str[i++]);
        }
    }

    /**
     * @brief Read one character from a serial port.
     *
     * It's not safe to call this function if the serial port has no data
     * available.
     *
     * @param dev Serial port to read from
     * @return byte read
     * @see usart_data_available()
     */
    static inline uint8_t usart_getc(usart_dev *dev)
    {
        return dev->rb->_pop();
    }

    /*
     * Roger Clark. 20141125,
     * added peek function.
     * @param dev Serial port to read from
     * @return byte read
     */
    static inline int usart_peek(usart_dev *dev)
    {
        return dev->rb->peek();
    }

    /**
     * @brief Return the amount of data available in a serial port's RX buffer.
     * @param dev Serial port to check
     * @return Number of bytes in dev's RX buffer.
     */
    static inline uint32_t usart_data_available(usart_dev *dev)
    {
        return dev->rb->count();
    }

    /**
     * @brief Discard the contents of a serial port's RX buffer.
     * @param dev Serial port whose buffer to empty.
     */
    static inline void usart_reset_rx(usart_dev *dev)
    {
        dev->rb->clear();
    }

    /**
     * @brief Discard the contents of a serial port's RX buffer.
     * @param dev Serial port whose buffer to empty.
     */
    static inline void usart_reset_tx(usart_dev *dev)
    {
        dev->wb->clear();
    }

    /**
     * map usart device registers to usart channel number
     */
    static inline uint8_t usart_dev_to_channel(M4_USART_TypeDef *dev_regs)
    {
        if (dev_regs == M4_USART1)
            return 1;
        if (dev_regs == M4_USART2)
            return 2;
        if (dev_regs == M4_USART3)
            return 3;
        if (dev_regs == M4_USART4)
            return 4;

        return 0xff;
    }

    static inline void usart_tx_irq(usart_dev *dev)
    {
        uint8_t ch;
        if (dev->wb->pop(ch))
        {
            core_hook_usart_tx_irq(ch, usart_dev_to_channel(dev->regs));
            USART_SendData(dev->regs, ch);
        }
        else
        {
            USART_FuncCmd(dev->regs, UsartTxEmptyInt, Disable);
            USART_FuncCmd(dev->regs, UsartTxCmpltInt, Enable);
        }
    }

    static inline void usart_rx_irq(usart_dev *dev)
    {
        uint8_t ch = (uint8_t)USART_RecData(dev->regs);
        core_hook_usart_rx_irq(ch, usart_dev_to_channel(dev->regs));
        dev->rb->push(ch, true);
    }

#ifdef __cplusplus
} // extern "C"
#endif
