/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
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
 * @file libmaple/include/libmaple/usart.h
 * @author Marti Bolivar <mbolivar@leaflabs.com>,
 *         Perry Hung <perry@leaflabs.com>
 * @brief USART definitions and prototypes
 */

#pragma once
#include <startup.h>
#include <hc32_ddl.h>
#include "RingBuffer.h"
#include "../../core_hooks.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define IRQ_INDEX_INT_USART1_RI         Int000_IRQn
#define IRQ_INDEX_INT_USART1_EI         Int001_IRQn
#define IRQ_INDEX_INT_USART1_TI         Int002_IRQn
#define IRQ_INDEX_INT_USART1_TCI        Int003_IRQn

#define IRQ_INDEX_INT_USART2_RI         Int004_IRQn
#define IRQ_INDEX_INT_USART2_EI         Int005_IRQn
#define IRQ_INDEX_INT_USART2_TI         Int006_IRQn
#define IRQ_INDEX_INT_USART2_TCI        Int007_IRQn

#define IRQ_INDEX_INT_USART3_RI         Int008_IRQn
#define IRQ_INDEX_INT_USART3_EI         Int009_IRQn
#define IRQ_INDEX_INT_USART3_TI         Int010_IRQn
#define IRQ_INDEX_INT_USART3_TCI        Int011_IRQn

#define IRQ_INDEX_INT_USART4_RI         Int012_IRQn
#define IRQ_INDEX_INT_USART4_EI         Int013_IRQn
#define IRQ_INDEX_INT_USART4_TI         Int014_IRQn
#define IRQ_INDEX_INT_USART4_TCI        Int015_IRQn

#define IRQ_INDEX_INT_DMA2_TC0          Int016_IRQn
#define IRQ_INDEX_INT_DMA2_TC1          Int017_IRQn
#define IRQ_INDEX_INT_DMA2_TC2          Int018_IRQn

#define IRQ_INDEX_INT_TMR01_GCMA        Int019_IRQn
#define IRQ_INDEX_INT_TMR01_GCMB        Int020_IRQn

#define IRQ_INDEX_INT_TMR02_GCMA        Int021_IRQn
#define IRQ_INDEX_INT_TMR02_GCMB        Int022_IRQn

#define IRQ_INDEX_INT_TMR41_GCMB        Int023_IRQn
#define IRQ_INDEX_INT_TMR42_GCMB        Int024_IRQn


extern uint8_t g_uart2_rx_buf[128];
extern uint8_t g_uart2_rx_index;


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
    void usart_set_stop_bits(usart_dev *dev, en_usart_stop_bit_t stop_bits);
    void usart_set_parity(usart_dev *dev, en_usart_parity_t parity);
    void usart_set_data_width(usart_dev *dev, en_usart_data_len_t data_width);
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
