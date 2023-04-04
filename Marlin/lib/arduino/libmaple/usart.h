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

#ifndef _LIBMAPLE_USART_H_
#define _LIBMAPLE_USART_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "../drivers/board/startup.h"
#include <libmaple/libmaple_types.h>
#include <libmaple/ring_buffer.h>

/*
 * Devices
 */

#define USART_RX_BUF_SIZE               128
#define USART_TX_BUF_SIZE               128

/** USART device type */
typedef struct usart_dev {
    M4_USART_TypeDef *regs;             /**< Register map */
    ring_buffer *rb;                 /**< RX ring buffer */
    ring_buffer *wb;                 /**< TX ring buffer */
    uint32 max_baud;                 /**< @brief Deprecated.
                                      * Maximum baud rate. */
    uint8 rx_buf[USART_RX_BUF_SIZE]; /**< @brief Deprecated.
                                      * Actual RX buffer used by rb.
                                      * This field will be removed in
                                      * a future release. */
    uint8 tx_buf[USART_TX_BUF_SIZE]; /**< Actual TX buffer used by wb */
    uint32 clk_id;
    stc_usart_uart_init_t *pstcInitCfg;
    IRQn_Type RX_IRQ;
    IRQn_Type TX_IRQ;           
    IRQn_Type RX_error_IRQ;
    IRQn_Type TX_complete_IRQ;           
    uint32_t IRQ_priority;
} usart_dev;

void usart_init(usart_dev *dev);

struct gpio_dev;                /* forward declaration */
/* FIXME [PRE 0.0.13] decide if flags are necessary */
/**
 * @brief Configure GPIOs for use as USART TX/RX.
 * @param udev USART device to use
 * @param rx_dev RX pin gpio_dev
 * @param rx     RX pin bit on rx_dev
 * @param tx_dev TX pin gpio_dev
 * @param tx     TX pin bit on tx_dev
 * @param flags  Currently ignored
 */
extern void usart_config_gpios_async(usart_dev *udev,
                                     struct gpio_dev *rx_dev, uint8 rx,
                                     struct gpio_dev *tx_dev, uint8 tx,
                                     unsigned flags);


struct usart_dev;
extern usart_dev usart4;
extern usart_dev usart3;
extern usart_dev usart2;
extern usart_dev usart1;
extern struct usart_dev *USART1;
extern struct usart_dev *USART2;
extern struct usart_dev *USART3;
extern struct usart_dev *USART4;

extern ring_buffer usart1_rb;
extern ring_buffer usart1_wb;
extern ring_buffer usart2_rb;
extern ring_buffer usart2_wb;
extern ring_buffer usart3_rb;
extern ring_buffer usart3_wb;
extern ring_buffer usart4_rb;
extern ring_buffer usart4_wb;

void usart_set_baud_rate(usart_dev *dev, uint32 baud);

void usart_enable(usart_dev *dev);
void usart_disable(usart_dev *dev);
void usart_foreach(void (*fn)(usart_dev *dev));
uint32 usart_tx(usart_dev *dev, const uint8 *buf, uint32 len);
uint32 usart_rx(usart_dev *dev, uint8 *buf, uint32 len);
void usart_putudec(usart_dev *dev, uint32 val);

/**
 * @brief Disable all serial ports.
 */
static inline void usart_disable_all(void) {
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
static inline void usart_putc(usart_dev* dev, uint8 byte) {
    while (!usart_tx(dev, &byte, 1))
        ;
}

/**
 * @brief Transmit a character string on a serial port.
 *
 * This function blocks until str is completely transmitted.
 *
 * @param dev Serial port to send on
 * @param str String to send
 */
static inline void usart_putstr(usart_dev *dev, const char* str) {
    uint32 i = 0;
    while (str[i] != '\0') {
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
static inline uint8 usart_getc(usart_dev *dev) {
    return rb_remove(dev->rb);
}

/*
 * Roger Clark. 20141125, 
 * added peek function.
 * @param dev Serial port to read from
 * @return byte read
 */
static inline int usart_peek(usart_dev *dev)
{
	return rb_peek(dev->rb);
}


/**
 * @brief Return the amount of data available in a serial port's RX buffer.
 * @param dev Serial port to check
 * @return Number of bytes in dev's RX buffer.
 */
static inline uint32 usart_data_available(usart_dev *dev) {
    return rb_full_count(dev->rb);
}

/**
 * @brief Discard the contents of a serial port's RX buffer.
 * @param dev Serial port whose buffer to empty.
 */
static inline void usart_reset_rx(usart_dev *dev) {
    rb_reset(dev->rb);
}

/**
 * @brief Discard the contents of a serial port's RX buffer.
 * @param dev Serial port whose buffer to empty.
 */
static inline void usart_reset_tx(usart_dev *dev) {
    rb_reset(dev->wb);
}

static inline __always_inline void usart_tx_irq(ring_buffer *wb,M4_USART_TypeDef *regs) {
        if (!rb_is_empty(wb))
        {
		USART_SendData(regs,rb_remove(wb)); 
        }
        else
        {
		USART_FuncCmd(regs, UsartTxEmptyInt, Disable);
		USART_FuncCmd(regs, UsartTxCmpltInt, Enable);
        }            
}
static inline __always_inline void usart_rx_irq(ring_buffer *rb,M4_USART_TypeDef *regs) {
        rb_push_insert(rb, (uint8)USART_RecData(regs));
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif
