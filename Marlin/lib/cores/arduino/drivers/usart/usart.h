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
#include <ring_buffer.h>
#include "../../core_hooks.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * Register map (common across supported STM32 series).
 */

/** USART register map type */
typedef struct usart_reg_map {
    __IO uint32_t SR;             /**< Status register */
    __IO uint32_t DR;             /**< Data register */
    __IO uint32_t BRR;            /**< Baud rate register */
    __IO uint32_t CR1;            /**< Control register 1 */
    __IO uint32_t CR2;            /**< Control register 2 */
    __IO uint32_t CR3;            /**< Control register 3 */
    __IO uint32_t GTPR;           /**< Guard time and prescaler register */
} usart_reg_map;

/*
 * Register bit definitions
 */

/* Status register */

/** Clear to send bit */
#define USART_SR_CTS_BIT                9
/** Line break detection bit */
#define USART_SR_LBD_BIT                8
/** Transmit data register empty bit */
#define USART_SR_TXE_BIT                7
/** Transmission complete bit */
#define USART_SR_TC_BIT                 6
/** Read data register not empty bit */
#define USART_SR_RXNE_BIT               5
/** IDLE line detected bit */
#define USART_SR_IDLE_BIT               4
/** Overrun error bit */
#define USART_SR_ORE_BIT                3
/** Noise error bit */
#define USART_SR_NE_BIT                 2
/**
 * @brief Synonym for USART_SR_NE_BIT.
 *
 * Some series (e.g. STM32F2) use "NF" for "noise flag" instead of the
 * original "NE" for "noise error". The meaning of the bit is
 * unchanged, but the NF flag can be disabled when the line is
 * noise-free.
 *
 * @see USART_SR_NE_BIT
 */
#define USART_SR_NF_BIT                 USART_SR_NE_BIT
/** Framing error bit */
#define USART_SR_FE_BIT                 1
/** Parity error bit */
#define USART_SR_PE_BIT                 0

/** Clear to send mask */
#define USART_SR_CTS                    BIT(USART_SR_CTS_BIT)
/** Line break detected mask */
#define USART_SR_LBD                    BIT(USART_SR_LBD_BIT)
/** Transmit data register empty mask */
#define USART_SR_TXE                    BIT(USART_SR_TXE_BIT)
/** Transmission complete mask */
#define USART_SR_TC                     BIT(USART_SR_TC_BIT)
/** Read data register not empty mask */
#define USART_SR_RXNE                   BIT(USART_SR_RXNE_BIT)
/** IDLE line detected mask */
#define USART_SR_IDLE                   BIT(USART_SR_IDLE_BIT)
/** Overrun error mask */
#define USART_SR_ORE                    BIT(USART_SR_ORE_BIT)
/** Noise error mask */
#define USART_SR_NE                     BIT(USART_SR_NE_BIT)
/**
 * @brief Synonym for USART_SR_NE.
 * @see USART_SR_NF_BIT
 */
#define USART_SR_NF                     USART_SR_NE
/** Framing error mask */
#define USART_SR_FE                     BIT(USART_SR_FE_BIT)
/** Parity error mask */
#define USART_SR_PE                     BIT(USART_SR_PE_BIT)

/* Data register */

/** Data register data value mask */
#define USART_DR_DR                     0xFF

/* Baud rate register */

/** Mantissa of USARTDIV mask */
#define USART_BRR_DIV_MANTISSA          (0xFFF << 4)
/** Fraction of USARTDIV mask */
#define USART_BRR_DIV_FRACTION          0xF

/* Control register 1 */

/** USART enable bit */
#define USART_CR1_UE_BIT                13
/** Word length bit */
#define USART_CR1_M_BIT                 12
/** Wakeup method bit */
#define USART_CR1_WAKE_BIT              11
/** Parity control enable bit */
#define USART_CR1_PCE_BIT               10
/** Parity selection bit */
#define USART_CR1_PS_BIT                9
/** Parity error interrupt enable bit */
#define USART_CR1_PEIE_BIT              8
/** Transmit data regsiter not empty interrupt enable bit */
#define USART_CR1_TXEIE_BIT             7
/** Transmission complete interrupt enable bit */
#define USART_CR1_TCIE_BIT              6
/** RXNE interrupt enable bit */
#define USART_CR1_RXNEIE_BIT            5
/** IDLE interrupt enable bit */
#define USART_CR1_IDLEIE_BIT            4
/** Transmitter enable bit */
#define USART_CR1_TE_BIT                3
/** Receiver enable bit */
#define USART_CR1_RE_BIT                2
/** Receiver wakeup bit */
#define USART_CR1_RWU_BIT               1
/** Send break bit */
#define USART_CR1_SBK_BIT               0

/** USART enable mask */
#define USART_CR1_UE                    BIT(USART_CR1_UE_BIT)
/** Word length mask */
#define USART_CR1_M                     BIT(USART_CR1_M_BIT)
/** Word length: 1 start bit, 8 data bits, n stop bit */
#define USART_CR1_M_8N1                 (0 << USART_CR1_M_BIT)
/** Word length: 1 start bit, 9 data bits, n stop bit */
#define USART_CR1_M_9N1                 (1 << USART_CR1_M_BIT)
/** Wakeup method mask */
#define USART_CR1_WAKE                  BIT(USART_CR1_WAKE_BIT)
/** Wakeup on idle line */
#define USART_CR1_WAKE_IDLE             (0 << USART_CR1_WAKE_BIT)
/** Wakeup on address mark */
#define USART_CR1_WAKE_ADDR             (1 << USART_CR1_WAKE_BIT)
/** Parity control enable mask */
#define USART_CR1_PCE                   BIT(USART_CR1_PCE_BIT)
/** Parity selection mask */
#define USART_CR1_PS                    BIT(USART_CR1_PS_BIT)
/** Parity selection: even parity */
#define USART_CR1_PS_EVEN               (0 << USART_CR1_PS_BIT)
/** Parity selection: odd parity */
#define USART_CR1_PS_ODD                (1 << USART_CR1_PS_BIT)
/** Parity error interrupt enable mask */
#define USART_CR1_PEIE                  BIT(USART_CR1_PEIE_BIT)
/** Transmit data register empty interrupt enable mask */
#define USART_CR1_TXEIE                 BIT(USART_CR1_TXEIE_BIT)
/** Transmission complete interrupt enable mask */
#define USART_CR1_TCIE                  BIT(USART_CR1_TCIE_BIT)
/** RXNE interrupt enable mask */
#define USART_CR1_RXNEIE                BIT(USART_CR1_RXNEIE_BIT)
/** IDLE line interrupt enable mask */
#define USART_CR1_IDLEIE                BIT(USART_CR1_IDLEIE_BIT)
/** Transmitter enable mask */
#define USART_CR1_TE                    BIT(USART_CR1_TE_BIT)
/** Receiver enable mask */
#define USART_CR1_RE                    BIT(USART_CR1_RE_BIT)
/** Receiver wakeup mask */
#define USART_CR1_RWU                   BIT(USART_CR1_RWU_BIT)
/** Receiver wakeup: receiver in active mode */
#define USART_CR1_RWU_ACTIVE            (0 << USART_CR1_RWU_BIT)
/** Receiver wakeup: receiver in mute mode */
#define USART_CR1_RWU_MUTE              (1 << USART_CR1_RWU_BIT)
/** Send break */
#define USART_CR1_SBK                   BIT(USART_CR1_SBK_BIT)

/* Control register 2 */

/** LIN mode enable bit */
#define USART_CR2_LINEN_BIT             14
/** Clock enable bit */
#define USART_CR2_CLKEN_BIT             11
/** Clock polarity bit */
#define USART_CR2_CPOL_BIT              10
/** Clock phase bit */
#define USART_CR2_CPHA_BIT              9
/** Last bit clock pulse bit */
#define USART_CR2_LBCL_BIT              8
/** LIN break detection interrupt enable bit */
#define USART_CR2_LBDIE_BIT             6
/** LIN break detection length bit */
#define USART_CR2_LBDL_BIT              5

/** LIN mode enable mask */
#define USART_CR2_LINEN                 BIT(USART_CR2_LINEN_BIT)
/** STOP bits mask */
#define USART_CR2_STOP                  (0x3 << 12)
/** STOP bits: 1 stop bit */
#define USART_CR2_STOP_BITS_1           (0x0 << 12)
/**
 * @brief STOP bits: 0.5 stop bits
 * Not available  on UART4, UART5. */
#define USART_CR2_STOP_BITS_POINT_5     (0x1 << 12)
/** STOP bits: 2 stop bits */
#define USART_CR2_STOP_BITS_2           (0x2 << 12)
/**
 * @brief STOP bits: 1.5 stop bits
 * Not available  on UART4, UART5. */
#define USART_CR2_STOP_BITS_1_POINT_5   (0x3 << 12)
/**
 * @brief Clock enable.
 * Not available on UART4, UART5 */
#define USART_CR2_CLKEN                 BIT(USART_CR2_CLKEN_BIT)
/**
 * @brief Clock polarity mask.
 * Not available on UART4, UART5 */
#define USART_CR2_CPOL                  BIT(USART_CR2_CPOL_BIT)
/** Clock polarity: low */
#define USART_CR2_CPOL_LOW              (0x0 << USART_CR2_CLKEN_BIT)
/** Clock polarity: high */
#define USART_CR2_CPOL_HIGH             (0x1 << USART_CR2_CLKEN_BIT)
/**
 * @brief Clock phase mask.
 * Not available on UART4, UART5 */
#define USART_CR2_CPHA                  BIT(USART_CR2_CPHA_BIT)
/**
 * @brief Clock phase: first
 * First clock transition is the first data capture edge. */
#define USART_CR2_CPHA_FIRST            (0x0 << USART_CR2_CPHA_BIT)
/**
 * @brief Clock phase: second
 * Second clock transition is the first data capture edge. */
#define USART_CR2_CPHA_SECOND           (0x1 << USART_CR2_CPHA_BIT)
/**
 * @brief Last bit clock pulse mask.
 *
 * When set, the last bit transmitted causes a clock pulse in
 * synchronous mode.
 *
 * Not available on UART4, UART5 */
#define USART_CR2_LBCL                  BIT(USART_CR2_LBCL_BIT)
/** LIN break detection interrupt enable mask. */
#define USART_CR2_LBDIE                 BIT(USART_CR2_LBDIE_BIT)
/** LIN break detection length. */
#define USART_CR2_LBDL                  BIT(USART_CR2_LBDL_BIT)
/** LIN break detection length: 10 bits */
#define USART_CR2_LBDL_10_BIT           (0 << USART_CR2_LBDL_BIT)
/** LIN break detection length: 11 bits */
#define USART_CR2_LBDL_11_BIT           (1 << USART_CR2_LBDL_BIT)
/**
 * @brief Address of the USART node
 * This is useful during multiprocessor communication. */
#define USART_CR2_ADD                   0xF

/* Control register 3 */

/** Clear to send interrupt enable bit */
#define USART_CR3_CTSIE_BIT             10
/** Clear to send enable bit */
#define USART_CR3_CTSE_BIT              9
/** Ready to send enable bit */
#define USART_CR3_RTSE_BIT              8
/** DMA enable transmitter bit */
#define USART_CR3_DMAT_BIT              7
/** DMA enable receiver bit */
#define USART_CR3_DMAR_BIT              6
/** Smartcard mode enable bit */
#define USART_CR3_SCEN_BIT              5
/** Smartcard NACK enable bit */
#define USART_CR3_NACK_BIT              4
/** Half-duplex selection bit */
#define USART_CR3_HDSEL_BIT             3
/** IrDA low power bit */
#define USART_CR3_IRLP_BIT              2
/** IrDA mode enable bit */
#define USART_CR3_IREN_BIT              1
/** Error interrupt enable bit */
#define USART_CR3_EIE_BIT               0

/**
 * @brief Clear to send interrupt enable
 * Not available on UART4, UART5. */
#define USART_CR3_CTSIE                 BIT(USART_CR3_CTSIE_BIT)
/**
 * @brief Clear to send enable
 * Not available on UART4, UART5. */
#define USART_CR3_CTSE                  BIT(USART_CR3_CTSE_BIT)
/**
 * @brief Ready to send enable
 * Not available on UART4, UART5. */
#define USART_CR3_RTSE                  BIT(USART_CR3_RTSE_BIT)
/**
 * @brief DMA enable transmitter
 * Not available on UART5. */
#define USART_CR3_DMAT                  BIT(USART_CR3_DMAT_BIT)
/**
 * @brief DMA enable receiver
 * Not available on UART5. */
#define USART_CR3_DMAR                  BIT(USART_CR3_DMAR_BIT)
/**
 * @brief Smartcard mode enable
 * Not available on UART4, UART5. */
#define USART_CR3_SCEN                  BIT(USART_CR3_SCEN_BIT)
/**
 * @brief Smartcard NACK enable
 * Not available on UART4, UART5. */
#define USART_CR3_NACK                  BIT(USART_CR3_NACK_BIT)
/**
 * @brief Half-duplex selection
 * When set, single-wire half duplex mode is selected.
 */
#define USART_CR3_HDSEL                 BIT(USART_CR3_HDSEL_BIT)
/** IrDA low power mode */
#define USART_CR3_IRLP                  BIT(USART_CR3_IRLP_BIT)
/** IrDA mode: normal */
#define USART_CR3_IRLP_NORMAL           (0U << USART_CR3_IRLP_BIT)
/** IrDA mode: low power */
#define USART_CR3_IRLP_LOW_POWER        (1U << USART_CR3_IRLP_BIT)
/** IrDA mode enable */
#define USART_CR3_IREN                  BIT(USART_CR3_IREN_BIT)
/** Error interrupt enable */
#define USART_CR3_EIE                   BIT(USART_CR3_EIE_BIT)

/* Guard time and prescaler register */

/**
 * @brief Guard time value mask
 * Used in Smartcard mode. Not available on UART4, UART5. */
#define USART_GTPR_GT                   (0xFF << 8)
/**
 * @brief Prescaler value mask
 * Restrictions on this value apply, depending on the USART mode. Not
 * available on UART4, UART5. */
#define USART_GTPR_PSC                  0xFF

#define IRQ_INDEX_USART1_INT_RI         Int000_IRQn
#define IRQ_INDEX_USART1_INT_EI         Int001_IRQn
#define IRQ_INDEX_USART1_INT_TI         Int002_IRQn
#define IRQ_INDEX_USART1_INT_TCI        Int003_IRQn

#define IRQ_INDEX_USART2_INT_RI         Int004_IRQn
#define IRQ_INDEX_USART2_INT_EI         Int005_IRQn
#define IRQ_INDEX_USART2_INT_TI         Int006_IRQn
#define IRQ_INDEX_USART2_INT_TCI        Int007_IRQn

#define IRQ_INDEX_USART3_INT_RI         Int008_IRQn
#define IRQ_INDEX_USART3_INT_EI         Int009_IRQn
#define IRQ_INDEX_USART3_INT_TI         Int010_IRQn
#define IRQ_INDEX_USART3_INT_TCI        Int011_IRQn

#define IRQ_INDEX_USART4_INT_RI         Int012_IRQn
#define IRQ_INDEX_USART4_INT_EI         Int013_IRQn
#define IRQ_INDEX_USART4_INT_TI         Int014_IRQn
#define IRQ_INDEX_USART4_INT_TCI        Int015_IRQn

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


void BSP_USART1_RIIrqHander(void);
void BSP_USART1_EIIrqHander(void);
void BSP_USART1_TIrqHander(void);
void BSP_USART1_TCIIrqHander(void);

void BSP_USART2_RIIrqHander(void);
void BSP_USART2_EIIrqHander(void);
void BSP_USART2_TIrqHander(void);
void BSP_USART2_TCIIrqHander(void);

void BSP_USART3_RIIrqHander(void);
void BSP_USART3_EIIrqHander(void);
void BSP_USART3_TIrqHander(void);
void BSP_USART3_TCIIrqHander(void);

void BSP_USART4_RIIrqHander(void);
void BSP_USART4_EIIrqHander(void);
void BSP_USART4_TIrqHander(void);
void BSP_USART4_TCIIrqHander(void);

/*
 * Devices
 */

#define USART_RX_BUF_SIZE               128
#define USART_TX_BUF_SIZE               128

    /** USART device type */
    typedef struct usart_dev
    {
        M4_USART_TypeDef *regs;             /**< Register map */
        ring_buffer *rb;                 /**< RX ring buffer */
        ring_buffer *wb;                 /**< TX ring buffer */
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
                                     struct gpio_dev *rx_dev, uint8_t rx,
                                     struct gpio_dev *tx_dev, uint8_t tx,
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
    static inline uint32_t usart_data_available(usart_dev *dev)
    {
        return rb_full_count(dev->rb);
    }

    /**
     * @brief Discard the contents of a serial port's RX buffer.
     * @param dev Serial port whose buffer to empty.
     */
    static inline void usart_reset_rx(usart_dev *dev)
    {
        rb_reset(dev->rb);
    }

    /**
     * @brief Discard the contents of a serial port's RX buffer.
     * @param dev Serial port whose buffer to empty.
     */
    static inline void usart_reset_tx(usart_dev *dev)
    {
        rb_reset(dev->wb);
    }

    static inline void usart_tx_irq(ring_buffer *wb,M4_USART_TypeDef *regs)
    {
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

    static inline void usart_rx_irq(ring_buffer *rb,M4_USART_TypeDef *regs)
    {
        rb_push_insert(rb, (uint8_t)USART_RecData(regs));
    }

#ifdef __cplusplus
} // extern "C"
#endif
