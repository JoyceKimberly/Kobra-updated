#include "usart_config.h"
#include "usart_handlers.h"
#include "../../WVariant.h"
#include "../../core_hooks.h"

// UART1
#define USART1_BAUDRATE                 (115200ul)
#define USART1_TX_PORT                  (PortA)  // Func_Grp1
#define USART1_TX_PIN                   (Pin09)
#define USART1_RX_PORT                  (PortA)  // Func_Grp1
#define USART1_RX_PIN                   (Pin15)
#define IRQ_INDEX_INT_USART1_RI         Int000_IRQn
#define IRQ_INDEX_INT_USART1_EI         Int001_IRQn
#define IRQ_INDEX_INT_USART1_TI         Int002_IRQn
#define IRQ_INDEX_INT_USART1_TCI        Int003_IRQn

// UART2
#define USART2_BAUDRATE                 (115200ul)
#define USART2_TX_PORT                  (PortA)  // Func_Grp1
#define USART2_TX_PIN                   (Pin02)
#define USART2_RX_PORT                  (PortA)  // Func_Grp1
#define USART2_RX_PIN                   (Pin03)
#define IRQ_INDEX_INT_USART2_RI         Int004_IRQn
#define IRQ_INDEX_INT_USART2_EI         Int005_IRQn
#define IRQ_INDEX_INT_USART2_TI         Int006_IRQn
#define IRQ_INDEX_INT_USART2_TCI        Int007_IRQn

// UART3
#define USART3_BAUDRATE                 (115200ul)
#define USART3_TX_PORT                  (PortA)  // Func_Grp1
#define USART3_TX_PIN                   (Pin11)
#define USART3_RX_PORT                  (PortA)  // Func_Grp1
#define USART3_RX_PIN                   (Pin12)
#define IRQ_INDEX_INT_USART3_RI         Int008_IRQn
#define IRQ_INDEX_INT_USART3_EI         Int009_IRQn
#define IRQ_INDEX_INT_USART3_TI         Int010_IRQn
#define IRQ_INDEX_INT_USART3_TCI        Int011_IRQn

// UART4
#define USART4_BAUDRATE                 (115200ul)
#define USART4_TX_PORT                  (PortB)  // Func_Grp2
#define USART4_TX_PIN                   (Pin10)
#define USART4_RX_PORT                  (PortH)  // Func_Grp2
#define USART4_RX_PIN                   (Pin02)
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

#define USART_RX_BUF_SIZE               128
#define USART_TX_BUF_SIZE               128

#ifndef SERIAL_BUFFER_SIZE
#define SERIAL_BUFFER_SIZE 64
#endif
#ifndef SERIAL_TX_BUFFER_SIZE
#define SERIAL_TX_BUFFER_SIZE SERIAL_BUFFER_SIZE
#endif
#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE SERIAL_BUFFER_SIZE
#endif

//
// USART configurations
//
usart_config_t USART1_config = {
    .peripheral = {
        .register_base = M4_USART1,
        .clock_id = PWC_FCG1_PERIPH_USART1,
    },
    .pins = {
        .tx_pin = USART1_TX_PIN,
        .rx_pin = USART1_RX_PIN,
    },
    .interrupts = {
        .rx_data_available = {
            .interrupt_number = IRQ_INDEX_INT_USART1_RI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART1_RI,
            .interrupt_handler = USARTx_rx_data_available_irq<1>,
        },
        .rx_error = {
            .interrupt_number = IRQ_INDEX_INT_USART1_EI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART1_EI,
            .interrupt_handler = USARTx_rx_error_irq<1>,
        },
        .tx_buffer_empty = {
            .interrupt_number = IRQ_INDEX_INT_USART1_TI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART1_TI,
            .interrupt_handler = USARTx_tx_buffer_empty_irq<1>,
        },
        .tx_complete = {
            .interrupt_number = IRQ_INDEX_INT_USART1_TCI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART1_TCI,
            .interrupt_handler = USARTx_tx_complete_irq<1>,
        },
    },
    .state = {
        .rx_buffer = new RingBuffer(SERIAL_RX_BUFFER_SIZE),
        .tx_buffer = new RingBuffer(SERIAL_TX_BUFFER_SIZE),
        .rx_error = usart_receive_error_t::None,
    },
};

usart_config_t USART2_config = {
    .peripheral = {
        .register_base = M4_USART2,
        .clock_id = PWC_FCG1_PERIPH_USART2,
    },
    .pins = {
        .tx_pin = USART2_TX_PIN,
        .rx_pin = USART2_RX_PIN,
    },
    .interrupts = {
        .rx_data_available = {
            .interrupt_number = IRQ_INDEX_INT_USART2_RI,
            .interrupt_priority = DDL_IRQ_PRIORITY_08,
            .interrupt_source = INT_USART2_RI,
            .interrupt_handler = USARTx_rx_data_available_irq<2>,
        },
        .rx_error = {
            .interrupt_number = IRQ_INDEX_INT_USART2_EI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART2_EI,
            .interrupt_handler = USARTx_rx_error_irq<2>,
        },
        .tx_buffer_empty = {
            .interrupt_number = IRQ_INDEX_INT_USART2_TI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART2_TI,
            .interrupt_handler = USARTx_tx_buffer_empty_irq<2>,
        },
        .tx_complete = {
            .interrupt_number = IRQ_INDEX_INT_USART2_TCI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART2_TCI,
            .interrupt_handler = USARTx_tx_complete_irq<2>,
        },
    },
    .state = {
        .rx_buffer = new RingBuffer(SERIAL_RX_BUFFER_SIZE),
        .tx_buffer = new RingBuffer(SERIAL_TX_BUFFER_SIZE),
        .rx_error = usart_receive_error_t::None,
    },
};

usart_config_t USART3_config = {
    .peripheral = {
        .register_base = M4_USART3,
        .clock_id = PWC_FCG1_PERIPH_USART3,
    },
    .pins = {
        .tx_pin = USART3_TX_PIN,
        .rx_pin = USART3_RX_PIN,
    },
    .interrupts = {
        .rx_data_available = {
            .interrupt_number = IRQ_INDEX_INT_USART3_RI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART3_RI,
            .interrupt_handler = USARTx_rx_data_available_irq<3>,
        },
        .rx_error = {
            .interrupt_number = IRQ_INDEX_INT_USART3_EI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART3_EI,
            .interrupt_handler = USARTx_rx_error_irq<3>,
        },
        .tx_buffer_empty = {
            .interrupt_number = IRQ_INDEX_INT_USART3_TI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART3_TI,
            .interrupt_handler = USARTx_tx_buffer_empty_irq<3>,
        },
        .tx_complete = {
            .interrupt_number = IRQ_INDEX_INT_USART3_TCI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART3_TCI,
            .interrupt_handler = USARTx_tx_complete_irq<3>,
        },
    },
    .state = {
        .rx_buffer = new RingBuffer(SERIAL_RX_BUFFER_SIZE),
        .tx_buffer = new RingBuffer(SERIAL_TX_BUFFER_SIZE),
        .rx_error = usart_receive_error_t::None,
    },
};

usart_config_t USART4_config = {
    .peripheral = {
        .register_base = M4_USART4,
        .clock_id = PWC_FCG1_PERIPH_USART4,
    },
    .pins = {
        .tx_pin = USART4_TX_PIN,
        .rx_pin = USART4_RX_PIN,
    },
    .interrupts = {
        .rx_data_available = {
            .interrupt_number = IRQ_INDEX_INT_USART4_RI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART4_RI,
            .interrupt_handler = USARTx_rx_data_available_irq<4>,
        },
        .rx_error = {
            .interrupt_number = IRQ_INDEX_INT_USART4_EI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART4_EI,
            .interrupt_handler = USARTx_rx_error_irq<4>,
        },
        .tx_buffer_empty = {
            .interrupt_number = IRQ_INDEX_INT_USART4_TI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART4_TI,
            .interrupt_handler = USARTx_tx_buffer_empty_irq<4>,
        },
        .tx_complete = {
            .interrupt_number = IRQ_INDEX_INT_USART4_TCI,
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART4_TCI,
            .interrupt_handler = USARTx_tx_complete_irq<4>,
        },
    },
    .state = {
        .rx_buffer = new RingBuffer(SERIAL_RX_BUFFER_SIZE),
        .tx_buffer = new RingBuffer(SERIAL_TX_BUFFER_SIZE),
        .rx_error = usart_receive_error_t::None,
    },
};
