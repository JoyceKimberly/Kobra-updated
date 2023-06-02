#include "usart.h"
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
        .tx_pin_function = Func_Usart1_Tx,
        .rx_pin_function = Func_Usart1_Rx,
    },
    .interrupts = {
        .rx_data_available = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART1_RI,
            .interrupt_handler = USARTx_rx_data_available_irq<1>,
        },
        .rx_error = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART1_EI,
            .interrupt_handler = USARTx_rx_error_irq<1>,
        },
        .tx_buffer_empty = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART1_TI,
            .interrupt_handler = USARTx_tx_buffer_empty_irq<1>,
        },
        .tx_complete = {
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
        .tx_pin_function = Func_Usart2_Tx,
        .rx_pin_function = Func_Usart2_Rx,
    },
    .interrupts = {
        .rx_data_available = {
            .interrupt_priority = DDL_IRQ_PRIORITY_08,
            .interrupt_source = INT_USART2_RI,
            .interrupt_handler = USARTx_rx_data_available_irq<2>,
        },
        .rx_error = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART2_EI,
            .interrupt_handler = USARTx_rx_error_irq<2>,
        },
        .tx_buffer_empty = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART2_TI,
            .interrupt_handler = USARTx_tx_buffer_empty_irq<2>,
        },
        .tx_complete = {
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
        .tx_pin_function = Func_Usart3_Tx,
        .rx_pin_function = Func_Usart3_Rx,
    },
    .interrupts = {
        .rx_data_available = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART3_RI,
            .interrupt_handler = USARTx_rx_data_available_irq<3>,
        },
        .rx_error = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART3_EI,
            .interrupt_handler = USARTx_rx_error_irq<3>,
        },
        .tx_buffer_empty = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART3_TI,
            .interrupt_handler = USARTx_tx_buffer_empty_irq<3>,
        },
        .tx_complete = {
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
        .tx_pin_function = Func_Usart3_Tx,
        .rx_pin_function = Func_Usart3_Rx,
    },
    .interrupts = {
        .rx_data_available = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART4_RI,
            .interrupt_handler = USARTx_rx_data_available_irq<4>,
        },
        .rx_error = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART4_EI,
            .interrupt_handler = USARTx_rx_error_irq<4>,
        },
        .tx_buffer_empty = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART4_TI,
            .interrupt_handler = USARTx_tx_buffer_empty_irq<4>,
        },
        .tx_complete = {
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

//
// USART1
//
void uart1_init(void)
{
    en_result_t enRet = Ok;
    const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkNoOutput,
        UsartClkDiv_16,
        UsartDataBits8,
        UsartDataLsbFirst,
        UsartOneStopBit,
        UsartParityNone,
        UsartSampleBit8,
        UsartStartBitFallEdge,
        UsartRtsEnable,
    };

    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART1, Enable);
    
    PORT_SetFunc(USART1_RX_PORT, USART1_RX_PIN, Func_Usart1_Rx, Disable);
    PORT_SetFunc(USART1_TX_PORT, USART1_TX_PIN, Func_Usart1_Tx, Disable);
    
    enRet = USART_UART_Init(M4_USART1, &stcInitCfg);
    if (enRet != Ok) {
        while(1);
    }
    
    enRet = USART_SetBaudrate(M4_USART1, USART1_BAUDRATE);
    if (enRet != Ok) {
        while(1);
    }

    stc_irq_regi_conf_t irqConf;
    irqConf.enIRQn = IRQ_INDEX_INT_USART1_RI;
    irqConf.pfnCallback = &Usart1RxIrqCallback;
    irqConf.enIntSrc = INT_USART1_RI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    irqConf.enIRQn = IRQ_INDEX_INT_USART1_EI;
    irqConf.pfnCallback = &Usart1ErrIrqCallback;
    irqConf.enIntSrc = INT_USART1_EI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    irqConf.enIRQn = IRQ_INDEX_INT_USART1_TI;
    irqConf.pfnCallback = &Usart1TxIrqCallback;
    irqConf.enIntSrc = INT_USART1_TI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    irqConf.enIRQn = IRQ_INDEX_INT_USART1_TCI;
    irqConf.pfnCallback = &Usart1TxCmpltIrqCallback;
    irqConf.enIntSrc = INT_USART1_TCI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    USART_FuncCmd(M4_USART1, UsartRx, Enable);
    USART_FuncCmd(M4_USART1, UsartTx, Enable);
    
    USART_FuncCmd(M4_USART1, UsartRxInt, Enable);
//  USART_FuncCmd(M4_USART1, UsartTxCmpltInt, Enable);
}

//
// USART2
//
void uart2_init(void)
{
    en_result_t enRet = Ok;
    const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkNoOutput,
        UsartClkDiv_16,
        UsartDataBits8,
        UsartDataLsbFirst,
        UsartOneStopBit,
        UsartParityNone,
        UsartSampleBit8,
        UsartStartBitFallEdge,
        UsartRtsEnable,
    };

    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART2, Enable);
    
    PORT_SetFunc(USART2_RX_PORT, USART2_RX_PIN, Func_Usart2_Rx, Disable);
    PORT_SetFunc(USART2_TX_PORT, USART2_TX_PIN, Func_Usart2_Tx, Disable);
    
    enRet = USART_UART_Init(M4_USART2, &stcInitCfg);
    if (enRet != Ok) {
        while(1);
    }
    
    enRet = USART_SetBaudrate(M4_USART2, USART2_BAUDRATE);
    if (enRet != Ok) {
        while(1);
    }

    stc_irq_regi_conf_t irqConf;
    irqConf.enIRQn = IRQ_INDEX_INT_USART2_RI;
    irqConf.pfnCallback = &Usart2RxIrqCallback;
    irqConf.enIntSrc = INT_USART2_RI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_08);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    irqConf.enIRQn = IRQ_INDEX_INT_USART2_EI;
    irqConf.pfnCallback = &Usart2ErrIrqCallback;
    irqConf.enIntSrc = INT_USART2_EI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    irqConf.enIRQn = IRQ_INDEX_INT_USART2_TI;
    irqConf.pfnCallback = &Usart2TxIrqCallback;
    irqConf.enIntSrc = INT_USART2_TI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    irqConf.enIRQn = IRQ_INDEX_INT_USART2_TCI;
    irqConf.pfnCallback = &Usart2TxCmpltIrqCallback;
    irqConf.enIntSrc = INT_USART2_TCI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    USART_FuncCmd(M4_USART2, UsartRx, Enable);
    USART_FuncCmd(M4_USART2, UsartTx, Enable);

    USART_FuncCmd(M4_USART2, UsartRxInt, Enable);
//  USART_FuncCmd(M4_USART2, UsartTxCmpltInt, Enable);
}

//
// USART3
//
void uart3_init(void)
{
    en_result_t enRet = Ok;
    const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkNoOutput,
        UsartClkDiv_16,
        UsartDataBits8,
        UsartDataLsbFirst,
        UsartOneStopBit,
        UsartParityNone,
        UsartSampleBit8,
        UsartStartBitFallEdge,
        UsartRtsEnable,
    };

    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART3, Enable);
    
    PORT_SetFunc(USART3_RX_PORT, USART3_RX_PIN, Func_Usart3_Rx, Disable);
    PORT_SetFunc(USART3_TX_PORT, USART3_TX_PIN, Func_Usart3_Tx, Disable);
    
    enRet = USART_UART_Init(M4_USART3, &stcInitCfg);
    if (enRet != Ok) {
        while(1);
    }
    
    enRet = USART_SetBaudrate(M4_USART3, USART3_BAUDRATE);
    if (enRet != Ok) {
        while(1);
    }

    stc_irq_regi_conf_t irqConf;
    irqConf.enIRQn = IRQ_INDEX_INT_USART3_RI;
    irqConf.pfnCallback = &Usart3RxIrqCallback;
    irqConf.enIntSrc = INT_USART3_RI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    irqConf.enIRQn = IRQ_INDEX_INT_USART3_EI;
    irqConf.pfnCallback = &Usart3ErrIrqCallback;
    irqConf.enIntSrc = INT_USART3_EI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    irqConf.enIRQn = IRQ_INDEX_INT_USART3_TI;
    irqConf.pfnCallback = &Usart3TxIrqCallback;
    irqConf.enIntSrc = INT_USART3_TI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    irqConf.enIRQn = IRQ_INDEX_INT_USART3_TCI;
    irqConf.pfnCallback = &Usart3TxCmpltIrqCallback;
    irqConf.enIntSrc = INT_USART3_TCI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    USART_FuncCmd(M4_USART3, UsartRx, Enable);
    USART_FuncCmd(M4_USART3, UsartTx, Enable);

    USART_FuncCmd(M4_USART3, UsartRxInt, Enable);
//  USART_FuncCmd(M4_USART3, UsartTxCmpltInt, Enable);
}

//
// USART4
//
void uart4_init(void)
{
    en_result_t enRet = Ok;
    const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkNoOutput,
        UsartClkDiv_16,
        UsartDataBits8,
        UsartDataLsbFirst,
        UsartOneStopBit,
        UsartParityNone,
        UsartSampleBit8,
        UsartStartBitFallEdge,
        UsartRtsEnable,
    };

    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART4, Enable);
    
    PORT_SetFunc(USART4_RX_PORT, USART4_RX_PIN, Func_Usart4_Rx, Disable);
    PORT_SetFunc(USART4_TX_PORT, USART4_TX_PIN, Func_Usart4_Tx, Disable);
    
    enRet = USART_UART_Init(M4_USART4, &stcInitCfg);
    if (enRet != Ok) {
        while(1);
    }
    
    enRet = USART_SetBaudrate(M4_USART4, USART4_BAUDRATE);
    if (enRet != Ok) {
        while(1);
    }

    stc_irq_regi_conf_t irqConf;
    irqConf.enIRQn = IRQ_INDEX_INT_USART4_RI;
    irqConf.pfnCallback = &Usart4RxIrqCallback;
    irqConf.enIntSrc = INT_USART4_RI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    irqConf.enIRQn = IRQ_INDEX_INT_USART4_EI;
    irqConf.pfnCallback = &Usart4ErrIrqCallback;
    irqConf.enIntSrc = INT_USART4_EI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    irqConf.enIRQn = IRQ_INDEX_INT_USART4_TI;
    irqConf.pfnCallback = &Usart4TxIrqCallback;
    irqConf.enIntSrc = INT_USART4_TI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    irqConf.enIRQn = IRQ_INDEX_INT_USART4_TCI;
    irqConf.pfnCallback = &Usart4TxCmpltIrqCallback;
    irqConf.enIntSrc = INT_USART4_TCI;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    USART_FuncCmd(M4_USART4, UsartRx, Enable);
    USART_FuncCmd(M4_USART4, UsartTx, Enable);

    USART_FuncCmd(M4_USART4, UsartRxInt, Enable);
//  USART_FuncCmd(M4_USART3, UsartTxCmpltInt, Enable);
}
