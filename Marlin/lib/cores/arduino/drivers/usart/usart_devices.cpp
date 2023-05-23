#include "usart.h"

// PCLK1 max is 100Mhz
// baudrate = clk/(8 �� (2 - over8 ) * (DIV_Integer + 1) )

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

    stc_irq_regi_conf_t stcIrqRegiCfg;
    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART1_RI;
    stcIrqRegiCfg.pfnCallback = &Usart1RxIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART1_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART1_EI;
    stcIrqRegiCfg.pfnCallback = &Usart1ErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART1_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART1_TI;
    stcIrqRegiCfg.pfnCallback = &Usart1TxIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART1_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART1_TCI;
    stcIrqRegiCfg.pfnCallback = &Usart1TxCmpltIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART1_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    USART_FuncCmd(M4_USART1, UsartRx, Enable);
    USART_FuncCmd(M4_USART1, UsartTx, Enable);
    
    USART_FuncCmd(M4_USART1, UsartRxInt, Enable);
//  USART_FuncCmd(M4_USART1, UsartTxCmpltInt, Enable);

    usart_dev *USART1 = &usart1;
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

    stc_irq_regi_conf_t stcIrqRegiCfg;
    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART2_RI;
    stcIrqRegiCfg.pfnCallback = &Usart2RxIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART2_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_08);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART2_EI;
    stcIrqRegiCfg.pfnCallback = &Usart2ErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART2_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART2_TI;
    stcIrqRegiCfg.pfnCallback = &Usart2TxIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART2_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART2_TCI;
    stcIrqRegiCfg.pfnCallback = &Usart2TxCmpltIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART2_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    USART_FuncCmd(M4_USART2, UsartRx, Enable);
    USART_FuncCmd(M4_USART2, UsartTx, Enable);

    USART_FuncCmd(M4_USART2, UsartRxInt, Enable);
//  USART_FuncCmd(M4_USART2, UsartTxCmpltInt, Enable);

    usart_dev *USART2 = &usart2;
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

    stc_irq_regi_conf_t stcIrqRegiCfg;
    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART3_RI;
    stcIrqRegiCfg.pfnCallback = &Usart3RxIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART3_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART3_EI;
    stcIrqRegiCfg.pfnCallback = &Usart3ErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART3_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART3_TI;
    stcIrqRegiCfg.pfnCallback = &Usart3TxIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART3_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART3_TCI;
    stcIrqRegiCfg.pfnCallback = &Usart3TxCmpltIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART3_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    USART_FuncCmd(M4_USART3, UsartRx, Enable);
    USART_FuncCmd(M4_USART3, UsartTx, Enable);

    USART_FuncCmd(M4_USART3, UsartRxInt, Enable);
//  USART_FuncCmd(M4_USART3, UsartTxCmpltInt, Enable);

    usart_dev *USART3 = &usart3;
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

    stc_irq_regi_conf_t stcIrqRegiCfg;
    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART4_RI;
    stcIrqRegiCfg.pfnCallback = &Usart4RxIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART4_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART4_EI;
    stcIrqRegiCfg.pfnCallback = &Usart4ErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART4_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART4_TI;
    stcIrqRegiCfg.pfnCallback = &Usart4TxIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART4_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_USART4_TCI;
    stcIrqRegiCfg.pfnCallback = &Usart4TxCmpltIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART4_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    USART_FuncCmd(M4_USART4, UsartRx, Enable);
    USART_FuncCmd(M4_USART4, UsartTx, Enable);

    USART_FuncCmd(M4_USART4, UsartRxInt, Enable);
//  USART_FuncCmd(M4_USART3, UsartTxCmpltInt, Enable);

    usart_dev *USART4 = &usart4;
}
