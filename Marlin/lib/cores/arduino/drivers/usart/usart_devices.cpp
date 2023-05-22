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
    
    PORT_SetFunc(USART1_RX_PORT, USART1_RX_PIN, USART1_RX_FUNC, Disable);
    PORT_SetFunc(USART1_TX_PORT, USART1_TX_PIN, USART1_TX_FUNC, Disable);
    
    enRet = USART_UART_Init(USART1_CH, &stcInitCfg);
    if (enRet != Ok) {
        while(1);
    }
    
    enRet = USART_SetBaudrate(USART1_CH, USART1_BAUDRATE);
    if (enRet != Ok) {
        while(1);
    }

    stc_irq_regi_conf_t stcIrqRegiCfg;
    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART1_INT_RI;
    stcIrqRegiCfg.pfnCallback = &Usart1RxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART1_INT_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART1_INT_EI;
    stcIrqRegiCfg.pfnCallback = &Usart1ErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART1_INT_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART1_INT_TI;
    stcIrqRegiCfg.pfnCallback = &Usart1TxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART1_INT_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART1_INT_TCI;
    stcIrqRegiCfg.pfnCallback = &Usart1TxCmpltIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART1_INT_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    USART_FuncCmd(USART1_CH, UsartRx, Enable);
    USART_FuncCmd(USART1_CH, UsartTx, Enable);
    
    USART_FuncCmd(USART1_CH, UsartRxInt, Enable);
//  USART_FuncCmd(USART1_CH, UsartTxCmpltInt, Enable);
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
    
    PORT_SetFunc(USART2_RX_PORT, USART2_RX_PIN, USART2_RX_FUNC, Disable);
    PORT_SetFunc(USART2_TX_PORT, USART2_TX_PIN, USART2_TX_FUNC, Disable);
    
    enRet = USART_UART_Init(USART2_CH, &stcInitCfg);
    if (enRet != Ok) {
        while(1);
    }
    
    enRet = USART_SetBaudrate(USART2_CH, USART2_BAUDRATE);
    if (enRet != Ok) {
        while(1);
    }

    stc_irq_regi_conf_t stcIrqRegiCfg;
    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART2_INT_RI;
    stcIrqRegiCfg.pfnCallback = &Usart2RxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART2_INT_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_08);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART2_INT_EI;
    stcIrqRegiCfg.pfnCallback = &Usart2ErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART2_INT_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART2_INT_TI;
    stcIrqRegiCfg.pfnCallback = &Usart2TxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART2_INT_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART2_INT_TCI;
    stcIrqRegiCfg.pfnCallback = &Usart2TxCmpltIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART2_INT_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    USART_FuncCmd(USART2_CH, UsartRx, Enable);
    USART_FuncCmd(USART2_CH, UsartTx, Enable);

    USART_FuncCmd(USART2_CH, UsartRxInt, Enable);
//  USART_FuncCmd(USART2_CH, UsartTxCmpltInt, Enable);
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
    
    PORT_SetFunc(USART3_RX_PORT, USART3_RX_PIN, USART3_RX_FUNC, Disable);
    PORT_SetFunc(USART3_TX_PORT, USART3_TX_PIN, USART3_TX_FUNC, Disable);
    
    enRet = USART_UART_Init(USART3_CH, &stcInitCfg);
    if (enRet != Ok) {
        while(1);
    }
    
    enRet = USART_SetBaudrate(USART3_CH, USART3_BAUDRATE);
    if (enRet != Ok) {
        while(1);
    }

    stc_irq_regi_conf_t stcIrqRegiCfg;
    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART3_INT_RI;
    stcIrqRegiCfg.pfnCallback = &Usart3RxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART3_INT_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART3_INT_EI;
    stcIrqRegiCfg.pfnCallback = &Usart3ErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART3_INT_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART3_INT_TI;
    stcIrqRegiCfg.pfnCallback = &Usart3TxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART3_INT_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART3_INT_TCI;
    stcIrqRegiCfg.pfnCallback = &Usart3TxCmpltIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART3_INT_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    USART_FuncCmd(USART3_CH, UsartRx, Enable);
    USART_FuncCmd(USART3_CH, UsartTx, Enable);

    USART_FuncCmd(USART3_CH, UsartRxInt, Enable);
//  USART_FuncCmd(USART3_CH, UsartTxCmpltInt, Enable);
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
    
    PORT_SetFunc(USART4_RX_PORT, USART4_RX_PIN, USART4_RX_FUNC, Disable);
    PORT_SetFunc(USART4_TX_PORT, USART4_TX_PIN, USART4_TX_FUNC, Disable);
    
    enRet = USART_UART_Init(USART4_CH, &stcInitCfg);
    if (enRet != Ok) {
        while(1);
    }
    
    enRet = USART_SetBaudrate(USART4_CH, USART4_BAUDRATE);
    if (enRet != Ok) {
        while(1);
    }

    stc_irq_regi_conf_t stcIrqRegiCfg;
    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART4_INT_RI;
    stcIrqRegiCfg.pfnCallback = &Usart4RxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART4_INT_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART4_INT_EI;
    stcIrqRegiCfg.pfnCallback = &Usart4ErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART4_INT_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART4_INT_TI;
    stcIrqRegiCfg.pfnCallback = &Usart4TxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART4_INT_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART4_INT_TCI;
    stcIrqRegiCfg.pfnCallback = &Usart4TxCmpltIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART4_INT_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    USART_FuncCmd(USART4_CH, UsartRx, Enable);
    USART_FuncCmd(USART4_CH, UsartTx, Enable);

    USART_FuncCmd(USART4_CH, UsartRxInt, Enable);
//  USART_FuncCmd(USART3_CH, UsartTxCmpltInt, Enable);
}
