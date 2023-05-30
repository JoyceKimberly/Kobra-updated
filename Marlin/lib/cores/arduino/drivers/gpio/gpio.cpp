#include "startup.h"
#include "fastio.h"
#include "gpio.h"
#include <init.h>

/*  Basically everything that is defined having ADC */
extern const uint8_t ADC_PINS[BOARD_NR_ADC_PINS] = {
    PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7,PB0,PB1,PC0,PC1,PC2,PC3,PC4,PC5
};

extern void setup_gpio(void )
{  
    stc_port_init_t stcPortInit;
    MEM_ZERO_STRUCT(stcPortInit);

    PORT_DebugPortSetting(0x1C,Disable);
    
     /*initiallize LED port*/
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Disable;
    stcPortInit.enPullUp = Disable;
    /* LED0 and LED1 Port/Pin initialization */
    //GPIO_Init(LED, &stcPortInit);
}

HardwareSerial MSerial1(M4_USART1);
HardwareSerial MSerial2(M4_USART2);
HardwareSerial MSerial3(M4_USART3);
HardwareSerial MSerial4(M4_USART4);

//DEFINE_HWSERIAL(Serial1, 1);
//DEFINE_HWSERIAL(Serial2, 2);
//DEFINE_HWSERIAL(Serial3, 3);

//HardwareSerial MSerial(LPUART1);
//HardwareSerial MotorUart2(LPUART2);
//HardwareSerial MotorUart8(LPUART8);
//DEFINE_HWSERIAL(Serial4, 4);
