/*
 * UART.h
 *
 *  Created on: 2019Äê7ÔÂ14ÈÕ
 *      Author: ZJN
 */

#ifndef UART_H_
#define UART_H_
char ch[30];

void UARTSend(const uint8_t *pucBuffer, uint32_t ulCount){
    while(ulCount--){
        UARTCharPut(UART0_BASE,*pucBuffer++);
    }
}

void UART0Init(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE,GPIO_PIN_0|GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE,SysCtlClockGet(),115200,(UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE,UART_INT_RX|UART_INT_RT);
    IntMasterEnable();
    UARTSend((uint8_t*)"Welcome!\r",25);
    UARTCharPut(UART0_BASE,'\n');
}

void UART0IntHandler(void)
{
    uint32_t ulStatus;
    uint8_t i=0;
    ulStatus=UARTIntStatus(UART0_BASE,true);
    UARTIntClear(UART0_BASE,ulStatus);
    while(UARTCharsAvail(UART0_BASE))
    {
        ch[i] = UARTCharGet(UART0_BASE);
        UARTCharPut(UART0_BASE,ch[i]);
        i++;
    }
    UARTCharPutNonBlocking(UART0_BASE,'\r');
    UARTCharPutNonBlocking(UART0_BASE,'\n');
}

#endif /* UART_H_ */
