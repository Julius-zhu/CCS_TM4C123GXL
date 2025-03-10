#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"

void UARTSend(const uint8_t *pucBuffer, uint32_t ulCount);

int main(void)
{
    FPUEnable();
    FPULazyStackingEnable();
    SysCtlClockSet(SYSCTL_SYSDIV_4| SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE,GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);

    //UARTConfigSetExpClk(UART0_BASE,SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    IntMasterEnable();
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE,UART_INT_RX | UART_INT_RT);

    UARTprintf("Hello, world!\n");
    while(1){

    }
}
void UARTSend(const uint8_t *pucBuffer, uint32_t ulCount){
    while(ulCount--){
        UARTCharPut(UART0_BASE,*pucBuffer++);
    }

}

void UARTIntHandler(void)
{
    uint32_t ulStatus;

    ulStatus = UARTIntStatus(UART0_BASE, true);

    UARTIntClear(UART0_BASE, ulStatus);

    while(UARTCharsAvail(UART0_BASE))
    {
        UARTCharPutNonBlocking(UART0_BASE,UARTCharGetNonBlocking(UART0_BASE));
    }
    double temp=0.33344;
    if(temp>=0){
    UARTprintf("%d.%d",(int32_t)temp ,(int32_t)((temp -(int32_t)temp )*1000));
    }
    else{
    temp = 0-temp;
    UARTprintf("-%d.%d",(int32_t)temp ,(int32_t)((temp -(int32_t)temp )*1000));

    }
}

