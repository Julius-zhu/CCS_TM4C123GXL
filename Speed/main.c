

/**
 * main.c
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "inc/hw_gpio.h"

void InitConsole(void);
void UARTSend(const uint8_t *pucBuffer, uint32_t ulCount);
int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    InitConsole();
    MPU9150_Initial();
    UARTCharPut(UART0_BASE, 0xFF);
    while(1){
        UARTCharPut(UART0_BASE, 0xFF);
        SysCtlDelay(10000000);
    }
}

void InitConsole(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    //使用16MHZ内部时钟源作为串口的时钟
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //串口号/波特率/时钟频率
    UARTStdioConfig(0, 115200, 16000000);
}
void UARTSend(const uint8_t *pucBuffer, uint32_t ulCount)
{
    while(ulCount--)
    {
        //将要发送的字符写进UART
        UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
        //UARTCharPut(UART0_BASE, *pucBuffer++);
    }
}
