#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "timer.h"
#include "hw_ints.h"

int main(void)
{
    uint32_t ui32Period;
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);

    ui32Period = (SysCtlClockGet()/2);
    TimerLoadSet(TIMER0_BASE,TIMER_A,ui32Period -1);

    IntEnable(INT_TIMER0A);//NVIC可接受中断，管理中断
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);//timer中断
    IntMasterEnable();//cpu可接受中断

    TimerEnable(TIMER0_BASE,TIMER_A);//启动timer

    while(1){
    }

}
void Timer0IntHandler(void)
    {
        TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);

        if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1))
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);
        else
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
    }
