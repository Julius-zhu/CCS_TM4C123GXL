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

uint32_t duty=0;
uint32_t cur_duty=0;
bool flag=true;

int main(void)
{
    uint32_t ui32Period;

    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);

    ui32Period = (SysCtlClockGet()/10000);
    TimerLoadSet(TIMER0_BASE,TIMER_A,ui32Period -1);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    TimerEnable(TIMER0_BASE,TIMER_A);

    while(1){
    }

}
void Timer0IntHandler(void)
    {
        TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);

        if(cur_duty>=99){
            cur_duty=0;
            if(flag)
                duty++;
            else
                duty--;
            if(duty>=99)
                flag=0;
            if(duty<=0)
                flag=1;
        }
        if(cur_duty<duty){
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
        }
        else{
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);
        }
        cur_duty++;

    }
