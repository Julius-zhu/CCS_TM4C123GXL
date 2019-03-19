#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "inc/hw_gpio.h"
#include "driverlib/pin_map.h"

int LEDStatus=0;
int main(void)
{
    FPUEnable();
    FPULazyStackingEnable();

    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);

    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);

    TimerLoadSet(TIMER0_BASE,TIMER_A,(SysCtlClockGet()/4) -1);

    IntMasterEnable();

    IntEnable(INT_TIMER0A);//NVIC可接受中断，管理中断
    IntEnable(INT_GPIOF);
    GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_INT_PIN_0, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTF_BASE,GPIO_INT_PIN_0);

    TimerEnable(TIMER0_BASE,TIMER_A);

}
void GPIOIntHandler(void){
    unsigned int Status;
    Status = GPIOIntStatus(GPIO_PORTF_BASE,true);
    SysCtlDelay(15000);
    if(Status == GPIO_PIN_0){
        switch(LEDStatus){
            case 0:
                LEDStatus=1;
                TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
                break;
            case 1:
                LEDStatus=2;
                TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
                GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
                GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_PIN_2);
                GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,GPIO_PIN_3);
                break;
            case 2:

                GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);
                GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,0);
                GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,0);
                LEDStatus=0;
                break;
        }
        GPIOIntClear(GPIO_PORTF_BASE,Status);
     }
}

void Timer0IntHandler(void)
    {
        TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);

        if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1)){
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,0);
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,0);
        }
        else
        {
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_PIN_2);
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,GPIO_PIN_3);
        }
    }
