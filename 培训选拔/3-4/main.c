#include <stdint.h>
#include <stdbool.h>
#include "math.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"

uint32_t ui32Period, ui32PWMPeriod,width,count,ui32duty,sample,base,amp;

int main(void)
{


    FPUEnable();
    FPULazyStackingEnable();

    sample = 1000;
    base = 50;
    amp = 50;
    count = 0;


    SysCtlClockSet(SYSCTL_SYSDIV_1|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_1);

    GPIOPinConfigure(GPIO_PF1_M1PWM5);


    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    ui32Period = (SysCtlClockGet() *2);
    ui32PWMPeriod = ui32Period/10000;

    PWMGenPeriodSet(PWM1_BASE,PWM_GEN_2,ui32PWMPeriod);

    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_5,2500);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);

    TimerLoadSet(TIMER0_BASE,TIMER_A,ui32Period/sample - 1);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    TimerEnable(TIMER0_BASE,TIMER_A);

    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);

    PWMGenEnable(PWM1_BASE,PWM_GEN_2);
    while(1){

    }
}

void Time0IntenHandler(void)
{
    TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);

    ui32duty = base+amp*sin(2*3.1415926535*count/sample);
    width = (ui32PWMPeriod*ui32duty/100);
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_5,width);
    count=count+1;
    if (count==sample){
        count=0;
    }
}
