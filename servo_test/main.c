//输出一路PWM波，初始化占空比为2.5%。通过按键SW1和SW2来控制占空比变化。
//按键SW1，占空比减小2.5%，直到占空比为12.5%，不再变化；
//按键SW2，占空比增加2.5%，直到占空比为2.5%，不再变化。
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
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

uint32_t duty,width,state1,state2;

int main (void)
{
    FPUEnable();
    FPULazyStackingEnable();

    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    width=SysCtlClockGet();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);

    GPIOPadConfigSet(GPIO_PORTF_BASE,(GPIO_PIN_1),GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

    IntEnable(INT_GPIOF);

    GPIOIntTypeSet(GPIO_PORTF_BASE,(GPIO_INT_PIN_1), GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTF_BASE,(GPIO_INT_PIN_1));
    IntMasterEnable();

    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
    PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1,100000);
    duty=7500;
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,7500);
    PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT), true);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,7500);
    PWMOutputState(PWM0_BASE, (PWM_OUT_3_BIT), true);
    PWMGenEnable(PWM0_BASE,PWM_GEN_1);
    while(1){
//        state1 = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
//        if(state1){
//            duty=2500;
//        }
//        else{
//            duty=7500;
//        }
//        if(state1!=state2){
//            PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,duty);
//            PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,duty);
//        }
//        state2 = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
    }

}
void GPIOIntHandler(void){
    unsigned int Status;
    Status = GPIOIntStatus(GPIO_PORTF_BASE,true);
    if(Status == GPIO_PIN_1){
        if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1))
            duty = 7500;
        else
            duty=2500;
    }
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,duty);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,duty);
    SysCtlDelay(150000);
    GPIOIntClear(GPIO_PORTF_BASE,Status);
}
