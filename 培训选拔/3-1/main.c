//输出两路PWM波，中心对齐/边缘对齐，20KHz
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/fpu.h"

int main (void)
{

    FPUEnable();
    FPULazyStackingEnable();

    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinTypePWM(GPIO_PORTE_BASE,GPIO_PIN_4);
    //GPIOPinTypePWM(GPIO_PORTE_BASE,GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_4);
    //GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    //GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    //GPIOPinConfigure(GPIO_PE5_M0PWM5);

    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_4,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_4,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    //GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    //GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_5,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);

    SysCtlPWMClockSet(SYSCTL_PWMDIV_2);

    PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE,PWM_GEN_2,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1,1000);
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_2,1000);

    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,500);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,500);
    //PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,500);
    //PWMPulseWidthSet(PWM0_BASE,PWM_OUT_5,500);

    PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT | PWM_OUT_4_BIT), true);
    //PWMOutputState(PWM0_BASE, (PWM_OUT_3_BIT | PWM_OUT_5_BIT), true);


    PWMGenEnable(PWM0_BASE,PWM_GEN_2);
    PWMGenEnable(PWM0_BASE,PWM_GEN_1);
    while(1){

    }

}
