//输出一路PWM波，初始化占空比为50%。通过按键SW1和SW2来控制占空比变化。
//按键SW1，占空比减小5%，直到占空比为10%，不再变化；
//按键SW2，占空比增加5%，直到占空比为90%，不再变化。
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

uint32_t duty,width;

int main (void)
{
    FPUEnable();
    FPULazyStackingEnable();

    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 |GPIO_PIN_4);

    GPIOPadConfigSet(GPIO_PORTF_BASE,(GPIO_PIN_0| GPIO_PIN_4),GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

    IntEnable(INT_GPIOF);

    GPIOIntTypeSet(GPIO_PORTF_BASE,(GPIO_INT_PIN_0| GPIO_INT_PIN_4), GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTF_BASE,(GPIO_INT_PIN_0 | GPIO_INT_PIN_4));
    IntMasterEnable();

    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM1_BASE,PWM_GEN_2,1000000);


    duty=25000;
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_5,25000);

    PWMOutputState(PWM1_BASE, (PWM_OUT_5_BIT), true);

    PWMGenEnable(PWM1_BASE,PWM_GEN_2);
    while(1){

    }

}
void GPIOIntHandler(void){
    unsigned int Status;
    Status = GPIOIntStatus(GPIO_PORTF_BASE,true);
    if(Status == GPIO_PIN_0){
        if(duty<125000)
            duty =duty+125000;
     }
    else if(Status == GPIO_PIN_4)
    {
        if(duty>25000)
            duty=duty-25000;
    }
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_5,duty);
    SysCtlDelay(1500000);
    GPIOIntClear(GPIO_PORTF_BASE,Status);
}

