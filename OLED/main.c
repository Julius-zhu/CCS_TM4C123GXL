//pe1»»³Épa4

/**
 * main.c
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "string.h"
#include "OLED.h"
unsigned char ch[3]="55";
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    OLED_Initial();
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_PIN_0);
    //LCD_P14x16Ch(0,0,ch);
    LCD_P14x16Ch(64,6,8);
    //OLED_DrawBMP(0,0,128,8,BMP1);
    //LCD_P8x16Str(16,0,ch);
    while(1){
//        LCD_P8x16Str(10,10,'5');
//        LCD_CLS();

//        LCD_Fill(0xff);
    }
}
