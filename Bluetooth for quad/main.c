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

uint32_t clock;
uint32_t CH[4];
uint32_t flag=0;
char ch[50];
void CHSet(uint8_t counter,uint32_t duty);
void UART0Send(const uint8_t *pucBuffer,uint32_t ulCount);
void UART3Send(const uint8_t *pucBuffer,uint32_t ulCount);
void GPIOPWM_Init();
void UART0_Init();
void UART3_Init();
int i;
int main (void)
{

    FPUEnable();
    FPULazyStackingEnable();

    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    clock=SysCtlClockGet();
    GPIOPWM_Init();
    UART0_Init();
    UART3_Init();

    CH[0]=7500;
    CH[1]=7500;
    CH[2]=5000;
    CH[3]=7500;
    //CH1-4分别对应滚转、俯仰、油门、偏航
    while(1)
    {
        if(strstr(ch,"LOCK")!=NULL)//锁定飞控
        {
            CH[0]=7500;
            CH[1]=7500;
            CH[2]=5000;
            CH[3]=7500;
            CHSet(1,CH[0]);
            CHSet(2,CH[1]);
            CHSet(3,CH[2]);
            CHSet(4,5000);
            memset(ch,0,sizeof(ch));
        }
        if(strstr(ch,"UNL")!=NULL)//解锁飞控
        {
            CH[0]=7500;
            CH[1]=7500;
            CH[2]=5000;
            CH[3]=7500;
            CHSet(1,CH[0]);
            CHSet(2,CH[1]);
            CHSet(3,CH[2]);
            CHSet(4,CH[3]);//先把CH4归中一次，才能正常解锁
            CHSet(4,10000);
            memset(ch,0,sizeof(ch));
        }
        if(strstr(ch,"MID")!=NULL)//除油门外，全部归中
        {
            CH[0]=7500;
            CH[1]=7500;
            CH[3]=7500;
            CHSet(1,CH[0]);
            CHSet(2,CH[1]);
            CHSet(4,CH[3]);
            memset(ch,0,sizeof(ch));
        }
        if(strstr(ch,"PIT+")!=NULL)//向前俯身
        {
            CH[1]+=5;
            CHSet(2,CH[1]);
            memset(ch,0,sizeof(ch));
        }
        if(strstr(ch,"PIT-")!=NULL)
        {
            CH[1]-=5;
            CHSet(2,CH[1]);
            memset(ch,0,sizeof(ch));
        }
        if(strstr(ch,"ROL+")!=NULL)//向右滚转
        {
            CH[0]+=5;
            CHSet(1,CH[0]);
            memset(ch,0,sizeof(ch));
        }
        if(strstr(ch,"ROL-")!=NULL)
        {
            CH[0]-=5;
            CHSet(1,CH[0]);
            memset(ch,0,sizeof(ch));
        }
        if(strstr(ch,"UPD+")!=NULL)//增加推力
        {
            if(CH[2]<=8000){CH[2]+=50;}//限制油门最大为60%
            CHSet(3,CH[2]);
            memset(ch,0,sizeof(ch));
        }
        if(strstr(ch,"UPD-")!=NULL)
        {
            if(CH[2]>5000){CH[2]-=50;}//限制油门下限
            CHSet(3,CH[2]);
            memset(ch,0,sizeof(ch));
        }
        if(strstr(ch,"YAW+")!=NULL)//向右旋转
        {
            CH[3]+=5;
            CHSet(4,CH[3]);
            memset(ch,0,sizeof(ch));
        }
        if(strstr(ch,"YAW-")!=NULL)
        {
            CH[3]-=5;
            CHSet(4,CH[3]);
            memset(ch,0,sizeof(ch));
        }
		if (strstr(ch, "s") != NULL)//设置各个通道参数 格式s500050005000
		{
			CH[0] = (ch[1]-'0') * 1000 + (ch[2]-'0') * 100 + (ch[3]-'0') * 10 + (ch[4]-'0');
			CH[1] = (ch[5]-'0') * 1000 + (ch[6]-'0') * 100 + (ch[7]-'0') * 10 + (ch[8]-'0');
			CH[2] = (ch[9]-'0') * 1000 + (ch[10]-'0') * 100 + (ch[11]-'0') * 10 + (ch[12]-'0');
			CHSet(1, CH[0]);
			CHSet(2, CH[1]);
			CHSet(3, CH[2]);
			memset(ch, 0, sizeof(ch));
		}
		if (strstr(ch, "GET") != NULL)//获取各个通道参数 格式5000+5000+5000+5000
		{
			ch[0] = CH[0] / 1000 + '0';
			ch[1] = CH[0] % 1000 / 100 + '0';
			ch[2] = CH[0] % 100 / 10 + '0';
			ch[3] = CH[0] % 10 + '0';
			ch[4] = '+';
			ch[5] = CH[1] / 1000 + '0';
			ch[6] = CH[1] % 1000 / 100 + '0';
			ch[7] = CH[1] % 100 / 10 + '0';
			ch[8] = CH[1] % 10 + '0';
			ch[9] = '+';
			ch[10] = 0;
            UART3Send((uint8_t *)ch, 11);
            SysCtlDelay(200000);
			ch[0] = CH[2] / 1000 + '0';
			ch[1] = CH[2] % 1000 / 100 + '0';
			ch[2] = CH[2] % 100 / 10 + '0';
			ch[3] = CH[2] % 10 + '0';
			ch[4] = '+';
			ch[5] = CH[3] / 1000 + '0';
			ch[6] = CH[3] % 1000 / 100 + '0';
			ch[7] = CH[3] % 100 / 10 + '0';
			ch[8] = CH[3] % 10 + '0';
			ch[9] = 0;
			UART3Send((uint8_t *)ch, 10);
			memset(ch, 0, sizeof(ch));
		}
    }

}

void CHSet(uint8_t counter,uint32_t duty)
{
    uint32_t OUT;
    switch(counter)
    {
    case 1:{OUT=PWM_OUT_2;break;}
    case 2:{OUT=PWM_OUT_3;break;}
    case 3:{OUT=PWM_OUT_4;break;}
    case 4:{OUT=PWM_OUT_5;break;}
    }
    PWMPulseWidthSet(PWM0_BASE,OUT,duty);
}


void GPIOPWM_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTE_BASE,GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTE_BASE,GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinConfigure(GPIO_PE5_M0PWM5);

    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_4,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_4,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_5,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);

    PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1,100000);
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_2,100000);

    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,10000);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,10000);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,5000);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_5,5000);


    /*PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,10000);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,10000);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,5000);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_5,10000);*/

    PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT | PWM_OUT_3_BIT | PWM_OUT_4_BIT | PWM_OUT_5_BIT), true);

    PWMGenEnable(PWM0_BASE,PWM_GEN_1);
    PWMGenEnable(PWM0_BASE,PWM_GEN_2);
}

void UART0Send(const uint8_t *pucBuffer,uint32_t ulCount)
{
    while(ulCount--)
    {
        UARTCharPutNonBlocking(UART0_BASE,*pucBuffer++);
    }
}

void UART3Send(const uint8_t *pucBuffer,uint32_t ulCount)
{
    while(ulCount--)
    {
        UARTCharPutNonBlocking(UART3_BASE,*pucBuffer++);
    }
}

void UART0_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE,GPIO_PIN_0|GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE,SysCtlClockGet(),115200,(UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE,UART_INT_RX|UART_INT_RT);
    IntMasterEnable();
    UART0Send((uint8_t*)"Welcome!\r",25);
    UARTCharPut(UART0_BASE,'\n');
}

void UART3_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);

    GPIOPinConfigure(GPIO_PC6_U3RX);
    GPIOPinConfigure(GPIO_PC7_U3TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE,GPIO_PIN_6|GPIO_PIN_7);

    UARTConfigSetExpClk(UART3_BASE,SysCtlClockGet(),9600,(UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE));

    UARTFIFOLevelSet(UART3_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);
    IntEnable(INT_UART3);
    UARTIntEnable(UART3_BASE,UART_INT_RX|UART_INT_RT);
    IntMasterEnable();
    UART3Send((uint8_t*)"Welcome!\r",25);
    UARTCharPut(UART3_BASE,'\n');
}

void UART0IntHandler(void)
{
    uint32_t ulStatus;
    uint8_t i=0;
    ulStatus=UARTIntStatus(UART0_BASE,true);
    UARTIntClear(UART0_BASE,ulStatus);


    while(UARTCharsAvail(UART0_BASE))
    {
        ch[i] = UARTCharGet(UART0_BASE);
        UARTCharPut(UART0_BASE,ch[i]);
        i++;
    }
    UARTCharPutNonBlocking(UART0_BASE,'\r');
    UARTCharPutNonBlocking(UART0_BASE,'\n');

}

void UART3IntHandler(void)
{
    uint32_t ulStatus;
    uint8_t i=0;
    ulStatus=UARTIntStatus(UART3_BASE,true);
    UARTIntClear(UART3_BASE,ulStatus);


    while(UARTCharsAvail(UART3_BASE))
    {
        ch[i] = UARTCharGet(UART3_BASE);
        UARTCharPut(UART3_BASE,ch[i]);
        i++;
    }
    UARTCharPutNonBlocking(UART3_BASE,'\r');
    UARTCharPutNonBlocking(UART3_BASE,'\n');

}
