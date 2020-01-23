/*
 * peripheral.c
 *
 *  Created on: 2019��8��6��
 *      Author: ASUS
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "string.h"

//OLED D0-PB2,D1-PB3,RES-PE3,DC-PE2,CS-PE1
//������ IO��-PA2
//��ť ���ϵ�������Ϊ PD0 PD1 PD2 PD3
//��������� Y-PA5,R-PA6,B-PA7


//ʹ�ܲ���������GPIO��
void GPIO_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    HWREG(GPIO_PORTA_BASE+GPIO_O_LOCK)=GPIO_LOCK_KEY;
    HWREG(GPIO_PORTA_BASE+GPIO_O_CR)=0xff;
    HWREG(GPIO_PORTA_BASE+GPIO_O_LOCK)=0;

    HWREG(GPIO_PORTB_BASE+GPIO_O_LOCK)=GPIO_LOCK_KEY;
    HWREG(GPIO_PORTB_BASE+GPIO_O_CR)=0xff;
    HWREG(GPIO_PORTB_BASE+GPIO_O_LOCK)=0;

    HWREG(GPIO_PORTC_BASE+GPIO_O_LOCK)=GPIO_LOCK_KEY;
    HWREG(GPIO_PORTC_BASE+GPIO_O_CR)=0xff;
    HWREG(GPIO_PORTC_BASE+GPIO_O_LOCK)=0;

    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK)=GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE+GPIO_O_CR)=0xff;
    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK)=0;

    HWREG(GPIO_PORTE_BASE+GPIO_O_LOCK)=GPIO_LOCK_KEY;
    HWREG(GPIO_PORTE_BASE+GPIO_O_CR)=0xff;
    HWREG(GPIO_PORTE_BASE+GPIO_O_LOCK)=0;

    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK)=GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR)=0xff;
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK)=0;
}

void buzzer_Init()
{
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_2);
    GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_2,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPD);   //������IO��������
}

void button_Init()
{
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_0);                                   //��ťΪ����
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_6);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_2);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_0,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);   //��ť������
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_6,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_2,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_3,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);


    GPIOIntTypeSet(GPIO_PORTD_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE);
    GPIOIntTypeSet(GPIO_PORTD_BASE,GPIO_PIN_6,GPIO_FALLING_EDGE);
    GPIOIntTypeSet(GPIO_PORTD_BASE,GPIO_PIN_2,GPIO_FALLING_EDGE);
    GPIOIntTypeSet(GPIO_PORTD_BASE,GPIO_PIN_3,GPIO_FALLING_EDGE);

    IntEnable(INT_GPIOD);
    GPIOIntEnable(GPIO_PORTD_BASE,GPIO_PIN_0);
    GPIOIntEnable(GPIO_PORTD_BASE,GPIO_PIN_6);
    GPIOIntEnable(GPIO_PORTD_BASE,GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTD_BASE,GPIO_PIN_3);
    IntMasterEnable();
}

void LED_Init()
{
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_5);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_6);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPD);       //LED������
    GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPD);
    GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_7,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPD);
}

void peripheral_Init()
{
    buzzer_Init();
    button_Init();
    LED_Init();
}

//����������
void buzzer_ON()
{
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_2,GPIO_PIN_2);
}
void buzzer_OFF()
{
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_2,0);
}


//LED_Y����
void LED_Y_ON()
{
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_PIN_5);
}
void LED_Y_OFF()
{
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,0);
}


//LED_R����
void LED_R_ON()
{
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_PIN_6);
}
void LED_R_OFF()
{
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,0);
}



//LED_B����
void LED_B_ON()
{
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,GPIO_PIN_7);
}
void LED_B_OFF()
{
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,0);
}


