#ifndef __SCCB_H
#define __SCCB_H
//#include "sys.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "uartstdio.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序参考自网友guanfu_wang代码。
//ALIENTEK MiniSTM32开发板
//OV7670 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/10/31
//版本：V1.0		    							    							  
//////////////////////////////////////////////////////////////////////////////////

//时钟线设置为推挽输出模式，数据线在输出时为推挽输出模式；在输入时是上拉输入模式
#define SCCB_SDA_IN()       {GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_3);\
                             GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);}//弱上拉才能用
#define SCCB_SDA_OUT()      {GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);\
                             GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);}

//IO操作函数	 
#define SCCB_SCL_H    		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2)	 	//SCL   PD2  置高
#define SCCB_SCL_L          GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0)                //SCL   PD2 置低
#define SCCB_SDA_H    		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3) 		//SDA	PD3
#define SCCB_SDA_L          GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0)                //SDA   PD3

#define SCCB_READ_SDA    	GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3)  		//输入SDA
#define SCCB_ID   			0X42  			//OV7670的ID

///////////////////////////////////////////
void SCCB_Init(void);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_No_Ack(void);
uint8_t SCCB_WR_Byte(uint8_t dat);
uint8_t SCCB_RD_Byte(void);
uint8_t SCCB_WR_Reg(uint8_t reg,uint8_t data);
uint8_t SCCB_RD_Reg(uint8_t reg);
void delay_us(uint16_t delay_time);
void delay_ms(uint16_t delay_time);
#endif













