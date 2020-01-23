#ifndef _OV7725_H
#define _OV7725_H
//#include "sys.h"
#include "sccb.h"
#include <stdint.h>
#include <stdbool.h>

//////////////////////////////////////////////////////////////////////////////////
//ALIENTEK MiniSTM32开发板
//OV7725 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2017/11/1
//版本：V1.0		    							    							  
//////////////////////////////////////////////////////////////////////////////////
 
#define OV7725_MID				0X7FA2    //Manufacturer ID
#define OV7725_PID				0X7721    //product ID

#define OV7725_WRST_H		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7)		//写指针复位          PA3
#define OV7725_WRST_L       GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0)
#define OV7725_RCK_H        HWREG(0x40004040) = 0x10;       //读数据时钟          PA4
#define OV7725_RCK_L        HWREG(0x40004040) = 0x00;
#define OV7725_RRST_H		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5)  		//读指针复位          PA5
#define OV7725_RRST_L       GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0)
#define OV7725_CS_H		    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6)  		//片选信号(OE)   PA6
#define OV7725_CS_L         GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0)
#define OV7725_WREN_H 		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3)		//写入FIFO使能      PA7
#define OV7725_WREN_L       GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0)
#define OV7725_VSYNC  	    GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_0)		                //同步信号检测IO  PE0//中断


#define OV7725_DATA   HWREG(0x400053fc)//数据输入端口

/////////////////////////////////////////									
	    				 
uint8_t   OV7725_Init(void);
void OV7725_Light_Mode(uint8_t mode);
void OV7725_Color_Saturation(int8_t sat);
void OV7725_Brightness(int8_t bright);
void OV7725_Contrast(int8_t contrast);
void OV7725_Special_Effects(uint8_t eft);
void OV7725_Window_Set(uint16_t width,uint16_t height,uint8_t mode);


#endif





















