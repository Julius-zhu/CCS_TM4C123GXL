//#include "sys.h"
#include "ov7725.h"
#include "ov7725config.h"	  
//#include "delay.h"
//#include "usart.h"
#include "sccb.h"
#include "uartstdio.h"
//////////////////////////////////////////////////////////////////////////////////
//ALIENTEK MiniSTM32开发板
//OV7725 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2017/11/1
//版本：V1.0		    							    							  
//////////////////////////////////////////////////////////////////////////////////
		    			    
//初始化OV7725
//返回0:成功
//返回其他值:错误代码
uint8_t OV7725_Init(void)
{
	uint16_t reg;
	uint16_t i=0;
	//设置IO
 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
 
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7|GPIO_PIN_4|GPIO_PIN_5);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7|GPIO_PIN_4|GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//推挽输出
    OV7725_WRST_H;OV7725_RCK_H;OV7725_RRST_H;

    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0); //PE0输入 上拉
    GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);
	
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);//推挽输出
    OV7725_CS_H;OV7725_WREN_H;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, 0x000000ff);//使能PB0~7
    GPIOPadConfigSet(GPIO_PORTB_BASE, 0x000000ff, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

 
	SCCB_Init();        		//初始化SCCB 的IO口
 	if(SCCB_WR_Reg(0x12,0x80))return 1;	//软复位OV7725
	delay_ms(50); 
	reg=SCCB_RD_Reg(0X1c);		//读取厂家ID 高八位
	reg<<=8;
	reg|=SCCB_RD_Reg(0X1d);		//读取厂家ID 低八位
	if(reg!=OV7725_MID)
	{
	    UARTprintf("MID:%d\r\n",reg);
		return 2;
	}
	reg=SCCB_RD_Reg(0X0a);		//读取厂家ID 高八位
	reg<<=8;
	reg|=SCCB_RD_Reg(0X0b);		//读取厂家ID 低八位
	if(reg!=OV7725_PID)
	{
	    UARTprintf("HID:%d\r\n",reg);
		return 3;
	}   
 	//初始化 OV7725,采用QVGA分辨率(320*240)  
	for(i=0;i<sizeof(ov7725_init_reg_tb1)/sizeof(ov7725_init_reg_tb1[0]);i++)
	{								
	   	SCCB_WR_Reg(ov7725_init_reg_tb1[i][0],ov7725_init_reg_tb1[i][1]);
 	} 
  	return 0x00; 	//ok
} 
////////////////////////////////////////////////////////////////////////////
//OV7725功能设置
//白平衡设置
//0:自动模式
//1:晴天
//2,多云
//3,办公室
//4,家里
//5,夜晚
void OV7725_Light_Mode(uint8_t mode)
{
	switch(mode)
	{
		case 0:	//Auto，自动模式
			SCCB_WR_Reg(0x13, 0xff); //AWB on 
			SCCB_WR_Reg(0x0e, 0x65);
			SCCB_WR_Reg(0x2d, 0x00);
			SCCB_WR_Reg(0x2e, 0x00);
			break;
		case 1://sunny，晴天
			SCCB_WR_Reg(0x13, 0xfd); //AWB off
			SCCB_WR_Reg(0x01, 0x5a);
			SCCB_WR_Reg(0x02, 0x5c);
			SCCB_WR_Reg(0x0e, 0x65);
			SCCB_WR_Reg(0x2d, 0x00);
			SCCB_WR_Reg(0x2e, 0x00);
			break;	
		case 2://cloudy，多云
			SCCB_WR_Reg(0x13, 0xfd); //AWB off
			SCCB_WR_Reg(0x01, 0x58);
			SCCB_WR_Reg(0x02, 0x60);
			SCCB_WR_Reg(0x0e, 0x65);
			SCCB_WR_Reg(0x2d, 0x00);
			SCCB_WR_Reg(0x2e, 0x00);
			break;	
		case 3://office，办公室
			SCCB_WR_Reg(0x13, 0xfd); //AWB off
			SCCB_WR_Reg(0x01, 0x84);
			SCCB_WR_Reg(0x02, 0x4c);
			SCCB_WR_Reg(0x0e, 0x65);
			SCCB_WR_Reg(0x2d, 0x00);
			SCCB_WR_Reg(0x2e, 0x00);
			break;	
		case 4://home，家里
			SCCB_WR_Reg(0x13, 0xfd); //AWB off
			SCCB_WR_Reg(0x01, 0x96);
			SCCB_WR_Reg(0x02, 0x40);
			SCCB_WR_Reg(0x0e, 0x65);
			SCCB_WR_Reg(0x2d, 0x00);
			SCCB_WR_Reg(0x2e, 0x00);
			break;	

		case 5://night，夜晚
			SCCB_WR_Reg(0x13, 0xff); //AWB on
			SCCB_WR_Reg(0x0e, 0xe5);
			break;
	}
}		  
//色度设置
//sat:-4~+4
void OV7725_Color_Saturation(int8_t sat)
{
 	if(sat>=-4 && sat<=4)
	{	
		SCCB_WR_Reg(USAT,(sat+4)<<4); 
		SCCB_WR_Reg(VSAT,(sat+4)<<4);
	}
}
//亮度设置
//bright：-4~+4
void OV7725_Brightness(int8_t bright)
{
	uint8_t bright_value,sign;
  	switch(bright)
	{
		case 4:
			bright_value = 0x48;
			sign = 0x06;
			break;
		case 3:
			bright_value = 0x38;
			sign = 0x06;		
			break;	
		case 2:
			bright_value = 0x28;
			sign = 0x06;			
			break;	
		case 1:
			bright_value = 0x18;
			sign = 0x06;			
			break;
		case 0:
			bright_value = 0x08;
			sign = 0x06;			
			break;	
		case -1:
			bright_value = 0x08;
			sign = 0x0e;		
			break;		
		case -2:
			bright_value = 0x18;
			sign = 0x0e;		
			break;	
		case -3:
			bright_value = 0x28;
			sign = 0x0e;		
			break;	
		case -4:
			bright_value = 0x38;
			sign = 0x0e;		
			break;	
	}
	SCCB_WR_Reg(BRIGHT, bright_value);
	SCCB_WR_Reg(SIGN, sign);
}
//对比度设置
//contrast：-4~+4
void OV7725_Contrast(int8_t contrast)
{
	if(contrast >= -4 && contrast <=4)
	{
		SCCB_WR_Reg(CNST,(0x30-(4-contrast)*4));
	}
}
//特效设置
//0:普通模式    
//1,负片
//2,黑白   
//3,偏红色
//4,偏绿色
//5,偏蓝色
//6,复古	    
void OV7725_Special_Effects(uint8_t eft)
{
	switch(eft)
	{
		case 0://正常
			SCCB_WR_Reg(0xa6, 0x06);
			SCCB_WR_Reg(0x60, 0x80);
			SCCB_WR_Reg(0x61, 0x80);
			break;
		case 1://黑白
			SCCB_WR_Reg(0xa6, 0x26);
			SCCB_WR_Reg(0x60, 0x80);
			SCCB_WR_Reg(0x61, 0x80);
			break;	
		case 2://偏蓝
			SCCB_WR_Reg(0xa6, 0x1e);
			SCCB_WR_Reg(0x60, 0xa0);
			SCCB_WR_Reg(0x61, 0x40);	
			break;
		case 3://复古
			SCCB_WR_Reg(0xa6, 0x1e);
			SCCB_WR_Reg(0x60, 0x40);
			SCCB_WR_Reg(0x61, 0xa0);	
			break;	
		case 4://偏红
			SCCB_WR_Reg(0xa6, 0x1e);
			SCCB_WR_Reg(0x60, 0x80);
			SCCB_WR_Reg(0x61, 0xc0);		
			break;	
		case 5://偏绿
			SCCB_WR_Reg(0xa6, 0x1e);
			SCCB_WR_Reg(0x60, 0x60);
			SCCB_WR_Reg(0x61, 0x60);		
			break;	
		case 6://反相
			SCCB_WR_Reg(0xa6, 0x46);
			break;	
	}
}	


//设置图像输出窗口
//width:输出图像宽度,<=320
//height:输出图像高度,<=240
//mode:0，QVGA输出模式；1，VGA输出模式
//QVGA模式可视范围广但近物不是很清晰，VGA模式可视范围小近物清晰
void OV7725_Window_Set(uint16_t width,uint16_t height,uint8_t mode)
{
	uint8_t raw,temp;
	uint16_t sx,sy;
	
	if(mode)
	{
		sx=(640-width)/2;
		sy=(480-height)/2;
		SCCB_WR_Reg(COM7,0x06);		//设置为VGA模式
		SCCB_WR_Reg(HSTART,0x23); 	//水平起始位置
		SCCB_WR_Reg(HSIZE,0xA0); 	//水平尺寸
		SCCB_WR_Reg(VSTRT,0x07); 	//垂直起始位置
		SCCB_WR_Reg(VSIZE,0xF0); 	//垂直尺寸
		SCCB_WR_Reg(HREF,0x00);
		SCCB_WR_Reg(HOutSize,0xA0); //输出尺寸
		SCCB_WR_Reg(VOutSize,0xF0); //输出尺寸
	}
	else
	{
		sx=(320-width)/2;
		sy=(240-height)/2;
		SCCB_WR_Reg(COM7,0x46);		//设置为QVGA模式
		SCCB_WR_Reg(HSTART,0x3f); 	//水平起始位置
		SCCB_WR_Reg(HSIZE, 0x50); 	//水平尺寸
		SCCB_WR_Reg(VSTRT, 0x03); 	//垂直起始位置
		SCCB_WR_Reg(VSIZE, 0x78); 	//垂直尺寸
		SCCB_WR_Reg(HREF,  0x00);
		SCCB_WR_Reg(HOutSize,0x50);	//输出尺寸
		SCCB_WR_Reg(VOutSize,0x78); //输出尺寸
	}
	raw=SCCB_RD_Reg(HSTART);
	temp=raw+(sx>>2);//sx高8位存在HSTART,低2位存在HREF[5:4]
	SCCB_WR_Reg(HSTART,temp);
	SCCB_WR_Reg(HSIZE,width>>2);//width高8位存在HSIZE,低2位存在HREF[1:0]
	
	raw=SCCB_RD_Reg(VSTRT);
	temp=raw+(sy>>1);//sy高8位存在VSTRT,低1位存在HREF[6]
	SCCB_WR_Reg(VSTRT,temp);
	SCCB_WR_Reg(VSIZE,height>>1);//height高8位存在VSIZE,低1位存在HREF[2]
	
	raw=SCCB_RD_Reg(HREF);
	temp=((sy&0x01)<<6)|((sx&0x03)<<4)|((height&0x01)<<2)|(width&0x03)|raw;
	SCCB_WR_Reg(HREF,temp);
	
	SCCB_WR_Reg(HOutSize,width>>2);
	SCCB_WR_Reg(VOutSize,height>>1);
	
	SCCB_RD_Reg(EXHCH);
	temp = (raw|(width&0x03)|((height&0x01)<<2));	
	SCCB_WR_Reg(EXHCH,temp);	
}



