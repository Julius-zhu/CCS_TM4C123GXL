/*
 * MPU9150.c
 *
 *  Created on: 2019��8��2��
 *      Author: ZJN
 */
// SCL=PD0 SDA=PD1
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "hw_gpio.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"
#include "uart.h"
#include "hw_ints.h"
#include "timer.h"
#include "rom.h"
#include "fpu.h"
#include "uart.h"
#include "pwm.h"

#define SMPLRT_DIV      0x19    //�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define CONFIG          0x1A    //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define GYRO_CONFIG     0x1B    //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define ACCEL_CONFIG    0x1C    //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48
#define PWR_MGMT_1      0x6B    //��Դ��������ֵ��0x00(��������)
#define WHO_AM_I            0x75    //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define SlaveAddress    0xD0    //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ
#define MPU9150_SDA_H GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,GPIO_PIN_1)
#define MPU9150_SDA_L GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,GPIO_PIN_1)
#define MPU9150_SCL_H GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0,GPIO_PIN_0)
#define MPU9150_SCL_H GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0,GPIO_PIN_0)
//****************************************
//�������ͼ�����
//****************************************
unsigned char dis[6];                           //��ʾ����(-511��512)���ַ�����
int dis_data;                       //����

//����ת�ַ���
void lcd_printf(unsigned char *s,int temp_data)
{
    if(temp_data<0)
    {
        temp_data=-temp_data;
        *s='-';
    }
    else *s=' ';

    *++s =temp_data/10000+0x30;
    temp_data=temp_data%10000;     //ȡ������

    *++s =temp_data/1000+0x30;
    temp_data=temp_data%1000;     //ȡ������

    *++s =temp_data/100+0x30;
    temp_data=temp_data%100;     //ȡ������
    *++s =temp_data/10+0x30;
    temp_data=temp_data%10;      //ȡ������
    *++s =temp_data+0x30;
}

void MPU9150_Initial(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_0|GPIO_PIN_1);
}
void MPU9150_Start()
{
    MPU9150_SDA_H;                    //����������
    MPU9150_SCL_H;                    //����ʱ����
    delay_us(5);                      //��ʱ
    MPU9150_SDA_L;                    //�����½���
    delay_us(5);                      //��ʱ
    MPU9150_SCL_L;                    //����ʱ����
}

void MPU9150_Stop()
{
    MPU9150_SDA_L;                    //����������
    MPU9150_SCL_H;                    //����ʱ����
    delay_us(5);                      //��ʱ
    MPU9150_SDA_H;                    //����������
    delay_us(5);                      //��ʱ
}

void MPU9150_SendACK(bit ack)
{
    GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,ack<<1);                  //дӦ���ź�
    MPU9150_SCL_H;                    //����ʱ����
    delay_us(5);                      //��ʱ
    MPU9150_SCL_L;                    //����ʱ����
    delay_us(5);                      //��ʱ
}

uint8_t MPU9150_RecvACK()
{
    uint8_t CY;
    MPU9150_SCL_H;                    //����ʱ����
    delay_us(5);                      //��ʱ
    CY = GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_1);                   //��Ӧ���ź�
    MPU9150_SDA_L;                    //����ʱ����
    delay_us(5);                      //��ʱ
    return CY;
}


void delay_us(uint16_t delay_time)
{
    SysCtlDelay(SysCtlClockGet()/3/1000000*delay_time);
}
void delay_ms(uint16_t delay_time)
{
    SysCtlDelay(SysCtlClockGet()/3/1000*delay_time);
}


void MPU9150_WrDat(uint8_t dat)
{
    uint8_t i = 0;
    GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0,0x00);
    for(i=0;i<8;i++)
    {
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2,0x00);
        if(dat&0x80)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,GPIO_PIN_3);
        }
        else
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0x00);
        }
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2,GPIO_PIN_2);
        dat<<=1;;
    }
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,GPIO_PIN_2);
}

void LCD_WrCmd(uint8_t cmd)
{
    uint8_t i = 0;
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0x00);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0x00);
    for(i=0;i<8;i++)
    {
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2,0x00);
        if(cmd&0x80)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,GPIO_PIN_3);
        }
        else
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0x00);
        }
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2,GPIO_PIN_2);
        cmd<<=1;;
    }
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,GPIO_PIN_2);
}


void OLED_Initial(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,0);
    Delay_Ms(50);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,GPIO_PIN_3);
    LCD_WrCmd(0xae);//--turn off oled panel
    LCD_WrCmd(0x00);//---set low column address
    LCD_WrCmd(0x10);//---set high column address
    LCD_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    LCD_WrCmd(0x81);//--set contrast control register
    LCD_WrCmd(0xcf); // Set SEG Output Current Brightness
    LCD_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0���ҷ���0xa1����
    LCD_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0���·���0xc8����
    LCD_WrCmd(0xa6);//--set normal display
    LCD_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
    LCD_WrCmd(0x3f);//--1/64 duty
    LCD_WrCmd(0xd3);//-set display offset Shift Mapping RAM Counter (0x00~0x3F)
    LCD_WrCmd(0x00);//-not offset
    LCD_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
    LCD_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
    LCD_WrCmd(0xd9);//--set pre-charge period
    LCD_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    LCD_WrCmd(0xda);//--set com pins hardware configuration
    LCD_WrCmd(0x12);
    LCD_WrCmd(0xdb);//--set vcomh
    LCD_WrCmd(0x40);//Set VCOM Deselect Level
    LCD_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
    LCD_WrCmd(0x02);//
    LCD_WrCmd(0x8d);//--set Charge Pump enable/disable
    LCD_WrCmd(0x14);//--set(0x10) disable
    LCD_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
    LCD_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7)
    LCD_WrCmd(0xaf);//--turn on oled panel
    LCD_Fill(0x00);  //��ʼ����
    LCD_Set_Pos(0,0);
}

void LCD_P6x8Str(unsigned char x,unsigned char y,unsigned char ch[])
{
  unsigned char c=0,i=0,j=0;
  while (ch[j]!='\0')
  {
    c =ch[j]-32;
    if(x>126){x=0;y++;}
    LCD_Set_Pos(x,y);
    for(i=0;i<6;i++)
      LCD_WrDat(F6x8[c][i]);
    x+=6;
    j++;
  }
}

void LCD_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[])
{
  unsigned char c=0,i=0,j=0;
  while (ch[j]!='\0')
  {
    c =ch[j]-32;
    if(x>120){x=0;y++;}
    LCD_Set_Pos(x,y);
    for(i=0;i<8;i++)
      LCD_WrDat(F8X16[c*16+i]);
    LCD_Set_Pos(x,y+1);
    for(i=0;i<8;i++)
      LCD_WrDat(F8X16[c*16+i+8]);
    x+=8;
    j++;
  }
}

//��������ַ���
void LCD_P14x16Ch(unsigned char x,unsigned char y,unsigned char N)
{
    unsigned char wm=0;//,ii = 0;
    unsigned int adder=28*N;
    LCD_Set_Pos(x , y);
    for(wm = 0;wm < 14;wm++)
    {
        LCD_WrDat(F14x16[adder]);
        adder += 1;
    }
    LCD_Set_Pos(x,y + 1);
    for(wm = 0;wm < 14;wm++)
    {
        LCD_WrDat(F14x16[adder]);
        adder += 1;
    }
}
