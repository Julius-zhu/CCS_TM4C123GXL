/*
 * MPU9150.h
 *
 *  Created on: 2019年8月2日
 *      Author: ZJN
 */

#ifndef MPU9150_H_
#define MPU9150_H_

#ifdef __cplusplus
extern "C"
{
#endif

void extern Delay_Ms(uint32_t n);
void extern LCD_WrDat(uint8_t dat);
void extern LCD_WrCmd(uint8_t cmd);
void extern LCD_Set_Pos(uint8_t x, uint8_t y);
void extern LCD_Fill(uint8_t bmp_dat);
void extern LCD_CLS(void);
void extern OLED_Initial(void);
void extern LCD_P6x8Str(unsigned char x,unsigned char y,unsigned char ch[]);
void extern LCD_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
void extern LCD_P14x16Ch(unsigned char x,unsigned char y,unsigned char N);

void extern InitMPU6050();                                                    //初始化MPU6050
void extern Delay5us();
void extern I2C_Start();
void extern I2C_Stop();
void extern I2C_SendACK(bit ack);
bit  extern I2C_RecvACK();
void extern I2C_SendByte(uchar dat);
uchar extern I2C_RecvByte();
void extern I2C_ReadPage();
void extern I2C_WritePage();
void extern display_ACCEL_x();
void extern display_ACCEL_y();
void extern display_ACCEL_z();
unsigned char extern Single_ReadI2C(uchar REG_Address);                        //读取I2C数据
void  Single_WriteI2C(uchar REG_Address,uchar REG_data);    //向I2C写入数据
#ifdef __cplusplus
}
#endif

#endif /* OLED_OLED_H_ */



#endif /* MPU9150_H_ */
