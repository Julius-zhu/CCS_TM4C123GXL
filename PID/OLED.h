/*
 * OLED.h
 *
 *  Created on: 2019Äê7ÔÂ24ÈÕ
 *      Author: zb
 */

#ifndef OLED_OLED_H_
#define OLED_OLED_H_

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

#ifdef __cplusplus
}
#endif

#endif /* OLED_OLED_H_ */
