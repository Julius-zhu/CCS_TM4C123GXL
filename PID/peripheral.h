/*
 * peripheral.h
 *
 *  Created on: 2019年8月6日
 *      Author: ASUS
 */

//OLED D0-PB2,D1-PB3,RES-PE3,DC-PE2,CS-PE1
//蜂鸣器 IO口-PA2
//按钮 从上到下依次为 PD0 PD1 PD2 PD3
//发光二极管 Y-PA5,R-PA6,B-PA7



//PF2 PF3 P30(PE0) PF0 PF4 PA3 PA4

#ifndef PERIPHERAL_H_
#define PERIPHERAL_H_


void GPIO_Init();
void buzzer_Init();
void button_Init();
void LED_Init();
void peripheral_Init();

void buzzer_ON();
void buzzer_OFF();
void LED_Y_ON();
void LED_Y_OFF();
void LED_R_ON();
void LED_R_OFF();
void LED_B_ON();
void LED_B_OFF();

#endif /* PERIPHERAL_H_ */
