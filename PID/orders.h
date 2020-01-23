/*
 * orders.h
 *
 *  Created on: 2019Äê7ÔÂ12ÈÕ
 *      Author: ASUS
 */

#ifndef ORDERS_H_
#define ORDERS_H_

void lock();
void unlock();
void mid();
void takeoff();
void land();
void althold();
void poshold();
void ROLplus();
void ROLminus();
void PITplus();
void PITminus();
void THRplus();
void THRminus();
void YAWplus();
void YAWminus();
void stablize_mode();
void althold_mode();
void land_mode();
void ret_althold();

void getorder(char *ch);


#endif /* ORDERS_H_ */
