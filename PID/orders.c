/*
 * orders.c
 *
 *  Created on: 2019年7月12日
 *      Author: SLW
 */

#include "string.h"
#include <stdint.h>
#include "inc/hw_memmap.h"
#include <stdbool.h>

extern uint32_t CH[5];
extern uint8_t mode;
extern char feedback[50];
extern float height;
extern float kp[3];
extern float ki[3];
extern float kd[3];
extern float height_history[5];
extern float MID1,MID2,MID3,MID4;
extern float tkf_MID1;
extern uint8_t systick;
extern uint8_t init_flag;
extern uint8_t ch1show_flag;
extern float height_set;
extern uint16_t tkfcoef,landcoef;
extern uint8_t land_flag;
extern uint8_t lock_cnt,unlock_cnt;
extern uint8_t pos_flag;
extern void UART3Send(const uint8_t *pucBuffer,uint32_t ulCount);
extern void CHSet(uint8_t counter,uint32_t duty);
extern bool UARTCharPutNonBlocking(uint32_t ui32Base, unsigned char ucData);


void lock()
{
    land_flag=0;
    CH[0]=7500;
    CH[1]=7500;
    CH[2]=5000;
    CH[3]=5000;
    CH[4]=7500;
    mode=0;
    ch1show_flag=0;
    pos_flag=0;
    tkfcoef=80;
    landcoef=100;
    CHSet(1,CH[0]);
    CHSet(2,CH[1]);
    CHSet(3,CH[2]);
    CHSet(4,7500);//先把CH4归中一次，才能正常锁定
    CHSet(5,CH[4]);
    systick=1;
    while(systick<20);
    systick=0;
    CHSet(4,CH[3]);
    height_history[0]=-1;
    height_history[1]=-1;
    height_history[2]=-1;
    height_history[3]=-1;
    height_history[4]=-1;   //清空高度值历史值，为下一次飞行做准备
    UART3Send((uint8_t*)"LOCK",4);
}

void unlock()
{
    land_flag=0;
    pos_flag=0;
    CH[0]=7500;
    CH[1]=7500;
    CH[2]=5000;
    CH[3]=10000;
    CH[4]=7500;
    mode=0;
    tkfcoef=80;
    landcoef=100;
    CHSet(1,CH[0]);
    CHSet(2,CH[1]);
    CHSet(3,CH[2]);
    CHSet(4,7500);//先把CH4归中一次，才能正常解锁
    CHSet(5,CH[4]);
    systick=1;
    while(systick<20);
    systick=0;
    CHSet(4,CH[3]);
    UART3Send((uint8_t*)"UNLOCK",6);
}

void mid()
{
    CH[0]=MID1;
    CH[1]=MID2;
    CH[3]=MID4;
    mode=0;
    CHSet(1,CH[0]);
    CHSet(2,CH[1]);
    CHSet(4,CH[3]);
    UART3Send((uint8_t*)"MID",3);
}

void takeoff()
{
    CH[4]=7500;
    CHSet(5,CH[4]);

    mode=1;
    init_flag=1;
    UART3Send((uint8_t*)"takeoff",7);
}

void land()
{
    CH[4]=7500;
    CHSet(5,CH[4]);
    mode=2;
    UART3Send((uint8_t*)"land",4);
}

void althold()
{
    CH[4]=7500;
    CHSet(5,CH[4]);
    mode=3;
    ch1show_flag=0;
    UART3Send((uint8_t*)"althold",7);
}

void poshold()
{
    CH[4]=7500;
    CHSet(5,CH[4]);
    mode=4;
    //pos_flag=1;
    CH[0]=MID1;
    UART3Send((uint8_t*)"poshold",7);
}

void ROLplus()
{
    CH[0]+=20;
    CHSet(1,CH[0]);
    UART3Send((uint8_t*)"ROL+",4);
}

void ROLminus()
{
    CH[0]-=20;
    CHSet(1,CH[0]);
    UART3Send((uint8_t*)"ROL-",4);
}

void PITplus()
{
    CH[1]+=20;
    CHSet(2,CH[1]);
    UART3Send((uint8_t*)"PIT+",4);
}

void PITminus()
{
    CH[1]-=20;
    CHSet(2,CH[1]);
    UART3Send((uint8_t*)"PIT-",4);
}

void THRplus()
{
    CH[2]+=50;
    if(CH[2]>9000){CH[2]=9000;}//限制油门最大为80%
    CHSet(3,CH[2]);
    mode=0;
    UART3Send((uint8_t*)"UPD+",4);
}

void THRminus()
{
    CH[2]-=50;
    if(CH[2]<5000){CH[2]=5000;}//限制油门下限
    CHSet(3,CH[2]);
    mode=0;
    UART3Send((uint8_t*)"UPD-",4);
}

void YAWplus()
{
    CH[3]+=50;
    CHSet(4,CH[3]);
    UART3Send((uint8_t*)"YAW+",4);
}

void YAWminus()
{
    CH[3]-=50;
    CHSet(4,CH[3]);
    UART3Send((uint8_t*)"YAW-",4);
}

void stablize_mode()
{
    CH[4]=7500;
    CHSet(5,CH[4]);
    UART3Send((uint8_t*)"stable",6);
}

void althold_mode()
{
    mode=0;
    CH[4]=10000;
    CHSet(5,CH[4]);
    //CH[2]=7500;                 //飞控定高要把油门调至50%
    //CHSet(3,CH[2]);
    UART3Send((uint8_t*)"holdalt",7);
}

void land_mode()
{
    mode=0;
    CH[4]=5000;
    CHSet(5,CH[4]);
    UART3Send((uint8_t*)"down",4);
}

void ret_althold()
{
    CH[0]=MID1;
    CH[1]=MID2;
    mode=3;
    pos_flag=0;
    CHSet(1,CH[0]);
    CHSet(2,CH[1]);
    UART3Send((uint8_t*)"reth",4);
}

void getorder(char *ch)
{
        //蓝牙指令集
    if(strncmp(ch,"LOCK",4)==0)//锁定飞控
    {
        lock();
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"UNL",3)==0)//解锁飞控
    {
        unlock();
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"MID",3)==0)//除油门外，全部归中
    {
        mid();
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"PIT+",4)==0)//向前俯身
    {
        PITplus();
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"PIT-",4)==0)
    {
        PITminus();
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"ROL+",4)==0)//向右滚转
    {
        ROLplus();
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"ROL-",4)==0)
    {
        ROLminus();
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"UPD+",4)==0)//增加推力
    {
        THRplus();
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"UPD-",4)==0)
    {
        THRminus();
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"YAW+",4)==0)//向右旋转
    {
        YAWplus();
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"YAW-",4)==0)
    {
        YAWminus();
        memset(ch,0,sizeof(ch));
    }
    if (strncmp(ch,"s",1) ==0)//设置各个通道参数 格式s500050005000
    {
        CH[0] = (ch[1]-'0') * 1000 + (ch[2]-'0') * 100 + (ch[3]-'0') * 10 + (ch[4]-'0');
        CH[1] = (ch[5]-'0') * 1000 + (ch[6]-'0') * 100 + (ch[7]-'0') * 10 + (ch[8]-'0');
        CH[2] = (ch[9]-'0') * 1000 + (ch[10]-'0') * 100 + (ch[11]-'0') * 10 + (ch[12]-'0');
        CHSet(1, CH[0]);
        CHSet(2, CH[1]);
        CHSet(3, CH[2]);
        mode=0;
        memset(ch, 0, sizeof(ch));
    }
    if (strncmp(ch, "GET",3)==0)//获取各个通道参数 格式5000+5000+5000+5000
    {
        UART3Send((uint8_t*)"GET",3);
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
        systick=1;
        while(systick<5);
        systick=0;
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
        systick=1;
        while(systick<5);
        systick=0;
        feedback[0]=(int)height/1000+'0';
        feedback[1]=((int)height-(int)height/1000*1000)/100+'0';
        feedback[2]=((int)height-(int)height/100*100)/10+'0';
        feedback[3]=(int)height%10+'0';
        UART3Send((uint8_t*)"height=",7);
        UARTCharPutNonBlocking(UART3_BASE,feedback[0]);
        UARTCharPutNonBlocking(UART3_BASE,feedback[1]);
        UARTCharPutNonBlocking(UART3_BASE,feedback[2]);
        UARTCharPutNonBlocking(UART3_BASE,feedback[3]);
        memset(ch, 0, sizeof(ch));
    }


    //起飞
    if(strncmp(ch,"tkf",3)==0)
    {
        takeoff();
        memset(ch,0,sizeof(ch));
    }
    //飞控自稳
    if(strncmp(ch,"able",4)==0)
    {
        stablize_mode();
        memset(ch,0,sizeof(ch));
    }
    //降落
    if(strncmp(ch,"land",4)==0)
    {
        land();
        memset(ch,0,sizeof(ch));
    }
    //飞控降落
    if(strncmp(ch,"down",4)==0)
    {
        land_mode();
        memset(ch,0,sizeof(ch));
    }
    //定高
    if(strncmp(ch,"alth",4)==0)
    {
        althold();
        memset(ch,0,sizeof(ch));
    }
    //飞控定高
    if(strncmp(ch,"halt",4)==0)
    {
        althold_mode();
        memset(ch,0,sizeof(ch));
    }
    //定高加定点
    if(strncmp(ch,"posh",4)==0)
    {
        poshold();
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"hset",4)==0)
    {
        height_set=(ch[4]-'0')*1000+(ch[5]-'0')*100+(ch[6]-'0')*10;
        UART3Send((uint8_t*)"hset",4);
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"1set",4)==0)
    {
        MID1=(ch[4]-'0')*1000+(ch[5]-'0')*100+(ch[6]-'0')*10+(ch[7]-'0');
        UART3Send((uint8_t*)"1set",4);
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"2set",4)==0)
    {
        MID2=(ch[4]-'0')*1000+(ch[5]-'0')*100+(ch[6]-'0')*10+(ch[7]-'0');
        UART3Send((uint8_t*)"2set",4);
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"3set",4)==0)
    {
        MID3=(ch[4]-'0')*1000+(ch[5]-'0')*100+(ch[6]-'0')*10+(ch[7]-'0');
        UART3Send((uint8_t*)"3set",4);
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"reth",4)==0)
    {
        ret_althold();
        memset(ch,0,sizeof(ch));
    }

    /*if(strncmp(ch,"geth",4)==0)
    {
        feedback[0]=(int)height/1000+'0';
        feedback[1]=((int)height-(int)height/1000*1000)/100+'0';
        feedback[2]=((int)height-(int)height/100*100)/10+'0';
        feedback[3]=(int)height%10+'0';
        UART3Send((uint8_t*)"height=",7);
        UARTCharPut(UART3_BASE,feedback[0]);
        UARTCharPut(UART3_BASE,feedback[1]);
        UARTCharPut(UART3_BASE,feedback[2]);
        UARTCharPut(UART3_BASE,feedback[3]);
        memset(ch,0,sizeof(ch));
    }*/
    //以下注释掉的代码为调整PID参数的指令，可在蓝牙串口中使用

                                                                                                    //1通道PID
    if(strncmp(ch,"1kp",3)==0)                                          //格式1kpx.xx
    {
        kp[0]=(ch[3]-'0')+0.1*(ch[5]-'0')+0.01*(ch[6]-'0');
        //sprintf(feedback,"kp=%d.%d%d",ch[3]-'0',ch[5]-'0',ch[6]-'0');
        //UART3Send((uint8_t*)feedback,10);                               //反馈kp修改后的值
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"1ki",3)==0)                                          //格式1kix.xxx
    {
        ki[0]=(ch[3]-'0')+0.1*(ch[5]-'0')+0.01*(ch[6]-'0')+0.001*(ch[7]-'0');
        //sprintf(feedback,"ki=%d.%d%d",ch[2]-'0',ch[4]-'0',ch[5]-'0');
        //UART0Send((uint8_t*)feedback,10);                               //反馈ki修改后的值
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"1kd",3)==0)
    {
        kd[0]=(ch[3]-'0')+0.1*(ch[5]-'0')+0.01*(ch[6]-'0');
        //sprintf(feedback,"kd=%d.%d%d",ch[2]-'0',ch[4]-'0',ch[5]-'0');
        //UART0Send((uint8_t*)feedback,10);                               //反馈kd修改后的值
        memset(ch,0,sizeof(ch));
    }


                                                                                                        //2通道PID
    if(strncmp(ch,"2kp",3)==0)                                          //格式2kpx.xx
    {
        kp[1]=(ch[3]-'0')+0.1*(ch[5]-'0')+0.01*(ch[6]-'0');
        //sprintf(feedback,"kp=%d.%d%d",ch[3]-'0',ch[5]-'0',ch[6]-'0');
        //UART3Send((uint8_t*)feedback,10);                               //反馈kp修改后的值
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"2ki",3)==0)                                          //格式2kix.xxx
    {
        ki[1]=(ch[3]-'0')+0.1*(ch[5]-'0')+0.01*(ch[6]-'0')+0.001*(ch[7]-'0');
        //sprintf(feedback,"ki=%d.%d%d",ch[2]-'0',ch[4]-'0',ch[5]-'0');
        //UART0Send((uint8_t*)feedback,10);                               //反馈ki修改后的值
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"2kd",3)==0)
    {
        kd[1]=(ch[3]-'0')+0.1*(ch[5]-'0')+0.01*(ch[6]-'0');
        //sprintf(feedback,"kd=%d.%d%d",ch[2]-'0',ch[4]-'0',ch[5]-'0');
        //UART0Send((uint8_t*)feedback,10);                               //反馈kd修改后的值
        memset(ch,0,sizeof(ch));
    }


                                                                                                        //3通道PID
    if(strncmp(ch,"3kp",3)==0)                                          //格式3kpx.xx
    {
        kp[2]=(ch[3]-'0')+0.1*(ch[5]-'0')+0.01*(ch[6]-'0');
        //sprintf(feedback,"kp=%d.%d%d",ch[3]-'0',ch[5]-'0',ch[6]-'0');
        //UART3Send((uint8_t*)feedback,10);                               //反馈kp修改后的值
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"3ki",3)==0)                                          //格式3kix.xxx
    {
        ki[2]=(ch[3]-'0')+0.1*(ch[5]-'0')+0.01*(ch[6]-'0')+0.001*(ch[7]-'0');
        //sprintf(feedback,"ki=%d.%d%d",ch[2]-'0',ch[4]-'0',ch[5]-'0');
        //UART0Send((uint8_t*)feedback,10);                               //反馈ki修改后的值
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"3kd",3)==0)
    {
        kd[2]=(ch[3]-'0')+0.1*(ch[5]-'0')+0.01*(ch[6]-'0');
        //sprintf(feedback,"kd=%d.%d%d",ch[2]-'0',ch[4]-'0',ch[5]-'0');
        //UART0Send((uint8_t*)feedback,10);                               //反馈kd修改后的值
        memset(ch,0,sizeof(ch));
    }

    /*
    if(strncmp(ch,"set",3)==0)
    {
        set=1000*(ch[2]-'0')+100*(ch[3]-'0')+10*(ch[4]-'0')+(ch[5]-'0');
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"out",3)==0)
    {
        output=1000*(ch[2]-'0')+100*(ch[3]-'0')+10*(ch[4]-'0')+(ch[5]-'0');
        memset(ch,0,sizeof(ch));
    }
    if(strncmp(ch,"res",3)==0)
    {
        cnt=0;
        e=output-set;
        e_int=e;
        e_dif=e;
        memset(ch,0,sizeof(ch));
    }*/
}



