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
#include "orders.h"
#include "OLED.h"
#include "peripheral.h"


#define horizon 80
#define vertical 120                                                    //图像的大小
float kp[3],ki[3],kd[3];                                                //PID参数
float e[3];                                                             //当前误差
float e_int[3],e_dif[3],e_temp[3],e_temp_past[2];                                                //误差的积分与微分
float p[3];
float height_set=800,height,height_history[5]={-1,-1,-1,-1,-1};                         //高度的目标值，高度（为平均值），高度的历史值
volatile int x_target=horizon/2,y_target=vertical/2;                                                         //目标点的横纵坐标，
//int x_history[3],y_history[3];
int x_center=horizon/2,y_center=vertical/2;                                             //摄像头的中心值（图像大小80×120）
uint8_t pos_cnt=0;                                                              //当目标点不在视野内（即x_target,y_target为0），开始计数，到10之前一直保持上次的调整量
uint8_t flag_out=0;                                                             //置1表明目标点已经在视野外
volatile float distance;                                                         //超声模块获取的距离值
uint8_t s_cnt=0,height_i;
uint8_t cnt=0;
int tick1,tick2,ticks;                                                  //超声模块的计数值
uint32_t ui32Period;
uint8_t flag1=0,flag2=0,flag3=0;
uint8_t mode=0;                                                         //0为手动模式，1为起飞模式，2为降落模式，3为定高模式,4为定高加定点模式
uint8_t flag_s1=0,flag_s2=0;
uint8_t systick;
uint32_t delay;
uint32_t land_cnt;
uint8_t ctl_cnt;
char ch[50];                                                            //ch存放蓝牙指令
char data[4];                                                          //data存放目标点坐标
char angle[8];                                                          //angle存放陀螺仪的角度数据
char feedback[50];
float out[20];

float MID1=7500,MID2=7500,MID3=7950,MID4=7500;                                              //四个通道的中间值，MID3油门通道为悬停时的PWM占空比
float tkf_MID1=7341,tkf_MID2=7350;
uint8_t init_flag;
float Slimit[3]={7731,0,8700},Llimit[3]={6931,0,7800};                                                            //1-3通道上下限
float LANDSPEED=7800,LANDSPEED_TRUE;
uint16_t tkfcoef=80,landcoef=100;                                                                     //起飞降落系数
//7318  7550 2019/7/11
//7340  7315 2019/7/12 14:23
//7335 7320 6500 2019/7/12 20:24
//新飞控
//7425 7320 2019/7/19 11:45
//7422 7321 2019/7/19 13:35
//7309 7348 2019/7/19 19:39
//7365 7333 2019/7/20 15:34
//防撞圈 新脚架
//7548 7610 2019/7/24 20:34
//7530 7610 2019/7/25 9:58

//7310 7335 2019/7/25 20:49




float Duty;
uint16_t h_send;
int p0_send;
int p1_send;
uint8_t ch1show_flag;
uint32_t CH[5];
uint8_t land_flag;
uint8_t lock_cnt;
uint8_t unlock_cnt;
uint8_t pos_flag;


char LCD_disp[10]="TASK";


void TIMER0A1A_Init();                                                  //timer0捕捉超声模块反馈的高电平边沿，timer1负责定时激励超声模块
void TIMER2A_Init();                                                    //timer2控制PID的刷新频率
void GPIOPWM_Init();
void UART0_Init();
void UART1_Init();
void UART3_Init();                                                      //UART3负责蓝牙串口通信
void UART4_Init();                                                      //UART4负责与处理图像的单片机通信
void GPIOB_Init();                                                      //GPIOB_Pin7作为超声模块的触发引脚，Pin6作为timer0A的CCP0
void Controller_Init();                                                 //主控板外设初始化

double GetV1(int data[],int length);                                    //获取速度
void UART0Send(const uint8_t *pucBuffer,uint32_t ulCount);
void UART1Send(const uint8_t *pucBuffer,uint32_t ulCount);
void UART3Send(const uint8_t *pucBuffer,uint32_t ulCount);
void UART4Send(const uint8_t *pucBuffer,uint32_t ulCount);
void CHSet(uint8_t counter,uint32_t duty);                              //五通道调节飞控模式，占空比5%-10%

int main(void)

{
    FPUEnable();
    FPULazyStackingEnable();

    SysCtlClockSet(SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_SYSDIV_5);//主频40M，与timer0的计数范围大小匹配，若要修改则timer0 load值应相应改动
    ui32Period=SysCtlClockGet();

    GPIO_Init();                //使能并解锁GPIO A-F
    peripheral_Init();

    Controller_Init();

//    OLED_Initial();
//
//    LCD_P8x16Str(0,0,"TASK:");
//    LCD_P8x16Str(40,0,"0");

    IntPriorityGroupingSet(3);                                      //配置中断优先级
    IntPrioritySet(INT_TIMER0A,0x0);
    IntPrioritySet(INT_TIMER1A,0x20);
    IntPrioritySet(INT_UART3,0x0);


    CH[0]=7500;
    CH[1]=7500;
    CH[2]=5000;
    CH[3]=7500;
    CH[4]=7500;


    //设置PID的参数值
    kp[0]=1.50;         ///kp[0]在计算中被乘上了10
    ki[0]=0.01;
    kd[0]=1.40;         //kd[0]在计算中被乘上了50

    //一通道
    //0.99 0.10 1.31
    //0.80 0.02 0.95
    //0.87*10 0.05(1000) 1.12*50   2019/8/1
    //0.96*10 0.03(3000) 1.40*50   2019/8/3

    kp[1]=0.75;         //kp[1]在计算中被乘上了10
    ki[1]=0.01;
    kd[1]=0.70;            //kd[1]在计算中被乘上了50

    //0.43*10 0.03 0.58*50 2019/8/1

    kp[2]=1.30;
    ki[2]=0.027;
    kd[2]=2.30;         //kd[2]在计算中被乘上了10


    //三通道
    //1.4 0.018(50000) 1.9*10 2019/7/28
    //1.4 0.020(50000) 1.9*10 2019/7/31
    //1.3 0.027(50000) 2.3*10 2019/8/1


    //sprintf(feedback,"kp=%f",output);
    //UART0Send((uint8_t*)feedback,20);

    while(1)
    {

        getorder(ch);                           //处理蓝牙发送的指令


        if(land_flag==1)
        {
            if(height<250)
            {
                CH[2]=LANDSPEED_TRUE-50;
                if(CH[2]>8100){CH[2]=8100;}
                else if(CH[2]<7800){CH[2]=7800;}
                CHSet(3,CH[2]);
            }
            if(height<150)                  //高度低于15cm直接锁定
            {
                lock();
            }
        }
//        if(height>1500)                                                         //如果高度超过1.5m，自动开启降落模式
//        {
//            mode=2;
//        }



        if(mode==1)                                                                     //模式1，起飞
        {
            if(init_flag)                                                               //三个通道初始化
            {
                CH[0]=tkf_MID1;
                CH[1]=tkf_MID2;
                CH[3]=MID4;
                CHSet(1,CH[0]);
                CHSet(2,CH[1]);
                CHSet(4,CH[3]);
                e[0]=0;e_int[0]=0;e_dif[0]=0;                                         //清空所有误差值
                e[1]=0;e_int[1]=0;e_dif[1]=0;
                e[2]=0;e_int[2]=0;e_dif[2]=0;
                delay=1;

                init_flag=0;                                                            //初始化只进行一次
            }

            if(delay>10)
            {
    //            LCD_P8x16Str(0,0,"TASK:");
    //            LCD_P8x16Str(40,0,"");


                CH[2]=MID3*tkfcoef/100;
                tkfcoef+=1;
                if(tkfcoef>100)
                {
                    tkfcoef=80;
                    mode=0;
                    delay=0;
                }
                if(CH[2]<5000){CH[2]=5000;}                                           //限制油门上下限
                if(CH[2]>8100){CH[2]=8100;}
                CHSet(3,CH[2]);
                delay=1;                    //延时20/60秒
            }


            /*if(height>500)                                                          //如果高度达到0.5m，自动开启定高模式
            {
                mode=3;
            }*/
        }

        if(mode==2)                                                                    //降落模式
        {

            //CH[0]=MID1;
            //CH[1]=MID2;
            LANDSPEED_TRUE=LANDSPEED-ki[2]*e_int[2];
            CH[2]=LANDSPEED_TRUE;
            if(CH[2]>8100){CH[2]=8100;}
            else if(CH[2]<7800){CH[2]=7800;}
            mode=0;
            CHSet(3,CH[2]);

            e[0]=0;e_int[0]=0;e_dif[0]=0;e_temp[0]=0;e_temp_past[0]=0;                                         //清空所有误差值
            e[1]=0;e_int[1]=0;e_dif[1]=0;e_temp[1]=0;e_temp_past[1]=0;
            e[2]=0;e_int[2]=0;e_dif[2]=0;e_temp[2]=0;

            land_flag=1;
        }


        //此处需要一个if语句，判断flag，做到与超声模块刷新同步；或直接将高度值刷新加入distance的计算函数中
        //从超声模块获取高度值，并对其进行滤波
        if(flag_s1&&flag_s2)
        {
            flag_s1=0;
            flag_s2=0;
            ticks=(tick1-tick2);
            if(ticks<0)
            {
                ticks+=65536*256;   //      65536*256
            }
            distance=ticks*0.034/8;//单位mm
            s_cnt=0;

            if(distance>0&&distance<2000)                         //distance单位为mm
            {
                for(height_i=1;height_i<5;++height_i)
                {
                    height_history[height_i-1]=height_history[height_i];//1->0,2->1,3->2,4->3,空出4用作输入当前高度值
                }

                if(((distance<height+300)&&(distance>height-300))||(height_history[0]==-1))//除去波动过大的高度值,但在height_history[0]未获得有效值前，不开启滤波
                {
                    height_history[4]=distance;//获取当前高度值
                }

                height=(height_history[0]+height_history[1]+height_history[2]+height_history[3]+height_history[4])/5;


                h_send=height;

                if(flag3&&(mode==3||mode==4))
                {
                    flag3=0;//清标志位，以20Hz的频率刷新PID的修正值
                    e_temp[2]=e[2];                                                 //上一次的误差值放入e_temp[2]
                    e[2]=height-height_set;
                    //在飞行器达到一定高度后再开始对于e_int项的累加                                 //这个阈值或许会对PID的调整效果产生影响，须注意
                    if(height>=200)
                    {
                        e_int[2]+=e[2];
                        if(e_int[2]>50000){e_int[2]=50000;}
                        if(e_int[2]<-50000){e_int[2]=-50000;}
                    }
                    e_dif[2]=e[2]-e_temp[2];

                    p[2]=kp[2]*e[2]+ki[2]*e_int[2]+kd[2]*e_dif[2]*10;
                    CH[2]=MID3-p[2];                                                 //获得最终的该通道PID控制后的占空比值，p[2]或许需要乘上一个系数（也可以将该系数分配到kp,ki,kd中？）


                    if(CH[2]>Slimit[2]){CH[2]=Slimit[2];}
                    if(CH[2]<Llimit[2]){CH[2]=Llimit[2];}
                    //调用函数，修改该通道的PWM输出占空比
                    CHSet(3,CH[2]);
                }
            }
            IntEnable(INT_TIMER0A);
        }

        if(mode==4||(pos_flag==1))
        {
            ch1show_flag=1;
            if(pos_cnt>13)
            {
                if(height>=400){mode=3;}            //高度在0.5m以上，进行定高飞行，否则降落
                if(height>=0&&height<400){mode=2;}
                e[0]=0;         //清除误差值，避免对下次开启定点定高模式产生影响
                e_temp[0]=0;
                e_temp_past[0]=0;
                e[1]=0;
                e_temp[1]=0;
                e_temp_past[1]=0;
                pos_cnt=0;
                CH[0]=MID1;
                CH[1]=MID2;
                CHSet(1,CH[0]);
                CHSet(2,CH[1]);
                ch1show_flag=0;
            }
            else
            {
                //对于位置坐标变量的定义，须加volatile

                if(flag1)
               {
                   flag1=0;//清标志位，以20Hz的频率刷新PID的修正值
                   //第二通道，控制前后
                   e_temp[0]=e[0];
                   //y向前为正,若目标在视野内，则刷新误差值
                   e[0]=x_center-x_target;                               //注意e[0]的正负与通道PWM大小代表的方向之间的关系,左：占空比减小，右：占空比增大
                   if(e[0]<30&&e[0]>-30)                                                              //与目标点在一定范围内再计算积分项
                   {
                       e_int[0]+=e[0];
                       if(e_int[0]>3000){e_int[0]=3000;}
                       if(e_int[0]<-3000){e_int[0]=-3000;}
                   }
                   e_dif[0]=e[0]-e_temp[0];

//                   p[0]=kd[0]*e_dif[0]*50;
//                   if(p[0]>100){p[0]=100;}
//                   if(p[0]<-300){p[0]=-300;}
//                   p[0]=p[0]+kp[0]*e[0]*10+ki[0]*e_int[0];
//                   if(p[0]>100){p[0]=100;}
//                   if(p[0]<-500){p[0]=-500;}

                   p[0]=kp[0]*e[0]*10;
                   if(p[0]>200){p[0]=200;}
                   if(p[0]<-200){p[0]=-200;}
                   p[0]=p[0]+kd[0]*e_dif[0]*50+ki[0]*e_int[0];
                   if(p[0]>300){p[0]=300;}
                   if(p[0]<-300){p[0]=-300;}
                   //由于图像的x方向正向与占空比大小的正向不一致，‘-’变‘+’
                   CH[0]=MID1-p[0];

                   p0_send=CH[0];
                   //调用函数，修改该通道的PWM输出占空比
                   CHSet(1,CH[0]);

               }

                if(flag2)
                {
                    flag2=0;//清标志位，以20Hz的频率刷新PID的修正值
                    //第二通道，控制前后
                    e_temp[1]=e[1];
                    //y向前为正,若目标在视野内，则刷新误差值
                    e[1]=y_center-y_target;                               //注意e[1]的正负与通道PWM大小代表的方向之间的关系,俯：占空比减小，仰：占空比增大
                    if(e[1]<45&&e[1]>-45)                                                              //与目标点在一定范围内再计算积分项
                    {
                        e_int[1]+=e[1];
                        if(e_int[1]>3000){e_int[1]=3000;}
                        if(e_int[1]<-3000){e_int[1]=-3000;}
                    }
                    e_dif[1]=e[1]-e_temp[1];

                    p[1]=kp[1]*e[1]*10;
                    if(p[1]>200){p[1]=200;}
                    if(p[1]<-200){p[1]=-200;}
                    p[1]=p[1]+kd[1]*e_dif[1]*50+ki[1]*e_int[1];
                    if(p[1]>300){p[1]=300;}
                    if(p[1]<-300){p[1]=-300;}
                    //由于图像的y方向正向与占空比大小的正向不一致，‘-’变‘+’
                    p1_send=CH[1];
                    CH[1]=MID2+p[1];                                                 //获得最终的该通道PID控制后的占空比值，p[2]或许需要乘上一个系数（也可以将该系数分配到kp,ki,kd中？）

                    //调用函数，修改该通道的PWM输出占空比
                    CHSet(2,CH[1]);
                }
            }
        }
    }
}


                                                                                            //功能函数


void UART0Send(const uint8_t *pucBuffer,uint32_t ulCount)
{
    while(ulCount--)
    {
        UARTCharPutNonBlocking(UART0_BASE,*pucBuffer++);
    }
}

void UART1Send(const uint8_t *pucBuffer,uint32_t ulCount)
{
    while(ulCount--)
    {
        UARTCharPutNonBlocking(UART1_BASE,*pucBuffer++);
    }
}

void UART4Send(const uint8_t *pucBuffer,uint32_t ulCount)
{
    while(ulCount--)
    {
        UARTCharPutNonBlocking(UART4_BASE,*pucBuffer++);
    }
}

void UART3Send(const uint8_t *pucBuffer,uint32_t ulCount)
{
    while(ulCount--)
    {
        UARTCharPutNonBlocking(UART3_BASE,*pucBuffer++);
    }
}

void CHSet(uint8_t counter,uint32_t duty)
{
    uint32_t OUT;
    switch(counter)
    {
    case 1:{OUT=PWM_OUT_2;PWMPulseWidthSet(PWM0_BASE,OUT,duty);break;}
    case 2:{OUT=PWM_OUT_3;PWMPulseWidthSet(PWM0_BASE,OUT,duty);break;}
    case 3:{OUT=PWM_OUT_4;PWMPulseWidthSet(PWM0_BASE,OUT,duty);break;}
    case 4:{OUT=PWM_OUT_5;PWMPulseWidthSet(PWM0_BASE,OUT,duty);break;}
    case 5:{OUT=PWM_OUT_5;PWMPulseWidthSet(PWM1_BASE,OUT,duty);break;}
    }
}
double GetV1(int data[],int length)
{
    int i=0;
    double SumX=0,SumY=0,SumXY=0,SumXX=0;
    for(i=0;i<length;++i){
        //if(((data[i]-data[i-1])>30)&&((data[i-1]-data[i])>30)) break;
        SumX+=i+1;
        SumY+=data[i];
        SumXY+=(i+1)*data[i];
        SumXX+=(i+1)*(i+1);
    }
    return (i*SumXY-SumX*SumY)/(length*SumXX-SumX*SumX);
}


                                                                                        //初始化函数

void TIMER0A1A_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME);
    TimerControlEvent(TIMER0_BASE,TIMER_A,TIMER_EVENT_BOTH_EDGES);
    TimerLoadSet(TIMER0_BASE,TIMER_A,65535);
    TimerPrescaleSet(TIMER0_BASE,TIMER_A,255);          //扩充TIMER0A的计数容量

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    /*TimerConfigure(TIMER1_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME);
    TimerControlEvent(TIMER1_BASE,TIMER_A,TIMER_EVENT_NEG_EDGE);
    TimerLoadSet(TIMER1_BASE,TIMER_A,50000);*/

    TimerConfigure(TIMER1_BASE,TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE,TIMER_A,ui32Period/60-1);//Timer1用作产生10μs的Trigger

    //TimerSynchronize(TIMER0_BASE,TIMER_0A_SYNC|TIMER_1A_SYNC);
    //TimerSynchronize(TIMER0_BASE,TIMER_0A_SYNC|TIMER_0B_SYNC);

    TimerIntEnable(TIMER0_BASE,TIMER_CAPA_EVENT);
    //TimerIntEnable(TIMER1_BASE,TIMER_CAPA_EVENT);
    TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER1A);
    IntMasterEnable();

    TimerEnable(TIMER0_BASE,TIMER_A);
    TimerEnable(TIMER1_BASE,TIMER_A);
}

void TIMER2A_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE,TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE,TIMER_A,ui32Period/60-1);

    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE,TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    TimerEnable(TIMER2_BASE,TIMER_A);
}

void UART0_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE,GPIO_PIN_0|GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE,SysCtlClockGet(),115200,(UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE,UART_INT_RX|UART_INT_RT);
    IntMasterEnable();
    UART0Send((uint8_t*)"Welcome!\r",25);
    UARTCharPut(UART0_BASE,'\n');
}

void UART1_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1);

    UARTConfigSetExpClk(UART1_BASE,SysCtlClockGet(),115200,(UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE,UART_INT_RX|UART_INT_RT);
    IntMasterEnable();
}

void UART3_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);

    GPIOPinConfigure(GPIO_PC6_U3RX);
    GPIOPinConfigure(GPIO_PC7_U3TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE,GPIO_PIN_6|GPIO_PIN_7);

    UARTConfigSetExpClk(UART3_BASE,SysCtlClockGet(),9600,(UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE));

    UARTFIFOLevelSet(UART3_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);
    IntEnable(INT_UART3);
    UARTIntEnable(UART3_BASE,UART_INT_RX|UART_INT_RT);
    IntMasterEnable();
    UART3Send((uint8_t*)"Welcome!\r",25);
    UARTCharPut(UART3_BASE,'\n');
}

void UART4_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);

    GPIOPinConfigure(GPIO_PC4_U4RX);
    GPIOPinConfigure(GPIO_PC5_U4TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE,GPIO_PIN_4|GPIO_PIN_5);

    UARTConfigSetExpClk(UART4_BASE,SysCtlClockGet(),9600,(UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART4);
    UARTIntEnable(UART4_BASE,UART_INT_RX|UART_INT_RT);
    IntMasterEnable();
}

void GPIOPWM_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK)=GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR)=0xff;
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK)=0;

    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTE_BASE,GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTE_BASE,GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_1);

    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinConfigure(GPIO_PE5_M0PWM5);
    GPIOPinConfigure(GPIO_PF1_M1PWM5);

    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_4,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_4,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_5,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);

    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);

    PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1,100000);
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_2,100000);
    PWMGenPeriodSet(PWM1_BASE,PWM_GEN_2,100000);



    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,10000);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,10000);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,5000);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_5,7500);
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_5,7500);


    /*PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,10000);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,10000);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,5000);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_5,10000);*/

    PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT | PWM_OUT_3_BIT | PWM_OUT_4_BIT | PWM_OUT_5_BIT), true);
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);

    PWMGenEnable(PWM0_BASE,PWM_GEN_1);
    PWMGenEnable(PWM0_BASE,PWM_GEN_2);
    PWMGenEnable(PWM1_BASE,PWM_GEN_2);
}

void GPIOB_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PB6_T0CCP0);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_7);
}

void Controller_Init()
{
    TIMER0A1A_Init();
    TIMER2A_Init();
    UART0_Init();
    UART1_Init();
    UART3_Init();
    UART4_Init();
    GPIOB_Init();
    GPIOPWM_Init();
}


                                                                                        //中断处理函数


void TIMER0AIntHandler()
{
    uint32_t status;
    status=TimerIntStatus(TIMER0_BASE,true);
    TimerIntClear(TIMER0_BASE,status);



    if(s_cnt==0)
    {
        tick1=TimerValueGet(TIMER0_BASE,TIMER_A);
        flag_s1=1;
    }
    if(s_cnt==1)
    {
        tick2=TimerValueGet(TIMER0_BASE,TIMER_A);
        flag_s2=1;
        IntDisable(INT_TIMER0A);
    }
    s_cnt++;
    //if(s_cnt==2){s_cnt=0;}

}

void TIMER1AIntHandler()
{
    //uint32_t status;
    //status=TimerIntStatus(TIMER1_BASE,true);
    TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT);

    if(GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_7))
    {
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,0);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,GPIO_PIN_7);
        s_cnt=0;
    }

}

void TIMER2AIntHandler(void)
{
    TimerIntClear(TIMER2_BASE,TIMER_TIMA_TIMEOUT);
    if(systick){systick++;}
    if(delay>0){delay++;}
    if(lock_cnt>0){lock_cnt++;}
    if(unlock_cnt>0){unlock_cnt++;}

    ctl_cnt++;
    ctl_cnt=ctl_cnt%10;
//    flag1=1;
//    flag2=1;
    flag3=1;

}

void UART0IntHandler(void)
{
    uint32_t ulStatus;
    uint8_t i=0;
    ulStatus=UARTIntStatus(UART0_BASE,true);
    UARTIntClear(UART0_BASE,ulStatus);


    while(UARTCharsAvail(UART0_BASE))
    {
        ch[i] = UARTCharGet(UART0_BASE);
        UARTCharPut(UART0_BASE,ch[i]);
        i++;
    }
    UARTCharPutNonBlocking(UART0_BASE,'\r');
    UARTCharPutNonBlocking(UART0_BASE,'\n');

}

void UART1IntHandler(void)
{
    uint32_t ulStatus;
    uint8_t i=0;
    ulStatus=UARTIntStatus(UART1_BASE,true);
    UARTIntClear(UART1_BASE,ulStatus);



    while(UARTCharsAvail(UART1_BASE))
    {
        angle[i] = UARTCharGet(UART1_BASE);
        //UARTCharPutNonBlocking(UART3_BASE,ch[i]);
        i++;
    }
    UARTCharPutNonBlocking(UART1_BASE,'\r');
    UARTCharPutNonBlocking(UART1_BASE,'\n');

}

void UART3IntHandler(void)
{
    uint32_t ulStatus;
    uint8_t i=0;
    ulStatus=UARTIntStatus(UART3_BASE,true);
    UARTIntClear(UART3_BASE,ulStatus);



    while(UARTCharsAvail(UART3_BASE))
    {
        ch[i] = UARTCharGet(UART3_BASE);
        //UARTCharPutNonBlocking(UART3_BASE,ch[i]);
        i++;
    }
    UARTCharPutNonBlocking(UART3_BASE,'\r');
    UARTCharPutNonBlocking(UART3_BASE,'\n');

}

void UART4IntHandler(void)
{
    uint32_t ulStatus;
    uint8_t i=0,j=0,k=0;
    char x[4];
    char y[4];
    char h[5];
    char changeX[6];
    char changeY[6];
    ulStatus=UARTIntStatus(UART4_BASE,true);
    UARTIntClear(UART4_BASE,ulStatus);


    while(UARTCharsAvail(UART4_BASE))
    {
        data[i] = UARTCharGet(UART4_BASE);
        i++;
    }

    for(i=0;i<4;++i)
    {
        if(data[i]=='X'){j=i+1;}        //'X'后一位为横坐标
        if(data[i]=='Y'){k=i+1;}        //'Y'后一位为纵坐标
    }


    if(data[j]==0||data[k]==0)
    {
        ++pos_cnt;
    }
    else
    {
        if(data[j]>1&&data[j]<=80)
        {
            x_target=data[j];
        }
        if(data[k]>1&&data[k]<=120)
        {
            y_target=data[k];
        }
        pos_cnt=0;
        flag1=1;
        flag2=1;
    }
    x[0]=data[j]/100+'0';
    x[1]=(data[j]%100)/10+'0';
    x[2]=data[j]%10+'0';

    y[0]=data[k]/100+'0';
    y[1]=(data[k]%100)/10+'0';
    y[2]=data[k]%10+'0';

    if(p0_send>=0)
    {
        changeX[0]=p0_send/1000+'0';
        changeX[1]=(p0_send%1000)/100+'0';
        changeX[2]=(p0_send%100)/10+'0';
        changeX[3]=p0_send%10+'0';
    }
    if(p1_send>=0)
    {
        changeY[0]=p1_send/1000+'0';
        changeY[1]=(p1_send%1000)/100+'0';
        changeY[2]=(p1_send%100)/10+'0';
        changeY[3]=p1_send%10+'0';
    }


    if(ch1show_flag)
    {
        UARTCharPutNonBlocking(UART3_BASE,changeX[0]);
        UARTCharPutNonBlocking(UART3_BASE,changeX[1]);
        UARTCharPutNonBlocking(UART3_BASE,changeX[2]);
        UARTCharPutNonBlocking(UART3_BASE,changeX[3]);
        UARTCharPutNonBlocking(UART3_BASE,'\r');

        UARTCharPutNonBlocking(UART3_BASE,x[0]);
        UARTCharPutNonBlocking(UART3_BASE,x[1]);
        UARTCharPutNonBlocking(UART3_BASE,x[2]);
        UARTCharPutNonBlocking(UART3_BASE,'\r');

        if(e_int[0]>0){UARTCharPutNonBlocking(UART3_BASE,'+');}
        else if(e_int[0]<0){UARTCharPutNonBlocking(UART3_BASE,'-');}

//        UARTCharPutNonBlocking(UART3_BASE,changeY[0]);
//        UARTCharPutNonBlocking(UART3_BASE,changeY[1]);
//        UARTCharPutNonBlocking(UART3_BASE,changeY[2]);
//        UARTCharPutNonBlocking(UART3_BASE,changeY[3]);
//        UARTCharPutNonBlocking(UART3_BASE,'\r');
//
//        UARTCharPutNonBlocking(UART3_BASE,y[0]);
//        UARTCharPutNonBlocking(UART3_BASE,y[1]);
//        UARTCharPutNonBlocking(UART3_BASE,y[2]);
//        UARTCharPutNonBlocking(UART3_BASE,'\r');
    }




//    h[0]=h_send/1000+'0';
//    h[1]=(h_send%1000)/100+'0';
//    h[2]=(h_send%100)/10+'0';
//    h[3]=h_send%10+'0';
//
//    UARTCharPutNonBlocking(UART3_BASE,h[0]);
//    UARTCharPutNonBlocking(UART3_BASE,h[1]);
//    UARTCharPutNonBlocking(UART3_BASE,h[2]);
//    UARTCharPutNonBlocking(UART3_BASE,h[3]);
//    UARTCharPutNonBlocking(UART3_BASE,'\r');


//    x_target=100*(data[0]-'0')+10*(data[1]-'0')+(data[2]-'0');
//    y_target=100*(data[3]-'0')+10*(data[4]-'0')+(data[5]-'0');      //格式应为xxxxxx(前三位横坐标，后三位纵坐标）
    memset(data,0,sizeof(data));
}


void GPIODIntHandler()
{
    uint32_t Status;

    Status=GPIOIntStatus(GPIO_PORTD_BASE,true);
    GPIOIntClear(GPIO_PORTD_BASE,Status);

    if(Status==GPIO_PIN_0)
    {
        LED_Y_ON();
        buzzer_OFF();
    }
    else if(Status==GPIO_PIN_6)
    {
        LED_R_ON();
        buzzer_OFF();
    }
    else if(Status==GPIO_PIN_2)
    {
        LED_B_ON();
        buzzer_OFF();
    }
    else if(Status==GPIO_PIN_3)
    {
        LED_Y_OFF();
        LED_R_OFF();
        LED_B_OFF();
        buzzer_ON();
    }
}


