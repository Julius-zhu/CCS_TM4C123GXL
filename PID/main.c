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
#define vertical 120                                                    //ͼ��Ĵ�С
float kp[3],ki[3],kd[3];                                                //PID����
float e[3];                                                             //��ǰ���
float e_int[3],e_dif[3],e_temp[3],e_temp_past[2];                                                //���Ļ�����΢��
float p[3];
float height_set=800,height,height_history[5]={-1,-1,-1,-1,-1};                         //�߶ȵ�Ŀ��ֵ���߶ȣ�Ϊƽ��ֵ�����߶ȵ���ʷֵ
volatile int x_target=horizon/2,y_target=vertical/2;                                                         //Ŀ���ĺ������꣬
//int x_history[3],y_history[3];
int x_center=horizon/2,y_center=vertical/2;                                             //����ͷ������ֵ��ͼ���С80��120��
uint8_t pos_cnt=0;                                                              //��Ŀ��㲻����Ұ�ڣ���x_target,y_targetΪ0������ʼ��������10֮ǰһֱ�����ϴεĵ�����
uint8_t flag_out=0;                                                             //��1����Ŀ����Ѿ�����Ұ��
volatile float distance;                                                         //����ģ���ȡ�ľ���ֵ
uint8_t s_cnt=0,height_i;
uint8_t cnt=0;
int tick1,tick2,ticks;                                                  //����ģ��ļ���ֵ
uint32_t ui32Period;
uint8_t flag1=0,flag2=0,flag3=0;
uint8_t mode=0;                                                         //0Ϊ�ֶ�ģʽ��1Ϊ���ģʽ��2Ϊ����ģʽ��3Ϊ����ģʽ,4Ϊ���߼Ӷ���ģʽ
uint8_t flag_s1=0,flag_s2=0;
uint8_t systick;
uint32_t delay;
uint32_t land_cnt;
uint8_t ctl_cnt;
char ch[50];                                                            //ch�������ָ��
char data[4];                                                          //data���Ŀ�������
char angle[8];                                                          //angle��������ǵĽǶ�����
char feedback[50];
float out[20];

float MID1=7500,MID2=7500,MID3=7950,MID4=7500;                                              //�ĸ�ͨ�����м�ֵ��MID3����ͨ��Ϊ��ͣʱ��PWMռ�ձ�
float tkf_MID1=7341,tkf_MID2=7350;
uint8_t init_flag;
float Slimit[3]={7731,0,8700},Llimit[3]={6931,0,7800};                                                            //1-3ͨ��������
float LANDSPEED=7800,LANDSPEED_TRUE;
uint16_t tkfcoef=80,landcoef=100;                                                                     //��ɽ���ϵ��
//7318  7550 2019/7/11
//7340  7315 2019/7/12 14:23
//7335 7320 6500 2019/7/12 20:24
//�·ɿ�
//7425 7320 2019/7/19 11:45
//7422 7321 2019/7/19 13:35
//7309 7348 2019/7/19 19:39
//7365 7333 2019/7/20 15:34
//��ײȦ �½ż�
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


void TIMER0A1A_Init();                                                  //timer0��׽����ģ�鷴���ĸߵ�ƽ���أ�timer1����ʱ��������ģ��
void TIMER2A_Init();                                                    //timer2����PID��ˢ��Ƶ��
void GPIOPWM_Init();
void UART0_Init();
void UART1_Init();
void UART3_Init();                                                      //UART3������������ͨ��
void UART4_Init();                                                      //UART4�����봦��ͼ��ĵ�Ƭ��ͨ��
void GPIOB_Init();                                                      //GPIOB_Pin7��Ϊ����ģ��Ĵ������ţ�Pin6��Ϊtimer0A��CCP0
void Controller_Init();                                                 //���ذ������ʼ��

double GetV1(int data[],int length);                                    //��ȡ�ٶ�
void UART0Send(const uint8_t *pucBuffer,uint32_t ulCount);
void UART1Send(const uint8_t *pucBuffer,uint32_t ulCount);
void UART3Send(const uint8_t *pucBuffer,uint32_t ulCount);
void UART4Send(const uint8_t *pucBuffer,uint32_t ulCount);
void CHSet(uint8_t counter,uint32_t duty);                              //��ͨ�����ڷɿ�ģʽ��ռ�ձ�5%-10%

int main(void)

{
    FPUEnable();
    FPULazyStackingEnable();

    SysCtlClockSet(SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_SYSDIV_5);//��Ƶ40M����timer0�ļ�����Χ��Сƥ�䣬��Ҫ�޸���timer0 loadֵӦ��Ӧ�Ķ�
    ui32Period=SysCtlClockGet();

    GPIO_Init();                //ʹ�ܲ�����GPIO A-F
    peripheral_Init();

    Controller_Init();

//    OLED_Initial();
//
//    LCD_P8x16Str(0,0,"TASK:");
//    LCD_P8x16Str(40,0,"0");

    IntPriorityGroupingSet(3);                                      //�����ж����ȼ�
    IntPrioritySet(INT_TIMER0A,0x0);
    IntPrioritySet(INT_TIMER1A,0x20);
    IntPrioritySet(INT_UART3,0x0);


    CH[0]=7500;
    CH[1]=7500;
    CH[2]=5000;
    CH[3]=7500;
    CH[4]=7500;


    //����PID�Ĳ���ֵ
    kp[0]=1.50;         ///kp[0]�ڼ����б�������10
    ki[0]=0.01;
    kd[0]=1.40;         //kd[0]�ڼ����б�������50

    //һͨ��
    //0.99 0.10 1.31
    //0.80 0.02 0.95
    //0.87*10 0.05(1000) 1.12*50   2019/8/1
    //0.96*10 0.03(3000) 1.40*50   2019/8/3

    kp[1]=0.75;         //kp[1]�ڼ����б�������10
    ki[1]=0.01;
    kd[1]=0.70;            //kd[1]�ڼ����б�������50

    //0.43*10 0.03 0.58*50 2019/8/1

    kp[2]=1.30;
    ki[2]=0.027;
    kd[2]=2.30;         //kd[2]�ڼ����б�������10


    //��ͨ��
    //1.4 0.018(50000) 1.9*10 2019/7/28
    //1.4 0.020(50000) 1.9*10 2019/7/31
    //1.3 0.027(50000) 2.3*10 2019/8/1


    //sprintf(feedback,"kp=%f",output);
    //UART0Send((uint8_t*)feedback,20);

    while(1)
    {

        getorder(ch);                           //�����������͵�ָ��


        if(land_flag==1)
        {
            if(height<250)
            {
                CH[2]=LANDSPEED_TRUE-50;
                if(CH[2]>8100){CH[2]=8100;}
                else if(CH[2]<7800){CH[2]=7800;}
                CHSet(3,CH[2]);
            }
            if(height<150)                  //�߶ȵ���15cmֱ������
            {
                lock();
            }
        }
//        if(height>1500)                                                         //����߶ȳ���1.5m���Զ���������ģʽ
//        {
//            mode=2;
//        }



        if(mode==1)                                                                     //ģʽ1�����
        {
            if(init_flag)                                                               //����ͨ����ʼ��
            {
                CH[0]=tkf_MID1;
                CH[1]=tkf_MID2;
                CH[3]=MID4;
                CHSet(1,CH[0]);
                CHSet(2,CH[1]);
                CHSet(4,CH[3]);
                e[0]=0;e_int[0]=0;e_dif[0]=0;                                         //����������ֵ
                e[1]=0;e_int[1]=0;e_dif[1]=0;
                e[2]=0;e_int[2]=0;e_dif[2]=0;
                delay=1;

                init_flag=0;                                                            //��ʼ��ֻ����һ��
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
                if(CH[2]<5000){CH[2]=5000;}                                           //��������������
                if(CH[2]>8100){CH[2]=8100;}
                CHSet(3,CH[2]);
                delay=1;                    //��ʱ20/60��
            }


            /*if(height>500)                                                          //����߶ȴﵽ0.5m���Զ���������ģʽ
            {
                mode=3;
            }*/
        }

        if(mode==2)                                                                    //����ģʽ
        {

            //CH[0]=MID1;
            //CH[1]=MID2;
            LANDSPEED_TRUE=LANDSPEED-ki[2]*e_int[2];
            CH[2]=LANDSPEED_TRUE;
            if(CH[2]>8100){CH[2]=8100;}
            else if(CH[2]<7800){CH[2]=7800;}
            mode=0;
            CHSet(3,CH[2]);

            e[0]=0;e_int[0]=0;e_dif[0]=0;e_temp[0]=0;e_temp_past[0]=0;                                         //����������ֵ
            e[1]=0;e_int[1]=0;e_dif[1]=0;e_temp[1]=0;e_temp_past[1]=0;
            e[2]=0;e_int[2]=0;e_dif[2]=0;e_temp[2]=0;

            land_flag=1;
        }


        //�˴���Ҫһ��if��䣬�ж�flag�������볬��ģ��ˢ��ͬ������ֱ�ӽ��߶�ֵˢ�¼���distance�ļ��㺯����
        //�ӳ���ģ���ȡ�߶�ֵ������������˲�
        if(flag_s1&&flag_s2)
        {
            flag_s1=0;
            flag_s2=0;
            ticks=(tick1-tick2);
            if(ticks<0)
            {
                ticks+=65536*256;   //      65536*256
            }
            distance=ticks*0.034/8;//��λmm
            s_cnt=0;

            if(distance>0&&distance<2000)                         //distance��λΪmm
            {
                for(height_i=1;height_i<5;++height_i)
                {
                    height_history[height_i-1]=height_history[height_i];//1->0,2->1,3->2,4->3,�ճ�4�������뵱ǰ�߶�ֵ
                }

                if(((distance<height+300)&&(distance>height-300))||(height_history[0]==-1))//��ȥ��������ĸ߶�ֵ,����height_history[0]δ�����Чֵǰ���������˲�
                {
                    height_history[4]=distance;//��ȡ��ǰ�߶�ֵ
                }

                height=(height_history[0]+height_history[1]+height_history[2]+height_history[3]+height_history[4])/5;


                h_send=height;

                if(flag3&&(mode==3||mode==4))
                {
                    flag3=0;//���־λ����20Hz��Ƶ��ˢ��PID������ֵ
                    e_temp[2]=e[2];                                                 //��һ�ε����ֵ����e_temp[2]
                    e[2]=height-height_set;
                    //�ڷ������ﵽһ���߶Ⱥ��ٿ�ʼ����e_int����ۼ�                                 //�����ֵ������PID�ĵ���Ч������Ӱ�죬��ע��
                    if(height>=200)
                    {
                        e_int[2]+=e[2];
                        if(e_int[2]>50000){e_int[2]=50000;}
                        if(e_int[2]<-50000){e_int[2]=-50000;}
                    }
                    e_dif[2]=e[2]-e_temp[2];

                    p[2]=kp[2]*e[2]+ki[2]*e_int[2]+kd[2]*e_dif[2]*10;
                    CH[2]=MID3-p[2];                                                 //������յĸ�ͨ��PID���ƺ��ռ�ձ�ֵ��p[2]������Ҫ����һ��ϵ����Ҳ���Խ���ϵ�����䵽kp,ki,kd�У���


                    if(CH[2]>Slimit[2]){CH[2]=Slimit[2];}
                    if(CH[2]<Llimit[2]){CH[2]=Llimit[2];}
                    //���ú������޸ĸ�ͨ����PWM���ռ�ձ�
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
                if(height>=400){mode=3;}            //�߶���0.5m���ϣ����ж��߷��У�������
                if(height>=0&&height<400){mode=2;}
                e[0]=0;         //������ֵ��������´ο������㶨��ģʽ����Ӱ��
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
                //����λ����������Ķ��壬���volatile

                if(flag1)
               {
                   flag1=0;//���־λ����20Hz��Ƶ��ˢ��PID������ֵ
                   //�ڶ�ͨ��������ǰ��
                   e_temp[0]=e[0];
                   //y��ǰΪ��,��Ŀ������Ұ�ڣ���ˢ�����ֵ
                   e[0]=x_center-x_target;                               //ע��e[0]��������ͨ��PWM��С����ķ���֮��Ĺ�ϵ,��ռ�ձȼ�С���ң�ռ�ձ�����
                   if(e[0]<30&&e[0]>-30)                                                              //��Ŀ�����һ����Χ���ټ��������
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
                   //����ͼ���x����������ռ�ձȴ�С������һ�£���-���䡮+��
                   CH[0]=MID1-p[0];

                   p0_send=CH[0];
                   //���ú������޸ĸ�ͨ����PWM���ռ�ձ�
                   CHSet(1,CH[0]);

               }

                if(flag2)
                {
                    flag2=0;//���־λ����20Hz��Ƶ��ˢ��PID������ֵ
                    //�ڶ�ͨ��������ǰ��
                    e_temp[1]=e[1];
                    //y��ǰΪ��,��Ŀ������Ұ�ڣ���ˢ�����ֵ
                    e[1]=y_center-y_target;                               //ע��e[1]��������ͨ��PWM��С����ķ���֮��Ĺ�ϵ,����ռ�ձȼ�С������ռ�ձ�����
                    if(e[1]<45&&e[1]>-45)                                                              //��Ŀ�����һ����Χ���ټ��������
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
                    //����ͼ���y����������ռ�ձȴ�С������һ�£���-���䡮+��
                    p1_send=CH[1];
                    CH[1]=MID2+p[1];                                                 //������յĸ�ͨ��PID���ƺ��ռ�ձ�ֵ��p[2]������Ҫ����һ��ϵ����Ҳ���Խ���ϵ�����䵽kp,ki,kd�У���

                    //���ú������޸ĸ�ͨ����PWM���ռ�ձ�
                    CHSet(2,CH[1]);
                }
            }
        }
    }
}


                                                                                            //���ܺ���


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


                                                                                        //��ʼ������

void TIMER0A1A_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME);
    TimerControlEvent(TIMER0_BASE,TIMER_A,TIMER_EVENT_BOTH_EDGES);
    TimerLoadSet(TIMER0_BASE,TIMER_A,65535);
    TimerPrescaleSet(TIMER0_BASE,TIMER_A,255);          //����TIMER0A�ļ�������

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    /*TimerConfigure(TIMER1_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME);
    TimerControlEvent(TIMER1_BASE,TIMER_A,TIMER_EVENT_NEG_EDGE);
    TimerLoadSet(TIMER1_BASE,TIMER_A,50000);*/

    TimerConfigure(TIMER1_BASE,TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE,TIMER_A,ui32Period/60-1);//Timer1��������10��s��Trigger

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


                                                                                        //�жϴ�����


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
        if(data[i]=='X'){j=i+1;}        //'X'��һλΪ������
        if(data[i]=='Y'){k=i+1;}        //'Y'��һλΪ������
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
//    y_target=100*(data[3]-'0')+10*(data[4]-'0')+(data[5]-'0');      //��ʽӦΪxxxxxx(ǰ��λ�����꣬����λ�����꣩
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


