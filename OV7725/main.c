/*OV7725���Ž���
SDA--PD3;SCL--PD2;WRST--PA3;RCLK--PA4;RRST--PA5;OE--PA6;WEN--PA7;
VSYNC--PE0;
D0~D7--PB0~PB7;
//PD1��PB7����һ���
 *A3��A7����
*/
//���ߵ�һ�˳�������ͷ�ĳ��������ҵߵ�
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "uartstdio.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "inc/hw_gpio.h"

#include "ov7725.h"
#include "sccb.h"
//���ʱ240*320
#define  OV7725_WINDOW_HEIGHT       240
#define  OV7725_WINDOW_WIDTH        320
#define  Gray_threshold             150  //256����
#define  Div                        2
//0�� 1��ɫ�� 2��ɫ 3��ɫ
char ch1;
uint8_t flag[4]={0,0,0,0};
uint8_t mode=2;
uint8_t ov_sta;
uint8_t ov_frame;
uint8_t ui8PinData=2;
uint32_t counter_1s=0;
uint32_t time_distance,dis;
uint8_t DATA_PB;
uint16_t color;
uint8_t color_high,color_low;
uint8_t ID_TEST;
uint8_t red,green,blue;
uint8_t Center_X=0,Center_Y=0;
uint16_t Gray;
uint8_t Gray_Array[OV7725_WINDOW_HEIGHT/Div][OV7725_WINDOW_WIDTH/Div];
uint16_t i,j;
uint32_t count_x=0,sum_x=0,count_y=0,sum_y=0;
void PortEIntHandler(void);
void camera_show_FullColor(void);
void camera_show_BlackAndWrite(void);
void camera_find_Center(void);
void UART3_Init(void);
void UART3Send(const uint8_t *pucBuffer,uint32_t ulCount);
void camera_find_Center2(void);
void OV7725_camera_getdata3(void)
{

    if(ov_sta>=2)
    {
        //UARTCharPut(UART0_BASE, 1);
        //UARTCharPut(UART0_BASE, 0xfe);

        OV7725_CS_L;
        OV7725_RRST_L;              //��ʼ��λ��ָ��
        OV7725_RCK_L;
        OV7725_RCK_H;
        OV7725_RCK_L;
        OV7725_RRST_H;              //��λ��ָ�����
        OV7725_RCK_H;
        for(i=0;i<OV7725_WINDOW_HEIGHT;i++)
        {
            for(j=0;j<OV7725_WINDOW_WIDTH;j++)
            {
                OV7725_RCK_L;
                if((i%2)&&!(j%4))
                {
                    color_low=OV7725_DATA;
                    //UARTCharPut(UART0_BASE, color_low);
                    if((color_low>140)&&!(color_low&0x04)){
                        OV7725_RCK_H;
                        OV7725_RCK_L;
//                        if(!OV7725_DATA&0x10)
                            Gray_Array[i/Div][j/4]=1;
//                        else{
//                            Gray_Array[i/Div][j/4]=0;
//                        }
                    }
                    else{
                        Gray_Array[i/Div][j/4]=0;
                        OV7725_RCK_H;
                        OV7725_RCK_L;
                    }
//                    if(Gray_Array[i/Div][j/4]==0)
//                        UARTCharPut(UART0_BASE, 0x00);
//                    else
//                        UARTCharPut(UART0_BASE, 0xfe);
//                    if(color_low<50)
//                        Gray_Array[i/Div][j/4]=1;
//                    else{
//                        Gray_Array[i/Div][j/4]=0;
//                    }
                    //UARTCharPut(UART0_BASE, OV7725_DATA);
                    OV7725_RCK_H;
                    //UARTCharPut(UART0_BASE, color_low);
                }
                else{
                    OV7725_RCK_H;
                    OV7725_RCK_L;
                    OV7725_RCK_H;
                }

            }
        }
        //UARTCharPut(UART0_BASE, 0xFf);
        //UARTCharPut(UART0_BASE, 1);
        OV7725_CS_H;
        OV7725_RCK_L;
        OV7725_RCK_H;
        GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);
        ov_sta=0;                   //����֡�жϱ��
        ov_frame++;
        camera_find_Center2();
    }
}
void OV7725_camera_getdata2(void)
{

    if(ov_sta>=2)
    {
        //UARTCharPut(UART0_BASE, 1);
        //UARTCharPut(UART0_BASE, 0xfe);

        OV7725_CS_L;
        OV7725_RRST_L;              //��ʼ��λ��ָ��
        OV7725_RCK_L;
        OV7725_RCK_H;
        OV7725_RCK_L;
        OV7725_RRST_H;              //��λ��ָ�����
        OV7725_RCK_H;
        for(i=0;i<OV7725_WINDOW_HEIGHT;i++)
        {
            for(j=0;j<OV7725_WINDOW_WIDTH;j++)
            {
                OV7725_RCK_L;
                if((i%2)&&!(j%4))
                {
                    color_low=OV7725_DATA;
                    //UARTCharPut(UART0_BASE, color_low);
//                    if((color_low>140)&&!(color_low&0x04)){
//                        OV7725_RCK_H;
//                        OV7725_RCK_L;
//                        if(!OV7725_DATA&0x18)
//                            Gray_Array[i/Div][j/4]=1;
//                        else{
//                            Gray_Array[i/Div][j/4]=0;
//                        }
//                    }
//                    else{
//                        Gray_Array[i/Div][j/4]=0;
//                        OV7725_RCK_H;
//                        OV7725_RCK_L;
//                    }
                    if(color_low<50)
                        Gray_Array[i/Div][j/4]=1;
                    else{
                        Gray_Array[i/Div][j/4]=0;
                    }
                    //UARTCharPut(UART0_BASE, OV7725_DATA);
                    OV7725_RCK_H;
                    OV7725_RCK_L;
                    OV7725_RCK_H;
                    //UARTCharPut(UART0_BASE, color_low);
                }
                else{
                    OV7725_RCK_H;
                    OV7725_RCK_L;
                    OV7725_RCK_H;
                }

            }
        }
        //UARTCharPut(UART0_BASE, 0xFf);
        //UARTCharPut(UART0_BASE, 1);
        OV7725_CS_H;
        OV7725_RCK_L;
        OV7725_RCK_H;
        GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);
        ov_sta=0;                   //����֡�жϱ��
        ov_frame++;
        camera_find_Center2();
    }
}
void OV7725_camera_getdata(void)
{

    if(ov_sta>=2)
    {
        //UARTCharPut(UART0_BASE, 1);
        //UARTCharPut(UART0_BASE, 0xfe);

        OV7725_CS_L;
        OV7725_RRST_L;              //��ʼ��λ��ָ��
        OV7725_RCK_L;
        OV7725_RCK_H;
        OV7725_RCK_L;
        OV7725_RRST_H;              //��λ��ָ�����
        OV7725_RCK_H;
        for(i=0;i<OV7725_WINDOW_HEIGHT;i++)
        {
            for(j=0;j<OV7725_WINDOW_WIDTH;j++)
            {
                OV7725_RCK_L;
                if((i%2)&&!(j%4))
                {
                    color_low=OV7725_DATA;
                    //UARTCharPut(UART0_BASE, color_low);
                    if(!(color_low&0x06)&&!(color_low&0x80)){
                        Gray_Array[i/Div][j/4]=1;
                        //UARTCharPut(UART0_BASE, 0xfe);
                    }
                    else{
                        Gray_Array[i/Div][j/4]=0;
                        //UARTCharPut(UART0_BASE, 0x00);
                    }
                }
                OV7725_RCK_H;
                OV7725_RCK_L;
                OV7725_RCK_H;
            }
        }


        //UARTCharPut(UART0_BASE, 0xFF);
        //UARTCharPut(UART0_BASE, 1);
        OV7725_CS_H;
        OV7725_RCK_L;
        OV7725_RCK_H;
        GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);
        ov_sta=0;                   //����֡�жϱ��
        ov_frame++;
        camera_find_Center();
    }
}
//ʹ��UARTprintf
void InitConsole(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    //ʹ��16MHZ�ڲ�ʱ��Դ��Ϊ���ڵ�ʱ��
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //���ں�/������/ʱ��Ƶ��
    UARTStdioConfig(0, 115200, 16000000);
}

//���ڷ��ͺ���
void UARTSend(const uint8_t *pucBuffer, uint32_t ulCount)
{
    while(ulCount--)
    {
        //��Ҫ���͵��ַ�д��UART
        UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
        //UARTCharPut(UART0_BASE, *pucBuffer++);
    }
}

int main(void)
{

    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    //2.5��Ƶ��ʹ��PLL���ⲿ����16M��systemʱ��Դѡ�� main osc��ϵͳʱ��80MHZ(��Ƶ�����)
    InitConsole();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);

    //PE0�ж�����
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOIntRegister(GPIO_PORTE_BASE, PortEIntHandler);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0 , GPIO_FALLING_EDGE);//PE0�½��ش����ж�
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0);
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0xff;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
    IntMasterEnable();
    UART3_Init();
    //UARTprintf("hello");

    while(1)
    {
        if(OV7725_Init()==0)//0��ʾ��ʼ���ɹ�
        {
            delay_ms(1000);
            OV7725_Light_Mode(0);
            OV7725_Color_Saturation(0);
            OV7725_Brightness(0);
            OV7725_Contrast(4);
            OV7725_Special_Effects(0);
            OV7725_Window_Set(OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT,0);//QVGAģʽ���
            OV7725_CS_L;
            //UARTprintf("OV7725��ʼ���ɹ�\r\n");


            //Timer5A�ж�����
            SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
            TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
            TimerLoadSet(TIMER5_BASE, TIMER_A, SysCtlClockGet()-1);//1s
            IntEnable(INT_TIMER5A);
            TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
            TimerEnable(TIMER5_BASE, TIMER_A);
            break;
        }

    }
    while(1)
    {
        if(mode==1)
            OV7725_camera_getdata();
        else if(mode==3)
            OV7725_camera_getdata2();
        else if(mode==2)
            OV7725_camera_getdata3();
    }
}

//Timer0Aռ��PB7?
void TIMER5AInterrupt(void)
{
    //��֡��
    TimerIntClear(TIMER5_BASE,TIMER_TIMA_TIMEOUT);//Clear the timer interrupt
    //UARTprintf("ov_frame=%d\n",ov_frame);//������ͷ��������ʱ��ע�ʹ˴���
    ov_frame=0;
}
/*
void TIMER0AInterrupt(void)
{
������ģ��   0.1ms�ж�
    TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);//Clear the timer interrupt
    counter_1s++;
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    if(counter_1s==10000)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        counter_1s=0;
    }

    if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_0))
    {
        time_distance++;
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    }]
    else
    {
        if(time_distance!=0)
        {
            dis=time_distance*340/100/2;
            //UARTprintf("%d\n",dis);
        }
        time_distance=0;
    }
}
*/

void PortEIntHandler(void)
{
    //����жϱ�־
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);
    if(ov_sta==0)
    {
        OV7725_WRST_L;      //��λдָ��
        OV7725_WRST_H;
        OV7725_WREN_H;      //����д��FIFO
    }else OV7725_WREN_L;    //��ֹд��FIFO
    ov_sta++;
}


void camera_show_FullColor(void)
{
    if(i%Div==0 && j%Div==0)
    {
        OV7725_RCK_L;
        //color=OV7725_DATA;  //������ �߰�λ
        //color_high=OV7725_DATA;
        //if(color_high==0xfe) color_high++;

        OV7725_RCK_H;
        //color<<=8;
        //SysCtlDelay(5);
        OV7725_RCK_L;
        //color|=OV7725_DATA; //������ �Ͱ�λ
        color_low=OV7725_DATA<<3;
        //if(color_low==0xfe) color_low++;
        //UARTCharPut(UART0_BASE, color_high);

        //UARTCharPut(UART0_BASE, color_low);
        if(color_low<Gray_threshold)
            Gray_Array[i/Div][j/Div]=1;
        else
            Gray_Array[i/Div][j/Div]=0;
    }
    else{
        OV7725_RCK_L;
        //color=OV7725_DATA;  //������ �߰�λ
        //color_high=OV7725_DATA;
        //if(color_high==0xfe) color_high++;

        OV7725_RCK_H;
        //color<<=8;
        //SysCtlDelay(5);
        OV7725_RCK_L;
        //color|=OV7725_DATA; //������ �Ͱ�λ
        //color_low=OV7725_DATA;
        //if(color_low==0xfe) color_low++;
    }

    OV7725_RCK_H;
    //SysCtlDelay(5);
    //UARTprintf("%x\n",color);
}

void camera_show_BlackAndWrite(void)
{
    OV7725_RCK_L;
    color=OV7725_DATA;  //������ �߰�λ
    color<<=8;
    OV7725_RCK_H;
    OV7725_RCK_L;
    color|=OV7725_DATA; //������ �Ͱ�λ
    if(i%2==0 && j%2==0) //ÿ���������ȡ���ֱ���Ϊ60*80
    {
        //������ɫ
        red = (OV7725_DATA & 0xF800 ) >> 11;
        green = (OV7725_DATA & 0x07E0 ) >> 5;
        blue = (OV7725_DATA & 0x001F);
        //��0����8λ
        red <<= 3;
        green <<= 2;
        blue <<= 3;
        //Gray = (red*30 + green*59 + blue*11 + 50) / 100;
        Gray = (red*38 + green*75 + blue*15) >> 7;
        if(Gray<Gray_threshold)
            Gray_Array[i/2][j/2]=1;
        else
            Gray_Array[i/2][j/2]=0;
        if(Gray<Gray_threshold)     //��ɫ����0 ����Ϊ�ӽ���ɫ
            {UARTCharPut(UART0_BASE, 0);UARTCharPut(UART0_BASE, 0);}
        else
            {UARTCharPut(UART0_BASE, 0xff);UARTCharPut(UART0_BASE, 0xff);}
    }

    OV7725_RCK_H;
}
void camera_find_Center(void)
{
    count_x=0,sum_x=0,count_y=0,sum_y=0;
    for(i=0;i<120-Center_Y;i++)
    {
        if(Gray_Array[Center_Y+i][40])
        {
            flag[0]=Center_Y+i;
            break;
        }
        flag[0]=120;
    }
    for(i=0;i<=Center_Y;i++)
    {
        if(Gray_Array[Center_Y-i][40])
        {
            flag[1]=Center_Y+i-i;
            break;
        }
        flag[1]=0;
    }
    for(j=0;j<80-Center_X;j++)
    {
        if(Gray_Array[60][Center_X+j])
        {
            flag[2]=Center_X+j;
            break;
        }
        flag[2]=80;
    }
    for(j=0;j<=Center_X;j++)
    {
        if(Gray_Array[60][Center_X-j])
        {
            flag[3]=Center_X-j;
            break;
        }
        flag[3]=0;
    }
    Center_Y=(flag[0]+flag[1])/2;
    Center_X=(flag[2]+flag[3])/2;
    UARTCharPut(UART3_BASE,'X');
    UARTCharPut(UART3_BASE,Center_X);
    UARTCharPut(UART3_BASE,'Y');
    UARTCharPut(UART3_BASE,Center_Y);
}
// YСֽƬ�ں��� Y��ֽƬ��ǰ��
// XСֽƬ����� X��ֽƬ���ұ�
void camera_find_Center2(void)
{
    count_x=0,sum_x=0,count_y=0,sum_y=0;
    for(i=1;i<OV7725_WINDOW_HEIGHT/Div-1;i++)
    {
        for (j=1;j<OV7725_WINDOW_WIDTH/4-1;j++)
        {
            if(Gray_Array[i][j]&&(Gray_Array[i-1][j]+Gray_Array[i][j-1]+Gray_Array[i+1][j]+Gray_Array[i][j+1]>1))
            {
                sum_y+=i;
                count_y++;
                sum_x+=j;
                count_x++;
            }
        }
    }
    Center_Y=sum_y/count_y;
    Center_X=sum_x/count_x;
    UARTCharPut(UART3_BASE,'X');
    UARTCharPut(UART3_BASE,Center_X);
    UARTCharPut(UART3_BASE,'Y');
    UARTCharPut(UART3_BASE,Center_Y);
}
// YСֽƬ�ں��� Y��ֽƬ��ǰ��
// XСֽƬ����� X��ֽƬ���ұ�

void UART3IntHandler(void)
{
    uint32_t ulStatus;
    ulStatus=UARTIntStatus(UART3_BASE,true);
    UARTIntClear(UART3_BASE,ulStatus);


    while(UARTCharsAvail(UART3_BASE))
    {
        ch1=UARTCharGet(UART3_BASE);
    }
//    if(ch1==0xf0)
//        mode=0;
//    else if(ch1==0xf1)
//        mode=1;
//    else
//        mode=2;
    //UARTCharPutNonBlocking(UART3_BASE,'\r');
    //UARTCharPutNonBlocking(UART3_BASE,'\n');
}

void UART3Send(const uint8_t *pucBuffer,uint32_t ulCount)
{
    while(ulCount--)
    {
        UARTCharPutNonBlocking(UART3_BASE,*pucBuffer++);
    }
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
   // UART3Send((uint8_t*)"Welcome!\r",25);
   // UARTCharPut(UART3_BASE,'\n');
}


