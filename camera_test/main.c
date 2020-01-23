//PA2 PA3 PA4 Ϊ�ж�����
//PD0-PD3 PC4-PC7��Y0-Y7
//VSY��ͬ���ź� �������� ������ 60hz  H ���ж� �������� 14khz���� pk ����ͬ��13.5mhz

#define TARGET_IS_BLIZZARD_RA1

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "driverlib/udma.h"
#include "DMA.h"
#include "UART.h"



#define ROW        60                 //��������ͷ���ɼ��Ķ�ά��������
#define COLUMN     60                //��������ͷ���ɼ��Ķ�ά��������
#define ROW_START  10                 //��������ͷ��ά�����п�ʼ��ֵ
#define ROW_MAX    200                //��������ͷ���ɼ��Ķ�ά���������ֵ
#define THRESHOLD  0x68               //ͼ����ֵ���������ɼ�ͼ������ֵ��С��ʵ�����������OV7620���ɼ�������ֵ��СΪ0--255��
#define PORTA
unsigned char Buffer[ROW][COLUMN]={0};       //���ɼ���ͼ���ά����
unsigned char Image_Center[ROW]={0};        //���ɼ���ͼ��������

unsigned char SampleFlag=1;       //��ż�����

unsigned int  m=0;                 //���б���

unsigned int  Line;               //���жϼ�������
unsigned int  hang;



unsigned int testcount,testcount2,test_flag;
unsigned int  Get_Image[]={

                           15  ,18  ,21  ,24  ,27  ,30  ,33  ,36  ,39  ,42  ,45  ,48  ,51  ,54  ,57  ,60  ,63  ,66  ,69  ,72  ,75  ,78  ,81  ,84  ,87  ,90  ,93  ,96  ,99  ,102 ,
                           105 ,108 ,111 ,114 ,117 ,120 ,123 ,126 ,129 ,132 ,135 ,138 ,141 ,144 ,147 ,150 ,153 ,156 ,159 ,162 ,165 ,168 ,171 ,174 ,177 ,180 ,183 ,186 ,189 ,192 ,
                         };     //��ÿ�����ļ��С�
void UARTSend(const uint8_t *pucBuffer, uint32_t ulCount);
void UART0Init();
void YNCInit();
void GPIOB_Init();

unsigned char i,j;
volatile char a;
char shift_flag=1;
uint32_t ClockPeriod;
int main(void)
{
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);           //��Ƶ40MHz
    ClockPeriod=ROM_SysCtlClockGet();
    UART0Init();
    YNCInit();
    GPIOB_Init();
    //UDMAInit();
    ROM_IntMasterEnable();
    //uDMAChannelEnable(UDMA_CHANNEL_ETH0TX);
    //uDMAChannelRequest(UDMA_CHANNEL_ETH0TX);
    while(1){
       if(test_flag){
//           if(shift_flag){
//               for(i=0;i<ROW;i++)
//               {
//                  for(j=0;j<COLUMN;j++)
//                  {
//                      Buffer[i][j]=Buffer[i][j]*16;
//                  }
//                  //SCI_Write(0x0D);
//                  //SCI_Write(0X0A);
//               }
//               shift_flag=0;
//           }
           for(i=0;i<ROW;i++)
           {
              for(j=0;j<COLUMN;j++)
              {
                  ROM_UARTCharPutNonBlocking(UART0_BASE,Buffer[i][j]);
                  ROM_SysCtlDelay(20000);
              }
              //SCI_Write(0x0D);
              //SCI_Write(0X0A);
           }
           test_flag=0;
           ROM_UARTCharPutNonBlocking(UART0_BASE,0xff);
       }
//        SysCtlDelay(10000000);
//        for(i1=0;i1<4800;i1++)
//        {
//            UARTCharPutNonBlocking(UART0_BASE,'a');
//            SysCtlDelay(5000);
//        }
        //UARTCharPutNonBlocking(UART0_BASE,0xe0);
    }

}


void YNCInit(){
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6);                                          //PC6 ���ж�
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7);                                          //PC7 ���ж�
    ROM_GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_6,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    ROM_GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_7,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    ROM_IntEnable(INT_GPIOC);
    ROM_GPIOIntTypeSet(GPIO_PORTC_BASE,GPIO_PIN_6, GPIO_RISING_EDGE);
    ROM_GPIOIntTypeSet(GPIO_PORTC_BASE,GPIO_PIN_7, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTC_BASE,GPIO_PIN_6);
    GPIOIntEnable(GPIO_PORTC_BASE,GPIO_PIN_7);
}

void GPIOCIntHandler(void){
    //uint32_t ui32Mode;
    unsigned int Status;
    Status = GPIOIntStatus(GPIO_PORTC_BASE,true);
    GPIOIntClear(GPIO_PORTC_BASE,Status);
//    if(test_flag)
//        return;
    if(Status==GPIO_PIN_7)
    {
        m=0;                      //���м�������㣬�Ա��ڿ�ʼ�ӰѲɼ���ͼ��ŵ�����ĵ�һ��
        Line=0;                   //���ж���ʱ��������
        hang=0;                   //����ʱ�������
        SampleFlag=(++SampleFlag)%5;   //���жϱ�־����֡ȡһ֡
        testcount++;
        if(testcount>1000)
            test_flag=1;
    }
    else if(Status == GPIO_PIN_6)
    {

        testcount2++;
        Line++;         //���жϼ�������
        if ( SampleFlag == 0 || Line<ROW_START || Line>ROW_MAX ){
            return;     //����Ҫ�ɼ�ͼ�����Ч�У�����
        }
        if( Line==Get_Image[hang]){
            //SysCtlDelay(12);//���ͼ�����м����޸�����
            //uDMAChannelTransferSet(UDMA_CHANNEL_ETH0TX | UDMA_PRI_SELECT,
            //                           UDMA_MODE_AUTO, (void *)(GPIO_PORTB_BASE + GPIO_O_DATA + (0x0ff << 2)), Buffer[m],
            //                           MEM_BUFFER_SIZE);
            //SysCtlDelay(12);
            //uDMAChannelEnable(UDMA_CHANNEL_ETH0TX);
            //uDMAChannelRequest(UDMA_CHANNEL_ETH0TX);

            Buffer[m][0]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][1]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][2]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][3]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][4]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][5]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][6]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][7]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][8]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][9]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][10]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][11]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][12]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][13]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][14]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][15]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][16]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][17]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][18]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][19]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][20]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][21]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][22]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][23]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][24]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][25]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][26]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][27]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][28]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][29]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][30]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][31]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][32]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][33]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][34]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][35]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][36]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][37]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][38]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][39]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][40]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][41]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][42]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][43]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][44]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][45]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][46]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][47]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][48]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][49]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][50]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][51]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][52]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][53]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][54]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][55]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][56]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][57]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][58]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);
            Buffer[m][59]=ROM_GPIOPinRead(GPIO_PORTB_BASE,0xe0);

            hang++;
            m++;
        }
    }
}
//void GPIOBIntHandler(void){
//    unsigned int Status;
//    Status = GPIOIntStatus(GPIO_PORTB_BASE,true);
//    GPIOIntClear(GPIO_PORTB_BASE,Status);
//    if(Status == GPIO_PIN_6){
//        m=0;                      //���м�������㣬�Ա��ڿ�ʼ�ӰѲɼ���ͼ��ŵ�����ĵ�һ��
//        Line=0;                   //���ж���ʱ��������
//        hang=0;                   //����ʱ�������
//        SampleFlag=(++SampleFlag)%3;   //���жϱ�־����֡ȡһ֡
//        //testcount++;
//        //if(testcount>1000)
//            //test_flag=1;
//    }
//}

void GPIOB_Init()
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTB_BASE + GPIO_O_CR) |= 0xff;
    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = 0;
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_0);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D0Ϊ�������� �����������2mA
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_1);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_1,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D1Ϊ�������� �����������2mA
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_2);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D2Ϊ�������� �����������2mA
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_3);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D3Ϊ�������� �����������2mA
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_4);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D4Ϊ�������� �����������2mA
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_5);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D5Ϊ�������� �����������2mA
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_6);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_6,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D6Ϊ�������� �����������2mA
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_7);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_7,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D7Ϊ�������� �����������2mA
}
