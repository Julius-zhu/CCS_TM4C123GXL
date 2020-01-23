//PA2 PA3 PA4 Ϊ�ж�����
//PD0-PD3 PC4-PC7��Y0-Y7
//VSY��ͬ���ź� �������� ������ 60hz  H ���ж� �������� 14khz���� pk ����ͬ��13.5mhz
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
#define ROW        40                 //��������ͷ���ɼ��Ķ�ά��������
#define COLUMN     256                //��������ͷ���ɼ��Ķ�ά��������
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

unsigned int testcount,test_flag;
unsigned int  Get_Image[]={
                         17,19,21,23,25,28,31,34,37,40,43,46,49,53,57,
                         61,65,69,73,77,81,85,89,94,99,105,111,117,123,
                         129,135,141,147,153,159,166,173,180,187,194
                         };     //��ÿ�����ļ��С�
void UARTSend(const uint8_t *pucBuffer, uint32_t ulCount);
void UART0Init();
void YNCInit();

unsigned char i,j;
volatile char a;
char shift_flag=1;
uint32_t ClockPeriod;
unsigned int Status;
int main(void)
{
    FPUEnable();
    FPULazyStackingEnable();
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    ClockPeriod=SysCtlClockGet();
    UART0Init();
    YNCInit();
    UDMAInit();
    IntMasterEnable();
    uDMAChannelEnable(UDMA_CHANNEL_USBEP3TX);
    uDMAChannelRequest(UDMA_CHANNEL_USBEP3TX);
    while(1){
//       if(test_flag){
//           for(i=0;i<ROW;i++)
//           {
//              for(j=0;j<COLUMN;j++)
//              {
//                  UARTCharPutNonBlocking(UART0_BASE,Buffer[i][j]);
//                  SysCtlDelay(10000);
//              }
//              //SCI_Write(0x0D);
//              //SCI_Write(0X0A);
//           }
//       }
//
////        SysCtlDelay(10000000);
////        for(i1=0;i1<4800;i1++)
////        {
////            UARTCharPutNonBlocking(UART0_BASE,'a');
////            SysCtlDelay(5000);
////        }
//        UARTCharPutNonBlocking(UART0_BASE,0xFF);
    }

}


void YNCInit(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    IntEnable(INT_GPIOC);
    GPIOIntTypeSet(GPIO_PORTC_BASE,GPIO_PIN_6, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTC_BASE,GPIO_PIN_7, GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTC_BASE,GPIO_PIN_6 | GPIO_PIN_7);
}

void GPIOCIntHandler(void){

    Status = GPIOIntStatus(GPIO_PORTC_BASE,true);
    GPIOIntClear(GPIO_PORTC_BASE,Status);

    if(Status == GPIO_PIN_6){
        Line++;         //���жϼ�������
        if ( SampleFlag == 0 || Line<ROW_START || Line>ROW_MAX ){
            return;     //����Ҫ�ɼ�ͼ�����Ч�У�����
        }
        if( Line==Get_Image[hang]){
            //SysCtlDelay(12);//���ͼ�����м����޸�����
            uDMAChannelTransferSet(UDMA_CHANNEL_USBEP3TX | UDMA_PRI_SELECT,
                                       UDMA_MODE_AUTO, (void *)(GPIO_PORTB_BASE + GPIO_O_DATA + (0x0ff << 2)), Buffer[m],
                                       MEM_BUFFER_SIZE);
            //SysCtlDelay(12);
            uDMAChannelEnable(UDMA_CHANNEL_USBEP3TX);
            uDMAChannelRequest(UDMA_CHANNEL_USBEP3TX);
            hang++;
            m++;
        }
    }
    else if(Status == GPIO_PIN_7){
        m=0;                      //���м�������㣬�Ա��ڿ�ʼ�ӰѲɼ���ͼ��ŵ�����ĵ�һ��
        Line=0;                   //���ж���ʱ��������
        hang=0;                   //����ʱ�������
        SampleFlag=~SampleFlag;   //���жϱ��ȡ��������ֻ�ɼ���������ͼ��
        //testcount++;
        //if(testcount>200)
            //test_flag=1;
    }
}

