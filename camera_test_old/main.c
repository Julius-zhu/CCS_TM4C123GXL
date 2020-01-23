//PA2 PA3 PA4 为中断引脚
//PD0-PD3 PC4-PC7连Y0-Y7
//VSY场同步信号 上升触发 短脉冲 60hz  H 行中断 上升触发 14khz左右 pk 像素同步13.5mhz
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
#define ROW        40                 //数字摄像头所采集的二维数组行数
#define COLUMN     256                //数字摄像头所采集的二维数组列数
#define ROW_START  10                 //数字摄像头二维数组行开始行值
#define ROW_MAX    200                //数字摄像头所采集的二维数组行最大值
#define THRESHOLD  0x68               //图像阈值，根据所采集图像亮度值大小的实际情况调整（OV7620所采集的亮度值大小为0--255）
#define PORTA
unsigned char Buffer[ROW][COLUMN]={0};       //所采集的图像二维数组
unsigned char Image_Center[ROW]={0};        //所采集的图像中心线

unsigned char SampleFlag=1;       //奇偶场标记

unsigned int  m=0;                 //换行变量

unsigned int  Line;               //行中断计数变量
unsigned int  hang;

unsigned int testcount,test_flag;
unsigned int  Get_Image[]={
                         17,19,21,23,25,28,31,34,37,40,43,46,49,53,57,
                         61,65,69,73,77,81,85,89,94,99,105,111,117,123,
                         129,135,141,147,153,159,166,173,180,187,194
                         };     //定每场采哪几行。
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
        Line++;         //行中断计数变量
        if ( SampleFlag == 0 || Line<ROW_START || Line>ROW_MAX ){
            return;     //不是要采集图像的有效行，返回
        }
        if( Line==Get_Image[hang]){
            //SysCtlDelay(12);//如果图像不在中间请修改这里
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
        m=0;                      //行中间变量清零，以便于开始从把采集的图像放到数组的第一行
        Line=0;                   //行中断临时变量清零
        hang=0;                   //行临时变量清除
        SampleFlag=~SampleFlag;   //场中断标记取反，这样只采集奇数场的图像
        //testcount++;
        //if(testcount>200)
            //test_flag=1;
    }
}

