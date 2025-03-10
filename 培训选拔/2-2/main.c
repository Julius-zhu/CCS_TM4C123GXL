//UART和AD配合，当UART发送1，则UART返回当前温度；
//当UART发送2，则UART返回D2管脚电压；
//当UART发送3，则UART返回D1管脚电压；
//当UART发送4，则UART返回D0管脚电压。
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"

void UARTSend(const uint8_t *pucBuffer, uint32_t ulCount);

int main(void)
{
    FPUEnable();
    FPULazyStackingEnable();
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeADC(GPIO_PORTD_BASE,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE,GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE,SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    ADCReferenceSet(ADC0_BASE,ADC_REF_EXT_3V);
    ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR,0);
    ADCSequenceStepConfigure(ADC0_BASE,0,0,ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE,0,1,ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE,0,2,ADC_CTL_CH5);
    ADCSequenceStepConfigure(ADC0_BASE,0,3,ADC_CTL_CH5);
    ADCSequenceStepConfigure(ADC0_BASE,0,4,ADC_CTL_CH6);
    ADCSequenceStepConfigure(ADC0_BASE,0,5,ADC_CTL_CH6);
    ADCSequenceStepConfigure(ADC0_BASE,0,6,ADC_CTL_CH7);
    ADCSequenceStepConfigure(ADC0_BASE,0,7,ADC_CTL_CH7|ADC_CTL_IE|ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE,0);

    IntMasterEnable();

    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE,UART_INT_RX | UART_INT_RT);

    while(1){
    }

}

void UARTIntHandler(void)
{
    uint32_t ulStatus;
    char ch = 0;
    ulStatus = UARTIntStatus(UART0_BASE, true);
    uint32_t ui32ADC0Value[8];
    volatile double ui32TempAvg;
    volatile double ui32TempValueC;
    uint8_t ui8output[4];

    UARTIntClear(UART0_BASE, ulStatus);

    ch = UARTCharGet(UART0_BASE);
    ADCIntClear(ADC0_BASE,0);
    ADCProcessorTrigger(ADC0_BASE,0);

    while (!ADCIntStatus(ADC0_BASE,0,false)){

    }

    ADCSequenceDataGet(ADC0_BASE,0,ui32ADC0Value);
    switch(ch)
    {
    case '1':

        ui32TempAvg =(ui32ADC0Value[0]+ui32ADC0Value[1]+1)/2.0;

        ui32TempValueC = (1475 - (2475*ui32TempAvg)/4096)/10.0;

        break;
    case '2':
        ui32TempValueC = (ui32ADC0Value[2] + ui32ADC0Value[3]+1)/2*3.3/4096;

        break;
    case '3':
        ui32TempValueC = (ui32ADC0Value[4] + ui32ADC0Value[5]+1)/2*3.3/4096;

        break;
    case '4':
        ui32TempValueC = (ui32ADC0Value[6] + ui32ADC0Value[7]+1)/2*3.3/4096;

        break;

    }

    ui8output[0] = ui32TempValueC/10;
    ui8output[1] = (ui32TempValueC-ui8output[0]*10)/1;
    ui8output[2] = (ui32TempValueC*10-ui8output[0]*100-ui8output[1]*10)/1;
    ui8output[3] = (ui32TempValueC*100-ui8output[0]*1000-ui8output[1]*100-ui8output[2]*10)/1;

    ui8output[0] = ui8output[0]+48;
    ui8output[1] = ui8output[1]+48;
    ui8output[2] = ui8output[2]+48;
    ui8output[3] = ui8output[3]+48;

    UARTSend(ui8output, 4);
    UARTCharPut(UART0_BASE,'\r');
    UARTCharPut(UART0_BASE,'\n');
}


void UARTSend(const uint8_t *pucBuffer, uint32_t ulCount){
    while(ulCount--){
        UARTCharPut(UART0_BASE,*pucBuffer++);
    }

}
