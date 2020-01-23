#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "inc/hw_gpio.h"

#define MEM_BUFFER_SIZE         1024

void UDMAInit(void);
uint8_t ui8ControlTable[1024];
unsigned int uIdx,ClockPeriod;
static unsigned char g_ui32DstBuf[MEM_BUFFER_SIZE];
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t ui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ui8ControlTable, 1024)
uint8_t ui8ControlTable[1024];
#else
uint8_t ui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

int main(void)
{
    FPUEnable();
    FPULazyStackingEnable();
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    ClockPeriod=SysCtlClockGet();
    UDMAInit();
}
//void
//uDMAIntHandler(void)
//{
//    uint32_t ui32Mode;
//
//    //
//    // Check for the primary control structure to indicate complete.
//    //
//    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_SW);
//    if(ui32Mode == UDMA_MODE_STOP)
//    {
//        //
//        // Increment the count of completed transfers.
//        //
//        g_ui32MemXferCount++;
//
//        //
//        // Configure it for another transfer.
//        //
//        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SW, UDMA_MODE_AUTO,
//                                     g_ui32SrcBuf, g_ui32DstBuf,
//                                     MEM_BUFFER_SIZE);
//
//        //
//        // Initiate another transfer.
//        //
//        ROM_uDMAChannelEnable(UDMA_CHANNEL_SW);
//        ROM_uDMAChannelRequest(UDMA_CHANNEL_SW);
//    }
//
//    //
//    // If the channel is not stopped, then something is wrong.
//    //
//    else
//    {
//        g_ui32BadISR++;
//    }
//}
void UDMAInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(ui8ControlTable);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //PD解锁
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0xff;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//配置D0为上拉电阻 输出电流能力2mA
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_1,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//配置D1为上拉电阻 输出电流能力2mA
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_2);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//配置D2为上拉电阻 输出电流能力2mA
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//配置D3为上拉电阻 输出电流能力2mA
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//配置D4为上拉电阻 输出电流能力2mA
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_5);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_5,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//配置D5为上拉电阻 输出电流能力2mA
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_6,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//配置D6为上拉电阻 输出电流能力2mA
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_7,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//配置D7为上拉电阻 输出电流能力2mA

    //IntEnable(INT_UDMA);

    uDMAChannelAssign(UDMA_CH7_GPIOD);
    uDMAChannelAttributeDisable(UDMA_CHANNEL_ETH0TX,
                                    UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
                                    (UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK));

    uDMAChannelControlSet(UDMA_CHANNEL_ETH0TX | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                              UDMA_ARB_8);

    uDMAChannelTransferSet(UDMA_CHANNEL_ETH0TX | UDMA_PRI_SELECT,
                               UDMA_MODE_AUTO, (void *)(GPIO_PORTD_BASE + GPIO_O_DATA + (0x0ff << 2)), g_ui32DstBuf,
                               MEM_BUFFER_SIZE);

    //uDMAChannelEnable(UDMA_CHANNEL_ETH0TX);
    //uDMAChannelRequest(UDMA_CHANNEL_ETH0TX);
}
