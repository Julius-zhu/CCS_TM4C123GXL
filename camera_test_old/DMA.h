/*
 * DMA.h
 *
 *  Created on: 2019��7��14��
 *      Author: ZJN
 */

#ifndef DMA_H_
#define DMA_H_

#define MEM_BUFFER_SIZE         256

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


void UDMAInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(ui8ControlTable);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //PB����
    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTB_BASE + GPIO_O_CR) |= 0xff;
    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = 0;
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D0Ϊ�������� �����������2mA
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_1,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D1Ϊ�������� �����������2mA
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_2);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D2Ϊ�������� �����������2mA
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D3Ϊ�������� �����������2mA
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D4Ϊ�������� �����������2mA
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_5);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D5Ϊ�������� �����������2mA
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_6,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D6Ϊ�������� �����������2mA
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_7,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);//����D7Ϊ�������� �����������2mA

    uDMAChannelAssign(UDMA_CH5_GPIOB);
    uDMAChannelAttributeDisable(UDMA_CHANNEL_USBEP3TX,
                                    UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
                                    (UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK));

    uDMAChannelControlSet(UDMA_CHANNEL_USBEP3TX | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                              UDMA_ARB_8);

    //uDMAChannelEnable(UDMA_CHANNEL_ETH0TX);
    //uDMAChannelRequest(UDMA_CHANNEL_ETH0TX);
}

#endif /* DMA_H_ */
