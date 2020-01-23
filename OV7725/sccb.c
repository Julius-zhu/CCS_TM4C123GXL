//#include "sys.h"
#include "sccb.h"
//#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ο�������guanfu_wang���롣
//ALIENTEK MiniSTM32������
//OV7670 ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/10/31
//�汾��V1.0		    							    							  
//////////////////////////////////////////////////////////////////////////////////

//��ʼ��SCCB�ӿ�
//CHECK OK
void delay_us(uint16_t delay_time)
{
    SysCtlDelay(SysCtlClockGet()/3/1000000*delay_time);
}
void delay_ms(uint16_t delay_time)
{
    SysCtlDelay(SysCtlClockGet()/3/1000*delay_time);
}
void SCCB_Init(void)
{											   
 
/*
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //ʹ��PC�˿�ʱ��
 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;				 // �˿�����
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOC,GPIO_Pin_4|GPIO_Pin_5);						 // �����
 
	SCCB_SDA_OUT();	   
*/

/*�ŵ�main������
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1|GPIO_PIN_2);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_1|GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//�������
*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//�������
    //IO���ٶȣ�
    SCCB_SCL_H;SCCB_SDA_H;
    SCCB_SDA_OUT();

}			 

//SCCB��ʼ�ź�
//��ʱ��Ϊ�ߵ�ʱ��,�����ߵĸߵ���,ΪSCCB��ʼ�ź�
//�ڼ���״̬��,SDA��SCL��Ϊ�͵�ƽ
void SCCB_Start(void)
{
    SCCB_SDA_OUT();
    SCCB_SDA_H;     //�����߸ߵ�ƽ
    delay_us(50);

    SCCB_SCL_H;	    //��ʱ���߸ߵ�ʱ���������ɸ�����
    delay_us(50);  

    SCCB_SDA_L;
    delay_us(50);

    SCCB_SCL_L;	    //�����߻ָ��͵�ƽ��������������Ҫ
    delay_us(50);
}

//SCCBֹͣ�ź�
//��ʱ��Ϊ�ߵ�ʱ��,�����ߵĵ͵���,ΪSCCBֹͣ�ź�
//����״����,SDA,SCL��Ϊ�ߵ�ƽ
void SCCB_Stop(void)
{
    SCCB_SDA_OUT();
    SCCB_SDA_L;
    delay_us(50);

    SCCB_SCL_H;
    delay_us(50); 

    SCCB_SDA_H;
    delay_us(50);
}  
//����NA�ź�
void SCCB_No_Ack(void)
{
    SCCB_SDA_OUT();
	SCCB_SDA_H;
	delay_us(50);

	SCCB_SCL_H;
	delay_us(50);

	SCCB_SCL_L;
	delay_us(50);

	SCCB_SDA_L;
	delay_us(50);
}
//SCCB,д��һ���ֽ�
//����ֵ:0,�ɹ�;1,ʧ��. 
uint8_t SCCB_WR_Byte(uint8_t dat)
{
	uint8_t j,res;
	for(j=0;j<8;j++) //ѭ��8�η�������
	{
		if(dat&0x80) SCCB_SDA_H;
		else SCCB_SDA_L;
		dat<<=1;
		delay_us(50);
		SCCB_SCL_H;
		delay_us(50);
		SCCB_SCL_L;
	}			 
	SCCB_SDA_IN();		//����SDAΪ���� 
	delay_us(50);
	SCCB_SCL_H;			//���յھ�λ,���ж��Ƿ��ͳɹ�
	delay_us(50);
	if(SCCB_READ_SDA)res=1;  //SDA=1����ʧ�ܣ�����1
	else res=0;         //SDA=0���ͳɹ�������0
	SCCB_SCL_L;
	SCCB_SDA_OUT();		//����SDAΪ���    
	//UARTprintf("SCCB_WR_Byte_res=%d\n",res);
	return res;  
}	 
//SCCB ��ȡһ���ֽ�
//��SCL��������,��������
//����ֵ:����������
uint8_t SCCB_RD_Byte(void)
{
	uint8_t temp=0,j;
	SCCB_SDA_IN();		//����SDAΪ����  
	delay_us(50);
	for(j=8;j>0;j--) 	//ѭ��8�ν�������
	{		     	  
		delay_us(50);
		SCCB_SCL_H;
		temp=temp<<1;
		if(SCCB_READ_SDA)temp++;   
		delay_us(50);
		SCCB_SCL_L;
	}	
	SCCB_SDA_OUT();		//����SDAΪ���    
	//UARTprintf("SCCB_RD_Byte_temp=%x\n",temp);
	return temp;
} 							    
//д�Ĵ���
//����ֵ:0,�ɹ�;1,ʧ��.
uint8_t SCCB_WR_Reg(uint8_t reg,uint8_t data)
{
	uint8_t res=0;
	SCCB_Start(); 					//����SCCB����
	if(SCCB_WR_Byte(SCCB_ID))res=1;	//д����ID	  
	delay_us(100);
  	if(SCCB_WR_Byte(reg))res=1;		//д�Ĵ�����ַ	  
	delay_us(100);
  	if(SCCB_WR_Byte(data))res=1; 	//д����	 
  	SCCB_Stop();

  	return	res;
}		  					    
//���Ĵ���
//����ֵ:�����ļĴ���ֵ
uint8_t SCCB_RD_Reg(uint8_t reg)
{
	uint8_t val=0;
	SCCB_Start(); 				//����SCCB����

	SCCB_WR_Byte(SCCB_ID);		//д����ID	����ֵres=0��д���ֽڳɹ�
	delay_us(100);	 

  	SCCB_WR_Byte(reg);			//д�Ĵ�����ַ	  
	delay_us(100);	  

	SCCB_Stop();   
	delay_us(100);	   
	//���üĴ�����ַ�󣬲��Ƕ�
	SCCB_Start();
	SCCB_WR_Byte(SCCB_ID|0X01);	//���Ͷ�����	  
	delay_us(100);
  	val=SCCB_RD_Byte();		 	//��ȡ����
  	SCCB_No_Ack();
  	SCCB_Stop();
  	return val;
}















