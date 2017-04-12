#include "button.h"

//�������򰴼��ⲿ�ж�����
void EXIT_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource0);     //����D0����Ϊ�ж�Դ
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource1);     //����D1����Ϊ�ж�Դ
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);     //����D2����Ϊ�ж�Դ
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);     //����D3����Ϊ�ж�Դ
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource4);     //����D4����Ϊ�ж�Դ	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3 | EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		        //�ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	        //�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				        //ʹ���ж���
	EXTI_Init(&EXTI_InitStructure);		
}


//�ر����򰴼��ⲿ�ж�
void EXIT_Close(void)			  
{
	EXTI_InitTypeDef EXTI_InitStructure;	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3 | EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		        //�ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	        //�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;				        //�ر��ж���
	EXTI_Init(&EXTI_InitStructure);	
}


/**********************���򰴼��ⲿ�жϷ�����***********************/
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0)!= RESET)	 //�ж��Ƿ����ж�
	{
	  USART1_sendStr("JOY Stick Pressed!\n");							 
	}
	EXTI_ClearITPendingBit(EXTI_Line0);		     //����жϱ�־			
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1)!= RESET)	 //�ж��Ƿ����ж�
	{
	  USART1_sendStr("JOY Stick Right!\n");							 
	}
	EXTI_ClearITPendingBit(EXTI_Line1);		     //����жϱ�־			
}

void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2)!= RESET)	 //�ж��Ƿ����ж�
	{
	  USART1_sendStr("JOY Stick Down!\n");							 
	}
	EXTI_ClearITPendingBit(EXTI_Line2);		     //����жϱ�־			
}

void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3)!= RESET)	 //�ж��Ƿ����ж�
	{
	  USART1_sendStr("JOY Stick Up!\n");							 
	}
	EXTI_ClearITPendingBit(EXTI_Line3);		     //����жϱ�־			
}

void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4)!= RESET)	 //�ж��Ƿ����ж�
	{
	  USART1_sendStr("JOY Stick Left!\n");							 
	}
	EXTI_ClearITPendingBit(EXTI_Line4);		     //����жϱ�־			
}

