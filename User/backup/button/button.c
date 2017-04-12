#include "button.h"

//配置五向按键外部中断配置
void EXIT_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource0);     //配置D0引脚为中断源
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource1);     //配置D1引脚为中断源
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);     //配置D2引脚为中断源
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);     //配置D3引脚为中断源
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource4);     //配置D4引脚为中断源	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3 | EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		        //中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	        //下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				        //使能中断线
	EXTI_Init(&EXTI_InitStructure);		
}


//关闭五向按键外部中断
void EXIT_Close(void)			  
{
	EXTI_InitTypeDef EXTI_InitStructure;	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3 | EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		        //中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	        //下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;				        //关闭中断线
	EXTI_Init(&EXTI_InitStructure);	
}


/**********************五向按键外部中断服务函数***********************/
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0)!= RESET)	 //判断是否发生中断
	{
	  USART1_sendStr("JOY Stick Pressed!\n");							 
	}
	EXTI_ClearITPendingBit(EXTI_Line0);		     //清除中断标志			
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1)!= RESET)	 //判断是否发生中断
	{
	  USART1_sendStr("JOY Stick Right!\n");							 
	}
	EXTI_ClearITPendingBit(EXTI_Line1);		     //清除中断标志			
}

void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2)!= RESET)	 //判断是否发生中断
	{
	  USART1_sendStr("JOY Stick Down!\n");							 
	}
	EXTI_ClearITPendingBit(EXTI_Line2);		     //清除中断标志			
}

void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3)!= RESET)	 //判断是否发生中断
	{
	  USART1_sendStr("JOY Stick Up!\n");							 
	}
	EXTI_ClearITPendingBit(EXTI_Line3);		     //清除中断标志			
}

void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4)!= RESET)	 //判断是否发生中断
	{
	  USART1_sendStr("JOY Stick Left!\n");							 
	}
	EXTI_ClearITPendingBit(EXTI_Line4);		     //清除中断标志			
}

