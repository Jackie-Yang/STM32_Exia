#ifndef SETUP_H
#define SETUP_H
#include "stm32f10x.h"


#define LED_OFF GPIO_SetBits(GPIOC,GPIO_Pin_13)
#define LED_ON	GPIO_ResetBits(GPIOC,GPIO_Pin_13)

/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)				{p->BSRR=i;}			//设置为高电平		
#define digitalLo(p,i)				{p->BRR	=i;}			//输出低电平
#define digitalToggle(p,i)		    {p->ODR ^=i;}			//输出反转状态

#define LED	     GPIO_Pin_13

/* 宏定义控制两盏LED */
#define TOGGLE(led)		digitalToggle(GPIOC, led)
#define ON(led)				digitalHi(GPIOC, led)
#define OFF(led)			digitalLo(GPIOC, led) 

void init(void);				   //系统初始化,所有模块初始化函数的集合
void RCC_Configuration(void);	   //系统、外设时钟配置
void GPIO_Configuration(void);	   //GPIO配置
void NVIC_Configuration(void);	   //中断向量配置
void EXIT_Configuration(void);	   //外部中断配置



#endif
