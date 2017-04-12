#ifndef LED_H
#define LED_H
#include "stm32f10x.h"

/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)				{p->BSRR=i;}			//设置为高电平		
#define digitalLo(p,i)				{p->BRR	=i;}			//输出低电平
#define digitalToggle(p,i)		    {p->ODR ^=i;}			//输出反转状态

#define LED1     GPIO_Pin_8 
#define LED2     GPIO_Pin_9 
#define LED3     GPIO_Pin_10 
#define LED4     GPIO_Pin_11 

/* 宏定义控制两盏LED */
#define TOGGLE(led)		digitalToggle(GPIOD, led)
#define ON(led)				digitalHi(GPIOD, led)
#define OFF(led)			digitalLo(GPIOD, led)


//两盏LED闪两下
#define WELCOME  ON(LED1)  ON(LED2)   ON(LED3)   ON(LED4)   delay_ms(200);  OFF(LED1)  OFF(LED2)  OFF(LED3)   OFF(LED4)   delay_ms(200);\
                 ON(LED1)  ON(LED2)   ON(LED3)   ON(LED4)   delay_ms(200);  OFF(LED1)  OFF(LED2)  OFF(LED3)   OFF(LED4)   delay_ms(200);

extern u8 timer3_State;


void Blink_ON(u8 mode);
void Blink_OFF(void);
void LED_run1(void);
void LED_run2(void);
void LED_run3(void);

#endif
