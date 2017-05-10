#ifndef LED_BLINK_H
#define LED_BLINK_H
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

#define BLINK_LOOP -1
#define BLINK_KEEP -2
#define BLINK_STOP -3
typedef const struct _LED_BLINK_MODE_
{
    int8_t Status;
    uint32_t Time;
} LED_BlinkMode, *pLED_BlinkMode;

typedef struct _BLINK_CYC_NODE_
{
    pLED_BlinkMode pBlinkMode;
    struct _BLINK_CYC_NODE_* Next;
    struct _BLINK_CYC_NODE_* Pre;
} BlinkCycNode, *pBlinkCyckNode;

extern LED_BlinkMode Blink_Init[];
extern LED_BlinkMode BT_Connect[];
extern LED_BlinkMode Blink_WARNING[];
extern LED_BlinkMode Blink_ERROR[];

void LED_Blink_Init(void); //定时器1初始化，用于LED闪烁
void StartBlink(pLED_BlinkMode pBlinkMode);
void StopBlink(pLED_BlinkMode pBlinkMode);
#endif
