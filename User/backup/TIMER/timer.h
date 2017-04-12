#ifndef TIMER_H
#define TIMER_H
#include "stm32f10x.h"
#include "LED.h"
#include "USART.h"

extern u8 LED_blink;


void TIMx_Int_Init(TIM_TypeDef * TIMx);
void timer3_ON(u32 time);


#endif
