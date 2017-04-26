#ifndef	TIMER_H
#define TIMER_H
#include "stm32f10x.h"

// #define REC(n) DMA_Buff[n + 2]

// #define REC_RUDD	REC(0) 	//Inputs[0].capture	 //方向舵 偏摆
// #define REC_THRO	REC(1) 	//Inputs[1].capture	 //油门
// #define REC_AILE	REC(2) 	//Inputs[2].capture	 //副翼   测滚
// #define REC_ELEV	REC(3) 	//Inputs[3].capture	 //升降	  俯仰


struct PWM_State
{
    u8 state;
    u16 rise;
    u16 fall;
//  u16 capture;
};

extern struct PWM_State Inputs[];

void TIM2_Init(void);		  //定时器2初始化，用于捕获接收器脉冲信号
void TIM3_Init(void);		  //定时器3初始化，用于输出PWM电机信号
void TIM4_Init(void);		  //定时器4初始化，用于产生定时器中断，执行采样周期10ms的数据采样
void set_motorPWM(u8 motor,u16 motorPWM);	   //设置定时器3输出的PWM

//extern u32 system_time;

 




#endif

