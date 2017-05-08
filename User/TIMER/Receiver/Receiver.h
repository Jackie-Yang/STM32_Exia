#ifndef RECEIVER_H
#define RECEIVER_H
#include "stm32f10x.h"

// #define REC(n) DMA_Buff[n + 2]

// #define REC_RUDD	REC(0) 	//Inputs[0].capture	 //方向舵 偏摆
// #define REC_THRO	REC(1) 	//Inputs[1].capture	 //油门
// #define REC_AILE	REC(2) 	//Inputs[2].capture	 //副翼   测滚
// #define REC_ELEV	REC(3) 	//Inputs[3].capture	 //升降	  俯仰

void Receiver_Init(void); //定时器3初始化，用于捕获接收器脉冲信号

#endif
