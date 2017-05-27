#ifndef SETUP_H
#define SETUP_H
#include "stm32f10x.h"
#include "Quadrotor_State.h"



//飞行器数据
extern Quadrotor_State stQuadrotor_State;
extern uint8_t DebugMode;

int8_t init(void);                 //系统初始化,所有模块初始化函数的集合
void RCC_Configuration(void);	   //系统、外设时钟配置
void GPIO_Configuration(void);	   //GPIO配置
void NVIC_Configuration(void);	   //中断向量配置
void EXIT_Configuration(void);	   //外部中断配置



#endif
