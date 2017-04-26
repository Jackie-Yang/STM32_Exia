#ifndef	TIMER_H
#define TIMER_H
#include "stm32f10x.h"

// #define REC(n) DMA_Buff[n + 2]

// #define REC_RUDD	REC(0) 	//Inputs[0].capture	 //����� ƫ��
// #define REC_THRO	REC(1) 	//Inputs[1].capture	 //����
// #define REC_AILE	REC(2) 	//Inputs[2].capture	 //����   ���
// #define REC_ELEV	REC(3) 	//Inputs[3].capture	 //����	  ����


struct PWM_State
{
    u8 state;
    u16 rise;
    u16 fall;
//  u16 capture;
};

extern struct PWM_State Inputs[];

void TIM2_Init(void);		  //��ʱ��2��ʼ�������ڲ�������������ź�
void TIM3_Init(void);		  //��ʱ��3��ʼ�����������PWM����ź�
void TIM4_Init(void);		  //��ʱ��4��ʼ�������ڲ�����ʱ���жϣ�ִ�в�������10ms�����ݲ���
void set_motorPWM(u8 motor,u16 motorPWM);	   //���ö�ʱ��3�����PWM

//extern u32 system_time;

 




#endif

