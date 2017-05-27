#ifndef SETUP_H
#define SETUP_H
#include "stm32f10x.h"
#include "Quadrotor_State.h"



//����������
extern Quadrotor_State stQuadrotor_State;
extern uint8_t DebugMode;

int8_t init(void);                 //ϵͳ��ʼ��,����ģ���ʼ�������ļ���
void RCC_Configuration(void);	   //ϵͳ������ʱ������
void GPIO_Configuration(void);	   //GPIO����
void NVIC_Configuration(void);	   //�ж���������
void EXIT_Configuration(void);	   //�ⲿ�ж�����



#endif
