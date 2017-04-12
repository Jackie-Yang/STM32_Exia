#ifndef SETUP_H
#define SETUP_H
#include "stm32f10x.h"


#define LED_OFF GPIO_SetBits(GPIOC,GPIO_Pin_13)
#define LED_ON	GPIO_ResetBits(GPIOC,GPIO_Pin_13)

/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)				{p->BSRR=i;}			//����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)				{p->BRR	=i;}			//����͵�ƽ
#define digitalToggle(p,i)		    {p->ODR ^=i;}			//�����ת״̬

#define LED	     GPIO_Pin_13

/* �궨�������յLED */
#define TOGGLE(led)		digitalToggle(GPIOC, led)
#define ON(led)				digitalHi(GPIOC, led)
#define OFF(led)			digitalLo(GPIOC, led) 

void init(void);				   //ϵͳ��ʼ��,����ģ���ʼ�������ļ���
void RCC_Configuration(void);	   //ϵͳ������ʱ������
void GPIO_Configuration(void);	   //GPIO����
void NVIC_Configuration(void);	   //�ж���������
void EXIT_Configuration(void);	   //�ⲿ�ж�����



#endif
