#ifndef USART_H
#define USART_H
#include "stm32f10x.h"





void USART_Configuration(void);			//��������
void USART1_sendData_u8(u8 data);		//���ڷ���8bit����
void USART1_sendData_u16(u16 data);	    //���ڷ���16bit����
void USART1_sendData_u32(u32 data);	    //���ڷ���32bit����
void USART1_sendStr(u8 *data);			//���ڷ����ַ���

void check_BT(void);					//�����������ģ���Ƿ���

extern u8 BT_state;



#endif
