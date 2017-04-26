#ifndef OS_H
#define OS_H

#include "string.h"			//�ַ�����������ָ��
#include "Setup.h"			//ϵͳʱ�ӡ�IO���жϳ�ʼ������
#include "USART.h"			
#include "LED.h"		   
#include "timer.h"
#include "button.h"
#include "ADC.h"
#include "DMA.h"
#include "AT24C02.h"


#define ORDER_SIZE 257            //����ָ����󳤶ȣ�ʵ������Ҫ�ȴγ���Сһ�ֽڣ���ΪҪ�����ַ�����������




void OS_init(void);
void runOrder(void);
u8 getOrder(void);
u8 getNum(void);
u8 getNum_HEX(void);
u8* getData(void);

#endif

