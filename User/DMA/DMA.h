#ifndef DMA_H
#define DMA_H
#include "stm32f10x.h"

#define USART1_DR_Base 0x40013804	   //���裨���ڣ������ݵ�ַ


void DMA_Configuration(void * DMA_Buff_Addr, uint32_t DMA_BufferSize);				  //����DMA���������ڴ洫�䵽����


#endif
