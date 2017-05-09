#ifndef DMA_H
#define DMA_H
#include "stm32f10x.h"

#define USART1_DR_Base 0x40013804	   //外设（串口）的数据地址


void DMA_Configuration(void * DMA_Buff_Addr, uint32_t DMA_BufferSize);				  //配置DMA，数据由内存传输到串口


#endif
