#ifndef DMA_H
#define DMA_H
#include "stm32f10x.h"
#include "USART.h"


#define ADC1_DR_Address ((uint32_t)0x40012400+0X4C)
#define ADC2_DR_Address ((uint32_t)0x40012800+0X4C)

void ADC_DMA_Configuration(ADC_TypeDef * ADCx);
void DMA_start(void);
void DMA_stop(void);

#endif



