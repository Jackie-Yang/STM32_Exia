#ifndef ADC_H
#define ADC_H

#include "stm32f10x.h"


void ADC_init(ADC_TypeDef * ADCx,uint8_t ADC_Channel);
u8* getADC_Result(ADC_TypeDef * ADCx);



#endif
