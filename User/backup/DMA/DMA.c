#include "DMA.h"

u16 DMA_data;


void ADC_DMA_Configuration(ADC_TypeDef * ADCx)
{
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Channel1);
	if( ADCx == ADC1)
	{
		DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	}
	if( ADCx == ADC2)
	{
		DMA_InitStructure.DMA_PeripheralBaseAddr = ADC2_DR_Address;
	}
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&DMA_data;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_DMACmd(ADCx,ENABLE);
}

void DMA_start(void)
{
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);       //ADC发送完成中断
}

void DMA_stop(void)
{
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,DISABLE);       //关闭中断
}


void DMA1_Channel1_IRQHandler(void)
{
	if((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == SET) 
	{
		float f_result;
		u8 s_DMA_data[6] = {0};
		f_result = (float)DMA_data/4096 * 3.3;
		s_DMA_data[0] = (u8)f_result + '0';
		s_DMA_data[1] = '.';
		s_DMA_data[2] = (u8)(f_result * 10) % 10 + '0';
		s_DMA_data[3] = (u8)(f_result * 100) % 10 + '0';
		s_DMA_data[4] = '\n';
		USART1_sendStr(s_DMA_data);
		DMA_ClearFlag(DMA1_FLAG_TC1);
	}
}
