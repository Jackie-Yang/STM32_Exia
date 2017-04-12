#include "DMA.h"		  

//u8 DMA_Buff[DMA_BUFF_SIZE + 6] = {0};
u16 DMA_Buff[DMA_BUFF_SIZE + 4] = {0};

//u16 temp[] = {0x1234,0x5678,(u16)-1};

void DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	//DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)DMA_Buff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = (DMA_BUFF_SIZE+4)*2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	/* Enable DMA channel4 */

	//���ݰ�ͷ����Χ�����ݴ�С��У��
	//ע�⣬stm32ΪС��(little endian),���ȷ��͵�λ�ֽ�
	DMA_Buff[0] = 0xFF7F;
	//DMA_Buff[1] = 0x;
	DMA_Buff[1] = DMA_BUFF_SIZE * 2;

	DMA_Buff[DMA_BUFF_SIZE + 4 - 2] = 123;
	//DMA_Buff[DMA_BUFF_SIZE + 6 - 2] = 0xFF;
	DMA_Buff[DMA_BUFF_SIZE + 4 - 1] = 0xFEFF;
	


}

void DMA_Buff_In_16(u16 data,u8 index)
{
//	index = index * 2 + 3;					 //�����ʵ�ʴ��λ�ã��������ݶ�Ϊ2�ֽڣ�������3�ֽڵİ�ͷ������λ��
//	
//	DMA_Buff[index] = (u8)(data & 0xFF);	  //�ȵ�λ���λ
//	DMA_Buff[index + 1] = (u8)(data >> 8);
	index = index + 2;
	DMA_Buff[index] = data;
}

void DMA_Buff_In_32(u32 data,u8 index)
{
//	index = index * 2 + 3;
//	DMA_Buff[index++] = (u8)(data & 0xFF);
//	DMA_Buff[index++] = (u8)((data >> 8) & 0xFF);
//	DMA_Buff[index++] = (u8)((data >> 16) & 0xFF);
//	DMA_Buff[index] = (u8)(data >> 24);
	index = index + 2;
	DMA_Buff[index++] = (u16)(data & 0xFFFF);
	DMA_Buff[index] = (u16)(data >> 16);
}
