#include "I2C.h"


void I2C_init(void)
{
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x0A;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;
	I2C_Init(I2C1,&I2C_InitStructure);
	I2C_Cmd(I2C1,ENABLE);
}

void I2C_write(void)
{
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));		 //�ȴ����߿���

	I2C_GenerateSTART(I2C1,ENABLE);						 //���Ϳ�ʼ�ź�
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));	//�ȴ��ɹ���������

	I2C_Send7bitAddress(I2C1,0xA0,I2C_Direction_Transmitter); //����Ϊ����ģʽ
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));	//�ȴ��ӻ�Ӧ�𣬳ɹ�����Ϊ����ģʽ


	I2C_SendData(I2C1,0x01);    //���Ͳ�����ַ
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(I2C1,0x01);    //��������
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));


	I2C_GenerateSTOP(I2C1,ENABLE);			 //���ͽ����ź�
}

u8 I2C_read(void)
{
	u8 data;
//		USART1_sendData(0xFF);

	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));		 //�ȴ����߿���

//	USART1_sendData(0x01);

	I2C_GenerateSTART(I2C1,ENABLE);						 //���Ϳ�ʼ�ź�
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));	//�ȴ��ɹ���������
	USART1_sendData(0x02);

	I2C_Send7bitAddress(I2C1,0xA0,I2C_Direction_Transmitter); //����Ϊ����ģʽ
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));	//�ȴ��ӻ�Ӧ�𣬳ɹ�����Ϊ����ģʽ
	USART1_sendData(0x03);

	I2C_Cmd(I2C1,ENABLE);		//???

	I2C_SendData(I2C1,0x01);    //���Ͳ�����ַ
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	USART1_sendData(0x04);

	I2C_GenerateSTART(I2C1,ENABLE);   //��������
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	USART1_sendData(0x05);

	I2C_Send7bitAddress(I2C1,0xA0,I2C_Direction_Receiver); //����Ϊ����ģʽ
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));	//�ȴ��ӻ�Ӧ�𣬳ɹ�����Ϊ����ģʽ
	USART1_sendData(0x06);

	I2C_AcknowledgeConfig(I2C1,DISABLE);    //���ü������գ���Ӧ��

	

	I2C_GenerateSTOP(I2C1,ENABLE);			 //���ͽ����ź�

	data = I2C_ReceiveData(I2C1);			 //������
		 // USART1_sendData(data);

	I2C_AcknowledgeConfig(I2C1,ENABLE);
	return data;
}





