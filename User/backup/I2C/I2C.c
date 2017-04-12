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
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));		 //等待总线空闲

	I2C_GenerateSTART(I2C1,ENABLE);						 //发送开始信号
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));	//等待成功控制总线

	I2C_Send7bitAddress(I2C1,0xA0,I2C_Direction_Transmitter); //设置为发送模式
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));	//等待从机应答，成功设置为发送模式


	I2C_SendData(I2C1,0x01);    //发送操作地址
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(I2C1,0x01);    //发送数据
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));


	I2C_GenerateSTOP(I2C1,ENABLE);			 //发送结束信号
}

u8 I2C_read(void)
{
	u8 data;
//		USART1_sendData(0xFF);

	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));		 //等待总线空闲

//	USART1_sendData(0x01);

	I2C_GenerateSTART(I2C1,ENABLE);						 //发送开始信号
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));	//等待成功控制总线
	USART1_sendData(0x02);

	I2C_Send7bitAddress(I2C1,0xA0,I2C_Direction_Transmitter); //设置为发送模式
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));	//等待从机应答，成功设置为发送模式
	USART1_sendData(0x03);

	I2C_Cmd(I2C1,ENABLE);		//???

	I2C_SendData(I2C1,0x01);    //发送操作地址
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	USART1_sendData(0x04);

	I2C_GenerateSTART(I2C1,ENABLE);   //重启总线
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	USART1_sendData(0x05);

	I2C_Send7bitAddress(I2C1,0xA0,I2C_Direction_Receiver); //设置为接收模式
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));	//等待从机应答，成功设置为接收模式
	USART1_sendData(0x06);

	I2C_AcknowledgeConfig(I2C1,DISABLE);    //不用继续接收，不应答

	

	I2C_GenerateSTOP(I2C1,ENABLE);			 //发送结束信号

	data = I2C_ReceiveData(I2C1);			 //存数据
		 // USART1_sendData(data);

	I2C_AcknowledgeConfig(I2C1,ENABLE);
	return data;
}





