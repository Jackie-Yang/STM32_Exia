#include "MPU6050.h"
#include "Setup.h"
#include "eeprom.h"
#include "USART.h"
#include "I2C.h"
#include "delay.h"
#include "math.h"

/********************ȫ�ֱ���*****************************/
int16_t MPU6050_Gyro_X = 0,MPU6050_Gyro_Y = 0,MPU6050_Gyro_Z = 0;
int16_t Gyro_offset_X = 0,Gyro_offset_Y = 0,Gyro_offset_Z = 0;
int16_t MPU6050_Accel_X = 0,MPU6050_Accel_Y = 0,MPU6050_Accel_Z = 0;
int16_t Accel_offset_X = 0,Accel_offset_Y = 0,Accel_offset_Z = 0;

float MPU6050_Temperature = 0;



/************************************************************   
* ������:Init_MPU6050   
* ���� ����ʼ��MPU6050��������Ҫ��ο�pdf�����޸�
* ���� :��   
* ��� :��    
*/
void MPU6050_Init(void)
{

	I2C_SendByte(MPU6050_Addr,PWR_MGMT_1, 0x00);	//�������״̬
	
	
	I2C_SendByte(MPU6050_Addr,SMPLRT_DIV, 0x07);    //�����ǲ�����
	I2C_SendByte(MPU6050_Addr,CONFIG, 0x06);        //5Hz 
	
	I2C_SendByte(MPU6050_Addr,INT_PIN_CFG, 0x42);   //ʹ����·I2C
	I2C_SendByte(MPU6050_Addr,USER_CTRL, 0x40);     //ʹ����·I2C
	
	I2C_SendByte(MPU6050_Addr,GYRO_CONFIG, 0x00);   //�������Լ켰������Χ0x1B����0x18(���Լ죬250deg/s)
	
	I2C_SendByte(MPU6050_Addr,ACCEL_CONFIG, 0x01); //���ټ��Լ������Χ����ͨ�˲�Ƶ��0x1C��0x01(���Լ죬2G��5Hz)

	 //��Flash��ȡ֮ǰ��У������
	EE_ReadVariable(OFFSET_AX_ADDR, (uint16_t*)&Accel_offset_X);
	EE_ReadVariable(OFFSET_AY_ADDR, (uint16_t*)&Accel_offset_Y);
	EE_ReadVariable(OFFSET_AZ_ADDR, (uint16_t*)&Accel_offset_Z);
	EE_ReadVariable(OFFSET_GX_ADDR, (uint16_t*)&Gyro_offset_X);
	EE_ReadVariable(OFFSET_GY_ADDR, (uint16_t*)&Gyro_offset_Y);
	EE_ReadVariable(OFFSET_GZ_ADDR, (uint16_t*)&Gyro_offset_Z);
	//��У�����ݷ��͵���λ��
//	DMA_Buff_In_16(Accel_offset_X,ACCEL_OFFSET_X_INDEX);
//	DMA_Buff_In_16(Accel_offset_Y,ACCEL_OFFSET_Y_INDEX);
//	DMA_Buff_In_16(Accel_offset_Z,ACCEL_OFFSET_Z_INDEX);
//	DMA_Buff_In_16(Gyro_offset_X,GYRO_OFFSET_X_INDEX);
//	DMA_Buff_In_16(Gyro_offset_Y,GYRO_OFFSET_Y_INDEX);
//	DMA_Buff_In_16(Gyro_offset_Z,GYRO_OFFSET_Z_INDEX);
	
}

/************************************************************   
* ������:MP6050_WHO_AM_I   
* ���� ����ȡMPU6050�豸��Ϣ
* ���� :��   
* ��� :��    
*/

void MPU6050_WHO_AM_I(void)
{
	u8 data = 0;
	I2C_ReadByte(MPU6050_Addr, WHO_AM_I, &data);
}





/************************************************************   
* ������:READ_MPU6050_Accel   
* ���� : ��ȡMPU6050���ٶ����ݣ������͵���λ��
* ����  :��   
* ���  :��  
**************************/



void READ_MPU6050_Accel(void)
{

	I2C_ReadBytes_BE(MPU6050_Addr, ACCEL_XOUT_H, sizeof(MPU6050_Accel_X), (uint8_t *)&MPU6050_Accel_X);
	if( (int32_t)MPU6050_Accel_X + Accel_offset_X <= 32768  &&  (int32_t)MPU6050_Accel_X + Accel_offset_X >= -32768)
	{
		MPU6050_Accel_X = MPU6050_Accel_X + Accel_offset_X;
	}
	stQuadrotor_State.s16_Accel_X = MPU6050_Accel_X;

	I2C_ReadBytes_BE(MPU6050_Addr, ACCEL_YOUT_H, sizeof(MPU6050_Accel_Y), (uint8_t *)&MPU6050_Accel_Y);
	if( (int32_t)MPU6050_Accel_Y + Accel_offset_Y <= 32768  &&  (int32_t)MPU6050_Accel_Y + Accel_offset_Y >= -32768)
	{
		MPU6050_Accel_Y = MPU6050_Accel_Y + Accel_offset_Y;
	}
	stQuadrotor_State.s16_Accel_Y = MPU6050_Accel_Y;

	I2C_ReadBytes_BE(MPU6050_Addr, ACCEL_ZOUT_H, sizeof(MPU6050_Accel_Z), (uint8_t *)&MPU6050_Accel_Z);
	if( (int32_t)MPU6050_Accel_Z + Accel_offset_Z <= 32768  &&  (int32_t)MPU6050_Accel_Z + Accel_offset_Z >= -32768)
	{
		MPU6050_Accel_Z = MPU6050_Accel_Z + Accel_offset_Z;
	}
	stQuadrotor_State.s16_Accel_Z = MPU6050_Accel_Z;
	stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
}



/************************************************************   
* ������:READ_MPU6050_Gyro   
* ���� : ��ȡMPU6050���������ݣ������͵���λ��
* ����  :��   
* ���  :��    
*/

void READ_MPU6050_Gyro(void)
{
	I2C_ReadBytes_BE(MPU6050_Addr, GYRO_XOUT_H, sizeof(MPU6050_Gyro_X), (uint8_t *)&MPU6050_Gyro_X);
	if( (int32_t)MPU6050_Gyro_X + Gyro_offset_X <= 32768  &&  (int32_t)MPU6050_Gyro_X + Gyro_offset_X >= -32768)
	{
		MPU6050_Gyro_X = MPU6050_Gyro_X + Gyro_offset_X;
	}
	stQuadrotor_State.s16_Gyro_X = MPU6050_Gyro_X;

	I2C_ReadBytes_BE(MPU6050_Addr, GYRO_YOUT_H, sizeof(MPU6050_Gyro_Y), (uint8_t *)&MPU6050_Gyro_Y);
	if( (int32_t)MPU6050_Gyro_Y + Gyro_offset_Y <= 32768  &&  (int32_t)MPU6050_Gyro_Y + Gyro_offset_Y >= -32768)
	{
		MPU6050_Gyro_Y = MPU6050_Gyro_Y + Gyro_offset_Y;
	}
	stQuadrotor_State.s16_Gyro_Y = MPU6050_Gyro_Y;

	I2C_ReadBytes_BE(MPU6050_Addr, GYRO_ZOUT_H, sizeof(MPU6050_Gyro_Z), (uint8_t *)&MPU6050_Gyro_Z);
	if( (int32_t)MPU6050_Gyro_Z + Gyro_offset_Z <= 32768  &&  (int32_t)MPU6050_Gyro_Z + Gyro_offset_Z >= -32768)
	{
		MPU6050_Gyro_Z = MPU6050_Gyro_Z + Gyro_offset_Z;
	}
	stQuadrotor_State.s16_Gyro_Z = MPU6050_Gyro_Z;
	stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
}


 /************************************************************   
* ������:MPU6050_SetOffset   
* ���� : ��MPU6050���ٶȴ������������ǽ���0ƫУ������ˮƽλַʱȡ200��ֵ��ƽ��
* ����  :��   
* ���  :��    
*/

void MPU6050_SetOffset(void)
{
	int16_t data;
	int32_t text_Ax = 0, text_Ay = 0, text_Az = 0;
	int32_t text_Gx = 0, text_Gy = 0, text_Gz = 0;
	u16 i;
	for (i = 0; i < 200; i++)
	{ //ǿ��ת����short int���ͣ�����Ǹ�������ת��32λʱ���Զ����Ϸ���
		I2C_ReadBytes_BE(MPU6050_Addr, ACCEL_XOUT_H, sizeof(data), (uint8_t *)&data);
		text_Ax += (short int)data;
		//delay_ms(1);
		I2C_ReadBytes_BE(MPU6050_Addr, ACCEL_YOUT_H, sizeof(data), (uint8_t *)&data);
		text_Ay += (short int)data;
		//delay_ms(1);
		I2C_ReadBytes_BE(MPU6050_Addr, ACCEL_ZOUT_H, sizeof(data), (uint8_t *)&data);
		text_Az += (short int)(data - 16384);
		//delay_ms(1);
		I2C_ReadBytes_BE(MPU6050_Addr, GYRO_XOUT_H, sizeof(data), (uint8_t *)&data);
		text_Gx += (short int)data;
		//delay_ms(1);
		I2C_ReadBytes_BE(MPU6050_Addr, GYRO_YOUT_H, sizeof(data), (uint8_t *)&data);
		text_Gy += (short int)data;
		//delay_ms(1);
		I2C_ReadBytes_BE(MPU6050_Addr, GYRO_ZOUT_H, sizeof(data), (uint8_t *)&data);
		text_Gz += (short int)data;
		//delay_ms(1);
	 }
	 Accel_offset_X = -(text_Ax / 200);
	 Accel_offset_Y = -(text_Ay / 200);
	 Accel_offset_Z = -(text_Az / 200);

	 Gyro_offset_X = -(text_Gx / 200);
	 Gyro_offset_Y = -(text_Gy / 200);
	 Gyro_offset_Z = -(text_Gz / 200);

	
	 EE_WriteVariable(OFFSET_AX_ADDR, Accel_offset_X);	   //��ƫ���������Flash�������´ο�����ȡ
	 EE_WriteVariable(OFFSET_AY_ADDR, Accel_offset_Y);
	 EE_WriteVariable(OFFSET_AZ_ADDR, Accel_offset_Z);
	 EE_WriteVariable(OFFSET_GX_ADDR, Gyro_offset_X);
	 EE_WriteVariable(OFFSET_GY_ADDR, Gyro_offset_Y);
	 EE_WriteVariable(OFFSET_GZ_ADDR, Gyro_offset_Z);

	 //��У�����ݷ��͵���λ��
//	DMA_Buff_In_16(Accel_offset_X,ACCEL_OFFSET_X_INDEX);
//	DMA_Buff_In_16(Accel_offset_Y,ACCEL_OFFSET_Y_INDEX);
//	DMA_Buff_In_16(Accel_offset_Z,ACCEL_OFFSET_Z_INDEX);
//	DMA_Buff_In_16(Gyro_offset_X,GYRO_OFFSET_X_INDEX);
//	DMA_Buff_In_16(Gyro_offset_Y,GYRO_OFFSET_Y_INDEX);
//	DMA_Buff_In_16(Gyro_offset_Z,GYRO_OFFSET_Z_INDEX);

}

void READ_MPU6050_TEMP(void)
{
	I2C_ReadBytes_BE(MPU6050_Addr, TEMP_OUT_H, sizeof(MPU6050_Temperature), (uint8_t *)&MPU6050_Temperature);
	//MPU6050_Temperature = MPU6050_Temperature / 340.0 + 36.53
	stQuadrotor_State.s16_MPU6050_Temp = MPU6050_Temperature;		   //�¶���ʱû�ã������¶���ʱ������λ��
	stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
}




