#include "MPU6050.h"
#include "Setup.h"
#include "eeprom.h"
#include "USART.h"
#include "I2C.h"
#include "delay.h"
#include "math.h"

/********************全局变量*****************************/
int16_t MPU6050_Gyro_X = 0,MPU6050_Gyro_Y = 0,MPU6050_Gyro_Z = 0;
int16_t Gyro_offset_X = 0,Gyro_offset_Y = 0,Gyro_offset_Z = 0;
int16_t MPU6050_Accel_X = 0,MPU6050_Accel_Y = 0,MPU6050_Accel_Z = 0;
int16_t Accel_offset_X = 0,Accel_offset_Y = 0,Accel_offset_Z = 0;

float MPU6050_Temperature = 0;



/************************************************************   
* 函数名:Init_MPU6050   
* 描述 ：初始化MPU6050，根据需要请参考pdf进行修改
* 输入 :无   
* 输出 :无    
*/
void MPU6050_Init(void)
{

	I2C_SendByte(MPU6050_Addr,PWR_MGMT_1, 0x00);	//解除休眠状态
	
	
	I2C_SendByte(MPU6050_Addr,SMPLRT_DIV, 0x07);    //陀螺仪采样率
	I2C_SendByte(MPU6050_Addr,CONFIG, 0x06);        //5Hz 
	
	I2C_SendByte(MPU6050_Addr,INT_PIN_CFG, 0x42);   //使能旁路I2C
	I2C_SendByte(MPU6050_Addr,USER_CTRL, 0x40);     //使能旁路I2C
	
	I2C_SendByte(MPU6050_Addr,GYRO_CONFIG, 0x00);   //陀螺仪自检及测量范围0x1B，：0x18(不自检，250deg/s)
	
	I2C_SendByte(MPU6050_Addr,ACCEL_CONFIG, 0x01); //加速计自检测量范围及高通滤波频率0x1C，0x01(不自检，2G，5Hz)

	 //从Flash读取之前的校正数据
	EE_ReadVariable(OFFSET_AX_ADDR, (uint16_t*)&Accel_offset_X);
	EE_ReadVariable(OFFSET_AY_ADDR, (uint16_t*)&Accel_offset_Y);
	EE_ReadVariable(OFFSET_AZ_ADDR, (uint16_t*)&Accel_offset_Z);
	EE_ReadVariable(OFFSET_GX_ADDR, (uint16_t*)&Gyro_offset_X);
	EE_ReadVariable(OFFSET_GY_ADDR, (uint16_t*)&Gyro_offset_Y);
	EE_ReadVariable(OFFSET_GZ_ADDR, (uint16_t*)&Gyro_offset_Z);
	//将校正数据发送到上位机
//	DMA_Buff_In_16(Accel_offset_X,ACCEL_OFFSET_X_INDEX);
//	DMA_Buff_In_16(Accel_offset_Y,ACCEL_OFFSET_Y_INDEX);
//	DMA_Buff_In_16(Accel_offset_Z,ACCEL_OFFSET_Z_INDEX);
//	DMA_Buff_In_16(Gyro_offset_X,GYRO_OFFSET_X_INDEX);
//	DMA_Buff_In_16(Gyro_offset_Y,GYRO_OFFSET_Y_INDEX);
//	DMA_Buff_In_16(Gyro_offset_Z,GYRO_OFFSET_Z_INDEX);
	
}

/************************************************************   
* 函数名:MP6050_WHO_AM_I   
* 描述 ：读取MPU6050设备信息
* 输入 :无   
* 输出 :无    
*/

void MPU6050_WHO_AM_I(void)
{
	u8 data = 0;
	I2C_ReadByte(MPU6050_Addr, WHO_AM_I, &data);
}





/************************************************************   
* 函数名:READ_MPU6050_Accel   
* 描述 : 读取MPU6050加速度数据，并发送到上位机
* 输入  :无   
* 输出  :无  
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
* 函数名:READ_MPU6050_Gyro   
* 描述 : 读取MPU6050陀螺仪数据，并发送到上位机
* 输入  :无   
* 输出  :无    
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
* 函数名:MPU6050_SetOffset   
* 描述 : 对MPU6050加速度传感器，陀螺仪进行0偏校正，在水平位址时取200次值作平均
* 输入  :无   
* 输出  :无    
*/

void MPU6050_SetOffset(void)
{
	int16_t data;
	int32_t text_Ax = 0, text_Ay = 0, text_Az = 0;
	int32_t text_Gx = 0, text_Gy = 0, text_Gz = 0;
	u16 i;
	for (i = 0; i < 200; i++)
	{ //强制转换成short int类型，如果是负数，再转成32位时会自动补上符号
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

	
	 EE_WriteVariable(OFFSET_AX_ADDR, Accel_offset_X);	   //将偏移量储存进Flash，方便下次开机读取
	 EE_WriteVariable(OFFSET_AY_ADDR, Accel_offset_Y);
	 EE_WriteVariable(OFFSET_AZ_ADDR, Accel_offset_Z);
	 EE_WriteVariable(OFFSET_GX_ADDR, Gyro_offset_X);
	 EE_WriteVariable(OFFSET_GY_ADDR, Gyro_offset_Y);
	 EE_WriteVariable(OFFSET_GZ_ADDR, Gyro_offset_Z);

	 //将校正数据发送到上位机
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
	stQuadrotor_State.s16_MPU6050_Temp = MPU6050_Temperature;		   //温度暂时没用，计算温度暂时交给上位机
	stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
}




