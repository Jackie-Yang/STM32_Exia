#ifndef __QUADROTOR_STATE_H__
#define __QUADROTOR_STATE_H__

#include <stdint.h>
//不进行内存对齐
#pragma  pack (push,1)
typedef struct __QUADROTOR_STATE__
{
	uint16_t DataHead;
	uint16_t DataSize;

	uint16_t Rudd;
	uint16_t Thro;
	uint16_t Aile;
	uint16_t Elev;
	uint16_t Check_Data;
	uint16_t Check_State;
	uint16_t Temp1;
	uint16_t Temp2;
	uint16_t Temp3;
	uint16_t Temp4;
	int16_t Accel_X;
	int16_t Accel_Y;
	int16_t Accel_Z;
	int16_t Gyro_X;
	int16_t Gyro_Y;
	int16_t Gyro_Z;
	uint16_t MPU6050_Temp;
	int32_t MS5611_Temp;
	int32_t MS5611_Press;
	int32_t MS5611_HIGH;
	int16_t HMC5883L_X;
	int16_t HMC5883L_Y;
	int16_t HMC5883L_Z;
	int16_t HMC5883L_Angle;
	uint16_t KS10X_High;
	uint16_t High_Accel;
	int16_t Roll;
	int16_t Pitch;
	int16_t Yaw;
	uint16_t Motor1;
	uint16_t Motor2;
	uint16_t Motor3;
	uint16_t Motor4;

	uint16_t DataCheckValue;
	uint16_t DataEnd;
} Quadrotor_State, *p_Quadrotor_State;
#pragma pack(pop) 

#endif
