#ifndef __QUADROTOR_STATE_H__
#define __QUADROTOR_STATE_H__

#include <stdint.h>

#define DATA_HEAD1 	0xFF
#define DATA_HEAD2 	0x7F
#define DATA_END	0xFEFF

//不进行内存对齐
#pragma  pack (push,1)
typedef struct __QUADROTOR_STATE__
{
	uint8_t u8_DataHead1;
	uint8_t u8_DataHead2;
	uint16_t u16_DataSize;

	uint16_t u16_Rudd;
	uint16_t u16_Thro;
	uint16_t u16_Aile;
	uint16_t u16_Elev;
	uint16_t u16_Check_Data;
	uint16_t u16_Check_State;
	uint16_t u16_Temp1;
	uint16_t u16_Temp2;
	uint16_t u16_Temp3;
	uint16_t u16_Temp4;
	int16_t s16_Accel_X;
	int16_t s16_Accel_Y;
	int16_t s16_Accel_Z;
	int16_t s16_Gyro_X;
	int16_t s16_Gyro_Y;
	int16_t s16_Gyro_Z;
	int16_t s16_MPU6050_Temp;
	int32_t s32_MS5611_Temp;
	int32_t s32_MS5611_Press;
	int32_t s32_MS5611_HIGH;
	int16_t s16_HMC5883L_X;
	int16_t s16_HMC5883L_Y;
	int16_t s16_HMC5883L_Z;
	float f_HMC5883L_Angle;
	uint16_t u16_KS10X_High;
	float f_High_Accel;
	float f_Roll;
	float f_Pitch;
	float f_Yaw;
	uint16_t u16_Motor1;
	uint16_t u16_Motor2;
	uint16_t u16_Motor3;
	uint16_t u16_Motor4;

	uint8_t u8_ROLL_G_Kp;
	uint8_t u8_ROLL_G_Ki;
	uint8_t u8_ROLL_G_Kd;
	uint8_t u8_ROLL_Angle_Kp;
	uint8_t u8_ROLL_Angle_Ki;
	uint8_t u8_ROLL_Angle_Kd;

	uint8_t u8_PITCH_G_Kp;
	uint8_t u8_PITCH_G_Ki;
	uint8_t u8_PITCH_G_Kd;
	uint8_t u8_PITCH_Angle_Kp;
	uint8_t u8_PITCH_Angle_Ki;
	uint8_t u8_PITCH_Angle_Kd;

	uint8_t u8_YAW_G_Kp;
	uint8_t u8_YAW_G_Ki;
	uint8_t u8_YAW_G_Kd;

	uint16_t u16_DataCheckValue;
	uint16_t u16_DataEnd;
} Quadrotor_State, *p_Quadrotor_State;
#pragma pack(pop) 

#endif
