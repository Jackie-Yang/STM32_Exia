#ifndef __QUADROTOR_STATE_H__
#define __QUADROTOR_STATE_H__

#include <stdint.h>

#define DATA_HEAD1 0xFF
#define DATA_HEAD2 0x7F
#define DATA_END 0xFEFF

//不进行内存对齐
#pragma pack(push, 1)
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
	int16_t s16_Accel[3];
	int16_t s16_Gyro[3];
	float f_MPU6050_Temp;
	float f_MS5611_Temp;
	float f_MS5611_Press;
	float f_MS5611_HIGH;
	int16_t s16_HMC5883L[3];
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

	uint16_t u16_ROLL_G_Kp;
	uint16_t u16_ROLL_G_Ki;
	uint16_t u16_ROLL_G_Kd;
	uint16_t u16_ROLL_Angle_Kp;
	uint16_t u16_ROLL_Angle_Ki;
	uint16_t u16_ROLL_Angle_Kd;

	uint16_t u16_PITCH_G_Kp;
	uint16_t u16_PITCH_G_Ki;
	uint16_t u16_PITCH_G_Kd;
	uint16_t u16_PITCH_Angle_Kp;
	uint16_t u16_PITCH_Angle_Ki;
	uint16_t u16_PITCH_Angle_Kd;

	uint16_t u16_YAW_G_Kp;
	uint16_t u16_YAW_G_Ki;
	uint16_t u16_YAW_G_Kd;

	uint16_t u16_DataCheckValue;
	uint16_t u16_DataEnd;
} Quadrotor_State, *p_Quadrotor_State;
#pragma pack(pop)

enum
{
	COMMAND_MPU6050_SETOFFSET = 0, //MPU6050设置校正，初始化四元数（非DMP下使用）
	COMMAND_SET_HIGH_REF,		   //设置参考高度
	COMMAND_SET_THRO,			   //设置油门
	COMMAND_SET_RUDD,			   //设置方向舵
	COMMAND_SET_ELEV,			   //设置升降舵
	COMMAND_SET_AILE,			   //设置副翼
	COMMAND_STOP,				   //紧急停止
	COMMAND_HORIZON,			   //中位姿态
	COMMAND_ROLL_GYRO_KP,		   //设置PID参数
	COMMAND_ROLL_GYRO_KI,
	COMMAND_ROLL_GYRO_KD,
	COMMAND_ROLL_ANGLE_KP,
	COMMAND_ROLL_ANGLE_KI,
	COMMAND_ROLL_ANGLE_KD,
	COMMAND_PITCH_GYRO_KP,
	COMMAND_PITCH_GYRO_KI,
	COMMAND_PITCH_GYRO_KD,
	COMMAND_PITCH_ANGLE_KP,
	COMMAND_PITCH_ANGLE_KI,
	COMMAND_PITCH_ANGLE_KD,
	COMMAND_YAW_GYRO_KP,
	COMMAND_YAW_GYRO_KI,
	COMMAND_YAW_GYRO_KD,
};

#endif
