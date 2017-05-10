#ifndef IMU_H
#define IMU_H
#include "stm32f10x.h"

//void IMUupdata(float gx, float gy, float gz, float ax, float ay, float az);
int8_t IMU_Init(void);
int8_t GetAttitude(int16_t *Accel, int16_t *Gyro, int16_t *Mag, __packed float *MagAngle, __packed float *Roll, __packed float *Pitch, __packed float *Yaw);



#endif


