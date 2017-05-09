#ifndef IMU_H
#define IMU_H
#include "stm32f10x.h"

//void IMUupdata(float gx, float gy, float gz, float ax, float ay, float az);

void init_quaternion(int16_t *Accel, int16_t *Mag, __packed float *MagAngle);
void AHRSupdate(int16_t *Accel, int16_t *Gyro, int16_t *Mag, __packed float *MagAngle);

float invSqrt(float x);


#endif


