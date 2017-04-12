#ifndef IMU_H
#define IMU_H

//void IMUupdata(float gx, float gy, float gz, float ax, float ay, float az);


void init_quaternion(void);
void AHRSupdate(void);

float invSqrt(float x);


#endif


