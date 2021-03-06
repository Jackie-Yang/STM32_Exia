#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f10x.h"

#define DMP_ENABLE		1

/**********************宏定义*************************/

#define	MPU6050_Addr   0xD0	  //定义器件在IIC总线中的从地址(已经左移一位)  

/*******************定义MPU6050内部地址*********************/
#define	SMPLRT_DIV		25		//陀螺仪采样率0x19，典型值：0x07(125Hz)
#define	CONFIG			26		//低通滤波频率0x1A，典型值：0x06(5Hz)
#define	GYRO_CONFIG		27		//陀螺仪自检及测量范围0x1B，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	28		//加速计自检测量范围及高通滤波频率0x1C，典型值：0x01(不自检，2G，5Hz)

#define INT_PIN_CFG     0x37    //设置旁路有效 打开值：0x42 AUX_DA的辅助I2C
#define USER_CTRL       0x6A    //用户配置寄存器 打开值：0x40  AUX_DA的辅助I2C

#define	ACCEL_XOUT_H	0x3B    // 存储最近的X轴、Y轴、Z轴加速度感应器的测量值
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41	// 存储的最近温度传感器的测量值 
#define	TEMP_OUT_L		0x42
								 // 存储最近的X轴、Y轴、Z轴陀螺仪感应器的测量值 */
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		107		//电源管理0x6B，典型值：0x00(正常启用)
#define	WHO_AM_I		117		//I2C地址寄存器0x75(默认数值0x68，只读)


int8_t MPU6050_Init(void); 			//MPU6050初始化配置参数

#if DMP_ENABLE
int8_t MPU6050_DMP_SelfTest(void);
int8_t Read_MPU6050_DMP(int16_t *Accel, int16_t *Gyro, __packed float *Roll, __packed float *Pitch, __packed float *Yaw);
#else
int8_t READ_MPU6050_Accel(int16_t *Accel); //读取加速度传感器数据
int8_t READ_MPU6050_Gyro(int16_t *Gyro);   //读取陀螺仪数据
void MPU6050_SetOffset(void);				   //对传感器进行零偏校正
#endif

int8_t READ_MPU6050_TEMP(__packed float *pTemp); //读取传感器温度
void MPU6050_WHO_AM_I(void); //获取MPU6050识别码检测(即I2C地址)

#endif

