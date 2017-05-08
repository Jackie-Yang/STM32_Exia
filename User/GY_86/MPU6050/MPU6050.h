#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f10x.h"

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



//extern u8 DMA_Buff [];

extern int16_t MPU6050_Accel_X,MPU6050_Accel_Y,MPU6050_Accel_Z;
extern int16_t Accel_offset_X,Accel_offset_Y,Accel_offset_Z; 

extern int16_t MPU6050_Gyro_X,MPU6050_Gyro_Y,MPU6050_Gyro_Z;
extern int16_t Gyro_offset_X,Gyro_offset_Y,Gyro_offset_Z;

extern float MPU6050_Temperature;


void MPU6050_Init(void); 			//MPU6050初始化配置参数
void MPU6050_WHO_AM_I(void);		//获取MPU6050识别码检测(即I2C地址)

void READ_MPU6050_Accel(void);		//读取加速度传感器数据
void READ_MPU6050_Gyro(void);		//读取陀螺仪数据
void MPU6050_SetOffset(void);		//对传感器进行零偏校正
void READ_MPU6050_TEMP(void);		//读取传感器温度

void MPU6050_DMP_Init(void);
void MPU6050_DMP_SelfTest(void);
void Read_MPU6050_DMP(void);

#endif

