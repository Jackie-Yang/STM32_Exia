#ifndef DMA_H
#define DMA_H
#include "stm32f10x.h"

// #define DMA_BUFF_SIZE 36	   //需要传输的数据字节数

#define USART1_DR_Base 0x40013804	   //外设（串口）的数据地址

/********************一些参数在缓冲数组中存放的位置***********************/
//接收器的数据为0~3，因为直接利用循环中的index作参数，因此不宏定义了

// #define RUDD_INDEX 0
// #define THRO_INDEX 1
// #define ALIE_INDEX 2
// #define ELEV_INDEX 3

// #define CHECK_DATA_INDEX 4
// #define CHECK_STATE_INDEX 5

// #define TEMP1_INDEX 6	  //临时数据，用于曲线临时监控PID输出
// #define TEMP2_INDEX 7
// #define TEMP3_INDEX 8
// #define TEMP4_INDEX 9


// #define ACCEL_X_INDEX 10
// //#define ACCEL_OFFSET_X_INDEX 5
// #define ACCEL_Y_INDEX 11
// //#define ACCEL_OFFSET_Y_INDEX 7
// #define ACCEL_Z_INDEX 12
// //#define ACCEL_OFFSET_Z_INDEX 9
// #define GYRO_X_INDEX 13
// //#define GYRO_OFFSET_X_INDEX 11
// #define GYRO_Y_INDEX 14
// //#define GYRO_OFFSET_Y_INDEX 13
// #define GYRO_Z_INDEX 15
// //#define GYRO_OFFSET_Z_INDEX 15
// #define MPU6050_TEMP_INDEX 16
// #define MS5611_TEMP_INDEX 17
// #define MS5611_PRESS_INDEX 19
// #define MS5611_HIGH_INDEX 21
// #define HMC5883L_X_INDEX 23
// #define HMC5883L_Y_INDEX 24
// #define HMC5883L_Z_INDEX 25
// #define HMC5883L_ANGLE_INDEX 26
// //#define HMC5883L_X_offset_INDEX 27
// //#define HMC5883L_Y_offse_INDEX 28
// //#define HMC5883L_Z_offse_INDEX 29	

// #define KS10X_HIGH_INDEX 27

// #define	HIGH_ACCEL_INDEX 28

// #define ROLL_INDEX 29
// #define PITCH_INDEX 30
// #define YAW_INDEX 31

// #define MOTOR_1_INDEX 32
// #define MOTOR_2_INDEX 33
// #define MOTOR_3_INDEX 34
// #define MOTOR_4_INDEX 35



// //




// //#define ROLL_GYRO_KP_INDEX 33
// //#define ROLL_GYRO_KI_INDEX 34
// //#define ROLL_GYRO_KD_INDEX 35
// //#define ROLL_ANGLE_KP_INDEX 36
// //#define ROLL_ANGLE_KI_INDEX 37
// //#define ROLL_ANGLE_KD_INDEX 38
// //
// //#define PITCH_GYRO_KP_INDEX 39
// //#define PITCH_GYRO_KI_INDEX 40
// //#define PITCH_GYRO_KD_INDEX 41
// //#define PITCH_ANGLE_KP_INDEX 42
// //#define PITCH_ANGLE_KI_INDEX 43
// //#define PITCH_ANGLE_KD_INDEX 44
// //
// //#define YAW_GYRO_KP_INDEX 49
// //#define YAW_GYRO_KI_INDEX 50
// //#define YAW_GYRO_KD_INDEX 51




 
// extern u16 DMA_Buff[ ];		   				  //DMA缓存数组

void DMA_Configuration(void * DMA_Buff_Addr, uint32_t DMA_BufferSize);				  //配置DMA，数据由内存传输到串口
// void DMA_Buff_In_16(u16 data,u8 index);		  //16位数据存入缓存数组
// void DMA_Buff_In_32(u32 data,u8 index);		  //32位数据存入缓存数组

#endif
