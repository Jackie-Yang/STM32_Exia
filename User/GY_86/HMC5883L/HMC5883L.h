#ifndef HMC5883L_H
#define HMC5883L_H

/*包含头------------------------------------------------------------------*/
#include "stm32f10x.h"



/**************宏定义**************************/
#define	HMC5883L_Addr                    0x3C	//磁场传感器器件地址   
#define HMC5883L_ConfigA                 0x00
//采样平均数
#define CONFIGA_AVERAGE_1                0x00
#define CONFIGA_AVERAGE_2                0x20
#define CONFIGA_AVERAGE_4                0x40
#define CONFIGA_AVERAGE_8                0x60
//采样频率
#define CONFIGA_RATE_0_75                0x00
#define CONFIGA_RATE_1_5                 0x04
#define CONFIGA_RATE_3                   0x08
#define CONFIGA_RATE_7_5                 0x0C
#define CONFIGA_RATE_15                  0x10
#define CONFIGA_RATE_30                  0x14
#define CONFIGA_RATE_75                  0x18
#define CONFIGA_RATE_0                   0x1C
//测量配置：正常，自测（正偏置、负偏置）
#define CONFIGA_BIAS_NORMAL              0x00
#define CONFIGA_BIAS_POSITIVE            0x01
#define CONFIGA_BIAS_NEGATIVE            0x02

#define HMC5883L_ConfigB                 0x01
#define CONFIGB_GAIN_1370                0x00
#define CONFIGB_GAIN_1090                0x20
#define CONFIGB_GAIN_820                 0x40
#define CONFIGB_GAIN_660                 0x60
#define CONFIGB_GAIN_440                 0x80
#define CONFIGB_GAIN_390                 0xA0
#define CONFIGB_GAIN_330                 0xC0
#define CONFIGB_GAIN_230                 0xE0

//测量模式：连续、单次、空闲
#define HMC5883L_Mode                    0x02
#define MODE_CONTINUOUS                  0x00
#define MODE_SINGLE                      0x01
#define MODE_IDLE                        0x02

#define HMC5883L_Output_X_MSB            0x03
#define HMC5883L_Output_X_LSB 			 0x04
#define HMC5883L_Output_Z_MSB            0x05
#define HMC5883L_Output_Z_LSB 			 0x06
#define HMC5883L_Output_Y_MSB            0x07
#define HMC5883L_Output_Y_LSB 			 0x08
#define HMC5883L_STATE                   0x09
#define HMC5883L_ID_A                    0x0a
#define HMC5883L_ID_B                    0x0b
#define HMC5883L_ID_C                    0x0c


int8_t HMC5883L_Init(void);			//模块初始化，配置相关寄存器
void Read_HMC5883L(int16_t *Mag, __packed float *MagAngle); //读取磁场数据

#endif
