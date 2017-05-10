#ifndef MS5611_H
#define MS5611_H

#include "stm32f10x.h"
	

/**********************宏定义*************************/

//定义器件在IIC总线中的从地址,根据CSB引脚不同修改
//#define MS561101BA_ADDR  0xec   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS561101BA_ADDR   0xee   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

// 定义MS561101BA内部地址
// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08
//#define  MS561101BA_D1_OSR_256 0x40 
//#define  MS561101BA_D1_OSR_512 0x42 
//#define  MS561101BA_D1_OSR_1024 0x44 
//#define  MS561101BA_D1_OSR_2048 0x46 
#define  MS561101BA_D1_OSR_4096 0x48 

//#define  MS561101BA_D2_OSR_256 0x50 
//#define  MS561101BA_D2_OSR_512 0x52 
//#define  MS561101BA_D2_OSR_1024 0x54 
#define  MS561101BA_D2_OSR_2048 0x56 
#define  MS561101BA_D2_OSR_4096 0x58 

#define MS561101BA_PROM_BASE_ADDR 0xA0 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.




/********************函数声明**************************/
int8_t MS5611_Init(void);        //芯片初始化，读取芯片中PROM数据


//需要向传感器发送读取温度请求，约8ms后获取数据，同样方式获得气压，接下来计算结果
int8_t MS5611_AskTemperature(void);
int8_t MS5611_GetTemperature(void);
int8_t MS5611_AskPressure(void);
int8_t MS5611_GetPressure(void);

//将下次测量的高度作为草靠高度
void MS5611_SetReference(void);
//计算气压结果，并换算成相对高度
void MS5611_CalResult(__packed float *Press, __packed float *Temp, __packed float *High);

#endif 
