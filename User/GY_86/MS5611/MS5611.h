#ifndef MS5611_H
#define MS5611_H

#include "stm32f10x.h"
	

/**********************�궨��*************************/

//����������IIC�����еĴӵ�ַ,����CSB���Ų�ͬ�޸�
//#define MS561101BA_ADDR  0xec   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS561101BA_ADDR   0xee   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

// ����MS561101BA�ڲ���ַ
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




/********************��������**************************/
int8_t MS5611_Init(void);        //оƬ��ʼ������ȡоƬ��PROM����


//��Ҫ�򴫸������Ͷ�ȡ�¶�����Լ8ms���ȡ���ݣ�ͬ����ʽ�����ѹ��������������
int8_t MS5611_AskTemperature(void);
int8_t MS5611_GetTemperature(void);
int8_t MS5611_AskPressure(void);
int8_t MS5611_GetPressure(void);

//���´β����ĸ߶���Ϊ�ݿ��߶�
void MS5611_SetReference(void);
//������ѹ��������������Ը߶�
void MS5611_CalResult(__packed float *Press, __packed float *Temp, __packed float *High);

#endif 
