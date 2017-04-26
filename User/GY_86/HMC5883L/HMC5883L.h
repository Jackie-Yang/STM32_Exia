#ifndef HMC5883L_H
#define HMC5883L_H

/*包含头------------------------------------------------------------------*/
#include "stm32f10x.h"



/**************宏定义**************************/
#define	HMC5883L_Addr   0x3C	//磁场传感器器件地址   
#define HMC5883L_ConfigurationRegisterA  0x00
#define HMC5883L_ConfigurationRegisterB  0x01
#define HMC5883L_ModeRegister            0x02
#define HMC5883L_Output_X_MSB            0x03
#define HMC5883L_Output_X_LSB 			 0x04
#define HMC5883L_Output_Z_MSB            0x05
#define HMC5883L_Output_Z_LSB 			 0x06
#define HMC5883L_Output_Y_MSB            0x07
#define HMC5883L_Output_Y_LSB 			 0x08



extern int16_t HMC5883L_X,HMC5883L_Y,HMC5883L_Z;
extern int16_t HMC5883L_X_offset,HMC5883L_Y_offset,HMC5883L_Z_offset;
extern float HMC5883L_angle;

void HMC5883L_Init(void);			//模块初始化，配置相关寄存器
void Read_HMC5883L(void);			//读取磁场数据
void HMC5883L_SetOffset(void);		//做简单的校正



#endif
