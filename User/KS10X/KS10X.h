#ifndef KS10X_H
#define KS10X_H
#include "stm32f10x.h"

#define KS10X_ADDRESS 0xe8

extern u16 KS10X_high;

void KS10X_init(void);
void KS10X_command(u8 command);
u16 KS10X_Get_Result(void);

void KS10X_Get_High(void);

/***********为KS10X的I2C速度制定的I2C函数**************/
void KS10X_I2C_Start(void);
void KS10X_I2C_Stop(void);
u8 KS10X_I2C_Wait_Ack(void);
void KS10X_I2C_Ack(void);
void KS10X_I2C_NAck(void);
u8 KS10X_I2C_Send_Data(u8 data);
u8 KS10X_I2C_Read_Data(u8 ack);




#endif

