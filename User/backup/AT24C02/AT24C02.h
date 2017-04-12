#ifndef AT24C02_H
#define AT24C02_H 

#include "I2C.h"
 

#define ADDRESS_24C02	0xA0
#define WRITE           0x00
#define READ            0x01 
 
void AT24C02_WriteOneByte(u8 address,u8 data);		         //写入一个字节					  
u8 AT24C02_ReadOneByte(u8 address);							 //读取一个字节

void AT24C02_WriteBytes(u8 address,u32 data,u8 dataSize);    //写入指定长度的数据
u32 AT24C02_ReadBytes(u8 address,u8 dataSize);			     //读取指定长度数据

void AT24C02_Write(u8 address,u8 *data,u16 dataNum);	     //写入指定数目的数据
void AT24C02_Read(u8 address,u8 *data,u16 dataNum);   	     //读取指定数目的数据

void AT24C02_Erase(void);                                    //清空数据



#endif
