#ifndef AT24C02_H
#define AT24C02_H 

#include "I2C.h"
 

#define ADDRESS_24C02	0xA0
#define WRITE           0x00
#define READ            0x01 
 
void AT24C02_WriteOneByte(u8 address,u8 data);		         //д��һ���ֽ�					  
u8 AT24C02_ReadOneByte(u8 address);							 //��ȡһ���ֽ�

void AT24C02_WriteBytes(u8 address,u32 data,u8 dataSize);    //д��ָ�����ȵ�����
u32 AT24C02_ReadBytes(u8 address,u8 dataSize);			     //��ȡָ����������

void AT24C02_Write(u8 address,u8 *data,u16 dataNum);	     //д��ָ����Ŀ������
void AT24C02_Read(u8 address,u8 *data,u16 dataNum);   	     //��ȡָ����Ŀ������

void AT24C02_Erase(void);                                    //�������



#endif
