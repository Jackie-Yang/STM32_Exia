#ifndef I2C_H
#define I2C_H
#include "stm32f10x.h"



//IO�ڲ����궨��
#define BITBAND(addr, bitnum)    ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5)+(bitnum << 2)) 
#define MEM_ADDR(addr)           *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

//GPIOB�ڵ�ַ 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8)  //0x40010C08 

 
//GPIOB�ڲ����궨��

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 


//IO��������
#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0x80000000;}
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0x30000000;}

//IO��������	 
#define I2C_SCL    PBout(6) //SCL
#define I2C_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //����SDA 

#define NACK 0
#define ACK 1

//I2C�����������̺���				 
void I2C_Start(void);				//����I2C��ʼ�ź�
void I2C_Stop(void);	  			//����I2Cֹͣ�ź�
u8 I2C_Send_Data(u8 data);		//I2C��������
u8 I2C_Read_Data(u8 ack);//I2C��ȡ����
u8 I2C_Wait_Ack(void); 				//I2C�ȴ�ACK�ź�
void I2C_Ack(void);					//I2C����ACK�ź�
void I2C_NAck(void);				//I2C������ACK�ź�


void I2C_ReadMode(u8 device_address,u8 address);			  //ָ���豸ָ����ַ��ʼI2C��ȡģʽ
void I2C_SendMode(u8 device_address,u8 address);			  //ָ���豸ָ����ַ��ʼI2Cд��ģʽ



//I2C���������������豸��ַ��ҪԤ�����һλ��д״̬λ
void I2C_SendByte(u8 device_address,u8 address,u8 data);	  //��ָ���豸ָ����ַ����һ���ֽ�����
u8 I2C_ReadByte(u8 device_address,u8 address);	  		      //��ָ���豸ָ����ַ��ȡһ���ֽ�����

void I2C_SendByte_NoAddr(u8 device_address,u8 data);	  	  //��ָ���豸����һ���ֽ�����

u16 I2C_Read_16(u8 device_address,u8 address);				  //��ָ���豸ָ����ַ��ȡ�����ֽ�����
u32 I2C_Read_32(u8 device_address,u8 address);				  //��ָ���豸ָ����ַ��ȡ�����ֽ�����		   	

#endif


