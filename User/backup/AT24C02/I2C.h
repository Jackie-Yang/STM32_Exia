#ifndef I2C_2_H
#define I2C_2_H
#include "stm32f10x.h"
#include "delay.h"


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

//I2C��������				 
void I2C_Start(void);				//����I2C��ʼ�ź�
void I2C_Stop(void);	  			//����I2Cֹͣ�ź�

void I2C_Send_Byte(u8 txd);			//I2C����һ���ֽ�
u8 I2C_Read_Byte(unsigned char ack);//I2C��ȡһ���ֽ�

u8 I2C_Wait_Ack(void); 				//I2C�ȴ�ACK�ź�
void I2C_Ack(void);					//I2C����ACK�ź�
void I2C_NAck(void);				//I2C������ACK�ź�


#endif


