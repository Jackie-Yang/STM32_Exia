#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f10x.h"

/**********************�궨��*************************/

#define	MPU6050_Addr   0xD0	  //����������IIC�����еĴӵ�ַ(�Ѿ�����һλ)  

/*******************����MPU6050�ڲ���ַ*********************/
#define	SMPLRT_DIV		25		//�����ǲ�����0x19������ֵ��0x07(125Hz)
#define	CONFIG			26		//��ͨ�˲�Ƶ��0x1A������ֵ��0x06(5Hz)
#define	GYRO_CONFIG		27		//�������Լ켰������Χ0x1B������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	28		//���ټ��Լ������Χ����ͨ�˲�Ƶ��0x1C������ֵ��0x01(���Լ죬2G��5Hz)

#define INT_PIN_CFG     0x37    //������·��Ч ��ֵ��0x42 AUX_DA�ĸ���I2C
#define USER_CTRL       0x6A    //�û����üĴ��� ��ֵ��0x40  AUX_DA�ĸ���I2C

#define	ACCEL_XOUT_H	0x3B    // �洢�����X�ᡢY�ᡢZ����ٶȸ�Ӧ���Ĳ���ֵ
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41	// �洢������¶ȴ������Ĳ���ֵ 
#define	TEMP_OUT_L		0x42
								 // �洢�����X�ᡢY�ᡢZ�������Ǹ�Ӧ���Ĳ���ֵ */
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		107		//��Դ����0x6B������ֵ��0x00(��������)
#define	WHO_AM_I		117		//I2C��ַ�Ĵ���0x75(Ĭ����ֵ0x68��ֻ��)



//extern u8 DMA_Buff [];

extern int16_t MPU6050_Accel_X,MPU6050_Accel_Y,MPU6050_Accel_Z;
extern int16_t Accel_offset_X,Accel_offset_Y,Accel_offset_Z; 

extern int16_t MPU6050_Gyro_X,MPU6050_Gyro_Y,MPU6050_Gyro_Z;
extern int16_t Gyro_offset_X,Gyro_offset_Y,Gyro_offset_Z;

extern float MPU6050_Temperature;


void MPU6050_Init(void); 			//MPU6050��ʼ�����ò���
void MPU6050_WHO_AM_I(void);		//��ȡMPU6050ʶ������(��I2C��ַ)

void READ_MPU6050_Accel(void);		//��ȡ���ٶȴ���������
void READ_MPU6050_Gyro(void);		//��ȡ����������
void MPU6050_SetOffset(void);		//�Դ�����������ƫУ��
void READ_MPU6050_TEMP(void);		//��ȡ�������¶�

void MPU6050_DMP_Init(void);
void MPU6050_DMP_SelfTest(void);
void Read_MPU6050_DMP(void);

#endif

