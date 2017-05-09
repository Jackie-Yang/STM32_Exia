#ifndef HMC5883L_H
#define HMC5883L_H

/*����ͷ------------------------------------------------------------------*/
#include "stm32f10x.h"



/**************�궨��**************************/
#define	HMC5883L_Addr   0x3C	//�ų�������������ַ   
#define HMC5883L_ConfigurationRegisterA  0x00
#define HMC5883L_ConfigurationRegisterB  0x01
#define HMC5883L_ModeRegister            0x02
#define HMC5883L_Output_X_MSB            0x03
#define HMC5883L_Output_X_LSB 			 0x04
#define HMC5883L_Output_Z_MSB            0x05
#define HMC5883L_Output_Z_LSB 			 0x06
#define HMC5883L_Output_Y_MSB            0x07
#define HMC5883L_Output_Y_LSB 			 0x08

extern int16_t Mag_Offset[];

void HMC5883L_Init(void);			//ģ���ʼ����������ؼĴ���
void Read_HMC5883L(int16_t *Mag, __packed float *MagAngle); //��ȡ�ų�����
void HMC5883L_SetOffset(int16_t *Mag, __packed float *MagAngle); //���򵥵�У��

#endif
