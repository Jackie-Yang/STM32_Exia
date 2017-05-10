#include "HMC5883L.h"
#include "math.h"
#include "I2C.h"
#include "delay.h"

float x_max = 0, x_min = 0, y_max = 0, y_min = 0, z_max = 0, z_min = 0;
float Mag_Offset[3] = {0};
float Mag_Gain[3] = {0};
float Scale[3] = {0};

//ģ���ʼ����������������һ��ƫ�õ�������ȡ������������У������ȡ��һ�δų��������Сֵ
int8_t HMC5883L_Init(void)
{
	int8_t ret = 0;
	float Mag[3];
	uint8_t Buff[6] = {0};

	//���üĴ���A������ƽ����1 �������30Hz ʩ������ƫ�õ��������Բ�
	ret |= I2C_SendByte(HMC5883L_Addr, HMC5883L_ConfigA, CONFIGA_AVERAGE_1 | CONFIGA_RATE_30 | CONFIGA_BIAS_POSITIVE);
	//���üĴ���B��������� 1090LSB/ga
	ret |= I2C_SendByte(HMC5883L_Addr, HMC5883L_ConfigB, CONFIGB_GAIN_1090);
	ret |= I2C_SendByte(HMC5883L_Addr, HMC5883L_Mode, MODE_SINGLE); //ģʽ�Ĵ��������β���ģʽ
	delay_ms(250);		//���ƫ�õ��������Բ������Ҫһ����ʱ
	ret |= I2C_ReadBytes_LE(HMC5883L_Addr, HMC5883L_Output_X_MSB, 6, Buff); //��ȡ�������
	Mag[0] = (Buff[0] << 8) | Buff[1];
	Mag[2] = (Buff[2] << 8) | Buff[3];
	Mag[1] = (Buff[4] << 8) | Buff[5];

	//ƫ�õ�������X��Y�����ϲ���1.16ga�Ĵų���Z������Ϊ1.08ga
	//ʹ�ò���ֵ������ֵ���бȽϵõ���������
	Scale[0] = 1.16 * 1090.0 / Mag[0];		
	Scale[1] = 1.16 * 1090.0 / Mag[1];
	Scale[2] = 1.08 * 1090.0 / Mag[2];
	if (Scale[0] > 1.2 || Scale[0] < 0.8 || Scale[1] > 1.2 || Scale[1] < 0.8 || Scale[2] > 1.2 || Scale[2] < 0.8)
	{
		ret = -1;	//������
	}

	//���üĴ���A������ƽ����1 �������75Hz ��������
	ret |= I2C_SendByte(HMC5883L_Addr, HMC5883L_ConfigA, CONFIGA_AVERAGE_1 | CONFIGA_RATE_75 | CONFIGA_BIAS_NORMAL);
	//���üĴ���B��������� 1090LSB/ga
	ret |= I2C_SendByte(HMC5883L_Addr, HMC5883L_ConfigB, CONFIGB_GAIN_1090);
	ret |= I2C_SendByte(HMC5883L_Addr, HMC5883L_Mode, MODE_CONTINUOUS); //ģʽ�Ĵ�������������ģʽ
	//��ȡ��һ��������Ϊ�����Сֵ�Ļ�׼
	delay_ms(250);													 //���ƫ�õ��������Բ������Ҫһ����ʱ
	ret |= I2C_ReadBytes_LE(HMC5883L_Addr, HMC5883L_Output_X_MSB, 6, Buff); //��ȡ�������
	Mag[0] = (int16_t)((Buff[0] << 8) | Buff[1]) * Scale[0];
	Mag[2] = (int16_t)((Buff[2] << 8) | Buff[3]) * Scale[2];
	Mag[1] = (int16_t)((Buff[4] << 8) | Buff[5]) * Scale[1];
	x_max = Mag[0];
	x_min = Mag[0];
	y_max = Mag[1];
	y_min = Mag[1];
	z_max = Mag[2];
	z_min = Mag[2];
	Mag_Offset[0] = (float)(x_max + x_min) / 2.0;
	Mag_Offset[1] = (float)(y_max + y_min) / 2.0;
	Mag_Offset[2] = (float)(z_max + z_min) / 2.0;
	Mag_Gain[0] = 1;
	Mag_Gain[1] = (float)(x_max - x_min) / (float)(y_max - y_min);
	Mag_Gain[2] = (float)(x_max - x_min) / (float)(z_max - z_min);
	return ret;
}

int8_t Read_HMC5883L(int16_t *Mag, __packed float *MagAngle) //��ȡ
{
	int8_t ret = 0;
	uint8_t Buff[6] = {0};
	float X_Heading, Y_Heading, Angle;
	float MagRaw[3] = {0};
	uint8_t bUpdateGain = 0;

	ret |= I2C_ReadBytes_LE(HMC5883L_Addr, HMC5883L_Output_X_MSB, 6, Buff);
	if(ret)
	{
		return ret;
	}
	//��ȡֵ�˱�������
	MagRaw[0] = (int16_t)((Buff[0] << 8) | Buff[1]) * Scale[0];
	MagRaw[2] = (int16_t)((Buff[2] << 8) | Buff[3]) * Scale[2];
	MagRaw[1] = (int16_t)((Buff[4] << 8) | Buff[5]) * Scale[1];

//����У׼����
	if (MagRaw[0] > x_max)
	{
		x_max = MagRaw[0];
		Mag_Offset[0] = (float)(x_max + x_min) / 2;
		bUpdateGain = 1;
	}
	else if (MagRaw[0] < x_min)
	{
		x_min = MagRaw[0];
		Mag_Offset[0] = (float)(x_max + x_min) / 2;
		bUpdateGain = 1;
	}
	if (MagRaw[1] > y_max)
	{
		y_max = MagRaw[1];
		Mag_Offset[1] = (float)(y_max + y_min) / 2;
		bUpdateGain = 1;
	}
	else if (MagRaw[1] < y_min)
	{
		y_min = MagRaw[1];
		Mag_Offset[1] = (float)(y_max + y_min) / 2;
		bUpdateGain = 1;
	}
	if (MagRaw[2] > z_max)
	{
		z_max = MagRaw[2];
		Mag_Offset[2] = (float)(z_max + z_min) / 2;
		bUpdateGain = 1;
	}
	else if (MagRaw[2] < z_min)
	{
		z_min = MagRaw[2];
		Mag_Offset[2] = (float)(z_max + z_min) / 2;
		bUpdateGain = 1;
	}

	if (bUpdateGain)
	{
		Mag_Gain[1] = (float)(x_max - x_min) / (float)(y_max - y_min);
		Mag_Gain[2] = (float)(x_max - x_min) / (float)(z_max - z_min);
	}

	X_Heading = MagRaw[0] - Mag_Offset[0];
	Y_Heading = Mag_Gain[1] * (MagRaw[1] - Mag_Offset[1]);

	Mag[0] = X_Heading;
	Mag[1] = Y_Heading;
	Mag[2] = Mag_Gain[2] * (MagRaw[2] - Mag_Offset[2]);

	

	//�ų���ֵ��Сһ�㲻�����
	// if (Mag[0] > 0x7fff)
	// 	Mag[0] -= 0xffff;
	// if (Mag[1] > 0x7fff)
	// 	Mag[1] -= 0xffff;
	// if (Mag[2] > 0x7fff)
	// 	Mag[2] -= 0xffff;
	// X_Heading = X_Heading * cos(Pitch) + Mag[1] * sin(Roll) * sin(Pitch) - Mag[2] * cos(Roll) * sin(Pitch);
	// Y_Heading = Y_Heading * cos(Roll) + Mag[2] * sin(Roll);
	Angle = atan2f(Y_Heading, X_Heading) * (180 / 3.14159265) + 180; // angle in degrees
	Angle += 91.09;	//���˳��ڼ������ԭ�򣬴������ĽǶ���ɿز�90�ȣ�1.09�ǹ��ݵĴ�ƫ��
	if (Angle > 360)
		Angle -= 360;
	*MagAngle = Angle;
	return 0;
}

// uint8_t A, B, Mode, State;
// void Check()	//���Ĵ���
// {
// 	I2C_ReadByte(HMC5883L_Addr, HMC5883L_ConfigA, &A);
// 	I2C_ReadByte(HMC5883L_Addr, HMC5883L_ConfigB, &B);
// 	I2C_ReadByte(HMC5883L_Addr, HMC5883L_Mode, &Mode);
// 	I2C_ReadByte(HMC5883L_Addr, HMC5883L_STATE, &State);
// }

