#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "Setup.h"
#include "eeprom.h"
#include "USART.h"
#include "I2C.h"
#include "delay.h"
#include "PID.h"
#include "math.h"

#define q30 1073741824.0f
#define DEFAULT_MPU_HZ (500)

/********************ȫ�ֱ���*****************************/
int16_t MPU6050_Gyro_X = 0,MPU6050_Gyro_Y = 0,MPU6050_Gyro_Z = 0;
int16_t Gyro_offset_X = 0,Gyro_offset_Y = 0,Gyro_offset_Z = 0;
int16_t MPU6050_Accel_X = 0,MPU6050_Accel_Y = 0,MPU6050_Accel_Z = 0;
int16_t Accel_offset_X = 0,Accel_offset_Y = 0,Accel_offset_Z = 0;

int16_t MPU6050_Temperature = 0;

static signed char gyro_orientation[9] = {-1, 0, 0,
										  0, -1, 0,
										  0, 0, 1};

/************************************************************   
* ������:Init_MPU6050   
* ���� ����ʼ��MPU6050��������Ҫ��ο�pdf�����޸�
* ���� :��   
* ��� :��    
*/
void MPU6050_Init(void)
{

	I2C_SendByte(MPU6050_Addr,PWR_MGMT_1, 0x00);	//�������״̬
	
	
	I2C_SendByte(MPU6050_Addr,SMPLRT_DIV, 0x07);    //�����ǲ�����
	I2C_SendByte(MPU6050_Addr,CONFIG, 0x06);        //5Hz 
	
	I2C_SendByte(MPU6050_Addr,INT_PIN_CFG, 0x42);   //ʹ����·I2C
	I2C_SendByte(MPU6050_Addr,USER_CTRL, 0x40);     //ʹ����·I2C
	
	I2C_SendByte(MPU6050_Addr,GYRO_CONFIG, 0x00);   //�������Լ켰������Χ0x1B����0x18(���Լ죬250deg/s)
	
	I2C_SendByte(MPU6050_Addr,ACCEL_CONFIG, 0x01); //���ټ��Լ������Χ����ͨ�˲�Ƶ��0x1C��0x01(���Լ죬2G��5Hz)

	 //��Flash��ȡ֮ǰ��У������
	EE_ReadVariable(OFFSET_AX_ADDR, (uint16_t*)&Accel_offset_X);
	EE_ReadVariable(OFFSET_AY_ADDR, (uint16_t*)&Accel_offset_Y);
	EE_ReadVariable(OFFSET_AZ_ADDR, (uint16_t*)&Accel_offset_Z);
	EE_ReadVariable(OFFSET_GX_ADDR, (uint16_t*)&Gyro_offset_X);
	EE_ReadVariable(OFFSET_GY_ADDR, (uint16_t*)&Gyro_offset_Y);
	EE_ReadVariable(OFFSET_GZ_ADDR, (uint16_t*)&Gyro_offset_Z);
	//��У�����ݷ��͵���λ��
//	DMA_Buff_In_16(Accel_offset_X,ACCEL_OFFSET_X_INDEX);
//	DMA_Buff_In_16(Accel_offset_Y,ACCEL_OFFSET_Y_INDEX);
//	DMA_Buff_In_16(Accel_offset_Z,ACCEL_OFFSET_Z_INDEX);
//	DMA_Buff_In_16(Gyro_offset_X,GYRO_OFFSET_X_INDEX);
//	DMA_Buff_In_16(Gyro_offset_Y,GYRO_OFFSET_Y_INDEX);
//	DMA_Buff_In_16(Gyro_offset_Z,GYRO_OFFSET_Z_INDEX);
	
}

/************************************************************   
* ������:MP6050_WHO_AM_I   
* ���� ����ȡMPU6050�豸��Ϣ
* ���� :��   
* ��� :��    
*/

void MPU6050_WHO_AM_I(void)
{
	u8 data = 0;
	I2C_ReadByte(MPU6050_Addr, WHO_AM_I, &data);
}





/************************************************************   
* ������:READ_MPU6050_Accel   
* ���� : ��ȡMPU6050���ٶ����ݣ������͵���λ��
* ����  :��   
* ���  :��  
**************************/



void READ_MPU6050_Accel(void)
{

	I2C_ReadBytes_BE(MPU6050_Addr, ACCEL_XOUT_H, sizeof(MPU6050_Accel_X), (uint8_t *)&MPU6050_Accel_X);
	if( (int32_t)MPU6050_Accel_X + Accel_offset_X <= 32768  &&  (int32_t)MPU6050_Accel_X + Accel_offset_X >= -32768)
	{
		MPU6050_Accel_X = MPU6050_Accel_X + Accel_offset_X;
	}
	stQuadrotor_State.s16_Accel_X = MPU6050_Accel_X;

	I2C_ReadBytes_BE(MPU6050_Addr, ACCEL_YOUT_H, sizeof(MPU6050_Accel_Y), (uint8_t *)&MPU6050_Accel_Y);
	if( (int32_t)MPU6050_Accel_Y + Accel_offset_Y <= 32768  &&  (int32_t)MPU6050_Accel_Y + Accel_offset_Y >= -32768)
	{
		MPU6050_Accel_Y = MPU6050_Accel_Y + Accel_offset_Y;
	}
	stQuadrotor_State.s16_Accel_Y = MPU6050_Accel_Y;

	I2C_ReadBytes_BE(MPU6050_Addr, ACCEL_ZOUT_H, sizeof(MPU6050_Accel_Z), (uint8_t *)&MPU6050_Accel_Z);
	if( (int32_t)MPU6050_Accel_Z + Accel_offset_Z <= 32768  &&  (int32_t)MPU6050_Accel_Z + Accel_offset_Z >= -32768)
	{
		MPU6050_Accel_Z = MPU6050_Accel_Z + Accel_offset_Z;
	}
	stQuadrotor_State.s16_Accel_Z = MPU6050_Accel_Z;
	stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
}



/************************************************************   
* ������:READ_MPU6050_Gyro   
* ���� : ��ȡMPU6050���������ݣ������͵���λ��
* ����  :��   
* ���  :��    
*/

void READ_MPU6050_Gyro(void)
{
	I2C_ReadBytes_BE(MPU6050_Addr, GYRO_XOUT_H, sizeof(MPU6050_Gyro_X), (uint8_t *)&MPU6050_Gyro_X);
	if( (int32_t)MPU6050_Gyro_X + Gyro_offset_X <= 32768  &&  (int32_t)MPU6050_Gyro_X + Gyro_offset_X >= -32768)
	{
		MPU6050_Gyro_X = MPU6050_Gyro_X + Gyro_offset_X;
	}
	stQuadrotor_State.s16_Gyro_X = MPU6050_Gyro_X;

	I2C_ReadBytes_BE(MPU6050_Addr, GYRO_YOUT_H, sizeof(MPU6050_Gyro_Y), (uint8_t *)&MPU6050_Gyro_Y);
	if( (int32_t)MPU6050_Gyro_Y + Gyro_offset_Y <= 32768  &&  (int32_t)MPU6050_Gyro_Y + Gyro_offset_Y >= -32768)
	{
		MPU6050_Gyro_Y = MPU6050_Gyro_Y + Gyro_offset_Y;
	}
	stQuadrotor_State.s16_Gyro_Y = MPU6050_Gyro_Y;

	I2C_ReadBytes_BE(MPU6050_Addr, GYRO_ZOUT_H, sizeof(MPU6050_Gyro_Z), (uint8_t *)&MPU6050_Gyro_Z);
	if( (int32_t)MPU6050_Gyro_Z + Gyro_offset_Z <= 32768  &&  (int32_t)MPU6050_Gyro_Z + Gyro_offset_Z >= -32768)
	{
		MPU6050_Gyro_Z = MPU6050_Gyro_Z + Gyro_offset_Z;
	}
	stQuadrotor_State.s16_Gyro_Z = MPU6050_Gyro_Z;
	stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
}


 /************************************************************   
* ������:MPU6050_SetOffset   
* ���� : ��MPU6050���ٶȴ������������ǽ���0ƫУ������ˮƽλַʱȡ200��ֵ��ƽ��
* ����  :��   
* ���  :��    
*/

void MPU6050_SetOffset(void)
{
	int16_t data;
	int32_t text_Ax = 0, text_Ay = 0, text_Az = 0;
	int32_t text_Gx = 0, text_Gy = 0, text_Gz = 0;
	u16 i;
	for (i = 0; i < 200; i++)
	{ //ǿ��ת����short int���ͣ�����Ǹ�������ת��32λʱ���Զ����Ϸ���
		I2C_ReadBytes_BE(MPU6050_Addr, ACCEL_XOUT_H, sizeof(data), (uint8_t *)&data);
		text_Ax += (short int)data;
		//delay_ms(1);
		I2C_ReadBytes_BE(MPU6050_Addr, ACCEL_YOUT_H, sizeof(data), (uint8_t *)&data);
		text_Ay += (short int)data;
		//delay_ms(1);
		I2C_ReadBytes_BE(MPU6050_Addr, ACCEL_ZOUT_H, sizeof(data), (uint8_t *)&data);
		text_Az += (short int)(data - 16384);
		//delay_ms(1);
		I2C_ReadBytes_BE(MPU6050_Addr, GYRO_XOUT_H, sizeof(data), (uint8_t *)&data);
		text_Gx += (short int)data;
		//delay_ms(1);
		I2C_ReadBytes_BE(MPU6050_Addr, GYRO_YOUT_H, sizeof(data), (uint8_t *)&data);
		text_Gy += (short int)data;
		//delay_ms(1);
		I2C_ReadBytes_BE(MPU6050_Addr, GYRO_ZOUT_H, sizeof(data), (uint8_t *)&data);
		text_Gz += (short int)data;
		//delay_ms(1);
	 }
	 Accel_offset_X = -(text_Ax / 200);
	 Accel_offset_Y = -(text_Ay / 200);
	 Accel_offset_Z = -(text_Az / 200);

	 Gyro_offset_X = -(text_Gx / 200);
	 Gyro_offset_Y = -(text_Gy / 200);
	 Gyro_offset_Z = -(text_Gz / 200);

	
	 EE_WriteVariable(OFFSET_AX_ADDR, Accel_offset_X);	   //��ƫ���������Flash�������´ο�����ȡ
	 EE_WriteVariable(OFFSET_AY_ADDR, Accel_offset_Y);
	 EE_WriteVariable(OFFSET_AZ_ADDR, Accel_offset_Z);
	 EE_WriteVariable(OFFSET_GX_ADDR, Gyro_offset_X);
	 EE_WriteVariable(OFFSET_GY_ADDR, Gyro_offset_Y);
	 EE_WriteVariable(OFFSET_GZ_ADDR, Gyro_offset_Z);

	 //��У�����ݷ��͵���λ��
//	DMA_Buff_In_16(Accel_offset_X,ACCEL_OFFSET_X_INDEX);
//	DMA_Buff_In_16(Accel_offset_Y,ACCEL_OFFSET_Y_INDEX);
//	DMA_Buff_In_16(Accel_offset_Z,ACCEL_OFFSET_Z_INDEX);
//	DMA_Buff_In_16(Gyro_offset_X,GYRO_OFFSET_X_INDEX);
//	DMA_Buff_In_16(Gyro_offset_Y,GYRO_OFFSET_Y_INDEX);
//	DMA_Buff_In_16(Gyro_offset_Z,GYRO_OFFSET_Z_INDEX);

}

void READ_MPU6050_TEMP(void)
{
	I2C_ReadBytes_BE(MPU6050_Addr, TEMP_OUT_H, sizeof(MPU6050_Temperature), (uint8_t *)&MPU6050_Temperature);
	//MPU6050_Temperature = MPU6050_Temperature / 340.0 + 36.53
	stQuadrotor_State.s16_MPU6050_Temp = MPU6050_Temperature;		   //�¶���ʱû�ã������¶���ʱ������λ��
	stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
}

void MPU6050_DMP_SelfTest(void)
{
	int result;
	long gyro[3], accel[3];

	result = mpu_run_self_test(gyro, accel);
	// if (result == 0x7)	//DMPû���ɵشŴ������������Լ���ֻ�����3�����ٶȣ����ٶȣ��شŴ�������ռһλ��
	if (result == 0x3)
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		// printf("setting bias succesfully ......\r\n");
	}
}

static unsigned short inv_row_2_scale(const signed char *row)
{
	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7; // error
	return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
	unsigned short scalar;
	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;
	return scalar;
}

/**************************************************************************
�������ܣ�MPU6050����DMP�ĳ�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void MPU6050_DMP_Init(void)
{
	// if (!MPU6050_testConnection())   //������ȡ����
	// {
	//     NVIC_SystemReset();
	// }
	//  printf("mpu_set_sensor complete ......\r\n");
	if (!mpu_init())
	{
		if (!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		{
			//  printf("mpu_set_sensor complete ......\r\n");
		}
		if (!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		{
			//  printf("mpu_configure_fifo complete ......\r\n");
		}
		if (!mpu_set_sample_rate(DEFAULT_MPU_HZ))
		{
			//  printf("mpu_set_sample_rate complete ......\r\n");
		}
		if (!dmp_load_motion_driver_firmware())
		{
			// printf("dmp_load_motion_driver_firmware complete ......\r\n");
		}
		if (!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
		{ //  printf("dmp_set_orientation complete ......\r\n");
		}
		if (!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
								DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
								DMP_FEATURE_GYRO_CAL))
		{ //  printf("dmp_enable_feature complete ......\r\n");
		}
		if (!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
		{ //  printf("dmp_set_fifo_rate complete ......\r\n");
		}
		MPU6050_DMP_SelfTest();
		if (!mpu_set_dmp_state(1))
		{ //  printf("mpu_set_dmp_state complete ......\r\n");
		}
		// if (!MPU6050_testConnection())   //������ȡ����
		// {
		//     NVIC_SystemReset();
		// }
	}
	else
	{
		NVIC_SystemReset();
	}
}
/**************************************************************************
�������ܣ���ȡMPU6050����DMP����̬��Ϣ
��ڲ�������
����  ֵ����
**************************************************************************/
void Read_MPU6050_DMP(void)
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	short gyro[3], accel[3], sensors;
	float dmp_q0 = 1.0f, dmp_q1 = 0.0f, dmp_q2 = 0.0f, dmp_q3 = 0.0f;

	if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
	{
		return;
	}
	if (sensors & INV_WXYZ_QUAT)
	{
		dmp_q0 = quat[0] / q30;
		dmp_q1 = quat[1] / q30;
		dmp_q2 = quat[2] / q30;
		dmp_q3 = quat[3] / q30;
		stQuadrotor_State.s16_Accel_X = accel[0];
		stQuadrotor_State.s16_Accel_Y = accel[1];
		stQuadrotor_State.s16_Accel_Z = accel[2];

		stQuadrotor_State.s16_Gyro_X = -gyro[0];
		stQuadrotor_State.s16_Gyro_Y = -gyro[1];
		stQuadrotor_State.s16_Gyro_Z = gyro[2];

		Pitch.Gyro_cur = (float)stQuadrotor_State.s16_Gyro_X / 16.4;
		Roll.Gyro_cur = (float)stQuadrotor_State.s16_Gyro_Y / 16.4;
		Yaw.Gyro_cur = (float)stQuadrotor_State.s16_Gyro_Z / 16.4;

		stQuadrotor_State.f_Roll = -asin(-2 * dmp_q1 * dmp_q3 + 2 * dmp_q0 * dmp_q2) * 57.3;
		stQuadrotor_State.f_Pitch = -atan2(2 * dmp_q2 * dmp_q3 + 2 * dmp_q0 * dmp_q1, -2 * dmp_q1 * dmp_q1 - 2 * dmp_q2 * dmp_q2 + 1) * 57.3; // roll
		stQuadrotor_State.f_Yaw = atan2(2 * (dmp_q1 * dmp_q2 + dmp_q0 * dmp_q3), dmp_q0 * dmp_q0 + dmp_q1 * dmp_q1 - dmp_q2 * dmp_q2 - dmp_q3 * dmp_q3) * 57.3;
		Yaw.angle_cur = stQuadrotor_State.f_Yaw;
		Pitch.angle_cur = stQuadrotor_State.f_Pitch;
		Roll.angle_cur = stQuadrotor_State.f_Roll;
		stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
		//  Dmp_Pitch = asin(-2 * dmp_q1 * dmp_q3 + 2 * dmp_q0* dmp_q2)* 57.3;
		//  Dmp_Roll = atan2(2 * dmp_q2 * dmp_q3 + 2 * dmp_q0 * dmp_q1, -2 * dmp_q1 * dmp_q1 - 2 * dmp_q2* dmp_q2 + 1)* 57.3; // roll
		//  Dmp_Yaw = 	atan2(2*(dmp_q1*dmp_q2 + dmp_q0*dmp_q3),dmp_q0*dmp_q0+dmp_q1*dmp_q1-dmp_q2*dmp_q2-dmp_q3*dmp_q3) * 57.3;
	}
}
