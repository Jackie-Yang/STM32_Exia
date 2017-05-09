#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "Setup.h"
#include "USART.h"
#include "I2C.h"
#include "delay.h"
#include "PID.h"
#include "math.h"

#define q30 1073741824.0f
#define DEFAULT_MPU_HZ (500)
#define SETOFFSET_TIMES 200

/********************ȫ�ֱ���*****************************/
int16_t Gyro_offset[3] = {0};
int16_t Accel_offset[3] = {0};


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

void READ_MPU6050_Accel(int16_t *Accel)
{
	int i = 0;
	uint8_t Buff[6] = {0};
	int16_t AccelRaw[3];
	I2C_ReadBytes_LE(MPU6050_Addr, ACCEL_XOUT_H, 6, Buff);
	AccelRaw[0] = (Buff[0] << 8) | Buff[1];
	AccelRaw[1] = (Buff[2] << 8) | Buff[3];
	AccelRaw[2] = (Buff[4] << 8) | Buff[5];
	for (i = 0; i < 3; i++)
	{
		if ((int32_t)AccelRaw[i] + Accel_offset[i] > 32767)
		{
			Accel[i] = 32767;
		}
		else if ((int32_t)AccelRaw[i] + Accel_offset[i] < -32768)
		{
			Accel[i] = -32768;
		}
		else
		{
			Accel[i] = AccelRaw[i] + Accel_offset[i];
		}
	}
}

/************************************************************   
* ������:READ_MPU6050_Gyro   
* ���� : ��ȡMPU6050���������ݣ������͵���λ��
* ����  :��   
* ���  :��    
*/

void READ_MPU6050_Gyro(int16_t *Gyro)
{
	int i = 0;
	uint8_t Buff[6] = {0};
	int16_t GyroRaw[3];
	I2C_ReadBytes_LE(MPU6050_Addr, GYRO_XOUT_H, 6, Buff);
	GyroRaw[0] = (Buff[0] << 8) | Buff[1];
	GyroRaw[1] = (Buff[2] << 8) | Buff[3];
	GyroRaw[2] = (Buff[4] << 8) | Buff[5];

	for (i = 0; i < 3; i++)
	{
		if ((int32_t)GyroRaw[i] + Gyro_offset[i] > 32767)
		{
			Gyro[i] = 32767;
		}
		else if ((int32_t)GyroRaw[i] + Gyro_offset[i] < -32768)
		{
			Gyro[i] = -32768;
		}
		else
		{
			Gyro[i] = GyroRaw[i] + Gyro_offset[i];
		}
	}
}


 /************************************************************   
* ������:MPU6050_SetOffset   
* ���� : ��MPU6050���ٶȴ������������ǽ���0ƫУ������ˮƽλַʱȡ200��ֵ��ƽ��
* ����  :��   
* ���  :��    
*/

void MPU6050_SetOffset(int16_t *Accel, int16_t *Gyro)
{
	int32_t Sum_Ax = 0, Sum_Ay = 0, Sum_Az = 0;
	int32_t Sum_Gx = 0, Sum_Gy = 0, Sum_Gz = 0;
	u16 i;
	for (i = 0; i < 3; i++)
	{
		Gyro_offset[i] = 0;
		Accel_offset[i] = 0;
	}
	for (i = 0; i < SETOFFSET_TIMES; i++)
	{
		READ_MPU6050_Accel(Accel);
		Sum_Ax += Accel[0];
		Sum_Ay += Accel[1];
		Sum_Az += (Accel[2] - 16384);

		READ_MPU6050_Gyro(Gyro);
		Sum_Gx += Gyro[0];
		Sum_Gy += Gyro[1];
		Sum_Gz += Gyro[2];
	 }
	 Accel_offset[0] = -(Sum_Ax / SETOFFSET_TIMES);
	 Accel_offset[1] = -(Sum_Ay / SETOFFSET_TIMES);
	 Accel_offset[2] = -(Sum_Az / SETOFFSET_TIMES);

	 Gyro_offset[0] = -(Sum_Gx / SETOFFSET_TIMES);
	 Gyro_offset[1] = -(Sum_Gy / SETOFFSET_TIMES);
	 Gyro_offset[2] = -(Sum_Gz / SETOFFSET_TIMES);
}

void READ_MPU6050_TEMP(__packed int16_t *pTemp)
{
	I2C_ReadBytes_BE(MPU6050_Addr, TEMP_OUT_H, sizeof(int16_t), (uint8_t *)&pTemp);
	//MPU6050_Temperature = MPU6050_Temperature / 340.0 + 36.53
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
void Read_MPU6050_DMP(int16_t *Accel, int16_t *Gyro)
{
	unsigned char more;
	long quat[4];
	short gyro[3], accel[3], sensors;
	float dmp_q0 = 1.0f, dmp_q1 = 0.0f, dmp_q2 = 0.0f, dmp_q3 = 0.0f;

	if (dmp_read_fifo(gyro, accel, quat, 0, &sensors, &more))
	{
		return;
	}
	if (sensors & INV_WXYZ_QUAT)
	{
		dmp_q0 = quat[0] / q30;
		dmp_q1 = quat[1] / q30;
		dmp_q2 = quat[2] / q30;
		dmp_q3 = quat[3] / q30;
		Accel[0] = accel[0];
		Accel[1] = accel[1];
		Accel[2] = accel[2];

		Gyro[0] = -gyro[0];
		Gyro[1] = -gyro[1];
		Gyro[2] = gyro[2];

		Pitch.Gyro_cur = (float)Gyro[0] / 16.4;
		Roll.Gyro_cur = (float)Gyro[1] / 16.4;
		Yaw.Gyro_cur = (float)Gyro[2] / 16.4;

		stQuadrotor_State.f_Roll = -asin(-2 * dmp_q1 * dmp_q3 + 2 * dmp_q0 * dmp_q2) * 57.3;
		stQuadrotor_State.f_Pitch = -atan2(2 * dmp_q2 * dmp_q3 + 2 * dmp_q0 * dmp_q1, -2 * dmp_q1 * dmp_q1 - 2 * dmp_q2 * dmp_q2 + 1) * 57.3; // roll
		stQuadrotor_State.f_Yaw = atan2(2 * (dmp_q1 * dmp_q2 + dmp_q0 * dmp_q3), dmp_q0 * dmp_q0 + dmp_q1 * dmp_q1 - dmp_q2 * dmp_q2 - dmp_q3 * dmp_q3) * 57.3;
		Yaw.angle_cur = stQuadrotor_State.f_Yaw;
		Pitch.angle_cur = stQuadrotor_State.f_Pitch;
		Roll.angle_cur = stQuadrotor_State.f_Roll;
		//  Dmp_Pitch = asin(-2 * dmp_q1 * dmp_q3 + 2 * dmp_q0* dmp_q2)* 57.3;
		//  Dmp_Roll = atan2(2 * dmp_q2 * dmp_q3 + 2 * dmp_q0 * dmp_q1, -2 * dmp_q1 * dmp_q1 - 2 * dmp_q2* dmp_q2 + 1)* 57.3; // roll
		//  Dmp_Yaw = 	atan2(2*(dmp_q1*dmp_q2 + dmp_q0*dmp_q3),dmp_q0*dmp_q0+dmp_q1*dmp_q1-dmp_q2*dmp_q2-dmp_q3*dmp_q3) * 57.3;
	}
}
