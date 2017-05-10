#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "I2C.h"
#include "delay.h"
#include "math.h"



/********************全局变量*****************************/
#if DMP_ENABLE
static signed char gyro_orientation[9] = {-1, 0, 0,
										  0, -1, 0,
										  0, 0, 1};
static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
#define q30 1073741824.0f
#else
#define SETOFFSET_TIMES 200
int16_t Gyro_offset[3] = {0};
int16_t Accel_offset[3] = {0};
#endif
/************************************************************   
* 函数名:Init_MPU6050   
* 描述 ：初始化MPU6050，根据需要请参考pdf进行修改
* 输入 :无   
* 输出 :无    
*/
int8_t MPU6050_Init(void)
{
	int8_t ret = 0;
#if DMP_ENABLE
	// if (!MPU6050_testConnection())   //经常获取错误
	// {
	//     NVIC_SystemReset();
	// }
	ret |= mpu_init();
	ret |= mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	ret |= mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	ret |= mpu_set_sample_rate(200);		//采样频率200
	ret |= dmp_load_motion_driver_firmware();
	ret |= dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
	ret |= dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
							  DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
							  DMP_FEATURE_GYRO_CAL);
	ret |= dmp_set_fifo_rate(100);			//fifo输出频率100Hz，不能太快，否则fifo溢出会更慢，根据读数频率设置
	ret |= MPU6050_DMP_SelfTest();
	ret |= mpu_set_dmp_state(1);
#else
	ret |= I2C_SendByte(MPU6050_Addr, PWR_MGMT_1, 0x00); //解除休眠状态

	ret |= I2C_SendByte(MPU6050_Addr, SMPLRT_DIV, 0x07); //陀螺仪采样率，1Khz（Config没配置滤波时为8Khz）/(1 + 7) = 125Hz
	ret |= I2C_SendByte(MPU6050_Addr, CONFIG, 0x06);	 //低通滤波器5Hz

	ret |= I2C_SendByte(MPU6050_Addr, INT_PIN_CFG, 0x42); //使能旁路I2C
	ret |= I2C_SendByte(MPU6050_Addr, USER_CTRL, 0x40);   //使能旁路I2C

	ret |= I2C_SendByte(MPU6050_Addr, GYRO_CONFIG, 0x18); //陀螺仪自检及测量范围0x1B，：0x18(不自检，2000deg/s)

	ret |= I2C_SendByte(MPU6050_Addr, ACCEL_CONFIG, 0x00); //加速计自检测量范围及高通滤波频率0x1C，0x01(不自检，2G，5Hz)

	//水平校正，当前状态将设置成水平
	MPU6050_SetOffset();
#endif
	return ret;
}




#if DMP_ENABLE

int8_t MPU6050_DMP_SelfTest(void)
{
	int8_t ret;
	long gyro[3], accel[3];

	ret = mpu_run_self_test(gyro, accel);
	// if (result == 0x7)	//DMP没集成地磁传感器，所以自检完只会等于3（加速度，角速度，地磁传感器各占一位）
	if (ret == 0x3)
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
		float sens;
		unsigned short accel_sens;
		ret = mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		ret |= dmp_set_gyro_bias(gyro);
		ret |= mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		ret |= dmp_set_accel_bias(accel);
		// printf("setting bias succesfully ......\r\n");
		return ret;
	}
	else
	{
		return -1;
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
函数功能：读取MPU6050内置DMP的姿态信息
入口参数：无
返回  值：无
**************************************************************************/
int8_t Read_MPU6050_DMP(int16_t *Accel, int16_t *Gyro, __packed float *Roll, __packed float *Pitch, __packed float *Yaw)
{
	unsigned char more;
	long quat[4];
	short gyro[3], accel[3], sensors;
	float dmp_q0 = 1.0f, dmp_q1 = 0.0f, dmp_q2 = 0.0f, dmp_q3 = 0.0f;

	if (dmp_read_fifo(gyro, accel, quat, 0, &sensors, &more))
	{
		return -1;
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


		*Roll = -asin(-2 * dmp_q1 * dmp_q3 + 2 * dmp_q0 * dmp_q2) * 57.3;
		*Pitch = -atan2(2 * dmp_q2 * dmp_q3 + 2 * dmp_q0 * dmp_q1, -2 * dmp_q1 * dmp_q1 - 2 * dmp_q2 * dmp_q2 + 1) * 57.3; // roll
		*Yaw = atan2(2 * (dmp_q1 * dmp_q2 + dmp_q0 * dmp_q3), dmp_q0 * dmp_q0 + dmp_q1 * dmp_q1 - dmp_q2 * dmp_q2 - dmp_q3 * dmp_q3) * 57.3;
		
		//  Dmp_Pitch = asin(-2 * dmp_q1 * dmp_q3 + 2 * dmp_q0* dmp_q2)* 57.3;
		//  Dmp_Roll = atan2(2 * dmp_q2 * dmp_q3 + 2 * dmp_q0 * dmp_q1, -2 * dmp_q1 * dmp_q1 - 2 * dmp_q2* dmp_q2 + 1)* 57.3; // roll
		//  Dmp_Yaw = 	atan2(2*(dmp_q1*dmp_q2 + dmp_q0*dmp_q3),dmp_q0*dmp_q0+dmp_q1*dmp_q1-dmp_q2*dmp_q2-dmp_q3*dmp_q3) * 57.3;
		return 0;
	}
	else
	{
		return -1;
	}
}

#else

/************************************************************   
* 函数名:READ_MPU6050_Accel   
* 描述 : 读取MPU6050加速度数据，并发送到上位机
* 输入  :无   
* 输出  :无  
**************************/

int8_t READ_MPU6050_Accel(int16_t *Accel)
{
	int i = 0;
	int8_t ret = 0;
	uint8_t Buff[6] = {0};
	int16_t AccelRaw[3];
	ret = I2C_ReadBytes_LE(MPU6050_Addr, ACCEL_XOUT_H, 6, Buff);
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
	return ret;
}

/************************************************************   
* 函数名:READ_MPU6050_Gyro   
* 描述 : 读取MPU6050陀螺仪数据，并发送到上位机
* 输入  :无   
* 输出  :无    
*/

int8_t READ_MPU6050_Gyro(int16_t *Gyro)
{
	int i = 0;
	int8_t ret = 0;
	uint8_t Buff[6] = {0};
	int16_t GyroRaw[3];
	ret = I2C_ReadBytes_LE(MPU6050_Addr, GYRO_XOUT_H, 6, Buff);
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
	return ret;
}

/************************************************************   
* 函数名:MPU6050_SetOffset   
* 描述 : 对MPU6050加速度传感器，陀螺仪进行0偏校正，在水平位址时取200次值作平均
* 输入  :无   
* 输出  :无    
*/

void MPU6050_SetOffset()
{
	int16_t Accel[3], Gyro[3];
	int32_t Sum_Ax = 0, Sum_Ay = 0, Sum_Az = 0;
	int32_t Sum_Gx = 0, Sum_Gy = 0, Sum_Gz = 0;
	uint16_t SumCountA = 0, SumCountG = 0;
	int8_t ret = 0;
	u16 i;
	for (i = 0; i < 3; i++)
	{
		Gyro_offset[i] = 0;
		Accel_offset[i] = 0;
	}
	for (i = 0; i < SETOFFSET_TIMES; i++)
	{
		ret = READ_MPU6050_Accel(Accel);
		if (!ret)
		{
			Sum_Ax += Accel[0];
			Sum_Ay += Accel[1];
			Sum_Az += (Accel[2] - 16384);
			SumCountA++;
		}

		ret = READ_MPU6050_Gyro(Gyro);
		if (!ret)
		{
			Sum_Gx += Gyro[0];
			Sum_Gy += Gyro[1];
			Sum_Gz += Gyro[2];
			SumCountG++;
		}
	}
	Accel_offset[0] = -(Sum_Ax / SumCountA);
	Accel_offset[1] = -(Sum_Ay / SumCountA);
	Accel_offset[2] = -(Sum_Az / SumCountA);

	Gyro_offset[0] = -(Sum_Gx / SumCountG);
	Gyro_offset[1] = -(Sum_Gy / SumCountG);
	Gyro_offset[2] = -(Sum_Gz / SumCountG);
}


#endif



int8_t READ_MPU6050_TEMP(__packed float *pTemp)
{
	int8_t ret = 0;
#if DMP_ENABLE
	long Temp;
	ret = mpu_get_temperature(&Temp, 0);
	*pTemp = Temp / 65536.0;
#else
	int16_t Temp;
	ret = I2C_ReadBytes_BE(MPU6050_Addr, TEMP_OUT_H, sizeof(int16_t), (uint8_t *)&Temp);
	*pTemp = Temp / 340.0 + 36.53;
#endif
	return ret;
}

/************************************************************   
* 函数名:MP6050_WHO_AM_I   
* 描述 ：读取MPU6050设备信息
* 输入 :无   
* 输出 :无    
*/

void MPU6050_WHO_AM_I(void)
{
	u8 data = 0;
	I2C_ReadByte(MPU6050_Addr, WHO_AM_I, &data);
}
