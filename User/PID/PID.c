
#include "PID.h"
#include "IMU.h"
#include "MPU6050.h"
#include "eeprom.h"
#include "Setup.h"
#include "TIMER.h"

//参数整定找最佳， 从小到大顺序查。  
//先是比例后积分， 最后再把微分加。  
//曲线振荡很频繁， 比例度盘要放大。 
//曲线漂浮绕大弯， 比例度盘往小扳。
//曲线偏离回复慢， 积分时间往下降。  
//曲线波动周期长， 积分时间再加长。
//曲线振荡频率快， 先把微分降下来。 
//动差大来波动慢， 微分时间应加长。 
//理想曲线两个波， 前高后低四比一。 
//一看二调多分析， 调节质量不会低。

//整定方法：
//1，将内外环PID都归0，适当增加内环的P，调整P至四轴从正面朝上自然转动到正面朝下时能感受到阻力，
//且没有抖动，有抖动就应减小P，当P减小到无抖动或者轻微抖动时即可。
//2，让内环的D慢慢增加，到你用手能明显感受到转动四轴产生排斥外力的阻力即可，D能抑制P产生的振荡，
//但是D过大也会导致高频振荡，调整D至系统无振荡且能抑制外界的力即可。
//3，给内环一点点I，注意的是I的积分要在油门开启后才开始，油门关闭就清0，且必须有积分限幅。
//I推荐取越小越好，我取的是0.01，I取大了会导致系统振荡。
//4，将内环P减半，将外环P调至内环的50-70倍，根据系统产生的高频振荡降低内环的D，直至高频振荡消除即可。
//5，给外环一点点I，同3.
//6，根据实际情况对参数进行优化调整，调整过程中要注意区分各个参数的作用，时刻记住，
//P是回复力，大了会低频振荡，D是抑制力，大了会高频振荡，I是静差消除力，越小越好，大了会产生振荡。


PID Roll,Pitch,Yaw;
PID_High High;


float Roll_Set = 0,Pitch_Set = 0,Yaw_Set = 0;

void PID_init(void)
{
	u16 read_temp;
	EE_ReadVariable(ROLL_GYRO_KP_ADDR, &read_temp);
	//DMA_Buff_In_16(read_temp,ROLL_GYRO_KP_INDEX);
	stQuadrotor_State.u16_ROLL_G_Kp = read_temp;
	Roll.Gyro_Kp = ((float)read_temp) / 100.0;

	EE_ReadVariable(ROLL_GYRO_KI_ADDR, &read_temp);
	//DMA_Buff_In_16(read_temp,ROLL_GYRO_KI_INDEX);
	stQuadrotor_State.u16_ROLL_G_Ki = read_temp;
	Roll.Gyro_Ki = ((float)read_temp) / 100.0;

	EE_ReadVariable(ROLL_GYRO_KD_ADDR, &read_temp);
	//DMA_Buff_In_16(read_temp,ROLL_GYRO_KD_INDEX);
	stQuadrotor_State.u16_ROLL_G_Kd = read_temp;
	Roll.Gyro_Kd = ((float)read_temp) / 100.0;

	EE_ReadVariable(ROLL_ANGLE_KP_ADDR, &read_temp);
	//DMA_Buff_In_16(read_temp,ROLL_ANGLE_KP_INDEX);
	stQuadrotor_State.u16_ROLL_Angle_Kp = read_temp;
	Roll.angle_Kp = ((float)read_temp) / 100.0;

	EE_ReadVariable(ROLL_ANGLE_KI_ADDR, &read_temp);
	//DMA_Buff_In_16(read_temp,ROLL_ANGLE_KI_INDEX);
	stQuadrotor_State.u16_ROLL_Angle_Ki = read_temp;
	Roll.angle_Ki = ((float)read_temp) / 100.0;

	EE_ReadVariable(ROLL_ANGLE_KD_ADDR, &read_temp);
	//DMA_Buff_In_16(read_temp,ROLL_ANGLE_KD_INDEX);
	stQuadrotor_State.u16_ROLL_Angle_Kd = read_temp;
	Roll.angle_Kd = ((float)read_temp) / 100.0;


	EE_ReadVariable(PITCH_GYRO_KP_ADDR, &read_temp);
	//DMA_Buff_In_16(read_temp,PITCH_GYRO_KP_INDEX);
	stQuadrotor_State.u16_PITCH_G_Kp = read_temp;
	Pitch.Gyro_Kp = ((float)read_temp) / 100.0;

	EE_ReadVariable(PITCH_GYRO_KI_ADDR, &read_temp);
//	DMA_Buff_In_16(read_temp,PITCH_GYRO_KI_INDEX);
	stQuadrotor_State.u16_PITCH_G_Ki = read_temp;
	Pitch.Gyro_Ki = ((float)read_temp) / 100.0;

	EE_ReadVariable(PITCH_GYRO_KD_ADDR, &read_temp);
//	DMA_Buff_In_16(read_temp,PITCH_GYRO_KD_INDEX);
	stQuadrotor_State.u16_PITCH_G_Kd = read_temp;
	Pitch.Gyro_Kd = ((float)read_temp) / 100.0;

	EE_ReadVariable(PITCH_ANGLE_KP_ADDR, &read_temp);
//	DMA_Buff_In_16(read_temp,PITCH_ANGLE_KP_INDEX);
	stQuadrotor_State.u16_PITCH_Angle_Kp = read_temp;
	Pitch.angle_Kp = ((float)read_temp) / 100.0;

	EE_ReadVariable(PITCH_ANGLE_KI_ADDR, &read_temp);
//	DMA_Buff_In_16(read_temp,PITCH_ANGLE_KI_INDEX);
	stQuadrotor_State.u16_PITCH_Angle_Ki = read_temp;
	Pitch.angle_Ki = ((float)read_temp) / 100.0;

	EE_ReadVariable(PITCH_ANGLE_KD_ADDR, &read_temp);
//	DMA_Buff_In_16(read_temp,PITCH_ANGLE_KD_INDEX);
	stQuadrotor_State.u16_PITCH_Angle_Kd = read_temp;
	Pitch.angle_Kd = ((float)read_temp) / 100.0;

	//Yaw
	EE_ReadVariable(YAW_GYRO_KP_ADDR, &read_temp);
//	DMA_Buff_In_16(read_temp,YAW_GYRO_KP_INDEX);
	stQuadrotor_State.u16_YAW_G_Kp = read_temp;
	Yaw.Gyro_Kp = ((float)read_temp) / 100.0;

	EE_ReadVariable(YAW_GYRO_KI_ADDR, &read_temp);
//	DMA_Buff_In_16(read_temp,YAW_GYRO_KI_INDEX);
	stQuadrotor_State.u16_YAW_G_Ki = read_temp;
	Yaw.Gyro_Ki = ((float)read_temp) / 100.0;

	EE_ReadVariable(YAW_GYRO_KD_ADDR, &read_temp);
//	DMA_Buff_In_16(read_temp,YAW_GYRO_KD_INDEX);
	stQuadrotor_State.u16_YAW_G_Kd = read_temp;
	Yaw.Gyro_Kd = ((float)read_temp) / 100.0;
	stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
}



void PID_set(PID * PID_in,float PID_set)
{
	float Gyro_offset;

	if (stQuadrotor_State.u16_Thro > 1180)
	{
	/****************外环角度环********************/
		float angle_offset = PID_set - PID_in->angle_cur;			//计算误差
		float PID_angle_out = 0;
	
		PID_in->angle_i += angle_offset;							//积分
		
		if(PID_in->angle_i > 600)									//积分限幅，防止积分效果过大
		{
			PID_in->angle_i = 600;
		}
		else if(PID_in->angle_i < -600)
		{
			PID_in->angle_i = -600;
		}
	
		PID_in->angle_d = PID_in->angle_last - PID_in->angle_cur;	//微分
		PID_in->angle_last = PID_in->angle_cur;	   					//保存留作下次微分
	
		PID_angle_out = PID_in->angle_Kp * angle_offset + PID_in->angle_Ki * PID_in->angle_i + PID_in->angle_Kd * PID_in->angle_d;	//外环输出
	

		//PID_angle_out = PID_set;	//角速度测试
	
		/****************内环角速度环****************************/
		Gyro_offset = PID_angle_out - PID_in->Gyro_cur;			   //计算误差

		if(PID_in->Gyro_i_time++ >= 2)							   //积分时间
		{
			PID_in->Gyro_i += Gyro_offset;						   //积分
		
		
			if(PID_in->Gyro_i > 800)							   //积分限幅
			{
				PID_in->Gyro_i = 800;
			}
			else if(PID_in->Gyro_i < -800)
			{
				PID_in->Gyro_i = -800;
			}
			PID_in->Gyro_i_time = 0;
		}
			
	//  if(PID_in->Gyro_d_time ++ > 5)
	//	{
			PID_in->Gyro_d = PID_in->Gyro_last - PID_in->Gyro_cur;		//角速度微分
			PID_in->Gyro_last = PID_in->Gyro_cur;						//保存用作下一次微分
	//		PID_in->Gyro_d_time = 0;
	//	}
		
	
		PID_in->PID_out = PID_in->Gyro_Kp * Gyro_offset + PID_in->Gyro_Ki * PID_in->Gyro_i + PID_in->Gyro_Kd * PID_in->Gyro_d;	//内环输出
	}
	else
	{
		PID_in->angle_i = 0;		   //油门低于10%时不作PID控制
		PID_in->Gyro_i = 0;
		PID_in->PID_out = 0;
	}

	//DMA_Buff[TEMP3_INDEX + 2] = (u16)((int16_t)PID_in->Gyro_i);
	//DMA_Buff[TEMP4_INDEX + 2] = (u16)((int16_t)PID_in->angle_i);
	
}

void PID_Gyro_set(PID * PID_in,float PID_set)
{
	if (stQuadrotor_State.u16_Thro > 1180)
	{
		float Gyro_offset = PID_set - PID_in->Gyro_cur;

	
		/****************内环角速度环****************************/

		if(PID_in->Gyro_i_time++ >= 2)
		{
			PID_in->Gyro_i += Gyro_offset;
		
		
			if(PID_in->Gyro_i > 800)
			{
				PID_in->Gyro_i = 800;
			}
			else if(PID_in->Gyro_i < -800)
			{
				PID_in->Gyro_i = -800;
			}
			PID_in->Gyro_i_time = 0;
		}
			
	//  if(PID_in->Gyro_d_time ++ > 5)
	//	{
			PID_in->Gyro_d = PID_in->Gyro_last - PID_in->Gyro_cur;		//角速度微分
			PID_in->Gyro_last = PID_in->Gyro_cur;
	//		PID_in->Gyro_d_time = 0;
	//	}
		
	
		PID_in->PID_out = PID_in->Gyro_Kp * Gyro_offset + PID_in->Gyro_Ki * PID_in->Gyro_i + PID_in->Gyro_Kd * PID_in->Gyro_d;
	}
	else
	{
		PID_in->Gyro_i = 0;
		PID_in->PID_out = 0;
	}

	
}



//void PID_Single_set(PID_SINGLE * PID_in,float PID_set)
//{
//	if(REC_THRO > 1180)
//	{
//		float offset = PID_set - PID_in->cur;
//
//		if(PID_in->i_time++ >= 2)
//		{
//			PID_in->i += offset;
//		
//		
//			if(PID_in->i > PID_in->i_max)
//			{
//				PID_in->i = PID_in->i_max;
//			}
//			else if(PID_in->i < -PID_in->i_max)
//			{
//				PID_in->i = -PID_in->i_max;
//			}
//			PID_in->i_time = 0;
//		}
//			
//	//  if(PID_in->Gyro_d_time ++ > 5)
//	//	{
//			PID_in->d = PID_in->last - PID_in->cur;		//角速度微分
//			PID_in->last = PID_in->cur;
//	//		PID_in->Gyro_d_time = 0;
//	//	}
//		
//	
//		PID_in->PID_out = PID_in->Kp * offset + PID_in->Ki * PID_in->i + PID_in->Kd * PID_in->d;
//	
//	
//	}
//	else
//	{
//		PID_in->i = 0;
//		PID_in->PID_out = 0;
//	}
//	
//}



void PID_High_Set(void)
{
//	float High_set_speed;
//
//	High.speed_cur = (High.high_cur - High.high_last) / 0.05f;
//	High.high_last = High.high_cur;
//
//	DMA_Buff_In_16((int16_t)(High.speed_cur * 1000),HIGH_SPEED_INDEX);


	if(stQuadrotor_State.u16_Thro > 1180)
	{
	}
	else
	{
//		High.Accel_i = 0;
//		High.speed_i = 0;
		High.high_out = stQuadrotor_State.u16_Thro;
	}

	




}



