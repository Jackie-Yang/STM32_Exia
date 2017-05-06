
#include "PID.h"
#include "IMU.h"
#include "MPU6050.h"
#include "eeprom.h"
#include "Setup.h"
#include "TIMER.h"

//������������ѣ� ��С����˳��顣  
//���Ǳ�������֣� ����ٰ�΢�ּӡ�  
//�����񵴺�Ƶ���� ��������Ҫ�Ŵ� 
//����Ư���ƴ��䣬 ����������С�⡣
//����ƫ��ظ����� ����ʱ�����½���  
//���߲������ڳ��� ����ʱ���ټӳ���
//������Ƶ�ʿ죬 �Ȱ�΢�ֽ������� 
//��������������� ΢��ʱ��Ӧ�ӳ��� 
//���������������� ǰ�ߺ���ı�һ�� 
//һ������������� ������������͡�

//����������
//1�������⻷PID����0���ʵ������ڻ���P������P����������泯����Ȼת�������泯��ʱ�ܸ��ܵ�������
//��û�ж������ж�����Ӧ��СP����P��С���޶���������΢����ʱ���ɡ�
//2�����ڻ���D�������ӣ��������������Ը��ܵ�ת����������ų��������������ɣ�D������P�������񵴣�
//����D����Ҳ�ᵼ�¸�Ƶ�񵴣�����D��ϵͳ���������������������ɡ�
//3�����ڻ�һ���I��ע�����I�Ļ���Ҫ�����ſ�����ſ�ʼ�����Źرվ���0���ұ����л����޷���
//I�Ƽ�ȡԽСԽ�ã���ȡ����0.01��Iȡ���˻ᵼ��ϵͳ�񵴡�
//4�����ڻ�P���룬���⻷P�����ڻ���50-70��������ϵͳ�����ĸ�Ƶ�񵴽����ڻ���D��ֱ����Ƶ���������ɡ�
//5�����⻷һ���I��ͬ3.
//6������ʵ������Բ��������Ż�����������������Ҫע�����ָ������������ã�ʱ�̼�ס��
//P�ǻظ��������˻��Ƶ�񵴣�D�������������˻��Ƶ�񵴣�I�Ǿ�����������ԽСԽ�ã����˻�����񵴡�


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
	/****************�⻷�ǶȻ�********************/
		float angle_offset = PID_set - PID_in->angle_cur;			//�������
		float PID_angle_out = 0;
	
		PID_in->angle_i += angle_offset;							//����
		
		if(PID_in->angle_i > 600)									//�����޷�����ֹ����Ч������
		{
			PID_in->angle_i = 600;
		}
		else if(PID_in->angle_i < -600)
		{
			PID_in->angle_i = -600;
		}
	
		PID_in->angle_d = PID_in->angle_last - PID_in->angle_cur;	//΢��
		PID_in->angle_last = PID_in->angle_cur;	   					//���������´�΢��
	
		PID_angle_out = PID_in->angle_Kp * angle_offset + PID_in->angle_Ki * PID_in->angle_i + PID_in->angle_Kd * PID_in->angle_d;	//�⻷���
	

		//PID_angle_out = PID_set;	//���ٶȲ���
	
		/****************�ڻ����ٶȻ�****************************/
		Gyro_offset = PID_angle_out - PID_in->Gyro_cur;			   //�������

		if(PID_in->Gyro_i_time++ >= 2)							   //����ʱ��
		{
			PID_in->Gyro_i += Gyro_offset;						   //����
		
		
			if(PID_in->Gyro_i > 800)							   //�����޷�
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
			PID_in->Gyro_d = PID_in->Gyro_last - PID_in->Gyro_cur;		//���ٶ�΢��
			PID_in->Gyro_last = PID_in->Gyro_cur;						//����������һ��΢��
	//		PID_in->Gyro_d_time = 0;
	//	}
		
	
		PID_in->PID_out = PID_in->Gyro_Kp * Gyro_offset + PID_in->Gyro_Ki * PID_in->Gyro_i + PID_in->Gyro_Kd * PID_in->Gyro_d;	//�ڻ����
	}
	else
	{
		PID_in->angle_i = 0;		   //���ŵ���10%ʱ����PID����
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

	
		/****************�ڻ����ٶȻ�****************************/

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
			PID_in->Gyro_d = PID_in->Gyro_last - PID_in->Gyro_cur;		//���ٶ�΢��
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
//			PID_in->d = PID_in->last - PID_in->cur;		//���ٶ�΢��
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



