#ifndef PID_H
#define PID_H
#include "stm32f10x.h"


//����PID���ƽǶȽṹ��
typedef struct PID_struct
{
	/*********�⻷�ǶȻ�����**************/
	float angle_Kp;
	float angle_Ki;
	float angle_Kd;

	float angle_cur;
	float angle_last;

	float angle_i;
	float angle_d; 
	/**********�ڻ����ٶȻ�����**************************/
	float Gyro_Kp;
	float Gyro_Ki;
	float Gyro_Kd;

	float Gyro_cur;
	float Gyro_last;

	float Gyro_i;
	float Gyro_d;
	 
	u8 Gyro_i_time;


	float PID_out;

}PID;

//����PID���ƽṹ��
typedef struct PID_SINGLE_struct
{

	/**********PID����**************************/
	float Kp;
	float Ki;
	float Kd;

	float cur;
	float last;

	float i,i_max;
	float d;
	 
	u8 i_time;


	float PID_out;
}PID_SINGLE;



typedef struct PID_HIGH_struct
{
	float high_cur;
	float high_last;
	/*********�⻷�ٶȲ���**************/
	float speed_Kp;
	float speed_Ki;
	float speed_Kd;

	float speed_cur;
	float speed_last;

	float speed_i;
	float speed_d; 
	/**********�ڻ����ٶȻ�����**************************/
	float Accel_Kp;
	float Accel_Ki;
	float Accel_Kd;

	float Accel_cur;
	float Accel_last;

	float Accel_i;
	float Accel_d;
	 
	u8 Accel_i_time;


	float high_out;

}PID_High;



void PID_init(void);
void PID_set(PID * PID_in,float PID_set);
void PID_Gyro_set(PID * PID_in,float PID_set);
//void PID_Single_set(PID_SINGLE * PID_in,float PID_set);
void PID_High_Set(void);

extern PID Roll,Pitch,Yaw;
extern float Roll_Set,Pitch_Set,Yaw_Set;

extern PID_High High;




#endif

