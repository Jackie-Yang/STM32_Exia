#include "TIMER.h"
#include "Setup.h"
#include "IMU.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MS5611.h"
#include "PID.h"
#include "KS10X.h"
#include "Receiver.h"
#include "Motor.h"
#include "LED_Blink.h"

u8 TIM4_state = 0;
u8 KS10X_check = 0;



//��ʼ����ʱ��4���ڼ�ʱ10ms
void UpdateTimer_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	//��ʱ��TIM4��ʼ��
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 9999; //��������	
	TIM_TimeBaseStructure.TIM_Prescaler = 71; //Ԥ��Ƶ72
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //���TIM4�����жϱ�־
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM4�ж�,��������ж�	
	TIM_Cmd(TIM4, ENABLE);		 
}




void TIM4_IRQHandler(void)   //TIM4�жϽ��в�������
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //���TIM4�����жϷ������
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //���TIM4�����жϱ�־
		
		//10msһ����̬����
		if(GetAttitude(stQuadrotor_State.s16_Accel,
					stQuadrotor_State.s16_Gyro,
					stQuadrotor_State.s16_HMC5883L,
					&stQuadrotor_State.f_HMC5883L_Angle,
					&stQuadrotor_State.f_Roll,
					&stQuadrotor_State.f_Pitch,
					&stQuadrotor_State.f_Yaw))
		{
			StartBlinkNow(Blink_ERROR_ONCE);
		}
		else
		{
			Pitch.Gyro_cur = (float)stQuadrotor_State.s16_Gyro[0] / 16.4;
			Roll.Gyro_cur = (float)stQuadrotor_State.s16_Gyro[1] / 16.4;
			Yaw.Gyro_cur = (float)stQuadrotor_State.s16_Gyro[2] / 16.4;

			Yaw.angle_cur = stQuadrotor_State.f_Yaw;
			Pitch.angle_cur = stQuadrotor_State.f_Pitch;
			Roll.angle_cur = stQuadrotor_State.f_Roll;
		}

		//��ȡ���������ݣ����������û���յ��źţ��򲻻���£���˴��ڿ�����û�н�����ʱ��������
		//��������������յ����ݣ���ʱ������������и��£���ʱ���ڵĿ��ƽ�������
		GetReceiverData(&stQuadrotor_State.u16_Rudd);

		Roll_Set = 50.0 * (stQuadrotor_State.u16_Aile - 1100.0) / 800.0 - 25.0;
		Pitch_Set = -(50.0 * (stQuadrotor_State.u16_Elev - 1100.0) / 800.0 - 25.0);
		Yaw_Set =  -(100.0 * (stQuadrotor_State.u16_Rudd - 1100.0) / 800.0 - 50.0);

		PID_set(&Roll,Roll_Set);
		PID_set(&Pitch,Pitch_Set);
		PID_Gyro_set(&Yaw,Yaw_Set);
	
		set_motorPWM(2,stQuadrotor_State.u16_Thro + Roll.PID_out + Yaw.PID_out);
		set_motorPWM(4,stQuadrotor_State.u16_Thro - Roll.PID_out + Yaw.PID_out);

		set_motorPWM(1,stQuadrotor_State.u16_Thro + Pitch.PID_out - Yaw.PID_out);
		set_motorPWM(3,stQuadrotor_State.u16_Thro - Pitch.PID_out - Yaw.PID_out);

		//��ʡʱ�䣬ÿ��ֻ��������һ�ֲ���
		if(KS10X_check++ >= 10)			 //ÿ100ms����һ�γ���������
		{	
			KS10X_Get_High( );
			KS10X_check = 0;
		}
		else
		{
			switch(TIM4_state++)		//ÿ10ms����һ�θ߶ȣ��¶ȸ���,��5������Ҫ����MS5611��ȡʱ�䳤�����ö�ʱ����ȡ��
			{
				case 0:
				{
					if(MS5611_AskTemperature()) //����ָ����MS5611�����¶�ֵ
					{
						TIM4_state = 3;	//ʧ�ܣ�����
					}
					break;
				}
				case 1:
				{
					if (MS5611_GetTemperature()) //10ms���ȡ����
					{
						TIM4_state = 3; //ʧ�ܣ�����
						break;
					}
					if (MS5611_AskPressure()) //����ָ����MS5611������ѹֵ
					{
						TIM4_state = 3; //ʧ�ܣ�����
						break;
					}
					break;
				}
				case 2:
				{
					if (MS5611_GetPressure()) //10ms���ȡ����
					{
						TIM4_state = 3; //ʧ�ܣ�����
					}
					else
					{ //�����¶ȡ���ѹֵ
						MS5611_CalResult(&stQuadrotor_State.f_MS5611_Press,
										 &stQuadrotor_State.f_MS5611_Temp,
										 &stQuadrotor_State.f_MS5611_HIGH);
					}
					PID_High_Set(); //���߶Ȳ������50ms
					break;									
				}
				case 3:
				{
					READ_MPU6050_TEMP(&stQuadrotor_State.f_MPU6050_Temp); //��ȡMPU6050�¶�
					TIM4_state = 0;
					break;
				}
				default:
				{
					TIM4_state = 0;
					break;
				}
	
			}
		}
	}
}

