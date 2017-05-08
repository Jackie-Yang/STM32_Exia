#include "TIMER.h"
#include "Setup.h"
#include "IMU.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MS5611.h"
#include "PID.h"
#include "KS10X.h"
#include "Motor.h"

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

		// AHRSupdate( );				//10msһ����̬����
		Read_MPU6050_DMP();

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
					MS5611_Read(MS561101BA_D2_OSR_4096);	  //����ָ����MS5611�����¶�ֵ
					break;
				}
				case 1:
				{
					D2_Temp = MS5611_Get( );			 	 //10ms���ȡ����
					break;
				}
				case 2:
				{
					MS5611_Read(MS561101BA_D1_OSR_4096);	  //����ָ����MS5611������ѹֵ
					break;
				}
				case 3:
				{
					D1_Pres = MS5611_Get( );
					MS5611_GetPressure( );					//10ms���ȡ����,�����¶ȡ���ѹֵ	
					PID_High_Set( );			
					break;									//���߶Ȳ������50ms
				}
				case 4:
				{
					READ_MPU6050_TEMP( );				   //��ȡMPU6050�¶�
					break;
				}
				case 5:
				{
					Read_HMC5883L();					//��ȡ�ش����ݣ��ý׶���ʱû���ںϵش����ݣ�
					TIM4_state = 0;
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

