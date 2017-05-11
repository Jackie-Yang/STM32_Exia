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



//初始化定时器4用于计时10ms
void UpdateTimer_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	//定时器TIM4初始化
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 9999; //计数周期	
	TIM_TimeBaseStructure.TIM_Prescaler = 71; //预分频72
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除TIM4更新中断标志
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断	
	TIM_Cmd(TIM4, ENABLE);		 
}




void TIM4_IRQHandler(void)   //TIM4中断进行参数更新
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM4更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除TIM4更新中断标志
		
		//10ms一次姿态更新
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

		//获取接收器数据，如果接收器没有收到信号，则不会更新，因此串口可以在没有接收器时进行设置
		//但如果接收器接收到数据，定时器将会持续进行更新，此时串口的控制将被屏蔽
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

		//节省时间，每次只进行下列一种操作
		if(KS10X_check++ >= 10)			 //每100ms进行一次超声波定高
		{	
			KS10X_Get_High( );
			KS10X_check = 0;
		}
		else
		{
			switch(TIM4_state++)		//每10ms进行一次高度，温度更新,分5步（主要由于MS5611读取时间长，才用定时器读取）
			{
				case 0:
				{
					if(MS5611_AskTemperature()) //发送指令让MS5611测量温度值
					{
						TIM4_state = 3;	//失败，跳过
					}
					break;
				}
				case 1:
				{
					if (MS5611_GetTemperature()) //10ms后获取数据
					{
						TIM4_state = 3; //失败，跳过
						break;
					}
					if (MS5611_AskPressure()) //发送指令让MS5611测量气压值
					{
						TIM4_state = 3; //失败，跳过
						break;
					}
					break;
				}
				case 2:
				{
					if (MS5611_GetPressure()) //10ms后获取数据
					{
						TIM4_state = 3; //失败，跳过
					}
					else
					{ //计算温度、气压值
						MS5611_CalResult(&stQuadrotor_State.f_MS5611_Press,
										 &stQuadrotor_State.f_MS5611_Temp,
										 &stQuadrotor_State.f_MS5611_HIGH);
					}
					PID_High_Set(); //即高度采样间隔50ms
					break;									
				}
				case 3:
				{
					READ_MPU6050_TEMP(&stQuadrotor_State.f_MPU6050_Temp); //读取MPU6050温度
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

