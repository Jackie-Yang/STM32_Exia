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

		// AHRSupdate( );				//10ms一次姿态更新
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
					MS5611_Read(MS561101BA_D2_OSR_4096);	  //发送指令让MS5611测量温度值
					break;
				}
				case 1:
				{
					D2_Temp = MS5611_Get( );			 	 //10ms后获取数据
					break;
				}
				case 2:
				{
					MS5611_Read(MS561101BA_D1_OSR_4096);	  //发送指令让MS5611测量气压值
					break;
				}
				case 3:
				{
					D1_Pres = MS5611_Get( );
					MS5611_GetPressure( );					//10ms后获取数据,计算温度、气压值	
					PID_High_Set( );			
					break;									//即高度采样间隔50ms
				}
				case 4:
				{
					READ_MPU6050_TEMP( );				   //读取MPU6050温度
					break;
				}
				case 5:
				{
					Read_HMC5883L();					//获取地磁数据（该阶段暂时没有融合地磁数据）
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

