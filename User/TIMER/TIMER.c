#include "TIMER.h"
#include "Setup.h"
#include "USART.h"
#include "Setup.h"
#include "IMU.h"
#include "MPU6050.h"
#include "MS5611.h"
#include "PID.h"
#include "KS10X.h"
#include "MPU6050_DMP.h"
static TIM_ICInitTypeDef  TIM_ICInitStructure = { 0 };

TIM_OCInitTypeDef PWM_TIM_OCInitStructure = { 0 };
//u32	system_time = 0;
u8 TIM4_state = 0;
u16 motorPWM_max = 1900,motorPWM_min = 1100;
u8 KS10X_check = 0;

const struct TIM_Channel 
{
    TIM_TypeDef *tim;
    u16 channel;
    u16 cc;
} Channels[] = 
{
    { TIM3, TIM_Channel_1, TIM_IT_CC1 },			 //方向舵
    { TIM3, TIM_Channel_2, TIM_IT_CC2 },			 //油门
    { TIM3, TIM_Channel_3, TIM_IT_CC3 },			 //副翼
    { TIM3, TIM_Channel_4, TIM_IT_CC4 },			 //升降舵
};


struct PWM_State Inputs[4] = { { 0 } };


void TIM2_Init(void)	 //定时器TIM2初始化，用于产生电机PWM信号输出
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1; //计数周期	
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; //预分频7200
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位	
	// PWM1,2,3,4
    TIM_Cmd(TIM2, ENABLE);

	//初始化P配置WM结构体
	PWM_TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			 		//PWM模式2，计时值小于比较值时为无效电平，否则为有效电平
    PWM_TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    PWM_TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	 		//低电平为有效电平
    
	 
}

void set_motorPWM(u8 motor,u16 motorPWM)
{
//	TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
//
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			 		//PWM模式2，计时值小于比较值时为无效电平，否则为有效电平
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    //TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	 		//低电平为有效电平
 // TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;	

	if(motorPWM > motorPWM_max)
	{
		motorPWM = motorPWM_max;
	}
//	else if(motorPWM < motorPWM_min)
//	{
//		motorPWM = motorPWM_min;
//	}

	PWM_TIM_OCInitStructure.TIM_Pulse = motorPWM;

	switch(motor)
	{
		case 1:	  //1号电机对应PA3,定时器通道4
		{
			TIM_OC4Init(TIM2, &PWM_TIM_OCInitStructure);			
    		TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
			stQuadrotor_State.Motor1 = motorPWM;
			break;
		}
		case 2:		//2号电机对应PA2,定时器通道3
		{
			TIM_OC3Init(TIM2, &PWM_TIM_OCInitStructure);
			TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
			stQuadrotor_State.Motor2 = motorPWM;
			break;
		}
		case 3:	  //3号电机对应PA0,定时器通道1
		{
			TIM_OC1Init(TIM2, &PWM_TIM_OCInitStructure);
    		TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
			stQuadrotor_State.Motor3 = motorPWM;
			break;
		}
		case 4:	 //4号电机对应PA1,定时器通道2
		{
			TIM_OC2Init(TIM2, &PWM_TIM_OCInitStructure);
			TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
			stQuadrotor_State.Motor4 = motorPWM;
			break;
		}
		default:break;
	}
	stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
   //TIM_ARRPreloadConfig(TIM2,ENABLE);
	TIM_CtrlPWMOutputs(TIM2, ENABLE);
}




//定时器3初始化，用于读取遥控器接收信号
void TIM3_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	u8 i;

	//时基设置
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);			  		//72分频，每次计数1us,即使实在APB1上的计时器，其频率也为72MHZ
    TIM_TimeBaseStructure.TIM_Period = 0xffff;				 		//从0计时到0xFFFF，
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//PWM输入捕获
	
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       //上升沿触发
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;    //管脚与寄存器对应关系
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;           //输入预分频。意思是控制在多少个输入周期做一次捕获，如果
	//输入的信号频率没有变，测得的周期也不会变。比如选择4分频，则每四个输入周期才做一次捕获，这样在输入信号变化不频繁的情况下，
	//可以减少软件被不断中断的次数。

	TIM_ICInitStructure.TIM_ICFilter = 0x0;                            //滤波设置，经历几个周期跳变认定波形稳定0x0～0xF

	for (i = 0; i < 4; i++) 
	{
            TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
            TIM_ICInit(TIM3, &TIM_ICInitStructure);
    }
	 //通道选择
	
	
//	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);                //选择IC2为始终触发源
	
	
//	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);				//TIM从模式：触发信号的上升沿重新初始化计数器和触发寄存器的更新事件
	
	
//	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable); //启动定时器的被动触发

	
	TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);	   //打开捕获中断 
	
	
	TIM_Cmd(TIM3, ENABLE);                                 //启动TIM3
  
}

//初始化定时器4用于计时10ms
void TIM4_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	//定时器TIM4初始化
	TIM_TimeBaseStructure.TIM_Period = 9999; //计数周期	
	TIM_TimeBaseStructure.TIM_Prescaler = 71; //预分频72
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除TIM4更新中断标志
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断	
	TIM_Cmd(TIM4, ENABLE);		 
}





//定时器3捕获接受器产生的边沿信号产生中断
void TIM3_IRQHandler(void)
{
	u8 i;
    u16 val = 0;
	__packed uint16_t *pData = &stQuadrotor_State.Rudd;
	for (i = 0; i < 4; i++) 
	{
        //struct TIM_Channel channel = Channels[i];
        //struct PWM_State *state = &Inputs[i];

        if (TIM_GetITStatus(TIM3, Channels[i].cc) == SET)
		{
            TIM_ClearITPendingBit(TIM3, Channels[i].cc);		//清除相应通道中断等待位

            switch (Channels[i].channel) 
			{
                case TIM_Channel_1:
                    val = TIM_GetCapture1(TIM3);			//获取输入捕获的值
                    break;
                case TIM_Channel_2:
                    val = TIM_GetCapture2(TIM3);
                    break;
                case TIM_Channel_3:
                    val = TIM_GetCapture3(TIM3);
                    break;
                case TIM_Channel_4:
                    val = TIM_GetCapture4(TIM3);
                    break;
            }



            if (Inputs[i].state == 0) 		  //改为捕获下降沿
			{
                // switch states
				
                TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
                TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
                TIM_ICInit(TIM3, &TIM_ICInitStructure);

				Inputs[i].state = 1;
				Inputs[i].rise = val;	 	//记录下上升沿开始时计数器的值
                
            } 
			else 							 //改为捕获上升沿
			{

				TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
                TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
                TIM_ICInit(TIM3, &TIM_ICInitStructure);

				Inputs[i].state = 0;
				Inputs[i].fall = val;	    //记录下降沿来临时记数器的值
			

				//计算高电平的时间
                if (Inputs[i].fall > Inputs[i].rise)
				{							  
//                  Inputs[i].capture = (Inputs[i].fall - Inputs[i].rise);
					*(pData + i) = Inputs[i].fall - Inputs[i].rise;
				}
                else
				{
//                  Inputs[i].capture = ((0xffff - Inputs[i].rise) + Inputs[i].fall);
					*(pData + i) = (0xffff - Inputs[i].rise) + Inputs[i].fall;
				}

				//REC(i) = Inputs[i].capture;			//改为直接使用DMA缓存，方便传输
				 //DMA_Buff_In_16(Inputs[i].capture,i);
                // switch state
                // ping failsafe
                //failsafeCnt = 0;        	   
            }													   
        }
    }
	stQuadrotor_State_DMA_BUFF = stQuadrotor_State;

}

void TIM4_IRQHandler(void)   //TIM4中断
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM4更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除TIM4更新中断标志

		AHRSupdate( );				//10ms一次姿态更新
		// Read_DMP();
		// Yaw.angle_cur = Dmp_Yaw;
    	// Pitch.angle_cur = Dmp_Pitch;
    	// Roll.angle_cur = Dmp_Roll;
	//    stQuadrotor_State.Roll = Dmp_Roll;
	//    stQuadrotor_State.Pitch = Dmp_Pitch;
	//    stQuadrotor_State.Yaw = Dmp_Yaw;
	//    stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
 
		Roll_Set = 50.0 * (stQuadrotor_State.Aile - 1100.0) / 800.0 - 25.0;
		Pitch_Set = -(50.0 * (stQuadrotor_State.Elev - 1100.0) / 800.0 - 25.0);
		Yaw_Set =  -(100.0 * (stQuadrotor_State.Rudd - 1100.0) / 800.0 - 50.0);

		PID_set(&Roll,Roll_Set);
		PID_set(&Pitch,Pitch_Set);
		PID_Gyro_set(&Yaw,Yaw_Set);
	
		set_motorPWM(2,stQuadrotor_State.Thro + Roll.PID_out + Yaw.PID_out);
		set_motorPWM(4,stQuadrotor_State.Thro - Roll.PID_out + Yaw.PID_out);

		set_motorPWM(1,stQuadrotor_State.Thro + Pitch.PID_out - Yaw.PID_out);
		set_motorPWM(3,stQuadrotor_State.Thro - Pitch.PID_out - Yaw.PID_out);

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
		//system_time++;
//		TOGGLE(LED);
		
//		USART1_sendData_u16(Inputs[0].capture);	   	//发送方向舵输入数据
//		USART1_sendData_u16(Inputs[1].capture);		//发送油门输入数据
//		USART1_sendData_u16(Inputs[2].capture);		//发送副翼输入数据
//		USART1_sendData_u16(Inputs[3].capture);		//发送升降舵输入数据
//
//		USART1_sendData_u16(MPU6050_Accel_X);		//发送MPU6050加速度，陀螺仪的值及偏移量
//		USART1_sendData_u16(Accel_offset_X);
//		USART1_sendData_u16(MPU6050_Accel_Y);
//		USART1_sendData_u16(Accel_offset_Y);
//		USART1_sendData_u16(MPU6050_Accel_Z);
//		USART1_sendData_u16(Accel_offset_Z);
//
//		USART1_sendData_u16(MPU6050_Gyro_X);
//		USART1_sendData_u16(Gyro_offset_X);
//		USART1_sendData_u16(MPU6050_Gyro_Y);
//		USART1_sendData_u16(Gyro_offset_Y);
//		USART1_sendData_u16(MPU6050_Gyro_Z);
//		USART1_sendData_u16(Gyro_offset_Z);
//
//		USART1_sendData_u16(MPU6050_Temperature);
//		USART1_sendData_u32((int32_t)MS5611_Temperature);
//		USART1_sendData_u32((int32_t)MS5611_Pressure);
//		USART1_sendData_u32((int32_t)(MS5611_high));
	}
}
