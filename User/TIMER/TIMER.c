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
    { TIM3, TIM_Channel_1, TIM_IT_CC1 },			 //�����
    { TIM3, TIM_Channel_2, TIM_IT_CC2 },			 //����
    { TIM3, TIM_Channel_3, TIM_IT_CC3 },			 //����
    { TIM3, TIM_Channel_4, TIM_IT_CC4 },			 //������
};


struct PWM_State Inputs[4] = { { 0 } };


void TIM2_Init(void)	 //��ʱ��TIM2��ʼ�������ڲ������PWM�ź����
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1; //��������	
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; //Ԥ��Ƶ7200
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ	
	// PWM1,2,3,4
    TIM_Cmd(TIM2, ENABLE);

	//��ʼ��P����WM�ṹ��
	PWM_TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			 		//PWMģʽ2����ʱֵС�ڱȽ�ֵʱΪ��Ч��ƽ������Ϊ��Ч��ƽ
    PWM_TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    PWM_TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	 		//�͵�ƽΪ��Ч��ƽ
    
	 
}

void set_motorPWM(u8 motor,u16 motorPWM)
{
//	TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
//
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			 		//PWMģʽ2����ʱֵС�ڱȽ�ֵʱΪ��Ч��ƽ������Ϊ��Ч��ƽ
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    //TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	 		//�͵�ƽΪ��Ч��ƽ
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
		case 1:	  //1�ŵ����ӦPA3,��ʱ��ͨ��4
		{
			TIM_OC4Init(TIM2, &PWM_TIM_OCInitStructure);			
    		TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
			stQuadrotor_State.Motor1 = motorPWM;
			break;
		}
		case 2:		//2�ŵ����ӦPA2,��ʱ��ͨ��3
		{
			TIM_OC3Init(TIM2, &PWM_TIM_OCInitStructure);
			TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
			stQuadrotor_State.Motor2 = motorPWM;
			break;
		}
		case 3:	  //3�ŵ����ӦPA0,��ʱ��ͨ��1
		{
			TIM_OC1Init(TIM2, &PWM_TIM_OCInitStructure);
    		TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
			stQuadrotor_State.Motor3 = motorPWM;
			break;
		}
		case 4:	 //4�ŵ����ӦPA1,��ʱ��ͨ��2
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




//��ʱ��3��ʼ�������ڶ�ȡң���������ź�
void TIM3_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	u8 i;

	//ʱ������
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);			  		//72��Ƶ��ÿ�μ���1us,��ʹʵ��APB1�ϵļ�ʱ������Ƶ��ҲΪ72MHZ
    TIM_TimeBaseStructure.TIM_Period = 0xffff;				 		//��0��ʱ��0xFFFF��
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//PWM���벶��
	
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       //�����ش���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;    //�ܽ���Ĵ�����Ӧ��ϵ
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;           //����Ԥ��Ƶ����˼�ǿ����ڶ��ٸ�����������һ�β������
	//������ź�Ƶ��û�б䣬��õ�����Ҳ����䡣����ѡ��4��Ƶ����ÿ�ĸ��������ڲ���һ�β��������������źű仯��Ƶ��������£�
	//���Լ�������������жϵĴ�����

	TIM_ICInitStructure.TIM_ICFilter = 0x0;                            //�˲����ã������������������϶������ȶ�0x0��0xF

	for (i = 0; i < 4; i++) 
	{
            TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
            TIM_ICInit(TIM3, &TIM_ICInitStructure);
    }
	 //ͨ��ѡ��
	
	
//	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);                //ѡ��IC2Ϊʼ�մ���Դ
	
	
//	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);				//TIM��ģʽ�������źŵ����������³�ʼ���������ʹ����Ĵ����ĸ����¼�
	
	
//	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable); //������ʱ���ı�������

	
	TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);	   //�򿪲����ж� 
	
	
	TIM_Cmd(TIM3, ENABLE);                                 //����TIM3
  
}

//��ʼ����ʱ��4���ڼ�ʱ10ms
void TIM4_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	//��ʱ��TIM4��ʼ��
	TIM_TimeBaseStructure.TIM_Period = 9999; //��������	
	TIM_TimeBaseStructure.TIM_Prescaler = 71; //Ԥ��Ƶ72
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //���TIM4�����жϱ�־
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM4�ж�,��������ж�	
	TIM_Cmd(TIM4, ENABLE);		 
}





//��ʱ��3��������������ı����źŲ����ж�
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
            TIM_ClearITPendingBit(TIM3, Channels[i].cc);		//�����Ӧͨ���жϵȴ�λ

            switch (Channels[i].channel) 
			{
                case TIM_Channel_1:
                    val = TIM_GetCapture1(TIM3);			//��ȡ���벶���ֵ
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



            if (Inputs[i].state == 0) 		  //��Ϊ�����½���
			{
                // switch states
				
                TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
                TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
                TIM_ICInit(TIM3, &TIM_ICInitStructure);

				Inputs[i].state = 1;
				Inputs[i].rise = val;	 	//��¼�������ؿ�ʼʱ��������ֵ
                
            } 
			else 							 //��Ϊ����������
			{

				TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
                TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
                TIM_ICInit(TIM3, &TIM_ICInitStructure);

				Inputs[i].state = 0;
				Inputs[i].fall = val;	    //��¼�½�������ʱ��������ֵ
			

				//����ߵ�ƽ��ʱ��
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

				//REC(i) = Inputs[i].capture;			//��Ϊֱ��ʹ��DMA���棬���㴫��
				 //DMA_Buff_In_16(Inputs[i].capture,i);
                // switch state
                // ping failsafe
                //failsafeCnt = 0;        	   
            }													   
        }
    }
	stQuadrotor_State_DMA_BUFF = stQuadrotor_State;

}

void TIM4_IRQHandler(void)   //TIM4�ж�
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //���TIM4�����жϷ������
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //���TIM4�����жϱ�־

		AHRSupdate( );				//10msһ����̬����
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
		
//		USART1_sendData_u16(Inputs[0].capture);	   	//���ͷ������������
//		USART1_sendData_u16(Inputs[1].capture);		//����������������
//		USART1_sendData_u16(Inputs[2].capture);		//���͸�����������
//		USART1_sendData_u16(Inputs[3].capture);		//������������������
//
//		USART1_sendData_u16(MPU6050_Accel_X);		//����MPU6050���ٶȣ������ǵ�ֵ��ƫ����
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
