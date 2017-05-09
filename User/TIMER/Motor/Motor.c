#include "Motor.h"
#include "Setup.h"

static TIM_OCInitTypeDef PWM_TIM_OCInitStructure = {0};
#define MOTOR_PWM_MAC 1900
#define MOTOR_PWM_MIN 1100

void Motor_Init(void) //定时器TIM2初始化，用于产生电机PWM信号输出
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;               //计数周期
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;               //预分频7200
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);             //根据指定的参数初始化TIMx的时间基数单位
                                                                // PWM1,2,3,4
    TIM_Cmd(TIM2, ENABLE);

    //初始化P配置WM结构体
    PWM_TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM模式2，计时值小于比较值时为无效电平，否则为有效电平
    PWM_TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    PWM_TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //低电平为有效电平
}

void set_motorPWM(u8 motor, u16 motorPWM)
{
    //	TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
    //
    //	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			 		//PWM模式2，计时值小于比较值时为无效电平，否则为有效电平
    //    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //    //TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    //    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	 		//低电平为有效电平
    // TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    if (motorPWM > MOTOR_PWM_MAC)
    {
        motorPWM = MOTOR_PWM_MAC;
    }
    //	else if(motorPWM < MOTOR_PWM_MIN)
    //	{
    //		motorPWM = motorPWM_min;
    //	}

    PWM_TIM_OCInitStructure.TIM_Pulse = motorPWM;

    switch (motor)
    {
    case 1: //1号电机对应PA3,定时器通道4
    {
        TIM_OC4Init(TIM2, &PWM_TIM_OCInitStructure);
        TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
        stQuadrotor_State.u16_Motor1 = motorPWM;
        break;
    }
    case 2: //2号电机对应PA2,定时器通道3
    {
        TIM_OC3Init(TIM2, &PWM_TIM_OCInitStructure);
        TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
        stQuadrotor_State.u16_Motor2 = motorPWM;
        break;
    }
    case 3: //3号电机对应PA0,定时器通道1
    {
        TIM_OC1Init(TIM2, &PWM_TIM_OCInitStructure);
        TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
        stQuadrotor_State.u16_Motor3 = motorPWM;
        break;
    }
    case 4: //4号电机对应PA1,定时器通道2
    {
        TIM_OC2Init(TIM2, &PWM_TIM_OCInitStructure);
        TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
        stQuadrotor_State.u16_Motor4 = motorPWM;
        break;
    }
    default:
        break;
    }
    //TIM_ARRPreloadConfig(TIM2,ENABLE);
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
}
