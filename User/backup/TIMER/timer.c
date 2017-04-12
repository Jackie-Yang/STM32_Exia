#include "timer.h"
u8 timer3_State = 0;

//存放时间，暂时只能用于定时器3
struct timer
{
	u32 currentTime;
	u32 setTime;
}T3; 



void TIMx_Int_Init(TIM_TypeDef * TIMx)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = 4999; //计数周期	
	TIM_TimeBaseStructure.TIM_Prescaler = 7199; //预分频7200
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断				 
}




//开启定时功能
void timer3_ON(u32 time)
{
	timer3_State = 1;
    T3.setTime = time * 2;
	T3.currentTime = 0;
	if( ! LED_blink )
	{
		TIM_Cmd(TIM3, ENABLE);  //使能TIMx
	}
}



void TIM3_IRQHandler(void)   //TIM3中断,同时控制定时功能以及LED闪烁
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志

		//定时器3定时功能
	   	if(timer3_State)
		{
			T3.currentTime++;
			if(T3.currentTime == T3.setTime)
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_5);              //蜂鸣器响
				USART1_sendStr("Timer3 Time Out!!\n");
				
			}
			if(T3.currentTime > T3.setTime)
			{
				timer3_State = 0;
				GPIO_ResetBits(GPIOB, GPIO_Pin_5);
				if( ! LED_blink )			 //若LED非闪烁模式，蜂鸣器响0.5秒后关闭定时器
				{
					TIM_Cmd(TIM3, DISABLE);  //关闭
				}
			}
		}


		//定时器3也同时用于控制灯闪烁模式
		switch(LED_blink)
		{
			case 1:
			case 2:
			case 3:
			{
				TOGGLE(LED1)
				TOGGLE(LED2)
				TOGGLE(LED3)
				TOGGLE(LED4)
				break;
			}
			case 4:
			{
				LED_run1();
				break;
			}
			case 5:
			{
				LED_run2();
				break;
			}
			case 6:
			{
				LED_run3();
				break;
			}
			default: break;
		}

	}
}
