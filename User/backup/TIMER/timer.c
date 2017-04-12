#include "timer.h"
u8 timer3_State = 0;

//���ʱ�䣬��ʱֻ�����ڶ�ʱ��3
struct timer
{
	u32 currentTime;
	u32 setTime;
}T3; 



void TIMx_Int_Init(TIM_TypeDef * TIMx)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = 4999; //��������	
	TIM_TimeBaseStructure.TIM_Prescaler = 7199; //Ԥ��Ƶ7200
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�				 
}




//������ʱ����
void timer3_ON(u32 time)
{
	timer3_State = 1;
    T3.setTime = time * 2;
	T3.currentTime = 0;
	if( ! LED_blink )
	{
		TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx
	}
}



void TIM3_IRQHandler(void)   //TIM3�ж�,ͬʱ���ƶ�ʱ�����Լ�LED��˸
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //���TIMx�����жϱ�־

		//��ʱ��3��ʱ����
	   	if(timer3_State)
		{
			T3.currentTime++;
			if(T3.currentTime == T3.setTime)
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_5);              //��������
				USART1_sendStr("Timer3 Time Out!!\n");
				
			}
			if(T3.currentTime > T3.setTime)
			{
				timer3_State = 0;
				GPIO_ResetBits(GPIOB, GPIO_Pin_5);
				if( ! LED_blink )			 //��LED����˸ģʽ����������0.5���رն�ʱ��
				{
					TIM_Cmd(TIM3, DISABLE);  //�ر�
				}
			}
		}


		//��ʱ��3Ҳͬʱ���ڿ��Ƶ���˸ģʽ
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
