#include "LED.h"

u8 LED_blink = 0,LED_State = 0,LED_Dir = 1; 	  //闪烁模式及流水灯模式下哪一盏灯亮,流水灯方向

//开始LED闪烁：参数为闪烁模式(1~6)
void Blink_ON(u8 mode)
{
	LED_blink = mode;
	switch(LED_blink)
	{
		case 1:	
		{
			OFF(LED1)
			OFF(LED2)
			OFF(LED3)
			OFF(LED4)
			break;
		}
		case 2:
		{
			ON(LED1)
			ON(LED2)
		    OFF(LED3)
		 	OFF(LED4)
			break;
		}
		case 3:
		{
			ON(LED1)
			OFF(LED2)
		    ON(LED3)
		 	OFF(LED4)
			break;
		}
		case 4:
		case 6:
		{
			ON(LED1)
			OFF(LED2)
			OFF(LED3)
			OFF(LED4)
			LED_State = 1;
			LED_Dir = 1;
			break;
		}
		case 5:
		{
			OFF(LED1)
			OFF(LED2)
			OFF(LED3)
			ON(LED4)
			LED_State = 4;
			LED_Dir = 0;
			break;
		}
		default:break;
	}
	
	if( ! timer3_State )
	{
		TIM_Cmd(TIM3, ENABLE);  //使能TIM3，控制灯闪
	}
}


//正向流水灯控制函数

void LED_run1(void)	  
{
	 switch(LED_State)
	 {
	 	case 1:	  ON(LED2) OFF(LED1) break;
		case 2:   ON(LED3) OFF(LED2) break; 
		case 3:   ON(LED4) OFF(LED3) break; 
		case 4:   ON(LED1) OFF(LED4) break;
		default:break; 
	 }
	 LED_State++;
	 if(LED_State > 4)
	 {
	 	LED_State = 1;
	 }
}

//反向流水灯控制函数
void LED_run2(void)
{
	 switch(LED_State)
	 {
	 	case 1:	  ON(LED4) OFF(LED1) break;
		case 2:   ON(LED1) OFF(LED2) break; 
		case 3:   ON(LED2) OFF(LED3) break; 
		case 4:   ON(LED3) OFF(LED4) break;
		default:break; 
	 }
	 LED_State--;
	 if(LED_State == 0)
	 {
	 	LED_State = 4;
	 }
}

//双向流水灯控制函数
void LED_run3(void)
{
	if(LED_Dir)
	{
		if(LED_State == 3)
		{
			LED_Dir = 0;
		}
		LED_run1();
	}
	else
	{
		if(LED_State == 2)
		{
			LED_Dir = 1;
		}
		LED_run2();
	}
}



//停止闪烁
void Blink_OFF(void)
{
	LED_blink = 0;
	OFF(LED1)
	OFF(LED2)
	OFF(LED3)
	OFF(LED4)
	if( ! timer3_State )
	{
		TIM_Cmd(TIM3, DISABLE);  //关闭TIMx
	}
}
