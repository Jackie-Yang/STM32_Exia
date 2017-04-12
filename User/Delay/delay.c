#include "delay.h"

/******************************************
函数名: delay_us
形参:   无
返回值:	 无
函数功能: 系统滴答定时器，延时nus，会卡CPU
*******************************************/
void delay_us(uint32_t nus)
{
	uint32_t temp;  
    SysTick->LOAD = nus*9;       /*主频72M的情况下每计数9次为1us*/
    SysTick->VAL=0x00;           /*计数器清0,因为currrent字段被手动清零时,load将自动重装到VAL中*/  
    SysTick->CTRL = 0x01;        /*配置使异常生效,也就是计数器倒数到0时将发出异常通知*/  
    do  
    {  
       temp = SysTick->CTRL;     /*时间到了之后,该位将被硬件置1,但被查询后自动清0 */ 
    }  
    while(temp & 0x01 && !(temp &(1<<16))); /*查询*/  
    SysTick->CTRL = 0x00;       /*关闭计数器*/  
    SysTick->VAL = 0x00;        /*清空val*/  
}

/******************************************
函数名: delay_ms
形参:   无
返回值:	 无
函数功能: 系统滴答定时器，延时nus，滴答定时器为24位，最大延时2^24/9000/1000=1.8641s
*******************************************/
void delay_ms(uint16_t nms)
{
 	uint32_t temp;
	SysTick->LOAD=nms*9000;
	SysTick->VAL=0X00;
	SysTick->CTRL=0X01;
	do
	{
	 temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));
	SysTick->CTRL=0X00;
	SysTick->VAL=0x00;
}
