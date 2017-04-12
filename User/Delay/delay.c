#include "delay.h"

/******************************************
������: delay_us
�β�:   ��
����ֵ:	 ��
��������: ϵͳ�δ�ʱ������ʱnus���ῨCPU
*******************************************/
void delay_us(uint32_t nus)
{
	uint32_t temp;  
    SysTick->LOAD = nus*9;       /*��Ƶ72M�������ÿ����9��Ϊ1us*/
    SysTick->VAL=0x00;           /*��������0,��Ϊcurrrent�ֶα��ֶ�����ʱ,load���Զ���װ��VAL��*/  
    SysTick->CTRL = 0x01;        /*����ʹ�쳣��Ч,Ҳ���Ǽ�����������0ʱ�������쳣֪ͨ*/  
    do  
    {  
       temp = SysTick->CTRL;     /*ʱ�䵽��֮��,��λ����Ӳ����1,������ѯ���Զ���0 */ 
    }  
    while(temp & 0x01 && !(temp &(1<<16))); /*��ѯ*/  
    SysTick->CTRL = 0x00;       /*�رռ�����*/  
    SysTick->VAL = 0x00;        /*���val*/  
}

/******************************************
������: delay_ms
�β�:   ��
����ֵ:	 ��
��������: ϵͳ�δ�ʱ������ʱnus���δ�ʱ��Ϊ24λ�������ʱ2^24/9000/1000=1.8641s
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
