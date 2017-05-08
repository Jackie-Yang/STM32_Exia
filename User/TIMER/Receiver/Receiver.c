#include "Receiver.h"
#include "Setup.h"

static TIM_ICInitTypeDef TIM_ICInitStructure = {0};

typedef struct _TIM_Channel_
{
    TIM_TypeDef *tim;
    u16 channel;
    u16 cc;
} TIM_Channel;

const TIM_Channel Channels[] =
{
        {TIM3, TIM_Channel_1, TIM_IT_CC1}, //方向舵
        {TIM3, TIM_Channel_2, TIM_IT_CC2}, //油门
        {TIM3, TIM_Channel_3, TIM_IT_CC3}, //副翼
        {TIM3, TIM_Channel_4, TIM_IT_CC4}, //升降舵
};

typedef struct _PWM_State_
{
    u8 state;
    u16 rise;
    u16 fall;
    //  u16 capture;
} PWM_State;

PWM_State Inputs[4] = {{0}};

//定时器3初始化，用于读取遥控器接收信号
void Receiver_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    u8 i;

    //时基设置
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);             //72分频，每次计数1us,即使实在APB1上的计时器，其频率也为72MHZ
    TIM_TimeBaseStructure.TIM_Period = 0xffff;                  //从0计时到0xFFFF，
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    //PWM输入捕获

    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;     //上升沿触发
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //管脚与寄存器对应关系
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;           //输入预分频。意思是控制在多少个输入周期做一次捕获，如果
    //输入的信号频率没有变，测得的周期也不会变。比如选择4分频，则每四个输入周期才做一次捕获，这样在输入信号变化不频繁的情况下，
    //可以减少软件被不断中断的次数。

    TIM_ICInitStructure.TIM_ICFilter = 0x0; //滤波设置，经历几个周期跳变认定波形稳定0x0～0xF

    for (i = 0; i < 4; i++)
    {
        TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
        TIM_ICInit(TIM3, &TIM_ICInitStructure);
    }
    //通道选择

    //	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);                //选择IC2为始终触发源

    //	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);				//TIM从模式：触发信号的上升沿重新初始化计数器和触发寄存器的更新事件

    //	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable); //启动定时器的被动触发

    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE); //打开捕获中断

    TIM_Cmd(TIM3, ENABLE); //启动TIM3
}

//定时器3捕获接受器产生的边沿信号产生中断
void TIM3_IRQHandler(void)
{
    u8 i;
    u16 val = 0;
    __packed uint16_t *pData = &stQuadrotor_State.u16_Rudd;
    for (i = 0; i < 4; i++)
    {
        //struct TIM_Channel channel = Channels[i];
        //PWM_State *state = &Inputs[i];

        if (TIM_GetITStatus(TIM3, Channels[i].cc) == SET)
        {
            TIM_ClearITPendingBit(TIM3, Channels[i].cc); //清除相应通道中断等待位

            switch (Channels[i].channel)
            {
            case TIM_Channel_1:
                val = TIM_GetCapture1(TIM3); //获取输入捕获的值
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

            if (Inputs[i].state == 0) //改为捕获下降沿
            {
                // switch states

                TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
                TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
                TIM_ICInit(TIM3, &TIM_ICInitStructure);

                Inputs[i].state = 1;
                Inputs[i].rise = val; //记录下上升沿开始时计数器的值
            }
            else //改为捕获上升沿
            {

                TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
                TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
                TIM_ICInit(TIM3, &TIM_ICInitStructure);

                Inputs[i].state = 0;
                Inputs[i].fall = val; //记录下降沿来临时记数器的值

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
