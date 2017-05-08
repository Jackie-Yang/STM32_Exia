#include "LED_Blink.h"

//初始化中
const LED_BlinkMode Blink_Init[] = {
    {1, 100},
    {0, 100},
    {BLINK_LOOP, 0}};

//蓝牙连接
const LED_BlinkMode BT_Connect[] = {
    {1, 300},
    {0, 300},
    {1, 300},
    {0, 300},
    {1, 300},
    {0, 300},
    {BLINK_STOP, 0}};

const LED_BlinkMode BT_DisConnect[] = {
    {1, 100},
    {0, 100},
    {1, 100},
    {0, 100},
    {1, 100},
    {0, 100},
    {BLINK_STOP, 0}};

uint32_t BlinkCount = 0;        //计数
uint32_t BlinkTimeCount = 0;    //计时
const LED_BlinkMode *pCurBlinkMode = 0; //当前模式

void LED_Blink_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    //定时器TIM1初始化
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 9999;                    //计数周期
    TIM_TimeBaseStructure.TIM_Prescaler = 71;                   //预分频72
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;            //重复计数设置,只有高级定时器需要，不设置的话可能几次溢出才中断一次
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);             //根据指定的参数初始化TIMx的时间基数单位

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //清除TIM1更新中断标志
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);  //使能指定的TIM1中断,允许更新中断
    TIM_Cmd(TIM1, ENABLE);
}

void TIM1_UP_IRQHandler(void) //TIM1中断
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //检查TIM1更新中断发生与否
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //清除TIM1更新中断标志
        if (pCurBlinkMode)
        {
            BlinkTimeCount += 10;
            if (BlinkTimeCount >= (pCurBlinkMode + BlinkCount)->Time)
            {
                ++BlinkCount;
                BlinkTimeCount = 0;
                if ((pCurBlinkMode + BlinkCount)->Status == BLINK_LOOP)
                {
                    BlinkCount = 0;
                }
                if ((pCurBlinkMode + BlinkCount)->Status == 1) //Set First Status
                {
                    LED_ON;
                }
                else if ((pCurBlinkMode + BlinkCount)->Status == 0)
                {
                    LED_OFF;
                }
                else if ((pCurBlinkMode + BlinkCount)->Status == BLINK_KEEP)    //Keep Last Status
                {
                    BlinkCount = 0;
                    BlinkTimeCount = 0;
                    pCurBlinkMode = 0;
                }
                else //if ((pCurBlinkMode + BlinkCount)->Status == BLINK_STOP)
                {
                    BlinkCount = 0;
                    BlinkTimeCount = 0;
                    pCurBlinkMode = 0;
                    LED_OFF;
                }
            }
        }
    }
}

void SetLEDBlinkMode(const LED_BlinkMode *pBlinkMode)
{
    //复位闪烁模式，闪烁计数、闪烁计时
    pCurBlinkMode = pBlinkMode;
    BlinkCount = 0;
    BlinkTimeCount = 0;
    if (pCurBlinkMode)
    {
        if (pCurBlinkMode->Status == 1) //Set First Status
        {
            LED_ON;
        }
        else if (pCurBlinkMode->Status == 0)
        {
            LED_OFF;
        }
    }
    else    //Empty Mode，Turn Off LED
    {
        LED_OFF;
    }
}
