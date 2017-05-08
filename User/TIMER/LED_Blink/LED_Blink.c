#include "LED_Blink.h"
#include "stdlib.h"

//初始化中
LED_BlinkMode Blink_Init[] = {
    {1, 100},
    {0, 100},
    {BLINK_LOOP, 0}};

//蓝牙连接
LED_BlinkMode BT_Connect[] = {
    {1, 300},
    {0, 300},
    {1, 300},
    {0, 300},
    {1, 300},
    {0, 300},
    {BLINK_LOOP, 0}};


uint32_t BlinkCount = 0;        //计数
uint32_t BlinkTimeCount = 0;    //计时
pLED_BlinkMode pCurBlinkMode = 0; //当前模式
pBlinkStackNode pBlinkStack = 0;

void LED_Blink_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};

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

        if (pCurBlinkMode)  //当前存在闪烁模式，进行计时
        {
            BlinkTimeCount += 10;   //计时，定时器中断设置成10ms，因此每次加10
            if (BlinkTimeCount >= (pCurBlinkMode + BlinkCount)->Time)
            {
                ++BlinkCount;   //时间到，灯切换到下一个状态
                BlinkTimeCount = 0;
            }
        }
        else
        {
            if (pBlinkStack) //当前不存在闪烁模式，从栈顶拿
            {
                pCurBlinkMode = pBlinkStack->pBlinkMode;    //闪灯模式设置为栈中最前的模式
                BlinkTimeCount = 0;
                BlinkCount = 0;
            }
            else
            {
                return; //仍无闪灯模式
            }
        }

        //循环的闪灯模式，如果当前模式为栈顶，则继续循环，否则会设置成栈顶的模式
        if ((pCurBlinkMode + BlinkCount)->Status == BLINK_LOOP)
        {
            if (!pBlinkStack)
            {
                pCurBlinkMode = 0;
                return; //闪灯模式已经全部被移除出栈
            }
            //如果设置了新的闪灯模式，就换成新的闪灯模式
            pCurBlinkMode = pBlinkStack->pBlinkMode;
            BlinkCount = 0;
        }
        //保持LED灯最后的状态并结束当前闪灯模式，将切换到栈顶的模式
        else if ((pCurBlinkMode + BlinkCount)->Status == BLINK_KEEP)
        {
            StopBlink(pCurBlinkMode);
            if (!pBlinkStack)
            {
                pCurBlinkMode = 0;
                return; //闪灯模式已经全部被移除出栈
            }
            //设置成栈顶的模式
            pCurBlinkMode = pBlinkStack->pBlinkMode;
            BlinkCount = 0;
        }
        //关闭LED灯并结束当前闪灯模式，将切换到栈顶的模式
        else if ((pCurBlinkMode + BlinkCount)->Status == BLINK_STOP)
        {
            LED_OFF;
            StopBlink(pCurBlinkMode);
            if (!pBlinkStack)
            {
                pCurBlinkMode = 0;
                return; //闪灯模式已经全部被移除出栈
            }
            //设置成栈顶的模式
            pCurBlinkMode = pBlinkStack->pBlinkMode;
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
        
    }
}

//将新的闪烁模式插入到栈顶，如果当前有正在运行的闪烁模式，会将当前闪烁模式跑完一次才会切换到新的模式
void StartBlink(pLED_BlinkMode pBlinkMode)
{
    pBlinkStackNode pPreNode = 0;
    pBlinkStackNode pNode = pBlinkStack;
    if (pBlinkMode)
    {
        while (pNode)   //若链表中找到该节点，取出来放到链表头，保证链表中的模式不重复
        {
            if (pNode->pBlinkMode == pBlinkMode)
            {
                //将该节点从链表中除去
                if (pPreNode)
                {
                    pPreNode->Next = pNode->Next;
                }
                else //该节点是首结点
                {
                    pBlinkStack = pNode->Next;
                }
                break;
            }
            pPreNode = pNode;
            pNode = pNode->Next;
        }

        if (!pNode) //链表中没有该节点，分配节点
        {
            pNode = malloc(sizeof(BlinkStackNode));
            if (!pNode)
            {
                return;
            }
        }
        
        pNode->pBlinkMode = pBlinkMode;
        pNode->Next = pBlinkStack;
        pBlinkStack = pNode;
    }
}

//能自己End的闪烁模式不要去Stop，由于它会自己释放，否则如果刚好Stop时定时器将它释放了可能会冲突
void StopBlink(pLED_BlinkMode pBlinkMode)
{
    pBlinkStackNode pPreNode = 0;
    pBlinkStackNode pNode = pBlinkStack;
    while (pNode)
    {
        if (pNode->pBlinkMode == pBlinkMode)
        {
            //将该节点从链表中除去
            if (pPreNode)
            {
                pPreNode->Next = pNode->Next;
            }
            else    //该节点是首结点
            {
                pBlinkStack = pNode->Next;
            }
            free(pNode);
            return;
        }
        pPreNode = pNode;
        pNode = pNode->Next;
    }
}
