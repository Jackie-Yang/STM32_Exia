#include "LED_Blink.h"
#include "stdlib.h"

//初始化中
LED_BlinkMode Blink_Init[] = {
    {1, 200},
    {0, 200},
    {1, 200},
    {0, 200},
    {1, 200},
    {0, 200},
    {1, 200},
    {0, 200},
    {1, 200},
    {0, 1000},
    {BLINK_LOOP, 0}};

//警告，普通错误
LED_BlinkMode Blink_WARNING[] = {
    {1, 200},
    {0, 200},
    {1, 100},
    {0, 100},
    {1, 100},
    {0, 1000},
    {BLINK_LOOP, 0}};
//严重错误
LED_BlinkMode Blink_ERROR[] = {
    {1, 100},
    {0, 100},
    {1, 100},
    {0, 100},
    {1, 100},
    {0, 1000},
    {BLINK_LOOP, 0}};

//蓝牙连接
LED_BlinkMode BT_Connect[] = {
    {1, 300},
    {0, 300},
    {1, 300},
    {0, 1000},
    {BLINK_LOOP, 0}};


uint32_t BlinkCount = 0;        //计数
uint32_t BlinkTimeCount = 0;    //计时
pLED_BlinkMode pCurBlinkMode = 0; //当前模式
pBlinkCycNode pBlinkCyc = 0, pBlinkCycNode = 0;

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
            else
            {
                return;     //时间没到，不用切换状态
            }
        }
        else
        {
            if (pBlinkCycNode) 
            {
                pCurBlinkMode = pBlinkCycNode->pBlinkMode; //闪灯模式设置为链表头的模式
                BlinkTimeCount = 0;
                BlinkCount = 0;
            }
            else
            {
                return; //仍无闪灯模式
            }
        }

        //循环的闪灯模式，该模式不会从环状链表移除，除非调用Stop
        if ((pCurBlinkMode + BlinkCount)->Status == BLINK_LOOP)
        {
            if (!pBlinkCycNode)     //当前的模式已经被移除
            {
                pCurBlinkMode = 0;
                return; //闪灯模式已经全部被移除出链表
            }
            else    //没被移除，跳到下一个模式
            {
                pBlinkCycNode = pBlinkCycNode->Next;
            }
            //如果设置了新的闪灯模式，就换成新的闪灯模式
            pCurBlinkMode = pBlinkCycNode->pBlinkMode;
            BlinkCount = 0;
        }
        //保持LED灯最后的状态并移除当前闪灯模式，切换到下一个模式
        else if ((pCurBlinkMode + BlinkCount)->Status == BLINK_KEEP)
        {
            StopBlink(pCurBlinkMode); //释放当前模式之后，会自动切换到下一个模式，如果已经没有模式在链表中则pBlinkCycNode变为0
            if (!pBlinkCycNode) //当前的模式已经被移除
            {
                pCurBlinkMode = 0;
                return; //闪灯模式已经全部被移除出链表
            }
            //设置成新的模式
            pCurBlinkMode = pBlinkCycNode->pBlinkMode;
            BlinkCount = 0;
        }
        //关闭LED灯并结束当前闪灯模式，将切换到栈顶的模式
        else if ((pCurBlinkMode + BlinkCount)->Status == BLINK_STOP)
        {
            LED_OFF;
            StopBlink(pCurBlinkMode); //释放当前模式之后，会自动切换到下一个模式，如果已经没有模式在链表中则pBlinkCycNode变为0
            if (!pBlinkCycNode)       //当前的模式已经被移除
            {
                pCurBlinkMode = 0;
                return; //闪灯模式已经全部被移除出链表
            }
            //设置成新的模式
            pCurBlinkMode = pBlinkCycNode->pBlinkMode;
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
    pBlinkCycNode pNewNode = 0;
    if (pBlinkMode)
    {
        if (pBlinkCyc)    //链表存在，寻找链表中相同的模式
        {
            pBlinkCycNode pNode = pBlinkCyc;
            do  //若链表中找到相同的模式，则不插入
            {
                if (pNode->pBlinkMode == pBlinkMode)
                {
                    pBlinkCycNode = pNode;  //从新设置的模式开始闪灯
                    return;
                }
                pNode = pNode->Next;
            } while (pNode != pBlinkCyc);
            //没找到该模式，新建节点存放
            pNewNode = malloc(sizeof(BlinkCycNode));
            if (!pNewNode)
            {
                return;
            }
            //插入环状链表
            pNewNode->pBlinkMode = pBlinkMode;
            pNewNode->Next = pBlinkCyc;
            pNewNode->Pre = pBlinkCyc->Pre;
            pBlinkCyc->Pre->Next = pNewNode;
            pBlinkCyc->Pre = pNewNode;
        }
        else //链表不存在，新建第一个节点
        {
            pNewNode = malloc(sizeof(BlinkCycNode));
            if (!pNewNode)
            {
                return;
            }
            pNewNode->pBlinkMode = pBlinkMode;
            pNewNode->Next = pNewNode; //环状链表
            pNewNode->Pre = pNewNode;
        }
        pBlinkCyc = pNewNode;         //新插入的节点作为链表头
        pBlinkCycNode = pBlinkCyc;    //从新设置的模式开始闪灯
    }
}

//能自己End的闪烁模式不要去Stop，由于它会自己释放，否则如果刚好Stop时定时器将它释放了可能会冲突
void StopBlink(pLED_BlinkMode pBlinkMode)
{
    pBlinkCycNode pNode = pBlinkCyc;
    if(!pNode)
    {
        return;
    }
    do
    {
        if (pNode->pBlinkMode == pBlinkMode)
        {
            if (pNode == pBlinkCyc) //该节点是首结点
            {
                if (pNode->Next == pNode) //只有一个节点
                {
                    pBlinkCyc = 0;    
                }
                else
                {
                    pBlinkCyc = pNode->Next;  //首节点指向下一个
                }
            }
            if (pNode == pBlinkCycNode) //该节点是当前节点
            {
                if (pNode->Next == pNode) //只有一个节点
                {
                    pBlinkCycNode = 0;
                }
                else
                {
                    pBlinkCycNode = pNode->Next; //当前节点指向下一个
                }
            }
            pNode->Pre->Next = pNode->Next;
            pNode->Next->Pre = pNode->Pre;
            free(pNode);
            return;
        }
        pNode = pNode->Next;
    } while (pNode != pBlinkCyc);
}
