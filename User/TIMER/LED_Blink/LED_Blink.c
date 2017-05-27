#include "LED_Blink.h"
#include "stdlib.h"

//急促：50ms
//短：100ms
//中：200ms
//长：300ms

//使用LED_BlinkMode数组进行配置
//每个数组元素又包含2个元素
//{BLINK_ON, xxx}或者{BLINK_OFF, xxx}          表示亮、灭xxx毫秒
//{BLINK_REPEAT, xxx}         表示重复前面的闪烁xxx次，如果要闪烁5次则为5，重复至少1次，设置为0也会闪烁一次
//{BLINK_REPEAT_START, xxx}   表示重复闪烁的起始位置，如果不指定则从0开始,xxx无作用

//每个数组结尾必须是以下两种之一,结束当前闪烁模式，后面的xxx无意义
//{BLINK_LOOP, xxx}      循环，该闪烁模式将不会从播放循环链表中移除，后面的播放模式播放完后将继续循环播放，
//{LINK_STOP, xxx}       该模式播放完后将从播放循环链表中移除，LED状态将保持最后的状态

//不要在多个中断中StopBlink同一个BlinkMode，否则在一个节点移除的过程中再有一个中断去移除它，有极小的可能会造成冲突
//因此能自己End的闪烁模式不要去Stop，它会在定时器中断中自己释放，否则如果刚好调用Stop的同时定时器将它释放了可能会冲突
//要彻底解决这个问题可以添加一个删除队列，删除的时候把节点加入这个队列里面，统一删除

//初始化中
LED_BlinkMode Blink_Init[] = {
    {BLINK_ON, 200},
    {BLINK_OFF, 200},
    {BLINK_REPEAT, 10},
    {BLINK_OFF, 1000},
    {BLINK_LOOP, 0}};

//调试模式
LED_BlinkMode Blink_DebugMode[] = {
    {BLINK_ON, 1000},
    {BLINK_OFF, 1000},
    {BLINK_LOOP, 0}};
    
//警告，普通错误
LED_BlinkMode Blink_WARNING[] = {
    {BLINK_ON, 100},
    {BLINK_OFF, 100},
    {BLINK_REPEAT, 3},
    {BLINK_OFF, 1000},
    {BLINK_LOOP, 0}};
//闪一次，普通错误
LED_BlinkMode Blink_WARNING_ONCE[] = {
    {BLINK_ON, 100},
    {BLINK_OFF, 100},
    {BLINK_REPEAT, 3},
    {BLINK_OFF, 1000},
    {BLINK_STOP, 0}};
//严重错误
LED_BlinkMode Blink_ERROR[] = {
    {BLINK_ON, 50},
    {BLINK_OFF, 50},
    {BLINK_REPEAT, 3},
    {BLINK_OFF, 500},
    {BLINK_LOOP, 0}};
//闪一次，严重错误
LED_BlinkMode Blink_ERROR_ONCE[] = {
    {BLINK_ON, 50},
    {BLINK_OFF, 50},
    {BLINK_REPEAT, 3},
    {BLINK_OFF, 500},
    {BLINK_STOP, 0}};

//蓝牙连接
LED_BlinkMode Blink_BT_Connect[] = {
    {BLINK_ON, 300},
    {BLINK_OFF, 300},
    {BLINK_REPEAT, 2},
    {BLINK_OFF, 2000},
    {BLINK_LOOP, 0}};

LED_BlinkMode Blink_ReceiveOrder[] = {
    {BLINK_ON, 100},
    {BLINK_OFF, 100},
    {BLINK_REPEAT, 2},
    {BLINK_OFF, 1000},
    {BLINK_STOP, 0}};

LED_BlinkMode Blink_ErrorOrder[] = {
    {BLINK_ON, 50},
    {BLINK_OFF, 50},
    {BLINK_REPEAT, 2},
    {BLINK_OFF, 1000},
    {BLINK_STOP, 0}};

uint32_t BlinkIndex = 0;        //计数
uint32_t BlinkTimeCount = 0;    //计时
uint32_t BlinkRepeatIndex = 0;  //重复计数
uint32_t BlinkRepeatCount = 0;  //重复计数
pLED_BlinkMode pCurBlinkMode = 0; //当前模式
pBlinkCycNode pCycHead = 0;     //环状链表头
pBlinkCycNode pCycNode = 0;     //当前模式的节点

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
            if (BlinkTimeCount >= (pCurBlinkMode + BlinkIndex)->Time)
            {
                ++BlinkIndex;   //时间到，灯切换到下一个状态
                BlinkTimeCount = 0;
            }
            else
            {
                return;     //时间没到，不用切换状态
            }
        }
        else
        {
            if (pCycNode) 
            {
                pCurBlinkMode = pCycNode->pBlinkMode; //闪灯模式设置为链表头的模式                        
                BlinkRepeatCount = 0;
                BlinkRepeatIndex = 0;
                BlinkTimeCount = 0;
                BlinkIndex = 0;
            }
            else
            {
                return; //仍无闪灯模式
            }
        }

        //重复闪灯开始的位置，如果没设置的话从0开始
        if ((pCurBlinkMode + BlinkIndex)->Status == BLINK_REPEAT_START)
        {
            BlinkRepeatIndex = ++BlinkIndex; //灯切换到下一个状态,并设置为重复开始位置
            BlinkRepeatCount = 0;
        }

        //重复闪灯，将会重复BLINK_REPEAT前的闪灯n次
        if ((pCurBlinkMode + BlinkIndex)->Status == BLINK_REPEAT)
        {
            if (++BlinkRepeatCount >= (pCurBlinkMode + BlinkIndex)->Time)
            {
                BlinkRepeatCount = 0;   //复位重复计数及索引
                BlinkRepeatIndex = 0;
                ++BlinkIndex; //重复完毕，灯切换到下一个状态
            }
            else
            {
                BlinkTimeCount = 0;
                BlinkIndex = BlinkRepeatIndex;
            }
        }

        //循环的闪灯模式，该模式不会从环状链表移除，除非调用Stop
        if ((pCurBlinkMode + BlinkIndex)->Status == BLINK_LOOP)
        {
            if (!pCycNode)     //当前的模式已经被移除
            {
                pCurBlinkMode = 0;
                return; //闪灯模式已经全部被移除出链表
            }
            else    //没被移除，跳到下一个模式
            {
                pCycNode = pCycNode->Next;
            }
            //如果设置了新的闪灯模式，就换成新的闪灯模式
            pCurBlinkMode = pCycNode->pBlinkMode;
            BlinkIndex = 0;
        }
        //保持LED灯最后的状态并移除当前闪灯模式，切换到下一个模式
        else if ((pCurBlinkMode + BlinkIndex)->Status == BLINK_STOP)
        {
            StopBlink(pCurBlinkMode); //释放当前模式之后，会自动切换到下一个模式，如果已经没有模式在链表中则pCycNode变为0
            if (!pCycNode) //当前的模式已经被移除
            {
                pCurBlinkMode = 0;
                return; //闪灯模式已经全部被移除出链表
            }
            //设置成新的模式
            pCurBlinkMode = pCycNode->pBlinkMode;
            BlinkIndex = 0;
        }

        if ((pCurBlinkMode + BlinkIndex)->Status == BLINK_ON) //Set First Status
        {
            LED_ON;
        }
        else if ((pCurBlinkMode + BlinkIndex)->Status == BLINK_OFF)
        {
            LED_OFF;
        }
        
    }
}

void StartBlinkNow(pLED_BlinkMode pBlinkMode)
{
    StartBlink(pBlinkMode);
    pCurBlinkMode = 0;
}

//将新的闪烁模式插入到栈顶，如果当前有正在运行的闪烁模式，会将当前闪烁模式跑完一次才会切换到新的模式
void StartBlink(pLED_BlinkMode pBlinkMode)
{
    pBlinkCycNode pNewNode = 0;
    if (pBlinkMode)
    {
        if (pCycHead)    //链表存在，寻找链表中相同的模式
        {
            pBlinkCycNode pNode = pCycHead;
            do  //若链表中找到相同的模式，则不插入
            {
                if (pNode->pBlinkMode == pBlinkMode)
                {
                    pCycNode = pNode;  //从新设置的模式开始闪灯
                    return;
                }
                pNode = pNode->Next;
            } while (pNode != pCycHead);
            //没找到该模式，新建节点存放
            pNewNode = malloc(sizeof(BlinkCycNode));
            if (!pNewNode)
            {
                return;
            }
            //插入环状链表
            pNewNode->pBlinkMode = pBlinkMode;
            pNewNode->Next = pCycHead;
            pNewNode->Pre = pCycHead->Pre;
            pCycHead->Pre->Next = pNewNode;
            pCycHead->Pre = pNewNode;
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
        pCycHead = pNewNode;         //新插入的节点作为链表头
        pCycNode = pCycHead;    //从新设置的模式开始闪灯
    }
}

//不要在多个中断中StopBlink同一个BlinkMode，否则在一个节点移除的过程中再有一个中断去移除它，有极小的可能会造成冲突
//因此能自己End的闪烁模式不要去Stop，它会在定时器中断中自己释放，否则如果刚好调用Stop的同时定时器将它释放了可能会冲突
void StopBlink(pLED_BlinkMode pBlinkMode)
{
    pBlinkCycNode pNode = pCycHead;
    if(!pNode)
    {
        return;
    }
    do
    {
        if (pNode->pBlinkMode == pBlinkMode)
        {
            //应该先把前后节点与该节点断开关系，否则在这里如果触发定时器中断，pCycNode可能会切换到这个没有完全被移除的节点，造成内存溢出
            pNode->Pre->Next = pNode->Next;
            pNode->Next->Pre = pNode->Pre;

            if (pNode == pCycHead) //该节点是首结点
            {
                if (pNode->Next == pNode) //只有一个节点
                {
                    pCycHead = 0;    
                }
                else
                {
                    pCycHead = pNode->Next;  //首节点指向下一个
                }
            }
            if (pNode == pCycNode) //该节点是当前节点
            {
                if (pNode->Next == pNode) //只有一个节点
                {
                    pCycNode = 0;
                }
                else
                {
                    pCycNode = pNode->Next; //当前节点指向下一个
                }
            }
            free(pNode);
            return;
        }
        pNode = pNode->Next;
    } while (pNode != pCycHead);
}
