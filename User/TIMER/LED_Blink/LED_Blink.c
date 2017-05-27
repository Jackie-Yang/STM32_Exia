#include "LED_Blink.h"
#include "stdlib.h"

//���٣�50ms
//�̣�100ms
//�У�200ms
//����300ms

//ʹ��LED_BlinkMode�����������
//ÿ������Ԫ���ְ���2��Ԫ��
//{BLINK_ON, xxx}����{BLINK_OFF, xxx}          ��ʾ������xxx����
//{BLINK_REPEAT, xxx}         ��ʾ�ظ�ǰ�����˸xxx�Σ����Ҫ��˸5����Ϊ5���ظ�����1�Σ�����Ϊ0Ҳ����˸һ��
//{BLINK_REPEAT_START, xxx}   ��ʾ�ظ���˸����ʼλ�ã������ָ�����0��ʼ,xxx������

//ÿ�������β��������������֮һ,������ǰ��˸ģʽ�������xxx������
//{BLINK_LOOP, xxx}      ѭ��������˸ģʽ������Ӳ���ѭ���������Ƴ�������Ĳ���ģʽ������󽫼���ѭ�����ţ�
//{LINK_STOP, xxx}       ��ģʽ������󽫴Ӳ���ѭ���������Ƴ���LED״̬����������״̬

//��Ҫ�ڶ���ж���StopBlinkͬһ��BlinkMode��������һ���ڵ��Ƴ��Ĺ���������һ���ж�ȥ�Ƴ������м�С�Ŀ��ܻ���ɳ�ͻ
//������Լ�End����˸ģʽ��ҪȥStop�������ڶ�ʱ���ж����Լ��ͷţ���������պõ���Stop��ͬʱ��ʱ�������ͷ��˿��ܻ��ͻ
//Ҫ���׽���������������һ��ɾ�����У�ɾ����ʱ��ѽڵ��������������棬ͳһɾ��

//��ʼ����
LED_BlinkMode Blink_Init[] = {
    {BLINK_ON, 200},
    {BLINK_OFF, 200},
    {BLINK_REPEAT, 10},
    {BLINK_OFF, 1000},
    {BLINK_LOOP, 0}};

//����ģʽ
LED_BlinkMode Blink_DebugMode[] = {
    {BLINK_ON, 1000},
    {BLINK_OFF, 1000},
    {BLINK_LOOP, 0}};
    
//���棬��ͨ����
LED_BlinkMode Blink_WARNING[] = {
    {BLINK_ON, 100},
    {BLINK_OFF, 100},
    {BLINK_REPEAT, 3},
    {BLINK_OFF, 1000},
    {BLINK_LOOP, 0}};
//��һ�Σ���ͨ����
LED_BlinkMode Blink_WARNING_ONCE[] = {
    {BLINK_ON, 100},
    {BLINK_OFF, 100},
    {BLINK_REPEAT, 3},
    {BLINK_OFF, 1000},
    {BLINK_STOP, 0}};
//���ش���
LED_BlinkMode Blink_ERROR[] = {
    {BLINK_ON, 50},
    {BLINK_OFF, 50},
    {BLINK_REPEAT, 3},
    {BLINK_OFF, 500},
    {BLINK_LOOP, 0}};
//��һ�Σ����ش���
LED_BlinkMode Blink_ERROR_ONCE[] = {
    {BLINK_ON, 50},
    {BLINK_OFF, 50},
    {BLINK_REPEAT, 3},
    {BLINK_OFF, 500},
    {BLINK_STOP, 0}};

//��������
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

uint32_t BlinkIndex = 0;        //����
uint32_t BlinkTimeCount = 0;    //��ʱ
uint32_t BlinkRepeatIndex = 0;  //�ظ�����
uint32_t BlinkRepeatCount = 0;  //�ظ�����
pLED_BlinkMode pCurBlinkMode = 0; //��ǰģʽ
pBlinkCycNode pCycHead = 0;     //��״����ͷ
pBlinkCycNode pCycNode = 0;     //��ǰģʽ�Ľڵ�

void LED_Blink_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};

    //��ʱ��TIM1��ʼ��
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 9999;                    //��������
    TIM_TimeBaseStructure.TIM_Prescaler = 71;                   //Ԥ��Ƶ72
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;            //�ظ���������,ֻ�и߼���ʱ����Ҫ�������õĻ����ܼ���������ж�һ��
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);             //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //���TIM1�����жϱ�־
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);  //ʹ��ָ����TIM1�ж�,��������ж�
    TIM_Cmd(TIM1, ENABLE);
}

void TIM1_UP_IRQHandler(void) //TIM1�ж�
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //���TIM1�����жϷ������
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //���TIM1�����жϱ�־

        if (pCurBlinkMode)  //��ǰ������˸ģʽ�����м�ʱ
        {
            BlinkTimeCount += 10;   //��ʱ����ʱ���ж����ó�10ms�����ÿ�μ�10
            if (BlinkTimeCount >= (pCurBlinkMode + BlinkIndex)->Time)
            {
                ++BlinkIndex;   //ʱ�䵽�����л�����һ��״̬
                BlinkTimeCount = 0;
            }
            else
            {
                return;     //ʱ��û���������л�״̬
            }
        }
        else
        {
            if (pCycNode) 
            {
                pCurBlinkMode = pCycNode->pBlinkMode; //����ģʽ����Ϊ����ͷ��ģʽ                        
                BlinkRepeatCount = 0;
                BlinkRepeatIndex = 0;
                BlinkTimeCount = 0;
                BlinkIndex = 0;
            }
            else
            {
                return; //��������ģʽ
            }
        }

        //�ظ����ƿ�ʼ��λ�ã����û���õĻ���0��ʼ
        if ((pCurBlinkMode + BlinkIndex)->Status == BLINK_REPEAT_START)
        {
            BlinkRepeatIndex = ++BlinkIndex; //���л�����һ��״̬,������Ϊ�ظ���ʼλ��
            BlinkRepeatCount = 0;
        }

        //�ظ����ƣ������ظ�BLINK_REPEATǰ������n��
        if ((pCurBlinkMode + BlinkIndex)->Status == BLINK_REPEAT)
        {
            if (++BlinkRepeatCount >= (pCurBlinkMode + BlinkIndex)->Time)
            {
                BlinkRepeatCount = 0;   //��λ�ظ�����������
                BlinkRepeatIndex = 0;
                ++BlinkIndex; //�ظ���ϣ����л�����һ��״̬
            }
            else
            {
                BlinkTimeCount = 0;
                BlinkIndex = BlinkRepeatIndex;
            }
        }

        //ѭ��������ģʽ����ģʽ����ӻ�״�����Ƴ������ǵ���Stop
        if ((pCurBlinkMode + BlinkIndex)->Status == BLINK_LOOP)
        {
            if (!pCycNode)     //��ǰ��ģʽ�Ѿ����Ƴ�
            {
                pCurBlinkMode = 0;
                return; //����ģʽ�Ѿ�ȫ�����Ƴ�������
            }
            else    //û���Ƴ���������һ��ģʽ
            {
                pCycNode = pCycNode->Next;
            }
            //����������µ�����ģʽ���ͻ����µ�����ģʽ
            pCurBlinkMode = pCycNode->pBlinkMode;
            BlinkIndex = 0;
        }
        //����LED������״̬���Ƴ���ǰ����ģʽ���л�����һ��ģʽ
        else if ((pCurBlinkMode + BlinkIndex)->Status == BLINK_STOP)
        {
            StopBlink(pCurBlinkMode); //�ͷŵ�ǰģʽ֮�󣬻��Զ��л�����һ��ģʽ������Ѿ�û��ģʽ����������pCycNode��Ϊ0
            if (!pCycNode) //��ǰ��ģʽ�Ѿ����Ƴ�
            {
                pCurBlinkMode = 0;
                return; //����ģʽ�Ѿ�ȫ�����Ƴ�������
            }
            //���ó��µ�ģʽ
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

//���µ���˸ģʽ���뵽ջ���������ǰ���������е���˸ģʽ���Ὣ��ǰ��˸ģʽ����һ�βŻ��л����µ�ģʽ
void StartBlink(pLED_BlinkMode pBlinkMode)
{
    pBlinkCycNode pNewNode = 0;
    if (pBlinkMode)
    {
        if (pCycHead)    //������ڣ�Ѱ����������ͬ��ģʽ
        {
            pBlinkCycNode pNode = pCycHead;
            do  //���������ҵ���ͬ��ģʽ���򲻲���
            {
                if (pNode->pBlinkMode == pBlinkMode)
                {
                    pCycNode = pNode;  //�������õ�ģʽ��ʼ����
                    return;
                }
                pNode = pNode->Next;
            } while (pNode != pCycHead);
            //û�ҵ���ģʽ���½��ڵ���
            pNewNode = malloc(sizeof(BlinkCycNode));
            if (!pNewNode)
            {
                return;
            }
            //���뻷״����
            pNewNode->pBlinkMode = pBlinkMode;
            pNewNode->Next = pCycHead;
            pNewNode->Pre = pCycHead->Pre;
            pCycHead->Pre->Next = pNewNode;
            pCycHead->Pre = pNewNode;
        }
        else //�������ڣ��½���һ���ڵ�
        {
            pNewNode = malloc(sizeof(BlinkCycNode));
            if (!pNewNode)
            {
                return;
            }
            pNewNode->pBlinkMode = pBlinkMode;
            pNewNode->Next = pNewNode; //��״����
            pNewNode->Pre = pNewNode;
        }
        pCycHead = pNewNode;         //�²���Ľڵ���Ϊ����ͷ
        pCycNode = pCycHead;    //�������õ�ģʽ��ʼ����
    }
}

//��Ҫ�ڶ���ж���StopBlinkͬһ��BlinkMode��������һ���ڵ��Ƴ��Ĺ���������һ���ж�ȥ�Ƴ������м�С�Ŀ��ܻ���ɳ�ͻ
//������Լ�End����˸ģʽ��ҪȥStop�������ڶ�ʱ���ж����Լ��ͷţ���������պõ���Stop��ͬʱ��ʱ�������ͷ��˿��ܻ��ͻ
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
            //Ӧ���Ȱ�ǰ��ڵ���ýڵ�Ͽ���ϵ���������������������ʱ���жϣ�pCycNode���ܻ��л������û����ȫ���Ƴ��Ľڵ㣬����ڴ����
            pNode->Pre->Next = pNode->Next;
            pNode->Next->Pre = pNode->Pre;

            if (pNode == pCycHead) //�ýڵ����׽��
            {
                if (pNode->Next == pNode) //ֻ��һ���ڵ�
                {
                    pCycHead = 0;    
                }
                else
                {
                    pCycHead = pNode->Next;  //�׽ڵ�ָ����һ��
                }
            }
            if (pNode == pCycNode) //�ýڵ��ǵ�ǰ�ڵ�
            {
                if (pNode->Next == pNode) //ֻ��һ���ڵ�
                {
                    pCycNode = 0;
                }
                else
                {
                    pCycNode = pNode->Next; //��ǰ�ڵ�ָ����һ��
                }
            }
            free(pNode);
            return;
        }
        pNode = pNode->Next;
    } while (pNode != pCycHead);
}
