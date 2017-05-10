#include "LED_Blink.h"
#include "stdlib.h"

//��ʼ����
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

//���棬��ͨ����
LED_BlinkMode Blink_WARNING[] = {
    {1, 200},
    {0, 200},
    {1, 100},
    {0, 100},
    {1, 100},
    {0, 1000},
    {BLINK_LOOP, 0}};
//���ش���
LED_BlinkMode Blink_ERROR[] = {
    {1, 100},
    {0, 100},
    {1, 100},
    {0, 100},
    {1, 100},
    {0, 1000},
    {BLINK_LOOP, 0}};

//��������
LED_BlinkMode BT_Connect[] = {
    {1, 300},
    {0, 300},
    {1, 300},
    {0, 1000},
    {BLINK_LOOP, 0}};


uint32_t BlinkCount = 0;        //����
uint32_t BlinkTimeCount = 0;    //��ʱ
pLED_BlinkMode pCurBlinkMode = 0; //��ǰģʽ
pBlinkCycNode pBlinkCyc = 0, pBlinkCycNode = 0;

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
            if (BlinkTimeCount >= (pCurBlinkMode + BlinkCount)->Time)
            {
                ++BlinkCount;   //ʱ�䵽�����л�����һ��״̬
                BlinkTimeCount = 0;
            }
            else
            {
                return;     //ʱ��û���������л�״̬
            }
        }
        else
        {
            if (pBlinkCycNode) 
            {
                pCurBlinkMode = pBlinkCycNode->pBlinkMode; //����ģʽ����Ϊ����ͷ��ģʽ
                BlinkTimeCount = 0;
                BlinkCount = 0;
            }
            else
            {
                return; //��������ģʽ
            }
        }

        //ѭ��������ģʽ����ģʽ����ӻ�״�����Ƴ������ǵ���Stop
        if ((pCurBlinkMode + BlinkCount)->Status == BLINK_LOOP)
        {
            if (!pBlinkCycNode)     //��ǰ��ģʽ�Ѿ����Ƴ�
            {
                pCurBlinkMode = 0;
                return; //����ģʽ�Ѿ�ȫ�����Ƴ�������
            }
            else    //û���Ƴ���������һ��ģʽ
            {
                pBlinkCycNode = pBlinkCycNode->Next;
            }
            //����������µ�����ģʽ���ͻ����µ�����ģʽ
            pCurBlinkMode = pBlinkCycNode->pBlinkMode;
            BlinkCount = 0;
        }
        //����LED������״̬���Ƴ���ǰ����ģʽ���л�����һ��ģʽ
        else if ((pCurBlinkMode + BlinkCount)->Status == BLINK_KEEP)
        {
            StopBlink(pCurBlinkMode); //�ͷŵ�ǰģʽ֮�󣬻��Զ��л�����һ��ģʽ������Ѿ�û��ģʽ����������pBlinkCycNode��Ϊ0
            if (!pBlinkCycNode) //��ǰ��ģʽ�Ѿ����Ƴ�
            {
                pCurBlinkMode = 0;
                return; //����ģʽ�Ѿ�ȫ�����Ƴ�������
            }
            //���ó��µ�ģʽ
            pCurBlinkMode = pBlinkCycNode->pBlinkMode;
            BlinkCount = 0;
        }
        //�ر�LED�Ʋ�������ǰ����ģʽ�����л���ջ����ģʽ
        else if ((pCurBlinkMode + BlinkCount)->Status == BLINK_STOP)
        {
            LED_OFF;
            StopBlink(pCurBlinkMode); //�ͷŵ�ǰģʽ֮�󣬻��Զ��л�����һ��ģʽ������Ѿ�û��ģʽ����������pBlinkCycNode��Ϊ0
            if (!pBlinkCycNode)       //��ǰ��ģʽ�Ѿ����Ƴ�
            {
                pCurBlinkMode = 0;
                return; //����ģʽ�Ѿ�ȫ�����Ƴ�������
            }
            //���ó��µ�ģʽ
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

//���µ���˸ģʽ���뵽ջ���������ǰ���������е���˸ģʽ���Ὣ��ǰ��˸ģʽ����һ�βŻ��л����µ�ģʽ
void StartBlink(pLED_BlinkMode pBlinkMode)
{
    pBlinkCycNode pNewNode = 0;
    if (pBlinkMode)
    {
        if (pBlinkCyc)    //������ڣ�Ѱ����������ͬ��ģʽ
        {
            pBlinkCycNode pNode = pBlinkCyc;
            do  //���������ҵ���ͬ��ģʽ���򲻲���
            {
                if (pNode->pBlinkMode == pBlinkMode)
                {
                    pBlinkCycNode = pNode;  //�������õ�ģʽ��ʼ����
                    return;
                }
                pNode = pNode->Next;
            } while (pNode != pBlinkCyc);
            //û�ҵ���ģʽ���½��ڵ���
            pNewNode = malloc(sizeof(BlinkCycNode));
            if (!pNewNode)
            {
                return;
            }
            //���뻷״����
            pNewNode->pBlinkMode = pBlinkMode;
            pNewNode->Next = pBlinkCyc;
            pNewNode->Pre = pBlinkCyc->Pre;
            pBlinkCyc->Pre->Next = pNewNode;
            pBlinkCyc->Pre = pNewNode;
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
        pBlinkCyc = pNewNode;         //�²���Ľڵ���Ϊ����ͷ
        pBlinkCycNode = pBlinkCyc;    //�������õ�ģʽ��ʼ����
    }
}

//���Լ�End����˸ģʽ��ҪȥStop�����������Լ��ͷţ���������պ�Stopʱ��ʱ�������ͷ��˿��ܻ��ͻ
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
            if (pNode == pBlinkCyc) //�ýڵ����׽��
            {
                if (pNode->Next == pNode) //ֻ��һ���ڵ�
                {
                    pBlinkCyc = 0;    
                }
                else
                {
                    pBlinkCyc = pNode->Next;  //�׽ڵ�ָ����һ��
                }
            }
            if (pNode == pBlinkCycNode) //�ýڵ��ǵ�ǰ�ڵ�
            {
                if (pNode->Next == pNode) //ֻ��һ���ڵ�
                {
                    pBlinkCycNode = 0;
                }
                else
                {
                    pBlinkCycNode = pNode->Next; //��ǰ�ڵ�ָ����һ��
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
