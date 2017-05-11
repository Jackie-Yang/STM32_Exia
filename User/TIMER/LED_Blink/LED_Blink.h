#ifndef LED_BLINK_H
#define LED_BLINK_H
#include "stm32f10x.h"
#define LED_OFF GPIO_SetBits(GPIOC,GPIO_Pin_13)
#define LED_ON	GPIO_ResetBits(GPIOC,GPIO_Pin_13)

/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)				{p->BSRR=i;}			//����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)				{p->BRR	=i;}			//����͵�ƽ
#define digitalToggle(p,i)		    {p->ODR ^=i;}			//�����ת״̬

#define LED	     GPIO_Pin_13

/* �궨�������յLED */
#define TOGGLE(led)		digitalToggle(GPIOC, led)
#define ON(led)				digitalHi(GPIOC, led)
#define OFF(led)			digitalLo(GPIOC, led)

#define BLINK_REPEAT_START      -1
#define BLINK_REPEAT            -2
#define BLINK_LOOP              -3
#define BLINK_STOP              -4

typedef const struct _LED_BLINK_MODE_
{
    int8_t Status;
    uint32_t Time;
} LED_BlinkMode, *pLED_BlinkMode;

typedef struct _BLINK_CYC_NODE_
{
    pLED_BlinkMode pBlinkMode;
    struct _BLINK_CYC_NODE_* Next;
    struct _BLINK_CYC_NODE_* Pre;
} BlinkCycNode, *pBlinkCycNode;

extern LED_BlinkMode Blink_Init[];
extern LED_BlinkMode Blink_BT_Connect[];
extern LED_BlinkMode Blink_WARNING[];
extern LED_BlinkMode Blink_ERROR[];
extern LED_BlinkMode Blink_WARNING_ONCE[];
extern LED_BlinkMode Blink_ERROR_ONCE[];
extern LED_BlinkMode Blink_ReceiveOrder[];
extern LED_BlinkMode Blink_ErrorOrder[];

void LED_Blink_Init(void); //��ʱ��1��ʼ��������LED��˸
//���̿�ʼ��˸�����жϵ���˸֮�����¿�ʼ
void StartBlinkNow(pLED_BlinkMode pBlinkMode);
//��ʼ��˸���ӳ٣���ǰ��˸��������
void StartBlink(pLED_BlinkMode pBlinkMode);
void StopBlink(pLED_BlinkMode pBlinkMode);
#endif
