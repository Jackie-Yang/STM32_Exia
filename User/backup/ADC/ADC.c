#include "ADC.h"

u8 s_ADC_result[6] = {0};


void ADC_init(ADC_TypeDef * ADCx,uint8_t ADC_Channel)
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_DeInit(ADCx);
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode=DISABLE;				                 	//��ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode=ENABLE;							//ѭ���ɼ�
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;			//��ʹ���ⲿ����ת��
    ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel=1;										//ɨ��ͨ����1
	ADC_Init(ADCx,&ADC_InitStructure);

	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	ADC_RegularChannelConfig(ADCx,ADC_Channel,1, ADC_SampleTime_55Cycles5);      //����ADC��ͨ��


	ADC_Cmd(ADCx,ENABLE);                              //ʹ��ADC

	//У׼
	ADC_ResetCalibration(ADCx);
	while(ADC_GetResetCalibrationStatus(ADCx));

    ADC_StartCalibration(ADCx);
	while(ADC_GetCalibrationStatus(ADCx));
	//�������ADCת��
	ADC_SoftwareStartConvCmd(ADCx,ENABLE);
}

u8* getADC_Result(ADC_TypeDef * ADCx)
{
	u16 ADC_result;
	float f_result;
	while( !ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) )	;        //�ȴ�ADCת������
	ADC_result = ADC_GetConversionValue(ADCx);
	f_result = (float)ADC_result/4096 * 3.3;
	s_ADC_result[0] = (u8)f_result + '0';
	s_ADC_result[1] = '.';
	s_ADC_result[2] = (u8)(f_result * 10) % 10 + '0';
	s_ADC_result[3] = (u8)(f_result * 100) % 10 + '0';
	s_ADC_result[4] = '\n';

	return s_ADC_result;
}

