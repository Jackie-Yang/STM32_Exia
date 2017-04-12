#include "ADC.h"

u8 s_ADC_result[6] = {0};


void ADC_init(ADC_TypeDef * ADCx,uint8_t ADC_Channel)
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_DeInit(ADCx);
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode=DISABLE;				                 	//单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode=ENABLE;							//循环采集
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;			//不使用外部触发转换
    ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel=1;										//扫描通道数1
	ADC_Init(ADCx,&ADC_InitStructure);

	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	ADC_RegularChannelConfig(ADCx,ADC_Channel,1, ADC_SampleTime_55Cycles5);      //配置ADC的通道


	ADC_Cmd(ADCx,ENABLE);                              //使能ADC

	//校准
	ADC_ResetCalibration(ADCx);
	while(ADC_GetResetCalibrationStatus(ADCx));

    ADC_StartCalibration(ADCx);
	while(ADC_GetCalibrationStatus(ADCx));
	//软件触发ADC转换
	ADC_SoftwareStartConvCmd(ADCx,ENABLE);
}

u8* getADC_Result(ADC_TypeDef * ADCx)
{
	u16 ADC_result;
	float f_result;
	while( !ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) )	;        //等待ADC转换结束
	ADC_result = ADC_GetConversionValue(ADCx);
	f_result = (float)ADC_result/4096 * 3.3;
	s_ADC_result[0] = (u8)f_result + '0';
	s_ADC_result[1] = '.';
	s_ADC_result[2] = (u8)(f_result * 10) % 10 + '0';
	s_ADC_result[3] = (u8)(f_result * 100) % 10 + '0';
	s_ADC_result[4] = '\n';

	return s_ADC_result;
}

