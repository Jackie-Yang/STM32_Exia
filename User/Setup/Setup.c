#include "Setup.h"
#include "delay.h"
#include "USART.h"
#include "TIMER.h"
#include "MPU6050.h"
#include "eeprom.h"
#include "MS5611.h"
#include "HMC5883L.h"
#include "DMA.h"
#include "IMU.h"
#include "PID.h"
#include "KS10X.h"
#include "string.h"

//���Ҫ���Flash�����ݵ�Flash�����ַ
uint16_t VirtAddVarTab[ ] = {OFFSET_AX_ADDR,OFFSET_AY_ADDR,OFFSET_AZ_ADDR,OFFSET_GX_ADDR,OFFSET_GY_ADDR,OFFSET_GZ_ADDR,HMC5883L_OFFSET_X_ADDR,HMC5883L_OFFSET_Y_ADDR,HMC5883L_OFFSET_Z_ADDR,ROLL_GYRO_KP_ADDR,ROLL_GYRO_KI_ADDR,ROLL_GYRO_KD_ADDR,ROLL_ANGLE_KP_ADDR,ROLL_ANGLE_KI_ADDR,ROLL_ANGLE_KD_ADDR,PITCH_GYRO_KP_ADDR,PITCH_GYRO_KI_ADDR,PITCH_GYRO_KD_ADDR,PITCH_ANGLE_KP_ADDR,PITCH_ANGLE_KI_ADDR,PITCH_ANGLE_KD_ADDR,YAW_GYRO_KP_ADDR,YAW_GYRO_KI_ADDR,YAW_GYRO_KD_ADDR};
Quadrotor_State stQuadrotor_State = {0};


/********************ϵͳ��ʼ��,����ģ���ʼ�������ļ���****************************/
void init(void)
{
	RCC_Configuration( );
	delay_ms(500);				//�ϵ�����ʱһС��ʱ�䣬����mpu6050���ʼ��ʧ��

	GPIO_Configuration( );	 
	USART_Configuration( );		

	memset(&stQuadrotor_State, 0, sizeof(stQuadrotor_State));
	stQuadrotor_State.DataHead = 0xFF7F;
	stQuadrotor_State.DataSize = sizeof(stQuadrotor_State) - 8;
	stQuadrotor_State.DataCheckValue = 123;
	stQuadrotor_State.DataEnd = 0xFEFF;
	DMA_Configuration(&stQuadrotor_State, sizeof(stQuadrotor_State));
		  	
	TIM2_Init( );
	TIM3_Init();	  
	
	  
	FLASH_Unlock();
	EE_Init();
		
	NVIC_Configuration( );	
	EXIT_Configuration( );

	MPU6050_Init(); //��ʼ��MPU6050	 
	MS5611_Init();	
	HMC5883L_Init();
	

	init_quaternion( );			//��ʼ����Ԫ��
	PID_init( );		   		//��ʼ��PID��������flash��ȡ

	

	KS10X_init( );    

	check_BT();		//��������״̬

	//MPU6050_SetOffset( );

	TIM4_Init();//������ʱ��4����ʼ��������
	
}


/*************************************************************************************
ʱ�����ã�����ʱ�����ڴ˼�������
*************************************************************************************/

void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;    // ����ö�����ͱ���HSEStartUpStatus
	RCC_DeInit();                    // ��λϵͳʱ������
	RCC_HSEConfig(RCC_HSE_ON);       // ����HSE */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();   // �ȴ�HSE�����ȶ�
	// HSE�Ƿ�����ɹ�
	if(HSEStartUpStatus == SUCCESS)
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1);                            // ѡ��HCLK(AHB)ʱ��ԴΪSYSCLK1��Ƶ
		RCC_PCLK2Config(RCC_HCLK_Div1);                             // ѡ��PCLK2ʱ��ԴΪHCLK(AHB)1��Ƶ
		RCC_PCLK1Config(RCC_HCLK_Div2);                             // ѡ��PCLK1ʱ��ԴΪHCLK(AHB)2��Ƶ
		FLASH_SetLatency(FLASH_Latency_2);                          // ��FLASH�ӳ�������Ϊ2
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);       // ʹ��FLASHԤȡ����
		
		// ѡ��PLLʱ��ԴΪHSE��1��Ƶ��9��Ƶ��PLL = 8MHz * 9 = 72MHz
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);   /* ʹ��PLL */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);         // �ȴ�PLL����ȶ�
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);                  // ѡ��PLLΪSYSCLKʱ��Դ
		while(RCC_GetSYSCLKSource() != 0x08);                       // �ȴ�PLL��ΪSYSCLKʱ��Դ
	}

	//����ʱ������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	


	//���PWM�ź�������ö�ʱ��2����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);      //ʱ������
	//�����������������ö�ʱ��3����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);      //ʱ������
	//��ʱ��4�ж�ʱ�ӣ���ʱ�򴮿ڷ������ݣ�
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);      //ʱ������


	/*GPIOʱ�����ã�
	GPIOA:��������,�������������� ,���PWM�ź����
	GPIOB:��������������,I2C�������
	GPIOC:����LED����	   */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	
	//DMAʱ������
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
	 
	//AFIOʱ�����ã����ù���ʱ�ӣ�
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  

	//ADCʱ��,����˿�GPIOCʱ������
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOC, ENABLE);  

 		

	//I2C�˿�GPIOBʱ������
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	
	//Ӳ��I2Cʱ�����ã�����ʹ�����I2C������Ҫ����       
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);         
}



//ϵͳIO������
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
//
/******************����������������************************/

//  /* USART1 Tx (PA.09) ����Ϊ��©��� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
//  /* USART1 Rx (PA.10) ����Ϊ�������� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//           ����ʹ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);	
//��������״̬
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

/********************������������������	  *************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;                              
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;                              
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

/**********************���PWM�ź����******************************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);	

 /****************I2C�������ã�PB6_SCL,PB7_SDA****************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   	
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
  	GPIO_Init(GPIOB, &GPIO_InitStructure);


//
//  /*****************LD1, LD2, LD3 , LD4 �������� *********************/	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  	GPIO_Init(GPIOD, &GPIO_InitStructure);
//
//   /*****************���򰴼����ã����Ӵ����������裬�����Ϊ��������*********************/
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  	GPIO_Init(GPIOD, &GPIO_InitStructure);
//
//	/*****************ADC������������**************************/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	  //ģ������
//  	GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//	/****************I2C�������ã�PB6_SCL,PB7_SDA****************/
///*	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
//  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  	GPIO_Init(GPIOB, &GPIO_InitStructure);	   */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   	
//  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
//  	GPIO_Init(GPIOB, &GPIO_InitStructure);	
//	
//	/*****************����������**********************************/
//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;            
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        
//  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   	
//  	GPIO_Init(GPIOB, &GPIO_InitStructure);		

/**************************����LED*************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);		  	

}



//ϵͳ�ж�����
void NVIC_Configuration(void)
{ 	
  	NVIC_InitTypeDef NVIC_InitStructure;			//NVIC������ṹ�����

	#ifdef  VECT_TAB_RAM  							//���������ַѡ��

	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);  	//��0x20000000��ַ��Ϊ���������ַ(RAM)
	#else  

	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); //��0x08000000��ַ��Ϊ���������ַ(FLASH)  
	#endif

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//�����ж���Ϊ2,����4����ռ���ȼ����Լ�4����Ӧ���ȼ�

/***********************************����������TIM3�ж�**********************/
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;                     //NVIC���� 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

		//��ʱ��4�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;             //TIM4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //��ռ���ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //��Ӧ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure); 


/*****************��������ָ��**********************************/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//���ô���1Ϊ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 	//������ռ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  	//������Ӧ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  	//ʹ�ܴ���1�ж�
	NVIC_Init(&NVIC_InitStructure);							  	//���ݲ�����ʼ���жϼĴ���

/********************��������/�Ͽ������ж�********************************/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;           
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


 /*
	//���򰴼��ж�����
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;            //��������
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;            //������
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;            //������
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;            //������
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;            //������
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//DMA��������ж�
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;            
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);			*/

}











void EXIT_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	//��������/�Ͽ������ж�
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);     //����D0����Ϊ�ж�Դ	
	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		        //�ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	        //�����½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				        //ʹ���ж���
	EXTI_Init(&EXTI_InitStructure);		
}





