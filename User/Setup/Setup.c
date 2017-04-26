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

//存放要存进Flash的数据的Flash虚拟地址
uint16_t VirtAddVarTab[ ] = {OFFSET_AX_ADDR,OFFSET_AY_ADDR,OFFSET_AZ_ADDR,OFFSET_GX_ADDR,OFFSET_GY_ADDR,OFFSET_GZ_ADDR,HMC5883L_OFFSET_X_ADDR,HMC5883L_OFFSET_Y_ADDR,HMC5883L_OFFSET_Z_ADDR,ROLL_GYRO_KP_ADDR,ROLL_GYRO_KI_ADDR,ROLL_GYRO_KD_ADDR,ROLL_ANGLE_KP_ADDR,ROLL_ANGLE_KI_ADDR,ROLL_ANGLE_KD_ADDR,PITCH_GYRO_KP_ADDR,PITCH_GYRO_KI_ADDR,PITCH_GYRO_KD_ADDR,PITCH_ANGLE_KP_ADDR,PITCH_ANGLE_KI_ADDR,PITCH_ANGLE_KD_ADDR,YAW_GYRO_KP_ADDR,YAW_GYRO_KI_ADDR,YAW_GYRO_KD_ADDR};
Quadrotor_State stQuadrotor_State = {0};


/********************系统初始化,所有模块初始化函数的集合****************************/
void init(void)
{
	RCC_Configuration( );
	delay_ms(500);				//上电先延时一小段时间，否则mpu6050会初始化失败

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

	MPU6050_Init(); //初始化MPU6050	 
	MS5611_Init();	
	HMC5883L_Init();
	

	init_quaternion( );			//初始化四元数
	PID_init( );		   		//初始化PID参数，从flash读取

	

	KS10X_init( );    

	check_BT();		//更新蓝牙状态

	//MPU6050_SetOffset( );

	TIM4_Init();//开启定时器4，开始参数更新
	
}


/*************************************************************************************
时钟配置，外设时钟需在此集中配置
*************************************************************************************/

void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;    // 定义枚举类型变量HSEStartUpStatus
	RCC_DeInit();                    // 复位系统时钟设置
	RCC_HSEConfig(RCC_HSE_ON);       // 开启HSE */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();   // 等待HSE起振并稳定
	// HSE是否起振成功
	if(HSEStartUpStatus == SUCCESS)
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1);                            // 选择HCLK(AHB)时钟源为SYSCLK1分频
		RCC_PCLK2Config(RCC_HCLK_Div1);                             // 选择PCLK2时钟源为HCLK(AHB)1分频
		RCC_PCLK1Config(RCC_HCLK_Div2);                             // 选择PCLK1时钟源为HCLK(AHB)2分频
		FLASH_SetLatency(FLASH_Latency_2);                          // 设FLASH延迟周期数为2
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);       // 使能FLASH预取缓冲
		
		// 选择PLL时钟源为HSE的1分频，9倍频，PLL = 8MHz * 9 = 72MHz
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);   /* 使能PLL */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);         // 等待PLL输出稳定
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);                  // 选择PLL为SYSCLK时钟源
		while(RCC_GetSYSCLKSource() != 0x08);                       // 等待PLL成为SYSCLK时钟源
	}

	//串口时钟配置
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	


	//电机PWM信号输出所用定时器2配置
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);      //时钟配置
	//接收器脉冲输入所用定时器3配置
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);      //时钟配置
	//定时器4中断时钟（定时向串口发送数据）
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);      //时钟配置


	/*GPIO时钟配置：
	GPIOA:蓝牙串口,接受器输入引脚 ,电机PWM信号输出
	GPIOB:接收器输入引脚,I2C输出引脚
	GPIOC:板载LED引脚	   */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	
	//DMA时钟配置
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
	 
	//AFIO时钟配置（复用功能时钟）
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  

	//ADC时钟,输入端口GPIOC时钟配置
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOC, ENABLE);  

 		

	//I2C端口GPIOB时钟配置
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	
	//硬件I2C时钟配置，由于使用软件I2C，不需要配置       
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);         
}



//系统IO口配置
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
//
/******************蓝牙串口引脚配置************************/

//  /* USART1 Tx (PA.09) 设置为开漏输出 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
//  /* USART1 Rx (PA.10) 设置为浮空输入 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//           蓝牙使能
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);	
//蓝牙连接状态
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

/********************接收器脉冲输入引脚	  *************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;                              
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;                              
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

/**********************电机PWM信号输出******************************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);	

 /****************I2C引脚配置：PB6_SCL,PB7_SDA****************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   	
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
  	GPIO_Init(GPIOB, &GPIO_InitStructure);


//
//  /*****************LD1, LD2, LD3 , LD4 引脚配置 *********************/	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  	GPIO_Init(GPIOD, &GPIO_InitStructure);
//
//   /*****************五向按键配置，板子带有上拉电阻，因此设为浮空输入*********************/
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  	GPIO_Init(GPIOD, &GPIO_InitStructure);
//
//	/*****************ADC引脚输入配置**************************/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	  //模拟输入
//  	GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//	/****************I2C引脚配置：PB6_SCL,PB7_SDA****************/
///*	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
//  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  	GPIO_Init(GPIOB, &GPIO_InitStructure);	   */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   	
//  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
//  	GPIO_Init(GPIOB, &GPIO_InitStructure);	
//	
//	/*****************蜂鸣器引脚**********************************/
//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;            
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        
//  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   	
//  	GPIO_Init(GPIOB, &GPIO_InitStructure);		

/**************************板载LED*************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);		  	

}



//系统中断配置
void NVIC_Configuration(void)
{ 	
  	NVIC_InitTypeDef NVIC_InitStructure;			//NVIC向量表结构体变量

	#ifdef  VECT_TAB_RAM  							//向量表基地址选择

	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);  	//将0x20000000地址作为向量表基地址(RAM)
	#else  

	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); //将0x08000000地址作为向量表基地址(FLASH)  
	#endif

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置中断组为2,则有4个抢占优先级，以及4个响应优先级

/***********************************接收器输入TIM3中断**********************/
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;                     //NVIC配置 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

		//定时器4中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;             //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //抢占优先级1级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //响应优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure); 


/*****************串口输入指令**********************************/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//配置串口1为中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 	//设置抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  	//设置响应优先级为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  	//使能串口1中断
	NVIC_Init(&NVIC_InitStructure);							  	//根据参数初始化中断寄存器

/********************蓝牙连接/断开触发中断********************************/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;           
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


 /*
	//五向按键中断配置
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;            //按键按下
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;            //按键右
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;            //按键下
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;            //按键上
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;            //按键左
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//DMA传输完成中断
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;            
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);			*/

}











void EXIT_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	//蓝牙连接/断开触发中断
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);     //配置D0引脚为中断源	
	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		        //中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	        //上升下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				        //使能中断线
	EXTI_Init(&EXTI_InitStructure);		
}





