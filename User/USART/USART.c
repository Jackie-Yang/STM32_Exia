#include "USART.h"
#include "Setup.h"
// #include "DMA.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "MS5611.h"
#include "IMU.h"
#include "TIMER.h"
#include "PID.h"
#include "eeprom.h"
#include "Motor.h"
#include "LED_Blink.h"
//#include "TIMER.h"
//#define REC_LEN 1

u8 ReceiveOrder;     //接收指令码
u8 BT_state = 0;

void USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 115200;									//设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//8位字符长度
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //使能串口中断接收
	//USART_ClearFlag(USART1, USART_FLAG_TC);

	USART_Cmd(USART1, ENABLE);		  //使能串口
	GPIO_SetBits(GPIOA, GPIO_Pin_12); //使能蓝牙 AT模式
	//GPIO_ResetBits(GPIOA,GPIO_Pin_12);	   //使能蓝牙 工作模式??
	USART_GetFlagStatus(USART1, USART_FLAG_TC); //读取TC能将其置位，解决第一字节发送失败的问题
}

void USART1_sendData_u8(u8 data)	 	//串口发送8位数据
{
    USART_SendData(USART1,data);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void USART1_sendData_u16(u16 data)	  //串口发送16位数据
{
	u8 temp;
	temp = (data & 0xFF00) >> 8;
    USART_SendData(USART1,temp);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	temp = (data & 0xFF);
	USART_SendData(USART1,temp);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void USART1_sendData_u32(u32 data)	  //串口发送16位数据
{
	u8 temp;
	temp = (data & 0xFF000000) >> 24;
    USART_SendData(USART1,temp);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	temp = (data & 0xFF0000) >> 16;
	USART_SendData(USART1,temp);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

	temp = (data & 0xFF00) >> 8;
    USART_SendData(USART1,temp);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	temp = (data & 0xFF);
	USART_SendData(USART1,temp);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}




void USART1_sendStr(u8 *data)
{
	u8 i = 0;
	while(*(data + i ))              //for(i=0;i<n;i++)
	{
		USART_SendData(USART1,*(data + i ));
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		i++;
	}
}






/**********************串口接收中断*******************************************/
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE))  	//读取接收中断标志位USART_IT_RXNE 
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE))	//直到读取完毕
		{
			ReceiveOrder = USART_ReceiveData(USART1);	//接收字符读取
			switch(ReceiveOrder)						//做出相应操作
			{
				case COMMAND_MPU6050_SETOFFSET:
				{
					// MPU6050_SetOffset(stQuadrotor_State.s16_Accel, stQuadrotor_State.s16_Gyro);
					// init_quaternion();
					#if DMP_ENABLE
					MPU6050_DMP_SelfTest();
					#else
					MPU6050_SetOffset();
					#endif
					break;
				}
				case COMMAND_SET_HIGH_REF:
				{
					MS5611_SetReference();
					High.Accel_last = 0;
					High.Accel_cur = 0;
					High.speed_last = 0;
					High.speed_cur = 0;
					High.high_last = 0;
					High.high_cur = 0;
					break;
				}
				case COMMAND_SET_THRO:
				{
	//				u8 i;
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));  //等待油门信号

	//				for(i = 0;i < 4; i++)
	//				{
	//					set_motorPWM(i + 1,receive_THRO);
	//				}

					if (stQuadrotor_State.u16_Rudd == 0) //没信号时初始化其它通道
					{
						stQuadrotor_State.u16_Rudd = 1500;
					}
					if(stQuadrotor_State.u16_Aile == 0)
					{
						stQuadrotor_State.u16_Aile = 1500;
					}

					if(stQuadrotor_State.u16_Elev == 0)
					{
						stQuadrotor_State.u16_Elev = 1500;
					}
					stQuadrotor_State.u16_Thro = 1100 + USART_ReceiveData(USART1) * 8;
					//DMA_Buff_In_16(REC_THRO,THRO_INDEX);

					//USART_ClearFlag(USART1,USART_FLAG_RXNE);
					break;
				}

				case COMMAND_SET_RUDD:
				{
	//				u8 i;
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));  //等待方向舵信号

	//				for(i = 0;i < 4; i++)
	//				{
	//					set_motorPWM(i + 1,receive_THRO);
	//				}

					stQuadrotor_State.u16_Rudd = 1100 + USART_ReceiveData(USART1) * 8;
					if (stQuadrotor_State.u16_Aile == 0) 	//没信号时初始化其它通道
					{
						stQuadrotor_State.u16_Aile = 1500;
					}

					if (stQuadrotor_State.u16_Elev == 0)
					{
						stQuadrotor_State.u16_Elev = 1500;
					}

					if (stQuadrotor_State.u16_Thro == 0) 
					{
						stQuadrotor_State.u16_Thro = 1100;
					}
					//DMA_Buff_In_16(REC_THRO,THRO_INDEX);

					//USART_ClearFlag(USART1,USART_FLAG_RXNE);
					break;
				}

				case COMMAND_SET_ELEV:
				{
	//				u8 i;
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));  //等待升降舵信号

	//				for(i = 0;i < 4; i++)
	//				{
	//					set_motorPWM(i + 1,receive_THRO);
	//				}

					stQuadrotor_State.u16_Elev = 1100 + USART_ReceiveData(USART1) * 8;
					if (stQuadrotor_State.u16_Aile == 0) //没信号时初始化其它通道
					{
						stQuadrotor_State.u16_Aile = 1500;
					}

					if (stQuadrotor_State.u16_Rudd == 0)
					{
						stQuadrotor_State.u16_Rudd = 1500;
					}

					if (stQuadrotor_State.u16_Thro == 0)
					{
						stQuadrotor_State.u16_Thro = 1100;
					}
					//DMA_Buff_In_16(REC_THRO,THRO_INDEX);

					//USART_ClearFlag(USART1,USART_FLAG_RXNE);
					break;
				}

				case COMMAND_SET_AILE:
				{
	//				u8 i;
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));  //等待副翼信号

	//				for(i = 0;i < 4; i++)
	//				{
	//					set_motorPWM(i + 1,receive_THRO);
	//				}

					stQuadrotor_State.u16_Aile = 1100 + USART_ReceiveData(USART1) * 8;
					if (stQuadrotor_State.u16_Elev == 0) //没信号时初始化其它通道
					{
						stQuadrotor_State.u16_Elev = 1500;
					}

					if (stQuadrotor_State.u16_Rudd == 0)
					{
						stQuadrotor_State.u16_Rudd = 1500;
					}

					if (stQuadrotor_State.u16_Thro == 0)
					{
						stQuadrotor_State.u16_Thro = 1100;
					}
					//DMA_Buff_In_16(REC_THRO,THRO_INDEX);

					//USART_ClearFlag(USART1,USART_FLAG_RXNE);
					break;
				}

				case COMMAND_STOP:
				{
					stQuadrotor_State.u16_Thro = 1100;
					stQuadrotor_State.u16_Aile = 1500;
					stQuadrotor_State.u16_Elev = 1500;
					stQuadrotor_State.u16_Rudd = 1500;
					set_motorPWM(1,stQuadrotor_State.u16_Thro);
					set_motorPWM(2,stQuadrotor_State.u16_Thro);
					set_motorPWM(3,stQuadrotor_State.u16_Thro);
					set_motorPWM(4,stQuadrotor_State.u16_Thro);
					break;
				}

				case COMMAND_HORIZON:
				{
					if(!stQuadrotor_State.u16_Thro)
					{
						stQuadrotor_State.u16_Thro = 1100;
					}
					stQuadrotor_State.u16_Aile = 1500;
					stQuadrotor_State.u16_Elev = 1500;
					stQuadrotor_State.u16_Rudd = 1500;
					break;
				}



				//PID整定参数输入
				case COMMAND_ROLL_GYRO_KP:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

				//	DMA_Buff_In_16(receive_Data,ROLL_GYRO_KP_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_ROLL_G_Kp = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(ROLL_GYRO_KP_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Roll.Gyro_Kp = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_ROLL_GYRO_KI:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

			//		DMA_Buff_In_16(receive_Data,ROLL_GYRO_KI_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_ROLL_G_Ki = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(ROLL_GYRO_KI_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Roll.Gyro_Ki = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_ROLL_GYRO_KD:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

			//		DMA_Buff_In_16(receive_Data,ROLL_GYRO_KD_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_ROLL_G_Kd = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(ROLL_GYRO_KD_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Roll.Gyro_Kd = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_ROLL_ANGLE_KP:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

			//		DMA_Buff_In_16(receive_Data,ROLL_ANGLE_KP_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_ROLL_Angle_Kp = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(ROLL_ANGLE_KP_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Roll.angle_Kp = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_ROLL_ANGLE_KI:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

		//			DMA_Buff_In_16(receive_Data,ROLL_ANGLE_KI_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_ROLL_Angle_Ki = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(ROLL_ANGLE_KI_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Roll.angle_Ki = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_ROLL_ANGLE_KD:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

			//		DMA_Buff_In_16(receive_Data,ROLL_ANGLE_KD_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_ROLL_Angle_Kd = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(ROLL_ANGLE_KD_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Roll.angle_Kd = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_PITCH_GYRO_KP:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

			//		DMA_Buff_In_16(receive_Data,PITCH_GYRO_KP_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_PITCH_G_Kp = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(PITCH_GYRO_KP_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Pitch.Gyro_Kp = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_PITCH_GYRO_KI:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

		//			DMA_Buff_In_16(receive_Data,PITCH_GYRO_KI_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_PITCH_G_Ki = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(PITCH_GYRO_KI_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Pitch.Gyro_Ki = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_PITCH_GYRO_KD:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

		//			DMA_Buff_In_16(receive_Data,PITCH_GYRO_KD_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_PITCH_G_Kd = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(PITCH_GYRO_KD_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Pitch.Gyro_Kd = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_PITCH_ANGLE_KP:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

					//DMA_Buff_In_16(receive_Data,PITCH_ANGLE_KP_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_PITCH_Angle_Kp = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(PITCH_ANGLE_KP_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Pitch.angle_Kp = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_PITCH_ANGLE_KI:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

				//	DMA_Buff_In_16(receive_Data,PITCH_ANGLE_KI_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_PITCH_Angle_Ki = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(PITCH_ANGLE_KI_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Pitch.angle_Ki = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_PITCH_ANGLE_KD:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

			//		DMA_Buff_In_16(receive_Data,PITCH_ANGLE_KD_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_PITCH_Angle_Kd = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(PITCH_ANGLE_KD_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Pitch.angle_Kd = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_YAW_GYRO_KP:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

			//		DMA_Buff_In_16(receive_Data,YAW_GYRO_KP_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_YAW_G_Kp = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(YAW_GYRO_KP_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Yaw.Gyro_Kp = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_YAW_GYRO_KI:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

			//		DMA_Buff_In_16(receive_Data,YAW_GYRO_KI_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_YAW_G_Ki = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(YAW_GYRO_KI_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Yaw.Gyro_Ki = ((float)receive_Data) / 100.0;		   //
					break;
				}
				case COMMAND_YAW_GYRO_KD:
				{
					u16 receive_Data = 0;

					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
					receive_Data = USART_ReceiveData(USART1);
					while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
					receive_Data |= USART_ReceiveData(USART1) << 8;

			//		DMA_Buff_In_16(receive_Data,YAW_GYRO_KD_INDEX);		//写入DMA回传到上位机
					stQuadrotor_State.u16_YAW_G_Kd = receive_Data;
					if(!DebugMode)
					{
						EE_WriteVariable(YAW_GYRO_KD_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
					}
					Yaw.Gyro_Kd = ((float)receive_Data) / 100.0;		   //
					break;
				}
				default:
				{
					StartBlinkNow(Blink_ErrorOrder);
					return;
				}
			}
		}
		StartBlinkNow(Blink_ReceiveOrder);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE); //清除中断标志位
	}  
}






void check_BT(void)      //检查蓝牙状态
{
	BT_state = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
	if (BT_state)
	{
		//TIM_Cmd(TIM4, ENABLE);  			//使能TIM4,开始数据传输
		DMA_Cmd(DMA1_Channel4, ENABLE);
		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
		StartBlink(Blink_BT_Connect);
		// LED_ON;
	}
	else
	{
		//	TIM_Cmd(TIM4, DISABLE);
		DMA_Cmd(DMA1_Channel4, DISABLE);
		USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
		StopBlink(Blink_BT_Connect);
		// LED_OFF;
	}
}

/************************蓝牙开启/断开触发中断*************************************/
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line11)!= RESET)	 //判断是否发生中断
	{
	  check_BT();		//更新蓝牙状态				 
	}
	EXTI_ClearITPendingBit(EXTI_Line11);		     //清除中断标志			
}


