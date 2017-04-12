#include "USART.h"
#include "setup.h"
#include "DMA.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "MS5611.h"
#include "IMU.h"
#include "TIMER.h"
#include "PID.h"
#include "eeprom.h"
//#include "TIMER.h"
//#define REC_LEN 1

u8 ReceiveOrder;     //接收指令码
u8 BT_state = 0;

void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;                          //设置波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;		  //8位字符长度
  USART_InitStructure.USART_StopBits = USART_StopBits_1;			  //一位停止位			    
  USART_InitStructure.USART_Parity = USART_Parity_No;				  //无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	  //收发模式
  USART_Init(USART1, &USART_InitStructure);				   
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);							//使能串口中断接收
  //USART_ClearFlag(USART1, USART_FLAG_TC);  

  USART_Cmd(USART1, ENABLE);	             //使能串口 
  GPIO_SetBits(GPIOA,GPIO_Pin_12); 			//使能蓝牙 AT模式
  //GPIO_ResetBits(GPIOA,GPIO_Pin_12);	   //使能蓝牙 工作模式??
  USART_GetFlagStatus(USART1, USART_FLAG_TC);//读取TC能将其置位，解决第一字节发送失败的问题

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
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);	//清除中断标志位	
		ReceiveOrder = USART_ReceiveData(USART1);	//接收字符读取
		switch(ReceiveOrder)						//做出相应操作
		{
			case 0:
			{
				MPU6050_SetOffset();
				init_quaternion();
				break;
			}
			case 1:
			{
				Zero_Pressure = 0;
				High.Accel_last = 0;
				High.Accel_cur = 0;
				High.speed_last = 0;
				High.speed_cur = 0;
				High.high_last = 0;
				High.high_cur = 0;
				break;
			}
			case 2:
			{
				HMC5883L_SetOffset( );
				break;
			}
			case 3:
			{
//				u8 i;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));  //等待油门信号

//				for(i = 0;i < 4; i++)
//				{
//					set_motorPWM(i + 1,receive_THRO);
//				}

				if(REC_RUDD == 0)					//归中位
				{
					REC_RUDD = 1500;
				}
				if(REC_AILE == 0)
				{
					REC_AILE = 1500;
				}

				if(REC_ELEV == 0)
				{
					REC_ELEV = 1500;
				}
				REC_THRO = 1100 + USART_ReceiveData(USART1) * 8;
				//DMA_Buff_In_16(REC_THRO,THRO_INDEX);

				//USART_ClearFlag(USART1,USART_FLAG_RXNE);
				break;
			}

			case 4:
			{
//				u8 i;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));  //等待方向舵信号

//				for(i = 0;i < 4; i++)
//				{
//					set_motorPWM(i + 1,receive_THRO);
//				}

				REC_RUDD = 1100 + USART_ReceiveData(USART1) * 8;
				//DMA_Buff_In_16(REC_THRO,THRO_INDEX);

				//USART_ClearFlag(USART1,USART_FLAG_RXNE);
				break;
			}

			case 5:
			{
//				u8 i;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));  //等待升降舵信号

//				for(i = 0;i < 4; i++)
//				{
//					set_motorPWM(i + 1,receive_THRO);
//				}

				REC_ELEV = 1100 + USART_ReceiveData(USART1) * 8;
				//DMA_Buff_In_16(REC_THRO,THRO_INDEX);

				//USART_ClearFlag(USART1,USART_FLAG_RXNE);
				break;
			}

			case 6:
			{
//				u8 i;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));  //等待副翼信号

//				for(i = 0;i < 4; i++)
//				{
//					set_motorPWM(i + 1,receive_THRO);
//				}

				REC_AILE = 1100 + USART_ReceiveData(USART1) * 8;
				//DMA_Buff_In_16(REC_THRO,THRO_INDEX);

				//USART_ClearFlag(USART1,USART_FLAG_RXNE);
				break;
			}


			case 7:
			{
				REC_THRO = 1100;

				REC_AILE = 1500;
				REC_ELEV = 1500;
				REC_RUDD = 1500;
				set_motorPWM(1,REC_THRO);
				set_motorPWM(2,REC_THRO);
				set_motorPWM(3,REC_THRO);
				set_motorPWM(4,REC_THRO);
				break;
			}
			case 8:
			{
				REC_AILE = 1500;
				REC_ELEV = 1500;
				REC_RUDD = 1500;
				break;
			}



			//PID整定参数输入
			case 9:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

			//	DMA_Buff_In_16(receive_Data,ROLL_GYRO_KP_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(ROLL_GYRO_KP_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Roll.Gyro_Kp = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 10:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,ROLL_GYRO_KI_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(ROLL_GYRO_KI_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Roll.Gyro_Ki = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 11:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,ROLL_GYRO_KD_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(ROLL_GYRO_KD_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Roll.Gyro_Kd = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 12:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,ROLL_ANGLE_KP_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(ROLL_ANGLE_KP_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Roll.angle_Kp = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 13:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

	//			DMA_Buff_In_16(receive_Data,ROLL_ANGLE_KI_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(ROLL_ANGLE_KI_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Roll.angle_Ki = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 14:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,ROLL_ANGLE_KD_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(ROLL_ANGLE_KD_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Roll.angle_Kd = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 15:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,PITCH_GYRO_KP_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(PITCH_GYRO_KP_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Pitch.Gyro_Kp = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 16:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

	//			DMA_Buff_In_16(receive_Data,PITCH_GYRO_KI_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(PITCH_GYRO_KI_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Pitch.Gyro_Ki = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 17:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

	//			DMA_Buff_In_16(receive_Data,PITCH_GYRO_KD_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(PITCH_GYRO_KD_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Pitch.Gyro_Kd = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 18:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

				//DMA_Buff_In_16(receive_Data,PITCH_ANGLE_KP_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(PITCH_ANGLE_KP_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Pitch.angle_Kp = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 19:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

			//	DMA_Buff_In_16(receive_Data,PITCH_ANGLE_KI_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(PITCH_ANGLE_KI_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Pitch.angle_Ki = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 20:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,PITCH_ANGLE_KD_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(PITCH_ANGLE_KD_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Pitch.angle_Kd = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 21:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,YAW_GYRO_KP_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(YAW_GYRO_KP_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Yaw.Gyro_Kp = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 22:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,YAW_GYRO_KI_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(YAW_GYRO_KI_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Yaw.Gyro_Ki = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 23:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//从串口读取两字节参数
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,YAW_GYRO_KD_INDEX);		//写入DMA回传到上位机
				EE_WriteVariable(YAW_GYRO_KD_ADDR,receive_Data);	   //将参数储存进Flash，方便下次开机读取
				Yaw.Gyro_Kd = ((float)receive_Data) / 100.0;		   //
				break;
			}
	//检查某一固定参数（例如PID，校正值这些相对不会变的值，不需频繁载入DMA，需要的时候发送相应指令，返回检查值）：
			case 24:		   //检查关闭
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(0xFFFF,CHECK_DATA_INDEX);
				break;
			}
			//检查MPU6050校正数据
			case 25:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Accel_offset_X,CHECK_DATA_INDEX);
				DMA_Buff_In_16(25,CHECK_STATE_INDEX);
				break;
			}
			case 26:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Accel_offset_Y,CHECK_DATA_INDEX);
				DMA_Buff_In_16(26,CHECK_STATE_INDEX);
				break;
			}
			case 27:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Accel_offset_X,CHECK_DATA_INDEX);
				DMA_Buff_In_16(27,CHECK_STATE_INDEX);
				break;
			}
			case 28:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Gyro_offset_Y,CHECK_DATA_INDEX);
				DMA_Buff_In_16(28,CHECK_STATE_INDEX);
				break;
			}
			case 29:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Gyro_offset_Z,CHECK_DATA_INDEX);
				DMA_Buff_In_16(29,CHECK_STATE_INDEX);
				break;
			}
			case 30:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Gyro_offset_Z,CHECK_DATA_INDEX);
				DMA_Buff_In_16(30,CHECK_STATE_INDEX);
				break;
			}
			//检查HMC5883L校正数据
			case 31:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(HMC5883L_X_offset,CHECK_DATA_INDEX);
				DMA_Buff_In_16(31,CHECK_STATE_INDEX);
				break;
			}
			case 32:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(HMC5883L_Y_offset,CHECK_DATA_INDEX);
				DMA_Buff_In_16(32,CHECK_STATE_INDEX);
				break;
			}
			case 33:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(HMC5883L_Z_offset,CHECK_DATA_INDEX);
				DMA_Buff_In_16(33,CHECK_STATE_INDEX);
				break;
			}
			//检查PID
			case 34:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Roll.Gyro_Kp * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(34,CHECK_STATE_INDEX);
				break;
			}
			case 35:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Roll.Gyro_Ki * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(35,CHECK_STATE_INDEX);
				break;
			}
			case 36:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Roll.Gyro_Kd * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(36,CHECK_STATE_INDEX);
				break;
			}

			case 37:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Roll.angle_Kp * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(37,CHECK_STATE_INDEX);
				break;
			}
			case 38:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Roll.angle_Ki * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(38,CHECK_STATE_INDEX);
				break;
			}
			case 39:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Roll.angle_Kd * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(39,CHECK_STATE_INDEX);
				break;
			}
			/******************************Pitch*********************************/
			case 40:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Pitch.Gyro_Kp * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(40,CHECK_STATE_INDEX);
				break;
			}
			case 41:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Pitch.Gyro_Ki * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(41,CHECK_STATE_INDEX);
				break;
			}
			case 42:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Pitch.Gyro_Kd * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(42,CHECK_STATE_INDEX);
				break;
			}

			case 43:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Pitch.angle_Kp * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(43,CHECK_STATE_INDEX);
				break;
			}
			case 44:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Pitch.angle_Ki * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(44,CHECK_STATE_INDEX);
				break;
			}
			case 45:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Pitch.angle_Kd * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(45,CHECK_STATE_INDEX);
				break;
			}
			/****************Yaw*************************/
			case 46:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Yaw.Gyro_Kp * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(46,CHECK_STATE_INDEX);
				break;
			}
			case 47:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Yaw.Gyro_Ki * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(47,CHECK_STATE_INDEX);
				break;
			}
			case 48:
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(Yaw.Gyro_Kd * 100,CHECK_DATA_INDEX);
				DMA_Buff_In_16(48,CHECK_STATE_INDEX);
				break;
			}


			default:break;
		}
		
			
	}  
}






void check_BT(void)      //检查蓝牙状态
{
	BT_state = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11);
	  if(BT_state)
	  {
	  	//TIM_Cmd(TIM4, ENABLE);  			//使能TIM4,开始数据传输	
		DMA_Cmd(DMA1_Channel4, ENABLE);	
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);	
		LED_ON;		
	  }	
	  else
	  {
	  //	TIM_Cmd(TIM4, DISABLE); 
		DMA_Cmd(DMA1_Channel4, DISABLE); 
		USART_DMACmd(USART1,USART_DMAReq_Tx,DISABLE);
		LED_OFF;
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


