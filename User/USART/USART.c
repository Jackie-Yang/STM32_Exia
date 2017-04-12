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

u8 ReceiveOrder;     //����ָ����
u8 BT_state = 0;

void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;                          //���ò�����
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;		  //8λ�ַ�����
  USART_InitStructure.USART_StopBits = USART_StopBits_1;			  //һλֹͣλ			    
  USART_InitStructure.USART_Parity = USART_Parity_No;				  //����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	  //�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure);				   
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);							//ʹ�ܴ����жϽ���
  //USART_ClearFlag(USART1, USART_FLAG_TC);  

  USART_Cmd(USART1, ENABLE);	             //ʹ�ܴ��� 
  GPIO_SetBits(GPIOA,GPIO_Pin_12); 			//ʹ������ ATģʽ
  //GPIO_ResetBits(GPIOA,GPIO_Pin_12);	   //ʹ������ ����ģʽ??
  USART_GetFlagStatus(USART1, USART_FLAG_TC);//��ȡTC�ܽ�����λ�������һ�ֽڷ���ʧ�ܵ�����

}

void USART1_sendData_u8(u8 data)	 	//���ڷ���8λ����
{
    USART_SendData(USART1,data);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void USART1_sendData_u16(u16 data)	  //���ڷ���16λ����
{
	u8 temp;
	temp = (data & 0xFF00) >> 8;
    USART_SendData(USART1,temp);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	temp = (data & 0xFF);
	USART_SendData(USART1,temp);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void USART1_sendData_u32(u32 data)	  //���ڷ���16λ����
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






/**********************���ڽ����ж�*******************************************/
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE))  	//��ȡ�����жϱ�־λUSART_IT_RXNE 
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);	//����жϱ�־λ	
		ReceiveOrder = USART_ReceiveData(USART1);	//�����ַ���ȡ
		switch(ReceiveOrder)						//������Ӧ����
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
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));  //�ȴ������ź�

//				for(i = 0;i < 4; i++)
//				{
//					set_motorPWM(i + 1,receive_THRO);
//				}

				if(REC_RUDD == 0)					//����λ
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
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));  //�ȴ�������ź�

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
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));  //�ȴ��������ź�

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
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));  //�ȴ������ź�

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



			//PID������������
			case 9:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

			//	DMA_Buff_In_16(receive_Data,ROLL_GYRO_KP_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(ROLL_GYRO_KP_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Roll.Gyro_Kp = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 10:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,ROLL_GYRO_KI_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(ROLL_GYRO_KI_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Roll.Gyro_Ki = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 11:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,ROLL_GYRO_KD_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(ROLL_GYRO_KD_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Roll.Gyro_Kd = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 12:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,ROLL_ANGLE_KP_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(ROLL_ANGLE_KP_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Roll.angle_Kp = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 13:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

	//			DMA_Buff_In_16(receive_Data,ROLL_ANGLE_KI_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(ROLL_ANGLE_KI_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Roll.angle_Ki = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 14:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,ROLL_ANGLE_KD_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(ROLL_ANGLE_KD_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Roll.angle_Kd = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 15:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,PITCH_GYRO_KP_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(PITCH_GYRO_KP_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Pitch.Gyro_Kp = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 16:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

	//			DMA_Buff_In_16(receive_Data,PITCH_GYRO_KI_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(PITCH_GYRO_KI_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Pitch.Gyro_Ki = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 17:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

	//			DMA_Buff_In_16(receive_Data,PITCH_GYRO_KD_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(PITCH_GYRO_KD_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Pitch.Gyro_Kd = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 18:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

				//DMA_Buff_In_16(receive_Data,PITCH_ANGLE_KP_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(PITCH_ANGLE_KP_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Pitch.angle_Kp = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 19:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

			//	DMA_Buff_In_16(receive_Data,PITCH_ANGLE_KI_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(PITCH_ANGLE_KI_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Pitch.angle_Ki = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 20:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,PITCH_ANGLE_KD_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(PITCH_ANGLE_KD_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Pitch.angle_Kd = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 21:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,YAW_GYRO_KP_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(YAW_GYRO_KP_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Yaw.Gyro_Kp = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 22:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,YAW_GYRO_KI_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(YAW_GYRO_KI_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Yaw.Gyro_Ki = ((float)receive_Data) / 100.0;		   //
				break;
			}
			case 23:
			{
				u16 receive_Data = 0;

				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 	//�Ӵ��ڶ�ȡ���ֽڲ���
				receive_Data = USART_ReceiveData(USART1);
				receive_Data <<= 8;
				while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)); 
				receive_Data |= USART_ReceiveData(USART1);

		//		DMA_Buff_In_16(receive_Data,YAW_GYRO_KD_INDEX);		//д��DMA�ش�����λ��
				EE_WriteVariable(YAW_GYRO_KD_ADDR,receive_Data);	   //�����������Flash�������´ο�����ȡ
				Yaw.Gyro_Kd = ((float)receive_Data) / 100.0;		   //
				break;
			}
	//���ĳһ�̶�����������PID��У��ֵ��Щ��Բ�����ֵ������Ƶ������DMA����Ҫ��ʱ������Ӧָ����ؼ��ֵ����
			case 24:		   //���ر�
			{
				DMA_Buff_In_16(0xFFFF,CHECK_STATE_INDEX);
				DMA_Buff_In_16(0xFFFF,CHECK_DATA_INDEX);
				break;
			}
			//���MPU6050У������
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
			//���HMC5883LУ������
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
			//���PID
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






void check_BT(void)      //�������״̬
{
	BT_state = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11);
	  if(BT_state)
	  {
	  	//TIM_Cmd(TIM4, ENABLE);  			//ʹ��TIM4,��ʼ���ݴ���	
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

/************************��������/�Ͽ������ж�*************************************/
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line11)!= RESET)	 //�ж��Ƿ����ж�
	{
	  check_BT();		//��������״̬				 
	}
	EXTI_ClearITPendingBit(EXTI_Line11);		     //����жϱ�־			
}


