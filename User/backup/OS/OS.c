#include "OS.h"

//orderState:�����Ƿ��յ�ָ��  ReceiveData[ORDER_SIZE]���������ݻ��� order[ORDER_SIZE]������յ���ָ��
u8 orderState = 0, ReceiveData[ORDER_SIZE] = {0}, order[ORDER_SIZE] = {0},err = 0;

//p_Data�����ڵ�ǰд�뻺���λ��  temp,��ʱ������ѭ����
u16 p_Data = 0,temp;

//���ָ����ת����ɵ���ֵ
u32 order_Num = 0;

//��Ŵ�AT24C02�ж��������ݣ� ��д��ַ
u8 temp_data[256] = {0},address = 0;






/********************ϵͳ��ʼ��*****************************/
//��ʼ��ϵͳʱ�ӣ���IO�ڣ��ж�
//��ʼ������
//��ʼ����ʱ��
/************************************************************/
void OS_init(void)
{
	RCC_Configuration( );	    
	GPIO_Configuration( );		 
	USART_Configuration( );		
	NVIC_Configuration( );		  
	TIMx_Int_Init(TIM3);		  
}



/********************ϵͳָ�******************************/
//�Դ����յ������ݽ����жϣ�������Ӧ��ģ��
/************************************************************/


void runOrder(void)
{


/********************LEDָ��*********************************/
//�򿪣��رգ��ı�LED1ָ�
	if(!strcmp((const char *)order,"LED1 on"))
	{
		ON(LED1)
		USART1_sendStr("Turn on LED 1\n");
		return;
	}
	if(!strcmp((const char *)order,"LED1 off"))
	{
		OFF(LED1)
		USART1_sendStr("Turn of LED 1\n");
		return;
	}
	if(!strcmp((const char *)order,"LED1 change"))
	{
		TOGGLE(LED1)
		USART1_sendStr("Change LED 1 state\n");
		return;
	}




//�򿪣��رգ��ı�LED2ָ�
	if(!strcmp((const char *)order,"LED2 on"))
	{
		ON(LED2)
		USART1_sendStr("Turn on LED 2\n");
		return;
	}
	if(!strcmp((const char *)order,"LED2 off"))
	{
		OFF(LED2)
		USART1_sendStr("Turn off LED 2\n");
		return;
	}
	if(!strcmp((const char *)order,"LED2 change"))
	{
		TOGGLE(LED2)
		USART1_sendStr("Change LED 2 state\n");
		return;
	}



//�򿪣��رգ��ı�LED3ָ�
	if(!strcmp((const char *)order,"LED3 on"))
	{
		ON(LED3)
		USART1_sendStr("Turn on LED 3\n");
		return;
	}
	if(!strcmp((const char *)order,"LED3 off"))
	{
		OFF(LED3)
		USART1_sendStr("Turn of LED 3\n");
		return;
	}
	if(!strcmp((const char *)order,"LED3 change"))
	{
		TOGGLE(LED3)
		USART1_sendStr("Change LED 3 state\n");
		return;
	}




//�򿪣��رգ��ı�LED4ָ�
	if(!strcmp((const char *)order,"LED4 on"))
	{
		ON(LED4)
		USART1_sendStr("Turn on LED 4\n");
		return;
	}
	if(!strcmp((const char *)order,"LED4 off"))
	{
		OFF(LED4)
		USART1_sendStr("Turn of LED 4\n");
		return;
	}
	if(!strcmp((const char *)order,"LED4 change"))
	{
		TOGGLE(LED4)
		USART1_sendStr("Change LED 4 state\n");
		return;
	}




//6�ֻ�����˸ָ��
	if(!strcmp((const char *)order,"LED blink1"))     //һ����
	{
		Blink_ON(1);
		USART1_sendStr("LED start to blink(mode 1)\n");
		return;
	}
	if(!strcmp((const char *)order,"LED blink2"))	 //12,34����ֿ���
	{
		Blink_ON(2);
		USART1_sendStr("LED start to blink(mode 2)\n");
		return;
	}
	if(!strcmp((const char *)order,"LED blink3"))	//13,24����ֿ���
	{
		Blink_ON(3);
		USART1_sendStr("LED start to blink(mode 3)\n");
		return;
	}
	if(!strcmp((const char *)order,"LED blink4"))  //������ˮ��
	{
		Blink_ON(4);
		USART1_sendStr("LED start to blink(mode 4)\n");
		return;
	}
	if(!strcmp((const char *)order,"LED blink5")) //������ˮ��
	{
		Blink_ON(5);
		USART1_sendStr("LED start to blink(mode 5)\n");
		return;
	}
	if(!strcmp((const char *)order,"LED blink6")) //˫����ˮ��
	{
		Blink_ON(6);
		USART1_sendStr("LED start to blink(mode 6)\n");
		return;
	}
	if(!strcmp((const char *)order,"LED blink off"))   //����������˸�������������˸��ȥ�ı�LED��״̬Ҳ�ɸı仨����
	{
		Blink_OFF( );
		USART1_sendStr("LED blink off\n");
		return;
	}



/*****************************��ʱ��ָ��******************************************/
	if(!strcmp((const char *)order,"Set time"))
	{
		USART1_sendStr("Please Set The Time(Second): ");
		if(getNum())
		{
			timer3_ON( order_Num );
			USART1_sendStr("\nStart!\n");
		}
		return;
	}
	
	
		
/**********************��ȡ����ָ��***********************************/
	if(!strcmp((const char *)order,"Read JoyStick"))
	{
		USART1_sendStr("Start Read JoyStick State: \n");
		EXIT_Configuration( );
		return;
	}

	if(!strcmp((const char *)order,"Stop Read JoyStick"))
	{
		USART1_sendStr("Stop Read JoyStick State: \n");
		EXIT_Close( );
		return;
	}




/**********************��ȡADCָ��***********************************/
	if(!strcmp((const char *)order,"ADC1 read Channel15"))
	{
		USART1_sendStr("ADC result: \n");
		ADC_init(ADC1,ADC_Channel_15);
		USART1_sendStr(getADC_Result(ADC1));
		return;
	}

	if(!strcmp((const char *)order,"ADC2 read Channel15"))
	{
		USART1_sendStr("ADC result: \n");
		ADC_init(ADC2,ADC_Channel_15);
		USART1_sendStr(getADC_Result(ADC2));
		return;
	}





/**********************DMA��ȡADCָ��***********************************/
	if(!strcmp((const char *)order,"read ADC1 Channel15 with DMA"))
	{
		ADC_init(ADC1,ADC_Channel_15);
		ADC_DMA_Configuration(ADC1);
		DMA_start( );
		return;
	}

/*	if(!strcmp((const char *)order,"read ADC2 Channel15 with DMA"))
	{
		ADC_init(ADC2,ADC_Channel_15);
		ADC_DMA_Configuration(ADC2);
		DMA_start( );
		return;
	} */



/**********************�ر�DMAָ��***********************************/
	if(!strcmp((const char *)order,"stop DMA"))
	{
		DMA_stop( );
		USART1_sendStr("stop DMA:\n");
		return;
	}






/*********************AT24C02����ָ��******************************/

//��ȫ����AT24C02
	if(!strcmp((const char *)order,"AT24C02 Erase"))
	{
		AT24C02_Erase( );
		USART1_sendStr("Eracs success!\n");
		return;
	}

//��AT24C02д��һ���ֽ�
	if(!strcmp((const char *)order,"AT24C02 Write One Byte"))
	{
		USART1_sendStr("Please input address(HEX):\n0x");
		if(getNum_HEX())	                        //��ȡ��ַ
		{
			address = (u8) order_Num;
			USART1_sendStr("Please input the data:");
			AT24C02_WriteOneByte(address,* getData( ));
			USART1_sendData( AT24C02_ReadOneByte(address) );	 //��AT24C02�ж�������������ݣ�ȷ������
			USART1_sendStr("\n");
		}
		return;
	}

//��AT24C02��ȡһ���ֽ�
	if(!strcmp((const char *)order,"AT24C02 Read One Byte"))
	{
		USART1_sendStr("Please input address(HEX):\n0x");
		if(getNum_HEX())	                        //��ȡ��ַ
		{
			address = (u8) order_Num;
			USART1_sendData( AT24C02_ReadOneByte(address) );
			USART1_sendStr("\n");
		}
		return;
	}

//��AT24C02д��һ�������ֽ�
	if(!strcmp((const char *)order,"AT24C02 Write Bytes"))
	{
		USART1_sendStr("Please input address(HEX):\n0x");
		if(getNum_HEX())	                        //��ȡ��ַ
		{
			address = (u8) order_Num;
			USART1_sendStr("Please input the number of data(byte):");		//����д���������Ŀ
			if(getNum())
			{
				if( ((u16)address+order_Num) > 256)
				{
					USART1_sendStr("illegal number !!\n");      //�������ֳ�����Χ					        
				}
				else
				{
					USART1_sendStr("Please input the data:\n");
					AT24C02_Write(address,getData( ),order_Num);	   //ֱ�ӽ���������д��
					AT24C02_Read(address,temp_data,order_Num);			//�����������ݼ������
					for(temp = 0;temp < order_Num;temp++)
					{
						USART1_sendData( temp_data[temp] );	    //��ȡ�����ڷ�������
					}
					
					USART1_sendStr("\n");
				}
			}
		}
		return;
	}

//��AT24C02��ȡһ�������ֽ�
	if(!strcmp((const char *)order,"AT24C02 Read Bytes"))
	{
		USART1_sendStr("Please input address(HEX):\n0x");
		if(getNum_HEX())	                        //��ȡ��ַ
		{
			address = (u8) order_Num;
			USART1_sendStr("Please input the number of data(byte):");		//�����ȡ��������Ŀ
			if(getNum())
			{
				if( ((u16)address+order_Num) > 256)
				{
					USART1_sendStr("illegal number !!\n");      //�������ֳ�����Χ					        
				}
				else
				{
					AT24C02_Read(address,temp_data,order_Num);
					for(temp = 0;temp < order_Num;temp++)
					{
						USART1_sendData( temp_data[temp] );	    //��ȡ�����ڷ�������
					}
					
					USART1_sendStr("\n");
				}
			}
		}
		return;
	}
	


	USART1_sendStr("Command ERROR!!\n");	//��������ָ�����������	
}






/*******************************��ȡ�û�ָ��(�ַ���)********************************************/
u8 getOrder(void)
{
	if(orderState)			  //���յ�ָ��
	{
		while(strcmp((const char *)ReceiveData,(const char *)order))	 //��order����յ����������ʱ����û������
		{
			strcpy((char *)order,(char *)ReceiveData);       //���յ�������Ƶ�order��������յ�������һ�£��ַ����������
			delay_ms(5);                     			     //�����ӳٵȴ����ݴ��뻺��
		}	
		if(err)						                         //ָ�����
		{
			for(p_Data = ORDER_SIZE; p_Data; p_Data--)       //ָ����ȫ����
			{
				ReceiveData[p_Data - 1] = 0;
			}
			USART1_sendStr("Command ERROR!!\n");
			err = 0;								         //�����־��λ
			orderState = 0;
		    order[0] = 0;            //�����һ���ַ����ɣ���ʡ����ʱ��
		}
		else
		{
			USART1_sendStr(order);				 //��ʾ�������ָ��
			USART1_sendStr(":\n");               //ð�Ż���
			for(; p_Data; p_Data--)		         //ָ������
			{
				ReceiveData[p_Data - 1] = 0;
			}
			orderState = 0;
			return 0;//�յ�ָ�����0
		}
	}
	return 1;   //����Чָ�����1
}




/**************************��ȡ�û�ָ��(ʮ������ֵ)********************************************/
u8 getNum(void)
{
	u8 temp = 0;
	order_Num = 0;
	while(getOrder( ));           //�����ַ�����ʽ��ȡ
	while( order[temp] )          //�������ַ���������־ʱ��ȡ��
	{
		if( order[temp] >= '0' && order[temp] <= '9')
		{
			order_Num = order_Num * 10 + order[temp] - '0';
			temp++;
		}
		else 
		{
			USART1_sendStr("illegal number !!\n");
			return 0;            //��������֣�����0
		}
	} 
	return 1;
}


/***************************��ȡ�û�����16��������(0x00~0xFF)************************************/
u8 getNum_HEX(void)
{
	u8 temp = 0;
	order_Num = 0;
	while(getOrder( ));           //�����ַ�����ʽ��ȡ
	while( order[temp] )          //�������ַ���������־ʱ��ȡ��
	{
		if( order[temp] >= '0' && order[temp] <= '9')
		{
			order_Num = (order_Num << 4) + order[temp] - '0';
			temp++;
		}
		else if( order[temp] >= 'A' && order[temp] <= 'F')
		{
			order_Num = (order_Num << 4) + order[temp] - 55;
			temp++;
		}
		else if( order[temp] >= 'a' && order[temp] <= 'f')
		{
			order_Num = (order_Num << 4) + order[temp] - 87;
			temp++;
		}
		else 
		{
			USART1_sendStr("illegal number !!\n");
			return 0;            //��������֣�����0
		}
	} 
	if(order_Num >= 256)
	{
		USART1_sendStr("illegal number !!\n");
		return 0;            //�������ֳ�����Χ������0
	}
	return 1;
}




/**************************ֱ�ӻ�ȡ�û���������*************************************************/
u8* getData(void)
{
	while(!orderState);			  //���յ�ָ��
	delay_ms(5);
	err = 0;                      //���ֽ��ձ�־��λ
	orderState = 0;
	p_Data = 0;
	return 	ReceiveData;         //ֱ�ӷ����յ�������
}




/**********************���ڽ����жϣ�������ջ�������*******************************************/
void USART1_IRQHandler()
{
	if((USART_GetITStatus(USART1,USART_IT_RXNE)))  	//��ȡ�����жϱ�־λUSART_IT_RXNE 
	{
	    orderState = 1;
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);	//����жϱ�־λ
		ReceiveData[p_Data]=USART_ReceiveData(USART1);	//�����ַ���������
		p_Data++;			
		if(p_Data >= ORDER_SIZE)                    //Ҫȷ������1���ַ��������ֽ�
		{
			ReceiveData[ORDER_SIZE - 1] = 0;	    //ȷ�����һ���ַ�Ϊ0
			err = 1;	
			p_Data = 0;
		}

	}  
}




