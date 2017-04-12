#include "OS.h"

//orderState:串口是否收到指令  ReceiveData[ORDER_SIZE]：串口数据缓存 order[ORDER_SIZE]：存放收到的指令
u8 orderState = 0, ReceiveData[ORDER_SIZE] = {0}, order[ORDER_SIZE] = {0},err = 0;

//p_Data：串口当前写入缓存的位置  temp,临时变量，循环用
u16 p_Data = 0,temp;

//存放指令中转换完成的数值
u32 order_Num = 0;

//存放从AT24C02中读出的数据， 读写地址
u8 temp_data[256] = {0},address = 0;






/********************系统初始化*****************************/
//初始化系统时钟，各IO口，中断
//初始化串口
//初始化定时器
/************************************************************/
void OS_init(void)
{
	RCC_Configuration( );	    
	GPIO_Configuration( );		 
	USART_Configuration( );		
	NVIC_Configuration( );		  
	TIMx_Int_Init(TIM3);		  
}



/********************系统指令集******************************/
//对串口收到的数据进行判断，调用相应的模块
/************************************************************/


void runOrder(void)
{


/********************LED指令*********************************/
//打开，关闭，改变LED1指令：
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




//打开，关闭，改变LED2指令：
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



//打开，关闭，改变LED3指令：
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




//打开，关闭，改变LED4指令：
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




//6种花样闪烁指令
	if(!strcmp((const char *)order,"LED blink1"))     //一起闪
	{
		Blink_ON(1);
		USART1_sendStr("LED start to blink(mode 1)\n");
		return;
	}
	if(!strcmp((const char *)order,"LED blink2"))	 //12,34两组分开闪
	{
		Blink_ON(2);
		USART1_sendStr("LED start to blink(mode 2)\n");
		return;
	}
	if(!strcmp((const char *)order,"LED blink3"))	//13,24两组分开闪
	{
		Blink_ON(3);
		USART1_sendStr("LED start to blink(mode 3)\n");
		return;
	}
	if(!strcmp((const char *)order,"LED blink4"))  //正向流水灯
	{
		Blink_ON(4);
		USART1_sendStr("LED start to blink(mode 4)\n");
		return;
	}
	if(!strcmp((const char *)order,"LED blink5")) //反向流水灯
	{
		Blink_ON(5);
		USART1_sendStr("LED start to blink(mode 5)\n");
		return;
	}
	if(!strcmp((const char *)order,"LED blink6")) //双向流水灯
	{
		Blink_ON(6);
		USART1_sendStr("LED start to blink(mode 6)\n");
		return;
	}
	if(!strcmp((const char *)order,"LED blink off"))   //结束花样闪烁（如果不结束闪烁，去改变LED灯状态也可改变花样）
	{
		Blink_OFF( );
		USART1_sendStr("LED blink off\n");
		return;
	}



/*****************************定时器指令******************************************/
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
	
	
		
/**********************读取按键指令***********************************/
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




/**********************读取ADC指令***********************************/
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





/**********************DMA读取ADC指令***********************************/
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



/**********************关闭DMA指令***********************************/
	if(!strcmp((const char *)order,"stop DMA"))
	{
		DMA_stop( );
		USART1_sendStr("stop DMA:\n");
		return;
	}






/*********************AT24C02操作指令******************************/

//完全擦除AT24C02
	if(!strcmp((const char *)order,"AT24C02 Erase"))
	{
		AT24C02_Erase( );
		USART1_sendStr("Eracs success!\n");
		return;
	}

//向AT24C02写入一个字节
	if(!strcmp((const char *)order,"AT24C02 Write One Byte"))
	{
		USART1_sendStr("Please input address(HEX):\n0x");
		if(getNum_HEX())	                        //获取地址
		{
			address = (u8) order_Num;
			USART1_sendStr("Please input the data:");
			AT24C02_WriteOneByte(address,* getData( ));
			USART1_sendData( AT24C02_ReadOneByte(address) );	 //从AT24C02中读出刚输入的数据，确认无误
			USART1_sendStr("\n");
		}
		return;
	}

//向AT24C02读取一个字节
	if(!strcmp((const char *)order,"AT24C02 Read One Byte"))
	{
		USART1_sendStr("Please input address(HEX):\n0x");
		if(getNum_HEX())	                        //获取地址
		{
			address = (u8) order_Num;
			USART1_sendData( AT24C02_ReadOneByte(address) );
			USART1_sendStr("\n");
		}
		return;
	}

//向AT24C02写入一定数量字节
	if(!strcmp((const char *)order,"AT24C02 Write Bytes"))
	{
		USART1_sendStr("Please input address(HEX):\n0x");
		if(getNum_HEX())	                        //获取地址
		{
			address = (u8) order_Num;
			USART1_sendStr("Please input the number of data(byte):");		//输入写入的数据数目
			if(getNum())
			{
				if( ((u16)address+order_Num) > 256)
				{
					USART1_sendStr("illegal number !!\n");      //输入数字超过范围					        
				}
				else
				{
					USART1_sendStr("Please input the data:\n");
					AT24C02_Write(address,getData( ),order_Num);	   //直接将缓存数据写入
					AT24C02_Read(address,temp_data,order_Num);			//读出缓存数据检查无误
					for(temp = 0;temp < order_Num;temp++)
					{
						USART1_sendData( temp_data[temp] );	    //读取并串口发送数据
					}
					
					USART1_sendStr("\n");
				}
			}
		}
		return;
	}

//向AT24C02读取一定数量字节
	if(!strcmp((const char *)order,"AT24C02 Read Bytes"))
	{
		USART1_sendStr("Please input address(HEX):\n0x");
		if(getNum_HEX())	                        //获取地址
		{
			address = (u8) order_Num;
			USART1_sendStr("Please input the number of data(byte):");		//输入读取的数据数目
			if(getNum())
			{
				if( ((u16)address+order_Num) > 256)
				{
					USART1_sendStr("illegal number !!\n");      //输入数字超过范围					        
				}
				else
				{
					AT24C02_Read(address,temp_data,order_Num);
					for(temp = 0;temp < order_Num;temp++)
					{
						USART1_sendData( temp_data[temp] );	    //读取并串口发送数据
					}
					
					USART1_sendStr("\n");
				}
			}
		}
		return;
	}
	


	USART1_sendStr("Command ERROR!!\n");	//不是以上指令则输入错误	
}






/*******************************获取用户指令(字符串)********************************************/
u8 getOrder(void)
{
	if(orderState)			  //接收到指令
	{
		while(strcmp((const char *)ReceiveData,(const char *)order))	 //当order与接收到的命令不符合时，即没接收完
		{
			strcpy((char *)order,(char *)ReceiveData);       //接收到的命令复制到order，若与接收到的命令一致，字符串接收完毕
			delay_ms(5);                     			     //稍作延迟等待数据存入缓存
		}	
		if(err)						                         //指令溢出
		{
			for(p_Data = ORDER_SIZE; p_Data; p_Data--)       //指令完全清零
			{
				ReceiveData[p_Data - 1] = 0;
			}
			USART1_sendStr("Command ERROR!!\n");
			err = 0;								         //错误标志复位
			orderState = 0;
		    order[0] = 0;            //清零第一个字符即可，节省运算时间
		}
		else
		{
			USART1_sendStr(order);				 //显示出输入的指令
			USART1_sendStr(":\n");               //冒号换行
			for(; p_Data; p_Data--)		         //指令清零
			{
				ReceiveData[p_Data - 1] = 0;
			}
			orderState = 0;
			return 0;//收到指令，返回0
		}
	}
	return 1;   //无有效指令，返回1
}




/**************************获取用户指令(十进制数值)********************************************/
u8 getNum(void)
{
	u8 temp = 0;
	order_Num = 0;
	while(getOrder( ));           //先以字符串形式获取
	while( order[temp] )          //当读到字符串结束标志时读取完
	{
		if( order[temp] >= '0' && order[temp] <= '9')
		{
			order_Num = order_Num * 10 + order[temp] - '0';
			temp++;
		}
		else 
		{
			USART1_sendStr("illegal number !!\n");
			return 0;            //输入非数字，返回0
		}
	} 
	return 1;
}


/***************************获取用户输入16进制数字(0x00~0xFF)************************************/
u8 getNum_HEX(void)
{
	u8 temp = 0;
	order_Num = 0;
	while(getOrder( ));           //先以字符串形式获取
	while( order[temp] )          //当读到字符串结束标志时读取完
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
			return 0;            //输入非数字，返回0
		}
	} 
	if(order_Num >= 256)
	{
		USART1_sendStr("illegal number !!\n");
		return 0;            //输入数字超过范围，返回0
	}
	return 1;
}




/**************************直接获取用户输入数据*************************************************/
u8* getData(void)
{
	while(!orderState);			  //接收到指令
	delay_ms(5);
	err = 0;                      //各种接收标志复位
	orderState = 0;
	p_Data = 0;
	return 	ReceiveData;         //直接返回收到的数据
}




/**********************串口接收中断，存入接收缓存数组*******************************************/
void USART1_IRQHandler()
{
	if((USART_GetITStatus(USART1,USART_IT_RXNE)))  	//读取接收中断标志位USART_IT_RXNE 
	{
	    orderState = 1;
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);	//清除中断标志位
		ReceiveData[p_Data]=USART_ReceiveData(USART1);	//接收字符存入数组
		p_Data++;			
		if(p_Data >= ORDER_SIZE)                    //要确保留出1个字符串结束字节
		{
			ReceiveData[ORDER_SIZE - 1] = 0;	    //确保最后一个字符为0
			err = 1;	
			p_Data = 0;
		}

	}  
}




