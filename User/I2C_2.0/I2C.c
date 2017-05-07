#include "I2C.h" 
#include "delay.h"
//#include "USART.h"

uint32_t I2C_HoldTime = 2;

/*******************************I2C基本操作***********************************/


/************************发送IIC开始信号*************************/ 
void I2C_Start(void)
{
	I2C_SDA=1;	  	  
	I2C_SCL=1;
	// delay_us(I2C_HoldTime);
	I2C_SDA=0;            //I2C时钟线SCL为高时拉低I2C数据线SDA，发送开始信号
	// delay_us(I2C_HoldTime);
	I2C_SCL=0;            //拉低I2C时钟线，准备数据传输 
}	 



 
/*************************发送I2C结束信号**************************/
void I2C_Stop(void)
{          
	I2C_SCL=0;
	I2C_SDA=0;
	// delay_us(I2C_HoldTime);
	I2C_SCL=1; 
	I2C_SDA=1;    	     //I2C时钟线SCL为高时拉高I2C数据线SDA，发送结束信号 
	// delay_us(I2C_HoldTime);	
	I2C_SCL=0;						   	
}



/*****************等待应答信号，返回0即收到应答，-1即没有收到************/ 
s8 I2C_Wait_Ack(void)
{
	u8 WaitTime=0;
	SDA_IN();                      //SDA设置为输入  
	I2C_SDA=1;					   //上拉电阻
////	delay_us(I2C_HoldTime);	   
	I2C_SCL=1;
////	delay_us(I2C_HoldTime);	   
	while(READ_SDA)
	{
		if(WaitTime++>250)
		{
			I2C_SCL=0;                     //时钟输出0
			SDA_OUT();   //改为输出之前I2C_SCL记得变为低电平，否则会发出开始或者结束信号
			I2C_Stop();
			return -1;
		}
		// delay_us(I2C_HoldTime);
	}
	I2C_SCL=0;                     //时钟输出0 	
	SDA_OUT();    //改为输出之前I2C_SCL记得变为低电平，否则会发出开始或者结束信号
	return 0;  
}
 
/********************产生ACK应答*****************/
void I2C_Ack(void)		  //应答信号位SCL为高电平时，SDA为低电平
{
	I2C_SCL=0;
	I2C_SDA=0;
////	delay_us(I2C_HoldTime);
	I2C_SCL=1;
	// delay_us(I2C_HoldTime);
	I2C_SCL=0;
}
/****************************不产生ACK应答**************************/		    
void I2C_NAck(void)		//非应答信号位SCL为高电平时，SDA为高电平
{
	I2C_SCL=0;
	I2C_SDA=1;
	// delay_us(I2C_HoldTime);
	I2C_SCL=1;
	// delay_us(I2C_HoldTime);
	I2C_SCL=0;
}	

				 				     
/*****************************I2C发送一个字节数据***************************/
//返回1,从机有应答;0，无应答		  
s8 I2C_Send_Data(u8 data)
{                        
    u8 i;   	    
    I2C_SCL=0;                       //拉低时钟开始数据传输
    for(i = 0;i < 8; i++)
    {              
        I2C_SDA=data >> 7;           //(data&0x80)>>7;		//从最高位开始取
        data<<=1; 	  
		// delay_us(I2C_HoldTime);                 
		I2C_SCL=1;
		// delay_us(I2C_HoldTime);                 
		I2C_SCL=0;	
		// delay_us(I2C_HoldTime);                 
    }	
	return I2C_Wait_Ack( ); 
} 	   


 
/******************************读1个字节*****************************/
//参数ack=1时，发送ACK，ack=0，发送nACK   
u8 I2C_Read_Data(u8 ack)
{
	unsigned char i,receive=0;
	SDA_IN();                        //SDA设置为输入
    for(i=0;i<8;i++ )                //高位先发送 
	{
        I2C_SCL=0; 
        // delay_us(I2C_HoldTime);
		I2C_SCL=1;
        receive<<=1;
        if(READ_SDA)
		{
			receive++;   	//读到该位为1，+1（即置1）
		}
		// delay_us(I2C_HoldTime); 
    }	
	I2C_SCL=0; 	
	SDA_OUT(); 		//改为输出之前I2C_SCL记得变为低电平，否则会发出开始或者结束信号		 
    if (ack)
		I2C_Ack();                   //发送ACK
    else
		I2C_NAck();                  //发送nACK
           
    return receive;
} 



/**************************I2C组合操作************************************/

s8 I2C_SendMode(u8 device_address, u8 address)
{
	s8 ret = 0;
	I2C_Start();
	ret |= I2C_Send_Data(device_address);		//发送器件地址,写入模式
	ret |= I2C_Send_Data(address);				//发送操作地址
	return ret;
}




s8 I2C_ReadMode(u8 device_address,u8 address)
{
	s8 ret = 0;
	I2C_Start();
	ret |= I2C_Send_Data(device_address);		//发送器件地址,写入模式
	ret |= I2C_Send_Data(address);				//发送操作地址

	I2C_Start();  	 	                   //重启总线
	ret |= I2C_Send_Data(device_address + 1); //发送器件地址,读数据模式
	return ret;
}








/*************************I2C完整操作，直接读写************************************/

//向某个地址发送一个字节
s8 I2C_SendByte(u8 device_address,u8 address,u8 data)	  //设备地址需要预留最后一位读写状态位
{
	s8 ret = 0;
	I2C_Start();
	ret |= I2C_Send_Data(device_address);		//发送器件地址,写入模式
	ret |= I2C_Send_Data(address);				//发送操作地址
	ret |= I2C_Send_Data(data);					//发送数据
	I2C_Stop();
	//delay_ms(10);
	return ret;
}

//不指定发送的地址，发送一个字节数据
s8 I2C_SendByte_NoAddr(u8 device_address,u8 data)	  //设备地址需要预留最后一位读写状态位
{
	s8 ret = 0;
	I2C_Start();
	ret |= I2C_Send_Data(device_address);   //发送器件地址,写入模式
	ret |= I2C_Send_Data(data);				//发送数据
	I2C_Stop();
	//delay_ms(10);
	return ret;
}

//向某个地址发送多个字节
s8 I2C_SendBytes(u8 device_address,u8 address, uint8_t len, uint8_t *data)
{
	s8 ret = 0;
	int i;
    I2C_Start();
	ret |= I2C_Send_Data(device_address);
	ret |= I2C_Send_Data(address);
	for (i = 0; i < len; i++) 
	{
		ret |= I2C_Send_Data(data[i]);
	}
    I2C_Stop();
    return ret;
}
//向某个地址读取一个字节
s8 I2C_ReadByte(u8 device_address, u8 address, u8 *data) //设备地址需要预留最后一位读写状态位
{
	s8 ret = 0;
	I2C_Start();                           //发送开始命令

	ret |= I2C_Send_Data(device_address);  //发送器件地址,写数据 	                        //等待从机应答
	ret |= I2C_Send_Data(address);		   //发送操作地址

	I2C_Start();  	 	                   //重启总线
	ret |= I2C_Send_Data(device_address + 1); //发送器件地址,读数据模式
	*data = I2C_Read_Data(NACK);			//接收数据，无应答
	I2C_Stop();                            //发送停止信号	    
	return ret;
}

//向某个地址读取多个字节(以小端顺序读取、存储，即先读到的数据存在低地址，即数据低位，但传感器一般是先发送高位的，因此用大端较多)
s8 I2C_ReadBytes_LE(u8 device_address, u8 address, uint8_t len, uint8_t *buf)
{
	s8 ret = 0;
	I2C_Start();
	ret |= I2C_Send_Data(device_address);
	ret |= I2C_Send_Data(address);
	I2C_Start();
	ret |= I2C_Send_Data(device_address + 1);
	while (len) 
	{
        if (len == 1)
            *buf = I2C_Read_Data(NACK);
        else
            *buf = I2C_Read_Data(ACK);
        buf++;
        len--;
    }
    I2C_Stop();
	return ret;
}

//向某个地址读取多个字节（传感器很多数据先读到的都是高字节的，即以大端的顺序得到数据，而STM32一般是小端，低字节在前，因此存放顺序应该相反）
s8 I2C_ReadBytes_BE(u8 device_address, u8 address, uint8_t len, uint8_t *buf)
{
	s8 ret = 0;
	I2C_Start();
	ret |= I2C_Send_Data(device_address);
	ret |= I2C_Send_Data(address);
	I2C_Start();
	ret |= I2C_Send_Data(device_address + 1);
	buf += len;
	while (len--)
	{
		if (len)
			*(--buf) = I2C_Read_Data(ACK);
		else
			*(--buf) = I2C_Read_Data(NACK);
	}
	I2C_Stop();
	return ret;
}




/**************************实现函数********************************************
*函数原型:		u8 I2C_WriteBits(u8 device_address,u8 address,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	device_address  目标设备地址
		address	   寄存器地址
		bitStart  目标字节的最高位
		length   位长度
		data    存放改变目标字节位的值
返回   void

说明:
|7bit|6bit|5bit|4bit|3bit|2bit|1bit|0bit|
要把3bit置为0，4bit置为1，
则参数data应该为10，bitStart为4，length为2
计算过程：
mask先用0xFF << (bitStart + 1)，得到一个目标位左边全是1的数11100000
0xFF >> (7 - bitStart + length)得到目标位右边全是1的数0000011
两个数位或即可得到一个目标位全为0的数，这样就可以清零目标位11100011
再把data移动 8-length=6,到了最左边，即整个变量只剩下这两位其他全部置零11000000
再右移动(7 - bitStart) = 00011000
用mask清零再用data赋值得到目标数
*******************************************************************************/ 
s8 I2C_WriteBits(u8 device_address,u8 address,u8 bitStart,u8 length,u8 data)
{
	s8 ret = 0;
	u8 mask = 0;
	u8 read_data = 0;
	ret |= I2C_ReadByte(device_address, address, &read_data);
	//要写入的几个位mask都为1
	mask = (0xFF << (bitStart + 1)) | 0xFF >> (7 - bitStart + length);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	read_data &= mask;	//将待写的几个位清零
	read_data |= data;
	ret |= I2C_SendByte(device_address, address, read_data);
	return ret;
}

/**************************实现函数********************************************
*函数原型:		u8 I2C_WriteBit(u8 device_address, u8 address, u8 bitNum, u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	device_address  目标设备地址
		address	   寄存器地址
		bitNum  要修改目标字节的bitNum位
		data  为0 时，目标位将被清0 否则将被置位
返回   void
*******************************************************************************/ 
s8 I2C_WriteBit(u8 device_address, u8 address, u8 bitNum, u8 data)
{
	s8 ret = 0;
	u8 read_data = 0;
	ret |= I2C_ReadByte(device_address, address, &read_data);
	read_data = (data != 0) ? (read_data | (1 << bitNum)) : (read_data & ~(1 << bitNum));
	ret |= I2C_SendByte(device_address, address, read_data);
	return ret;
}
