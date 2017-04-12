#include "I2C.h" 
#include "delay.h"
//#include "USART.h"



/*******************************I2C基本操作***********************************/


/************************发送IIC开始信号*************************/ 
void I2C_Start(void)
{
	I2C_SDA=1;	  	  
	I2C_SCL=1;
//	delay_us(1);
 	I2C_SDA=0;            //I2C时钟线SCL为高时拉低I2C数据线SDA，发送开始信号 
//	delay_us(1);
	I2C_SCL=0;            //拉低I2C时钟线，准备数据传输 
}	 



 
/*************************发送I2C结束信号**************************/
void I2C_Stop(void)
{          
	I2C_SCL=0;
	I2C_SDA=0;         
// 	delay_us(1);
	I2C_SCL=1; 
	I2C_SDA=1;    	     //I2C时钟线SCL为高时拉高I2C数据线SDA，发送结束信号 
//	delay_us(1);	
	I2C_SCL=0;						   	
}



/*****************等待应答信号，返回1即收到应答，0即没有收到************/ 
u8 I2C_Wait_Ack(void)
{
	u8 WaitTime=0;
	SDA_IN();                      //SDA设置为输入  
	I2C_SDA=1;					   //上拉电阻
////	delay_us(1);	   
	I2C_SCL=1;
////	delay_us(1);	 
	while(READ_SDA)
	{
		if(WaitTime++>250)
		{
			I2C_Stop();
			return 0;
		}
	}
	I2C_SCL=0;                     //时钟输出0 	
	SDA_OUT();    
	return 1;  
}
 
/********************产生ACK应答*****************/
void I2C_Ack(void)		  //应答信号位SCL为高电平时，SDA为低电平
{
	I2C_SCL=0;
	I2C_SDA=0;
////	delay_us(1);
	I2C_SCL=1;
//	delay_us(1);
	I2C_SCL=0;
}
/****************************不产生ACK应答**************************/		    
void I2C_NAck(void)		//非应答信号位SCL为高电平时，SDA为高电平
{
	I2C_SCL=0;
	I2C_SDA=1;
//	delay_us(1);
	I2C_SCL=1;
//	delay_us(1);
	I2C_SCL=0;
}	

				 				     
/*****************************I2C发送一个字节数据***************************/
//返回1,从机有应答;0，无应答		  
u8 I2C_Send_Data(u8 data)
{                        
    u8 i;   	    
    I2C_SCL=0;                       //拉低时钟开始数据传输
    for(i = 0;i < 8; i++)
    {              
        I2C_SDA=data >> 7;           //(data&0x80)>>7;		//从最高位开始取
        data<<=1; 	  
		//delay_us(1);                 
		I2C_SCL=1;
		//delay_us(1); 
		I2C_SCL=0;	
		//delay_us(1);
    }	
	I2C_Wait_Ack( ); 
	return 1; 
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
        //delay_us(1);
		I2C_SCL=1;
        receive<<=1;
        if(READ_SDA)
		{
			receive++;   	//读到该位为1，+1（即置1）
		}
		//delay_us(1); 
    }		
	SDA_OUT(); 			 
    if (ack)
		I2C_Ack();                   //发送ACK
    else
		I2C_NAck();                  //发送nACK
           
    return receive;
} 



/**************************I2C组合操作************************************/


void I2C_SendMode(u8 device_address,u8 address)
{
	I2C_Start();  
	I2C_Send_Data(device_address);   		//发送器件地址,写入模式 	 
	I2C_Send_Data(address);     		        //发送操作地址							   
	
}




void I2C_ReadMode(u8 device_address,u8 address)
{
	I2C_Start();  
	I2C_Send_Data(device_address);   		//发送器件地址,写入模式 	 
	I2C_Send_Data(address);     		        //发送操作地址							   

	I2C_Start();  	 	                   //重启总线 
	I2C_Send_Data(device_address + 1);   	//发送器件地址,读数据模式		    
	
}








/*************************I2C完整操作，直接读取************************************/


void I2C_SendByte(u8 device_address,u8 address,u8 data)	  //设备地址需要预留最后一位读写状态位
{
	I2C_Start();  
	I2C_Send_Data(device_address);   		//发送器件地址,写入模式 	 
	I2C_Send_Data(address);     		        //发送操作地址							      	 										  		   
	I2C_Send_Data(data);     		        //发送数据							     		    	   
    I2C_Stop();
	//delay_ms(10); 
}


void I2C_SendByte_NoAddr(u8 device_address,u8 data)	  //设备地址需要预留最后一位读写状态位
{
	I2C_Start();  
	I2C_Send_Data(device_address);   		//发送器件地址,写入模式 	   	 										  		   
	I2C_Send_Data(data);     		        //发送数据							   		    	   
    I2C_Stop();
	//delay_ms(10); 
}

u8 I2C_ReadByte(u8 device_address,u8 address)	  //设备地址需要预留最后一位读写状态位
{
	u8 data=0;		  	    																 
    I2C_Start();                           //发送开始命令 
	
	I2C_Send_Data(device_address);  		//发送器件地址,写数据 	                        //等待从机应答 
    I2C_Send_Data(address);                //发送操作地址	            
	 
	I2C_Start();  	 	                   //重启总线 
	I2C_Send_Data(device_address + 1);   	//发送器件地址,读数据模式		   	  
    data = I2C_Read_Data(NACK);		   		   //接收数据，无应答 
    I2C_Stop();                            //发送停止信号	    
	return data;
}

//对设备连续读取，设备应该会自动将操作地址递增
u16 I2C_Read_16(u8 device_address,u8 address)	  //设备地址需要预留最后一位读写状态位
{
	u16 data=0;		  	    																 
    I2C_Start();                           //发送开始命令 
	
	I2C_Send_Data(device_address);  		//发送器件地址,写数据 	                     
    I2C_Send_Data(address);                //发送操作地址           
	 
	I2C_Start();  	 	                   //重启总线 
	I2C_Send_Data(device_address + 1);   	//发送器件地址,读数据		   	
  
    data = I2C_Read_Data(ACK);		   		   //接收数据，应答
	data <<= 8;		   
	data |= I2C_Read_Data(NACK);

    I2C_Stop();                            //发送停止信号	    
	return data;
}

u32 I2C_Read_32(u8 device_address,u8 address)	  //设备地址需要预留最后一位读写状态位
{
	u32 data=0;		  	    																 
    I2C_Start();                           //发送开始命令 
	
	I2C_Send_Data(device_address);  		//发送器件地址,写数据 	              
    I2C_Send_Data(address);                //发送操作地址           
	 
	I2C_Start();  	 	                   //重启总线 
	I2C_Send_Data(device_address + 1);   	//发送器件地址,读数据		   
  
	data = I2C_Read_Data(ACK);		   		   //接收数据，应答
	data <<= 8;	
    data |= I2C_Read_Data(ACK);		   		   //接收数据，应答
	data <<= 8;	
	data |= I2C_Read_Data(ACK);		   //接收数据，应答
	data <<= 8; 	   
	data |= I2C_Read_Data(NACK);

    I2C_Stop();                            //发送停止信号	    
	return data;
}
