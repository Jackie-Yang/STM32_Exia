#include "KS10X.h"
#include "I2C.h"
#include "delay.h"
#include "Setup.h"

/*************进入配置模式**************/
//#define KS109_DEBUG

/**************进行不同配置(一次只进行一个)*********************/
//#define KS10X_SCL		   //测量过程SCL不被拉低
//#define KS109_INIT	   //配置KS109

u16 KS10X_high = 0;



void KS10X_init(void)	  
{
	delay_ms(1000);		   //模块初始化
	delay_ms(1000);


/********************以下配置只需要执行一次，执行完需要重启(需要配置时宏定义)*********/

#if (defined KS10X_SCL) && (defined KS109_DEBUG)		//配置为测量过程SCL不被拉低，不配置的话会影响其他模块使用	
			
	KS10X_command(0xc3);	
	delay_us(100);

#elif (defined KS109_INIT) && (defined KS109_DEBUG)		//配置KS109波束角10°，带温度补偿

	KS10X_command(0x9c);	   //三个时序指令
	delay_ms(2);
	KS10X_command(0x95);
	delay_ms(2);
	KS10X_command(0x98);
	delay_ms(2);
	KS10X_command(0x7d);	  //配置模式


//#elif (defined KS103_INIT) && (defined KS109_DEBUG)	   //配置KS103，没什么配...

#endif



#ifdef KS109_DEBUG			    //配置后需要2秒让传感器配置，之后需要手动重启模块

	delay_ms(1000);
	delay_ms(1000);
#else						   //若不是配置模式，发送第一次获取高度指令

	KS10X_command(0xbc);

#endif
}



void KS10X_Get_High(void)
{
#ifndef KS109_DEBUG

	u16 result;
	delay_us(80);
	result = KS10X_Get_Result( );		//获取结果
	if(result < 0x2c10)					//防止超过11.280m(最大值)
	{
		KS10X_high = result;
		stQuadrotor_State.KS10X_High = KS10X_high;
	}	
	delay_us(80);
	KS10X_command(0xbc);	   //发送指令，让传感器探测

#endif
}




void KS10X_command(u8 command)			//发送指令
{
	
	KS10X_I2C_Start( );

	KS10X_I2C_Send_Data(KS10X_ADDRESS);
	KS10X_I2C_Send_Data(0x02);
	KS10X_I2C_Send_Data(command);

	KS10X_I2C_Stop( );



}

u16 KS10X_Get_Result(void)			   //获取结果(时序2)
{
	u16 data;
	KS10X_I2C_Start( );
	KS10X_I2C_Send_Data(KS10X_ADDRESS+1);

	delay_us(62);

	data = KS10X_I2C_Read_Data(NACK) ;
	data <<= 8;
	
	KS10X_I2C_Start( );
	KS10X_I2C_Send_Data(KS10X_ADDRESS+1);

	delay_us(62);

	data |= KS10X_I2C_Read_Data(NACK) ;

	KS10X_I2C_Stop( );

	return data;

}

/***********为KS10X的I2C速度制定的I2C函数**************/


/************************发送IIC开始信号*************************/ 
void KS10X_I2C_Start(void)
{         
	I2C_SDA=1;	  	  
	I2C_SCL=1;
	delay_us(5);
 	I2C_SDA=0;            //I2C时钟线SCL为高时拉低I2C数据线SDA，发送开始信号 
	delay_us(5);
	I2C_SCL=0;            //拉低I2C时钟线，准备数据传输 
}	 

 
/*************************发送I2C结束信号**************************/
void KS10X_I2C_Stop(void)
{          
	I2C_SCL=0;
	I2C_SDA=0;         
 	delay_us(5);
	I2C_SCL=1; 
	I2C_SDA=1;    	     //I2C时钟线SCL为高时拉高I2C数据线SDA，发送结束信号 
	delay_us(5);	
	I2C_SCL=0;						   	
}

/*****************等待应答信号，返回1即收到应答，0即没有收到************/ 
u8 KS10X_I2C_Wait_Ack(void)
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
			SDA_OUT(); 
			I2C_Stop();	
			return 0;
		}
	}
	I2C_SCL=0;                     //时钟输出0 	
	SDA_OUT();    
	return 1;  
}

/********************产生ACK应答*****************/
void KS10X_I2C_Ack(void)		  //应答信号位SCL为高电平时，SDA为低电平
{
	I2C_SCL=0;
	I2C_SDA=0;
	I2C_SCL=1;
	delay_us(5);
	I2C_SCL=0;
	delay_us(5);
}
/****************************不产生ACK应答**************************/		    
void KS10X_I2C_NAck(void)		//非应答信号位SCL为高电平时，SDA为高电平
{
	I2C_SCL=0;
	I2C_SDA=1;
	I2C_SCL=1;
	delay_us(5);
	I2C_SCL=0;
	delay_us(5);
}	

				 				     
/*****************************I2C发送一个字节数据***************************/
//返回1,从机有应答;0，无应答		  
u8 KS10X_I2C_Send_Data(u8 data)
{                        
    u8 i;   	    
    I2C_SCL=0;                       //拉低时钟开始数据传输
    for(i = 0;i < 8; i++)
    {              
        I2C_SDA=data >> 7;           //(data&0x80)>>7;		//从最高位开始取
        data<<=1; 	  
		//delay_us(1);                 
		I2C_SCL=1;
		delay_us(5); 
		I2C_SCL=0;	
		delay_us(5);
    }	
	KS10X_I2C_Wait_Ack( ); 
	return 1; 
} 	   


 
/******************************读1个字节*****************************/
//参数ack=1时，发送ACK，ack=0，发送nACK   
u8 KS10X_I2C_Read_Data(u8 ack)
{
	unsigned char i,receive=0;
	SDA_IN();                        //SDA设置为输入
    for(i=0;i<8;i++ )                //高位先发送 
	{
        I2C_SCL=0; 
        delay_us(5);
		I2C_SCL=1;
        receive<<=1;
        if(READ_SDA)
		{
			receive++;   	//读到该位为1，+1（即置1）
		}
		delay_us(5); 
    }		
	SDA_OUT(); 			 
    if (ack)
		KS10X_I2C_Ack();                   //发送ACK
    else
		KS10X_I2C_NAck();                  //发送nACK
           
    return receive;
} 

