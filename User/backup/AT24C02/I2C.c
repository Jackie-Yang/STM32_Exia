#include "I2C.h" 


/************************发送IIC开始信号*************************/ 
void I2C_Start(void)
{
	SDA_OUT();          
	I2C_SDA=1;	  	  
	I2C_SCL=1;
	delay_us(4);
 	I2C_SDA=0;            //先拉低I2C数据线 
	delay_us(4);
	I2C_SCL=0;            //拉低I2C时钟线，准备数据传输 
}	 



 
/*************************发送I2C结束信号**************************/
void I2C_Stop(void)
{
	SDA_OUT();           
	I2C_SCL=0;
	I2C_SDA=0;         
 	delay_us(4);
	I2C_SCL=1; 
	I2C_SDA=1;    	     //SDA线应在SCL时钟线高电平时改变，否则视为结束信号 
	delay_us(4);							   	
}



/*****************等待应答信号，返回1即收到应答，0即没有收到************/ 
u8 I2C_Wait_Ack(void)
{
	u8 ErrTime=0;
	SDA_IN();                      //SDA设置为输入  
	I2C_SDA=1;
	delay_us(1);	   
	I2C_SCL=1;
	delay_us(1);	 
	while(READ_SDA)
	{
		ErrTime++;
		if(ErrTime>250)
		{
			I2C_Stop();
			return 1;
		}
	}
	I2C_SCL=0;                     //时钟输出0 	   
	return 0;  
}
 
/********************产生ACK应答*****************/
void I2C_Ack(void)
{
	I2C_SCL=0;
	SDA_OUT();
	I2C_SDA=0;
	delay_us(2);
	I2C_SCL=1;
	delay_us(2);
	I2C_SCL=0;
}
/****************************不产生ACK应答**************************/		    
void I2C_NAck(void)
{
	I2C_SCL=0;
	SDA_OUT();
	I2C_SDA=1;
	delay_us(2);
	I2C_SCL=1;
	delay_us(2);
	I2C_SCL=0;
}					 				     
/*****************************I2C发送一个字节***************************/
//返回1,从机有应答;0，无应答		  
void I2C_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    I2C_SCL=0;                       //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        I2C_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);                 //对TEA5767这三个延时都是必须的
		I2C_SCL=1;
		delay_us(2); 
		I2C_SCL=0;	
		delay_us(2);
    }	 
} 	    
/******************************读1个字节*****************************/
//参数ack=1时，发送ACK，ack=0，发送nACK   
u8 I2C_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();                        //SDA设置为输入
    for(i=0;i<8;i++ )                //高位先发送 
	{
        I2C_SCL=0; 
        delay_us(2);
		I2C_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        I2C_NAck();                  //发送nACK
    else
        I2C_Ack();                   //发送ACK   
    return receive;
} 
