#include "I2C.h" 


/************************����IIC��ʼ�ź�*************************/ 
void I2C_Start(void)
{
	SDA_OUT();          
	I2C_SDA=1;	  	  
	I2C_SCL=1;
	delay_us(4);
 	I2C_SDA=0;            //������I2C������ 
	delay_us(4);
	I2C_SCL=0;            //����I2Cʱ���ߣ�׼�����ݴ��� 
}	 



 
/*************************����I2C�����ź�**************************/
void I2C_Stop(void)
{
	SDA_OUT();           
	I2C_SCL=0;
	I2C_SDA=0;         
 	delay_us(4);
	I2C_SCL=1; 
	I2C_SDA=1;    	     //SDA��Ӧ��SCLʱ���߸ߵ�ƽʱ�ı䣬������Ϊ�����ź� 
	delay_us(4);							   	
}



/*****************�ȴ�Ӧ���źţ�����1���յ�Ӧ��0��û���յ�************/ 
u8 I2C_Wait_Ack(void)
{
	u8 ErrTime=0;
	SDA_IN();                      //SDA����Ϊ����  
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
	I2C_SCL=0;                     //ʱ�����0 	   
	return 0;  
}
 
/********************����ACKӦ��*****************/
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
/****************************������ACKӦ��**************************/		    
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
/*****************************I2C����һ���ֽ�***************************/
//����1,�ӻ���Ӧ��;0����Ӧ��		  
void I2C_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    I2C_SCL=0;                       //����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        I2C_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);                 //��TEA5767��������ʱ���Ǳ����
		I2C_SCL=1;
		delay_us(2); 
		I2C_SCL=0;	
		delay_us(2);
    }	 
} 	    
/******************************��1���ֽ�*****************************/
//����ack=1ʱ������ACK��ack=0������nACK   
u8 I2C_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();                        //SDA����Ϊ����
    for(i=0;i<8;i++ )                //��λ�ȷ��� 
	{
        I2C_SCL=0; 
        delay_us(2);
		I2C_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        I2C_NAck();                  //����nACK
    else
        I2C_Ack();                   //����ACK   
    return receive;
} 
