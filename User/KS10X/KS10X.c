#include "KS10X.h"
#include "I2C.h"
#include "delay.h"
#include "Setup.h"

/*************��������ģʽ**************/
//#define KS109_DEBUG

/**************���в�ͬ����(һ��ֻ����һ��)*********************/
//#define KS10X_SCL		   //��������SCL��������
//#define KS109_INIT	   //����KS109

u16 KS10X_high = 0;



void KS10X_init(void)	  
{
	delay_ms(1000);		   //ģ���ʼ��
	delay_ms(1000);


/********************��������ֻ��Ҫִ��һ�Σ�ִ������Ҫ����(��Ҫ����ʱ�궨��)*********/

#if (defined KS10X_SCL) && (defined KS109_DEBUG)		//����Ϊ��������SCL�������ͣ������õĻ���Ӱ������ģ��ʹ��	
			
	KS10X_command(0xc3);	
	delay_us(100);

#elif (defined KS109_INIT) && (defined KS109_DEBUG)		//����KS109������10�㣬���¶Ȳ���

	KS10X_command(0x9c);	   //����ʱ��ָ��
	delay_ms(2);
	KS10X_command(0x95);
	delay_ms(2);
	KS10X_command(0x98);
	delay_ms(2);
	KS10X_command(0x7d);	  //����ģʽ


//#elif (defined KS103_INIT) && (defined KS109_DEBUG)	   //����KS103��ûʲô��...

#endif



#ifdef KS109_DEBUG			    //���ú���Ҫ2���ô��������ã�֮����Ҫ�ֶ�����ģ��

	delay_ms(1000);
	delay_ms(1000);
#else						   //����������ģʽ�����͵�һ�λ�ȡ�߶�ָ��

	KS10X_command(0xbc);

#endif
}



void KS10X_Get_High(void)
{
#ifndef KS109_DEBUG

	u16 result;
	delay_us(80);
	result = KS10X_Get_Result( );		//��ȡ���
	if(result < 0x2c10)					//��ֹ����11.280m(���ֵ)
	{
		KS10X_high = result;
		stQuadrotor_State.KS10X_High = KS10X_high;
	}	
	delay_us(80);
	KS10X_command(0xbc);	   //����ָ��ô�����̽��

#endif
}




void KS10X_command(u8 command)			//����ָ��
{
	
	KS10X_I2C_Start( );

	KS10X_I2C_Send_Data(KS10X_ADDRESS);
	KS10X_I2C_Send_Data(0x02);
	KS10X_I2C_Send_Data(command);

	KS10X_I2C_Stop( );



}

u16 KS10X_Get_Result(void)			   //��ȡ���(ʱ��2)
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

/***********ΪKS10X��I2C�ٶ��ƶ���I2C����**************/


/************************����IIC��ʼ�ź�*************************/ 
void KS10X_I2C_Start(void)
{         
	I2C_SDA=1;	  	  
	I2C_SCL=1;
	delay_us(5);
 	I2C_SDA=0;            //I2Cʱ����SCLΪ��ʱ����I2C������SDA�����Ϳ�ʼ�ź� 
	delay_us(5);
	I2C_SCL=0;            //����I2Cʱ���ߣ�׼�����ݴ��� 
}	 

 
/*************************����I2C�����ź�**************************/
void KS10X_I2C_Stop(void)
{          
	I2C_SCL=0;
	I2C_SDA=0;         
 	delay_us(5);
	I2C_SCL=1; 
	I2C_SDA=1;    	     //I2Cʱ����SCLΪ��ʱ����I2C������SDA�����ͽ����ź� 
	delay_us(5);	
	I2C_SCL=0;						   	
}

/*****************�ȴ�Ӧ���źţ�����1���յ�Ӧ��0��û���յ�************/ 
u8 KS10X_I2C_Wait_Ack(void)
{
	u8 WaitTime=0;
	SDA_IN();                      //SDA����Ϊ����  
	I2C_SDA=1;					   //��������
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
	I2C_SCL=0;                     //ʱ�����0 	
	SDA_OUT();    
	return 1;  
}

/********************����ACKӦ��*****************/
void KS10X_I2C_Ack(void)		  //Ӧ���ź�λSCLΪ�ߵ�ƽʱ��SDAΪ�͵�ƽ
{
	I2C_SCL=0;
	I2C_SDA=0;
	I2C_SCL=1;
	delay_us(5);
	I2C_SCL=0;
	delay_us(5);
}
/****************************������ACKӦ��**************************/		    
void KS10X_I2C_NAck(void)		//��Ӧ���ź�λSCLΪ�ߵ�ƽʱ��SDAΪ�ߵ�ƽ
{
	I2C_SCL=0;
	I2C_SDA=1;
	I2C_SCL=1;
	delay_us(5);
	I2C_SCL=0;
	delay_us(5);
}	

				 				     
/*****************************I2C����һ���ֽ�����***************************/
//����1,�ӻ���Ӧ��;0����Ӧ��		  
u8 KS10X_I2C_Send_Data(u8 data)
{                        
    u8 i;   	    
    I2C_SCL=0;                       //����ʱ�ӿ�ʼ���ݴ���
    for(i = 0;i < 8; i++)
    {              
        I2C_SDA=data >> 7;           //(data&0x80)>>7;		//�����λ��ʼȡ
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


 
/******************************��1���ֽ�*****************************/
//����ack=1ʱ������ACK��ack=0������nACK   
u8 KS10X_I2C_Read_Data(u8 ack)
{
	unsigned char i,receive=0;
	SDA_IN();                        //SDA����Ϊ����
    for(i=0;i<8;i++ )                //��λ�ȷ��� 
	{
        I2C_SCL=0; 
        delay_us(5);
		I2C_SCL=1;
        receive<<=1;
        if(READ_SDA)
		{
			receive++;   	//������λΪ1��+1������1��
		}
		delay_us(5); 
    }		
	SDA_OUT(); 			 
    if (ack)
		KS10X_I2C_Ack();                   //����ACK
    else
		KS10X_I2C_NAck();                  //����nACK
           
    return receive;
} 

