#include "I2C.h" 
#include "delay.h"
//#include "USART.h"



/*******************************I2C��������***********************************/


/************************����IIC��ʼ�ź�*************************/ 
void I2C_Start(void)
{
	I2C_SDA=1;	  	  
	I2C_SCL=1;
//	delay_us(1);
 	I2C_SDA=0;            //I2Cʱ����SCLΪ��ʱ����I2C������SDA�����Ϳ�ʼ�ź� 
//	delay_us(1);
	I2C_SCL=0;            //����I2Cʱ���ߣ�׼�����ݴ��� 
}	 



 
/*************************����I2C�����ź�**************************/
void I2C_Stop(void)
{          
	I2C_SCL=0;
	I2C_SDA=0;         
// 	delay_us(1);
	I2C_SCL=1; 
	I2C_SDA=1;    	     //I2Cʱ����SCLΪ��ʱ����I2C������SDA�����ͽ����ź� 
//	delay_us(1);	
	I2C_SCL=0;						   	
}



/*****************�ȴ�Ӧ���źţ�����1���յ�Ӧ��0��û���յ�************/ 
u8 I2C_Wait_Ack(void)
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
			I2C_Stop();
			return 0;
		}
	}
	I2C_SCL=0;                     //ʱ�����0 	
	SDA_OUT();    
	return 1;  
}
 
/********************����ACKӦ��*****************/
void I2C_Ack(void)		  //Ӧ���ź�λSCLΪ�ߵ�ƽʱ��SDAΪ�͵�ƽ
{
	I2C_SCL=0;
	I2C_SDA=0;
////	delay_us(1);
	I2C_SCL=1;
//	delay_us(1);
	I2C_SCL=0;
}
/****************************������ACKӦ��**************************/		    
void I2C_NAck(void)		//��Ӧ���ź�λSCLΪ�ߵ�ƽʱ��SDAΪ�ߵ�ƽ
{
	I2C_SCL=0;
	I2C_SDA=1;
//	delay_us(1);
	I2C_SCL=1;
//	delay_us(1);
	I2C_SCL=0;
}	

				 				     
/*****************************I2C����һ���ֽ�����***************************/
//����1,�ӻ���Ӧ��;0����Ӧ��		  
u8 I2C_Send_Data(u8 data)
{                        
    u8 i;   	    
    I2C_SCL=0;                       //����ʱ�ӿ�ʼ���ݴ���
    for(i = 0;i < 8; i++)
    {              
        I2C_SDA=data >> 7;           //(data&0x80)>>7;		//�����λ��ʼȡ
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


 
/******************************��1���ֽ�*****************************/
//����ack=1ʱ������ACK��ack=0������nACK   
u8 I2C_Read_Data(u8 ack)
{
	unsigned char i,receive=0;
	SDA_IN();                        //SDA����Ϊ����
    for(i=0;i<8;i++ )                //��λ�ȷ��� 
	{
        I2C_SCL=0; 
        //delay_us(1);
		I2C_SCL=1;
        receive<<=1;
        if(READ_SDA)
		{
			receive++;   	//������λΪ1��+1������1��
		}
		//delay_us(1); 
    }		
	SDA_OUT(); 			 
    if (ack)
		I2C_Ack();                   //����ACK
    else
		I2C_NAck();                  //����nACK
           
    return receive;
} 



/**************************I2C��ϲ���************************************/


void I2C_SendMode(u8 device_address,u8 address)
{
	I2C_Start();  
	I2C_Send_Data(device_address);   		//����������ַ,д��ģʽ 	 
	I2C_Send_Data(address);     		        //���Ͳ�����ַ							   
	
}




void I2C_ReadMode(u8 device_address,u8 address)
{
	I2C_Start();  
	I2C_Send_Data(device_address);   		//����������ַ,д��ģʽ 	 
	I2C_Send_Data(address);     		        //���Ͳ�����ַ							   

	I2C_Start();  	 	                   //�������� 
	I2C_Send_Data(device_address + 1);   	//����������ַ,������ģʽ		    
	
}








/*************************I2C����������ֱ�Ӷ�ȡ************************************/


void I2C_SendByte(u8 device_address,u8 address,u8 data)	  //�豸��ַ��ҪԤ�����һλ��д״̬λ
{
	I2C_Start();  
	I2C_Send_Data(device_address);   		//����������ַ,д��ģʽ 	 
	I2C_Send_Data(address);     		        //���Ͳ�����ַ							      	 										  		   
	I2C_Send_Data(data);     		        //��������							     		    	   
    I2C_Stop();
	//delay_ms(10); 
}


void I2C_SendByte_NoAddr(u8 device_address,u8 data)	  //�豸��ַ��ҪԤ�����һλ��д״̬λ
{
	I2C_Start();  
	I2C_Send_Data(device_address);   		//����������ַ,д��ģʽ 	   	 										  		   
	I2C_Send_Data(data);     		        //��������							   		    	   
    I2C_Stop();
	//delay_ms(10); 
}

u8 I2C_ReadByte(u8 device_address,u8 address)	  //�豸��ַ��ҪԤ�����һλ��д״̬λ
{
	u8 data=0;		  	    																 
    I2C_Start();                           //���Ϳ�ʼ���� 
	
	I2C_Send_Data(device_address);  		//����������ַ,д���� 	                        //�ȴ��ӻ�Ӧ�� 
    I2C_Send_Data(address);                //���Ͳ�����ַ	            
	 
	I2C_Start();  	 	                   //�������� 
	I2C_Send_Data(device_address + 1);   	//����������ַ,������ģʽ		   	  
    data = I2C_Read_Data(NACK);		   		   //�������ݣ���Ӧ�� 
    I2C_Stop();                            //����ֹͣ�ź�	    
	return data;
}

//���豸������ȡ���豸Ӧ�û��Զ���������ַ����
u16 I2C_Read_16(u8 device_address,u8 address)	  //�豸��ַ��ҪԤ�����һλ��д״̬λ
{
	u16 data=0;		  	    																 
    I2C_Start();                           //���Ϳ�ʼ���� 
	
	I2C_Send_Data(device_address);  		//����������ַ,д���� 	                     
    I2C_Send_Data(address);                //���Ͳ�����ַ           
	 
	I2C_Start();  	 	                   //�������� 
	I2C_Send_Data(device_address + 1);   	//����������ַ,������		   	
  
    data = I2C_Read_Data(ACK);		   		   //�������ݣ�Ӧ��
	data <<= 8;		   
	data |= I2C_Read_Data(NACK);

    I2C_Stop();                            //����ֹͣ�ź�	    
	return data;
}

u32 I2C_Read_32(u8 device_address,u8 address)	  //�豸��ַ��ҪԤ�����һλ��д״̬λ
{
	u32 data=0;		  	    																 
    I2C_Start();                           //���Ϳ�ʼ���� 
	
	I2C_Send_Data(device_address);  		//����������ַ,д���� 	              
    I2C_Send_Data(address);                //���Ͳ�����ַ           
	 
	I2C_Start();  	 	                   //�������� 
	I2C_Send_Data(device_address + 1);   	//����������ַ,������		   
  
	data = I2C_Read_Data(ACK);		   		   //�������ݣ�Ӧ��
	data <<= 8;	
    data |= I2C_Read_Data(ACK);		   		   //�������ݣ�Ӧ��
	data <<= 8;	
	data |= I2C_Read_Data(ACK);		   //�������ݣ�Ӧ��
	data <<= 8; 	   
	data |= I2C_Read_Data(NACK);

    I2C_Stop();                            //����ֹͣ�ź�	    
	return data;
}
