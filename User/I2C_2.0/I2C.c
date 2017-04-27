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
			I2C_SCL=0;                     //ʱ�����0
			SDA_OUT();   //��Ϊ���֮ǰI2C_SCL�ǵñ�Ϊ�͵�ƽ������ᷢ����ʼ���߽����ź�
			I2C_Stop();
			return 0;
		}
		// delay_us(1);
	}
	I2C_SCL=0;                     //ʱ�����0 	
	SDA_OUT();    //��Ϊ���֮ǰI2C_SCL�ǵñ�Ϊ�͵�ƽ������ᷢ����ʼ���߽����ź�
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
	return I2C_Wait_Ack( ); 
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
	I2C_SCL=0; 	
	SDA_OUT(); 		//��Ϊ���֮ǰI2C_SCL�ǵñ�Ϊ�͵�ƽ������ᷢ����ʼ���߽����ź�		 
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








/*************************I2C����������ֱ�Ӷ�д************************************/

//��ĳ����ַ����һ���ֽ�
void I2C_SendByte(u8 device_address,u8 address,u8 data)	  //�豸��ַ��ҪԤ�����һλ��д״̬λ
{
	I2C_Start();  
	I2C_Send_Data(device_address);   		//����������ַ,д��ģʽ 	 
	I2C_Send_Data(address);     		        //���Ͳ�����ַ							      	 										  		   
	I2C_Send_Data(data);     		        //��������							     		    	   
    I2C_Stop();
	//delay_ms(10); 
}

//��ָ�����͵ĵ�ַ������һ���ֽ�����
void I2C_SendByte_NoAddr(u8 device_address,u8 data)	  //�豸��ַ��ҪԤ�����һλ��д״̬λ
{
	I2C_Start();  
	I2C_Send_Data(device_address);   		//����������ַ,д��ģʽ 	   	 										  		   
	I2C_Send_Data(data);     		        //��������							   		    	   
    I2C_Stop();
	//delay_ms(10); 
}

//��ĳ����ַ���Ͷ���ֽ�
u8 I2C_SendBytes(u8 device_address,u8 address, uint8_t len, uint8_t *data)
{
	int i;
    I2C_Start();
    I2C_Send_Data(device_address);
    I2C_Send_Data(address);
	for (i = 0; i < len; i++) 
	{
        I2C_Send_Data(data[i]);
    }
    I2C_Stop();
    return 0;
}
//��ĳ����ַ��ȡһ���ֽ�
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

//��ĳ����ַ��ȡ����ֽ�
u8 I2C_ReadBytes(u8 device_address, u8 address, uint8_t len, uint8_t *buf)
{
    I2C_Start();
    I2C_Send_Data(device_address);
    I2C_Send_Data(address);
    I2C_Start();
	I2C_Send_Data(device_address + 1);
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
    return 0;
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



/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 I2C_WriteBits(u8 device_address,u8 address,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	device_address  Ŀ���豸��ַ
		address	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ����λ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   void

˵��:
|7bit|6bit|5bit|4bit|3bit|2bit|1bit|0bit|
Ҫ��3bit��Ϊ0��4bit��Ϊ1��
�����dataӦ��Ϊ10��bitStartΪ4��lengthΪ2
������̣�
mask����0xFF << (bitStart + 1)���õ�һ��Ŀ��λ���ȫ��1����11100000
0xFF >> (7 - bitStart + length)�õ�Ŀ��λ�ұ�ȫ��1����0000011
������λ�򼴿ɵõ�һ��Ŀ��λȫΪ0�����������Ϳ�������Ŀ��λ11100011
�ٰ�data�ƶ� 8-length=6,��������ߣ�����������ֻʣ������λ����ȫ������11000000
�����ƶ�(7 - bitStart) = 00011000
��mask��������data��ֵ�õ�Ŀ����
*******************************************************************************/ 
void I2C_WriteBits(u8 device_address,u8 address,u8 bitStart,u8 length,u8 data)
{
	u8 read_data = I2C_ReadByte(device_address, address);
	//Ҫд��ļ���λmask��Ϊ1
	u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> (7 - bitStart + length);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	read_data &= mask;	//����д�ļ���λ����
	read_data |= data;
	I2C_SendByte(device_address, address, read_data);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 I2C_WriteBit(u8 device_address, u8 address, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	device_address  Ŀ���豸��ַ
		address	   �Ĵ�����ַ
		bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
		data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   void
*******************************************************************************/ 
void I2C_WriteBit(u8 device_address, u8 address, u8 bitNum, u8 data)
{
    u8 read_data = I2C_ReadByte(device_address, address);
    read_data = (data != 0) ? (read_data | (1 << bitNum)) : (read_data & ~(1 << bitNum));
    I2C_SendByte(device_address, address, read_data);
}
