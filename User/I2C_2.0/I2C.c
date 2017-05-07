#include "I2C.h" 
#include "delay.h"
//#include "USART.h"

uint32_t I2C_HoldTime = 2;

/*******************************I2C��������***********************************/


/************************����IIC��ʼ�ź�*************************/ 
void I2C_Start(void)
{
	I2C_SDA=1;	  	  
	I2C_SCL=1;
	// delay_us(I2C_HoldTime);
	I2C_SDA=0;            //I2Cʱ����SCLΪ��ʱ����I2C������SDA�����Ϳ�ʼ�ź�
	// delay_us(I2C_HoldTime);
	I2C_SCL=0;            //����I2Cʱ���ߣ�׼�����ݴ��� 
}	 



 
/*************************����I2C�����ź�**************************/
void I2C_Stop(void)
{          
	I2C_SCL=0;
	I2C_SDA=0;
	// delay_us(I2C_HoldTime);
	I2C_SCL=1; 
	I2C_SDA=1;    	     //I2Cʱ����SCLΪ��ʱ����I2C������SDA�����ͽ����ź� 
	// delay_us(I2C_HoldTime);	
	I2C_SCL=0;						   	
}



/*****************�ȴ�Ӧ���źţ�����0���յ�Ӧ��-1��û���յ�************/ 
s8 I2C_Wait_Ack(void)
{
	u8 WaitTime=0;
	SDA_IN();                      //SDA����Ϊ����  
	I2C_SDA=1;					   //��������
////	delay_us(I2C_HoldTime);	   
	I2C_SCL=1;
////	delay_us(I2C_HoldTime);	   
	while(READ_SDA)
	{
		if(WaitTime++>250)
		{
			I2C_SCL=0;                     //ʱ�����0
			SDA_OUT();   //��Ϊ���֮ǰI2C_SCL�ǵñ�Ϊ�͵�ƽ������ᷢ����ʼ���߽����ź�
			I2C_Stop();
			return -1;
		}
		// delay_us(I2C_HoldTime);
	}
	I2C_SCL=0;                     //ʱ�����0 	
	SDA_OUT();    //��Ϊ���֮ǰI2C_SCL�ǵñ�Ϊ�͵�ƽ������ᷢ����ʼ���߽����ź�
	return 0;  
}
 
/********************����ACKӦ��*****************/
void I2C_Ack(void)		  //Ӧ���ź�λSCLΪ�ߵ�ƽʱ��SDAΪ�͵�ƽ
{
	I2C_SCL=0;
	I2C_SDA=0;
////	delay_us(I2C_HoldTime);
	I2C_SCL=1;
	// delay_us(I2C_HoldTime);
	I2C_SCL=0;
}
/****************************������ACKӦ��**************************/		    
void I2C_NAck(void)		//��Ӧ���ź�λSCLΪ�ߵ�ƽʱ��SDAΪ�ߵ�ƽ
{
	I2C_SCL=0;
	I2C_SDA=1;
	// delay_us(I2C_HoldTime);
	I2C_SCL=1;
	// delay_us(I2C_HoldTime);
	I2C_SCL=0;
}	

				 				     
/*****************************I2C����һ���ֽ�����***************************/
//����1,�ӻ���Ӧ��;0����Ӧ��		  
s8 I2C_Send_Data(u8 data)
{                        
    u8 i;   	    
    I2C_SCL=0;                       //����ʱ�ӿ�ʼ���ݴ���
    for(i = 0;i < 8; i++)
    {              
        I2C_SDA=data >> 7;           //(data&0x80)>>7;		//�����λ��ʼȡ
        data<<=1; 	  
		// delay_us(I2C_HoldTime);                 
		I2C_SCL=1;
		// delay_us(I2C_HoldTime);                 
		I2C_SCL=0;	
		// delay_us(I2C_HoldTime);                 
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
        // delay_us(I2C_HoldTime);
		I2C_SCL=1;
        receive<<=1;
        if(READ_SDA)
		{
			receive++;   	//������λΪ1��+1������1��
		}
		// delay_us(I2C_HoldTime); 
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

s8 I2C_SendMode(u8 device_address, u8 address)
{
	s8 ret = 0;
	I2C_Start();
	ret |= I2C_Send_Data(device_address);		//����������ַ,д��ģʽ
	ret |= I2C_Send_Data(address);				//���Ͳ�����ַ
	return ret;
}




s8 I2C_ReadMode(u8 device_address,u8 address)
{
	s8 ret = 0;
	I2C_Start();
	ret |= I2C_Send_Data(device_address);		//����������ַ,д��ģʽ
	ret |= I2C_Send_Data(address);				//���Ͳ�����ַ

	I2C_Start();  	 	                   //��������
	ret |= I2C_Send_Data(device_address + 1); //����������ַ,������ģʽ
	return ret;
}








/*************************I2C����������ֱ�Ӷ�д************************************/

//��ĳ����ַ����һ���ֽ�
s8 I2C_SendByte(u8 device_address,u8 address,u8 data)	  //�豸��ַ��ҪԤ�����һλ��д״̬λ
{
	s8 ret = 0;
	I2C_Start();
	ret |= I2C_Send_Data(device_address);		//����������ַ,д��ģʽ
	ret |= I2C_Send_Data(address);				//���Ͳ�����ַ
	ret |= I2C_Send_Data(data);					//��������
	I2C_Stop();
	//delay_ms(10);
	return ret;
}

//��ָ�����͵ĵ�ַ������һ���ֽ�����
s8 I2C_SendByte_NoAddr(u8 device_address,u8 data)	  //�豸��ַ��ҪԤ�����һλ��д״̬λ
{
	s8 ret = 0;
	I2C_Start();
	ret |= I2C_Send_Data(device_address);   //����������ַ,д��ģʽ
	ret |= I2C_Send_Data(data);				//��������
	I2C_Stop();
	//delay_ms(10);
	return ret;
}

//��ĳ����ַ���Ͷ���ֽ�
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
//��ĳ����ַ��ȡһ���ֽ�
s8 I2C_ReadByte(u8 device_address, u8 address, u8 *data) //�豸��ַ��ҪԤ�����һλ��д״̬λ
{
	s8 ret = 0;
	I2C_Start();                           //���Ϳ�ʼ����

	ret |= I2C_Send_Data(device_address);  //����������ַ,д���� 	                        //�ȴ��ӻ�Ӧ��
	ret |= I2C_Send_Data(address);		   //���Ͳ�����ַ

	I2C_Start();  	 	                   //��������
	ret |= I2C_Send_Data(device_address + 1); //����������ַ,������ģʽ
	*data = I2C_Read_Data(NACK);			//�������ݣ���Ӧ��
	I2C_Stop();                            //����ֹͣ�ź�	    
	return ret;
}

//��ĳ����ַ��ȡ����ֽ�(��С��˳���ȡ���洢�����ȶ��������ݴ��ڵ͵�ַ�������ݵ�λ����������һ�����ȷ��͸�λ�ģ�����ô�˽϶�)
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

//��ĳ����ַ��ȡ����ֽڣ��������ܶ������ȶ����Ķ��Ǹ��ֽڵģ����Դ�˵�˳��õ����ݣ���STM32һ����С�ˣ����ֽ���ǰ����˴��˳��Ӧ���෴��
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
s8 I2C_WriteBits(u8 device_address,u8 address,u8 bitStart,u8 length,u8 data)
{
	s8 ret = 0;
	u8 mask = 0;
	u8 read_data = 0;
	ret |= I2C_ReadByte(device_address, address, &read_data);
	//Ҫд��ļ���λmask��Ϊ1
	mask = (0xFF << (bitStart + 1)) | 0xFF >> (7 - bitStart + length);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	read_data &= mask;	//����д�ļ���λ����
	read_data |= data;
	ret |= I2C_SendByte(device_address, address, read_data);
	return ret;
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
s8 I2C_WriteBit(u8 device_address, u8 address, u8 bitNum, u8 data)
{
	s8 ret = 0;
	u8 read_data = 0;
	ret |= I2C_ReadByte(device_address, address, &read_data);
	read_data = (data != 0) ? (read_data | (1 << bitNum)) : (read_data & ~(1 << bitNum));
	ret |= I2C_SendByte(device_address, address, read_data);
	return ret;
}
