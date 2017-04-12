#include "AT24C02.h" 




/***************************д��һ������***************************/
//����:    address:д��ĵ�ַ  data��Ҫд�������  
void AT24C02_WriteOneByte(u8 address,u8 data)
{				   	  	    																 
    I2C_Start();  
	I2C_Send_Byte(ADDRESS_24C02 | WRITE);   //����������ַ,д��ģʽ 	 
	I2C_Wait_Ack();	   
    I2C_Send_Byte(address);       		    //���Ͳ�����ַ
	I2C_Wait_Ack(); 	 										  		   
	I2C_Send_Byte(data);     		        //��������							   
	I2C_Wait_Ack();  		    	   
    I2C_Stop();
	delay_ms(10); 
}


/**********************��ȡһ������*************************/
//��������ȡ��ַ 
//����ֵ  :����������
u8 AT24C02_ReadOneByte(u8 address)
{				  
	u8 data=0;		  	    																 
    I2C_Start();                           //���Ϳ�ʼ���� 
	
	I2C_Send_Byte(ADDRESS_24C02 | WRITE);  //����������ַ,д���� 	 
	I2C_Wait_Ack();                        //�ȴ��ӻ�Ӧ�� 
    I2C_Send_Byte(address);                //���Ͳ�����ַ
	I2C_Wait_Ack();	            
	 
	I2C_Start();  	 	                   //�������� 
	I2C_Send_Byte(ADDRESS_24C02 | READ);   //�������ģʽ			   
	I2C_Wait_Ack();	  
    data = I2C_Read_Byte(0);		   		   //�������� 
    I2C_Stop();                            //����ֹͣ�ź�	    
	return data;
}






/***********************д��һ�����ȵ�����**********************/
//������ д��ĵ�ַ  ��д������ݳ���
void AT24C02_WriteBytes(u8 address,u32 data,u8 dataSize)
{  	
	u8 byte;
	for(byte = 0; byte < dataSize; byte++)     //һ���ֽ�һ���ֽ�ȡ��д�� 
	{
		AT24C02_WriteOneByte(address + byte, ( data >> (8 * byte) ) & 0xff);    
	}												    
}


/***********************��ȡһ�����ȵ�����**********************/
//������ ��ȡ�ĵ�ַ  ����ȡ�����ݳ���
//����ֵ����ȡ������ 
u32 AT24C02_ReadBytes(u8 address,u8 dataSize)
{  	
	u8 byte;
	u32 data = 0;
	for(byte = 0; byte < dataSize; byte++)
	{
		data |= AT24C02_ReadOneByte(address + byte) << (8 * byte); 	 				   
	}
	return data;												    
}


/***************д��һ������������********************/ 
//������ д��ĵ�ַ����һ�����ݵ�ַ�����ݵĸ��� 
void AT24C02_Write(u8 address,u8 *data,u16 dataNum)
{
	while(dataNum--)
	{
		AT24C02_WriteOneByte(address,*data);
		address++;
		data++;
	}
}


/***************��ȡһ������������********************/ 
//������ ��ȡ�ĵ�ַ����һ�����ݵ�ַ�����ݵĸ��� 
void AT24C02_Read(u8 address,u8 *data,u16 dataNum)
{
	while(dataNum--)
	{
		*data = AT24C02_ReadOneByte(address);	
		address++;
		data++;
	}
} 

void AT24C02_Erase(void) 
{
	u8 dataNum = 0;
	do
	{
		AT24C02_WriteOneByte(dataNum,0);
	}while(++dataNum);	                  //һ���ֽڵı������ӵ�256��Ϊ0
}



