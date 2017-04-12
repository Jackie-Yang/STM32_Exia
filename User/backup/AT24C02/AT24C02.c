#include "AT24C02.h" 




/***************************写入一个数据***************************/
//参数:    address:写入的地址  data：要写入的数据  
void AT24C02_WriteOneByte(u8 address,u8 data)
{				   	  	    																 
    I2C_Start();  
	I2C_Send_Byte(ADDRESS_24C02 | WRITE);   //发送器件地址,写入模式 	 
	I2C_Wait_Ack();	   
    I2C_Send_Byte(address);       		    //发送操作地址
	I2C_Wait_Ack(); 	 										  		   
	I2C_Send_Byte(data);     		        //发送数据							   
	I2C_Wait_Ack();  		    	   
    I2C_Stop();
	delay_ms(10); 
}


/**********************读取一个数据*************************/
//参数：读取地址 
//返回值  :读到的数据
u8 AT24C02_ReadOneByte(u8 address)
{				  
	u8 data=0;		  	    																 
    I2C_Start();                           //发送开始命令 
	
	I2C_Send_Byte(ADDRESS_24C02 | WRITE);  //发送器件地址,写数据 	 
	I2C_Wait_Ack();                        //等待从机应答 
    I2C_Send_Byte(address);                //发送操作地址
	I2C_Wait_Ack();	            
	 
	I2C_Start();  	 	                   //重启总线 
	I2C_Send_Byte(ADDRESS_24C02 | READ);   //进入接收模式			   
	I2C_Wait_Ack();	  
    data = I2C_Read_Byte(0);		   		   //接收数据 
    I2C_Stop();                            //发送停止信号	    
	return data;
}






/***********************写入一定长度的数据**********************/
//参数： 写入的地址  ，写入的数据长度
void AT24C02_WriteBytes(u8 address,u32 data,u8 dataSize)
{  	
	u8 byte;
	for(byte = 0; byte < dataSize; byte++)     //一个字节一个字节取出写入 
	{
		AT24C02_WriteOneByte(address + byte, ( data >> (8 * byte) ) & 0xff);    
	}												    
}


/***********************读取一定长度的数据**********************/
//参数： 读取的地址  ，读取的数据长度
//返回值：读取的数据 
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


/***************写入一定个数的数据********************/ 
//参数： 写入的地址，第一个数据地址，数据的个数 
void AT24C02_Write(u8 address,u8 *data,u16 dataNum)
{
	while(dataNum--)
	{
		AT24C02_WriteOneByte(address,*data);
		address++;
		data++;
	}
}


/***************读取一定个数的数据********************/ 
//参数： 读取的地址，第一个数据地址，数据的个数 
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
	}while(++dataNum);	                  //一个字节的变量，加到256即为0
}



