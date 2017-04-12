#include "MS5611.h"
#include "math.h"
#include "DMA.h"
#include "I2C.h"
#include "delay.h"
#include "filter.h"
#include "PID.h"


u16 PROM_C[6];  //用于存放PROM中的6组数据

u32 D2_Temp = 0,D1_Pres = 0;	//读取的温度传感器数据 ,读取的气压数据; 
double dT;  //读取的值与参考温度之间的差异

double MS5611_Temperature = 0,Last_Temperature = 0;	   //计算完成后的温度值
double MS5611_Pressure = 0,Last_Pressure = 0,Zero_Pressure = 0,Zero_T = 0;				//温度补偿大气压结果
double MS5611_high = 0;
double high_buf[10] = {0};



/************************************************************   
* 函数名:MS561101BA_Init   
* 描述 : MS561101BA初始化
* 输入  :无   
* 输出  :无    
*/ 
void MS5611_Init(void)
{
//	u8 i;
	MS5611_Reset();
	delay_ms(10);
	MS5611_readPROM();
	delay_ms(10);



//	for(i = 0;i < 20; i++)
//	{
//		MS5611_Read(MS561101BA_D2_OSR_4096);	  //发送指令让MS5611测量温度值
//		delay_ms(10);
//		D2_Temp = MS5611_Get( );			 	 //10ms后获取数据
//		delay_ms(10);
//		MS5611_Read(MS561101BA_D1_OSR_4096);	  //发送指令让MS5611测量气压值
//	    delay_ms(10);
//		D1_Pres = MS5611_Get( );
//		MS5611_GetPressure( );					//10ms后获取数据,计算温度、气压值
//		delay_ms(10);
//	}

//	Zero_Pressure = 0;			
					
} 

/************************************************************   
* 函数名:MS561101BA_Reset   
* 描述 : 复位  
* 输入  :无   
* 输出  :无    
*/ 
void MS5611_Reset(void)
{
	I2C_SendByte_NoAddr(MS561101BA_ADDR,MS561101BA_RESET);
}

/************************************************************   
* 函数名:MS561101BA_readPROM   
* 描述 : 从PROM读取出厂校准数据
* 输入  :无   
* 输出  :无    
  */
void MS5611_readPROM(void)
{   
	u8 i;
	for (i=1;i<=MS561101BA_PROM_REG_COUNT;i++) //逐个读取PROM
	{
		 PROM_C[i - 1]=I2C_Read_16(MS561101BA_ADDR,MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
	}
   
}

/************************************************************   
* 函数名:MS561101BA_DO_CONVERSION   
* 描述 : 向MS5611发送指令，读取数据(由于发送指令后，需要等待8ms才能读取数据，因此分开这两部，用定时器操作) 
* 输入  :无   
* 输出  :无    
*/
void MS5611_Read(uint8_t command)
{	
	I2C_Start();  
	I2C_Send_Data(MS561101BA_ADDR);   		//发送器件地址,写入模式 	 
	I2C_Send_Data(command);     		    //发送相应指令							   
	I2C_Stop();
}	    

	 										  		   
	//delay_ms(8);//延时,去掉数据错误
	//delay_us(800);


uint32_t MS5611_Get(void)
{
	uint32_t data = 0;
	I2C_Start(); 
	I2C_Send_Data(MS561101BA_ADDR);
	I2C_Send_Data(0);
		

	I2C_Start();  	 	                   //重启总线 
	I2C_Send_Data(MS561101BA_ADDR + 1);   	//发送器件地址,读数据		   
	

    data = I2C_Read_Data(ACK);		   		   //接收数据，应答
	data <<= 8;	
	data |= I2C_Read_Data(ACK);		   //接收数据，应答
	data <<= 8; 	   
	data |= I2C_Read_Data(NACK);
		  
    I2C_Stop();                            //发送停止信号
 
    return data;

}	

/************************************************************   
* 函数名:MS561101BA_GetTemperature   
* 描述 : 读取数字温度
* 输入  :无  
* 输出  :无    
*/
double MS5611_GetTemperature(void)
{
   	
	//D2_Temp = MS5611_Read(MS561101BA_D2_OSR_4096);  //读取温度传感器数据，参数为采样精度	
	
	dT=D2_Temp - ( ( (u32)PROM_C[4] ) << 8 );		//实际和参考温度之间的差异

	return ( 2000 + dT  / 8388608.0  * ( (uint32_t)PROM_C[5]));	//算出温度值的100倍，2001表示20.01°
}  



/************************************************************   
* 函数名:MS561101BA_GetPressure   
* 描述 : 读取数字气压
* 输入  :无  
* 输出  :无    
*/
void MS5611_GetPressure(void)
{
	double OFF,SENS;  //实际温度抵消,实际温度灵敏度
	double Aux,OFF2,SENS2;  //温度校验值
	double Temperature,Temperature2 = 0;
	
	
	Temperature = MS5611_GetTemperature();	//获取温度值
	//D1_Pres = MS5611_Read(MS561101BA_D1_OSR_4096);		  //读取气压传感器数据，参数为采样精度	

	
	OFF= ((uint32_t)PROM_C[1]<<16)+(PROM_C[3] / 128.0 * dT);
	SENS=((uint32_t)PROM_C[0]<<15)+(PROM_C[2] / 256.0 * dT);
	//二阶温度补偿
	if(Temperature < 2000)// second order temperature compensation when under 20 degrees C
	{
		Temperature2 = (dT*dT) / 0x80000000;
		Aux = (Temperature-2000)*(Temperature-2000);
		OFF2 = 2.5*Aux;
		SENS2 = 1.25*Aux;
		if(Temperature < -1500)
		{
			Aux = (Temperature+1500)*(Temperature+1500);
			OFF2 = OFF2 + 7*Aux;
			SENS2 = SENS + 5.5*Aux;
		}
	}else  //(Temperature > 2000)
	{
		Temperature2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	
	Temperature = Temperature - Temperature2;

//	Last_Temperature = MS5611_Temperature;

	MS5611_Temperature = Temperature;						//计算校正后温度
	DMA_Buff_In_32(MS5611_Temperature,MS5611_TEMP_INDEX);   //存入上位机发送缓存
	OFF = OFF - OFF2;	 	//进行温度补偿
	SENS = SENS - SENS2;
	MS5611_Pressure=(D1_Pres/2097152.0*SENS-OFF)/32768.0;  //计算校正后气压，发到上位机缓存
	DMA_Buff_In_32(MS5611_Pressure,MS5611_PRESS_INDEX);
		
    if(Zero_Pressure == 0)
	{	
		Zero_Pressure = MS5611_Pressure;   //如果没有参考高度气压，将当前气压当作参考高度气压
//		Zero_T =  (double)(MS5611_Temperature) / 100 + 273.15;
	}
	else
	{
		//用当前气压与参考高度气压计算出当前相对高度（不是很准）
	//	Last_Pressure = MS5611_Pressure;
//		MS5611_high += 18400 * (1 + (Last_Pressure + MS5611_Pressure) / 200.0 / 273) * log10( Last_Pressure / MS5611_Pressure);
		MS5611_high = 4433000.0 * (1 - pow((MS5611_Pressure / Zero_Pressure), 0.1903));
	//	MS5611_high = 15384.62 * Zero_T * ( 1 - exp( 0.190259 * log10(MS5611_Pressure / Zero_Pressure)));	  //高度乘100
		MS5611_high = high_filter(MS5611_high,high_buf);
	    DMA_Buff_In_32((int32_t)MS5611_high,MS5611_HIGH_INDEX);
		//High.high_cur = MS5611_high / 100.0f;
	}
	

}

