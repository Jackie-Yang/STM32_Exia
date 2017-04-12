#include "MS5611.h"
#include "math.h"
#include "DMA.h"
#include "I2C.h"
#include "delay.h"
#include "filter.h"
#include "PID.h"


u16 PROM_C[6];  //���ڴ��PROM�е�6������

u32 D2_Temp = 0,D1_Pres = 0;	//��ȡ���¶ȴ��������� ,��ȡ����ѹ����; 
double dT;  //��ȡ��ֵ��ο��¶�֮��Ĳ���

double MS5611_Temperature = 0,Last_Temperature = 0;	   //������ɺ���¶�ֵ
double MS5611_Pressure = 0,Last_Pressure = 0,Zero_Pressure = 0,Zero_T = 0;				//�¶Ȳ�������ѹ���
double MS5611_high = 0;
double high_buf[10] = {0};



/************************************************************   
* ������:MS561101BA_Init   
* ���� : MS561101BA��ʼ��
* ����  :��   
* ���  :��    
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
//		MS5611_Read(MS561101BA_D2_OSR_4096);	  //����ָ����MS5611�����¶�ֵ
//		delay_ms(10);
//		D2_Temp = MS5611_Get( );			 	 //10ms���ȡ����
//		delay_ms(10);
//		MS5611_Read(MS561101BA_D1_OSR_4096);	  //����ָ����MS5611������ѹֵ
//	    delay_ms(10);
//		D1_Pres = MS5611_Get( );
//		MS5611_GetPressure( );					//10ms���ȡ����,�����¶ȡ���ѹֵ
//		delay_ms(10);
//	}

//	Zero_Pressure = 0;			
					
} 

/************************************************************   
* ������:MS561101BA_Reset   
* ���� : ��λ  
* ����  :��   
* ���  :��    
*/ 
void MS5611_Reset(void)
{
	I2C_SendByte_NoAddr(MS561101BA_ADDR,MS561101BA_RESET);
}

/************************************************************   
* ������:MS561101BA_readPROM   
* ���� : ��PROM��ȡ����У׼����
* ����  :��   
* ���  :��    
  */
void MS5611_readPROM(void)
{   
	u8 i;
	for (i=1;i<=MS561101BA_PROM_REG_COUNT;i++) //�����ȡPROM
	{
		 PROM_C[i - 1]=I2C_Read_16(MS561101BA_ADDR,MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
	}
   
}

/************************************************************   
* ������:MS561101BA_DO_CONVERSION   
* ���� : ��MS5611����ָ���ȡ����(���ڷ���ָ�����Ҫ�ȴ�8ms���ܶ�ȡ���ݣ���˷ֿ����������ö�ʱ������) 
* ����  :��   
* ���  :��    
*/
void MS5611_Read(uint8_t command)
{	
	I2C_Start();  
	I2C_Send_Data(MS561101BA_ADDR);   		//����������ַ,д��ģʽ 	 
	I2C_Send_Data(command);     		    //������Ӧָ��							   
	I2C_Stop();
}	    

	 										  		   
	//delay_ms(8);//��ʱ,ȥ�����ݴ���
	//delay_us(800);


uint32_t MS5611_Get(void)
{
	uint32_t data = 0;
	I2C_Start(); 
	I2C_Send_Data(MS561101BA_ADDR);
	I2C_Send_Data(0);
		

	I2C_Start();  	 	                   //�������� 
	I2C_Send_Data(MS561101BA_ADDR + 1);   	//����������ַ,������		   
	

    data = I2C_Read_Data(ACK);		   		   //�������ݣ�Ӧ��
	data <<= 8;	
	data |= I2C_Read_Data(ACK);		   //�������ݣ�Ӧ��
	data <<= 8; 	   
	data |= I2C_Read_Data(NACK);
		  
    I2C_Stop();                            //����ֹͣ�ź�
 
    return data;

}	

/************************************************************   
* ������:MS561101BA_GetTemperature   
* ���� : ��ȡ�����¶�
* ����  :��  
* ���  :��    
*/
double MS5611_GetTemperature(void)
{
   	
	//D2_Temp = MS5611_Read(MS561101BA_D2_OSR_4096);  //��ȡ�¶ȴ��������ݣ�����Ϊ��������	
	
	dT=D2_Temp - ( ( (u32)PROM_C[4] ) << 8 );		//ʵ�ʺͲο��¶�֮��Ĳ���

	return ( 2000 + dT  / 8388608.0  * ( (uint32_t)PROM_C[5]));	//����¶�ֵ��100����2001��ʾ20.01��
}  



/************************************************************   
* ������:MS561101BA_GetPressure   
* ���� : ��ȡ������ѹ
* ����  :��  
* ���  :��    
*/
void MS5611_GetPressure(void)
{
	double OFF,SENS;  //ʵ���¶ȵ���,ʵ���¶�������
	double Aux,OFF2,SENS2;  //�¶�У��ֵ
	double Temperature,Temperature2 = 0;
	
	
	Temperature = MS5611_GetTemperature();	//��ȡ�¶�ֵ
	//D1_Pres = MS5611_Read(MS561101BA_D1_OSR_4096);		  //��ȡ��ѹ���������ݣ�����Ϊ��������	

	
	OFF= ((uint32_t)PROM_C[1]<<16)+(PROM_C[3] / 128.0 * dT);
	SENS=((uint32_t)PROM_C[0]<<15)+(PROM_C[2] / 256.0 * dT);
	//�����¶Ȳ���
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

	MS5611_Temperature = Temperature;						//����У�����¶�
	DMA_Buff_In_32(MS5611_Temperature,MS5611_TEMP_INDEX);   //������λ�����ͻ���
	OFF = OFF - OFF2;	 	//�����¶Ȳ���
	SENS = SENS - SENS2;
	MS5611_Pressure=(D1_Pres/2097152.0*SENS-OFF)/32768.0;  //����У������ѹ��������λ������
	DMA_Buff_In_32(MS5611_Pressure,MS5611_PRESS_INDEX);
		
    if(Zero_Pressure == 0)
	{	
		Zero_Pressure = MS5611_Pressure;   //���û�вο��߶���ѹ������ǰ��ѹ�����ο��߶���ѹ
//		Zero_T =  (double)(MS5611_Temperature) / 100 + 273.15;
	}
	else
	{
		//�õ�ǰ��ѹ��ο��߶���ѹ�������ǰ��Ը߶ȣ����Ǻ�׼��
	//	Last_Pressure = MS5611_Pressure;
//		MS5611_high += 18400 * (1 + (Last_Pressure + MS5611_Pressure) / 200.0 / 273) * log10( Last_Pressure / MS5611_Pressure);
		MS5611_high = 4433000.0 * (1 - pow((MS5611_Pressure / Zero_Pressure), 0.1903));
	//	MS5611_high = 15384.62 * Zero_T * ( 1 - exp( 0.190259 * log10(MS5611_Pressure / Zero_Pressure)));	  //�߶ȳ�100
		MS5611_high = high_filter(MS5611_high,high_buf);
	    DMA_Buff_In_32((int32_t)MS5611_high,MS5611_HIGH_INDEX);
		//High.high_cur = MS5611_high / 100.0f;
	}
	

}

