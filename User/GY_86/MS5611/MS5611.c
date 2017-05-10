#include "MS5611.h"
#include "math.h"
#include "Setup.h"
#include "I2C.h"
#include "delay.h"
// #include "filter.h"
#include "PID.h"

//����
u16 PROM_C[6];  //���ڴ��PROM�е�6������
u32 D2_Temp = 0,D1_Pres = 0;	//��ȡ���¶ȴ��������� ,��ȡ����ѹ����; 
// #define float double
float dT;  //��ȡ��ֵ��ο��¶�֮��Ĳ���
//�ο��߶ȵ���ѹֵ
float Zero_Pressure;
// float MS5611_Temperature = 0; //������ɺ���¶�ֵ
// float Last_Temperature = 0;
// float MS5611_Pressure = 0; //�¶Ȳ�������ѹ���
// float Last_Pressure = 0;
// float Zero_T = 0;
// float MS5611_high = 0;
// float high_buf[10] = {0};
int8_t MS5611_Reset(void);	//оƬ����
int8_t MS5611_readPROM(void); //��ȡоƬ��PROM����

/************************************************************   
* ������:MS561101BA_Init   
* ���� : MS561101BA��ʼ��
* ����  :��   
* ���  :��    
*/ 
int8_t MS5611_Init(void)
{
	int8_t ret = 0;
	ret |= MS5611_Reset();
	delay_ms(10);
	ret |= MS5611_readPROM();
	delay_ms(10);
	return ret;
	// u8 i;
	// for(i = 0;i < 20; i++)
	// {
	// 	MS5611_AskData(MS561101BA_D2_OSR_4096);	  //����ָ����MS5611�����¶�ֵ
	// 	delay_ms(10);
	// 	D2_Temp = MS5611_GetData( );			 	 //10ms���ȡ����
	// 	delay_ms(10);
	// 	MS5611_AskData(MS561101BA_D1_OSR_4096);	  //����ָ����MS5611������ѹֵ
	//     delay_ms(10);
	// 	D1_Pres = MS5611_GetData( );
	// 	MS5611_GetPressure( );					//10ms���ȡ����,�����¶ȡ���ѹֵ
	// 	delay_ms(10);
	// }

	// Zero_Pressure = 0;			
					
} 

/************************************************************   
* ������:MS561101BA_Reset   
* ���� : ��λ  
* ����  :��   
* ���  :��    
*/ 
int8_t MS5611_Reset(void)
{
	return I2C_SendByte_NoAddr(MS561101BA_ADDR,MS561101BA_RESET);
}

/************************************************************   
* ������:MS561101BA_readPROM   
* ���� : ��PROM��ȡ����У׼����
* ����  :��   
* ���  :��    
  */
int8_t MS5611_readPROM(void)
{
	u8 i;
	int8_t ret = 0;
	for (i = 1; i <= MS561101BA_PROM_REG_COUNT; i++) //�����ȡPROM
	{
		ret |= I2C_ReadBytes_BE(MS561101BA_ADDR, MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE), MS561101BA_PROM_REG_SIZE, (uint8_t *)(PROM_C + i - 1));
	}
	return ret;
}

/************************************************************   
* ������:MS561101BA_DO_CONVERSION   
* ���� : ��MS5611����ָ���ȡ����(���ڷ���ָ�����Ҫ�ȴ�8ms���ܶ�ȡ���ݣ���˷ֿ����������ö�ʱ������) 
* ����  :��   
* ���  :��    
*/
int8_t MS5611_AskData(uint8_t command)
{
	return I2C_SendByte_NoAddr(MS561101BA_ADDR, command);
}

int8_t MS5611_GetData(uint32_t *pData)
{
	//���ݼĴ�����24λ�ģ�3���ֽ�
	return I2C_ReadBytes_BE(MS561101BA_ADDR, 0, 3, (uint8_t *)pData);
}


int8_t MS5611_AskTemperature(void)
{
	return MS5611_AskData(MS561101BA_D2_OSR_4096);
}

int8_t MS5611_GetTemperature(void)
{
	return MS5611_GetData(&D2_Temp);
}

int8_t MS5611_AskPressure(void)
{
	return MS5611_AskData(MS561101BA_D1_OSR_4096);
}

int8_t MS5611_GetPressure(void)
{
	return MS5611_GetData(&D1_Pres);
}

//���´β����ĸ߶���Ϊ�ݿ��߶�
void MS5611_SetReference(void)
{
	Zero_Pressure = 0;
}

/************************************************************   
* ������:MS561101BA_CalPressure   
* ���� : ����������ѹ
* ����  :��  
* ���  :��    
*/
void MS5611_CalResult(__packed float *Press, __packed float *Temp, __packed float *High)
{
	float OFF,SENS;  //ʵ���¶ȵ���,ʵ���¶�������
	float Aux,OFF2,SENS2;  //�¶�У��ֵ
	float Temperature,Temperature2 = 0;
	float Pressure = 0;

	dT = D2_Temp - (((u32)PROM_C[4]) << 8); //ʵ�ʺͲο��¶�֮��Ĳ���
	Temperature = (2000 + dT / 8388608.0 * ((uint32_t)PROM_C[5])); //��ȡ�¶�ֵ

	// Temperature = MS5611_CalTemperature();						   //��ȡ�¶�ֵ
	// D1_Pres = MS5611_AskData(MS561101BA_D1_OSR_4096);		  //��ȡ��ѹ���������ݣ�����Ϊ��������

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
	//����У�����¶�,���������������ʵ���¶�Ҫ����100
	*Temp = Temperature;
	// MS5611_Temperature = Temperature;							
	// stQuadrotor_State.s32_MS5611_Temp = MS5611_Temperature;		//������λ�����ͻ���


	OFF = OFF - OFF2;	 	//�����¶Ȳ���
	SENS = SENS - SENS2;
	//����У������ѹ�����������������ʵ����ѹҪ����100(mbar),���߳���1000(kPa)
	Pressure = (D1_Pres / 2097152.0 * SENS - OFF) / 32768.0;
	*Press = Pressure;
	// MS5611_Pressure = (D1_Pres / 2097152.0 * SENS - OFF) / 32768.0;
	// stQuadrotor_State.s32_MS5611_Press = MS5611_Pressure;

	//���û�вο��߶���ѹ������ǰ��ѹ�����ο��߶���ѹ
	if(Zero_Pressure == 0)
	{
		Zero_Pressure = Pressure;
		*High = 0;
		// Zero_Pressure = MS5611_Pressure;   
//		Zero_T =  (float)(MS5611_Temperature) / 100 + 273.15;
	}
	else
	{
		//�õ�ǰ��ѹ��ο��߶���ѹ�������ǰ��Ը߶ȣ����Ǻ�׼��
		//	Last_Pressure = MS5611_Pressure;
		//		MS5611_high += 18400 * (1 + (Last_Pressure + MS5611_Pressure) / 200.0 / 273) * log10( Last_Pressure / MS5611_Pressure);
		*High = 4433000.0 * (1 - pow((Pressure / Zero_Pressure), 0.1903));
		//	MS5611_high = 15384.62 * Zero_T * ( 1 - exp( 0.190259 * log10(MS5611_Pressure / Zero_Pressure)));	  //�߶ȳ�100
		// MS5611_high = high_filter(MS5611_high,high_buf);
		// stQuadrotor_State.s32_MS5611_HIGH = (int32_t)MS5611_high;
		//High.high_cur = MS5611_high / 100.0f;
	}

}

// /************************************************************   
// * ������:MS561101BA_CalTemperature   
// * ���� : ��ȡ�����¶�
// * ����  :��  
// * ���  :��    
// */
// float MS5611_CalTemperature(void)
// {

// 	//D2_Temp = MS5611_AskData(MS561101BA_D2_OSR_4096);  //��ȡ�¶ȴ��������ݣ�����Ϊ��������

// 	dT = D2_Temp - (((u32)PROM_C[4]) << 8); //ʵ�ʺͲο��¶�֮��Ĳ���

// 	return (2000 + dT / 8388608.0 * ((uint32_t)PROM_C[5])); //����¶�ֵ��100����2001��ʾ20.01��
// }
