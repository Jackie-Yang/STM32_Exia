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
		stQuadrotor_State.u16_KS10X_High = KS10X_high;		//���ص�ֵΪ����
	}	
	delay_us(80);
	KS10X_command(0xbc);	   //����ָ��ô�����̽��

#endif
}

uint8_t KS10X_command(u8 command) //����ָ��
{
	uint8_t ret = 0;
	uint32_t OldHoldTime = I2C_GetHoldTime();
	I2C_SetHoldTime(5);
	ret = I2C_SendByte(KS10X_ADDRESS, 0x02, command);
	I2C_SetHoldTime(OldHoldTime);
	return ret;
}

u16 KS10X_Get_Result(void)			   //��ȡ���(ʱ��2)
{
	u16 data;

	uint32_t OldHoldTime = I2C_GetHoldTime();
	I2C_SetHoldTime(5);

	I2C_Start();
	I2C_Send_Data(KS10X_ADDRESS+1);

	delay_us(62);

	data = I2C_Read_Data(NACK) ;
	data <<= 8;

	I2C_Start();
	I2C_Send_Data(KS10X_ADDRESS+1);

	delay_us(62);

	data |= I2C_Read_Data(NACK) ;

	I2C_Stop();

	I2C_SetHoldTime(OldHoldTime);
	return data;
}


