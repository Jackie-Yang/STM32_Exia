#include "KS10X.h"
#include "I2C.h"
#include "delay.h"
#include "Setup.h"

/*************进入配置模式**************/
//#define KS109_DEBUG

/**************进行不同配置(一次只进行一个)*********************/
//#define KS10X_SCL		   //测量过程SCL不被拉低
//#define KS109_INIT	   //配置KS109

u16 KS10X_high = 0;



void KS10X_init(void)	  
{
	delay_ms(1000);		   //模块初始化
	delay_ms(1000);


/********************以下配置只需要执行一次，执行完需要重启(需要配置时宏定义)*********/

#if (defined KS10X_SCL) && (defined KS109_DEBUG)		//配置为测量过程SCL不被拉低，不配置的话会影响其他模块使用	
			
	KS10X_command(0xc3);	
	delay_us(100);

#elif (defined KS109_INIT) && (defined KS109_DEBUG)		//配置KS109波束角10°，带温度补偿

	KS10X_command(0x9c);	   //三个时序指令
	delay_ms(2);
	KS10X_command(0x95);
	delay_ms(2);
	KS10X_command(0x98);
	delay_ms(2);
	KS10X_command(0x7d);	  //配置模式


//#elif (defined KS103_INIT) && (defined KS109_DEBUG)	   //配置KS103，没什么配...

#endif



#ifdef KS109_DEBUG			    //配置后需要2秒让传感器配置，之后需要手动重启模块

	delay_ms(1000);
	delay_ms(1000);
#else						   //若不是配置模式，发送第一次获取高度指令

	KS10X_command(0xbc);

#endif
}



void KS10X_Get_High(void)
{
#ifndef KS109_DEBUG

	u16 result;
	delay_us(80);
	result = KS10X_Get_Result( );		//获取结果
	if(result < 0x2c10)					//防止超过11.280m(最大值)
	{
		KS10X_high = result;
		stQuadrotor_State.u16_KS10X_High = KS10X_high;		//返回的值为毫米
	}	
	delay_us(80);
	KS10X_command(0xbc);	   //发送指令，让传感器探测

#endif
}

uint8_t KS10X_command(u8 command) //发送指令
{
	uint8_t ret = 0;
	uint32_t OldHoldTime = I2C_GetHoldTime();
	I2C_SetHoldTime(5);
	ret = I2C_SendByte(KS10X_ADDRESS, 0x02, command);
	I2C_SetHoldTime(OldHoldTime);
	return ret;
}

u16 KS10X_Get_Result(void)			   //获取结果(时序2)
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


