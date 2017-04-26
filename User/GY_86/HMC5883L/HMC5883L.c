#include "HMC5883L.h"
#include "math.h"
#include "Setup.h"
#include "I2C.h"
#include "delay.h"
#include "eeprom.h"

/************************************************************   
* 函数名:Read_HMC5883L   
* 描述 : 读取磁场强度 
* 输入  :无   
* 输出  :无    
*/

int16_t HMC5883L_X = 0,HMC5883L_Y = 0,HMC5883L_Z = 0;
int16_t HMC5883L_X_offset = 0,HMC5883L_Y_offset = 0,HMC5883L_Z_offset = 0;
float HMC5883L_angle = 0;

void HMC5883L_Init(void)
{
//	I2C_SendByte(HMC5883L_Addr,HMC5883L_ConfigurationRegisterA,0x14);   //配置寄存器A：采样平均数1 输出速率30Hz 正常测量
//	I2C_SendByte(HMC5883L_Addr,HMC5883L_ConfigurationRegisterB,0x20);   //配置寄存器B：增益控制
//	I2C_SendByte(HMC5883L_Addr,HMC5883L_ModeRegister,0x00);   //模式寄存器：连续测量模式

	EE_ReadVariable(HMC5883L_OFFSET_X_ADDR, (uint16_t*)&HMC5883L_X_offset);
	EE_ReadVariable(HMC5883L_OFFSET_Y_ADDR, (uint16_t*)&HMC5883L_Y_offset);
	EE_ReadVariable(HMC5883L_OFFSET_Z_ADDR, (uint16_t*)&HMC5883L_Z_offset);

//	DMA_Buff_In_16(HMC5883L_X_offset,27);	
//	DMA_Buff_In_16(HMC5883L_Y_offset,28);	
//	DMA_Buff_In_16(HMC5883L_Z_offset,29);
	
}


void Read_HMC5883L(void)//读取
{
	I2C_SendByte(HMC5883L_Addr,HMC5883L_ConfigurationRegisterA,0x14);   //配置寄存器A：采样平均数1 输出速率30Hz 正常测量
	I2C_SendByte(HMC5883L_Addr,HMC5883L_ConfigurationRegisterB,0x20);   //配置寄存器B：增益控制
	I2C_SendByte(HMC5883L_Addr,HMC5883L_ModeRegister,0x00);   //模式寄存器：连续测量模式
	

	HMC5883L_X=I2C_Read_16(HMC5883L_Addr,HMC5883L_Output_X_MSB) - HMC5883L_X_offset;

	HMC5883L_Y=I2C_Read_16(HMC5883L_Addr,HMC5883L_Output_Y_MSB) - HMC5883L_Y_offset;//OUT_Y_L_A
	
  	HMC5883L_Z=I2C_Read_16(HMC5883L_Addr,HMC5883L_Output_Z_MSB) - HMC5883L_Z_offset;//OUT_Z_L_A

//	Magn_x=Magn_x-X_Offset;
//	Magn_y=Magn_y-Y_Offset;
//	Magn_z=Magn_z-Z_Offset;
 // Magn_x=Magn_x;
//	Magn_y=Magn_y;
	//Magn_y=(Magn_y*HMC5883L_GAIN_Y)/10000;

	stQuadrotor_State.HMC5883L_X = HMC5883L_X;
	stQuadrotor_State.HMC5883L_Y = HMC5883L_Y;
	stQuadrotor_State.HMC5883L_Z = HMC5883L_Z;

	if (HMC5883L_X > 0x7fff)
		HMC5883L_X -= 0xffff;
	if (HMC5883L_Y > 0x7fff)
		HMC5883L_Y -= 0xffff;
	if (HMC5883L_Z > 0x7fff)
		HMC5883L_Z -= 0xffff;
	HMC5883L_angle = (float)atan2f((float)HMC5883L_Y, (float)HMC5883L_X) * (180 / 3.14159265) + 180; // angle in degrees
	HMC5883L_angle=HMC5883L_angle+100;
	if(HMC5883L_angle>360)
		HMC5883L_angle=HMC5883L_angle-360;

	stQuadrotor_State.HMC5883L_Angle = HMC5883L_angle;
	stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
}

void HMC5883L_SetOffset(void)
{
	int i;
	int16_t x_max,x_min,y_max,y_min,z_max,z_min;

	HMC5883L_X_offset = 0;
	HMC5883L_Y_offset = 0;
	HMC5883L_Z_offset = 0;

//	DMA_Buff_In_16(HMC5883L_X_offset,27);	
//	DMA_Buff_In_16(HMC5883L_Y_offset,28);	
//	DMA_Buff_In_16(HMC5883L_Z_offset,29);

	Read_HMC5883L();
	x_max = HMC5883L_X;
	x_min = HMC5883L_X;
	y_max = HMC5883L_Y;
	y_min = HMC5883L_Y;
	z_max = HMC5883L_Z;
	z_min = HMC5883L_Z;

	for(i = 0;i < 2000;i++)
	{
		Read_HMC5883L();
		if(HMC5883L_X > x_max )		
		{
			x_max = HMC5883L_X;
			i = 0;
		}
		if(HMC5883L_X < x_min )		
		{
			x_min = HMC5883L_X;
			i = 0;
		}
		if(HMC5883L_Y > y_max ) 
		{
			y_max = HMC5883L_Y;
			i = 0;
		}
		if(HMC5883L_Y < y_min ) 	
		{
			y_min = HMC5883L_Y;
			i = 0;
		}
		if(HMC5883L_Z > z_max ) 	
		{
			z_max = HMC5883L_Z;
			i = 0;
		}
		if(HMC5883L_Z < z_min ) 	
		{
			z_min = HMC5883L_Z;
			i = 0;
		}
		delay_ms(2);
	}
	HMC5883L_X_offset = (x_max + x_min) / 2;
	HMC5883L_Y_offset = (y_max + y_min) / 2;
	HMC5883L_Z_offset = (z_max + z_min) / 2;

	EE_WriteVariable(HMC5883L_OFFSET_X_ADDR, HMC5883L_X_offset);	   //将偏移量储存进Flash，方便下次开机读取
	EE_WriteVariable(HMC5883L_OFFSET_Y_ADDR, HMC5883L_Y_offset);
	EE_WriteVariable(HMC5883L_OFFSET_Z_ADDR, HMC5883L_Z_offset);

	  //准备传送校正数据

	// DMA_Buff_In_16(0,CHECK_STATE_INDEX);
	// DMA_Buff_In_16(HMC5883L_X_offset,CHECK_DATA_INDEX);
	// DMA_Buff_In_16(25,CHECK_STATE_INDEX);

//	DMA_Buff_In_16(HMC5883L_X_offset,27);	
//	DMA_Buff_In_16(HMC5883L_Y_offset,28);	
//	DMA_Buff_In_16(HMC5883L_Z_offset,29);

}



