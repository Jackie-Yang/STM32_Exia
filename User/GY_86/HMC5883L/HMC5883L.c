#include "HMC5883L.h"
#include "math.h"
#include "I2C.h"
#include "delay.h"
#include "eeprom.h"

/************************************************************   
* 函数名:Read_HMC5883L   
* 描述 : 读取磁场强度 
* 输入  :无   
* 输出  :无    
*/

int16_t Mag_Offset[3] = {0};

void HMC5883L_Init(void)
{
	//	I2C_SendByte(HMC5883L_Addr,HMC5883L_ConfigurationRegisterA,0x14);   //配置寄存器A：采样平均数1 输出速率30Hz 正常测量
	//	I2C_SendByte(HMC5883L_Addr,HMC5883L_ConfigurationRegisterB,0x20);   //配置寄存器B：增益控制
	//	I2C_SendByte(HMC5883L_Addr,HMC5883L_ModeRegister,0x00);   //模式寄存器：连续测量模式

	EE_ReadVariable(HMC5883L_OFFSET_X_ADDR, (uint16_t *)&Mag_Offset[0]);
	EE_ReadVariable(HMC5883L_OFFSET_Y_ADDR, (uint16_t *)&Mag_Offset[1]);
	EE_ReadVariable(HMC5883L_OFFSET_Z_ADDR, (uint16_t *)&Mag_Offset[2]);

	//	DMA_Buff_In_16(HMC5883L_X_offset,27);
	//	DMA_Buff_In_16(HMC5883L_Y_offset,28);
	//	DMA_Buff_In_16(HMC5883L_Z_offset,29);
}

void Read_HMC5883L(int16_t *Mag, __packed float *MagAngle) //读取
{
	uint8_t Buff[6] = {0};
	float Angle;

	I2C_SendByte(HMC5883L_Addr,HMC5883L_ConfigurationRegisterA,0x14);   //配置寄存器A：采样平均数1 输出速率30Hz 正常测量
	I2C_SendByte(HMC5883L_Addr,HMC5883L_ConfigurationRegisterB,0x20);   //配置寄存器B：增益控制
	I2C_SendByte(HMC5883L_Addr,HMC5883L_ModeRegister,0x00);   //模式寄存器：连续测量模式

	I2C_ReadBytes_LE(HMC5883L_Addr, HMC5883L_Output_X_MSB, 6, Buff);
	Mag[0] = ((Buff[0] << 8) | Buff[1]) - Mag_Offset[0];
	Mag[2] = ((Buff[2] << 8) | Buff[3]) - Mag_Offset[2];
	Mag[1] = ((Buff[4] << 8) | Buff[5]) - Mag_Offset[1];

	//	Magn_x=Magn_x-X_Offset;
	//	Magn_y=Magn_y-Y_Offset;
	//	Magn_z=Magn_z-Z_Offset;
	// Magn_x=Magn_x;
	//	Magn_y=Magn_y;
	//Magn_y=(Magn_y*HMC5883L_GAIN_Y)/10000;

//磁场的值很小一般不会溢出
	// if (Mag[0] > 0x7fff)
	// 	Mag[0] -= 0xffff;
	// if (Mag[1] > 0x7fff)
	// 	Mag[1] -= 0xffff;
	// if (Mag[2] > 0x7fff)
	// 	Mag[2] -= 0xffff;
	Angle = (float)atan2f((float)Mag[1], (float)Mag[0]) * (180 / 3.14159265) + 180; // angle in degrees
	Angle += 100;
	if (Angle > 360)
		Angle -= 360;
	*MagAngle = Angle;
}

void HMC5883L_SetOffset(int16_t *Mag, __packed float *MagAngle)
{
	int i;
	int16_t x_max,x_min,y_max,y_min,z_max,z_min;

	Mag_Offset[0] = 0;
	Mag_Offset[1] = 0;
	Mag_Offset[2] = 0;

	//	DMA_Buff_In_16(HMC5883L_X_offset,27);
	//	DMA_Buff_In_16(HMC5883L_Y_offset,28);
	//	DMA_Buff_In_16(HMC5883L_Z_offset,29);

	Read_HMC5883L(Mag, MagAngle);
	x_max = Mag[0];
	x_min = Mag[0];
	y_max = Mag[1];
	y_min = Mag[1];
	z_max = Mag[2];
	z_min = Mag[2];

	for(i = 0;i < 2000;i++)
	{
		Read_HMC5883L(Mag, MagAngle);
		if(Mag[0] > x_max )		
		{
			x_max = Mag[0];
			i = 0;
		}
		else if(Mag[0] < x_min )		
		{
			x_min = Mag[0];
			i = 0;
		}
		if(Mag[1] > y_max ) 
		{
			y_max = Mag[1];
			i = 0;
		}
		else if(Mag[1] < y_min ) 	
		{
			y_min = Mag[1];
			i = 0;
		}
		if(Mag[2] > z_max ) 	
		{
			z_max = Mag[2];
			i = 0;
		}
		else if(Mag[2] < z_min ) 	
		{
			z_min = Mag[2];
			i = 0;
		}
		delay_ms(2);
	}
	Mag_Offset[0] = (x_max + x_min) / 2;
	Mag_Offset[1] = (y_max + y_min) / 2;
	Mag_Offset[2] = (z_max + z_min) / 2;

	EE_WriteVariable(HMC5883L_OFFSET_X_ADDR, Mag_Offset[0]); //将偏移量储存进Flash，方便下次开机读取
	EE_WriteVariable(HMC5883L_OFFSET_Y_ADDR, Mag_Offset[1]);
	EE_WriteVariable(HMC5883L_OFFSET_Z_ADDR, Mag_Offset[2]);

	//准备传送校正数据

	// DMA_Buff_In_16(0,CHECK_STATE_INDEX);
	// DMA_Buff_In_16(HMC5883L_X_offset,CHECK_DATA_INDEX);
	// DMA_Buff_In_16(25,CHECK_STATE_INDEX);

//	DMA_Buff_In_16(HMC5883L_X_offset,27);	
//	DMA_Buff_In_16(HMC5883L_Y_offset,28);	
//	DMA_Buff_In_16(HMC5883L_Z_offset,29);

}



