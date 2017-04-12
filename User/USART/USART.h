#ifndef USART_H
#define USART_H
#include "stm32f10x.h"





void USART_Configuration(void);			//串口配置
void USART1_sendData_u8(u8 data);		//串口发送8bit数据
void USART1_sendData_u16(u16 data);	    //串口发送16bit数据
void USART1_sendData_u32(u32 data);	    //串口发送32bit数据
void USART1_sendStr(u8 *data);			//串口发送字符串

void check_BT(void);					//检查蓝牙串口模块是否工作

extern u8 BT_state;



#endif
