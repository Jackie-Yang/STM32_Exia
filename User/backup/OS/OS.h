#ifndef OS_H
#define OS_H

#include "string.h"			//字符串处理，处理指令
#include "Setup.h"			//系统时钟、IO、中断初始化函数
#include "USART.h"			
#include "LED.h"		   
#include "timer.h"
#include "button.h"
#include "ADC.h"
#include "DMA.h"
#include "AT24C02.h"


#define ORDER_SIZE 257            //输入指令最大长度，实际命令要比次长度小一字节，因为要包括字符串结束符号




void OS_init(void);
void runOrder(void);
u8 getOrder(void);
u8 getNum(void);
u8 getNum_HEX(void);
u8* getData(void);

#endif

