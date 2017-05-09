#include "stm32f10x.h"
#include "Setup.h"
#include "delay.h"




int main(void)
{
	int8_t InitRet = 0;
	InitRet = !init();

	while (InitRet)
	{

	}
	delay(10);
	NVIC_SystemReset();
	return 0;
}
