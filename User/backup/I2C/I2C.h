#ifndef I2C_H
#define I2C_H

#include "stm32f10x.h"
#include "USART.h"



void I2C_init(void);
void I2C_write(void);
u8 I2C_read(void);

#endif
