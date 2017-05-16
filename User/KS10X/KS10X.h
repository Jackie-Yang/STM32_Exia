#ifndef KS10X_H
#define KS10X_H
#include "stm32f10x.h"

#define KS10X_ADDRESS 0xe8

extern u16 KS10X_high;

void KS10X_init(void);
uint8_t KS10X_command(u8 command);
u16 KS10X_Get_Result(void);

void KS10X_Get_High(void);






#endif

