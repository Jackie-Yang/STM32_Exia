#ifndef MOTOR_H
#define MOTOR_H
#include "stm32f10x.h"

void Motor_Init(void);                     //定时器2初始化，于输出PWM电机信号
void set_motorPWM(u8 motor, u16 motorPWM); //设置定时器2输出的PWM


#endif
