#ifndef FILTER_H
#define FILTER_H
#include "stm32f10x.h"

double high_filter(double in_data,double buf[]);
float filter(float in_data,float buf[]);


#endif
