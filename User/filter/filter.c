#include "filter.h"

double high_filter(double in_data,double buf[])
{
	double buf_max = buf[8],buf_min = buf[8];
	double buf_sum = 0;
	u8 i;
	for(i = 9;i > 0;i--)
	{
		buf[i] = buf[i - 1];
		buf_sum +=	buf[i];
		if(buf[i] > buf_max)
		{
			buf_max = buf[i];
		}
		if(buf[i] < buf_min)
		{
			buf_min = buf[i];
		}
	}
	buf[0] = in_data;
	buf_sum += in_data;
	return (buf_sum - buf_max - buf_min) / 8;
}

float filter(float in_data,float buf[])
{
	float buf_max = buf[8],buf_min = buf[8];
	float buf_sum = 0;
	u8 i;
	for(i = 9;i > 0;i--)
	{
		buf[i] = buf[i - 1];
		buf_sum +=	buf[i];
		if(buf[i] > buf_max)
		{
			buf_max = buf[i];
		}
		if(buf[i] < buf_min)
		{
			buf_min = buf[i];
		}
	}
	buf[0] = in_data;
	buf_sum += in_data;
	return (buf_sum - buf_max - buf_min) / 8;
}

