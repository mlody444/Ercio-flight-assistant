/*
 * gyro_math.c
 *
 * Created: 2018-11-12 19:19:25
 *  Author: Bartek
 */ 

#include <avr/io.h>
#include <math.h>
#include <stdlib.h>

#include "Uart.h"
#include "Common.h"


double ProcessAngle(int16_t width, int16_t high)
{
	double angle = 0;
	if (width != 0)	//for 0 width atan can be 0 or 180, but should be 0
	{
		angle = atan((double)high / (double)width);
		/* Process to correct angles */
		if (width > 0)
			angle = M_PI_2 - angle;
		else
			angle = (M_PI_2 + angle) * (-1);

		angle = (angle * 180) / M_PI;
	}
	return angle;
}