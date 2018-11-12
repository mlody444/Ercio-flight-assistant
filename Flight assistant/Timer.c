/*
* Timer.c
*
* Created: 2018-11-12 12:29:39
*  Author: Bartek
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>

#include "Timer.h"
#include "Common.h"
#include "Uart.h"
#include "gyro_math.h"

void test(void);

TIMER_DECREMENT decrement_common[COMMON_TIMERS] = {
	{&test_counter, test},
};

void InitCommonTimer(void)
{
	TIMSK0 |= (1<<TOIE0);			//Overflow interupt enable
	TCCR0B |= (1<<CS01)| (1<<CS00);	//Prescaler 64
}

void CheckCommonTimer(void)
{
	uint8_t i;
	for (i=0; i<COMMON_TIMERS;i++)
	{
		if (*decrement_common[i].counter == 0)
		{
			decrement_common[i].timer_service();
		}
	}
}

void test(void)
{
	test_counter = 25;
	int16_t gyro[3], acc[3];
	if (SamplesGyroBuf())
		ReadGyroBufAveraged(gyro);

	if (SamplesAccBuf())
		ReadAccBufAveraged(acc);
	
	double x_pos_acc = ProcessAngle(acc[Y_AXIS], acc[Z_AXIS]);
	double y_pos_acc = (ProcessAngle(acc[X_AXIS], acc[Z_AXIS]) * (-1));

	char text[8];

	dtostrf(x_pos_acc, 0, 3, text);
	SendString("P ");
	SendString(text);
	
	dtostrf(y_pos_acc, 0, 3, text);
	SendString(" ");
	SendLine(text);
}

ISR (TIMER0_OVF_vect)	//each 1,024ms
{
	uint8_t i;
	for (i=0; i<COMMON_TIMERS;i++)
	{
		if (*decrement_common[i].counter != 0 && *decrement_common[i].counter != 0xFFFF)
		{
			*decrement_common[i].counter = *decrement_common[i].counter - 1;
		}
	}
}