/*
* Timer.c
*
* Created: 2018-11-12 12:29:39
*  Author: Bartek
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Timer.h"
#include "Common.h"

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
	test_counter = 20;
	LED_TGL;
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