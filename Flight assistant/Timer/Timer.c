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
#include "Sbus.h"
#include "PWM_control.h"
#include "Timer3.h"
#include "Timer4.h"

#define POS_FS 50
#define POS_TIMING (1000 / POS_FS)

void test(void);

TIMER_DECREMENT decrement_common[COMMON_TIMERS] = {
	{&test_counter, test},
	{&sbus_buffor_process_counter, Sbus_process_buffor},
};

void InitCommonTimer(void)
{
	TIMSK0 |= (1<<TOIE0);			//Overflow interupt enable
	TCCR0B |= (1<<CS01)| (1<<CS00);	//Prescaler 64
}


void InitPWMTimers(void)
{
	InitTimer3();
	InitTimer4();
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
	test_counter = POS_TIMING;
	int16_t gyro[3], acc[3];
	if (SamplesGyroBuf())
		ReadGyroBufAveraged(gyro);

	if (SamplesAccBuf())
		ReadAccBufAveraged(acc);

	ProcessPosition(gyro, acc);

	SendStringInt("B X ", pos_x);
	SendStringInt("B Y ", pos_y);
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

	if (sbus_buffor_start_counter != 0xFF) sbus_buffor_start_counter++;
	if (sbus_frame_delay != 0xFF) sbus_frame_delay++;
	if (sbus_last_frame_counter!= 0xFF) sbus_last_frame_counter++;
}