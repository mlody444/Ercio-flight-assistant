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

#define POS_FS 50
#define POS_TIMING (1000 / POS_FS)

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
	test_counter = POS_TIMING;
	int16_t gyro[3], acc[3];
	if (SamplesGyroBuf())
		ReadGyroBufAveraged(gyro);

	if (SamplesAccBuf())
		ReadAccBufAveraged(acc);

	ProcessPosition(gyro, acc);

// 	int32_t pomocnicza;
// 	double x_pos_acc = ProcessAngle(acc[Y_AXIS], acc[Z_AXIS]);
// 	double y_pos_acc = (ProcessAngle(acc[X_AXIS], acc[Z_AXIS]) * (-1));
// 	int32_t pos_acc;
//
// 	x_pos_acc *= 1;
// 	y_pos_acc *= 1;
// 
// 	int32_t x_pos_gyro = (int32_t)gyro[X_AXIS] * 100;
// 	int32_t y_pos_gyro = (int32_t)gyro[Y_AXIS] * 100;
// 
// 	x_pos_gyro = x_pos_gyro / (50 * MPU6050_LSB);
// 	y_pos_gyro = y_pos_gyro / (50 * MPU6050_LSB);
// 
// 	pos_acc = (int32_t)x_pos_acc;
// 	pomocnicza = pos_x + x_pos_gyro;
// 	pomocnicza *= 99;
// 	pomocnicza /= 100;
// 	pos_x = pos_acc + pomocnicza;
// 	
// 	pos_acc = (int32_t)y_pos_acc;
// 	pomocnicza = pos_y + y_pos_gyro;
// 	pomocnicza *= 99;
// 	pomocnicza /= 100;
// 	pos_y = pos_acc + pomocnicza;


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
}