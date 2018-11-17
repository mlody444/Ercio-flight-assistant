/*
 * Timer3.c
 *
 * Created: 2018-02-21 18:26:21
 *  Author: Bartek
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Common.h"
#include "Timer4.h"
#include "Timer3.h"
#include "Uart.h"
#include "PWM_control.h"

#define FIRST_INTERRUPT	5000
#define NEXT_PWM_DELAY	5000

uint8_t selected_PWM_channel;

PWM_CHANNEL pwm_channels[PWM_CHANNELS];


void InitTimer3(void)
{
	//TC3 General timer
	TCCR3B |= (1<<WGM32);	//Mode 4: CTC top on OCR3A
	OCR3A   = 40000;		//Timer each 20ms = 50Hz

	//Overflow interupt
	TIFR3  |= (1<<OCF3A);	//Interrupt compare A
	TIMSK3 |= (1<<OCIE3A);	//Enable interrupt on A

	//Compare interrupt
	TIFR3  |= (1<<OCF3B);	//Interrupt compare B
	TIMSK3 |= (1<<OCIE3B);	//Enable interrupt on B

	TCCR3B |= (1<<CS31);	//Prescaler: 8
}

ISR (TIMER3_COMPA_vect)		//Interrupt on top value
{
 	//set flag to process
	events.flag.process_PWMs = 1;
	//processing new values will be done in main loop as event (to keep interrupt short)

	OCR3B = FIRST_INTERRUPT;	//first interrupt after processing values
	selected_PWM_channel = 0;
}

ISR (TIMER3_COMPB_vect)		//PWM init interrupt
{
	if (pwm_channels[selected_PWM_channel].counter > PWM_MAX)
	{
		pwm_channels[selected_PWM_channel].counter = PWM_MAX;
	}
	else if (pwm_channels[selected_PWM_channel].counter < PWM_MIN)
	{
		pwm_channels[selected_PWM_channel].counter = PWM_MIN;
	}
	Register_PWM_Pin(pwm_channels[selected_PWM_channel].counter, selected_PWM_channel);
	selected_PWM_channel++;
	if (selected_PWM_channel < PWM_CHANNELS){
		OCR3B = FIRST_INTERRUPT + NEXT_PWM_DELAY*(selected_PWM_channel);
	}
}