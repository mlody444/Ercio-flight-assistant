/*
 * Timer4.c
 *
 * Created: 2018-02-21 18:26:44
 *  Author: Bartek
 */ 

#include <avr/interrupt.h>

#include "Common.h"



uint8_t *PORT;
uint8_t PIN;

void InitTimer4(void)
{
	TCCR4B |= (1<<WGM42);	//Mode 4: CTC top on OCR4A

	//Overflow interupt
	TIFR4  |= (1<<OCF4A);	//Interrupt compare A
	TIMSK4 |= (1<<OCIE4A);	//Enable interrupt on A
}

void Register_PWM_Pin (uint16_t time_value, uint8_t new_pin)
{
//Register pin for PWM :
	if (time_value != 0)	//check if timer value is not 0
	{
		PIN = new_pin;

		TCNT4 = 0;			//clear timer
		OCR4A = time_value;	//set new top value

		PORTC |= (1<<PIN);	//Set pin high

		TCCR4B |= (1<<CS40);	//prescaller 1
	}
}

ISR (TIMER4_COMPA_vect)
{
	PORTC &= ~(1<<PIN);	//Set pin Low
	//TCCR4B &= ~((1<<CS40) | (1<<CS41) | (1<<CS42));	//turn of timer
	TCCR4B &= ~(1<<CS40);	//turn off timer
}