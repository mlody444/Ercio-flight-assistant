/*
 * Flight assistant.c
 *
 * Created: 2018-05-08 17:01:31
 * Author : Bartek
 */

 #define F_CPU 16000000L

#include <avr/io.h>
#include <math.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "Uart.h"
#include "i2c_twi.h"
#include "twi_buf_pos.h"
#include "MPU6050.h"

#define LED_ON  PORTB |=  (1<<PB5)
#define LED_OFF PORTB &= ~(1<<PB5)
#define LED_TGL PORTB ^=  (1<<PB5)

uint8_t test;

int main(void)
{
//	InitTWI();
	DDRB = (1<<PB5);

		TWSR0 = 0x00;
		TWBR0 = 0x0C;
		//enable TWI
		TWCR0 = (1<<TWEN);

	test = I2C_read_byte(0x68, 0x75);
	test = test & 0b01111110;


	InitUartCOM();
	sei();

	SendLine("Uart dziala");

	SendStringUint("Warto�� = ", test);

	float dummy1, dummy2;

	CalibrateMPU6050(&dummy1, &dummy2);

    while (1) 
    {
		CheckRxBuffor();
    }
}
