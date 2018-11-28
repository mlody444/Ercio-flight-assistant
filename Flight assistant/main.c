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
#include <stdlib.h>

#include "Uart.h"
#include "i2c_twi.h"
#include "twi_buf_pos.h"
#include "MPU6050.h"
#include "MPU6050_registers.h"
#include "Common.h"
#include "Timer.h"
#include "Sbus.h"

#define LED_ON  PORTB |=  (1<<PB5)
#define LED_OFF PORTB &= ~(1<<PB5)
#define LED_TGL PORTB ^=  (1<<PB5)

int main(void)
{
	InitVariables();

//	InitTWI();
	DDRB  = (1<<PB5);
	DDRB |= (1<<PB1);		//PB1 - buzer
	DDRC |= (1<<PC0) | (1<<PC1)| (1<<PC2);
	TWSR0 = 0x00;
	TWBR0 = 0x0C;
	TWCR0 = (1<<TWEN);	//enable TWI

	LED_OFF;
	BUZ_OFF;
	for (uint8_t i=0; i<11; i++){
		LED_TGL;
		BUZ_TGL;
		_delay_ms(200);
	}

	InitUartCOM();
	sei();
	SendLine("Uart dziala");

	CalibrateMPU6050(gyro_offset, acc_offset);

	InitMPU6050();
	InitCommonTimer();
	InitUartSBUS();
	InitPWMTimers();

	LED_OFF;
	BUZ_OFF;

    while (1) 
    {
		CheckRxBuffor();
		ReadMPU6050();
		CheckCommonTimer();
		Events();
    }
}