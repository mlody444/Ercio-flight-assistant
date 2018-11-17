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
	pos_x = 0;
	pos_y = 0;
//	InitTWI();
	DDRB = (1<<PB5);
	TWSR0 = 0x00;
	TWBR0 = 0x0C;
	TWCR0 = (1<<TWEN);	//enable TWI

// 	test = I2C_read_byte(0x68, 0x75);
// 	test = test & 0b01111110;

	InitUartCOM();
	InitUartSBUS();
	InitPWMTimers();
	sei();
	SendLine("Uart dziala");

	test_counter = 200;

	CalibrateMPU6050(gyro_offset, acc_offset);

	for (uint8_t i=0; i<10; i++){
		LED_TGL;
		_delay_ms(100);
	}

	InitMPU6050();
	InitCommonTimer();

    while (1) 
    {
		CheckRxBuffor();
		ReadMPU6050();
		CheckCommonTimer();
    }
}