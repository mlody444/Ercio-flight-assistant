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
#include "MPU6050_registers.h"

#define LED_ON  PORTB |=  (1<<PB5)
#define LED_OFF PORTB &= ~(1<<PB5)
#define LED_TGL PORTB ^=  (1<<PB5)

uint8_t test;

void InitMPU6050(void);

void InitMPU6050(void)
{
	// reset device, reset all registers, clear gyro and accelerometer bias registers
	I2C_write_byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	_delay_ms(100);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	I2C_write_byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
	I2C_write_byte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
	_delay_ms(200);

	// Configure device for bias calculation
	I2C_write_byte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	I2C_write_byte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	I2C_write_byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	I2C_write_byte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	I2C_write_byte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	I2C_write_byte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	_delay_ms(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	I2C_write_byte(MPU6050_ADDRESS, CONFIG, 0x03);      // Set low-pass filter to 44 Hz
	I2C_write_byte(MPU6050_ADDRESS, SMPLRT_DIV, 20);  // Set sample rate to 50 Hz
	I2C_write_byte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	I2C_write_byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	I2C_write_byte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	I2C_write_byte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
}

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

	SendStringUint("Wartoœæ = ", test);

	int16_t gyro_offset[3];
	int16_t acc_offset[3];

	CalibrateMPU6050(gyro_offset, acc_offset);
//Send callibration data
//	SendStringInt("Gyro X = ", gyro_offset[0]);
//	SendStringInt("Gyro Y = ", gyro_offset[1]);
//	SendStringInt("Gyro Z = ", gyro_offset[2]);

//	SendStringInt("Acc  X = ", acc_offset[0]);
//	SendStringInt("Acc  Y = ", acc_offset[1]);
//	SendStringInt("Acc  Z = ", acc_offset[2]);


	InitMPU6050();

	int16_t gyro[3];
	int16_t acc[3];
	uint8_t data[12];
	uint8_t fifo_count, packet_count;

	uint8_t i;
    while (1) 
    {
		CheckRxBuffor();
		I2C_read_buf(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
		I2C_read_buf(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
		fifo_count = ((uint16_t)data[0] << 8) | data[1];
		packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
		for (i = 0; i < packet_count; i++)
		{
			I2C_read_buf(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
			acc[0] = (int16_t) (((int16_t)data[0] << 8)  | data[1] ) ;  // Form signed 16-bit integer for each sample in FIFO
			acc[1] = (int16_t) (((int16_t)data[2] << 8)  | data[3] ) ;
			acc[2] = (int16_t) (((int16_t)data[4] << 8)  | data[5] ) ;
			gyro[0]  = (int16_t) (((int16_t)data[6] << 8)  | data[7] ) ;
			gyro[1]  = (int16_t) (((int16_t)data[8] << 8)  | data[9] ) ;
			gyro[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
			
			acc[0] -= acc_offset[0];
			acc[1] -= acc_offset[1];
			acc[2] -= acc_offset[2];

			gyro[0] -= gyro_offset[0];
			gyro[1] -= gyro_offset[1];
			gyro[2] -= gyro_offset[2];


			SendStringInt("Gyro X ", gyro[0]);
			SendStringInt("Gyro Y ", gyro[1]);
			SendStringInt("Gyro Z ", gyro[2]);

			SendStringInt("Acc X ", acc[0]);
			SendStringInt("Acc Y ", acc[1]);
			SendStringInt("Acc Z ", acc[2]);
		}
    }
}

