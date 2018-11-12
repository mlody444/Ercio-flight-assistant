/*
 * MPU6050.c
 *
 * Created: 2018-05-15 16:33:40
 *  Author: Bartek
 */ 

#define F_CPU 16000000L

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "MPU6050.h"
#include "MPU6050_registers.h"
#include "Uart.h"
#include "Common.h"
#include "i2c_twi.h"

void CalibrateMPU6050(int16_t gyro_offset[], int16_t acc_offset[])
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, jj, packet_count, packet_total, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0};
	int32_t accel_bias[3] = {0, 0, 0};
	packet_total = 0;

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
	I2C_write_byte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	I2C_write_byte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	I2C_write_byte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	I2C_write_byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	I2C_write_byte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	I2C_write_byte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
	_delay_ms(5);

	for (jj = 0; jj < 20; jj++)	//read samples for 1 second
	{
		_delay_ms(32); // accumulate 50 samples in 50 milliseconds = 600 bytes
		// At end of sample accumulation, turn off FIFO sensor read
		if (jj == 19)	// Disable gyro and accelerometer sensors for FIFO
			I2C_write_byte(MPU6050_ADDRESS, FIFO_EN, 0x00);
		I2C_read_buf(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
		fifo_count = ((uint16_t)data[0] << 8) | data[1];

		packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

		for (ii = 0; ii < packet_count; ii++) 
		{
			int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
			I2C_read_buf(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
			if (packet_total < 1024)
			{
				packet_total++;
				accel_temp[0] = (int16_t) (((int16_t)data[0] << 8)  | data[1] ) ;  // Form signed 16-bit integer for each sample in FIFO
				accel_temp[1] = (int16_t) (((int16_t)data[2] << 8)  | data[3] ) ;
				accel_temp[2] = (int16_t) (((int16_t)data[4] << 8)  | data[5] ) ;
				gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8)  | data[7] ) ;
				gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8)  | data[9] ) ;
				gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
		
				accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
				accel_bias[1] += (int32_t) accel_temp[1];
				accel_bias[2] += (int32_t) accel_temp[2];
				gyro_bias[0]  += (int32_t) gyro_temp[0];
				gyro_bias[1]  += (int32_t) gyro_temp[1];
				gyro_bias[2]  += (int32_t) gyro_temp[2];
			}
		}
	}

	accel_bias[0] /= (int32_t) packet_total; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_total;
	accel_bias[2] /= (int32_t) packet_total;
	gyro_bias[0]  /= (int32_t) packet_total;
	gyro_bias[1]  /= (int32_t) packet_total;
	gyro_bias[2]  /= (int32_t) packet_total;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}
	
	gyro_offset[0] = (int16_t) gyro_bias[0];
	gyro_offset[1] = (int16_t) gyro_bias[1];
	gyro_offset[2] = (int16_t) gyro_bias[2];

	acc_offset[0]  = (int16_t) accel_bias[0];
	acc_offset[1]  = (int16_t) accel_bias[1];
	acc_offset[2]  = (int16_t) accel_bias[2];
}


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
	I2C_write_byte(MPU6050_ADDRESS, SMPLRT_DIV, 4);  // Set sample rate to 1000 Hz
	I2C_write_byte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	I2C_write_byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	I2C_write_byte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	I2C_write_byte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
}

void ReadMPU6050(void)
{
	uint8_t i, fifo_count, packet_count;
	uint8_t data[12];
	int16_t gyro[3];
	int16_t acc[3];

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

		PlaceInGyroBuffor(gyro);
		PlaceInAccBuffor(acc);
	}
}