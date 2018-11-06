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

void CalibrateMPU6050(float * dest1, float * dest2)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

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
	_delay_ms(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	I2C_write_byte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	I2C_read_buf(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	
	SendStringUint("data[0]    = ", data[0]);
	SendStringUint("data[0]<<8 = ", data[0]<<8);
	SendStringUint("data[1]    = ", data[1]);

	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	SendStringUint("Packet count = ", packet_count);

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		I2C_read_buf(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
		
		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers; works well for gyro but not for accelerometer
// 	writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);
// 	writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
// 	writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
// 	writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
// 	writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
// 	writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    I2C_read_buf(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    I2C_read_buf(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    I2C_read_buf(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++) {
		if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Push accelerometer biases to hardware registers; doesn't work well for accelerometer
	//Are we handling the temperature compensation bit correctly?
// 	writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
// 	writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
// 	writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
// 	writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
// 	writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
// 	writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

	    // Output scaled accelerometer biases for manual subtraction in the main program
	    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
	    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;

		SendStringInt("Gyro X = ", gyro_bias[0]);
		SendStringInt("Gyro Y = ", gyro_bias[1]);
		SendStringInt("Gyro Z = ", gyro_bias[2]);

		SendStringInt("Acc  X = ", accel_bias[0]);
		SendStringInt("Acc  Y = ", accel_bias[1]);
		SendStringInt("Acc  Z = ", accel_bias[2]);
}