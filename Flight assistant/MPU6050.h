/*
 * MPU6050.h
 *
 * Created: 2018-05-15 16:33:22
 *  Author: Bartek
 */ 


#ifndef MPU6050_H_
#define MPU6050_H_

void CalibrateMPU6050(int16_t gyro_offset[], int16_t acc_offset[]);

void InitMPU6050(void);


#endif /* MPU6050_H_ */