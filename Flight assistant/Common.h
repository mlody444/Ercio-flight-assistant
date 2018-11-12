/*
 * Common.h
 *
 * Created: 2018-11-10 08:30:05
 *  Author: Bartek
 */ 


#ifndef COMMON_H_
#define COMMON_H_

#define SAMPLES_BUFF_SIZE 20
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2


int16_t gyro_offset[3];
int16_t acc_offset[3];


void PlaceInGyroBuffor(int16_t samples[]);
void PlaceInAccBuffor(int16_t samples[]);

void ReadGyroSample(int16_t reading_buf[]);
void ReadAccSample(int16_t reading_buf[]);

uint8_t SamplesGyroBuf();
uint8_t SamplesAccBuf();

void ReadGyroBuf(int16_t reading_buf[3][SAMPLES_BUFF_SIZE], uint8_t sam_nr);
void ReadAccBuf( int16_t reading_buf[3][SAMPLES_BUFF_SIZE], uint8_t sam_nr);

void AveragingLinearMPU6050(int16_t data[3][SAMPLES_BUFF_SIZE], uint8_t samples, int16_t results[3]);
void AveragingWeightingMPU6050(int16_t data[3][SAMPLES_BUFF_SIZE], uint8_t samples, int16_t results[3]);


#endif /* COMMON_H_ */