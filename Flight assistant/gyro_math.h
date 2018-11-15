/*
 * gyro_math.h
 *
 * Created: 2018-11-12 19:20:00
 *  Author: Bartek
 */ 


#ifndef GYRO_MATH_H_
#define GYRO_MATH_H_


double ProcessAngle(int16_t high, int16_t width);
void ProcessPosition(int16_t gyro[3], int16_t acc[3]);
uint32_t CheckTotalG(int16_t acc_vector[3]);



#endif /* GYRO_MATH_H_ */