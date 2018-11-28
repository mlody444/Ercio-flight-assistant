/*
 * gyro_math.h
 *
 * Created: 2018-11-12 19:20:00
 *  Author: Bartek
 */ 


#ifndef GYRO_MATH_H_
#define GYRO_MATH_H_

typedef struct{
	uint16_t  P_gain;
	uint16_t  I_gain;
	uint16_t  D_gain;
	int16_t I_max;
	int32_t i_mem;
	int32_t last_error;
}PID_PARAMETERS;

PID_PARAMETERS pid_roll;
PID_PARAMETERS pid_pitch;

double ProcessAngle(int16_t high, int16_t width);
void ProcessPosition(int16_t gyro[3], int16_t acc[3]);
uint32_t CheckTotalG(int16_t acc_vector[3]);

int32_t ProcessPID(int32_t pos_actual, int32_t pos_set, PID_PARAMETERS * PID);

#endif /* GYRO_MATH_H_ */