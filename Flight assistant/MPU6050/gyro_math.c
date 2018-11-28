/*
 * gyro_math.c
 *
 * Created: 2018-11-12 19:19:25
 *  Author: Bartek
 */ 

#include <avr/io.h>
#include <math.h>
#include <stdlib.h>

#include "Uart.h"
#include "Common.h"
#include "gyro_math.h"

#define POS_MIX_NORMAL 95
#define POS_MIX_SPECIAL 99

#define ACC_NORMAL (100 - POS_MIX_NORMAL)
#define ACC_SPECIAL (100 - POS_MIX_SPECIAL)

#define GYRO_NORMAL POS_MIX_NORMAL
#define GYRO_SPECIAL POS_MIX_SPECIAL

#define GYRO_DIV (POS_FS * MPU6050_LSB)

int32_t CalculateNewPos(double pos_acc, int32_t pos_gyro, int32_t pos_total, uint32_t total_g);


double ProcessAngle(int16_t width, int16_t high)
{
	double angle = 0;
	if (width != 0)	//for 0 width atan can be 0 or 180, but should be 0
	{
		angle = atan((double)high / (double)width);
		/* Process to correct angles */
		if (width > 0)
			angle = M_PI_2 - angle;
		else
			angle = (M_PI_2 + angle) * (-1);

		angle = (angle * 180) / M_PI;
	}
	return angle;
}

void ProcessPosition(int16_t gyro[3], int16_t acc[3])
{
	int32_t pomocnicza, pos_acc;
	double x_pos_acc = ProcessAngle(acc[Y_AXIS], acc[Z_AXIS]);
	double y_pos_acc = (ProcessAngle(acc[X_AXIS], acc[Z_AXIS]) * (-1));	

	uint32_t total_g = CheckTotalG(acc);

	/* by default all values stored in _pos are in degrees*100*/
	int32_t x_pos_gyro = (int32_t)gyro[X_AXIS] * 100;
	int32_t y_pos_gyro = (int32_t)gyro[Y_AXIS] * 100;

	x_pos_gyro = x_pos_gyro / GYRO_DIV;
	y_pos_gyro = y_pos_gyro / GYRO_DIV;

	pos_x = CalculateNewPos(x_pos_acc, x_pos_gyro, pos_x, total_g);
	pos_y = CalculateNewPos(y_pos_acc, y_pos_gyro, pos_y, total_g);
}

uint32_t CheckTotalG(int16_t acc_vector[3])
{
	uint32_t value;
	value =	(uint32_t)acc_vector[X_AXIS] * (uint32_t)acc_vector[X_AXIS] +
			(uint32_t)acc_vector[Y_AXIS] * (uint32_t)acc_vector[Y_AXIS] +
			(uint32_t)acc_vector[Z_AXIS] * (uint32_t)acc_vector[Z_AXIS];

	return value;
}

int32_t CalculateNewPos(double pos_acc, int32_t pos_gyro, int32_t pos_total, uint32_t total_g)
{
	int32_t pos_acc_calc, temporary;

	if (total_g > G_SPECIAL_MAX || total_g < G_SPECIAL_MIN)
	{
		pos_acc *= ACC_SPECIAL;
		pos_acc_calc = (int32_t)pos_acc;
		temporary = pos_total + pos_gyro;
		temporary *= GYRO_SPECIAL;
		temporary /= 100;
		return pos_acc_calc + temporary;
	}
	else
	{
		pos_acc *= ACC_NORMAL;
		pos_acc_calc = (int32_t)pos_acc;
		temporary = pos_total + pos_gyro;
		temporary *= GYRO_NORMAL;
		temporary /= 100;
		return pos_acc_calc + temporary;
	}
}

int32_t ProcessPID(int32_t pos_actual, int32_t pos_set, PID_PARAMETERS * PID)
{
	int32_t error_temp = pos_set - pos_actual;
	int32_t result;
	
	if (PID->I_gain){
		PID->i_mem += PID->I_gain * error_temp;

		if (PID->i_mem > ((int32_t)PID->P_gain * 500)){
			PID->i_mem = (int32_t)PID->P_gain * 500;
		}
		else if (PID->i_mem < -((int32_t)PID->P_gain * 500)){
			PID->i_mem = -(int32_t)PID->P_gain * 500;
		}
	}
	else{
		PID->i_mem = 0;
	}

	result = PID->P_gain * error_temp + PID->i_mem + PID->D_gain*(error_temp - PID->last_error);

	PID->last_error = error_temp;

	return result;
}