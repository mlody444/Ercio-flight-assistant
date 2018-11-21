/*
 * Common.c
 *
 * Created: 2018-11-10 08:52:25
 *  Author: Bartek
 */ 

#include <avr/io.h>
#include <math.h>

#include "Common.h"
#include "PWM_control.h"
#include "Sbus.h"
#include "Timer3.h"
#include "Timer.h"	//DBG counter used for data sending over serial port


#define SAMPLES SAMPLES_BUFF_SIZE + 1

void ProcessPWMs(void);
void Read_Channels(CHANNELS *channels);
void Process_trimming(void);

EVENT_REGISTER events;

//buffors for samples
int16_t X_gyro_buff[SAMPLES];
int16_t Y_gyro_buff[SAMPLES];
int16_t Z_gyro_buff[SAMPLES];
/*Head & Tail must start from the same position*/
uint8_t gyro_head = 0;
uint8_t gyro_tail = 0;

int16_t X_acc_buff[SAMPLES];
int16_t Y_acc_buff[SAMPLES];
int16_t Z_acc_buff[SAMPLES];
/*Head & Tail must start from the same position*/
uint8_t acc_head = 0;
uint8_t acc_tail = 0;

uint8_t counter_testing = 0;


void PlaceInGyroBuffor(int16_t gyro_samples[])
{
	X_gyro_buff[gyro_head] = gyro_samples[0];
	Y_gyro_buff[gyro_head] = gyro_samples[1];
	Z_gyro_buff[gyro_head] = gyro_samples[2];

	gyro_head++;
	if (gyro_head == SAMPLES)
	gyro_head = 0;

	if (gyro_head == gyro_tail)
		gyro_tail++;

	if (gyro_tail == SAMPLES)
		gyro_tail = 0;
}

void PlaceInAccBuffor(int16_t acc_samples[])
{
	X_acc_buff[acc_head] = acc_samples[0];
	Y_acc_buff[acc_head] = acc_samples[1];
	Z_acc_buff[acc_head] = acc_samples[2];

	acc_head++;
	if (acc_head == SAMPLES)
		acc_head = 0;

	if (acc_head == acc_tail)
		acc_tail++;

	if (acc_tail == SAMPLES)
		acc_tail = 0;
}

void ReadGyroSample(int16_t reading_buf[])
{
	reading_buf[0] = X_gyro_buff[gyro_tail];
	reading_buf[1] = Y_gyro_buff[gyro_tail];
	reading_buf[2] = Z_gyro_buff[gyro_tail];

	gyro_tail++;

	if (gyro_tail == SAMPLES)
		gyro_tail = 0;
}

void ReadAccSample(int16_t reading_buf[])
{
	reading_buf[0] = X_acc_buff[acc_tail];
	reading_buf[1] = Y_acc_buff[acc_tail];
	reading_buf[2] = Z_acc_buff[acc_tail];

	acc_tail++;

	if (acc_tail == SAMPLES)
		acc_tail = 0;
}

void ReadGyroSample_ViewOnly(int16_t reading_buf[])
{
	uint8_t temporary = gyro_head;
	temporary--;
	if (temporary == 0xff)
		temporary = SAMPLES_BUFF_SIZE;

	reading_buf[0] = X_gyro_buff[temporary];
	reading_buf[1] = Y_gyro_buff[temporary];
	reading_buf[2] = Z_gyro_buff[temporary];
}

void ReadAccSample_ViewOnly(int16_t reading_buf[])
{
	uint8_t temporary = acc_head;
	temporary--;
	if (temporary == 0xff)
		temporary = SAMPLES_BUFF_SIZE;

	reading_buf[0] = X_acc_buff[temporary];
	reading_buf[1] = Y_acc_buff[temporary];
	reading_buf[2] = Z_acc_buff[temporary];
}

uint8_t SamplesGyroBuf()
{
	uint8_t gyro_samples;
	gyro_samples = gyro_head - gyro_tail;
	if (gyro_samples > SAMPLES_BUFF_SIZE)
	{
		gyro_samples -= (255 - SAMPLES_BUFF_SIZE);
	}
	return gyro_samples;
}

uint8_t SamplesAccBuf()
{
	uint8_t acc_samples;
	acc_samples = acc_head - acc_tail;
	if (acc_samples > SAMPLES_BUFF_SIZE)
	{
		acc_samples -= (255 - SAMPLES_BUFF_SIZE);
	}
	return acc_samples;
}

void ReadGyroBuf(int16_t  reading_buf[3][SAMPLES_BUFF_SIZE], uint8_t sam_nr)
{
	int16_t bufor[3];
	uint8_t i;
	for(i = 0; i < sam_nr; i++)	//read all new samples from buffor
	{
 		ReadGyroSample(bufor);
 		reading_buf[X_AXIS][i] = bufor[X_AXIS];
 		reading_buf[Y_AXIS][i] = bufor[Y_AXIS];
 		reading_buf[Z_AXIS][i] = bufor[Z_AXIS];
	}
}

void ReadAccBuf(int16_t  reading_buf[3][SAMPLES_BUFF_SIZE], uint8_t sam_nr)
{
	int16_t bufor[3];
	uint8_t i;
	for(i = 0; i < sam_nr; i++)	//read all new samples from buffor
	{
		ReadAccSample(bufor);
 		reading_buf[X_AXIS][i] = bufor[X_AXIS];
 		reading_buf[Y_AXIS][i] = bufor[Y_AXIS];
 		reading_buf[Z_AXIS][i] = bufor[Z_AXIS];
	}
}

/*** Before calling this function, make sure there are samples in bufor ***/
void ReadGyroBufAveraged(int16_t gyro[3])
{
	int16_t temp_buf[3][SAMPLES_BUFF_SIZE];	//bufor to store samples
	uint8_t samples;		//number of samples in bufor

	samples = SamplesGyroBuf();

	ReadGyroBuf(temp_buf, samples);
	AveragingLinearMPU6050(temp_buf, samples, gyro);
}

/*** Before calling this function, make sure there are samples in bufor ***/
void ReadAccBufAveraged(int16_t acc[3])
{
	int16_t temp_buf[3][SAMPLES_BUFF_SIZE];	//bufor to store samples
	uint8_t samples;	//number of samples in bufor

	samples = SamplesAccBuf();

	ReadAccBuf(temp_buf, samples);
	AveragingWeightingMPU6050(temp_buf, samples, acc);
}

void AveragingLinearMPU6050(int16_t data[3][SAMPLES_BUFF_SIZE], uint8_t samples, int16_t results[3])
{
	int32_t sum[3];
	sum[0] = 0;
	sum[1] = 0;
	sum[2] = 0;

	for(uint8_t i = 0; i < samples; i++)
	{
		sum[X_AXIS] += data[X_AXIS][i];
		sum[Y_AXIS] += data[Y_AXIS][i];
		sum[Z_AXIS] += data[Z_AXIS][i];
	}

	sum[X_AXIS] /= (int32_t) samples;
	sum[Y_AXIS] /= (int32_t) samples;
	sum[Z_AXIS] /= (int32_t) samples;

	results[X_AXIS] = (int16_t) sum[X_AXIS];
	results[Y_AXIS] = (int16_t) sum[Y_AXIS];
	results[Z_AXIS] = (int16_t) sum[Z_AXIS];
}

void AveragingWeightingMPU6050(int16_t data[3][SAMPLES_BUFF_SIZE], uint8_t samples, int16_t results[3])
{
	int32_t sum[3];
	sum[0] = 0;
	sum[1] = 0;
	sum[2] = 0;
	uint8_t average = 0;

	for(uint8_t i = 0; i < samples; i++)
	{
		average += (i+1)*(i+1);
		sum[X_AXIS] += ((int32_t)data[X_AXIS][i] * (i+1)*(i+1));
		sum[Y_AXIS] += ((int32_t)data[Y_AXIS][i] * (i+1)*(i+1));
		sum[Z_AXIS] += ((int32_t)data[Z_AXIS][i] * (i+1)*(i+1));
	}

	sum[X_AXIS] /= (int32_t) average;
	sum[Y_AXIS] /= (int32_t) average;
	sum[Z_AXIS] /= (int32_t) average;

	results[X_AXIS] = (int16_t) sum[X_AXIS];
	results[Y_AXIS] = (int16_t) sum[Y_AXIS];
	results[Z_AXIS] = (int16_t) sum[Z_AXIS];
}

void Events(void)
{
	if (events.flag.process_PWMs == 1)
	{
		ProcessPWMs();
		events.flag.process_PWMs = 0;
	}
}

void ProcessPWMs(void)
{
	channel_old = channel_new;
	Read_Channels(&channel_new);
	Process_trimming();

	//Set PWMs
	uint16_t sensitivity = (uint16_t)(((uint32_t)410 + (uint32_t)channel_new.sensitivity) * 10) / 203;	//div by 2025

	int16_t roll = ((int32_t)channel_new.roll * ELEVON_RANGE/2) / 820;
	int16_t pitch = ((int32_t)channel_new.pitch * ELEVON_RANGE/2) / 820;

	uint32_t left_pwm	= MID_PWM - roll - ram_trim.roll - pitch - ram_trim.pitch;
	uint32_t right_pwm	= MID_PWM - roll - ram_trim.roll + pitch + ram_trim.pitch;
	uint32_t throtle	= THROTLE_MIN + channel_new.throtle * 7;

	pwm_channels[0].counter = throtle;

	pwm_channels[1].counter = left_pwm;
	pwm_channels[2].counter = right_pwm;
}

void Read_Channels(CHANNELS *channels)
{
	channels->throtle	= channel_value[1] - 172;
	channels->roll		= channel_value[2] - 992;
	channels->pitch		= channel_value[3] - 992;
	channels->yaw		= channel_value[4] - 992;
	channels->bit_values= channel_value[0];
	
	
	if (channel_value[6] < 582)
	channels->trim_select = Roll;

	else if (channel_value[6] > 1574)
	channels->trim_select = Pitch;

	else
	channels->trim_select = None;
	
	channels->trim_value = ((int16_t)channel_value[7] - 992) / TRIM_MAX_FACTOR;

	if (channel_value[9] > 992)
	channels->buzzer = 1;
	
	else
	channels->buzzer = 0;

	uint16_t sens_help = (channel_value[8] - 172);

	channels->sensitivity = (uint16_t) sens_help;
	


	if (channel_value[10] < 582)
	channels->trim_reset = Reset_Roll;

	else if (channel_value[10] > 1574)
	channels->trim_reset = Reset_Pitch;

	else
	channels->trim_reset = Reset_None;
}

void Process_trimming(void)
{
	switch (channel_new.trim_select)
	{
		case Roll:
		ram_trim.roll = ram_trim.roll + channel_new.trim_value;

		if (ram_trim.roll > TRIM_MAX)
		{
			ram_trim.roll = TRIM_MAX;
		}
		else if (ram_trim.roll < - TRIM_MAX)
		{
			ram_trim.roll = - TRIM_MAX;
		}
		break;

		case Pitch:
		ram_trim.pitch = ram_trim.pitch + channel_new.trim_value;
		if (ram_trim.pitch > TRIM_MAX)
		{
			ram_trim.pitch = TRIM_MAX;
		}
		else if (ram_trim.pitch < - TRIM_MAX)
		{
			ram_trim.pitch = - TRIM_MAX;
		}
		break;
		case None:
		if (channel_old.trim_select != None)	//process saving to EEPROM
		{
			eeprom_write_block(&ram_trim, &eem_trim, sizeof(ram_trim));
		}
		break;
	}
}

/*Serial debug is enabled for 10 sec*/
void EnableDebug(void)
{
	dbg_counter = 10000;
	flag1.serial_dbg = 1;
}