/*
* RecieveCommands.c
*
* Created: 2018-01-12 07:26:05
*  Author: Bartek
*/ 
#define F_CPU 16000000L

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stdlib.h>
#include <util/delay.h>

#include "Uart.h"
#include "Uart_HW.h"
#include "RecieveCommands.h"
#include "Common.h"

#define RX_COMMANDS 3	//number of rX commands

int8_t er_service(char * params);
int8_t er_read(char * params);
int8_t er_buf(char * params);


const ER_CMD ER_commands [RX_COMMANDS] PROGMEM = {
	{"ER", er_service},
	{"ER_READ", er_read},
	{"ER_BUF", er_buf},
};


void ProcessString(uint8_t start, uint8_t stop)
{
	//test
	int8_t (*_er_srv)(char * params);

	
	//	stop = (stop +1) & RX_SIZE_BIN;
	char string[RX_SIZE];
	uint8_t i;	//counter value

	for (i=0; i<RX_SIZE; i++)
	{
		string[i] = 0;
	}

	i=0;

	// Assign rX string to command
	while (start!=stop)
	{
		string[i] = RxBuffor[start];
		start = (start + 1) & RX_SIZE_BIN;
		i++;
	}

	char * rest;
	char * begining;

	begining = strtok_r(string, " ", &rest);

	uint8_t j;
	uint8_t command_length;
	uint8_t string_length = strlen(begining);
	for (j=0; j < RX_COMMANDS; j++)
	{
		command_length = strlen_P(ER_commands[j].command);
		
		if (command_length == string_length)
		{
			if (0 == strncasecmp_P(begining, ER_commands[j].command, i))
			{
				_er_srv = (void *)pgm_read_word(&ER_commands[j].ER_service);
				_er_srv(rest);
				break;
			}
		}
	}
	if (j==RX_COMMANDS)
	{
		SendString("Wrong command: ");
		SendString(string);
		SendEnter();
	}
		
}

int8_t er_service(char * params)
{
	SendLine("Uart is working");
	return 0;
}

int8_t er_read(char * params)
{
	int16_t gyro[3];
	int16_t acc[3];
	
	ReadGyroSample(gyro);
	ReadAccSample(acc);

	SendStringInt("G X ", gyro[0]);
	SendStringInt("G Y ", gyro[1]);
	SendStringInt("G Z ", gyro[2]);
	SendStringInt("A X ", acc[0]);
	SendStringInt("A Y ", acc[1]);
	SendStringInt("A Z ", acc[2]);

	return 0;
}

int8_t er_buf(char * params)
{
	int16_t bufor[3][SAMPLES_BUFF_SIZE];
	int32_t sum[3];
	uint8_t i, samples;	

	samples = SamplesGyroBuf();
	if (samples != 0)
	{
		ReadGyroBuf(bufor, &samples);
		sum[0] = 0;
		sum[1] = 0;
		sum[2] = 0;

		for(i = 0; i < samples; i++)
		{
			sum[X_AXIS] += bufor[X_AXIS][i];
			sum[Y_AXIS] += bufor[Y_AXIS][i];
			sum[Z_AXIS] += bufor[Z_AXIS][i];
		}

		sum[X_AXIS] /= (int16_t) samples;
		sum[Y_AXIS] /= (int16_t) samples;
		sum[Z_AXIS] /= (int16_t) samples;

 	 	SendStringInt("G X ", sum[0]);
 	 	SendStringInt("G Y ", sum[1]);
 	 	SendStringInt("G Z ", sum[2]);
	}

 	samples = SamplesAccBuf();
 	if (samples != 0)
 	{
		ReadAccBuf(bufor, &samples);
		sum[0] = 0;
		sum[1] = 0;
		sum[2] = 0;
 
		for(i = 0; i < samples; i++)
		{
			sum[X_AXIS] += bufor[X_AXIS][i];
			sum[Y_AXIS] += bufor[Y_AXIS][i];
			sum[Z_AXIS] += bufor[Z_AXIS][i];
		}

		sum[X_AXIS] /= (int16_t) samples;
		sum[Y_AXIS] /= (int16_t) samples;
		sum[Z_AXIS] /= (int16_t) samples;
		SendStringInt("A X ", sum[0]);
		SendStringInt("A Y ", sum[1]);
		SendStringInt("A Z ", sum[2]);
 	}

	SendStringInt("S ", samples);
	return 0;
}