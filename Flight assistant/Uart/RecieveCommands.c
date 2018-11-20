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
#include <math.h>

#include "Uart.h"
#include "Uart_HW.h"
#include "RecieveCommands.h"
#include "Common.h"
#include "Sbus.h"

#define RX_COMMANDS 5	//number of rX commands

int8_t er_service(char * params);
int8_t er_read(char * params);
int8_t er_buf(char * params);
int8_t er_dbg(char * params);
int8_t er_sbus(char * params);


const ER_CMD ER_commands [RX_COMMANDS] PROGMEM = {
	{"ER", er_service},
	{"ER_READ", er_read},
	{"ER_BUF", er_buf},
	{"ER_DBG", er_dbg},
	{"ER_SBUS", er_sbus},
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
	
	ReadGyroSample_ViewOnly(gyro);
	ReadAccSample_ViewOnly(acc);

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
	int16_t data[3];

	if (SamplesGyroBuf())
	{
		ReadGyroBufAveraged(data);

 	 	SendStringInt("G X ", data[0]);
 	 	SendStringInt("G Y ", data[1]);
// 	 	SendStringInt("G Z ", data[2]);
	}

 	if (SamplesAccBuf())
 	{
		ReadAccBufAveraged(data);

		SendStringInt("A X ", data[0]);
		SendStringInt("A Y ", data[1]);
		SendStringInt("A Z ", data[2]);
 	}

	SendStringInt("S ", SamplesAccBuf());
	return 0;
}

int8_t er_dbg(char * params)
{
	if (*params == '0')
		flag1.serial_dbg = 0;
	else if (*params == '1')
		flag1.serial_dbg = 1;
}

int8_t er_sbus(char * params)
{
	SendStringUint("Ch 1 ", channel_value[1]);
	SendStringUint("Ch 2 ", channel_value[2]);
	SendStringUint("Ch 3 ", channel_value[3]);
	SendStringUint("Ch 4 ", channel_value[4]);
	return 0;
}