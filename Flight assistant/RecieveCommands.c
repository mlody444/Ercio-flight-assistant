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

#define RX_COMMANDS 1	//number of rX commands

int8_t er_service(char * params);


const ER_CMD ER_commands [RX_COMMANDS] PROGMEM = {
	{"ER", er_service},
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