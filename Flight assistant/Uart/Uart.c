/*
 * Uart.c
 *
 * Created: 2018-01-06 21:27:32
 *  Author: Bartek
 */ 

#include <avr/io.h>
#include "Uart_HW.h"
#include "Uart.h"
#include "RecieveCommands.h"

char RxBuffor[RX_SIZE];
volatile uint8_t RxTail = 0;
volatile uint8_t RxHead = 0;
 
char TxBuffor[TX_SIZE];
volatile uint8_t TxTail = 0;
volatile uint8_t TxHead = 0;

 //void HandleCR(uint8_t pos);



void InitUartCOM(void)
{
	InitUartCOM_HW();
}

void ReceiveChar(char rX)
{
	switch(rX)
	{
		case 10:	//CR
			//ignore
		case 0:
			//ignore empty
			break;
		default:
			RxBuffor[RxHead] = rX;
			RxHead = (RxHead + 1) & RX_SIZE_BIN;

			if (RxHead == RxTail)	//Bufor overrided incdication
			{
				
			}
			break;
	}
}

void SendChar(char x)
{
	PutCharToBuffor(x);
}

void SendString(char string[])
{
	uint8_t i=0;

	while (string[i] != 0)
	{
		SendChar(string[i]);
		i++;
	}
}

void SendLine(char string[])
{
	SendString(string);
	//End of line:
	SendEnter();
}

void SendStringUint(char string[], uint16_t number)
{
	SendString(string);
	SendUint(number);
	SendEnter();
}

void SendStringInt(char string[], int32_t number)
{
	SendString(string);
	SendInt(number);
	SendEnter();
}

void SendEnter(void)
{
	SendChar(13);
	SendChar(10);
}

void SendInt(int32_t number)
{
	char string[20];
	char reverse[20];	//int16 = 6 cyfr + znak + 0
	int8_t znak = 1;
	uint8_t pozycja = 0;
	uint8_t mnoznik = 10;
	
	for (uint8_t i=0; i<8; i++)	//zmienna tablicowa lokalna, trzeba wyzerowaæ wartoœci
	{
		reverse[i] = 0;
		string[i] = 0;
	}
	
	if (number<0) znak = -1;
	
	number = number * znak;
	
	for (uint8_t i=0; i<20; i++)
	{
		reverse[i] = number % mnoznik + 48;
		number = number - number % mnoznik;
		number = number / 10;
		pozycja = i;
		if (number == 0) break;
	}
	
	pozycja++;
	
	if (znak<0)
	{
		reverse[pozycja] = 45;
		pozycja++;
	}
	
	for (uint8_t i=0; i<pozycja; i++)
	{
		string[i] = reverse[pozycja-1-i];
	}
	string[pozycja] = 0;
	SendString(string);
}

void SendUint(uint16_t number)
{
		char string[8];
		char reverse[8];	//int16 = 6 cyfr + znak + 0
		uint8_t pozycja = 0;
		uint8_t mnoznik = 10;
		
		for (uint8_t i=0; i<8; i++)	//zmienna tablicowa lokalna, trzeba wyzerowaæ wartoœci
		{
			reverse[i] = 0;
			string[i] = 0;
		}
		
		
		for (uint8_t i=0; i<8; i++)
		{
			reverse[i] = number % mnoznik + 48;
			number = number - number % mnoznik;
			number = number / 10;
			pozycja = i;
			if (number == 0) break;
		}
		
		pozycja++;
		
		for (uint8_t i=0; i<pozycja; i++)
		{
			string[i] = reverse[pozycja-1-i];
		}
		string[pozycja] = 0;
		SendString(string);
}

void CheckRxBuffor(void)
{
	if (RxHead != RxTail)	//Check if buffer is not empty
	{
		uint8_t i = RxTail;

		while(i != RxHead)	//search for end of line
		{
			//If end of line process Command
			
			if (RxBuffor[i] == 13)
			{
				ProcessString(RxTail, i);
				i = (i+1) & RX_SIZE_BIN;
				RxTail = i;
				break;
			}

			i = (i+1) & RX_SIZE_BIN;
		}
	}
}