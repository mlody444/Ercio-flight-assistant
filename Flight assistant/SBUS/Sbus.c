/*
 * Sbus.c
 *
 * Created: 2018-01-18 18:37:28
 *  Author: Bartek
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>

#include "Uart.h"
#include "Timer.h"
#include "Sbus.h"

#define SBUS_BUFFOR_SIZE 26
#define START_FRAME 0b00001111

volatile uint8_t Sbus_Buffor[SBUS_BUFFOR_SIZE];
volatile uint8_t sbus_position;

volatile uint16_t channel_value[17];

uint16_t sbus_buffor_process_counter = 0xFFFF;		//3,3,3,3,3,3,3,2 ms between first & last byte
uint8_t  sbus_buffor_start_counter = 0xFF;		//9,9,9,9,9,9,9,8 ms between start frames
uint8_t	 sbus_last_frame_counter = 0xFF;


void Sbus_process_buffor(void)
{
	if (Sbus_Buffor[0] == 0b00001111 && Sbus_Buffor[24] == 0 && sbus_position == 24)	//frame can be processed
	{
		channel_value[1] = ((Sbus_Buffor[1]     |Sbus_Buffor[2]<<8)						 & 0x07FF);
		channel_value[2] = ((Sbus_Buffor[2] >>3 |Sbus_Buffor[3]<<5)						 & 0x07FF);
		channel_value[3] = ((Sbus_Buffor[3] >>6 |Sbus_Buffor[4]<<2 |Sbus_Buffor[5]<<10)	 & 0x07FF);
		channel_value[4] = ((Sbus_Buffor[5] >>1 |Sbus_Buffor[6]<<7)                      & 0x07FF);
		channel_value[5] = ((Sbus_Buffor[6] >>4 |Sbus_Buffor[7]<<4)                      & 0x07FF);
		channel_value[6] = ((Sbus_Buffor[7] >>7 |Sbus_Buffor[8]<<1 |Sbus_Buffor[9]<<9)	 & 0x07FF);
		channel_value[7] = ((Sbus_Buffor[9] >>2 |Sbus_Buffor[10]<<6)                     & 0x07FF);
		channel_value[8] = ((Sbus_Buffor[10]>>5 |Sbus_Buffor[11]<<3)                     & 0x07FF);
		channel_value[9] = ((Sbus_Buffor[12]    |Sbus_Buffor[13]<<8)                     & 0x07FF);
		channel_value[10]= ((Sbus_Buffor[13]>>3 |Sbus_Buffor[14]<<5)                     & 0x07FF);
		channel_value[11]= ((Sbus_Buffor[14]>>6 |Sbus_Buffor[15]<<2|Sbus_Buffor[16]<<10) & 0x07FF);
		channel_value[12]= ((Sbus_Buffor[16]>>1 |Sbus_Buffor[17]<<7)                     & 0x07FF);
		channel_value[13]= ((Sbus_Buffor[17]>>4 |Sbus_Buffor[18]<<4)                     & 0x07FF);
		channel_value[14]= ((Sbus_Buffor[18]>>7 |Sbus_Buffor[19]<<1|Sbus_Buffor[20]<<9)	 & 0x07FF);
		channel_value[15]= ((Sbus_Buffor[20]>>2 |Sbus_Buffor[21]<<6)                     & 0x07FF);
		channel_value[16]= ((Sbus_Buffor[21]>>5 |Sbus_Buffor[22]<<3)                     & 0x07FF);
		channel_value[0] = Sbus_Buffor[23];
	}
	else
	{
		if (Sbus_Buffor[0] != 0b00001111) SendLine("Wrong Sbus frame");
		if (Sbus_Buffor[24] != 0) SendLine("Wrong last frame");
		if (sbus_position != 24){
			SendStringUint("sbus = ", sbus_position);
		}
	}

	for (uint8_t i=0; i<SBUS_BUFFOR_SIZE; i++)
	{
		Sbus_Buffor[i] = 0xFF;
	}
	sbus_buffor_process_counter = 0xFFFF;
	sbus_position = 0xFF;
}

void InitUartSBUS(void)
{
	//Setting Sbus UART
	UCSR1C |= (1<<UPM10)| (1<<USBS1);	// Parity EVEN, 2 stop bits

	UBRR1 = 9;	//100k at 16MHz clock

	UCSR1B = (1<<RXCIE1)| (1<<RXEN1);	//Rx interrupt enable
}

void SendSbusBuffor(void)
{
	for(uint8_t i=0; i<SBUS_BUFFOR_SIZE; i++)
	{
		SendChar(Sbus_Buffor[i]);
	}
	SendEnter();
}

ISR (USART1_RX_vect)	//COM Rx
{
	uint8_t x = UDR1;
	
	if (x == 0b00001111 && sbus_last_frame_counter > 3)	//first frame procedure
	{
		sbus_position = 0;
		Sbus_Buffor[sbus_position] = x;
		sbus_buffor_process_counter = 4;
		sbus_buffor_start_counter = 0;
	}
	else if (sbus_position == 24)	//this should never appear, delete obtained buffor
	{
		Sbus_Buffor[sbus_position] = 0xFF;	//frame will be rejected
		sbus_buffor_process_counter = 0;
	}

	else if (sbus_position != 0xFF)	//obtain frames
	{
		sbus_position++;
		Sbus_Buffor[sbus_position] = x;
	}
	sbus_last_frame_counter = 0;
}