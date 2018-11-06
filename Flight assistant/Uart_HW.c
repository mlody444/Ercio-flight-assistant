/*
 * Uart.c
 *
 * Created: 2018-01-05 16:32:36
 *  Author: Bartek
 */ 
 
 #include <avr/io.h>
 #include <avr/interrupt.h>

 #include "Uart_HW.h"
 #include "Uart.h"




void InitUartCOM_HW(void)
{
	UBRR0 = 3;		//Baud rate 250k at 16MHz clock

	UCSR0B = (1<<RXEN0) | (1<<RXCIE0) |
			 (1<<TXEN0) | (1<<TXCIE0);
}

void InitUartCOM_asSbus_HW(void)
{
	UCSR0C |= (1<<UPM10)| (1<<USBS1);	// Parity EVEN, 2 stop bits


	UBRR0 = 9;	//100k at 16MHz clock

	//Init COM Uart
	UCSR0B = (1<<RXEN0) | (1<<RXCIE0) |
			 (1<<TXEN0) | (1<<TXCIE0);
}


void PutCharToBuffor(uint8_t x)
{
	if ((UCSR0A & (1<<UDRE0)) && (TxHead == TxTail))	//check if can put next char
	{
		UDR0 = x;		
	}
	else	//if not ready put char to sending queue
	{
		TxBuffor[TxHead] = x;
		TxHead = (TxHead+1) & TX_SIZE_BIN;
	}
}


ISR (USART0_TX_vect)
{
	if (TxTail == TxHead)
	{
		uint8_t x = UDR0;
	}

	else
	{
		UDR0 = TxBuffor[TxTail];
		TxTail = (TxTail+1) & TX_SIZE_BIN;
	}
}

ISR (USART0_RX_vect)	//COM Rx
{
// 	uint8_t x = UDR0;
// 	ReceiveChar(x);
// 	PutCharToBuffor(x);
	ReceiveChar(UDR0);
}