/*
 * Uart.h
 *
 * Created: 2018-01-06 21:27:23
 *  Author: Bartek
 */ 


#ifndef UART_H_
#define UART_H_

#define RX_SIZE_BIN 0b00011111
#define RX_SIZE 32
#define RX_CR_SIZE 8

#define TX_SIZE_BIN 0b11111111
#define TX_SIZE 256

extern char RxBuffor[RX_SIZE];
extern char TxBuffor[TX_SIZE];


void SendStringUint(char string[], uint16_t number);

void SendStringInt(char string[], int32_t number);

void SendEnter(void);

void InitUartCOM(void);

void ReceiveChar(char rX);

void SendChar(char x);

void SendString(char string[]);

void SendLine(char string[]);

void SendInt(int32_t number);

void SendUint(uint16_t number);

void CheckRxBuffor(void);

#endif /* UART_H_ */