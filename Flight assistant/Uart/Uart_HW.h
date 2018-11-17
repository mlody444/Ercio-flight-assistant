/*
 * Uart.h
 *
 * Created: 2018-01-05 16:32:28
 *  Author: Bartek
 */ 


void InitUartCOM_HW(void);

void InitUartCOM_asSbus_HW(void);

void PutCharToBuffor(uint8_t x);






 extern char RxBuffor[];
 extern volatile uint8_t RxTail;
 extern volatile uint8_t RxHead;

 extern char TxBuffor[];
 extern volatile uint8_t TxTail;
 extern volatile uint8_t TxHead;
