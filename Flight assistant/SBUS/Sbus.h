/*
 * SBUS.h
 *
 * Created: 2018-01-18 18:36:19
 *  Author: Bartek
 */ 


#ifndef SBUS_H_
#define SBUS_H_

extern uint16_t sbus_buffor_process_counter;
extern uint8_t	sbus_buffor_start_counter;
extern uint8_t	sbus_last_frame_counter;
extern volatile uint16_t channel_value[17];

void InitUartSBUS(void);

void SendSbusBuffor(void);

void Sbus_process_buffor(void);

uint16_t ReadSbusChannel(uint8_t channel);

#endif /* SBUS_H_ */