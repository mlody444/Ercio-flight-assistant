/*
 * PWM_control.c
 *
 * Created: 2018-01-28 21:54:37
 *  Author: Bartek
 */ 


#include <avr/io.h>
#include <avr/eeprom.h>

#include "PWM_control.h"
#include "Uart.h"


CHANNELS channel_old, channel_new;
uint8_t sbus_frame_delay;

PWM_TIMER pwm_timers [3];