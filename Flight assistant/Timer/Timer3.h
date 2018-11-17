/*
 * Timer3.h
 *
 * Created: 2018-02-21 18:26:12
 *  Author: Bartek
 */ 


#ifndef TIMER3_H_
#define TIMER3_H_

#define PWM_CHANNELS 3

typedef struct
{
	uint8_t pin;
	uint16_t counter;
}PWM_CHANNEL;

extern PWM_CHANNEL pwm_channels[PWM_CHANNELS];

void InitTimer3(void);



#endif /* TIMER3_H_ */