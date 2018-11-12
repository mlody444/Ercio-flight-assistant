/*
 * Timer.h
 *
 * Created: 2018-11-12 12:31:02
 *  Author: Bartek
 */ 


#ifndef TIMER_H_
#define TIMER_H_

typedef struct
{
	uint16_t *counter;
	void (*timer_service)(void);
}TIMER_DECREMENT;

void InitCommonTimer(void);

void CheckCommonTimer(void);


#endif /* TIMER_H_ */