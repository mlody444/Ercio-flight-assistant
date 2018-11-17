/*
 * RecieveCommands.h
 *
 * Created: 2018-01-12 07:25:52
 *  Author: Bartek
 */ 


#ifndef RECIEVECOMMANDS_H_
#define RECIEVECOMMANDS_H_

void ProcessString(uint8_t start, uint8_t stop);

typedef struct {
	char command[20];
	int8_t (*ER_service)(char * params);
} ER_CMD;

#endif /* RECIEVECOMMANDS_H_ */