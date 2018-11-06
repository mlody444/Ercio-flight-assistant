/*
 * TWI.h
 *
 * Created: 2018-04-02 18:24:06
 *  Author: Bartek
 */ 


#ifndef TWI_H_
#define TWI_H_

#define MAX_TWI_BUFOR_SIZE 10

typedef struct{
	uint8_t sla;	//slave adress
	uint8_t reg;	//register
}TWI_DATA;

typedef enum {	//if empty => tail == head
	no_data = 0,	//0
	done,			//1
	timeout,		//2
	pending,		//3
	start,			//3
	write_adress,	//5
	write_register,	//6
	restart,		//7
	set_to_read,	//8
	read,			//9
	stop_reading,	//10
	write,			//11
	last_byte_write,//12
	stop,			//13
}TWI_STATUS;

typedef struct{
	union
	{
		struct
		{
			uint8_t head: 3;
			uint8_t tail: 3;
			uint8_t unused: 2;
		};
	};
}TWI_BUFOR_POS;

typedef struct{
	union
	{
		struct
		{
			uint8_t read: 1;	//operation mode
			uint8_t sla	: 7;	//device register
		};
	};
	uint8_t reg;						//device register

	uint8_t bufor[MAX_TWI_BUFOR_SIZE];	//bufor to send data
	uint8_t buf_pos;					//last sent bufor position
	uint8_t *bufor_ref;
	uint8_t lenght;						//number of bytes to send/recieve
	uint8_t *flag_register;				//register where recieved mask will be stored
	uint8_t flag_mask;					//flag mask for register
	TWI_STATUS status;
}TWI_BUFOR_STRUCT;

void InitTWI(void);
void EventTWI(void);

void TWI_Timeout(void);


void TWI_write_bufor(uint8_t sla, uint8_t reg, uint8_t bufor[MAX_TWI_BUFOR_SIZE], uint8_t lenght);
void TWI_read_bufor(uint8_t sla, uint8_t reg, uint8_t *bufor, uint8_t lenght, uint8_t *flag_register, uint8_t flag_mask);
void TWI_write_byte(uint8_t sla, uint8_t reg, uint8_t byte);
void TWI_read_byte(uint8_t sla, uint8_t reg, uint8_t *byte, uint8_t *flag_register, uint8_t flag_mask);

TWI_BUFOR_STRUCT TWI_get_buf(uint8_t pos);

extern TWI_BUFOR_POS twi_buf_pos;



#endif /* TWI_H_ */