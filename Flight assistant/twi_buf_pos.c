/*
 * TWI.c
 *
 * Created: 2018-04-02 18:23:57
 *  Author: Bartek
 */ 
#define F_CPU 16000000L
#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include "twi_buf_pos.h"
#include "Uart.h"

#define ACK 1
#define NACK 0


#define MAX_TWI_QUEUE_SIZE 8
#define TWI_MAX_COMMANDS (6 + MAX_TWI_BUFOR_SIZE)	//6 commands + max data

void TWI_start(void);
void TWI_stop(void);
void TWI_write(uint8_t bajt);
void TWI_read(uint8_t ack);


uint8_t twi_reset_counter;


void (*twi_command[1])(uint8_t buf_pos);

TWI_BUFOR_STRUCT twi_bufor[MAX_TWI_QUEUE_SIZE];
TWI_BUFOR_POS twi_buf_pos;

void InitTWI(void)
{
	//set SCL to 400kHz for 16MHz
	TWSR0 = 0x00;
	TWBR0 = 0x0C;
	//enable TWI
}

void EventTWI(void)	//Check if TWI can be started
{
	if (twi_bufor[twi_buf_pos.tail].status == done)
	{
		twi_bufor[twi_buf_pos.tail].status = no_data;
		twi_buf_pos.tail ++;
		SendStringUint("T", twi_buf_pos.tail);
	}

	if(twi_bufor[twi_buf_pos.tail].status == pending)
	{
		TWI_start();
		twi_bufor[twi_buf_pos.tail].status = start;
		twi_reset_counter = 5;	//time out in ms
	}
}

void TWI_Timeout(void)	//drop current tusk, restart TWI
{
	twi_bufor[twi_buf_pos.tail].status = timeout;
	TWI_stop();
}


void TWI_write_bufor(uint8_t sla, uint8_t reg, uint8_t bufor[MAX_TWI_BUFOR_SIZE], uint8_t lenght)
{
	//set all parameters
	twi_bufor[twi_buf_pos.head].read   = 0;
	twi_bufor[twi_buf_pos.head].sla	   = sla;
	twi_bufor[twi_buf_pos.head].reg    = reg;

	for (uint8_t i=0; i<lenght; i++)
	{
		twi_bufor[twi_buf_pos.head].bufor[i] = bufor[i];
	}

	twi_bufor[twi_buf_pos.head].lenght = lenght;
	twi_bufor[twi_buf_pos.head].flag_register = 0;
	twi_bufor[twi_buf_pos.head].flag_mask	  = 0;

	twi_bufor[twi_buf_pos.head].status = pending;

	//set position for new task
	twi_buf_pos.head++;
}
void TWI_read_bufor(uint8_t sla, uint8_t reg, uint8_t *bufor, uint8_t lenght, uint8_t *flag_register, uint8_t flag_mask)
{
	twi_bufor[twi_buf_pos.head].read   = 1;
	twi_bufor[twi_buf_pos.head].sla	   = sla;
	twi_bufor[twi_buf_pos.head].reg    = reg;

	twi_bufor[twi_buf_pos.head].bufor_ref = bufor;

	twi_bufor[twi_buf_pos.head].lenght = lenght;
	twi_bufor[twi_buf_pos.head].flag_register = flag_register;
	twi_bufor[twi_buf_pos.head].flag_mask	  = flag_mask;

	twi_bufor[twi_buf_pos.head].status = pending;

	//set position for new task
	twi_buf_pos.head++;
}
void TWI_write_byte(uint8_t sla, uint8_t reg, uint8_t byte)
{
	//set all parameters
	twi_bufor[twi_buf_pos.head].read   = 0;
	twi_bufor[twi_buf_pos.head].sla	   = sla;
	twi_bufor[twi_buf_pos.head].reg    = reg;

	twi_bufor[twi_buf_pos.head].bufor[0] = byte;

	twi_bufor[twi_buf_pos.head].lenght = 1;
	twi_bufor[twi_buf_pos.head].flag_register = 0;
	twi_bufor[twi_buf_pos.head].flag_mask	  = 0;

	twi_bufor[twi_buf_pos.head].status = pending;

	//set position for new task
	twi_buf_pos.head++;
}
void TWI_read_byte(uint8_t sla, uint8_t reg, uint8_t *byte, uint8_t *flag_register, uint8_t flag_mask)
{
	//set all parameters
	twi_bufor[twi_buf_pos.head].read   = 1;
	twi_bufor[twi_buf_pos.head].sla	   = sla;
	twi_bufor[twi_buf_pos.head].reg    = reg;

	twi_bufor[twi_buf_pos.head].bufor_ref = byte;

	twi_bufor[twi_buf_pos.head].lenght = 1;
	twi_bufor[twi_buf_pos.head].flag_register = 0;
	twi_bufor[twi_buf_pos.head].flag_mask	  = 0;

	twi_bufor[twi_buf_pos.head].status = pending;
	
	//set position for new task
	twi_buf_pos.head++;
}

TWI_BUFOR_STRUCT TWI_get_buf(uint8_t pos)
{
	if (pos > 7)
	{
		pos = 7;
	}
	return twi_bufor[pos];
}

//low level functions

void TWI_start(void) {
	TWCR0 = (1<<TWINT)|(1<<TWEN)|(1<<TWSTA)|(1<<TWIE);
}

void TWI_stop(void) {
	TWCR0 = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO)|(1<<TWIE);
}

void TWI_write(uint8_t bajt) {
	TWDR0 = bajt;
	TWCR0 = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
}

void TWI_read(uint8_t ack) {
	TWCR0 = (1<<TWINT)|(ack<<TWEA)|(1<<TWEN)|(1<<TWIE);
}

ISR (TWI0_vect)	//TWI finished vector
{	//TWI interrupt

	uint8_t SLA = ((uint8_t)twi_bufor[twi_buf_pos.tail].sla << 1);
	uint8_t REG = twi_bufor[twi_buf_pos.tail].reg;
	
	switch (twi_bufor[twi_buf_pos.tail].status){
		case start:
			TWI_write(SLA);
			twi_bufor[twi_buf_pos.tail].status = write_adress;
			break;

		case write_adress:
			TWI_write(REG);
			twi_bufor[twi_buf_pos.tail].status = write_register;
			break;

		case write_register:
			if (twi_bufor[twi_buf_pos.tail].read)	//check if reading is required
			{
				TWI_start();
				twi_bufor[twi_buf_pos.tail].status = restart;
			}
			else	//process writing
			{
				if (twi_bufor[twi_buf_pos.tail].lenght > 1){	//if more than 1 char to write
					TWI_write(twi_bufor[twi_buf_pos.tail].bufor[0]);
					twi_bufor[twi_buf_pos.tail].status = write;
					twi_bufor[twi_buf_pos.tail].buf_pos = 0;
				}

				else {	//only 1 byte to send
					TWI_write(twi_bufor[twi_buf_pos.tail].bufor[0]);
					twi_bufor[twi_buf_pos.tail].status = last_byte_write;
				}
				twi_bufor[twi_buf_pos.tail].lenght --;
			}
			break;

		case restart:
			SLA++;
			TWI_write(SLA);
			twi_bufor[twi_buf_pos.tail].status = set_to_read;
			break;

		case set_to_read:
			if (twi_bufor[twi_buf_pos.tail].lenght > 1){
				TWI_read(ACK);
				twi_bufor[twi_buf_pos.tail].status = read;

			}
			
			else{
				TWI_read(NACK);
				twi_bufor[twi_buf_pos.tail].status = stop_reading;
			}

			twi_bufor[twi_buf_pos.tail].lenght --;
			break;

		case read:
			*twi_bufor[twi_buf_pos.tail].bufor_ref++ = TWDR0;	//read received byte
			if (twi_bufor[twi_buf_pos.tail].lenght > 1){
				TWI_read(ACK);	//going to read more
				twi_bufor[twi_buf_pos.tail].status = read;
			}
			else{
				TWI_read(NACK);	//finishing reading
				twi_bufor[twi_buf_pos.tail].status = stop_reading;
			}
			twi_bufor[twi_buf_pos.tail].lenght --;
			break;

		case stop_reading:
			*twi_bufor[twi_buf_pos.tail].bufor_ref++ = TWDR0;	//read last received byte
			*twi_bufor[twi_buf_pos.tail].flag_register = twi_bufor[twi_buf_pos.tail].flag_mask;	//set read done flag
			TWI_stop();
			twi_bufor[twi_buf_pos.tail].status = done;
			break;

		case write:
			if (twi_bufor[twi_buf_pos.tail].lenght > 1){	//if more than 1 char to write
				TWI_write(twi_bufor[twi_buf_pos.tail].bufor[0]);
				twi_bufor[twi_buf_pos.tail].status = write;
				twi_bufor[twi_buf_pos.tail].buf_pos = 0;
			}

			else {	//only 1 byte to send
				TWI_write(twi_bufor[twi_buf_pos.tail].bufor[0]);
				twi_bufor[twi_buf_pos.tail].status = last_byte_write;
			}
			twi_bufor[twi_buf_pos.tail].lenght --;
			break;

		case last_byte_write:
			TWI_stop();
			twi_bufor[twi_buf_pos.tail].status = done;
			twi_reset_counter = 0xFF;	//turn of timer, sending procedure finished correctly
			break;

		case done:
			//shouldn't happen
//			SendLine("Interrupt after TWI stop ocured");
			break;

		default:

			break;
	}
}