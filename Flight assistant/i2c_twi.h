/*
 * i2c_twi.h
 *
 *  Created on: 2010-09-07
 *       Autor: Miros³aw Kardaœ
 */

#ifndef I2C_TWI_H_
#define I2C_TWI_H_

#define ACK 1
#define NACK 0



// funkcje

// ustawiamy prêdkoœæ na I2C w kHz (standardowa prêdkoœæ to 100 kHz)  i2cSetBitrate( 100 );
//void i2cSetBitrate(uint16_t bitrateKHz);


void I2C_start(void);
void I2C_stop(void);
void I2C_write(uint8_t bajt);
uint8_t I2C_read(uint8_t ack);

void I2C_write_buf( uint8_t SLA, uint8_t adr, uint8_t len, uint8_t *buf );
void I2C_read_buf(uint8_t SLA, uint8_t adr, uint8_t len, uint8_t *buf);

void I2C_write_byte(uint8_t SLA, uint8_t adr, uint8_t byte);
uint8_t I2C_read_byte(uint8_t SLA, uint8_t adr);


#endif /* I2C_TWI_H_ */
