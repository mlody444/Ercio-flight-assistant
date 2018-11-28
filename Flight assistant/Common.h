/*
 * Common.h
 *
 * Created: 2018-11-10 08:30:05
 *  Author: Bartek
 */ 


#ifndef COMMON_H_
#define COMMON_H_

#define LED_ON	PORTB |= (1<<PB5)
#define LED_OFF	PORTB &= ~(1<<PB5)
#define LED_TGL	PORTB ^= (1<<PB5)

#define BUZ_ON	PORTB &= ~(1<<PB1)
#define BUZ_OFF	PORTB |= (1<<PB1)
#define BUZ_TGL	PORTB ^= (1<<PB1) 

#define SAMPLES_BUFF_SIZE 20
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define COMMON_TIMERS 5

#define MPU6050_FS 250
#define POS_FS 50
#define MPU6050_LSB 65

#define POS_MIX_NORMAL 95
#define POS_MIX_SPECIAL 99

// 1g = 16384
// 1g^2 * 1.2 = 322122547
#define G_SPECIAL_MAX 322122547
// 1g^2 * 0.8 = 214748364
#define G_SPECIAL_MIN 214748364

volatile int16_t gyro_offset[3];
volatile int16_t acc_offset[3];

//pos values are stored as degrees + 100 - avoid double with enough accuracy
volatile int32_t pos_x;
volatile int32_t pos_y;

volatile uint16_t calculate_position_counter;
volatile uint16_t send_position_counter;
volatile uint16_t additional_timer;

typedef struct
{
	union
	{
		uint8_t bytes;
		struct
		{
			uint8_t serial_dbg	: 1;
			uint8_t unused		: 7;
		};
	};
}FLAG1;

volatile FLAG1 flag1;

typedef struct
{
	union
	{
		uint8_t flag_register;
		struct
		{
			uint8_t process_PWMs	: 1;
			uint8_t unused			: 7;
		}flag;
	};
}EVENT_REGISTER;

extern EVENT_REGISTER events;

void InitVariables(void);

void Events(void);

void PlaceInGyroBuffor(int16_t samples[3]);
void PlaceInAccBuffor(int16_t samples[3]);

void ReadGyroSample(int16_t reading_buf[3]);
void ReadAccSample(int16_t reading_buf[3]);

uint8_t SamplesGyroBuf();
uint8_t SamplesAccBuf();

void ReadGyroSample_ViewOnly(int16_t reading_buf[3]);
void ReadAcccSample_ViewOnly(int16_t reading_buf[3]);


void ReadGyroBuf(int16_t reading_buf[3][SAMPLES_BUFF_SIZE], uint8_t sam_nr);
void ReadAccBuf( int16_t reading_buf[3][SAMPLES_BUFF_SIZE], uint8_t sam_nr);

void ReadGyroBufAveraged(int16_t gyro[3]);
void ReadAccBufAveraged(int16_t gyro[3]);

void AveragingLinearMPU6050(int16_t data[3][SAMPLES_BUFF_SIZE], uint8_t samples, int16_t results[3]);
void AveragingWeightingMPU6050(int16_t data[3][SAMPLES_BUFF_SIZE], uint8_t samples, int16_t results[3]);

void EnableDebug(void);

#endif /* COMMON_H_ */