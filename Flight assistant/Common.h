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

#define SAMPLES_BUFF_SIZE 20
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define COMMON_TIMERS 2

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

#define TRIM_MAX_FACTOR 50
#define SENSITIVITY_MIN 20
#define SENSITIVITY_MAX 100


volatile int16_t gyro_offset[3];
volatile int16_t acc_offset[3];

//pos values are stored as degrees + 100 - avoid double with enough accuracy
volatile int32_t pos_x;
volatile int32_t pos_y;

volatile uint16_t test_counter;

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


#endif /* COMMON_H_ */