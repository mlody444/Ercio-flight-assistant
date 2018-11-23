/*
 * PWM_control.h
 *
 * Created: 2018-01-28 21:54:23
 *  Author: Bartek
 */ 


#ifndef PWM_CONTROL_H_
#define PWM_CONTROL_H_

#define PWM_MIN 17000
#define PWM_MAX 32000
#define ELEVON_MID 24500
#define ELEVON_RANGE 8000

#define THROTLE_MIN 20000
#define THROTLE_MAX 27500

#define MID_PWM  22000

#define PWM_THROTLE	0
#define PWM_LEFT	1
#define PWM_RIGHT	2


typedef struct {
	int16_t roll;
	int16_t pitch;
}TRIM;


typedef struct{
	uint16_t PWM;
	uint8_t *PORT;
	uint8_t mask;
}PWM_TIMER;

extern TRIM ram_trim;
extern TRIM eem_trim;

typedef enum
{
	Roll,
	Pitch,
	None,
} trim_selector;

typedef enum
{
	MOD_manual,
	MOD_gyro,
} FLIGHT_MODE;

typedef struct
{
	union{
		uint8_t bit_values;
		struct //8 bits total  
		{
			uint8_t DCH1		: 1;
			uint8_t DCH2		: 1;
			uint8_t frame_lost	: 1;
			uint8_t failsafe	: 1;
			uint8_t unused1		: 1;
			uint8_t unused2		: 1;
			uint8_t unused3		: 1;
			uint8_t ignore_last_frame : 1;
		} bits;
	};
	uint16_t throtle;
	int16_t  roll;
	int16_t  pitch;
	int16_t yaw;
	int16_t roll_trim_manual;
	int16_t pitch_trim_manual;
	int16_t roll_trim_gyro;
	int16_t pitch_trim_gyro;
	uint8_t buzzer;
	FLIGHT_MODE flight_mode;
	uint16_t sensitivity;
}CHANNELS;

extern CHANNELS channel_old, channel_new;
extern uint8_t sbus_frame_delay;

extern PWM_TIMER pwm_timers [3];

void InitTrimming(void);

#endif /* PWM_CONTROL_H_ */