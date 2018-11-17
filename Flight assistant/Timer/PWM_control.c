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

TRIM ram_trim;
TRIM eem_trim;


CHANNELS channel_old, channel_new;
uint8_t sbus_frame_delay;

PWM_TIMER pwm_timers [3];



void InitTrimming(void)
{
		eeprom_read_block(&ram_trim, &eem_trim, sizeof(ram_trim));

		if (ram_trim.pitch == 0xFFFF)
		{
			SendLine("Pitch trim reset");
			ram_trim.pitch = 0;
			eeprom_write_block(&ram_trim.pitch, &eem_trim.pitch, sizeof(ram_trim.pitch));
		}
		else
		{
//			SendStringUint("Pitch trim already set: ", ram_trim.pitch);
		}

		if (ram_trim.roll == 0xFFFF)
		{
			SendLine("Roll trim reset");
			ram_trim.roll = 0;
			eeprom_write_block(&ram_trim.roll, &eem_trim.roll, sizeof(ram_trim.roll));
		}
		else
		{
//			SendStringUint("Roll trim already set: ", ram_trim.pitch);
		}
}