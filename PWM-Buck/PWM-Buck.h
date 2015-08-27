/*
 * PWM_V1.h
 *
 * T4Forum-PWM-Module
 *
 * Created: 03.03.2012 13:33:06
 *  Author: hubert (Robert)
 */ 

#ifndef PWM_BUCK_H
#define PWM_BUCK_H

#include <avr/io.h>
#include <stdbool.h>
//#include <stdlib.h>
#include <avr/eeprom.h>
//#include "dog_display."
#ifndef F_CPU
#define F_CPU 8000000
#endif


/* I/O descriptions */

//layout version: buck
#define DISPLAY_PORT PORTB
#define DISPLAY_DDR DDRB
#define DISPLAY_PIN PINB
#define DISPLAY_LIGHT PB0

#define SH_PORT PORTD
#define SH_DDR DDRD
#define SH_PIN PIND

#define IGNITION	PD1
#define POT_SWITCH	PD2
#define AUX_HEAT	PD3
//#define PROG_PWM_VALUE		PD6
#define PROG_PWM_FREQ		PD7
#define PROG_WATER_TEMP     PD5


#define POT_VALUE	PC0
#define BAT2VOLTAGE PC1
#define TEMP_WATER	PC3
#define TEMP_FET	PC2


#define LED			PD4
#define PWM			PB1

#define FILTER_VALUE 3 //must be between 0 and 6

#define SYM_BAT1 "\x01"
#define SYM_BAT2 "\x02"
#define SYM_FET "\x03"
#define SYM_TEMP "\x04"
#define SYM_WTEMP1 "\x05"
#define SYM_WTEMP2 "\x06"

#define MIN_PWM_FREQUENCY 20
#define MAX_PWM_FREQUENCY 32768

#define MAX_FET_TEMP 115

#define MIN_BAT_VOLTAGE 1120//set in centiVolts

typedef enum stat{
	OFF = 0,
	IGNITION_ON,
	AUX_HEAT_ON,
	PROGRAM,
	ERROR,
}status_t;

typedef struct vol_val{
	int8_t integer;
	uint8_t fraction;
}voltage_value_t;

bool compare_string(uint16_t *str1, uint16_t *str2, uint8_t length);
int16_t calculate_temperature(uint16_t adc);
uint16_t read_adc(uint8_t portbit);
uint8_t calculate_pwm_percent(void);
voltage_value_t calculate_voltage(uint16_t adc);
void transmit_value(uint8_t page, uint8_t code, uint16_t value);
void print_error(void);

#endif