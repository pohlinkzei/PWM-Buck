/*
 * pwm.h
 *
 * Created: 16.10.2014 09:05:40
 *  Author: Hubert
 */ 


#ifndef PWM_H_
#define PWM_H_
#include <avr/io.h>
#include <stdbool.h>
//#include <stdlib.h>
#include <avr/eeprom.h>
#include "PWM-Buck.h"

#define PWM_FILTER_VALUE 4


bool test_eeprom_string(uint16_t* str,uint8_t length);
void save_pwm_values(void);
void load_pwm_values(void);
uint16_t set_pwm(uint16_t duty);
uint8_t set_pwm_prescaler(uint8_t div);
uint16_t set_pwm_freq(uint16_t freq);
void enable_pwm(uint16_t duty);
void disable_pwm(void);

#endif /* PWM_H_ */