#ifndef TWI_TASK_H
#define TWI_TASK_H

#include <avr/io.h>
#include "twi_slave.h"


#define PWM_VALUE 0//..31
#define FREQUENCY 32//33
#define WATER_VAL 34
#define DELAY_VAL 35
#define STATUS	  36
#define PWM_PERCENT 37
#define T_FET 38
#define T_WATER 40
#define V_BAT	42//43

 
void twi_task(void);
uint8_t calculateID(char* name);
#endif