/*
 * twi_task.c
 *
 * Created: 21.08.2014 09:32:08
 *  Author: Hubert
 */ 

#include <avr/io.h>
#include "twi_slave.h"
#include "twi_task.h"
#include "PWM-Buck.h"


extern volatile uint8_t i2crxdata[i2c_buffer_size]; 
extern volatile uint8_t i2ctxdata[i2c_buffer_size]; 
extern volatile uint8_t buffer_adr;



extern volatile uint16_t pwm_freq;
extern volatile uint16_t pwm_value_aux;
extern volatile uint8_t pwm_percent;
extern volatile uint8_t water_value;
extern volatile uint8_t delay_value;
extern volatile int16_t water_temp;
extern volatile int16_t fet_temp;
extern volatile voltage_value_t vbat;

uint8_t calculateID(char* name){
	//calculate an ID from the first 3 Letter of its name
	uint8_t ID;
	ID = (name[0]-48) * 3 + (name[1]-48) * 2 + (name[2]-48);
	ID >> 2;
	return ID;
}

void twi_task(void){
	if(buffer_adr == 0){
		uint8_t i;
		//receiver
		pwm_freq = ((i2crxdata[FREQUENCY] << 8) + i2crxdata[FREQUENCY + 1]);
		//water_value
		water_value = i2ctxdata[WATER_VAL];
		//delay_value
		delay_value = i2ctxdata[DELAY_VAL];
		//pwm_percent
		
		//transmitter
		//pwm_freq
		i2ctxdata[FREQUENCY + 1] = (uint8_t) (pwm_freq & 0x00FF);
		i2ctxdata[FREQUENCY]     = (uint8_t) ((pwm_freq & 0xFF00) >> 8);
		//water_value
		i2ctxdata[WATER_VAL] = water_value;
		//delay_value
		i2ctxdata[DELAY_VAL] = delay_value;
		//status
		i2ctxdata[STATUS] = 0x00;
		//pwm_percent
		i2ctxdata[PWM_PERCENT] = pwm_percent;
		//water_temp
		i2ctxdata[T_WATER]     = (uint8_t) ((water_temp & 0xFF00) >> 8);
		i2ctxdata[T_WATER + 1] = (uint8_t) (water_temp & 0x00FF);
		//fet_temp
		i2ctxdata[T_FET]     = (uint8_t) ((fet_temp & 0xFF00) >> 8);
		i2ctxdata[T_FET + 1] = (uint8_t) (fet_temp & 0x00FF);
		//vbat1
		i2ctxdata[V_BAT] = vbat.integer;
		i2ctxdata[V_BAT + 1] = vbat.fraction;
		
	}		
}
