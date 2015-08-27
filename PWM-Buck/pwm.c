/*
 * pwm.c
 *
 * Created: 16.10.2014 09:05:22
 *  Author: Hubert
 */ 
#include <avr/io.h>
#include "PWM-Buck.h"
#include "pwm.h"


extern volatile uint16_t max_ocr_value;
extern uint8_t dummy[200] EEMEM;
extern uint16_t ee_pwm_freq EEMEM;
extern uint8_t ee_water_value EEMEM;
extern uint8_t ee_delay_value EEMEM;
extern uint8_t ee_target_in_temp EEMEM;
extern volatile uint16_t pwm_freq;
extern volatile uint16_t pwm_value_aux;
extern volatile uint16_t pwm_top_value;
extern volatile uint16_t pwm_temp_value;
extern volatile uint8_t pwm_percent;
extern volatile bool pwm_running;
extern volatile uint8_t pwm_prescaler;
extern volatile uint8_t water_value;
extern volatile uint8_t delay_value;
extern volatile uint16_t adc_value[4];
extern volatile uint16_t old_adc_value[4];
extern volatile int i;
extern volatile uint16_t old_duty;
extern volatile int16_t water_temp;
extern volatile int16_t fet_temp;
extern volatile voltage_value_t vbat;
extern volatile uint16_t battery_voltage;
extern volatile uint8_t target_in_temp;

  /***********************************************************************************************/
 /* pwm functions																				*/
/***********************************************************************************************/
bool test_eeprom_string(uint16_t* str,uint8_t length){
	uint8_t i;
	for(i=0;i<length;i++){
		if((uint16_t) str[i]>1023) return false;
	}
	return true;
}
/***********************************************************************************************/
/***********************************************************************************************/
void save_pwm_values(void){ // write pwm_values into eeprom if they changed ;)
	//*
	volatile uint16_t temp_freq, _freq;
	_freq = pwm_freq;
	temp_freq = eeprom_read_word(&ee_pwm_freq);
	eeprom_busy_wait();
	//_delay_ms(5);
	volatile uint8_t temp_water_value, _water_value;
	_water_value = water_value;
	temp_water_value = eeprom_read_byte(&ee_water_value);
	eeprom_busy_wait();
	//_delay_ms(5);
	volatile uint8_t temp_delay_value, _delay_value;
	_delay_value = delay_value;
	temp_delay_value = eeprom_read_byte(&ee_delay_value);
	eeprom_busy_wait();
	
	if(temp_freq != pwm_freq){
		eeprom_update_word(&ee_pwm_freq,pwm_freq);
		eeprom_busy_wait();
	}
	if(temp_water_value != water_value){
		eeprom_update_byte(&ee_water_value,water_value);
		eeprom_busy_wait();
	}
	if(temp_delay_value != delay_value){
		eeprom_update_byte(&ee_delay_value,delay_value);
		eeprom_busy_wait();
	}
}
/***********************************************************************************************/
void load_pwm_values(void){
	uint8_t i;
	
	eeprom_busy_wait();
	uint16_t temp_pwm_freq = eeprom_read_word(&ee_pwm_freq);
	if(temp_pwm_freq<32768){
		pwm_freq = temp_pwm_freq;
	}else{
		pwm_freq = 32000;
	}
	eeprom_busy_wait();
	uint8_t temp_water_value = eeprom_read_byte((uint8_t*) &ee_water_value);
	if(temp_water_value<103){
		water_value = temp_water_value;
	}else{
		water_value = 25;
	}
	eeprom_busy_wait();
	uint8_t temp_delay_value = eeprom_read_byte((uint8_t*) &ee_delay_value);
	if(temp_delay_value<61){
		delay_value = temp_delay_value;
	}else{
		delay_value = 15;
	}
}
/***********************************************************************************************/
uint16_t set_pwm(uint16_t duty){ //0<duty<1023
	if(duty<0){
		duty = 0;
	}
	if(duty>1023){
		duty = 1023;
	}
	if(duty==0){
		pwm_running = false;
	}else{
		pwm_running = true;
	}
	uint32_t ocr_temp = duty * (uint32_t) ICR1 + duty;
	ocr_temp /= 1024;
	OCR1A = (uint16_t) ocr_temp;
	TCNT1 = 0;
	return OCR1A;
}
/***********************************************************************************************/
uint8_t set_pwm_prescaler(uint8_t div){ // set timer prescaler for pwm frequency setup
	uint8_t prescaler = 0;	//
	if (div == 64)
		prescaler |= ( (1<<CS11) | (1<<CS10) );
	if (div == 8)
		prescaler |= (1<<CS11);
	if (div == 1)
		prescaler |= (1<<CS10);
	return prescaler;
}
/***********************************************************************************************/
uint16_t set_pwm_freq(uint16_t freq){
	if(freq < 50) freq = 50;
	if(freq > 65472) freq = 65472;//32756
	/*fOCnxPWM = fclk_I/O / ( prescaler * (1 + ICR1))*/
	uint32_t duty = OCR1A * 1024; //0<=OCR1A<=ICR1
	uint16_t icr = ICR1; //255<=ICR1<=65535
	duty /= icr;
	uint8_t divider = 0;
	//if(freq != pwm_freq){
		TCCR1B &= ~(1<<CS12) & ~(1<<CS11) & ~(1<<CS10);
		if (freq < 16){
			divider = 64;
		}else if (freq < 122){
			divider = 8;
		}else{
			divider = 1;
		}			
		pwm_freq = freq;
		// top = f_cpu / ( N * f ) - 1
		pwm_top_value = F_CPU / (divider * freq) - 1;
		//if(icr != pwm_top_value){
			ICR1 = pwm_top_value;
			//set_pwm((uint16_t) duty);
			//_delay_ms(1); // wait a little
		//}
		TCCR1B |= set_pwm_prescaler(divider);
		set_pwm((uint16_t) duty);
		//_delay_ms(100);
	//}
	return divider;
}

/***********************************************************************************************/
void enable_pwm(uint16_t duty){
// 	if(TCCR1A == 0x00 || TCCR1B == 0x00){
// 		TCCR1A |= (1<<COM1A1) | (1<<WGM11);		// frequency and phasecorrect, non-inverted PWM
// 		TCCR1B |= (1<<WGM13) | (1<<WGM12);// | (1<<WGM10);	// Mode 15		
// 	}
	uint32_t temp = (old_duty << PWM_FILTER_VALUE) - old_duty;
	temp += duty;
	temp = temp >> PWM_FILTER_VALUE;
	if(old_duty < duty) temp += 1;
	set_pwm((uint16_t) temp);
	old_duty = temp;
}
/***********************************************************************************************/
void disable_pwm(void){
	uint32_t temp = (old_duty << PWM_FILTER_VALUE) - old_duty;
	temp = temp >> PWM_FILTER_VALUE;
	set_pwm((uint16_t) temp);
	old_duty = temp;
// 	if(temp == 0){
// 		TCCR1A = 0x00;
// 		TCCR1B = 0x00;
// 	}
}