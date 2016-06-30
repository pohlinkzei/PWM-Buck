/*
 * PWM_V1.c
 *
 * T4Forum-PWM-Module
 *
 * Created: 03.03.2012 13:33:06
 *  Author: hubert (RobertP)
 *
 */ 

#ifndef F_CPU
#define F_CPU 8000000
#endif

#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "dog_display.h"
#include "PWM-Buck.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "twi_slave.h"
#include "twi_task.h"
#include "pwm.h"
#include <avr/sleep.h>
#define LED_CLR()	SH_PORT &= ~(1<<LED);

  /***********************************************************************************************/
 /* globals																						*/
/***********************************************************************************************/
 volatile uint16_t max_ocr_value;
 uint8_t dummy[200] EEMEM;
 uint16_t ee_pwm_freq EEMEM = 12345;
 uint8_t ee_water_value EEMEM = 45;
 uint8_t ee_delay_value EEMEM = 45;
 volatile uint16_t pwm_freq = 0;
 volatile uint16_t pwm_value_aux = 0;
 volatile uint16_t pwm_top_value = 0;
 volatile uint16_t pwm_temp_value = 0;
 volatile uint8_t pwm_percent = 0;
 volatile bool pwm_running = false;
 volatile uint8_t water_value = 0;
 volatile uint8_t delay_value = 0;
 volatile uint16_t adc_value[4] = {0,};
 volatile uint16_t old_adc_value[4] = {0,};
 volatile int i;
 volatile uint16_t old_duty = 0;
 volatile int16_t water_temp = 0;
 volatile int16_t fet_temp = 0;

 volatile voltage_value_t vbat = {0,0};
 volatile uint16_t battery_voltage = 0;

 volatile uint8_t target_in_temp = 20;


 volatile bool prog_reset = false;
 volatile bool off = false;
 volatile bool timer1_disabled = true;

char centigrade[2] = {0x18,'C'};//"\x18" "C    ";
char batsym[2] = {0x01,0x02};//SYM_BAT1 SYM_BAT2;
char tfetsym[2] = {0x03, 0x04};//SYM_FET SYM_TEMP;
char twatersym[2] = {0x05,0x06};//SYM_WTEMP1 SYM_WTEMP2;

 volatile uint16_t t2_val = 0;
 volatile uint16_t _t2_val = 0;
 volatile uint16_t second = 0;
 volatile uint8_t pwm_prescaler = 0;
 volatile status_t status = OFF;
 volatile uint8_t led_count = 0;
 volatile bool adc_read_enable = false;
 
 void __delay_ms(uint16_t ms);
  /***********************************************************************************************/
 /* misc functions																				*/
/***********************************************************************************************/
bool compare_string(uint16_t *str1, uint16_t *str2, uint8_t length){
	while(length--){
		if(str1[length] != str2[length]) return false;
	}
	return true;
}
/***********************************************************************************************/
void noop(void){	//no operation function to circumvent the compiler's optimizing-feature
	__asm__ __volatile__ ("nop" ::);
	}
/***********************************************************************************************/
/***********************************************************************************************/
void led_set(status_t status){
	switch (status){
		case AUX_HEAT_ON:{
			TIMSK0 = 0x00;
			SH_PORT |= (1<<LED);
			break;
		}	
		case PROGRAM:{
			OCR0A = 200;
			TIMSK0 |= (1<<OCIE0A);
			break;
		}	
		case ERROR:{
			OCR0A = 50;
			TIMSK0 |= (1<<OCIE0A);
			break;
		}
		case IGNITION_ON:{
			TIMSK0 = 0x00;
			SH_PORT &= ~(1<<LED);
			break;
		}
		default:{ 
			TIMSK0 = 0x00;
			SH_PORT &= ~(1<<LED);
			break;
		}			
	}	
}
/***********************************************************************************************/
status_t get_status(status_t old){
	status_t status = OFF;
	if(SH_PIN & (1<<IGNITION)) status = IGNITION_ON;
	if(SH_PIN & (1<<AUX_HEAT) && !(SH_PIN & (1<<IGNITION))) status = AUX_HEAT_ON;
	if(!(PIND & (1<<PROG_PWM_FREQ))  ||  !(PIND & (1<<PROG_WATER_TEMP))) status = PROGRAM;
	if((((calculate_voltage(adc_value[BAT2VOLTAGE]).integer * 100 + calculate_voltage(adc_value[BAT2VOLTAGE]).fraction) < (MIN_BAT_VOLTAGE)) && (SH_PIN & (1<<AUX_HEAT) && !(SH_PIN & (1<<IGNITION)))) || (calculate_temperature(adc_value[TEMP_FET]) > MAX_FET_TEMP)) status = ERROR;
	if((old == ERROR) && (calculate_temperature(adc_value[TEMP_FET]) > (MAX_FET_TEMP - 50))) status = ERROR;
	if((old == ERROR) && ((SH_PIN & (1<<AUX_HEAT) && (!(SH_PIN & (1<<IGNITION))))) && ((calculate_voltage(adc_value[BAT2VOLTAGE]).integer * 100 + calculate_voltage(adc_value[BAT2VOLTAGE]).fraction) < (1250))) status = ERROR;
	if((old == ERROR) && ((SH_PIN & (1<<AUX_HEAT) && (!(SH_PIN & (1<<IGNITION))))) && !(SH_PIN & (1<<POT_SWITCH))) status = ERROR;
	if(old != status ){
		led_set(status);
		if(status == AUX_HEAT_ON){
			second = 0;
			dog_init();
			//timer1_init();
		}
		if(status == IGNITION_ON){ 
			dog_init();
			//timer1_init();
		}			
		if(old == OFF){
			dog_init();
			off = false;
		}			
		if(old == PROGRAM){
			dog_init();
		}
	}
	return status;
}
/***********************************************************************************************/
uint16_t read_adc(uint8_t portbit){
	volatile uint16_t result;
	ADMUX |= portbit;
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC)){;}
	result = ADCW; //dummy readout
	result = 0;
	for(i=0;i<4;i++){
		while(ADCSRA & (1<<ADSC)){;}
		result += ADCW;
	}
	ADMUX &= ~portbit;
	return (result>>2);
}
/***********************************************************************************************/
void read_whole_adc(void){
	int i;
	adc_value[0] = read_adc(0);
	for(i=1; i<4;i++){//read all adc channels
		//old_adc_value[i] = adc_value[i];
		uint16_t temp = (adc_value[i]<<FILTER_VALUE) - adc_value[i];
		temp += read_adc(i);
		adc_value[i] = temp>>FILTER_VALUE;
	}
}
  /***********************************************************************************************/
 /* initialisazions																				*/
/***********************************************************************************************/
void var_init(void){
	//ee_pwm_value[16] EEMEM  = {0,255,511,511,786,786,786,786,1023,1023,1023,1023,1023,1023,1023,1023}; //preset for four concrete stages;
	//ee_pwm_freq EEMEM = 12345;
	//ee_water_value EEMEM = 45;
	//ee_delay_value EEMEM = 45;
	//ee_target_in_temp EEMEM = 0;
	//pwm_value[16] = {0,255,511,511,786,786,786,786,1023,1023,1023,1023,1023,1023,1023,1023}; //preset for four concrete stages
	pwm_freq = 0;
	pwm_value_aux = 0;
	pwm_top_value = 0;
	pwm_temp_value = 0;
	pwm_percent = 0;
	pwm_running = false;
	water_value = 0;
	delay_value = 0;
	adc_value[0] = 0;
	adc_value[1] = 0;
	adc_value[2] = 0;
	adc_value[3] = 0;
	old_adc_value[0] = 0;
	old_adc_value[1] = 0;
	old_adc_value[2] = 0;
	old_adc_value[3] = 0;
	old_duty = 0;
	water_temp = 0;
	fet_temp = 0;
	vbat.integer = 0;
	vbat.fraction = 0;
	battery_voltage = 0;

	prog_reset = false;
	off = false;
}

void timer0_init(void){//LED timer for status led 
	TCCR0A |= (1<<WGM01);//ctc
	TCCR0B |= (1<<CS02) | (1<<CS00); //prescaler:1024
	OCR0A = 125;
}
/***********************************************************************************************/
void timer1_init(void){
	// FastPWM Mode 14 (ICR1 = Top)) from 2 to 33.500 kHz;
    TCCR1A = 0x00;
	TCCR1B = 0x00;
    TCCR1A |= (1<<COM1A1) | (1<<WGM11);		// frequency and phasecorrect, non-inverted PWM
	TCCR1B |= (1<<WGM13) | (1<<WGM12);// | (1<<WGM10);	// Mode 15		
}
void timer1_disable(void){
	// FastPWM Mode 14 (ICR1 = Top)) from 2 to 33.500 kHz;
    TCCR1A = 0x00;
	TCCR1B = 0x00;
    //TCCR1A |= (1<<COM1A1) | (1<<WGM11);		// frequency and phasecorrect, non-inverted PWM
	//TCCR1B |= (1<<WGM13) | (1<<WGM12);// | (1<<WGM10);	// Mode 15		
}
/***********************************************************************************************/
void timer2_init(void){ 
	ACSR = 0x80;
	ASSR  = (1<< AS2);              
	__delay_ms(1000);               
	TCCR2B  = (1<<CS22) /*| (1<<CS21) |(1<<CS20) */;              
	while((ASSR & (1<< TCR2AUB)));   
	TIFR2   = (1<<TOV2);             
	TIMSK2 |= (1<<TOIE2);
}
/***********************************************************************************************/
void adc_init(void){
	ADMUX |= (1<<REFS0);										// VCC with external capacitor at AREF
	ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // ADC enabled, prescaler: 128
	ADCSRB = 0x00;
}
/***********************************************************************************************/
void avr_init(void){
	ACSR |= (1<<ACD);
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;
	SH_DDR |= (1<<LED);
	DISPLAY_DDR |= (1<<PWM) | (1<<DISPLAY_LIGHT);
	SH_PORT |= (1<<POT_SWITCH) | (1<<LED)| (1<<PROG_PWM_FREQ) /*| (1<<PROG_PWM_VALUE)*/ | (1<<PROG_WATER_TEMP);
	timer0_init();
	timer1_init();
	timer2_init();
	adc_init();
	//connect a Dogm132 or Dogm128 to SPI-Interface
	dog_spi_init();
	dog_init();
	DISPLAY_PORT &= ~(1<<DISPLAY_LIGHT);
	sei();
	SH_DDR |= (1<<LED);
}
/***********************************************************************************************/
voltage_value_t calculate_voltage(uint16_t adc){
	uint32_t voltage32 = (716 * (uint32_t) adc);
	voltage32 = voltage32 / 256;
	voltage_value_t voltage;
	voltage.integer = voltage32 / 100;
	voltage.fraction = voltage32 - (100*voltage.integer);
	return voltage;
}	
/***********************************************************************************************/
void sprint_voltage(char* str,voltage_value_t voltage){
	uint8_t tenner = 0;
	uint8_t singles = 0;
	singles = voltage.integer % 10;
	tenner = (voltage.integer - singles) / 10;
	str[0] = ' ';
	if(tenner == 0){
		str[1] = str[0];
		str[0] = ' ';
	}else{
		str[1] = tenner + '0';
	}
	str[2] = singles + '0';
	tenner = 0;
	singles = 0;
	str[3] = ',';
	singles = voltage.fraction % 10;
	tenner = (voltage.fraction - singles) / 10;
	str[4] = tenner + '0';
	str[5] = singles + '0';
}
/***********************************************************************************************/
void sprint_temperature(char* str,int16_t temperature){
	uint8_t one = 0, ten = 0, hundred = 0;
	bool _signed = false;
	if(temperature < 0){
		temperature *= -1;
		str[0] = '-';
		_signed = true;
		if(temperature > 99) temperature = 99;
	}
	one = temperature % 10;
	ten = (temperature % 100) / 10;
	hundred = (temperature % 1000) / 100;
	if(hundred == 0){
		if(ten == 0){
			if(_signed){
				str[1] = str[0];
			}else{
				str[1] = ' ';
			}
			str[0] = ' ';
		}else{
			if(_signed){
				str[0] = str[0];
			}else{
				str[0] = ' ';
			}
			str[1] = ten + '0';
		}
	}else{
		str[0] = hundred + '0';
		str[1] = ten + '0';
	}		
	str[2] = one + '0';
}
/***********************************************************************************************/
void uint8_to_string(char* str, uint8_t integer){
	uint8_t one = 0;
	uint8_t ten = 0;
	uint8_t hundred = 0;
	one = integer % 10;
	ten = (integer % 100) / 10;
	hundred = (integer % 1000) / 100;
	if(hundred == 0){
		if(ten == 0){
			str[0] = ' ';
			str[1] = ' ';
			str[2] = one + '0';
		}else{
			str[0] = ' ';
			str[1] = ten + '0';
			str[2] = one + '0';
		}
	}else{
		str[0] = hundred + '0';
		str[1] = ten + '0';
		str[2] = one + '0';
	}
}	
/***********************************************************************************************/
void uint16_to_string(char* str, uint16_t integer){
	uint8_t one = 0;
	uint8_t ten = 0;
	uint8_t hundred = 0;
	uint8_t thousand = 0;
	uint8_t tenthousand = 0;
	one = integer % 10;
	ten = (integer % 100) / 10;
	hundred = (integer % 1000) / 100;
	thousand = (integer % 10000) / 1000;
	tenthousand = (integer % 100000) / 10000;
	if(tenthousand == 0){
		if(thousand == 0){
			if(hundred == 0){
				if(ten == 0){
					str[0] = ' ';
					str[1] = ' ';
					str[2] = ' ';
					str[3] = ' ';
					str[4] = one + '0';
				}else{
					str[0] = ' ';
					str[1] = ' ';
					str[2] = ' ';
					str[3] = ten + '0';
					str[4] = one + '0';
				}
			}else{
				str[0] = ' ';
				str[1] = ' ';
				str[2] = hundred + '0';
				str[3] = ten + '0';
				str[4] = one + '0';
			}
		}else{
			str[0] = ' ';
			str[1] = thousand + '0';
			str[2] = hundred + '0';
			str[3] = ten + '0';
			str[4] = one + '0';
		}
	}else{
		str[0] = tenthousand + '0';
		str[1] = thousand + '0';
		str[2] = hundred + '0';
		str[3] = ten + '0';
		str[4] = one + '0';
	}
}	
/***********************************************************************************************/
void int16_to_string(char* str, int16_t integer){
	if(integer < 0){
		integer *= -1;
		str[0] = '-';
	}else{
		str[0] = ' ';
	}
	uint16_to_string(&str[1],integer);
}
/***********************************************************************************************/
uint8_t calculate_pwm_percent(){
	uint16_t ocr = OCR1A;
	uint16_t icr = ICR1;
	uint32_t temp = ((uint32_t) ocr * 100);
	temp /= (uint32_t) icr;
	return (uint8_t) temp;
}	
/***********************************************************************************************/
int16_t calculate_temperature(uint16_t adc){
	//t=0,64516129*adc-154,19354
	//#warning "TODO: update equation" done ;)
	int32_t adc_temp = 13213 * (int32_t) adc;
	adc_temp = adc_temp / 2048;
	adc_temp = adc_temp - 1547;
	adc_temp = adc_temp / 10;
	return (int16_t) adc_temp;
}
/***********************************************************************************************/
void calculate_values(){
	vbat = calculate_voltage(adc_value[BAT2VOLTAGE]); //zweitbat
	battery_voltage = battery_voltage;
	water_temp = calculate_temperature(adc_value[TEMP_WATER]);
	fet_temp = calculate_temperature(adc_value[TEMP_FET]);
	pwm_percent = calculate_pwm_percent();
}
/**********************************************************************************************/
void stringcopy(char* source, char* destination, uint8_t length){
	uint8_t i;
	for(i=0;i<length;i++){
		destination[i] = source[i];
	}
}
/***********************************************************************************************/
void print_pgm_water_val(uint8_t water_val){
	//											012345678901234
	dog_write_mid_string(NEW__POSITION(0,4,3), " Einschalttemp.");
						
	char str1[3] = {0,};
		//			01234567890
	char str[] = " T:     C  ";

	uint8_to_string(str1,water_value);
	str[4] = str1[1];
	str[5] = str1[2];
	str[6] = centigrade[0];
	dog_write_big_string(NEW__POSITION(3,4,5),str);
	dog_write_empty_line(NEW_POSITION(6,0));
	dog_write_empty_line(NEW_POSITION(7,0));
}
/***********************************************************************************************/
void print_pgm_delay(uint8_t delay_val){
	//dog_write_small_string(" Einschaltverz" "\x1F" "gerung ");
	dog_write_mid_string(NEW__POSITION(0,4,3), " Einschaltverz.");
	char str1[3] = {0,};
		//			01234567890
	char str[] = " t:    min ";
	//sprintf(str1," t:%2u min   ",delay_value);
	uint8_to_string(str1,delay_value);
	str[4] = str1[1];
	str[5] = str1[2];
	dog_write_big_string(NEW__POSITION(3,4,5),str);
	dog_write_empty_line(NEW_POSITION(6,0));
	dog_write_empty_line(NEW_POSITION(7,0));
}
/***********************************************************************************************/
void print_pgm_freq(uint16_t freq){
	//					  012345678901234
	dog_write_mid_string(NEW__POSITION(0,4,3)," PWM-Frequenz  ");
						//  1234567890123456789012
	
	//			  01234567890
	char str[] = "f:     Hz ";	//sprintf(str1," f: %5dHz  ",pwm_freq);
	uint16_to_string(&str[3],freq);
	//stringcopy(str1,&str[4],5);
	dog_write_big_string(NEW__POSITION(3,4,5),str);
	dog_write_empty_line(NEW_POSITION(6,0));
	dog_write_empty_line(NEW_POSITION(7,0));
}
/***********************************************************************************************/
/***********************************************************************************************/
void print_ign_aux(void){
	dog_home();
	if (adc_value[TEMP_WATER] > 999){// kein Tempsensor
		// 01234567890 big
		//  V1 12,34V
		//  TF 104 gC
		//  TW  25 gc
		char str0[] = " VB 12,34V ";
		str0[1] = batsym[0];
		str0[2] = batsym[1];
		sprint_voltage(&str0[3],vbat);
		char str1[] = " TW  20 gC ";
		sprint_temperature(&str1[4],fet_temp);
		str1[1] = tfetsym[0];
		str1[2] = tfetsym[1];
		str1[8] = centigrade[0];
		str1[9] = centigrade[1];
		dog_write_big_string(NEW__POSITION(0,0,7), str0);
		dog_write_big_string(NEW__POSITION(3,0,5), str1);
		//dog_write_big_string(NEW__POSITION(7,0,0), "           ");
		dog_write_empty_line(NEW_POSITION(6,0));
	}else{// mit Tempsensor	
		char str0[] = " VB 12,34V ";
		str0[1] = batsym[0];
		str0[2] = batsym[1];
		sprint_voltage(&str0[3],vbat);
		char str1[] = " TW  20 gC ";
		sprint_temperature(&str1[4],fet_temp);
		str1[1] = tfetsym[0];
		str1[2] = tfetsym[1];
		str1[8] = centigrade[0];
		str1[9] = centigrade[1];
		char str2[] = " TF 120 gC ";
		sprint_temperature(&str2[4],water_temp);
		str2[1] = twatersym[0];
		str2[2] = twatersym[1];
		str2[8] = centigrade[0];
		str2[9] = centigrade[1];
		dog_write_big_string(NEW__POSITION(0,0,0), str0);
		dog_write_big_string(NEW__POSITION(2,0,4), str1);
		dog_write_big_string(NEW__POSITION(5,0,0), str2);
		//dog_write_numbered_bat_symbol(NEW__POSITION(1,4,4),1);	
	}
}	
/***********************************************************************************************/
void print_error(){

	char vbat_s[5] = {0,}; 
	/* 0123456789012345678901
	 *       BS 12,34 V      
	 *      FFEEHHLLEERR     
	 *      FFEEHHLLEERR     
	 *       FT 125 °C       
	 */
	//			   0123456789012345678901
	char str0[] = "        V   ";
	char tfet_s[3] = {0,};
	sprint_temperature(tfet_s, fet_temp);
	sprint_voltage(vbat_s,vbat);
	dog_set_page(0);
	str0[0] = batsym[0];
	str0[1] = batsym[1];
	stringcopy(vbat_s,&str0[2],5);
	dog_write_big_string(NEW_POSITION(0,0),str0);
	dog_write_big_string(NEW_POSITION(2,0),"  FEHLER   ");
	dog_write_big_string(NEW_POSITION(4,0),"  FEHLER   ");
	dog_set_page(3);
	for(i=6;i<11;i++){
		str0[i] = ' ';
	}
	str0[0] = tfetsym[0];
	str0[1] = tfetsym[1];
	str0[2] = ' ';
	stringcopy(tfet_s, &str0[3],3);
	str0[7] = centigrade[0];
	str0[8] = centigrade[1];
	dog_write_big_string(NEW_POSITION(6,0),str0);
	//*/
}
/***********************************************************************************************/
void draw_bar_graph(uint8_t value){ // draw a per cent bar graph at the buttom line
//	char debug[22];
	 uint8_t i;
// 	for(i=0;i<22;i++) debug[i] = 0x20;
// 	uint16_to_string(debug, adc_value[POT_VALUE]);
// 	uint16_to_string(&debug[6], pwm_value_aux);
// 	uint16_to_string(&debug[12], value);
// 	uint16_to_string(&debug[18], ICR1);
	dog_set_position(ROWS - 1,0);
//	dog_write_small_string(debug);
//	return;
	dog_transmit_data(0x00);
	dog_transmit_data(0x00);
	dog_transmit_data(0x00);
	dog_transmit_data(0x00);
	
	//paint a fan
	uint8_t fan[8] = {0x00,0x39,0x1B,0x0F,0x3C,0x36,0x27,0x00};
	
	for(i=0; i<8; i++){
		dog_transmit_data(fan[i]);
	}
	//*
	//char text[22];
	//sprintf(text,"PWM: %i                  ",value);
	//dog_write_small_string(text);* /
	//dog_transmit_data(0xFF);
	for(i = 1; i < value; i++){
		if((i) % 5){
			dog_transmit_data(0x3F);//7e
		}else{
			dog_transmit_data(0x00);
		}
	}
	dog_transmit_data(0x00);
	while(i++ < COLUMNS-32){
		dog_transmit_data(0x00);
	}
	
	dog_transmit_data(0x0C);
	dog_transmit_data(0x1E);
	dog_transmit_data(0x3F);
	
	for(i=0; i<24; i++){
		dog_transmit_data(0x00);
	}	
	//char pwm_percent[3];
	//uint8_to_string(pwm_percent,calculate_pwm_percent());
	//sprintf(pwm_percent,"%3u",calculate_pwm_percent());
	//dog_write_small_string(pwm_percent);
	//dog_write_small_digit('%');
	//*/
}
/***********************************************************************************************/
void __delay_us(uint16_t us){
	while(us){
		_delay_us(1);
		us--;
	}
}
/***********************************************************************************************/
void __delay_ms(uint16_t ms){
	while(ms){
		_delay_ms(1);
		ms--;
	}
}

/***********************************************************************************************/
int main(void){
	avr_init();
	uint8_t addr = calculateID("PWM");
	init_twi_slave(addr);
	DISPLAY_PORT &= ~(1<<DISPLAY_LIGHT);
	_delay_ms(200);
	dog_clear_lcd();
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	adc_value[BAT2VOLTAGE] = 428;
	adc_value[TEMP_WATER] = 270;
	adc_value[TEMP_FET] = 270;
	status = OFF;
	LED_CLR();
	load_pwm_values();
	set_pwm_freq(pwm_freq);
	set_pwm(0);
	dog_clear_lcd();
	sei();
	while(1){
		noop();
		status_t status_old = status;
		status = get_status(status_old);
		if(adc_read_enable){
			read_whole_adc();				//measure all adc data
			calculate_values();
			adc_read_enable = false;
		}			
		dog_home();
		
		twi_rx_task();
		// we got new data via twi
		// save new values to variables
		pwm_freq = rx.pwm_freq;
		set_pwm_freq(pwm_freq);
		delay_value = rx.time_value;
		water_value = rx.water_value;
	
		switch (status){
			case OFF:{
				if(calculate_pwm_percent() > 0){
					disable_pwm();
					print_ign_aux();
					draw_bar_graph(calculate_pwm_percent());
					off = false;
				}else if((vbat.integer*100 + vbat.fraction > 1280)){
					print_ign_aux();
					dog_write_empty_line(NEW_POSITION(7,0));
					off = false;
				}else{					
					if(off == false){
						DISPLAY_PORT &= ~(1<<DISPLAY_LIGHT);
						DDRB &= ~(1<<PWM);
						save_pwm_values();
						dog_clear_lcd();
						dog_transmit(LCD_OFF);
						off = true;
					}else{
						OCR2A = 0;                       // Dummyzugriff
						while((ASSR & (1<< OCR2AUB)));   // Warte auf das Ende des Zugriffs
						sleep_mode();
					}
			
				}
		
				break;
			}				
			case IGNITION_ON:{
				DISPLAY_PORT &= ~(1<<DISPLAY_LIGHT);
				off = false;
				print_ign_aux();
				dog_write_empty_line(NEW_POSITION(7,0));
				break;
			}
			case AUX_HEAT_ON:{
				off = false;
				if((adc_value[TEMP_WATER] < 1016)){//check if sensor mounted!
					if(water_temp > water_value){	//check water temp
						if(!(SH_PIN & (1<<POT_SWITCH))){			//check if fan enabled
							pwm_value_aux = adc_value[POT_VALUE];	//start fan
							enable_pwm(pwm_value_aux);
													
						}else{
							disable_pwm();						//stop fan
						}
					}else{
						if(water_temp < (water_value - 10)){
							disable_pwm();						//water too cold -> stop fan
						}					
					}
				}else{
					if(second/60 >= delay_value){
						pwm_value_aux = adc_value[POT_VALUE];	//start fan
						enable_pwm(pwm_value_aux);
						second = delay_value*60;
					}
				}
				DISPLAY_PORT |= (1<<DISPLAY_LIGHT);
				print_ign_aux();
				draw_bar_graph(calculate_pwm_percent());
				break;
			}
			case PROGRAM:{
				DISPLAY_PORT |= (1<<DISPLAY_LIGHT);
				off = false;
				if(!(PIND & (1<<PROG_PWM_FREQ))){
					if(!(SH_PIN & (1<<POT_SWITCH))){
						pwm_freq = (adc_value[POT_VALUE]*64);
						set_pwm_freq(pwm_freq);
					}
					print_pgm_freq(pwm_freq);
										
				}else if(!(PIND & (1<<PROG_WATER_TEMP))){//program water temp / delay value
					if(adc_value[TEMP_WATER] > 1015){
						if(!(SH_PIN & (1<<POT_SWITCH))){
							delay_value = adc_value[POT_VALUE] / 17;
							if(delay_value > 60) delay_value = 60;
							if(delay_value < 0) delay_value = 0;
						}							
						print_pgm_delay(delay_value);
						
					}else{
						if(!(SH_PIN & (1<<POT_SWITCH))){
							water_value = adc_value[POT_VALUE] / 10;
							if(water_value > 102) water_value = 102;
							if(water_value < 0) water_value = 0;
							
						}
						print_pgm_water_val(water_value);
					}
				}
				break;
			}
			case ERROR:{	// any error occured -> stop fan immeately!
				DISPLAY_PORT &= ~(1<<DISPLAY_LIGHT);
				off = false;
				adc_value[POT_VALUE] = 0;
				set_pwm(0);
				pwm_value_aux = 0;
				print_error();
				break;
			}
		}
		//rw data
		tx.pwm_freq = pwm_freq;
		rx.pwm_freq = pwm_freq;
		tx.time_value = delay_value;
		rx.time_value = delay_value;
		tx.water_value = water_value;
		rx.water_value = water_value;
		tx.cal_temperature = 0;
		rx.cal_temperature = 0;
		tx.cal_voltage = 0;
		rx.cal_voltage = 0;
		//ro data
		tx.water_temp = water_temp;
		tx.fet_temp = fet_temp;
		tx.vbat = vbat.integer * 100 + vbat.fraction;
		twi_tx_task();
		
	}
}
/***********************************************************************************************/
ISR(TIMER0_COMPA_vect){
	if(status == PROGRAM){
		if(led_count >= 100){
			SH_PORT |= (1<<LED);
			led_count = 0;
		}else{
			if(led_count == 50){
				SH_PORT &= ~(1<<LED);
			}
			led_count++;
		}
	}
	if(status == ERROR){
		if(led_count >= 100){
			SH_PORT |= (1<<LED);
			led_count = 0;
		}else{
			if(led_count == 50){
				SH_PORT &= ~(1<<LED);
			}
			led_count++;
		}
	}
}
/***********************************************************************************************/
ISR(TIMER2_OVF_vect){
	//t2_val++;
	//if(t2_val >= 64){
		adc_read_enable = true;
		//t2_val = 0;
		_t2_val++;
		if(_t2_val >= 2){
			second++;
			_t2_val = 0;
			if(second >= 3666){
				second = 0;
			}
		}
	//}		
}
/***********************************************************************************************/
