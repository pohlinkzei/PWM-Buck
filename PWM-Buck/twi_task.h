#ifndef TWI_TASK_H
#define TWI_TASK_H

#include <avr/io.h>
#include "twi_slave.h"

#define CRC_POLYNOME 0xAB

typedef struct rxdata {
	uint16_t pwm_freq;
	uint8_t cal_temperature;
	uint8_t cal_voltage;
	uint8_t water_value;
	uint8_t time_value;
}rx_t;

typedef struct txdata {
	uint16_t pwm_freq;
	uint8_t cal_temperature;
	uint8_t cal_voltage;
	uint8_t water_value;
	uint8_t time_value;
	uint16_t vbat;
	uint8_t water_temp;
	uint8_t fet_temp;
}tx_t;

rx_t *rx;// = (rx_t*) malloc(sizeof(rx_t));
tx_t *tx;// = (tx_t*) malloc(sizeof(tx_t));

 
uint8_t twi_rx_task(void);
uint8_t twi_tx_task(void);
uint8_t calculateCRC8(uint8_t crc, volatile uint8_t* data, uint8_t len);
uint8_t calculateID(char* name);

#endif