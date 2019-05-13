/*
 * uartlite_laser_sensor.h
 *
 *  Created on: 29 Jun 2018
 *      Author: smiric
 */

#ifndef SRC_UARTLITE_LASER_SENSOR_H_
#define SRC_UARTLITE_LASER_SENSOR_H_


#include "hw_cfg.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "xil_printf.h"
#include "pwm.h"
#include <stdlib.h>


// global variables
extern uint32_t uartlite_stat_reg;
extern char laser_sensor_str[30];


void uartlite_read_stat(void);
void uartlite_tx_enable(void);
void uartlite_rx_enable(void);
void uartlite_send_byte(uint8_t Data2Send);
bool uartlite_is_transmit_full(void);
bool uartlite_rx_has_data(void);
bool uartlite_is_transmit_empty(void);
uint8_t uartlite_read_byte(void);
void uartlite_send_command(char* command_string);
double uartlite_str2double(char pos_str[30]);
bool uartlite_rx_is_full(void);






#endif /* SRC_UARTLITE_LASER_SENSOR_H_ */
