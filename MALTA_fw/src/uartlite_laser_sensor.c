/*
 * uartlite_laser_sensor.c
 *
 *  Created on: 29 Jun 2018
 *      Author: smiric
 */

#include "uartlite_laser_sensor.h"


// Global variables
uint32_t uartlite_stat_reg = 0;
char laser_sensor_str[30];



/*
 * send and receive data bit
 */
void uartlite_tx_enable(void)
{
	volatile uint32_t* const cfg_register = (void*)UARTLITE_FLOW_ADDR;

	*cfg_register = (uint32_t)1;
}

void uartlite_rx_enable(void)
{
	volatile uint32_t* const cfg_register = (void*)UARTLITE_FLOW_ADDR;

	*cfg_register = (uint32_t)0;
}



// Read uartlite status register
void uartlite_read_stat(void)
{
	volatile uint32_t* const cfg_register = (void*)UARTLITE_STAT_ADDR;

	uartlite_stat_reg = *cfg_register;
}


// Check whether the transmit is full
bool uartlite_is_transmit_full(void)
{
	uartlite_read_stat();
	// mask uartlite_stat_reg for the bit 3
	uint32_t aux_var = uartlite_stat_reg & (uint32_t)(1<<3);

	if (aux_var) {
		//transmit FIFO is full
		return true;
	}

	// transmit FIFO is not full
	return false;
}

// Check whether the transmit is full
bool uartlite_is_transmit_empty(void)
{
	uartlite_read_stat();
	// mask uartlite_stat_reg for the bit 3
	uint32_t aux_var = uartlite_stat_reg & (uint32_t)(1<<2);

	if (aux_var) {
		//transmit FIFO is empty
		return true;
	}

	// transmit FIFO is not empty
	return false;
}

// Check whether RX FIFO has data
bool uartlite_rx_has_data(void)
{
	// update stat register var
	uartlite_read_stat();

	// mask uartlite_stat_reg for the bit 3
	uint32_t aux_var = uartlite_stat_reg & (uint32_t)(1<<0);

	if (aux_var) {
		//transmit FIFO is full
		return true;
	}

	// transmit FIFO is not full
	return false;
}

// Check whether RX FIFO is full
bool uartlite_rx_is_full(void)
{
	// update stat register var
	uartlite_read_stat();

	// mask uartlite_stat_reg for the bit 3
	uint32_t aux_var = uartlite_stat_reg & (uint32_t)(1<<1);

	if (aux_var) {
		//transmit FIFO is full
		return true;
	}

	// transmit FIFO is not full
	return false;
}

// Send data
void uartlite_send_byte(uint8_t Data2Send)
{
	volatile uint32_t* const Tx_register = (void*)UARTLITE_TXFIFO_ADDR;
	while (uartlite_is_transmit_full()) {};

	*Tx_register = (uint32_t)Data2Send;
}

 // Read data

uint8_t uartlite_read_byte(void)
{
	volatile uint32_t* const Rx_register = (void*)UARTLITE_RXFIFO_ADDR;
	// check status register if there is any data arrived
	//while(!uartlite_rx_has_data());

	return (uint8_t)(*Rx_register);
}



/*
 * This function sends commands to the BAumer sensor.
 * It takes as input PAYLOAD string (see datasheet of the sensor)
 */
void uartlite_send_command(char* command_string)
{
	uint32_t n_str = strlen(command_string);
	for ( uint32_t i=0; i<n_str; i++ ){
		uartlite_send_byte(command_string[i]);
	}
}



/*
 * This function gives double value from the sensor reading
 */
double uartlite_str2double(char pos_str[30])
{
	bool flag = false;
	int aux_i = 0;
	char aux_str[7] = "0000000";
	//
	for (int i=0;i<12;i++){
		if (flag) {
			aux_str[aux_i++] = pos_str[i];
		}
		//
		if ( pos_str[i] == ';' ){
			if (flag == false){
				flag = true;
			} else {
				flag = false;
			}
		}
	}
	// now, aux_str to double
	return atof(aux_str);
}









