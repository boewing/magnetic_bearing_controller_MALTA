/*
 * malta_configure.c
 *
 *  Created on: 9 Mar 2018
 *      Author: smiric
 */


#include "malta_configure.h"


void malta_initialise(void)
{
	//Set the switching frequency
	set_sw_freq(SW_FREQ);

	//Set the dead time in ns
	set_dead_time(60);

	//Downsamples the current measurement interupts
	configure_strb_downsample1();
	configure_strb_downsample2();
	configure_strb_downsample3();

	//Set the shunt reading constants
	curr_meas_const[0] = -0.00535;
	curr_meas_const[1] = -0.00535;
	curr_meas_const[2] = -0.00535;
	curr_meas_const[3] = -0.00535;
	curr_meas_const[4] = -0.00535;
	curr_meas_const[5] = -0.00535;
	curr_meas_const[6] = -0.00535;
	curr_meas_const[7] = -0.00535;
	curr_meas_const[8] = -0.00535;
	curr_meas_const[9] = -0.00535;
	curr_meas_const[10] = -0.00535;
	curr_meas_const[11] = -0.00535;
	curr_meas_const[12] = -0.00535;
	curr_meas_const[13] = -0.00535;
	curr_meas_const[14] = -0.00535;
	curr_meas_const[15] = -0.00535;
	curr_meas_const[16] = -0.00535;
	curr_meas_const[17] = -0.00535;
	curr_meas_const[18] = -0.00535;
	curr_meas_const[19] = -0.00535;
	curr_meas_const[20] = -0.00535;
	curr_meas_const[21] = -0.00535;
	curr_meas_const[22] = -0.00535;
	curr_meas_const[23] = -0.00535;
}

// current measurement adc's configuration
// if you want different configuration settings, change this function
void configure_current_adc(void)
{
	volatile uint32_t* const cfg_register = (void*)ADCCTRL_CONF_ADDR;
	uint32_t cfg = *cfg_register; //Read the register into a local variable

	// Set LSB to zero in order to take strobes from PWM generator.
	//cfg &= (uint32_t)~(1<<0); //Set bit 0 to 0; shift the 1 zero times to the left
	cfg = 1;
	*cfg_register = cfg; //write the modified value back
}

// read the current adc's configuration
uint32_t read_current_adc_cfg(void)
{
	volatile uint32_t* const cfg_register = (void*)ADCCTRL_CONF_ADDR;
	return *cfg_register;
}

// Interupt1, set and reset the bit
void interupt1_togglebit(void)
{
	volatile uint32_t* const cfg_register = (void*)ADCCTRL_CONF_ADDR;
	uint32_t cfg = *cfg_register; //Read the register into a local variable

	//set the bit
	cfg |= (uint32_t)(1<<2); //Set bit (2) to 1
	*cfg_register = cfg;

	//printf("upisao: %d;  ", (int)*cfg_register);

	//reset the bit
	cfg &= (uint32_t)~(1<<2); //Set bit (2) to 1
	//cfg = 0;
	*cfg_register = cfg;

	//printf("resetovao: %d;   ", (int)*cfg_register);
}


// Interupt1, set and reset the bit
void interupt2_togglebit(void)
{
	volatile uint32_t* const cfg_register = (void*)ADCCTRL_POSCONF_ADDR ;
	uint32_t cfg = *cfg_register; //Read the register into a local variable

	//set the bit
	cfg |= (uint32_t)(1<<2); //Set bit (2) to 1
	*cfg_register = cfg;

	//printf("upisao: %d;  ", (int)*cfg_register);

	//reset the bit
	cfg &= (uint32_t)~(1<<2); //Set bit (2) to 1
	//cfg = 0;
	*cfg_register = cfg;

	//printf("resetovao: %d;   ", (int)*cfg_register);
}

// position measurement adc's configuration
void configure_position_adc(void)
{
	volatile uint32_t* const cfg_register = (void*)ADCCTRL_POSCONF_ADDR;
	uint32_t cfg = *cfg_register; //Read the register into a local variable

	// Set LSB to zero in order to take strobes from PWM generator.
	//cfg &= (uint32_t)~(1<<0); //Set bit 0 to 0; shift the 1 zero times to the left

	// For the moment I do not want to raise the interupt 2, so I set the config to
	// 0000000000000000000000000000001
	//cfg &= (uint32_t)~(1<<1); //Set bit (1) to 0
	//cfg |= (uint32_t)(1<<0); //Set bit (0) to 1
	cfg = 1;

	*cfg_register = cfg; //write the modified value back
}

// read the position adc's configuration
uint32_t read_position_adc_cfg(void)
{
	volatile uint32_t* const cfg_register = (void*)ADCCTRL_POSCONF_ADDR;
	return *cfg_register;
}


// strobe down sample configuration for current measurement
void configure_strb_downsample1(void)
{
	volatile uint32_t* const cfg_register = (void*)ADCCTRL_STRBDOWN1_ADDR;

	*cfg_register = (uint32_t)(F_DOWN_CURRENT-1);
	// 0 - every pwm strobe is sampled
	// 1 - one pwm strobe is skipped
	// 2 - two pwm strobes are skipped, ... etc
}

// strobe down sample configuration for position measurement
void configure_strb_downsample2(void)
{
	volatile uint32_t* const cfg_register = (void*)ADCCTRL_STRBDOWN2_ADDR;

	*cfg_register = (uint32_t)(F_DOWN_POSITION-1);
	// 0 - every pwm strobe is sampled
	// 1 - one pwm strobe is skipped
	// 2 - two pwm strobes are skipped, ... etc
}

void configure_strb_downsample3(void)
{
	volatile uint32_t* const cfg_register = (void*)ADCCTRL_STRBDOWN3_ADDR;

	*cfg_register = (uint32_t)1;
	// 0 - every pwm strobe is sampled
	// 1 - one pwm strobe is skipped
	// 2 - two pwm strobes are skipped, ... etc
}


// read the position adc's configuration
uint32_t read_strb_downsample1_cfg(void)
{
	volatile uint32_t* const cfg_register = (void*)ADCCTRL_STRBDOWN1_ADDR;
	return *cfg_register;
}


void random_fnct_that_uses_global_variable(void)
{
	global_var = 0;
	global_var_for_isr = 0;

}



