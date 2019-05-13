/*
 * pwm.c
 *
 *  Created on: 6 Feb 2018
 *      Author: smiric
 */
#include "pwm.h"


//Global variables:
//double udc = 4.0; //DC link voltage

// from mario
uint32_t global_var = 0;
volatile uint32_t global_var_for_isr = 0; //Volatile only needed when modified in ISR.


// Sets the switching frequency in Hz
// No return value
void set_sw_freq(uint32_t val)
{
	// Top value from the pwm register, 100MHz is the clock frequency
	volatile uint32_t* const top_register = (void*)PWM_TOP_ADDR;
	val = 2*val; // times 2 because the triangular signal is counted up and down
	if (val > MAX_SW_FREQ) {
		val = MAX_SW_FREQ;
		//xil_printf("Error: Switching frequency higher than 150kHz.\n");
	}
	if (val < MIN_SW_FREQ){
		val = MIN_SW_FREQ;
		//xil_printf("Error: Switching frequency lower than 40kHz.\n");
	}
	*top_register = ZYNQ_CLK_FREQ / val;
}

// Read swFreq from the register
uint32_t read_sw_freq(void)
{
	volatile uint32_t* const top_register = (void*)PWM_TOP_ADDR;
	uint32_t val;
	val = *top_register;
	val = ZYNQ_CLK_FREQ / val / 2;
	return val;
}


// Duty cycle
// Sets the compare value from duty cycle
// Duty cycle range: [0, 0.95]
// Duty cycle range: [0, 1] new try
// input: duty cycle, half-bridge number
void set_modindex(float modindex, uint32_t hb_val)
{
	// variables
	uint32_t top_val;
	// pwm compare value address
	volatile uint32_t* ptr_cmp_register = (uint32_t*)(PWM_CMP_ADDR);
	// pwm top value address
	volatile uint32_t* const top_register = (uint32_t*)PWM_TOP_ADDR;
	top_val = *top_register;
	// check the duty cycle range
	if (modindex < 0.0) {
		modindex = 0.0;
	}
	if (modindex > 0.95) {
		modindex = 0.95;
	}
	// Calculate the compare value value
	*(ptr_cmp_register+(hb_val-1)) = (uint32_t)(top_val * modindex);
	//printf("%lu\n",(ptr_cmp_register+(hb_val-1)));
}




void set_modindex_array(double *modindex_arr, uint32_t first_hb, uint32_t N_hb)
{
	// variables
	uint32_t top_val;
	static uint32_t temp_arr[24];
	// pwm compare value address
	uint32_t* ptr_cmp_register = (uint32_t*)(PWM_CMP_ADDR);
	// pwm top value address
	volatile uint32_t* const top_register = (uint32_t*)PWM_TOP_ADDR;
	top_val = *top_register;
	// check the duty cycle range
    //printf("Array\n");

	for (int i=0;i<N_hb;i++)
	{

	if (modindex_arr[i] < 0.0) {
		temp_arr[i] = 0;
	}
	else if (modindex_arr[i] > 0.95) {
		temp_arr[i] = (uint32_t)(0.95*top_val);
	}
	else {
		temp_arr[i] = (uint32_t)(modindex_arr[i]*top_val);
	}

	}
	// Calculate the compare value value

	memcpy((ptr_cmp_register+(first_hb-1)), temp_arr, sizeof(temp_arr[0])*N_hb); //Copy data
;
}




// Read from the PWM cmp register
float read_modindex(uint32_t hb_val)
{
	// variables
	uint32_t cmp_val, top_val;
	// Read the top and the cmp value from the register
	volatile uint32_t* const cmp_register = (void*)(PWM_CMP_ADDR + (hb_val-1)*4);
	cmp_val = *cmp_register;
	volatile uint32_t* const top_register = (void*)PWM_TOP_ADDR;
	top_val = *top_register;
	return (float)cmp_val / (float)top_val;
}

void enable_pwm(void)
{
	volatile uint32_t* const cfg_register = (void*)PWM_CONF_ADDR;
	uint32_t cfg = *cfg_register; //Read the register into a local variable

	cfg |= (uint32_t)(1<<0); //Set bit 0; shift the 1 zero times to the left
	//cfg = cfg | (uint32_t)(1<<0);
	//cfg = cfg | (uint32_t)(00000000000000000000000000000001);

	*cfg_register = cfg; //write the modified value back to the HW
}


void disable_pwm(void)
{
	volatile uint32_t* const cfg_register = (void*)PWM_CONF_ADDR;
	uint32_t cfg = *cfg_register; //Read the register into a local variable

	cfg &= (uint32_t)~(1<<0); //Set bit 0
	//cfg = cfg & (uint32_t~(1<<0);
	//cfg = cfg & (uint32_t)(11111111111111111111111111111110);
	*cfg_register = cfg; //write the modified value back to the HW
}


// Set the dead time in nano seconds
// min value 50ns
void set_dead_time(uint32_t val)
{
	volatile uint32_t* const dead_time_register = (void*)DEAD_TIME_ADDR;
	uint32_t aux_var;
	aux_var = *dead_time_register;
	aux_var &= (uint32_t)~(0x000007FF);
	if (val < 5) {
		*dead_time_register = (uint32_t)5 | aux_var;
	}
	else {
		*dead_time_register = (val / (uint32_t)10) | aux_var;
	}
}

// Read from the dead time register
uint32_t read_dead_time(void)
{
	volatile uint32_t* const cfg_register = (void*)DEAD_TIME_ADDR;
	uint32_t val;
	val = *cfg_register;
	val &= (uint32_t)(0x000007FF);
	val *= (uint32_t)(10); // convert to nano seconds
	return val;
}

// Enable the dead time, 32nd bit in config register
// Always first set the dead time and than enable it, i.e. first call set_dead_time(10)
void enable_dead_time(void)
{
	volatile uint32_t* const cfg_register = (void*)DEAD_TIME_ADDR;
	uint32_t cfg = *cfg_register; //Read the register value into a local variable

	cfg |= (uint32_t)(1<<31); // shift 1 onto the 32nd place and do the mask
	*cfg_register = cfg;
}

// Disable the dead time, 32nd bit in config register
void disable_dead_time(void)
{
	volatile uint32_t* const cfg_register = (void*)DEAD_TIME_ADDR;
	uint32_t cfg = *cfg_register; //Read the register value into a local variable

	cfg &= (uint32_t)~(1<<31); // shift 1 onto the 32nd place and do the mask
	*cfg_register = cfg;
	//printf("Dead Time DISABLED: Gate Signals are Low!\n");
}



// Let's assume this is an interrupt service routine:
void I_am_a_random_routine_maybe_an_interrupt_routine(void)
{
	global_var_for_isr += 1; //This has to be a volatile global variable!

}

// This is not an interrupt routine
void I_am_a_random_fnct_no_interrupt_routine(void)
{
	global_var += 1;
	static float test = 0.0; //Static: variable will retain value when leaving function, and can continuously be used.

	if(1)
		test += 1.0;
}



