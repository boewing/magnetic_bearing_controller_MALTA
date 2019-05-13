/*
 * pwm.h
 *
 *  Created on: 6 Feb 2018
 *      Author: smiric
 */

#ifndef SRC_PWM_H_
#define SRC_PWM_H_

#include "malta_configure.h"
#include "hw_cfg.h"
#include <stdint.h>
#include <stdio.h>
#include "xil_printf.h"

// Function Prototypes:
// switching frequency
void set_sw_freq(uint32_t val);
uint32_t read_sw_freq(void);
// modulation index
void set_modindex(float modindex, uint32_t hb_val);
void set_modindex_array(double *modindex_arr, uint32_t first_hb, uint32_t N_hb);
float read_modindex(uint32_t hb_val);
//
void enable_pwm(void);
void disable_pwm(void);
// dead time
void set_dead_time(uint32_t val);
uint32_t read_dead_time(void);
void enable_dead_time(void);
void disable_dead_time(void);

//Global variables
extern uint32_t fsw;
extern double udc;


//from mario
extern uint32_t global_var;
extern volatile uint32_t global_var_for_isr;


#endif /* SRC_PWM_H_ */
