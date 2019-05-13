/*
 * malta_configure.h
 *
 *  Created on: 9 Mar 2018
 *      Author: smiric
 */

#ifndef SRC_MALTA_CONFIGURE_H_
#define SRC_MALTA_CONFIGURE_H_

#include "malta_parameters.h"
#include "hw_cfg.h"
#include <stdint.h>
#include <stdio.h>
#include "xil_printf.h"
#include "pwm.h"
#include "adc.h"





//Base frequency is SW_FREQ*2
#define F_DOWN_CURRENT 10
#define F_DOWN_POSITION 10

//z-Position offset from measure_z();
#define Z_POS_OFFSET 0.0307
#define Z_POS_OFFSET_MET2 0.0332  //Offset for method 2


#define I_M_PI 0.318309886183791

void malta_initialise(void);
void configure_current_adc(void);
uint32_t read_current_adc_cfg(void);
void interupt1_togglebit(void);
void interupt2_togglebit(void);
void configure_position_adc(void);
uint32_t read_position_adc_cfg(void);
void configure_strb_downsample1(void);
uint32_t read_strb_downsample1_cfg(void);
void configure_strb_downsample2(void);
void configure_strb_downsample3(void);





#endif /* SRC_MALTA_CONFIGURE_H_ */
