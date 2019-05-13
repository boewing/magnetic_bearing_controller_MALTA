/*
 * malta_control.h
 *
 *  Created on: 28 Mar 2018
 *      Author: smiric
 */

#ifndef SRC_MALTA_CONTROL_H_
#define SRC_MALTA_CONTROL_H_

#include "hw_cfg.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "xil_printf.h"
#include "pwm.h"
#include "adc.h"

double malta_current_controller(double iref, double im);
void malta_test_current_controller(double iref);
double current_ref_pulse(double pulse_freq, double iref_amplitude);
void measurement_update(void);
double sin_reference(double ref_freq, double ref_amp);
void dq2abc(double xd, double xq);
void abc2dq(double i1, double i2, double i3);
void theta_lin(double theta_freq);
void theta_lin1(double theta_freq);
void theta_sin(double theta_freq);

extern volatile double theta; //axial position
extern volatile double phi; //circumferential position
extern volatile double id_m;
extern volatile double iq_m;
extern volatile double ud_ref;
extern volatile double uq_ref;
extern volatile double xabc[3];
extern volatile double Udc;
extern volatile double iabc_m[24];
extern volatile double uabc[24];
extern volatile double dabc[24];


#endif /* SRC_MALTA_CONTROL_H_ */
