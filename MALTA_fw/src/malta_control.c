/*
 * malta_control.c
 *
 *  Created on: 28 Mar 2018
 *      Author: smiric
 */

#include "malta_control.h"

volatile double theta = 0.0; //axial position
volatile double phi = 0.0; //circumferential position
volatile double id_m = 0.0;
volatile double iq_m = 0.0;
volatile double ud_ref = 0.0;
volatile double uq_ref = 0.0;
volatile double xabc[3] = {0.0};
//measurements
volatile double Udc = 0.0;
volatile double iabc_m[24] = {0.0};
volatile double uabc[24] = {0.0};
volatile double dabc[24] = {0.0};



// Execution of the current controller,
double malta_current_controller(double iref, double im)
{
	//PI controller gains
	double Kp = 40.0;
	double Ki = 450000.0;
	//Limits, anti-wind-up
	double PIout_min = 0;
	double PIout_max = 0.95 * Udc;
	//Controller accumulator variable
	static double acc = 0.0;
	//Integration time step
	double PIdt = 0.5 * (1.0 / (double)SW_FREQ);

	//PI controller
	double PIin = iref - im;
	double PIout = Kp*PIin + Ki * acc;
	if ( PIout > PIout_max ){
		PIout = PIout_max;
	} else if ( PIout < PIout_min ){
		PIout = PIout_min;
	} else{
		acc = acc + PIdt * PIin;
	}

	return PIout;
}

//Measurement update, this function updates the global variables
void measurement_update(void)
{
	Udc = measure_dc();
	//
	for(uint32_t k = 0; k < 9; k++){
		iabc_m[k] = measure_current1(k+1);
	}
}

//abc2dq transformation, it updates the id_m and iq_m values
void abc2dq(double i1, double i2, double i3){
	double K = 2.0 / 3.0;
	//id
	id_m = i1*cos(theta) + i2*cos(theta - 2.0*PI/3.0) + i3*cos(theta + 2.0*PI/3.0);
	id_m = K * id_m;
	//iq
	iq_m = i1*sin(theta) + i2*sin(theta - 2.0*PI/3.0) + i3*sin(theta + 2.0*PI/3.0);
	iq_m = K * iq_m;
}

//dq2abc transformation, it updates the global variable xabc
//it uses the global angle variable 'theta'
void dq2abc(double xd, double xq){
	//x1
	xabc[0] = xd*cos(theta) + xq*sin(theta);
	//x2
	xabc[1] = xd*cos(theta - 2.0*PI/3.0) + xq*sin(theta - 2.0*PI/3.0);
	//x3
	xabc[2] = xd*cos(theta + 2.0*PI/3.0) + xq*sin(theta + 2.0*PI/3.0);
}



//Generation of different theta waveforms
void theta_lin(double theta_freq)
{
	int nmax = (uint32_t)( 2 * SW_FREQ/theta_freq );
	static int acc_count = 0;
	acc_count += 1;
	//
	if (acc_count > nmax){
		acc_count = 0;
	}
	theta = 2*PI * ( (double)acc_count / (double)nmax );
}


void theta_sin(double theta_freq)
{
	int nmax = (int)( 0.5 * (double)SW_FREQ/theta_freq );
	static int acc_count = 0;
	double theta_aux = 0.0;
	//
	acc_count += 1;
	if (acc_count > nmax){
		acc_count = 0;
	}
	//
	theta_aux = 2.0 * PI * ( (double)acc_count / (double)nmax );
	theta = 0.5 * PI * sin( theta_aux );
}




// test current controller
void malta_test_current_controller(double iref)
{
	//Controller gains
	double Kp = 40.0;//40.0;
	double Ki = 450000.0;//450000.0;
	double Udc = measure_dc();
	//Limits, antiwindup
	double PIout_min = 0;
	double PIout_max = 0.95 * Udc;
	//Controller accumulator variable
	static double acc = 0.0;
	//Switching frequency is a global variable: fsw
	double PIdt = 0.5 * (1.0 / (double)SW_FREQ); //0.5 is because the interupt is raised every half a sw period
	//current measurement
	double im = measure_current1(1);

	//PI controller
	double PIin = iref - im;
	double PIout = Kp*PIin + Ki * acc;
	if ( PIout > PIout_max ){
		PIout = PIout_max;
	} else if ( PIout < PIout_min ){
		PIout = PIout_min;
	} else{
		acc = acc + PIdt * PIin;
	}

	//Write the duty cycle
	double d1 = PIout / Udc;

	set_modindex(d1, 1);
}



double current_ref_pulse(double pulse_freq, double iref_amplitude)
{
	uint32_t nmax = (uint32_t)( 0.5 * SW_FREQ/pulse_freq );
	static uint32_t acc_count = 0;
	acc_count += 1;

	if ( acc_count > 2*nmax ){
		acc_count = 0;
	}
	if (acc_count > nmax){
		return iref_amplitude;
	}
	return 0.0;
}

double sin_reference(double ref_freq, double ref_amp)
{
	uint32_t nmax = (uint32_t)( 2 * SW_FREQ/ref_freq );
	static int acc_count = 0;
	acc_count += 1;

	if (acc_count > nmax){
		acc_count = 0;
	}

	return ref_amp * sin( 2.0 * PI * (double)acc_count / (double)nmax ) + 0.6;
}


