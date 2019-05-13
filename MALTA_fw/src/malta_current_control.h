/*
 * malta_current_control.h
 *
 *  Created on: 30.04.2018
 *      Author: fabiand
 */

#ifndef SRC_MALTA_CURRENT_CONTROL_H_
#define SRC_MALTA_CURRENT_CONTROL_H_

#include "adc.h"
#include "hw_cfg.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "malta_parameters.h"
#include "2dq_transformation.h"
#include "filter_malta.h"
#include "malta_statemachine.h"
#include "PID.h"



// Definitions
#define NUM_PI_CUR_CONTROLLER 4

#define NUM_CURRENT_CONTROLLERS 2
#define M1_CUR_CONT 0
#define M2_CUR_CONT 1

/*#define MAX_RMS_CURRENT 3.5           //Max qd current to apply
#define NORM_OUT 0.0416666666666667   //Normalisation for output voltage
#define DENORM_OUT 24
#define I_NORM_IN 0.2    //Normalisation for current measurements
#define NORM_IN 5*/


//#define MAX_RMS_CURRENT 3.5           //Max qd current to apply
//#define DC_LINK_VOLTAGE 45
//#define VOLTAGE_SAT 22.5
#define DC_LINK_VOLTAGE 50
#define VOLTAGE_SAT 25
#define NORM_OUT 0.044444444444444   //Normalisation for output voltage
#define DENORM_OUT 22.5
#define I_NORM_IN 0.2    //Normalisation for current measurements
#define NORM_IN 5
#define PC_Voltage_FF 1
#define PC_CONTROLLER_SUPPRESSION 1
#define boost 1.7321 		//voltage increase due to sqrt(3) voltage margin

//#define NORM_OUT 1                    //Normalisation for output voltage
//#define I_NORM_IN 1    //Normalisation for current measurements

// Current controller constants
//100kHz Controller
//#define CUR_CONTROL_KP 146
//#define CUR_CONTROL_KI 6.1
//#define CUR_CONTROL_KD 0

//25kHz Controller for Udc 50V
//#define CUR_CONTROL_KP 2.04319232980541
//#define CUR_CONTROL_KI 0.0729602435417523
//#define CUR_CONTROL_KD 0

//22.22kHz Controller for Udc 24V and normalised system BW 1000rad/s
//#define CUR_CONTROL_KP 0.3622
//#define CUR_CONTROL_KI 0.0182
//#define CUR_CONTROL_KD 0

//22.22kHz Controller for Udc 24V and normalised system BW 5000rad/s
//#define CUR_CONTROL_KP 2.2695
//#define CUR_CONTROL_KI 0.0854
//#define CUR_CONTROL_KD 0


//22.22kHz Controller for Udc 24V and normalised system BW 10000rad/s
//#define CUR_CONTROL_KP 4.650585911311042
//#define CUR_CONTROL_KI 0.317734140585968
//#define CUR_CONTROL_KD 0

//20kHz Controller for Udc 24V and normalised system BW 10000rad/s
//#define CUR_CONTROL_KP 4.70966753953229
//#define CUR_CONTROL_KI 0.554687554143591
//#define CUR_CONTROL_KD 0

//20kHz Controller for Udc 24V and normalised system BW 5000rad/s
//#define CUR_CONTROL_KP 2.06869322590505
//#define CUR_CONTROL_KI 0.172037421401291
//#define CUR_CONTROL_KD 0

//20kHz Controller for Udc 24V and normalised system BW 5000rad/s | No Input delay
//#define CUR_CONTROL_KP 5.374259810909749
//#define CUR_CONTROL_KI 0.482766304924166
//#define CUR_CONTROL_KD 0

//20kHz Controller for Udc 24V and normalised system BW 3000rad/s
//#define CUR_CONTROL_KP 3.2897
//#define CUR_CONTROL_KI 0.1940
//#define CUR_CONTROL_KD 0

//20kHz Controller for Udc 45V and normalised system BW 3000rad/s
#define CUR_CONTROL_KP 1.7822
#define CUR_CONTROL_KI 0.0939
#define CUR_CONTROL_KD 0







//#define CUR_CONTROL_TSAMP (1/(2.0*(double)SW_FREQ))*(N_CURRENT_SAMP_SKIPPED+1)
#define CUR_CONTROL_TSAMP 0.00005


#define CUR_CONTROL_NORM_IN 1
#define CUR_CONTROL_NORM_OUT 1

#define CUR_CONTROL_SAT_MIN -1
#define CUR_CONTROL_SAT_MAX 1

// Decoupled dq current indexes
#define DQ_CUR_BEARING_INDEX (1)
#define DQ_CUR_DRIVING_INDEX (2)

#define M_PI_3 1.0471975511966

// Type defs for data structures of the malta current control

typedef struct
{
	mat_double currents;       //dq currents to apply
	double theta_z;          //dq angle for driving
	double theta_b;         //dq angle for bearing
}ref_data_type;

typedef struct
{
  double Udc;
  double Udc_mea;
  mat_double abc_currents;
  mat_double dq_ref_currents;
  double theta_z;
  double theta_b;
  mat_double dq_currents;
  mat_double dq_voltages;
  mat_double abc_voltages;
  mat_double abc_pwmduty;

  int coil_start_idx;
  filter_data_type input_filter; //Measurement signal filter
  dq_trans_data dq_data;

  PID_data PI_current_control[NUM_PI_CUR_CONTROLLER];
}curent_control_data_type;


void init_malta_current_control(int idx_c,int coil_start_index);
void reset_current_controller();
void measurement_update_cc(int idx_c);
void reference_update_cc(int idx_c,ref_data_type *ref_vals);
void current_control_action(int idx_c,mat_double *ref_vals);
void compute_pwm(int idx_c);
void apply_current_control(int idx_c);
void normalize_dq_current(int idx_c);
void denormalize_dq_voltages(int idx_c);

void execute_current_controller(ref_data_type *data_M1,ref_data_type *data_M2);

double read_current_cc(int N_cur, int Module);
double read_current_dq_cc(int N_cur, int Module);
double read_voltage_dq_cc(int N_voltage, int Module);
double read_ref_current_dq_cc(int N_cur,int Module);
double read_voltage_abc_cc(int N_voltage,int Module);
double read_voltage_dc_cc(int Module);

void M1_current_controller(void);
void M2_current_controller(void);

void limit_dq_current(double *current);

#endif /* SRC_MALTA_CURRENT_CONTROL_H_ */
