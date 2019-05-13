/*
 * PID.h
 *
 *  Created on: 30.04.2018
 *      Author: fabiand
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_


#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//************ Define PID data structure  *******************//
// PID Gains
typedef struct
{
  double Kp;
  double Ki;
  double Kd;
} PID_gains;

//*************** Normalisation Constants ******************//
typedef struct
{
  double input;
  double output;
} PID_norm;

//*************** Saturation Limits ******************//
typedef struct
{
  double min;
  double max;
} PID_sat;


//*************** PID data ******************//
typedef struct
{
  PID_gains gains;
  PID_norm norm ;
  PID_sat sat;
  double T_samp;      // Sample time
  double last_val;    // last measurement value for difference computation
  double cum_sum;     // cumulative sum for integral action
} PID_data;


#endif /* SRC_PID_H_ */

void PID_init(PID_data *PID,double Kp, double Ki, double Kd,double T_sample, double Norm_input, double Norm_output, double Sat_min, double Sat_max );
void PID_reset(PID_data *PID);


double PID_compute_action(PID_data *PID,double ref_val, double mea_val, double ff);
//double PID_AWR(PID_data *PID,double error);

void PID_integration_step(PID_data *PID,double error);

void PID_print_settings(PID_data *PID);


