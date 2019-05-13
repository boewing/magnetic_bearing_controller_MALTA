/*
 * PID.c
 *
 *  Created on: 30.04.2018
 *      Author: fabiand
 */

#include "PID.h" // general PID controller with Anti-Reset Windup

void PID_init(PID_data *PID,double Kp, double Ki, double Kd,double T_sample, double Norm_input, double Norm_output, double Sat_min, double Sat_max ) // D - part does not work+
{
 PID->gains.Kp = Kp;
 PID->gains.Ki = Ki;
 PID->gains.Kd = Kd;

 PID->T_samp = T_sample;

 PID->norm.input = Norm_input;
 PID->norm.output = Norm_output;

 PID->sat.min = Sat_min;
 PID->sat.max = Sat_max;

 PID->last_val = 0;
 PID->cum_sum  = 0;

}

void PID_reset(PID_data *PID)
{
	 PID->last_val = 0;
	 PID->cum_sum  = 0;
}

void PID_integration_step(PID_data *PID,double error)
{
  //Eulerforward integration step
  //PID->cum_sum += (PID->T_samp)*error;
   PID->cum_sum += error;

}



double PID_compute_action(PID_data *PID,double ref_val, double mea_val, double ff)
{
  double error;
  double u;

  //Error computation and normalisation
  error = (ref_val-mea_val)*PID->norm.output;
  //Compute output
  u = ff + (PID->gains.Kp)*error+(PID->gains.Ki)*(PID->cum_sum);

  // Saturation and anti reset windup
	if ( u >= PID->sat.max ){
		u = PID->sat.max;
	} else if ( u <= PID->sat.min ){
		u = PID->sat.min;
	} else{
		PID_integration_step(PID,error);
	}

  // Denormalisation
  u = u*(PID->norm.input);
  return u;
}

void PID_print_settings(PID_data *PID)
{
	printf("Gains:Kp,Ki,Kd\n");
	printf("%.2f,%.2f,%.2f\n",PID->gains.Kp,PID->gains.Ki,PID->gains.Kd);
	printf("Sampling Time: %f \n",PID->T_samp);
	printf("Cum Sum: %f \n",PID->cum_sum);
	printf("Norm In,Out: %.2f,%.2f \n",PID->norm.input,PID->norm.output);

}
