/*
 * malta_position_control.h
 *
 *  Created on: 02.05.2018
 *      Author: fabiand
 */

#ifndef SRC_MALTA_POSITION_CONTROL_H_
#define SRC_MALTA_POSITION_CONTROL_H_

#include "adc.h"
#include "matrix_operations.h"
#include "filter_malta.h"
#include "malta_current_control.h"
#include "medfilt.h"


#define N_STATES 10
#define N_INPUTS 5
#define N_OUTPUTS 10
#define N_MEASURE_VAR 5

#define Y1_IDX 1
#define Y2_IDX 3

#define PC_MEA_TSAMPLE 0.00005
#define PC_MEA_FSAMPLE 20000

//#define N_SKIP 4
//#define PC_TSAMPLE 0.0002
//#define PC_FSAMPLE 5000

#define N_SKIP 0
#define PC_TSAMPLE 0.00005
#define PC_FSAMPLE 20000

#define LB1 0.04225
#define LB2 0.04225
#define GRAV_ACC 9.81

//Conversion Gains for the Forces
//#define I_MACHINE_CONSTANT_DRIVE 0.227901422048536
#define I_MACHINE_CONSTANT_DRIVE 0.192579522636934
#define I_MACHINE_CONSTANT_BEARING 0.167785234899329
//#define I_MACHINE_CONSTANT_BEARING 0.147785234899329
//#define I_MACHINE_CONSTANT_BEARING_M1 I_MACHINE_CONSTANT_BEARING
//#define I_MACHINE_CONSTANT_BEARING_M2 I_MACHINE_CONSTANT_BEARING

#define I_MACHINE_CONSTANT_BEARING_M1 0.183284457478006
#define I_MACHINE_CONSTANT_BEARING_M2 0.202339039294241

#define PC_CURRENT_LIMIT 10  //Limits the maximum current. Higher current as 3.5A currently induces a noise that makes the system unstable
//#define MOVER_MASS 0.1738 //in kg
//#define MOVER_MASS 0.3476 //in kg
//#define MOVER_MASS 0.332408956939862 //in kg
#define MOVER_MASS 0.365 //in kg measured with scale
#define PC_FF 1
#define GRAVITATIONAL_ENABLE 1


//Defines for angles
#define M_ANG_45 0.785398163397448


//Theta Estimation Parameters
#define THETA_EST_A_M1 196.9
#define THETA_EST_B_M1 0.4184

#define THETA_EST_A_M2 -200.1
#define THETA_EST_B_M2 0.4286

#define THETA_EST_OFFSET_M1 0.384195838
#define THETA_EST_OFFSET_M2 0.313112068

#define M_PI_LP 209.43951023932
#define M_LP_PI 0.004774648292757




typedef struct
{
  mat_double mea_vec;         //Measurement vector [x1 y1 x2 y2 z]
  mat_double proc_mea_vec;    //Processed Measurement vector [x1 y1 x2 y2 z]
  mat_double last_mea_vec;    //Last measurement vector
  mat_double mea_div_vec;         //Measurement derivative vector [x1 y1 x2 y2 z]
  mat_double integration_vec;         //Error integration vector [ex1 ey1 ex2 ey2 ez]
  mat_double flux_vec;        //Flux measurement data [fl1 fl2]

  filter_data_type z_mea_filter; //Filter for z position
  filter_data_type mea_filter; //Measurement signal filter
  filter_data_type div_filter; //Derivative filter
  filter_data_type out_filter; //Derivative filter

  med_filter_data_type medfilt_M1fx;
  med_filter_data_type medfilt_M1fy;
  med_filter_data_type medfilt_M2fx;
  med_filter_data_type medfilt_M2fy;

  mat_double control_vec;     //Stacked measurement vector with derivation [y_c dy_c]
  mat_double temp_control_vec;
  mat_double temp_i_control_vec;
  mat_double ref_vec;         //Reference value vector
  mat_double force_vec;       //Forces to apply without steady state [fx1 fy1 fx2 fy2 fd]
  mat_double force_steady_vec;       //Forces to apply [fx1 fy1 fx2 fy2 fd]
  mat_double force_i_vec;     //Forces to apply [fx1 fy1 fx2 fy2 fd]

  ref_data_type M1dq;
  ref_data_type M2dq;

  mat_double gravitational_forces; //Steady state forces
  mat_double in_denorm_vec;
  mat_double in_norm_vec;
  mat_double out_norm_vec;
  mat_double out_denorm_vec;
  mat_double pos_norm_vec;
  mat_double pos_denorm_vec;
  mat_double state_norm_vec;
  mat_double state_denorm_vec;

  mat_double L;
  mat_double L_no_z;
  mat_double AmLC;
  mat_double AmLC_no_z;
  mat_double Bobs;
  mat_double Cobs;
  mat_double state_est;
  mat_double last_state_est;
  mat_double state_est_temp_yu;
  mat_double state_est_temp_x;
  mat_double mea_est;
  mat_double control_pdgain_state_mat;


  mat_double control_pdgain_mat;  //Gain matrix for control
  mat_double control_igain_mat;  //Gain matrix for control

  double rel_theta_z_M1;
  double rel_theta_z_M2;
  double z_laser;
  double laser_new;
  double z_disturbance;
  double u_dist_rej;
  double sigma_z;
  double sigma_x1;
  double sigma_y1;
  double sigma_x2;
  double sigma_y2;

  int cnt_control_downsample;


}pos_cont_data_type;


void init_malta_position_control(pos_cont_data_type *pos_controller);
void low_pass_malta_position_control(pos_cont_data_type *pos_controller);
void low_pass_div_malta_position_control(pos_cont_data_type *pos_controller);
void update_mea_malta_position_control(pos_cont_data_type *pos_controller);
void update_ref_malta_position_control(pos_cont_data_type *pos_controller,mat_double *ref);
void estimate_theta_mpc(pos_cont_data_type *pos_controller);
void state_estimation(pos_cont_data_type *pos_controller);
void compute_cont_vec_malta_position_control(pos_cont_data_type *pos_controller);
void compute_forces_malta_position_control(pos_cont_data_type *pos_controller);
void compute_dq_action_malta_position_control(pos_cont_data_type *pos_controller);
void compute_z_position(pos_cont_data_type *pos_controller);
void limit_dq_currents(double *current);
void read_laser(pos_cont_data_type *pos_controller);
void disturbance_estimation(pos_cont_data_type *pos_controller);
void output_feedback_error_integration(pos_cont_data_type *pos_controller);


#endif /* SRC_MALTA_POSITION_CONTROL_H_ */
