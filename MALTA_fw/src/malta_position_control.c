/*
 * malta_position_control.c
 *
 *  Created on: 02.05.2018
 *      Author: fabiand
 */


#include "malta_position_control.h"
#include "position_control_gains.h"
#include "position_control_gains_state_feedback.h"
#include "position_control_integral_augmented.h"
#include "position_control_observer_matrizes.h"
#include "position_normalization.h"
#include "softstart.h"
#include "reference_generation.h"
#include "uartlite_laser_sensor.h"

//index
#define STATE_Z_IDX 2
#define STATE_DZ_IDX 7
#define FORCE_Z_IDX 4

//disturbance estimator
#define DISTURBANCE_ESTIMATOR 0

//laser filtering
#define LASER_FILTER 1
#define LASER_USE 0

#define INTEGRAL 1
#define INTEGRAL_DECREASE_FACTOR 12

//ramping variable
double p = 0;
double p_step = 1/20000;

//Transformation Constanst
const double DrivingAngleZeroRef = -M_ANG_45;
const double DrivingAnglePhaseMod = 1.497492498211136;

//Shutdown on sensor error sensitivity
const double shutdown_threshold = 0.002;

void init_malta_position_control(pos_cont_data_type *pos_controller)
{
	//Setup gain matrix
	if (PC_INTEGRAL_EN==1) {
	set_mat(&cont_pdgain_mat_PID[0][0], MatSize_Row(cont_pdgain_mat_PID), MatSize_Col(cont_pdgain_mat_PID),&(pos_controller->control_pdgain_mat));
	set_mat(&cont_igain_mat_PID[0][0], MatSize_Row(cont_igain_mat_PID), MatSize_Col(cont_igain_mat_PID),&(pos_controller->control_igain_mat));
	}
	else {
	set_mat(&cont_pdgain_mat_PD[0][0], MatSize_Row(cont_pdgain_mat_PD), MatSize_Col(cont_pdgain_mat_PD),&(pos_controller->control_pdgain_mat));
	set_mat(&KpL[0][0], MatSize_Row(KpL), MatSize_Col(KpL),&(pos_controller->control_pdgain_state_mat));
	Init_Matrix(&(pos_controller->control_igain_mat),MatSize_Row(cont_igain_mat_PID), MatSize_Col(cont_igain_mat_PID));
	}




	//Setup normalization vectors
	set_mat(&pc_input_denorm_vec[0][0], MatSize_Row(pc_input_denorm_vec), MatSize_Col(pc_input_denorm_vec),&(pos_controller->in_denorm_vec));
	set_mat(&pc_input_norm_vec[0][0], MatSize_Row(pc_input_norm_vec), MatSize_Col(pc_input_norm_vec),&(pos_controller->in_norm_vec));
	set_mat(&pc_output_norm_vec[0][0], MatSize_Row(pc_output_norm_vec), MatSize_Col(pc_output_norm_vec),&(pos_controller->out_norm_vec));
	set_mat(&pc_output_denorm_vec[0][0], MatSize_Row(pc_output_denorm_vec), MatSize_Col(pc_output_denorm_vec),&(pos_controller->out_denorm_vec));
	set_mat(&pc_pos_norm_vec[0][0], MatSize_Row(pc_pos_norm_vec), MatSize_Col(pc_pos_norm_vec),&(pos_controller->pos_norm_vec));
	set_mat(&pc_pos_denorm_vec[0][0], MatSize_Row(pc_pos_denorm_vec), MatSize_Col(pc_pos_denorm_vec),&(pos_controller->pos_denorm_vec));
	set_mat(&pc_state_norm_vec[0][0], MatSize_Row(pc_state_norm_vec), MatSize_Col(pc_state_norm_vec),&(pos_controller->state_norm_vec));
	set_mat(&pc_state_denorm_vec[0][0], MatSize_Row(pc_state_denorm_vec), MatSize_Col(pc_state_denorm_vec),&(pos_controller->state_denorm_vec));

	//Observer setup
	set_mat(&AmLC[0][0], MatSize_Row(AmLC), MatSize_Col(AmLC),&(pos_controller->AmLC));
	set_mat(&AmLC_no_z[0][0], MatSize_Row(AmLC_no_z), MatSize_Col(AmLC_no_z),&(pos_controller->AmLC_no_z));
	set_mat(&Bobs[0][0], MatSize_Row(Bobs), MatSize_Col(Bobs),&(pos_controller->Bobs));
	set_mat(&Cobs[0][0], MatSize_Row(Cobs), MatSize_Col(Cobs),&(pos_controller->Cobs));
	set_mat(&L[0][0], MatSize_Row(L), MatSize_Col(L),&(pos_controller->L));
	set_mat(&L_no_z[0][0], MatSize_Row(L_no_z), MatSize_Col(L_no_z),&(pos_controller->L_no_z));
	Init_Matrix(&(pos_controller->state_est),N_STATES, 1);
	Init_Matrix(&(pos_controller->last_state_est),N_STATES, 1);
	Init_Matrix(&(pos_controller->state_est_temp_yu),N_MEASURE_VAR, 1);
	Init_Matrix(&(pos_controller->state_est_temp_x),N_STATES, 1);
	Init_Matrix(&(pos_controller->mea_est),N_OUTPUTS, 1);


	if (PC_BEARING_EN){
	set_mat(&pc_steady_state[0][0], MatSize_Row(pc_steady_state), MatSize_Col(pc_steady_state),&(pos_controller->gravitational_forces));}
	else{
	Init_Matrix(&(pos_controller->gravitational_forces),N_INPUTS,1);}

	Init_Matrix(&(pos_controller->mea_vec),N_MEASURE_VAR, 1);
	Init_Matrix(&(pos_controller->proc_mea_vec),N_MEASURE_VAR, 1);
	Init_Matrix(&(pos_controller->mea_div_vec),N_MEASURE_VAR, 1);
	Init_Matrix(&(pos_controller->last_mea_vec),N_MEASURE_VAR, 1);
	Init_Matrix(&(pos_controller->integration_vec),N_MEASURE_VAR, 1);
	Init_Matrix(&(pos_controller->flux_vec),2, 1);


	Init_Matrix(&(pos_controller->control_vec),N_OUTPUTS, 1);
	Init_Matrix(&(pos_controller->temp_control_vec),N_OUTPUTS, 1);
	Init_Matrix(&(pos_controller->temp_i_control_vec),N_MEASURE_VAR, 1);

	Init_Matrix(&(pos_controller->ref_vec),N_OUTPUTS, 1);
	Init_Matrix(&(pos_controller->force_vec),N_INPUTS, 1);
	Init_Matrix(&(pos_controller->force_steady_vec),N_INPUTS, 1);
	Init_Matrix(&(pos_controller->force_i_vec),N_INPUTS, 1);


	// Filter initialisation
	//init_filter(&(pos_controller->z_mea_filter),1,LP_FILTER_A_150Hz_20kHz,LP_FILTER_B_150Hz_20kHz);
	init_filter(&(pos_controller->z_mea_filter),1,LP_FILTER_A_150Hz_20kHz,LP_FILTER_B_150Hz_20kHz);
	//init_filter(&(pos_controller->mea_filter),N_MEASURE_VAR,LP_FILTER_A_350Hz_20kHz,LP_FILTER_B_350Hz_20kHz);
	//init_filter(&(pos_controller->div_filter),N_MEASURE_VAR,LP_FILTER_A_350Hz_20kHz,LP_FILTER_B_350Hz_20kHz);
	init_filter(&(pos_controller->mea_filter),N_MEASURE_VAR,LP_FILTER_A_150Hz_20kHz,LP_FILTER_B_150Hz_20kHz);
	init_filter(&(pos_controller->div_filter),N_MEASURE_VAR,LP_FILTER_A_150Hz_20kHz,LP_FILTER_B_150Hz_20kHz);
	init_filter(&(pos_controller->out_filter),N_INPUTS,LP_FILTER_A_300Hz_20kHz,LP_FILTER_B_300Hz_20kHz);



	Init_Matrix(&(pos_controller->M1dq.currents),4, 1);
	Init_Matrix(&(pos_controller->M2dq.currents),4, 1);
	pos_controller->M1dq.theta_z = 0;
	pos_controller->M1dq.theta_b = 0;
	pos_controller->M2dq.theta_z = 0;
	pos_controller->M2dq.theta_b = 0;


	pos_controller->cnt_control_downsample = 0;

	pos_controller->z_disturbance = 0;
	pos_controller->z_laser = 0;
	pos_controller->u_dist_rej = 0;
	pos_controller->sigma_z = 0;
	pos_controller->sigma_x1 = 0;
	pos_controller->sigma_y1 = 0;
	pos_controller->sigma_x2 = 0;
	pos_controller->sigma_y2 = 0;

	init_medfilt_3 (&(pos_controller->medfilt_M1fx));
	init_medfilt_3 (&(pos_controller->medfilt_M1fy));
	init_medfilt_3 (&(pos_controller->medfilt_M2fx));
	init_medfilt_3 (&(pos_controller->medfilt_M2fy));

}



void estimate_theta_mpc(pos_cont_data_type *pos_controller)
{
	// Reconstruction with steelrod extension
	//Compute Theta M1 from raw z position
	//pos_controller->M1dq.theta_z = 155.1*pos_controller->mea_vec.data[4]-5.22;
	//Compute Theta M2 from raw z position
	//pos_controller->M2dq.theta_z = -188.6*pos_controller->mea_vec.data[4]+7.256;

	// Reconstruction with 3D printed extension
	//Compute Theta M1 from raw z position
		pos_controller->M1dq.theta_z = 205.4*(pos_controller->state_est.data[2]+0.03613) - 7.186;
		//Compute Theta M2 from raw z position
		//pos_controller->M2dq.theta_z = -199*pos_controller->mea_vec.data[4]+7.915;
		//Compute Theta M2 from raw z position aligned with inertial frame
		pos_controller->M2dq.theta_z = 201.1*(pos_controller->state_est.data[2]+0.03613) - 5.896;


    // Exact reconstruction
	//	double temp = pos_controller->mea_vec.data[4];
	//	pos_controller->M1dq.theta_z = 7.319*sin(28.14*temp+5.298)+0.9123*sin(367.1*temp+1.025)+1.006*sin(374.7*temp+10.11);
	//	pos_controller->M2dq.theta_z = 8.031*sin(25.4*temp+2.132) + 0.08998*sin(500.4*temp-15.34);

}



void update_mea_malta_position_control(pos_cont_data_type *pos_controller)
{

	//static int temp = 1;
	//printf("temp : %d \n",temp);

	pos_controller->mea_vec.data[0] = measure_m1_x();          // x1 measurement
	pos_controller->mea_vec.data[1] = measure_m1_y();          // y1 measurement
	pos_controller->mea_vec.data[2] = measure_m2_x();          // x2 measurement
	pos_controller->mea_vec.data[3] = measure_m2_y();          // y2 measurement

	read_flux_V9(&(pos_controller->flux_vec));
	compute_z_position(pos_controller);
	//pos_controller->mea_vec.data[4] = measure_z();  // z measurement

	//Laser measurement
	read_laser(pos_controller);

	 estimate_theta_mpc(pos_controller);

	//Apply offset to z position
	//pos_controller->mea_vec.data[4] = pos_controller->mea_vec.data[4]-0.03613;
	//pos_controller->z_laser = pos_controller->z_laser - 0.03613;

	//printf("z_laser: %f \n", pos_controller->z_laser);
	//printf("Set mea Vec\n");
	//printMat(&pos_controller->mea_vec);

	if (EN_MEA_FILTER) {
	 first_ord_filter(&(pos_controller->mea_vec), &(pos_controller->mea_filter));}
		    else {
			mat_copy(&(pos_controller->mea_filter.last_out_vec),&(pos_controller->mea_vec));}

			mat_copy(&(pos_controller->proc_mea_vec),&(pos_controller->mea_filter.last_out_vec));

			if (EN_Z_MEA_FILTER) {
			first_ord_filter_val((pos_controller->mea_vec.data[4]), &(pos_controller->z_mea_filter));}
			else {
			pos_controller->z_mea_filter.last_out_vec.data[0]=pos_controller->mea_vec.data[4];}

			pos_controller->proc_mea_vec.data[4] = pos_controller->z_mea_filter.last_out_vec.data[0];



	//printf("Processed mea Vec\n");
	//printMat(&pos_controller->proc_mea_vec);


    /*if ((temp < 2)==1) {
	pos_controller->proc_mea_vec.data[0] = 0.0005;
	pos_controller->proc_mea_vec.data[1]	= 0.0005;
	pos_controller->proc_mea_vec.data[2] = 0.0005;
	pos_controller->proc_mea_vec.data[3] = 0.0005;
	pos_controller->proc_mea_vec.data[4] = 0.003;
    temp = 3;}
    else{
	pos_controller->proc_mea_vec.data[0] = 0.0004;
	pos_controller->proc_mea_vec.data[1]	= 0.0004;
	pos_controller->proc_mea_vec.data[2] = 0.0004;
	pos_controller->proc_mea_vec.data[3] = 0.0004;
	pos_controller->proc_mea_vec.data[4] = 0.002;
    temp = 1;}
    printf("tata: %d\n",temp);



    printf("Set mea Vec\n");
    printMat(&pos_controller->mea_vec);*/


	// Position estimation via observer
	output_feedback_error_integration(pos_controller);
	state_estimation(pos_controller);
}

void sign_integrator(const double k_I, const double Ts,
		double *ref, double *measurement, double *sigma, double *force) {
	double up =
			k_I * Ts
					* (*ref
							- *measurement);
	if ((up > 0 && *sigma < 0)
			|| (up < 0 && *sigma > 0)) {
		*sigma = *sigma + INTEGRAL_DECREASE_FACTOR * up;
	} else {
		*sigma = *sigma + up;
	}
	*force += *sigma;
}

void output_feedback_error_integration(pos_cont_data_type *pos_controller){

	if(INTEGRAL == 0){
		return;
	}
	//only for state estimation
	// integrated control error
//	const double k_I_z = -2.5e5; // integrational parameter for 45 V
//	const double k_I_x1 = -2.5e5;
//	const double k_I_y1 = -2.5e5;
//	const double k_I_x2 = -2.5e5;
//	const double k_I_y2 = -2.5e5;

	const double k_I_z = -2e5; // integrational parameter for 50 V
	const double k_I_x1 = -5e4;
	const double k_I_y1 = -5e4;
	const double k_I_x2 = -5e4;
	const double k_I_y2 = -5e4;

	const double Ts = 5e-5; // [sec] sampling time

	if(feed_forward_only() != 1){
		sign_integrator(k_I_z, Ts,&(pos_controller->ref_vec.data[2]), &(pos_controller->mea_vec.data[4]), &(pos_controller->sigma_z), &(pos_controller->force_vec.data[4]));
	}else{
		pos_controller->sigma_z = 0;
//		pos_controller->sigma_x1 = 0;
//		pos_controller->sigma_y1 = 0;
//		pos_controller->sigma_x2 = 0;
//		pos_controller->sigma_y2 = 0;
	}

	sign_integrator(k_I_x1, Ts,&(pos_controller->ref_vec.data[4]), &(pos_controller->mea_vec.data[0]), &(pos_controller->sigma_x1), &(pos_controller->force_vec.data[0]));
//	pos_controller->sigma_x1 = pos_controller->sigma_x1 + k_I_x1*Ts*(0 - pos_controller->mea_vec.data[0]);
//	pos_controller->force_vec.data[0] += pos_controller->sigma_x1;
//
	sign_integrator(k_I_y1, Ts,&(pos_controller->ref_vec.data[4]), &(pos_controller->mea_vec.data[1]), &(pos_controller->sigma_y1), &(pos_controller->force_vec.data[1]));
//	pos_controller->sigma_y1 = pos_controller->sigma_y1 + k_I_y1*Ts*(0 - pos_controller->mea_vec.data[1]);
//	pos_controller->force_vec.data[1] += pos_controller->sigma_y1;

	sign_integrator(k_I_x2, Ts,&(pos_controller->ref_vec.data[4]), &(pos_controller->mea_vec.data[2]), &(pos_controller->sigma_x2), &(pos_controller->force_vec.data[2]));
//	pos_controller->sigma_x2 = pos_controller->sigma_x2 + k_I_x2*Ts*(0 - pos_controller->mea_vec.data[2]);
//	pos_controller->force_vec.data[2] += pos_controller->sigma_x2;

	sign_integrator(k_I_y2, Ts,&(pos_controller->ref_vec.data[4]), &(pos_controller->mea_vec.data[3]), &(pos_controller->sigma_y2), &(pos_controller->force_vec.data[3]));
//	pos_controller->sigma_y2 = pos_controller->sigma_y2 + k_I_y2*Ts*(0 - pos_controller->mea_vec.data[3]);
//	pos_controller->force_vec.data[3] += pos_controller->sigma_y2;

}

void read_laser(pos_cont_data_type *pos_controller)
{
	static int state = -2;
	static char position_uart[6];
	char in;
	static double z_old = 0;
	double z;

	while(uartlite_rx_has_data()){
		in = uartlite_read_byte();
		//printf("%c",in);

		if(state == -2){
			if(in == 'S')
				state = -1;
		}else if(state == -1){
			if(in == ';')
				state = 0;
			else
				state = -2;
		}else if(state >= 0 && state <=6){
			if(in == ';'){
				state = 7;
			}else{
				position_uart[state] = in;
				state++;
			}
		}
		if(state == 7){
			//position_uart[state] = (uint8_t)in;
			//printf("read_str: %s  \n", position_uart);
			z = atof(position_uart);
			z = -MM_TO_M*(z-100);

			//check if position is realistic: no greater changes than 3mm between two measurements and not out of bounds
			if(!(LASER_FILTER) || (fabs(z_old - z) < 0.0015 && z <= 0.022 && z >= -0.020)){
				pos_controller->laser_new = 1.0;
				pos_controller->z_laser = z;
			}else{
				//printf("wrong laser value: %s", position_uart);
			}
			z_old = z;
			state = -2;
			//printf("before %f\n",pos_controller->z_laser);
			break;
		}
	}
}

void disturbance_estimation(pos_cont_data_type *pos_controller){
	//ONLY available with flux!
	//use these variables
	// the following variables are in SI units:
//	pos_controller->state_est.data[STATE_Z_IDX];
//	pos_controller->state_est.data[STATE_DZ_IDX];
//	pos_controller->force_vec.data[FORCE_Z_IDX];

	static double internal_state = 0;  // filter
	double d_hat;				// input to filter
	double b;					// = normalized_B_red*x_tilde_red

	// note: constants are in SI units
	//const double norm_B_scalar = 1.582899794951726e2;
	const double norm_B_scalar = 4.4725e3;
	//const double norm_B_scalar = 1e4;

	b = norm_B_scalar * (pos_controller->mea_vec.data[4] - pos_controller->state_est.data[STATE_Z_IDX]);
	d_hat = (pos_controller->force_vec.data[FORCE_Z_IDX] - pos_controller->u_dist_rej) + b;

	// low pass filter parameters // x(k+1) = delta x(k) + epsilon d_hat(k); d_tilde(k) = kappa x(k)
	const double delta 		= 0.995012479192682;
	const double epsilon 	= 0.0625;
	const double kappa 		= 0.079800332917083;

	internal_state = delta*internal_state + epsilon * d_hat;
	pos_controller->z_disturbance = kappa*internal_state; // d_tilde


	pos_controller->u_dist_rej = pos_controller->force_vec.data[FORCE_Z_IDX] - pos_controller->z_disturbance;


	//and write in the end to this
//	pos_controller->z_disturbance;
//	pos_controller->u_dist_rej;
}


void state_estimation(pos_cont_data_type *pos_controller)
{
	//***************State estimation via observer*****************************
	// Testdata
	/*mat_double test_mat;
	Init_Matrix(&test_mat,5, 1);
	test_mat.data[0] = 0.001;
	test_mat.data[1] = 0.002;
	test_mat.data[2] = 0.003;
	test_mat.data[3] = 0.004;
	test_mat.data[4] = 0.005;

	mat_double test_u;
	Init_Matrix(&test_u,5, 1);
	test_u.data[0] = 1;
	test_u.data[1] = 2;
	test_u.data[2] = 3;
	test_u.data[3] = 4;
	test_u.data[4] = 5;

	mat_double test_xo;
	Init_Matrix(&test_xo,10, 1);
	test_xo.data[0] = 0.001;
	test_xo.data[1] = 0.002;
	test_xo.data[2] = 0.003;
	test_xo.data[3] = 0.004;
	test_xo.data[4] = 0.005;
	test_xo.data[5] = 0.001;
	test_xo.data[6] = 0.002;
	test_xo.data[7] = 0.003;
	test_xo.data[8] = 0.004;
	test_xo.data[9] = 0.005;

	mat_copy(&(pos_controller->mea_vec),&(test_mat));
	mat_copy(&(pos_controller->force_vec),&(test_u));
	mat_copy(&(pos_controller->last_state_est),&(test_xo));*/


    if (!LASER_USE || pos_controller->laser_new == 1.0){

    	if(LASER_USE){
    		pos_controller->mea_vec.data[4] = pos_controller->z_laser;
    	}
    	matmul(&(pos_controller->L),&(pos_controller->mea_vec),&(pos_controller->state_est));
    	matmul(&(pos_controller->Bobs),&(pos_controller->force_vec),&(pos_controller->state_est_temp_x));
    	matadd(&(pos_controller->state_est),&(pos_controller->state_est_temp_x),&(pos_controller->state_est_temp_x));

    	matmul(&(pos_controller->AmLC),&(pos_controller->last_state_est),&(pos_controller->state_est));
    	pos_controller->laser_new = 0.0;
    }else{
    	matmul(&(pos_controller->L_no_z),&(pos_controller->mea_vec),&(pos_controller->state_est));
    	matmul(&(pos_controller->Bobs),&(pos_controller->force_vec),&(pos_controller->state_est_temp_x));
    	matadd(&(pos_controller->state_est),&(pos_controller->state_est_temp_x),&(pos_controller->state_est_temp_x));

    	matmul(&(pos_controller->AmLC_no_z),&(pos_controller->last_state_est),&(pos_controller->state_est));
    }

    /*printf("state dynamics:\n");
    printMat(&(pos_controller->state_est));
    */
    //Add dynamics to correction
    matadd(&(pos_controller->state_est),&(pos_controller->state_est_temp_x),&(pos_controller->state_est));
    /*printf("total dynamics:\n");
    printMat(&(pos_controller->state_est));
    */
    // Update last estimation
    mat_copy(&(pos_controller->last_state_est),&(pos_controller->state_est));
    //Compute measurement estimate
    //matmul(&(pos_controller->Cobs),&(pos_controller->state_est),&(pos_controller->mea_est));
    /*printf("Estimated measurement:\n");
    printMat(&(pos_controller->mea_est));
    printf("Denormalization vector:\n");
    printMat(&(pos_controller->out_denorm_vec));*/
    //matmul_element(&(pos_controller->mea_est),&(pos_controller->out_denorm_vec),&(pos_controller->mea_est));
    /*printf("Denormalized Estimated mea:\n");
       printMat(&(pos_controller->mea_est));*/
    //Denormalize State
    //matmul_element(&(pos_controller->state_est),&(pos_controller->state_denorm_vec),&(pos_controller->state_est));
    /*printf("Denormalized States:\n");
    printMat(&(pos_controller->state_est));*/
}

void update_ref_malta_position_control(pos_cont_data_type *pos_controller,mat_double *ref)
{
	//Copies ref into the 5 first positions since the reference vector for control is
	//[rx1 ry1 rx2 ry2 z | 0 0 0 0 0]'
	mat_copy(&(pos_controller->ref_vec),ref);

}


void compute_cont_vec_malta_position_control(pos_cont_data_type *pos_controller)
{
	//Computes derivative of the measured positions from a previous sample of the positions and the current one.
	matsub(&(pos_controller->proc_mea_vec),&(pos_controller->last_mea_vec),&(pos_controller->last_mea_vec));
	matmul_val(&(pos_controller->last_mea_vec),PC_FSAMPLE,&(pos_controller->last_mea_vec));			//is a 5x1 vector with all position derivatives


	if (EN_DIV_FILTER) {
	first_ord_filter(&(pos_controller->last_mea_vec), &(pos_controller->div_filter));}
	else {
	mat_copy(&(pos_controller->div_filter.last_out_vec),&(pos_controller->last_mea_vec));}
	//Constructs control vector
	mat_cat_1dim(&(pos_controller->proc_mea_vec),&(pos_controller->div_filter.last_out_vec),&(pos_controller->control_vec));

	//Stores current measurement vector for next execution
	mat_copy(&(pos_controller->last_mea_vec),&(pos_controller->proc_mea_vec));

	//printf("Measurement Vector\n");
	//printMat(&(pos_controller->proc_mea_vec));
	//printf("Control Vector\n");
	//printMat(&(pos_controller->control_vec));


}

void dynamic_gravitational_compensation(pos_cont_data_type* pos_controller) {
	//calculate gravitational compensation forces
	double z;
	//z = pos_controller->state_est.data[4];
	//z = pos_controller->mea_vec.data[4];
	z = pos_controller->ref_vec.data[2];

	//z = pos_controller->z_laser;
	pos_controller->gravitational_forces.data[Y1_IDX] = MOVER_MASS * GRAV_ACC / (LB1 + LB2) * (LB2 - z);
	pos_controller->gravitational_forces.data[Y2_IDX] = MOVER_MASS * GRAV_ACC / (LB1 + LB2) * (LB1 + z);
	//printf("grav_comp %.2f, %.2f, %.6f \n",pos_controller->gravitational_forces.data[Y1_IDX], pos_controller->gravitational_forces.data[Y2_IDX], z);
}

void compute_forces_malta_position_control(pos_cont_data_type *pos_controller)
{
	//compute_cont_vec_malta_position_control(pos_controller);
	//Compute error
	if (PC_STATE_CONTROL == 1) {
	matsub(&(pos_controller->ref_vec),&(pos_controller->state_est),&(pos_controller->temp_control_vec));  //#Observer
	} else {
	matsub(&(pos_controller->ref_vec),&(pos_controller->control_vec),&(pos_controller->temp_control_vec)); //error vector = reference - state derived from derivative
	}

	//SHUTDOWN if unstable
	//printf("%.4f\n",pos_controller->temp_control_vec.data[4]);
	//if(pos_controller->temp_control_vec.data[4] < -shutdown_threshold || pos_controller->temp_control_vec.data[4] > shutdown_threshold){
	//	cstop(NULL, NULL);
	//	printf("stopped on general sensor error");
	//}

	//printf("Reference Vector\n");
	//printMat(&(pos_controller->ref_vec));
	//printf("Error Vector\n");
	//printMat(&(pos_controller->temp_control_vec));

   //Exclude different positions
   if (PC_Z_POS_EN == 0)
   {
	   pos_controller->temp_control_vec.data[4] = 0;
	   pos_controller->temp_control_vec.data[9] = 0;
   }

   if (PC_X_POS_EN == 0 || PC_BEARING_M1_EN == 0 || PC_BEARING_M2_EN == 0)
   {
	   if (PC_BEARING_M1_EN == 0) {
	   pos_controller->temp_control_vec.data[0] = 0;
	   pos_controller->temp_control_vec.data[5] = 0;}
	   if (PC_BEARING_M2_EN == 0) {
	   pos_controller->temp_control_vec.data[2] = 0;
	   pos_controller->temp_control_vec.data[7] = 0;}
   }

   if (PC_Y_POS_EN == 0 || PC_BEARING_M1_EN == 0 || PC_BEARING_M2_EN == 0)
   {
	   if (PC_BEARING_M1_EN == 0) {
	   pos_controller->temp_control_vec.data[1] = 0;
	   pos_controller->temp_control_vec.data[6] = 0;}
	   if (PC_BEARING_M2_EN == 0) {
	   pos_controller->temp_control_vec.data[3] = 0;
	   pos_controller->temp_control_vec.data[8] = 0;}
   }


   //printf("Error\n");
   //printMat(&(pos_controller->temp_control_vec));

   if (PC_INTEGRAL_EN){
	   // Extract only position error
	   vec_copy_part(&(pos_controller->temp_i_control_vec),&(pos_controller->temp_control_vec),0,4);
	   //Integration step for the error
	   matadd(&(pos_controller->temp_i_control_vec),&(pos_controller->integration_vec),&(pos_controller->integration_vec));
	   //printf("Integration Mat\n");
	   //printMat(&(pos_controller->integration_vec));
	   //Integral normalization
	   matmul_element(&(pos_controller->integration_vec),&(pos_controller->pos_norm_vec),&(pos_controller->temp_i_control_vec));
	   //Compute Integral action
	   matmul(&(pos_controller->control_igain_mat),&(pos_controller->temp_i_control_vec),&(pos_controller->force_i_vec));
	   matmul_val(&(pos_controller->force_i_vec),(double)(-1*PC_TSAMPLE),&(pos_controller->force_i_vec));
	   // Normalisation of the measuerement values
	   matmul_element(&(pos_controller->control_vec),&(pos_controller->out_norm_vec),&(pos_controller->temp_control_vec));
	   matmul_val(&(pos_controller->temp_control_vec),-1,&(pos_controller->temp_control_vec));
   }
   else {
	   // Normalisation of the measuerement values
	   if (PC_STATE_CONTROL == 1) {
		matmul_element(&(pos_controller->temp_control_vec),&(pos_controller->state_norm_vec),&(pos_controller->temp_control_vec));} //#Obs
	   else {
	    matmul_element(&(pos_controller->temp_control_vec),&(pos_controller->out_norm_vec),&(pos_controller->temp_control_vec));
	   }
   }
   //printf("Normalized Error\n");
   //printMat(&(pos_controller->temp_control_vec));
   // Control action computation
   //printf("force_vec is overwritten \n");
   if (PC_STATE_CONTROL == 1) {
   matmul(&(pos_controller->control_pdgain_state_mat),&(pos_controller->temp_control_vec),&(pos_controller->force_vec));} //#Obs
   else {matmul(&(pos_controller->control_pdgain_mat),&(pos_controller->temp_control_vec),&(pos_controller->force_vec));}
   //printf("Control Action\n");
   //printMat(&(pos_controller->force_vec));

   //Sum control forces
   if (PC_INTEGRAL_EN){
   matadd(&(pos_controller->force_i_vec),&(pos_controller->force_vec),&(pos_controller->force_vec));}
   //printf("Force i Mat\n");
   //printMat(&(pos_controller->force_i_vec));

   // Denormalization of control action
   matmul_element(&(pos_controller->force_vec),&(pos_controller->in_denorm_vec),&(pos_controller->force_vec));

   //disturbance estimation
   if (DISTURBANCE_ESTIMATOR){
	   disturbance_estimation(pos_controller);
	   pos_controller->force_vec.data[FORCE_Z_IDX] = pos_controller->u_dist_rej;
   }

   //printf("Denormalized Control\n");
   //printMat(&(pos_controller->force_vec));

   //calculate gravitational compensation forces
   if(GRAVITATIONAL_ENABLE){
	   dynamic_gravitational_compensation(pos_controller);
   }

   // Add gravitational compensation
   matadd(&(pos_controller->force_vec),&(pos_controller->gravitational_forces),&(pos_controller->force_steady_vec));
   //printf("Forces to apply\n");
   //printMat(&(pos_controller->force_vec));

   //Feedforward FF Control in z direction
   pos_controller->force_steady_vec.data[4] += PC_FF*double_step_acceleration_reference()*MOVER_MASS;


}

void compute_dq_action_malta_position_control(pos_cont_data_type *pos_controller)
{
	//Set fy
	//pos_controller->force_vec.data[1] = 0;
	//pos_controller->force_vec.data[3] = 0;

	//pos_controller->force_vec.data[0] = 0;
	//pos_controller->force_vec.data[2] = 0;


 // Compute bearing force and angle

	//pos_controller->force_vec.data[0] =  medfilt_3(pos_controller->force_vec.data[0],&(pos_controller->medfilt_M1fx));
	//pos_controller->force_vec.data[1] =  medfilt_3(pos_controller->force_vec.data[1],&(pos_controller->medfilt_M1fy));
	//pos_controller->force_vec.data[2] =  medfilt_3(pos_controller->force_vec.data[2],&(pos_controller->medfilt_M2fx));
	//pos_controller->force_vec.data[3] =  medfilt_3(pos_controller->force_vec.data[3],&(pos_controller->medfilt_M2fy));

	if (PC_BEARING_EN) {

    //pos_controller->M1dq.theta_b = -(atan2(pos_controller->out_filter.last_out_vec.data[1],-pos_controller->out_filter.last_out_vec.data[0])+M_PI_2);
	//pos_controller->M2dq.theta_b = -(atan2(pos_controller->out_filter.last_out_vec.data[3],-pos_controller->out_filter.last_out_vec.data[2])+M_PI_2);
	pos_controller->M1dq.theta_b = -(atan2(pos_controller->force_steady_vec.data[1],-pos_controller->force_steady_vec.data[0])+M_PI_2);
    pos_controller->M2dq.theta_b = -(atan2(pos_controller->force_steady_vec.data[3],-pos_controller->force_steady_vec.data[2])+M_PI_2);
	}
	else {
	pos_controller->M1dq.theta_b = 0;
    pos_controller->M2dq.theta_b = 0;}


	pos_controller->M1dq.currents.data[0] = 0;
	pos_controller->M1dq.currents.data[3] = 0;

	pos_controller->M2dq.currents.data[0] = 0;
	pos_controller->M2dq.currents.data[3] = 0;



	if (PC_Z_POS_EN) {
		//pos_controller->M1dq.currents.data[2] = pos_controller->out_filter.last_out_vec.data[4] * I_MACHINE_CONSTANT_DRIVE*0.5;
		//pos_controller->M2dq.currents.data[2] = pos_controller->out_filter.last_out_vec.data[4] * I_MACHINE_CONSTANT_DRIVE*-1;
	pos_controller->M1dq.currents.data[2] = pos_controller->force_steady_vec.data[4] * I_MACHINE_CONSTANT_DRIVE*0.5;
	pos_controller->M2dq.currents.data[2] = pos_controller->force_steady_vec.data[4] * I_MACHINE_CONSTANT_DRIVE*0.5;
		}

	if (PC_BEARING_EN) {
		//pos_controller->M1dq.currents.data[1] = sqrt(pow(pos_controller->out_filter.last_out_vec.data[0],2)+pow(pos_controller->out_filter.last_out_vec.data[1],2))*I_MACHINE_CONSTANT_BEARING;
		//pos_controller->M2dq.currents.data[1] = sqrt(pow(pos_controller->out_filter.last_out_vec.data[2],2)+pow(pos_controller->out_filter.last_out_vec.data[3],2))*I_MACHINE_CONSTANT_BEARING;
		if (PC_BEARING_M1_EN) {
		pos_controller->M1dq.currents.data[1] = sqrt(pow(pos_controller->force_steady_vec.data[0],2)+pow(pos_controller->force_steady_vec.data[1],2))*I_MACHINE_CONSTANT_BEARING_M1;}
		if (PC_BEARING_M2_EN) {
        pos_controller->M2dq.currents.data[1] = sqrt(pow(pos_controller->force_steady_vec.data[2],2)+pow(pos_controller->force_steady_vec.data[3],2))*I_MACHINE_CONSTANT_BEARING_M2;}
	}

	limit_dq_currents(&pos_controller->M1dq.currents.data[1]);
	limit_dq_currents(&pos_controller->M1dq.currents.data[2]);
	limit_dq_currents(&pos_controller->M2dq.currents.data[1]);
	limit_dq_currents(&pos_controller->M2dq.currents.data[2]);


}


// 3 order approximation
void compute_z_position(pos_cont_data_type *pos_controller)
	{
	// Reconstruction for steel extension rods
	//int N_Poly = 4;
	//Reconstruction for 3D Printed extension rods
	int N_Poly = 3;

	       double x_temp[N_Poly];
	       double y_temp[N_Poly];

	       x_temp[0] = pos_controller->flux_vec.data[0];
	       y_temp[0] = pos_controller->flux_vec.data[1];

	       // Reconstruction for steel extension rods
	              //x_temp[0] = ( x_temp[0] - 15900.0 ) / 7583.0;
	              //y_temp[0] = ( y_temp[0] - 15840.0 ) / 7709.0;
	              //int N_Poly = 4;

	              //Reconstruction for 3D Printed extension rods
	              x_temp[0] = ( x_temp[0] - 14680.0 ) / 7796.0;
	              y_temp[0] = ( y_temp[0] - 16220.0 ) / 6439.0;


	              //Compute powers for x
	              for (int i=1;i<N_Poly;i++)
	              {
	           	   x_temp[i] = x_temp[0]*x_temp[i-1];
	              }

	              //Compute powers for x
	                 for (int i=1;i<N_Poly;i++)
	                 {
	              	   y_temp[i] = y_temp[0]*y_temp[i-1];
	                 }




	              // z position in mm
	              double z;
	              // Reconstruction for steel extension rods
	              /*z = 31.1299 + 16.0972*x_temp[0] - 9.4814*y_temp[0] \
	                          + 1.4141*x_temp[1] - 19.2734*x_temp[0]*y_temp[0] + 7.2485*y_temp[1] \
	                          - 5.6149*x_temp[2] + 13.9207*x_temp[1]*y_temp[0] + 0.7460*x_temp[0]*y_temp[1] + 2.6229*y_temp[2] \ */

	              //Reconstruction for 3D Printed extension rods
	              z = 20.5557 + 7.1453*x_temp[0] - 3.9629*y_temp[0] \
	                                 + 7.2253*x_temp[1] - 2.7375*x_temp[0]*y_temp[0] + 10.1524*y_temp[1] \
	                                 + 0.7049*x_temp[2] + 1.8234*x_temp[1]*y_temp[0] - 4.3258*x_temp[0]*y_temp[1] + 2.3428*y_temp[2] ;

	      	       pos_controller->mea_vec.data[4] = z*MM_TO_M - 0.03613;
	}

// 5 order approximation
/*void compute_z_position(pos_cont_data_type *pos_controller)
	{
	// Reconstruction for steel extension rods
	//int N_Poly = 4;
	//Reconstruction for 3D Printed extension rods
	int N_Poly = 5;

	       double x_temp[N_Poly];
	       double y_temp[N_Poly];

	       x_temp[0] = pos_controller->flux_vec.data[0];
	       y_temp[0] = pos_controller->flux_vec.data[1];

	       // Reconstruction for steel extension rods
	              //x_temp[0] = ( x_temp[0] - 15900.0 ) / 7583.0;
	              //y_temp[0] = ( y_temp[0] - 15840.0 ) / 7709.0;
	              //int N_Poly = 4;

	              //Reconstruction for 3D Printed extension rods
	              x_temp[0] = ( x_temp[0] - 14680.0 ) / 7796.0;
	              y_temp[0] = ( y_temp[0] - 16220.0 ) / 6439.0;


	              //Compute powers for x
	              for (int i=1;i<N_Poly;i++)
	              {
	           	   x_temp[i] = x_temp[0]*x_temp[i-1];
	              }

	              //Compute powers for x
	                 for (int i=1;i<N_Poly;i++)
	                 {
	              	   y_temp[i] = y_temp[0]*y_temp[i-1];
	                 }




	              // z position in mm
	              double z;
	              // Reconstruction for steel extension rods
	              z = 19.5479 + 10.4416*x_temp[0] - 0.6362*y_temp[0] \
	                          + 13.4736*x_temp[1] - 7.3098*x_temp[0]*y_temp[0] + 21.4172*y_temp[1] \
	                          + 0.6406 *x_temp[2] - 1.9681*x_temp[1]*y_temp[0] - 13.8047*x_temp[0]*y_temp[1] - 0.3223*y_temp[2] \
	                          - 3.8649*x_temp[3] + 3.3351*x_temp[2]*y_temp[0] - 4.2336*x_temp[1]*y_temp[1] + 8.9916*x_temp[0]*y_temp[2] - 6.2712*y_temp[3] \
	                          - 1.4926*x_temp[4] + 1.4664*x_temp[3]*y_temp[0] + 0.6250*x_temp[2]*y_temp[1] - 5.5503*x_temp[1]*y_temp[2] + 4.0272*x_temp[0]*y_temp[3] - 1.5241*y_temp[4];

	      	       pos_controller->mea_vec.data[4] = z*MM_TO_M;
	}
*/

void limit_dq_currents(double *current)
{
	//printf("%.2f\n",check_softstart());
	double l;
	l = PC_CURRENT_LIMIT*check_softstart();

	if(*current < -l){
	 *current = -l;}
	if(*current > l){
 	 *current = l;}
}






