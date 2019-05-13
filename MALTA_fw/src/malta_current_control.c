/*
 * malta_current_control.c
 *
 *  Created on: 30.04.2018
 *      Author: fabiand
 */

#include "malta_current_control.h"
#include "reference_generation.h"

//************* Current controller setup *******************//
static curent_control_data_type Current_Controller[NUM_CURRENT_CONTROLLERS];

void init_malta_current_control(int idx_c,int coil_start_index)
{
	Current_Controller[idx_c].Udc = 0;
	Current_Controller[idx_c].Udc_mea = 0;

	Current_Controller[idx_c].coil_start_idx = coil_start_index;

	Init_Matrix(&(Current_Controller[idx_c].abc_currents),3,3);
	Init_Matrix(&(Current_Controller[idx_c].dq_currents),2,2);
	Init_Matrix(&(Current_Controller[idx_c].dq_voltages),2,2);
	Init_Matrix(&(Current_Controller[idx_c].abc_voltages),3,3);
	Init_Matrix(&(Current_Controller[idx_c].abc_pwmduty),3,3);
	Init_Matrix(&(Current_Controller[idx_c].dq_ref_currents),2,2);
	Current_Controller[idx_c].theta_z = 0;
	Current_Controller[idx_c].theta_b = 0;

	init_filter(&(Current_Controller[idx_c].input_filter),9,LP_FILTER_A_5kHz,LP_FILTER_B_5kHz);

	init_2Ddq_transform(&(Current_Controller[idx_c].dq_data));

	for (int i=0;i<NUM_PI_CUR_CONTROLLER;i++)
	{
		PID_init(&(Current_Controller[idx_c].PI_current_control[i]),CUR_CONTROL_KP, CUR_CONTROL_KI, CUR_CONTROL_KD,CUR_CONTROL_TSAMP, CUR_CONTROL_NORM_IN, CUR_CONTROL_NORM_OUT, CUR_CONTROL_SAT_MIN, CUR_CONTROL_SAT_MAX );

	}

}

void reset_current_controller()
{

for	(int j=0;j<NUM_CURRENT_CONTROLLERS;j++)
{
//Resets the pid controllers
 for (int i=0;i<NUM_PI_CUR_CONTROLLER;i++)
			{
				PID_reset(&(Current_Controller[j].PI_current_control[i]));

			}
 //Clear dq-currents
 mat_clear(&Current_Controller[j].dq_currents);
 mat_clear(&Current_Controller[j].dq_voltages);
 mat_clear(&Current_Controller[j].abc_currents);
 mat_clear(&Current_Controller[j].abc_voltages);
 mat_clear(&Current_Controller[j].abc_pwmduty);
 mat_clear(&Current_Controller[j].dq_ref_currents);
 Current_Controller[j].Udc_mea = 0;

}

}
void measurement_update_cc(int idx_c)
{
	//printf("Udc dir %f\n",measure_dc());
    Current_Controller[idx_c].Udc_mea = measure_dc();
	Current_Controller[idx_c].Udc = DC_LINK_VOLTAGE;//measure_dc();
   	//
	/*for(uint32_t k = 0; k < 9; k++)
	{
		(Current_Controller[idx_c].abc_currents.data)[k] = measure_current1(k+1);
	}*/

	measure_current_arr((Current_Controller[idx_c].abc_currents.data),Current_Controller[idx_c].coil_start_idx, 9);

	if(EN_MEA_FILTER_CC){
	first_ord_filter(&(Current_Controller[idx_c].abc_currents), &(Current_Controller[idx_c].input_filter));}
	else {
	mat_copy(&(Current_Controller[idx_c].input_filter.last_out_vec),&(Current_Controller[idx_c].abc_currents));}

}

double read_current_cc(int N_cur,int Module)
{
	return Current_Controller[Module-1].abc_currents.data[N_cur];
}

// Reads dq currents
// M1: 1
// M2: 2
double read_current_dq_cc(int N_cur,int Module)
{
	return Current_Controller[Module-1].dq_currents.data[N_cur]*NORM_IN;
}

double read_voltage_dq_cc(int N_voltage,int Module)
{
	return Current_Controller[Module-1].dq_voltages.data[N_voltage];
}

double read_voltage_abc_cc(int N_voltage,int Module)
{
	return Current_Controller[Module-1].abc_voltages.data[N_voltage];
}

double read_voltage_dc_cc(int Module)
{
	return Current_Controller[Module-1].Udc_mea;
}

double read_ref_current_dq_cc(int N_cur,int Module)
{
	return Current_Controller[Module-1].dq_ref_currents.data[N_cur]*NORM_IN;
}


void normalize_dq_current(int idx_c)
{
	matmul_val(&(Current_Controller[idx_c].dq_currents),I_NORM_IN,&(Current_Controller[idx_c].dq_currents));
}


void denormalize_dq_voltages(int idx_c)
{
	matmul_val(&(Current_Controller[idx_c].dq_voltages),DENORM_OUT,&(Current_Controller[idx_c].dq_voltages));
}

void reference_update_cc(int idx_c,ref_data_type *ref_vals)
{
    //Updates and normalizes the reference
	matmul_val(&(ref_vals->currents),I_NORM_IN,&(Current_Controller[idx_c].dq_ref_currents));
	Current_Controller[idx_c].theta_z = ref_vals->theta_z;
	//Transform bearing angle from force frame to the coile aligned frame
	Current_Controller[idx_c].theta_b = ref_vals->theta_b+M_PI_3;

}

void current_control_action(int idx_c,mat_double *ref_vals)
{
  for (int i=0;i<NUM_PI_CUR_CONTROLLER;i++)
  {
	  double ff = boost*double_step_voltage_reference()*NORM_OUT;
	  //add normalized voltage feed forward term in z direction
	  if (i == 2 && feed_forward_only() == 1 && PC_Voltage_FF && PC_CONTROLLER_SUPPRESSION){
		  Current_Controller[idx_c].dq_voltages.data[i] = ff;
	  }else{
		  Current_Controller[idx_c].dq_voltages.data[i] = PID_compute_action(&(Current_Controller[idx_c].PI_current_control[i]),ref_vals->data[i], Current_Controller[idx_c].dq_currents.data[i], 0);
	  }
  }
}

void compute_pwm(int idx_c)
{
	 // Computes duty cycle u = (Vabc+Udc/2)/Udc
	 matadd_val(&Current_Controller[idx_c].abc_voltages,Current_Controller[idx_c].Udc/2.0,&Current_Controller[idx_c].abc_pwmduty);
	 matmul_val(&Current_Controller[idx_c].abc_pwmduty,1/Current_Controller[idx_c].Udc,&Current_Controller[idx_c].abc_pwmduty);

}

void apply_current_control(int idx_c)
{

	//printf("PWM Mat \n");
	//printMat(&(Current_Controller[idx_c].abc_pwmduty));
	set_modindex_array((Current_Controller[idx_c].abc_pwmduty.data), Current_Controller[idx_c].coil_start_idx, 9);


}


void execute_current_controller(ref_data_type *data_M1,ref_data_type *data_M2)
{
	//Update and normalize reference values
    reference_update_cc(M1_CUR_CONT,data_M1);
    reference_update_cc(M2_CUR_CONT,data_M2);


    //Run controller

    if (CURRENT_CONTROL_M1) {
	    measurement_update_cc(M1_CUR_CONT);
	    abc2dq_2Ddq_transform(&Current_Controller[M1_CUR_CONT].dq_currents,&Current_Controller[M1_CUR_CONT].abc_currents, Current_Controller[M1_CUR_CONT].theta_b,Current_Controller[M1_CUR_CONT].theta_z,&Current_Controller[M1_CUR_CONT].dq_data);
	    normalize_dq_current(M1_CUR_CONT);
	    current_control_action(M1_CUR_CONT,&(Current_Controller[M1_CUR_CONT].dq_ref_currents));
	    denormalize_dq_voltages(M1_CUR_CONT);
	    dq2abc_2Ddq_transform(&Current_Controller[M1_CUR_CONT].dq_voltages,&Current_Controller[M1_CUR_CONT].abc_voltages, Current_Controller[M1_CUR_CONT].theta_b,Current_Controller[M1_CUR_CONT].theta_z,&Current_Controller[M1_CUR_CONT].dq_data);
	    compute_pwm(M1_CUR_CONT);

	}
    //clear_time_pin();
    //set_time_pin();
    if (CURRENT_CONTROL_M2) {
	    measurement_update_cc(M2_CUR_CONT);
	    abc2dq_2Ddq_transform(&Current_Controller[M2_CUR_CONT].dq_currents,&Current_Controller[M2_CUR_CONT].abc_currents, Current_Controller[M2_CUR_CONT].theta_b,Current_Controller[M2_CUR_CONT].theta_z,&Current_Controller[M2_CUR_CONT].dq_data);
	    normalize_dq_current(M2_CUR_CONT);
	    current_control_action(M2_CUR_CONT,&(Current_Controller[M2_CUR_CONT].dq_ref_currents));
	    denormalize_dq_voltages(M2_CUR_CONT);
	    dq2abc_2Ddq_transform(&Current_Controller[M2_CUR_CONT].dq_voltages,&Current_Controller[M2_CUR_CONT].abc_voltages, Current_Controller[M2_CUR_CONT].theta_b,Current_Controller[M2_CUR_CONT].theta_z,&Current_Controller[M2_CUR_CONT].dq_data);
	    compute_pwm(M2_CUR_CONT);

	}


    //clear_time_pin();
    //set_time_pin();
	   if (CURRENT_CONTROL_M1) {
       apply_current_control(M1_CUR_CONT);
		   }

       if (CURRENT_CONTROL_M2) {
       apply_current_control(M2_CUR_CONT);
    	   }


}


/*void limit_dq_current(double *current)
{
 if(*current<-PC_CURRENT_LIMIT){
	 *current = -PC_CURRENT_LIMIT;}
 if(*current>PC_CURRENT_LIMIT){
 	 *current = PC_CURRENT_LIMIT;}
}*/
