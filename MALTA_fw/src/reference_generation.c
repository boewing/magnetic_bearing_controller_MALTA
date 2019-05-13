/*
 * reference_generation.c
 *
 *  Created on: 06.06.2018
 *      Author: fabiand
 */

#include "reference_generation.h"
#include "reference_step_vectors.h"

static reference_generation_data_type ref_data;

void init_reference_generation()
{
	// Set default reference selection to manual
	ref_data.selected_reference = REF_GEN_MANUAL;
	// Initialises default z position to 0
	ref_data.manual_z_reference = 0;

	//Initialises default sine frequency to 1Hz
	ref_data.sin_ref_period = 1;
	ref_data.sin_ref_stroke = 5*MM_TO_M;
	ref_data.cnt = 0;
	ref_data.ff_offset = (int)(0.0*20); //time difference between reference trajectory and ff_control
	//printf("T_offset = %d\n",ref_data.ff_offset);

	//boost of 1.7321 and 45 V
//	ref_data.dstep.t1 = 101;
//	ref_data.dstep.t2 = 207;
//	ref_data.dstep.t3 = 223;

	//boost of 1.7321 and 50 V
	ref_data.dstep.t1 = 97;
	ref_data.dstep.t2 = 204;
	ref_data.dstep.t3 = 219;


//for 9mm and boost of 1
//	ref_data.dstep.t1 = 132;
//	ref_data.dstep.t2 = 253;
//	ref_data.dstep.t3 = 269;

//	ref_data.dstep.t1 = 126;
//	ref_data.dstep.t2 = 249;
//	ref_data.dstep.t3 = 266;
	ref_data.dstep.t4 = 350;
	ref_data.dstep.stepsize = 0.009;
}


void select_reference(int ref)
{
	if(ref>=REF_GEN_MANUAL && ref<=REF_GEN_DSTEP)  //Check if selected reference is in range of available references
	{
	ref_data.selected_reference = ref;}
	else {
		printf("Error: Selectable references:\n0: Manual\n1: Sinusoidal\n2: Double Step");
	}

	//Temporary setting of the double step
	if (ref_data.selected_reference==REF_GEN_DSTEP)
	{
		init_double_step_position_reference();
	}
}

int feed_forward_only(){
	int t = ref_data.cnt - ref_data.dstep.step_time_1;
	if(t < ref_data.dstep.t4 && t >= 0 && ref_data.dstep.step_en == 1)
		return 1;
	return 0;
}

void set_manual_z_position_ref(double z_ref)
{
	if (z_ref>=-REF_GEN_LIMIT_POSITION && z_ref<=REF_GEN_LIMIT_POSITION) {
		ref_data.manual_z_reference = z_ref;}
	else {
	printf("Selected z position is out of limits. Limit is (mm): +-%.2f\n",REF_GEN_LIMIT_POSITION*1000);}

}

void set_sin_position_ref_frequency(double frequ)
{

	if (frequ>=0 && frequ<=REF_GEN_LIMIT_SINFREQ) {
		ref_data.sin_ref_period = 1/frequ;}
	else {
	printf("Selected frequency is out of limits. Limit is (Hz): %d\n",REF_GEN_LIMIT_SINFREQ);}


}

void set_sin_position_ref_stroke(double stroke)
{
	if (stroke>=0 && stroke<=REF_GEN_LIMIT_POSITION) {
	ref_data.sin_ref_stroke = stroke;}
	else {
	printf("Selected stroke is out of limits. Limit is (mm): %.2f\n",REF_GEN_LIMIT_POSITION*1000);}

}

double get_reference()
{
	  switch(ref_data.selected_reference) {
  	case REF_GEN_MANUAL: return manual_position_reference(); break;
  	case REF_GEN_SINUS: return sin_position_reference(); break;
  	case REF_GEN_DSTEP: return double_step_position_reference(); break;

  	default: return 0; break;}


}

double get_vel_reference()
{
	  switch(ref_data.selected_reference) {
  	case REF_GEN_MANUAL: return 0; break;
  	case REF_GEN_SINUS: return 0; break;
  	case REF_GEN_DSTEP: return double_step_velocity_reference(); break;

  	default: return 0; break;}


}

void start_double_step_position_reference()
{
	ref_data.dstep.step_en = 1;
}


//************************** Trajectory Generation functions *****************

double manual_position_reference()
{
	return ref_data.manual_z_reference;
}

double get_step_size(){
	return ref_data.dstep.stepsize;
}

void set_step_size(double stepsize){
	ref_data.dstep.stepsize = stepsize;
}

double double_step_voltage_reference()
{
	double temp_ref = 0;

	//If step process is enable -> count the sampling intervals for changing the reference
	if (ref_data.dstep.step_en)
	{
		//ref_data.cnt++;

		// If count is past first step time return pos2
		if (ref_data.cnt < ref_data.dstep.step_time_1 + ref_data.ff_offset)
		{
			//temp_ref = 0;
		}else if(ref_data.cnt < ref_data.ff_offset + ref_data.dstep.step_time_1 + ref_data.dstep.t3){
			if(ref_data.dstep.param == 1){
				int t = ref_data.cnt - ref_data.dstep.step_time_1;
				if(t >= 0 && t < ref_data.dstep.t1){
					temp_ref = VOLTAGE_SAT;}
				else {if(t < ref_data.dstep.t2){
					temp_ref = -VOLTAGE_SAT;}
				else {if(t < ref_data.dstep.t3){
					temp_ref = VOLTAGE_SAT;}}}
				//printf("t = %d and voltage = %f \n", t, temp_ref);
			}else
				temp_ref = V_ref_step[0][ref_data.cnt - ref_data.dstep.step_time_1 - ref_data.ff_offset];
		}else if(ref_data.cnt < ref_data.dstep.step_time_2){
			//temp_ref = 0;
		}else{
		// If count is past first step time return pos1 and disable step process
		ref_data.dstep.step_en = 0;
		ref_data.cnt = 0;
		}
	}
	return temp_ref;
}

double double_step_position_reference()
{
	double temp_ref = ref_data.dstep.pos1;

	//If step process is enable -> count the sampling intervals for changing the reference
	if (ref_data.dstep.step_en)
	{
		ref_data.cnt++;

		// If count is past first step time return pos2
		if (ref_data.cnt < ref_data.dstep.step_time_1 + ref_data.ff_offset)
		{
			temp_ref = ref_data.dstep.pos1;
		}else if(ref_data.cnt < ref_data.ff_offset + ref_data.dstep.step_time_1 + ref_data.dstep.traj_len){
			temp_ref = z_ref_step[0][ref_data.cnt - ref_data.dstep.step_time_1 - ref_data.ff_offset];
		}else if(ref_data.cnt < ref_data.dstep.step_time_2){
			temp_ref = ref_data.dstep.pos2;
		}else{
		// If count is past first step time return pos1 and disable step process
		ref_data.dstep.step_en = 0;
		ref_data.cnt = 0;
		temp_ref = ref_data.dstep.pos1;
		}
	}
	return temp_ref;
}

double double_step_velocity_reference()
{

	//If step process is enable -> count the sampling intervals for changing the reference
	if (ref_data.dstep.step_en)
	{
		//ref_data.cnt++;

		// If count is past first step time return pos2
		if (ref_data.cnt < ref_data.dstep.step_time_1 + ref_data.ff_offset)
		{
			return 0;
		}else if(ref_data.cnt < ref_data.ff_offset + ref_data.dstep.step_time_1 + ref_data.dstep.traj_len){
			return dz_ref_step[0][ref_data.cnt - ref_data.dstep.step_time_1 - ref_data.ff_offset];
		}else if(ref_data.cnt < ref_data.dstep.step_time_2){
			return 0;
		}else{
		// If count is past first step time return pos1 and disable step process
		ref_data.dstep.step_en = 0;
		ref_data.cnt = 0;
		}
	}
	return 0;
}

double double_step_acceleration_reference()
{

	//If step process is enable -> count the sampling intervals for changing the reference
	if (ref_data.dstep.step_en)
	{
		//ref_data.cnt++;

		// If count is past first step time return pos2
		if (ref_data.cnt < ref_data.dstep.step_time_1)
		{
			return 0;
		}else if(ref_data.cnt < ref_data.dstep.step_time_1 + ref_data.dstep.traj_len){
			return a_ref_step[0][ref_data.cnt - ref_data.dstep.step_time_1];
		}else if(ref_data.cnt < ref_data.dstep.step_time_2){
			return 0;
		}else{
		// If count is past first step time return pos1 and disable step process
		ref_data.dstep.step_en = 0;
		ref_data.cnt = 0;
		}
	}
	return 0;
}


void init_double_step_position_reference()
{
	ref_data.dstep.pos1 = ref_data.manual_z_reference;   // .0 is required | otherwise compiler assumes integers -> 0
	ref_data.dstep.pos2 = ref_data.manual_z_reference + ref_data.dstep.stepsize;    // .0 is required | otherwise compiler assumes integers -> 0
	ref_data.dstep.step_time_1 = 500;   //Number of controller sample times to step 1
	ref_data.dstep.step_time_2 = 3500;   //Number of controller sample times to step 1
	ref_data.dstep.step_en = 0;          //Disables step process
	ref_data.cnt = 0;				//counter: zero is step_time before the actual first step
	//ref_data.dstep.t1 = 116;
	ref_data.dstep.param = 1;
	ref_data.dstep.traj_len = sizeof(a_ref_step[0])/sizeof(a_ref_step[0][0]);
	//printf("length = %d", ref_data.dstep.traj_len);
	//printf("init is called\n");

}

//Sinusoidal movement
double sin_position_reference()
   {
     int nmax = (int)((double)(PC_FSAMPLE)*ref_data.sin_ref_period  );
     static int acc_count = 0;
     double theta_aux = 0.0;

     acc_count += 1;
     if (acc_count > nmax){
     acc_count = 0;
         }
     //
     theta_aux = 2.0 * M_PI * ( (double)acc_count / (double)nmax );
     return (ref_data.sin_ref_stroke *0.5 * sin( theta_aux ));
}

int get_t4(){
	return ref_data.dstep.t4;
}
int get_t3(){
	return ref_data.dstep.t3;
}
int get_t2(){
	return ref_data.dstep.t2;
}
int get_t1(){
	return ref_data.dstep.t1;
}

int get_step_time2(){
	return ref_data.dstep.step_time_2;
}

int get_step_time1(){
	return ref_data.dstep.step_time_1;
}

int get_t(int i){
	if(i == 1){
		return ref_data.dstep.t1;
	}else if(i == 2){
		return ref_data.dstep.t2;
	}else if(i == 3){
		return ref_data.dstep.t3;
	}
}

void set_t(int i, int newt){
	if(i == 1){
		ref_data.dstep.t1 = newt;
	}else if(i == 2){
		ref_data.dstep.t2 = newt;
	}else if(i == 3){
		ref_data.dstep.t3 = newt;
	}
}

void set_all_t(int* newt){
	ref_data.dstep.t1 = newt[1];
	ref_data.dstep.t2 = newt[2];
	ref_data.dstep.t3 = newt[3];
}

void set_t1(double newt){
	//printf("t1 = %d",ref_data.dstep.t1);
	ref_data.dstep.t1 = newt;
}
void set_t2(double newt){
	ref_data.dstep.t2 = newt;
}
void set_t3(double newt){
	ref_data.dstep.t3 = newt;
}
