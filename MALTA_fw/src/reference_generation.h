/*
 * reference_generation.h
 *
 *  Created on: 06.06.2018
 *      Author: fabiand
 */

#ifndef SRC_REFERENCE_GENERATION_H_
#define SRC_REFERENCE_GENERATION_H_

#include "math.h"
#include "malta_position_control.h"
#include "malta_parameters.h"

//Reference defines
#define REF_GEN_MANUAL 0
#define REF_GEN_SINUS 1
#define REF_GEN_DSTEP 2   //Double step reference

//Limits for reference positions
#define REF_GEN_LIMIT_POSITION (0.009)
#define REF_GEN_LIMIT_SINFREQ (20)

typedef struct {

	double pos1;      // First position
	double pos2;      // Second position after step_time_1
	double stepsize;  // size of the step in mm
	int step_time_1; // Time to step to pos2
	int step_time_2;  // Time to step back to pos1
	int step_en;      // Starts double step
	int traj_len; 	  // the length of the reference trajectory
	int t1;
	int t2;
	int t3;
	int t4;
	int param;
}double_step_data_type;

typedef struct {
	int t1;
	int t2;
	int t3;
	int t4;
	int step_en;
}step_data_type;

typedef struct {
 int selected_reference;
 double manual_z_reference;

 double sin_ref_period;    //Use period instead of frequency to avoid division in reference generation
 double sin_ref_stroke;    //Stroke length of movement
 int cnt; 					//
 int ff_offset;

 double_step_data_type dstep;  // Double step reference data

}reference_generation_data_type;

void init_reference_generation(void);
double get_reference(void);
double get_vel_reference(void);

void select_reference(int ref);
void set_manual_z_position_ref(double z_ref);
void set_sin_position_ref_frequency(double frequ);
void set_sin_position_ref_stroke(double stroke);

double manual_position_reference(void);
double sin_position_reference(void);
double double_step_voltage_reference();
double double_step_position_reference();
double double_step_velocity_reference();
double double_step_acceleration_reference();
void init_double_step_position_reference();
void start_double_step_position_reference();
int feed_forward_only();

int get_t4();
int get_t3();
int get_t2();
int get_t1();

int get_step_time2();
int get_step_time1();
double get_step_size();
void set_step_size(double stepsize);
int get_t(int i);
void set_t(int i, int newt);
void set_all_t(int* newt);
void set_t1(double newt);
void set_t2(double newt);
void set_t3(double newt);


#endif /* SRC_REFERENCE_GENERATION_H_ */
