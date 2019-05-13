/*
 * variable_recorder.h
 *
 *  Created on: 24.05.2018
 *      Author: fabiand
 */

#ifndef SRC_VARIABLE_RECORDER_H_
#define SRC_VARIABLE_RECORDER_H_

#include <stdio.h>
#include <stdlib.h>

//Max number of samples. 500MB (max. 1GB) can be used. A float needs 8 Bytes.
#define VAR_REC_MAX_NUMSAMP 62500
#define NUM_VAR_RECORDER 45

typedef struct{
	double *log_var;    //Pointer to variable to log
	int n_samp;         //Number of samples
	int write_pointer;  //Current writing position
	double *data_store; //Data memory
	int log_enable;    //Enables the recording
	int start_on_setpoint; //Flag to start on a set point change
	char* name;

}variable_rec_data_type;


int init_var_recoder(double *log_var,int n_samples,const char* name);
void close_var_recorder();
void start_var_recorder();
void stop_var_recorder();
void reset_var_recorder();
int sample_var_recorder();
void set_setpoint_change_var_recorder();
void setpoint_change_var_recorder();
void print_memory_var_recorder();
variable_rec_data_type get_z_rec();


#endif /* SRC_VARIABLE_RECORDER_H_ */
