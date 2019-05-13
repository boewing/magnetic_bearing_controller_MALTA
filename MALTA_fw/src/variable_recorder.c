/*
 * variable_recorder.c
 *
 *  Created on: 24.05.2018
 *      Author: fabiand
 */


#include "variable_recorder.h"
#include "reference_generation.h"

static variable_rec_data_type variable_recorder[NUM_VAR_RECORDER];
int num_last_var_rec = -1;


// Initialises the variable recorder returns 0 if ok, or 1 if initialisation failed
int init_var_recoder(double *log_var,int n_samples,const char* const name)
{
	++num_last_var_rec;
	if(num_last_var_rec >= NUM_VAR_RECORDER){
		printf("Varrec Array to small. Increase NUM_VAR_RECORDER!\n");
		return 1;
	}

	variable_recorder[num_last_var_rec].log_var = log_var;
	variable_recorder[num_last_var_rec].n_samp = n_samples;
	variable_recorder[num_last_var_rec].write_pointer = 0;
	variable_recorder[num_last_var_rec].log_enable = 0;
	variable_recorder[num_last_var_rec].start_on_setpoint = 0;
	variable_recorder[num_last_var_rec].name = name;

	//Allocates memory for the recording
	variable_recorder[num_last_var_rec].data_store = calloc(n_samples , sizeof(double));

	//Checks if memory allocation was successful
	if (variable_recorder[num_last_var_rec].data_store == NULL){
		printf("Memory allocation failed, less samples\n");
		}
	else{
		printf("Variable recorder %10s with number %2d initialised\n",name,num_last_var_rec);
		}
	return 0;
}

void start_var_recorder()
{
	for (int i=0;i<NUM_VAR_RECORDER;i++)
	{
		variable_recorder[i].log_enable = 1;
	}

}

void set_setpoint_change_var_recorder(int flag)
{
	for (int i=0;i<NUM_VAR_RECORDER;i++)
	{
		variable_recorder[i].start_on_setpoint = flag;
	}
}

void setpoint_change_var_recorder()
{
	for (int i=0;i<NUM_VAR_RECORDER;i++) {
	if (variable_recorder[i].start_on_setpoint)
	{
		variable_recorder[i].log_enable = 1;
	}
	}


}

void stop_var_recorder()
{
	for (int i=0;i<NUM_VAR_RECORDER;i++)
	{
		variable_recorder[i].log_enable = 0;
	}

}

void reset_var_recorder()
{
	for (int i=0;i<NUM_VAR_RECORDER;i++)
	{
		variable_recorder[i].write_pointer = 0;
	}
}


int sample_var_recorder()
{
	for (int i=0;i<NUM_VAR_RECORDER;i++)
	{
	//if ((variable_recorder.write_pointer<variable_recorder.n_samp)&&variable_recorder.log_enable==1) {
	if ((variable_recorder[i].write_pointer<variable_recorder[i].n_samp)&&variable_recorder[i].log_enable==1) {
	variable_recorder[i].data_store[variable_recorder[i].write_pointer] = *variable_recorder[i].log_var;
	variable_recorder[i].write_pointer++;
		}
	}
	return 0;

}

void print_memory_var_recorder()
{
	//print the headerline with the variable names
	printf("\n###HEADER###\n");
	for (int j=0;j<num_last_var_rec;j++){
		printf("%s,",variable_recorder[j].name);
	}
	printf("%s\n",variable_recorder[num_last_var_rec].name);

	//print the data
   for (int i=0;i<variable_recorder[0].n_samp;i++)
   {
	  for (int j=0;j<num_last_var_rec;j++){
	  printf("%.*f,",7,variable_recorder[j].data_store[i]);
	  }
	  printf("%.*f\n",7,variable_recorder[num_last_var_rec].data_store[i]);
   }


}


void close_var_recorder()
{
	printf("Variable recorder closed\n");
	for (int i=1;i<NUM_VAR_RECORDER;i++)
	{
		free(variable_recorder[i].data_store);
		variable_recorder[i].log_enable = 0;
	}



}

variable_rec_data_type get_z_rec(){
	int z_idx = 31;
	for (int j=0;j<=num_last_var_rec;j++){
		if (strcmp(variable_recorder[j].name, "z_est") == 0){
			z_idx = j;
			//printf("found z_est at index %d\n", z_idx);
		}
	}
	return variable_recorder[z_idx];
}
