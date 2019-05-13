/*
 * commands.c
 *
 *  Created on: 8 Mar 2018
 *      Author: smiric
 */
/**************************************************************************************************************
 **
 ** Origin: BitHound and miniLINK
 ** (c) 2013 Lukas Schrittwieser, Mario Mauerer
 ** Modified for Zynq by MM, August 2016
 ** Modified for MALTA by Spasoje Miric, 2018
 **
 ** PES ETH Zurich - All rights reserved
 **
 **************************************************************************************************************
 **
 ** commands.c
 **
 *************************************************************************************************************/

#include "commands.h"
#include "step_optimizer.h"

//extern volatile uint32_t sysTick;

/*
 * list of supported commands and their handlers etc
 * Note: command names must not include white spaces (eg blanks or tabs)
 */
const cmd_t cmdList[] = {{"help", &cHelp, "Show command information", &listCmdNames}, \
		{"test", &ctest, "Test message", NULL}, \
		{"start", &cstart, "Starts the system", NULL}, \
		{"stop", &cstop, "Stops the system", NULL}, \
		{"readoff", &creadoff, "Reads the offsets", NULL}, \
		{"readz", &creadz, "Reads axial position", NULL}, \
		{"readzraw", &creadzraw, "Reads raw axial position", NULL}, \
		{"readzrmean", &creadzrmean, "Reads raw axial position", NULL}, \
		{"readzmean", &creadzmean, "Reads mean axial position", NULL}, \
		{"set", &cset, "Sets global variables", NULL}, \
		{"setpos", &csetpos, "Sets position reference", NULL}, \
		{"readtheta", &creadtheta, "Reads theta", NULL}, \
		{"readforce", &creadforce, "Reads force vector to apply", NULL}, \
		{"readerror", &creaderror, "Plots the position error vector", NULL}, \
		{"readcur", &creadcurrent, "Plots the measured currents", NULL}, \
		{"readpos", &creadposition, "Plots the measured position", NULL}, \
		{"readx1rmean", &creadx1rmean, "Reads mean raw axial position", NULL}, \
		{"readx2rmean", &creadx2rmean, "Reads mean raw axial position", NULL}, \
		{"ready1rmean", &cready1rmean, "Reads mean raw axial position", NULL}, \
		{"ready2rmean", &cready2rmean, "Reads mean raw axial position", NULL}, \
		{"readM1xmean", &creadM1xmean, "Reads mean raw axial position", NULL}, \
		{"readM1ymean", &creadM1ymean, "Reads mean raw axial position", NULL}, \
		{"readM2xmean", &creadM2xmean, "Reads mean raw axial position", NULL}, \
		{"readM2ymean", &creadM2ymean, "Reads mean raw axial position", NULL}, \
		{"dstep", &cdstep_with_varrec, "measures a dstep and prints the results to the console", NULL}, \
		{"readBAng", &creadBAng, "Reads mean raw axial position", NULL}, \
		{"readDAng", &creadDAng, "Reads mean raw axial position", NULL}, \
		{"readflraw", &creadflraw, "Reads mean raw axial position", NULL}, \
		{"readmean", &creadmean, "Reads mean raw axial position", NULL}, \
		{"calibrateDAng", &ccalibrateDAng, "Reads mean raw axial position", NULL}, \
		{"stopcalibrateDAng", &cstopcalibrateDAng, "Reads mean raw axial position", NULL}, \
		{"rec", &crec, "Records the data", NULL}, \
		{"setstepsize", &cset_step_size, "Sets the stepsize of the double step", NULL}, \
		{"opt", &coptimize, "optimize the voltage parameters", NULL}, \
		{"activatelaser", &cActivateLaser, "activated the streaming mode on the laser sensor", NULL}, \
		{"varrec", &cvarrec, "Records the data of a variable", NULL}};


void coptimize(char* cmd, char* params)
{
	activate_optimizer();
}

/*
 * Test Command
 */
void ctest (char* cmd, char* params)
{
	printf("Test command output: Blubb\n");
}

/*
 * Start command. Enables PWM block and starts the system.
 */
void cstart(char* cmd, char* params)
{
	//Enable "dead time" and "pwm" blocks
	start_softstart(); // Produces a softstart -> Blocks output for the first cycles to prevent large spikes
	reset_current_controller(); //Clears the integral part of the current controllers and resets the error
	start_control_system();
	enable_pwm();
	enable_dead_time();

}

/*
 * Start command. Enables PWM block and starts the system.
 */
void cstop(char* cmd, char* params)
{
	//Enable "dead time" and "pwm" blocks
	disable_dead_time();
	disable_pwm();
	stop_control_system();
	init_softstart(100,40000);
	reset_optimizer();
}


/*
 * Read the offsets, DC voltage, current, position ADCs
 * During this process, set variables to their zero position.
 */
void creadoff(char* cmd, char* params)
{
	//Measure DC link voltage offset measurement
	enable_pwm();
	measure_dc_offset();
	measure_current_offset1();
	disable_pwm();
	printf("\n\nDC link voltage offset: %f Volts\n\n", dc_voltage_offset);
	for (uint32_t k = 1; k <= 24; k++){
		printf("i%d = %f Amps\n", (int)k, current_offset1[k-1]);
	}
	printf("\n\n");
}

/*
 * Read z position
 */
void creadz(char* cmd, char* params)
{

	double aux_var;
	aux_var = measure_z();
	printf( "z position is: %.4f\n", aux_var*1000 );
}

void creadflraw(char* cmd, char* params)
{
	printf("Flux M1,M2:\n");
	printf("%ld,%ld \n",((int32_t)measure_pos3s2_raw()),((int32_t)measure_pos3s1_raw()));
}

/*
 * Read z position raw
 */
void creadzraw(char* cmd, char* params)
{
	double aux_var = 0.0;
	aux_var = measure_z();
	printf( "raw z position is: %.4f\n", aux_var*1000 );
}

void creadx1rmean(char* cmd, char* params)
{
	int temp = 0;
	for (int i=0;i<1000;i++)
	{
		temp += measure_pos2s1_raw();

	}
	temp = temp/1000;
	printf( "mean x sensor 1 is: %d ADC Value\n", temp );
}

void creadx2rmean(char* cmd, char* params)
{
	int temp = 0;
	for (int i=0;i<1000;i++)
	{
		temp += measure_pos2s2_raw();

	}
	temp = temp/1000;
	printf( "mean x sensor 2 is: %d ADC Value\n", temp );
}

void cready1rmean(char* cmd, char* params)
{
	int temp = 0;
	for (int i=0;i<1000;i++)
	{
		temp += measure_pos1s1_raw();
	}
	temp = temp/1000;
	printf( "mean y sensor 1 is: %d ADC Value\n", temp );
}

void cready2rmean(char* cmd, char* params)
{
	int temp = 0;
	for (int i=0;i<1000;i++)
	{
		temp += measure_pos1s2_raw();

	}
	temp = temp/1000;
	printf( "mean y sensor 2 is: %d ADC Value\n", temp );
}

void creadM1xmean(char* cmd, char* params)
{
	double temp = 0;
	for (int i=0;i<1000;i++)
	{
		temp += measure_m1_x();

	}
	temp = temp/1000.0;
	printf( "mean M1 x position is: %.4f\n", temp*1000 );
}

void creadM1ymean(char* cmd, char* params)
{
	double temp = 0;
	for (int i=0;i<1000;i++)
	{
		temp += measure_m1_y();

	}
	temp = temp/1000;
	printf( "mean M1 y position is: %.4f\n", temp*1000 );
}

void creadM2xmean(char* cmd, char* params)
{
	double temp = 0;
	for (int i=0;i<1000;i++)
	{
		temp += measure_m2_x();

	}
	temp = temp/1000;
	printf( "mean M2 x position is: %.4f\n", temp*1000 );
}


void creadM2ymean(char* cmd, char* params)
{
	double temp = 0;
	for (int i=0;i<1000;i++)
	{
		temp += measure_m2_y();

	}
	temp = temp/1000;
	printf( "mean M2 y position is: %.4f\n", temp*1000 );
}

void cdstep_with_varrec(char* cmd, char* params)
{
	select_reference(2);
	reset_var_recorder();
	start_var_recorder();
	start_double_step_position_reference();
}

void creadBAng(char* cmd, char* params)
{
	set_event(PLOT_BEAR_ANGLE);
	//g_state_to_execute |= PLOT_BEAR_ANGLE;
}

void cset_step_size(char* cmd, char* params){
	double aux_var = 0;
	if(parse_double(params, 0.0, 10.0, &aux_var) !=0)
	{
		xil_printf("Error: z is in the range -10.00 and 10.00\n");
		return;
	}
	printf("%.1f \n",aux_var);
	set_step_size(aux_var*1e-3);
}


void creadzmean(char* cmd, char* params)
{
	double temp = 0;
	for (int i=0;i<1000;i++)
	{
		temp += (measure_z());



	}
	temp = temp/1000;
	printf( "mean z position is: %.4f\n", temp*1000 );
}

void creadzrmean(char* cmd, char* params)
{
	double temp = 0;
	for (int i=0;i<1000;i++)
	{
		temp += (measure_z());

	}
	temp = temp/1000;
	printf( "mean z position is: %.4f\n", temp*1000 );
}

/*
 * Read force to apply
 */
void creadforce(char* cmd, char* params)
{
	//set_event(PLOT_FORCE);
	//g_state_to_execute |= PLOT_FORCE;
}

/*
 * Read position error
 */
void creaderror(char* cmd, char* params)
{
	//set_event(PLOT_ERROR);
	//g_state_to_execute |= PLOT_ERROR;
}

/*
 * Read currents
 */
void creadcurrent(char* cmd, char* params)
{
	set_event(PLOT_CURRENT);
	//g_state_to_execute |= PLOT_CURRENT;
}

/*
 * Read currents
 */
void creadposition(char* cmd, char* params)
{
	set_event(PLOT_POSITION);
	//g_state_to_execute |= PLOT_POSITION;
}

void creadDAng(char* cmd, char* params)
{
	set_event(PLOT_EST_DRIVE_ANGLE);
	//g_state_to_execute |= PLOT_POSITION;
}

void ccalibrateDAng(char* cmd, char* params)
{
	set_event(CALIBRATE_THETA_D);
	//g_state_to_execute |= PLOT_POSITION;
}

void cstopcalibrateDAng(char* cmd, char* params)
{
	clear_event(CALIBRATE_THETA_D);
	//g_state_to_execute |= PLOT_POSITION;
}


/*
 * Read theta position
 */
void creadtheta(char* cmd, char* params)
{

	printf( "theta is: %.4f\n", ctheta );
}


/*
 * Return number of commands in command list
 */
int getNCommands()
{
	return N_COMMANDS;
}

/* return list of command names, needed for auto completer of help command
 *  argInd: index of the argument to be auto completed. Has to be 0 otherwise there are no options
 *         (Because we have only one argument for the help command)
 * cmtInd: index of the command name
 */
const char* listCmdNames(int argInd, int cmdInd)
{
	if (argInd != 0)
		return NULL;
	// check for the end of the command list
	if (cmdInd >= N_COMMANDS)
		return NULL;
	return cmdList[cmdInd].name;
}

/*
 * Handle the "help" command
 */
void cHelp (char* cmd, char* params)
{
	if ((params == NULL) || (*params == '\0'))
	{
		// user passed no arguments -> list all available commands
		printf("Use 'help [command]' for command specific help\nSupported commands:");
		int i;
		for(i=0; i<N_COMMANDS; i++)
		{
			printf("\n  ");
			printf(cmdList[i].name);
		}
		printf("\n");
		printf("Detailed commands: \n");
		printf("test ==> Output Test Message\n");
		printf("\n");
		return;
	}

	int i;
	for(i=0; i<N_COMMANDS; i++)
	{
		if (strcmp(params, cmdList[i].name) == 0)
		{
			if (cmdList[i].helpText != NULL)
			{
				xil_printf(params);
				xil_printf(": ");
				xil_printf(cmdList[i].helpText);
				xil_printf("\n");
			}
			else
				xil_printf("No help text available.\n");
			return;
		}
	}
	// fall through if no such command was found
	printf("Can't find command '%s'\n", params);
}








/* This function gets a string and converts it into a int32_t. The string must be properly terminated!
 ** It also checks whether the number is within min and max.
 ** Return: 0: everything OK. 1: Error while parsing or value out of min/max bounds
 ** params: Pointer to the beginning of the string. After this function, the pointer points to the end of the string!
 ** value: pointer to the value parsed.
 */
uint32_t ArgParse(char* params, int32_t min, int32_t max, int32_t* value)
{
	char* end = params;
	int32_t val = strtol(params, &end, 10);
	// ignore any trailing white spaces
	while ((*end != '\0') && (isspace((int32_t)*end)))
		end++;
	if ((*end != '\0') || (val < min) || (val > max))
	{
		//xil_printf("Range Error! min:%d,  max:%d\n", min, max);
		return 1;
	}
	else
	{
		*value = (int32_t)val;
		return 0;
	}
	return 1;
}

uint32_t parse_double(char* params, double min, double max, double* value)
{
	char* end = params;
	int32_t val = atof(params);
	// ignore any trailing white spaces
	if ((val < min) || (val > max))
	{
		//xil_printf("Range Error! min:%d,  max:%d\n", min, max);
		return 1;
	}
	else
	{
		*value = (int32_t)val;
		return 0;
	}
	return 1;
}

/*
 * Set global vars function
 */
void cset(char* cmd, char* params)
{

	/*
	 * Set theta
	 */
	if(strncmp(params,"theta", 5) == 0)
	{
		params += 5; // Hop over "theta"
		int32_t th = 0;
		if(ArgParse(params, -100, 100, &th) !=0 )
		{
			xil_printf("Error: Channel Range: 1-14. Terminating.\n");
			return;
		}
		// set the global theta in the increments of pi/100
		ctheta = (double)th * PI/100.0;
		printf("theta is set to: %.4f\n", ctheta);
	}
	/*
	 * Set theta for bearing
	 */
	if(strncmp(params,"btheta", 6) == 0)
	{
		params += 6; // Hop over "theta"
		int32_t th = 0;
		if(ArgParse(params, -360, 360, &th) !=0 )
		{
			xil_printf("Error: Channel Range: 1-14. Terminating.\n");
			return;
		}
		// set the global theta in the increments of pi/100
		cthetab = (double)th;
		printf("theta is set to: %.4f\n", cthetab);
	}


	/*
	 * set zpos
	 */
	if(strncmp(params, "bcur", 4) == 0)
	{
		params +=4; // Hop over "zpos"
		int32_t aux_var = 0;
		if(ArgParse(params, -100, 100, &aux_var) !=0 )
		{
			xil_printf("Error: z is in the range 0 and 30. Type 20 for 2.00\n");
			return;
		}
		cbear_cur = ((double)aux_var)/10.0;
		printf("Bearing current is set to: %.4f\n", (double)cbear_cur);
		// update theta
		//zref += 38.75;
		//ctheta =  0.1962 * zref - 7.102;
	}

	/*
	 * set zpos
	 */
	if(strncmp(params, "dcur", 4) == 0)
	{
		params +=4; // Hop over "zpos"
		int32_t aux_var = 0;
		if(ArgParse(params, -100, 100, &aux_var) !=0 )
		{
			xil_printf("Error: z is in the range 0 and 30. Type 20 for 2.00\n");
			return;
		}
		cdrive_cur = ((double)aux_var)/10.0;
		printf("Drive current is set to: %.4f\n", (double)cdrive_cur);
		// update theta
		//zref += 38.75;
		//ctheta =  0.1962 * zref - 7.102;
	}

	/*
	 * set zpos
	 */
	if(strncmp(params, "M2fx", 4) == 0)
	{
		params +=4; // Hop over "zpos"
		int32_t aux_var = 0;
		if(ArgParse(params, -100, 100, &aux_var) !=0 )
		{
			xil_printf("Error: z is in the range 0 and 30. Type 20 for 2.00\n");
			return;
		}
		cM2fx = ((double)aux_var);
		printf("Module 2 force x: %.4f\n", (double)cM2fx);
		// update theta
		//zref += 38.75;
		//ctheta =  0.1962 * zref - 7.102;
	}

	/*
	 * set zpos
	 */
	if(strncmp(params, "M2fy", 4) == 0)
	{
		params +=4; // Hop over "zpos"
		int32_t aux_var = 0;
		if(ArgParse(params, -100, 100, &aux_var) !=0 )
		{
			xil_printf("Error: z is in the range 0 and 30. Type 20 for 2.00\n");
			return;
		}
		cM2fy = ((double)aux_var);
		printf("Module 2 force y: %.4f\n", (double)cM2fy);
		// update theta
		//zref += 38.75;
		//ctheta =  0.1962 * zref - 7.102;
	}

	/*
	 * set zpos
	 */
	if(strncmp(params, "M1fx", 4) == 0)
	{
		params +=4; // Hop over "zpos"
		int32_t aux_var = 0;
		if(ArgParse(params, -100, 100, &aux_var) !=0 )
		{
			xil_printf("Error: z is in the range 0 and 30. Type 20 for 2.00\n");
			return;
		}
		cM1fx = ((double)aux_var);
		printf("Module 1 force x: %.4f\n", (double)cM1fx);
		// update theta
		//zref += 38.75;
		//ctheta =  0.1962 * zref - 7.102;
	}

	/*
	 * set zpos
	 */
	if(strncmp(params, "M1fy", 4) == 0)
	{
		params +=4; // Hop over "zpos"
		int32_t aux_var = 0;
		if(ArgParse(params, -100, 100, &aux_var) !=0 )
		{
			xil_printf("Error: z is in the range 0 and 30. Type 20 for 2.00\n");
			return;
		}
		cM1fy = ((double)aux_var);
		printf("Module 1 force y: %.4f\n", (double)cM1fy);
		// update theta
		//zref += 38.75;
		//ctheta =  0.1962 * zref - 7.102;
	}

	if(strncmp(params, "M1fz", 4) == 0)
	{
		params +=4; // Hop over "zpos"
		int32_t aux_var = 0;
		if(ArgParse(params, -20, 20, &aux_var) !=0 )
		{
			xil_printf("Error: z is in the range 0 and 30. Type 20 for 2.00\n");
			return;
		}
		cM1fz = ((double)aux_var);
		printf("Module 1 force z: %.4f\n", (double)cM1fz);
		// update theta
		//zref += 38.75;
		//ctheta =  0.1962 * zref - 7.102;
	}


}

/*
 * Set global vars function
 */
void csetpos(char* cmd, char* params)
{

	if(strncmp(params, "refsel", 6) == 0)
	{
		params +=6; // Hop over "zpos"
		int32_t aux_var = 0;
		ArgParse(params, 0, 10, &aux_var);

		select_reference((int)aux_var);
		printf("Reference is set to: %d\n", (int)aux_var);

	}

	/*
	 * set zpos
	 */
	if(strncmp(params, "z", 1) == 0)
	{
		params +=1; // Hop over "zpos"
		int32_t aux_var = 0;
		if(ArgParse(params, -100, 100, &aux_var) !=0 )
		{
			xil_printf("Error: z is in the range -100.00 and 100.00. Type 90 for 9.00\n");
			return;
		}

		set_manual_z_position_ref((double)aux_var / 10000.0);
		setpoint_change_var_recorder();
		printf("zpos is set to: %.6f mm\n", (double)aux_var/ 10.0);

	}


	if(strncmp(params, "sinf", 4) == 0)
	{
		params +=4; // Hop over "zpos"
		int32_t aux_var = 0;
		if(ArgParse(params, 0, 200, &aux_var) !=0 )
		{
			xil_printf("Error: z is in the range 0 and 30. Type 20 for 2.00\n");
			return;
		}
		//cfsin = ((double)aux_var)/10.0;
		set_sin_position_ref_frequency((double)(aux_var)/10.0);
		printf("Sin reference frequency is set to (Hz): %.4f\n", (double)(aux_var)/10.0);
		// update theta
		//zref += 38.75;
		//ctheta =  0.1962 * zref - 7.102;
	}

	if(strncmp(params, "sinl", 4) == 0)
	{
		params +=4; // Hop over "zpos"
		int32_t aux_var = 0;
		if(ArgParse(params, 0, 100, &aux_var) !=0 )
		{
			xil_printf("Error: z is in the range 0 and 100. Type 20 for 2.00\n");
			return;
		}
		//cfsin = ((double)aux_var)/10.0;
		set_sin_position_ref_stroke((double)(aux_var)/10.0*MM_TO_M);
		printf("Sin reference stroke set to (mm): %.4f\n", (double)(aux_var)/10.0);
		// update theta
		//zref += 38.75;
		//ctheta =  0.1962 * zref - 7.102;
	}

	if(strncmp(params, "dstep", 5) == 0)
	{
		params +=4; // Hop over "zpos"
		start_double_step_position_reference();
		setpoint_change_var_recorder(); //Start variable recorder
		// update theta
		//zref += 38.75;
		//ctheta =  0.1962 * zref - 7.102;
	}







}



void M2x_raw_mean()
{
double temp = 0;
for (int i=0;i<1000;i++)
{
	temp += (double)(measure_pos2s1_raw());

}
temp = temp/1000;
printf( "mean x Module 2 is: %.4f\n", temp );
}

void M2y_raw_mean()
{
double temp = 0;
for (int i=0;i<1000;i++)
{
	temp += (double)(measure_pos1s1_raw());

}
temp = temp/1000;
printf( "mean y Module 2 is: %.4f\n", temp );
}

void M2f_raw_mean()
{
double temp = 0;
for (int i=0;i<1000;i++)
{
	temp += (double)(measure_pos3s1_raw());

}
temp = temp/1000;
printf( "mean f Module 2 is: %.4f\n", temp );
}


void creadmean(char* cmd, char* params)
{


			printf("x,y,f \n");
			M2x_raw_mean();
			M2y_raw_mean();
			M2f_raw_mean();
			printf("\n");




}


/*
 * Recorder Functions
 */
void crec (char* cmd, char* params)
{


	/*
	 * Start a recording
	 */
	if(strcmp(params,"start") == 0)
	{
		if(rec_clear_start() != 0) //Clear the start bit to reset the recorder
		{
			xil_printf("Error: Could not Reset Start Bit\n");
			return;
		}
		// Fill the memory with a default value (see recorder.h). Such that we can later detect faulty hw-writes
		xil_printf("Writing %d to memory. Please wait.\n",(uint32_t)MEMDEFAULT);
		rec_init_mem();
		if(rec_set_start() != 0) //Set the start bit - start a new recording
		{
			xil_printf("Error: Could not Set Start Bit.\n");
			return;
		}
		else
		{
			//g_RecStat = 1; //We have started a recording.
			xil_printf("Recorder started.\n");
		}
	}
	/*
	 * Interrupt a recording
	 */
	else if(strcmp(params,"stop") == 0)
	{
		rec_set_stop();
		rec_clear_stop();
		//g_RecStat = 0;
		xil_printf("Recorder stopped.\n");
	}
	/*
	 * Configure the data mux
	 */
	else if(strncmp(params,"setmux",6)==0)
	{
		params += 6; //Hop over the "setmux"
		int32_t ch = 0;
		if(ArgParse(params, 1, 15, &ch) !=0 )
		{
			xil_printf("Error: Channel Range: 1-14. Terminating.\n");
			return;
		}
		if(rec_set_channel((uint32_t)ch) != 0)
		{
			xil_printf("Error when setting channel.\n");
			return;
		}
	}
	/*
	 * Write the number of samples to be recorded
	 */
	else if(strncmp(params, "numsampl",8)==0)
	{
		params += 8;
		int32_t numsampl = 0;
		if(ArgParse(params,1,(int32_t)MAXNUMSAMP,&numsampl)!=0)
		{
			xil_printf("Number of samples out of range.\n");
			printf("Allowed range: 1-%d. Terminating.\n",(int)MAXNUMSAMP);
			return;
		}
		if(rec_write_numsamp((uint32_t)numsampl) != 0)
		{
			xil_printf("Error when setting number of samples.\n");
			return;
		}
	}
	/*
	 * Read the samples to UART:
	 */
	else if(strcmp(params,"readuart") == 0)
	{
		if(rec_read_to_uart()!=0)
		{
			xil_printf("Samples could not be read. Recorder is likely busy.\n");
			return;
		}
	}

	else
	{
		xil_printf("Wrong argument. Options: start, stop, setmux <uint>, numsampl <uint>, readuart, readeth, ethtxperf\n");
	}
}


/*
 * Variable Recorder Functions
 */
void cvarrec (char* cmd, char* params)
{


	/*
	 * Start a recording
	 */
	if(strcmp(params,"start") == 0)
	{
		printf("Variable Recorder started\n");
		reset_var_recorder();
		start_var_recorder();
	}
	/*
	 * Interrupt a recording
	 */
	else if(strcmp(params,"stop") == 0)
	{
		printf("Variable Recorder stopped\n");
		stop_var_recorder();
	}

	else if(strcmp(params,"reset") == 0)
	{
		printf("Variable Recorder reseted\n");
		reset_var_recorder();
	}

	else if(strcmp(params,"startsetp") == 0)
	{
		printf("Variable Recorder set to start on setpoint change\n");
		set_setpoint_change_var_recorder(1);
	}

	else if(strcmp(params,"stopsetp") == 0)
	{
		printf("Variable Recorder does not start on setpoint change\n");
		set_setpoint_change_var_recorder(0);
	}

	/*
	 * Read the samples to UART:
	 */
	else if(strcmp(params,"readuart") == 0)
	{
		cstop(NULL,NULL);
		print_memory_var_recorder();
	}

	/*
	 * Calculate the amount of swinging after the first step.
	 */
	/*else if(strcmp(params,"readerror") == 0)
	{
		printf("The Cost Function value of the d_step FF parameters is: %.f", read_error());
	}*/

	else
	{
		xil_printf("Wrong argument. Options: start, stop, setmux <uint>, numsampl <uint>, readuart, readeth, ethtxperf\n");
	}
}

void cActivateLaser(char* cmd, char* params)
{
    /*
     * Stuff for the UART lite for the laser sensor
     * Spasoje
     */
    char TURN_ON_UART[30] = {':', '0', '1', 'W', '0', '1', '0', ';', '0', ';', 'E', '9', 'C', '3', '\r', '\n'};
    char STREAMING_MODE_ON[30] = {':', '0', '1', 'W', '0', '4', '4', ';', '1', ';', '4', '9', '0', 'F', '\r', '\n'};

    uartlite_tx_enable();

	for ( uint32_t i=0; i<1000; i++ ){
		printf("wait\n");
	}

    uartlite_send_command(TURN_ON_UART);

	for ( uint32_t i=0; i<1000; i++ ){
		printf("wait\n");
	}

    uartlite_send_command(STREAMING_MODE_ON);

	for ( uint32_t i=0; i<1000; i++ ){
		printf("wait\n");
	}

    uartlite_send_command(STREAMING_MODE_ON);

	for ( uint32_t i=0; i<1000; i++ ){
		printf("wait\n");
	}

    uartlite_send_command(STREAMING_MODE_ON);

	for ( uint32_t i=0; i<1000; i++ ){
		printf("wait\n");
	}

    uartlite_rx_enable();

}

//end of file


