/*
 * commands.h
 *
 *  Created on: 8 Mar 2018
 *      Author: smiric
 */

/**************************************************************************************************************
 **
 ** SPAHD Zynq firmware
 ** Mauerer August 2016
 **
 ** Origin: BitHound and miniLINK
 ** (c) 2013 Lukas Schrittwieser, Mario Mauerer
 ** Modified for Zynq by MM, August 2016
 **
 ** PES ETH Zurich - All rights reserved
 **
 **************************************************************************************************************
 **
 ** commands.h
 **
 *************************************************************************************************************/

#ifndef SRC_COMMANDS_H_
#define SRC_COMMANDS_H_

#include <stdint.h>
#include "xil_printf.h"
#include <stdio.h>
#include <stdlib.h>
#include "adc.h"
#include "pwm.h"
#include "malta_configure.h"
#include "malta_control.h"
#include "interrupt.h"
#include "recorder.h"
#include "variable_recorder.h"
#include "reference_generation.h"
#include "softstart.h"
#include "Control_Handle.h"


// define a macro to determine the number of commands in the command list
#define N_COMMANDS		(sizeof(cmdList)/sizeof(cmd_t))




// create struct type for commands
typedef struct
{
	const char* name;				// command name (as enterd by user)
	void (*handler)(char*, char*);	// handler (called when command is executed)
	const char* helpText;			// short description of command for help function
	const char*(*listFct)(int, int);// List function for auto-complete, first int is argument index, second is the index
									// for possibilities.
} cmd_t;


extern const cmd_t cmdList[];


int getNCommands();

const char* listCmdNames(int argInd, int cmdInd);

void cHelp (char* cmd, char* params);
void ctest (char* cmd, char* params);
void cstart (char* cmd, char* params);
void cstop (char* cmd, char* params);
void creadoff(char* cmd, char* params);
void creadz(char* cmd, char* params);
void creadzraw(char* cmd, char* params);
void creadzrmean(char* cmd, char* params);
void creadzmean(char* cmd, char* params);
void cset(char* cmd, char* params);
void creadtheta(char* cmd, char* params);
void creadforce(char* cmd, char* params);
void creaderror(char* cmd, char* params);
void creadcurrent(char* cmd, char* params);
void creadposition(char* cmd, char* params);
void creadx1rmean(char* cmd, char* params);
void creadx2rmean(char* cmd, char* params);
void cready1rmean(char* cmd, char* params);
void cready2rmean(char* cmd, char* params);

void creadM1xmean(char* cmd, char* params);
void creadM1ymean(char* cmd, char* params);
void creadM2xmean(char* cmd, char* params);
void creadM2ymean(char* cmd, char* params);
void cdstep_with_varrec(char* cmd, char* params);

void creadBAng(char* cmd, char* params);
void creadDAng(char* cmd, char* params);
void creadflraw(char* cmd, char* params);

void creadmean(char* cmd, char* params);

void ccalibrateDAng(char* cmd, char* params);
void cstopcalibrateDAng(char* cmd, char* params);

uint32_t ArgParse(char* params, int32_t min, int32_t max, int32_t* value); //Convert string to int
uint32_t parse_double(char* params, double min, double max, double* value); //Convert string to doouble

void M2x_raw_mean();
void M2y_raw_mean();
void M2f_raw_mean();

void crec (char* cmd, char* params);
void cvarrec (char* cmd, char* params);
void csetpos(char* cmd, char* params);
void cset_step_size(char* cmd, char* params);
void coptimize(char* cmd, char* params);
void cActivateLaser(char* cmd, char* params);


#endif /* SRC_COMMANDS_H_ */

