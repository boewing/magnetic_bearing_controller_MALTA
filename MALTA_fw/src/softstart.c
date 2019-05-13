/*
 * softstart.c
 *
 *  Created on: 06.06.2018
 *      Author: fabiand
 */

#include "softstart.h"

static softstart_data_type data;


void init_softstart(int start, int ramp)
{
 data.cnt_ramp = ramp;
 data.cnt_start = start;
 data.enabled = 0;
 data.cnt = -start;
}

void start_softstart()
{
	data.enabled = 1;
	data.cnt = -data.cnt_start;
}

//Returns 1 when softstart is finished. Returns a ramp from 0 to 1 while softly starting
double check_softstart()
{
	//zero output for the first 'start' samples
	if(data.cnt < 0)
		return 0;

	//ramp for the next 'ramp' samples
	if(data.cnt < data.cnt_ramp){
		return data.cnt / (double)data.cnt_ramp;
	}

	return 1;
}

void advance_softstart()
{
	//printf("%d\n",data.cnt);
	if(data.cnt < data.cnt_ramp)
		data.cnt++;
}
