/*
 * Control_Handle.c
 *
 *  Created on: 08.06.2018
 *      Author: fabiand
 */

#include "Control_Handle.h"


static Control_Handle_data_type control_handle;




void init_control_system()
{
	control_handle.Control_Sys_Run = 0;
}


void start_control_system()
{
	control_handle.Control_Sys_Run = 1;
}


void stop_control_system()
{
	control_handle.Control_Sys_Run = 0;
}

int read_control_system_state()
{
  return control_handle.Control_Sys_Run;
}
