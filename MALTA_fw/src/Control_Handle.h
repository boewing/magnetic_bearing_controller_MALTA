/*
 * Control_Handle.h
 *
 *  Created on: 08.06.2018
 *      Author: fabiand
 */

#ifndef SRC_CONTROL_HANDLE_H_
#define SRC_CONTROL_HANDLE_H_

typedef struct {
 int Control_Sys_Run;

}Control_Handle_data_type;


void init_control_system();
void start_control_system();
void stop_control_system();
int read_control_system_state();





#endif /* SRC_CONTROL_HANDLE_H_ */
