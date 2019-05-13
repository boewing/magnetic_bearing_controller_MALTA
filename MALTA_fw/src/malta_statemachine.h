/*
 * malta_statemachine.h
 *
 *  Created on: 30.04.2018
 *      Author: fabiand
 */

#ifndef SRC_MALTA_STATEMACHINE_H_
#define SRC_MALTA_STATEMACHINE_H_

#include "malta_configure.h"

#define NO_EVENT ((int)0)
#define CURRENT_CONTROL_EVENT ((int)1)
#define POSITION_CONTROL_EVENT ((int)2)
#define POSITION_MEA_EVENT ((int)4)

#define VAR_SAMPLE ((int)8)
#define CALIBRATE_THETA_D ((int)16)

#define SOFT_START_EVENT ((int)32)
#define POLL_UART ((int)64)

#define CONTROL_EVENT ((int)128)


#define PLOT_CURRENT ((int)256)
#define PLOT_POSITION ((int)512)
#define PLOT_BEAR_ANGLE ((int)1024)
#define PLOT_EST_DRIVE_ANGLE ((int)2048)



//extern volatile int g_state_to_execute;
extern volatile double ctheta;
extern volatile double cthetab;
extern volatile double cbear_cur;
extern volatile double cdrive_cur;
extern volatile double cM1fx;
extern volatile double cM1fy;
extern volatile double cM1fz;
extern volatile double cM2fx;
extern volatile double cM2fy;


void set_time_pin();
void clear_time_pin();
void toggle_time_pin();
void Clear_Event_Loc(volatile int *local_events,int event);

void set_event(int event);
void clear_event(int event);
void copy_events_to_local(volatile int *local_events);
int read_events();

int wait_time(void);


#endif /* SRC_MALTA_STATEMACHINE_H_ */
