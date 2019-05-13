/*
 * malta_state_machine.c
 *
 *  Created on: 30.04.2018
 *      Author: fabiand
 */



#include "malta_statemachine.h"


static volatile int g_state_to_execute = 0;

volatile double ctheta = 0;
volatile double cthetab = 0;
volatile double cbear_cur = 0;
volatile double cdrive_cur = 0;
volatile double cM1fx = 0;
volatile double cM1fy = 0;
volatile double cM1fz = 0;
volatile double cM2fx = 0;
volatile double cM2fy = 0;



void set_time_pin()
{
	if (TIMING_PIN_EN) {
	volatile uint32_t* const cfg_register = (void*)AXI_GP_BASE;
	uint32_t cfg = *cfg_register; //Read the register into a local variable
    //printf("Set: %d\n",cfg);
	cfg |= (uint32_t)(1<<2); //Set 2 bit 1

	*cfg_register = cfg; //write the modified value back to the HW
	}
}

void clear_time_pin()
{
	if (TIMING_PIN_EN) {
	volatile uint32_t* const cfg_register = (void*)AXI_GP_BASE;
	uint32_t cfg = *cfg_register; //Read the register into a local variable
	//printf("Clear: %d\n",cfg);
	cfg &= (uint32_t)~(1<<2); //Set 2 bit 0
	*cfg_register = cfg; //write the modified value back to the HW
	}
}

void toggle_time_pin()
{
	if (TIMING_PIN_EN) {
	volatile uint32_t* const cfg_register = (void*)AXI_GP_BASE;
	uint32_t cfg = *cfg_register; //Read the register into a local variable
    //printf("Set: %d\n",cfg);
	cfg ^= (uint32_t)(1<<2); //Set 2 bit 1

	*cfg_register = cfg; //write the modified value back to the HW
	}
}

void set_event(int event)
{
	g_state_to_execute |= event;
}

void clear_event(int event)
{
	g_state_to_execute &= (~event);
}

void copy_events_to_local(volatile int *local_events)
{
	*local_events = g_state_to_execute;
	g_state_to_execute &= ~*local_events;

}

int read_events()
{
	return g_state_to_execute;
}

void Clear_Event_Loc(volatile int *local_events,int event)
{
	*local_events =  *local_events & ~(event);

}

int wait_time()
{
  static int i = 0;
  static int j = 0;

  i++;
  if (i>=60)
  {
	  i=0;
	  j++;
  }
  if (j>=100000)
  {
	  j = 0;
	  return 1;
  }
  else {return 0;}
}




