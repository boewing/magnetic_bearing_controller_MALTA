/*
 * softstart.h
 *
 *  Created on: 06.06.2018
 *      Author: fabiand
 */

#ifndef SRC_SOFTSTART_H_
#define SRC_SOFTSTART_H_

#include "malta_parameters.h"

typedef struct{
	int cnt;
	int enabled;
	int cnt_ramp;
	int cnt_start;
}softstart_data_type;

void init_softstart(int start, int ramp);
void start_softstart(void);
double check_softstart(void);
void advance_softstart(void);

#endif /* SRC_SOFTSTART_H_ */
