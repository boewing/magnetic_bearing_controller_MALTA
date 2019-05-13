/*
 * interrupt.h
 *
 *  Created on: 8 Mar 2018
 *      Author: smiric
 */

/**************************************************************************************************************
**
** SPAHD Zynq firmware
** Mauerer August 2016
**
** PES ETH Zurich - All rights reserved
**
**************************************************************************************************************
**
** interrupt.h
**
** Configures Zynq-PS Interrupts
**
*************************************************************************************************************/
#ifndef SRC_INTERRUPT_H_
#define SRC_INTERRUPT_H_

#include <stdint.h>
#include "Xscugic.h"
#include "xil_printf.h"
#include "malta_configure.h"
#include "adc.h"
#include "malta_control.h"
#include "commands.h"
#include "malta_statemachine.h"

void gic_init(void);
void init_first_interrupt(void);
void init_second_interrupt(void);


#endif /* SRC_INTERRUPT_H_ */

