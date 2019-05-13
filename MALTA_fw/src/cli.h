/*
 * cli.h
 *
 *  Created on: 8 Mar 2018
 *      Author: smiric
 */

/**************************************************************************************************************
 **
 ** Origin: BitHound and miniLINK
 ** (c) 2013 Lukas Schrittwieser, Mario Mauerer
 ** Modified for Zynq by MM, August 2016
 **
 ** PES ETH Zurich - All rights reserved
 **
 **************************************************************************************************************
 **
 ** cli.h
 **
 *************************************************************************************************************/

#ifndef SRC_CLI_H_
#define SRC_CLI_H_

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "commands.h"
#include "xuartps_hw.h"


// define max line length that may be entered by the user
#define MAX_LINE_LEN	70
// define the depth of the command history buffer, be careful this can use lots of memory
#define HIST_MAX_LEN	10
// define max length of a control sequence
#define MAX_CTRL_SEQ_LEN	20
// define esc char used as control sequence intializer
#define ESC				0x1B
#define CSI				0x9B

#define TERMINAL_BELL()	xil_printf("\a")

#define ERROR_MSG(...)      { xil_printf("\x1b[31;1m"); printf(__VA_ARGS__); xil_printf("\x1b[0m\n"); }while(0)
#define WARN_MSG(...)      { xil_printf("\x1b[33m"); printf(__VA_ARGS__); xil_printf("\x1b[0m\n"); }while(0)


// process data coming in on the user interface and manage line buffer
void cliInput (char ch);

// re-print line buffer. This has to be used if some function outside of cli.c prints text (eg status message)
// This function prints the current line buffer and places the cursor at the correct position
void cliPrintLineBuf();

void poll_uart0_execute_cli(void); //Check for byte in uart0 buffer and forward it to cli.


#endif /* SRC_CLI_H_ */

