/*
 * cli.c
 *
 *  Created on: 8 Mar 2018
 *      Author: smiric
 */
/**************************************************************************************************************
 **
 **
 ** Origin: BitHound and miniLINK
 ** (c) 2013 Lukas Schrittwieser, Mario Mauerer
 ** Modified for Zynq by MM, August 2016
 **
 ** PES ETH Zurich - All rights reserved
 **
 **************************************************************************************************************
 **
 ** cli.c
 **
 *************************************************************************************************************/

#include "cli.h"


/***********************************************************************************************************************
 *   P R O T O T Y P E S
 */

// receive an terminal control sequence (eg cursor keys) and process it
// returns 0 if more chars are needed and 1 if the sequence was received completely
int rxCtrlSeq(char ch);

// process a control sequence received from the terminal
// cmd: command char (last char of control sequence
// arg: argument string (containig any number of arguments)
// len: length of arg in bytes
void processCtrlSeq(char cmd, const char* arg, int len);

// set cursor to given column (first column is 0)
void setCursorH (int c);

// execute command from line buffer
void execute();

// return list of command names, needed for auto completer of help command
// argInd: index of the argument to be autocompleted. Has to be 0 otherwise there are no options
//         (Because we have only one argument for the help command)
// cmtInd: index of the command name
const char* listCmdNames(int argInd, int cmdInd);

// autocomplete list function for config names
const char* aListCfgNames(int argInd, int nameInd);

// autocomplete command names or call command specifiec autocompleter
void autocomplete();

// insert a string into line buffer or delete from line buffer
// n: number of bytes to be inserted if positive or to be deleted if negative, 0: take length of insert
// insert: string to be inserted, can be null for deletes
void modLineBuf(int n, const char* insert);


/***********************************************************************************************************************
 *   G L O B A L S
 */
// allocate a buffer for strings comming in from the serial port. This is what is modified by the user
char lineBuf[MAX_LINE_LEN+1] = {'\0'};

// line buffer length (the total number of bytes currently stored in the buffer)
int lineBufLen = 0;

// line buffer index, this is the cursor of the user interface, ie where the next byte will be placed
int lineBufInd = 0;

// histroy buffer with previous commands
char historyBufs[HIST_MAX_LEN][MAX_LINE_LEN+1];

// next entry (to be written) in history buffer
int historyWrInd = 0;

// number of valid command lines in history buffer
int historyLen = 0;

// history index requested by user. -1: no history
int historyOffs = -1;


/***********************************************************************************************************************
 *   I M P L E M E N T A T I O N
 */


/*
 * Check, if the UART has received a Byte. If yes, forward to cli.
 */
void poll_uart0_execute_cli(void)
{
	if(XUartPs_IsReceiveData(XPS_UART0_BASEADDR))
	{
		cliInput(XUartPs_RecvByte(XPS_UART0_BASEADDR));
	}
}


/*
 * process data coming in on the user interface and manage line buffer
 */
void cliInput (char ch)
{
	static uint8_t ignoreNewLine = 0;
	static uint8_t isCtrlSeq = 0;

	if (isCtrlSeq)
	{
		// char is part of a control sequence -> process it
		if (rxCtrlSeq(ch))
			isCtrlSeq = 0;	// processing done
		return;
	}

	// check for tab (auto complete)
	if (ch == '\t')
	{
		autocomplete();
		return;
	}

	// check for new line (ie execute command)
	if ((ch == '\n') || (ch == '\r'))
	{
		if (ch == '\r')
			ignoreNewLine = 1;	// ignore a following \n (probably sent by win sytle terminal)
		if ((ch == '\n') && ignoreNewLine)
		{
			ignoreNewLine = 0;	// new line is ignored now
			return;
		}
		xil_printf("\n");	// feed the users new-line back to the terminal
		// ignore empty command lines
		if (lineBufLen == 0)
			return;
		// copy line buffer into history buffer
		strcpy(historyBufs[historyWrInd++], lineBuf);
		// wrap write index around
		if (historyWrInd >= HIST_MAX_LEN)
			historyWrInd = 0;
		// we have one more entry in the history
		if (historyLen < HIST_MAX_LEN)
			historyLen++;
		historyOffs = -1;	// rewind history browsing offset back to 'edit line buffer' state
		// process command in line buffer
		execute();
		// buffer is empty now
		lineBufLen = 0;
		lineBufInd = 0;
		lineBuf[0] = '\0';
		return;
	}
	ignoreNewLine = 0;	// this char is not a \n -> stop ignoring

	// check for backspace
	if ((ch == 0x08) || (ch == 0x7F))
	{
		if (lineBufInd == 0)
			return;	// can't delete from before the beginning of the line
		// move text left one char
		modLineBuf(-1, NULL);	// this updates lineBufLen as well
		lineBufInd--;

		setCursorH(lineBufInd);	// cursor horizontal absolute command
		xil_printf("\x1b[K");	// erase from cursor to end of line
		//lineBuf[lineBufLen] = '\0';
		fputs(lineBuf+lineBufInd,stdout);	// print from cursor on forward
		setCursorH(lineBufInd);	// cursor horizontal absolute command
		return;
	}

	// process regular chars
	if (isprint((int)ch))
	{
		// this is regular character ->	put it into line buffer
		modLineBuf(1, &ch);	// this increases lineBufLen as well

		if (lineBufLen > lineBufInd)
		{
			// the cursor is not at the end
			xil_printf("\x1b[K");	// erase from cursor to end of line
		}
		// print everything from the cursor on forward
		xil_printf(lineBuf+lineBufInd);
		lineBufInd++;	// move index because we have on more char
		setCursorH (lineBufInd);	// cursor horizontal absolute command
		return;
	}

	// check for a start of control sequence ESC char
	if ((ch == ESC) || (ch == CSI))
	{
		isCtrlSeq = 1;
		return;
	}

}


// re-print line buffer. This has to be used if some function outside of cli.c prints text (eg status message)
// This function prints the current line buffer and places the cursor at the correct position
void cliPrintLineBuf()
{
	xil_printf("\n");
	xil_printf(lineBuf);
	setCursorH(lineBufInd);
}

// receive an terminal control sequence (eg cursor keys) and process it
// returns 0 if more chars are needed and 1 if the sequence was received completely
int rxCtrlSeq(char ch)
{
	static char	seq[MAX_CTRL_SEQ_LEN];
	static int 	len = 0;

	// ignore leading [ as it is part of the ctrl seq intializer
	if ((len == 0) && (ch == '['))
		return 0;

	// check for trailing char of a sequence
	if ((ch >= 64) && (ch <= 126))
	{
		// process command
		processCtrlSeq(ch, seq, len);
		len = 0;	// prepare for next sequence
		return 1;
	}

	// this is a regular char -> store it in buffer
	seq[len++] = ch;
	return 0;
}

// process a control sequence received from the terminal
// cmd: command char (last char of control sequence
// arg: argument string (containig any number of arguments)
// len: length of arg in bytes
void processCtrlSeq(char cmd, const char* arg, int len)
{
	int i;
	switch (cmd)
	{
	case 'A':	// cursor up -> search in history towards older entry
		if (historyOffs >= (historyLen-1))
		{
			TERMINAL_BELL();
			break;	// limit reached -> abort
		}
		// increase offset
		historyOffs++;
		// calc which history buffer should be loaded (ring buffer index calc)
		i = historyWrInd - historyOffs -1 ;	// -1 because index points to _next_ buffer to be written.
		// make sure the value is positive
		if (i < 0)
			i += HIST_MAX_LEN;
		// copy history buffer into line buffer
		strcpy(lineBuf, historyBufs[i]);
		lineBufLen = strlen(lineBuf);
		lineBufInd = lineBufLen;
		xil_printf("\x1b[2K");	// clear entire line
		setCursorH(0);
		xil_printf(lineBuf);
		setCursorH(lineBufInd);	// set cursor to end of line
		break;

	case 'B':	// cursor down -> search in history towards newer entry
		if (historyOffs <= -1)
		{
			TERMINAL_BELL();
			break;	// history is deactiavted already
		}

		historyOffs--;	// move to newer entry
		if (historyOffs >= 0)
		{
			// calc which history buffer should be loaded (ring buffer index calc)
			i = historyWrInd - historyOffs -1 ;	// -1 because index points to _next_ buffer to be written.
			// make sure the value is positive
			if (i < 0)
				i += HIST_MAX_LEN;
			// copy history buffer into line buffer
			strcpy(lineBuf, historyBufs[i]);
		}
		else
		{
			// clear newest history entry from line buffer
			lineBuf[0] = '\0';
		}
		lineBufLen = strlen(lineBuf);
		lineBufInd = lineBufLen;
		xil_printf("\x1B[2K");	// clear entire line
		setCursorH(0);
		xil_printf(lineBuf);
		setCursorH(lineBufInd);	// set cursor to end of line
		break;

	case 'C':	// cursor right -> move cursor
		if (lineBufInd < lineBufLen)	// move line buffer pointer
			lineBufInd++;
		setCursorH(lineBufInd);
		break;

	case 'D':	// cursor left -> move cursor
		if (lineBufInd > 0)				// move line buffer pointer
			lineBufInd--;
		setCursorH(lineBufInd);
		break;
	}
}

// set cursor to given column (first column is 0)
void setCursorH (int c)
{

	if (c < 0)
		c = 0;
	xil_printf("%c[%iG", ESC, c+1);	// cursor horizontal absolute command

}

// execute command from line buffer
void execute()
{
	// determine length of command word (before parameters)
	// search for first space (strchr returns a pointer to it) and calculate its index
	char* params = strchr(lineBuf, ' ');
	int len = (int)(params - lineBuf);

	if (params == NULL)
		len = strlen(lineBuf);
	// split the line buffer into two separate strings by introducing a new \0 if there are parameters
	if (params != NULL)
	{
		*params = '\0';
		params++; // params points to a '\0' now -> go to next char
	}

	// skip any blanks at beginning of parameter string
	while ((params != NULL) && (*params != '\0') && (isblank((int)*params)))
		params++;

	//printf("cmd: '%s'\nparams: '%s'\n",lineBuf, params);

	int i;
	for(i=0; i<getNCommands(); i++)
	{
		if ((strncmp(lineBuf, cmdList[i].name, len) == 0) && (strlen(cmdList[i].name) == len))
		{
			// command found -> call handler
			if (cmdList[i].handler == 0)
			{
				xil_printf("ERR: No command handler defined.\n");
			}
			else
			{
				// call command handler
				cmdList[i].handler(lineBuf, params);
				return;
			}
		}
	}
	// command not found
	xil_printf("Command '%s' not found\n", lineBuf);
}


// autocomplete command names or call command specifiec autocompleter
void autocomplete()
{
	int argInd = 0;
	int cmdNameLen = -1;	// count length of the command name in the buffer
	int startInd = 0;		// index at which completion starts (assume start of buffer)

	// count how many arguments have been entered so far
	for(int i=0; i<lineBufInd; i++)
	{
		if (isblank((int)lineBuf[i]))
		{
			argInd++;
			if (cmdNameLen < 0)
				cmdNameLen = i;	// this is the first blank -> gives us the command name length
			// skip additional blanks
			while ((i<lineBufInd) && isblank((int)lineBuf[i]))
				i++;
			// assume the next char is the start of the value to be completed
			startInd = i;
		}
	}
	// if the index is not at the end of the line and not pointing at a blank we are not the end of a word
	if ((lineBufInd != lineBufLen) && (!isblank((int)lineBuf[lineBufInd])))
		return;

	const char* (*listFct)(int, int) = NULL;	// function pointer for list function
	if (argInd == 0)				// the first 'argument' is the command name
		listFct = &listCmdNames;
	else
	{
		argInd--;
		// check the command name to determine the list function

		for(int i=0; i<getNCommands(); i++)
		{
			if ((strncmp(lineBuf, cmdList[i].name, cmdNameLen) == 0) && (strlen(cmdList[i].name) == cmdNameLen))
			{
				listFct = cmdList[i].listFct;
				break;
			}
		}
	}

	if (listFct == NULL)
		return;	// no list function for possible values defined -> nothing to do

	const char* hit = NULL;
	int nHit = 0;	// number of matching commands

	// check the number of hits
	int i=0;
	int nCommon=0;	// number of chars common to all hits
	const char* candidate =  listFct(argInd, i);
	while (candidate != NULL)
	{
		// compare text in line buffer and the candidate
		if (strncmp(lineBuf+startInd, candidate, lineBufInd-startInd) == 0)
		{
			nHit++;
			if (nHit == 1)
				nCommon = strlen(candidate);
			else
			{
				// reduce common length until we find a match between the last hit
				// and the candidate (which we know is a hit as well)
				while ((nCommon>0) && (strncmp(candidate, hit, nCommon)!=0))
					nCommon--;
			}
			hit = candidate;
		}
		i++;
		// get next candidate from the list
		candidate =  listFct(argInd, i);
	}

	if (nHit == 0)
	{
		TERMINAL_BELL();
		return;	// nothing found
	}

	if (nHit == 1)
	{
		// is there anything to be inserted?
		int n = strlen(hit) - (lineBufInd-startInd);
		if (n <= 0)
			return;
		modLineBuf(n, hit+lineBufInd-startInd);	// insert missing part of command (everything which is beyond the cursor)
		// send update to user
		xil_printf("\x1b[K");	// erase from cursor to end of line
		xil_printf(lineBuf+lineBufInd);
		// move cursor to end of word
		lineBufInd += n;
		setCursorH(lineBufInd);
		return;
	}

	nHit = 0;
	// the result is not unique
	if (nCommon > 0)
	{
		// all hits have some common chars, insert those into the line buffer
		int n = nCommon - (lineBufInd-startInd);	// calc length which has to be inserted (common len minus already entered chars)
		if (n > 0)
		{
			modLineBuf(n, hit+lineBufInd-startInd);	// insert missing part of command (everything which is beyond the cursor)
			// move cursor to end of word
			lineBufInd += n;
		}
	}

	// -> show a list of all commands

	xil_printf("\n");
	i = 0;
	candidate =  listFct(argInd, i);
	while (candidate != NULL)
	{
		// compare text in line buffer (up to index, ie cursor) and command list
		if (strncmp(lineBuf+startInd, candidate, lineBufInd-startInd) == 0)
		{
			// show the matching command and start a new line every 4 matches
			xil_printf("  ");
			xil_printf(candidate);
			nHit++;
			if (nHit == 4)
			{
				nHit = 0;
				xil_printf("\n");
			}
		}
		i++;
		candidate = listFct(argInd, i);
	}
	// start new line if needed
	if (nHit != 0)
		xil_printf("\n");
	// reprint command line
	xil_printf(lineBuf);
	setCursorH(lineBufInd);
}


// insert a string into line buffer or delete from line buffer
// insertion or deletion appear at location pointed to by line buffer index.
// no commands are sent to the terminal (serial port)
// n: number of bytes to be inserted if positive or to be deleted if negative, 0: take length of insert
// insert: string to be inserted, can be null for deletes
void modLineBuf(int n, const char* insert)
{
	if (n==0)
		n = strlen(insert);

	// make room for new data or replace old one
	memmove(lineBuf+lineBufInd+n, lineBuf+lineBufInd, lineBufLen-lineBufInd);
	if (n > 0)
		memcpy(lineBuf+lineBufInd, insert, n);	// transfer new data into buffer
	lineBufLen += n;
	lineBuf[lineBufLen] = '\0';
}


