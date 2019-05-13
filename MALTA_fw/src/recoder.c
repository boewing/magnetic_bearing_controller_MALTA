/*
 * recorder.c
 *
 *  Created on: 23 Apr 2018
 *      Author: smiric
 */



#include "recorder.h"

/*
 * The recorder is a hardware-unit that writes data (via DMA) to the RAM.
 * It can then be read (via UART or Ethernet).
 */


/**************************************************************************************************************
 * Globals
 **************************************************************************************************************/
volatile int32_t g_RecArray[MAXNUMSAMP+SPARESAMPLES] = {0}; //Recorder memory. Don't use MEMDEFAULT here - the .elf-file will get huge...
uint32_t g_RecActive = 0;

/**************************************************************************************************************
 * Prototypes
 **************************************************************************************************************/

/**************************************************************************************************************
 * Local Defines
 **************************************************************************************************************/


/**************************************************************************************************************
 * Function Implementations
 **************************************************************************************************************/


/*
 * Fill the entire memory with a default value - to be able to detect false writes by the hardware
 */
void rec_init_mem(void)
{
	//Just to be sure the values are all written, we flush the cashes before and after.
	Xil_DCacheFlush();
	for(uint32_t i=0;i<(MAXNUMSAMP+SPARESAMPLES);i++)
	{
		g_RecArray[i] = (uint32_t)MEMDEFAULT;
	}
	Xil_DCacheFlush();
}

/*
 * Write the base-address of the recorder-array to the HW for DMA access.
 * Return 0 if all OK, 1 if failed.
 */
uint32_t rec_write_baseaddr(void)
{
	volatile uint32_t* const baseaddr_reg = (void*)REC_BASE;
	const uint32_t BaseAddr = (uint32_t)g_RecArray; //This is the address of our array, casted to uint32_t
	*baseaddr_reg = BaseAddr;
	volatile uint32_t Readback = *baseaddr_reg;
	if(Readback != BaseAddr)
	{
		return 1;
	}
	else
		return 0;
}

/*
 * Check the status of the recording unit
 * Return 0 if idle
 * Return 1 if Running
 * Return 2 if Error
 */
uint32_t rec_check_status(void)
{
	volatile uint32_t* const stat_reg = (void*)REC_STAT;
	uint32_t status = *stat_reg;
	uint32_t running = status & 0x00000001; //running-bit is bit 0
	uint32_t err = status & 0x000000002; //error-bit is bit 1
	err = err >> 1;
	if(running == 1)
	{
		return 1;
	}
	else if(err == 1)
	{
		return 2;
	}
	else
	{
		return 0;
	}
}

/*
 * Set the channel Mux of the Recorder
 * Return 0 if all OK, 1 if failed.
 */
uint32_t rec_set_channel(uint32_t ch)
{
	//Check, if the recorder is currently running
	if(rec_check_status() != 0)
	{
		return 1;
	}
	if(ch > 255)
	{
		return 1;
	}
	volatile uint32_t* const ctrl_reg = (void*)REC_CTRL;
	uint32_t ctrl = *ctrl_reg;
	ctrl = ctrl & 0xFFFFFF00; //clear old channel selection (lower 8 bits)
	ctrl = ctrl | ch; //set the new channel
	*ctrl_reg = ctrl; //write the value back
	return 0;
}


/*
 * Start the recording.
 * Return 0 if all OK, 1 if failed.
 */
uint32_t rec_set_start(void)
{
	//Check, if the recorder is currently running
	if(rec_check_status() == 1)
	{
		return 1;
	}
	volatile uint32_t* const ctrl_reg = (void*)REC_CTRL;
	uint32_t ctrl = *ctrl_reg;
	ctrl = ctrl | (uint32_t)(1<<8); //Start-bit is Bit 8
	*ctrl_reg = ctrl;
	g_RecActive = 1;
	return 0;
}

/*
 * Clear the start-bit
 * Return 0 if all OK, 1 if failed.
 */
uint32_t rec_clear_start(void)
{
	//Check, if the recorder is currently running
	if(rec_check_status() == 1)
	{
		return 1;
	}
	volatile uint32_t* const ctrl_reg = (void*)REC_CTRL;
	uint32_t ctrl = *ctrl_reg;
	ctrl = ctrl & (uint32_t)~(1<<8); //Start-bit is Bit 8
	*ctrl_reg = ctrl;
	return 0;
}

/*
 * Set the stop-bit
 */
void rec_set_stop(void)
{
	volatile uint32_t* const ctrl_reg = (void*)REC_CTRL;
	uint32_t ctrl = *ctrl_reg;
	ctrl = ctrl | (uint32_t)(1<<9); //Stop-Bit is Nr. 9
	*ctrl_reg = ctrl;
}

/*
 * Clear the stop-bit
 */
void rec_clear_stop(void)
{
	volatile uint32_t* const ctrl_reg = (void*)REC_CTRL;
	uint32_t ctrl = *ctrl_reg;
	ctrl = ctrl & (uint32_t)~(1<<9); //Stop-Bit is Nr. 9
	*ctrl_reg = ctrl;
}

/*
 * Write the number of samples to record to the hardware:
 * Return 0 if all OK, 1 if failed.
 */
uint32_t rec_write_numsamp(uint32_t numsamp)
{
	if(numsamp > (uint32_t)MAXNUMSAMP)
	{
		return 1;
	}
	volatile uint32_t* const num_reg = (void*)REC_NUMSAMPL;
	*num_reg = (numsamp+(uint32_t)SPARESAMPLES); //record a bit more, as the first few samples seem not always to arrive in the memory.
	if(rec_get_numsamp() != numsamp+(uint32_t)SPARESAMPLES)
		return 1;
	else
		return 0;
}

/*
 * Get the number of samples that have been recorded.
 * This includes the first few SPARESAMPLES
 */
uint32_t rec_get_numsamp(void)
{
	volatile uint32_t* const num_reg = (void*)REC_NUMSAMPL;
	uint32_t num = *num_reg;
	if(num == 0)
		num = 1; //prevents other functions from crashing. Just in case.
	return num;
}


/*
 * Read the recorder and print to uart
 * Return 0 if OK; 1 if failed.
 */
uint32_t rec_read_to_uart(void)
{
	//Check if recorder is still running:
	if(rec_check_status() != 0)
		return 1;
	//Before reading, the cashes should be flushed, as the MMU does not know that data in this area has been updated...
	//Otherwise, we might get old data from the cashes back and not new samples.
	Xil_DCacheFlush();
	uint32_t numsamp = rec_get_numsamp(); //read the number of samples that have been written
	xil_printf("Printing recorder data to UART\n");
	for(uint32_t i=SPARESAMPLES;i<numsamp;i++)
	{
		xil_printf("%d\n",g_RecArray[i]);
	}
	return 0;
}
