/*
 * interrupt.c
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
 ** interrupt.c
 **
 ** Configures Zynq-PS Interrupts
 **
 *************************************************************************************************************/

#include "interrupt.h"


XScuGic GICInst; //Instance of the interrupt controller's driver struct

void I_am_An_Example();
void interupt2();


/*
 * Initialize the Zynq's GIC
 */
void gic_init(void)
{
	XScuGic_Config *IntConfig;

	IntConfig = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);
	if(IntConfig==NULL)
	{
		xil_printf("gic_init: LookupConfig failed");
		return;
	}
	//xil_printf("GIC base address: 0x%08x\n", (unsigned int)IntConfig->CpuBaseAddress);

	int stat = XScuGic_CfgInitialize(&GICInst, IntConfig, IntConfig->CpuBaseAddress);
	if(stat != XST_SUCCESS)
	{
		xil_printf("gic_init: CfgInitialize failed");
	}

	stat = XScuGic_SelfTest(&GICInst);
	if(stat != XST_SUCCESS)
	{
		xil_printf("gic_init: SelfTest failed");
	}

	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, &GICInst); //register the driver

	Xil_ExceptionEnable(); //enable the IRQ

}



/*
 * Initialize a specific interrupt
 * In this case, it's XPAS_FPGA0_INT_ID, 0 being the first input to the Zynq-block in the BD
 */
void init_first_interrupt(void)
{

	int stat = XScuGic_Connect(&GICInst, XPS_FPGA0_INT_ID, &I_am_An_Example, NULL);
	if(stat != XST_SUCCESS)
	{
		xil_printf("interrupt connect failed\n");
	}
	XScuGic_SetPriorityTriggerType(&GICInst, XPS_FPGA0_INT_ID, 0, 1); //priority: 0 (highest). Sensitivity: 1: Sensitive to "high"
	XScuGic_Enable(&GICInst, XPS_FPGA0_INT_ID);
}


void init_second_interrupt(void)
{

       int stat = XScuGic_Connect(&GICInst, XPS_FPGA1_INT_ID, &interupt2, NULL);
       if(stat != XST_SUCCESS)
       {
              xil_printf("interrupt connect failed\n");
       }
       XScuGic_SetPriorityTriggerType(&GICInst, XPS_FPGA1_INT_ID, 1, 1); //priority: 0 (highest). Sensitivity: 1: Sensitive to "high"
       XScuGic_Enable(&GICInst, XPS_FPGA1_INT_ID);
}

/*
 *
 * This is an example ISR.
 * It can be anywhere in the project!
 */
void I_am_An_Example()
{
	set_time_pin();


	int temp_event = 0;
	static int Down_Samp = 0;
	Down_Samp ++;
	if (Down_Samp >= 2000)   //Downsamples currentwontroller event by 9
	{
		temp_event |= POLL_UART;

		Down_Samp = 0;
	}
	//temp_event |= (POSITION_MEA_EVENT|POSITION_CONTROL_EVENT|CURRENT_CONTROL_EVENT|VAR_SAMPLE);
	temp_event |= (CONTROL_EVENT|VAR_SAMPLE);
	set_event(temp_event);


	// Here, you would have to acknowledge the interrupt via the AXI-control register such that the unit who raised the interrupt, takes it Low again.
	interupt1_togglebit();

	//clear_time_pin();

}


void interupt2(){

	set_time_pin();

	clear_time_pin();

	interupt2_togglebit();
    return;
}


